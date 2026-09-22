// SocialNav Nav2 controller plugin (MASTER_PROMPT §16-§25).
//
// Samples candidate (v, omega) trajectories, HARD-rejects unsafe ones against the costmap
// (§17-18), scores the survivors with the social cost model (§19), applies behavior-mode
// speed scaling (§20), and commands the best. Falls back to plain path-following when no
// humans are present (§21: "behave like a normal local planner").
#ifndef SOCIAL_NAV_CONTROLLER__SOCIAL_NAV_CONTROLLER_HPP_
#define SOCIAL_NAV_CONTROLLER__SOCIAL_NAV_CONTROLLER_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

#include "social_nav_msgs/msg/planner_metrics.hpp"

#include "social_nav_msgs/msg/human_array.hpp"

#include "social_nav_core/trajectory_generator.hpp"
#include "social_nav_core/trajectory_scorer.hpp"

namespace social_nav_controller
{

class SocialNavController : public nav2_core::Controller
{
public:
  SocialNavController() = default;
  ~SocialNavController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("SocialNavController")};

  nav_msgs::msg::Path global_plan_;
  rclcpp::Subscription<social_nav_msgs::msg::HumanArray>::SharedPtr humans_sub_;
  std::mutex humans_mutex_;
  social_nav_msgs::msg::HumanArray latest_humans_;

  // Parameters (§26).
  double controller_frequency_{20.0};
  double max_linear_vel_{0.8};
  double max_angular_vel_{1.2};
  double lookahead_dist_{1.5};        ///< distance ahead on the plan for the local goal
  double transform_tolerance_{0.2};
  double human_timeout_{1.0};         ///< drop human data older than this (§54)
  double robot_radius_{0.30};
  double goal_dist_tolerance_{0.25};
  // Pure-pursuit base + social perturbation (hybrid) params.
  double desired_linear_vel_{0.5};
  double curvature_threshold_{0.6};
  double min_approach_speed_{0.1};
  double deviation_weight_{2.0};      ///< cost of straying from the pure-pursuit command
  double rotate_threshold_{0.9};      ///< rad: carrot bearing beyond this => rotate in place
  double rotate_gain_{1.5};

  social_nav_core::DiffDriveLimits limits_{};
  social_nav_core::SamplingParams sampling_{};
  social_nav_core::CostWeights weights_{};
  social_nav_core::ModeThresholds mode_th_{};
  social_nav_core::SocialZoneParams human_zone_{};
  double ttc_horizon_{4.0};
  double ttc_danger_distance_{0.8};
  double obstacle_influence_{1.0};
  double group_radius_{1.5};

  social_nav_core::BehaviorMode current_mode_{social_nav_core::BehaviorMode::kNormal};

  // setSpeedLimit() state.
  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
    global_path_pub_;

  // Debug (§32): the locally chosen trajectory + current behavior mode.
  bool debug_mode_{true};
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> sel_traj_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> mode_pub_;
  // Failure-state reporting (§34): OK / EMPTY_PLAN / NO_VALID_TRAJECTORY /
  // STALE_HUMAN_DATA / EMERGENCY_STOP, published on /social_nav/debug/status.
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> status_pub_;
  bool last_humans_stale_{false};
  void publishStatus(const char * status);

  // Performance metrics (§30): rolling window of computeVelocityCommands latencies.
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<social_nav_msgs::msg::PlannerMetrics>>
    metrics_pub_;
  std::deque<double> compute_ms_;
  std::deque<double> call_period_ms_;
  rclcpp::Time last_call_time_;
  bool have_last_call_{false};
  rclcpp::Time last_metrics_time_;

  void recordAndPublishMetrics(double compute_ms, int num_humans, const char * mode);

  double effectiveMaxSpeed() const;

  /// Pruned plan points (world frame) from nearest-to-robot forward, and the local goal.
  void prunePlan(
    const social_nav_core::Pose2D & robot,
    std::vector<Eigen::Vector2d> & path_out,
    Eigen::Vector2d & goal_out) const;

  /// Lethal/inscribed costmap cells as world points within `radius` of the robot, for the
  /// scorer's SOFT obstacle-proximity term (§19). Not used for hard rejection.
  std::vector<Eigen::Vector2d> extractObstacles(
    const social_nav_core::Pose2D & robot, double radius) const;

  /// Hard collision test against the costmap directly (§17-18): a trajectory collides if
  /// any point lands on an inscribed/lethal cell or leaves the costmap. The costmap's
  /// inflation already accounts for the robot radius, so we must NOT inflate again.
  bool trajectoryCollides(const social_nav_core::Trajectory & traj) const;

  /// Snapshot the freshest non-stale humans as scorer inputs (§54 staleness).
  std::vector<social_nav_core::ScoredHuman> currentHumans();
};

}  // namespace social_nav_controller

#endif  // SOCIAL_NAV_CONTROLLER__SOCIAL_NAV_CONTROLLER_HPP_
