// SocialLayer: a Nav2 costmap layer that stamps each person's anisotropic personal-space
// cost into the costmap (MASTER_PROMPT §15). Added to the GLOBAL costmap it makes the
// global planner route AROUND people; the cost is kept below the lethal/inscribed band so
// a physically-passable gap never becomes impassable (§22).
#ifndef SOCIAL_NAV_COSTS__SOCIAL_LAYER_HPP_
#define SOCIAL_NAV_COSTS__SOCIAL_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_nav_msgs/msg/human_array.hpp"

namespace social_nav_costs
{

class SocialLayer : public nav2_costmap_2d::Layer
{
public:
  SocialLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override {}
  bool isClearable() override {return false;}
  void onFootprintChanged() override {}

private:
  struct Person
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  void humansCallback(social_nav_msgs::msg::HumanArray::SharedPtr msg);

  /// Snapshot the freshest non-stale people (already in the costmap's global frame in the
  /// demo, where map==odom). Returns empty if the data is stale (§54).
  std::vector<Person> currentPeople();

  rclcpp::Subscription<social_nav_msgs::msg::HumanArray>::SharedPtr sub_;
  std::mutex mutex_;
  social_nav_msgs::msg::HumanArray latest_;

  // Parameters (§15).
  bool enabled_param_{true};
  double front_sigma_{1.0};
  double side_sigma_{0.7};
  double rear_sigma_{0.5};
  double max_cost_{200.0};      ///< peak inserted cost (< 253 so it is soft, not lethal)
  double cutoff_distance_{3.0}; ///< stamp cost out to this radius around each person (m)
  double human_timeout_{1.0};

  // Region touched last cycle, so stale cost is cleared as people move.
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};
  bool has_last_bounds_{false};
  std::vector<Person> people_;  ///< cached for this update cycle
};

}  // namespace social_nav_costs

#endif  // SOCIAL_NAV_COSTS__SOCIAL_LAYER_HPP_
