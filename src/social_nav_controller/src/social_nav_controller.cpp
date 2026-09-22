#include "social_nav_controller/social_nav_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "social_nav_core/kinematics.hpp"
#include "social_nav_controller/path_geometry.hpp"

namespace social_nav_controller
{

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool finite2(const Eigen::Vector2d & v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y());
}
}  // namespace

void SocialNavController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("SocialNavController: parent lifecycle node expired");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node_->get_logger();

  using nav2_util::declare_parameter_if_not_declared;
  auto declare = [&](const std::string & key, auto def) {
    declare_parameter_if_not_declared(node_, plugin_name_ + "." + key,
      rclcpp::ParameterValue(def));
  };
  declare("controller_frequency", 20.0);
  declare("max_linear_velocity", 0.8);
  declare("max_angular_velocity", 1.2);
  declare("max_linear_acceleration", 0.5);
  declare("max_angular_acceleration", 1.0);
  declare("min_linear_velocity", 0.0);
  declare("desired_linear_vel", 0.5);
  declare("curvature_threshold", 0.6);
  declare("min_approach_speed", 0.1);
  declare("deviation_weight", 2.0);
  declare("rotate_in_place_threshold", 0.9);
  declare("rotate_gain", 1.5);
  declare("lookahead_dist", 1.5);
  declare("human_timeout", 1.0);
  declare("robot_radius", 0.30);
  declare("v_samples", 8);
  declare("omega_samples", 15);
  declare("sim_time", 2.0);
  declare("sim_dt", 0.1);
  declare("ttc_horizon", 4.0);
  declare("ttc_danger_distance", 0.8);
  declare("obstacle_influence", 1.0);
  declare("front_social_sigma", 1.2);
  declare("side_social_sigma", 0.8);
  declare("rear_social_sigma", 0.6);
  declare("cautious_clearance", 2.0);
  declare("crowded_clearance", 1.2);
  declare("emergency_clearance", 0.5);
  declare("emergency_ttc", 1.0);
  declare("crowded_num_humans", 4);
  declare("mode_hysteresis", 0.25);
  declare("humans_topic", std::string("/social_nav/humans"));
  // Cost weights.
  declare("weights.goal", 1.0);
  declare("weights.path", 2.0);
  declare("weights.obstacle", 10.0);
  declare("weights.human", 5.0);
  declare("weights.ttc", 20.0);
  declare("weights.group", 6.0);
  declare("weights.direction", 1.0);
  declare("weights.smoothness", 1.0);
  declare("weights.velocity", 1.0);
  declare("weights.progress", 2.0);

  auto g = [&](const std::string & key, auto & out) {
    node_->get_parameter(plugin_name_ + "." + key, out);
  };
  g("controller_frequency", controller_frequency_);
  g("max_linear_velocity", max_linear_vel_);
  g("max_angular_velocity", max_angular_vel_);
  g("max_linear_acceleration", limits_.max_accel);
  g("max_angular_acceleration", limits_.max_alpha);
  g("min_linear_velocity", limits_.min_v);
  g("desired_linear_vel", desired_linear_vel_);
  g("curvature_threshold", curvature_threshold_);
  g("min_approach_speed", min_approach_speed_);
  g("deviation_weight", deviation_weight_);
  g("rotate_in_place_threshold", rotate_threshold_);
  g("rotate_gain", rotate_gain_);
  g("lookahead_dist", lookahead_dist_);
  g("human_timeout", human_timeout_);
  g("robot_radius", robot_radius_);
  g("v_samples", sampling_.v_samples);
  g("omega_samples", sampling_.omega_samples);
  g("sim_time", sampling_.sim_time);
  g("sim_dt", sampling_.sim_dt);
  g("ttc_horizon", ttc_horizon_);
  g("ttc_danger_distance", ttc_danger_distance_);
  g("obstacle_influence", obstacle_influence_);
  g("front_social_sigma", human_zone_.front_sigma);
  g("side_social_sigma", human_zone_.side_sigma);
  g("rear_social_sigma", human_zone_.rear_sigma);
  g("cautious_clearance", mode_th_.cautious_clearance);
  g("crowded_clearance", mode_th_.crowded_clearance);
  g("emergency_clearance", mode_th_.emergency_clearance);
  g("emergency_ttc", mode_th_.emergency_ttc);
  g("crowded_num_humans", mode_th_.crowded_num_humans);
  g("mode_hysteresis", mode_th_.hysteresis);
  g("weights.goal", weights_.goal);
  g("weights.path", weights_.path);
  g("weights.obstacle", weights_.obstacle);
  g("weights.human", weights_.human);
  g("weights.ttc", weights_.ttc);
  g("weights.group", weights_.group);
  g("weights.direction", weights_.direction);
  g("weights.smoothness", weights_.smoothness);
  g("weights.velocity", weights_.velocity);
  g("weights.progress", weights_.progress);
  limits_.max_v = max_linear_vel_;

  if (max_linear_vel_ <= 0.0 || max_angular_vel_ <= 0.0 || lookahead_dist_ <= 0.0 ||
    sampling_.sim_time <= 0.0 || sampling_.sim_dt <= 0.0)
  {
    throw std::runtime_error("SocialNavController: invalid (non-positive) parameter");
  }

  std::string humans_topic;
  node_->get_parameter(plugin_name_ + ".humans_topic", humans_topic);
  humans_sub_ = node_->create_subscription<social_nav_msgs::msg::HumanArray>(
    humans_topic, rclcpp::QoS(10),
    [this](social_nav_msgs::msg::HumanArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(humans_mutex_);
      latest_humans_ = *msg;
    });

  declare("debug_mode", true);
  node_->get_parameter(plugin_name_ + ".debug_mode", debug_mode_);

  global_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  sel_traj_pub_ =
    node_->create_publisher<nav_msgs::msg::Path>("/social_nav/debug/selected_trajectory", 1);
  mode_pub_ = node_->create_publisher<std_msgs::msg::String>("/social_nav/debug/planner_mode", 1);
  status_pub_ = node_->create_publisher<std_msgs::msg::String>("/social_nav/debug/status", 1);
  metrics_pub_ =
    node_->create_publisher<social_nav_msgs::msg::PlannerMetrics>("/social_nav/debug/metrics", 1);
  last_metrics_time_ = node_->now();

  RCLCPP_INFO(
    logger_, "SocialNavController '%s' configured (DWA social: %d x %d samples, sim %.1fs)",
    plugin_name_.c_str(), sampling_.v_samples, sampling_.omega_samples, sampling_.sim_time);
}

void SocialNavController::activate()
{
  if (global_path_pub_) {global_path_pub_->on_activate();}
  if (sel_traj_pub_) {sel_traj_pub_->on_activate();}
  if (mode_pub_) {mode_pub_->on_activate();}
  if (status_pub_) {status_pub_->on_activate();}
  if (metrics_pub_) {metrics_pub_->on_activate();}
  RCLCPP_INFO(logger_, "SocialNavController '%s' activated", plugin_name_.c_str());
}
void SocialNavController::deactivate()
{
  if (global_path_pub_) {global_path_pub_->on_deactivate();}
  if (sel_traj_pub_) {sel_traj_pub_->on_deactivate();}
  if (mode_pub_) {mode_pub_->on_deactivate();}
  if (status_pub_) {status_pub_->on_deactivate();}
  if (metrics_pub_) {metrics_pub_->on_deactivate();}
}
void SocialNavController::cleanup()
{
  global_path_pub_.reset();
  sel_traj_pub_.reset();
  mode_pub_.reset();
  status_pub_.reset();
  metrics_pub_.reset();
  humans_sub_.reset();
}

void SocialNavController::publishStatus(const char * status)
{
  if (status_pub_ && status_pub_->is_activated()) {
    std_msgs::msg::String s;
    s.data = status;
    status_pub_->publish(s);
  }
}

void SocialNavController::recordAndPublishMetrics(
  double compute_ms, int num_humans, const char * mode)
{
  compute_ms_.push_back(compute_ms);
  while (compute_ms_.size() > 200) {compute_ms_.pop_front();}

  if (!metrics_pub_ || !metrics_pub_->is_activated()) {return;}
  if ((node_->now() - last_metrics_time_).seconds() < 0.5) {return;}  // ~2 Hz
  last_metrics_time_ = node_->now();

  std::vector<double> s(compute_ms_.begin(), compute_ms_.end());
  std::sort(s.begin(), s.end());
  auto pct = [&](double p) -> double {
    if (s.empty()) {return 0.0;}
    const std::size_t idx = std::min(s.size() - 1,
        static_cast<std::size_t>(p * static_cast<double>(s.size())));
    return s[idx];
  };
  const double deadline = 1000.0 / std::max(controller_frequency_, 1.0);
  int misses = 0;
  for (double v : s) {if (v > deadline) {misses++;}}

  double mean_period = 0.0;
  if (!call_period_ms_.empty()) {
    for (double p : call_period_ms_) {mean_period += p;}
    mean_period /= static_cast<double>(call_period_ms_.size());
  }

  social_nav_msgs::msg::PlannerMetrics m;
  m.header.stamp = node_->now();
  m.target_frequency_hz = controller_frequency_;
  m.actual_frequency_hz = mean_period > 1e-6 ? 1000.0 / mean_period : 0.0;
  m.p50_ms = pct(0.50);
  m.p95_ms = pct(0.95);
  m.p99_ms = pct(0.99);
  m.max_ms = s.empty() ? 0.0 : s.back();
  m.deadline_ms = deadline;
  m.deadline_misses = misses;
  m.window_samples = static_cast<int>(s.size());
  m.mode = mode;
  m.num_humans = num_humans;
  metrics_pub_->publish(m);
}

void SocialNavController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (global_path_pub_ && global_path_pub_->is_activated()) {
    global_path_pub_->publish(global_plan_);
  }
}

double SocialNavController::effectiveMaxSpeed() const
{
  if (speed_limit_ <= 0.0) {return max_linear_vel_;}
  const double limit = speed_limit_is_percentage_ ?
    max_linear_vel_ * speed_limit_ / 100.0 : speed_limit_;
  return std::clamp(limit, 0.0, max_linear_vel_);
}

bool SocialNavController::trajectoryCollides(const social_nav_core::Trajectory & traj) const
{
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {return false;}
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  for (const auto & pt : traj.points) {
    unsigned int mx, my;
    if (!costmap->worldToMap(pt.pose.x, pt.pose.y, mx, my)) {
      return true;  // left the costmap region
    }
    const unsigned char c = costmap->getCost(mx, my);
    // Inflation already bakes in the robot radius: an inscribed/lethal cell means the
    // robot centre there collides. Unknown (255) is treated as free (allow_unknown).
    if (c == nav2_costmap_2d::LETHAL_OBSTACLE ||
      c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return true;
    }
  }
  return false;
}

std::vector<social_nav_core::ScoredHuman> SocialNavController::currentHumans()
{
  std::vector<social_nav_core::ScoredHuman> out;
  last_humans_stale_ = false;
  social_nav_msgs::msg::HumanArray snapshot;
  {
    std::lock_guard<std::mutex> lk(humans_mutex_);
    snapshot = latest_humans_;
  }
  if (snapshot.humans.empty()) {return out;}
  // Drop stale human data.
  // Use .seconds() on each side: avoids throwing when clock types differ, and both run on
  // sim time in the demo. (Publishers must stamp on the same clock or they look stale.)
  const double age = node_->now().seconds() - rclcpp::Time(snapshot.header.stamp).seconds();
  if (age > human_timeout_) {  // drop stale data (a small future offset is tolerated)
    last_humans_stale_ = true;
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
      "Human data stale (age=%.2fs) - ignoring", age);
    return out;
  }
  for (const auto & h : snapshot.humans) {
    social_nav_core::ScoredHuman sh;
    sh.position = Eigen::Vector2d(h.pose.position.x, h.pose.position.y);
    sh.velocity = Eigen::Vector2d(h.velocity.linear.x, h.velocity.linear.y);
    sh.heading = yawFromQuaternion(h.pose.orientation);
    sh.zone = human_zone_;
    if (finite2(sh.position) && finite2(sh.velocity)) {  // reject NaN/Inf
      out.push_back(sh);
    }
  }
  return out;
}

geometry_msgs::msg::TwistStamped SocialNavController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  // Measure planning latency and inter-call period.
  const auto t_start = std::chrono::steady_clock::now();
  const rclcpp::Time call_now = node_->now();
  if (have_last_call_) {
    call_period_ms_.push_back((call_now - last_call_time_).seconds() * 1000.0);
    while (call_period_ms_.size() > 200) {call_period_ms_.pop_front();}
  }
  last_call_time_ = call_now;
  have_last_call_ = true;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000, "Empty plan - stopping");
    publishStatus("EMPTY_PLAN");
    return cmd;
  }

  const social_nav_core::Pose2D robot{
    pose.pose.position.x, pose.pose.position.y, yawFromQuaternion(pose.pose.orientation)};
  const Eigen::Vector2d r(robot.x, robot.y);
  const Eigen::Vector2d final_goal(
    global_plan_.poses.back().pose.position.x, global_plan_.poses.back().pose.position.y);
  const double dist_to_goal = (final_goal - r).norm();

  // 1. Pure-pursuit base command (reliable path following).
  std::vector<Eigen::Vector2d> path_robot;
  path_robot.reserve(global_plan_.poses.size());
  for (const auto & ps : global_plan_.poses) {
    path_robot.push_back(
      toRobotFrame(robot, Eigen::Vector2d(ps.pose.position.x, ps.pose.position.y)));
  }
  const Carrot carrot = findLookaheadPoint(path_robot, lookahead_dist_);
  if (!carrot.valid) {publishStatus("NO_VALID_TRAJECTORY"); return cmd;}

  const double bearing = std::atan2(carrot.point.y(), carrot.point.x());
  double v_pp, omega_pp;
  if (std::abs(bearing) > rotate_threshold_) {
    // Carrot is well off the heading (e.g. a sharp/initial turn): rotate in place.
    v_pp = 0.0;
    omega_pp = std::clamp(rotate_gain_ * bearing, -max_angular_vel_, max_angular_vel_);
  } else {
    const double curv = purePursuitCurvature(carrot.point);
    v_pp = regulateLinearSpeed(desired_linear_vel_, curv, curvature_threshold_,
        min_approach_speed_);
    v_pp = std::min(v_pp, goalApproachSpeed(dist_to_goal, desired_linear_vel_, lookahead_dist_));
    v_pp = std::clamp(v_pp, 0.0, effectiveMaxSpeed());
    omega_pp = std::clamp(curv * v_pp, -max_angular_vel_, max_angular_vel_);
  }

  // 2. Candidate commands sampled AROUND the pure-pursuit command.
  std::vector<std::pair<double, double>> candidates;
  const double v_factors[] = {1.0, 0.6, 0.3, 0.0};
  const double w_offsets[] = {0.0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9, 1.3, -1.3};
  for (double vf : v_factors) {
    for (double wo : w_offsets) {
      const double v = std::clamp(v_pp * vf, 0.0, effectiveMaxSpeed());
      const double w = std::clamp(omega_pp + wo, -max_angular_vel_, max_angular_vel_);
      candidates.emplace_back(v, w);
    }
  }

  // 3. Social context: only human/TTC/group terms matter here; path-following is the
  // pure-pursuit base + the deviation cost below, re-weighted for the hybrid.
  social_nav_core::ScoringContext ctx;
  ctx.humans = currentHumans();
  ctx.ttc_horizon = ttc_horizon_;
  ctx.ttc_danger_distance = ttc_danger_distance_;
  social_nav_core::CostWeights sw;  // social-only
  sw.goal = 0.0; sw.path = 0.0; sw.progress = 0.0; sw.direction = 0.0;
  sw.velocity = 0.0; sw.smoothness = 0.0; sw.obstacle = 0.0;
  sw.human = weights_.human; sw.ttc = weights_.ttc; sw.group = weights_.group;

  // 4. Pick the safe candidate with the lowest social + deviation cost.
  double best_cost = std::numeric_limits<double>::infinity();
  double v = 0.0, omega = 0.0;
  bool any_safe = false;
  social_nav_core::Trajectory best_traj;
  for (const auto & [cv, cw] : candidates) {
    const auto traj = social_nav_core::rollOut(robot, cv, cw, sampling_.sim_time, sampling_.sim_dt);
    if (trajectoryCollides(traj)) {continue;}
    any_safe = true;
    const double social = social_nav_core::scoreTrajectory(traj, ctx, sw).total;
    const double dev = std::abs(cv - v_pp) / std::max(max_linear_vel_, 1e-3) +
      std::abs(cw - omega_pp) / std::max(max_angular_vel_, 1e-3);
    const double cost = social + deviation_weight_ * dev;
    if (cost < best_cost) {best_cost = cost; v = cv; omega = cw; best_traj = traj;}
  }
  if (!any_safe) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
      "No collision-free trajectory - stopping (NO_VALID_TRAJECTORY)");
    publishStatus("NO_VALID_TRAJECTORY");
    return cmd;  // safe stop -> Nav2 recovery engages
  }

  // 5. Behavior mode from the human situation, with hysteresis.
  social_nav_core::ModeInputs mi;
  mi.num_humans = static_cast<int>(ctx.humans.size());
  const Eigen::Vector2d v_robot = velocity.linear.x *
    Eigen::Vector2d(std::cos(robot.theta), std::sin(robot.theta));
  for (const auto & h : ctx.humans) {
    mi.min_human_clearance = std::min(mi.min_human_clearance, (h.position - r).norm());
    const auto ca =
      social_nav_core::computeClosestApproach(h.position - r, h.velocity - v_robot);
    if (ca.approaching) {mi.min_ttc = std::min(mi.min_ttc, ca.time);}
  }
  current_mode_ = social_nav_core::nextMode(current_mode_, mi, mode_th_);
  const double scale = social_nav_core::modeSpeedScale(current_mode_);

  v *= scale;
  omega *= (current_mode_ == social_nav_core::BehaviorMode::kEmergency) ? 0.0 : 1.0;
  v = std::clamp(v, 0.0, effectiveMaxSpeed());
  omega = std::clamp(omega, -max_angular_vel_, max_angular_vel_);

  cmd.twist.linear.x = v;
  cmd.twist.angular.z = omega;

  // Debug topics: the chosen local trajectory + current behavior mode.
  if (debug_mode_) {
    if (sel_traj_pub_ && sel_traj_pub_->is_activated()) {
      nav_msgs::msg::Path path;
      path.header.stamp = cmd.header.stamp;
      path.header.frame_id = pose.header.frame_id;
      for (const auto & pt : best_traj.points) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = pt.pose.x;
        ps.pose.position.y = pt.pose.y;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
      }
      sel_traj_pub_->publish(path);
    }
    if (mode_pub_ && mode_pub_->is_activated()) {
      std_msgs::msg::String m;
      m.data = social_nav_core::toString(current_mode_);
      mode_pub_->publish(m);
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    logger_, *node_->get_clock(), 700,
    "robot=(%.2f,%.2f,%.0fdeg) bearing=%.0fdeg pp(v=%.2f w=%.2f) "
    "cmd(v=%.2f w=%.2f) humans=%d mode=%s goaldist=%.2f",
    robot.x, robot.y, robot.theta * 180.0 / M_PI, bearing * 180.0 / M_PI,
    v_pp, omega_pp, v, omega, mi.num_humans,
    social_nav_core::toString(current_mode_), dist_to_goal);

  publishStatus(
    current_mode_ == social_nav_core::BehaviorMode::kEmergency ? "EMERGENCY_STOP" :
    (last_humans_stale_ ? "STALE_HUMAN_DATA" : "OK"));

  recordAndPublishMetrics(
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count(),
    mi.num_humans, social_nav_core::toString(current_mode_));

  return cmd;
}

void SocialNavController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

}  // namespace social_nav_controller

PLUGINLIB_EXPORT_CLASS(social_nav_controller::SocialNavController, nav2_core::Controller)
