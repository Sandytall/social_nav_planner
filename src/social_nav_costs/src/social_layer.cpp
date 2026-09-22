#include "social_nav_costs/social_layer.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "nav2_costmap_2d/cost_values.hpp"
#include "social_nav_core/social_cost.hpp"

namespace social_nav_costs
{

void SocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SocialLayer: parent node expired");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("humans_topic", rclcpp::ParameterValue(std::string("/social_nav/humans")));
  declareParameter("front_sigma", rclcpp::ParameterValue(1.0));
  declareParameter("side_sigma", rclcpp::ParameterValue(0.7));
  declareParameter("rear_sigma", rclcpp::ParameterValue(0.5));
  declareParameter("max_cost", rclcpp::ParameterValue(200.0));
  declareParameter("cutoff_distance", rclcpp::ParameterValue(3.0));
  declareParameter("human_timeout", rclcpp::ParameterValue(1.0));

  node->get_parameter(name_ + "." + "enabled", enabled_param_);
  std::string topic;
  node->get_parameter(name_ + "." + "humans_topic", topic);
  node->get_parameter(name_ + "." + "front_sigma", front_sigma_);
  node->get_parameter(name_ + "." + "side_sigma", side_sigma_);
  node->get_parameter(name_ + "." + "rear_sigma", rear_sigma_);
  node->get_parameter(name_ + "." + "max_cost", max_cost_);
  node->get_parameter(name_ + "." + "cutoff_distance", cutoff_distance_);
  node->get_parameter(name_ + "." + "human_timeout", human_timeout_);

  enabled_ = enabled_param_;
  current_ = true;

  sub_ = node->create_subscription<social_nav_msgs::msg::HumanArray>(
    topic, rclcpp::QoS(10),
    std::bind(&SocialLayer::humansCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "SocialLayer '%s' initialized (topic=%s, sigmas f/s/r=%.1f/%.1f/%.1f, max_cost=%.0f)",
    name_.c_str(), topic.c_str(), front_sigma_, side_sigma_, rear_sigma_, max_cost_);
}

void SocialLayer::humansCallback(social_nav_msgs::msg::HumanArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  latest_ = *msg;
}

std::vector<SocialLayer::Person> SocialLayer::currentPeople()
{
  std::vector<Person> out;
  social_nav_msgs::msg::HumanArray snap;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    snap = latest_;
  }
  if (snap.humans.empty()) {
    return out;
  }
  auto node = node_.lock();
  if (node) {
    const double age =
      node->now().seconds() - rclcpp::Time(snap.header.stamp).seconds();
    if (age > human_timeout_) {  // §54: ignore stale data
      return out;
    }
  }
  for (const auto & h : snap.humans) {
    const auto & q = h.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double x = h.pose.position.x;
    const double y = h.pose.position.y;
    if (std::isfinite(x) && std::isfinite(y)) {  // §54
      out.push_back({x, y, yaw});
    }
  }
  return out;
}

void SocialLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  people_ = currentPeople();

  // Re-touch the previous footprint so cost from people who moved/left gets cleared.
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  bool any = false;
  double lo_x = 0, lo_y = 0, hi_x = 0, hi_y = 0;
  for (const auto & p : people_) {
    const double a = p.x - cutoff_distance_, b = p.x + cutoff_distance_;
    const double c = p.y - cutoff_distance_, d = p.y + cutoff_distance_;
    if (!any) {lo_x = a; hi_x = b; lo_y = c; hi_y = d; any = true;} else {
      lo_x = std::min(lo_x, a); hi_x = std::max(hi_x, b);
      lo_y = std::min(lo_y, c); hi_y = std::max(hi_y, d);
    }
  }
  if (any) {
    *min_x = std::min(*min_x, lo_x);
    *min_y = std::min(*min_y, lo_y);
    *max_x = std::max(*max_x, hi_x);
    *max_y = std::max(*max_y, hi_y);
    last_min_x_ = lo_x; last_min_y_ = lo_y; last_max_x_ = hi_x; last_max_y_ = hi_y;
    has_last_bounds_ = true;
  } else {
    has_last_bounds_ = false;
  }
}

void SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || people_.empty()) {
    return;
  }
  const social_nav_core::SocialZoneParams zone{front_sigma_, side_sigma_, rear_sigma_};
  const double cutoff2 = cutoff_distance_ * cutoff_distance_;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j),
        wx, wy);

      double best01 = 0.0;
      for (const auto & p : people_) {
        const double dx = wx - p.x, dy = wy - p.y;
        if (dx * dx + dy * dy > cutoff2) {
          continue;
        }
        best01 = std::max(best01,
            social_nav_core::anisotropicSocialCost(
              Eigen::Vector2d(p.x, p.y), p.yaw, Eigen::Vector2d(wx, wy), zone));
      }
      if (best01 <= 0.02) {
        continue;
      }
      const auto add = static_cast<unsigned char>(
        std::clamp(best01 * max_cost_, 0.0, max_cost_));
      const unsigned char old = master_grid.getCost(i, j);
      // Never lower existing cost, and never overwrite real obstacles (keep them lethal).
      if (old == nav2_costmap_2d::NO_INFORMATION ||
        old == nav2_costmap_2d::LETHAL_OBSTACLE ||
        old == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }
      if (add > old) {
        master_grid.setCost(i, j, add);
      }
    }
  }
}

}  // namespace social_nav_costs

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(social_nav_costs::SocialLayer, nav2_costmap_2d::Layer)
