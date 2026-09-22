#include "social_nav_costs/restricted_zone_layer.hpp"

#include <algorithm>

#include "nav2_costmap_2d/cost_values.hpp"

namespace social_nav_costs
{

std::vector<RestrictedZoneLayer::Zone> RestrictedZoneLayer::parseZones(
  const std::vector<double> & flat, const rclcpp::Logger & logger)
{
  std::vector<Zone> out;
  const size_t remainder = flat.size() % 4;
  if (remainder != 0) {
    RCLCPP_WARN(
      logger,
      "RestrictedZoneLayer: 'zones' has %zu values, not a multiple of 4 "
      "[xmin,ymin,xmax,ymax]; dropping the trailing %zu.",
      flat.size(), remainder);
  }
  const size_t n = flat.size() - remainder;
  out.reserve(n / 4);
  for (size_t k = 0; k < n; k += 4) {
    Zone z;
    // Normalize corner order so a rectangle given as [max,min] still works.
    z.xmin = std::min(flat[k], flat[k + 2]);
    z.xmax = std::max(flat[k], flat[k + 2]);
    z.ymin = std::min(flat[k + 1], flat[k + 3]);
    z.ymax = std::max(flat[k + 1], flat[k + 3]);
    out.push_back(z);
  }
  return out;
}

void RestrictedZoneLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RestrictedZoneLayer: parent node expired");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("cost", rclcpp::ParameterValue(200.0));
  // Flat list [xmin,ymin,xmax,ymax, ...] in the costmap global (map) frame. Default empty so
  // the layer is a safe no-op when loaded in a world that defines no restricted zones.
  declareParameter("zones", rclcpp::ParameterValue(std::vector<double>{}));

  node->get_parameter(name_ + "." + "enabled", enabled_param_);
  node->get_parameter(name_ + "." + "cost", cost_);
  std::vector<double> flat;
  node->get_parameter(name_ + "." + "zones", flat);

  zones_ = parseZones(flat, node->get_logger());
  enabled_ = enabled_param_;
  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "RestrictedZoneLayer '%s' initialized (%zu zone(s), cost=%.0f, enabled=%s)",
    name_.c_str(), zones_.size(), cost_, enabled_ ? "true" : "false");
}

void RestrictedZoneLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || zones_.empty()) {
    return;
  }
  // The zones are static, but the global costmap is a rolling window; re-touch each zone's
  // full extent every cycle so the cost is (re)stamped as the window scrolls over it.
  for (const auto & z : zones_) {
    *min_x = std::min(*min_x, z.xmin);
    *min_y = std::min(*min_y, z.ymin);
    *max_x = std::max(*max_x, z.xmax);
    *max_y = std::max(*max_y, z.ymax);
  }
}

void RestrictedZoneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || zones_.empty()) {
    return;
  }
  // Keep the stamp sub-inscribed: a deterrent the planner routes around, never a hard wall.
  const auto cost = static_cast<unsigned char>(
    std::clamp(
      cost_, 0.0,
      static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1.0));

  for (const auto & z : zones_) {
    // Map the zone corners to cell indices (clamped to the grid), then to the window this
    // update touches, so we only write cells inside both the zone and the active region.
    int lo_i = 0, lo_j = 0, hi_i = 0, hi_j = 0;
    master_grid.worldToMapEnforceBounds(z.xmin, z.ymin, lo_i, lo_j);
    master_grid.worldToMapEnforceBounds(z.xmax, z.ymax, hi_i, hi_j);
    lo_i = std::max(lo_i, min_i);
    lo_j = std::max(lo_j, min_j);
    hi_i = std::min(hi_i, max_i - 1);
    hi_j = std::min(hi_j, max_j - 1);

    for (int j = lo_j; j <= hi_j; ++j) {
      for (int i = lo_i; i <= hi_i; ++i) {
        const unsigned char old = master_grid.getCost(i, j);
        // Never overwrite a real (lethal / inscribed) obstacle - keep sensed hazards hard.
        if (old == nav2_costmap_2d::LETHAL_OBSTACLE ||
          old == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          continue;
        }
        // Raise free/unknown cells to the keep-out cost; never lower an already-higher cost.
        if (old == nav2_costmap_2d::NO_INFORMATION || cost > old) {
          master_grid.setCost(i, j, cost);
        }
      }
    }
  }
}

}  // namespace social_nav_costs

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(social_nav_costs::RestrictedZoneLayer, nav2_costmap_2d::Layer)
