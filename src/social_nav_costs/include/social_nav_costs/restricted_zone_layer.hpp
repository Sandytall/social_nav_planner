// RestrictedZoneLayer: a Nav2 costmap layer that stamps configured axis-aligned rectangles
// (map frame) into the costmap at a high but sub-lethal cost. Added to the GLOBAL costmap it
// makes the global planner route AROUND industrial restricted areas and heavy-machinery
// keep-outs, while the cost stays below the inscribed/lethal band so it is a strong deterrent
// rather than a hard wall. The zones are pure costmap cost: no planner logic keys off them, so
// the SocialNav planner stays fully responsible for human-aware navigation.
#ifndef SOCIAL_NAV_COSTS__RESTRICTED_ZONE_LAYER_HPP_
#define SOCIAL_NAV_COSTS__RESTRICTED_ZONE_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace social_nav_costs
{

class RestrictedZoneLayer : public nav2_costmap_2d::Layer
{
public:
  RestrictedZoneLayer() = default;

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

  /// An axis-aligned restricted rectangle in the costmap global (map) frame, min <= max.
  struct Zone
  {
    double xmin{0.0};
    double ymin{0.0};
    double xmax{0.0};
    double ymax{0.0};
  };

  /// Parse the flat [xmin,ymin,xmax,ymax, ...] parameter into normalized zones. Corner order
  /// is normalized (min/max sorted) so callers need not order them. A tail that is not a
  /// multiple of four is dropped with a warning. Static + header-visible so it can be unit
  /// tested without a running node.
  static std::vector<Zone> parseZones(
    const std::vector<double> & flat, const rclcpp::Logger & logger);

private:
  std::vector<Zone> zones_;
  double cost_{200.0};       ///< inserted cost (< 253 lethal => strong deterrent, not a wall)
  bool enabled_param_{true};
};

}  // namespace social_nav_costs

#endif  // SOCIAL_NAV_COSTS__RESTRICTED_ZONE_LAYER_HPP_
