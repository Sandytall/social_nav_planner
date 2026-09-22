#include "social_nav_human_model/social_zones.hpp"

#include <algorithm>
#include <cmath>

namespace social_nav_human_model
{

double normalizedZoneDistance(
  const Eigen::Vector2d & human_position,
  double human_heading,
  const Eigen::Vector2d & query,
  const ZoneRadii & radii)
{
  // Transform the query into the human's local frame (+x = facing direction).
  const Eigen::Vector2d offset = query - human_position;
  const double c = std::cos(human_heading);
  const double s = std::sin(human_heading);
  const double longitudinal = c * offset.x() + s * offset.y();
  const double lateral = -s * offset.x() + c * offset.y();

  const double rx = longitudinal >= 0.0 ? radii.front : radii.rear;
  const double ry = radii.side;

  // Guard against non-positive radii.
  const double ax = std::max(rx, 1e-6);
  const double ay = std::max(ry, 1e-6);

  const double nx = longitudinal / ax;
  const double ny = lateral / ay;
  return std::sqrt(nx * nx + ny * ny);
}

SocialZone classifyZone(
  const Eigen::Vector2d & human_position,
  double human_heading,
  const Eigen::Vector2d & query,
  const SocialZoneParams & params)
{
  if (normalizedZoneDistance(human_position, human_heading, query, params.personal) <= 1.0) {
    return SocialZone::kPersonal;
  }
  if (normalizedZoneDistance(human_position, human_heading, query, params.comfort) <= 1.0) {
    return SocialZone::kComfort;
  }
  if (normalizedZoneDistance(human_position, human_heading, query, params.caution) <= 1.0) {
    return SocialZone::kCaution;
  }
  return SocialZone::kNone;
}

}  // namespace social_nav_human_model
