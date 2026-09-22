#include "social_nav_core/social_cost.hpp"

#include <algorithm>
#include <cmath>

namespace social_nav_core
{

double anisotropicSocialCost(
  const Eigen::Vector2d & human_position,
  double human_heading,
  const Eigen::Vector2d & query,
  const SocialZoneParams & params)
{
  // Transform the query point into the human's local frame: rotate the world
  // offset by -heading so +x points along the human's facing direction.
  const Eigen::Vector2d offset = query - human_position;
  const double c = std::cos(human_heading);
  const double s = std::sin(human_heading);
  const double longitudinal = c * offset.x() + s * offset.y();   // x: forward(+)/back(-)
  const double lateral = -s * offset.x() + c * offset.y();       // y: left/right

  // sigma_x depends on whether the point is in front of or behind the human.
  const double sigma_x = longitudinal >= 0.0 ? params.front_sigma : params.rear_sigma;
  const double sigma_y = params.side_sigma;

  // Guard against non-positive sigmas so a misconfiguration cannot divide by zero.
  const double sx = std::max(sigma_x, 1e-6);
  const double sy = std::max(sigma_y, 1e-6);

  const double exponent =
    (longitudinal * longitudinal) / (2.0 * sx * sx) +
    (lateral * lateral) / (2.0 * sy * sy);

  return std::exp(-exponent);
}

}  // namespace social_nav_core
