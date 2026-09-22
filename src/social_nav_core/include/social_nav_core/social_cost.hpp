// Anisotropic social cost around a human.
//
// A human is NOT a circle. The comfortable clearance is larger in front than to the
// side, and larger to the side than behind (front > side > rear). We model this as
// an anisotropic Gaussian in the human's own heading frame:
//
//   C = exp( -( x^2 / (2 sigma_x^2) + y^2 / (2 sigma_y^2) ) )
//
// where x is the longitudinal (forward) offset and y the lateral offset of the query
// point in the human's frame; sigma_x switches between a front and rear value depending
// on which side of the human the point lies.
#ifndef SOCIAL_NAV_CORE__SOCIAL_COST_HPP_
#define SOCIAL_NAV_CORE__SOCIAL_COST_HPP_

#include <Eigen/Core>

namespace social_nav_core
{

/// Shape of one human's social zone (front/side/rear_sigma parameters).
/// All sigmas are standard deviations in metres and must be strictly positive.
struct SocialZoneParams
{
  double front_sigma{1.5};  ///< Larger => more space demanded in front.
  double side_sigma{1.0};
  double rear_sigma{0.7};   ///< Smaller => the robot may pass closer behind.
};

/// Cost in [0, 1] of the robot occupying `query` given a human at `human_position`
/// facing `human_heading` (radians, world frame).
///
/// Returns 1.0 at the human's centre and decays anisotropically outward. The query and
/// human position share the same (world/planning) frame.
double anisotropicSocialCost(
  const Eigen::Vector2d & human_position,
  double human_heading,
  const Eigen::Vector2d & query,
  const SocialZoneParams & params);

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__SOCIAL_COST_HPP_
