// Layered anisotropic social zones.
//
// A human is not a circle. We model three nested zones - personal, comfort, caution -
// each an ellipse in the human's heading frame with independent front/side/rear radii so
// front clearance > side clearance > rear clearance. A query point is classified as
// the innermost zone it falls inside.
#ifndef SOCIAL_NAV_HUMAN_MODEL__SOCIAL_ZONES_HPP_
#define SOCIAL_NAV_HUMAN_MODEL__SOCIAL_ZONES_HPP_

#include <Eigen/Core>

namespace social_nav_human_model
{

/// Anisotropic radii (metres) of one elliptical zone.
struct ZoneRadii
{
  double front{1.0};
  double side{0.75};
  double rear{0.5};
};

/// The three nested zones. `personal` must be inside `comfort` inside `caution`.
struct SocialZoneParams
{
  ZoneRadii personal{0.5, 0.4, 0.3};
  ZoneRadii comfort{1.2, 0.9, 0.6};
  ZoneRadii caution{2.0, 1.5, 1.0};
};

enum class SocialZone
{
  kNone = 0,
  kCaution,
  kComfort,
  kPersonal,
};

/// Normalised anisotropic distance of `query` from a human at `human_position` facing
/// `human_heading`, for the ellipse `radii`. <= 1 means inside that ellipse.
double normalizedZoneDistance(
  const Eigen::Vector2d & human_position,
  double human_heading,
  const Eigen::Vector2d & query,
  const ZoneRadii & radii);

/// Classify `query` into the innermost social zone it lies within.
SocialZone classifyZone(
  const Eigen::Vector2d & human_position,
  double human_heading,
  const Eigen::Vector2d & query,
  const SocialZoneParams & params);

}  // namespace social_nav_human_model

#endif  // SOCIAL_NAV_HUMAN_MODEL__SOCIAL_ZONES_HPP_
