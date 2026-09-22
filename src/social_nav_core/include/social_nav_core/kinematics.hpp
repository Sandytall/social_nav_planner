// Relative-motion reasoning: time/distance to closest approach (MASTER_PROMPT §11, §12).
//
// These functions implement the core insight of §11: a human 1.5 m away moving AWAY is
// not the same as one 1.5 m away moving TOWARD the robot. We reason about the constant-
// velocity relative motion, not raw Euclidean distance.
#ifndef SOCIAL_NAV_CORE__KINEMATICS_HPP_
#define SOCIAL_NAV_CORE__KINEMATICS_HPP_

#include <Eigen/Core>

namespace social_nav_core
{

/// Result of a closest-approach query between two constant-velocity agents.
struct ClosestApproach
{
  /// Time of the unconstrained minimum-distance instant, relative to now.
  /// May be negative, meaning the closest approach already happened (agents are
  /// separating). Callers deciding safety should look at `approaching`.
  double time{0.0};

  /// Minimum separation over future time (t >= 0). If the agents are separating,
  /// this equals the current separation.
  double distance{0.0};

  /// True if the closest approach lies in the future (time > 0), i.e. the agents
  /// are currently getting closer.
  bool approaching{false};
};

/// Compute the closest approach between two agents under a constant relative velocity.
///
/// @param rel_position  Position of the other agent relative to this one, now (H - R).
/// @param rel_velocity  Velocity of the other agent relative to this one (v_h - v_r).
///
/// If there is no relative motion the current separation is returned with time 0.
ClosestApproach computeClosestApproach(
  const Eigen::Vector2d & rel_position,
  const Eigen::Vector2d & rel_velocity);

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__KINEMATICS_HPP_
