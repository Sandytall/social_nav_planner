#include "social_nav_core/kinematics.hpp"

namespace social_nav_core
{

ClosestApproach computeClosestApproach(
  const Eigen::Vector2d & rel_position,
  const Eigen::Vector2d & rel_velocity)
{
  ClosestApproach result;

  const double speed_sq = rel_velocity.squaredNorm();

  // No relative motion: the closest approach is the current separation, forever.
  if (speed_sq < 1e-9) {
    result.time = 0.0;
    result.distance = rel_position.norm();
    result.approaching = false;
    return result;
  }

  // Minimise |rel_position + rel_velocity * t|^2 over t.
  // d/dt = 0  =>  t* = -(rel_position . rel_velocity) / |rel_velocity|^2.
  const double t_star = -rel_position.dot(rel_velocity) / speed_sq;
  result.time = t_star;
  result.approaching = t_star > 0.0;

  // Future minimum separation: clamp the evaluation time to t >= 0 (§11 cares about
  // what is about to happen, not what already happened).
  const double t_eval = t_star > 0.0 ? t_star : 0.0;
  result.distance = (rel_position + rel_velocity * t_eval).norm();

  return result;
}

}  // namespace social_nav_core
