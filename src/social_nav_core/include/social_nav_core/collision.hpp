// Trajectory collision checking.
//
// Safety is a HARD constraint: a trajectory whose footprint touches an obstacle is
// rejected outright, never traded off against a lower social cost. Here we model the
// robot footprint as a disc of `robot_radius` and obstacles as points (e.g. lethal
// costmap cells or laser returns).
#ifndef SOCIAL_NAV_CORE__COLLISION_HPP_
#define SOCIAL_NAV_CORE__COLLISION_HPP_

#include <limits>
#include <vector>

#include <Eigen/Core>

#include "social_nav_core/types.hpp"

namespace social_nav_core
{

/// Minimum distance from any point on the trajectory to the nearest obstacle.
/// Returns +inf if there are no obstacles or the trajectory is empty.
double minObstacleClearance(
  const Trajectory & traj, const std::vector<Eigen::Vector2d> & obstacles);

/// True if the disc footprint (radius `robot_radius`) collides with any obstacle at any
/// point along the trajectory. A hard-reject test.
bool inCollision(
  const Trajectory & traj,
  const std::vector<Eigen::Vector2d> & obstacles,
  double robot_radius);

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__COLLISION_HPP_
