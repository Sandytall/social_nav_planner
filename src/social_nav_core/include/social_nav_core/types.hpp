// Core value types shared across the SocialNav planner.
// Header-only, dependency-light: no ROS or Nav2 types leak into the core math so the
// math can be unit-tested in isolation.
#ifndef SOCIAL_NAV_CORE__TYPES_HPP_
#define SOCIAL_NAV_CORE__TYPES_HPP_

#include <cstddef>
#include <vector>

namespace social_nav_core
{

/// A planar pose. theta is in radians, measured CCW from the +x axis.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/// A planar velocity. vy is only meaningful for holonomic robots; for
/// differential drive it stays 0 and only vx/omega are sampled.
struct Velocity2D
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

/// One sample along a simulated candidate trajectory.
struct TrajectoryPoint
{
  Pose2D pose;
  Velocity2D velocity;
  double time{0.0};  ///< Seconds from the start of the trajectory.
};

/// A forward-simulated candidate trajectory.
struct Trajectory
{
  std::vector<TrajectoryPoint> points;

  bool empty() const { return points.empty(); }
  std::size_t size() const { return points.size(); }
};

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__TYPES_HPP_
