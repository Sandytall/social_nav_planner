#include "social_nav_core/collision.hpp"

namespace social_nav_core
{

double minObstacleClearance(
  const Trajectory & traj, const std::vector<Eigen::Vector2d> & obstacles)
{
  double best = std::numeric_limits<double>::infinity();
  if (traj.points.empty() || obstacles.empty()) {
    return best;
  }
  for (const auto & pt : traj.points) {
    const Eigen::Vector2d p(pt.pose.x, pt.pose.y);
    for (const auto & obs : obstacles) {
      const double d = (p - obs).norm();
      if (d < best) {
        best = d;
      }
    }
  }
  return best;
}

bool inCollision(
  const Trajectory & traj,
  const std::vector<Eigen::Vector2d> & obstacles,
  double robot_radius)
{
  if (traj.points.empty() || obstacles.empty()) {
    return false;
  }
  const double r2 = robot_radius * robot_radius;
  for (const auto & pt : traj.points) {
    const Eigen::Vector2d p(pt.pose.x, pt.pose.y);
    for (const auto & obs : obstacles) {
      if ((p - obs).squaredNorm() <= r2) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace social_nav_core
