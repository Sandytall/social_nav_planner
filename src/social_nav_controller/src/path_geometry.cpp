#include "social_nav_controller/path_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace social_nav_controller
{

Eigen::Vector2d toRobotFrame(
  const social_nav_core::Pose2D & robot_pose, const Eigen::Vector2d & world_point)
{
  const double dx = world_point.x() - robot_pose.x;
  const double dy = world_point.y() - robot_pose.y;
  const double c = std::cos(robot_pose.theta);
  const double s = std::sin(robot_pose.theta);
  // Rotate the world offset by -theta into the robot frame.
  return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
}

Carrot findLookaheadPoint(
  const std::vector<Eigen::Vector2d> & path_robot_frame, double lookahead_distance)
{
  Carrot carrot;
  if (path_robot_frame.empty()) {
    return carrot;  // invalid
  }
  for (const auto & p : path_robot_frame) {
    if (p.norm() >= lookahead_distance) {
      carrot.valid = true;
      carrot.point = p;
      return carrot;
    }
  }
  // No point is far enough: aim at the last (goal-most) point.
  carrot.valid = true;
  carrot.point = path_robot_frame.back();
  return carrot;
}

double purePursuitCurvature(const Eigen::Vector2d & carrot)
{
  const double l2 = carrot.squaredNorm();
  if (l2 < 1e-6) {
    return 0.0;
  }
  return 2.0 * carrot.y() / l2;
}

double regulateLinearSpeed(
  double desired_speed, double curvature, double curvature_threshold, double min_speed)
{
  const double abs_k = std::abs(curvature);
  if (abs_k <= curvature_threshold || curvature_threshold <= 0.0) {
    return desired_speed;
  }
  // Inverse scaling past the threshold; clamp to [min_speed, desired_speed].
  const double scaled = desired_speed * (curvature_threshold / abs_k);
  return std::clamp(scaled, min_speed, desired_speed);
}

double goalApproachSpeed(double dist_to_goal, double desired_speed, double approach_dist)
{
  if (approach_dist <= 0.0 || dist_to_goal >= approach_dist) {
    return desired_speed;
  }
  const double frac = std::clamp(dist_to_goal / approach_dist, 0.0, 1.0);
  return desired_speed * frac;
}

}  // namespace social_nav_controller
