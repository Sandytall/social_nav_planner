// Pure, ROS-free path-following geometry for the SocialNav controller (MASTER_PROMPT §25).
//
// Phase 2 uses a regulated-pure-pursuit style command so the robot follows the global
// plan (DoD: "Robot navigates without humans"). Later phases (§16-§19) replace the raw
// command with sampled candidate trajectories scored by the social cost model; this
// geometry remains useful as the goal/path-following term.
#ifndef SOCIAL_NAV_CONTROLLER__PATH_GEOMETRY_HPP_
#define SOCIAL_NAV_CONTROLLER__PATH_GEOMETRY_HPP_

#include <vector>

#include <Eigen/Core>

#include "social_nav_core/types.hpp"

namespace social_nav_controller
{

/// Transform a world-frame point into the robot's base frame given the robot pose.
Eigen::Vector2d toRobotFrame(
  const social_nav_core::Pose2D & robot_pose, const Eigen::Vector2d & world_point);

/// Result of a lookahead ("carrot") search along the plan, in the robot frame.
struct Carrot
{
  bool valid{false};
  Eigen::Vector2d point{0.0, 0.0};
};

/// Find the lookahead point on `path_robot_frame`: the first point at or beyond
/// `lookahead_distance` from the robot origin. If none is that far, the last point is
/// used (approaching the goal). Returns invalid for an empty path.
Carrot findLookaheadPoint(
  const std::vector<Eigen::Vector2d> & path_robot_frame, double lookahead_distance);

/// Pure-pursuit path curvature to reach a carrot in the robot frame:
///   kappa = 2 * y / L^2,  L^2 = x^2 + y^2.
/// Returns 0 for a degenerate (near-zero-distance) carrot.
double purePursuitCurvature(const Eigen::Vector2d & carrot);

/// Regulate linear speed on tight curvature: speed is scaled down as |curvature| grows
/// past `curvature_threshold`, never below `min_speed` (§20 CAUTIOUS-style slowing).
double regulateLinearSpeed(
  double desired_speed, double curvature, double curvature_threshold, double min_speed);

/// Decelerate on final approach so the robot stops at the goal instead of overshooting.
/// Beyond `approach_dist` the desired speed is unchanged; within it the speed scales
/// linearly with the remaining distance to the goal (0 at the goal).
double goalApproachSpeed(double dist_to_goal, double desired_speed, double approach_dist);

}  // namespace social_nav_controller

#endif  // SOCIAL_NAV_CONTROLLER__PATH_GEOMETRY_HPP_
