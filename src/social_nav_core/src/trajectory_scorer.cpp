#include "social_nav_core/trajectory_scorer.hpp"

#include <cmath>
#include <limits>

#include "social_nav_core/angles.hpp"
#include "social_nav_core/kinematics.hpp"

namespace social_nav_core
{

namespace
{
Eigen::Vector2d xy(const TrajectoryPoint & p) { return {p.pose.x, p.pose.y}; }

double nearest(const Eigen::Vector2d & p, const std::vector<Eigen::Vector2d> & pts)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & q : pts) {
    best = std::min(best, (p - q).norm());
  }
  return best;
}
}  // namespace

CostBreakdown scoreTrajectory(
  const Trajectory & traj, const ScoringContext & ctx, const CostWeights & w)
{
  CostBreakdown cb;
  if (traj.points.empty()) {
    cb.total = std::numeric_limits<double>::infinity();
    return cb;
  }

  const auto & front = traj.points.front();
  const auto & back = traj.points.back();
  const Eigen::Vector2d start = xy(front);
  const Eigen::Vector2d end = xy(back);
  const double v = front.velocity.vx;
  const double omega = front.velocity.omega;
  const double n = static_cast<double>(traj.points.size());

  // Goal: how far the endpoint is from the (local) goal.
  const double goal_d = (end - ctx.goal).norm();
  cb.goal = goal_d;

  // Progress: ratio of remaining distance to starting distance (<1 => made progress).
  const double start_d = (start - ctx.goal).norm();
  cb.progress = goal_d / (start_d + 1e-3);

  // Path following: mean distance of trajectory points to the reference path.
  if (!ctx.path.empty()) {
    double sum = 0.0;
    for (const auto & pt : traj.points) {
      sum += nearest(xy(pt), ctx.path);
    }
    cb.path = sum / n;
  }

  // Obstacle proximity (collisions already hard-rejected; this keeps a margin).
  const double infl = std::max(ctx.obstacle_influence, 1e-3);
  if (!ctx.obstacles.empty()) {
    for (const auto & pt : traj.points) {
      const double d = nearest(xy(pt), ctx.obstacles);
      if (d < infl) {
        cb.obstacle += (infl - d) / infl;
      }
    }
  }

  // Human social cost: anisotropic cost summed over points and humans.
  for (const auto & h : ctx.humans) {
    for (const auto & pt : traj.points) {
      cb.human += anisotropicSocialCost(h.position, h.heading, xy(pt), h.zone);
    }
  }

  // Time-to-collision with each human, using the robot's initial heading velocity.
  const Eigen::Vector2d v_robot = v * Eigen::Vector2d(std::cos(front.pose.theta),
      std::sin(front.pose.theta));
  for (const auto & h : ctx.humans) {
    const ClosestApproach ca =
      computeClosestApproach(h.position - start, h.velocity - v_robot);
    if (ca.approaching && ca.time < ctx.ttc_horizon && ca.distance < ctx.ttc_danger_distance) {
      const double time_term = (ctx.ttc_horizon - ca.time) / ctx.ttc_horizon;
      const double dist_term = (ctx.ttc_danger_distance - ca.distance) / ctx.ttc_danger_distance;
      cb.ttc += time_term * dist_term;
    }
  }

  // Group intrusion: penalise points inside a group's bounding disc.
  for (const auto & g : ctx.groups) {
    if (g.radius <= 0.0) {
      continue;
    }
    for (const auto & pt : traj.points) {
      const double d = (xy(pt) - g.centroid).norm();
      if (d < g.radius) {
        cb.group += (g.radius - d) / g.radius;
      }
    }
  }

  // Direction: misalignment of the final heading with the bearing to the goal.
  const double bearing = std::atan2(ctx.goal.y() - end.y(), ctx.goal.x() - end.x());
  cb.direction = 0.5 * (1.0 - std::cos(normalizeAngle(bearing - back.pose.theta)));

  // Smoothness: penalise angular rate.
  cb.smoothness = std::abs(omega);

  // Velocity: prefer moving near max speed (0 at max, 1 at standstill).
  cb.velocity = (ctx.max_linear_velocity - v) / std::max(ctx.max_linear_velocity, 1e-3);

  cb.total =
    w.goal * cb.goal + w.path * cb.path + w.obstacle * cb.obstacle +
    w.human * cb.human + w.ttc * cb.ttc + w.group * cb.group +
    w.direction * cb.direction + w.smoothness * cb.smoothness +
    w.velocity * cb.velocity + w.progress * cb.progress;
  return cb;
}

int selectBestTrajectory(
  const std::vector<Trajectory> & trajectories,
  const ScoringContext & ctx,
  const CostWeights & weights,
  std::vector<CostBreakdown> * breakdowns)
{
  if (breakdowns) {
    breakdowns->clear();
    breakdowns->reserve(trajectories.size());
  }
  int best_idx = -1;
  double best_total = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectories.size(); ++i) {
    const CostBreakdown cb = scoreTrajectory(trajectories[i], ctx, weights);
    if (breakdowns) {
      breakdowns->push_back(cb);
    }
    if (cb.total < best_total) {
      best_total = cb.total;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

const char * toString(BehaviorMode mode)
{
  switch (mode) {
    case BehaviorMode::kNormal: return "NORMAL";
    case BehaviorMode::kCautious: return "CAUTIOUS";
    case BehaviorMode::kCrowded: return "CROWDED";
    case BehaviorMode::kEmergency: return "EMERGENCY";
  }
  return "NORMAL";
}

namespace
{
// Classify inputs into a mode. `margin` widens the clearance thresholds (margin > 1
// makes calmer modes harder to reach => used for hysteresis on de-escalation).
BehaviorMode classify(const ModeInputs & in, const ModeThresholds & th, double margin)
{
  if (in.min_human_clearance < th.emergency_clearance * margin || in.min_ttc < th.emergency_ttc) {
    return BehaviorMode::kEmergency;
  }
  if (in.min_human_clearance < th.crowded_clearance * margin ||
    in.num_humans >= th.crowded_num_humans)
  {
    return BehaviorMode::kCrowded;
  }
  if (in.min_human_clearance < th.cautious_clearance * margin) {
    return BehaviorMode::kCautious;
  }
  return BehaviorMode::kNormal;
}
}  // namespace

BehaviorMode nextMode(
  BehaviorMode current, const ModeInputs & in, const ModeThresholds & th)
{
  const int cur = static_cast<int>(current);
  // Escalate promptly on the base thresholds.
  const BehaviorMode up = classify(in, th, 1.0);
  if (static_cast<int>(up) > cur) {
    return up;
  }
  // De-escalate only when the widened thresholds also say we are calmer (hysteresis).
  const BehaviorMode down = classify(in, th, 1.0 + th.hysteresis);
  if (static_cast<int>(down) < cur) {
    return down;
  }
  return current;
}

double modeSpeedScale(BehaviorMode mode)
{
  switch (mode) {
    case BehaviorMode::kNormal: return 1.0;
    case BehaviorMode::kCautious: return 0.6;
    case BehaviorMode::kCrowded: return 0.4;
    case BehaviorMode::kEmergency: return 0.0;
  }
  return 1.0;
}

}  // namespace social_nav_core
