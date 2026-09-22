#include "social_nav_core/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

#include "social_nav_core/angles.hpp"

namespace social_nav_core
{

Trajectory rollOut(
  const Pose2D & start, double v, double omega, double sim_time, double sim_dt)
{
  Trajectory traj;
  if (sim_dt <= 0.0 || sim_time <= 0.0) {
    return traj;
  }
  const int steps = static_cast<int>(std::floor(sim_time / sim_dt + 1e-9));
  traj.points.reserve(static_cast<std::size_t>(steps) + 1);

  Pose2D pose = start;
  // Include the start pose at t=0 so downstream checks see the current footprint.
  traj.points.push_back({pose, Velocity2D{v, 0.0, omega}, 0.0});
  for (int i = 1; i <= steps; ++i) {
    pose.x += v * std::cos(pose.theta) * sim_dt;
    pose.y += v * std::sin(pose.theta) * sim_dt;
    pose.theta = normalizeAngle(pose.theta + omega * sim_dt);
    traj.points.push_back({pose, Velocity2D{v, 0.0, omega}, i * sim_dt});
  }
  return traj;
}

namespace
{
// Evenly spaced samples across [lo, hi]; a single sample returns the midpoint.
std::vector<double> linspace(double lo, double hi, int n)
{
  std::vector<double> out;
  if (n <= 0 || hi < lo) {
    return out;
  }
  if (n == 1) {
    out.push_back(0.5 * (lo + hi));
    return out;
  }
  out.reserve(static_cast<std::size_t>(n));
  const double step = (hi - lo) / (n - 1);
  for (int i = 0; i < n; ++i) {
    out.push_back(lo + step * i);
  }
  return out;
}
}  // namespace

std::vector<Trajectory> generateTrajectories(
  const Pose2D & start,
  const Velocity2D & current_vel,
  const DiffDriveLimits & limits,
  const SamplingParams & sampling,
  double dt_reachable)
{
  double v_lo = limits.min_v;
  double v_hi = limits.max_v;
  double w_lo = -limits.max_omega;
  double w_hi = limits.max_omega;

  // Constrain the sampled window to what acceleration allows within one period (§18).
  if (dt_reachable > 0.0) {
    v_lo = std::max(v_lo, current_vel.vx - limits.max_accel * dt_reachable);
    v_hi = std::min(v_hi, current_vel.vx + limits.max_accel * dt_reachable);
    w_lo = std::max(w_lo, current_vel.omega - limits.max_alpha * dt_reachable);
    w_hi = std::min(w_hi, current_vel.omega + limits.max_alpha * dt_reachable);
  }

  const auto vs = linspace(v_lo, v_hi, sampling.v_samples);
  const auto ws = linspace(w_lo, w_hi, sampling.omega_samples);

  std::vector<Trajectory> trajectories;
  trajectories.reserve(vs.size() * ws.size());
  for (double v : vs) {
    for (double w : ws) {
      trajectories.push_back(rollOut(start, v, w, sampling.sim_time, sampling.sim_dt));
    }
  }
  return trajectories;
}

}  // namespace social_nav_core
