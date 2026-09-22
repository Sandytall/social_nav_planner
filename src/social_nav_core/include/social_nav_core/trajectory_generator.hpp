// Candidate trajectory generation by velocity sampling.
//
// For a differential-drive robot we sample (v, omega) pairs within the robot's limits and
// forward-simulate a unicycle model to produce candidate trajectories. The planner then
// checks each for collisions and scores the survivors.
#ifndef SOCIAL_NAV_CORE__TRAJECTORY_GENERATOR_HPP_
#define SOCIAL_NAV_CORE__TRAJECTORY_GENERATOR_HPP_

#include <vector>

#include "social_nav_core/types.hpp"

namespace social_nav_core
{

/// Differential-drive velocity/acceleration limits (hard constraints).
struct DiffDriveLimits
{
  double max_v{0.8};
  double min_v{0.0};       ///< allow 0 (in-place rotate) but not reverse by default
  double max_omega{1.2};
  double max_accel{0.5};   ///< linear, m/s^2
  double max_alpha{1.0};   ///< angular, rad/s^2
};

/// Sampling resolution and roll-out horizon.
struct SamplingParams
{
  int v_samples{7};
  int omega_samples{15};
  double sim_time{2.5};    ///< seconds to roll out
  double sim_dt{0.1};
};

/// Generate candidate trajectories from the robot's current state.
///
/// Velocity samples are limited both by the absolute limits and by what is reachable
/// from `current_vel` within one control period (`dt_reachable`), so acceleration limits
/// are respected. Pass dt_reachable <= 0 to ignore reachability (sample full range).
std::vector<Trajectory> generateTrajectories(
  const Pose2D & start,
  const Velocity2D & current_vel,
  const DiffDriveLimits & limits,
  const SamplingParams & sampling,
  double dt_reachable = 0.0);

/// Forward-simulate a single (v, omega) command from `start` (unicycle model).
Trajectory rollOut(
  const Pose2D & start, double v, double omega, double sim_time, double sim_dt);

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__TRAJECTORY_GENERATOR_HPP_
