// Social trajectory scoring (MASTER_PROMPT §19) and behavior modes (§20, §23).
//
// After unsafe candidates are hard-rejected (§17-18), the survivors are scored by a
// weighted sum of independently-measurable cost terms (§1, §19). Safety is NOT in this
// sum - it is a prior hard constraint. Lower total score is better.
//
//   J = w_goal*Goal + w_path*Path + w_obs*Obstacle + w_human*HumanClearance
//     + w_ttc*TTC + w_group*GroupIntrusion + w_dir*Direction + w_smooth*Smoothness
//     + w_vel*Velocity + w_progress*Progress
#ifndef SOCIAL_NAV_CORE__TRAJECTORY_SCORER_HPP_
#define SOCIAL_NAV_CORE__TRAJECTORY_SCORER_HPP_

#include <vector>

#include <Eigen/Core>

#include "social_nav_core/social_cost.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav_core
{

/// A human as seen by the scorer (world frame), with its social-zone shape.
struct ScoredHuman
{
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double heading{0.0};
  SocialZoneParams zone{};
};

/// A group to avoid intruding into (§13).
struct ScoredGroup
{
  Eigen::Vector2d centroid{0.0, 0.0};
  double radius{0.0};
};

/// Per-term cost weights (§1, §26). All configurable; no hard-coded behaviour.
struct CostWeights
{
  double goal{1.0};
  double path{2.0};
  double obstacle{10.0};
  double human{5.0};
  double ttc{20.0};
  double group{6.0};
  double direction{1.0};
  double smoothness{1.0};
  double velocity{1.0};
  double progress{2.0};
};

/// The individual cost terms for one trajectory (§19: each independently measurable).
struct CostBreakdown
{
  double goal{0.0};
  double path{0.0};
  double obstacle{0.0};
  double human{0.0};
  double ttc{0.0};
  double group{0.0};
  double direction{0.0};
  double smoothness{0.0};
  double velocity{0.0};
  double progress{0.0};
  double total{0.0};
};

/// Everything the scorer needs about the world for one planning cycle.
struct ScoringContext
{
  Eigen::Vector2d goal{0.0, 0.0};              ///< local goal (world frame)
  std::vector<Eigen::Vector2d> path;           ///< reference path points (world frame)
  std::vector<Eigen::Vector2d> obstacles;      ///< lethal points (world frame)
  std::vector<ScoredHuman> humans;
  std::vector<ScoredGroup> groups;
  double max_linear_velocity{0.8};             ///< for the velocity term
  double ttc_horizon{4.0};                     ///< seconds; TTC beyond this is ignored
  double ttc_danger_distance{0.8};             ///< DCA below this is dangerous
  double obstacle_influence{1.0};              ///< metres; obstacle proximity penalty range
};

/// Score one trajectory. Returns the weighted breakdown (does NOT check safety - the
/// caller rejects unsafe trajectories first, §18).
CostBreakdown scoreTrajectory(
  const Trajectory & traj, const ScoringContext & ctx, const CostWeights & weights);

/// Index of the lowest-total-cost trajectory among `trajectories`, or -1 if the input is
/// empty. `breakdowns` (if non-null) is filled with each trajectory's breakdown, for the
/// debug output required by §19/§32.
int selectBestTrajectory(
  const std::vector<Trajectory> & trajectories,
  const ScoringContext & ctx,
  const CostWeights & weights,
  std::vector<CostBreakdown> * breakdowns = nullptr);

// ===== Behavior modes (§20) with hysteresis (§23) =====

enum class BehaviorMode
{
  kNormal = 0,
  kCautious,
  kCrowded,
  kEmergency,
};

const char * toString(BehaviorMode mode);

/// Inputs that drive mode selection (§20).
struct ModeInputs
{
  int num_humans{0};
  double min_human_clearance{1e9};  ///< metres to the nearest human
  double min_ttc{1e9};              ///< seconds to closest approach with any human
};

/// Thresholds for mode selection. Hysteresis is applied via separate enter/exit bands so
/// the mode does not oscillate (§20, §23).
struct ModeThresholds
{
  int crowded_num_humans{4};
  double cautious_clearance{2.0};
  double crowded_clearance{1.2};
  double emergency_clearance{0.5};
  double emergency_ttc{1.0};
  double hysteresis{0.25};          ///< fractional band widening on de-escalation
};

/// Compute the next mode from the current mode and inputs, applying hysteresis so that
/// escalation is prompt but de-escalation requires clearing the threshold by the band.
BehaviorMode nextMode(
  BehaviorMode current, const ModeInputs & in, const ModeThresholds & th);

/// Speed scale in [0, 1] for a mode (EMERGENCY = 0 => stop). §20.
double modeSpeedScale(BehaviorMode mode);

}  // namespace social_nav_core

#endif  // SOCIAL_NAV_CORE__TRAJECTORY_SCORER_HPP_
