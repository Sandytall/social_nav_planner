#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "social_nav_core/trajectory_generator.hpp"
#include "social_nav_core/trajectory_scorer.hpp"

using namespace social_nav_core;
using Eigen::Vector2d;

namespace
{
// Two candidates from the origin: one straight toward +x, one straight toward -x.
Trajectory towardPlusX() { return rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 2.0, 0.1); }
Trajectory towardMinusX() { return rollOut(Pose2D{0, 0, M_PI}, 0.5, 0.0, 2.0, 0.1); }
}  // namespace

TEST(Scorer, PrefersTrajectoryTowardGoal)
{
  ScoringContext ctx;
  ctx.goal = Vector2d(2.0, 0.0);  // goal ahead in +x

  CostWeights w;
  const double toward = scoreTrajectory(towardPlusX(), ctx, w).total;
  const double away = scoreTrajectory(towardMinusX(), ctx, w).total;
  EXPECT_LT(toward, away);
}

TEST(Scorer, SelectBestPicksLowestTotalAndFillsBreakdowns)
{
  ScoringContext ctx;
  ctx.goal = Vector2d(2.0, 0.0);
  CostWeights w;
  std::vector<Trajectory> trajs{towardMinusX(), towardPlusX()};
  std::vector<CostBreakdown> br;
  const int best = selectBestTrajectory(trajs, ctx, w, &br);
  EXPECT_EQ(best, 1);              // the toward-goal trajectory
  ASSERT_EQ(br.size(), 2u);
  EXPECT_GT(br[0].total, br[1].total);
}

// A trajectory that drives into a human's social zone costs more (human term)
// than one that keeps clearance.
TEST(Scorer, HumanZoneRaisesHumanCost)
{
  ScoringContext near_ctx;
  near_ctx.goal = Vector2d(2.0, 0.0);
  ScoredHuman h;
  h.position = Vector2d(1.0, 0.0);  // right on the +x path
  h.heading = M_PI;                 // facing the robot
  near_ctx.humans = {h};

  ScoringContext far_ctx = near_ctx;
  far_ctx.humans[0].position = Vector2d(1.0, 5.0);  // far to the side

  CostWeights w;
  const double c_near = scoreTrajectory(towardPlusX(), near_ctx, w).human;
  const double c_far = scoreTrajectory(towardPlusX(), far_ctx, w).human;
  EXPECT_GT(c_near, c_far);
}

// A human on a collision course adds TTC cost; the same human moving away does not.
TEST(Scorer, ApproachingHumanRaisesTtcCost)
{
  ScoringContext ctx;
  ctx.goal = Vector2d(3.0, 0.0);
  ctx.ttc_horizon = 5.0;
  ctx.ttc_danger_distance = 1.0;

  ScoredHuman approaching;
  approaching.position = Vector2d(3.0, 0.0);
  approaching.velocity = Vector2d(-1.0, 0.0);  // closing head-on
  ctx.humans = {approaching};
  const double ttc_closing = scoreTrajectory(towardPlusX(), ctx, CostWeights{}).ttc;

  ctx.humans[0].velocity = Vector2d(1.0, 0.0);  // moving away
  const double ttc_away = scoreTrajectory(towardPlusX(), ctx, CostWeights{}).ttc;

  EXPECT_GT(ttc_closing, 0.0);
  EXPECT_NEAR(ttc_away, 0.0, 1e-9);
}

// Driving through a group is penalised.
TEST(Scorer, GroupIntrusionPenalised)
{
  ScoringContext ctx;
  ctx.goal = Vector2d(2.0, 0.0);
  ScoredGroup g;
  g.centroid = Vector2d(1.0, 0.0);  // on the path
  g.radius = 0.8;
  ctx.groups = {g};
  const double intruding = scoreTrajectory(towardPlusX(), ctx, CostWeights{}).group;
  EXPECT_GT(intruding, 0.0);

  ctx.groups[0].centroid = Vector2d(1.0, 5.0);  // off to the side
  const double clear = scoreTrajectory(towardPlusX(), ctx, CostWeights{}).group;
  EXPECT_NEAR(clear, 0.0, 1e-9);
}

TEST(Scorer, EmptyTrajectoryIsInfinite)
{
  EXPECT_TRUE(std::isinf(scoreTrajectory(Trajectory{}, ScoringContext{}, CostWeights{}).total));
  EXPECT_EQ(selectBestTrajectory({}, ScoringContext{}, CostWeights{}), -1);
}

// Reproduces the sim regression: with a short pruned plan straight ahead and the sim's
// weights, the planner must pick a FORWARD trajectory, not one that stalls or reverses.
TEST(Scorer, FollowsShortPlanForward)
{
  DiffDriveLimits limits;
  limits.max_v = 0.8; limits.min_v = 0.0; limits.max_omega = 1.2;
  SamplingParams s; s.v_samples = 8; s.omega_samples = 21; s.sim_time = 2.0; s.sim_dt = 0.1;
  const auto trajs = generateTrajectories(Pose2D{0, 0, 0}, Velocity2D{}, limits, s, 0.0);

  ScoringContext ctx;
  ctx.goal = Vector2d(1.5, 0.0);
  ctx.path = {{0.2, 0}, {0.5, 0}, {0.8, 0}, {1.1, 0}, {1.5, 0}};
  ctx.max_linear_velocity = 0.8;

  CostWeights w;  // sim weights
  w.goal = 1.0; w.path = 3.0; w.obstacle = 12.0; w.human = 8.0; w.ttc = 25.0;
  w.group = 6.0; w.direction = 0.5; w.smoothness = 0.5; w.velocity = 1.0; w.progress = 3.0;

  const int best = selectBestTrajectory(trajs, ctx, w);
  ASSERT_GE(best, 0);
  const auto & end = trajs[static_cast<size_t>(best)].points.back().pose;
  EXPECT_GT(end.x, 0.5) << "planner did not choose a forward trajectory (x=" << end.x << ")";
  EXPECT_NEAR(end.y, 0.0, 0.4);
}
