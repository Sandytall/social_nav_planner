#include <gtest/gtest.h>

#include <cmath>

#include "social_nav_core/trajectory_generator.hpp"

using namespace social_nav_core;

TEST(RollOut, StraightLineWhenNoRotation)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 2.0, 0.1);
  ASSERT_FALSE(traj.empty());
  const auto & last = traj.points.back();
  EXPECT_NEAR(last.time, 2.0, 1e-9);
  EXPECT_NEAR(last.pose.x, 1.0, 1e-6);   // 0.5 m/s * 2 s
  EXPECT_NEAR(last.pose.y, 0.0, 1e-6);
  EXPECT_NEAR(last.pose.theta, 0.0, 1e-9);
}

TEST(RollOut, RotatesInPlaceWhenNoLinearVel)
{
  const auto traj = rollOut(Pose2D{1, 1, 0}, 0.0, 1.0, 1.0, 0.1);
  const auto & last = traj.points.back();
  EXPECT_NEAR(last.pose.x, 1.0, 1e-9);
  EXPECT_NEAR(last.pose.y, 1.0, 1e-9);
  EXPECT_NEAR(last.pose.theta, 1.0, 1e-6);  // omega * t
}

TEST(RollOut, CurvesLeftWithPositiveOmega)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.5, 2.0, 0.05);
  // Turning left while moving forward => ends with y > 0 and heading > 0.
  EXPECT_GT(traj.points.back().pose.y, 0.0);
  EXPECT_GT(traj.points.back().pose.theta, 0.0);
}

TEST(Generate, ProducesSampleGridWithinLimits)
{
  DiffDriveLimits limits;
  limits.max_v = 0.8;
  limits.min_v = 0.0;
  limits.max_omega = 1.0;
  SamplingParams s;
  s.v_samples = 5;
  s.omega_samples = 7;
  s.sim_time = 1.0;
  s.sim_dt = 0.1;

  const auto trajs = generateTrajectories(Pose2D{0, 0, 0}, Velocity2D{}, limits, s);
  EXPECT_EQ(trajs.size(), 35u);  // 5 * 7
  for (const auto & t : trajs) {
    ASSERT_FALSE(t.empty());
    const double v = t.points.back().velocity.vx;
    const double w = t.points.back().velocity.omega;
    EXPECT_GE(v, limits.min_v - 1e-9);
    EXPECT_LE(v, limits.max_v + 1e-9);
    EXPECT_LE(std::abs(w), limits.max_omega + 1e-9);
  }
}

TEST(Generate, ReachabilityLimitsSampleWindow)  // §18 acceleration limits
{
  DiffDriveLimits limits;
  limits.max_v = 1.0;
  limits.min_v = 0.0;
  limits.max_omega = 2.0;
  limits.max_accel = 0.5;
  limits.max_alpha = 1.0;
  SamplingParams s;
  s.v_samples = 9;
  s.omega_samples = 3;
  s.sim_time = 1.0;
  s.sim_dt = 0.1;

  // Currently stopped; within a 0.2 s period we can only reach v <= 0.1 m/s.
  const auto trajs = generateTrajectories(
    Pose2D{0, 0, 0}, Velocity2D{0.0, 0.0, 0.0}, limits, s, 0.2);
  for (const auto & t : trajs) {
    EXPECT_LE(t.points.back().velocity.vx, 0.1 + 1e-9);
  }
}
