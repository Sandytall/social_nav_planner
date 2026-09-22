#include <gtest/gtest.h>

#include <Eigen/Core>

#include "social_nav_core/kinematics.hpp"

using social_nav_core::computeClosestApproach;
using Eigen::Vector2d;

// A head-on approach must be detected as an imminent, near-zero-distance event.
TEST(ClosestApproach, HeadOnCollisionCourse)
{
  // Human 5 m ahead at (5,0); relative velocity closing at 2 m/s along -x.
  const auto ca = computeClosestApproach(Vector2d(5.0, 0.0), Vector2d(-2.0, 0.0));
  EXPECT_TRUE(ca.approaching);
  EXPECT_NEAR(ca.time, 2.5, 1e-6);       // 5 m / 2 m/s
  EXPECT_NEAR(ca.distance, 0.0, 1e-6);   // paths intersect
}

// Same Euclidean distance, but moving AWAY -> not approaching, distance stays.
TEST(ClosestApproach, SeparatingAgentsAreNotApproaching)
{
  const auto ca = computeClosestApproach(Vector2d(5.0, 0.0), Vector2d(2.0, 0.0));
  EXPECT_FALSE(ca.approaching);
  EXPECT_LT(ca.time, 0.0);               // closest approach was in the past
  EXPECT_NEAR(ca.distance, 5.0, 1e-6);   // future min separation == current
}

// A glancing pass: closest approach is the perpendicular offset, in the future.
TEST(ClosestApproach, GlancingPassKeepsLateralOffset)
{
  // Human at (5, 1), closing along -x at 1 m/s: passes 1 m to the side.
  const auto ca = computeClosestApproach(Vector2d(5.0, 1.0), Vector2d(-1.0, 0.0));
  EXPECT_TRUE(ca.approaching);
  EXPECT_NEAR(ca.time, 5.0, 1e-6);
  EXPECT_NEAR(ca.distance, 1.0, 1e-6);
}

// Robustness: zero relative motion must not divide by zero.
TEST(ClosestApproach, ZeroRelativeVelocityReturnsCurrentDistance)
{
  const auto ca = computeClosestApproach(Vector2d(3.0, 4.0), Vector2d(0.0, 0.0));
  EXPECT_FALSE(ca.approaching);
  EXPECT_NEAR(ca.time, 0.0, 1e-9);
  EXPECT_NEAR(ca.distance, 5.0, 1e-9);   // |(3,4)| == 5
}
