#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "social_nav_controller/path_geometry.hpp"
#include "social_nav_core/types.hpp"

using namespace social_nav_controller;
using social_nav_core::Pose2D;
using Eigen::Vector2d;

TEST(PathGeometry, WorldToRobotFrameAtOriginNoRotation)
{
  const Pose2D robot{0.0, 0.0, 0.0};
  const auto p = toRobotFrame(robot, Vector2d(2.0, 1.0));
  EXPECT_NEAR(p.x(), 2.0, 1e-9);
  EXPECT_NEAR(p.y(), 1.0, 1e-9);
}

TEST(PathGeometry, WorldToRobotFrameRotated90)
{
  // Robot at (1,1) facing +y. A world point straight ahead of it at (1,3) should be
  // at (2,0) in the robot frame (2 m forward, 0 lateral).
  const Pose2D robot{1.0, 1.0, M_PI / 2.0};
  const auto p = toRobotFrame(robot, Vector2d(1.0, 3.0));
  EXPECT_NEAR(p.x(), 2.0, 1e-9);
  EXPECT_NEAR(p.y(), 0.0, 1e-9);
}

TEST(PathGeometry, LookaheadPicksFirstPointBeyondDistance)
{
  const std::vector<Vector2d> path{{0.1, 0.0}, {0.4, 0.0}, {0.9, 0.0}, {1.5, 0.0}};
  const auto c = findLookaheadPoint(path, 0.8);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.point.x(), 0.9, 1e-9);  // first point with norm >= 0.8
}

TEST(PathGeometry, LookaheadFallsBackToLastPoint)
{
  const std::vector<Vector2d> path{{0.1, 0.0}, {0.3, 0.0}};
  const auto c = findLookaheadPoint(path, 1.0);  // nothing that far
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.point.x(), 0.3, 1e-9);
}

TEST(PathGeometry, LookaheadEmptyPathIsInvalid)
{
  EXPECT_FALSE(findLookaheadPoint({}, 1.0).valid);
}

TEST(PathGeometry, CurvatureZeroForStraightAhead)
{
  EXPECT_NEAR(purePursuitCurvature(Vector2d(1.0, 0.0)), 0.0, 1e-9);
}

TEST(PathGeometry, CurvatureSignFollowsLateralOffset)
{
  // Carrot to the left (+y) -> positive curvature (turn left).
  EXPECT_GT(purePursuitCurvature(Vector2d(1.0, 0.5)), 0.0);
  // Carrot to the right (-y) -> negative curvature.
  EXPECT_LT(purePursuitCurvature(Vector2d(1.0, -0.5)), 0.0);
  // kappa = 2y / (x^2+y^2): carrot (1,1) -> 2*1/2 = 1.0.
  EXPECT_NEAR(purePursuitCurvature(Vector2d(1.0, 1.0)), 1.0, 1e-9);
}

TEST(PathGeometry, SpeedRegulationSlowsOnTightCurvature)
{
  const double desired = 0.8;
  // Gentle curvature below threshold -> unchanged.
  EXPECT_NEAR(regulateLinearSpeed(desired, 0.1, 0.5, 0.1), desired, 1e-9);
  // Sharp curvature -> reduced but not below min.
  const double slow = regulateLinearSpeed(desired, 2.0, 0.5, 0.1);
  EXPECT_LT(slow, desired);
  EXPECT_GE(slow, 0.1);
}

TEST(PathGeometry, GoalApproachDeceleratesToZero)
{
  const double desired = 0.5;
  const double approach = 0.6;
  EXPECT_NEAR(goalApproachSpeed(2.0, desired, approach), desired, 1e-9);  // far: full speed
  EXPECT_NEAR(goalApproachSpeed(0.3, desired, approach), desired * 0.5, 1e-9);  // halfway
  EXPECT_NEAR(goalApproachSpeed(0.0, desired, approach), 0.0, 1e-9);      // at goal: stop
}
