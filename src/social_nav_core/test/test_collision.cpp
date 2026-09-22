#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "social_nav_core/collision.hpp"
#include "social_nav_core/trajectory_generator.hpp"

using namespace social_nav_core;
using Eigen::Vector2d;

TEST(Collision, StraightPathHitsObstacleOnItsLine)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 2.0, 0.1);  // to x=1
  const std::vector<Vector2d> obs{{0.5, 0.0}};                     // right on the path
  EXPECT_TRUE(inCollision(traj, obs, 0.3));
}

TEST(Collision, ClearPathDoesNotCollide)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 2.0, 0.1);
  const std::vector<Vector2d> obs{{0.5, 2.0}};  // 2 m off to the side
  EXPECT_FALSE(inCollision(traj, obs, 0.3));
}

TEST(Collision, RadiusMatters)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 2.0, 0.1);
  const std::vector<Vector2d> obs{{0.5, 0.35}};  // 0.35 m off the path line
  EXPECT_FALSE(inCollision(traj, obs, 0.3));  // small footprint clears
  EXPECT_TRUE(inCollision(traj, obs, 0.4));   // larger footprint hits
}

TEST(Collision, ClearanceIsMinDistanceOverPath)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 2.0, 0.1);
  const std::vector<Vector2d> obs{{1.0, 0.5}};  // nearest approach 0.5 m laterally
  EXPECT_NEAR(minObstacleClearance(traj, obs), 0.5, 1e-6);
}

TEST(Collision, NoObstaclesMeansInfiniteClearanceNoCollision)
{
  const auto traj = rollOut(Pose2D{0, 0, 0}, 0.5, 0.0, 1.0, 0.1);
  EXPECT_FALSE(inCollision(traj, {}, 0.3));
  EXPECT_TRUE(std::isinf(minObstacleClearance(traj, {})));
}
