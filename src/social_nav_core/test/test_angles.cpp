#include <gtest/gtest.h>

#include <cmath>

#include "social_nav_core/angles.hpp"

using social_nav_core::normalizeAngle;
using social_nav_core::shortestAngularDistance;

TEST(Angles, WrapsWithinRange)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-9);
  EXPECT_NEAR(normalizeAngle(M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(-M_PI + 0.1), -M_PI + 0.1, 1e-9);
}

TEST(Angles, WrapsLargeMagnitudes)
{
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(-3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizeAngle(2.0 * M_PI + 0.25), 0.25, 1e-9);
}

TEST(Angles, ShortestDistanceTakesShortWayAround)
{
  // From +170deg to -170deg is +20deg the short way, not -340deg.
  const double from = 170.0 * M_PI / 180.0;
  const double to = -170.0 * M_PI / 180.0;
  EXPECT_NEAR(shortestAngularDistance(from, to), 20.0 * M_PI / 180.0, 1e-9);
}
