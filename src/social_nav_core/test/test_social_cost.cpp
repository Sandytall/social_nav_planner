#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>

#include "social_nav_core/social_cost.hpp"

using social_nav_core::anisotropicSocialCost;
using social_nav_core::SocialZoneParams;
using Eigen::Vector2d;

namespace
{
constexpr double kFront = 1.5;
constexpr double kSide = 1.0;
constexpr double kRear = 0.7;
SocialZoneParams params() { return SocialZoneParams{kFront, kSide, kRear}; }
}  // namespace

TEST(SocialCost, PeaksAtHumanCentre)
{
  const double c = anisotropicSocialCost(Vector2d(2.0, 3.0), 0.4, Vector2d(2.0, 3.0), params());
  EXPECT_NEAR(c, 1.0, 1e-9);
}

TEST(SocialCost, DecaysWithDistance)
{
  const Vector2d h(0.0, 0.0);
  const double near = anisotropicSocialCost(h, 0.0, Vector2d(0.5, 0.0), params());
  const double far = anisotropicSocialCost(h, 0.0, Vector2d(2.0, 0.0), params());
  EXPECT_LT(far, near);
  EXPECT_GT(near, 0.0);
}

// Comfortable clearance ordering front > side > rear. At equal distance the cost
// should therefore be highest in front and lowest behind.
TEST(SocialCost, AnisotropyFrontGreaterThanSideGreaterThanRear)
{
  const Vector2d h(0.0, 0.0);
  const double heading = 0.0;  // facing +x
  const double d = 1.0;

  const double front = anisotropicSocialCost(h, heading, Vector2d(d, 0.0), params());
  const double side = anisotropicSocialCost(h, heading, Vector2d(0.0, d), params());
  const double rear = anisotropicSocialCost(h, heading, Vector2d(-d, 0.0), params());

  EXPECT_GT(front, side);
  EXPECT_GT(side, rear);

  // Exact expected values from C = exp(-d^2 / (2 sigma^2)).
  EXPECT_NEAR(front, std::exp(-1.0 / (2.0 * kFront * kFront)), 1e-9);
  EXPECT_NEAR(side, std::exp(-1.0 / (2.0 * kSide * kSide)), 1e-9);
  EXPECT_NEAR(rear, std::exp(-1.0 / (2.0 * kRear * kRear)), 1e-9);
}

// The zone must rotate with the human's heading: a point that is "in front" for a
// human facing +x is "to the side" when the human faces +y.
TEST(SocialCost, RotatesWithHeading)
{
  const Vector2d h(0.0, 0.0);
  const Vector2d query(1.0, 0.0);

  const double facing_x = anisotropicSocialCost(h, 0.0, query, params());        // in front
  const double facing_y = anisotropicSocialCost(h, M_PI / 2.0, query, params()); // to the (right) side

  // Facing +x the point is straight ahead (front_sigma); facing +y it is off to the
  // side (side_sigma < front_sigma) so the cost is lower.
  EXPECT_GT(facing_x, facing_y);
  EXPECT_NEAR(facing_y, std::exp(-1.0 / (2.0 * kSide * kSide)), 1e-9);
}
