#include <gtest/gtest.h>

#include <Eigen/Core>

#include "social_nav_human_model/social_zones.hpp"

using namespace social_nav_human_model;
using Eigen::Vector2d;

namespace
{
SocialZoneParams params()
{
  SocialZoneParams p;
  p.personal = ZoneRadii{0.5, 0.4, 0.3};
  p.comfort = ZoneRadii{1.2, 0.9, 0.6};
  p.caution = ZoneRadii{2.0, 1.5, 1.0};
  return p;
}
}  // namespace

TEST(SocialZones, CentreIsPersonal)
{
  EXPECT_EQ(classifyZone(Vector2d(0, 0), 0.0, Vector2d(0, 0), params()), SocialZone::kPersonal);
}

TEST(SocialZones, NestedBandsInFront)
{
  const Vector2d h(0, 0);
  const double heading = 0.0;  // facing +x
  EXPECT_EQ(classifyZone(h, heading, Vector2d(0.4, 0.0), params()), SocialZone::kPersonal);
  EXPECT_EQ(classifyZone(h, heading, Vector2d(1.0, 0.0), params()), SocialZone::kComfort);
  EXPECT_EQ(classifyZone(h, heading, Vector2d(1.8, 0.0), params()), SocialZone::kCaution);
  EXPECT_EQ(classifyZone(h, heading, Vector2d(3.0, 0.0), params()), SocialZone::kNone);
}

// Anisotropy: the same distance is "more intrusive" in front than behind, because the
// front radii are larger. 1.4 m ahead is still inside the caution ellipse; 1.4 m behind
// is outside every zone.
TEST(SocialZones, FrontReachesFartherThanRear)
{
  const Vector2d h(0, 0);
  const double heading = 0.0;
  EXPECT_NE(classifyZone(h, heading, Vector2d(1.4, 0.0), params()), SocialZone::kNone);
  EXPECT_EQ(classifyZone(h, heading, Vector2d(-1.4, 0.0), params()), SocialZone::kNone);
}

TEST(SocialZones, RotatesWithHeading)
{
  const Vector2d h(0, 0);
  // 1.8 m along +x. Front caution reach is 2.0 m, side caution reach is only 1.5 m,
  // so this point is inside the caution zone when the human faces it, but outside every
  // zone when the human faces perpendicular (it is then 1.8 m off to the side > 1.5 m).
  const Vector2d query(1.8, 0.0);
  EXPECT_NE(classifyZone(h, 0.0, query, params()), SocialZone::kNone);          // ahead
  EXPECT_EQ(classifyZone(h, M_PI / 2.0, query, params()), SocialZone::kNone);   // to the side
}

TEST(SocialZones, NormalizedDistanceUnitAtBoundary)
{
  // A point exactly at the front personal radius has normalized distance 1.0.
  const double nd = normalizedZoneDistance(Vector2d(0, 0), 0.0, Vector2d(0.5, 0.0),
    ZoneRadii{0.5, 0.4, 0.3});
  EXPECT_NEAR(nd, 1.0, 1e-9);
}
