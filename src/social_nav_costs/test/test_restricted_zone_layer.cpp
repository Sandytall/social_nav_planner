// Logic tests for RestrictedZoneLayer::parseZones - the pure config parsing that turns the
// flat [xmin,ymin,xmax,ymax, ...] parameter into normalized zones. No running node needed.
#include <gtest/gtest.h>

#include <vector>

#include "rclcpp/logging.hpp"
#include "social_nav_costs/restricted_zone_layer.hpp"

using social_nav_costs::RestrictedZoneLayer;

namespace
{
rclcpp::Logger testLogger()
{
  return rclcpp::get_logger("restricted_zone_test");
}
}  // namespace

TEST(RestrictedZoneParse, EmptyListYieldsNoZones)
{
  // The documented default: an empty list makes the layer a safe no-op.
  const auto zones = RestrictedZoneLayer::parseZones({}, testLogger());
  EXPECT_TRUE(zones.empty());
}

TEST(RestrictedZoneParse, SingleRectParsedAsGiven)
{
  const auto zones = RestrictedZoneLayer::parseZones({-3.7, 3.4, -1.5, 6.2}, testLogger());
  ASSERT_EQ(zones.size(), 1u);
  EXPECT_DOUBLE_EQ(zones[0].xmin, -3.7);
  EXPECT_DOUBLE_EQ(zones[0].ymin, 3.4);
  EXPECT_DOUBLE_EQ(zones[0].xmax, -1.5);
  EXPECT_DOUBLE_EQ(zones[0].ymax, 6.2);
}

TEST(RestrictedZoneParse, MultipleRectsAllParsed)
{
  const auto zones = RestrictedZoneLayer::parseZones(
    {-3.7, 3.4, -1.5, 6.2, 9.5, 1.6, 10.7, 3.2}, testLogger());
  ASSERT_EQ(zones.size(), 2u);
  EXPECT_DOUBLE_EQ(zones[1].xmin, 9.5);
  EXPECT_DOUBLE_EQ(zones[1].ymax, 3.2);
}

TEST(RestrictedZoneParse, CornerOrderIsNormalized)
{
  // Given as [xmax,ymax,xmin,ymin]: the parser must sort min/max so the rectangle is valid.
  const auto zones = RestrictedZoneLayer::parseZones({-1.5, 6.2, -3.7, 3.4}, testLogger());
  ASSERT_EQ(zones.size(), 1u);
  EXPECT_LE(zones[0].xmin, zones[0].xmax);
  EXPECT_LE(zones[0].ymin, zones[0].ymax);
  EXPECT_DOUBLE_EQ(zones[0].xmin, -3.7);
  EXPECT_DOUBLE_EQ(zones[0].xmax, -1.5);
  EXPECT_DOUBLE_EQ(zones[0].ymin, 3.4);
  EXPECT_DOUBLE_EQ(zones[0].ymax, 6.2);
}

TEST(RestrictedZoneParse, MalformedTailIsDropped)
{
  // 6 values = one full rect plus a two-value tail; the tail is ignored, not treated as a rect.
  const auto zones = RestrictedZoneLayer::parseZones({0.0, 0.0, 1.0, 1.0, 5.0, 5.0}, testLogger());
  ASSERT_EQ(zones.size(), 1u);
  EXPECT_DOUBLE_EQ(zones[0].xmax, 1.0);
}
