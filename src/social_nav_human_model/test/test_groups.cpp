#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "social_nav_human_model/groups.hpp"

using namespace social_nav_human_model;
using Eigen::Vector2d;

namespace
{
HumanForGrouping h(std::uint64_t id, double x, double y, double vx, double vy)
{
  return HumanForGrouping{id, Vector2d(x, y), Vector2d(vx, vy)};
}
GroupingParams params()
{
  GroupingParams p;
  p.group_radius = 1.5;
  p.velocity_similarity = 0.5;
  return p;
}
}  // namespace

TEST(Groups, TwoCloseSimilarHumansFormOneGroup)
{
  const std::vector<HumanForGrouping> humans{
    h(1, 0.0, 0.0, 1.0, 0.0), h(2, 0.8, 0.0, 1.0, 0.0)};
  const auto groups = detectGroups(humans, params());
  ASSERT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups[0].member_ids.size(), 2u);
  EXPECT_NEAR(groups[0].centroid.x(), 0.4, 1e-9);
  EXPECT_NEAR(groups[0].velocity.x(), 1.0, 1e-9);
}

TEST(Groups, FarApartHumansAreNotAGroup)
{
  const std::vector<HumanForGrouping> humans{
    h(1, 0.0, 0.0, 1.0, 0.0), h(2, 5.0, 0.0, 1.0, 0.0)};
  EXPECT_TRUE(detectGroups(humans, params()).empty());
}

TEST(Groups, CloseButOppositeVelocitiesAreNotAGroup)
{
  const std::vector<HumanForGrouping> humans{
    h(1, 0.0, 0.0, 1.0, 0.0), h(2, 0.8, 0.0, -1.0, 0.0)};
  EXPECT_TRUE(detectGroups(humans, params()).empty());
}

TEST(Groups, TransitiveChainFormsSingleGroup)
{
  // A-B and B-C are each within radius; A-C is not, but transitivity links all three.
  const std::vector<HumanForGrouping> humans{
    h(1, 0.0, 0.0, 0.5, 0.0), h(2, 1.2, 0.0, 0.5, 0.0), h(3, 2.4, 0.0, 0.5, 0.0)};
  const auto groups = detectGroups(humans, params());
  ASSERT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups[0].member_ids.size(), 3u);
}

TEST(Groups, TwoSeparateGroups)
{
  const std::vector<HumanForGrouping> humans{
    h(1, 0.0, 0.0, 1.0, 0.0), h(2, 0.7, 0.0, 1.0, 0.0),
    h(3, 10.0, 0.0, -1.0, 0.0), h(4, 10.7, 0.0, -1.0, 0.0)};
  EXPECT_EQ(detectGroups(humans, params()).size(), 2u);
}

TEST(Groups, EmptyAndSingleInputsYieldNoGroups)
{
  EXPECT_TRUE(detectGroups({}, params()).empty());
  EXPECT_TRUE(detectGroups({h(1, 0.0, 0.0, 0.0, 0.0)}, params()).empty());
}
