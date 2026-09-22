#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "social_nav_human_model/tracker.hpp"

using namespace social_nav_human_model;
using Eigen::Vector2d;

namespace
{
Detection det(double x, double y) { return Detection{Vector2d(x, y), 1.0}; }
}  // namespace

TEST(Tracker, CreatesTrackForNewDetection)
{
  HumanTracker tr;
  const auto & tracks = tr.update({det(1.0, 0.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NEAR(tracks[0].position.x(), 1.0, 1e-9);
  EXPECT_FALSE(tracks[0].has_velocity);  // no velocity from a single observation
}

TEST(Tracker, KeepsStableIdAndEstimatesVelocity)
{
  HumanTracker tr;
  tr.update({det(0.0, 0.0)}, 0.0);
  const auto & t1 = tr.update({det(0.1, 0.0)}, 0.1);  // 1 m/s along +x
  ASSERT_EQ(t1.size(), 1u);
  const std::uint64_t id = t1[0].id;
  const auto & t2 = tr.update({det(0.2, 0.0)}, 0.2);
  ASSERT_EQ(t2.size(), 1u);
  EXPECT_EQ(t2[0].id, id);  // stable identity via NN association
  EXPECT_TRUE(t2[0].has_velocity);
  EXPECT_NEAR(t2[0].velocity.x(), 1.0, 0.2);  // EMA converges toward 1 m/s
}

// §7 / §21: one missing frame must NOT delete the track.
TEST(Tracker, SingleMissedDetectionDoesNotDeleteTrack)
{
  TrackerParams p;
  p.track_timeout = 1.0;
  HumanTracker tr(p);
  tr.update({det(0.0, 0.0)}, 0.0);
  tr.update({det(0.1, 0.0)}, 0.1);
  const auto & after_miss = tr.update({}, 0.2);  // no detections this frame
  ASSERT_EQ(after_miss.size(), 1u);
  EXPECT_NEAR(after_miss[0].time_since_update, 0.1, 1e-9);
}

TEST(Tracker, RemovesTrackAfterTimeout)
{
  TrackerParams p;
  p.track_timeout = 0.5;
  HumanTracker tr(p);
  tr.update({det(0.0, 0.0)}, 0.0);
  EXPECT_EQ(tr.update({}, 0.4).size(), 1u);  // still within timeout
  EXPECT_EQ(tr.update({}, 0.6).size(), 0u);  // now expired
}

TEST(Tracker, DetectionFarAwayCreatesSecondTrack)
{
  TrackerParams p;
  p.max_association_distance = 1.0;
  HumanTracker tr(p);
  tr.update({det(0.0, 0.0)}, 0.0);
  const auto & tracks = tr.update({det(0.05, 0.0), det(5.0, 5.0)}, 0.1);
  EXPECT_EQ(tracks.size(), 2u);  // near det matches, far det spawns new track
}

TEST(Tracker, ClassifiesStationaryHuman)
{
  TrackerParams p;
  p.stationary_speed_threshold = 0.1;
  HumanTracker tr(p);
  tr.update({det(2.0, 2.0)}, 0.0);
  tr.update({det(2.0, 2.0)}, 0.1);
  const auto & tracks = tr.update({det(2.0, 2.0)}, 0.2);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_TRUE(tracks[0].stationary);
}

TEST(Tracker, TrackingAgeGrows)
{
  HumanTracker tr;
  tr.update({det(0.0, 0.0)}, 1.0);
  tr.update({det(0.1, 0.0)}, 1.5);
  const auto & tracks = tr.update({det(0.2, 0.0)}, 2.0);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NEAR(tracks[0].tracking_age, 1.0, 1e-9);  // created at t=1.0, now t=2.0
}

TEST(Tracker, TwoCrossingHumansKeepSeparateIds)  // §21
{
  TrackerParams p;
  p.max_association_distance = 0.6;
  HumanTracker tr(p);
  // A moving +x, B moving -x, on parallel lines 2 m apart so they never associate wrongly.
  tr.update({det(0.0, 0.0), det(4.0, 2.0)}, 0.0);
  const auto & t1 = tr.update({det(0.5, 0.0), det(3.5, 2.0)}, 0.5);
  ASSERT_EQ(t1.size(), 2u);
  const auto & t2 = tr.update({det(1.0, 0.0), det(3.0, 2.0)}, 1.0);
  ASSERT_EQ(t2.size(), 2u);
}
