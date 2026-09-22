#include <gtest/gtest.h>

#include "social_nav_core/trajectory_scorer.hpp"

using namespace social_nav_core;

namespace
{
ModeThresholds th()
{
  ModeThresholds t;
  t.crowded_num_humans = 4;
  t.cautious_clearance = 2.0;
  t.crowded_clearance = 1.2;
  t.emergency_clearance = 0.5;
  t.emergency_ttc = 1.0;
  t.hysteresis = 0.25;
  return t;
}
ModeInputs in(double clearance, int num = 0, double ttc = 1e9)
{
  ModeInputs m;
  m.min_human_clearance = clearance;
  m.num_humans = num;
  m.min_ttc = ttc;
  return m;
}
}  // namespace

TEST(Modes, SpeedScaleOrdering)
{
  EXPECT_DOUBLE_EQ(modeSpeedScale(BehaviorMode::kNormal), 1.0);
  EXPECT_GT(modeSpeedScale(BehaviorMode::kCautious), modeSpeedScale(BehaviorMode::kCrowded));
  EXPECT_DOUBLE_EQ(modeSpeedScale(BehaviorMode::kEmergency), 0.0);
}

TEST(Modes, EscalatesPromptly)
{
  // Open space -> NORMAL.
  EXPECT_EQ(nextMode(BehaviorMode::kNormal, in(3.0), th()), BehaviorMode::kNormal);
  // Moderate clearance -> CAUTIOUS.
  EXPECT_EQ(nextMode(BehaviorMode::kNormal, in(1.5), th()), BehaviorMode::kCautious);
  // Tight -> CROWDED.
  EXPECT_EQ(nextMode(BehaviorMode::kNormal, in(1.0), th()), BehaviorMode::kCrowded);
  // Very close -> EMERGENCY.
  EXPECT_EQ(nextMode(BehaviorMode::kNormal, in(0.4), th()), BehaviorMode::kEmergency);
  // Low TTC forces EMERGENCY even with clearance.
  EXPECT_EQ(nextMode(BehaviorMode::kNormal, in(3.0, 0, 0.5), th()), BehaviorMode::kEmergency);
  // Many humans -> at least CROWDED.
  EXPECT_EQ(nextMode(BehaviorMode::kNormal, in(3.0, 5), th()), BehaviorMode::kCrowded);
}

// §23 hysteresis: once CAUTIOUS, clearance just above the threshold does NOT immediately
// drop back to NORMAL; it must exceed threshold * (1 + hysteresis).
TEST(Modes, HysteresisPreventsFlapping)
{
  const auto t = th();
  // cautious_clearance = 2.0, hysteresis 0.25 => must exceed 2.5 to leave CAUTIOUS.
  EXPECT_EQ(nextMode(BehaviorMode::kCautious, in(2.2), t), BehaviorMode::kCautious);  // stays
  EXPECT_EQ(nextMode(BehaviorMode::kCautious, in(2.6), t), BehaviorMode::kNormal);    // clears band
}

TEST(Modes, StringNames)
{
  EXPECT_STREQ(toString(BehaviorMode::kEmergency), "EMERGENCY");
  EXPECT_STREQ(toString(BehaviorMode::kNormal), "NORMAL");
}
