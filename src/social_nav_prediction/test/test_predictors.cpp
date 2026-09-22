#include <gtest/gtest.h>

#include <Eigen/Core>

#include "social_nav_prediction/predictor.hpp"

using namespace social_nav_prediction;
using Eigen::Vector2d;

namespace
{
PredictionParams params(double horizon = 3.0, double dt = 0.1)
{
  PredictionParams p;
  p.horizon = horizon;
  p.dt = dt;
  p.initial_position_stddev = 0.05;
  p.uncertainty_growth_rate = 0.3;
  return p;
}
}  // namespace

TEST(StepCount, DivisibleHorizonKeepsLastStep)
{
  EXPECT_EQ(predictionStepCount(params(3.0, 0.1)), 30);
  EXPECT_EQ(predictionStepCount(params(1.0, 0.25)), 4);
}

TEST(StepCount, InvalidParamsYieldZero)
{
  EXPECT_EQ(predictionStepCount(params(3.0, 0.0)), 0);
  EXPECT_EQ(predictionStepCount(params(3.0, -0.1)), 0);
  EXPECT_EQ(predictionStepCount(params(0.0, 0.1)), 0);
}

TEST(ConstantVelocity, MovesInStraightLine)
{
  HumanMotionState s;
  s.position = Vector2d(1.0, 2.0);
  s.velocity = Vector2d(0.5, 0.0);  // 0.5 m/s along +x

  ConstantVelocityPredictor pred;
  const auto path = pred.predict(s, params(2.0, 0.1));

  ASSERT_EQ(path.states.size(), 20u);
  // First sample at t=0.1.
  EXPECT_NEAR(path.states.front().time, 0.1, 1e-12);
  EXPECT_NEAR(path.states.front().position.x(), 1.0 + 0.5 * 0.1, 1e-9);
  // Last sample at t=2.0 -> x = 1.0 + 0.5*2.0 = 2.0.
  EXPECT_NEAR(path.states.back().position.x(), 2.0, 1e-9);
  EXPECT_NEAR(path.states.back().position.y(), 2.0, 1e-9);
  // Velocity is unchanged.
  EXPECT_NEAR(path.states.back().velocity.x(), 0.5, 1e-12);
}

TEST(ConstantAcceleration, AddsQuadraticTerm)
{
  HumanMotionState s;
  s.position = Vector2d(0.0, 0.0);
  s.velocity = Vector2d(1.0, 0.0);
  s.acceleration = Vector2d(0.0, 2.0);  // accelerating along +y

  ConstantAccelerationPredictor pred;
  const auto path = pred.predict(s, params(1.0, 0.1));

  const auto & last = path.states.back();  // t = 1.0
  EXPECT_NEAR(last.time, 1.0, 1e-9);
  EXPECT_NEAR(last.position.x(), 1.0, 1e-9);             // x = v t
  EXPECT_NEAR(last.position.y(), 0.5 * 2.0 * 1.0, 1e-9); // y = 1/2 a t^2 = 1.0
  EXPECT_NEAR(last.velocity.y(), 2.0, 1e-9);             // v_y = a t
}

// The distinguishing property of Model C: uncertainty must grow with the horizon.
TEST(UncertaintyAware, UncertaintyIncreasesWithTime)
{
  HumanMotionState s;
  s.velocity = Vector2d(0.4, 0.0);

  UncertaintyAwarePredictor pred;
  const auto path = pred.predict(s, params(3.0, 0.1));

  ASSERT_GE(path.states.size(), 2u);
  EXPECT_NEAR(path.states.front().position_stddev, 0.05 + 0.3 * 0.1, 1e-9);

  // Strictly monotonically increasing stddev.
  for (std::size_t i = 1; i < path.states.size(); ++i) {
    EXPECT_GT(path.states[i].position_stddev, path.states[i - 1].position_stddev);
  }
  // At the far horizon it is much larger than near-term.
  EXPECT_GT(path.states.back().position_stddev, path.states.front().position_stddev);
}

// Models A and B keep a constant (measurement-noise) uncertainty; only C grows it.
TEST(UncertaintyAware, ConstantModelsDoNotGrowUncertainty)
{
  HumanMotionState s;
  s.velocity = Vector2d(0.4, 0.0);

  ConstantVelocityPredictor cv;
  const auto path = cv.predict(s, params(3.0, 0.1));
  for (const auto & st : path.states) {
    EXPECT_NEAR(st.position_stddev, 0.05, 1e-12);
  }
}

TEST(Predictor, InvalidParamsProduceEmptyPath)  // fail-safe
{
  HumanMotionState s;
  s.velocity = Vector2d(1.0, 0.0);
  ConstantVelocityPredictor pred;
  const auto path = pred.predict(s, params(3.0, 0.0));
  EXPECT_TRUE(path.states.empty());
}
