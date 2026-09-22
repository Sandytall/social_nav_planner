#include "social_nav_prediction/predictor.hpp"

#include <cmath>

namespace social_nav_prediction
{

int predictionStepCount(const PredictionParams & params)
{
  if (params.dt <= 0.0 || params.horizon <= 0.0) {
    return 0;  // §54: invalid config -> no prediction, caller falls back safely.
  }
  // Number of whole dt steps that fit in the horizon. The small epsilon keeps a horizon
  // that is an exact multiple of dt (e.g. 3.0 / 0.1) from losing its last step to
  // floating-point error.
  return static_cast<int>(std::floor(params.horizon / params.dt + 1e-9));
}

namespace
{
// Shared scaffold: build a path whose per-step position/velocity/stddev are supplied by
// the caller as functions of time. Keeps the three models free of loop boilerplate.
template<typename PosFn, typename VelFn, typename StdFn>
PredictedPath buildPath(
  const PredictionParams & params, PosFn pos, VelFn vel, StdFn stddev)
{
  PredictedPath path;
  path.dt = params.dt;
  path.horizon = params.horizon;

  const int steps = predictionStepCount(params);
  path.states.reserve(static_cast<std::size_t>(steps));
  for (int i = 1; i <= steps; ++i) {
    const double t = i * params.dt;
    PredictedState s;
    s.time = t;
    s.position = pos(t);
    s.velocity = vel(t);
    s.position_stddev = stddev(t);
    path.states.push_back(s);
  }
  return path;
}
}  // namespace

PredictedPath ConstantVelocityPredictor::predict(
  const HumanMotionState & state, const PredictionParams & params) const
{
  const Eigen::Vector2d p0 = state.position;
  const Eigen::Vector2d v = state.velocity;
  const double sigma0 = params.initial_position_stddev;
  return buildPath(
    params,
    [&](double t) { return Eigen::Vector2d(p0 + v * t); },
    [&](double) { return v; },
    [&](double) { return sigma0; });
}

PredictedPath ConstantAccelerationPredictor::predict(
  const HumanMotionState & state, const PredictionParams & params) const
{
  const Eigen::Vector2d p0 = state.position;
  const Eigen::Vector2d v = state.velocity;
  const Eigen::Vector2d a = state.acceleration;
  const double sigma0 = params.initial_position_stddev;
  return buildPath(
    params,
    [&](double t) { return Eigen::Vector2d(p0 + v * t + 0.5 * a * t * t); },
    [&](double t) { return Eigen::Vector2d(v + a * t); },
    [&](double) { return sigma0; });
}

PredictedPath UncertaintyAwarePredictor::predict(
  const HumanMotionState & state, const PredictionParams & params) const
{
  const Eigen::Vector2d p0 = state.position;
  const Eigen::Vector2d v = state.velocity;
  const double sigma0 = params.initial_position_stddev;
  const double rate = params.uncertainty_growth_rate;
  // CV mean, but 1-sigma uncertainty grows linearly with the horizon (§8): the planner
  // treats farther-future predictions as less trustworthy and behaves more cautiously.
  return buildPath(
    params,
    [&](double t) { return Eigen::Vector2d(p0 + v * t); },
    [&](double) { return v; },
    [&](double t) { return sigma0 + rate * t; });
}

}  // namespace social_nav_prediction
