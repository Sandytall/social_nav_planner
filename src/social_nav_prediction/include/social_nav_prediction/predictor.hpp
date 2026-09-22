// Human motion prediction (MASTER_PROMPT §8).
//
// Three models, from simplest to most conservative:
//   A. Constant velocity                  p(t) = p0 + v t
//   B. Constant velocity + acceleration   p(t) = p0 + v t + 1/2 a t^2
//   C. Uncertainty-aware                  CV mean, but positional uncertainty grows
//                                         with the horizon so the planner becomes more
//                                         conservative the further ahead it looks.
//
// The library is ROS-free so the models are unit-testable in isolation (§35). A thin
// bridge converts a PredictedPath into social_nav_msgs/PredictedHumanTrajectory
// elsewhere (§5).
#ifndef SOCIAL_NAV_PREDICTION__PREDICTOR_HPP_
#define SOCIAL_NAV_PREDICTION__PREDICTOR_HPP_

#include <memory>
#include <vector>

#include <Eigen/Core>

namespace social_nav_prediction
{

/// Current motion estimate for one human (produced by the tracker, §7).
struct HumanMotionState
{
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  Eigen::Vector2d acceleration{0.0, 0.0};
};

/// Knobs shared by all predictors (§8, §15/§26 parameters).
struct PredictionParams
{
  double horizon{3.0};                  ///< seconds to predict ahead (> 0)
  double dt{0.1};                       ///< sample spacing (> 0)
  double initial_position_stddev{0.05}; ///< sigma at t=0 (measurement noise), metres
  double uncertainty_growth_rate{0.3};  ///< extra sigma per second (Model C), metres/s
};

/// One predicted sample. `position_stddev` is the 1-sigma isotropic positional
/// uncertainty (§8, Model C); models A/B report the constant initial value.
struct PredictedState
{
  double time{0.0};
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double position_stddev{0.0};
};

/// A full predicted path, sampled at t = dt, 2 dt, ... up to the horizon.
struct PredictedPath
{
  std::vector<PredictedState> states;
  double horizon{0.0};
  double dt{0.0};
};

/// Abstract predictor so the planner can swap models via configuration (§8, §49 keep
/// it modular).
class MotionPredictor
{
public:
  virtual ~MotionPredictor() = default;
  virtual PredictedPath predict(
    const HumanMotionState & state, const PredictionParams & params) const = 0;
  virtual const char * name() const = 0;
};

/// Model A: constant velocity.
class ConstantVelocityPredictor : public MotionPredictor
{
public:
  PredictedPath predict(
    const HumanMotionState & state, const PredictionParams & params) const override;
  const char * name() const override { return "constant_velocity"; }
};

/// Model B: constant velocity + acceleration.
class ConstantAccelerationPredictor : public MotionPredictor
{
public:
  PredictedPath predict(
    const HumanMotionState & state, const PredictionParams & params) const override;
  const char * name() const override { return "constant_acceleration"; }
};

/// Model C: uncertainty-aware. CV mean, uncertainty grows linearly with time.
class UncertaintyAwarePredictor : public MotionPredictor
{
public:
  PredictedPath predict(
    const HumanMotionState & state, const PredictionParams & params) const override;
  const char * name() const override { return "uncertainty_aware"; }
};

/// Number of future samples for the given params (t = dt .. N*dt <= horizon).
/// Returns 0 if params are invalid (non-positive dt/horizon) so callers fail safe (§54).
int predictionStepCount(const PredictionParams & params);

}  // namespace social_nav_prediction

#endif  // SOCIAL_NAV_PREDICTION__PREDICTOR_HPP_
