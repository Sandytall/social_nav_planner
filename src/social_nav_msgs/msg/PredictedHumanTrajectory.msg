# A predicted future trajectory for one human.
# Poses/velocities are sampled at fixed `timestep` intervals out to `prediction_horizon`.
# `predicted_poses`, `predicted_velocities` and `prediction_confidence` MUST have equal
# length; consumers must validate this.

uint64 human_id                                 # Matches HumanState.id.

geometry_msgs/Pose[] predicted_poses            # Predicted poses, index 0 = first future step.
geometry_msgs/Twist[] predicted_velocities      # Predicted velocity at each step.

# Per-step confidence in [0, 1]. For uncertainty-aware prediction (Model C) this
# typically decays with the horizon.
float32[] prediction_confidence

float32 prediction_horizon                      # Seconds covered by the prediction.
float32 timestep                                # Seconds between consecutive samples.
