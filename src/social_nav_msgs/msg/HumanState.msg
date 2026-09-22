# A single tracked human as consumed by the SocialNav planner (MASTER_PROMPT §5).
# This is a generic, detector-agnostic representation: the planner never depends on
# how the human was detected, only on this state.

uint64 id                       # Stable track id (0 is a valid id).

geometry_msgs/Pose pose         # Human pose in the frame given by the enclosing HumanArray.header.
geometry_msgs/Twist velocity    # Estimated linear/angular velocity in the same frame.

# Row-major pose covariance. Length is either 0 (unknown) or 36 for a 6x6
# [x, y, z, roll, pitch, yaw] block. Consumers MUST tolerate an empty array (§54).
float32[] covariance

float32 confidence              # Detection/track confidence in [0, 1].
float32 tracking_age            # Seconds this track has existed (see §7 timeout logic).

bool stationary                 # True if the tracker classifies this human as not moving.
bool group_member               # True if this human is assigned to a group (§13).
int32 group_id                  # Group id when group_member is true; -1 otherwise.
