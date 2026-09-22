# A time-stamped set of tracked humans (MASTER_PROMPT §5).
# The header carries the frame_id and stamp for every HumanState in `humans`.
# Never assume this stamp equals the planner clock (§5, §54: stale-data handling).

std_msgs/Header header
social_nav_msgs/HumanState[] humans
