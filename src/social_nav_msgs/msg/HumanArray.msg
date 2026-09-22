# A time-stamped set of tracked humans.
# The header carries the frame_id and stamp for every HumanState in `humans`.
# Never assume this stamp equals the planner clock (stale-data handling).

std_msgs/Header header
social_nav_msgs/HumanState[] humans
