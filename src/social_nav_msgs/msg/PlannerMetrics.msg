# Real-time performance report from the SocialNav controller.
# Published on /social_nav/debug/metrics. Latencies are per computeVelocityCommands call
# over a rolling window; these are measured, never fabricated.

std_msgs/Header header

float64 target_frequency_hz   # configured controller_frequency
float64 actual_frequency_hz   # measured 1 / mean(call period)
float64 p50_ms                # median compute time
float64 p95_ms
float64 p99_ms
float64 max_ms
float64 deadline_ms           # deadline = 1000 / target_frequency
int32   deadline_misses       # calls exceeding the deadline in the window
int32   window_samples

string mode                   # current behavior mode (NORMAL/CAUTIOUS/CROWDED/EMERGENCY)
int32  num_humans             # people considered this cycle
