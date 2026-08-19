# How far the filtered estimate has diverged from ground truth.
#
# Errors are measured in the frame of the pose held at the moment the reference
# was marked, so the constant offset between the map frame and the simulator's
# world frame does not count against the estimate.

std_msgs/Header header

float64 position_error        # m
float64 heading_error         # rad, wrapped to [-pi, pi]
float64 worst_position_error  # m, running maximum since the reference
float64 worst_heading_error   # rad, running maximum of the absolute error

# Ground truth path totals since the reference. A robot that never moved
# trivially has no error, so these are what stop a stuck run from passing.
float64 path_length           # m travelled along the path
float64 path_rotation         # rad of yaw accumulated, unsigned
