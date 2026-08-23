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

# How far ground truth strayed from the route that was planned for it. Arriving
# is not the same as going the right way, and without this a controller that
# wanders across the aisle and back scores the same as one that holds the line.
#
# Measured against the first plan published for each goal, not the latest one:
# Nav2 replans from wherever the robot currently is, so the live plan is drawn
# through the wandering and always reads near zero.
float64 cross_track_error       # m from the plan, at the last sample
float64 worst_cross_track_error # m, running maximum since the reference
float64 mean_cross_track_error  # m, averaged over every sample since then

# What it took to get there. `recovery_count` counts goals sent to the
# behaviour server - backing up, driving on heading, waiting - which is the
# navigator admitting it was stuck. Costmap clearing is a service call and
# leaves no trace on any topic, so it is not counted here.
int32 recovery_count            # distinct recovery goals since the reference
float64 recovery_time           # s spent with a recovery behaviour running

float64 elapsed_time            # s of simulated time since the reference
