# Where the aisle is, measured from the trunks standing either side of it.
#
# Quantities are in the sensor's frame. The Kraken's lidar is bolted on without
# rotation, so they read directly as the robot's error against the row: x
# forward, y left.

std_msgs/Header header

bool valid          # false when one side was not seen well enough to fit
string reason       # why not, when invalid

float64 width           # m, perpendicular distance between the two trunk lines
float64 lateral_offset  # m, where the centreline sits, positive to the left
float64 row_heading     # rad, bearing of the row ahead, positive running left

int32 left_trunks   # trunks that survived the fit on each side
int32 right_trunks
