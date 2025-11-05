# Standard deviations of pose uncertainty extracted from EKF covariance
# Used for adaptive NDT scan matcher parameter control

std_msgs/Header header

# Standard deviation in x direction (meters)
float64 std_x

# Standard deviation in y direction (meters)
float64 std_y

# Standard deviation in z direction (meters)
float64 std_z

# Standard deviation in yaw angle (radians)
float64 std_yaw
