# Implements SBP MSG_BASELINE_NED

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
std_msgs/Header header

# Mode
uint8 mode

# Number of satellites used in the solution
uint8 satellites_used

# Baseline NED [meters]
float64 baseline_n_m
float64 baseline_e_m
float64 baseline_d_m

# Estimated baseline error [meters]
float64 baseline_err_h_m     # Horizontal
float64 baseline_err_v_m     # Vertical

# Baseline length [meters]
float64 baseline_length_m    # 3D
float64 baseline_length_h_m  # Horizontal

# RTK orientation validity
bool baseline_orientation_valid

# Direction [degrees] in range [0..360)
float64 baseline_dir_deg

# Estimated direction error [degrees] in range [0..180]
float64 baseline_dir_err_deg

# Dip [degrees] in range [-90..90]
float64 baseline_dip_deg

# Estimated dip error [degrees] in range [0..90]
float64 baseline_dip_err_deg
