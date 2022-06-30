# Implements SBP MSG_IMU_RAW

uint32 tow    # Milliseconds since reference epoch and time status.
uint8 tow_f   # Milliseconds since reference epoch, fractional part.
int16 acc_x   # Acceleration in the IMU frame X axis
int16 acc_y   # Acceleration in the IMU frame Y axis
int16 acc_z   # Acceleration in the IMU frame Z axis
int16 gyr_x   # Angular rate around IMU frame X axis
int16 gyr_y   # Angular rate around IMU frame Y axis
int16 gyr_z   # Angular rate around IMU frame Z axis
