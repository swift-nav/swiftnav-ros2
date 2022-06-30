# Implements SBP MSG_ORIENT_EULER

uint32 tow                # GPS Time of Week
int32 roll                # rotation about the forward axis of the vehicle
int32 pitch               # rotation about the rightward axis of the vehicle
int32 yaw                 # rotation about the downward axis of the vehicle
float32 roll_accuracy     # Estimated standard deviation of roll
float32 pitch_accuracy    # Estimated standard deviation of pitch
float32 yaw_accuracy      # Estimated standard deviation of yaw
uint8 flags               # # Status ﬂags (See SBP manual for details)
