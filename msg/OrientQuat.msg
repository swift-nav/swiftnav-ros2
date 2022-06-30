# Implements SBP MSG_ORIENT_QUAT

uint32 tow            # GPS Time of Week
int32 w               # Real component
int32 x               # 1st imaginary component
int32 y               # 2nd imaginary component
int32 z               # 3rd imaginary component
float32 w_accuracy    # Estimated standard deviation of w
float32 x_accuracy    # Estimated standard deviation of x
float32 y_accuracy    # Estimated standard deviation of y
float32 z_accuracy    # Estimated standard deviation of z
uint8 flags           # Status ﬂags (See SBP manual for details)
