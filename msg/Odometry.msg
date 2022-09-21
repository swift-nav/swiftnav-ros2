# Implements SBP MSG_ODOMETRY

uint32 tow      # Time ﬁeld representing either milliseconds in the
                # GPS Week or local CPU time from the producing
                # system in milliseconds. See the tow_source ﬂag
                # for the exact source of this timestamp.
int32 velocity  # The signed forward component of vehicle velocity.
uint8 flags     # Status ﬂags (See SBP manual for details)
