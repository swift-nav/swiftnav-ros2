# Implements SBP MSG_WHEELTICK

uint64 time   # Time ﬁeld representing either microseconds since
              # the last PPS, microseconds in the GPS Week or
              # local CPU time from the producing system in microseconds.
              # See the synch_type ﬁeld for the exact meaning of this timestamp.
uint8 flags   # Field indicating the type of timestamp contained in the time ﬁeld.
uint8 source  # ID of the sensor producing this message
int32 ticks   # Free-running counter of the accumulated distance
              # for this sensor. The counter should be incrementing if travelling
              # into one direction and decrementing when travelling in the opposite direction.
