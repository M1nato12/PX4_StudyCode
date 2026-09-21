uint64 timestamp                  # [us] time since system start

float32 north_m                   # [m] local north position relative to first valid GNSS position
float32 east_m                    # [m] local east position relative to first valid GNSS position
float32 down_m                    # [m] local down position relative to first valid GNSS altitude

uint64 beijing_time_usec          # [us] UTC + 8 h, for logging/display only

float32 baseline_length_m         # [m] dual-antenna baseline length from PQTMTAR
uint8 quality                     # heading solution quality from PQTMTAR
uint8 used_sv                     # satellites used for heading solution
