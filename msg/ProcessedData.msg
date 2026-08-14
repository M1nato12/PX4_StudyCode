uint64 timestamp                    # time since system start when published (microseconds)
uint64 timestamp_sample             # timestamp of the original sampled data (microseconds)

float32[4] attitude_q               # processed attitude quaternion (unitless)
float32[3] angular_velocity         # processed body angular velocity (rad/s)

# TOPICS processed_data
