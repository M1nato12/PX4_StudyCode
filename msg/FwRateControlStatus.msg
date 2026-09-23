# Fixed-wing pitch rate controller comparison status

uint64 timestamp                # time since system start [us]

bool adrc_active                # true when ADRC supplies the applied control

float32 pitch_rate_sp           # pitch rate setpoint supplied to the controller [rad/s]
float32 pitch_rate              # measured pitch rate [rad/s]

float32 pid_output              # raw PID output before airspeed scaling; NaN when inactive
float32 adrc_output             # raw ADRC output before airspeed scaling; NaN when inactive
float32 selected_output         # raw output selected by the active controller
float32 applied_output          # pitch output after airspeed scaling, trim and limiting [-1, 1]

float32 adrc_z1                 # ADRC ESO pitch-rate estimate; NaN when inactive [rad/s]
float32 adrc_z2                 # ADRC ESO pitch disturbance estimate; NaN when inactive
float32 adrc_last_u             # ADRC pitch input used by the ESO; NaN when inactive
