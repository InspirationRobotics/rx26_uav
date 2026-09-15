# A few autopilot parameters the ground station's pre-flight checks and battery
# readout depend on, READ BACK FROM THE AUTOPILOT rather than copied into our YAML.
#
# WHY READ, NOT CONFIGURE. A copy of BATT_LOW_VOLT in uav_params.yaml would be a
# second source of truth that drifts the first time someone changes the real one
# in QGC -- and the readout built on it would then be confidently wrong about
# when the aircraft is going to turn for home. telemetry_bridge asks for these
# with PARAM_REQUEST_READ (read-only: nothing about how the aircraft flies
# changes) and also picks them up whenever the autopilot broadcasts a change.
#
# Blanks over guesses: every field is NaN until its value has actually been read.
# Latched (TRANSIENT_LOCAL) and published on change, so a late subscriber still
# gets the last known values.
std_msgs/Header header

float64 batt_low_volt       # BATT_LOW_VOLT: the low-voltage failsafe threshold, V
float64 batt_crt_volt       # BATT_CRT_VOLT: the critical threshold, V
float64 batt_capacity_mah   # BATT_CAPACITY
float64 fence_enable        # FENCE_ENABLE (0/1)
float64 fence_alt_max       # FENCE_ALT_MAX, metres above home
