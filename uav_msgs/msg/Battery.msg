# The flight battery, as the autopilot reports it (MAVLink SYS_STATUS, plus the
# consumed charge from BATTERY_STATUS). Published by telemetry_bridge only while
# fresh, like every other RX stream: silence means the autopilot went quiet, not
# that the battery is fine.
#
# READ THE CURRENT WITH SUSPICION. Ekko's current sensor still carries the
# CubeOrange default calibration (BATT_AMP_PERVLT 39.877) as of 2026-09-14, so
# current and consumed_mah may be scaled wrong by an unknown factor. voltage is
# the trustworthy number, and anything that estimates time left should lean on it.
std_msgs/Header header

# Pack voltage in volts, measured under whatever load the aircraft has RIGHT NOW.
# Under hover current it sags roughly 0.5 V below the resting voltage, and the
# autopilot's low-voltage failsafe compares against this loaded number.
float64 voltage

# Amps drawn. NaN when the autopilot reports no current sensor (-1 on the wire).
float64 current

# mAh drawn since the autopilot booted. NaN when unknown.
float64 consumed_mah

# The autopilot's own 0-100 estimate, or -1 when it has none. It is capacity
# minus consumed, so it ASSUMES THE PACK WAS FULL AT BOOT: a half-charged pack
# reads high. Shown for completeness, never used for a decision here.
int8 remaining_pct
