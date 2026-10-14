# Each ESC's own telemetry, as the autopilot relays it (MAVLink ESC_TELEMETRY_*).
# Published by telemetry_bridge only while fresh, like every other RX stream.
#
# One entry per ESC that is actually reporting -- its packet count above zero.
# An ESC that has never reported is ABSENT, never a 0 that would read as a cold
# one, so the three arrays are always the same length and may be empty.
#
# Ekko: Lumenier ELITE PRO 60A AM32 4-in-1, telemetry on SERIAL1, autopilot
# outputs 9-12 = indices 8-11. The ESC's current and voltage fields are NOT
# carried: on 2026-10-13 they read 81 A and 16.5 V per channel on a disarmed
# aircraft at 24.9 V.
std_msgs/Header header

# Autopilot output, 0-based (8 = SERVO9).
uint8[] index

# Degrees C, from the ESC's own sensor. Reached 96 C after 4.5 min of hover at
# 5.4 kg on 2026-10-13, which is why the page shows it.
float32[] temperature_c

# Motor rpm as the autopilot reports it.
float32[] rpm
