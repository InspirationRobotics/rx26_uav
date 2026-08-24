# Where the aircraft is in a takeoff/landing cycle, from MAVLink
# EXTENDED_SYS_STATE. The autopilot's own answer to "is it flying?".
#
# WHY THIS IS ITS OWN TOPIC AND NOT A FIELD ON FcuStatus. EXTENDED_SYS_STATE and
# HEARTBEAT are separate MAVLink streams on separate rate groups (SR0_EXT_STAT
# and SR0_EXTRA1) and they die independently. Folding landed_state into
# FcuStatus would make one stream's staleness silently gate the other's — and
# these two have very different consequences downstream: a stale mode string
# costs a greyed-out readout, while a stale landed_state must SUPPRESS the OCS
# heartbeat entirely, because RoboNation refuses FLIGHT_PHASE_UNKNOWN from a UAV
# and a guessed flight phase is relayed onward for Network Remote ID.
#
# Same reasoning Attitude.msg gives for staying separate from GlobalPos.
std_msgs/Header header

# MAV_LANDED_STATE:
#   0 UNDEFINED   autopilot cannot tell — NOT the same as "on the ground"
#   1 ON_GROUND
#   2 IN_AIR
#   3 TAKEOFF
#   4 LANDING
uint8 landed_state

# False when landed_state is UNDEFINED. Carried explicitly so a consumer cannot
# read the 0 as a state: UNDEFINED and ON_GROUND are one integer apart and
# opposite in meaning, which is exactly the confusion worth spending a byte on.
bool valid

uint8 LANDED_STATE_UNDEFINED = 0
uint8 LANDED_STATE_ON_GROUND = 1
uint8 LANDED_STATE_IN_AIR = 2
uint8 LANDED_STATE_TAKEOFF = 3
uint8 LANDED_STATE_LANDING = 4
