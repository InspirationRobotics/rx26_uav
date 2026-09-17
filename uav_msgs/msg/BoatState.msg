# Where Crusader is and what it is doing, heard over the RFD900 link.
#
# The UAV needs this for Task 1 DISRUPTIVE: it stations over the boat's NEXT
# gate, and a buoy the boat has already passed cannot become that gate, so
# "ahead" and "behind" are measured from this position along the entry -> exit
# line. Published by telemetry_bridge from the boat's own MAVLink position, and
# `activity` from the one byte the boat sends about what it is doing.
#
# Blanks over guesses: published only while it is being heard. Nothing on this
# topic means the boat is not talking, never a remembered position.

std_msgs/Header header

float64 latitude
float64 longitude

uint8 activity                # what the boat says it is doing
uint8 ACTIVITY_UNKNOWN = 0
uint8 ACTIVITY_HOLDING = 1    # station keeping, waiting on us
uint8 ACTIVITY_CIRCLING_ENTRY = 2
uint8 ACTIVITY_TRANSIT = 3    # driving the passage
uint8 ACTIVITY_CIRCLING_EXIT = 4
uint8 ACTIVITY_DONE = 5

uint32 target_buoy            # the buoy id it is working to, 0 = not said
