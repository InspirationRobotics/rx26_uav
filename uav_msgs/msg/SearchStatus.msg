# What the buoy search is doing, for the ground station. Published by
# uav_mission's search_node twice a second, whole every time.

std_msgs/Header header

# off | not_ready | ready | climb | sweep | divert | hover | return | paused |
# holding | rtl | complete
string phase
# One line for the pilot, in words: "confirming B4 (6 s of 10)".
string text
# Why it would not start right now. Empty once ready.
string[] waiting_for

bool enabled              # the ground station's on/off
uint16 buoys_to_find      # the ground station's count
uint16 found              # confirmed (locked) buoys inside the fence
bool flying               # true while it is the one moving the aircraft

# Task 1 tier: "advanced" (map it, go home) or "disruptive" (stay with the boat).
string tier
# The gate being kept an eye on, and the buoys queued for a re-read after a
# light changed. Disruptive only.
uint32[] watching
uint32[] rechecking
# What Crusader may drive to, CONFIRMED on a fresh look: the boat's next gate as
# [red, green], or the exit as [id]. Set only while escorting, with the aircraft
# over it and every light in it seen in the last few seconds. `watching` is
# where the aircraft is going; this is what it has actually checked.
uint32[] confirmed
string note               # why a gate was lost, in words

uint16 pass_number        # 1-based; each pass turns the lines 90 degrees
uint16 leg                # waypoint index within the pass, 1-based
uint16 legs               # waypoints in the pass

uint32[] hover_buoys      # buoys being confirmed now
float32 hover_s           # seconds over them so far
float32 give_up_s         # the limit
uint32[] skipped          # given up on during this pass

# Where it is heading, or NaN.
float64 target_latitude
float64 target_longitude

# The current pass as a path, and the fence shrunk by the inset it keeps.
float64[] plan_latitude
float64[] plan_longitude
float64[] inset_latitude
float64[] inset_longitude
