# Fly to this point and hold there. The ONLY way a node on this aircraft asks
# the autopilot to move it (published by uav_mission's search_node).
#
# telemetry_bridge forwards it as SET_POSITION_TARGET_GLOBAL_INT only while ALL
# of these hold, re-checked on every message:
#   - the autopilot reports GUIDED and ARMED, freshly. GUIDED is the pilot's
#     switch: flip out of it and the autopilot itself ignores these, whatever
#     the companion computer sends.
#   - this message is recent (header.stamp), so a queued backlog is never flown.
#   - the point is inside the fence the autopilot holds (/uav/fence), and at or
#     below FENCE_ALT_MAX - FENCE_MARGIN.
# Nothing here arms, disarms, takes off, lands or picks a mode.

std_msgs/Header header

float64 latitude      # deg
float64 longitude     # deg
float32 alt_rel_m     # metres above HOME (MAV_FRAME_GLOBAL_RELATIVE_ALT)
float32 yaw_deg       # compass heading to hold, 0 = north, clockwise
float32 speed_mps     # horizontal speed for the leg to this point
