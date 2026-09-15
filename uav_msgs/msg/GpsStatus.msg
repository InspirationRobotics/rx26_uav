# Fix quality of the primary GPS (MAVLink GPS_RAW_INT). Published by
# telemetry_bridge only while fresh.
#
# WHY THE GROUND STATION NEEDS THIS. Every buoy position is the aircraft's GPS
# position plus a camera ray, so the map is only as good as the fix. The
# 2026-09-13 park map repeated to 0.5 m with fix_type 4 (DGPS), 23-26 satellites
# and HDOP 0.53-0.67; a 3D fix on 12 satellites would map noticeably worse, and
# without this topic nothing on screen would say why.
std_msgs/Header header

# GPS_FIX_TYPE as ArduPilot sends it.
uint8 FIX_NO_GPS=0
uint8 FIX_NONE=1
uint8 FIX_2D=2
uint8 FIX_3D=3
uint8 FIX_DGPS=4
uint8 FIX_RTK_FLOAT=5
uint8 FIX_RTK_FIXED=6
uint8 fix_type

# Satellites used. 255 when unknown.
uint8 satellites

# Horizontal dilution of precision: the geometry of the satellites in view, not
# an error in metres. Below 1 is good, above 2 is poor. NaN when unknown.
float64 hdop

# The receiver's own horizontal accuracy estimate, metres. NaN when the receiver
# does not report one (0 on the wire).
float64 h_acc_m
