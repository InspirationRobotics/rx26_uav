# Aircraft attitude from the Pixhawk EKF (MAVLink ATTITUDE).
#
# Angles [rad], ArduPilot/NED body axes (x forward, y RIGHT, z DOWN):
#   roll  + = right side down
#   pitch + = nose up
#   yaw   + = clockwise from TRUE north (compass convention)
#
# NOTE the axes are NOT REP-103 (x fwd, y LEFT, z UP). These are the autopilot's
# own axes, republished UNCONVERTED so the numbers on the wire match what QGC
# shows during a bench check. Converting here would put a frame convention in
# the gateway, where nothing can check it; uav_common.geo owns that instead.
#
# The RxReport heartbeat wants roll_deg/pitch_deg — degrees, same axes. The
# conversion is one math.degrees() in ocs_client and lives there, not here.
std_msgs/Header header
float64 roll
float64 pitch
float64 yaw

# Body-axis angular rates [rad/s], same axes and signs as the angles above.
# Carried because they arrive in the same MAVLink frame at zero cost.
float64 rollspeed
float64 pitchspeed
float64 yawspeed
