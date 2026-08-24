# Autopilot status from MAVProxy rebroadcast (HEARTBEAT-derived).
std_msgs/Header header
string mode            # e.g. "STABILIZE", "LOITER", "GUIDED", "RTL", "LAND"
bool armed
uint8 system_status    # MAV_STATE value
