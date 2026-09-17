# One MAVLink frame that crossed the RFD900 radio, as telemetry_bridge sent it or
# heard it. Published on /uav/radio/traffic for the ground station's Radio tab.
#
# A RECORD, NOT A CHANNEL. Nothing may act on this topic. It is published after
# the fact by the one node that owns the link, so an operator can see what went
# out and what came in; acting on it would be a second path to the boat with
# none of telemetry_bridge's checks on it.
std_msgs/Header header    # when telemetry_bridge sent or received the frame

uint8 DIR_TX=1
uint8 DIR_RX=2
uint8 direction

uint8 src_system          # who sent it; for TX, telemetry_bridge's own id
uint8 src_component
uint8 dst_system          # who it is addressed to; 0 = broadcast, or no target field

string msg_name           # BOAT, BUOY_MAP, TEST, TUNNEL_0x80xx, or the MAVLink name
uint16 payload_type       # the TUNNEL payload_type; 0 for any other message
string summary            # one line, decoded where uav_common/boat_link.py knows the format
uint16 frame_bytes        # bytes on the air, MAVLink framing included
