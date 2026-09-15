# Every buoy box from ONE frame, plus everything needed to put them on a map.
#
# WHY THE POSE RIDES IN THE MESSAGE instead of the mapper subscribing to
# /uav/pose itself. camera_node already records, for every frame, the pose and
# gimbal angles it held when that frame arrived -- that is the _frames.csv row.
# It stamps the same numbers onto the viewer stream, the detector copies them
# here, and so the live mapper and the offline replay tool read IDENTICAL
# inputs. A mapper that re-derived pose from its own subscriptions would be
# answering a slightly different question from the recording, and a live map
# that disagrees with its own replay is a map nobody can debug.
#
# UNKNOWN IS NaN, NEVER A GUESS. When camera_node had no fresh pose or gimbal
# answer for this frame, the field is NaN and the mapper refuses the frame.
std_msgs/Header header        # stamp = ROS time the frame ARRIVED at camera_node
                              # (its ros_time_ns), not when detection finished.
                              # Falls back to detection time only when
                              # frame_meta_ok is false.

# False when the frame arrived without camera_node's metadata (an older
# camera_node, or a frame the metadata could not be matched to). Every pose
# field below is then NaN and the mapper uses none of these detections.
bool frame_meta_ok
string session                # camera_node session stem; joins to _frames.csv
uint64 frame_idx              # the _frames.csv join key
uint32 image_width
uint32 image_height

# Aircraft pose at the frame (camera_node's pose/attitude caches).
float64 latitude              # deg
float64 longitude             # deg
float64 altitude_rel          # m above HOME, not above the surface
float64 roll                  # rad, autopilot axes (see Attitude.msg)
float64 pitch                 # rad
float64 yaw                   # rad, 0 = true north, clockwise positive
float64 pose_age_s            # age of the pose sample when the frame arrived

# Gimbal, MEASURED (siyi_client), at the frame.
float64 gimbal_pitch          # deg, nadir is -90
float64 gimbal_yaw            # deg, as the gimbal reports it -- see buoy_mapper
                              # params gimbal_yaw_mode / gimbal_yaw_sign
float64 gimbal_yaw_rate       # deg/s; large = camera still swinging round
float64 gimbal_age_s          # age of the gimbal sample when the frame arrived

BuoyDetection[] detections
