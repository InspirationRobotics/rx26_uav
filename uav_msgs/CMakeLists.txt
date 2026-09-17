cmake_minimum_required(VERSION 3.8)
project(uav_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# A .msg not listed here is silently NOT generated: the build stays green and
# the import fails at node start, on the airframe. Add the file AND a line.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GlobalPos.msg"
  "msg/Attitude.msg"
  "msg/FlightState.msg"
  "msg/FcuStatus.msg"
  "msg/RcChannels.msg"
  "msg/CameraStatus.msg"
  # Battery, GPS quality and read-back autopilot limits (telemetry_bridge).
  "msg/Battery.msg"
  "msg/GpsStatus.msg"
  "msg/FcuParams.msg"
  # Buoy mapping (uav_perception). BuoyDetections contains BuoyDetection and
  # BuoyMap contains Buoy, so all four must be listed or the import fails.
  "msg/BuoyDetection.msg"
  "msg/BuoyDetections.msg"
  "msg/Buoy.msg"
  "msg/BuoyMap.msg"
  # The buoy search (uav_mission): the fence read back from the autopilot, the
  # one way a node asks to be flown somewhere, and what the search is doing.
  "msg/Fence.msg"
  "msg/GuidedTarget.msg"
  "msg/SearchStatus.msg"
  # Crusader's position and what it is doing (Task 1 disruptive).
  "msg/BoatState.msg"
  "msg/RadioFrame.msg"
  DEPENDENCIES std_msgs
)

ament_package()
