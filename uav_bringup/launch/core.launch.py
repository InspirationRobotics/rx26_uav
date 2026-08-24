"""core.launch.py — the always-on UAV telemetry stack.

Starts exactly one node:

  * telemetry_bridge — uav_fcu; the single ROS-side gateway to MAVProxy's
                       rebroadcast (/uav/pose, /uav/attitude, /uav/fcu_status,
                       /uav/flight_state, /uav/rc_channels, /uav/autonomy_drop),
                       the sanctioned RC-override TX path, and the geofence
                       uploader.

WHAT IS NOT HERE, AND WHY. `ground_station` and `ocs_client` are started by
their own systemd units (tools/systemd/), not by this launch. That is
deliberate: a launch file is one process tree, so a crash-looping ground station
would take the telemetry gateway down with it on every restart. Separate units
mean the gateway keeps flying while a display or a reporting link is being
fixed, and `systemctl` can disable either without editing this file.

MAVProxy itself (the sole Pixhawk owner) is started outside ROS by systemd — see
scripts/start_mavproxy.sh — before this launch. Nothing here may open
/dev/uav-pixhawk.

  ros2 launch uav_bringup core.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The same file uav_common/config.py resolves for declaration defaults.
    # Both paths must point at one file, or a node's declared default and its
    # launched value can disagree — which is invisible until behaviour differs.
    params = os.path.join(
        get_package_share_directory("uav_bringup"),
        "config", "uav_params.yaml")

    return LaunchDescription([
        Node(package="uav_fcu", executable="telemetry_bridge",
             output="screen", parameters=[params]),
    ])
