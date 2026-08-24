#!/usr/bin/env bash
# THE one blessed rebuild path. Run INSIDE the `uav` container.
#
#     docker exec uav bash -lc '/root/robotx_ws/src/rx26_uav/tools/scripts/rebuild.sh'
#
# WHY THIS EXISTS RATHER THAN "just run colcon build". Two reasons, both of
# which have cost people an afternoon on the ASV:
#
#   1. --packages-up-to uav_bringup is what builds everything we ship, because
#      bringup exec_depends on all of them. Plain `colcon build` from the wrong
#      directory builds whatever it happens to find.
#   2. A GIT PULL ALONE CHANGES NOTHING THAT IS RUNNING. colcon installs Python
#      into install/; the node does not import from src/. "The change did
#      nothing" is almost always a skipped rebuild, or a skipped restart after
#      one.
set -euo pipefail

WS="${UAV_WS:-/root/robotx_ws}"
ROS_SETUP="${UAV_ROS_SETUP:-/opt/ros/humble/setup.bash}"

[[ -f "$ROS_SETUP" ]] || { echo "ERROR: no ROS at $ROS_SETUP — are you inside the container?" >&2; exit 1; }
[[ -d "$WS/src/rx26_uav" ]] || { echo "ERROR: $WS/src/rx26_uav not found. Is the workspace bind-mounted?" >&2; exit 1; }

# shellcheck disable=SC1090
source "$ROS_SETUP"
cd "$WS"

# Config first: it is the file that can ground the aircraft, it needs no build,
# and finding a broken geofence AFTER a five-minute colcon run is five minutes
# wasted.
python3 src/rx26_uav/tools/scripts/check_config.py

colcon build --packages-up-to uav_bringup --symlink-install

echo
echo "Built. The running nodes are still the OLD code until you restart them:"
echo "  systemctl restart uav-groundstation uav-ocs-client   # on the HOST"
echo "  systemctl restart uav-container                      # for the bridge"
