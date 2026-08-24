#!/usr/bin/env bash
# Set the workspace up INSIDE the `uav` container. Run once per container.
#
#     docker exec uav bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh
#
# The host half is setup/install_jetson_host.sh; this is the other half. They
# are separate because they touch different machines' state: udev and systemd
# are host-level, and the ROS build is not.
set -euo pipefail

WS="${UAV_WS:-/root/robotx_ws}"
REPO="$WS/src/rx26_uav"
ROS_SETUP="${UAV_ROS_SETUP:-/opt/ros/humble/setup.bash}"

[[ -f "$ROS_SETUP" ]] || { echo "ERROR: no ROS at $ROS_SETUP — this must run INSIDE the container." >&2; exit 1; }
[[ -d "$REPO" ]] || {
  echo "ERROR: $REPO not found." >&2
  echo "       The workspace is not bind-mounted. Recreate the container with" >&2
  echo "         -v ~/robotx_ws:/root/robotx_ws" >&2
  echo "       (see setup/install_jetson_host.sh step 4)." >&2
  exit 1; }

echo "== python deps =="
# pymavlink for telemetry_bridge, pyyaml for the params loader. MAVProxy itself
# runs on the HOST, not here — it owns a serial device and this container must
# not.
python3 -m pip install --no-cache-dir pymavlink pyyaml

echo "== first build =="
bash "$REPO/tools/scripts/rebuild.sh"

echo "== smoke check =="
# ROS 2's setup.bash reads variables it never sets (AMENT_TRACE_SETUP_FILES,
# COLCON_TRACE, ...), so `set -u` aborts the moment it is sourced:
#   /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# nounset is worth keeping for OUR code, so it is lifted only across the source
# and restored immediately. Do not "simplify" this by dropping -u from the
# script: a typo'd variable name here silently builds the wrong workspace.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
source "$WS/install/setup.bash"
set -u
python3 - <<'PY'
from uav_common import config, geo, fence_core
from uav_groundstation import heartbeat_core, node_registry, ocs_link
print("  params:", config.DEFAULT_CONFIG_PATH)
print("  geofence vertices:", len(fence_core.items_from_polygon(
    config.node_params("telemetry_bridge")["geofence"])))
print("  registry:", [n.name for n in node_registry.REGISTRY])
PY
ros2 pkg executables uav_fcu
ros2 pkg executables uav_groundstation

echo
echo "Done. Start the units from the HOST:"
echo "  systemctl start uav-mavproxy uav-container uav-groundstation uav-ocs-client"
