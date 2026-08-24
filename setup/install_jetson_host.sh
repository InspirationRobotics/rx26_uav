#!/usr/bin/env bash
# ============================================================================
# setup/install_jetson_host.sh — JETSON HOST setup (outside the container)
#
# Run ONCE per Jetson, and again after cabling changes. Installs the whole
# unattended boot chain in dependency order:
#
#   [1] workspace layout      ~/robotx_ws/src/rx26_uav
#   [2] udev rules            stable /dev/uav-pixhawk
#   [3] systemd units         mavproxy -> container -> ground station + OCS client
#   [4] container             created if absent; mounts CHECKED if present
#   [5] sanity checks
#
# Usage:   sudo bash setup/install_jetson_host.sh
#
# Idempotent: safe to re-run. It never deletes a container and never moves a git
# repo — both of those can destroy work, so where one is needed it prints the
# command and stops.
# ============================================================================
set -euo pipefail

# Fail LOUDLY. `set -e` aborts with NO message at all, so a step that dies
# halfway leaves a partially-installed boot chain looking like a clean run. The
# ASV's version of this script once stopped after step 1 with nothing but a
# warning on screen, and no systemd units were written.
trap 'rc=$?; echo >&2; echo "ERROR: install_jetson_host.sh ABORTED at line $LINENO (exit $rc)." >&2; echo "       The install is INCOMPLETE — no step after this one ran." >&2; echo "       Fix the cause and re-run; the script is idempotent." >&2' ERR

cd "$(dirname "$0")/.."          # repo root
UAV_REPO="$(pwd)"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "ERROR: must run as root (udev + systemd installs)." >&2
  echo "       sudo bash setup/install_jetson_host.sh" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# The invoking user, and their REAL home.
#
# Under sudo, `~` is /root and $HOME is root's. Every path derived from it would
# be silently wrong — the workspace would be built in root's home, the units
# would point at it, and the mount would not match. Ask passwd, not the
# environment.
# ---------------------------------------------------------------------------
UAV_USER="${SUDO_USER:-$USER}"
id -u "$UAV_USER" >/dev/null 2>&1 || {
  echo "ERROR: user '$UAV_USER' does not exist — cannot install units." >&2
  echo "       Run with sudo from that user's session, or set SUDO_USER." >&2
  exit 1; }
USER_HOME="$(getent passwd "$UAV_USER" | cut -d: -f6)"
UAV_CONTAINER="${UAV_CONTAINER:-uav}"
UAV_IMAGE="${UAV_IMAGE:-uav}"
POWER_SOCK="${UAV_POWER_SOCKET:-/run/uav-power.sock}"

echo "== [1/5] workspace layout =="
# The workspace is DERIVED FROM THE REPO, not from a constant, so a workspace
# somewhere other than ~/robotx_ws keeps working.
#   <WS>/src/rx26_uav  ->  WS is two levels up
WS_HOST="$(dirname "$(dirname "$UAV_REPO")")"
SRC_PARENT="$(basename "$(dirname "$UAV_REPO")")"

if [[ "$SRC_PARENT" != "src" ]]; then
  # REFUSE TO RELOCATE. Moving someone's git repo unasked is how uncommitted
  # work disappears — print the two commands and let them decide.
  echo "ERROR: this repo is not inside a colcon workspace." >&2
  echo "       Found:    $UAV_REPO" >&2
  echo "       Expected: <workspace>/src/rx26_uav" >&2
  echo >&2
  echo "       Everything downstream assumes that layout: the bind mount, the" >&2
  echo "       systemd units' repo path, uav_common/config.py's source-tree" >&2
  echo "       fallback, and rebuild.sh." >&2
  echo >&2
  echo "       Move it, then re-run from the new location:" >&2
  echo "         mkdir -p $USER_HOME/robotx_ws/src" >&2
  echo "         mv '$UAV_REPO' $USER_HOME/robotx_ws/src/" >&2
  echo "         cd $USER_HOME/robotx_ws/src/rx26_uav" >&2
  echo "         sudo bash setup/install_jetson_host.sh" >&2
  exit 1
fi

created=""
for d in "$WS_HOST" "$WS_HOST/src"; do
  if [[ ! -d "$d" ]]; then
    mkdir -p "$d"
    created="$created $d"
  fi
done
# chown BACK. mkdir under sudo produces root-owned directories, and the user
# then cannot write to their own workspace: colcon fails, git pull fails, and
# the error appears three steps downstream of the cause.
chown "$UAV_USER":"$(id -gn "$UAV_USER")" "$WS_HOST" "$WS_HOST/src" 2>/dev/null || true
echo "   workspace:  $WS_HOST"
echo "   repo:       $UAV_REPO"
echo "   user:       $UAV_USER  (home $USER_HOME)"
[[ -n "$created" ]] && echo "   created:   $created"
# Other package sources may live in src/ alongside ours. --packages-up-to
# uav_bringup is what keeps the build to this repo; COLCON_IGNORE is for
# anything that should not even be scanned.
other="$(find "$WS_HOST/src" -maxdepth 1 -mindepth 1 -type d ! -name rx26_uav -printf '%f ' 2>/dev/null || true)"
[[ -n "$other" ]] && echo "   note: other sources in src/: $other (drop a COLCON_IGNORE in any you do not build)"

echo "== [2/5] udev rules (stable /dev/uav-pixhawk) =="
bash tools/udev/install_udev.sh

echo "== [3/5] systemd units =="
# The units are TEMPLATES: the service account, repo path and container name
# differ per Jetson, and a hardcoded /home/<someone> fails silently at boot —
# which you discover in the field.
echo "   container:  $UAV_CONTAINER"
UNITS=(uav-mavproxy uav-container uav-groundstation uav-ocs-client uav-power)
for unit in "${UNITS[@]}"; do
  sed -e "s|__UAV_USER__|$UAV_USER|g" \
      -e "s|__UAV_REPO__|$UAV_REPO|g" \
      -e "s|__UAV_CONTAINER__|$UAV_CONTAINER|g" \
      "tools/systemd/$unit.service" > "/etc/systemd/system/$unit.service"
  chmod 644 "/etc/systemd/system/$unit.service"
  # Fail loudly rather than enabling a unit that still carries a placeholder.
  if grep -q "__UAV_" "/etc/systemd/system/$unit.service"; then
    echo "ERROR: $unit.service still has unsubstituted placeholders." >&2
    grep -n "__UAV_" "/etc/systemd/system/$unit.service" >&2
    exit 1
  fi
  echo "   installed $unit.service"
done

# Seed /etc/default/uav if it does not exist. The units reference it with a
# leading `-` so a missing file is fine, but having it there with the knobs
# named is how anyone discovers they exist.
if [[ ! -f /etc/default/uav ]]; then
  cat > /etc/default/uav <<'DEFAULTS'
# Environment for the uav-* systemd units. Edit here, not in the unit files.

# SUBNET BROADCAST ADDRESS for MAVProxy's GCS output (NOT a laptop IP).
# .255 for a /24. Change this if the field network hands out a different subnet.
UAV_BCAST_ADDR=192.168.8.255

# Escape hatch for laptops the broadcast does not reach (AP client isolation,
# the other side of a bridge). Space-separated and QUOTED — systemd takes the
# rest of the line as one value. Additive: broadcast stays on regardless.
#GCS_IPS="192.168.8.50 192.168.1.20"

# Where the power helper listens. Must be bind-mounted into the container for
# the ground station's System tab to reach it.
UAV_POWER_SOCKET=/run/uav-power.sock
UAV_POWER_GROUP=uav
DEFAULTS
  echo "   seeded /etc/default/uav"
fi

systemctl daemon-reload
systemctl enable "${UNITS[@]}" >/dev/null
echo "   enabled all five; start now with:"
echo "     systemctl start ${UNITS[*]}"

echo "== [4/5] container =="
# ---------------------------------------------------------------------------
# THE POWER SOCKET MUST EXIST BEFORE THE CONTAINER IS CREATED.
#
# `docker run -v /run/uav-power.sock:/run/uav-power.sock` when that path does
# not exist makes Docker create a DIRECTORY there — on the host. Two things then
# break, and neither is obvious:
#   * the container mounts a directory where it expects a socket, so the System
#     tab reports the helper unreachable forever;
#   * uav_power_helper.py's os.unlink() of the stale socket fails with
#     IsADirectoryError, so uav-power.service crash-loops at every boot.
# Starting the helper first means the socket is a real socket when Docker binds
# it. This is cheap and idempotent, so it runs unconditionally.
if [[ -d "$POWER_SOCK" ]]; then
  # Left by exactly the mistake described above, on an earlier run.
  echo "   removing a DIRECTORY at $POWER_SOCK (Docker made it; it belongs to"
  echo "   an earlier container created before the helper was running)"
  systemctl stop uav-container 2>/dev/null || true
  rmdir "$POWER_SOCK" 2>/dev/null || rm -rf "$POWER_SOCK"
fi
# The helper chowns the socket to this group. Without it the socket is
# root-only and the helper logs a warning that reads like a failure — harmless
# here (the container runs as root) but confusing.
groupadd -f "${UAV_POWER_GROUP:-uav}" 2>/dev/null || true
systemctl start uav-power || echo "WARN: uav-power did not start; the System tab will say so."
for _ in 1 2 3 4 5; do [[ -S "$POWER_SOCK" ]] && break; sleep 1; done
if [[ -S "$POWER_SOCK" ]]; then
  echo "   power socket ready at $POWER_SOCK"
else
  echo "WARN: $POWER_SOCK is not a socket yet. The container will still be"
  echo "      created, but bind-mounting a missing path makes Docker invent a"
  echo "      directory — so the power tab will not work until you fix the"
  echo "      helper (journalctl -u uav-power) and recreate the container."
fi

# `docker start` CANNOT add mounts — they are fixed when the container is
# CREATED. So this step splits on a distinction that matters:
#   absent  -> create it (nothing exists to lose)
#   present -> CHECK the mounts and print the fix; NEVER `docker rm`
if ! command -v docker >/dev/null; then
  echo "WARN: docker is not installed — uav-container.service will fail."
elif docker inspect "$UAV_CONTAINER" >/dev/null 2>&1; then
  MOUNTS="$(docker inspect -f '{{range .Mounts}}{{.Source}}:{{.Destination}} {{end}}' "$UAV_CONTAINER" 2>/dev/null || true)"
  MISSING=""
  case "$MOUNTS" in *":/root/robotx_ws"*) ;; *) MISSING="$MISSING workspace" ;; esac
  case "$MOUNTS" in *"$POWER_SOCK"*) ;; *) MISSING="$MISSING power-socket" ;; esac
  if [[ -n "$MISSING" ]]; then
    echo "WARN: container '$UAV_CONTAINER' is MISSING mounts:$MISSING"
    # Only the missing ones. Listing both consequences whatever is absent reads
    # as "your work will be destroyed" to someone whose workspace mount is fine,
    # and sends them recreating a container they did not need to touch.
    case "$MISSING" in *workspace*)
      echo "        workspace    -> a 'git pull' on the host is INVISIBLE inside"
      echo "                        the container, a rebuild silently changes"
      echo "                        nothing, and 'docker rm' discards the lot." ;;
    esac
    case "$MISSING" in *power-socket*)
      echo "        power-socket -> the System tab cannot power the Jetson down;"
      echo "                        it will say so. Everything else works." ;;
    esac
    echo "      Mounts are fixed at CREATE time, so this cannot be fixed by a"
    echo "      restart. Recreating is YOUR call — 'docker rm' throws away"
    echo "      anything living only inside the container:"
    echo
    echo "        docker rm -f $UAV_CONTAINER"
    echo "        docker run -d --name $UAV_CONTAINER \\"
    echo "          --restart unless-stopped --network host --privileged \\"
    echo "          -v $WS_HOST:/root/robotx_ws \\"
    echo "          -v $POWER_SOCK:$POWER_SOCK \\"
    echo "          -v /dev:/dev \\"
    echo "          $UAV_IMAGE tail -f /dev/null"
    echo
    echo "      Then: docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  else
    echo "   mounts OK: workspace and power socket are both bind-mounted."
  fi
elif docker image inspect "$UAV_IMAGE" >/dev/null 2>&1; then
  # Nothing to lose: create it with both mounts already right.
  echo "   no container named '$UAV_CONTAINER' — creating it"
  docker run -d --name "$UAV_CONTAINER" \
    --restart unless-stopped --network host --privileged \
    -v "$WS_HOST":/root/robotx_ws \
    -v "$POWER_SOCK":"$POWER_SOCK" \
    -v /dev:/dev \
    "$UAV_IMAGE" tail -f /dev/null >/dev/null
  echo "   created. Build the workspace inside it:"
  echo "     docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
else
  echo "WARN: no image '$UAV_IMAGE' and no container '$UAV_CONTAINER'."
  echo "      Build the image first, then re-run this script:"
  echo "        docker build -t $UAV_IMAGE $UAV_REPO"
fi

echo "== [5/5] sanity checks =="
command -v mavproxy.py >/dev/null \
  || echo "WARN: mavproxy.py not on PATH — uav-mavproxy.service will fail."
[[ -e /dev/uav-pixhawk ]] \
  || echo "WARN: /dev/uav-pixhawk does not exist yet. Plug/replug the Pixhawk, or confirm its VID/PID (tools/udev/99-uav.rules says how)."
python3 "$UAV_REPO/tools/scripts/check_config.py" >/dev/null 2>&1 \
  || echo "WARN: check_config.py reports problems — run it directly to see them."

echo
echo "Done. Ports on this vehicle: 14541 -> ROS, 14540 -> GCS (the boat uses 1455x)."
echo "  systemctl start ${UNITS[*]}"
echo "  http://\$(hostname -I | awk '{print \$1}'):8090"
echo
echo "Everyday loop after a code change:"
echo "  cd $UAV_REPO && git pull"
echo "  docker exec $UAV_CONTAINER bash -lc '/root/robotx_ws/src/rx26_uav/tools/scripts/rebuild.sh'"
echo "  systemctl restart uav-groundstation uav-ocs-client"
echo "A pull WITHOUT a rebuild changes nothing that is running — colcon installs"
echo "into install/, and the node does not import from src/."
