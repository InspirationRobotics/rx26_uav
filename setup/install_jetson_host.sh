#!/usr/bin/env bash
# ============================================================================
# setup/install_jetson_host.sh — JETSON HOST setup (outside the container)
#
# Run ONCE per Jetson, and again after cabling changes. Installs the whole
# unattended boot chain in dependency order:
#
#   [1] workspace layout      ~/robotx_ws/src/rx26_uav
#   [2] executable bits       or systemd fails ExecStart with a bare 203/EXEC
#   [3] host prerequisites    docker + MAVProxy + the docker group, with the
#                             exact fix for each
#   [4] udev rules + dialout  stable /dev/uav-pixhawk, and permission to OPEN it
#   [5] systemd units         mavproxy -> container -> ground station + OCS client
#   [6] power helper          started BEFORE the container, so its socket exists
#   [7] container             created if absent; mounts CHECKED if present
#
# Usage:   sudo bash setup/install_jetson_host.sh
#
# Idempotent: safe to re-run. It never deletes a container and never moves a git
# repo — both of those can destroy work, so where one is needed it prints the
# command and stops.
#
# EVERY CHECK IN HERE EXISTS BECAUSE IT ACTUALLY BIT SOMEONE during the first
# bring-up on Ekko (2026-08-24). In order: the params file used a nested list
# that rcl cannot declare; rebuild.sh aborted because `set -u` met ROS's
# setup.bash; systemd refused the scripts with 203/EXEC because git carried no
# +x; MAVProxy was not installed and gave a bare 127; and the service user was
# not in dialout, so MAVProxy imported for 1.4 s and then failed to open the
# autopilot with status=1. None of those name their own cause. That is what the
# steps below are for.
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
POWER_SOCK="${UAV_POWER_SOCKET:-/run/uav/power.sock}"
# The container bind-mounts this DIRECTORY, never the socket file itself.
# See step [6] for why that distinction is the whole ballgame.
POWER_DIR="$(dirname "$POWER_SOCK")"
POWER_SOCK_LEGACY=/run/uav-power.sock

echo "== [1/7] workspace layout =="
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

echo "== [2/7] executable bits =="
# ---------------------------------------------------------------------------
# Make the scripts systemd will exec DIRECTLY executable.
#
# systemd calls execve() on ExecStart=, so a script without +x fails with
#   Main PID: ... (code=exited, status=203/EXEC)
# which names no file and no reason. It is the single least informative failure
# in the whole boot chain.
#
# Git records the mode bit, so a clone is fine — but these files also travel by
# zip, scp, rsync -r and cloud sync, every one of which drops it. Re-asserting
# it here costs nothing and removes a failure mode that is pure lost time.
# ---------------------------------------------------------------------------
for d in scripts setup tools/udev tools/scripts tools/bench; do
  chmod +x "$UAV_REPO/$d"/*.sh "$UAV_REPO/$d"/*.py 2>/dev/null || true
done
# Named explicitly rather than trusted to the loop: these three are the ones
# systemd execs, so if any of them is still not executable, say so now instead
# of at the next boot.
for s in scripts/start_mavproxy.sh scripts/run_in_container.sh; do
  [[ -x "$UAV_REPO/$s" ]] || { echo "ERROR: $s is not executable and chmod did not fix it." >&2
                               echo "       systemd will fail it with 203/EXEC." >&2
                               ls -l "$UAV_REPO/$s" >&2; exit 1; }
done
echo "   scripts are executable"

echo "== [3/7] host prerequisites =="
# ---------------------------------------------------------------------------
# Checked HERE, before any unit is enabled, because both of these fail at boot
# with an error that names neither the missing thing nor where it comes from.
# A WARN at the end of a long install scrolls past; a named fix does not.
# ---------------------------------------------------------------------------
missing_prereq=0

if command -v mavproxy.py >/dev/null 2>&1; then
  echo "   mavproxy.py: $(command -v mavproxy.py)"
else
  missing_prereq=1
  echo "MISSING: mavproxy.py is not on PATH."
  echo "   uav-mavproxy.service will fail with status=127 (command not found)."
  echo "   MAVProxy runs on the HOST — it owns the Pixhawk serial link and is"
  echo "   deliberately NOT in the container. Install it SYSTEM-WIDE:"
  echo "     sudo apt install -y python3-pip python3-dev python3-lxml"
  echo "     sudo pip3 install MAVProxy"
  echo "   NOT 'pip3 install --user': that lands in ~/.local/bin, which is not"
  echo "   on systemd's PATH, so it works in your shell and keeps failing here."
fi

if command -v docker >/dev/null 2>&1; then
  echo "   docker:      $(command -v docker)"
  # -------------------------------------------------------------------------
  # The service user must be in `docker`, or uav-container.service cannot reach
  # the daemon at all.
  #
  # This script runs as root, so the container it creates in step [7] works and
  # `sudo docker ps` works — but uav-container.service runs as User=$UAV_USER,
  # and /var/run/docker.sock is root:docker 0660. Without the group,
  # `docker start -a` exits 1 after a few MILLISECONDS with "permission denied
  # while trying to connect to the Docker daemon socket", and systemd reports a
  # bare status=1/FAILURE that names neither Docker nor permissions. It reads
  # exactly like a container that will not boot.
  #
  # It does not stay a one-unit problem either. uav-container's ExecStop runs
  # `docker stop` on every failed restart cycle, and a manual stop clears the
  # container's `--restart unless-stopped` flag — so Docker stops bringing it up
  # at boot too, and now the container really is down for a second reason.
  # -------------------------------------------------------------------------
  if id -nG "$UAV_USER" | tr ' ' '\n' | grep -qx docker; then
    echo "   $UAV_USER is already in the docker group"
  else
    groupadd -f docker
    usermod -aG docker "$UAV_USER"
    echo "   added $UAV_USER to the docker group (uav-container.service needs it)"
    echo "   NOTE: your existing shell does not have it until you log out and"
    echo "         back in; systemd services pick it up on their next start."
  fi
else
  missing_prereq=1
  echo "MISSING: docker is not installed — uav-container.service will fail."
fi

if (( missing_prereq )); then
  echo
  echo "   Continuing: the units below will be installed and enabled, but any"
  echo "   service whose prerequisite is missing will fail until you fix it."
  echo "   Re-run this script afterwards; it is idempotent."
fi

echo "== [4/7] udev rules + dialout (stable /dev/uav-pixhawk) =="
bash tools/udev/install_udev.sh

# ---------------------------------------------------------------------------
# The service user must be in `dialout`, or it cannot OPEN the autopilot.
#
# 99-uav.rules sets MODE="0660" GROUP="dialout" on the Pixhawk tty, and
# uav-mavproxy.service runs as User=$UAV_USER. If that user is not in dialout,
# MAVProxy starts, pays the full Python import cost, fails to open the device
# and exits 1 — roughly 1.4 s of CPU for a permission error. The unit reports
# `status=1/FAILURE` and nothing about permissions.
#
# Group membership only takes effect on a NEW session, which is why this is
# worth doing here rather than leaving to the operator: systemd starts a fresh
# process each time, so the service picks it up on the next start even though
# the user's existing shell will not.
# ---------------------------------------------------------------------------
if id -nG "$UAV_USER" | tr ' ' '\n' | grep -qx dialout; then
  echo "   $UAV_USER is already in dialout"
else
  usermod -aG dialout "$UAV_USER"
  echo "   added $UAV_USER to dialout (needed to open /dev/uav-pixhawk)"
  echo "   NOTE: your existing shell does not have it until you log out and"
  echo "         back in; systemd services pick it up on their next start."
fi

echo "== [5/7] systemd units =="
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

# Where the power helper listens. Its PARENT DIRECTORY is what gets
# bind-mounted into the container — never the socket file, which is a fresh
# inode on every helper restart and would leave the container holding a deleted
# one. Move this and you must recreate the container to match.
UAV_POWER_SOCKET=/run/uav/power.sock
UAV_POWER_GROUP=uav
DEFAULTS
  echo "   seeded /etc/default/uav"
fi

systemctl daemon-reload
systemctl enable "${UNITS[@]}" >/dev/null
echo "   enabled all five; start now with:"
echo "     systemctl start ${UNITS[*]}"

echo "== [6/7] power helper, then [7/7] container =="
# ---------------------------------------------------------------------------
# THE CONTAINER MOUNTS THE DIRECTORY /run/uav, NOT THE SOCKET FILE.
#
# It used to mount the socket itself, and that is a trap with two jaws:
#
#   * A bind mount of a FILE pins an INODE. uav_power_helper.py unlinks and
#     re-binds its socket on every start, so one `systemctl restart uav-power`
#     leaves the container holding a deleted inode. Connections fail, the
#     System tab says "unreachable", and only a CONTAINER restart fixes it —
#     nothing on either side logs why.
#   * Docker, told to bind-mount a path that does not exist yet, INVENTS A
#     DIRECTORY at it on the host. The container then has a directory where it
#     wants a socket, and the helper's os.unlink() hits IsADirectoryError and
#     crash-loops. Worse, the container's own rootfs keeps that directory, so
#     once the host path IS a real socket, `docker start` dies with
#     "not a directory: Are you trying to mount a directory onto a file" and
#     the container cannot be started AT ALL until it is recreated.
#
# Mounting the parent directory removes both: a directory is what Docker would
# create anyway, and the container resolves the socket name on each connect, so
# a helper restart is invisible to it.
#
# uav-power.service creates /run/uav via RuntimeDirectory=uav (with
# RuntimeDirectoryPreserve=yes so a restart does not delete it out from under
# the running container). We create it here too, because the container may be
# created before that unit has ever started.
install -d -m 0755 "$POWER_DIR"

# MIGRATION off the old layout. Both forms of the old path are removed: the
# socket file left by a previous helper, and the directory Docker invented.
if [[ -e "$POWER_SOCK_LEGACY" ]]; then
  echo "   removing legacy $POWER_SOCK_LEGACY ($(stat -c '%F' "$POWER_SOCK_LEGACY"))"
  echo "   the socket now lives at $POWER_SOCK, inside a mounted directory"
  rm -rf "$POWER_SOCK_LEGACY"
fi

# The helper chowns the socket to this group. Without it the socket is
# root-only and the helper logs a warning that reads like a failure — harmless
# here (the container runs as root) but confusing.
groupadd -f "${UAV_POWER_GROUP:-uav}" 2>/dev/null || true
systemctl start uav-power || echo "WARN: uav-power did not start; the System tab will say so."
for _ in 1 2 3 4 5; do [[ -S "$POWER_SOCK" ]] && break; sleep 1; done
if [[ -S "$POWER_SOCK" ]]; then
  echo "   power socket ready at $POWER_SOCK (mounting $POWER_DIR)"
else
  echo "WARN: $POWER_SOCK is not a socket yet. The container will still be"
  echo "      created and $POWER_DIR is a real directory, so this is now"
  echo "      recoverable WITHOUT recreating the container — fix the helper"
  echo "      (journalctl -u uav-power) and it will appear in there."
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
  # The DIRECTORY, specifically. A container carrying the old socket-file
  # mount matches neither, and is reported below as needing a recreate.
  case "$MOUNTS" in *"$POWER_DIR:$POWER_DIR"*) ;; *) MISSING="$MISSING power-socket" ;; esac
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
    # Name the case they are almost certainly in, because its symptom is not
    # "the button is missing" — it is a container that will not start at all.
    case "$MOUNTS" in *"$POWER_SOCK_LEGACY"*)
      echo
      echo "      THIS CONTAINER HAS THE OLD SOCKET-FILE MOUNT"
      echo "        ($POWER_SOCK_LEGACY, bind-mounted as a file)."
      echo "      It will fail to start with 'not a directory: Are you trying"
      echo "      to mount a directory onto a file'. It CANNOT be repaired in"
      echo "      place — mounts are fixed at CREATE time. Recreate it." ;;
    esac
    echo "      Mounts are fixed at CREATE time, so this cannot be fixed by a"
    echo "      restart. Recreating is YOUR call — 'docker rm' throws away"
    echo "      anything living only inside the container:"
    echo
    echo "        docker rm -f $UAV_CONTAINER"
    echo "        docker run -d --name $UAV_CONTAINER \\"
    echo "          --restart unless-stopped --network host --privileged \\"
    echo "          -v $WS_HOST:/root/robotx_ws \\"
    echo "          -v $POWER_DIR:$POWER_DIR \\"
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
    -v "$POWER_DIR":"$POWER_DIR" \
    -v /dev:/dev \
    "$UAV_IMAGE" tail -f /dev/null >/dev/null
  echo "   created. Build the workspace inside it:"
  echo "     docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
else
  echo "WARN: no image '$UAV_IMAGE' and no container '$UAV_CONTAINER'."
  echo "      Build the image first, then re-run this script:"
  echo "        docker build -t $UAV_IMAGE $UAV_REPO"
fi

echo "== verification =="
# Prerequisites were checked in [3/7]; these are the things that can only be
# judged AFTER the rules and units are in place.
problems=0

if [[ -e /dev/uav-pixhawk ]]; then
  echo "   /dev/uav-pixhawk -> $(readlink -f /dev/uav-pixhawk)"
  # The rule sets GROUP="dialout"; confirm the DEVICE agrees. A rule that
  # matched with a different group leaves the service failing to open the
  # autopilot with status=1 and no mention of permissions anywhere.
  dev_grp="$(stat -c '%G' "$(readlink -f /dev/uav-pixhawk)" 2>/dev/null || echo '?')"
  if [[ "$dev_grp" != "dialout" ]]; then
    echo "WARN: it is group '$dev_grp', not dialout. $UAV_USER cannot open it,"
    echo "      and uav-mavproxy will exit 1 without saying why."
    echo "      Fix GROUP= in tools/udev/99-uav.rules, or add $UAV_USER to it."
    problems=1
  fi
else
  echo "WARN: /dev/uav-pixhawk does not exist yet."
  echo "      Either the Pixhawk is unplugged, or its USB VID/PID is not in"
  echo "      tools/udev/99-uav.rules — that file says its four Pixhawk lines"
  echo "      are a CANDIDATE list, not a confirmed one. To find the real one:"
  echo "        udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct' | head -4"
  problems=1
fi

if ! python3 "$UAV_REPO/tools/scripts/check_config.py" >/dev/null 2>&1; then
  echo "WARN: check_config.py reports problems — the params file can ground the"
  echo "      aircraft, so read them:  python3 tools/scripts/check_config.py"
  problems=1
fi

echo
echo "======================================================================"
if (( problems || missing_prereq )); then
  echo "Host install COMPLETE, with warnings above to clear first."
else
  echo "Host install COMPLETE."
fi
echo "Ports on this vehicle: 14541 -> ROS, 14540 -> GCS (the boat uses 1455x)."
echo
echo "NEXT, in order:"
# Container first, matching step [7]'s own order: an existing container works
# whether or not its image is still around, so advising a rebuild there would
# send someone down a path they do not need.
if docker inspect "$UAV_CONTAINER" >/dev/null 2>&1; then
  echo "  1. docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  echo "  2. systemctl start ${UNITS[*]}"
  echo "  3. open http://<this jetson>:8090"
elif ! docker image inspect "$UAV_IMAGE" >/dev/null 2>&1; then
  echo "  1. docker build -t $UAV_IMAGE $UAV_REPO      # ~10-20 min on an Orin"
  echo "  2. sudo bash setup/install_jetson_host.sh   # re-run: creates the container"
  echo "  3. docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  echo "  4. systemctl start ${UNITS[*]}"
else
  echo "  1. sudo bash setup/install_jetson_host.sh   # re-run: creates the container"
  echo "  2. docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  echo "  3. systemctl start ${UNITS[*]}"
fi
echo
echo "If a unit fails, README.md has a Troubleshooting table keyed by the exact"
echo "status code systemd prints (203/EXEC, 127, 1/FAILURE, ...)."
echo
echo "Everyday loop after a code change:"
echo "  cd $UAV_REPO && git pull"
echo "  docker exec $UAV_CONTAINER bash -lc '/root/robotx_ws/src/rx26_uav/tools/scripts/rebuild.sh'"
echo "  systemctl restart uav-groundstation uav-ocs-client"
echo "A pull WITHOUT a rebuild changes nothing that is running — colcon installs"
echo "into install/, and the node does not import from src/."
echo "======================================================================"
