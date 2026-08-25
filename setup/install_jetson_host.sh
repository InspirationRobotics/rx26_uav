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
#   [5] systemd units         mavproxy -> container -> ground station + OCS
#                             client, plus the shutdown/reboot .path pair
#   [6] power request path    <ws>/logs + the boot-time sweep of stale requests
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
UAV_CONTAINER="${UAV_CONTAINER:-uav_ekko}"
UAV_IMAGE="${UAV_IMAGE:-uav}"

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
# FULL FILENAMES, because the power pair are .path units, not .service ones.
UNITS=(uav-mavproxy.service uav-container.service uav-groundstation.service
       uav-ocs-client.service
       uav-shutdown.path uav-shutdown.service
       uav-reboot.path uav-reboot.service)
# ENABLE IS A DIFFERENT LIST. uav-shutdown.service and uav-reboot.service are
# TRIGGERED by their .path units and carry no [Install] section on purpose:
# enabling them directly would power the Jetson off, or reboot it, every single
# time it finished booting.
ENABLE=(uav-mavproxy uav-container uav-groundstation uav-ocs-client
        uav-shutdown.path uav-reboot.path)
# The long-running ones, for the "start now" hint. `enable` is what arms a .path
# unit; starting the oneshot it triggers would BE the shutdown.
STARTABLE=(uav-mavproxy uav-container uav-groundstation uav-ocs-client)
for unit in "${UNITS[@]}"; do
  sed -e "s|__UAV_USER__|$UAV_USER|g" \
      -e "s|__UAV_REPO__|$UAV_REPO|g" \
      -e "s|__UAV_CONTAINER__|$UAV_CONTAINER|g" \
      -e "s|__UAV_WS__|$WS_HOST|g" \
      "tools/systemd/$unit" > "/etc/systemd/system/$unit"
  chmod 644 "/etc/systemd/system/$unit"
  # Fail loudly rather than enabling a unit that still carries a placeholder.
  if grep -q "__UAV_" "/etc/systemd/system/$unit"; then
    echo "ERROR: $unit still has unsubstituted placeholders." >&2
    grep -n "__UAV_" "/etc/systemd/system/$unit" >&2
    exit 1
  fi
  echo "   installed $unit"
done

# ---------------------------------------------------------------------------
# RETIRE THE SOCKET-BASED POWER HELPER, if this Jetson still has one.
#
# Replaced by the .path units above. Left behind it does nothing worse than
# crash-loop against a socket nothing reads any more — but a failed unit in
# `systemctl status` during a pre-flight check costs someone real time working
# out that it does not matter, which is exactly when nobody has time.
# ---------------------------------------------------------------------------
if [[ -e /etc/systemd/system/uav-power.service ]]; then
  echo "   retiring uav-power.service (its socket helper is gone from the repo)"
  systemctl disable --now uav-power >/dev/null 2>&1 || true
  rm -f /etc/systemd/system/uav-power.service
  rm -rf /run/uav /run/uav-power.sock
fi

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

DEFAULTS
  echo "   seeded /etc/default/uav"
fi

systemctl daemon-reload
systemctl enable "${ENABLE[@]}" >/dev/null
echo "   enabled ${#ENABLE[@]} units; start the long-running ones with:"
echo "     systemctl start ${STARTABLE[*]}"

echo "== [6/7] power request path, then [7/7] container =="
# ---------------------------------------------------------------------------
# THE GROUND STATION ASKS FOR A SHUTDOWN BY DROPPING A FILE.
#
# It runs inside the container, which has no init of its own to ask, so it
# cannot power the Jetson off directly. It writes <ws>/logs/shutdown.request
# into the WORKSPACE BIND MOUNT it already has, and uav-shutdown.path — enabled
# in step [5] — sees it appear and runs a oneshot that deletes the file and
# calls `systemctl poweroff`.
#
# WHAT THIS REPLACED, and why (rx26_uav, 2026-08-24). It used to be a root
# daemon on a Unix socket, bind-mounted into the container. That needed its own
# mount, and a bind mount of a FILE pins an inode: the helper unlinks and
# re-binds on every start, so one `systemctl restart uav-power` left the
# container holding a deleted inode, with a System tab reporting "unreachable"
# and nothing in either journal saying why. And when Docker was asked to mount
# a socket path that did not exist yet it invented a DIRECTORY on the host,
# after which the container would not start AT ALL. Three separate failures,
# all from the same decision to mount a file.
#
# The file-drop design has no socket, no daemon, no extra mount, no group, and
# nothing in /run to survive a reboot. It rides the workspace mount that must
# work anyway for the node's code to be in there — so if the ground station is
# running at all, the channel it needs is already proven. Borrowed from
# robotx_graey_2026 (deploy/systemd/graey-shutdown.path), where it has flown.
# ---------------------------------------------------------------------------
LOGS_DIR="$WS_HOST/logs"
install -d -o "$UAV_USER" -g "$(id -gn "$UAV_USER")" -m 0775 "$LOGS_DIR"
echo "   power requests: $LOGS_DIR/{shutdown,reboot}.request"

# ---------------------------------------------------------------------------
# SWEEP STALE REQUESTS AT BOOT — the one hazard this design does have.
#
# PathExists= fires when the unit starts and the file is ALREADY there, not
# only on creation. So a request file that outlived an interrupted shutdown —
# battery pulled between the write and the unit's `rm` — would power the Jetson
# off again the instant it finished booting. Every time. An aircraft that will
# not stay booted, with nothing on screen to explain it.
#
# systemd-tmpfiles-setup runs in sysinit.target, long before multi-user.target
# arms the .path units, so this always wins the race. The `!` restricts it to
# boot: it must never delete a request an operator just made.
# ---------------------------------------------------------------------------
cat > /etc/tmpfiles.d/uav.conf <<TMPFILES
# Written by setup/install_jetson_host.sh. Removes power requests that outlived
# a shutdown, at BOOT ONLY (the '!'), before uav-*.path can act on them.
r! $LOGS_DIR/shutdown.request
r! $LOGS_DIR/reboot.request
TMPFILES
# Apply now as well, in case one is sitting there from before this ran.
systemd-tmpfiles --remove --boot /etc/tmpfiles.d/uav.conf >/dev/null 2>&1 || true
echo "   installed /etc/tmpfiles.d/uav.conf (boot-time sweep of stale requests)"

for u in uav-shutdown.path uav-reboot.path; do
  systemctl restart "$u" >/dev/null 2>&1 || echo "WARN: $u did not start; the System tab will say power is unavailable."
done
if systemctl is-active --quiet uav-shutdown.path && systemctl is-active --quiet uav-reboot.path; then
  echo "   uav-shutdown.path and uav-reboot.path are watching"
else
  echo "WARN: a power .path unit is not active. The System tab will refuse to"
  echo "      power down and say so, which is the intended failure — a button"
  echo "      that silently does nothing is worse. Check:"
  echo "        systemctl status uav-shutdown.path uav-reboot.path"
fi

# `docker start` CANNOT add mounts — they are fixed when the container is
# CREATED. So this step splits on a distinction that matters:
#   absent  -> create it (nothing exists to lose)
#   present -> CHECK the mounts and print the fix; NEVER `docker rm`
if ! command -v docker >/dev/null; then
  echo "WARN: docker is not installed — uav-container.service will fail."
# `docker container inspect`, NOT `docker inspect`. The bare form matches
# containers AND images AND volumes AND networks alike.
#
# THIS COST THREE BRING-UP RUNS (2026-08-24). The container was then named `uav`
# and so was the image. With no container present, `docker inspect uav` SUCCEEDS
# against the IMAGE, whose .Mounts is empty — so this branch reported
# "MISSING mounts: workspace" and never fell through to the create step. Every
# run printed a plausible recreate recipe for a container that did not exist.
#
# The container is now `uav_ekko` and the image `uav`, so the names no longer
# collide. Keep the explicit `container` anyway: the rename removes THIS
# collision, the typed call removes the whole class of them, and someone will
# eventually set UAV_CONTAINER=uav again.
elif docker container inspect "$UAV_CONTAINER" >/dev/null 2>&1; then
  MOUNTS="$(docker container inspect -f '{{range .Mounts}}{{.Source}}:{{.Destination}} {{end}}' "$UAV_CONTAINER" 2>/dev/null || true)"
  MISSING=""
  case "$MOUNTS" in *":/root/robotx_ws"*) ;; *) MISSING="$MISSING workspace" ;; esac

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
    echo "      Mounts are fixed at CREATE time, so this cannot be fixed by a"
    echo "      restart. Recreating is YOUR call — 'docker rm' throws away"
    echo "      anything living only inside the container:"
    echo
    echo "        docker rm -f $UAV_CONTAINER"
    echo "        docker run -d --name $UAV_CONTAINER \\"
    echo "          --restart unless-stopped --network host --privileged \\"
    echo "          -v $WS_HOST:/root/robotx_ws \\"
    echo "          -v /dev:/dev \\"
    echo "          $UAV_IMAGE tail -f /dev/null"
    echo
    echo "      Then: docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  else
    echo "   mounts OK: the workspace is bind-mounted."
  fi

  # CHECKED SEPARATELY, because it is not a missing mount — it is a mount that
  # actively breaks the container, and the workspace can be perfectly fine
  # alongside it. The symptom is not "the button is missing", it is a container
  # that will not start at all.
  case "$MOUNTS" in *"/run/uav-power.sock"*|*"/run/uav:/run/uav"*)
    echo "WARN: container '$UAV_CONTAINER' still mounts the RETIRED power socket."
    echo "      Nothing serves it any more, and the socket-FILE form makes"
    echo "      'docker start' fail outright with 'not a directory: Are you"
    echo "      trying to mount a directory onto a file'."
    echo "      Mounts are fixed at CREATE time, so recreate it — the power"
    echo "      request now rides the workspace mount and needs no mount of"
    echo "      its own:"
    echo
    echo "        docker rm -f $UAV_CONTAINER"
    echo "        sudo bash setup/install_jetson_host.sh" ;;
  esac
elif docker image inspect "$UAV_IMAGE" >/dev/null 2>&1; then
  # Nothing to lose: create it with the mounts already right. Note there is
  # no power mount — the shutdown request is a file in the workspace.
  echo "   no container named '$UAV_CONTAINER' — creating it"
  docker run -d --name "$UAV_CONTAINER" \
    --restart unless-stopped --network host --privileged \
    -v "$WS_HOST":/root/robotx_ws \
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
if docker container inspect "$UAV_CONTAINER" >/dev/null 2>&1; then
  echo "  1. docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  echo "  2. systemctl start ${STARTABLE[*]}"
  echo "  3. open http://<this jetson>:8090"
elif ! docker image inspect "$UAV_IMAGE" >/dev/null 2>&1; then
  echo "  1. docker build -t $UAV_IMAGE $UAV_REPO      # ~10-20 min on an Orin"
  echo "  2. sudo bash setup/install_jetson_host.sh   # re-run: creates the container"
  echo "  3. docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  echo "  4. systemctl start ${STARTABLE[*]}"
else
  echo "  1. sudo bash setup/install_jetson_host.sh   # re-run: creates the container"
  echo "  2. docker exec $UAV_CONTAINER bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh"
  echo "  3. systemctl start ${STARTABLE[*]}"
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
