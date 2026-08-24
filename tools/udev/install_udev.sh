#!/usr/bin/env bash
# Install the UAV udev rules on the Jetson host (NOT inside the container).
#
#     sudo bash tools/udev/install_udev.sh
#
# Also called by setup/install_jetson_host.sh as its step 2.
set -euo pipefail
# Unmatched globs expand to nothing rather than to the literal pattern, so the
# device-scan loops below behave when no serial devices are plugged in.
shopt -s nullglob

RULES_DIR="$(dirname "$0")"

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo." >&2
  exit 1
fi

# List the serial devices present, so an unexpected one is visible at install
# time rather than discovered as a missing symlink later. This is also how you
# CONFIRM the Pixhawk's VID/PID, which 99-uav.rules says is still a guess.
echo "attached serial devices:"
for dev in /dev/ttyACM* /dev/ttyUSB*; do
  [[ -e "$dev" ]] || continue
  # DIAGNOSTIC ONLY — this must never abort the install. grep exits 1 when a
  # device exposes none of these attributes (or when udevadm itself fails), and
  # under `set -e` + `pipefail` that killed the ASV's whole script HERE, before
  # a single rule was installed — silently, since set -e prints nothing.
  info=$(udevadm info -a -n "$dev" 2>/dev/null \
           | grep -m3 -E 'idVendor|idProduct|\{serial\}' \
           | tr -d ' ' | paste -sd' ' - || true)
  echo "  $dev  ${info:-(no usb attributes readable)}"
done

# Purge installed rules that no longer exist in the repo. `install` only ever
# ADDS files, so deleting a rules file from the repo left the old copy live in
# /etc/udev/rules.d forever — and re-running this script looked like it had
# resolved the problem. On the ASV that is how a stale port-chain rule kept
# mapping a symlink onto the Pixhawk's tty long after the repo had dropped it.
# Rules are removed here, not just overwritten, so the collision guard below
# reflects the repo's intent rather than its history.
for installed in /etc/udev/rules.d/99-uav*.rules; do
  name="$(basename "$installed")"
  if [[ ! -e "$RULES_DIR/$name" ]]; then
    echo "removing stale $name (no longer in the repo)"
    rm -f "$installed"
  fi
done

for f in "$RULES_DIR"/99-uav*.rules; do
  echo "installing $(basename "$f")"
  install -m 0644 "$f" "/etc/udev/rules.d/$(basename "$f")"
done

udevadm control --reload-rules
udevadm trigger

echo "Installed. Verify symlinks:"
# NOT `ls -l /dev/uav-*`: with nullglob an unmatched glob expands to NOTHING, so
# ls would get no arguments and cheerfully list the current directory — which
# reads as success. Collect into an array and test it.
uav_links=(/dev/uav-*)
if (( ${#uav_links[@]} )); then
  ls -l "${uav_links[@]}"
else
  echo "  (none yet — plug/replug devices, or the rules match no attached device)"
  echo "  If the Pixhawk IS plugged in, its VID/PID is not in 99-uav.rules yet."
  echo "  Read it off the scan above and add a line; the file says the list is"
  echo "  a candidate set, not a confirmed one."
fi

# --- Collision guard -------------------------------------------------------
# Two rules claiming one device is silent and dangerous. On the ASV a stale
# generated rule once pointed a second name at the SAME tty as the autopilot, so
# a node opening that name would have become a second owner of the Pixhawk
# serial link while MAVProxy held it — on a vehicle with live motors. It was
# caught only by eyeballing an `ls`. The failure mode belongs to udev, not to
# the script that produced it, so the guard stays.
declare -A uav_seen=()
collision=0
for link in "${uav_links[@]}"; do
  target="$(readlink -f "$link")" || continue
  name="$(basename "$link")"
  if [[ -n "${uav_seen[$target]:-}" ]]; then
    echo "ERROR: $name and ${uav_seen[$target]} BOTH resolve to $target" >&2
    case "$name ${uav_seen[$target]}" in
      *uav-pixhawk*)
        echo "       This aliases the AUTOPILOT. Anything opening the other" >&2
        echo "       name becomes a second owner of the Pixhawk serial link," >&2
        echo "       which MAVProxy already holds." >&2 ;;
    esac
    collision=1
  else
    uav_seen[$target]="$name"
  fi
done
if (( collision )); then
  echo >&2
  echo "ERROR: conflicting udev symlinks — refusing to report success." >&2
  echo "       Check for stale rules left by an earlier install:" >&2
  echo "         ls -l /etc/udev/rules.d/99-uav*" >&2
  echo "       Two devices sharing a VID/PID will do this too. The Pixhawk" >&2
  echo "       rules in 99-uav.rules are a candidate list covering several" >&2
  echo "       boards; if two of them match at once, prune it to the board" >&2
  echo "       actually aboard." >&2
  exit 1
fi
