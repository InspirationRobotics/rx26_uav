#!/bin/bash
# MAVProxy is the SOLE owner of the Pixhawk serial link (only ONE process may
# open it). It rebroadcasts MAVLink over UDP to everything else:
#   127.0.0.1:14541      -> telemetry_bridge (this repo's single ROS-side consumer)
#   127.0.0.1:14540      -> spare local consumer (QGC on the Jetson itself)
#   <BCAST_ADDR>:14540   -> Mission Planner / QGC on ANY laptop on the field
#                           WiFi (broadcast, not unicast), so nothing has to be
#                           typed in on the laptop side and no per-laptop IP has
#                           to be passed here.
#   <each GCS_IPS>:14540 -> explicit unicast, for the laptops broadcast does not
#                           reach. See "when broadcast is not enough" below.
#
# ============================================================================
# WHY 1454x AND NOT THE ASV'S 1455x. The boat and this aircraft are on ONE
# subnet. The ASV broadcasts to <subnet>:14550; if this vehicle did the same, a
# GCS bound to 14550 would receive BOTH aircraft interleaved and show one
# vehicle's telemetry under the other's name — a failure that looks like wild
# GPS noise rather than like two vehicles. So:
#
#     the boat lives on 1455x, the aircraft on 1454x
#
# Loopback ports cannot collide across machines, but keeping the whole set
# distinct means an operator reading a --out line or a netstat never has to work
# out which vehicle they are looking at. tools/scripts/check_config.py fails the
# build if mav_endpoint drifts back into the 1455x range.
# ============================================================================
#
# CAVEAT: broadcast is subnet-scoped, not laptop-scoped. If two teams share the
# same field network, every laptop on it sees every broadcasting vehicle — pick
# the right one in QGC's connection list. And if the field network ever hands
# out a different subnet than 192.168.8.0/24, update BCAST_ADDR to match (it
# must be that subnet's broadcast address, i.e. the network address with the
# host bits set to 1 — .255 for a /24).
#
# WHEN BROADCAST IS NOT ENOUGH. A subnet broadcast only reaches hosts ON that
# subnet, and only if nothing between drops it — an AP with client isolation, or
# a laptop on the other side of a bridge, sees nothing while the aircraft looks
# perfectly healthy from the Jetson. GCS_IPS adds an explicit unicast --out per
# address for exactly those:
#
#     GCS_IPS="192.168.8.50 192.168.1.20" ./start_mavproxy.sh
#
# Space-separated. Under systemd put it in /etc/default/uav (the unit's
# EnvironmentFile), quoted, and it reaches this script through the environment
# with no unit edit. Unicast is ADDITIVE — broadcast stays on, so a laptop that
# was already working keeps working whether or not its address is listed.
#
# Uses the stable udev symlink /dev/uav-pixhawk (tools/udev/99-uav.rules)
# instead of a per-airframe /dev/serial/by-id path.
#
# Usage: ./start_mavproxy.sh [BCAST_ADDR]      (env: GCS_IPS, UAV_PIXHAWK_DEV)
set -euo pipefail

BCAST_ADDR="${1:-192.168.8.255}"         # override if the field subnet changes
MASTER="${UAV_PIXHAWK_DEV:-/dev/uav-pixhawk}"

if [ ! -e "$MASTER" ]; then
  echo "ERROR: Pixhawk device $MASTER not found." >&2
  echo "  Check the udev symlink (sudo bash tools/udev/install_udev.sh) or set" >&2
  echo "  UAV_PIXHAWK_DEV. To see what actually enumerated:" >&2
  echo "    ls -l /dev/serial/by-id/" >&2
  echo "    udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct'" >&2
  exit 1
fi

# Built as an array so each --out is one argv element. 14541 stays FIRST because
# it is the one output the aircraft cannot fly without: telemetry_bridge is the
# sole ROS-side consumer, and everything downstream of it — the OCS heartbeat,
# the ground station, the geofence uploader — goes dark if it is missing.
OUTS=(--out=udp:127.0.0.1:14541
      --out=udp:127.0.0.1:14540
      --out=udpbcast:"${BCAST_ADDR}":14540)

# UNQUOTED on purpose: GCS_IPS is a space-separated list and this relies on word
# splitting to turn it into one --out per address. Quoting "${GCS_IPS}" would
# produce a single bogus --out containing spaces, and MAVProxy would fail to
# parse the address rather than obviously ignoring it. The :- keeps `set -u`
# happy when the variable is unset, which is the normal case.
for ip in ${GCS_IPS:-}; do
  OUTS+=(--out=udp:"${ip}":14540)
done

# --daemon is REQUIRED under systemd, not a preference. MAVProxy runs an
# interactive console by default; with stdin on /dev/null it prints the "MAV> "
# prompt, immediately reads EOF, treats that as "quit", and unloads every module
# and exits 1. systemd restarts it, and you get a clean-looking crash loop whose
# log ends in an orderly shutdown rather than an error. Run it by hand (a TTY)
# and you get the console; drop --daemon here and the service loops forever.
#
# --streamrate=-1 means "request NOTHING; leave the vehicle's own SRx_* rates
# alone". It is not a tuning choice — without it MAVProxy sends
# REQUEST_DATA_STREAM(MAV_DATA_STREAM_ALL, 4Hz) on every connect and after every
# reconnect, which overwrites SR0_* in the autopilot's RAM. A rate set in QGC is
# saved to the Pixhawk's EEPROM, looks correct in QGC forever, and is silently
# stomped back to 4 Hz the moment this service restarts.
#
# So per-message rates are set ONCE, in QGC, on SR0_* (USB = SERIAL0, the port
# this --master opens). The three this stack needs:
#   SR0_POSITION  >0   GLOBAL_POSITION_INT -> /uav/pose (lat/lon/alt/climb)
#   SR0_EXTRA1    30   ATTITUDE            -> /uav/attitude
#   SR0_EXT_STAT  >0   EXTENDED_SYS_STATE  -> /uav/flight_state, which is the
#                      AUTHORITATIVE source of flight_phase. Leave it at 0 and
#                      ocs_client silently falls back to an armed+altitude
#                      guess — it says so loudly, but the fix is here.
#
# If SR0_* is ever left at 0 the vehicle streams nothing and telemetry_bridge
# sits at "still no heartbeat" — check the params before suspecting the link.
exec mavproxy.py \
  --master="$MASTER" \
  --daemon \
  --streamrate=-1 \
  "${OUTS[@]}"
