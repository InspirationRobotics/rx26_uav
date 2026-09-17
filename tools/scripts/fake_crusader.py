#!/usr/bin/env python3
"""fake_crusader — be Crusader on the radio, from a laptop.

    python3 tools/scripts/fake_crusader.py --port COM5            # RFD900
    python3 tools/scripts/fake_crusader.py --udp udpin:0.0.0.0:14560   # bench

For the park, where there is no boat: a second laptop with an RFD900ux plays
Crusader, so the whole link can be tested for real -- Ekko hears a boat and
stations over the gate that boat needs next, and this prints the buoy map Ekko
sends back, decoded.

It sends ONLY what the boat sends (uav_common/boat_link.py): where it is and one
byte for what it is doing. It receives ONLY what the drone sends: every buoy's
id, position and light.

Typed commands, one per line:
    2                 set what the boat is doing (0-5, see the list it prints)
    m 32.9242 -117.019   move the boat to a position
    n 12              nudge 12 m along the current course (repeat to "drive")
    q                 quit

WHY A FAKE BOAT AND NOT A REAL ONE: the drone's behaviour depends on where the
boat is and what it says it is doing, and neither can be tested by flying alone.
A laptop that lies convincingly is enough to prove the aircraft reacts.
"""
import argparse
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "uav_common"))
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

try:
    from uav_common import boat_link  # noqa: E402
except ImportError:
    # Copied onto a bare laptop with boat_link.py beside it, which is all this
    # tool actually needs: two files and pymavlink, no repo, no ROS.
    sys.path.insert(0, HERE)
    import boat_link  # noqa: E402

M_PER_DEG = 111_139.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="serial port of the RFD900 (COM5, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--udp", help="MAVLink endpoint instead of a serial port")
    ap.add_argument("--sysid", type=int, default=2,
                    help="this boat's MAVLink system id (must differ from Ekko's)")
    ap.add_argument("--target", type=int, default=0,
                    help="who to address: 0 broadcasts to every channel, which is "
                         "what reaches the Jetson through the autopilot")
    ap.add_argument("--at", default="", help="starting LAT,LON")
    ap.add_argument("--heading", type=float, default=90.0,
                    help="course for the 'n' nudge command, degrees")
    ap.add_argument("--hz", type=float, default=1.0)
    args = ap.parse_args()
    if not args.port and not args.udp:
        ap.error("give --port (the radio) or --udp (a bench link)")

    link = mavutil.mavlink_connection(
        args.udp or args.port, baud=args.baud, source_system=args.sysid,
        source_component=190)
    state = {"lat": 0.0, "lon": 0.0, "activity": 1, "target": 0, "run": True}
    if args.at:
        state["lat"], state["lon"] = [float(v) for v in args.at.replace(",", " ").split()]

    def rx():
        """Print the buoy map Ekko sends, decoded, whenever it changes."""
        last = None
        while state["run"]:
            msg = link.recv_match(blocking=True, timeout=1.0)
            if msg is None or msg.get_type() != "TUNNEL":
                continue
            if msg.payload_type != boat_link.PAYLOAD_BUOYS:
                continue
            rep = boat_link.unpack_buoys(boat_link.body(msg))
            buoys = rep["buoys"]
            shown = [(b["id"], b["label"]) for b in buoys] + [rep["confirmed"]]
            if shown == last:
                continue
            last = shown
            print()
            ok = rep["confirmed"]
            print("--- Ekko says (%d buoys, from sysid %d): %s ---"
                  % (len(buoys), msg.get_srcSystem(),
                     "GO: confirmed gate B%d/B%d" % tuple(ok) if len(ok) == 2
                     else "GO: confirmed the exit B%d" % ok[0] if ok
                     else "nothing confirmed -- hold"))
            for b in buoys:
                print("   B%-3d %-15s %.7f, %.7f"
                      % (b["id"], b["label"], b["lat"], b["lon"]))
            print("> ", end="", flush=True)

    def tx():
        while state["run"]:
            # A heartbeat first, always. The autopilot forwards a message to the
            # channel it last HEARD that system on, so a boat that never says
            # hello gets the buoy map sent somewhere else -- or nowhere.
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                    mavutil.mavlink.MAV_STATE_ACTIVE)
            payload = boat_link.pack_boat(state["lat"], state["lon"],
                                          state["activity"], state["target"])
            link.mav.tunnel_send(args.target, 0, boat_link.PAYLOAD_BOAT,
                                 len(payload), boat_link.pad(payload))
            time.sleep(1.0 / args.hz)

    for fn in (rx, tx):
        threading.Thread(target=fn, daemon=True).start()

    print("pretending to be Crusader, sysid %d, %.1f Hz" % (args.sysid, args.hz))
    for i, name in enumerate(boat_link.ACTIVITY):
        print("   %d = %s" % (i, name))
    print("commands: <number> | m LAT LON | n METRES | q")
    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line == "q":
                break
            if line[0] == "m":
                _c, lat, lon = line.replace(",", " ").split()
                state["lat"], state["lon"] = float(lat), float(lon)
            elif line[0] == "n":
                d = float(line.split()[1])
                h = math.radians(args.heading)
                state["lat"] += d * math.cos(h) / M_PER_DEG
                state["lon"] += d * math.sin(h) / (
                    M_PER_DEG * math.cos(math.radians(state["lat"])))
            elif line.isdigit() and int(line) < len(boat_link.ACTIVITY):
                state["activity"] = int(line)
                print("   now: %s" % boat_link.ACTIVITY[state["activity"]])
            else:
                print("   ? commands: <number> | m LAT LON | n METRES | q")
            print("   boat at %.7f, %.7f doing %s"
                  % (state["lat"], state["lon"],
                     boat_link.ACTIVITY[state["activity"]]))
    except (EOFError, KeyboardInterrupt):
        pass
    state["run"] = False


if __name__ == "__main__":
    main()
