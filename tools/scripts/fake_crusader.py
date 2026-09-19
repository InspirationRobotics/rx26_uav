#!/usr/bin/env python3
"""fake_crusader -- be Crusader on the RFD900ux mesh, from a laptop.

    python fake_crusader.py --port COM5                  # the team laptop's radio
    python fake_crusader.py --udp udpin:0.0.0.0:14556    # the simulator's second radio

For testing Task 1 Disruptive with no boat in the water: three RFD900ux on one
mesh -- Chris's laptop (the master, QGC), Ekko, and a team laptop running this.
It needs exactly two files, this one and uav_common/boat_link.py beside it, plus
`pip install pymavlink`. No repo, no ROS.

WHAT IT HEARS, decoded and printed as it changes:
  Ekko's buoy positions (sent once each), every light by buoy, and Ekko's
  CONFIRMATION -- the gate or exit Crusader may drive to right now -- plus who
  is on the mesh (heartbeats) and the radio's own link report (RADIO_STATUS).
  EVERYTHING decoded goes to a log file, one JSON object per line.

WHAT IT SENDS, once a second, exactly what the real boat will: a heartbeat, and
its position, one byte for what it is doing, the buoy it is heading for, and
the positions it holds -- the acknowledgement that stops Ekko re-sending them.

Commands (type, then Enter):
    h          holding
    e          circling the entry buoy
    g          heading for the gate Ekko CONFIRMS (refused if none is confirmed)
    x          heading for the exit buoy
    ce         circling the exit buoy
    d          done
    t 7        heading for buoy B7 (sets the target, not the activity)
    m 32.9242 -117.019   move the boat
    n 10       nudge 10 m along --heading
    s          show everything known now
    q          quit
"""
import argparse
import json
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
    # Copied onto a bare laptop with boat_link.py beside it.
    sys.path.insert(0, HERE)
    import boat_link  # noqa: E402

M_PER_DEG = 111_139.0
HOLDING, CIRCLING_ENTRY, TRANSIT, CIRCLING_EXIT, DONE = 1, 2, 3, 4, 5


class Crusader:
    """The fake boat's state, what it has heard, and the log."""

    def __init__(self, log_path, heading_deg):
        self.rx = boat_link.Receiver()
        self.lock = threading.Lock()
        self.lat = self.lon = 0.0
        self.activity, self.target = HOLDING, 0
        self.heading = heading_deg
        self.heard = {}             # sysid -> (type name, last time)
        self.radio = None           # last RADIO_STATUS
        self.last_lights = {}
        self.last_confirmed = None
        self.log = open(log_path, "a", encoding="utf-8")
        self.log_path = log_path
        self.run = True

    def write(self, kind, **data):
        data.update(t=round(time.time(), 3), kind=kind)
        self.log.write(json.dumps(data) + "\n")
        self.log.flush()

    # ---- hearing
    def hear(self, msg):
        typ = msg.get_type()
        src = msg.get_srcSystem()
        if typ == "HEARTBEAT":
            name = mavutil.mavlink.enums["MAV_TYPE"].get(msg.type)
            name = name.name.replace("MAV_TYPE_", "") if name else str(msg.type)
            new = src not in self.heard
            self.heard[src] = (name, time.time())
            if new:
                say("on the mesh: sysid %d (%s)" % (src, name))
                self.write("heartbeat", sysid=src, vehicle=name)
            return
        if typ == "RADIO_STATUS":
            self.radio = dict(rssi=msg.rssi, remrssi=msg.remrssi, noise=msg.noise,
                              remnoise=msg.remnoise, txbuf=msg.txbuf,
                              errors=msg.rxerrors, fixed=msg.fixed)
            self.write("radio_status", **self.radio)
            return
        if typ != "TUNNEL":
            return
        with self.lock:
            kind = self.rx.hear(msg.payload_type, boat_link.body(msg))
            if kind == "positions":
                recs = boat_link.unpack_positions(boat_link.body(msg))
                say("EKKO positions: %s" % ", ".join(
                    "B%d (%.7f, %.7f)" % (r["id"], r["lat"], r["lon"]) for r in recs))
                self.write("positions", sysid=src, buoys=recs)
            elif kind == "lights":
                lights = {b["id"]: b["label"] for b in self.rx.buoys()}
                changed = {k: v for k, v in lights.items() if self.last_lights.get(k) != v}
                if changed:
                    say("EKKO lights: %s" % ", ".join(
                        "B%d %s" % (k, v) for k, v in sorted(changed.items())))
                self.last_lights = lights
                conf = self.rx.confirmed()
                if conf != self.last_confirmed:
                    say(">>> EKKO CONFIRMS: %s" % describe(conf, lights))
                    self.last_confirmed = conf
                self.write("lights", sysid=src, lights=lights, confirmed=conf,
                           unplaced_slots=sorted(set(self.rx.lights) - set(self.rx.positions)))

    # ---- sending
    def report(self, link, target_sysid):
        with self.lock:
            payload = boat_link.pack_boat(self.lat, self.lon, self.activity,
                                          self.rx.slot_of(self.target), self.rx.acked())
            acked = sorted(self.rx.acked())
        link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                mavutil.mavlink.MAV_STATE_ACTIVE)
        link.mav.tunnel_send(target_sysid, 0, boat_link.PAYLOAD_BOAT,
                             len(payload), boat_link.pad(payload))
        self.write("sent_boat", lat=self.lat, lon=self.lon, activity=self.activity,
                   target=self.target, acked_slots=acked)

    # ---- commands
    def command(self, line):
        parts = line.split()
        c = parts[0].lower()
        with self.lock:
            lights = {b["id"]: b["label"] for b in self.rx.buoys()}
            conf = self.rx.confirmed()
        if c == "h":
            self.activity, self.target = HOLDING, 0
        elif c == "e":
            entry = [k for k, v in lights.items() if v == "FLASHING_BLUE"]
            self.activity, self.target = CIRCLING_ENTRY, (entry[0] if entry else 0)
        elif c == "g":
            if len(conf) != 2:
                say("   no gate confirmed by Ekko right now -- not moving")
                return
            self.activity, self.target = TRANSIT, conf[0]
        elif c == "x":
            exits = [k for k, v in lights.items() if v == "SOLID_BLUE"]
            if not exits:
                say("   no exit buoy (SOLID_BLUE) heard yet")
                return
            self.activity, self.target = TRANSIT, exits[0]
        elif c == "ce":
            self.activity = CIRCLING_EXIT
        elif c == "d":
            self.activity, self.target = DONE, 0
        elif c == "t" and len(parts) == 2:
            self.target = int(parts[1].lstrip("Bb"))
        elif c == "m" and len(parts) == 3:
            self.lat, self.lon = float(parts[1]), float(parts[2])
        elif c == "n" and len(parts) == 2:
            d, h = float(parts[1]), math.radians(self.heading)
            self.lat += d * math.cos(h) / M_PER_DEG
            self.lon += d * math.sin(h) / (M_PER_DEG * math.cos(math.radians(self.lat)))
        elif c == "s":
            self.show()
            return
        else:
            say("   ? h e g x ce d | t N | m LAT LON | n METRES | s | q")
            return
        self.write("command", line=line, activity=self.activity, target=self.target)
        say("   now: %s%s at %.7f, %.7f" % (
            boat_link.ACTIVITY[self.activity],
            " -> B%d" % self.target if self.target else "", self.lat, self.lon))

    def show(self):
        with self.lock:
            buoys, conf = self.rx.buoys(), self.rx.confirmed()
        say("--- heard on the mesh ---")
        for sysid, (name, t) in sorted(self.heard.items()):
            say("  sysid %-3d %-14s %.0f s ago" % (sysid, name, time.time() - t))
        if self.radio:
            say("  radio: rssi %(rssi)d remote %(remrssi)d noise %(noise)d txbuf %(txbuf)d "
                "errors %(errors)d" % self.radio)
        say("  %d buoy positions held; Ekko confirms: %s" % (len(buoys), describe(conf, {b["id"]: b["label"] for b in buoys})))
        for b in buoys:
            say("  B%-3d %-15s %.7f, %.7f" % (b["id"], b["label"], b["lat"], b["lon"]))
        say("  boat: %s%s at %.7f, %.7f   (log: %s)" % (
            boat_link.ACTIVITY[self.activity],
            " -> B%d" % self.target if self.target else "", self.lat, self.lon, self.log_path))


def describe(conf, lights):
    if len(conf) == 2:
        return "GO -- gate B%d/B%d" % tuple(conf)
    if len(conf) == 1:
        return "GO -- the exit B%d" % conf[0]
    return "nothing -- HOLD"


_print_lock = threading.Lock()


def say(text):
    with _print_lock:
        print("\r" + text)
        print("> ", end="", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="the RFD900ux's serial port (COM5, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--udp", help="a MAVLink endpoint instead of a serial port")
    ap.add_argument("--sysid", type=int, default=42,
                    help="this boat's system id: Ekko reads boat reports from "
                         "42, the real Crusader's rxl_link_node")
    ap.add_argument("--target", type=int, default=0,
                    help="who reports are addressed to: 0 = everyone on the mesh")
    ap.add_argument("--at", default="", help="starting LAT,LON")
    ap.add_argument("--heading", type=float, default=90.0, help="course for 'n', degrees")
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument("--log", default="fake_crusader_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
    args = ap.parse_args()
    if not args.port and not args.udp:
        ap.error("give --port (the radio) or --udp (a simulator link)")

    link = mavutil.mavlink_connection(args.udp or args.port, baud=args.baud,
                                      source_system=args.sysid, source_component=190)
    boat = Crusader(args.log, args.heading)
    if args.at:
        boat.lat, boat.lon = [float(v) for v in args.at.replace(",", " ").split()]
    boat.write("start", sysid=args.sysid, link=args.udp or args.port)

    def rx():
        while boat.run:
            try:
                msg = link.recv_match(blocking=True, timeout=1.0)
            except Exception as exc:  # noqa: BLE001
                say("   link error: %s" % exc)
                time.sleep(1.0)
                continue
            if msg is not None and msg.get_srcSystem() != args.sysid:
                try:
                    boat.hear(msg)
                except Exception as exc:  # noqa: BLE001
                    say("   could not decode a %s: %s" % (msg.get_type(), exc))

    def tx():
        while boat.run:
            try:
                boat.report(link, args.target)
            except Exception as exc:  # noqa: BLE001
                say("   send error: %s" % exc)
            time.sleep(1.0 / args.hz)

    for fn in (rx, tx):
        threading.Thread(target=fn, daemon=True).start()

    print("pretending to be Crusader, sysid %d, %.1f Hz, logging to %s"
          % (args.sysid, args.hz, args.log))
    print("commands: h e g x ce d | t N | m LAT LON | n METRES | s | q")
    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line.lower() == "q":
                break
            boat.command(line)
    except (EOFError, KeyboardInterrupt):
        pass
    boat.run = False
    boat.write("stop")


if __name__ == "__main__":
    main()
