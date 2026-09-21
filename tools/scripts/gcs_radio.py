#!/usr/bin/env python3
"""gcs_radio — Ekko's ground station, on THIS laptop, fed by the RFD900ux.

    python tools/scripts/radio_link.py --port COM28      # in one terminal
    python tools/scripts/gcs_radio.py                    # in another
    # then open http://localhost:8090

WHY THIS EXISTS. The page on the aircraft (`ekko.local:8090`) is served BY the
Jetson over WiFi, so its range is WiFi's range — a few tens of metres. The
radio reaches far further, and the aircraft's state has to stay visible at the
range it actually flies. So the page moves to the laptop and takes its numbers
off the radio:

    the radio  -> everything about the AIRCRAFT: position, attitude, altitude,
                  battery, GPS, mode, armed, the fence it holds, and the buoy
                  map + confirmations that already cross the link as TUNNEL
    WiFi       -> only what lives on the Jetson and cannot be squeezed into a
                  7 kbit/s link: the camera image, the node list, the logs, the
                  host's own health. Out of WiFi range those go BLANK and the
                  page says so, rather than showing the last thing they said.

CONTROL IS NOT ON THE RADIO. Every button posts to the Jetson over WiFi, where
the real checks live (the gateway cannot be stopped, power is locked while
armed, and a GET to an action endpoint is refused). Out of WiFi range the
buttons refuse, in words. Nothing here can command the aircraft, and the radio
carries no command this program can originate.

It serves the REAL page (`uav_groundstation.gcs_page`) through the REAL server,
so what an operator learns here is true of the aircraft's own page.
"""
import argparse
import json
import math
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
for pkg in ("uav_common", "uav_groundstation"):
    sys.path.insert(0, os.path.join(REPO, pkg))

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil                                       # noqa: E402

from uav_common import boat_link                                    # noqa: E402
from uav_common import camera_frame                                 # noqa: E402
from uav_common import geo                                          # noqa: E402
from uav_common.stream_cache import StreamCache                     # noqa: E402
from uav_groundstation import battery_core                          # noqa: E402
from uav_groundstation import map_origin, preflight_core            # noqa: E402
from uav_groundstation.gcs_page import render                       # noqa: E402
from uav_groundstation.gcs_server import GcsServer                  # noqa: E402

#: ArduCopter custom_mode -> name. Only the ones this aircraft flies.
MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
         6: "RTL", 9: "LAND", 16: "POSHOLD", 17: "BRAKE", 20: "GUIDED_NOGPS"}
LANDED = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF", 4: "LANDING"}
#: EXACTLY what this page needs, and nothing else: (message id, name, Hz).
#: Asked for per MESSAGE, not per stream group -- a group is a bundle, and
#: MAV2_EXTRA1 at 10 Hz drags AHRS2, SIMSTATE and ESC telemetry onto the radio
#: with ATTITUDE, which measured 23 kbit/s and 6 CRC failures a second on a
#: bench-range link. EXTENDED_SYS_STATE is in NO group and can only be had this
#: way. SET_MESSAGE_INTERVAL takes effect at once, unlike the MAVn_* parameters,
#: which ArduPilot 4.7 latches at boot -- so this is also what makes the page
#: work without rebooting the aircraft.
WANTED = [(30, "ATTITUDE", 8.0),
          (33, "GLOBAL_POSITION_INT", 3.0),
          (74, "VFR_HUD", 2.0),
          (1, "SYS_STATUS", 1.0),
          (24, "GPS_RAW_INT", 1.0),
          (65, "RC_CHANNELS", 1.0),
          (245, "EXTENDED_SYS_STATE", 1.0)]

#: Pre-flight chips the radio cannot answer: they are about the camera and the
#: nodes, which live on the Jetson. Taken from the aircraft's own page while
#: WiFi reaches it.
JETSON_CHIPS = {"gimbal", "stream", "rec", "map"}

#: How long a stream may be silent before the page shows a blank instead.
TIMEOUT_S = 3.0
#: Working altitude of the buoy pass, for the pre-flight fence check.
WORKING_ALT_M = 10.0


class ArmedSince:
    """Armed time, counted from the radio, since this program started.

    NOT uav_groundstation.armed_clock: that one persists against the JETSON's
    boot id so the tile survives a ground_station restart, which a laptop
    cannot honestly claim to know. `resumed` is false for the same reason --
    this is what THIS program has watched, and it says so by never pretending
    to have been there earlier.
    """

    def __init__(self):
        self.total_s = 0.0
        self.flights = 0
        self._since = None
        self._last = None

    def update(self, now, known, armed):
        if not known:
            if self._since is not None:
                self.total_s += now - self._since
                self._since = None
            self._last = None
            return
        if armed and self._since is None:
            self._since = now
            if self._last is False:
                self.flights += 1
        elif not armed and self._since is not None:
            self.total_s += now - self._since
            self._since = None
        self._last = bool(armed)

    def snapshot(self, now):
        secs = self.total_s + ((now - self._since) if self._since else 0.0)
        return {"seconds": round(secs, 1), "flights": self.flights,
                "armed": self._since is not None, "resumed": False}


class Radio:
    """Everything heard on the link, and the page's snapshot built from it."""

    def __init__(self, endpoint, jetson, cam_port, sysid):
        self.sysid = sysid
        self.jetson = jetson
        self.cam_port = cam_port
        self.lock = threading.Lock()
        self.conn = mavutil.mavlink_connection(endpoint, source_system=254,
                                               source_component=190)
        self.pose = StreamCache(TIMEOUT_S)
        self.att = StreamCache(TIMEOUT_S)
        self.hb = StreamCache(TIMEOUT_S)
        self.gps = StreamCache(TIMEOUT_S)
        self.sys_status = StreamCache(TIMEOUT_S)
        self.vfr = StreamCache(TIMEOUT_S)
        self.flight = StreamCache(TIMEOUT_S)
        self.batt = battery_core.BatteryEstimator()
        self.clock = ArmedSince()
        self.rx = boat_link.Receiver()          # POSITIONS + LIGHTS off the air
        self.buoy_seen = 0.0
        self.consumed = 0.0
        self.fence = []                         # [[lat, lon], ...] from the FC
        self.fence_src = map_origin.PARAMS
        self.origin = None
        self.wifi = None                        # the Jetson's own snapshot
        self.wifi_t = 0.0
        self.trail_origin = None
        #: MISSION_* frames, routed OUT of the read loop. Two threads
        #: calling recv_match on one connection race, and the reader wins:
        #: the fence dialog would wait for a MISSION_COUNT that had already
        #: been swallowed. telemetry_bridge solves it the same way.
        self.mission_q = queue.Queue()

    # ---------------------------------------------------------------- reading

    def read_loop(self):
        while True:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1.0)
            except Exception:
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            t = time.monotonic()
            typ = msg.get_type()
            if typ == "BAD_DATA":
                continue
            src = msg.get_srcSystem()
            with self.lock:
                if typ in ("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ACK"):
                    self.mission_q.put(msg)
                    continue
                if typ == "TUNNEL":
                    self.rx.hear(msg.payload_type, boat_link.body(msg))
                    if msg.payload_type in (boat_link.PAYLOAD_POSITIONS,
                                            boat_link.PAYLOAD_LIGHTS):
                        self.buoy_seen = t
                    continue
                if src != self.sysid:
                    continue            # the boat and the other GCS are not us
                # COMPONENT 1, THE AUTOPILOT, AND NOTHING ELSE ON IT. Ekko's
                # system 1 also carries the gimbal, which sends its own
                # HEARTBEAT with custom_mode 0 and no armed bit. Taking those
                # made the page flip between LOITER and STABILIZE, and -- far
                # worse -- made `armed` flicker false several times a second
                # while QGC, which filters properly, sat steady on LOITER.
                if msg.get_srcComponent() != 1:
                    continue
                if typ == "GLOBAL_POSITION_INT":
                    self.pose.set(msg, t)
                elif typ == "ATTITUDE":
                    self.att.set(msg, t)
                elif typ == "HEARTBEAT":
                    self.hb.set(msg, t)
                elif typ == "GPS_RAW_INT":
                    self.gps.set(msg, t)
                elif typ == "SYS_STATUS":
                    self.sys_status.set(msg, t)
                elif typ == "VFR_HUD":
                    self.vfr.set(msg, t)
                elif typ == "EXTENDED_SYS_STATE":
                    self.flight.set(msg, t)
                elif typ == "BATTERY_STATUS" and msg.id == 0:
                    self.consumed = float(msg.current_consumed or 0)

    def request_loop(self):
        """Ask for the message set above, and keep asking.

        Re-sent every 30 s because a request is state in the AUTOPILOT: it is
        lost when it reboots, and a page that silently stops updating after a
        battery swap is worse than one that never worked.
        """
        while True:
            if self.hb.get(time.monotonic()) is not None:
                for msgid, _name, hz in WANTED:
                    try:
                        self.conn.mav.command_long_send(
                            self.sysid, 1,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            float(msgid), 1e6 / hz, 0, 0, 0, 0, 0)
                    except Exception:
                        break
                    time.sleep(0.2)
            time.sleep(30.0)

    def wifi_loop(self):
        """The Jetson's own snapshot, for what the radio cannot carry.

        Short timeout and no retry: out of range this must fail FAST and leave
        the page's radio-fed half updating at full rate.
        """
        url = "http://%s:8090/state" % self.jetson
        while True:
            try:
                raw = urllib.request.urlopen(url, timeout=1.5).read()
                snap = json.loads(raw)
            except Exception:
                snap = None
            with self.lock:
                if snap is not None:
                    self.wifi, self.wifi_t = snap, time.monotonic()
            time.sleep(2.0)

    def fence_loop(self):
        """Read the fence the autopilot HOLDS, over the radio's mission protocol.

        The same thing telemetry_bridge does on the aircraft, for the same
        reason: the map must draw the fence being ENFORCED, not one from a
        parameter file. Re-read every 60 s, because it can be redrawn in QGC.
        """
        while True:
            try:
                poly = self._read_fence()
                if poly:
                    with self.lock:
                        self.fence = poly
                        self.fence_src = map_origin.AUTOPILOT
                        if self.origin is None:
                            self.origin = map_origin.centroid(poly)
            except Exception:
                pass
            time.sleep(60.0)

    def _await(self, want, match, timeout):
        """The next queued MISSION_* frame of `want` that `match` accepts."""
        end = time.time() + timeout
        while time.time() < end:
            try:
                msg = self.mission_q.get(timeout=max(0.1, end - time.time()))
            except queue.Empty:
                return None
            if msg.get_type() == want and match(msg):
                return msg
        return None

    def _read_fence(self):
        m, ftype = self.conn, mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        while not self.mission_q.empty():        # anything left from last time
            self.mission_q.get_nowait()
        m.mav.mission_request_list_send(self.sysid, 1, ftype)
        got = self._await("MISSION_COUNT",
                          lambda x: getattr(x, "mission_type", ftype) == ftype, 8.0)
        if not got or not got.count:
            return []
        pts = []
        for i in range(got.count):
            m.mav.mission_request_int_send(self.sysid, 1, i, ftype)
            item = self._await("MISSION_ITEM_INT", lambda x, i=i: x.seq == i, 5.0)
            if item is None:
                return []                        # a partial fence is not a fence
            pts.append([item.x / 1e7, item.y / 1e7])
        m.mav.mission_ack_send(self.sysid, 1, 0, ftype)
        return pts

    # ---------------------------------------------------------------- the page

    def _xy(self, lat, lon):
        if self.origin is None or not map_origin.is_fix(lat, lon):
            return None
        return geo.latlon_to_xy(lat, lon, self.origin)

    def attitude(self):
        """GET /attitude — the 3D view, at whatever rate the radio delivers."""
        with self.lock:
            now = time.monotonic()
            a = self.att.get(now)
            hb = self.hb.get(now)
            if a is None:
                return {"ok": False}
            return {"ok": True,
                    "roll": math.degrees(a.roll),
                    "pitch": math.degrees(a.pitch),
                    "heading": math.degrees(a.yaw) % 360.0,
                    "age_s": round(self.att.age(now) or 0.0, 3)}

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            pose, att = self.pose.get(now), self.att.get(now)
            hb, gps_m = self.hb.get(now), self.gps.get(now)
            st, vfr = self.sys_status.get(now), self.vfr.get(now)
            fl = self.flight.get(now)
            wifi = self.wifi if (now - self.wifi_t) < 6.0 else None

            armed = bool(hb and hb.base_mode & 128)
            mode = MODES.get(hb.custom_mode, "mode %d" % hb.custom_mode) if hb else ""
            self.clock.update(now, hb is not None, armed)

            lat = pose.lat / 1e7 if pose else None
            lon = pose.lon / 1e7 if pose else None
            placed = self._xy(lat, lon) if pose else None
            if self.origin is None and pose and map_origin.is_fix(lat, lon):
                self.origin = (lat, lon)
                placed = self._xy(lat, lon)

            gps = None
            if gps_m is not None:
                gps = {"fix_type": gps_m.fix_type,
                       "fix_name": {0: "NO GPS", 1: "NO FIX", 2: "2D", 3: "3D",
                                    4: "DGPS", 5: "RTK FLOAT",
                                    6: "RTK FIXED"}.get(gps_m.fix_type, "?"),
                       "satellites": gps_m.satellites_visible,
                       "hdop": (gps_m.eph / 100.0) if gps_m.eph not in (0, 65535) else None,
                       "h_acc_m": (getattr(gps_m, "h_acc", 0) or 0) / 1000.0 or None}
            batt = None
            if st is not None:
                self.batt.feed(now, (st.voltage_battery or 0) / 1000.0,
                               (st.current_battery or 0) / 100.0,
                               self.consumed, st.battery_remaining, armed)
                batt = self.batt.snapshot(now, self._low_volt(wifi), 3.0)

            fence, fence_src = self.fence, self.fence_src
            if not fence and wifi:
                wmap = wifi.get("map") or {}
                if wmap.get("fence"):
                    fence = wmap["fence"]
                    fence_src = wmap.get("fence_src") or map_origin.AUTOPILOT
                    if self.origin is None:
                        self.origin = map_origin.centroid(fence)
                        placed = self._xy(lat, lon) if pose else None
            buoys = self._buoys(now)
            search = (wifi or {}).get("map", {}).get("search") if wifi else None
            ceiling = preflight_core.altitude_ceiling(self._fence_params(wifi))
            alt_rel = (pose.relative_alt / 1000.0) if pose else None

            footprint = None
            if placed and alt_rel and alt_rel > 1.0 and att is not None:
                hdg = math.degrees(att.yaw) % 360.0
                corners = camera_frame.nadir_footprint(alt_rel, hdg, 81.0)
                footprint = [[placed[0] + e, placed[1] + n] for e, n in corners]

            pre = preflight_core.checks({
                "fcu_ok": hb is not None, "pose_ok": pose is not None,
                "armed": armed, "gps": gps, "battery": batt,
                "working_alt_m": WORKING_ALT_M, "search": search,
                "camera": (wifi or {}).get("preflight_camera"),
                "record_gate": ((wifi or {}).get("cam") or {}).get("record_gate"),
                "mapping": self._mapping(wifi),
                **self._fence_params(wifi),
            })

            # The chips about the CAMERA and the NODES cannot be judged from the
            # radio at all. Ours would say "camera_node not running" when the
            # truth is "not visible from here" -- a guess wearing a
            # measurement's clothes. Take the aircraft's own verdict for those
            # while WiFi reaches it, and let them go blank when it does not.
            if wifi and wifi.get("preflight"):
                theirs = {c["key"]: c for c in wifi["preflight"]}
                pre = [theirs.get(c["key"], c) if c["key"] in JETSON_CHIPS else c
                       for c in pre]

            snap = {
                "groups": (wifi or {}).get("groups", []),
                "batt": batt, "gps": gps,
                "armed_time": self.clock.snapshot(now),
                "preflight": pre,
                "tel": {
                    "pose_ok": pose is not None, "att_ok": att is not None,
                    "fcu_ok": hb is not None, "flight_ok": fl is not None,
                    "lat": lat, "lon": lon,
                    "heading": (pose.hdg / 100.0) if pose and pose.hdg != 65535 else None,
                    "speed": vfr.groundspeed if vfr else None,
                    "climb": (-pose.vz / 100.0) if pose else None,
                    "alt_amsl": (pose.alt / 1000.0) if pose else None,
                    "alt_rel": alt_rel,
                    "alt_hae": None,
                    "inside": None,
                    "roll": math.degrees(att.roll) if att else None,
                    "pitch": math.degrees(att.pitch) if att else None,
                    "yaw": (math.degrees(att.yaw) % 360.0) if att else None,
                    "mode": mode, "armed": armed,
                    "landed": LANDED.get(fl.landed_state) if fl else None,
                },
                "ocs": (wifi or {}).get("ocs", {"present": False}),
                "map": {
                    # The fence over the radio needs a REPLY from the autopilot,
                    # and a busy downlink starves the uplink that asks for it
                    # (measured 20 Sep). While WiFi is up, take the fence the
                    # Jetson already read over USB -- it is the same fence, read
                    # the same way, and a drawn fence beats an empty map.
                    "fence": fence, "fence_src": fence_src,
                    "fence_problem": ("" if fence else
                                      "no fence yet: the autopilot has not "
                                      "answered over the radio and the Jetson "
                                      "is not reachable"),
                    "origin_id": 1, "search": search,
                    "veh": ({"x": placed[0], "y": placed[1],
                             "heading": (pose.hdg / 100.0) if pose else 0.0}
                            if placed else None),
                    "inside": None, "trail_gate": 0.5, "trail_max": 600,
                    "buoys": buoys, "mapper": None,
                    "footprint": footprint, "ceiling": ceiling,
                },
                "sys": (wifi or {}).get("sys"),
                "power": {"allowed": False,
                          "reason": ("power is only offered on the aircraft's own "
                                     "page, over WiFi")},
                "cam": self._cam(wifi),
                "link": {"radio": hb is not None, "wifi": wifi is not None},
            }
            return snap

    # -------------------------------------------------------------- the pieces

    def _low_volt(self, wifi):
        b = (wifi or {}).get("batt") or {}
        return b.get("low_volt") or 21.6

    def _fence_params(self, wifi):
        """The fence numbers the checklist needs. From the Jetson when it is
        reachable (it reads them back from the autopilot); otherwise Chris's
        setup, which is what the aircraft has been flown with."""
        ceil = ((wifi or {}).get("map") or {}).get("ceiling") or {}
        return {"fence_enable": 1.0,
                "fence_alt_max": ceil.get("alt_max") or 12.0,
                "fence_margin": 2.0, "fence_type": 5}

    def _mapping(self, wifi):
        names = {n["name"] for g in (wifi or {}).get("groups", [])
                 for n in g.get("nodes", []) if n.get("running")}
        if not wifi:
            return None
        return {"detector": "detector_node" in names,
                "mapper": "buoy_mapper" in names}

    def _buoys(self, now):
        """The buoy field as it arrived over the radio.

        Slots, ids, positions and lights come from boat_link; everything the
        mapper knows and the radio does not (sightings, spread, how long it was
        watched) is left out rather than invented.
        """
        rows = []
        for b in self.rx.buoys():
            xy = self._xy(b["lat"], b["lon"])
            if xy is None:
                continue
            rows.append({"id": b["id"], "x": xy[0], "y": xy[1],
                         "lat": b["lat"], "lon": b["lon"],
                         "label": b["label"], "state": b["label"].split("_")[0],
                         "colour": (b["label"].split("_")[1]
                                    if "_" in b["label"] else ""),
                         "locked": True, "sightings": None, "spread_m": None,
                         "age_s": round(time.monotonic() - self.buoy_seen, 1)})
        if not rows:
            return None
        return {"buoys": rows, "stem": "radio",
                "confirmed": self.rx.confirmed(),
                "over_radio": True}

    def _cam(self, wifi):
        """The camera is the one thing that stays on WiFi: it is a video
        stream and it is never going to fit on a 7 kbit/s radio. `host` points
        the browser straight at the Jetson, because this page is served from
        localhost and the image is not."""
        cam = (wifi or {}).get("cam")
        if not cam:
            return {"source": None, "starting": False,
                    "offline": "the camera is on WiFi, and the Jetson is not "
                               "reachable from here"}
        out = dict(cam)
        out["host"] = self.jetson
        return out

    # ------------------------------------------------------------ the buttons

    def action(self, path, payload):
        """Every button goes to the aircraft's own page, over WiFi.

        The rules that matter are enforced THERE, server-side, and duplicating
        them here would mean two copies that can disagree. Out of WiFi range
        there is no control path at all, which is the correct answer: the radio
        carries the map and the telemetry, never a command.
        """
        url = "http://%s:8090%s" % (self.jetson, path)
        req = urllib.request.Request(url, data=json.dumps(payload or {}).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            return json.load(urllib.request.urlopen(req, timeout=6))
        except (urllib.error.URLError, OSError, ValueError) as e:
            return {"ok": False,
                    "message": "no WiFi path to %s: the radio carries telemetry, "
                               "not controls (%s)" % (self.jetson, e)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--from", dest="endpoint", default="udpin:127.0.0.1:14543",
                    help="where radio_link.py puts the radio (default "
                         "udpin:127.0.0.1:14543)")
    ap.add_argument("--jetson", default="ekko.local",
                    help="the aircraft, for the camera and the buttons, over WiFi")
    ap.add_argument("--port", type=int, default=8090, help="page port")
    ap.add_argument("--sysid", type=int, default=1, help="the autopilot's system id")
    args = ap.parse_args()

    radio = Radio(args.endpoint, args.jetson, args.port + 1, args.sysid)
    for fn in (radio.read_loop, radio.request_loop, radio.wifi_loop,
               radio.fence_loop):
        threading.Thread(target=fn, daemon=True).start()

    server = GcsServer(render(200.0), radio.snapshot, radio.action,
                       radio.attitude).start(args.port, "127.0.0.1")
    print("ground station on the radio: http://localhost:%d" % args.port)
    print("  telemetry, buoys and the fence come off %s" % args.endpoint)
    print("  the camera and the buttons need WiFi to %s" % args.jetson)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
