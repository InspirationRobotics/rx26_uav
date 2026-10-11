#!/usr/bin/env python3
"""bench_rxl_uplink -- Ekko's reports to the OCS over the radio, with no radio.

    python3 tools/bench/bench_rxl_uplink.py

No ROS, no MAVLink, no aircraft. What it covers, and why:

  copy        uav_common/rxl.py is still the OCS's rxl_codec.py, line for line, and
              the payload types cannot collide with boat_link's.
  task        /uav/current_task parses the way the boat's does; an unrecognised value
              is ignored, not turned into a stand-down; heartbeat_core carries it.
  heartbeat   24 bytes; the flag bits; every case where a valid flag must clear or
              nothing may be sent; the 2 Hz cadence at a 20 Hz tick.
  passage     21+9n bytes; which buoy is the entry and which the exit; what is kept
              when there are more than ten; sent only while the task is
              TASK_SAFE_PASSAGE; the debounce and the refresh; nothing for an empty
              or stale map.
  budget      the added load stays under ~120 B/s.
  ocs         our bytes decoded, and re-encoded, by the OCS's OWN codec -- loaded by
              file path, only here, and skipped when the OCS checkout is absent.
  bridge      telemetry_bridge's glue (ROS stubbed): the buoy-map callback, the shielded
              uplink tick, the TUNNEL send, that every YAML key has a PARAM_SPEC, and that
              ocs_client's WiFi heartbeat carries the same task as the radio's.
"""
import importlib.util
import math
import os
import sys
import time
import types
from unittest.mock import MagicMock

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for pkg in ("uav_common", "uav_groundstation", "uav_fcu"):
    sys.path.insert(0, os.path.join(REPO, pkg))

from uav_common import boat_link, rxl, rxl_uplink, task_core  # noqa: E402
from uav_groundstation import heartbeat_core  # noqa: E402

OCS_CODEC = os.path.normpath(os.path.join(REPO, "..", "..", "OCS", "fleet", "rxl_codec.py"))

GEOID = -24.6
T0 = 1000.0


def check(name, passed, detail=""):
    print("  %-62s %-4s %s" % (name, "PASS" if passed else "FAIL", str(detail)[:60]))
    return bool(passed)


def skip(name, why):
    print("  %-62s SKIP %s" % (name, why))
    return True


# ------------------------------------------------------------------ inputs, as the bridge holds them

def pose(lat=1.2806, lon=103.8557, hdg=90.0, spd=6.5, amsl=42.0, rel=20.0):
    return (lat, lon, hdg, spd, amsl, rel, 0.0)


def status(mode="LOITER", armed=True):
    return (mode, armed, 4)


def att(roll=0.05, pitch=-0.02):
    return (roll, pitch, 1.0, 0.0, 0.0, 0.0)


class Log:
    def __init__(self):
        self.lines = []

    def __call__(self, level, text):
        self.lines.append((level, text))

    def has(self, *words):
        return any(all(w in text for w in words) for _, text in self.lines)


def uplink(hz=2.0, log=None):
    return rxl_uplink.RxlUplink(
        heartbeat_hz=hz, geoid_separation_m=GEOID, airborne_alt_m=1.0,
        build_heartbeat=heartbeat_core.build_heartbeat, log=log)


def tick(up, t, task="TASK_NONE", **kw):
    kw.setdefault("pose", pose())
    kw.setdefault("status", status())
    kw.setdefault("attitude", att())
    kw.setdefault("landed", heartbeat_core.LANDED_IN_AIR)
    kw.setdefault("buoy_map", None)
    return up.tick(t, task=task, **kw)


def only(frames, ptype):
    return [p for t, p in frames if t == ptype]


def run(up, seconds, task, start=T0, hz=20.0, **kw):
    """Tick at 20 Hz for `seconds`; every (time, payload_type, payload) sent."""
    sent, n = [], int(seconds * hz)
    for i in range(n + 1):
        t = start + i / hz
        kw_i = {k: (v(t) if callable(v) else v) for k, v in kw.items()}
        for ptype, payload in tick(up, t, task, **kw_i):
            sent.append((t, ptype, payload))
    return sent


def buoy(bid, label, lat=1.2800, lon=103.8550):
    return (bid, lat + bid * 1e-5, lon + bid * 1e-5, label)


def gate_map():
    return [buoy(1, "FLASHING_BLUE"), buoy(2, "FLASHING_RED"), buoy(3, "FLASHING_GREEN"),
            buoy(4, "OFF"), buoy(5, "SOLID_BLUE")]


def fresh(buoys, stamp_ms=123456):
    return (buoys, stamp_ms)


# --------------------------------------------------------------------------- the cases

def case_copy():
    r = []
    ours = os.path.join(REPO, "uav_common", "uav_common", "rxl.py")
    if os.path.isfile(OCS_CODEC):
        a = open(OCS_CODEC, encoding="utf-8", newline="").read()
        b = open(ours, encoding="utf-8", newline="").read()
        r.append(check("rxl.py == OCS rxl_codec.py after its header comment",
                       b.endswith(a) and b[:len(b) - len(a)].lstrip().startswith("#"),
                       "differs" if not b.endswith(a) else ""))
    else:
        r.append(skip("rxl.py == OCS rxl_codec.py after its header comment", "no OCS checkout"))
    r.append(check("HEARTBEAT is 0x8010 and SAFE_PASSAGE 0x8011",
                   (rxl_uplink.PAYLOAD_HEARTBEAT, rxl_uplink.PAYLOAD_SAFE_PASSAGE)
                   == (0x8010, 0x8011)))
    taken = {boat_link.PAYLOAD_BOAT, boat_link.PAYLOAD_POSITIONS, boat_link.PAYLOAD_LIGHTS,
             boat_link.PAYLOAD_GNSS, boat_link.PAYLOAD_TEST, *boat_link.RETIRED}
    r.append(check("neither payload type collides with boat_link's",
                   not taken & {rxl_uplink.PAYLOAD_HEARTBEAT, rxl_uplink.PAYLOAD_SAFE_PASSAGE}))
    r.append(check("task tokens == the codec's RxTask names minus TASK_UNKNOWN",
                   set(task_core.RX_TASKS) == set(rxl.RX_TASK.values()) - {"TASK_UNKNOWN"}))
    r.append(check("every beacon label maps to the right BeaconState name",
                   all(rxl.BEACON_STATE[code] == "BEACON_STATE_" + ("STEADY_BLUE" if lab == "SOLID_BLUE" else lab)
                       for lab, code in rxl_uplink.BEACON_BY_LABEL.items())))
    r.append(check("the mapper's own labels are the ones boat_link knows",
                   set(rxl_uplink.BEACON_BY_LABEL) | {"UNKNOWN"} == set(boat_link.STATES)))
    return r


def case_task():
    r = []
    for tok in sorted(task_core.RX_TASKS):
        r.append(check("%s is accepted" % tok, task_core.coerce_task(tok) == (tok, None)))
    r.append(check("case and whitespace are tolerated",
                   task_core.coerce_task("  task_safe_passage ")[0] == "TASK_SAFE_PASSAGE"))
    for bad in ("TASK_UNKNOWN", "TASK_SAFEPASSAGE", "SAFE_PASSAGE", "", None, 7):
        r.append(check("%r is refused" % (bad,), task_core.coerce_task(bad)[0] is None))
    t = task_core.CurrentTask()
    r.append(check("nothing published yet: TASK_NONE", t.value == "TASK_NONE"))
    r.append(check("a new task is a change", t.update("TASK_SAFE_PASSAGE") == (True, None)
                   and t.value == "TASK_SAFE_PASSAGE"))
    r.append(check("the same task again is not", t.update("TASK_SAFE_PASSAGE") == (False, None)))
    changed, err = t.update("TASK_SAFEPASSAGE")
    r.append(check("a typo is IGNORED (the task stays) and explained once",
                   not changed and err and t.value == "TASK_SAFE_PASSAGE", err))
    r.append(check("...the same typo again is silent", t.update("TASK_SAFEPASSAGE") == (False, None)))
    r.append(check("TASK_NONE stands down", t.update("TASK_NONE") == (True, None)
                   and t.value == "TASK_NONE"))
    return r


def case_heartbeat():
    r = []
    up = uplink()
    frames = tick(up, T0)
    r.append(check("one heartbeat, type 0x8010, 24 bytes",
                   len(frames) == 1 and frames[0][0] == 0x8010 and len(frames[0][1]) == 24,
                   [(hex(t), len(p)) for t, p in frames]))
    p = frames[0][1]
    m = rxl.decode(0x8010, p)[1]
    r.append(check("node 3, AUTO, AIRBORNE, TASK_NONE, phase 0xFF",
                   (m["vehicle"], m["robot_state"], m["flight_phase"], m["task"], m["phase"])
                   == (3, rxl.STATE_AUTO, rxl.PHASE_AIRBORNE, rxl.TASK_NONE, 0xFF), m))
    r.append(check("flags byte: state 3 | phase 2<<2 | pos/att/alt valid = 0x7B", p[1] == 0x7B, hex(p[1])))
    r.append(check("position, speed, heading round-trip",
                   abs(m["lat"] - 1.2806) < 1e-7 and abs(m["lon"] - 103.8557) < 1e-7
                   and m["spd_mps"] == 6.5 and m["hdg_deg"] == 90.0, m))
    r.append(check("roll/pitch are DEGREES of the autopilot's radians",
                   abs(m["roll_deg"] - math.degrees(0.05)) < 0.01
                   and abs(m["pitch_deg"] - math.degrees(-0.02)) < 0.01, m))
    r.append(check("altitude is HAE = AMSL + geoid (42.0 - 24.6 = 17.4 m)", m["alt_m"] == 17.4, m["alt_m"]))
    r.append(check("dead_reckoned is never claimed", m["dead_reckoned"] is False))

    body = heartbeat_core.build_heartbeat(
        pose=rxl_uplink.Pose(*pose()), status=rxl_uplink.Status(*status()),
        attitude=rxl_uplink.Attitude(*att()), landed=2, geoid_separation_m=GEOID,
        airborne_alt_m=1.0)[0]
    r.append(check("the same body as the WiFi heartbeat, field for field",
                   abs(m["alt_m"] - body["altitude_hae_m"]) < 0.01 and m["spd_mps"] == body["spd_mps"]
                   and rxl.ROBOT_STATE[m["robot_state"]] == body["state"]
                   and rxl.FLIGHT_PHASE[m["flight_phase"]] == body["flight_phase"]
                   and rxl.RX_TASK[m["task"]] == body["current_task"]))

    # flags: clear a flag, never invent a value
    m = rxl.decode(0x8010, tick(uplink(), T0, pose=pose(hdg=float("nan")))[0][1])[1]
    r.append(check("GPS yaw unresolved: heading is the 0xFFFF marker, flags stay set",
                   m["hdg_deg"] is None and m["pos_valid"] and m["att_valid"] and m["alt_valid"], m["hdg_deg"]))
    m = rxl.decode(0x8010, tick(uplink(), T0, attitude=None)[0][1])[1]
    r.append(check("attitude stale: att_valid clear, roll/pitch blank",
                   not m["att_valid"] and m["roll_deg"] is None and m["pitch_deg"] is None
                   and m["pos_valid"]))
    m = rxl.decode(0x8010, tick(uplink(), T0, pose=pose(lat=0.0, lon=0.0))[0][1])[1]
    r.append(check("position exactly 0,0 (no EKF origin): pos_valid clear",
                   not m["pos_valid"] and m["lat"] is None and m["lon"] is None))
    log = Log()
    tick(uplink(log=log), T0, attitude=None)
    r.append(check("...and the log says what will not be relayed", log.has("attitude", "not relay"), log.lines))
    m = rxl.decode(0x8010, tick(uplink(), T0, task="TASK_SAFE_PASSAGE")[0][1])[1]
    r.append(check("task rides along: TASK_SAFE_PASSAGE is 2", m["task"] == 2))
    m = rxl.decode(0x8010, tick(uplink(), T0, status=status("STABILIZE", True))[0][1])[1]
    r.append(check("armed in a manual mode: STATE_MANUAL", m["robot_state"] == rxl.STATE_MANUAL))
    m = rxl.decode(0x8010, tick(uplink(), T0, landed=heartbeat_core.LANDED_ON_GROUND)[0][1])[1]
    r.append(check("on the ground: GROUNDED", m["flight_phase"] == rxl.PHASE_GROUNDED))

    # send nothing
    log = Log()
    r.append(check("pose stale: nothing", tick(uplink(log=log), T0, pose=None) == []))
    r.append(check("...and the log says why", log.has("not sent", "pose is stale"), log.lines))
    r.append(check("fcu_status stale: nothing", tick(uplink(), T0, status=None) == []))
    r.append(check("no flight phase and no pose: nothing", tick(uplink(), T0, pose=None, landed=None) == []))
    r.append(check("a landed_state the table lacks: nothing", tick(uplink(), T0, landed=99) == []))
    r.append(check("ocs_rxl_heartbeat_hz = 0: nothing, whatever the task",
                   run(uplink(hz=0.0), 3, "TASK_NONE") == []))
    log = Log()
    tick(uplink(log=log), T0, pose=pose(spd=float("nan")))
    r.append(check("a speed that is not a number: nothing, and it is logged",
                   tick(uplink(), T0, pose=pose(spd=float("nan"))) == [] and log.has("not sent"), log.lines))
    r.append(check("a task token that is not an RxTask: nothing",
                   tick(uplink(), T0, task="TASK_UNKNOWN") == []))

    # cadence
    sent = run(uplink(), 10.0, "TASK_NONE")
    n = len(only([(pt, pl) for _, pt, pl in sent], 0x8010))
    r.append(check("2 Hz at a 20 Hz tick: 20 heartbeats in 10 s (21 counting t=0)", n in (20, 21), n))
    n4 = len(run(uplink(hz=4.0), 10.0, "TASK_NONE"))
    r.append(check("4 Hz is the ceiling and works", n4 in (40, 41), n4))
    try:
        uplink(hz=4.5)
        r.append(check("above 4 Hz is refused", False))
    except ValueError:
        r.append(check("above 4 Hz is refused", True))
    return r


def case_selection():
    r = []
    p = rxl_uplink.select_passage(gate_map())
    r.append(check("entry = the FLASHING_BLUE buoy, exit = the SOLID_BLUE buoy",
                   p.entry == gate_map()[0][1:3] and p.exit == gate_map()[4][1:3], (p.entry, p.exit)))
    r.append(check("all five reported, by id, with RXL states",
                   [(b[0], b[3]) for b in p.buoys] == [(1, 4), (2, 2), (3, 3), (4, 1), (5, 5)], p.buoys))
    p = rxl_uplink.select_passage([buoy(1, "UNKNOWN"), buoy(2, "FLASHING_RED")])
    r.append(check("UNKNOWN buoys are left out, and that is not a 'drop'",
                   [b[0] for b in p.buoys] == [2] and not p.dropped))
    p = rxl_uplink.select_passage([buoy(1, "FLASHING_RED")])
    r.append(check("no blue buoys: entry and exit unknown", p.entry is None and p.exit is None))
    p = rxl_uplink.select_passage([buoy(1, "SOLID_RED"), buoy(2, "OFF")])
    r.append(check("a label with no RXL state is dropped and named, not relabelled",
                   [b[0] for b in p.buoys] == [2] and p.dropped and p.dropped[0][:2] == (1, "SOLID_RED"),
                   p.dropped))
    p = rxl_uplink.select_passage([buoy(1, "FLASHING_BLUE"), buoy(2, "FLASHING_BLUE"), buoy(3, "SOLID_BLUE")])
    r.append(check("two entry candidates: entry left unknown, with a note; exit unaffected",
                   p.entry is None and p.exit is not None and p.notes and len(p.buoys) == 3, p.notes))
    p = rxl_uplink.select_passage([buoy(1, "FLASHING_RED"), (2, float("nan"), 103.0, "OFF")])
    r.append(check("a buoy without a usable position is dropped", [b[0] for b in p.buoys] == [1]
                   and p.dropped[0][0] == 2))

    many = ([buoy(i, "OFF") for i in range(1, 7)]          # 6 obstacles, ids 1-6
            + [buoy(7, "FLASHING_RED"), buoy(8, "FLASHING_GREEN"),
               buoy(9, "FLASHING_RED"), buoy(10, "FLASHING_GREEN"),
               buoy(11, "FLASHING_BLUE"), buoy(12, "SOLID_BLUE")])
    p = rxl_uplink.select_passage(many)
    kept = [b[0] for b in p.buoys]
    r.append(check("12 mapped: ten kept, entry/exit/red/green first, then OFF by id",
                   kept == [1, 2, 3, 4, 7, 8, 9, 10, 11, 12], kept))
    r.append(check("...the two highest-id obstacles are the ones dropped, named in `dropped`",
                   sorted(d[0] for d in p.dropped) == [5, 6] and all("limit" in d[2] for d in p.dropped),
                   p.dropped))
    r.append(check("...and entry/exit are still known", p.entry is not None and p.exit is not None))
    up_log = Log()
    up = uplink(log=up_log)
    tick(up, T0, task="TASK_SAFE_PASSAGE", buoy_map=fresh(many))
    r.append(check("...and the log names what was dropped", up_log.has("B5", "B6", "leaves out"), up_log.lines))
    return r


def case_passage():
    r = []
    for n in range(0, 11):
        size = len(rxl_uplink.passage_payload(rxl_uplink.Passage(None, None, tuple(
            (i, 1.28, 103.85, 2) for i in range(n)), (), ()), 0))
        if size != 21 + 9 * n:
            r.append(check("SAFE_PASSAGE is 21 + 9n bytes (n=%d)" % n, False, size))
            break
    else:
        r.append(check("SAFE_PASSAGE is 21 + 9n bytes, n = 0..10 (111 at ten)", True))

    up = uplink()
    sent = tick(up, T0, task="TASK_SAFE_PASSAGE", buoy_map=fresh(gate_map(), 123456))
    sp = only(sent, 0x8011)
    r.append(check("task = TASK_SAFE_PASSAGE, a decided map: sent at once (+ the heartbeat)",
                   len(sp) == 1 and len(only(sent, 0x8010)) == 1))
    m = rxl.decode(0x8011, sp[0])[1]
    r.append(check("payload: 5 buoys, entry/exit placed, time_boot_ms = the map's stamp",
                   len(m["buoys"]) == 5 and m["entry"] and m["exit"] and m["time_boot_ms"] == 123456, m["time_boot_ms"]))
    r.append(check("...states [FLASHING_BLUE, RED, GREEN, OFF, STEADY_BLUE] = [4, 2, 3, 1, 5]",
                   [b["state"] for b in m["buoys"]] == [4, 2, 3, 1, 5]))
    r.append(check("time_boot_ms wraps at 2**32, never overflows the field",
                   rxl.decode(0x8011, rxl_uplink.passage_payload(
                       rxl_uplink.select_passage(gate_map()), 2 ** 32 + 7))[1]["time_boot_ms"] == 7))

    # gating
    for task in ("TASK_NONE", "TASK_INFRA_SURVEY_REPAIR", "TASK_COORDINATED_LOGISTICS", "TASK_DYNAMIC_INCIDENT"):
        sent = run(uplink(), 12.0, task, buoy_map=fresh(gate_map()))
        r.append(check("%s: never a SAFE_PASSAGE" % task, not [1 for _, pt, _p in sent if pt == 0x8011]))
    log = Log()
    r.append(check("task right, map stale/absent: nothing, and the log says so",
                   not [1 for _, pt, _p in run(uplink(log=log), 12.0, "TASK_SAFE_PASSAGE", buoy_map=None)
                        if pt == 0x8011] and log.has("no fresh buoy map"), log.lines))
    log = Log()
    r.append(check("task right, empty map: nothing",
                   not [1 for _, pt, _p in run(uplink(), 12.0, "TASK_SAFE_PASSAGE", buoy_map=fresh([]))
                        if pt == 0x8011]))
    r.append(check("task right, only UNKNOWN buoys: nothing (no state is decided)",
                   not [1 for _, pt, _p in run(uplink(log=log), 12.0, "TASK_SAFE_PASSAGE",
                                               buoy_map=fresh([buoy(1, "UNKNOWN"), buoy(2, "UNKNOWN")]))
                        if pt == 0x8011] and log.has("no buoy has a decided state"), log.lines))

    # debounce + refresh timeline at 20 Hz
    def timeline(map_at):
        sent = run(uplink(hz=0.0), 14.0, "TASK_SAFE_PASSAGE", buoy_map=lambda t: fresh(map_at(t)))
        return [round(t - T0, 2) for t, pt, _ in sent if pt == 0x8011], sent

    times, _ = timeline(lambda t: gate_map())
    r.append(check("steady map: first at 0, then a refresh every 5 s (0, 5, 10)", times == [0.0, 5.0, 10.0], times))

    def change_at_5_2(t):
        m = gate_map()
        if t - T0 >= 5.2:
            m[3] = buoy(4, "FLASHING_RED")          # the OFF buoy turns out to be lit
        return m
    times, sent = timeline(change_at_5_2)
    r.append(check("a state change at 5.2 s (a refresh went at 5.0): held to the 1 s debounce",
                   times == [0.0, 5.0, 6.0, 11.0], times))
    last = rxl.decode(0x8011, [p for _, pt, p in sent if pt == 0x8011][2])[1]
    r.append(check("...and what went at 6.0 is the NEW state", last["buoys"][3]["state"] == 2))

    def new_buoy_at_2(t):
        return gate_map() + ([buoy(6, "OFF")] if t - T0 >= 2.0 else [])
    times, _ = timeline(new_buoy_at_2)
    r.append(check("a buoy joining the set at 2.0 s: sent at once (2.0)", times[:2] == [0.0, 2.0], times))

    def moves(t):                                     # centimetre jitter, every tick
        return [(i, la + 1e-7 * ((t - T0) * 20 % 7), lo, lab) for i, la, lo, lab in gate_map()]
    times, _ = timeline(moves)
    r.append(check("position jitter alone is not a change: refresh only (0, 5, 10)", times == [0.0, 5.0, 10.0], times))

    def flapping(t):
        m = gate_map()
        m[3] = buoy(4, "OFF" if int((t - T0) * 20) % 2 else "FLASHING_RED")
        return m
    times, _ = timeline(flapping)
    r.append(check("a state flipping EVERY tick still costs one packet a second", len(times) <= 15 and
                   all(b - a >= 1.0 - 1e-6 for a, b in zip(times, times[1:])), len(times)))

    def task_flap(t):
        return "TASK_SAFE_PASSAGE" if (t - T0) % 4 < 2 else "TASK_NONE"
    up = uplink(hz=0.0)
    sent = []
    for i in range(int(11.5 * 20) + 1):
        t = T0 + i / 20.0
        if tick(up, t, task_flap(t), buoy_map=fresh(gate_map())):
            sent.append(round(t - T0, 2))
    r.append(check("leaving and re-entering the task re-sends at once (0, 4, 8)", sent == [0.0, 4.0, 8.0], sent))
    return r


def case_budget():
    r = []
    ten = ([buoy(i, "OFF") for i in range(1, 9)] + [buoy(9, "FLASHING_BLUE"), buoy(10, "SOLID_BLUE")])
    sent = run(uplink(), 60.0, "TASK_SAFE_PASSAGE", buoy_map=fresh(ten))
    overhead = 17        # 12 of MAVLink 2 + 5 of TUNNEL header (EKKO_RADIO_LINK.md)
    per_s = sum(overhead + len(p) for _, _, p in sent) / 60.0
    sizes = sorted({len(p) for _, _, p in sent})
    r.append(check("heartbeat 41 B frame, ten-buoy SAFE_PASSAGE 128 B frame",
                   sizes == [24, 111], sizes))
    r.append(check("steady added load, ten buoys, 2 Hz: <= 120 B/s", per_s <= 120.0, "%.1f B/s" % per_s))
    return r


def case_describe():
    r = []
    frames = tick(uplink(), T0, task="TASK_SAFE_PASSAGE", buoy_map=fresh(gate_map()))
    hb, sp = [rxl_uplink.describe(t, p) for t, p in frames]
    r.append(check("the Radio tab can read a heartbeat", hb and hb[0] == "HEARTBEAT" and "AUTO" in hb[1]
                   and "AIRBORNE" in hb[1] and "TASK_SAFE_PASSAGE" in hb[1], hb))
    r.append(check("...and a SAFE_PASSAGE", sp and sp[0] == "SAFE_PASSAGE" and "5 buoy" in sp[1]
                   and "entry known" in sp[1], sp))
    r.append(check("...and a short garbage frame without raising",
                   (rxl_uplink.describe(0x8010, b"\x01") or ("", ""))[1].startswith("does not decode")))
    r.append(check("a boat_link type is not ours to describe",
                   rxl_uplink.describe(boat_link.PAYLOAD_LIGHTS, b"") is None
                   and rxl_uplink.describe(boat_link.PAYLOAD_TEST, b"hi") is None))
    return r


def case_ocs():
    if not os.path.isfile(OCS_CODEC):
        return [skip("decode and re-encode with the OCS's own codec", "no OCS checkout at " + OCS_CODEC)]
    spec = importlib.util.spec_from_file_location("ocs_rxl_codec", OCS_CODEC)
    ocs = importlib.util.module_from_spec(spec)
    sys.modules["ocs_rxl_codec"] = ocs
    spec.loader.exec_module(ocs)
    r = []
    frames = tick(uplink(), T0, task="TASK_SAFE_PASSAGE", buoy_map=fresh(gate_map(), 99))
    for ptype, payload in frames:
        name, theirs = ocs.decode(ptype, payload)
        _, ours = rxl.decode(ptype, payload)
        r.append(check("%s: the OCS decodes our bytes to the same fields" % name, theirs == ours))
    hb = rxl.decode(0x8010, frames[0][1])[1]
    again = ocs.pack_heartbeat(
        vehicle=hb["vehicle"], robot_state=hb["robot_state"], flight_phase=hb["flight_phase"],
        pos_valid=hb["pos_valid"], att_valid=hb["att_valid"], alt_valid=hb["alt_valid"],
        lat=hb["lat"], lon=hb["lon"], spd_mps=hb["spd_mps"], hdg_deg=hb["hdg_deg"],
        roll_deg=hb["roll_deg"], pitch_deg=hb["pitch_deg"], alt_m=hb["alt_m"], task=hb["task"],
        phase=hb["phase"])
    r.append(check("HEARTBEAT: the OCS re-encodes it to the same 24 bytes", again == frames[0][1]))
    sp = rxl.decode(0x8011, frames[1][1])[1]
    again = ocs.pack_safe_passage(
        time_boot_ms=sp["time_boot_ms"], entry=(sp["entry"]["lat"], sp["entry"]["lon"]),
        exit_=(sp["exit"]["lat"], sp["exit"]["lon"]),
        buoys=[(b["lat"], b["lon"], b["state"]) for b in sp["buoys"]])
    r.append(check("SAFE_PASSAGE: the OCS re-encodes it to the same bytes", again == frames[1][1]))
    # what the OCS will refuse, per its translate rules, restated on the decoded dict
    refused = (not hb["pos_valid"] or not hb["att_valid"] or not hb["alt_valid"]
               or hb["hdg_deg"] is None or hb["robot_state"] == 0 or hb["flight_phase"] == 0
               or hb["task"] not in ocs.RX_TASK or hb["task"] == 0)
    r.append(check("a fully-vouched heartbeat trips none of the OCS's refusal rules", not refused))
    r.append(check("no buoy in the SAFE_PASSAGE is UNKNOWN", all(b["state"] != 0 for b in sp["buoys"])))
    return r


# ------------------------------------------------------------------ telemetry_bridge, ROS stubbed

def load_bridge():
    """telemetry_bridge imported with rclpy / uav_msgs / std_msgs stubbed, so its
    glue can run on a laptop. (It also does `import tty`, which a Windows laptop lacks.)"""
    class Stub(types.ModuleType):
        """Every name imported from it is a MagicMock: a message type, a QoS enum..."""
        def __getattr__(self, name):
            if name.startswith("__"):
                raise AttributeError(name)
            return MagicMock(name=name)

    for name in ("rclpy", "rclpy.qos", "rclpy.executors", "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv",
                 "uav_msgs", "uav_msgs.msg"):
        sys.modules[name] = Stub(name)
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {})
    sys.modules["rclpy.node"] = node
    try:
        import tty  # noqa: F401
    except ImportError:
        sys.modules["tty"] = types.ModuleType("tty")
    spec = importlib.util.spec_from_file_location(
        "bridge_under_test", os.path.join(REPO, "uav_fcu", "uav_fcu", "telemetry_bridge.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class FakeLogger:
    """A node logger that keeps its lines and swallows rclpy's throttle kwargs."""

    def __init__(self):
        self.lines = []

    def info(self, text, **kw):
        self.lines.append(("info", text))

    def warning(self, text, **kw):
        self.lines.append(("warning", text))

    warn = warning

    def error(self, text, **kw):
        self.lines.append(("error", text))


def case_bridge():
    r = []
    try:
        tb = load_bridge()
    except Exception as e:                                # noqa: BLE001
        return [check("telemetry_bridge imports with ROS stubbed", False, "%s: %s" % (type(e).__name__, e))]
    r.append(check("telemetry_bridge imports with ROS stubbed", True))

    # every YAML key is declared, and every declared key is in the YAML, in range
    import yaml
    from uav_common import param_utils
    cfg = yaml.safe_load(open(os.path.join(REPO, "uav_bringup", "config", "uav_params.yaml"),
                              encoding="utf-8"))["telemetry_bridge"]["ros__parameters"]
    spec = tb.PARAM_SPEC
    r.append(check("YAML keys == PARAM_SPEC keys (a stray key kills the node at start)",
                   set(cfg) == set(spec), sorted(set(cfg) ^ set(spec))))
    bad = [k for k, sp in spec.items() if k in cfg and sp.get("lo") is not None
           and param_utils.check_range(k, cfg[k], {k: (sp["lo"], sp["hi"])})]
    r.append(check("every YAML value is inside its declared range", not bad, bad))
    ints = [k for k in ("ocs_rxl_heartbeat_hz", "geoid_separation_m", "airborne_alt_m")
            if not isinstance(cfg[k], float)]
    r.append(check("the three new values are FLOATS (an int fails the float range)", not ints, ints))

    logger = FakeLogger()
    fake = types.SimpleNamespace(get_logger=lambda: logger,
                                 _passage_map=tb.StreamCache(rxl_uplink.BUOY_MAP_FRESH_S))
    msg = types.SimpleNamespace(
        buoys=[types.SimpleNamespace(id=1, latitude=1.28, longitude=103.85, label="FLASHING_BLUE")],
        export_stem="map-a", header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=1760000000, nanosec=250000000)))
    now = time.monotonic()
    tb.TelemetryBridge._buoy_map_cb(fake, msg)
    r.append(check("the buoy-map callback keeps the lights path's tuples unchanged",
                   fake._buoys == [(1, 1.28, 103.85, "FLASHING_BLUE")] and fake._map_id == "map-a"))
    held = fake._passage_map.get(now + 0.5)
    r.append(check("...and caches them with the map's stamp in Unix ms",
                   held == (fake._buoys, 1760000000250), held))
    r.append(check("...fresh for 2 s, gone after 3: blank, not the last map",
                   fake._passage_map.get(now + 2.0) is not None and fake._passage_map.get(now + 3.5) is None))

    # the shielded tick
    fake._task = task_core.CurrentTask()
    fake._task.update("TASK_SAFE_PASSAGE")
    sent = []
    fake._send_tunnel = lambda sysid, ptype, payload: sent.append((sysid, ptype))
    fake._rxl = uplink()
    fake._passage_map.set((gate_map(), 5), now)
    tb.TelemetryBridge._rxl_tick(fake, now, pose(), status(), att(), 2)
    r.append(check("_rxl_tick sends heartbeat + SAFE_PASSAGE as broadcasts (target 0)",
                   sorted(sent) == [(0, 0x8010), (0, 0x8011)], sent))

    def broken(*a):
        raise OSError("socket gone")
    fake._send_tunnel = broken
    fake._rxl = uplink()
    try:
        tb.TelemetryBridge._rxl_tick(fake, now, pose(), status(), att(), 2)
        ok = any(lvl == "error" and "OSError" in text for lvl, text in logger.lines)
        r.append(check("a failing send is logged, never raised into the 20 Hz tick", ok, logger.lines[-1:]))
    except Exception as e:                                # noqa: BLE001
        r.append(check("a failing send is logged, never raised into the 20 Hz tick", False, repr(e)))

    # _send_tunnel: target 0, the payload type, the Radio tab's name and summary
    calls, published = [], []
    mav = types.SimpleNamespace(
        tunnel_encode=lambda *a: calls.append(a) or types.SimpleNamespace(get_msgbuf=lambda: b"x" * 41),
        send=lambda m: None)
    f2 = types.SimpleNamespace(
        conn=types.SimpleNamespace(mav=mav, source_system=200, source_component=191),
        radio_pub=types.SimpleNamespace(publish=published.append),
        get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0)),
        _radio_frame=lambda *a: a)
    payload = tick(uplink(), T0)[0][1]
    tb.TelemetryBridge._send_tunnel(f2, 0, 0x8010, payload)
    r.append(check("_send_tunnel: target 0/0, type 0x8010, 24 real bytes in the 128-byte field",
                   calls and calls[0][:4] == (0, 0, 0x8010, 24) and len(calls[0][4]) == 128,
                   calls and calls[0][:4]))
    rec = published[0]
    r.append(check("...and the Radio tab logs it as HEARTBEAT, 41 B, with a readable summary",
                   rec[4] == "HEARTBEAT" and "AIRBORNE" in rec[6] and rec[7] == 41, rec[4:8]))
    tb.TelemetryBridge._send_tunnel(f2, 0, boat_link.PAYLOAD_TEST, b"hi")
    r.append(check("...and a boat_link frame is still described by boat_link",
                   published[1][4] == "TEST", published[1][4:7]))
    return r + case_task_topic(tb)


def case_task_topic(tb):
    """The subscription both nodes share, and the WiFi heartbeat's use of it."""
    r = []
    subs, logger = [], FakeLogger()
    node = types.SimpleNamespace(
        get_logger=lambda: logger,
        create_subscription=lambda msg_type, topic, cb, qos: subs.append((topic, cb)))
    task = task_core.CurrentTask()
    task.subscribe(node)
    r.append(check("CurrentTask.subscribe listens on /uav/current_task",
                   [t for t, _ in subs] == ["/uav/current_task"], subs and subs[0][0]))
    cb = subs[0][1]
    cb(types.SimpleNamespace(data="TASK_SAFE_PASSAGE"))
    r.append(check("...a published task takes effect and is logged",
                   task.value == "TASK_SAFE_PASSAGE" and any("TASK_SAFE_PASSAGE" in t for _, t in logger.lines)))
    cb(types.SimpleNamespace(data="TASK_SAFEPASSAGE"))
    cb(types.SimpleNamespace(data="TASK_SAFEPASSAGE"))
    errors = [t for lvl, t in logger.lines if lvl == "error"]
    r.append(check("...a typo is logged ONCE as an error and the task stays",
                   task.value == "TASK_SAFE_PASSAGE" and len(errors) == 1, errors))

    spec = importlib.util.spec_from_file_location(
        "ocs_client_under_test", os.path.join(REPO, "uav_groundstation", "uav_groundstation", "ocs_client_node.py"))
    oc = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(oc)
    except Exception as e:                                # noqa: BLE001
        return r + [check("ocs_client_node imports with ROS stubbed", False, "%s: %s" % (type(e).__name__, e))]
    now = time.monotonic()
    caches = {}
    for name, value in (("_pose", rxl_uplink.Pose(*pose())), ("_status", rxl_uplink.Status(*status())),
                        ("_att", rxl_uplink.Attitude(*att())),
                        ("_flight", types.SimpleNamespace(landed_state=2, valid=True))):
        caches[name] = tb.StreamCache(1.0)
        caches[name].set(value, now)
    fake = types.SimpleNamespace(
        p={"fake_telemetry": False, "geoid_separation_m": GEOID, "airborne_alt_m": 1.0,
           "team_id": "ASTA", "vehicle_id": "UAV1"},
        _task=task, _skipped=0, _quiet_reason=None, _phase_source=None,
        get_logger=lambda: logger, **caches)
    wifi = oc.OcsClient._build(fake, now)["heartbeat"]
    radio = rxl.decode(0x8010, tick(uplink(), T0, task=task.value)[0][1])[1]
    r.append(check("the WiFi heartbeat carries the task from /uav/current_task",
                   wifi["current_task"] == "TASK_SAFE_PASSAGE", wifi["current_task"]))
    r.append(check("...and it is the SAME task the radio heartbeat carries (the OCS fallback never flips it)",
                   rxl.RX_TASK[radio["task"]] == wifi["current_task"]))
    task.update("TASK_NONE")
    r.append(check("...stood down on both: TASK_NONE",
                   oc.OcsClient._build(fake, now)["heartbeat"]["current_task"] == "TASK_NONE"
                   and rxl.RX_TASK[rxl.decode(0x8010, tick(uplink(), T0, task=task.value)[0][1])[1]["task"]]
                   == "TASK_NONE"))
    return r


def main():
    sections = [("the copy and the numbers", case_copy), ("the current task", case_task),
                ("heartbeat", case_heartbeat), ("buoy selection", case_selection),
                ("safe passage", case_passage), ("radio budget", case_budget),
                ("the Radio tab", case_describe), ("against the OCS's own codec", case_ocs),
                ("telemetry_bridge glue", case_bridge)]
    results = []
    for title, fn in sections:
        print("\n" + title)
        results += fn()
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
