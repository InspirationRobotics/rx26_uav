#!/usr/bin/env python3
"""bench_radio -- the Radio tab's log, statistics and frame descriptions.

No ROS, no aircraft, no radio:

    python3 tools/bench/bench_radio.py

Drives uav_groundstation.radio_core with its own clock, so a minute of boat link
takes no time, and uav_common.boat_link.describe() with packets built by the same
pack functions telemetry_bridge and fake_crusader.py use.
"""
import os
import sys

REPO = os.path.join(os.path.dirname(__file__), "..", "..")
sys.path.insert(0, os.path.join(REPO, "uav_groundstation"))
sys.path.insert(0, os.path.join(REPO, "uav_common"))

from uav_common import boat_link                            # noqa: E402
from uav_groundstation import radio_core as rc              # noqa: E402


def check(name, ok, detail=""):
    print("  [%s] %s%s" % ("ok" if ok else "FAIL", name,
                           "  -- " + str(detail) if detail and not ok else ""))
    return bool(ok)


class Clock:
    """A monotonic clock the bench moves by hand."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def boat_packet(log, clock):
    payload = boat_link.pack_boat(32.9238, -117.0386, 3, 4)
    name, summary = boat_link.describe(boat_link.PAYLOAD_BOAT, payload)
    log.add(rc.RX, 42, 191, 200, name, boat_link.PAYLOAD_BOAT, summary, 27)


def main():
    r = []

    print("\nboat_link.describe")
    name, s = boat_link.describe(boat_link.PAYLOAD_BOAT,
                                 boat_link.pack_boat(32.9238, -117.0386, 3, 4))
    r.append(check("boat packet named BOAT", name == "BOAT", name))
    r.append(check("  ...says what the boat is doing and its target",
                   "transiting" in s and "wants slot 4" in s, s))
    _, s2 = boat_link.describe(
        boat_link.PAYLOAD_BOAT,
        boat_link.pack_boat(32.9238, -117.0386, 3, 4, acked=(1, 2, 3)))
    r.append(check("  ...and how many positions it is holding -- the ack",
                   "holds 3 position(s)" in s2, s2))
    # v2 splits the old whole-map packet in two: POSITIONS once per buoy, LIGHTS
    # every period. Both travel as radio SLOTS, so the summaries name slots --
    # except POSITIONS, which is where a slot is tied to its buoy id.
    name, s = boat_link.describe(
        boat_link.PAYLOAD_POSITIONS,
        boat_link.pack_positions([(1, 7, 32.9, -117.0), (2, 4, 32.9001, -117.0)])[0])
    r.append(check("positions packet named POSITIONS", name == "POSITIONS", name))
    r.append(check("  ...ties each buoy id to the slot it travels as",
                   "2 buoy(s)" in s and "B7 as slot 1" in s and "B4 as slot 2" in s, s))
    _, s = boat_link.describe(boat_link.PAYLOAD_POSITIONS, b"\x00")
    r.append(check("  ...an empty positions packet says so", s == "empty", s))
    lights = {1: "FLASHING_RED", 2: "FLASHING_GREEN"}
    name, s = boat_link.describe(boat_link.PAYLOAD_LIGHTS,
                                 boat_link.pack_lights(lights, [1, 2]))
    r.append(check("lights packet named LIGHTS", name == "LIGHTS", name))
    r.append(check("  ...a confirmed gate reads red then green",
                   "2 light(s)" in s and "red slot 1" in s and "green slot 2" in s, s))
    _, s = boat_link.describe(boat_link.PAYLOAD_LIGHTS,
                              boat_link.pack_lights(lights, [2]))
    r.append(check("  ...a single confirmed slot reads as the exit", "exit slot 2" in s, s))
    _, s = boat_link.describe(boat_link.PAYLOAD_LIGHTS, boat_link.pack_lights(lights))
    r.append(check("  ...no confirmation reads as nothing", "confirmed nothing" in s, s))
    name, s = boat_link.describe(boat_link.PAYLOAD_TEST,
                                 boat_link.pack_test("test 1 from ekko"))
    r.append(check("test frame decodes to its text",
                   name == "TEST" and s == "test 1 from ekko", (name, s)))
    r.append(check("pack_test cuts to one TUNNEL payload",
                   len(boat_link.pack_test("x" * 500)) == boat_link.MAX_PAYLOAD))
    name, s = boat_link.describe(0x8018, bytes(5))
    r.append(check("an unknown type is named by its number",
                   name == "TUNNEL_0x8018" and "5 bytes" in s, (name, s)))
    try:
        name, s = boat_link.describe(boat_link.PAYLOAD_BOAT, b"\x01\x02\x03")
        r.append(check("a truncated boat packet is reported, not raised",
                       name == "TUNNEL_0x8000" and "does not decode" in s, (name, s)))
    except Exception as e:                  # noqa: BLE001
        r.append(check("a truncated boat packet is reported, not raised", False, e))

    print("\nRadioLog: records")
    clock = Clock()
    log = rc.RadioLog(capacity=5, boat_sysid=42, clock=clock, wall=lambda: 0.0)
    recs, newest, dropped = log.read()
    r.append(check("empty log reads empty", recs == [] and newest == 0 and dropped == 0))
    log.add(rc.TX, 200, 1, 42, "LIGHTS", boat_link.PAYLOAD_LIGHTS,
        "10 light(s), confirmed nothing", 120)
    boat_packet(log, clock)
    recs, newest, _ = log.read()
    r.append(check("records come back oldest first",
                   [x["name"] for x in recs] == ["LIGHTS", "BOAT"], recs))
    r.append(check("TX is named by where it went, RX by who sent it",
                   recs[0]["who"] == "Crusader (link node)"
                   and recs[1]["who"] == "Crusader (link node)"))
    r.append(check("the cursor returns only newer records",
                   log.read(since_seq=newest)[0] == []))
    for _ in range(8):
        log.add(rc.RX, 255, 190, 0, "HEARTBEAT", 0, "GCS", 21)
    recs, newest, dropped = log.read()
    r.append(check("the ring keeps capacity and counts what fell off",
                   len(recs) == 5 and dropped == 5 and newest == 10,
                   (len(recs), dropped, newest)))
    systems = {s["sys"]: s for s in log.systems()}
    r.append(check("one entry per peer, sent and heard counted together",
                   systems[42]["tx"] == 1 and systems[42]["rx"] == 1, systems.get(42)))
    r.append(check("the ground station is named", systems[255]["name"] == "Ground station"))
    log.add(rc.TX, 200, 1, 0, "TEST", boat_link.PAYLOAD_TEST, "hi", 34)
    r.append(check("a broadcast is named as one", log.read()[0][-1]["who"] == "broadcast"))
    clock.t += 7
    r.append(check("ages come from the clock",
                   abs({s["sys"]: s for s in log.systems()}[42]["heard_s"] - 7) < 1e-9))

    print("\nRadioLog: boat link estimate")
    clock = Clock()
    log = rc.RadioLog(boat_sysid=42, clock=clock)
    b = log.boat_link()
    r.append(check("never heard: no rate and no percentage",
                   b["rate_hz"] is None and b["pct"] is None))
    for _ in range(60):
        boat_packet(log, clock)
        clock.t += 1.0
    b = log.boat_link()
    r.append(check("a steady 1 Hz boat scores ~100%", b["pct"] > 99, b))
    r.append(check("  ...with a longest silence of about a second",
                   abs(b["longest_gap_s"] - 1.0) < 1e-6, b))
    clock = Clock()
    log = rc.RadioLog(boat_sysid=42, clock=clock)
    for i in range(60):
        if not 20 <= i < 30:
            boat_packet(log, clock)
        clock.t += 1.0
    b = log.boat_link()
    r.append(check("ten missing packets lower the estimate",
                   80 < b["pct"] < 90, b))
    r.append(check("  ...and show as an eleven-second silence",
                   abs(b["longest_gap_s"] - 11.0) < 1e-6, b))
    clock = Clock()
    log = rc.RadioLog(boat_sysid=42, clock=clock)
    for _ in range(10):
        boat_packet(log, clock)
        clock.t += 1.0
    r.append(check("a link heard for 10 s is not scored against a minute",
                   log.boat_link()["pct"] == 100.0, log.boat_link()))
    log.add(rc.RX, 2, 191, 0, "HEARTBEAT", 0, "SURFACE_BOAT", 21)
    r.append(check("the boat's heartbeats do not count as boat packets",
                   log.boat_link()["heard"] == 10))
    clock.t += 120
    b = log.boat_link()
    r.append(check("a boat gone quiet for two minutes scores 0%",
                   b["pct"] == 0.0 and abs(b["heard_s"] - 121) < 1e-6, b))
    r.append(check("  ...and its silence fills the whole window",
                   abs(b["longest_gap_s"] - rc.RATE_WINDOW_S) < 1e-6, b))
    r.append(check("no boat id: says why instead of guessing",
                   rc.RadioLog(boat_sysid=None).boat_link()["sys"] is None
                   and "boat_sysid" in rc.RadioLog(boat_sysid=None).boat_link()["why"]))
    log.clear()
    recs, newest, _ = log.read()
    r.append(check("clear empties the log and keeps the sequence",
                   recs == [] and newest == 11 and log.systems() == []
                   and log.boat_link()["rate_hz"] is None, (recs, newest)))

    print("\n%d/%d" % (sum(r), len(r)))
    return 0 if all(r) else 1


if __name__ == "__main__":
    raise SystemExit(main())
