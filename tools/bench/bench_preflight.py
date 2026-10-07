#!/usr/bin/env python3
"""bench_preflight — the ground station's battery, GPS and pre-flight logic, with
no aircraft and no ROS.

    python3 tools/bench/bench_preflight.py

What it covers, and why each is here:

  fcu_decode      MAVLink "unknown" sentinels become blanks, never numbers. A
                  -1 amps read as a measurement is a battery readout that lies.
  battery_core    the time-to-failsafe estimate against a modelled pack, and the
                  property it is built on: scaling every current reading by 2
                  (an uncalibrated sensor) must not change the answer.
  preflight_core  each check fires on the failure that earned it a place, and
                  an input that has not arrived is "unknown", never "ok".
  camera_frame    the footprint rectangle points where the camera heading says,
                  at the size the field of view says.
  siyi decode     the camera's encoding reply decodes to what preflight_camera.sh
                  prints.
  armed_clock     armed time this power-on: counts only seen armed time, counts a
                  flight only on a seen disarmed->armed edge, resumes after a
                  ground station restart, resets on a new boot.
"""
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for pkg in ("uav_common", "uav_groundstation", "uav_camera"):
    sys.path.insert(0, os.path.join(REPO, pkg))

from uav_camera.siyi_client import decode_encoding  # noqa: E402
from uav_common import camera_frame, fcu_decode  # noqa: E402
from uav_groundstation import (armed_clock, battery_core, fence_view,  # noqa: E402
                              preflight_core)


def check(name, passed, detail=""):
    print("%-48s %-4s %s" % (name, "PASS" if passed else "FAIL", str(detail)[:60]))
    return bool(passed)


def isnan(x):
    return isinstance(x, float) and math.isnan(x)


# ---------------------------------------------------------------- fcu_decode

def case_decode():
    r = []
    v, a, pct = fcu_decode.battery_from_sys_status(22614, 4512, 77)
    r.append(check("SYS_STATUS mV/cA -> V/A", abs(v - 22.614) < 1e-9
                   and abs(a - 45.12) < 1e-9 and pct == 77))
    v, a, pct = fcu_decode.battery_from_sys_status(65535, -1, -1)
    r.append(check("SYS_STATUS unknowns -> NaN / -1", isnan(v) and isnan(a)
                   and pct == -1))
    r.append(check("consumed -1 -> NaN", isnan(fcu_decode.consumed_mah(-1))))
    fix, sats, hdop, acc = fcu_decode.gps_from_raw(4, 57, 25, 450)
    r.append(check("GPS_RAW_INT -> DGPS, 25, 0.57, 0.45 m",
                   (fix, sats) == (4, 25) and abs(hdop - 0.57) < 1e-9
                   and abs(acc - 0.45) < 1e-9))
    _f, _s, hdop, acc = fcu_decode.gps_from_raw(3, 65535, 255, 0)
    r.append(check("GPS unknown eph / no h_acc -> NaN", isnan(hdop) and isnan(acc)))
    r.append(check("param id NUL-padded bytes -> name",
                   fcu_decode.param_name(b"BATT_LOW_VOLT\x00\x00\x00") == "BATT_LOW_VOLT"))
    return r


# ---------------------------------------------------------------- battery_core

REST_START_V = 24.0
REST_SLOPE_V_PER_MIN = -0.05
R_TRUE = 0.0115
LOW_VOLT = 21.6


def fly(est, seconds, current_scale=1.0, t0=0.0, amps_fn=None):
    """Feed a modelled 10 Hz hover. Returns the last t."""
    t = t0
    for k in range(int(seconds * 10)):
        t = t0 + k / 10.0
        amps = amps_fn(t) if amps_fn else 45.0 + 8.0 * math.sin(t * 0.7)
        volts = REST_START_V + REST_SLOPE_V_PER_MIN * t / 60.0 - R_TRUE * amps
        est.feed(t, volts, amps * current_scale, float("nan"), -1, True)
    return t


def expected_minutes(t):
    rest = REST_START_V + REST_SLOPE_V_PER_MIN * t / 60.0
    return (rest - R_TRUE * 45.0 - LOW_VOLT) / -REST_SLOPE_V_PER_MIN


def case_battery():
    r = []
    est = battery_core.BatteryEstimator()
    t = fly(est, 30)
    s = est.snapshot(t, LOW_VOLT, 1.0)
    r.append(check("no estimate before 60 s of flight", s["minutes"] is None,
                   s["basis"]))
    t = fly(est, 150)
    s = est.snapshot(t, LOW_VOLT, 1.0)
    want = expected_minutes(t)
    r.append(check("minutes within 5%% of the model (%.1f)" % want,
                   s["minutes"] is not None and abs(s["minutes"] - want) / want < 0.05,
                   s["minutes"]))
    r.append(check("sag resistance fitted", s["r_ohm"] is not None
                   and abs(s["r_ohm"] - R_TRUE) < 0.002, s["r_ohm"]))

    doubled = battery_core.BatteryEstimator()
    t2 = fly(doubled, 150, current_scale=2.0)
    s2 = doubled.snapshot(t2, LOW_VOLT, 1.0)
    r.append(check("current sensor reading 2x -> same minutes",
                   abs(s2["minutes"] - s["minutes"]) < 0.5,
                   "%.2f vs %.2f" % (s2["minutes"], s["minutes"])))

    steady = battery_core.BatteryEstimator()
    t3 = fly(steady, 150, amps_fn=lambda _t: 45.0)
    s3 = steady.snapshot(t3, LOW_VOLT, 1.0)
    r.append(check("constant current -> falls back to measured R",
                   s3["minutes"] is not None and "13 Sep" in s3["basis"], s3["basis"]))

    r.append(check("no BATT_LOW_VOLT -> no estimate, says why",
                   est.snapshot(t, float("nan"), 1.0)["minutes"] is None))
    r.append(check("stale reading -> None", est.snapshot(t + 5.0, LOW_VOLT, 1.0) is None))
    blank = est.snapshot(t, LOW_VOLT, 1.0)
    r.append(check("NaN current/consumed leave as None (JSON-safe)",
                   blank["consumed_mah"] is None))

    swap = battery_core.BatteryEstimator()
    fly(swap, 150)
    t4 = fly(swap, 20, t0=400.0)       # a later flight on another pack
    r.append(check("a landing gap restarts the trend",
                   swap.snapshot(t4, LOW_VOLT, 1.0)["minutes"] is None))
    r.append(check("6S from BATT_LOW_VOLT 21.6", battery_core.cells_for(21.6, 23.0) == 6))
    return r


# ---------------------------------------------------------------- preflight_core

def good_inputs():
    return {
        "fcu_ok": True, "pose_ok": True, "armed": False,
        "gps": {"fix_type": 4, "satellites": 25, "hdop": 0.57},
        "battery": {"voltage": 24.3, "low_volt": 21.6, "cells": 6},
        "fence_enable": 1.0, "fence_alt_max": 12.0, "fence_margin": 2.0,
        "fence_type": 5.0, "fence_radius": 35.0, "fence_polygon": True,
        "working_alt_m": 10.0,
        "camera": {"running": True, "gimbal_ok": True, "gimbal_pitch": -90.1,
                   "nadir_pitch": -90.0, "codec": "H264", "width": 1920,
                   "height": 1080, "kbps": 1570, "encoding_age_s": 12.0,
                   "expected_codec": "h264"},
        "record_gate": "will discard: never armed",
        "mapping": {"detector": True, "mapper": True},
        "search": {"running": True, "phase": "ready",
                   "text": "Ready: flip SC to GUIDED to start"},
    }


def states(inp):
    return {c["key"]: c["state"] for c in preflight_core.checks(inp)}


def case_preflight():
    r = []
    st = states(good_inputs())
    r.append(check("a good aircraft is all ok", set(st.values()) == {"ok"}, st))

    def one(mutate, key, want, name):
        inp = good_inputs()
        mutate(inp)
        got = states(inp)[key]
        r.append(check(name, got == want, got))

    one(lambda i: i.update(fence_alt_max=10.0), "fence", "bad",
        "13 Sep fence: 10 m ceiling, 2 m margin, 10 m pass -> bad")
    one(lambda i: i.update(fence_alt_max=9.0, fence_margin=0.0), "fence", "bad",
        "ceiling under the pass altitude -> bad")
    # Sideways: the circle and the polygon each count when they exist.
    one(lambda i: i.update(fence_type=7.0), "fence", "ok",
        "FENCE_TYPE 7, polygon held -> ok, polygon + circle")
    one(lambda i: i.update(fence_type=7.0, fence_polygon=False), "fence", "ok",
        "6 Oct: FENCE_TYPE 7, no polygon, 35 m circle -> ok")
    one(lambda i: i.update(fence_type=7.0, fence_radius=None), "fence", "unknown",
        "circle bit, FENCE_RADIUS unread -> unknown")
    one(lambda i: i.update(fence_type=5.0, fence_polygon=False), "fence", "warn",
        "polygon bit, no polygon, no circle -> warn")
    one(lambda i: i.update(fence_type=5.0, fence_polygon=None), "fence", "unknown",
        "polygon not read back yet -> unknown")
    one(lambda i: i.update(fence_type=1.0), "fence", "warn",
        "FENCE_TYPE 1 (ceiling only) -> warn")
    six = [c for c in preflight_core.checks(dict(good_inputs(), fence_type=7.0,
                                                  fence_polygon=False))
           if c["key"] == "fence"][0]
    r.append(check("the circle is named, with its radius",
                   six["detail"].startswith("circle 35 m around home"), six["detail"]))
    one(lambda i: i.update(fence_type=4.0, fence_alt_max=5.0), "fence", "ok",
        "polygon only: ceiling not enforced, not judged -> ok")
    one(lambda i: i.update(search={"running": True, "phase": "not_ready",
                                   "text": "Not ready: take off first"}),
        "search", "warn", "search on but not ready -> warn")
    one(lambda i: i.update(search={"running": False}), "search", "off",
        "search_node not running -> off")
    one(lambda i: i.update(search={"running": True, "phase": "hover",
                                   "text": "Confirming B4"}),
        "search", "ok", "search hovering -> ok")
    one(lambda i: i["camera"].update(kbps=2000), "stream", "bad",
        "2000 kbps main stream -> bad")
    one(lambda i: i["camera"].update(width=1280, height=720), "stream", "warn",
        "720p revert -> warn")
    one(lambda i: i["camera"].update(codec="H265"), "stream", "bad",
        "H265 into an h264 pipeline -> bad")
    one(lambda i: i["camera"].update(codec="", encoding_age_s=None), "stream",
        "unknown", "encoding never read -> unknown, not ok")
    one(lambda i: i["camera"].update(gimbal_pitch=0.3), "gimbal", "bad",
        "gimbal at the horizon -> bad")
    one(lambda i: i.update(record_gate="will discard: no FCU status"), "rec", "bad",
        "no FCU status gate -> bad")
    one(lambda i: i.update(fcu_ok=False), "tel", "bad", "telemetry down -> bad")
    one(lambda i: i.update(gps={"fix_type": 3, "satellites": 8, "hdop": 1.6}),
        "gps", "warn", "3D fix, 8 sats, HDOP 1.6 -> warn")
    one(lambda i: i.update(gps=None), "gps", "unknown", "no GPS data -> unknown")
    # The receiver's own accuracy decides when it gives one (Septentrio HAS).
    has = lambda **k: dict({"fix_type": 3, "satellites": 11, "hdop": 1.42, "h_acc_m": 0.10}, **k)
    one(lambda i: i.update(gps=has()), "gps", "ok",
        "2 Oct HAS: 11 sats, HDOP 1.42, 0.10 m -> ok")
    one(lambda i: i.update(gps=has(h_acc_m=0.9)), "gps", "warn",
        "HAS still settling, 0.9 m -> warn")
    one(lambda i: i.update(gps=has(h_acc_m=1.9)), "gps", "bad",
        "standalone just after power-on, 1.9 m -> bad")
    one(lambda i: i.update(gps=has(satellites=8)), "gps", "warn",
        "HAS on 8 sats -> warn even at 0.10 m")
    one(lambda i: i.update(gps=has(satellites=5)), "gps", "bad",
        "HAS on 5 sats -> bad even at 0.10 m")
    # RTK FIXED is judged on accuracy: GPS+Galileo-only corrections ride on
    # 8-11 satellites (6 Oct 2026), so only the poor count vetoes it.
    rtk = lambda **k: has(fix_type=6, satellites=8, hdop=1.26, h_acc_m=0.03, **k)
    one(lambda i: i.update(gps=rtk()), "gps", "ok", "RTK fixed on 8 sats, 0.03 m -> ok")
    one(lambda i: i.update(gps=dict(rtk(), satellites=5)), "gps", "bad",
        "RTK fixed on 5 sats -> bad")
    one(lambda i: i.update(gps=dict(rtk(), h_acc_m=0.8)), "gps", "warn",
        "RTK fixed but 0.8 m -> warn (accuracy still decides)")
    one(lambda i: i.update(gps=has(fix_type=5, satellites=8, h_acc_m=0.3)), "gps", "warn",
        "RTK FLOAT on 8 sats -> warn (only FIXED is exempt)")
    one(lambda i: i.update(gps=has(hdop=2.4)), "gps", "warn", "HAS at HDOP 2.4 -> warn")
    one(lambda i: i.update(gps=has(hdop=3.5)), "gps", "bad", "HAS at HDOP 3.5 -> bad")
    one(lambda i: i.update(gps=has(h_acc_m=0.0)), "gps", "warn",
        "accuracy 0 = not reported -> old rule (11, 1.42) -> warn")
    gap = [c for c in preflight_core.checks(dict(good_inputs(), gps=has(fix_type=1)))
           if c["key"] == "gps"][0]
    r.append(check("HAS gap (no fix, 11 sats) -> bad, says restart",
                   gap["state"] == "bad" and "restart the receiver" in gap["detail"], gap["detail"][:60]))
    ok = [c for c in preflight_core.checks(dict(good_inputs(), gps=has()))
          if c["key"] == "gps"][0]
    r.append(check("detail shows the accuracy", "±0.10 m" in ok["detail"], ok["detail"]))
    # The receiver's own word, when the radio carries it (boat_link GNSS packet).
    rx = {"mode": 10, "mode_name": "HAS", "satellites": 11, "h_acc_m": 0.10, "v_acc_m": 0.71}
    lab = [c for c in preflight_core.checks(dict(good_inputs(), gps=has(), gnss=rx))
           if c["key"] == "gps"][0]
    r.append(check("receiver says HAS -> the chip says HAS, not 3D",
                   lab["state"] == "ok" and lab["detail"].startswith("HAS · 11 sats"), lab["detail"]))
    gap = [c for c in preflight_core.checks(dict(good_inputs(), gps=has(fix_type=1), gnss=rx))
           if c["key"] == "gps"][0]
    half = [c for c in preflight_core.checks(dict(
        good_inputs(), gps=has(h_acc_m=0.40), gnss=dict(rx, h_acc_m=0.80))) if c["key"] == "gps"][0]
    r.append(check("receiver's 0.80 m (95%) wins over the autopilot's halved 0.40",
                   half["state"] == "warn" and "±0.80 m" in half["detail"], half["detail"]))
    r.append(check("autopilot no fix + receiver HAS -> bad, names the gap",
                   gap["state"] == "bad" and "itself is in HAS on 11" in gap["detail"]
                   and gap["detail"].startswith("no fix (receiver: HAS)"), gap["detail"]))
    one(lambda i: i["battery"].update(voltage=22.8), "batt", "warn",
        "13 Sep pack at 22.8 V resting -> warn")
    one(lambda i: i.update(armed=True, battery={"voltage": 21.8, "low_volt": 21.6,
                                                "cells": 6}),
        "batt", "bad", "armed, 0.2 V above failsafe -> bad")
    one(lambda i: i.update(mapping={"detector": False, "mapper": False}), "map",
        "off", "mapping nodes not started -> off, not bad")
    one(lambda i: i.update(mapping={"detector": True, "mapper": False,
                                    "other_model": "Tins and circles (Task 2/3)"}),
        "map", "off", "detector on the tins model -> off, no 'start both' nag")
    return r


# ---------------------------------------------------------------- camera_frame

def case_fence_view():
    """fence_view: what binds sideways, and inside/outside against it."""
    r = []
    home = (32.9239385, -117.0385936)              # Ekko's home, 6 Oct 2026
    on = {"fence_enable": 1.0, "fence_type": 7.0, "fence_radius": 35.0}
    v = fence_view.view(on, None, home)
    r.append(check("6 Oct: circle only, described", v is not None and v["polygon"] is None
                   and v["desc"] == "circle 35 m around home", v))
    r.append(check("parked 5.9 m from home -> inside",
                   fence_view.inside(v, 32.9239896, -117.0385759) is True))
    r.append(check("40 m north of home -> outside",
                   fence_view.inside(v, home[0] + 40 / 111320.0, home[1]) is False))
    r.append(check("no position -> not judged", fence_view.inside(v, None, None) is None))
    sq = [(home[0] - 1e-4, home[1] - 1e-4), (home[0] - 1e-4, home[1] + 1e-4),
          (home[0] + 1e-4, home[1] + 1e-4), (home[0] + 1e-4, home[1] - 1e-4)]
    both = fence_view.view(on, sq, home)
    r.append(check("polygon + circle, described", both["desc"] ==
                   "polygon + circle 35 m around home", both["desc"]))
    r.append(check("inside the circle but outside the polygon -> outside",
                   fence_view.inside(both, home[0] + 20 / 111320.0, home[1]) is False))
    r.append(check("home not read yet -> no view",
                   fence_view.view(on, None, None) is None))
    r.append(check("radius not read yet -> no view",
                   fence_view.view(dict(on, fence_radius=float("nan")), None, home) is None))
    r.append(check("fence off -> no view",
                   fence_view.view(dict(on, fence_enable=0.0), sq, home) is None))
    r.append(check("polygon bit but nothing uploaded, no circle -> no view",
                   fence_view.view(dict(on, fence_type=5.0), None, home) is None))
    xy = fence_view.circle_xy(v, home)
    r.append(check("circle on a map centred at home", xy is not None and
                   abs(xy["x"]) < 1e-6 and abs(xy["y"]) < 1e-6 and xy["r"] == 35.0, xy))
    return r


def case_footprint():
    r = []
    c = camera_frame.nadir_footprint(10.0, 0.0, 81.0)
    half_w = 10.0 * math.tan(math.radians(40.5))
    r.append(check("heading north: top edge is north", c[0][1] > 0 and c[1][1] > 0
                   and c[2][1] < 0, c[0]))
    r.append(check("width from HFOV 81 at 10 m (%.2f m)" % (2 * half_w),
                   abs((c[1][0] - c[0][0]) - 2 * half_w) < 1e-9))
    r.append(check("height from the 16:9 aspect",
                   abs((c[0][1] - c[3][1]) - 2 * half_w * 9 / 16) < 1e-9))
    e = camera_frame.nadir_footprint(10.0, 90.0, 81.0)
    r.append(check("heading east: top edge is east", e[0][0] > 0 and e[1][0] > 0
                   and e[2][0] < 0, e[0]))
    r.append(check("unknown heading -> None",
                   camera_frame.nadir_footprint(10.0, None, 81.0) is None))
    r.append(check("on the ground -> None",
                   camera_frame.nadir_footprint(0.0, 0.0, 81.0) is None))
    return r


# ---------------------------------------------------------------- siyi decode

def case_encoding():
    payload = struct.pack("<BBHHHB", 1, 1, 1920, 1080, 1570, 0)
    r = [check("0x20 reply -> H264 1920x1080 1570",
               decode_encoding(payload) == ("H264", 1920, 1080, 1570))]
    r.append(check("short reply -> None", decode_encoding(b"\x01\x01") is None))
    r.append(check("unknown codec is named, not dropped",
                   decode_encoding(struct.pack("<BBHHHB", 1, 9, 1, 1, 1, 0))[0]
                   == "codec 9"))
    return r


def case_armed_clock():
    import tempfile
    r = []
    d = tempfile.mkdtemp()
    path = os.path.join(d, "armed_time.json")
    c = armed_clock.ArmedClock(path, "boot-A")
    c.update(0.0, True, False)
    c.update(1.0, True, True)          # takeoff
    c.update(61.0, True, True)
    c.update(71.0, True, False)        # landed after 70 s
    r.append(check("70 s armed, 1 flight", abs(c.seconds(71.0) - 70.0) < 1e-9
                   and c.flights == 1, c.snapshot(71.0)))
    c.update(80.0, True, True)
    c.update(90.0, False, False)       # telemetry lost mid-flight...
    c.update(94.0, False, False)       # ...for longer than the grace: stop at 90
    c.update(95.0, True, True)         # back, still armed: no new flight
    c.update(105.0, True, False)
    r.append(check("dropout: unseen time not counted, not a new flight",
                   abs(c.seconds(105.0) - 90.0) < 1e-9 and c.flights == 2,
                   c.snapshot(105.0)))
    f = armed_clock.ArmedClock(None, None)
    f.update(0.0, True, False)
    f.update(1.0, True, True)
    for k in range(10):                # a 1 s heartbeat flickering stale
        f.update(2.0 + k * 3.0, False, False)
        f.update(2.5 + k * 3.0, True, True)
    f.update(40.0, True, False)
    r.append(check("heartbeat flicker keeps counting", abs(f.seconds(40.0) - 39.0) < 1e-9
                   and f.flights == 1, f.snapshot(40.0)))
    again = armed_clock.ArmedClock(path, "boot-A")
    r.append(check("ground_station restart resumes the total",
                   again.resumed and abs(again.seconds(0.0) - 90.0) < 0.2
                   and again.flights == 2, again.snapshot(0.0)))
    fresh = armed_clock.ArmedClock(path, "boot-B")
    r.append(check("new boot id (power cycle) starts at zero",
                   fresh.seconds(0.0) == 0.0 and fresh.flights == 0))
    nofile = armed_clock.ArmedClock(None, None)
    nofile.update(0.0, True, False)
    nofile.update(1.0, True, True)
    r.append(check("works with nowhere to save", abs(nofile.seconds(3.0) - 2.0) < 1e-9))
    return r


def main():
    results = []
    for title, fn in (("fcu_decode", case_decode), ("battery_core", case_battery),
                      ("preflight_core", case_preflight),
                      ("fence_view", case_fence_view),
                      ("camera_frame", case_footprint),
                      ("siyi encoding", case_encoding),
                      ("armed_clock", case_armed_clock)):
        print("\n" + title)
        results += fn()
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
