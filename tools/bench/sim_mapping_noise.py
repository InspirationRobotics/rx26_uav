#!/usr/bin/env python3
"""sim_mapping_noise — how the buoy mapper holds up against non-RTK GPS.

NOT a pass/fail bench. An experiment, kept so its numbers can be re-derived
after any change to the tracker instead of being remembered:

    python3 tools/bench/sim_mapping_noise.py

It replays bench_mapping's synthetic flight (two buoys 3 m apart, one flashing
blue, one solid blue, the aircraft circling between them through a full turn
for 40 s) with three kinds of realistic error added:

  GPS wander  a slow random walk, not white noise -- that is what non-RTK GPS
              does, and it is what looks like a real bias over a few seconds
  pixel jitter on every box
  misclassification: the model calling a lit beacon dark or vice versa

and prints, per noise level over 8 seeds, the position error, how often both
buoys got the right state, and what --solve-yaw recovers for a true 7 deg mount
offset.

Recorded 2026-09-13 with per-frame assignment and wander-following association:
  wander 0.3 m: position ~0.15 m median, 8/8 states right, offset solved 3..9
  wander 0.6 m: position ~0.3 m median, 7/8 right (one extra UNKNOWN), -2..12
  wander 1.0 m: position ~0.6 m median, ~4/8 right: 3 m apart starts to mix
This circles both buoys continuously; hovering over each for 6 s, as flown, lets
far less wander build up inside one buoy's watch.
"""
import importlib.util
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "bench_mapping", os.path.join(HERE, "bench_mapping.py"))
bm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bm)

from uav_common import geo  # noqa: E402
from uav_perception.mapping_core import replay, solve_mount_yaw  # noqa: E402


def noisy(flight, rng, wander_m, px, misclass):
    out = []
    wx = wy = 0.0
    for t, f, dets, w, h in flight:
        wx = 0.97 * wx + rng.gauss(0, wander_m * 0.25)
        wy = 0.97 * wy + rng.gauss(0, wander_m * 0.25)
        f = dict(f)
        x, y = geo.latlon_to_xy(f["lat"], f["lon"], (bm.LAT0, bm.LON0))
        f["lat"], f["lon"] = geo.xy_to_latlon(x + wx, y + wy, (bm.LAT0, bm.LON0))
        nd = []
        for d in dets:
            d = dict(d)
            jx, jy = rng.gauss(0, px), rng.gauss(0, px)
            d["x0"] += jx
            d["x1"] += jx
            d["y0"] += jy
            d["y1"] += jy
            if rng.random() < misclass:
                d["class_name"] = "dark" if d["class_name"] == "blue" else "blue"
            nd.append(d)
        out.append((t, f, nd, w, h))
    return out


def nearest_error(buoys, bx, by):
    def err(b):
        x, y = geo.latlon_to_xy(b["lat"], b["lon"], (bm.LAT0, bm.LON0))
        return math.hypot(x - bx, y - by)
    return min(err(b) for b in buoys)


def main():
    for wander, px, mis in ((0.3, 3, 0.05), (0.6, 4, 0.08), (1.0, 6, 0.10)):
        errs, right, solved = [], 0, []
        for seed in range(8):
            fl = noisy(bm.synthetic_flight(seconds=40.0), random.Random(seed),
                       wander, px, mis)
            _, buoys, _ = replay(fl, bm._proj_kw(mount_yaw_offset_deg=7.0),
                                 bm._trk_kw(), 0.35)
            if buoys:
                errs += [nearest_error(buoys, bx, by) for (bx, by), _ in bm.TRUTH]
            if sorted(b["label"] for b in buoys) == ["FLASHING_BLUE", "SOLID_BLUE"]:
                right += 1
            best = solve_mount_yaw(fl, bm._proj_kw(), bm._trk_kw(), 0.35,
                                   offsets=range(-20, 21), signs=(1.0, -1.0))[0]
            solved.append("%+.0f%s" % (best["mount_yaw_offset_deg"],
                                       "" if best["gimbal_yaw_sign"] > 0 else "(-)"))
        errs.sort()
        print("wander %.1f m, %d px, %2d%% misclass: position median %.2f m, "
              "worst %.2f m; both states right %d/8; offset (true +7) %s"
              % (wander, px, mis * 100, errs[len(errs) // 2], errs[-1], right,
                 " ".join(solved)), flush=True)


if __name__ == "__main__":
    sys.exit(main())
