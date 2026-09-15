#!/usr/bin/env python3
"""map_session — map a recorded flight after landing, with the aircraft's own code.

The buoy mapper has no in-flight tuning, by design. This is where a map gets
checked and a constant gets corrected: it runs the SAME geolocate + tracker the
aircraft runs (uav_perception.mapping_core) over a recorded flight, with
uav_params.yaml's buoy_mapper settings unless told otherwise.

THREE WAYS TO FEED IT

  1. A live mapper's sightings log -- every detection with its pose, so no model
     is needed and it runs anywhere, laptop included:
       python3 tools/scripts/map_session.py --sightings logs/buoy_maps/<stem>_sightings.csv

  2. A camera session's frame index and stills, running the model (needs
     ultralytics, so run it inside the container on the Jetson):
       python3 tools/scripts/map_session.py \\
           --frames logs/video/<session>_frames.csv \\
           --stills logs/video/<session>_stills --model models/ekko_colour_yolo26n.pt

  3. A frame index plus YOLO label files (hand labels, or a Roboflow export).
     Class names come from --names or a data.yaml beside the labels:
       python3 tools/scripts/map_session.py --frames <session>_frames.csv --labels <dir>

WHAT IT PRINTS
  every buoy: position, state, spread, sightings, watch time
  why detections were refused, counted
  the distance between every pair of buoys  -- check against a tape measure
  --truth LAT,LON[,NAME]  distance from a surveyed point to the nearest buoy.
                          Set Ekko on the ground beside a buoy and read its
                          GPS: same receiver, so a fair comparison.
  --solve-yaw             the mount_yaw_offset_deg and gimbal_yaw_sign that make
                          each buoy's sightings agree best. Needs a flight that
                          TURNED with buoys off-centre, and quiet GPS: it is a
                          cross-check on the bench measurement, not a
                          replacement for it. See solve_mount_yaw.

  --set key=value         override any buoy_mapper parameter for this run only.
  --out DIR               also write the map files (kml, csv, plan, json).

Stills are written at ~2 Hz, not the detector's 4 Hz, so a stills replay gets
half the colour samples of a live flight in the same watch time. The tool says
so when it sees it, rather than quietly reporting UNKNOWN everywhere.
"""
import argparse
import csv
import glob
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for _pkg in ("uav_common", "uav_camera", "uav_perception"):
    sys.path.insert(0, os.path.join(REPO, _pkg))

from uav_common import config as uav_config  # noqa: E402
from uav_common import geo  # noqa: E402
from uav_perception import detector_core, map_export  # noqa: E402
from uav_perception.mapping_core import (  # noqa: E402
    FRAGMENT_PENALTY_M, PROJECTOR_KEYS, TRACKER_KEYS, read_sightings, replay,
    solve_mount_yaw)

_FLOAT_COLS = ("lat", "lon", "alt_rel", "roll", "pitch", "yaw", "gimbal_pitch",
               "pose_age_s", "gimbal_yaw", "gimbal_yaw_rate", "gimbal_age_s")


# ------------------------------------------------------------------ settings

def load_settings(params_path, overrides):
    """buoy_mapper's parameters, plus detector_node.edge_margin_px, plus --set."""
    p = dict(uav_config.node_params("buoy_mapper", params_path))
    p["edge_margin_px"] = uav_config.node_params(
        "detector_node", params_path).get("edge_margin_px", 12)
    for item in overrides:
        key, _, raw = item.partition("=")
        if key not in p:
            sys.exit("--set %s: no such buoy_mapper parameter (have: %s)"
                     % (key, ", ".join(sorted(p))))
        cur = p[key]
        if isinstance(cur, bool):
            p[key] = raw.strip().lower() in ("1", "true", "yes")
        elif isinstance(cur, int):
            p[key] = int(raw)
        elif isinstance(cur, float):
            p[key] = float(raw)
        else:
            p[key] = raw
    return p


# ------------------------------------------------------------------- inputs

def read_frames_csv(path):
    """-> ({frame_idx: frame}, session, has_gimbal_yaw)."""
    session = os.path.basename(path).replace("_frames.csv", "")
    frames = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        has_yaw = "gimbal_yaw" in (reader.fieldnames or [])
        for row in reader:
            fr = {k: (float(row[k]) if row.get(k) not in (None, "") else None)
                  for k in _FLOAT_COLS}
            fr["frame_idx"] = int(row["frame_idx"])
            fr["session"] = session
            fr["t"] = int(row["ros_time_ns"]) / 1e9
            frames[fr["frame_idx"]] = fr
    return frames, session, has_yaw


def label_frame_idx(path, session):
    """The frame index a label or still file belongs to, or None.

    Handles plain stills (00012953.txt), the renamed dataset
    (0433_163358_232518Z_00012953.txt) and Roboflow exports
    (..._00012953_jpg.rf.<hash>.txt). When the name carries a session token
    ('232518Z') for a DIFFERENT session, it is skipped: one labelled folder can
    hold several sessions, and a frame index is only unique within one.
    """
    stem = os.path.basename(path).split("_jpg.rf.")[0]
    stem = os.path.splitext(stem)[0]
    tokens = stem.split("_")
    zs = [t for t in tokens if len(t) == 7 and t.endswith("Z") and t[:6].isdigit()]
    if session and zs and session[-7:] not in zs:
        return None
    digits = [t for t in tokens if t.isdigit()]
    return int(digits[-1]) if digits else None


def find_names(labels_dir, explicit):
    if explicit:
        return [n.strip() for n in explicit.split(",")]
    import yaml
    for d in (labels_dir, os.path.dirname(labels_dir.rstrip("/\\"))):
        y = os.path.join(d, "data.yaml")
        if os.path.exists(y):
            names = yaml.safe_load(open(y, encoding="utf-8")).get("names")
            if isinstance(names, dict):
                names = [names[k] for k in sorted(names)]
            if names:
                print("class names from %s: %s" % (y, names))
                return list(names)
    sys.exit("no class names: pass --names, or put a data.yaml beside the labels")


def from_labels(frames, session, labels_dir, names, size, margin):
    w, h = size
    out = []
    for path in sorted(glob.glob(os.path.join(labels_dir, "*.txt"))):
        idx = label_frame_idx(path, session)
        if idx is None or idx not in frames:
            continue
        dets = []
        for line in open(path, encoding="utf-8"):
            parts = line.split()
            if len(parts) < 5:
                continue
            c, cx, cy, bw, bh = int(parts[0]), *map(float, parts[1:5])
            box = ((cx - bw / 2) * w, (cy - bh / 2) * h,
                   (cx + bw / 2) * w, (cy + bh / 2) * h)
            dets.append(_det(names[c] if c < len(names) else str(c), 1.0, box,
                             detector_core.full_view(box, w, h, margin)))
        out.append((frames[idx]["t"], frames[idx], dets, w, h))
    return out


def from_model(frames, session, stills_dir, model_path, conf, imgsz, margin):
    import cv2
    from ultralytics import YOLO
    model = YOLO(model_path)
    names = dict(model.names)
    print("model %s classes %s" % (os.path.basename(model_path), names))
    out = []
    stills = sorted(glob.glob(os.path.join(stills_dir, "*.jpg")))
    for k, path in enumerate(stills):
        idx = label_frame_idx(path, session)
        if idx is None or idx not in frames:
            continue
        img = cv2.imread(path)
        if img is None:
            continue
        h, w = img.shape[:2]
        res = model.predict(img, imgsz=imgsz, conf=conf, verbose=False)[0]
        dets = []
        for b, c, cid in zip(res.boxes.xyxy.cpu().numpy(),
                             res.boxes.conf.cpu().numpy(),
                             res.boxes.cls.cpu().numpy()):
            box = tuple(float(v) for v in b)
            dets.append(_det(names.get(int(cid), str(int(cid))), float(c), box,
                             detector_core.full_view(box, w, h, margin)))
        out.append((frames[idx]["t"], frames[idx], dets, w, h))
        if k % 100 == 0:
            print("  %d/%d stills" % (k, len(stills)), flush=True)
    return out


def _det(name, conf, box, full):
    return {"class_name": name, "confidence": conf, "x0": box[0], "y0": box[1],
            "x1": box[2], "y1": box[3], "full_view": bool(full)}


# ------------------------------------------------------------------- report

def dist_m(a_lat, a_lon, b_lat, b_lon):
    x, y = geo.latlon_to_xy(b_lat, b_lon, (a_lat, a_lon))
    return math.hypot(x, y)


def warn_sample_rate(frames, p):
    ts = sorted(t for t, _, dets, _, _ in frames if dets)
    if len(ts) < 3:
        return
    gap = statistics.median(b - a for a, b in zip(ts, ts[1:]) if b > a)
    rate = 1.0 / gap if gap > 0 else float("inf")
    need = p["min_samples"] / rate if rate else float("inf")
    if need > p["min_observe_s"] * 1.25:
        print("\nNOTE: detections arrive at ~%.1f Hz here, so min_samples=%d "
              "needs ~%.0f s of full-view watching per buoy (the aircraft's 4 Hz "
              "needs %.0f s). Expect UNKNOWN states from a stills replay; for a "
              "rough read try --set min_samples=%d."
              % (rate, p["min_samples"], need, p["min_observe_s"],
                 max(4, int(rate * p["min_observe_s"]))))


def report(buoys, outcomes, truths):
    print("\n%-4s %-16s %-24s %9s %9s %8s %5s" % (
        "id", "state", "lat, lon", "spread", "sightings", "watched", "lit"))
    for b in sorted(buoys, key=lambda b: b["id"]):
        print("B%-3d %-16s %11.7f,%12.7f %7.2f m %9d %6.1f s %4.0f%%" % (
            b["id"], b["label"], b["lat"], b["lon"], b["spread_m"],
            b["sightings"], b["observed_s"], 100 * b["lit_fraction"]))
    if not buoys:
        print("(no buoys)")
    print("\ndetection outcomes:")
    for k, v in outcomes.most_common():
        print("  %6d  %s" % (v, k))
    if len(buoys) > 1:
        print("\ndistance between buoys (check against the tape measure):")
        bs = sorted(buoys, key=lambda b: b["id"])
        for i, a in enumerate(bs):
            for b in bs[i + 1:]:
                print("  B%d - B%d  %.2f m" % (a["id"], b["id"],
                                             dist_m(a["lat"], a["lon"],
                                                    b["lat"], b["lon"])))
    for lat, lon, name in truths:
        if not buoys:
            break
        near = min(buoys, key=lambda b: dist_m(lat, lon, b["lat"], b["lon"]))
        print("truth %-10s nearest is B%d at %.2f m"
              % (name, near["id"], dist_m(lat, lon, near["lat"], near["lon"])))


def parse_truth(items):
    out = []
    for i, s in enumerate(items):
        parts = s.split(",")
        name = parts[2] if len(parts) > 2 else "T%d" % (i + 1)
        out.append((float(parts[0]), float(parts[1]), name))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sightings", help="a live mapper's <stem>_sightings.csv")
    src.add_argument("--frames", help="a camera session's <session>_frames.csv")
    ap.add_argument("--labels", help="YOLO label .txt directory (with --frames)")
    ap.add_argument("--stills", help="stills directory (with --frames --model)")
    ap.add_argument("--model", help="weights for --stills")
    ap.add_argument("--names", help="class names in id order, comma separated")
    ap.add_argument("--image-size", default="1920x1080", help="for --labels")
    ap.add_argument("--conf", type=float, default=0.25, help="for --model")
    ap.add_argument("--imgsz", type=int, default=1920, help="for --model")
    ap.add_argument("--params", default=None, help="uav_params.yaml to read")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--truth", action="append", default=[],
                    metavar="LAT,LON[,NAME]")
    ap.add_argument("--solve-yaw", action="store_true")
    ap.add_argument("--out", help="also write the map files here")
    args = ap.parse_args()

    p = load_settings(args.params, args.set)
    stem = "replay"
    if args.sightings:
        frames = read_sightings(open(args.sightings, newline="", encoding="utf-8"))
        stem = "replay_" + os.path.basename(args.sightings).replace(
            "_sightings.csv", "")
    else:
        rows, session, has_yaw = read_frames_csv(args.frames)
        stem = "replay_" + session
        if not has_yaw and p["gimbal_yaw_mode"] != "aircraft":
            print("NOTE: %s predates the gimbal_yaw column. Using "
                  "gimbal_yaw_mode=aircraft (the nose heading) for this run; "
                  "positions in turns will be less accurate."
                  % os.path.basename(args.frames))
            p["gimbal_yaw_mode"] = "aircraft"
        if args.labels:
            w, h = (int(v) for v in args.image_size.lower().split("x"))
            frames = from_labels(rows, session, args.labels,
                                 find_names(args.labels, args.names), (w, h),
                                 p["edge_margin_px"])
        elif args.stills and args.model:
            frames = from_model(rows, session, args.stills, args.model,
                                args.conf, args.imgsz, p["edge_margin_px"])
        else:
            sys.exit("--frames needs either --labels DIR or --stills DIR --model PT")
    frames.sort(key=lambda fr: fr[0])
    print("%d frames with detections input" % sum(1 for fr in frames if fr[2]))

    proj_kw = {k: p[k] for k in PROJECTOR_KEYS}
    trk_kw = {k: p[k] for k in TRACKER_KEYS}
    warn_sample_rate(frames, p)
    _, buoys, outcomes = replay(frames, proj_kw, trk_kw, p["min_conf"])
    print("\nsettings: yaw mode %s, sign %+.0f, mount offset %+.1f deg, launch "
          "%+.2f m above surface" % (p["gimbal_yaw_mode"], p["gimbal_yaw_sign"],
                                     p["mount_yaw_offset_deg"],
                                     p["launch_height_above_surface_m"]))
    report(buoys, outcomes, parse_truth(args.truth))

    if args.solve_yaw:
        signs = (1.0, -1.0) if p["gimbal_yaw_mode"] == "body" else (p["gimbal_yaw_sign"],)
        ranked = solve_mount_yaw(frames, proj_kw, trk_kw, p["min_conf"], signs=signs)
        if not ranked:
            print("\nsolve-yaw: no candidate produced any buoy")
        else:
            print("\nsolve-yaw, best first (score = mean spread + %.1f m per extra "
                  "buoy):" % FRAGMENT_PENALTY_M)
            for r in ranked[:5]:
                print("  offset %+5.1f deg  sign %+.0f  %d buoys  spread %.2f m  "
                      "score %.2f" % (r["mount_yaw_offset_deg"],
                                      r["gimbal_yaw_sign"], r["buoys"],
                                      r["spread_m"], r["score"]))
            worst = ranked[-1]["score"]
            if worst - ranked[0]["score"] < 0.05:
                print("  every offset scores alike: this flight has no turns with "
                      "buoys off-centre, so it cannot tell. Do not change the "
                      "constant from this run.")
            else:
                best = ranked[0]
                print("  best fit: mount_yaw_offset_deg %.1f, gimbal_yaw_sign %.1f"
                      % (best["mount_yaw_offset_deg"], best["gimbal_yaw_sign"]))
                print("  CROSS-CHECK ONLY. GPS wander above ~0.3 m moves this by "
                      "several degrees (simulated: a true 7 solved anywhere from "
                      "-2 to +12 at 0.6 m). Change the parameter only if it "
                      "agrees with the bench measurement, or with a second "
                      "flight to within ~2 deg.")

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        opts = {"alt_m": p["waypoint_alt_m"], "hold_s": p["waypoint_hold_s"]}
        for path in map_export.write_all(args.out, stem, buoys, opts):
            print("wrote", path)


if __name__ == "__main__":
    main()
