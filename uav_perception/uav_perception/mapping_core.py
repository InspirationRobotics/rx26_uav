"""mapping_core — one frame's detections in, buoys out. Shared by live and replay.

buoy_mapper_node feeds this from /uav/perception/buoy_detections;
tools/scripts/map_session.py feeds it from a recorded flight. Neither does any
geometry or bookkeeping of its own, which is the point: if the live map and a
replay of the same flight ever disagree, the difference is in the inputs, not in
two copies of the logic that drifted apart.

PURE: no ROS, no clock, no files. Also owns the SIGHTINGS LOG format, one row per
detection with every input needed to re-run it, so a flight can be re-mapped with
a corrected constant after landing without re-running the model.

A FRAME is a mapping with recorder_core.FRAME_FIELDS names (lat, lon, alt_rel,
roll, pitch, yaw, pose_age_s, gimbal_pitch, gimbal_yaw, gimbal_yaw_rate,
gimbal_age_s), plus frame_idx and session. A DETECTION is a mapping with
class_name, confidence, x0, y0, x1, y1, full_view.
"""
import csv
import io
import math
from collections import Counter

from uav_perception.buoy_tracker import BuoyTracker, TrackerConfig
from uav_perception.geolocate import Projector, known

# Which buoy_mapper parameters configure which core. Here rather than in the node
# so the replay tool, which has no ROS, builds its cores from the same list.
PROJECTOR_KEYS = ("hfov_deg", "nadir_pitch_deg", "max_off_nadir_deg",
                  "gimbal_yaw_mode", "gimbal_yaw_sign", "mount_yaw_offset_deg",
                  "launch_height_above_surface_m", "target_height_m",
                  "min_alt_m", "max_pose_age_s", "max_gimbal_age_s",
                  "max_gimbal_yaw_rate_dps")
TRACKER_KEYS = ("assoc_radius_m", "merge_radius_m", "min_sightings",
                "min_observe_s", "min_samples", "max_sample_gap_s",
                "solid_min_lit", "off_max_lit", "min_flash_transitions",
                "min_colour_agreement", "lock_state")

# Outcomes, as written to the sightings log and counted on BuoyMap.
USED = "used"
PARTIAL = "partial"
LOW_CONF = "low_conf"
NO_RAY = "no_ray"
REJECTED = "rejected"          # followed by ": <reason>"

FRAME_COLUMNS = ("session", "frame_idx", "t", "width", "height", "lat", "lon",
                 "alt_rel", "roll", "pitch", "yaw", "pose_age_s", "gimbal_pitch",
                 "gimbal_yaw", "gimbal_yaw_rate", "gimbal_age_s")
DETECTION_COLUMNS = ("class_name", "confidence", "x0", "y0", "x1", "y1",
                     "full_view")
RESULT_COLUMNS = ("outcome", "est_lat", "est_lon", "weight", "buoy_id")
SIGHTING_FIELDS = FRAME_COLUMNS + DETECTION_COLUMNS + RESULT_COLUMNS

_FLOATS = {"t", "lat", "lon", "alt_rel", "roll", "pitch", "yaw", "pose_age_s",
           "gimbal_pitch", "gimbal_yaw", "gimbal_yaw_rate", "gimbal_age_s",
           "confidence", "x0", "y0", "x1", "y1", "est_lat", "est_lon", "weight"}
_INTS = {"frame_idx", "width", "height", "buoy_id"}


class Mapper:
    """Projector + tracker + the reasons detections were or were not used."""

    def __init__(self, projector=None, tracker_cfg=None, min_conf=0.35):
        self.projector = projector or Projector()
        self.tracker = BuoyTracker(tracker_cfg or TrackerConfig())
        self.min_conf = float(min_conf)
        self.reset_counters()

    def reset_counters(self):
        self.used = self.partial = self.low_conf = self.frames_rejected = 0
        self.last_reject_reason = ""

    def clear(self):
        self.tracker.clear()
        self.reset_counters()

    def ingest(self, t, frame, detections, width, height):
        """Process one frame. Returns one result dict per detection, in order.

        A frame with no detections is not counted as rejected: nothing was lost.
        """
        results = []
        if not detections:
            return results
        reason = self.projector.reject_reason(frame)
        if reason is not None:
            self.frames_rejected += 1
            self.last_reject_reason = reason
            return [_result(REJECTED + ": " + reason) for _ in detections]
        located = []                   # (result index, lat, lon, weight, class)
        for d in detections:
            if not d["full_view"]:
                self.partial += 1
                results.append(_result(PARTIAL))
                continue
            if d["confidence"] < self.min_conf:
                self.low_conf += 1
                results.append(_result(LOW_CONF))
                continue
            u = (d["x0"] + d["x1"]) / 2.0
            v = (d["y0"] + d["y1"]) / 2.0
            got = self.projector.locate(frame, u, v, width, height)
            if got is None:
                results.append(_result(NO_RAY))
                continue
            results.append(None)
            located.append((len(results) - 1,) + got + (d["class_name"],))
        # The whole frame's sightings go to the tracker TOGETHER: see
        # BuoyTracker.add_frame for why one-at-a-time mixes up nearby buoys.
        ids = self.tracker.add_frame(t, [s[1:] for s in located])
        for (i, lat, lon, w, _cls), bid in zip(located, ids):
            results[i] = _result(USED, lat, lon, w, bid)
            self.used += 1
        return results

    def buoys(self, now=None):
        return self.tracker.buoys(now)

    def stats(self):
        return {"detections_used": self.used, "detections_partial": self.partial,
                "detections_low_conf": self.low_conf,
                "frames_rejected": self.frames_rejected,
                "last_reject_reason": self.last_reject_reason}


def _result(outcome, lat=None, lon=None, weight=None, buoy_id=None):
    return {"outcome": outcome, "est_lat": lat, "est_lon": lon,
            "weight": weight, "buoy_id": buoy_id}


# ---------------------------------------------------------------- sightings log

def sightings_header() -> str:
    return ",".join(SIGHTING_FIELDS)


def _cell(v):
    if not known(v):
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return "%.9g" % v
    return str(v)


def sighting_rows(t, frame, detections, width, height, results):
    """CSV lines (no trailing newline) for one ingested frame."""
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    base = [frame.get("session", ""), frame.get("frame_idx"), t, width, height]
    base += [frame.get(k) for k in FRAME_COLUMNS[5:]]
    for d, r in zip(detections, results):
        w.writerow([_cell(v) for v in base]
                   + [_cell(d[k]) for k in DETECTION_COLUMNS]
                   + [_cell(r[k]) for k in RESULT_COLUMNS])
    return out.getvalue().splitlines()


def _parse(name, s):
    if s == "":
        return None
    if name in _INTS:
        return int(float(s))
    if name in _FLOATS:
        return float(s)
    if name == "full_view":
        return s in ("1", "true", "True")
    return s


def read_sightings(lines):
    """Sightings log lines -> [(t, frame, detections, width, height)] per frame,
    in file order. The recorded RESULT columns are ignored: replay recomputes."""
    frames = []
    key = None
    for row in csv.DictReader(lines):
        rec = {k: _parse(k, row.get(k, "")) for k in SIGHTING_FIELDS}
        k = (rec["session"], rec["frame_idx"], rec["t"])
        det = {c: rec[c] for c in DETECTION_COLUMNS}
        if k != key:
            frame = {c: rec[c] for c in FRAME_COLUMNS}
            frames.append((rec["t"], frame, [det], rec["width"], rec["height"]))
            key = k
        else:
            frames[-1][2].append(det)
    return frames


def replay(frames, projector_kw, tracker_kw, min_conf):
    """Map a whole recorded flight. -> (mapper, buoys, outcome counts).

    `frames` is [(t, frame, detections, width, height)] in time order, as
    read_sightings or the replay tool's frames.csv reader produce.
    """
    m = Mapper(Projector(**projector_kw), TrackerConfig(**tracker_kw), min_conf)
    outcomes = Counter()
    last_t = None
    for t, f, dets, w, h in frames:
        for r in m.ingest(t, f, dets, w, h):
            outcomes[r["outcome"]] += 1      # "rejected: <reason>" stays whole
        last_t = t if last_t is None else max(last_t, t)
    return m, m.buoys(now=last_t), outcomes


# A wrong heading offset smears one buoy into an arc that the tracker may split
# into two tighter tracks, which would score a SMALLER spread for a WORSE answer.
# Each buoy beyond the fewest any candidate produced costs this much, in metres.
FRAGMENT_PENALTY_M = 0.5


def solve_mount_yaw(frames, projector_kw, tracker_kw, min_conf,
                    offsets=range(-45, 46), signs=(1.0, -1.0)):
    """Find the mount yaw offset (and gimbal yaw sign) that makes each buoy's
    sightings agree best. -> candidates sorted best first.

    Only meaningful for a flight that TURNED with buoys away from the image
    centre: a heading error moves an off-centre sighting and leaves an overhead
    one where it is, so a flight that never turned gives every offset the same
    score.

    AND ONLY WHEN THE GPS IS QUIET. The offset shows up only in ABSOLUTE
    positions, which is exactly what GPS wander corrupts. Simulated
    (tools/bench/sim_mapping_noise.py), a true 7 deg offset solved to 3-9 deg at
    0.3 m of wander, to -2..+12 deg at 0.6 m, and to nonsense at 1 m. So this is
    a CROSS-CHECK for a bench measurement of the mount offset, not a substitute:
    believe it only when two separate flights agree to within a couple of degrees.
    """
    results = []
    for sign in signs:
        for off in offsets:
            kw = dict(projector_kw, mount_yaw_offset_deg=float(off),
                      gimbal_yaw_sign=float(sign))
            _, buoys, _ = replay(frames, kw, tracker_kw, min_conf)
            n = sum(b["sightings"] for b in buoys)
            if not n:
                continue
            results.append({
                "mount_yaw_offset_deg": float(off), "gimbal_yaw_sign": float(sign),
                "buoys": len(buoys),
                "spread_m": sum(b["spread_m"] * b["sightings"] for b in buoys) / n,
            })
    if not results:
        return []
    fewest = min(r["buoys"] for r in results)
    for r in results:
        r["score"] = r["spread_m"] + FRAGMENT_PENALTY_M * (r["buoys"] - fewest)
    return sorted(results, key=lambda r: r["score"])


def nan_to_none(frame):
    """Message fields use NaN for unknown; the cores accept either, but logs and
    JSON read cleaner with None."""
    return {k: (None if isinstance(v, float) and math.isnan(v) else v)
            for k, v in frame.items()}
