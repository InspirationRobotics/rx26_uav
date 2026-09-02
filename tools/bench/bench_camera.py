#!/usr/bin/env python3
"""bench_camera — drive uav_camera's pure cores with no camera attached.

Two of them: recorder_core (session naming, the frame index, the disk guard)
and pipeline (the gst-launch string, built as text precisely so it can be
asserted -- and pasted into gst-launch-1.0 -- without GStreamer installed).

No camera, no GStreamer, no ROS. The cases that matter are the ones a real
flight produces only by going wrong: a pose that has gone stale mid-recording,
and a filesystem that fills.

    python3 tools/bench/bench_camera.py

The index is the only thing that will connect a frame to a position six months
from now. Every case below asserts it refuses to guess.
"""
import os
import sys
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "uav_camera"))

from uav_camera.pipeline import Pipeline  # noqa: E402
from uav_camera.recorder_core import (  # noqa: E402
    FRAME_FIELDS, DiskGuard, csv_header, csv_row, session_stem, should_rotate)

POSE = (1.2801000, 103.8552000, 18.250)
ATT = (0.012345, -0.006789, 1.570796)
RTSP = "rtsp://192.168.144.25:8554/main.264"


def check(name, passed, detail=""):
    print("%-26s %-4s %s" % (name, "PASS" if passed else "FAIL", detail[:70]))
    return passed


# ---------------------------------------------------------------- stems

def case_stem_utc():
    when = datetime(2026, 9, 1, 14, 25, 30, tzinfo=timezone.utc)
    got = session_stem(when)
    return check("stem from UTC", got == "20260901T142530Z", got)


def case_stem_naive_refused():
    try:
        session_stem(datetime(2026, 9, 1, 14, 25, 30))
    except ValueError as e:
        return check("naive datetime refused", True, str(e)[:60])
    return check("naive datetime refused", False, "accepted a naive datetime")


def case_stem_offset_refused():
    tz = timezone(timedelta(hours=8))          # Singapore
    try:
        session_stem(datetime(2026, 9, 1, 22, 25, 30, tzinfo=tz))
    except ValueError as e:
        return check("non-UTC refused", True, str(e)[:60])
    return check("non-UTC refused", False, "accepted a +08:00 datetime")


def case_stems_sort():
    a = session_stem(datetime(2026, 9, 1, 9, 5, 0, tzinfo=timezone.utc))
    b = session_stem(datetime(2026, 9, 1, 14, 25, 30, tzinfo=timezone.utc))
    return check("stems sort chronologically", a < b, "%s < %s" % (a, b))


# ---------------------------------------------------------------- index

def case_header_matches_fields():
    n_head = len(csv_header().split(","))
    return check("header matches FRAME_FIELDS",
                 n_head == len(FRAME_FIELDS), "%d columns" % n_head)


def case_row_width():
    row = csv_row(0, 1_000_000, 2_000_000, pose=POSE, attitude=ATT,
                  gimbal_pitch=-89.7, pose_age_s=0.031)
    n = len(row.split(","))
    return check("row width == header width",
                 n == len(FRAME_FIELDS), "%d fields" % n)


def case_row_full():
    row = csv_row(7, 123, 456, pose=POSE, attitude=ATT,
                  gimbal_pitch=-89.7, pose_age_s=0.031).split(",")
    ok = (row[0] == "7" and row[3] == "1.2801000" and row[5] == "18.250"
          and row[9] == "-89.70" and row[10] == "0.031")
    return check("populated row formats", ok, ",".join(row[:6]))


def case_stale_pose_blank():
    """The case this file exists for: pose stale, age still recorded."""
    row = csv_row(9, 1, 2, pose=None, attitude=ATT,
                  gimbal_pitch=-90.0, pose_age_s=0.912).split(",")
    blank = row[3] == "" and row[4] == "" and row[5] == ""
    kept = row[6] != "" and row[10] == "0.912"
    return check("stale pose -> blank, age kept", blank and kept,
                 "lat=%r lon=%r age=%r" % (row[3], row[4], row[10]))


def case_stale_attitude_independent():
    """Pose and attitude go stale independently; one must not blank the other."""
    row = csv_row(10, 1, 2, pose=POSE, attitude=None,
                  gimbal_pitch=-90.0, pose_age_s=0.02).split(",")
    ok = row[3] != "" and row[6] == "" and row[7] == "" and row[8] == ""
    return check("attitude stale, pose kept", ok,
                 "lat=%r roll=%r" % (row[3], row[6]))


def case_no_pose_ever():
    row = csv_row(0, 1, 2).split(",")
    ok = all(f == "" for f in row[3:]) and row[0] == "0"
    return check("no pose ever -> all blank", ok, ",".join(row))


def case_gimbal_unknown_blank():
    row = csv_row(3, 1, 2, pose=POSE, attitude=ATT, gimbal_pitch=None,
                  pose_age_s=0.01).split(",")
    return check("gimbal unknown -> blank", row[9] == "", "gimbal=%r" % row[9])


def case_latlon_precision():
    """7 dp keeps ~11 mm; fewer would quantise below the projection error."""
    row = csv_row(0, 1, 2, pose=(1.28010005, 103.85520009, 1.0)).split(",")
    ok = len(row[3].split(".")[1]) == 7 and len(row[4].split(".")[1]) == 7
    return check("lat/lon keep 7 dp", ok, "%s %s" % (row[3], row[4]))


# ---------------------------------------------------------------- disk

def case_disk_room():
    g = DiskGuard(min_free_mb=500)
    may, newly, reason = g.check(2048)
    return check("disk with room", may and not newly and reason == "")


def case_disk_trips_once():
    g = DiskGuard(min_free_mb=500)
    a = g.check(100)
    b = g.check(90)
    ok = (not a[0] and a[1]) and (not b[0] and not b[1])
    return check("disk trips exactly once", ok, a[2])


def case_disk_recovers():
    g = DiskGuard(min_free_mb=500)
    g.check(100)
    may, _, _ = g.check(900)
    again = g.check(100)
    return check("recovery re-arms the edge", may and again[1])


def case_disk_bad_floor():
    try:
        DiskGuard(min_free_mb=0)
    except ValueError:
        return check("zero floor refused", True)
    return check("zero floor refused", False, "accepted min_free_mb=0")


# ---------------------------------------------------------------- rotation

def case_rotate():
    on = should_rotate(100.0, 100.0 + 300.0, 300.0)
    off = should_rotate(100.0, 100.0 + 299.0, 300.0)
    disabled = should_rotate(100.0, 100.0 + 99999.0, 0.0)
    return check("rotation boundary", on and not off and not disabled)


# ---------------------------------------------------------------- pipeline

def case_pipeline_full():
    d = Pipeline(RTSP, want_frames=True, preview_fps=5).describe("/tmp/a.mkv")
    need = ("rtspsrc", "rtph265depay", "h265parse", "tee name=enc",
            "matroskamux", "filesink", "nvv4l2decoder", "tee name=dec",
            "appsink name=frames", "appsink name=preview", "framerate=5/1")
    missing = [n for n in need if n not in d]
    return check("pipeline has every branch", not missing, ",".join(missing))


def case_pipeline_record_only():
    """want_frames=False is the data-collection sortie: no decode at all."""
    d = Pipeline(RTSP, want_frames=False).describe("/tmp/a.mkv")
    ok = "nvv4l2decoder" not in d and "appsink" not in d and "filesink" in d
    return check("record-only omits decode", ok)


def case_pipeline_no_sink():
    """Nothing consuming the tee is a config mistake, not a silent no-op."""
    try:
        Pipeline(RTSP, want_frames=False).describe(None)
    except ValueError as e:
        return check("no-sink pipeline refused", True, str(e)[:52])
    return check("no-sink pipeline refused", False, "built a graph with no sink")


def case_pipeline_bad_url():
    """The SIYI UDP protocol shares the port; refuse it before GStreamer does."""
    try:
        Pipeline("udp://192.168.144.25:8554")
    except ValueError as e:
        return check("non-rtsp url refused", True, str(e)[:52])
    return check("non-rtsp url refused", False, "accepted a non-rtsp url")


def case_pipeline_leaky():
    """The record queue must NOT be leaky; the others must be.

    A leaky record queue drops frames from the FILE under load, which is the one
    thing the recording exists to avoid. A non-leaky preview queue stalls the
    pipeline back to the socket when a browser reads slowly.
    """
    d = Pipeline(RTSP, want_frames=True).describe("/tmp/a.mkv")
    rec = d.split("matroskamux")[0].split("enc.")[-1]
    return check("record queue not leaky",
                 "leaky=no" in rec and d.count("leaky=downstream") >= 2)


def main():
    print("frame index columns: %s\n" % csv_header())
    results = [
        case_stem_utc(),
        case_stem_naive_refused(),
        case_stem_offset_refused(),
        case_stems_sort(),
        case_header_matches_fields(),
        case_row_width(),
        case_row_full(),
        case_stale_pose_blank(),
        case_stale_attitude_independent(),
        case_no_pose_ever(),
        case_gimbal_unknown_blank(),
        case_latlon_precision(),
        case_disk_room(),
        case_disk_trips_once(),
        case_disk_recovers(),
        case_disk_bad_floor(),
        case_rotate(),
        case_pipeline_full(),
        case_pipeline_record_only(),
        case_pipeline_no_sink(),
        case_pipeline_bad_url(),
        case_pipeline_leaky(),
    ]
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
