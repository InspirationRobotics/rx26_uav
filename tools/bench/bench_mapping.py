#!/usr/bin/env python3
"""bench_mapping — drive the buoy mapper's pure cores with no aircraft, no camera
and no model.

    python3 tools/bench/bench_mapping.py

What it covers, and why each is here:

  geolocate     the pixel -> lat/lon arithmetic, against numbers worked out by
                hand. A sign error here does not crash; it mirrors the whole map,
                and nothing on the aircraft would notice.
  buoy_tracker  the five Task 1 states from simulated 4 Hz samples of a real
                1 s on / 1 s off flash, plus the ways a state must NOT be decided:
                too little watching, a buoy that changed rather than flashed,
                colours that disagree.
  mapping_core  partial and low-confidence boxes are refused, and the sightings
                log replays to the same map it was written from.
  map_export    every format parses as what it claims to be.
  map_server    GET-only over real HTTP; downloads carry a filename.
  detector_core camera_node's per-frame metadata survives the MJPEG stream,
                including a header that arrives in a different read than its JPEG.
"""
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for pkg in ("uav_common", "uav_camera", "uav_perception"):
    sys.path.insert(0, os.path.join(REPO, pkg))

from uav_common import geo  # noqa: E402
from uav_camera.mjpeg_server import FrameSlot, MjpegServer  # noqa: E402
from uav_perception import detector_core as dc  # noqa: E402
from uav_perception import map_export  # noqa: E402
from uav_perception.buoy_tracker import BuoyTracker, TrackerConfig  # noqa: E402
from uav_perception.geolocate import (  # noqa: E402
    Projector, camera_heading_deg, focal_px, ground_offset)
from uav_perception.map_server import MapServer  # noqa: E402
from uav_perception.mapping_core import (  # noqa: E402
    LOW_CONF, PARTIAL, USED, Mapper, read_sightings, sighting_rows,
    sightings_header)

W, H = 1920, 1080
F = focal_px(W, 81.0)
LAT0, LON0 = 32.9241721, -117.0192415     # the park, from a real index row


def check(name, passed, detail=""):
    print("%-44s %-4s %s" % (name, "PASS" if passed else "FAIL", str(detail)[:60]))
    return bool(passed)


def near(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ================================================================ geolocate

def case_focal():
    # 960 / tan(40.5 deg) = 1124.0. The 3 Sep measurement said ~1130 at 1080p;
    # 81 deg is that measurement rounded, and the 0.5% between them is ~2 cm at
    # the frame edge from 10 m.
    return check("focal length from HFOV",
                 near(F, 1124.0, 0.5) and near(focal_px(1280, 81.0), 749.3, 0.5),
                 "1920 -> %.1f px, 1280 -> %.1f px" % (F, focal_px(1280, 81.0)))


def case_centre_is_below():
    off = ground_offset(W / 2, H / 2, W, H, F, 10.0, -90.0, 0.0)
    return check("image centre lands directly below",
                 near(off[0], 0.0) and near(off[1], 0.0), off)


def case_right_is_east_facing_north():
    off = ground_offset(W / 2 + 565, H / 2, W, H, F, 10.0, -90.0, 0.0)
    return check("right of centre, facing north -> east",
                 near(off[0], 0.0, 1e-9) and near(off[1], 10.0 * 565 / F, 1e-9),
                 "N %.3f E %.3f" % off)


def case_up_is_forward():
    """Image-up must be the direction the camera faces. Get this backwards and
    every buoy is mirrored through the aircraft."""
    off = ground_offset(W / 2, H / 2 - 300, W, H, F, 10.0, -90.0, 0.0)
    return check("above centre, facing north -> north",
                 off[0] > 0 and near(off[0], 10.0 * 300 / F, 1e-9)
                 and near(off[1], 0.0, 1e-9), "N %.3f E %.3f" % off)


def case_heading_east_rotates():
    right = ground_offset(W / 2 + 400, H / 2, W, H, F, 10.0, -90.0, 90.0)
    up = ground_offset(W / 2, H / 2 - 400, W, H, F, 10.0, -90.0, 90.0)
    d = 10.0 * 400 / F
    ok = (near(right[0], -d, 1e-9) and near(right[1], 0.0, 1e-9)
          and near(up[0], 0.0, 1e-9) and near(up[1], d, 1e-9))
    return check("facing east: right -> south, up -> east", ok,
                 "right N%.2f E%.2f up N%.2f E%.2f" % (right + up))


def case_tilt_forward():
    """-80 is 10 deg up from nadir, toward the front: the centre ray lands
    height*tan(10 deg) ahead."""
    off = ground_offset(W / 2, H / 2, W, H, F, 10.0, -80.0, 0.0)
    return check("gimbal 10 deg off nadir -> tan(10) ahead",
                 near(off[0], 10.0 * math.tan(math.radians(10)), 1e-9),
                 "N %.3f" % off[0])


def case_above_horizon_refused():
    off = ground_offset(W / 2, 0, W, H, F, 10.0, 0.0, 0.0)
    return check("ray that never reaches surface -> None", off is None)


def case_heading_modes():
    yaw = math.radians(90.0)
    ok = (near(camera_heading_deg(yaw, 10.0, "body", 1.0, 0.0), 100.0)
          and near(camera_heading_deg(yaw, 10.0, "body", -1.0, 0.0), 80.0)
          and near(camera_heading_deg(yaw, 10.0, "body", 1.0, -5.0), 95.0)
          and near(camera_heading_deg(None, 200.0, "earth", 1.0, 0.0), 200.0)
          and near(camera_heading_deg(yaw, None, "aircraft", 1.0, 0.0), 90.0)
          and camera_heading_deg(yaw, None, "body", 1.0, 0.0) is None
          and near(camera_heading_deg(math.radians(350), 20.0, "body", 1.0, 0.0), 10.0))
    return check("camera heading: body/earth/aircraft, sign, wrap", ok)


def frame(**over):
    f = {"session": "20260913T000000Z", "frame_idx": 1, "lat": LAT0, "lon": LON0,
         "alt_rel": 10.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "pose_age_s": 0.05,
         "gimbal_pitch": -90.0, "gimbal_yaw": 0.0, "gimbal_yaw_rate": 0.0,
         "gimbal_age_s": 0.05}
    f.update(over)
    return f


def case_gates():
    p = Projector()
    cases = [
        (frame(), None),
        (frame(lat=None), "no pose for this frame"),
        (frame(pose_age_s=0.9), "pose too old"),
        (frame(alt_rel=1.0), "below min_alt_m"),
        (frame(gimbal_age_s=float("nan")), "no fresh gimbal reading"),
        (frame(gimbal_pitch=-70.0), "gimbal off nadir"),
        (frame(gimbal_yaw_rate=45.0), "camera still turning"),
        (frame(gimbal_yaw=None), "no camera heading"),
    ]
    bad = [(want, p.reject_reason(f)) for f, want in cases
           if p.reject_reason(f) != want]
    return check("frame gates refuse each bad frame, in words", not bad, bad)


def case_locate_latlon():
    p = Projector(target_height_m=0.0)
    lat, lon, w = p.locate(frame(), W / 2 + 565, H / 2, W, H)
    x, y = geo.latlon_to_xy(lat, lon, (LAT0, LON0))
    return check("locate -> lat/lon agrees with geo",
                 near(x, 10.0 * 565 / F, 1e-3) and near(y, 0.0, 1e-3)
                 and w < 4.0, "E %.3f N %.3f w %.2f" % (x, y, w))


def case_launch_height():
    """A dock 1.5 m above the water adds 1.5 m of height to every ray."""
    p = Projector(target_height_m=0.0, launch_height_above_surface_m=1.5)
    lat, lon, _ = p.locate(frame(), W / 2 + 565, H / 2, W, H)
    x, _ = geo.latlon_to_xy(lat, lon, (LAT0, LON0))
    return check("launch height above surface is added",
                 near(x, 11.5 * 565 / F, 1e-3), "E %.3f" % x)


# ============================================================= buoy_tracker

def xy_to_ll(x, y):
    return geo.xy_to_latlon(x, y, (LAT0, LON0))


def feed(tracker, x, y, seconds, pattern, hz=4.0, t0=0.0, jitter=0.0):
    """pattern(t) -> class name. Adds a sighting every 1/hz s."""
    n = int(seconds * hz)
    for i in range(n):
        t = t0 + i / hz
        j = jitter * math.sin(i * 1.7)
        lat, lon = xy_to_ll(x + j, y - j)
        tracker.add(t, lat, lon, 1.0, pattern(t))
    return t0 + n / hz


def flash(colour):
    return lambda t: colour if int(t) % 2 == 0 else "dark"


def one_buoy(tr):
    b = tr.buoys(now=10.0)
    return b[0] if len(b) == 1 else None


def case_flashing():
    tr = BuoyTracker()
    feed(tr, 0, 0, 6.0, flash("blue"), jitter=0.2)
    b = one_buoy(tr)
    return check("1 s on / 1 s off at 4 Hz -> FLASHING_BLUE",
                 b and b["label"] == "FLASHING_BLUE" and b["locked"],
                 b and "%s lit %.2f chg %d" % (b["label"], b["lit_fraction"],
                                                b["flash_transitions"]))


def case_solid():
    tr = BuoyTracker()
    feed(tr, 0, 0, 6.0, lambda t: "blue")
    b = one_buoy(tr)
    return check("always lit -> SOLID_BLUE", b and b["label"] == "SOLID_BLUE",
                 b and b["label"])


def case_off():
    tr = BuoyTracker()
    feed(tr, 0, 0, 6.0, lambda t: "dark")
    b = one_buoy(tr)
    return check("never lit -> OFF", b and b["label"] == "OFF", b and b["label"])


def case_red_green():
    tr = BuoyTracker()
    feed(tr, 0, 0, 6.0, flash("red"))
    feed(tr, 20, 0, 6.0, flash("green"), t0=10.0)
    labels = sorted(b["label"] for b in tr.buoys(now=20.0))
    return check("flashing red and green, separately",
                 labels == ["FLASHING_GREEN", "FLASHING_RED"], labels)


def case_too_short():
    tr = BuoyTracker()
    feed(tr, 0, 0, 3.0, lambda t: "blue")
    b = one_buoy(tr)
    return check("3 s of watching -> still UNKNOWN",
                 b and b["label"] == "UNKNOWN" and not b["locked"],
                 b and "%.2fs" % b["observed_s"])


def case_gaps_not_watch_time():
    """Passing over for 1 s, three times, 2 s apart, is 3 s of looking, not 7."""
    tr = BuoyTracker()
    t = 0.0
    for _ in range(4):
        feed(tr, 0, 0, 1.0, lambda t: "blue", t0=t)
        t += 3.0
    b = one_buoy(tr)
    return check("gaps between passes add no watch time",
                 b and b["observed_s"] < 3.5 and b["label"] == "UNKNOWN",
                 b and "%.2fs" % b["observed_s"])


def case_on_then_off():
    """Lit, then dark: a buoy that CHANGED. Must not be called flashing."""
    tr = BuoyTracker()
    feed(tr, 0, 0, 5.0, lambda t: "blue" if t < 2.25 else "dark")
    b = one_buoy(tr)
    return check("on then off -> UNKNOWN, not FLASHING",
                 b and b["label"] == "UNKNOWN",
                 b and "%s chg %d" % (b["label"], b["flash_transitions"]))


def case_colour_disagreement():
    tr = BuoyTracker()
    feed(tr, 0, 0, 6.0, lambda t: "blue" if int(t * 4) % 2 else "green")
    b = one_buoy(tr)
    return check("lit colours that disagree -> UNKNOWN",
                 b and b["label"] == "UNKNOWN",
                 b and "agree %.2f" % b["colour_agreement"])


def case_gate_buoys_separate():
    """Task 1 gates are 3 m apart. Noisy sightings must still give two buoys."""
    tr = BuoyTracker()
    for k in range(6):
        feed(tr, 0, 0, 1.0, flash("red"), t0=2.0 * k, jitter=0.4)
        feed(tr, 3.0, 0, 1.0, flash("green"), t0=2.0 * k + 1.0, jitter=0.4)
    bs = tr.buoys(now=20.0)
    return check("buoys 3 m apart stay two buoys", len(bs) == 2,
                 "%d buoys" % len(bs))


def case_merge():
    """First seen at the frame edge 1.8 m off, then overhead: one buoy, first id."""
    tr = BuoyTracker(TrackerConfig(min_sightings=1))
    tr.add(0.0, *xy_to_ll(0.0, 0.0), 1.0, "blue")
    tr.add(0.25, *xy_to_ll(1.8, 0.0), 1.0, "blue")
    for i in range(20):
        tr.add(0.5 + i * 0.25, *xy_to_ll(0.9, 0.0), 4.0, "blue")
    bs = tr.buoys(now=10.0)
    return check("two tracks that converge merge, keep id 1",
                 len(bs) == 1 and bs[0]["id"] == 1, [b["id"] for b in bs])


def case_stray_hidden():
    tr = BuoyTracker()
    tr.add(0.0, LAT0, LON0, 1.0, "blue")
    return check("a single stray sighting is not a buoy", tr.buoys(now=1.0) == [])


# ============================================================ mapping_core

def det(cls="blue", conf=0.8, dx=0.0, full=True):
    cx, cy = W / 2 + dx, H / 2
    return {"class_name": cls, "confidence": conf, "x0": cx - 26, "y0": cy - 26,
            "x1": cx + 26, "y1": cy + 26, "full_view": full}


def case_partial_and_low_conf():
    m = Mapper()
    r = m.ingest(0.0, frame(), [det(full=False), det(conf=0.1), det()], W, H)
    out = [x["outcome"] for x in r]
    return check("partial and low-confidence boxes refused",
                 out == [PARTIAL, LOW_CONF, USED] and m.partial == 1
                 and m.low_conf == 1 and m.used == 1, out)


def case_rejected_frame_counted():
    m = Mapper()
    m.ingest(0.0, frame(pose_age_s=5.0), [det()], W, H)
    m.ingest(0.1, frame(pose_age_s=5.0), [], W, H)
    return check("rejected frame counted once, empty frame not",
                 m.frames_rejected == 1 and m.last_reject_reason == "pose too old",
                 m.stats())


def case_log_replays():
    """Write a flashing buoy's sightings log, read it back, re-map: same answer."""
    m = Mapper()
    lines = [sightings_header()]
    for i in range(28):
        t = i / 4.0
        f = frame(frame_idx=i)
        d = [det(cls="blue" if int(t) % 2 == 0 else "dark", dx=40 * math.sin(i))]
        lines += sighting_rows(t, f, d, W, H, m.ingest(t, f, d, W, H))
    live = m.buoys(now=10.0)
    r = Mapper()
    for t, f, d, w, h in read_sightings(lines):
        r.ingest(t, f, d, w, h)
    replay = r.buoys(now=10.0)
    ok = (len(live) == len(replay) == 1
          and live[0]["label"] == replay[0]["label"] == "FLASHING_BLUE"
          and near(live[0]["lat"], replay[0]["lat"], 1e-7))
    return check("sightings log replays to the same map", ok,
                 replay and replay[0]["label"])


# ============================================================== map_export

def sample_buoys():
    tr = BuoyTracker()
    feed(tr, 0, 0, 6.0, flash("blue"))
    feed(tr, 3, 0, 6.0, lambda t: "blue", t0=10.0)
    return tr.buoys(now=20.0)


def case_exports_parse():
    bs = sample_buoys()
    csv_lines = map_export.to_csv(bs).strip().splitlines()
    kml = ET.fromstring(map_export.to_kml(bs, "stem").encode())
    marks = kml.findall(".//{http://www.opengis.net/kml/2.2}Placemark")
    plan = json.loads(map_export.to_qgc_plan(bs, alt_m=10.0, hold_s=6.0))
    items = plan["mission"]["items"]
    js = json.loads(map_export.to_json(bs, "stem"))
    ok = (len(csv_lines) == 3 and len(marks) == 2 and len(items) == 2
          and items[0]["params"][0] == 6.0 and items[0]["params"][6] == 10.0
          and near(items[0]["params"][4], bs[0]["lat"], 1e-6)
          and len(js["buoys"]) == 2)
    return check("CSV, KML, QGC plan and JSON all parse", ok,
                 "%d csv rows, %d placemarks, %d waypoints"
                 % (len(csv_lines) - 1, len(marks), len(items)))


def case_export_empty():
    ok = True
    for fmt in map_export.FORMATS:
        try:
            map_export.render(fmt, [], "stem")
        except Exception:
            ok = False
    return check("an empty map still exports", ok)


# ============================================================== map_server

def case_server():
    bs = sample_buoys()
    srv = MapServer(lambda: {"stem": "s", "buoys": bs},
                    lambda fmt: (None if fmt not in map_export.FORMATS else
                                 map_export.render(fmt, bs, "s")
                                 + ("s_buoys.%s" % fmt,)))
    srv.start(0, "127.0.0.1")
    base = "http://127.0.0.1:%d" % srv.port
    try:
        st = json.loads(urllib.request.urlopen(base + "/state", timeout=3).read())
        r = urllib.request.urlopen(base + "/buoys.kml", timeout=3)
        disp = r.headers.get("Content-Disposition", "")
        r.read()
        missing = post_refused = False
        try:
            urllib.request.urlopen(base + "/buoys.exe", timeout=3)
        except urllib.error.HTTPError as e:
            missing = e.code == 404
        try:
            urllib.request.urlopen(urllib.request.Request(
                base + "/state", data=b"{}", method="POST"), timeout=3)
        except urllib.error.HTTPError as e:
            post_refused = e.code in (405, 501)
    finally:
        srv.stop()
    ok = (len(st["buoys"]) == 2 and "s_buoys.kml" in disp and missing
          and post_refused)
    return check("map server: GET only, downloads named", ok, disp)


# ===================================================== metadata on the stream

def case_meta_split_across_reads():
    meta = {"frame_idx": 7, "ros_time_ns": 123, "lat": LAT0, "gimbal_yaw": -3.5}
    head = (b"--uavframe\r\nContent-Type: image/jpeg\r\nContent-Length: 10\r\n"
            b"X-Frame-Meta: " + json.dumps(meta).encode() + b"\r\n\r\n")
    jpeg = b"\xff\xd8abcdef\xff\xd9"
    parts, rem = dc.split_parts(head)            # header arrives alone
    parts2, rem = dc.split_parts(rem + jpeg[:5])  # then half the image
    parts3, rem = dc.split_parts(rem + jpeg[5:] + b"\r\n")
    ok = (parts == [] and parts2 == [] and len(parts3) == 1
          and parts3[0][0] == meta and parts3[0][1] == jpeg)
    return check("metadata survives a header split from its JPEG", ok,
                 parts3 and parts3[0][0])


def case_meta_absent_or_bad():
    a, _ = dc.split_parts(b"--b\r\n\r\n\xff\xd8x\xff\xd9")
    b, _ = dc.split_parts(b"X-Frame-Meta: {not json}\r\n\r\n\xff\xd8x\xff\xd9")
    return check("no or bad metadata -> None, frame still kept",
                 a[0][0] is None and b[0][0] is None and a[0][1] and b[0][1])


def case_full_view():
    ok = (dc.full_view((100, 100, 200, 200), W, H, 12)
          and not dc.full_view((5, 100, 60, 150), W, H, 12)
          and not dc.full_view((1880, 500, 1915, 560), W, H, 12))
    return check("edge-touching boxes are not full view", ok)


def case_meta_end_to_end():
    """camera_node's FrameSlot -> the real MjpegServer -> detector's parser."""
    slot = FrameSlot()
    srv = MjpegServer(slot, lambda: {})
    srv.start(0, "127.0.0.1")
    port = srv._server.server_address[1]
    meta = {"frame_idx": 42, "ros_time_ns": 987654321, "gimbal_yaw": 12.5}
    jpeg = b"\xff\xd8" + b"\x11" * 64 + b"\xff\xd9"

    def producer():
        for _ in range(40):
            slot.put(jpeg, json.dumps(meta, separators=(",", ":")))
            time.sleep(0.02)
    threading.Thread(target=producer, daemon=True).start()
    got = None
    try:
        r = urllib.request.urlopen("http://127.0.0.1:%d/stream.mjpg" % port,
                                   timeout=3)
        buf = b""
        for _ in range(50):
            buf += r.read(256)
            parts, buf = dc.split_parts(buf)
            if parts:
                got = parts[0]
                break
        r.close()
    finally:
        srv.stop()
    return check("frame metadata crosses the real MJPEG server",
                 got is not None and got[0] == meta and got[1] == jpeg,
                 got and got[0])


def case_meta_newline_refused():
    try:
        FrameSlot().put(b"x", '{"a":1}\r\nX-Evil: 1')
    except ValueError:
        return check("metadata with a line break is refused", True)
    return check("metadata with a line break is refused", False)


# ======================================================= synthetic flight

TRUE_OFFSET_DEG = 7.0
TRUTH = [((0.0, 0.0), "flash"), ((3.0, 0.0), "solid")]   # (east, north) m


def synthetic_flight(seconds=24.0, hz=4.0, offset=TRUE_OFFSET_DEG, sign=1.0):
    """A flight that circles between two buoys 3 m apart while turning a full
    360, rendered into pixels with a camera heading that includes a hidden
    mount offset. The inverse of geolocate.ground_offset, written out
    independently so an error in one does not cancel in the other."""
    from uav_perception.geolocate import Projector as _P
    target_h = _P().target_height_m
    out = []
    for i in range(int(seconds * hz)):
        t = i / hz
        ax = 1.5 + 2.0 * math.sin(t / 4.0)
        ay = 1.2 * math.cos(t / 3.0)
        yaw_deg = (t / seconds) * 360.0
        g_yaw = 4.0 * math.sin(t)                    # follow-mode lag, body frame
        lat, lon = geo.xy_to_latlon(ax, ay, (LAT0, LON0))
        f = frame(frame_idx=i, lat=lat, lon=lon, yaw=math.radians(yaw_deg),
                  gimbal_yaw=g_yaw, gimbal_yaw_rate=4.0 * math.cos(t) * 57.3 / 10)
        heading = math.radians(yaw_deg + sign * g_yaw + offset)
        h = f["alt_rel"] - target_h
        dets = []
        for (bx, by), kind in TRUTH:
            dn, de = by - ay, bx - ax
            fwd = dn * math.cos(heading) + de * math.sin(heading)
            right = -dn * math.sin(heading) + de * math.cos(heading)
            u, v = W / 2 + F * right / h, H / 2 - F * fwd / h
            box = (u - 26, v - 26, u + 26, v + 26)
            cls = "blue" if kind == "solid" or int(t) % 2 == 0 else "dark"
            dets.append({"class_name": cls, "confidence": 0.9, "x0": box[0],
                         "y0": box[1], "x1": box[2], "y1": box[3],
                         "full_view": dc.full_view(box, W, H, 12)})
        out.append((t, f, dets, W, H))
    return out


def _proj_kw(**over):
    from uav_perception.mapping_core import PROJECTOR_KEYS
    base = Projector()
    kw = {k: getattr(base, k) for k in PROJECTOR_KEYS}
    kw.update(over)
    return kw


def _trk_kw():
    from uav_perception.mapping_core import TRACKER_KEYS
    base = TrackerConfig()
    return {k: getattr(base, k) for k in TRACKER_KEYS}


def case_synthetic_flight_maps_truth():
    from uav_perception.mapping_core import replay
    fl = synthetic_flight()
    _, good, _ = replay(fl, _proj_kw(mount_yaw_offset_deg=TRUE_OFFSET_DEG),
                        _trk_kw(), 0.35)
    _, bad, _ = replay(fl, _proj_kw(mount_yaw_offset_deg=0.0), _trk_kw(), 0.35)
    errs, labels = [], []
    for (bx, by), _kind in TRUTH:
        b = min(good, key=lambda b: math.hypot(*[a - c for a, c in zip(
            geo.latlon_to_xy(b["lat"], b["lon"], (LAT0, LON0)), (bx, by))]))
        x, y = geo.latlon_to_xy(b["lat"], b["lon"], (LAT0, LON0))
        errs.append(math.hypot(x - bx, y - by))
        labels.append(b["label"])
    good_spread = max(b["spread_m"] for b in good)
    bad_spread = max(b["spread_m"] for b in bad) if bad else 0.0
    ok = (len(good) == 2 and max(errs) < 0.15
          and labels == ["FLASHING_BLUE", "SOLID_BLUE"]
          and bad_spread > 3 * good_spread)
    return check("synthetic flight: right offset maps the truth", ok,
                 "err %.2f/%.2f m, spread %.2f vs %.2f m with offset 0"
                 % (errs[0], errs[1], good_spread, bad_spread))


def case_solve_recovers_offset():
    from uav_perception.mapping_core import solve_mount_yaw
    ranked = solve_mount_yaw(synthetic_flight(), _proj_kw(), _trk_kw(), 0.35,
                             offsets=range(-20, 21), signs=(1.0, -1.0))
    best = ranked[0]
    return check("solve-yaw recovers a hidden 7 deg offset and sign",
                 abs(best["mount_yaw_offset_deg"] - TRUE_OFFSET_DEG) <= 1.0
                 and best["gimbal_yaw_sign"] == 1.0,
                 "best %+.0f deg sign %+.0f spread %.2f m"
                 % (best["mount_yaw_offset_deg"], best["gimbal_yaw_sign"],
                    best["spread_m"]))


def case_replay_filenames():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "map_session", os.path.join(REPO, "tools", "scripts", "map_session.py"))
    ms = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ms)
    s = "20260912T232518Z"
    cases = [("00012953.jpg", s, 12953),
             ("0433_163358_232518Z_00012953.txt", s, 12953),
             ("0433_163358_232518Z_00012953_jpg.rf.ab12cd.txt", s, 12953),
             ("0500_165000_234911Z_00001234.txt", s, None)]   # other session
    bad = [(n, want, ms.label_frame_idx(n, sess)) for n, sess, want in cases
           if ms.label_frame_idx(n, sess) != want]
    return check("replay joins stills/labels to frames by name", not bad, bad)


def main():
    cases = [
        case_focal, case_centre_is_below, case_right_is_east_facing_north,
        case_up_is_forward, case_heading_east_rotates, case_tilt_forward,
        case_above_horizon_refused, case_heading_modes, case_gates,
        case_locate_latlon, case_launch_height,
        case_flashing, case_solid, case_off, case_red_green, case_too_short,
        case_gaps_not_watch_time, case_on_then_off, case_colour_disagreement,
        case_gate_buoys_separate, case_merge, case_stray_hidden,
        case_partial_and_low_conf, case_rejected_frame_counted, case_log_replays,
        case_exports_parse, case_export_empty, case_server,
        case_meta_split_across_reads, case_meta_absent_or_bad, case_full_view,
        case_meta_end_to_end, case_meta_newline_refused,
        case_synthetic_flight_maps_truth, case_solve_recovers_offset,
        case_replay_filenames,
    ]
    results = []
    for c in cases:
        try:
            results.append(c())
        except Exception as e:
            results.append(check(c.__name__, False, "raised %r" % e))
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
