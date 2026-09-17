"""preflight_core — the pre-flight strip: each check, and the state it is in.

No ROS, no I/O. gcs_node gathers the inputs from its caches into one plain dict
and this turns them into a list the page draws as a row of chips on every tab.

EVERY CHECK HERE IS SOMETHING THAT HAS ALREADY COST A SORTIE, OR NEARLY:
  - a gimbal at the horizon after a battery swap, with gimbal_ok true;
  - a main-stream bitrate above 1570 kbps, which shredded every frame above 5 m;
  - telemetry_bridge down, so camera_node threw the session away as bench time;
  - FENCE_ALT_MAX at 10 m with the colour pass flown at 10 m;
  - a pack started half charged that came within 0.3 V of the failsafe.

States: "ok", "warn", "bad", "unknown" (the input has not arrived -- never
shown as ok), and "off" (not in use, e.g. the mapping nodes on a plain data
flight). The page shows every chip while disarmed, and only the ones that are
not ok once armed.

The thresholds are module constants, not parameters: they encode lessons, and a
tunable lesson is one that gets tuned away the day it is inconvenient.
"""
import math

#: Above this the A8 mini's encoder emits a broken bitstream (2026-09-11).
MAX_MAIN_KBPS = 1570
MAIN_RESOLUTION = (1920, 1080)
#: An encoding read older than this is shown as a warning, not trusted.
ENCODING_STALE_S = 180.0
#: How far off nadir the gimbal may sit and still count as pointing down.
NADIR_TOLERANCE_DEG = 5.0
#: GPS good enough to map on. 13 Sep mapped to 0.5 m on 23-26 satellites, HDOP
#: 0.57; these are the floor for "fine", not the target.
GPS_MIN_SATS = 10
GPS_MAX_HDOP = 1.2
#: Resting volts per cell before takeoff. 3.80 V/cell is roughly half charge,
#: which is where the 13 Sep pack started and nearly met the failsafe.
REST_OK_PER_CELL = 3.85
REST_BAD_PER_CELL = 3.70
#: While armed: loaded margin above BATT_LOW_VOLT.
FLY_OK_MARGIN_V = 0.6
FLY_BAD_MARGIN_V = 0.3
#: FENCE_TYPE bits (ArduPilot).
FENCE_TYPE_ALT_MAX, FENCE_TYPE_CIRCLE, FENCE_TYPE_POLYGON = 1, 2, 4

FIX_NAMES = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS",
             5: "RTK float", 6: "RTK fixed"}


def _num(x):
    return x is not None and not (isinstance(x, float) and math.isnan(x))


def _chip(key, label, state, detail):
    return {"key": key, "label": label, "state": state, "detail": detail}


def telemetry(inp):
    if inp.get("fcu_ok") and inp.get("pose_ok"):
        return _chip("tel", "telemetry", "ok", "autopilot status and position live")
    missing = [n for n, k in (("status", "fcu_ok"), ("position", "pose_ok"))
               if not inp.get(k)]
    return _chip("tel", "telemetry", "bad",
                 "no fresh %s from telemetry_bridge. Recording will be "
                 "discarded as bench time." % " or ".join(missing))


def gps(inp):
    g = inp.get("gps")
    if not g:
        return _chip("gps", "GPS", "unknown", "no GPS data yet")
    fix, sats, hdop = g.get("fix_type", 0), g.get("satellites", 255), g.get("hdop")
    name = FIX_NAMES.get(fix, "fix %d" % fix)
    parts = [name]
    if sats != 255:
        parts.append("%d sats" % sats)
    if _num(hdop):
        parts.append("HDOP %.2f" % hdop)
    detail = " · ".join(parts)
    if fix < 3:
        return _chip("gps", "GPS", "bad", detail + ". No 3D fix: do not fly a map.")
    if (sats != 255 and sats < GPS_MIN_SATS) or (_num(hdop) and hdop > GPS_MAX_HDOP):
        return _chip("gps", "GPS", "warn",
                     detail + ". Flyable, but buoy positions will be worse.")
    return _chip("gps", "GPS", "ok", detail)


def battery(inp):
    b = inp.get("battery")
    if not b:
        return _chip("batt", "battery", "unknown", "no battery reading yet")
    v, low, cells = b.get("voltage"), b.get("low_volt"), b.get("cells")
    if inp.get("armed"):
        if not _num(low):
            return _chip("batt", "battery", "unknown",
                         "%.2f V; BATT_LOW_VOLT not read yet" % v)
        margin = v - low
        state = ("ok" if margin >= FLY_OK_MARGIN_V else
                 "warn" if margin >= FLY_BAD_MARGIN_V else "bad")
        return _chip("batt", "battery", state,
                     "%.2f V, %.2f V above the %.1f V failsafe" % (v, margin, low))
    if not cells:
        return _chip("batt", "battery", "unknown", "%.2f V, cell count unknown" % v)
    per = v / cells
    state = ("ok" if per >= REST_OK_PER_CELL else
             "warn" if per >= REST_BAD_PER_CELL else "bad")
    detail = "%.2f V resting = %.2f V/cell (%dS)" % (v, per, cells)
    if state != "ok":
        detail += ". Below about 60%: expect a short flight."
    return _chip("batt", "battery", state, detail)


def fence(inp):
    """The fence the working altitude is flown under.

    The number that matters is not FENCE_ALT_MAX but FENCE_ALT_MAX minus
    FENCE_MARGIN: that is where the autopilot's avoidance stops a climb in
    Loiter, and the buoy search refuses to fly above it. Chris flies 12 m and a
    2 m margin for a 10 m pass on purpose -- a stick climb stops at exactly the
    working altitude -- so that is ok, and 10 m with a 2 m margin is bad.
    """
    enable, ceiling = inp.get("fence_enable"), inp.get("fence_alt_max")
    margin, ftype = inp.get("fence_margin"), inp.get("fence_type")
    work = inp.get("working_alt_m")
    if not _num(enable) or not _num(ceiling):
        return _chip("fence", "fence", "unknown",
                     "FENCE_ENABLE / FENCE_ALT_MAX not read from the autopilot yet")
    if enable < 0.5:
        return _chip("fence", "fence", "warn", "FENCE_ENABLE is 0: no fence at all")
    bits = int(ftype) if _num(ftype) else None
    stop = ceiling - (margin if _num(margin) else 0.0)
    if (bits is None or bits & FENCE_TYPE_ALT_MAX) and _num(work):
        if ceiling < work:
            return _chip("fence", "fence", "bad",
                         "ceiling %.0f m is under the %.0f m working altitude: the "
                         "fence will act mid-pass" % (ceiling, work))
        if stop < work - 0.05:
            return _chip("fence", "fence", "bad",
                         "climbs stop at %.0f m (FENCE_ALT_MAX %.0f - FENCE_MARGIN "
                         "%.0f), under the %.0f m working altitude. The search "
                         "will refuse to start." % (stop, ceiling, margin or 0, work))
    if bits is not None and not bits & FENCE_TYPE_POLYGON:
        return _chip("fence", "fence", "warn",
                     "FENCE_TYPE %d has no polygon: nothing stops it sideways, and "
                     "the search needs one" % bits)
    if bits is not None and bits & FENCE_TYPE_CIRCLE:
        return _chip("fence", "fence", "warn",
                     "FENCE_TYPE %d includes the circle: FENCE_RADIUS around home "
                     "triggers too, not just the polygon" % bits)
    return _chip("fence", "fence", "ok",
                 "enabled, ceiling %.0f m, climbs stop at %.0f m" % (ceiling, stop))


def gimbal(inp):
    c = inp.get("camera") or {}
    if not c.get("running"):
        return _chip("gimbal", "gimbal", "unknown", "camera_node not running")
    pitch, nadir = c.get("gimbal_pitch"), c.get("nadir_pitch", -90.0)
    if not c.get("gimbal_ok") or not _num(pitch):
        return _chip("gimbal", "gimbal", "bad",
                     "gimbal not answering. A camera power cycle drops the link: "
                     "restart camera_node once the camera has booted.")
    if abs(pitch - nadir) > NADIR_TOLERANCE_DEG:
        return _chip("gimbal", "gimbal", "bad",
                     "pitch %+.0f, not at nadir (%+.0f). A battery swap returns "
                     "it to the horizon." % (pitch, nadir))
    return _chip("gimbal", "gimbal", "ok", "pitch %+.1f (nadir)" % pitch)


def stream(inp):
    c = inp.get("camera") or {}
    if not c.get("running"):
        return _chip("stream", "stream", "unknown", "camera_node not running")
    codec, age = c.get("codec") or "", c.get("encoding_age_s")
    if not codec or not _num(age):
        return _chip("stream", "stream", "unknown",
                     "encoding not read from the camera yet (it drops about one "
                     "query in three)")
    w, h, kbps = c.get("width"), c.get("height"), c.get("kbps")
    detail = "%s %dx%d @ %d kbps" % (codec, w, h, kbps)
    want = (c.get("expected_codec") or "").upper()
    if kbps > MAX_MAIN_KBPS:
        return _chip("stream", "stream", "bad", detail + ". Above %d kbps the camera "
                     "corrupts every frame above ~5 m." % MAX_MAIN_KBPS)
    if want and codec.upper() != want:
        return _chip("stream", "stream", "bad", detail + ". camera_node is set up "
                     "for %s; this decodes to white frames." % want)
    if (w, h) != MAIN_RESOLUTION:
        return _chip("stream", "stream", "warn", detail + ". Not 1080p: the camera "
                     "has reverted before.")
    if age > ENCODING_STALE_S:
        return _chip("stream", "stream", "warn", detail + " (read %.0f s ago)" % age)
    return _chip("stream", "stream", "ok",
                 detail + ". A settings read: preflight_camera.sh is still the gate.")


def recording(inp):
    gate = inp.get("record_gate")
    if gate is None:
        return _chip("rec", "recording", "unknown", "no camera status")
    if "no FCU status" in gate:
        return _chip("rec", "recording", "bad", gate)
    if gate.startswith("keeping"):
        return _chip("rec", "recording", "ok", gate)
    return _chip("rec", "recording", "ok",
                 "%s -- kept automatically once armed" % gate if gate else
                 "no session open")


def mapping(inp):
    m = inp.get("mapping") or {}
    det, mapper = m.get("detector"), m.get("mapper")
    if det and mapper:
        return _chip("map", "mapping", "ok", "detector_node and buoy_mapper running")
    if det or mapper:
        return _chip("map", "mapping", "warn", "only %s running: start both to map"
                     % ("detector_node" if det else "buoy_mapper"))
    return _chip("map", "mapping", "off", "not started (not needed for a data flight)")


CHECKS = (telemetry, gps, battery, fence, gimbal, stream, recording, mapping)


def checks(inp):
    """-> list of chips, in the order the page shows them."""
    return [fn(inp) for fn in CHECKS]
