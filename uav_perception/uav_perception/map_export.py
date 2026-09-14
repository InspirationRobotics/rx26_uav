"""map_export — the buoy map as files people and other vehicles can open.

PURE formatting, plus one atomic writer. Four formats, one per consumer:

  CSV   a spreadsheet: one row per buoy, every piece of evidence beside the answer
  KML   Google Earth / Google My Maps: pins on satellite imagery, the fastest way
        to see whether a map is plausible while standing in the field
  PLAN  QGroundControl mission: one waypoint per buoy at waypoint_alt_m with a
        hover of hold_s. ArduPilot flies it as-is -- Ekko's hover-over-each-buoy
        pass, or Crusader's Pixhawk heading for the gates
  JSON  the same map as data, for Crusader's ROS 2 stack or anything else that
        wants to parse rather than look

Every file carries the map's evidence (spread, sightings, lit share), not just
positions: a pin with a 3 m spread and a pin with a 0.3 m spread must not look
equally trustworthy to whoever opens the file later.
"""
import csv
import io
import json
import os

CSV_FIELDS = ("id", "label", "state", "colour", "lat", "lon", "spread_m",
              "sightings", "samples", "observed_s", "lit_fraction",
              "colour_agreement", "flash_transitions", "locked")

# KML colours are aabbggrr. Grey for anything without a decided colour, so an
# UNKNOWN pin never reads as an OFF one.
_KML_COLOUR = {"RED": "ff3232ff", "GREEN": "ff32c832", "BLUE": "ffff8c1e",
               "OFF": "ff202020", "UNKNOWN": "ff9a9a9a"}

# MAVLink: MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT, ArduPilot, quad.
_NAV_WAYPOINT = 16
_FRAME_REL_ALT = 3
_FIRMWARE_ARDUPILOT = 3
_VEHICLE_QUAD = 2


def _row(b):
    return {
        "id": b["id"], "label": b["label"], "state": b["state"],
        "colour": b["colour"], "lat": "%.7f" % b["lat"], "lon": "%.7f" % b["lon"],
        "spread_m": "%.2f" % b["spread_m"], "sightings": b["sightings"],
        "samples": b["samples"], "observed_s": "%.1f" % b["observed_s"],
        "lit_fraction": "%.2f" % b["lit_fraction"],
        "colour_agreement": "%.2f" % b["colour_agreement"],
        "flash_transitions": b["flash_transitions"],
        "locked": "yes" if b["locked"] else "no",
    }


def to_csv(buoys) -> str:
    out = io.StringIO()
    w = csv.DictWriter(out, fieldnames=CSV_FIELDS, lineterminator="\n")
    w.writeheader()
    for b in buoys:
        w.writerow(_row(b))
    return out.getvalue()


def to_json(buoys, stem="") -> str:
    return json.dumps({"stem": stem, "buoys": list(buoys)}, indent=2) + "\n"


def _xml(s) -> str:
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def to_kml(buoys, stem="") -> str:
    styles = "".join(
        '<Style id="%s"><IconStyle><color>%s</color><scale>1.1</scale>'
        '<Icon><href>http://maps.google.com/mapfiles/kml/shapes/'
        'placemark_circle.png</href></Icon></IconStyle></Style>' % (k, v)
        for k, v in _KML_COLOUR.items())
    marks = []
    for b in buoys:
        style = b["colour"] or ("OFF" if b["state"] == "OFF" else "UNKNOWN")
        desc = ("state %s<br/>spread %.2f m from %d sightings<br/>"
                "lit %.0f%% of %d samples over %.1f s, %d flash changes"
                % (b["label"], b["spread_m"], b["sightings"],
                   100.0 * b["lit_fraction"], b["samples"], b["observed_s"],
                   b["flash_transitions"]))
        marks.append(
            "<Placemark><name>B%d %s</name><styleUrl>#%s</styleUrl>"
            "<description><![CDATA[%s]]></description>"
            "<Point><coordinates>%.7f,%.7f,0</coordinates></Point></Placemark>"
            % (b["id"], _xml(b["label"]), style, desc, b["lon"], b["lat"]))
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
            "<name>Ekko buoy map %s</name>%s%s</Document></kml>\n"
            % (_xml(stem), styles, "".join(marks)))


def to_qgc_plan(buoys, *, alt_m=10.0, hold_s=6.0, cruise_mps=3.0,
                hover_mps=1.5) -> str:
    """A QGroundControl .plan: one waypoint per buoy, in id order.

    Id order is discovery order, which is a reasonable flight order for a pass
    that found them. The planned home is the first buoy -- QGC needs one, and a
    home far from the buoys would draw the first leg from the wrong place.
    """
    items = []
    for i, b in enumerate(sorted(buoys, key=lambda b: b["id"]), start=1):
        items.append({
            "AMSLAltAboveTerrain": None, "Altitude": alt_m, "AltitudeMode": 1,
            "autoContinue": True, "command": _NAV_WAYPOINT, "doJumpId": i,
            "frame": _FRAME_REL_ALT,
            "params": [hold_s, 0, 0, None, round(b["lat"], 7),
                       round(b["lon"], 7), alt_m],
            "type": "SimpleItem"})
    home = ([round(buoys[0]["lat"], 7), round(buoys[0]["lon"], 7), 0]
            if buoys else [0, 0, 0])
    plan = {
        "fileType": "Plan", "groundStation": "QGroundControl", "version": 1,
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {"cruiseSpeed": cruise_mps, "hoverSpeed": hover_mps,
                    "firmwareType": _FIRMWARE_ARDUPILOT,
                    "vehicleType": _VEHICLE_QUAD, "version": 2,
                    "globalPlanAltitudeMode": 1,
                    "plannedHomePosition": home, "items": items},
    }
    return json.dumps(plan, indent=2) + "\n"


FORMATS = {
    "csv": ("text/csv", lambda b, stem, o: to_csv(b)),
    "kml": ("application/vnd.google-earth.kml+xml",
            lambda b, stem, o: to_kml(b, stem)),
    "plan": ("application/json", lambda b, stem, o: to_qgc_plan(b, **o)),
    "json": ("application/json", lambda b, stem, o: to_json(b, stem)),
}


def render(fmt, buoys, stem="", plan_opts=None):
    """-> (content_type, text). `fmt` is a FORMATS key."""
    ctype, fn = FORMATS[fmt]
    return ctype, fn(buoys, stem, plan_opts or {})


def write_all(directory, stem, buoys, plan_opts=None):
    """Write <stem>_buoys.<fmt> for every format. Returns the paths written.

    Atomic per file (write .part, then rename), because these are downloaded
    over WiFi while the map is still updating, and a half-written KML opens as
    an error rather than as the previous, complete map.
    """
    os.makedirs(directory, exist_ok=True)
    paths = []
    for fmt in FORMATS:
        _, text = render(fmt, buoys, stem, plan_opts)
        path = os.path.join(directory, "%s_buoys.%s" % (stem, fmt))
        tmp = path + ".part"
        with open(tmp, "w", encoding="utf-8", newline="") as f:
            f.write(text)
        os.replace(tmp, path)
        paths.append(path)
    return paths
