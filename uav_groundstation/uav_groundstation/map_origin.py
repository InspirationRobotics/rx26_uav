"""map_origin — where the Map tab's local metres are measured from.

The map is centred on THE FENCE THE AUTOPILOT HOLDS once telemetry_bridge has
read it back; until then on the `geofence` param. That param is a stand-in (the
Singapore test course), so far from it the map is centred on the first REAL
position heard instead, Ekko's or Crusader's, whichever comes first.

0, 0 IS NOT A POSITION. ArduPilot reports latitude 0, longitude 0 until it has
one, and on a cold boot that arrives before any GPS fix. The ground station once
took it as "far from the params fence, centre here", and with no fence in the
autopilot to take over, the map stayed centred in the Gulf of Guinea: Ekko and
Crusader drawn 13,000 km away for the rest of the session. A blank, not a guess.

No ROS here; gcs_node feeds it and bench_gcs drives it.
"""
import math

from uav_common import geo

#: Farther than this from the params fence, it is not this venue's fence.
FOREIGN_ORIGIN_M = 50_000.0

PARAMS = "params"
FAR = "params (far away)"
AUTOPILOT = "autopilot"


def centroid(polygon):
    """Mean vertex of a polygon given open or closed; (0, 0) if empty."""
    pts = list(polygon)
    if len(pts) > 1 and tuple(pts[0]) == tuple(pts[-1]):
        pts = pts[:-1]
    if not pts:
        return (0.0, 0.0)
    return (sum(a for a, _ in pts) / len(pts), sum(b for _, b in pts) / len(pts))


def is_fix(lat, lon):
    """A real position: not ArduPilot's 0, 0 for "none yet", and not NaN."""
    if lat is None or lon is None or math.isnan(lat) or math.isnan(lon):
        return False
    return not (lat == 0.0 and lon == 0.0)


def recentre(src, origin, lat, lon):
    """Should a position just heard re-centre the map?

    -> (new src, new origin), or None to leave the map where it is. Only while
    the params fence is all there is and this position is far from it; never on
    a position that is not a fix, and never once the autopilot's fence is read.
    """
    if src != PARAMS or not is_fix(lat, lon):
        return None
    if math.hypot(*geo.latlon_to_xy(lat, lon, origin)) <= FOREIGN_ORIGIN_M:
        return None
    return FAR, (lat, lon)
