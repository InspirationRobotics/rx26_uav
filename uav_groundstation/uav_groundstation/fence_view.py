"""fence_view -- what the autopilot is holding the aircraft inside, sideways.

PURE (no ROS): gcs_node and tools/scripts/gcs_radio.py both judge "inside the
fence" with it, and bench_preflight checks it with hand-written numbers.

ArduPilot's FENCE_TYPE is a bitmask: 1 the ceiling, 2 a CIRCLE of FENCE_RADIUS
around HOME, 4 the POLYGON uploaded over the mission protocol. A bit binds only
when its fence exists -- the polygon bit with no polygon uploaded holds nothing
-- and that is how Ekko was set on 6 Oct 2026: FENCE_TYPE 7, no polygon, a 35 m
circle. Before this, the page knew only polygons and fell back to the params
stand-in, a fence on the other side of the world, so it said "outside" of an
aircraft parked 6 m from home.

The view is what is ENFORCED, and a position is inside only when it is inside
every limit in it. When a limit is switched on but cannot be placed yet (the
radius or home not read back), there is no view at all: a judgement against
half a fence would be a guess.
"""
import math

from uav_common import geo

FENCE_TYPE_CIRCLE = 2
FENCE_TYPE_POLYGON = 4


def _num(x):
    return x if isinstance(x, (int, float)) and not math.isnan(x) else None


def view(params, polygon, home):
    """The horizontal fence the autopilot enforces, or None if not known.

    params:  {"fence_enable", "fence_type", "fence_radius"}; None/NaN = unread.
    polygon: [(lat, lon)] READ BACK from the autopilot, or None/[] for none.
    home:    (lat, lon) or None.

    -> {"circle": (lat, lon, radius_m) or None,
        "polygon": [(lat, lon)] or None,
        "desc": "circle 35 m around home" / "polygon" / "polygon + circle ..."}
       or None when the fence is off, unread, holds nothing sideways, or has a
       circle whose radius or centre is not known yet.
    """
    enable, bits = _num(params.get("fence_enable")), _num(params.get("fence_type"))
    if enable is None or bits is None or enable < 0.5:
        return None
    bits = int(bits)
    circle = None
    if bits & FENCE_TYPE_CIRCLE:
        radius = _num(params.get("fence_radius"))
        if radius is None or home is None:
            return None
        if radius > 0:
            circle = (home[0], home[1], float(radius))
    poly = list(polygon) if bits & FENCE_TYPE_POLYGON and polygon else None
    if circle is None and poly is None:
        return None
    parts = (["polygon"] if poly else []) + (
        ["circle %.0f m around home" % circle[2]] if circle else [])
    return {"circle": circle, "polygon": poly, "desc": " + ".join(parts)}


def inside(fence, lat, lon):
    """True or False against every limit in `fence` (a view()), or None when
    there is no fence to judge by or no position."""
    if fence is None or lat is None or lon is None:
        return None
    if fence["circle"]:
        clat, clon, r = fence["circle"]
        if math.hypot(*geo.latlon_to_xy(lat, lon, (clat, clon))) > r:
            return False
    if fence["polygon"] and not geo.point_in_polygon(lat, lon, fence["polygon"]):
        return False
    return True


def circle_xy(fence, origin):
    """The circle in a map's local metres: {"x", "y", "r"}, or None."""
    if fence is None or not fence["circle"] or origin is None:
        return None
    clat, clon, r = fence["circle"]
    x, y = geo.latlon_to_xy(clat, clon, origin)
    return {"x": x, "y": y, "r": r}
