"""Shared frame/geodesy helpers (no ROS imports — used by nodes AND off-board
tooling).

Conventions:
  WORLD: x = east+ [m], y = north+ [m], z = up+ [m], anchored at `origin`.
  heading/yaw: radians, 0 = true north, clockwise positive (compass convention).

Equirectangular approximation — fine at course scale (<5 km), and the same
111_139 m/deg the rest of the fleet's GIS math uses. Do not "improve" one
vehicle's constant in isolation: a target the USV reports and the UAV re-derives
must land in the same place.
"""
import math

M_PER_DEG = 111_139.0


def ground_speed_mps(vx_cms: float, vy_cms: float) -> float:
    """Horizontal ground speed [m/s] from GLOBAL_POSITION_INT's vx/vy.

    MAVLink reports those as int16 cm/s in the NED frame; the unit conversion
    lives here rather than inline in telemetry_bridge so it cannot be silently
    re-derived (wrongly) by a second consumer later.

    HORIZONTAL ONLY, and deliberately so: the RxReport heartbeat's `spd_mps` is
    a horizontal ground speed, and vertical motion is a different question with
    its own field. Use climb_rate_mps() for that — do not fold vz in here to
    make one "total speed" number that answers neither question.
    """
    return math.hypot(vx_cms, vy_cms) / 100.0


def climb_rate_mps(vz_cms: float) -> float:
    """Climb rate [m/s], POSITIVE UP, from GLOBAL_POSITION_INT's vz.

    MAVLink vz is int16 cm/s in NED, which is positive DOWN. The negation is the
    entire content of this function and the entire reason it exists as one:
    a sign flip re-derived at each call site is the kind of error that reads
    correctly, plots upside down, and is only noticed on a descent.
    """
    return -vz_cms / 100.0


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def latlon_to_xy(lat: float, lon: float, origin) -> tuple:
    lat0, lon0 = origin
    y = (lat - lat0) * M_PER_DEG
    x = (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0))
    return x, y


def xy_to_latlon(x: float, y: float, origin) -> tuple:
    lat0, lon0 = origin
    lat = lat0 + y / M_PER_DEG
    lon = lon0 + x / (M_PER_DEG * math.cos(math.radians(lat0)))
    return lat, lon


def point_in_polygon(lat: float, lon: float, polygon) -> bool:
    """Is (lat, lon) inside `polygon`? Ray casting, in degrees.

    `polygon` is a sequence of (lat, lon). It may be closed (first point ==
    last) or not — the algorithm walks edges pairwise with wraparound, so a
    repeated final point contributes a zero-length edge that crosses nothing.
    That matters because the geofence is stored CLOSED (the OCS declaration
    wants it that way) and stripped only on the way to the autopilot; this
    function has to accept both spellings without a caller remembering which.

    Degrees, not metres: the fence is small enough that the longitude
    convergence which latlon_to_xy corrects for cannot flip an inside/outside
    answer, and doing it here in the native units keeps this free of an origin.

    A point exactly on an edge is not promised either answer — floating point
    decides. This drives a display readout, not the autopilot's own fence, which
    is the authority and is enforced onboard.
    """
    pts = list(polygon)
    if len(pts) < 3:
        return False
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        lat_i, lon_i = pts[i][0], pts[i][1]
        lat_j, lon_j = pts[j][0], pts[j][1]
        if (lat_i > lat) != (lat_j > lat):
            # longitude of the edge at this latitude
            span = lat_j - lat_i
            if span != 0.0:
                x = lon_i + (lat - lat_i) * (lon_j - lon_i) / span
                if lon < x:
                    inside = not inside
        j = i
    return inside


def polygon_is_closed(polygon) -> bool:
    """First point identical to last. The OCS declaration requires this."""
    pts = list(polygon)
    return len(pts) >= 2 and tuple(pts[0]) == tuple(pts[-1])
