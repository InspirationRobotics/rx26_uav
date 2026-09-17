"""sweep_core — where a buoy search flies: the fence, shrunk, filled with lines.

PURE: no ROS, no clock, no I/O. Local metres, x east / y north, about an origin
the caller picks (uav_common.geo conventions). bench_search drives all of it.

THE FENCE MUST BE CONVEX, and that is a deliberate limit rather than a missing
feature. Inside a convex polygon the straight line between any two points stays
inside, so every leg this search flies -- line to line, sweep to buoy, buoy back
to the line -- is inside the fence by construction, with no path planner to get
wrong. A fence with a real inward corner is refused with the vertex named, and
the pilot draws it again without one; that costs a minute at the field, where a
planner bug costs an RTL from a fence breach. (A vertex dragged a few tens of
centimetres off straight is tolerated -- see MAX_DENT_M.)

THE LINES. Spaced no wider than `spacing`, run parallel to the fence edge that
makes the area narrowest (fewest turns), and centred across the shrunk fence so
the camera's swath is not spent on ground outside it. Each pass after the first
turns the lines 90 degrees, and every other pair of passes shifts them half a
spacing, so a buoy missed from one geometry is looked at from another.
"""
import math

from uav_common import geo

#: How far a hand-placed vertex may sit INSIDE the line between its neighbours
#: and still be treated as straight. QGC vertices are dragged by eye, and a point
#: 20 cm off a straight edge is not an inward corner anyone meant. The dent is
#: added to the inset, so the search still keeps its full distance from the real
#: boundary.
MAX_DENT_M = 0.75


def centroid(points):
    """Mean of the vertices. For a fence this is the local-frame origin."""
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


def to_local(polygon_latlon, origin):
    return [geo.latlon_to_xy(lat, lon, origin) for lat, lon in polygon_latlon]


def to_latlon(points_xy, origin):
    return [geo.xy_to_latlon(x, y, origin) for x, y in points_xy]


def open_ring(points):
    """Drop a repeated closing vertex, if there is one."""
    pts = [tuple(p) for p in points]
    return pts[:-1] if len(pts) > 1 and pts[0] == pts[-1] else pts


def signed_area(pts):
    """Positive for counter-clockwise."""
    return 0.5 * sum(pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1]
                     for i in range(len(pts)))


def ccw(pts):
    """The same polygon, counter-clockwise."""
    return list(pts) if signed_area(pts) >= 0 else list(reversed(pts))


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(pts):
    """CCW convex hull (monotone chain), collinear points dropped."""
    p = sorted(set((float(a), float(b)) for a, b in pts))
    if len(p) < 3:
        return p

    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and _cross(h[-2], h[-1], q) <= 0:
                h.pop()
            h.append(q)
        return h
    lower, upper = half(p), half(reversed(p))
    return lower[:-1] + upper[:-1]


def _segments_cross(a, b, c, d):
    """Proper crossing of segments ab and cd (touching at an end is not one)."""
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def fence_region(pts):
    """The fence (open ring, any winding) as something to plan inside.

    -> (hull, dent_m, problem). `hull` is the CCW convex hull and `dent_m` how
    far the real fence dips inside it; plan inside inset(hull, inset + dent_m)
    and every point is at least `inset` inside the real fence. problem is "" or,
    in words for the pilot, why this fence cannot be searched. Vertex numbers are
    1-based, the way QGC lists them.
    """
    pts = open_ring(pts)
    if len(pts) < 3:
        return [], 0.0, "the fence has %d corners; it needs at least 3" % len(pts)
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue                     # neighbours share a vertex
            if _segments_cross(pts[i], pts[(i + 1) % n], pts[j], pts[(j + 1) % n]):
                return [], 0.0, "the fence crosses over itself. Re-draw it in QGC."
    hull = convex_hull(pts)
    if len(hull) < 3 or abs(signed_area(hull)) < 1.0:
        return [], 0.0, "the fence encloses no area"
    dent, worst = 0.0, None
    for i, p in enumerate(pts):
        if contains(hull, p, eps=1e-9) and p not in hull:
            d = distance_to_edges(hull, p)
            if d > dent:
                dent, worst = d, i
    if dent > MAX_DENT_M:
        return [], dent, ("the fence has an inward corner at vertex %d (%.1f m). "
                          "Move that point outward in QGC so every corner points "
                          "out." % (worst + 1, dent))
    return hull, dent, ""


def _clip(poly, a, b, d):
    """Keep the part of convex `poly` at least `d` to the LEFT of edge a->b."""
    ex, ey = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return poly
    nx, ny = -ey / length, ex / length            # inward normal of a CCW edge

    def depth(p):
        return (p[0] - a[0]) * nx + (p[1] - a[1]) * ny - d

    out = []
    for i in range(len(poly)):
        p, q = poly[i - 1], poly[i]
        dp, dq = depth(p), depth(q)
        if dq >= 0:
            if dp < 0:
                t = dp / (dp - dq)
                out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
            out.append(q)
        elif dp >= 0:
            t = dp / (dp - dq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def inset(pts, d):
    """Convex `pts` with every edge moved `d` metres inward. [] if nothing is left.

    Clipping by each moved edge in turn, not moving the vertices: a short edge
    that vanishes at this inset simply stops contributing, where a vertex offset
    would fold the polygon over itself.
    """
    poly = ccw(pts)
    edges = [(poly[i - 1], poly[i]) for i in range(len(poly))]
    out = list(poly)
    for a, b in edges:
        out = _clip(out, a, b, d)
        if len(out) < 3:
            return []
    return out if abs(signed_area(out)) > 0.01 else []


def contains(poly, p, eps=1e-6):
    """Is p inside (or on) CCW convex `poly`."""
    return all(_cross(poly[i - 1], poly[i], p) >= -eps for i in range(len(poly)))


def distance_to_edges(poly, p):
    """Shortest distance from p to the polygon's boundary."""
    return min(_segment_distance(poly[i - 1], poly[i], p)[0]
               for i in range(len(poly)))


def _segment_distance(a, b, p):
    ex, ey = b[0] - a[0], b[1] - a[1]
    L2 = ex * ex + ey * ey
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * ex
                                                   + (p[1] - a[1]) * ey) / L2))
    q = (a[0] + t * ex, a[1] + t * ey)
    return math.hypot(p[0] - q[0], p[1] - q[1]), q


def clamp_inside(poly, p, nudge=0.05):
    """p if it is inside CCW convex `poly`, else the nearest point inside.

    A buoy between the shrunk fence and the real one is hovered over from the
    nearest point the search may fly to. At 10 m the camera sees 8 m either side,
    so a 2 m inset still puts that buoy well inside the frame.
    """
    if contains(poly, p):
        return (p[0], p[1])
    best = min((_segment_distance(poly[i - 1], poly[i], p)
                for i in range(len(poly))), key=lambda r: r[0])[1]
    c = centroid(poly)
    k = math.hypot(c[0] - best[0], c[1] - best[1])
    if k < 1e-9:
        return best
    return (best[0] + nudge * (c[0] - best[0]) / k,
            best[1] + nudge * (c[1] - best[1]) / k)


def _uv(p, theta):
    c, s = math.cos(theta), math.sin(theta)
    return (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)


def _xy(u, v, theta):
    c, s = math.cos(theta), math.sin(theta)
    return (u * c - v * s, u * s + v * c)


def width_across(poly, theta):
    vs = [_uv(p, theta)[1] for p in poly]
    return max(vs) - min(vs)


def best_angle(poly):
    """Direction (radians from east, CCW) of the lines: along the fence edge
    across which the area is narrowest, which is the fewest lines."""
    best = None
    for i in range(len(poly)):
        a, b = poly[i - 1], poly[i]
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6:
            continue
        theta = math.atan2(b[1] - a[1], b[0] - a[0]) % math.pi
        w = width_across(poly, theta)
        if best is None or w < best[0] - 1e-6:
            best = (w, theta)
    return 0.0 if best is None else best[1]


def pass_geometry(pass_index, base_theta):
    """(theta, shifted) for pass 0, 1, 2 ...: turn 90 degrees every pass, shift
    half a spacing every other pair."""
    theta = (base_theta + (math.pi / 2 if pass_index % 2 else 0.0)) % math.pi
    return theta, bool((pass_index // 2) % 2)


def lines(poly, theta, spacing, shifted=False):
    """Sweep lines across CCW convex `poly`: [(start, end), ...] by offset.

    Centred: n lines at the middles of n equal strips, n the fewest that keeps
    the gap under `spacing`. Shifted: the n+1 strip EDGES instead, which puts a
    line along each side of the shrunk fence and every other line between the
    centred ones.
    """
    uv = [_uv(p, theta) for p in poly]
    vmin, vmax = min(v for _, v in uv), max(v for _, v in uv)
    width = vmax - vmin
    n = max(1, int(math.ceil(width / spacing - 1e-9)))
    step = width / n
    if shifted:
        offsets = [vmin + i * step for i in range(n + 1)]
    else:
        offsets = [vmin + (i + 0.5) * step for i in range(n)]
    # A hair inside, so a line exactly on a vertex still meets two edges.
    eps = min(0.01, width / 4)
    out = []
    for v in offsets:
        v = max(vmin + eps, min(vmax - eps, v))
        us = []
        for i in range(len(uv)):
            (u1, v1), (u2, v2) = uv[i - 1], uv[i]
            if (v1 - v) * (v2 - v) <= 0 and abs(v2 - v1) > 1e-12:
                us.append(u1 + (v - v1) * (u2 - u1) / (v2 - v1))
        if len(us) >= 2:
            out.append((_xy(min(us), v, theta), _xy(max(us), v, theta)))
    return out


def waypoints(line_list, start):
    """Line ends in flying order, back and forth, starting at whichever of the
    four outer corners is nearest `start`."""
    if not line_list:
        return []
    options = []
    for order in (line_list, list(reversed(line_list))):
        for flip in (False, True):
            wps = []
            for i, (a, b) in enumerate(order):
                fwd = (i % 2 == 0) != flip
                wps.extend([a, b] if fwd else [b, a])
            d = math.hypot(wps[0][0] - start[0], wps[0][1] - start[1])
            options.append((d, wps))
    return min(options, key=lambda o: o[0])[1]


def heading_deg(theta):
    """Compass heading of the +u direction (0 = north, clockwise)."""
    return (90.0 - math.degrees(theta)) % 360.0
