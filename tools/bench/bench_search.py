#!/usr/bin/env python3
"""bench_search — the buoy search's geometry and decisions, with no aircraft.

    python3 tools/bench/bench_search.py

Three parts:

  sweep_core   the fence is shrunk correctly, the lines cover every point of it,
               every waypoint stays inside, and the fences that cannot be
               searched are refused with a reason a pilot can act on.
  search_core  flown against a toy world: a point-mass aircraft that goes where
               it is told at the set speed, and a stand-in for buoy_mapper that
               puts a buoy on the map after a few sightings in the camera's
               footprint and locks it after enough time in view. The toy world
               is deliberately crude; what is checked is the RULES -- who may
               start it, what stops it, what counts, when it gives up, that it
               asks for RTL only in GUIDED -- not how well it flies.
  guided_gate  telemetry_bridge's last check on every target: each refusal
               fires, and an unchanged target is not re-sent at the publish rate.

The real flying is checked against ArduPilot's own simulator, outside this repo.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for pkg in ("uav_common", "uav_mission"):
    sys.path.insert(0, os.path.join(REPO, pkg))

from uav_common import geo, guided_gate  # noqa: E402
from uav_mission import search_core as core  # noqa: E402
from uav_mission import sweep_core as sc  # noqa: E402

HOME = (32.9244, -117.0201)


def check(name, passed, detail=""):
    print("%-52s %-4s %s" % (name, "PASS" if passed else "FAIL", str(detail)[:70]))
    return bool(passed)


def latlon_ring(xy):
    return [geo.xy_to_latlon(x, y, HOME) for x, y in xy]


RECT = [(-30.0, -20.0), (30.0, -20.0), (30.0, 20.0), (-30.0, 20.0)]


# ================================================================ sweep_core

def case_geometry():
    r = []
    hull, dent, problem = sc.fence_region(RECT)
    r.append(check("rectangle is a searchable fence", not problem and dent == 0.0,
                   problem))
    ins = sc.inset(hull, 2.0)
    xs, ys = [p[0] for p in ins], [p[1] for p in ins]
    r.append(check("2 m inset of 60x40 is 56x36",
                   abs(max(xs) - min(xs) - 56) < 1e-6 and abs(max(ys) - min(ys) - 36) < 1e-6,
                   "%.2f x %.2f" % (max(xs) - min(xs), max(ys) - min(ys))))
    theta = sc.best_angle(ins)
    r.append(check("lines run along the long side", abs(math.sin(theta)) < 1e-6,
                   "theta %.1f deg" % math.degrees(theta)))

    for shifted in (False, True):
        ls = sc.lines(ins, theta, 10.0, shifted)
        worst = 0.0
        for gx in range(-28, 29):
            for gy in range(-18, 19):
                d = min(sc._segment_distance(a, b, (gx, gy))[0] for a, b in ls)
                worst = max(worst, d)
        limit = 5.0 if not shifted else 4.6
        r.append(check("every point within half a spacing (%s)" %
                       ("shifted" if shifted else "centred"), worst <= limit + 0.02,
                       "%d lines, worst %.2f m" % (len(ls), worst)))

    wps = sc.waypoints(sc.lines(ins, theta, 10.0), (27.0, 17.0))
    r.append(check("first waypoint is the corner nearest the aircraft",
                   math.hypot(wps[0][0] - 28, wps[0][1] - 13.5) < 0.1, wps[0]))
    inside = all(sc.contains(hull, w) and sc.distance_to_edges(hull, w) >= 2.0 - 0.02
                 for w in wps)
    r.append(check("every waypoint 2 m inside the fence", inside))

    # A rotated, irregular convex fence: same guarantees.
    rot = [(math.cos(0.5) * x - math.sin(0.5) * y, math.sin(0.5) * x + math.cos(0.5) * y)
           for x, y in [(-35, -10), (20, -25), (40, 5), (10, 30), (-30, 20)]]
    hull, dent, problem = sc.fence_region(rot)
    ins = sc.inset(hull, 2.0)
    ok = not problem and ins
    for k in range(4):
        th, sh = sc.pass_geometry(k, sc.best_angle(ins))
        for w in sc.waypoints(sc.lines(ins, th, 10.0, sh), (0, 0)):
            ok = ok and sc.distance_to_edges(hull, w) >= 1.98 and sc.contains(hull, w)
    r.append(check("irregular fence: 4 passes, all waypoints inside", ok, problem))
    th0, _ = sc.pass_geometry(0, 0.3)
    th1, _ = sc.pass_geometry(1, 0.3)
    r.append(check("each pass turns the lines 90 deg",
                   abs(abs(th1 - th0) - math.pi / 2) < 1e-9))

    L = [(0, 0), (40, 0), (40, 40), (20, 40), (20, 15), (0, 15)]
    _h, _d, problem = sc.fence_region(L)
    r.append(check("L-shaped fence refused, vertex named",
                   "inward corner at vertex 5" in problem, problem))
    dented = [(0, 0), (20, 0.3), (40, 0), (40, 30), (0, 30)]
    hull, dent, problem = sc.fence_region(dented)
    r.append(check("a vertex 0.3 m off straight is tolerated",
                   not problem and abs(dent - 0.3) < 0.01, "dent %.2f" % dent))
    ins = sc.inset(hull, 2.0 + dent)
    r.append(check("...and the dent is added to the inset",
                   all(p[1] >= 2.3 - 1e-6 for p in ins)))
    _h, _d, problem = sc.fence_region([(0, 0), (30, 30), (30, 0), (0, 30)])
    r.append(check("self-crossing fence refused", "crosses over itself" in problem,
                   problem))
    hull, _d, problem = sc.fence_region([(0, 0), (3, 0), (3, 30), (0, 30)])
    r.append(check("3 m wide fence has nothing left after a 2 m inset",
                   not problem and sc.inset(hull, 2.0) == []))
    hull, _d, _p = sc.fence_region(RECT)
    ins = sc.inset(hull, 2.0)
    c = sc.clamp_inside(ins, (29.5, 0.0))
    r.append(check("buoy in the margin: hover from the nearest point inside",
                   sc.contains(ins, c) and abs(c[0] - 27.95) < 0.01, c))
    r.append(check("heading of lines along east is 090",
                   abs(sc.heading_deg(0.0) - 90.0) < 1e-9))
    return r


# ================================================================ toy world

class World:
    """A point-mass aircraft and a crude stand-in for buoy_mapper."""

    FOOT_ALONG, FOOT_ACROSS = 4.3, 8.0     # half-footprint at 10 m, metres

    def __init__(self, buoys, fence=RECT, cfg=None, count=6):
        self.fence = latlon_ring(fence)
        self.truth = [dict(id=i + 1, xy=xy, kind=kind, sightings=0, seen_s=0.0)
                      for i, (xy, kind) in enumerate(buoys)]
        self.xy, self.alt, self.heading = (-25.0, -15.0), 6.0, 0.0
        self.mode, self.armed, self.t = "LOITER", True, 0.0
        self.enabled, self.count = True, count
        self.map_live, self.fence_age = True, 5.0
        self.search = core.BuoySearch(cfg or core.SearchConfig())
        self.log, self.targets, self.rtl_t, self.speed = [], [], [], 0.0
        self.fence_params = dict(fence_enable=1.0, fence_type=5.0,
                                 fence_alt_max=12.0, fence_margin=2.0)
        self.last = None

    def map(self):
        if not self.map_live:
            return None
        out = []
        for b in self.truth:
            if b["sightings"] >= 3:
                lat, lon = geo.xy_to_latlon(b["xy"][0] + 0.2, b["xy"][1] - 0.1, HOME)
                locked = b["kind"] == "normal" and b["seen_s"] >= 6.0
                out.append(core.Buoy(b["id"], lat, lon, locked,
                                     "FLASHING_RED" if locked else "UNKNOWN"))
        return out

    def inputs(self):
        lat, lon = geo.xy_to_latlon(self.xy[0], self.xy[1], HOME)
        return core.Inputs(now=self.t, enabled=self.enabled, buoys_to_find=self.count,
                           pose=(lat, lon, self.alt, self.speed), mode=self.mode,
                           armed=self.armed, in_air=self.alt > 0.5,
                           fence=self.fence, fence_read_t=self.t - self.fence_age,
                           buoys=self.map(), **self.fence_params)

    def tick(self, dt=0.2):
        d = self.search.step(self.inputs())
        self.last = d
        self.log.extend(d.events)
        if d.request_rtl:
            self.rtl_t.append(self.t)
        tgt = d.target if self.mode == "GUIDED" else None
        self.speed = 0.0
        if tgt is not None:
            self.targets.append((self.t, tgt))
            gx, gy = geo.latlon_to_xy(tgt.lat, tgt.lon, HOME)
            dx, dy = gx - self.xy[0], gy - self.xy[1]
            dist = math.hypot(dx, dy)
            step = min(dist, tgt.speed_mps * dt)
            if dist > 1e-9:
                self.xy = (self.xy[0] + dx / dist * step, self.xy[1] + dy / dist * step)
                self.speed = step / dt
            self.alt += max(-dt, min(dt, tgt.alt_m - self.alt))
            if not math.isnan(tgt.yaw_deg):
                self.heading = tgt.yaw_deg
        h = math.radians(self.heading)
        fwd, right = (math.sin(h), math.cos(h)), (math.cos(h), -math.sin(h))
        for b in self.truth:
            rx, ry = b["xy"][0] - self.xy[0], b["xy"][1] - self.xy[1]
            along, across = rx * fwd[0] + ry * fwd[1], rx * right[0] + ry * right[1]
            if self.alt >= 3.0 and abs(along) <= self.FOOT_ALONG and abs(across) <= self.FOOT_ACROSS:
                b["sightings"] += 1
                b["seen_s"] += dt
        self.t += dt
        return d

    def run(self, seconds):
        end = self.t + seconds
        while self.t < end:
            self.tick()

    def phase(self):
        return self.last.status["phase"] if self.last else None


SIX = [((-20.0, 10.0), "normal"), ((-17.0, 10.0), "normal"), ((5.0, -12.0), "normal"),
       ((22.0, 14.0), "normal"), ((27.5, -17.5), "normal"), ((0.0, 3.0), "normal")]


def case_search():
    r = []

    w = World(SIX)
    w.mode = "GUIDED"                 # already in GUIDED when enabled
    w.run(5)
    r.append(check("in GUIDED already: nothing flies without a switch",
                   not w.targets and w.phase() == "ready", w.last.status["text"]))
    w.mode = "LOITER"
    w.run(1)
    w.mode = "GUIDED"                 # the pilot's switch
    w.run(600)
    alts = [t.alt_m for _t, t in w.targets]
    r.append(check("switch into GUIDED starts it, climbing first",
                   w.targets and w.targets[0][0] < 6.5
                   and any(e == "search started" for e in w.log)
                   and w.targets[0][1].alt_m == 10.0, "first target %.1f s"
                   % (w.targets[0][0] if w.targets else -1)))
    r.append(check("went back to the line after each buoy",
                   sum(1 for e in w.log if "spotted" in e) >= 4))
    r.append(check("all six found, then RTL asked for", w.rtl_t and
                   w.last.status["found"] == 6, "RTL at %.0f s" % (w.rtl_t[0] if w.rtl_t else -1)))
    r.append(check("every target at 10 m (or a hold)",
                   all(abs(a - 10.0) < 1e-6 for a in alts), set(round(a, 1) for a in alts)))
    hull = sc.fence_region(RECT)[0]
    worst = min(sc.distance_to_edges(hull, geo.latlon_to_xy(t.lat, t.lon, HOME))
                for _t, t in w.targets)
    r.append(check("no target closer than 2 m to the fence", worst >= 1.95,
                   "closest %.2f m" % worst))
    gate = [e for e in w.log if "joins the hover" in e]
    r.append(check("a 3 m gate pair is confirmed in one hover", gate, gate[:1]))
    retries = [b - a for a, b in zip(w.rtl_t, w.rtl_t[1:])]
    r.append(check("RTL re-asked every 3 s while still GUIDED",
                   len(w.rtl_t) >= 2 and all(abs(x - 3.0) < 0.21 for x in retries),
                   "%d asks" % len(w.rtl_t)))
    w.mode = "RTL"
    w.run(1)
    r.append(check("...and stops asking once the autopilot is in RTL",
                   w.phase() == "complete" and not w.last.request_rtl,
                   w.last.status["text"]))
    w.mode = "GUIDED"
    n = len(w.rtl_t)
    w.run(5)
    r.append(check("back in GUIDED after completion: no RTL, no sweep",
                   len(w.rtl_t) == n and w.phase() == "complete"))

    # A buoy that never locks: given up after 10 s overhead, the rest carry on.
    buoys = SIX[:5] + [((0.0, 3.0), "stubborn")]
    w = World(buoys, count=5)
    w.run(1)
    w.mode = "GUIDED"
    w.run(900)
    gave = [e for e in w.log if e.startswith("gave up on B6")]
    r.append(check("stubborn buoy given up on after 10 s", gave, gave[:1]))
    r.append(check("...and the other five still found -> RTL",
                   w.rtl_t and w.last.status["found"] == 5))
    w = World(buoys, count=6)
    w.run(1)
    w.mode = "GUIDED"
    w.run(1500)
    passes = [e for e in w.log if e.startswith("pass ") and "waypoints" in e]
    tries = [e for e in w.log if e.startswith("gave up on B6")]
    r.append(check("count not reachable: keeps searching, new passes",
                   len(passes) >= 3 and not w.rtl_t, "%d passes" % len(passes)))
    r.append(check("...and retries the stubborn buoy each pass",
                   len(tries) >= 2, "%d give-ups" % len(tries)))

    # Pilot takes over mid-sweep, and gives it back.
    w = World(SIX)
    w.run(1)
    w.mode = "GUIDED"
    w.run(40)
    wp_before = w.search.wp
    w.mode = "LOITER"
    n = len(w.targets)
    w.run(10)
    r.append(check("pilot flips SB to Loiter: paused, no targets",
                   w.phase() == "paused" and len(w.targets) == n, w.last.status["text"]))
    w.mode = "GUIDED"
    w.run(1)
    r.append(check("back to GUIDED: resumes where it was",
                   w.search.sub in core.FLYING and w.search.wp >= wp_before))

    # Switched off from the page while flying: hold, and the page cannot restart it.
    w = World(SIX)
    w.run(1)
    w.mode = "GUIDED"
    w.run(30)
    w.enabled = False
    w.run(1)
    hold = w.last.target
    here = geo.xy_to_latlon(w.xy[0], w.xy[1], HOME)
    r.append(check("switched off in flight: holds where it is",
                   hold is not None and math.isnan(hold.yaw_deg)
                   and abs(hold.lat - here[0]) < 2e-5, w.last.status["text"]))
    w.enabled = True
    w.run(5)
    r.append(check("switched back on: still holding until the pilot's switch",
                   w.phase() == "holding" and w.search.sub is None))
    w.mode = "LOITER"
    w.run(1)
    w.mode = "GUIDED"
    w.run(1)
    r.append(check("...SC off and on: resumes", w.search.sub in core.FLYING))

    # Buoy map lost in flight: hold; map back: resumes by itself.
    w = World(SIX)
    w.run(1)
    w.mode = "GUIDED"
    w.run(30)
    w.map_live = False
    w.run(3)
    r.append(check("buoy map stops: holds position", w.phase() == "holding",
                   w.last.status["text"]))
    w.map_live = True
    w.run(1)
    r.append(check("buoy map back: resumes without a switch",
                   w.search.sub in core.FLYING))

    # The autopilot's status goes stale for 2 s mid-flight: not the pilot.
    w = World(SIX)
    w.run(1)
    w.mode = "GUIDED"
    w.run(30)
    saved, w.mode = w.mode, None
    w.run(2)
    stalled = w.phase()
    w.mode = saved
    w.run(1)
    r.append(check("status stale for 2 s: pauses, then resumes by itself",
                   stalled == "paused" and w.search.sub in core.FLYING, stalled))
    w.mode = None
    w.run(1)
    w.mode = "LOITER"
    w.run(1)
    w.mode = "GUIDED"
    w.run(1)
    r.append(check("...but stale then LOITER then GUIDED is a switch: resumes",
                   w.search.sub in core.FLYING))
    w.mode = "LOITER"
    w.run(1)
    w.mode = None
    w.run(1)
    w.mode = "GUIDED"
    w.run(1)
    r.append(check("LOITER, stale, then GUIDED seen: counts as the switch",
                   w.search.sub in core.FLYING))

    # Refusals, in words.
    def reasons(**kw):
        w = World(SIX)
        for k, v in kw.items():
            if k in w.fence_params:
                w.fence_params[k] = v
            else:
                setattr(w, k, v)
        w.run(0.4)
        return w.last.status["waiting_for"]

    rs = reasons(fence_alt_max=10.0)
    r.append(check("FENCE_ALT_MAX 10, margin 2: refused (stops climb at 8)",
                   any("FENCE_ALT_MAX 10" in x for x in rs), rs[:1]))
    rs = reasons()
    r.append(check("FENCE_ALT_MAX 12, margin 2, 10 m search: allowed", rs == [], rs))
    rs = reasons(fence_enable=0.0)
    r.append(check("fence switched off: refused", any("FENCE_ENABLE 0" in x for x in rs)))
    rs = reasons(fence_type=1.0)
    r.append(check("FENCE_TYPE without polygon: refused",
                   any("does not include the polygon" in x for x in rs)))
    rs = reasons(fence_type=4.0, fence_alt_max=None)
    r.append(check("polygon only (no ceiling): allowed", rs == [], rs))
    rs = reasons(map_live=False)
    r.append(check("no buoy map: refused", any("buoy map" in x for x in rs)))
    rs = reasons(fence=latlon_ring([(0, 0), (40, 0), (40, 40), (20, 40), (20, 15), (0, 15)]))
    r.append(check("L-shaped fence: refused with the vertex",
                   any("vertex 5" in x for x in rs), rs[:1]))
    rs = reasons(fence=None)
    r.append(check("no fence read: refused", any("no fence" in x for x in rs)))

    w = World(SIX)
    w.fence_age = 200.0
    w.run(0.4)
    r.append(check("fence not read from the autopilot for 200 s: refused",
                   any("last read 200 s ago" in x
                       for x in w.last.status["waiting_for"]),
                   w.last.status["waiting_for"][:1]))
    w.fence_age = 5.0
    w.run(0.4)
    r.append(check("...and allowed again as soon as it is re-read",
                   w.last.status["waiting_for"] == []))

    w = World(SIX, count=2)
    for b in w.truth[:2]:
        b["sightings"], b["seen_s"] = 10, 10.0
    w.run(0.4)
    r.append(check("map already holds the count: refused",
                   any("already has 2" in x for x in w.last.status["waiting_for"])))

    # A buoy outside the fence is never flown to and never counted.
    w = World(SIX[:2] + [((34.0, 0.0), "normal")], count=3)
    w.truth[2]["sightings"], w.truth[2]["seen_s"] = 10, 10.0
    w.run(1)
    w.mode = "GUIDED"
    w.run(300)
    r.append(check("buoy outside the fence: not counted",
                   not w.rtl_t and w.last.status["found"] == 2,
                   "found %d" % w.last.status["found"]))

    # Disarm forgets the flight.
    w.armed = False
    w.mode = "LOITER"
    w.run(1)
    r.append(check("disarm resets the search", w.search.wps == [] and not w.search.flown))
    return r


# ================================================================ guided_gate

def case_gate():
    r = []
    fence = latlon_ring(RECT)
    lat, lon = geo.xy_to_latlon(0.0, 0.0, HOME)
    ok = dict(lat=lat, lon=lon, alt_m=10.0, speed_mps=2.0, age_s=0.1,
              status=("GUIDED", True), fence=fence, fence_type=5.0,
              fence_alt_max=12.0, fence_margin=2.0)

    def why(**kw):
        a = dict(ok)
        a.update(kw)
        return guided_gate.refusal(**a)

    r.append(check("gate: a good target passes", why() == "", why()))
    for name, kw, want in [
            ("gate: LOITER refused", dict(status=("LOITER", True)), "not GUIDED"),
            ("gate: status unknown refused", dict(status=None), "unknown"),
            ("gate: disarmed refused", dict(status=("GUIDED", False)), "not armed"),
            ("gate: 2 s old target refused", dict(age_s=2.0), "stale"),
            ("gate: no fence refused", dict(fence=None), "no valid fence"),
            ("gate: outside the fence refused",
             dict(lat=geo.xy_to_latlon(40.0, 0.0, HOME)[0],
                  lon=geo.xy_to_latlon(40.0, 0.0, HOME)[1]), "outside"),
            ("gate: 10.5 m under 12-2 refused", dict(alt_m=10.5), "FENCE_ALT_MAX"),
            ("gate: 1 m refused", dict(alt_m=1.0), "below"),
            ("gate: FENCE_TYPE unread refused", dict(fence_type=None), "FENCE_TYPE"),
            ("gate: NaN position refused", dict(lat=float("nan")), "blank"),
            ("gate: 12 m/s refused", dict(speed_mps=12.0), "speed")]:
        got = why(**kw)
        r.append(check(name, want in got, got))
    r.append(check("gate: polygon-only fence, 30 m allowed",
                   why(fence_type=4.0, fence_alt_max=None, alt_m=30.0) == ""))

    rs = guided_gate.Resender()
    t = 100.0
    first = rs.target_due(lat, lon, 10.0, 90.0, 1, t)
    rs.target_sent(lat, lon, 10.0, 90.0, 1, t)
    r.append(check("resend: first target sent, same one 1 s later not",
                   first and not rs.target_due(lat, lon, 10.0, 90.0, 1, t + 1.0)))
    lat2, lon2 = geo.xy_to_latlon(1.0, 0.0, HOME)
    r.append(check("resend: moved 1 m -> sent",
                   rs.target_due(lat2, lon2, 10.0, 90.0, 1, t + 1.0)))
    r.append(check("resend: new entry into GUIDED -> sent",
                   rs.target_due(lat, lon, 10.0, 90.0, 2, t + 1.0)))
    r.append(check("resend: unchanged after 5 s -> sent again",
                   rs.target_due(lat, lon, 10.0, 90.0, 1, t + 5.0)))
    r.append(check("resend: heading to no-heading (a hold) -> sent",
                   rs.target_due(lat, lon, 10.0, float("nan"), 1, t + 1.0)))
    return r


def main():
    results = case_geometry() + case_search() + case_gate()
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
