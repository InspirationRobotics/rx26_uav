"""search_core — the Task 1 buoy search as a state machine.

PURE: no ROS, no clock of its own, no I/O. search_node feeds it one Inputs per
tick and flies the Target it gets back; bench_search feeds it invented ones.

WHAT IT DOES. Sweeps the fence the autopilot holds at one altitude, in lines.
The moment the buoy map has an UNKNOWN buoy inside the fence it flies over it
and hovers until buoy_mapper locks its state, then goes back to the point on the
line where it left and carries on. A buoy that has not locked after `give_up_s`
overhead is given up on for the rest of that pass. When the map holds
`buoys_to_find` confirmed buoys it asks for RTL. If a whole pass ends short, the
next pass turns the lines 90 degrees and looks again.

WHO IS FLYING -- the rules this file exists to hold:

  ONLY THE PILOT STARTS IT. The search starts on a CHANGE into GUIDED (the SC
  switch), while enabled and ready. Already being in GUIDED is not a start: a
  search enabled from the page, or a search_node restarted mid-air, never moves
  the aircraft on its own. WiFi can switch the search off; it can never make
  the aircraft go.

  THE PILOT TAKES IT BACK WITH ANY SWITCH. Out of GUIDED -- SB to Loiter or
  Brake, SC off, an RTL -- the autopilot ignores everything this code sends, and
  the search pauses with its place kept. Back into GUIDED resumes it.

  STOPPING NEVER LEAVES IT FLYING SOMEWHERE. Switched off from the page, or its
  buoy map lost, while still in GUIDED: it holds position (a target at where the
  aircraft is), rather than going silent and letting the autopilot finish
  whatever leg it was on. A lost map resumes by itself when the map returns;
  switched off from the page needs the pilot's switch again.

  THE ONLY MODE IT EVER ASKS FOR IS RTL, once every buoy is found, and only while
  in GUIDED -- never over a mode the pilot chose.

WHAT COUNTS AS FOUND: a LOCKED buoy inside the fence. Buoys already confirmed on
the map when the search starts count too -- that is what lets a search carry on
after a battery swap -- so a map left over from an earlier flight has to be
cleared first, and the search refuses to start when it would already be done.
"""
import math
from dataclasses import dataclass, field
from typing import Optional

from uav_common import fcu_decode, geo
from uav_mission import escort_core as ec
from uav_mission import sweep_core as sc

FLYING = ("climb", "sweep", "divert", "hover", "return", "escort", "recheck")
#: Phases that are the DISRUPTIVE half: staying with the boat once the map is
#: made, instead of going home.
ESCORT = ("escort", "recheck")

#: Arrived, if within a few metres and barely moving for this long. The fence's
#: own avoidance can stop the aircraft just short of a point near the edge; a
#: search that waited for the last half metre would wait forever.
STALL_RADIUS_M = 3.0
STALL_SPEED_MPS = 0.15
STALL_S = 3.0
#: A leg given this much more than it should need is abandoned and the next one
#: taken, so wind or a stuck position can never park the search.
LEG_SLACK_S = 15.0
CLIMB_TIMEOUT_S = 45.0
#: How often RTL is asked for again while the autopilot still says GUIDED.
RTL_RETRY_S = 3.0
#: THE NOSE POINTS WHERE THE AIRCRAFT IS GOING (Chris, 19 Sep; it used to hold
#: the sweep lines' heading for a whole pass and fly the return legs backwards).
#: Within this of the goal the heading is FROZEN instead: over a buoy, the boat's
#: gate or a line end it never turns, so nothing is confirmed mid-turn and it
#: never spins chasing a point it is already over. The cost is a turn at every
#: line end, while the gimbal catches up and buoy_mapper refuses the frames.
FACE_HOLD_M = 3.0


@dataclass
class SearchConfig:
    search_alt_m: float = 10.0
    speed_mps: float = 2.0
    line_spacing_m: float = 10.0
    fence_inset_m: float = 2.0
    arrive_radius_m: float = 1.0
    hover_radius_m: float = 1.5
    give_up_s: float = 10.0
    divert_timeout_s: float = 45.0
    cluster_radius_m: float = 4.0
    recenter_m: float = 0.75
    climb_tolerance_m: float = 1.0
    fence_max_age_s: float = 90.0
    rtl_when_done: bool = True
    # "advanced": map the field, then RTL -- the passage does not change.
    # "disruptive": stay on station over the boat's next gate and re-check the
    # lights, because the passage may change while it transits (handbook 3.3.2).
    tier: str = "advanced"
    recheck_dwell_s: float = 6.0
    boat_timeout_s: float = 6.0


@dataclass
class Buoy:
    id: int
    lat: float
    lon: float
    locked: bool
    label: str = ""
    #: seconds since this buoy was last actually seen; None = unknown. A state
    #: nobody has looked at recently is not evidence that a gate still stands.
    age: float = None


@dataclass
class Inputs:
    now: float                              # monotonic seconds
    enabled: bool = False
    buoys_to_find: int = 0
    pose: Optional[tuple] = None            # (lat, lon, alt_rel_m, ground_speed)
    mode: Optional[str] = None              # None = autopilot status stale
    armed: Optional[bool] = None
    in_air: Optional[bool] = None           # None = flight state unknown
    fence: Optional[list] = None            # [(lat, lon)] valid polygon, or None
    fence_problem: str = ""
    fence_read_t: Optional[float] = None    # monotonic time of that read
    fence_enable: Optional[float] = None
    fence_type: Optional[float] = None
    fence_alt_max: Optional[float] = None
    fence_margin: Optional[float] = None
    buoys: Optional[list] = None            # [Buoy]; None = map not live
    #: (lat, lon, activity, age_s) from Crusader over the radio, or None.
    #: activity is the one byte the boat sends about what it is doing.
    boat: Optional[tuple] = None


@dataclass
class Target:
    lat: float
    lon: float
    alt_m: float
    yaw_deg: float                          # NaN = let the autopilot choose
    speed_mps: float


@dataclass
class Decision:
    target: Optional[Target]
    request_rtl: bool
    status: dict
    events: list = field(default_factory=list)


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _local(geom, lat, lon):
    """A lat/lon in the fence's local metres."""
    return geo.latlon_to_xy(lat, lon, geom["origin"])


class BuoySearch:

    def __init__(self, cfg: SearchConfig = None):
        self.cfg = cfg or SearchConfig()
        self._last_mode = None
        self._was_armed = None
        self._armed_t = None
        self._fence_key = None
        self._geom = None
        self._new_flight()

    def _new_flight(self):
        """Forget everything about the flight. Called on arm and on disarm."""
        self.sub = None           # a FLYING phase, "rtl", "complete", or None
        self.resume_sub = None    # the FLYING phase a pause or stop interrupted
        self.after_climb = "sweep"
        self.plan_key = None      # the fence the waypoints were planned in
        self.pass_index = 0
        self.wps, self.wp, self.yaw = [], 0, float("nan")
        self.face_yaw = float("nan")   # the heading last commanded; NaN = none yet
        self.skipped = set()
        self.cluster = {}         # buoy id -> time its clock started, or None
        self.hover_xy = None
        self.divert_t = None
        self.resume_xy = None
        self.climb = None         # (xy, start time)
        self.hold = None          # (xy, alt, needs_switch, reason)
        self.leg = None           # (goal xy, start time, timeout s)
        self.slow_since = None
        self.rtl_t = None
        self.flown = False
        # Disruptive: what the boat's next gate is, what we are re-checking.
        self.watch = None          # (red id, green id) being kept an eye on
        self.queue = []            # buoy ids still to re-read after a loss
        self.dwell_t = None        # when the aircraft got over the one in hand
        self.escort_note = ""      # the last thing worth saying about the pair
        # What Crusader is ALLOWED to drive to: the boat's next gate as
        # [red, green], or the exit as [id], and only while the aircraft is over
        # it and has just seen it. Empty the rest of the time -- including all
        # the way through the sweep, which is what keeps the boat at the entry.
        self.confirmed = []
        # Unknown buoys between the boat and the gate being watched are read
        # first; one that will not settle is given up on like any other.
        self.between_t = {}        # buoy id -> when the aircraft got over it
        self.between_skip = set()
        # Stopped because an INPUT went away (position, buoy map), not because
        # anyone asked. It comes back by itself when the input does: a one
        # second gap in the pose must not end a search that is mid-passage.
        self.lost_input = ""
        # Paused because the autopilot's status went stale, not because the
        # pilot chose a mode. Resumes by itself if it comes back still GUIDED.
        self.status_lost = False

    # ------------------------------------------------------------------ inputs

    def _track_arming(self, inp, ev):
        if inp.armed is None:
            return
        if inp.armed and not self._was_armed:
            # An arm this code SAW. Started already armed (a node restarted in
            # the air), the arm time is unknown, and the fence-read check that
            # depends on it is skipped rather than failed.
            if self._was_armed is not None:
                self._armed_t = inp.now
                self._new_flight()
        elif not inp.armed and self._was_armed:
            if self.flown:
                ev.append("disarmed: search reset")
            self._new_flight()
            self._armed_t = None
        self._was_armed = inp.armed

    def _geometry(self, inp):
        """The fence in local metres, shrunk, cached until the fence changes."""
        if not inp.fence:
            return None
        ring = sc.open_ring(inp.fence)
        key = tuple((round(a, 7), round(b, 7)) for a, b in ring)
        if key != self._fence_key:
            origin = sc.centroid(ring)
            hull, dent, problem = sc.fence_region(sc.to_local(ring, origin))
            shrunk = sc.inset(hull, self.cfg.fence_inset_m + dent) if not problem else []
            if not problem and not shrunk:
                problem = ("the fence is too small to stay %.0f m inside it"
                           % self.cfg.fence_inset_m)
            self._geom = {"key": key, "origin": origin, "ring": ring,
                          "hull": hull, "inset": shrunk, "problem": problem,
                          "theta": sc.best_angle(shrunk) if shrunk else 0.0}
            self._fence_key = key
        return self._geom

    def _inside_fence(self, geom, lat, lon):
        return geom is None or geo.point_in_polygon(lat, lon, geom["ring"])

    def _found(self, inp, geom):
        if inp.buoys is None:
            return 0
        return sum(1 for b in inp.buoys
                   if b.locked and self._inside_fence(geom, b.lat, b.lon))

    def _waiting_for(self, inp, geom, found):
        """Everything standing between now and a start, in the pilot's words."""
        c, r = self.cfg, []
        if inp.pose is None:
            r.append("no position from telemetry_bridge")
        if inp.armed is None:
            r.append("autopilot status unknown")
        elif not inp.armed:
            r.append("arm and take off")
        elif inp.in_air is False:
            r.append("take off first")
        if inp.fence is None:
            r.append(inp.fence_problem or "no fence read from the autopilot yet")
        elif geom["problem"]:
            r.append(geom["problem"])
        elif (inp.fence_read_t is None
              or inp.now - inp.fence_read_t > c.fence_max_age_s):
            # Not "read since arming": one read that failed at the arm edge then
            # blocked the search for the whole flight. telemetry_bridge re-reads
            # every 30 s in the air, so a stale read means IT is the problem.
            r.append("the autopilot's fence was last read %s ago -- is "
                     "telemetry_bridge up?"
                     % ("never" if inp.fence_read_t is None
                        else "%.0f s" % (inp.now - inp.fence_read_t)))
        if inp.fence_enable is None or inp.fence_type is None:
            r.append("fence settings not read from the autopilot yet")
        else:
            ftype = int(inp.fence_type)
            if inp.fence_enable < 0.5:
                r.append("the fence is switched off (FENCE_ENABLE 0)")
            if not ftype & fcu_decode.FENCE_TYPE_POLYGON:
                r.append("FENCE_TYPE does not include the polygon")
            if ftype & fcu_decode.FENCE_TYPE_ALT_MAX:
                if inp.fence_alt_max is None:
                    r.append("FENCE_ALT_MAX not read yet")
                else:
                    stop = inp.fence_alt_max - (inp.fence_margin or 0.0)
                    if c.search_alt_m > stop + 0.05:
                        r.append("search altitude %.0f m is above where the fence "
                                 "stops a climb (FENCE_ALT_MAX %.0f - FENCE_MARGIN "
                                 "%.0f = %.0f m)" % (c.search_alt_m,
                                                     inp.fence_alt_max,
                                                     inp.fence_margin or 0.0, stop))
        if inp.buoys is None:
            r.append("no buoy map: start detector_node and buoy_mapper")
        if inp.buoys_to_find < 1:
            r.append("set how many buoys to find")
        elif (found >= inp.buoys_to_find and not self.flown
              and c.tier != "disruptive"):
            # Advanced only. In Disruptive a complete map is not "already done",
            # it is where the job starts: the aircraft goes straight to watching
            # the boat's next gate.
            r.append("the map already has %d confirmed: clear the buoys, or raise "
                     "the count" % found)
        if (inp.pose is not None and geom is not None and geom["hull"]
                and not geom["problem"]):
            xy = _local(geom, inp.pose[0], inp.pose[1])
            if not sc.contains(geom["hull"], xy):
                r.append("Ekko is outside the fence")
        return r

    # ----------------------------------------------------------------- flying

    def _plan_pass(self, geom, xy, ev):
        theta, shifted = sc.pass_geometry(self.pass_index, geom["theta"])
        self.wps = sc.waypoints(
            sc.lines(geom["inset"], theta, self.cfg.line_spacing_m, shifted), xy)
        self.wp = 0
        self.yaw = sc.heading_deg(theta)
        self.plan_key = geom["key"]
        ev.append("pass %d: %d waypoints, lines at %03.0f"
                  % (self.pass_index + 1, len(self.wps), self.yaw))

    def _begin_leg(self, goal, now, xy):
        if self.leg is None or _dist(self.leg[0], goal) > 0.5:
            d = _dist(xy, goal)
            self.leg = (goal, now, LEG_SLACK_S + 2.0 * d / max(0.3, self.cfg.speed_mps))
            self.slow_since = None

    def _arrived(self, inp, xy, goal, radius, ev):
        self._begin_leg(goal, inp.now, xy)
        d = _dist(xy, goal)
        if d <= radius:
            return True
        speed = inp.pose[3]
        if d <= STALL_RADIUS_M and speed is not None and speed < STALL_SPEED_MPS:
            if self.slow_since is None:
                self.slow_since = inp.now
            elif inp.now - self.slow_since >= STALL_S:
                ev.append("stopped %.1f m short of a waypoint; taking it as reached" % d)
                return True
        else:
            self.slow_since = None
        if inp.now - self.leg[1] > self.leg[2]:
            ev.append("leg took over %.0f s, %.1f m still to go; moving on"
                      % (self.leg[2], d))
            return True
        return False

    def _unknown_buoys(self, inp, geom):
        return [b for b in (inp.buoys or [])
                if not b.locked and b.id not in self.skipped
                and self._inside_fence(geom, b.lat, b.lon)]

    def _update_cluster(self, inp, geom, ev):
        """Drop members that locked or vanished; take in unknown neighbours.
        -> the cluster centre in local metres, or None if it is empty."""
        by_id = {b.id: b for b in inp.buoys or []}
        for bid in list(self.cluster):
            b = by_id.get(bid)
            if b is None:
                del self.cluster[bid]
                ev.append("B%d left the map (merged into another)" % bid)
            elif b.locked:
                del self.cluster[bid]
                ev.append("B%d confirmed %s" % (bid, b.label))
        if not self.cluster:
            return None

        def xy_of(b):
            return _local(geom, b.lat, b.lon)
        members = [xy_of(by_id[i]) for i in self.cluster]
        centre = sc.centroid(members)
        for b in self._unknown_buoys(inp, geom):
            if b.id not in self.cluster and _dist(xy_of(b), centre) <= self.cfg.cluster_radius_m:
                self.cluster[b.id] = inp.now if self.sub == "hover" else None
                ev.append("B%d joins the hover" % b.id)
                members.append(xy_of(b))
        return sc.centroid(members)

    def _start(self, inp, geom, xy, ev):
        if self.plan_key != geom["key"]:
            self.pass_index = 0
            self._plan_pass(geom, xy, ev)
        after = self.resume_sub or "sweep"
        if after in ("hover", "climb"):
            after = "divert" if after == "hover" else "sweep"
        if after == "divert":
            self.divert_t = inp.now
            self.cluster = {k: None for k in self.cluster}
            self.hover_xy = None
        ev.append("search resumed" if self.flown else "search started")
        self.resume_sub, self.hold, self.leg, self.flown = None, None, None, True
        if abs(inp.pose[2] - self.cfg.search_alt_m) > self.cfg.climb_tolerance_m:
            self.climb, self.after_climb, self.sub = (xy, inp.now), after, "climb"
        else:
            self.sub = after

    def _hold_here(self, inp, xy, needs_switch, reason):
        """Hold at xy, at the altitude it is at (never above the search altitude,
        never below 2 m). needs_switch: resuming takes the pilot's SC switch."""
        alt = min(self.cfg.search_alt_m, max(2.0, inp.pose[2]))
        self.hold = (xy, alt, needs_switch, reason)

    def _stop(self, inp, xy, needs_switch, reason, ev):
        """Leave flying; hold where the aircraft is if it is still in GUIDED."""
        if self.sub in FLYING:
            self.resume_sub = self.sub
        self.sub = None
        self.leg = None
        if inp.mode == "GUIDED" and xy is not None:
            self._hold_here(inp, xy, needs_switch, reason)
        ev.append("holding: %s" % reason if self.hold else "stopped: %s" % reason)

    def _fly(self, inp, geom, xy, found, ev):
        """One tick of flying. -> local-metre goal, or None."""
        c = self.cfg
        if self.sub in ESCORT:
            return self._escort(inp, geom, xy, ev)
        if found >= inp.buoys_to_find:
            if c.tier == "disruptive":
                ev.append("all %d found: staying on station for the boat" % found)
                self.sub, self.watch, self.queue = "escort", None, []
                return self._escort(inp, geom, xy, ev)
            self.sub = "rtl" if c.rtl_when_done else "complete"
            self._hold_here(inp, xy, True, "all %d found" % found)
            ev.append("all %d buoys found%s" % (found, ": asking for RTL"
                                                if c.rtl_when_done else ""))
            return None

        if self.sub in ("sweep", "return"):
            unknown = self._unknown_buoys(inp, geom)
            if unknown:
                near = min(unknown, key=lambda b: _dist(_local(geom, b.lat, b.lon), xy))
                if self.sub == "sweep":
                    self.resume_xy = xy
                self.cluster = {near.id: None}
                self.hover_xy, self.divert_t, self.sub = None, inp.now, "divert"
                ev.append("B%d spotted: flying over it" % near.id)

        if self.sub == "climb":
            if (abs(inp.pose[2] - c.search_alt_m) <= c.climb_tolerance_m
                    or inp.now - self.climb[1] > CLIMB_TIMEOUT_S):
                self.sub = self.after_climb
                if self.sub == "divert":
                    self.divert_t = inp.now
            else:
                return self.climb[0]

        if self.sub in ("divert", "hover"):
            centre = self._update_cluster(inp, geom, ev)
            if centre is None:
                self.sub = "return"
            else:
                centre = sc.clamp_inside(geom["inset"], centre)
                if self.hover_xy is None or _dist(centre, self.hover_xy) > c.recenter_m:
                    self.hover_xy = centre
                if self.sub == "divert":
                    if _dist(xy, self.hover_xy) <= c.hover_radius_m:
                        self.sub = "hover"
                        self.cluster = {k: inp.now for k in self.cluster}
                    elif inp.now - self.divert_t > c.divert_timeout_s:
                        ev.append("could not reach B%s in %.0f s; skipping"
                                  % (",".join(map(str, sorted(self.cluster))),
                                     c.divert_timeout_s))
                        self.skipped.update(self.cluster)
                        self.cluster = {}
                        self.sub = "return"
                if self.sub == "hover":
                    for bid, t0 in list(self.cluster.items()):
                        if t0 is None:
                            self.cluster[bid] = inp.now
                        elif inp.now - t0 >= c.give_up_s:
                            del self.cluster[bid]
                            self.skipped.add(bid)
                            ev.append("gave up on B%d after %.0f s overhead"
                                      % (bid, c.give_up_s))
                    if not self.cluster:
                        self.sub = "return"
            if self.sub in ("divert", "hover"):
                return self.hover_xy

        if self.sub == "return":
            if self.resume_xy is None or self._arrived(inp, xy, self.resume_xy,
                                                       c.arrive_radius_m, ev):
                self.resume_xy, self.sub = None, "sweep"
            else:
                return self.resume_xy

        # sweep
        while self.wp < len(self.wps) and self._arrived(inp, xy, self.wps[self.wp],
                                                        c.arrive_radius_m, ev):
            self.wp += 1
            self.leg = None
        if self.wp >= len(self.wps):
            self.pass_index += 1
            self.skipped.clear()
            ev.append("pass %d finished with %d of %d found"
                      % (self.pass_index, found, inp.buoys_to_find))
            self._plan_pass(geom, xy, ev)
            if not self.wps:
                return xy
        return self.wps[self.wp]

    # ------------------------------------------------------------ disruptive

    def _escort_view(self, inp, geom):
        """The map in local metres, as escort_core wants it."""
        return [{"id": b.id, "xy": _local(geom, b.lat, b.lon), "label": b.label,
                 "age": b.age}
                for b in (inp.buoys or [])
                if self._inside_fence(geom, b.lat, b.lon)]

    @staticmethod
    def _seen_now(bu, ids):
        """Every one of these buoys actually sighted within FRESH_S. A missing
        age is NOT fresh: a confirmation is a claim about what was just seen."""
        by_id = {b["id"]: b for b in bu}
        return all(i in by_id and by_id[i].get("age") is not None
                   and by_id[i]["age"] <= ec.FRESH_S for i in ids)

    def _escort(self, inp, geom, xy, ev):
        """Stay with the boat: watch its next gate, and when a light changes, go
        and re-read the buoys that could be the new one. -> goal, or None."""
        c = self.cfg
        bu = self._escort_view(inp, geom)
        crs = ec.course(bu)
        if crs is None:
            self.escort_note = ("no entry and exit buoys on the map: nothing to "
                                "measure the passage along")
            return xy
        entry, _exit_xy, axis = crs
        fresh_boat = (inp.boat is not None
                      and (inp.boat[3] is None or inp.boat[3] <= c.boat_timeout_s))
        boat_xy = _local(geom, inp.boat[0], inp.boat[1]) if fresh_boat else entry
        here = ec.along(boat_xy, entry, axis)

        live = {(g["red"], g["green"]) for g in ec.gates(bu, None, axis)}
        if self.watch is not None and self.watch not in live:
            # gate_holds names the buoy that actually changed, in the same words
            # the page shows -- "B2 is now off", not a list of both lights.
            _ok, why = ec.gate_holds({"red": self.watch[0], "green": self.watch[1]},
                                     bu, fresh_s=None)
            why = why or "the pair no longer reads red and green"
            ev.append("GATE LOST: %s" % why)
            self.escort_note = why
            self.watch, self.dwell_t = None, None
            self.queue = [b["id"] for b in
                          ec.recheck_order(bu, boat_xy, xy, crs)]
            self.sub = "recheck"

        if self.sub == "recheck":
            # A gate only counts as FOUND when it has just been looked at:
            # the whole point of the re-check is fresh evidence.
            ahead = [g for g in ec.gates(bu, ec.FRESH_S, axis)
                     if ec.along(g["mid"], entry, axis) > here + ec.AHEAD_MARGIN_M]
            if ahead:
                g = min(ahead, key=lambda g: ec.along(g["mid"], entry, axis))
                self.watch = (g["red"], g["green"])
                self.sub, self.dwell_t = "escort", None
                self.escort_note = ""
                ev.append("new gate found: B%d/B%d -- sent to the boat"
                          % (g["red"], g["green"]))
            elif self.queue:
                by_id = {b["id"]: b for b in bu}
                b = by_id.get(self.queue[0])
                if b is None:
                    self.queue.pop(0)
                    return xy
                goal = sc.clamp_inside(geom["inset"], b["xy"])
                if _dist(xy, goal) <= c.hover_radius_m:
                    if self.dwell_t is None:
                        self.dwell_t = inp.now
                    elif inp.now - self.dwell_t >= c.recheck_dwell_s:
                        self.queue.pop(0)
                        self.dwell_t = None
                return goal
            elif ec.unread_ahead(bu, boat_xy, crs):
                # Something ahead is still unread (it dropped off the map, or
                # the re-read did not settle): keep reading rather than deciding.
                self.queue = [b["id"] for b in ec.unread_ahead(bu, boat_xy, crs)]
                self.escort_note = "still reading ahead of the boat"
            else:
                # Everything ahead has been read and none of it is a gate. The
                # boat's next element is the exit, so go and watch that; sitting
                # over the boat would tell it nothing it does not know.
                self.sub, self.dwell_t = "escort", None
                self.escort_note = ("nothing ahead of the boat is a gate: "
                                    "watching the exit")
                ev.append("re-check finished: no gate ahead of the boat")

        el = ec.next_element(boat_xy, bu, crs,
                             transiting=fresh_boat and inp.boat[2] == ec.BOAT_TRANSIT)
        if el["kind"] == "gate":
            g = el["gate"]
            if self.watch != (g["red"], g["green"]):
                self.between_t, self.between_skip = {}, set()
            self.watch = (g["red"], g["green"])
            # Anything still UNKNOWN between the boat and this gate is read
            # before the gate is confirmed: it may be half of a nearer one.
            # UNKNOWN only, never merely stale -- a stale buoy read on the way
            # would be stale again by the time the aircraft reached the gate,
            # and it would shuttle between the two for ever.
            gate_at = ec.along(g["mid"], entry, axis)
            between = [b for b in ec.unread_ahead(bu, boat_xy, crs)
                       if b["label"] == "UNKNOWN"
                       and b["id"] not in self.between_skip
                       and ec.along(b["xy"], entry, axis) < gate_at]
            if between:
                b = between[0]
                goal = sc.clamp_inside(geom["inset"], b["xy"])
                self.escort_note = "reading B%d first" % b["id"]
                if _dist(xy, goal) <= c.hover_radius_m:
                    t0 = self.between_t.setdefault(b["id"], inp.now)
                    if inp.now - t0 >= c.give_up_s:
                        self.between_skip.add(b["id"])
                        ev.append("gave up reading B%d after %.0f s overhead"
                                  % (b["id"], c.give_up_s))
                return goal
            self.escort_note = ""
            goal = sc.clamp_inside(geom["inset"], g["mid"])
            held, _why = ec.gate_holds(g, bu, fresh_s=ec.FRESH_S)
            if (_dist(xy, goal) <= c.hover_radius_m and held
                    and self._seen_now(bu, self.watch)):
                self.confirmed = [g["red"], g["green"]]
            return goal
        else:
            # No gate ahead. If anything ahead is unread, that is why -- go and
            # read it. Only when the picture ahead is complete does "no gate
            # ahead" mean the boat's next element really is the exit.
            unread = ec.unread_ahead(bu, boat_xy, crs)
            if unread:
                self.watch, self.dwell_t = None, None
                self.queue = [b["id"] for b in unread]
                self.sub = "recheck"
                self.escort_note = "nothing confirmed ahead of the boat"
                ev.append("no gate ahead: reading %d buoys in front of the boat"
                          % len(unread))
                return sc.clamp_inside(geom["inset"],
                                       [b for b in bu
                                        if b["id"] == self.queue[0]][0]["xy"])
            self.watch = None
            self.escort_note = "watching the exit buoy"
            goal = sc.clamp_inside(geom["inset"], el["xy"])
            ex = min((b for b in bu if b["label"] == ec.EXIT),
                     key=lambda b: _dist(b["xy"], el["xy"]), default=None)
            if (ex is not None and _dist(xy, goal) <= c.hover_radius_m
                    and self._seen_now(bu, [ex["id"]])):
                self.confirmed = [ex["id"]]
            return goal

    # ------------------------------------------------------------------- step

    def step(self, inp: Inputs) -> Decision:
        ev = []
        # A confirmation is made fresh on every tick the escort runs, and on no
        # other tick. Without this, the tick the pilot flips back into GUIDED
        # could report one made before the pause, before anything re-checked.
        self.confirmed = []
        self._track_arming(inp, ev)
        guided = inp.mode == "GUIDED"
        edge = (guided and self._last_mode is not None
                and self._last_mode != "GUIDED")
        if inp.mode is not None:
            self._last_mode = inp.mode
        geom = self._geometry(inp)
        found = self._found(inp, geom)
        waiting = self._waiting_for(inp, geom, found)
        xy = None
        if inp.pose is not None and geom is not None and not geom["problem"]:
            xy = _local(geom, inp.pose[0], inp.pose[1])

        goal, rtl = None, False
        if self.sub in FLYING:
            if not guided:
                self.resume_sub, self.sub, self.leg = self.sub, None, None
                self.status_lost = inp.mode is None
                ev.append("paused: autopilot status lost" if self.status_lost
                          else "paused: pilot selected %s" % inp.mode)
            elif not inp.enabled:
                self._stop(inp, xy, True, "search switched off", ev)
            elif xy is None:
                self.lost_input = "position"
                self._stop(inp, None, False, "no position", ev)
            elif self.plan_key != geom["key"]:
                self._stop(inp, xy, True, "the fence changed", ev)
            elif inp.buoys is None:
                self.lost_input = "buoy map"
                self._stop(inp, xy, False, "the buoy map stopped", ev)
            else:
                goal = self._fly(inp, geom, xy, found, ev)
        elif self.sub is None and guided and inp.enabled:
            # A hold after a lost input, or a pause while the autopilot's status
            # was stale (the mode never seen to change), resumes by itself once
            # the input is back; every other start needs the pilot's switch.
            lost = ((self.hold is not None and not self.hold[2])
                    or self.status_lost or self.lost_input)
            if (edge or lost) and not waiting:
                if self.lost_input:
                    ev.append("%s is back: carrying on" % self.lost_input)
                    self.lost_input = ""
                self._start(inp, geom, xy, ev)
                goal = self._fly(inp, geom, xy, found, ev)
        if self.sub == "rtl":
            if guided and (self.rtl_t is None or inp.now - self.rtl_t >= RTL_RETRY_S):
                rtl, self.rtl_t = True, inp.now
            elif inp.mode is not None and not guided and self.rtl_t is not None:
                self.sub = "complete"
                ev.append("search complete (autopilot now %s)" % inp.mode)
        if not guided and self.hold is not None:
            self.hold = None
        if inp.mode is not None and not guided:
            self.status_lost = False        # a mode the pilot chose: needs SC
        if self.sub in FLYING:
            self.status_lost = False

        target = None
        if goal is not None and self.sub in FLYING:
            lat, lon = geo.xy_to_latlon(goal[0], goal[1], geom["origin"])
            target = Target(lat, lon, self.cfg.search_alt_m, self._face(xy, goal),
                            self.cfg.speed_mps)
        elif self.hold is not None and guided and geom is not None:
            (hx, hy), alt = self.hold[0], self.hold[1]
            lat, lon = geo.xy_to_latlon(hx, hy, geom["origin"])
            target = Target(lat, lon, alt, float("nan"), self.cfg.speed_mps)
        return Decision(target, rtl, self._status(inp, geom, found, waiting,
                                                  guided, target), ev)

    def _face(self, xy, goal):
        """The heading for a target: toward the goal, or frozen within
        FACE_HOLD_M of it (see FACE_HOLD_M). NaN until there is one to hold,
        which lets the autopilot keep the heading it has."""
        if _dist(xy, goal) > FACE_HOLD_M:
            self.face_yaw = math.degrees(
                math.atan2(goal[0] - xy[0], goal[1] - xy[1])) % 360.0
        return self.face_yaw

    # ----------------------------------------------------------------- status

    def phase(self, inp, guided, waiting):
        if self.sub is not None:
            return self.sub
        if self.hold is not None and guided:
            return "holding"
        if not inp.enabled:
            return "off"
        if self.resume_sub is not None and not guided:
            return "paused"
        return "not_ready" if waiting else "ready"

    def _text(self, phase, inp, found, waiting, guided):
        c, n = self.cfg, inp.buoys_to_find
        tally = "%d of %d found" % (found, n)
        ids = ", ".join("B%d" % i for i in sorted(self.cluster))
        overhead = [inp.now - t for t in self.cluster.values() if t is not None]
        return {
            "off": "Search is off",
            "not_ready": "Not ready: %s" % (waiting[0] if waiting else ""),
            "ready": ("Ready: flip SC off and on to start" if guided
                      else "Ready: flip SC to GUIDED to start"),
            "climb": "Climbing to %.0f m" % c.search_alt_m,
            "sweep": "Sweeping, pass %d, leg %d of %d · %s"
                     % (self.pass_index + 1, min(self.wp + 1, len(self.wps)),
                        len(self.wps), tally),
            "divert": "Flying to %s · %s" % (ids, tally),
            "hover": "Confirming %s: %.0f s of %.0f · %s"
                     % (ids, max(overhead) if overhead else 0.0, c.give_up_s, tally),
            "return": "Back to the sweep line · %s" % tally,
            "escort": self._escort_text(),
            "recheck": ("Gate lost (%s): re-reading B%s, %d to go"
                        % (self.escort_note or "a light changed", self.queue[0],
                           len(self.queue) - 1) if self.queue else
                        "Gate lost: %s" % (self.escort_note or "looking")),
            "holding": "Holding position: %s%s" % (
                self.hold[3] if self.hold else "",
                " (flip SC off and on to resume)" if self.hold and self.hold[2]
                else ""),
            "paused": "Paused, you have control (%s). Flip SC to GUIDED to resume"
                      % (inp.mode or "?"),
            "rtl": "All %d found: returning home (RTL)" % found,
            "complete": "Search complete: %s%s" % (
                tally, " · autopilot in %s" % inp.mode if inp.mode else ""),
        }[phase]

    def _escort_text(self):
        """The pilot's line while escorting: confirmed, or still checking."""
        if self.confirmed and self.watch:
            return "Confirmed the boat's gate B%d/B%d: Crusader may go" % self.watch
        if self.confirmed:
            return "Confirmed the exit B%d: Crusader may go" % self.confirmed[0]
        if self.watch:
            return ("Checking the boat's gate B%d/B%d%s" % (
                self.watch + (": " + self.escort_note if self.escort_note else "",)))
        return "On station: %s" % (self.escort_note or "watching the passage")

    def _status(self, inp, geom, found, waiting, guided, target):
        ph = self.phase(inp, guided, waiting)
        origin = geom["origin"] if geom else None
        plan = sc.to_latlon(self.wps, origin) if origin and self.wps else []
        shrunk = sc.to_latlon(geom["inset"], origin) if origin and geom["inset"] else []
        overhead = [inp.now - t for t in self.cluster.values() if t is not None]
        nan = float("nan")
        return {
            "phase": ph,
            "text": self._text(ph, inp, found, waiting, guided),
            "waiting_for": [] if ph in FLYING else waiting,
            "enabled": bool(inp.enabled),
            "buoys_to_find": int(inp.buoys_to_find),
            "found": int(found),
            "flying": ph in FLYING and guided,
            "pass_number": self.pass_index + 1 if self.wps else 0,
            "leg": min(self.wp + 1, len(self.wps)),
            "legs": len(self.wps),
            "tier": self.cfg.tier,
            "watching": list(self.watch) if self.watch else [],
            # Only ever while escorting: a pilot who takes the aircraft back
            # takes the confirmation with it, and the boat stops.
            "confirmed": list(self.confirmed) if ph == "escort" else [],
            "rechecking": list(self.queue),
            "note": self.escort_note,
            "hover_buoys": sorted(self.cluster) if ph in ("divert", "hover") else [],
            "hover_s": max(overhead) if overhead and ph == "hover" else 0.0,
            "give_up_s": self.cfg.give_up_s,
            "skipped": sorted(self.skipped),
            "target_lat": target.lat if target else nan,
            "target_lon": target.lon if target else nan,
            "plan": plan,
            "inset": shrunk,
        }
