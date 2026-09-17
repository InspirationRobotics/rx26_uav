"""escort_core -- Task 1 DISRUPTIVE: watching the passage while Crusader drives it.

PURE: no ROS, no clock, no I/O. Local metres, x east / y north, like sweep_core.

WHAT DISRUPTIVE ADDS (handbook 3.3.2): the safe passage may CHANGE while the
surface system is transiting, and the change is visible only from above. So the
UAV stays on station instead of going home, and the USV drives on what it is
told in real time.

THE BUOYS DO NOT MOVE -- THE LIGHTS CHANGE. Same anchored hulls, a different
pair lit red and green. So a "lost gate" is never a search for a moved buoy: it
is a re-read of beacons on buoys already pinned, which is far cheaper and is why
the drone can keep up with the boat.

THE 4 METRE RULE. RoboNation confirmed Task 1 gates are 3 m apart, so a buoy
with no neighbour within PAIR_MAX_M can never be half of a gate whatever its
light does: it is LONE, and it is checked only when everything pairable has been
checked and nothing was found. That one rule is what keeps the re-check short
enough to beat the boat.

WHAT IS AHEAD. Buoys behind the boat cannot become its next gate, so the
re-check starts with what is in front of it, measured along the entry -> exit
line. Only if nothing ahead pairs does it look behind: a passage that changed
behind the boat is not this run's problem, but being wrong about that is.
"""
import math

#: Gates are 3 m apart (RoboNation, Discord). A buoy with no neighbour this
#: close cannot pair with anything, whatever its beacon does.
PAIR_MAX_M = 4.0
#: A buoy this much further along the course than the boat counts as ahead.
AHEAD_MARGIN_M = 1.0
#: A state older than this is not evidence WHEN CONFIRMING a gate: the drone is
#: hovering over that gate, so it has just looked. It is NOT applied when
#: choosing which gate to fly to -- the aircraft can only look at one place at a
#: time, and a map where every buoy but the one underneath is "too old" leaves
#: nothing to plan with. Planning uses the map as it stands; confirming needs a
#: fresh look. Pass fresh_s=None to skip the age test.
FRESH_S = 4.0

RED, GREEN = "FLASHING_RED", "FLASHING_GREEN"
ENTRY, EXIT = "FLASHING_BLUE", "SOLID_BLUE"

#: Handbook 3.3.2: a FLASHING RED light is passed on the boat's STARBOARD side
#: and a FLASHING GREEN on its PORT side. Driving entry -> exit, that means green
#: on the left and red on the right. A red/green pair the other way round is not
#: a gate for this direction at all -- sending the boat through it would put it
#: the wrong side of both lights.


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def pairable(buoys):
    """ids of buoys with a neighbour within PAIR_MAX_M. The rest are LONE and
    can never be half of a gate."""
    out = set()
    for a in buoys:
        for b in buoys:
            if a["id"] != b["id"] and _d(a["xy"], b["xy"]) <= PAIR_MAX_M:
                out.add(a["id"])
                break
    return out


def gates(buoys, fresh_s=None, axis=None):
    """Every red/green pair close enough to be a gate, as
    {"red", "green", "mid", "width"}.

    fresh_s=None uses the map as it stands (planning: where do we go next).
    fresh_s=FRESH_S accepts only states just looked at (confirming: is this
    really still a gate).
    axis: the course direction. Given, only pairs the boat can legally pass are
    returned -- green to port, red to starboard. Pass it whenever the answer is
    going to be steered by.
    """
    live = [b for b in buoys
            if fresh_s is None or b.get("age") is None or b["age"] <= fresh_s]
    reds = [b for b in live if b["label"] == RED]
    greens = [b for b in live if b["label"] == GREEN]
    out = []
    for r in reds:
        for g in greens:
            w = _d(r["xy"], g["xy"])
            if w > PAIR_MAX_M:
                continue
            if axis is not None:
                # Left of the course direction is (-y, x). Green must be there.
                side = ((g["xy"][0] - r["xy"][0]) * -axis[1]
                        + (g["xy"][1] - r["xy"][1]) * axis[0])
                if side <= 0:
                    continue
            out.append({"red": r["id"], "green": g["id"], "width": w,
                        "mid": ((r["xy"][0] + g["xy"][0]) / 2.0,
                                (r["xy"][1] + g["xy"][1]) / 2.0)})
    return out


def course(buoys):
    """(entry xy, exit xy, unit vector along the passage) or None.

    The passage runs entry -> exit; everything "ahead" and "behind" is measured
    along it. Without both blues there is no course and the escort says so
    rather than guessing a direction.
    """
    entry = [b for b in buoys if b["label"] == ENTRY]
    exit_ = [b for b in buoys if b["label"] == EXIT]
    if len(entry) != 1 or len(exit_) != 1:
        return None
    a, b = entry[0]["xy"], exit_[0]["xy"]
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    if n < 1.0:
        return None
    return a, b, (dx / n, dy / n)


def along(point, entry, axis):
    """How far along the passage a point is, in metres from the entry buoy."""
    return (point[0] - entry[0]) * axis[0] + (point[1] - entry[1]) * axis[1]


def next_element(boat_xy, buoys, crs, fresh_s=None):
    """What the boat has to do next: the first gate ahead of it, else the exit.

    -> {"kind": "gate"|"exit", "gate": {...} or None, "xy": station point}
    """
    entry, exit_xy, axis = crs
    boat = along(boat_xy, entry, axis)
    ahead = sorted((g for g in gates(buoys, fresh_s, axis)
                    if along(g["mid"], entry, axis) > boat + AHEAD_MARGIN_M),
                   key=lambda g: along(g["mid"], entry, axis))
    if ahead:
        return {"kind": "gate", "gate": ahead[0], "xy": ahead[0]["mid"]}
    return {"kind": "exit", "gate": None, "xy": exit_xy}


def gate_holds(gate, buoys, fresh_s=FRESH_S):
    # fresh_s=None skips the age test, for explaining a change rather than
    # confirming one.
    """Is this gate still a gate? (both lights still red/green, seen recently)

    -> (True, "") or (False, why in the pilot's words).
    """
    by_id = {b["id"]: b for b in buoys}
    for role, want in (("red", RED), ("green", GREEN)):
        b = by_id.get(gate[role])
        if b is None:
            return False, "B%d is no longer on the map" % gate[role]
        if fresh_s is not None and b.get("age") is not None and b["age"] > fresh_s:
            return False, "B%d has not been seen for %.0f s" % (b["id"], b["age"])
        if b["label"] != want:
            return False, "B%d is now %s" % (b["id"], b["label"].replace("_", " ").lower())
    return True, ""


def unread_ahead(buoys, boat_xy, crs, fresh_s=FRESH_S):
    """Buoys ahead of the boat whose light nobody can vouch for, nearest first.

    UNKNOWN, or a state too old to be evidence. While any of these exist the
    picture in front of the boat is incomplete, and an incomplete picture is
    the one thing a Disruptive run must never drive into: the drone goes and
    reads them rather than parking over the exit and calling it done.
    """
    entry, _exit, axis = crs
    here = along(boat_xy, entry, axis)
    pair = pairable(buoys)
    out = [b for b in buoys
           if b["id"] in pair
           and b["label"] not in (ENTRY, EXIT)
           and along(b["xy"], entry, axis) > here + AHEAD_MARGIN_M
           and (b["label"] == "UNKNOWN"
                or (b.get("age") is not None and b["age"] > fresh_s))]
    return sorted(out, key=lambda b: along(b["xy"], entry, axis))


def recheck_order(buoys, boat_xy, drone_xy, crs, skip=()):
    """Which buoys to go and look at after a gate fails, in order.

    Pairable first (a LONE buoy cannot become a gate at all), then ahead of the
    boat (one it has passed cannot be its next gate), and within those the one
    EARLIEST ALONG THE PASSAGE first -- not the one nearest the aircraft. The
    boat's next gate is the first it will reach, so that is the one worth
    knowing about first; flying to the nearest instead can confirm a gate far
    down the course while the boat waits at one it could already have passed.
    """
    entry, _exit, axis = crs
    boat = along(boat_xy, entry, axis)
    pair = pairable(buoys)

    def rank(b):
        ahead = along(b["xy"], entry, axis) > boat + AHEAD_MARGIN_M
        return (0 if b["id"] in pair else 1,
                0 if ahead else 1,
                # ahead: the earliest the boat will meet. behind: the nearest
                # to the boat, working back from it.
                along(b["xy"], entry, axis) if ahead
                else -along(b["xy"], entry, axis))
    return [b for b in sorted(buoys, key=rank)
            if b["id"] not in skip and b["label"] not in (ENTRY, EXIT)]
