#!/usr/bin/env python3
"""bench_heartbeat — drive heartbeat_core through every case that must go quiet.

No ROS, no link, no aircraft.

    python3 tools/bench/bench_heartbeat.py

The interesting cases are the ones where the honest answer is SILENCE. An OCS
heartbeat that is merely absent shows as rising silence and someone investigates;
one that is present and invented is indistinguishable from a real fix and is
relayed onward for Singapore's Network Remote ID. Every row below that expects
`quiet` is a case where inventing would have been easy.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "uav_groundstation"))

from uav_groundstation import heartbeat_core as hc  # noqa: E402
from uav_groundstation import mission_core as mc  # noqa: E402


class Pose:
    def __init__(self, heading=90.0, alt_rel=0.0, alt_amsl=42.0):
        self.latitude, self.longitude = 1.2806, 103.8557
        self.heading = heading
        self.ground_speed = 6.5
        self.altitude_amsl = alt_amsl
        self.altitude_rel = alt_rel
        self.climb = 0.0


class Status:
    def __init__(self, mode="LOITER", armed=True):
        self.mode, self.armed = mode, armed


class Att:
    roll, pitch, yaw = 0.05, -0.02, 1.0


GEOID = -24.6


def build(**kw):
    kw.setdefault("pose", Pose())
    kw.setdefault("status", Status())
    kw.setdefault("attitude", Att())
    kw.setdefault("landed", hc.LANDED_ON_GROUND)
    # state and task now come from mission_planner; heartbeat_core only
    # carries them. Defaults here keep these checks about flight phase.
    kw.setdefault("state", "STATE_MANUAL")
    kw.setdefault("task", "TASK_NONE")
    return hc.build_heartbeat(geoid_separation_m=GEOID, airborne_alt_m=1.0, **kw)


def check(name, expect, **kw):
    hb, info = build(**kw)
    if expect == "quiet":
        ok = hb is None
        detail = info if hb is None else "SENT a heartbeat: %s" % info
    else:
        ok = hb is not None and hb.get("flight_phase") == expect
        detail = ("phase=%s source=%s" % (hb.get("flight_phase"), info)
                  if hb else "went quiet: %s" % info)
    print("  %-34s %-4s %s" % (name, "PASS" if ok else "FAIL", detail[:78]))
    return ok


def main():
    r = []
    print("\nflight_phase — the autopilot's own answer is preferred")
    r.append(check("on ground", hc.GROUNDED, landed=hc.LANDED_ON_GROUND))
    r.append(check("in air", hc.AIRBORNE, landed=hc.LANDED_IN_AIR))
    r.append(check("taking off -> AIRBORNE", hc.AIRBORNE, landed=hc.LANDED_TAKEOFF))
    r.append(check("landing -> AIRBORNE", hc.AIRBORNE, landed=hc.LANDED_LANDING))

    print("\nfallback, only when the autopilot is silent")
    r.append(check("no EXT_SYS_STATE, armed+high", hc.AIRBORNE,
                   landed=None, pose=Pose(alt_rel=20.0), status=Status(armed=True)))
    r.append(check("no EXT_SYS_STATE, armed+low", hc.GROUNDED,
                   landed=None, pose=Pose(alt_rel=0.2), status=Status(armed=True)))
    r.append(check("no EXT_SYS_STATE, disarmed high", hc.GROUNDED,
                   landed=None, pose=Pose(alt_rel=20.0), status=Status(armed=False)))
    r.append(check("UNDEFINED falls back, not GROUNDED", hc.AIRBORNE,
                   landed=hc.LANDED_UNDEFINED, pose=Pose(alt_rel=20.0)))

    print("\nsilence — every case where inventing would have been easy")
    r.append(check("pose stale", "quiet", pose=None))
    r.append(check("fcu_status stale", "quiet", status=None))
    r.append(check("no phase and no pose", "quiet", landed=None, pose=None))
    r.append(check("no phase and no status", "quiet", landed=None, status=None))
    r.append(check("unmapped landed_state", "quiet", landed=99))

    print("\nfields")
    hb, src = build(landed=hc.LANDED_IN_AIR, pose=Pose(alt_amsl=42.0))
    r.append(("altitude_hae_m = AMSL + geoid", abs(hb["altitude_hae_m"] - 17.4) < 1e-9))
    r.append(("vehicle_type TYPE_UAV", hb["vehicle_type"] == "TYPE_UAV"))
    r.append(("depth_m 0.0", hb["depth_m"] == 0.0))
    r.append(("current_task never UNKNOWN", hb["current_task"] == "TASK_NONE"))
    r.append(("no UNKNOWN anywhere",
              not any(str(v).endswith("UNKNOWN") for v in hb.values())))
    hbn, _ = build(landed=hc.LANDED_IN_AIR, pose=Pose(heading=float("nan")))
    r.append(("NaN heading survives unscrubbed", math.isnan(hbn["heading_deg"])))
    hba, _ = build(landed=hc.LANDED_IN_AIR, attitude=None)
    r.append(("stale attitude drops 2 fields, keeps hb",
              hba is not None and "roll_deg" not in hba))
    for name, ok in r[-7:]:
        print("  %-34s %s" % (name, "PASS" if ok else "FAIL"))

    print("\nstate (mission_core -- the rule moved out of heartbeat_core)")
    # NOTE the first two cases INVERT the old expectations, deliberately.
    # LOITER is the autopilot holding position for a human unless a mission
    # is driving it; the old list called it autonomous unconditionally and
    # so reported STATE_AUTO for a hand-flown aircraft.
    rs = mc.robot_state
    st = [("armed LOITER, no mission -> MANUAL",
           rs("LOITER", True)[0] == "STATE_MANUAL"),
          ("armed LOITER, mission running -> AUTO",
           rs("LOITER", True, True)[0] == "STATE_AUTO"),
          ("armed GUIDED -> AUTO regardless of mission",
           rs("GUIDED", True)[0] == "STATE_AUTO"),
          ("armed RTL -> AUTO",
           rs("RTL", True)[0] == "STATE_AUTO"),
          ("armed STABILIZE, no mission -> MANUAL",
           rs("STABILIZE", True)[0] == "STATE_MANUAL"),
          ("disarmed GUIDED -> MANUAL",
           rs("GUIDED", False)[0] == "STATE_MANUAL"),
          ("disarmed with a mission running -> MANUAL",
           rs("GUIDED", False, True)[0] == "STATE_MANUAL"),
          ("stale fcu_status -> no state at all",
           rs(None, None)[0] is None),
          ("never STATE_UNKNOWN",
           rs("", False)[0] != "STATE_UNKNOWN")]
    for name, ok in st:
        print("  %-34s %s" % (name, "PASS" if ok else "FAIL"))

    flat = [x if isinstance(x, bool) else x[1] for x in r] + [ok for _, ok in st]
    print("\n%d/%d" % (sum(bool(x) for x in flat), len(flat)))
    return 0 if all(flat) else 1


if __name__ == "__main__":
    raise SystemExit(main())
