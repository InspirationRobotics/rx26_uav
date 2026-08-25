"""mission_core — what state and task the aircraft is in. Pure: no ROS, no clock.

mission_planner is the thin wrapper that feeds this freshness-checked telemetry
and publishes the answer. Same split as heartbeat_core next door, and for the
same reason: this is safety-relevant logic in the specific sense that a wrong
answer is relayed to a regulator, and a rule you can exercise without rclpy in
the way is a rule you can trust.

WHY THE AIRCRAFT'S RULE IS LONGER THAN THE BOAT'S. On a hull the mode is the
whole answer: GUIDED or AUTO and armed means it is driving itself. An aircraft
has a case the mode alone cannot express -- a mission that deliberately flies in
LOITER while computer vision does the steering. In that configuration the
aircraft IS autonomous and the mode says LOITER, so the mode alone would report
STATE_MANUAL through the whole task.

So there are two ways to be autonomous here:

  1. A SELF-FLYING MODE. GUIDED, RTL and their variants: the autopilot has the
     aircraft regardless of what any mission is doing.
  2. AN EXECUTING MISSION. Any other mode, but only while the mission executive
     says a mission is actually running.

WHAT THIS DELIBERATELY EXCLUDES, and why it is a change. heartbeat_core's old
AUTONOMOUS_MODES listed LOITER, CIRCLE, LAND, FOLLOW and ZIGZAG unconditionally,
so a pilot hand-loitering the aircraft reported STATE_AUTO to RoboCommand. Those
modes are the autopilot holding a position or a pattern for a HUMAN; they are
autonomy only when a mission is driving them. Claiming autonomy we do not have
is the failure worth avoiding, so an unlisted mode with no mission running
reports STATE_MANUAL.

NEVER STATE_UNKNOWN. It is the proto zero value, the OCS validator refuses it
outright, and for a UAV nothing may be UNKNOWN at all. When the truth is not
knowable this returns None and the caller publishes nothing -- a gap reads on
the OCS as rising silence, a fabricated state does not.
"""
from __future__ import annotations

AUTO = "STATE_AUTO"
MANUAL = "STATE_MANUAL"

#: Modes in which an armed aircraft is flying itself whatever else is going on.
#: ArduCopter names. Kept narrow on purpose -- see the docstring.
SELF_FLYING_MODES = frozenset({
    "AUTO", "AUTO_RTL", "GUIDED", "GUIDED_NOGPS", "RTL", "SMART_RTL",
})

#: The proto's RxTask names. TASK_UNKNOWN is absent deliberately: it is the zero
#: value and the handbook forbids sending it.
TASK_NONE = "TASK_NONE"
TASKS = (
    TASK_NONE,
    "TASK_SAFE_PASSAGE",
    "TASK_INFRA_SURVEY_REPAIR",
    "TASK_COORDINATED_LOGISTICS",
    "TASK_DYNAMIC_INCIDENT",
)


def robot_state(mode, armed, mission_active=False):
    """(state, reason). state is None when it cannot be answered truthfully.

    mode:           FcuStatus.mode, or None if the stream is stale.
    armed:          FcuStatus.armed, or None if the stream is stale.
    mission_active: the mission executive is running a mission right now.
    """
    if mode is None or armed is None:
        return None, "fcu_status stale -- mode and armed both unknown"
    if not armed:
        # Disarmed is never autonomous, whatever mode is selected and whatever
        # a mission believes it is doing.
        return MANUAL, "disarmed"
    m = (mode or "").upper()
    if m in SELF_FLYING_MODES:
        return AUTO, "armed in %s" % m
    if mission_active:
        return AUTO, "armed in %s under an executing mission" % m
    return MANUAL, "armed in %s with no mission running" % m


def decide(mode, armed, *, mission_active=False, mission_name="",
           task=TASK_NONE, bench_auto=False):
    """The planner's whole answer: (state, task, reason, mission_active)."""
    if task not in TASKS:
        # A task the proto does not have would be refused by the OCS validator
        # anyway; failing here names the planner rather than the wire.
        raise ValueError("unknown task %r; expected one of %s" % (task, TASKS))

    state, reason = robot_state(mode, armed, mission_active)
    if bench_auto:
        # Reported in the reason so it can never be mistaken for a real mode on
        # a page or in a log.
        return AUTO, task, "BENCH_AUTO -- not from the autopilot (%s)" % reason, \
            mission_active
    return state, task, reason, mission_active
