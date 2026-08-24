"""heartbeat_core — what the OCS heartbeat says, and when it must say nothing.

Pure: no ROS, no sockets, no clock. ocs_client_node is the thin wrapper that
feeds it freshness-checked telemetry and hands the result to the link. The split
follows the fleet's standing convention that safety logic belongs in a core
that can be read and exercised without rclpy between you and it — and this is
safety logic, in the specific sense that a wrong answer here is relayed to a
regulator.

THE ONE RULE THIS FILE ENFORCES: never invent. Every path that cannot produce a
truthful field returns None, meaning "send no heartbeat". A gap is visible on
the OCS as rising silence; a fabricated value is indistinguishable from a real
one, and scores — or is filed with Singapore's Network Remote ID — as though the
aircraft were somewhere, or in a state, that it is not.

WHY flight_phase IS MANDATORY HERE AND OPTIONAL ON THE BOAT.
rx26_ocs/rx_bridge/config.py:

    "TYPE_USV": ("flight_phase",)     # may be UNKNOWN
    "TYPE_UUV": ("flight_phase",)     # may be UNKNOWN
    "TYPE_UAV": ()                    # NOTHING may be UNKNOWN

A hull has no honest flight phase and the proto offers no "not applicable"
value, so UNKNOWN is truthful from a boat. From an aircraft it means the
reporter forgot.
"""

# MAV_LANDED_STATE values (mirrors uav_msgs/FlightState.msg, kept as plain ints
# so this module imports with no ROS present).
LANDED_UNDEFINED = 0
LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2
LANDED_TAKEOFF = 3
LANDED_LANDING = 4

GROUNDED = "FLIGHT_PHASE_GROUNDED"
AIRBORNE = "FLIGHT_PHASE_AIRBORNE"

# TAKEOFF and LANDING are AIRBORNE: the aircraft is off the ground in both, and
# the proto offers only the two values. Reporting a mid-takeoff aircraft as
# grounded is the more dangerous of the two roundings.
_PHASE_BY_LANDED = {
    LANDED_ON_GROUND: GROUNDED,
    LANDED_IN_AIR: AIRBORNE,
    LANDED_TAKEOFF: AIRBORNE,
    LANDED_LANDING: AIRBORNE,
}

#: Modes in which an armed ArduCopter is flying itself. Anything else — and
#: anything at all while disarmed — reports STATE_MANUAL. STATE_UNKNOWN is the
#: proto zero value and is never sent.
AUTONOMOUS_MODES = frozenset({
    "AUTO", "GUIDED", "GUIDED_NOGPS", "RTL", "SMART_RTL", "LAND", "LOITER",
    "AUTO_RTL", "CIRCLE", "FOLLOW", "ZIGZAG",
})


def flight_phase(landed, *, armed=None, altitude_rel=None, airborne_alt_m=1.0):
    """(phase, source). phase is None when it cannot be determined truthfully.

    Args:
      landed: the autopilot's landed_state, or None when /uav/flight_state is
        stale or absent. An UNDEFINED landed_state is NOT the same as absent —
        the autopilot is saying it does not know — but both fall through to the
        same fallback, because in each case we have no answer from it.
      armed, altitude_rel: for the fallback. Both must be known.
      airborne_alt_m: fallback threshold on height above home.

    source is "autopilot", "fallback", or None — the caller logs it so which
    one is live is visible rather than inferred from behaviour.
    """
    if landed is not None and landed != LANDED_UNDEFINED:
        phase = _PHASE_BY_LANDED.get(landed)
        if phase is not None:
            return phase, "autopilot"
        # A landed_state outside the mapping means a newer MAVLink than this
        # table. Guessing is exactly what the UNKNOWN ban forbids.
        return None, None

    if armed is None or altitude_rel is None:
        return None, None
    airborne = bool(armed) and altitude_rel > airborne_alt_m
    return (AIRBORNE if airborne else GROUNDED), "fallback"


def robot_state(mode, armed):
    """RobotState for the heartbeat. Never STATE_UNKNOWN (the zero value)."""
    if not armed:
        return "STATE_MANUAL"
    return "STATE_AUTO" if (mode or "").upper() in AUTONOMOUS_MODES \
        else "STATE_MANUAL"


def build_heartbeat(*, pose, status, attitude, landed, geoid_separation_m,
                    airborne_alt_m):
    """The heartbeat body, or (None, reason) when it must not be sent.

    Returns (dict, source) on success and (None, reason) on suppression, so the
    caller can log WHY it went quiet — "no heartbeat" with no reason is the kind
    of silence that gets debugged by guesswork at a flight line.

    `pose`, `status` and `attitude` are already freshness-checked by the caller:
    None means stale or never seen. Passing a stale value in is the one thing
    this function cannot detect, which is why the StreamCache API makes the
    stale case return None rather than offering a bare accessor.
    """
    if pose is None:
        return None, "pose is stale"
    if status is None:
        return None, "fcu_status is stale"

    phase, source = flight_phase(
        landed,
        armed=None if status is None else status.armed,
        altitude_rel=None if pose is None else pose.altitude_rel,
        airborne_alt_m=airborne_alt_m)
    if phase is None:
        return None, ("flight phase unknown — the OCS refuses "
                      "FLIGHT_PHASE_UNKNOWN from a UAV, and a guess is relayed "
                      "for Network Remote ID")

    hb = {
        "state": robot_state(status.mode, status.armed),
        "position": {"latitude": pose.latitude, "longitude": pose.longitude},
        "spd_mps": pose.ground_speed,
        # Passed through as-is, NaN and all. telemetry_bridge sets it NaN when
        # GPS yaw is unresolved and the OCS validator exists to catch exactly
        # that — scrubbing it here would hide a real fault.
        "heading_deg": pose.heading,
        # AMSL -> HAE. These are NOT the same number; see the module header of
        # ocs_client_node and the venue warning on geoid_separation_m.
        "altitude_hae_m": pose.altitude_amsl + geoid_separation_m,
        "depth_m": 0.0,
        "vehicle_type": "TYPE_UAV",
        # No task-state source exists on the aircraft yet. TASK_NONE is the
        # honest value and TASK_UNKNOWN is refused by the OCS, so this is not a
        # placeholder that can silently rot.
        "current_task": "TASK_NONE",
        "flight_phase": phase,
    }
    if attitude is not None:
        import math
        # Attitude is radians in the autopilot's own NED axes; the report wants
        # degrees. Attitude going stale costs two fields, not the heartbeat.
        hb["roll_deg"] = math.degrees(attitude.roll)
        hb["pitch_deg"] = math.degrees(attitude.pitch)
    return hb, source
