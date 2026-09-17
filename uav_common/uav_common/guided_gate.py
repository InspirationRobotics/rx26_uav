"""guided_gate — may this GuidedTarget reach the autopilot, and does it need to.

PURE: no ROS, no MAVLink. telemetry_bridge asks refusal() about every target
before it becomes a SET_POSITION_TARGET_GLOBAL_INT, and Resender whether the
autopilot already has it. bench_search exercises both.

This is the second of two locks, and deliberately does not trust the first.
search_core already refuses to plan outside the fence or off its altitude; this
file assumes it might be wrong anyway -- or that something else entirely has
started publishing /uav/guided_target -- and checks the target against what the
AUTOPILOT reports: its mode, its arm state, its fence, its ceiling. Every check
fails closed: an unknown is a refusal, never a pass.
"""
import math

from uav_common import fcu_decode, geo

#: The autopilot's mode and arm state count as known for this long after the last
#: HEARTBEAT. Heartbeats are 1 Hz, and the 1.0 s republishing timeout the rest of
#: the bridge uses flickers to "unknown" between two of them (seen in SITL: six
#: times in eight minutes), which here would refuse an RTL request or a changed
#: target at random. 2.5 s is two missed heartbeats. The autopilot enforces
#: GUIDED itself either way; this check is the second lock, not the first.
STATUS_MAX_AGE_S = 2.5
#: A target older than this is not flown. The search publishes at 5 Hz; anything
#: this late is a backlog or a node that has stopped, not an instruction.
TARGET_MAX_AGE_S = 1.0
#: Below this, a target is a landing, and nothing here lands.
MIN_ALT_M = 2.0
#: With no altitude fence enabled, still never above RoboNation's 60 m cap.
HARD_CEILING_M = 60.0
SPEED_RANGE_MPS = (0.2, 8.0)
#: The same target is sent again this often. Resending an unchanged position
#: target restarts the autopilot's leg, so it is not sent at the publish rate --
#: only when it changes, on every fresh entry into GUIDED, and on this period in
#: case one was lost.
RESEND_S = 5.0
MOVED_M = 0.3
ALT_CHANGED_M = 0.3
YAW_CHANGED_DEG = 3.0


def _finite(*xs):
    return all(x is not None and not math.isnan(x) for x in xs)


def refusal(lat, lon, alt_m, speed_mps, age_s, status, fence, fence_type,
            fence_alt_max, fence_margin):
    """"" if the target may be flown, else why not.

    status: (mode, armed) from a FRESH heartbeat, or None.
    fence:  [(lat, lon)] the autopilot holds, or None if not read / not valid.
    fence_*: the autopilot's parameters, None where not read yet.
    """
    if status is None:
        return "autopilot status unknown"
    mode, armed = status
    if mode != "GUIDED":
        return "autopilot is in %s, not GUIDED" % mode
    if not armed:
        return "not armed"
    if age_s is None or age_s > TARGET_MAX_AGE_S or age_s < -TARGET_MAX_AGE_S:
        return "target is stale (%s s old)" % ("?" if age_s is None else "%.1f" % age_s)
    if not _finite(lat, lon, alt_m, speed_mps):
        return "target has a blank position, altitude or speed"
    if not fence:
        return "no valid fence read from the autopilot"
    if not geo.point_in_polygon(lat, lon, fence):
        return "target is outside the fence"
    if alt_m < MIN_ALT_M:
        return "target altitude %.1f m is below %.0f m" % (alt_m, MIN_ALT_M)
    if fence_type is None:
        return "FENCE_TYPE not read yet"
    if int(fence_type) & fcu_decode.FENCE_TYPE_ALT_MAX:
        if fence_alt_max is None:
            return "FENCE_ALT_MAX not read yet"
        stop = fence_alt_max - (fence_margin or 0.0)
        if alt_m > stop + 0.05:
            return ("target altitude %.1f m is above FENCE_ALT_MAX - FENCE_MARGIN "
                    "= %.1f m" % (alt_m, stop))
    elif alt_m > HARD_CEILING_M:
        return "target altitude %.0f m is above the %.0f m cap" % (alt_m, HARD_CEILING_M)
    lo, hi = SPEED_RANGE_MPS
    if not lo <= speed_mps <= hi:
        return "speed %.1f m/s outside %.1f-%.1f" % (speed_mps, lo, hi)
    return ""


class Resender:
    """Send a target only when the autopilot does not already have it."""

    def __init__(self):
        self.last = None          # (lat, lon, alt, yaw, epoch, sent_at)
        self.speed = None         # (speed, epoch, sent_at)

    def target_due(self, lat, lon, alt_m, yaw_deg, epoch, now):
        """epoch: bumps on every entry into GUIDED, which resets the target."""
        if self.last is None:
            return True
        l_lat, l_lon, l_alt, l_yaw, l_epoch, l_t = self.last
        if epoch != l_epoch or now - l_t >= RESEND_S:
            return True
        dx, dy = geo.latlon_to_xy(lat, lon, (l_lat, l_lon))
        if math.hypot(dx, dy) > MOVED_M or abs(alt_m - l_alt) > ALT_CHANGED_M:
            return True
        if math.isnan(yaw_deg) != math.isnan(l_yaw):
            return True
        if not math.isnan(yaw_deg):
            d = abs((yaw_deg - l_yaw + 180.0) % 360.0 - 180.0)
            if d > YAW_CHANGED_DEG:
                return True
        return False

    def target_sent(self, lat, lon, alt_m, yaw_deg, epoch, now):
        self.last = (lat, lon, alt_m, yaw_deg, epoch, now)

    def speed_due(self, speed_mps, epoch, now):
        return (self.speed is None or abs(self.speed[0] - speed_mps) > 0.05
                or self.speed[1] != epoch or now - self.speed[2] >= RESEND_S)

    def speed_sent(self, speed_mps, epoch, now):
        self.speed = (speed_mps, epoch, now)
