"""mission_planner — the one node that decides what state and task the UAV is in.

    ros2 run uav_groundstation mission_planner

NOT PROVEN IN FLIGHT. Started by hand or by its own systemd unit; deliberately
absent from core.launch.py until it has carried a flight.

WHY THIS NODE EXISTS. The autonomy decision used to live inside heartbeat_core,
in the same path whose job is to transmit it, as a flat list of modes. That list
counted LOITER as autonomous unconditionally — so a pilot hand-loitering the
aircraft reported STATE_AUTO to RoboCommand and, through the Network Remote ID
relay, to a regulator. The rule needs a fact the mode cannot carry: whether a
mission is executing. One node owns it now, and the rule lives in mission_core
where it can be exercised with no rclpy in the way.

WHAT IT WILL BECOME. This is the seed of the mission executive: it already holds
the shape (state, task, whether a mission is running, which one) that starting
and sequencing missions needs. Today it only reports, which is all the
Communications Proof of Readiness requires. `set_mission()` is the seam — when
the executive arrives it calls that, and everything downstream already works.

WHAT IT MUST NEVER DO. It does not change flight mode, arm, or disarm. An OCS
standby-auto directive arrives here as a request, is logged, and changes
nothing. The standing rule holds without exception: the RC transmitter is the
only safety path, and WiFi is never a safety mechanism. A pilot puts this
aircraft into GUIDED.
"""
import json
import time

from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
from uav_common.stream_cache import StreamCache
from uav_msgs.msg import FcuStatus, MissionState
from rclpy.node import Node
from std_msgs.msg import String

from uav_groundstation import mission_core

PARAM_SPEC = {
    "rate_hz": dict(read_only=True, lo=0.5, hi=20.0,
                    description="how often mission_state is published; must be "
                                "at least the OCS heartbeat rate or ocs_client "
                                "sees it as stale"),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="stale fcu_status -> publish nothing"),
    "bench_auto": dict(read_only=True,
                       description="report STATE_AUTO with no autopilot, for "
                                   "producing rc-test logs on the ground. "
                                   "NEVER true for a scored flight."),
}


class MissionPlanner(Node):
    def __init__(self):
        super().__init__("mission_planner")
        p = declare_from_config(
            self, uav_config.node_params("mission_planner"), PARAM_SPEC)
        self.p = p

        self._status = StreamCache(p["status_timeout_s"])
        self._task = mission_core.TASK_NONE
        self._mission_active = False
        self._mission_name = ""
        self._last_published = None
        self._quiet_reason = None

        self.pub = self.create_publisher(MissionState, "/uav/mission_state", 10)
        self.create_subscription(FcuStatus, "/uav/fcu_status", self._on_status, 10)
        # Advisory input from the OCS, republished by ocs_client. Read, logged,
        # never acted on -- see the module docstring.
        self.create_subscription(String, "/uav/ocs_directive",
                                 self._on_directive, 10)

        if p["bench_auto"]:
            self.get_logger().warning(
                "bench_auto is ON — this node will report STATE_AUTO regardless "
                "of what the autopilot is doing. Never leave this set for a "
                "scored flight.")

        self.create_timer(1.0 / float(p["rate_hz"]), self._tick)
        self.get_logger().info(
            "mission_planner up: %.1f Hz. Self-flying modes %s; any other mode "
            "counts as autonomous only while a mission is executing."
            % (p["rate_hz"], ", ".join(sorted(mission_core.SELF_FLYING_MODES))))

    # ---- the seam the mission executive will use --------------------------

    def set_mission(self, name: str, task: str, active: bool) -> None:
        """Declare what the aircraft is doing. Not called by anything yet.

        This is the whole reason the node is shaped this way: when a mission
        starts flying the aircraft in LOITER under CV control, it says so here,
        and the state reported to RoboCommand becomes STATE_AUTO without the
        autopilot mode changing at all.
        """
        self._mission_name = name
        self._task = task
        self._mission_active = bool(active)
        self.get_logger().info(
            "mission %s: %s task=%s" % (name or "(none)",
                                        "ACTIVE" if active else "idle", task))

    # ---- inputs -----------------------------------------------------------

    def _on_status(self, msg):
        self._status.set(msg, time.monotonic())

    def _on_directive(self, msg):
        """An OCS directive. We note it; the autopilot is not touched."""
        try:
            d = json.loads(msg.data).get("ocs_directive", {})
        except (ValueError, AttributeError):
            self.get_logger().warning("undecodable OCS directive, ignored")
            return
        action = d.get("action", "?")
        if action == "standby_auto":
            self.get_logger().info(
                "OCS asks us to stand by in auto for declaration_seq=%s. "
                "NOT changing mode — a pilot puts this aircraft in GUIDED."
                % d.get("declaration_seq"))
        else:
            self.get_logger().info("OCS directive %r noted, no action" % action)

    # ---- the answer -------------------------------------------------------

    def _tick(self):
        now = time.monotonic()
        status = self._status.get(now)
        state, task, reason, mission_active = mission_core.decide(
            mode=None if status is None else status.mode,
            armed=None if status is None else status.armed,
            mission_active=self._mission_active,
            mission_name=self._mission_name,
            task=self._task,
            bench_auto=bool(self.p["bench_auto"]))

        if state is None:
            # Publish nothing rather than guess. ocs_client sees mission_state
            # go stale and lets the heartbeat lapse, which is the truth.
            if reason != self._quiet_reason:
                self._quiet_reason = reason
                self.get_logger().warning(
                    "no mission_state: %s — the OCS will show rising silence"
                    % reason)
            return
        self._quiet_reason = None

        if (state, task) != self._last_published:
            self._last_published = (state, task)
            self.get_logger().info("state=%s task=%s (%s)" % (state, task, reason))

        m = MissionState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = state
        m.task = task
        m.reason = reason
        m.mission_active = bool(mission_active)
        m.mission_name = self._mission_name
        self.pub.publish(m)


def main(args=None):
    run_node(MissionPlanner, args)


if __name__ == "__main__":
    main()
