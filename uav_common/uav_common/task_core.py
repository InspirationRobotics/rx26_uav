"""task_core -- which RoboNation task Ekko says it is attempting, and who says it.

ONE SOURCE, TWO LINKS. /uav/current_task is read by BOTH things that report to the
OCS: ocs_client (the 2 Hz heartbeat over WiFi) and telemetry_bridge (the same
heartbeat over the RFD radio, plus the Task 1 SafePassageReport). The OCS falls
back to the WiFi heartbeat when the radio's goes quiet, and the handbook (3.4.11)
makes the transition INTO a task value the start of that attempt and the
transition to TASK_NONE the stand-down. If the radio said TASK_SAFE_PASSAGE and the
fallback then said TASK_NONE, the fallback itself would end the attempt. So both
links parse the topic with THIS file and cannot disagree about what a token means.

Mirrors the boat: /crsd/current_task, std_msgs/String, RELIABLE + TRANSIENT_LOCAL,
depth 1 (Boat/rx26_asv/crusader_groundstation/.../ocs_client_node.py, ocs_link.py
coerce_task and RX_TASKS). The tokens are RoboCommand's own RxTask names
(rx_common.proto), transcribed -- change them here in the same commit if RoboNation
ever changes the enum.

TRANSIENT_LOCAL because the topic is published on TRANSITIONS, not periodically: a
VOLATILE subscriber that started (or restarted) mid-attempt would never learn the
task was under way. Nothing published yet is TASK_NONE.

WHAT IS DELIBERATELY NOT PORTED FROM THE BOAT: its "the publisher vanished, so stand
down to TASK_NONE" liveness check. `ros2 topic pub --once` -- the bench way to set
the task -- exits as soon as it has published, which would stand the task down
again at once. When a real mission node publishes this topic, revisit it: until
then a task set here stays set until something publishes another.

Pure apart from CurrentTask.subscribe(), whose rclpy imports are function-local so
the rest imports (and is tested) anywhere.
"""

#: RoboCommand's RxTask names the OCS can parse. TASK_UNKNOWN (the proto's zero
#: value, "unset or unparsed -- NOT a stand-down signal") is deliberately absent:
#: sending it claims nothing while looking like a report, and the OCS refuses it.
RX_TASKS = frozenset((
    "TASK_NONE",                    # deliberately idle / not attempting a task
    "TASK_SAFE_PASSAGE",            # Task 1
    "TASK_INFRA_SURVEY_REPAIR",     # Task 2
    "TASK_COORDINATED_LOGISTICS",   # Task 3
    "TASK_DYNAMIC_INCIDENT",        # Task 4
))

TASK_NONE = "TASK_NONE"
TASK_SAFE_PASSAGE = "TASK_SAFE_PASSAGE"

TOPIC = "/uav/current_task"


def coerce_task(token):
    """(token, None) for an RxTask name, else (None, why).

    Tolerates case and surrounding whitespace, as the boat does ("  task_safe_
    passage  " is TASK_SAFE_PASSAGE). Anything else is NOT replaced by a
    substitute: the caller keeps what it had, because a typo is not an order to
    stand down.
    """
    name = str(token).strip().upper()
    if name in RX_TASKS:
        return name, None
    return None, ("%r is not an RxTask the OCS can parse (valid: %s)"
                  % (name, ", ".join(sorted(RX_TASKS))))


class CurrentTask:
    """The task Ekko currently claims: TASK_NONE until something publishes one.

    An unrecognised value is logged (once per distinct bad token, so a typo
    republished at 2 Hz does not bury the log) and IGNORED: the previous task
    stays. Falling back to TASK_NONE instead would be an unrequested stand-down.
    """

    def __init__(self):
        self.value = TASK_NONE
        self._warned = set()

    def update(self, raw):
        """Apply one published value -> (changed, error).

        `error` is the reason an unrecognised value was ignored, and only the
        FIRST time that value is seen; None otherwise.
        """
        token, why = coerce_task(raw)
        if token is None:
            if str(raw) in self._warned:
                return False, None
            self._warned.add(str(raw))
            return False, "ignoring task token: " + why
        changed = token != self.value
        self.value = token
        return changed, None

    def subscribe(self, node):
        """Subscribe `node` to TOPIC and keep this object current, logging each
        change on the node's logger. The one place the QoS lives, so the two
        subscribing nodes cannot drift apart."""
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        def on_task(msg):
            changed, error = self.update(msg.data)
            if error:
                node.get_logger().error(error)
            elif changed:
                node.get_logger().info("current_task -> %s" % self.value)

        return node.create_subscription(
            String, TOPIC, on_task,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
