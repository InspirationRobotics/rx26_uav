"""ocs_client — publishes the UAV's heartbeat to the Operator Control Station.

NOT PROVEN IN FLIGHT. It is started by systemd (tools/systemd/uav-ocs-client
.service) because nobody will SSH into this Jetson between flights, but it has
never carried a scored run. `systemctl disable uav-ocs-client` is the way back.

    ros2 run uav_groundstation ocs_client

The OCS holds the team's single connection to RoboNation's RoboCommand and is
the only thing allowed to talk to it. This node's whole job is to keep the OCS
supplied with a 2 Hz heartbeat — the rate the handbook mandates for every active
vehicle — over one TCP connection that also carries commands back.

WHAT THIS NODE MUST NEVER DO IS ACT. Inbound commands are republished to
/uav/ocs_command and nothing else. A mission executive may choose to read them;
this node has no opinion. WiFi is a convenience, never a control path, and a
shore network that can move the aircraft inverts that.

THIS FILE IS THE PLUMBING ONLY. What the heartbeat says, and — more importantly
— when it must say nothing, lives in heartbeat_core, which has no ROS in it and
can be exercised directly (tools/bench/bench_heartbeat.py). The rules there are
the safety-relevant part: an aircraft may not report FLIGHT_PHASE_UNKNOWN,
because that field is relayed to Garuda Robotics for Singapore's Network Remote
ID, and a guess there misinforms a regulator rather than a scoreboard.

This node's own job is the part a core cannot do: decide what is FRESH. Every
value handed to heartbeat_core has passed a StreamCache freshness check, and a
stale one arrives as None. That division is deliberate — staleness needs a clock
and a subscription, truthfulness does not.
"""
import json
import time
from collections import deque

from rclpy.node import Node
from std_msgs.msg import String

from uav_msgs.msg import Attitude, FcuStatus, FlightState, GlobalPos

from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
from uav_common.stream_cache import StreamCache

from uav_groundstation import heartbeat_core
from uav_groundstation.ocs_link import OcsLink, fake_report, rfc3339

PARAM_SPEC = {
    "ocs_host": dict(read_only=True,
                     description="the OCS on the TEAM subnet (192.168.8.107). "
                                 "No discovery: give it a DHCP reservation. "
                                 "NOT 127.0.0.1, which is this Jetson."),
    "ocs_port": dict(read_only=True, lo=1024, hi=65535,
                     description="= [tcp] port in the OCS bridge.toml"),
    "vehicle_id": dict(read_only=True,
                       description="MUST match a [[vehicle]] id in the OCS "
                                   "bridge.toml, or every frame is dropped"),
    "team_id": dict(read_only=True, description="= bridge.toml team_id"),
    "rate_hz": dict(read_only=True, lo=0.1, hi=10.0,
                    description="2.0 is the handbook's mandated heartbeat rate"),
    "fake_telemetry": dict(read_only=True,
                           description="synthesize a circle instead of "
                                       "subscribing; for bench use with no "
                                       "Pixhawk. Logs a warning while on."),
    "geoid_separation_m": dict(read_only=True, lo=-120.0, hi=120.0,
                               description="HAE = AMSL + this. VENUE-SPECIFIC; "
                                           "check before the competition"),
    "airborne_alt_m": dict(read_only=True, lo=0.2, hi=50.0,
                           description="fallback airborne threshold on "
                                       "altitude_rel, used only when "
                                       "EXTENDED_SYS_STATE is absent"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s; stale -> no "
                                       "heartbeat at all"),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                               description="stale -> roll/pitch omitted"),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="stale -> no heartbeat, since state "
                                         "cannot be known"),
    "flight_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="stale -> fall back to armed+altitude, "
                                         "and say so"),
}


class OcsClient(Node):
    def __init__(self):
        super().__init__("ocs_client")
        p = declare_from_config(self, uav_config.node_params("ocs_client"),
                                PARAM_SPEC)
        self.p = p

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._flight = StreamCache(p["flight_timeout_s"])
        self._t0 = time.time()
        self._skipped = 0
        self._phase_source = None       # "autopilot" | "fallback" | None
        self._quiet_reason = None

        # Commands arrive on the link thread; rclpy publishers are not promised
        # to be thread-safe, so they are parked here and published from the
        # timer. Bounded: if nothing is draining, dropping the oldest advisory
        # is better than growing without limit.
        self._inbox = deque(maxlen=32)
        self._cmd_pub = self.create_publisher(String, "/uav/ocs_command", 10)

        if p["fake_telemetry"]:
            self.get_logger().warning(
                "fake_telemetry is ON — this node is publishing invented "
                "positions to the OCS. Never leave this set for a scored run.")
        else:
            self.create_subscription(GlobalPos, "/uav/pose", self._on_pose, 10)
            self.create_subscription(Attitude, "/uav/attitude", self._on_att, 10)
            self.create_subscription(FcuStatus, "/uav/fcu_status",
                                     self._on_status, 10)
            self.create_subscription(FlightState, "/uav/flight_state",
                                     self._on_flight, 10)

        self.link = OcsLink(p["ocs_host"], int(p["ocs_port"]),
                            on_command=self._inbox.append,
                            log=lambda m: self.get_logger().info(str(m)))
        self.link.start()
        self.get_logger().info(
            "ocs_client: %s -> %s:%d at %.1f Hz (geoid sep %.1f m)"
            % (p["vehicle_id"], p["ocs_host"], int(p["ocs_port"]),
               p["rate_hz"], p["geoid_separation_m"]))

        self.create_timer(1.0 / float(p["rate_hz"]), self._tick)

    def destroy_node(self):
        self.link.stop()
        return super().destroy_node()

    # ---- subscriptions ---------------------------------------------------

    def _on_pose(self, msg):
        self._pose.set(msg, time.monotonic())

    def _on_att(self, msg):
        self._att.set(msg, time.monotonic())

    def _on_status(self, msg):
        self._status.set(msg, time.monotonic())

    def _on_flight(self, msg):
        self._flight.set(msg, time.monotonic())

    # ---- link state, for the ground station page --------------------------

    def link_state(self):
        sent, received = self.link.counts
        return {"connected": self.link.connected, "sent": sent,
                "received": received, "skipped": self._skipped,
                "phase_source": self._phase_source,
                "quiet_reason": self._quiet_reason}

    # ---- the heartbeat ---------------------------------------------------

    def _tick(self):
        while self._inbox:
            self._republish(self._inbox.popleft())

        report = self._build(time.monotonic())
        if report is None:
            return
        self.link.publish(report)

    def _republish(self, cmd):
        """Hand an OCS command to whoever wants it. This node does not act."""
        self._cmd_pub.publish(String(data=json.dumps(cmd)))
        self.get_logger().info("OCS command -> /uav/ocs_command: %s"
                               % sorted(cmd))

    def _build(self, now):
        p = self.p
        if p["fake_telemetry"]:
            return fake_report(p["vehicle_id"], p["team_id"], self._t0)

        # Freshness is decided HERE and nowhere else; heartbeat_core sees only
        # values or None. A stale FlightState arrives as landed=None, which is
        # what sends it to the armed+altitude fallback.
        fs = self._flight.get(now)
        hb, info = heartbeat_core.build_heartbeat(
            pose=self._pose.get(now),
            status=self._status.get(now),
            attitude=self._att.get(now),
            landed=(fs.landed_state if fs is not None and fs.valid else None),
            geoid_separation_m=p["geoid_separation_m"],
            airborne_alt_m=p["airborne_alt_m"])

        if hb is None:
            self._skipped += 1
            # Once per change of reason, not twice a second. The reason is the
            # whole value of the message: "no heartbeat" alone gets debugged by
            # guesswork at a flight line.
            if info != self._quiet_reason:
                self._quiet_reason = info
                self.get_logger().warning(
                    "no heartbeat: %s — the OCS will show rising silence, "
                    "which is the truth" % info)
            return None

        self._quiet_reason = None
        if info != self._phase_source:
            self._phase_source = info
            self.get_logger().info("flight_phase source: %s" % info)
        if info == "fallback":
            self.get_logger().warning(
                "flight_phase from armed+altitude, NOT from the autopilot — "
                "/uav/flight_state is stale or absent. Set SR0_EXT_STAT > 0.",
                throttle_duration_sec=15.0)

        return {"team_id": p["team_id"], "vehicle_id": p["vehicle_id"],
                "sent_at": rfc3339(), "heartbeat": hb}


def main(args=None):
    run_node(OcsClient, args)


if __name__ == "__main__":
    main()
