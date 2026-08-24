"""telemetry_bridge — the single ROS-side gateway to MAVProxy's rebroadcast.

Three jobs, deliberately fused into one node because all three need the one
MAVLink connection and there may only ever be one:

1. RX: consume MAVProxy's rebroadcast (pymavlink over UDP — NEVER a serial
   device; the Pixhawk has exactly one owner and it is MAVProxy) and republish
   as topics:
     /uav/pose           uav_msgs/GlobalPos    (GLOBAL_POSITION_INT)
     /uav/attitude       uav_msgs/Attitude     (ATTITUDE, on arrival)
     /uav/fcu_status     uav_msgs/FcuStatus    (HEARTBEAT)
     /uav/flight_state   uav_msgs/FlightState  (EXTENDED_SYS_STATE)
     /uav/rc_channels    uav_msgs/RcChannels   (RC_CHANNELS)
     /uav/autonomy_drop  std_msgs/Bool         (latched, TRANSIENT_LOCAL)
   Other nodes subscribe to these instead of opening their own MAVLink
   connection — this node existing is what keeps the single-owner rule
   enforceable rather than merely stated.

2. TX (autonomy): the ONLY sanctioned path for RC overrides. Nodes publish
   uav_msgs/RcChannels on /uav/rc_override; this node forwards them to the
   autopilot — UNLESS the autonomy-drop latch (uav_common.drop_latch) has
   tripped, in which case it sends release frames (all-zero override) and drops
   every subsequent override until the operator resets via the
   /uav/autonomy_drop_reset service. Because misbehaving nodes have no MAVLink
   connection of their own, a tripped latch cannot be bypassed from the ROS
   graph.

   NOTE: this repo currently ships NO publisher on /uav/rc_override. The
   override path and its latch are kept here so the enforcement point exists
   before anything needs it, but nothing exercises them yet.

3. TX (geofence): uploads the competition geofence to the autopilot as an
   inclusion fence, via the /uav/fence_upload service. See fence_core.

THERE IS NO DISARM PATH IN THIS NODE, AND THAT IS DELIBERATE.
-------------------------------------------------------------
The ASV's telemetry_bridge carries a force-disarm TX path — a
MAV_CMD_COMPONENT_ARM_DISARM with the 21196 force magic, driven by its RC-loss
watchdog. Do not port it here. On a boat a force-disarm stops the thrusters and
the hull floats; on a multirotor it stops the motors and the aircraft falls out
of the sky. The two look like the same command and are opposite in effect.

RC-loss on this vehicle is handled where it belongs: the autopilot's own
failsafe parameters, which can RTL or LAND without a companion computer being
alive to have an opinion. A ROS node cannot do better than that and can very
easily do worse — it is on the wrong side of the link that just failed.

The autonomy-drop latch above is NOT a disarm. Tripping it releases overridden
channels back to the pilot and leaves the aircraft flying.

WHY ATTITUDE IS ITS OWN TOPIC, and published from the RX thread rather than on
the 20 Hz tick: ATTITUDE is the one stream whose value is the instantaneous
number, not the latest known state. Resampling a 30 Hz stream onto a 20 Hz tick
drops one frame in three and time-shifts the rest by up to 50 ms — at 2 rad/s
that is several degrees of error. Publishing on arrival also makes the staleness
rule below automatic for this topic: there is no cached value to replay, so
silence is silence for free. The StreamCache is kept anyway, purely so
_publish_tick still logs the stale edge.

Staleness rule (safety-relevant): each RX stream is republished ONLY while it is
fresh, and its header carries the stamp captured at RECEIPT. Rebroadcasting the
last cached frame with a fresh stamp makes a dead MAVProxy indistinguishable
from a healthy one. Downstream that is worse than silence: ocs_client is
required to send a 2 Hz heartbeat and equally required not to invent one, and a
frozen cache turns "the link died" into a stream of confident wrong positions
relayed onward for Network Remote ID. Silence must stay silent.

Parameters:
  mav_endpoint       (str,  udp:127.0.0.1:14541)  MAVProxy --out for ROS.
                     14541, NOT the ASV's 14551 — see the port table in README.
  mav_source_system  (int,  200)   our MAVLink sysid; must differ from MAVProxy's
  drop_channel       (int,  7)     RC channel of the autonomy-drop switch
  drop_threshold     (int,  1700)  us; >= trips (or <= if drop_invert)
  drop_invert        (bool, False)
  rc_stale_timeout   (float, 1.0)  s without RC_CHANNELS -> trip
  stream_timeout_s   (float, 1.0)  s without a frame before a stream stops being
                                   republished
  geofence           (float[])     FLAT [lat, lon, lat, lon, ...], CLOSED;
                                   uploaded on request. Flat because ROS
                                   parameters cannot nest — see
                                   fence_core.polygon_from_flat
  fence_timeout_s    (float, 5.0)  per-exchange timeout in the mission dialog
"""
import queue
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from uav_msgs.msg import Attitude, FcuStatus, FlightState, GlobalPos, RcChannels

from uav_common import config as uav_config
from uav_common import geo
from uav_common.drop_latch import DropLatch
from uav_common.fence_core import (FenceError, FenceProtocol, MavFenceTransport,
                                   items_from_polygon, polygon_from_flat)
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
from uav_common.stream_cache import StreamCache

# All bridge params are SAFETY/STRUCTURAL CONFIG -> read_only: `ros2 param set`
# is rejected; the change path is config/uav_params.yaml + node restart.
PARAM_SPEC = {
    "mav_endpoint": dict(read_only=True,
                         description="MAVProxy rebroadcast (udp/tcp only)"),
    "mav_source_system": dict(read_only=True, lo=1, hi=254,
                              description="our MAVLink sysid; must differ from "
                                          "MAVProxy's 255 or mission replies "
                                          "are ambiguous"),
    "drop_channel": dict(read_only=True, lo=1, hi=18,
                         description="autonomy-drop RC channel"),
    "drop_threshold": dict(read_only=True, lo=800, hi=2200,
                           description="us; crossing trips the latch"),
    "drop_invert": dict(read_only=True, description="low = drop position"),
    "rc_stale_timeout": dict(read_only=True, lo=0.2, hi=10.0,
                             description="s without RC before trip"),
    "stream_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="s without a MAVLink frame before "
                                         "that stream stops being republished"),
    # FLAT [lat, lon, lat, lon, ...]: ROS parameters cannot nest. See
    # fence_core.polygon_from_flat.
    "geofence": dict(read_only=True,
                     description="FLAT [lat,lon,...] CLOSED ring; MUST equal "
                                 "uav_geofence in the OCS bridge.toml"),
    "fence_timeout_s": dict(read_only=True, lo=1.0, hi=60.0,
                            description="per-exchange timeout, fence dialog"),
}

RELEASE_FRAMES = 5          # all-zero override frames sent on trip
PUB_RATE_HZ = 20.0

#: MISSION_* messages the fence dialog consumes. Routed off the RX thread into a
#: queue rather than handled there, so a blocking request/response exchange
#: never stalls telemetry republishing.
_MISSION_TYPES = ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
                  "MISSION_COUNT", "MISSION_ITEM", "MISSION_ITEM_INT")


class TelemetryBridge(Node):

    def __init__(self):
        super().__init__("telemetry_bridge")
        p = declare_from_config(self, uav_config.node_params("telemetry_bridge"),
                                PARAM_SPEC)
        self.p = p

        endpoint = p["mav_endpoint"]
        if not (endpoint.startswith("udp") or endpoint.startswith("tcp")):
            # never a serial device — single-Pixhawk-owner rule, fail loudly
            raise ValueError(
                "mav_endpoint %r is not udp/tcp; refusing (MAVProxy is the sole "
                "Pixhawk owner; this node consumes its rebroadcast)" % endpoint)

        # Validated at construction, not at the first service call. A malformed
        # geofence is a config error, and finding it when someone presses the
        # button — which will be on a flight line — is finding it too late.
        self._fence_items = items_from_polygon(polygon_from_flat(p["geofence"]))
        self.get_logger().info(
            "geofence: %d vertices, ready to upload on /uav/fence_upload"
            % len(self._fence_items))

        self.latch = DropLatch(
            channel=p["drop_channel"],
            threshold=p["drop_threshold"],
            invert=p["drop_invert"],
            stale_timeout=p["rc_stale_timeout"])

        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(GlobalPos, "/uav/pose", 10)
        self.att_pub = self.create_publisher(Attitude, "/uav/attitude", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/uav/fcu_status", 10)
        self.flight_pub = self.create_publisher(FlightState, "/uav/flight_state", 10)
        self.rc_pub = self.create_publisher(RcChannels, "/uav/rc_channels", 10)
        self.drop_pub = self.create_publisher(Bool, "/uav/autonomy_drop", latched_qos)

        self.create_subscription(RcChannels, "/uav/rc_override",
                                 self._override_cb, 10)
        self.create_service(Trigger, "/uav/autonomy_drop_reset", self._reset_cb)
        self.create_service(Trigger, "/uav/fence_upload", self._fence_cb)

        # Each stream is republished ONLY while it is fresh. See the header.
        self._lock = threading.Lock()
        t_out = p["stream_timeout_s"]
        self._pose = StreamCache(t_out)    # (lat, lon, hdg, spd, amsl, rel, climb)
        self._att = StreamCache(t_out)     # (r, p, y, rspd, pspd, yspd) [rad, rad/s]
        self._status = StreamCache(t_out)  # (mode_str, armed, system_status)
        self._flight = StreamCache(t_out)  # int landed_state
        self._rc = StreamCache(t_out)      # list[int] 18

        # Fed by the RX loop, drained by the fence dialog on a service thread.
        # Bounded: a burst of another GCS's mission traffic must not grow without
        # limit while nobody is running an upload.
        self._mission_q = queue.Queue(maxsize=256)
        self._fence_lock = threading.Lock()
        self._ext_state_seen = False

        from pymavlink import mavutil
        self._mavutil = mavutil
        # source_system distinct from MAVProxy's default 255. While this node
        # only reads, the sysid is cosmetic; the moment it runs a mission dialog
        # it decides whether MISSION_REQUEST_INT replies are addressed to us or
        # to MAVProxy, and two listeners on one id is not a thing to debug at a
        # flight line.
        self.conn = mavutil.mavlink_connection(
            endpoint, source_system=int(p["mav_source_system"]))
        self.get_logger().info("waiting for heartbeat on %s ..." % endpoint)

        self._stop = threading.Event()          # deterministic teardown
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(1.0 / PUB_RATE_HZ, self._publish_tick)
        self._publish_drop_state()               # initial state (STARTUP=blocked)
        self.get_logger().info(
            "autonomy-drop: ch%d thr=%d invert=%s — overrides BLOCKED until "
            "safe RC seen" % (self.latch.channel, self.latch.threshold,
                              self.latch.invert))

    # ---------- MAVLink RX ----------

    def _rx_loop(self):
        # interruptible heartbeat wait: 1 s slices so the stop Event works even
        # before MAVProxy is up, with a periodic loud reminder (never silent)
        waited = 0
        while not self._stop.is_set():
            if self.conn.wait_heartbeat(timeout=1.0):
                break
            waited += 1
            if waited % 10 == 0:
                self.get_logger().warn(
                    "still no heartbeat after %ds — is MAVProxy running?"
                    % waited)
        else:
            return
        self.get_logger().info("heartbeat OK")
        while not self._stop.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = time.monotonic()
            mtype = msg.get_type()

            if mtype in _MISSION_TYPES:
                # Handed to the fence dialog, never handled here: it is a
                # blocking request/response exchange and the RX loop is what
                # keeps every other stream alive.
                try:
                    self._mission_q.put_nowait(msg)
                except queue.Full:
                    pass          # no dialog draining it; nothing to preserve
                continue

            # captured at RECEIPT, not at publish, so a republished frame
            # carries the age it actually has
            stamp = self.get_clock().now().to_msg()
            att_now = None
            with self._lock:
                if mtype == "GLOBAL_POSITION_INT":
                    hdg = msg.hdg / 100.0 if msg.hdg != 65535 else float("nan")
                    self._pose.set((msg.lat / 1e7, msg.lon / 1e7, hdg,
                                    geo.ground_speed_mps(msg.vx, msg.vy),
                                    msg.alt / 1000.0,          # mm AMSL -> m
                                    msg.relative_alt / 1000.0,  # mm -> m
                                    geo.climb_rate_mps(msg.vz)),
                                   t, stamp)
                elif mtype == "ATTITUDE":
                    # Republished in the autopilot's own axes/units (rad, NED
                    # body) — see Attitude.msg. Converting here would put a
                    # frame convention in the gateway, where nothing can check
                    # it; uav_common.geo owns that instead.
                    att_now = (msg.roll, msg.pitch, msg.yaw,
                               msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                    # cached ONLY so _publish_tick can log the stale edge; the
                    # publish itself happens below, not on the tick
                    self._att.set(att_now, t, stamp)
                elif mtype == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    mode = self._mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode &
                                 self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._status.set((mode, armed, msg.system_status), t, stamp)
                elif mtype == "EXTENDED_SYS_STATE":
                    self._ext_state_seen = True
                    self._flight.set(int(msg.landed_state), t, stamp)
                elif mtype == "RC_CHANNELS":
                    rc = [getattr(msg, "chan%d_raw" % i, 0) or 0
                          for i in range(1, 19)]
                    self._rc.set(rc, t, stamp)
                    if self.latch.rc_sample(rc, t):
                        self._handle_trip()
            # Outside the lock: a publish must never be held up by, or hold up,
            # the RC path.
            if att_now is not None:
                m = Attitude()
                m.header.stamp = stamp
                (m.roll, m.pitch, m.yaw,
                 m.rollspeed, m.pitchspeed, m.yawspeed) = att_now
                self.att_pub.publish(m)

    # ---------- publishing ----------

    def _publish_tick(self):
        t = time.monotonic()
        with self._lock:
            pose = self._pose.get(t)
            status = self._status.get(t)
            flight = self._flight.get(t)
            rc = self._rc.get(t)
            # one loud line per stream the moment it goes stale — a silent
            # gateway must be diagnosable from the log, and consumers that judge
            # health by arrival need the silence to be real
            stale = [name for name, c in (("pose", self._pose),
                                          ("attitude", self._att),
                                          ("fcu_status", self._status),
                                          ("flight_state", self._flight),
                                          ("rc_channels", self._rc))
                     if c.went_stale(t)]
            ext_seen = self._ext_state_seen
            if self.latch.tick(t):
                self._handle_trip()
        for name in stale:
            self.get_logger().error(
                "MAVLink stream %r stale (> %.1fs) — NOT republishing; is "
                "MAVProxy still up?" % (name, self._pose.timeout_s))
        if not ext_seen:
            # Loud and repeated, never substituted. Without EXTENDED_SYS_STATE
            # ocs_client falls back to an altitude test that is a worse answer
            # than the autopilot's own, and it must be obvious which one is live.
            self.get_logger().warn(
                "no EXTENDED_SYS_STATE yet — /uav/flight_state is silent and "
                "flight_phase will fall back to armed+altitude. Set "
                "SR0_EXT_STAT > 0 on the autopilot.",
                throttle_duration_sec=30.0)
        if pose is not None:
            m = GlobalPos()
            m.header.stamp = self._pose.stamp
            (m.latitude, m.longitude, m.heading, m.ground_speed,
             m.altitude_amsl, m.altitude_rel, m.climb) = pose
            self.pose_pub.publish(m)
        if status is not None:
            m = FcuStatus()
            m.header.stamp = self._status.stamp
            m.mode, m.armed, m.system_status = status
            self.status_pub.publish(m)
        if flight is not None:
            m = FlightState()
            m.header.stamp = self._flight.stamp
            m.landed_state = flight
            # UNDEFINED is one integer from ON_GROUND and opposite in meaning;
            # `valid` is what stops a consumer reading the 0 as a state.
            m.valid = flight != FlightState.LANDED_STATE_UNDEFINED
            self.flight_pub.publish(m)
        if rc is not None:
            m = RcChannels()
            m.header.stamp = self._rc.stamp
            m.channels = rc
            self.rc_pub.publish(m)

    def _publish_drop_state(self):
        self.drop_pub.publish(Bool(data=not self.latch.allowed))

    # ---------- override TX (the enforcement point) ----------

    def _override_cb(self, msg: RcChannels):
        if not self.latch.allowed:
            return                   # dropped/startup: overrides die here
        self._send_override(list(msg.channels[:8]))

    def _send_override(self, ch8):
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system, self.conn.target_component, *ch8)

    def _handle_trip(self):
        # called with latch already DROPPED; release every channel to the pilot
        self.get_logger().error(
            "AUTONOMY DROP: %s — releasing RC overrides" % self.latch.trip_reason)
        for _ in range(RELEASE_FRAMES):
            self._send_override([0] * 8)
        self._publish_drop_state()

    # ---------- reset service ----------

    def _reset_cb(self, request, response):
        ok, reason = self.latch.reset(time.monotonic())
        response.success = ok
        response.message = reason
        (self.get_logger().warn if ok else self.get_logger().error)(
            "autonomy-drop reset: %s" % reason)
        self._publish_drop_state()
        return response

    # ---------- geofence upload ----------

    def _fence_cb(self, request, response):
        """Upload the configured geofence, then read it back and verify.

        Runs on a service callback thread, NOT the RX thread — the dialog blocks
        on each autopilot reply and the RX thread is what keeps every other
        stream alive.
        """
        # Non-blocking: a second caller gets told so rather than queueing up a
        # concurrent mission dialog on one link, which interleaves two sequences
        # of MISSION_REQUEST and produces a fence made of both.
        if not self._fence_lock.acquire(blocking=False):
            response.success = False
            response.message = "a fence upload is already in progress"
            self.get_logger().warn(response.message)
            return response
        try:
            with self._lock:
                status = self._status.get(time.monotonic())
            # Unknown armed state refuses too: a missing FcuStatus most often
            # means the gateway is down, which is not evidence the aircraft is
            # sitting on the ground.
            if status is None:
                response.success = False
                response.message = (
                    "refused: armed state unknown (no fresh FcuStatus). "
                    "Rewriting a fence without knowing whether the aircraft is "
                    "flying is not a thing to do from a web button.")
                self.get_logger().error(response.message)
                return response
            if status[1]:
                response.success = False
                response.message = (
                    "refused: vehicle is ARMED. Disarm before changing the "
                    "geofence.")
                self.get_logger().error(response.message)
                return response

            transport = MavFenceTransport(self.conn, self._mission_q,
                                          self._mavutil.mavlink)
            proto = FenceProtocol(transport,
                                  timeout_s=float(self.p["fence_timeout_s"]))
            self.get_logger().info(
                "fence upload: sending %d vertices ..." % len(self._fence_items))
            try:
                proto.upload_and_verify(self._fence_items)
            except FenceError as e:
                response.success = False
                response.message = str(e)
                self.get_logger().error("fence upload FAILED: %s" % e)
                return response
            response.success = True
            response.message = (
                "uploaded and read back %d vertices" % len(self._fence_items))
            self.get_logger().warn(
                "fence upload OK: %d vertices verified. FENCE_ENABLE is NOT set "
                "by this node — enable it deliberately at a ground station."
                % len(self._fence_items))
            return response
        finally:
            self._fence_lock.release()

    # ---------- teardown ----------

    def destroy_node(self):
        self._stop.set()
        self._rx_thread.join(timeout=2.0)
        try:
            self.conn.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(TelemetryBridge, args=args)


if __name__ == "__main__":
    main()
