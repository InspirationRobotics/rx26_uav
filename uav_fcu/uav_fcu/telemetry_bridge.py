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
     /uav/battery        uav_msgs/Battery      (SYS_STATUS + BATTERY_STATUS)
     /uav/gps            uav_msgs/GpsStatus    (GPS_RAW_INT)
     /uav/fcu_params     uav_msgs/FcuParams    (PARAM_VALUE; latched, on change)
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

4. RX (the fence the autopilot HOLDS): read back over the mission protocol at
   startup, every FENCE_POLL_S while disarmed (so a fence drawn in QGC shows up)
   and once more on each arm, and published latched on /uav/fence. Read-only.
   On its own worker thread, because a mission dialog blocks for a second and
   the executor is what keeps every topic flowing.

5. RX/TX (Crusader and the ground laptop, over the RFD900ux mesh on the
   telemetry port): MAVLink TUNNEL, shapes in uav_common/boat_link.py. OUT, to
   EVERY radio (target 0, so the laptop hears it before any boat says hello):
   each buoy's position ONCE, when its light is first decided -- re-sent only if
   the boat does not acknowledge it -- and every second the lights, one byte a
   buoy, with the ids the SEARCH has confirmed for the boat on a fresh look
   (from /uav/search/status). IN: the boat's position, one byte for what it is
   doing, and the positions it holds (the acknowledgement) -> /uav/boat.
   The confirmation is the one thing the boat cannot work out for itself -- a
   buoy the aircraft has not reached yet is not in the map at all, and a light
   mapped a minute ago is not a light seen now.
   The UAV still sends the boat FACTS AND NOTHING ELSE: never a course, a
   waypoint or an instruction. Crusader plans its own passage from the buoys.
   Every frame that crosses the radio -- the map out, the boat's packets and any
   other system's frames in -- is also published on /uav/radio/traffic, after
   the fact, for the ground station's Radio tab. /uav/radio/send_test puts one
   text frame (boat_link.PAYLOAD_TEST) on the air that neither end acts on.

6. TX (guided targets, for the buoy search): /uav/guided_target becomes a
   SET_POSITION_TARGET_GLOBAL_INT, and /uav/rtl_from_guided a mode change to
   RTL. Both are refused unless the autopilot freshly reports GUIDED and ARMED;
   a target must also be recent and inside the fence read in (4), under the
   ceiling. uav_common.guided_gate holds those rules. GUIDED is the PILOT'S
   switch, and the autopilot itself ignores position targets in any other mode,
   so the pilot's way back never depends on this node or its checks.

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
import math
import queue
import threading
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from uav_common.param_utils import stale_msgs_message
try:
    from uav_msgs.msg import (Attitude, Battery, BoatState, BuoyMap, FcuParams,
                              FcuStatus, Fence, FlightState, GlobalPos, GpsStatus,
                              GuidedTarget, RadioFrame, RcChannels, SearchStatus)
except ImportError as e:          # see stale_msgs_message
    raise ImportError(stale_msgs_message(e)) from e

from uav_common import config as uav_config
from uav_common import fcu_decode
from uav_common import geo
from uav_common import boat_link
from uav_common import guided_gate
from uav_common.drop_latch import DropLatch
from uav_common.fence_core import (MISSION_TYPE_FENCE, FenceError, FenceProtocol,
                                   MavFenceTransport, items_from_polygon,
                                   polygon_from_flat, polygon_from_items)
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
    "boat_sysid": dict(read_only=True, lo=0, hi=255,
                       description="Crusader's MAVLink system id: whose reports "
                                   "are read. 0 = no boat link at all, send "
                                   "nothing"),
    "boat_report_hz": dict(read_only=True, lo=0.1, hi=10.0,
                           description="how often the lights go out over the "
                                       "radio"),
}

RELEASE_FRAMES = 5          # all-zero override frames sent on trip
PUB_RATE_HZ = 20.0

#: EXTENDED_SYS_STATE is in NO stream-rate group. No MAVn_* value (SRx_* before
#: 4.7) will ever produce it -- it must be asked for per-message. 2 Hz matches
#: what ocs_client reports at; flight phase does not change faster than that.
EXT_STATE_INTERVAL_US = 500000
#: How often to re-ask while it is still absent. The request lives on the MAVLink
#: CHANNEL, so an autopilot reboot silently discards it -- one request at startup
#: would leave /uav/flight_state dead for the rest of the sortie.
EXT_STATE_REREQUEST_S = 30.0

#: Autopilot parameters read back for the ground station (FcuParams.msg). Asked
#: for with PARAM_REQUEST_READ -- read-only, it changes nothing about how the
#: aircraft flies. Any missing name is re-asked on this period; all of them are
#: refreshed on the slower one, because a change made in QGC is broadcast but a
#: change made while this node was down is not.
PARAM_REREQUEST_S = 30.0
PARAM_REFRESH_S = 300.0

#: MISSION_* messages the fence dialog consumes. Routed off the RX thread into a
#: queue rather than handled there, so a blocking request/response exchange
#: never stalls telemetry republishing.
_MISSION_TYPES = ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
                  "MISSION_COUNT", "MISSION_ITEM", "MISSION_ITEM_INT")

#: How often the fence is read back while disarmed, so a fence re-drawn in QGC
#: reaches the search before takeoff, and while armed, so the search always has a
#: RECENT read rather than one taken at arming that may have failed. The read is
#: a few MAVLink frames and changes nothing. After a failed read, try again sooner.
FENCE_POLL_S = 15.0
FENCE_ARMED_POLL_S = 30.0
FENCE_RETRY_S = 5.0

#: search_node publishes twice a second; four missed in a row and we stop
#: passing on a confirmation it may no longer be making.
SEARCH_MAX_AGE_S = 2.0

#: ArduCopter's custom mode number for RTL.
COPTER_MODE_RTL = 6
#: SET_POSITION_TARGET_GLOBAL_INT type_mask: position and yaw used; velocity,
#: acceleration and yaw rate ignored. YAW_IGNORE is added when the target has no
#: heading (a hold).
_TYPEMASK_POS_YAW = 8 | 16 | 32 | 64 | 128 | 256 | 2048
_TYPEMASK_YAW_IGNORE = 1024
_FRAME_GLOBAL_RELATIVE_ALT_INT = 6


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
        self.batt_pub = self.create_publisher(Battery, "/uav/battery", 10)
        self.gps_pub = self.create_publisher(GpsStatus, "/uav/gps", 10)
        self.params_pub = self.create_publisher(FcuParams, "/uav/fcu_params",
                                                latched_qos)
        self.drop_pub = self.create_publisher(Bool, "/uav/autonomy_drop", latched_qos)
        self.fence_pub = self.create_publisher(Fence, "/uav/fence", latched_qos)
        self.boat_pub = self.create_publisher(BoatState, "/uav/boat", 10)
        # Every frame that crosses the radio, for the ground station's Radio tab.
        # A record published after the fact -- see RadioFrame.msg.
        self.radio_pub = self.create_publisher(RadioFrame, "/uav/radio/traffic",
                                               50)
        self._radio_test_n = 0
        # The buoy map goes out over the radio from HERE, not from the mapper:
        # this node owns the one link, and what leaves the aircraft should leave
        # through the thing that checks what leaves the aircraft.
        self._buoys = None
        # What goes on the air and when: positions once (re-sent only if the
        # boat has not acknowledged them), lights every period.
        self._radio = boat_link.Sender()
        self._radio.LIGHTS_PERIOD_S = 1.0 / float(self.p["boat_report_hz"])
        self._radio_sent = {boat_link.PAYLOAD_POSITIONS: 0, boat_link.PAYLOAD_LIGHTS: 0}
        # What the SEARCH has confirmed for the boat. The boat cannot work this
        # out from the map it receives, so the aircraft has to say it.
        self._search = StreamCache(SEARCH_MAX_AGE_S)
        self.create_subscription(BuoyMap, "/uav/perception/buoy_map",
                                 self._buoy_map_cb, 10)
        self.create_subscription(SearchStatus, "/uav/search/status",
                                 self._search_cb, 10)

        self.create_subscription(RcChannels, "/uav/rc_override",
                                 self._override_cb, 10)
        self.create_subscription(GuidedTarget, "/uav/guided_target",
                                 self._guided_target_cb, 10)
        self.create_service(Trigger, "/uav/autonomy_drop_reset", self._reset_cb)
        self.create_service(Trigger, "/uav/fence_upload", self._fence_cb)
        self.create_service(Trigger, "/uav/rtl_from_guided", self._rtl_cb)
        self.create_service(Trigger, "/uav/radio/send_test", self._radio_test_cb)

        # Each stream is republished ONLY while it is fresh. See the header.
        self._lock = threading.Lock()
        t_out = p["stream_timeout_s"]
        self._pose = StreamCache(t_out)    # (lat, lon, hdg, spd, amsl, rel, climb)
        self._att = StreamCache(t_out)     # (r, p, y, rspd, pspd, yspd) [rad, rad/s]
        self._status = StreamCache(t_out)  # (mode_str, armed, system_status)
        self._flight = StreamCache(t_out)  # int landed_state
        self._rc = StreamCache(t_out)      # list[int] 18
        self._batt = StreamCache(t_out)    # (volts, amps, remaining_pct)
        self._consumed = StreamCache(t_out)  # mAh
        self._gps = StreamCache(t_out)     # (fix, sats, hdop, h_acc_m)
        # Read-back autopilot params: {FcuParams field: value}. Values do not go
        # stale -- a parameter keeps its value until changed -- so this is a
        # plain dict, published whenever an entry changes.
        self._params = {}
        self._params_dirty = False
        self._params_req_t = -PARAM_REREQUEST_S
        self._params_refresh_t = 0.0

        # Fed by the RX loop, drained by the fence dialog on a service thread.
        # Bounded: a burst of another GCS's mission traffic must not grow without
        # limit while nobody is running an upload.
        self._mission_q = queue.Queue(maxsize=256)
        self._fence_lock = threading.Lock()
        # The fence as last READ from the autopilot: [(lat, lon)] when it held
        # one usable polygon, else None. Guided targets are checked against it.
        self._held_fence = None
        # Bumped on every change INTO GUIDED: the autopilot resets its target on
        # entry, so the next target must be sent even if it has not changed.
        self._guided_epoch = 0
        self._last_mode = None
        self._resender = guided_gate.Resender()
        self._last_refusal = ("", 0.0)
        self._ext_state_seen = False
        self._ext_state_req_t = time.monotonic()

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
        self._fence_thread = threading.Thread(target=self._fence_read_loop,
                                              daemon=True)
        self._fence_thread.start()

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
        self._request_ext_sys_state()
        while not self._stop.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = time.monotonic()
            mtype = msg.get_type()

            if mtype in _MISSION_TYPES:
                # Only the FENCE dialog addressed to US. MAVProxy rebroadcasts
                # every GCS's mission traffic to this port too; QGC fetching its
                # own copy of the fence must not be read as our answer.
                if (getattr(msg, "target_system", self.conn.source_system)
                        != self.conn.source_system
                        or getattr(msg, "mission_type", MISSION_TYPE_FENCE)
                        != MISSION_TYPE_FENCE):
                    continue
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
            radio = self._radio_rx(msg, mtype, stamp)
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
                    if mode == "GUIDED" and self._last_mode != "GUIDED":
                        self._guided_epoch += 1
                    self._last_mode = mode
                elif mtype == "EXTENDED_SYS_STATE":
                    self._ext_state_seen = True
                    self._flight.set(int(msg.landed_state), t, stamp)
                elif mtype == "RC_CHANNELS":
                    rc = [getattr(msg, "chan%d_raw" % i, 0) or 0
                          for i in range(1, 19)]
                    self._rc.set(rc, t, stamp)
                    if self.latch.rc_sample(rc, t):
                        self._handle_trip()
                elif mtype == "SYS_STATUS":
                    self._batt.set(fcu_decode.battery_from_sys_status(
                        msg.voltage_battery, msg.current_battery,
                        msg.battery_remaining), t, stamp)
                elif mtype == "BATTERY_STATUS" and msg.id == 0:
                    self._consumed.set(
                        fcu_decode.consumed_mah(msg.current_consumed), t, stamp)
                elif mtype == "GPS_RAW_INT":
                    self._gps.set(fcu_decode.gps_from_raw(
                        msg.fix_type, msg.eph, msg.satellites_visible,
                        getattr(msg, "h_acc", 0)), t, stamp)
                elif mtype == "TUNNEL":
                    # Crusader over the radio: where it is, what it is doing,
                    # and which buoy positions it holds.
                    if (msg.payload_type == boat_link.PAYLOAD_BOAT
                            and msg.get_srcSystem() == int(self.p["boat_sysid"])):
                        boat = boat_link.unpack_boat(boat_link.body(msg))
                        self._radio.boat_heard(boat["acked"], t)
                        m = BoatState()
                        m.header.stamp = stamp
                        m.latitude, m.longitude = boat["lat"], boat["lon"]
                        m.activity = boat["activity"]
                        m.target_buoy = self._radio.id_of(boat["target_slot"]) or 0
                        self.boat_pub.publish(m)
                elif mtype == "PARAM_VALUE":
                    field = fcu_decode.FCU_PARAMS.get(
                        fcu_decode.param_name(msg.param_id))
                    if field and self._params.get(field) != msg.param_value:
                        self._params[field] = float(msg.param_value)
                        self._params_dirty = True
            # Outside the lock: a publish must never be held up by, or hold up,
            # the RC path.
            if radio is not None:
                self.radio_pub.publish(radio)
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
            batt = self._batt.get(t)
            consumed = self._consumed.get(t)
            gps = self._gps.get(t)
            # one loud line per stream the moment it goes stale — a silent
            # gateway must be diagnosable from the log, and consumers that judge
            # health by arrival need the silence to be real
            stale = [name for name, c in (("pose", self._pose),
                                          ("attitude", self._att),
                                          ("fcu_status", self._status),
                                          ("flight_state", self._flight),
                                          ("rc_channels", self._rc),
                                          ("battery", self._batt),
                                          ("gps", self._gps))
                     if c.went_stale(t)]
            ext_seen = self._ext_state_seen
            params = dict(self._params) if self._params_dirty else None
            self._params_dirty = False
            missing = [n for n, f in fcu_decode.FCU_PARAMS.items()
                       if f not in self._params]
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
                "flight_phase will fall back to armed+altitude. This is NOT a "
                "stream-rate parameter — no MAVn_*/SRx_* value produces it. It is "
                "re-requested every %.0fs; if this persists the autopilot is not "
                "answering SET_MESSAGE_INTERVAL." % EXT_STATE_REREQUEST_S,
                throttle_duration_sec=30.0)
        # Self-healing, and deliberately NOT gated on _ext_state_seen: the
        # per-message request lives on the MAVLink CHANNEL, so an autopilot
        # reboot discards it while this node stays up, connected, and with
        # _ext_state_seen still True from before the reboot. Keying off the
        # CACHE going cold covers both "never arrived" and "stopped arriving";
        # the seen-flag alone would leave /uav/flight_state dead for the rest
        # of the sortie after any in-air FC reset.
        if flight is None and t - self._ext_state_req_t >= EXT_STATE_REREQUEST_S:
            self._request_ext_sys_state()
        # Only once the autopilot is talking (a fresh FcuStatus), so requests
        # are not fired into a link with nobody on the other end.
        if status is not None:
            if t - self._params_refresh_t >= PARAM_REFRESH_S:
                self._params_refresh_t = t
                self._request_params(list(fcu_decode.FCU_PARAMS))
            elif missing and t - self._params_req_t >= PARAM_REREQUEST_S:
                self._request_params(missing)
        if params is not None:
            m = FcuParams()
            m.header.stamp = self.get_clock().now().to_msg()
            for f in fcu_decode.FCU_PARAMS.values():
                setattr(m, f, params.get(f, float("nan")))
            self.params_pub.publish(m)
        if batt is not None:
            m = Battery()
            m.header.stamp = self._batt.stamp
            m.voltage, m.current, m.remaining_pct = batt
            m.consumed_mah = consumed if consumed is not None else float("nan")
            self.batt_pub.publish(m)
        if gps is not None:
            m = GpsStatus()
            m.header.stamp = self._gps.stamp
            m.fix_type, m.satellites, m.hdop, m.h_acc_m = gps
            self.gps_pub.publish(m)
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
        self._radio_tick(t)

    def _search_cb(self, msg: SearchStatus):
        self._search.set(msg, time.monotonic())

    def _confirmed(self):
        """The ids search_node is confirming for the boat RIGHT NOW, or [].
        A silent search_node confirms nothing: the boat is told nothing and
        stops, rather than driving on a confirmation nobody is still making."""
        st = self._search.get(time.monotonic())
        return [] if st is None else [int(i) for i in st.confirmed][:2]

    def _buoy_map_cb(self, msg: BuoyMap):
        self._buoys = [(b.id, b.latitude, b.longitude, b.label) for b in msg.buoys]

    def _radio_tick(self, t):
        """Whatever is due on the mesh: new positions, re-sends the boat has not
        acknowledged, and the lights once a period. Addressed to everyone."""
        if not int(self.p["boat_sysid"]) or not self._buoys:
            return
        self._radio.feed(self._buoys, self._confirmed(), t)
        for ptype, payload in self._radio.due(t):
            self._send_tunnel(0, ptype, payload)
            self._radio_sent[ptype] += 1
        if self._radio.overflow:
            self.get_logger().warn(
                "%d buoys have no radio slot left (%d used): the mesh is not "
                "hearing B%s" % (len(self._radio.overflow), boat_link.MAX_SLOT,
                                 ", B".join(map(str, self._radio.overflow[:6]))),
                throttle_duration_sec=60.0)
        # One line a minute: at the field "is anyone being told anything?" needs
        # an answer that does not require a second laptop.
        conf = self._confirmed()
        self.get_logger().info(
            "radio: %d buoy positions sent, %d acknowledged by the boat%s; lights "
            "packets %d; confirmed %s"
            % (len(self._radio.slots), len(self._radio.acked),
               "" if self._radio.boat_listening(t) else " (no boat heard)",
               self._radio_sent[boat_link.PAYLOAD_LIGHTS],
               "/".join("B%d" % i for i in conf) or "nothing"),
            throttle_duration_sec=60.0)

    # ---------- the radio, as a record (for the Radio tab) ----------

    def _send_tunnel(self, sysid, payload_type, payload):
        """Send one TUNNEL to `sysid`, and record it on /uav/radio/traffic.

        tunnel_encode + send is exactly what tunnel_send does in one call; it is
        split here only so the packed frame is still in hand to measure.
        """
        m = self.conn.mav.tunnel_encode(sysid, 0, payload_type, len(payload),
                                        boat_link.pad(payload))
        self.conn.mav.send(m)
        name, summary = boat_link.describe(payload_type, payload)
        self.radio_pub.publish(self._radio_frame(
            RadioFrame.DIR_TX, self.conn.source_system,
            self.conn.source_component, sysid, name, payload_type, summary,
            len(m.get_msgbuf()), self.get_clock().now().to_msg()))

    def _radio_rx(self, msg, mtype, stamp):
        """A RadioFrame for a frame another system put on the link, or None.

        Another system is anything that is neither the autopilot this node talks
        to nor this node. MAVProxy only rebroadcasts what the autopilot hands it,
        so a frame from any other system id reached the autopilot through one of
        its telemetry ports -- the RFD900. Ekko's own telemetry is not radio
        traffic and is never recorded.

        Built before self._lock is taken and published after it is released,
        like attitude: a record of the link must never hold up the RC path.
        """
        src = msg.get_srcSystem()
        if src in (self.conn.target_system, self.conn.source_system):
            return None
        name, ptype, summary = mtype, 0, ""
        if mtype == "TUNNEL":
            ptype = msg.payload_type
            name, summary = boat_link.describe(ptype, boat_link.body(msg))
        elif mtype == "HEARTBEAT":
            summary = self._heartbeat_type(msg.type)
        return self._radio_frame(
            RadioFrame.DIR_RX, src, msg.get_srcComponent(),
            getattr(msg, "target_system", 0), name, ptype, summary,
            len(msg.get_msgbuf()), stamp)

    def _heartbeat_type(self, mav_type):
        """SURFACE_BOAT, GCS, ... for a HEARTBEAT's type field, or its number."""
        try:
            return self._mavutil.mavlink.enums["MAV_TYPE"][mav_type].name[
                len("MAV_TYPE_"):]
        except (KeyError, AttributeError):
            return "type %d" % mav_type

    @staticmethod
    def _radio_frame(direction, src, comp, dst, name, payload_type, summary,
                     nbytes, stamp):
        m = RadioFrame()
        m.header.stamp = stamp
        m.direction = direction
        m.src_system = int(src) & 0xFF
        m.src_component = int(comp) & 0xFF
        m.dst_system = int(dst) & 0xFF
        m.msg_name = str(name)
        m.payload_type = int(payload_type) & 0xFFFF
        m.summary = str(summary)
        m.frame_bytes = min(int(nbytes), 0xFFFF)
        return m

    def _radio_test_cb(self, request, response):
        """Put one PAYLOAD_TEST frame on the radio, addressed to the boat.

        For the Radio tab's button. A text line neither end acts on: the point is
        a known frame an operator can watch arrive, in QGC's MAVLink Inspector or
        with check_mesh.py on a laptop radio.
        """
        sysid = int(self.p["boat_sysid"])
        if not sysid:
            response.success = False
            response.message = ("refused: boat_sysid is 0, so there is no boat "
                                "to address")
            self.get_logger().warn(response.message)
            return response
        self._radio_test_n += 1
        text = "test %d from ekko" % self._radio_test_n
        self._send_tunnel(sysid, boat_link.PAYLOAD_TEST, boat_link.pack_test(text))
        response.success = True
        response.message = "sent %r to system %d" % (text, sysid)
        self.get_logger().info(response.message)
        return response

    def _publish_drop_state(self):
        self.drop_pub.publish(Bool(data=not self.latch.allowed))

    def _request_ext_sys_state(self):
        """Ask the autopilot for EXTENDED_SYS_STATE explicitly.

        EXTENDED_SYS_STATE belongs to no stream group, so it cannot be turned
        on with a parameter -- scripts/start_mavproxy.sh's advice to raise
        SR0_EXT_STAT for it is wrong on every firmware version, and SR0_* does
        not even exist on 4.7 (it is MAV1_*). Verified on Ekko 2026-09-02:
        MAV1_EXT_STAT=3 streams SYS_STATUS, GPS_RAW_INT and BATTERY_STATUS but
        never EXTENDED_SYS_STATE; one SET_MESSAGE_INTERVAL produces it at once.

        Read-only in effect: it changes what the autopilot SENDS to this
        channel and nothing about how it flies.
        """
        self._ext_state_req_t = time.monotonic()
        try:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(self._mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE),
                float(EXT_STATE_INTERVAL_US), 0, 0, 0, 0, 0)
            self.get_logger().info(
                "requested EXTENDED_SYS_STATE at %.1f Hz"
                % (1e6 / EXT_STATE_INTERVAL_US))
        except Exception as e:
            # Never fatal. Every other stream is unaffected, and the warning
            # above already makes a silent /uav/flight_state obvious.
            self.get_logger().warn(
                "could not request EXTENDED_SYS_STATE: %s" % e)

    def _request_params(self, names):
        """PARAM_REQUEST_READ for each name. Read-only; never fatal."""
        self._params_req_t = time.monotonic()
        try:
            for name in names:
                self.conn.mav.param_request_read_send(
                    self.conn.target_system, self.conn.target_component,
                    name.encode("ascii"), -1)
        except Exception as e:
            self.get_logger().warn("could not request autopilot params: %s" % e)

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
            response.message = ("a fence dialog (an upload, or the periodic "
                                 "read-back) is in progress; try again in a few "
                                 "seconds")
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

    # ---------- fence read-back (job 4) ----------

    def _fence_read_loop(self):
        """Keep /uav/fence equal to what the autopilot holds. Own thread."""
        last_try = None
        last_ok = False
        was_armed = None
        while not self._stop.wait(1.0):
            t = time.monotonic()
            with self._lock:
                status = self._status.get(t)
            if status is None:
                continue                  # nobody answering yet; do not ask
            armed = status[1]
            if armed and was_armed is False:
                need = True             # read this flight's fence at once
            else:
                period = FENCE_ARMED_POLL_S if armed else FENCE_POLL_S
                need = last_try is None or t - last_try >= (
                    period if last_ok else FENCE_RETRY_S)
            was_armed = armed
            if not need:
                continue
            last_try = t
            last_ok = self._read_fence()

    def _read_fence(self):
        """One download; publish what it found. -> True if the dialog completed."""
        if not self._fence_lock.acquire(blocking=False):
            return False              # an upload is running; read again after
        try:
            proto = FenceProtocol(
                MavFenceTransport(self.conn, self._mission_q, self._mavutil.mavlink),
                timeout_s=float(self.p["fence_timeout_s"]))
            try:
                items = proto.download()
            except FenceError as e:
                self.get_logger().warn(
                    "could not read the fence back from the autopilot: %s" % e,
                    throttle_duration_sec=60.0)
                return False
        finally:
            self._fence_lock.release()
        polygon, problem = polygon_from_items(items)
        with self._lock:
            changed = (polygon or None) != self._held_fence
            self._held_fence = polygon or None
        if changed:
            if polygon:
                self.get_logger().info(
                    "fence on the autopilot: a %d-corner polygon" % len(polygon))
            else:
                self.get_logger().warn("fence on the autopilot is not usable: %s"
                                       % problem)
        m = Fence()
        m.header.stamp = self.get_clock().now().to_msg()
        m.valid = bool(polygon)
        m.problem = problem
        m.latitude = [a for a, _ in polygon]
        m.longitude = [b for _, b in polygon]
        m.item_count = len(items)
        self.fence_pub.publish(m)
        return True

    # ---------- guided targets and RTL (job 6) ----------

    def _fresh_status(self):
        """(mode, armed) from a heartbeat under guided_gate.STATUS_MAX_AGE_S
        old, or None. Deliberately not self._status.get(): see that constant."""
        with self._lock:
            st, age = self._status.value, self._status.age(time.monotonic())
        if st is None or age is None or age >= guided_gate.STATUS_MAX_AGE_S:
            return None
        return (st[0], st[1])

    def _guided_target_cb(self, msg: GuidedTarget):
        t = time.monotonic()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        with self._lock:
            fence, epoch = self._held_fence, self._guided_epoch
            params = dict(self._params)
        lat, lon = msg.latitude, msg.longitude
        alt, yaw, speed = float(msg.alt_rel_m), float(msg.yaw_deg), float(msg.speed_mps)
        why = guided_gate.refusal(lat, lon, alt, speed, age, self._fresh_status(),
                                  fence, params.get("fence_type"),
                                  params.get("fence_alt_max"),
                                  params.get("fence_margin"))
        if why:
            last_why, last_t = self._last_refusal
            if why != last_why or t - last_t >= 10.0:
                self.get_logger().warn("guided target NOT forwarded: %s" % why)
                self._last_refusal = (why, t)
            return
        self._last_refusal = ("", 0.0)
        mav = self._mavutil.mavlink
        if self._resender.speed_due(speed, epoch, t):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mav.MAV_CMD_DO_CHANGE_SPEED, 0,
                1.0, speed, -1.0, 0, 0, 0, 0)          # ground speed, m/s
            self._resender.speed_sent(speed, epoch, t)
        if self._resender.target_due(lat, lon, alt, yaw, epoch, t):
            no_yaw = math.isnan(yaw)
            self.conn.mav.set_position_target_global_int_send(
                0, self.conn.target_system, self.conn.target_component,
                _FRAME_GLOBAL_RELATIVE_ALT_INT,
                _TYPEMASK_POS_YAW | (_TYPEMASK_YAW_IGNORE if no_yaw else 0),
                int(round(lat * 1e7)), int(round(lon * 1e7)), alt,
                0, 0, 0, 0, 0, 0,
                0.0 if no_yaw else math.radians(yaw), 0)
            self._resender.target_sent(lat, lon, alt, yaw, epoch, t)

    def _rtl_cb(self, request, response):
        """RTL, asked for by the search once every buoy is found. GUIDED only:
        any other mode is the pilot's choice and is left alone."""
        status = self._fresh_status()
        if status is None or not status[1] or status[0] != "GUIDED":
            response.success = False
            response.message = (
                "refused: RTL is only requested from GUIDED while armed; the "
                "autopilot reports %s" % ("nothing fresh" if status is None else
                                          "%s, %s" % (status[0], "armed" if status[1]
                                                      else "disarmed")))
            self.get_logger().warn(response.message)
            return response
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            self._mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            float(self._mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(COPTER_MODE_RTL), 0, 0, 0, 0, 0)
        response.success = True
        response.message = "RTL requested (was GUIDED)"
        self.get_logger().warn(response.message)
        return response

    # ---------- teardown ----------

    def destroy_node(self):
        self._stop.set()
        self._rx_thread.join(timeout=2.0)
        self._fence_thread.join(timeout=float(self.p["fence_timeout_s"]) + 1.0)
        try:
            self.conn.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(TelemetryBridge, args=args)


if __name__ == "__main__":
    main()
