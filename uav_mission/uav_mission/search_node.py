"""search_node — flies the Task 1 buoy search.

    ros2 run uav_mission search_node

Wiring only; every decision is search_core's, and its header is where the rules
live (only the pilot's switch into GUIDED starts it, any other mode pauses it,
the page can only stop it, RTL is asked for once every buoy is found).

Subscribes: /uav/pose, /uav/fcu_status, /uav/flight_state, /uav/fence (latched),
/uav/fcu_params (latched), /uav/perception/buoy_map.
Publishes:  /uav/guided_target (uav_msgs/GuidedTarget), /uav/search/status.
Calls:      /uav/rtl_from_guided (telemetry_bridge), once everything is found.

It holds NO MAVLink connection. telemetry_bridge is the only thing that talks to
the autopilot, and it re-checks every target -- GUIDED, armed, fresh, inside the
fence -- before forwarding it, so a bug in here can ask for a bad point but
cannot fly one.

Two parameters are set from the ground station page, and are the only dynamic
ones: `enabled` and `buoys_to_find`. Everything that shapes how the aircraft
flies is read-only and comes from uav_params.yaml.

If this node dies mid-search the autopilot finishes the leg it was given and
holds there, still in GUIDED, until the pilot flips a switch. Every leg ends
inside the fence at the search altitude, so that is a safe place to be left.
"""
import math
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from uav_msgs.msg import (BuoyMap, FcuParams, FcuStatus, Fence, FlightState,
                          GlobalPos, GuidedTarget, SearchStatus)

from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config, make_set_callback
from uav_common.stream_cache import StreamCache

from uav_mission import search_core

PARAM_SPEC = {
    # ---- set from the ground station page
    "enabled": dict(read_only=False,
                    description="the page's on/off. Turning it on never starts "
                                "a flight: that takes the pilot's switch into "
                                "GUIDED"),
    "buoys_to_find": dict(read_only=False, lo=1, hi=50,
                          description="confirmed buoys inside the fence before "
                                      "the search asks for RTL"),
    # ---- how it flies (read-only: change the YAML and restart)
    "search_alt_m": dict(read_only=True, lo=3.0, hi=60.0,
                         description="altitude above home for the whole search"),
    "speed_mps": dict(read_only=True, lo=0.5, hi=8.0,
                      description="horizontal speed on every leg"),
    "line_spacing_m": dict(read_only=True, lo=2.0, hi=40.0,
                           description="widest gap between sweep lines"),
    "fence_inset_m": dict(read_only=True, lo=0.5, hi=20.0,
                          description="how far inside the fence every point it "
                                      "flies to stays"),
    "arrive_radius_m": dict(read_only=True, lo=0.3, hi=5.0,
                            description="a waypoint this close is reached"),
    "hover_radius_m": dict(read_only=True, lo=0.3, hi=5.0,
                           description="this close to a buoy counts as over it; "
                                       "the give-up clock starts here"),
    "give_up_s": dict(read_only=True, lo=2.0, hi=120.0,
                      description="seconds over a buoy before giving up on it "
                                  "for the rest of the pass"),
    "divert_timeout_s": dict(read_only=True, lo=5.0, hi=300.0,
                             description="seconds to reach a buoy before "
                                         "skipping it"),
    "cluster_radius_m": dict(read_only=True, lo=0.0, hi=20.0,
                             description="unknown buoys this close are hovered "
                                         "over together (a gate pair)"),
    "recenter_m": dict(read_only=True, lo=0.1, hi=5.0,
                       description="move the hover point once the buoy's mapped "
                                   "position has shifted this far"),
    "climb_tolerance_m": dict(read_only=True, lo=0.2, hi=5.0,
                              description="within this of search_alt_m = climbed"),
    "fence_max_age_s": dict(read_only=True, lo=5.0, hi=600.0,
                            description="refuse to start when the autopilot's "
                                        "fence was last read longer ago than "
                                        "this; the bridge re-reads every 30 s"),
    "rtl_when_done": dict(read_only=True,
                          description="ask for RTL once every buoy is found; "
                                      "false = hold position instead"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s"),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                             description="autopilot status older than this is "
                                         "unknown, and unknown is never GUIDED"),
    "buoy_map_timeout_s": dict(read_only=True, lo=1.0, hi=30.0,
                               description="no buoy map for this long = hold"),
    "tick_hz": dict(read_only=True, lo=1.0, hi=20.0,
                    description="decision rate"),
}

CONFIG_KEYS = ("search_alt_m", "speed_mps", "line_spacing_m", "fence_inset_m",
               "arrive_radius_m", "hover_radius_m", "give_up_s",
               "divert_timeout_s", "cluster_radius_m", "recenter_m",
               "climb_tolerance_m", "fence_max_age_s", "rtl_when_done")

STATUS_PERIOD_S = 0.5
_IN_AIR = (FlightState.LANDED_STATE_IN_AIR, FlightState.LANDED_STATE_TAKEOFF,
           FlightState.LANDED_STATE_LANDING)


def _num(x):
    """NaN (a param not read yet) -> None, which search_core reads as unknown."""
    return None if x is None or math.isnan(x) else float(x)


def _latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SearchNode(Node):

    def __init__(self):
        super().__init__("search_node")
        p = declare_from_config(self, uav_config.node_params("search_node"),
                                PARAM_SPEC)
        self.p = p
        ranges = {n: (s["lo"], s["hi"]) for n, s in PARAM_SPEC.items()
                  if not s.get("read_only") and "lo" in s}
        self.add_on_set_parameters_callback(
            make_set_callback(self, ranges, self._apply))

        self.core = search_core.BuoySearch(search_core.SearchConfig(
            **{k: p[k] for k in CONFIG_KEYS}))

        self._pose = StreamCache(float(p["pose_timeout_s"]))
        self._status = StreamCache(float(p["status_timeout_s"]))
        self._flight = StreamCache(float(p["status_timeout_s"]))
        self._map = StreamCache(float(p["buoy_map_timeout_s"]))
        self._fence = None                  # (polygon or None, problem, read_t)
        self._params = {}

        self.create_subscription(GlobalPos, "/uav/pose", self._cache(self._pose), 10)
        self.create_subscription(FcuStatus, "/uav/fcu_status",
                                 self._cache(self._status), 10)
        self.create_subscription(FlightState, "/uav/flight_state",
                                 self._cache(self._flight), 10)
        self.create_subscription(BuoyMap, "/uav/perception/buoy_map",
                                 self._cache(self._map), 10)
        self.create_subscription(Fence, "/uav/fence", self._on_fence, _latched())
        self.create_subscription(FcuParams, "/uav/fcu_params", self._on_params,
                                 _latched())

        self.target_pub = self.create_publisher(GuidedTarget, "/uav/guided_target", 10)
        self.status_pub = self.create_publisher(SearchStatus, "/uav/search/status", 10)
        self._rtl_cli = self.create_client(Trigger, "/uav/rtl_from_guided")
        self._last_status_t = 0.0
        self._last_phase = None

        self.create_timer(1.0 / float(p["tick_hz"]), self._tick)
        self.get_logger().info(
            "buoy search ready: %.0f m, %.1f m/s, lines %.0f m apart, %.0f m "
            "inside the fence, give up after %.0f s. %s. Starts only on a switch "
            "into GUIDED." % (p["search_alt_m"], p["speed_mps"], p["line_spacing_m"],
                              p["fence_inset_m"], p["give_up_s"],
                              "RTL when done" if p["rtl_when_done"] else
                              "holds when done"))

    # ------------------------------------------------------------ inputs

    def _apply(self, changes):
        self.p.update(changes)
        if "enabled" in changes:
            self.get_logger().warn("search switched %s from the ground station"
                                   % ("ON" if changes["enabled"] else "OFF"))
        if "buoys_to_find" in changes:
            self.get_logger().info("buoys to find: %d" % changes["buoys_to_find"])

    @staticmethod
    def _cache(cache):
        return lambda msg: cache.set(msg, time.monotonic())

    def _on_fence(self, msg):
        # Monotonic time of the READ, not of receipt: a latched fence delivered
        # to a node started later is as old as the read, whatever arrives now.
        age = (self.get_clock().now().nanoseconds
               - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)) / 1e9
        read_t = time.monotonic() - max(0.0, age)
        poly = list(zip(msg.latitude, msg.longitude)) if msg.valid else None
        self._fence = (poly, msg.problem, read_t)

    def _on_params(self, msg):
        self._params = {f: _num(getattr(msg, f)) for f in
                        ("fence_enable", "fence_type", "fence_alt_max",
                         "fence_margin")}

    def _inputs(self, now):
        pose = self._pose.get(now)
        st = self._status.get(now)
        fl = self._flight.get(now)
        bm = self._map.get(now)
        fence, problem, read_t = self._fence or (None, "", None)
        return search_core.Inputs(
            now=now,
            enabled=bool(self.p["enabled"]),
            buoys_to_find=int(self.p["buoys_to_find"]),
            pose=None if pose is None else (pose.latitude, pose.longitude,
                                            pose.altitude_rel, pose.ground_speed),
            mode=None if st is None else st.mode,
            armed=None if st is None else bool(st.armed),
            in_air=(None if fl is None or not fl.valid
                    else fl.landed_state in _IN_AIR),
            fence=fence, fence_problem=problem, fence_read_t=read_t,
            buoys=None if bm is None else [
                search_core.Buoy(int(b.id), b.latitude, b.longitude,
                                 bool(b.locked), b.label,
                                 _num(b.last_seen_age_s)) for b in bm.buoys],
            **{k: self._params.get(k) for k in ("fence_enable", "fence_type",
                                                "fence_alt_max", "fence_margin")})

    # ------------------------------------------------------------ output

    def _tick(self):
        now = time.monotonic()
        d = self.core.step(self._inputs(now))
        for e in d.events:
            self.get_logger().info(e)
        phase = d.status["phase"]
        if phase != self._last_phase:
            self.get_logger().info("search: %s" % d.status["text"])
            self._last_phase = phase
        if d.target is not None:
            m = GuidedTarget()
            m.header.stamp = self.get_clock().now().to_msg()
            m.latitude, m.longitude = d.target.lat, d.target.lon
            m.alt_rel_m = float(d.target.alt_m)
            m.yaw_deg = float(d.target.yaw_deg)
            m.speed_mps = float(d.target.speed_mps)
            self.target_pub.publish(m)
        if d.request_rtl:
            self._ask_rtl()
        if now - self._last_status_t >= STATUS_PERIOD_S:
            self._last_status_t = now
            self.status_pub.publish(self._status_msg(d.status))

    def _ask_rtl(self):
        if not self._rtl_cli.service_is_ready():
            self.get_logger().error(
                "every buoy is found but telemetry_bridge is not offering "
                "/uav/rtl_from_guided -- the aircraft is HOLDING in GUIDED. Take "
                "over with SB.", throttle_duration_sec=5.0)
            return
        fut = self._rtl_cli.call_async(Trigger.Request())

        def done(f):
            try:
                res = f.result()
                (self.get_logger().warn if res.success else self.get_logger().error)(
                    "RTL request: %s" % res.message)
            except Exception as e:
                self.get_logger().error("RTL request failed: %s" % e)
        fut.add_done_callback(done)

    def _status_msg(self, s):
        m = SearchStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.phase, m.text = s["phase"], s["text"]
        m.waiting_for = list(s["waiting_for"])
        m.enabled, m.buoys_to_find, m.found = s["enabled"], s["buoys_to_find"], s["found"]
        m.flying = s["flying"]
        m.pass_number, m.leg, m.legs = s["pass_number"], s["leg"], s["legs"]
        m.hover_buoys = [int(i) for i in s["hover_buoys"]]
        m.hover_s, m.give_up_s = float(s["hover_s"]), float(s["give_up_s"])
        m.skipped = [int(i) for i in s["skipped"]]
        m.target_latitude, m.target_longitude = s["target_lat"], s["target_lon"]
        m.plan_latitude = [a for a, _ in s["plan"]]
        m.plan_longitude = [b for _, b in s["plan"]]
        m.inset_latitude = [a for a, _ in s["inset"]]
        m.inset_longitude = [b for _, b in s["inset"]]
        return m


def main(args=None):
    run_node(SearchNode, args=args)


if __name__ == "__main__":
    main()
