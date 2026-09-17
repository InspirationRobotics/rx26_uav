"""ground_station — the whole aircraft in one browser tab.

In use on Ekko in flight (Nodes, Map and Camera tabs at the park, 2026-09-13). It
starts, stops and restarts real nodes and can power the Jetson off. It is
started by systemd (tools/systemd/uav-groundstation.service) because nobody will
SSH into this Jetson between flights; `systemctl disable uav-groundstation` is
the way back.

    ros2 run uav_groundstation ground_station
    # then, on the laptop:  http://<JETSON_IP>:8090

Subscribes (read-only): /uav/pose, /uav/attitude, /uav/fcu_status,
/uav/flight_state, /uav/battery, /uav/gps, /uav/fcu_params, /uav/camera/status,
/uav/perception/buoy_map, /uav/fence, /uav/search/status. Publishes
nothing. Its only outward effects are the processes it spawns, the two power
verbs it can hand to the host helper — both gated, both re-checked server-side —
two services it can call: camera capture, and clearing the buoy map (which
saves the map before clearing it, so neither can lose data) — and the buoy
search's two page settings, on/off and how many buoys to find. Switching the
search ON cannot move the aircraft: only the pilot's switch into GUIDED starts
it (search_core). Switching it OFF makes it hold position.

THE MAP IS DRAWN AROUND THE FENCE THE AUTOPILOT HOLDS (/uav/fence), and falls
back to the `geofence` param only while none has been read. The map's origin
follows the same choice; far from both, it is the first position fix. An origin
half a world away is not cosmetic: local metres are scaled by the cosine of the
origin's latitude, so a Singapore origin stretched every east-west distance at a
San Diego park by 19%, the tape measure and the grid with it.

THE TWO RULES THIS NODE HOLDS, and holds again on every request no matter what
the page rendered:

  THE MAVLINK GATEWAY CANNOT BE STOPPED FROM HERE. telemetry_bridge is the only
  thing that speaks MAVLink; stopping it blinds the OCS heartbeat, the
  RC-override gate and the geofence uploader at once. WiFi is a convenience,
  never a control path for that. Starting it is allowed — that can only move the
  aircraft toward observable.

  POWER IS LOCKED WHILE ARMED, and then needs the hostname typed. An *unknown*
  armed state locks it too: a missing FcuStatus usually means the bridge is
  down, which is not evidence the aircraft is safe to reboot.

WHY IT SCANS /proc AS WELL AS THE ROS GRAPH. Both this node and ocs_client are
normally started by systemd, so the page has no Popen handle for either and
`self.procs.running()` knows nothing about them. The graph sees ROS nodes; /proc
sees processes whatever started them. A dashboard that reported the telemetry
bridge down because it did not personally start it would be worse than no
dashboard.
"""
import math
import socket
import time
from collections import deque

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from std_srvs.srv import SetBool, Trigger
from uav_common.param_utils import stale_msgs_message
try:
    from uav_msgs.msg import (Attitude, Battery, BoatState, BuoyMap, CameraStatus,
                              FcuParams, FcuStatus, Fence, FlightState, GlobalPos,
                              GpsStatus, RadioFrame, SearchStatus)
except ImportError as e:          # see stale_msgs_message
    raise ImportError(stale_msgs_message(e)) from e

from uav_common import camera_frame
from uav_common import config as uav_config
from uav_common import geo
from uav_common.fence_core import polygon_from_flat
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config, make_set_callback
from uav_common.stream_cache import StreamCache

from uav_groundstation import armed_clock, battery_core, preflight_core, radio_core
from uav_groundstation import node_registry as reg
from uav_groundstation import power_client, proc_scan, system_info
from uav_groundstation.gcs_page import render as render_page
from uav_groundstation.gcs_server import GcsServer
from uav_groundstation.log_buffer import LogBuffer
from uav_groundstation.process_manager import ProcessManager

PARAM_SPEC = {
    "port": dict(read_only=True, lo=1024, hi=65535, description="HTTP port"),
    "bind_host": dict(read_only=True,
                      description="0.0.0.0 so the laptop can reach it"),
    "tools_dir": dict(read_only=True,
                      description="where tools/*.py live, for script nodes"),
    "power_request_dir": dict(read_only=True,
                              description="where the shutdown/reboot request "
                                          "files are dropped for the host's "
                                          ".path units; see power_client.py"),
    "disk_path": dict(read_only=True, description="filesystem to report free"),
    "workspace_path": dict(read_only=True,
                           description="checked for being a bind mount, so the "
                                       "git-pull-then-rebuild loop is known to "
                                       "work rather than assumed"),
    "geofence": dict(read_only=True,
                     description="FLAT [lat,lon,...]; = telemetry_bridge."
                                 "geofence. Drawn on the map"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "flight_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "poll_period_s": dict(read_only=True, lo=0.05, hi=5.0,
                          description="how often the browser asks for /state"),
    "graph_period_s": dict(read_only=False, lo=0.2, hi=10.0,
                           description="how often the ROS node graph is scanned"),
    "trail_length": dict(read_only=False, lo=0, hi=20000),
    "trail_min_move_m": dict(read_only=False, lo=0.0, hi=50.0),
    "allow_power": dict(read_only=True,
                        description="master switch for the power tab"),
    "log_capacity": dict(read_only=True, lo=100, hi=20000,
                         description="/rosout lines kept in the ring"),
    "armed_time_file": dict(read_only=True,
                            description="where time armed this power-on is "
                                        "kept; see armed_clock"),
}

_LANDED_NAME = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF",
                4: "LANDING"}

# buoy_mapper publishes the whole map once a second. Three missed maps and the
# page greys the buoys out rather than drawing positions nobody is vouching for.
BUOY_MAP_TIMEOUT_S = 3.0

# How long the page waits on a service it calls. The HTTP thread blocks for this
# at most; see _call_service for why waiting there is safe.
SERVICE_TIMEOUT_S = 5.0

# search_node publishes its status twice a second.
SEARCH_STATUS_TIMEOUT_S = 2.0
# Crusader reports about once a second over the radio.
BOAT_TIMEOUT_S = 5.0
_BOAT_DOING = {0: "", 1: "holding", 2: "circling the entry buoy",
               3: "transiting", 4: "circling the exit buoy", 5: "done"}
BUOYS_TO_FIND_RANGE = (1, 50)
# The page's two selectors. Only Task 1 exists; the tier decides what happens
# once the field is mapped (go home, or stay with the boat).
TASKS = ("task1",)
TIERS = ("advanced", "disruptive")
# What each tier means for the buoy mapper. Advanced freezes a state once
# decided and weighs every sample; Disruptive must SEE a light change, so
# nothing is frozen and only the last few seconds decide. The operator picks a
# tier, never these: two settings that must agree with a third are two settings
# somebody will forget.
TIER_MAPPER = {"advanced": {"lock_state": True, "decide_window_s": 0.0},
               "disruptive": {"lock_state": False, "decide_window_s": 5.0}}
# Farther than this from the params fence, it is not this venue's fence and the
# map is centred on the aircraft instead.
FOREIGN_ORIGIN_M = 50_000.0

# Latched topics (telemetry_bridge's /uav/fcu_params and /uav/fence): a
# subscriber must be TRANSIENT_LOCAL too, or a ground station restarted after the
# publish never hears the value.
_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)

_FCU_PARAM_FIELDS = ("batt_low_volt", "batt_crt_volt", "batt_capacity_mah",
                 "fence_enable", "fence_alt_max", "fence_type", "fence_margin",
                 "fence_action")


def _finite(x):
    """A float for JSON: NaN becomes None, because JSON has no NaN and the page
    treats a blank as unknown."""
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else x


def _centroid(polygon):
    """Mean vertex of a polygon given open or closed; (0, 0) if empty."""
    pts = list(polygon)
    if len(pts) > 1 and tuple(pts[0]) == tuple(pts[-1]):
        pts = pts[:-1]
    if not pts:
        return (0.0, 0.0)
    return (sum(a for a, _ in pts) / len(pts), sum(b for _, b in pts) / len(pts))


class GroundStation(Node):
    """Subscriptions and a process table in, one JSON snapshot out."""

    def __init__(self):
        super().__init__("ground_station")
        p = declare_from_config(self, uav_config.node_params("ground_station"),
                                PARAM_SPEC)
        self.p = p

        ranges = {n: (s["lo"], s["hi"]) for n, s in PARAM_SPEC.items()
                  if not s.get("read_only") and "lo" in s}
        self.add_on_set_parameters_callback(
            make_set_callback(self, ranges, self._apply))

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._flight = StreamCache(p["flight_timeout_s"])

        self._trail = deque(maxlen=int(p["trail_length"]) or 1)

        # The fence is the map's origin. Anchoring on it rather than on the
        # first fix means the polygon does not jump the moment GPS arrives, and
        # two sessions draw the same picture. The params fence stands in until
        # the autopilot's is read (see the module header); origin_id tells the
        # page to drop a trail drawn about the old origin.
        # Flat [lat, lon, ...] in the params because ROS parameters cannot
        # nest; paired here by the one function that owns that conversion.
        self._fence = polygon_from_flat(p["geofence"])
        self._fence_src = "params"
        self._fence_problem = ""
        self._origin_id = 0
        self._set_origin(_centroid(self._fence))
        self._cpu = system_info.CpuMeter()
        # {port: (checked_at, is_open)} — see _port_open. Bounded by the number
        # of NodeSpecs that declare a port.
        self._port_probe = {}
        # {node name: monotonic time of its last restart from the page}. Read by
        # reg.restarting / reg.start_refusal; see RESTART_GRACE_S for why.
        self._restart_t = {}
        self._hostname = socket.gethostname()

        self.procs = ProcessManager(
            tools_dir=p["tools_dir"],
            logger=lambda m: self.get_logger().info(m))

        # /rosout rather than journalctl: we are inside a container and the host
        # journal is on the other side of that boundary, while /rosout crosses
        # the DDS domain and needs no privilege.
        self.logs = LogBuffer(int(p["log_capacity"]))
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)

        self._graph = set()
        self._proc = {}
        self.create_timer(p["graph_period_s"], self._scan_graph)

        self.create_subscription(GlobalPos, "/uav/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/uav/attitude", self._on_att, 10)
        self.create_subscription(FcuStatus, "/uav/fcu_status", self._on_status, 10)
        self.create_subscription(FlightState, "/uav/flight_state",
                                 self._on_flight, 10)

        # Camera status is read ONLY so the REC button can show what is actually
        # happening. recording_sd on this topic is MEASURED by camera_node, not
        # the last thing anybody asked for, and the A8 mini's record command is a
        # toggle whose ack can be lost -- so a button driven by intent rather
        # than by this would eventually show the opposite of the truth.
        self._cam_status = StreamCache(float(p["status_timeout_s"]))
        self.create_subscription(CameraStatus, "/uav/camera/status",
                                 self._on_cam_status, 10)
        # Created eagerly, called rarely. A client that is only built on first
        # use makes the first press of REC slower than every later one, which
        # reads as the button being broken.
        self._capture_cli = self.create_client(SetBool, "/uav/camera/capture")

        # The buoy map, drawn on the Map tab. Kept whole (the mapper never sends
        # deltas), and aged like every other stream.
        self._buoy_map = StreamCache(BUOY_MAP_TIMEOUT_S)
        self.create_subscription(BuoyMap, "/uav/perception/buoy_map",
                                 self._on_buoy_map, 10)
        self._clear_map_cli = self.create_client(
            Trigger, "/uav/perception/clear_buoy_map")

        # The fence the autopilot holds (latched by telemetry_bridge), and the
        # buoy search: its status for the Map tab, and its two page settings.
        self.create_subscription(
            Fence, "/uav/fence", self._on_fence,
            _LATCHED)
        # Crusader, heard over the radio by telemetry_bridge. Drawn on the map
        # so the operator can see what the drone is escorting; blank while the
        # boat is not being heard, never a remembered position.
        self._boat = StreamCache(BOAT_TIMEOUT_S)
        self.create_subscription(BoatState, "/uav/boat", self._on_boat, 10)
        # Every frame telemetry_bridge sends to, or hears from, another system
        # over the radio, for the Radio tab. The boat's id comes from the
        # bridge's own params, as _read_camera_cfg reads the camera's.
        self._radio = radio_core.RadioLog(boat_sysid=self._read_boat_sysid())
        self.create_subscription(RadioFrame, "/uav/radio/traffic",
                                 self._on_radio, 50)
        self._radio_test_cli = self.create_client(Trigger,
                                                  "/uav/radio/send_test")
        self._search = StreamCache(SEARCH_STATUS_TIMEOUT_S)
        self.create_subscription(SearchStatus, "/uav/search/status",
                                 self._on_search, 10)
        self._search_params_cli = self.create_client(
            SetParameters, "/search_node/set_parameters")
        self._mapper_params_cli = self.create_client(
            SetParameters, "/buoy_mapper/set_parameters")

        # Battery and GPS, for the header and the pre-flight strip. Battery
        # samples also feed the time-to-failsafe estimator; see battery_core
        # for why it works from the voltage trend rather than from mAh.
        self._battery = battery_core.BatteryEstimator()
        self.create_subscription(Battery, "/uav/battery", self._on_battery, 10)
        self._gps = StreamCache(float(p["status_timeout_s"]))
        self.create_subscription(GpsStatus, "/uav/gps", self._on_gps, 10)
        # Autopilot params read back by telemetry_bridge. Latched on that side,
        # so this subscription must be TRANSIENT_LOCAL too or a ground station
        # restarted after the read never hears the values.
        self._fcu_params = None
        self.create_subscription(
            FcuParams, "/uav/fcu_params", self._on_fcu_params,
            _LATCHED)
        # Camera geometry and working altitude, read once from the params the
        # camera and mapper nodes load themselves -- one source, as _geoid does
        # for the OCS constant. None if unreadable: the footprint and the fence
        # head-room check then say so instead of drawing from guessed numbers.
        self._cam_cfg = self._read_camera_cfg()

        # Time armed this power-on, for the flight log. Ticked by its own timer,
        # not by the page's poll: it must count whether or not a browser is open.
        self._armed_clock = armed_clock.ArmedClock(p["armed_time_file"],
                                                   armed_clock.read_boot_id())
        self.create_timer(1.0, self._tick_armed_clock)

        self._workspace = self._check_workspace(p["workspace_path"])

        self.server = GcsServer(render_page(p["poll_period_s"] * 1000.0),
                                self._snapshot, self._action)
        self.server.start(int(p["port"]), p["bind_host"])
        self.get_logger().info(
            "ground station on http://<JETSON_IP>:%d — nodes, telemetry, map, "
            "logs, system. power %s."
            % (int(p["port"]),
               "ENABLED" if p["allow_power"] else "disabled by param"))

    # ---------- startup checks ----------

    def _set_origin(self, origin):
        """Re-anchor the map's local metres, and everything drawn in them."""
        self._origin = origin
        self._fence_xy = [list(geo.latlon_to_xy(a, b, origin)) for a, b in self._fence]
        self._trail.clear()
        self._origin_id += 1

    def _check_workspace(self, path):
        """Is the workspace a bind mount, and say so once at startup.

        Checked at construction rather than per poll: mounts do not change under
        a running container, and someone needs the answer BEFORE they spend an
        afternoon wondering why a `git pull` changed nothing.
        """
        mp, src, persists = system_info.mount_for(path)
        if persists:
            self.get_logger().info(
                "workspace %s is bind-mounted from %s (on %s) — a `git pull` on "
                "the host is visible in here" % (path, src, mp))
        else:
            self.get_logger().warn(
                "WORKSPACE IS NOT A BIND MOUNT: %s is on the container's own "
                "filesystem. A `git pull` on the host is INVISIBLE in here, a "
                "rebuild will silently change nothing, and `docker rm` discards "
                "the lot. Recreate the container with "
                "-v ~/robotx_ws:/root/robotx_ws (see README)." % path)
        return {"persists": bool(persists), "mount": mp, "source": src}

    def _apply(self, changes):
        if "trail_length" in changes:
            self._trail = deque(self._trail,
                                maxlen=int(changes["trail_length"]) or 1)
        self.p.update(changes)

    # ---------- inputs ----------

    def _on_fence(self, msg: Fence):
        self._fence_problem = "" if msg.valid else msg.problem
        if not msg.valid:
            return                  # keep drawing the last fence known
        poly = list(zip(msg.latitude, msg.longitude))
        if poly == self._fence and self._fence_src == "autopilot":
            return
        self._fence, self._fence_src = poly, "autopilot"
        self._set_origin(_centroid(poly))
        self.get_logger().info("map: drawing the autopilot's %d-corner fence"
                               % len(poly))

    def _on_search(self, msg: SearchStatus):
        self._search.set(msg, time.monotonic())

    def _on_boat(self, msg: BoatState):
        self._boat.set(msg, time.monotonic())

    def _on_radio(self, msg: RadioFrame):
        self._radio.add(msg.direction, msg.src_system, msg.src_component,
                        msg.dst_system, msg.msg_name, msg.payload_type,
                        msg.summary, msg.frame_bytes,
                        stamp=msg.header.stamp.sec
                        + msg.header.stamp.nanosec * 1e-9)

    def _read_boat_sysid(self):
        """telemetry_bridge's boat_sysid, or None if it is 0 or unreadable --
        then the Radio tab says it cannot score the boat link, rather than
        guessing which system is the boat."""
        try:
            return int(uav_config.node_params("telemetry_bridge")["boat_sysid"]) or None
        except Exception as e:
            self.get_logger().warn(
                "boat_sysid unreadable from uav_params.yaml (%s): the Radio tab "
                "will not estimate the boat link" % e)
            return None

    def _boat_state(self, now):
        """Crusader in the map's local metres, or None while it is not heard."""
        b = self._boat.get(now)
        if b is None:
            return None
        x, y = geo.latlon_to_xy(b.latitude, b.longitude, self._origin)
        return {"x": x, "y": y, "doing": _BOAT_DOING.get(int(b.activity), ""),
                "target": int(b.target_buoy)}

    def _on_pose(self, msg: GlobalPos):
        # NaN heading is kept, not dropped: the readout says so and the operator
        # needs to know GPS yaw is unresolved. Only the trail skips it, because
        # a NaN cannot be plotted.
        x, y = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
        if self._fence_src == "params" and math.hypot(x, y) > FOREIGN_ORIGIN_M:
            self._fence_src = "params (far away)"
            self._set_origin((msg.latitude, msg.longitude))
            x, y = 0.0, 0.0
        self._pose.set((msg, x, y), time.monotonic())
        gate = self.p["trail_min_move_m"]
        if not self._trail or math.hypot(x - self._trail[-1][0],
                                         y - self._trail[-1][1]) >= gate:
            self._trail.append((x, y))

    def _on_att(self, msg: Attitude):
        self._att.set(msg, time.monotonic())

    def _on_status(self, msg: FcuStatus):
        self._status.set(msg, time.monotonic())

    def _on_flight(self, msg: FlightState):
        self._flight.set(msg, time.monotonic())

    def _on_rosout(self, msg: Log):
        """Every node's logger output, from anywhere in the DDS domain."""
        self.logs.add(msg.name, msg.level, msg.msg,
                      stamp=msg.stamp.sec + msg.stamp.nanosec * 1e-9)

    def _scan_graph(self):
        """Which registry nodes are present, from the graph AND /proc."""
        try:
            self._graph = {n for n, _ns in self.get_node_names_and_namespaces()}
        except Exception:
            pass                       # discovery hiccup; keep the last answer
        self._proc = proc_scan.scan([s.executable for s in reg.REGISTRY])

    # ---------- the snapshot ----------

    def _armed(self):
        """(known, armed). Unknown is NOT the same as disarmed, and the power
        interlock treats it as unsafe."""
        s = self._status.get(time.monotonic())
        return (s is not None), (bool(s.armed) if s else False)

    def _node_items(self):
        now = time.monotonic()
        running_names = set()
        items = []
        for spec in reg.REGISTRY:
            state, detail = self.procs.status(spec.name)
            pids = self._proc.get(spec.executable) or []
            running = (state == "running") or (spec.name in self._graph) or bool(pids)
            if running:
                running_names.add(spec.name)
                if state != "running":
                    # Not ours. Say where we saw it, because that decides how it
                    # would be signalled if it is ever stoppable.
                    detail = ("in the ROS graph" if spec.name in self._graph
                              else "pid %s (/proc)" % ",".join(map(str, pids)))
            verb = reg.stop_verb(spec.name)
            allowed, reason = reg.may_stop(spec.name, verb)
            items.append({
                "name": spec.name, "label": spec.label, "group": spec.group,
                "running": running, "detail": detail, "note": spec.note,
                "may_stop": bool(allowed and running), "stop_reason": reason,
                # "restart" or "stop": the page labels the button with this.
                "verb": verb, "unit": spec.unit,
                "restarting": reg.restarting(spec.name, running_names,
                                             self._restart_t.get(spec.name), now),
                "owned": state == "running", "pids": pids,
            })
        return items, running_names

    def _snapshot(self):
        now = time.monotonic()
        items, running = self._node_items()
        groups = [{"key": k, "label": label, "why": why,
                   "nodes": [i for i in items if i["group"] == k]}
                  for k, label, why in reg.GROUPS]

        pose_e = self._pose.get(now)
        att = self._att.get(now)
        st = self._status.get(now)
        fs = self._flight.get(now)
        pose = pose_e[0] if pose_e else None

        tel = {
            "pose_ok": pose is not None,
            "att_ok": att is not None,
            "fcu_ok": st is not None,
            "flight_ok": fs is not None,
        }
        if pose is not None:
            hdg = pose.heading
            tel.update({
                "lat": pose.latitude, "lon": pose.longitude,
                "heading": None if math.isnan(hdg) else hdg,
                "speed": pose.ground_speed, "climb": pose.climb,
                "alt_amsl": pose.altitude_amsl, "alt_rel": pose.altitude_rel,
                # The same sum ocs_client sends. Shown so an operator can sanity
                # check the venue geoid constant against a known field
                # elevation, which is the only way it ever gets caught.
                "alt_hae": pose.altitude_amsl + self._geoid(),
                "inside": geo.point_in_polygon(pose.latitude, pose.longitude,
                                               self._fence),
            })
        if att is not None:
            tel.update({"roll": math.degrees(att.roll),
                        "pitch": math.degrees(att.pitch),
                        "yaw": math.degrees(att.yaw)})
        if st is not None:
            tel.update({"mode": st.mode, "armed": bool(st.armed)})
        if fs is not None:
            tel["landed"] = _LANDED_NAME.get(fs.landed_state, "?%d" % fs.landed_state)

        known, armed = self._armed()
        cam = self._cam_state(running)
        params = self._fcu_params or {}
        batt = self._battery.snapshot(now, params.get("batt_low_volt"),
                                      float(self.p["status_timeout_s"]))
        gps = self._gps.get(now)
        search = self._search_state(now, running)
        return {
            "groups": groups,
            "tel": tel,
            "batt": batt,
            "gps": gps,
            "armed_time": self._armed_clock.snapshot(now),
            "preflight": preflight_core.checks(self._preflight_inputs(
                tel, batt, gps, params, cam, running, search)),
            "ocs": self._ocs_state(),
            "map": {
                "fence": self._fence_xy,
                # "autopilot" = read back from it; anything else is the params
                # stand-in, and the page says so.
                "fence_src": self._fence_src,
                "fence_problem": self._fence_problem,
                "origin_id": self._origin_id,
                "search": search,
                "veh": (None if pose is None else
                        {"x": pose_e[1], "y": pose_e[2],
                         "heading": (0.0 if math.isnan(pose.heading)
                                     else pose.heading)}),
                "inside": tel.get("inside"),
                "trail_gate": self.p["trail_min_move_m"],
                "trail_max": int(self.p["trail_length"]),
                "boat": self._boat_state(now),
                "buoys": self._buoy_state(now),
                "mapper": self._mapper_state(running),
                "footprint": self._footprint(pose_e, att, tel),
            },
            "sys": self._sys_state(),
            "power": self._power_state(known, armed),
            "cam": cam,
        }

    # ---------- battery, GPS, pre-flight, footprint ----------

    def _tick_armed_clock(self):
        known, armed = self._armed()
        self._armed_clock.update(time.monotonic(), known, armed)

    def _on_battery(self, msg):
        known, armed = self._armed()
        self._battery.feed(time.monotonic(), msg.voltage, msg.current,
                           msg.consumed_mah, int(msg.remaining_pct),
                           known and armed)

    def _on_gps(self, msg):
        fix = int(msg.fix_type)
        self._gps.set({"fix_type": fix,
                       "fix_name": preflight_core.FIX_NAMES.get(fix, "fix %d" % fix),
                       "satellites": int(msg.satellites),
                       "hdop": _finite(msg.hdop), "h_acc_m": _finite(msg.h_acc_m)},
                      time.monotonic())

    def _on_fcu_params(self, msg):
        self._fcu_params = {f: _finite(getattr(msg, f)) for f in _FCU_PARAM_FIELDS}

    def _search_state(self, now, running):
        """The buoy search for the Map tab and the checklist, in map metres.

        {"running": False} when search_node is not running; phase None when it
        runs but its status is stale -- the page then shows no plan rather than
        the last one heard, drawn as if current.
        """
        if "search_node" not in running:
            return {"running": False}
        s = self._search.get(now)
        if s is None:
            return {"running": True, "phase": None}
        o = self._origin

        def xy(lats, lons):
            return [list(geo.latlon_to_xy(a, b, o)) for a, b in zip(lats, lons)]
        target = (None if math.isnan(s.target_latitude) else
                  list(geo.latlon_to_xy(s.target_latitude, s.target_longitude, o)))
        return {
            "running": True, "phase": s.phase, "text": s.text,
            "tier": s.tier, "note": s.note,
            "watching": list(s.watching), "rechecking": list(s.rechecking),
            "waiting": list(s.waiting_for), "enabled": s.enabled,
            "count": s.buoys_to_find, "found": s.found, "flying": s.flying,
            "pass": s.pass_number, "leg": s.leg, "legs": s.legs,
            "hover": list(s.hover_buoys), "hover_s": _finite(s.hover_s),
            "give_up_s": _finite(s.give_up_s), "skipped": list(s.skipped),
            "target": target,
            "plan": xy(s.plan_latitude, s.plan_longitude),
            "inset": xy(s.inset_latitude, s.inset_longitude),
        }

    def _read_camera_cfg(self):
        try:
            cam = uav_config.node_params("camera_node")
            mp = uav_config.node_params("buoy_mapper")
            return {
                "hfov_deg": float(mp["hfov_deg"]),
                "yaw_mode": str(mp["gimbal_yaw_mode"]),
                "yaw_sign": float(mp["gimbal_yaw_sign"]),
                "mount_offset_deg": float(mp["mount_yaw_offset_deg"]),
                "launch_height_m": float(mp["launch_height_above_surface_m"]),
                "max_off_nadir_deg": float(mp["max_off_nadir_deg"]),
                "working_alt_m": float(mp["waypoint_alt_m"]),
                "nadir_pitch": float(cam["gimbal_pitch_deg"]),
                "expected_codec": str(cam["rtsp_codec"]),
            }
        except Exception as e:
            self.get_logger().warn(
                "camera geometry unreadable from uav_params.yaml (%s): no camera "
                "footprint on the map, and no fence head-room check" % e)
            return None

    def _footprint(self, pose_e, att, tel):
        """The ground patch the camera sees, as map x/y corners, or None.

        Drawn only at nadir: off nadir the patch is a pitch-dependent trapezoid,
        and the mapper refuses those frames anyway. The heading goes through
        camera_frame.camera_heading_deg, the same function the mapper uses, so
        the rectangle and the buoy positions agree on where the camera points.
        """
        cfg = self._cam_cfg
        c = self._cam_status.get(time.monotonic())
        if cfg is None or pose_e is None or att is None or c is None:
            return None
        pitch = c["gimbal_pitch"]
        if not c["gimbal_ok"] or math.isnan(pitch) \
                or abs(pitch - cfg["nadir_pitch"]) > cfg["max_off_nadir_deg"]:
            return None
        heading = camera_frame.camera_heading_deg(
            att.yaw, c["gimbal_yaw"], cfg["yaw_mode"], cfg["yaw_sign"],
            cfg["mount_offset_deg"])
        corners = camera_frame.nadir_footprint(
            tel.get("alt_rel", float("nan")) + cfg["launch_height_m"], heading,
            cfg["hfov_deg"])
        if corners is None:
            return None
        return [[pose_e[1] + e, pose_e[2] + n] for e, n in corners]

    def _preflight_inputs(self, tel, batt, gps, params, cam, running, search):
        """Gather what preflight_core needs into one plain dict."""
        c = self._cam_status.get(time.monotonic())
        cfg = self._cam_cfg or {}
        camera = {"running": cam.get("source") is not None}
        if c is not None:
            camera.update({
                "gimbal_ok": c["gimbal_ok"], "gimbal_pitch": _finite(c["gimbal_pitch"]),
                "nadir_pitch": cfg.get("nadir_pitch", -90.0),
                "codec": c["codec"], "width": c["width"], "height": c["height"],
                "kbps": c["kbps"], "encoding_age_s": _finite(c["encoding_age_s"]),
                "expected_codec": cfg.get("expected_codec", ""),
            })
        return {
            "fcu_ok": tel.get("fcu_ok"), "pose_ok": tel.get("pose_ok"),
            "armed": tel.get("armed"), "gps": gps, "battery": batt,
            "fence_enable": params.get("fence_enable"),
            "fence_alt_max": params.get("fence_alt_max"),
            "fence_margin": params.get("fence_margin"),
            "fence_type": params.get("fence_type"),
            "working_alt_m": cfg.get("working_alt_m"),
            "camera": camera,
            "record_gate": None if c is None else c["record_gate"],
            "mapping": {"detector": "detector_node" in running,
                        "mapper": "buoy_mapper" in running},
            "search": search,
        }

    def _on_buoy_map(self, msg):
        self._buoy_map.set(msg, time.monotonic())

    def _buoy_state(self, now):
        """Buoys for the Map tab, in the same local metres as the fence and trail.

        None when the map is stale -- the page then says the mapper is silent
        instead of drawing the last positions it heard as if they were current.
        """
        m = self._buoy_map.get(now)
        if m is None:
            return None
        out = []
        for b in m.buoys:
            x, y = geo.latlon_to_xy(b.latitude, b.longitude, self._origin)
            out.append({
                "id": b.id, "x": x, "y": y, "lat": b.latitude,
                "lon": b.longitude, "label": b.label, "state": b.state,
                "colour": b.colour, "locked": b.locked,
                "spread_m": b.spread_m, "sightings": b.sightings,
                "observed_s": b.observed_s, "lit_fraction": b.lit_fraction,
            })
        return {"buoys": out, "stem": m.export_stem,
                "used": m.detections_used, "partial": m.detections_partial,
                "low_conf": m.detections_low_conf,
                "rejected": m.frames_rejected,
                "reject_reason": m.last_reject_reason}

    def _mapper_state(self, running):
        """Where buoy_mapper's downloads are, if it is serving them.

        Same running/serving distinction as the camera: a download link to a
        port that has not bound yet is a link that fails.
        """
        spec = reg.BY_NAME.get("buoy_mapper")
        if spec is None or not spec.port or "buoy_mapper" not in running:
            return None
        return {"serving": self._port_open(spec.port), "port": spec.port}

    def _on_cam_status(self, msg):
        # Cached as ONE entry so one staleness timeout covers every field.
        # Splitting them would let the page show a fresh reason beside a stale
        # record state, or a fresh gimbal angle beside a stale encoding.
        self._cam_status.set({
            "recording_sd": bool(msg.recording_sd),
            "record_gate": str(msg.record_gate),
            "gimbal_ok": bool(msg.gimbal_ok),
            "gimbal_pitch": msg.gimbal_pitch, "gimbal_yaw": msg.gimbal_yaw,
            "codec": str(msg.stream_codec), "width": int(msg.stream_width),
            "height": int(msg.stream_height), "kbps": int(msg.stream_kbps),
            "encoding_age_s": msg.encoding_age_s,
        }, time.monotonic())

    def _cam_state(self, running):
        """What the Camera tab should point at, if anything.

        The running/serving distinction is the whole reason tab_source exists:
        camera_node appears in /proc within a second of starting and then spends
        several more negotiating RTSP before its viewer port binds. A tab that
        trusted "running" alone would point an <img> at a closed port, get
        connection-refused, and stay on that error until someone reloaded by
        hand — see tab_source's docstring.
        """
        spec = reg.BY_NAME.get("camera_node")
        if spec is None or not spec.port:
            return {"source": None, "starting": False}
        serving = {spec.name} if self._port_open(spec.port) else set()
        name, starting = reg.tab_source((spec.name,), running, serving)
        st = self._cam_status.get(time.monotonic())
        rec = st["recording_sd"] if st else None
        gate = st["record_gate"] if st else ""
        return {
            "source": name,
            "starting": starting,
            "port": spec.port,
            "path": spec.stream_path,
            # None (not False) when the status topic is stale, so the page can
            # grey the button out rather than assert a state it cannot see.
            "recording_sd": rec,
            # Why the open session will be kept or discarded, in the camera
            # node's own words. Shown verbatim: this is the line that stops a
            # discarded sortie from being a silent surprise.
            "record_gate": gate,
            # The detector's annotated stream, if that node is up. Reported
            # separately from the camera so the tab can offer the toggle only
            # when there is something to toggle TO -- a button that points an
            # <img> at a closed port produces connection-refused and sits on
            # that error until someone reloads by hand.
            "det": self._det_state(running),
        }

    def _det_state(self, running):
        """Where the annotated view lives, if it is being served.

        Same running/serving distinction as the camera: detector_node appears
        in /proc immediately and then spends several seconds loading a model
        and connecting to the source before its port binds.
        """
        spec = reg.BY_NAME.get("detector_node")
        if spec is None or not spec.port or "detector_node" not in running:
            return None
        if not self._port_open(spec.port):
            return {"starting": True}
        return {"starting": False, "port": spec.port, "path": spec.stream_path}

    def _port_open(self, port):
        """Is something accepting on this port, checked at most once a second.

        Rate-limited because the browser polls /state five times a second and a
        connect attempt per poll would be a self-inflicted port scan on a flight
        computer. The cached answer being up to a second stale is fine: the tab
        it feeds takes longer than that to render anyway.
        """
        now = time.monotonic()
        cached = self._port_probe.get(port)
        if cached is not None and (now - cached[0]) < 1.0:
            return cached[1]
        s = socket.socket()
        # Short: this is a loopback connect to a port that is either bound or
        # not. Anything slower than this is a machine in trouble, and blocking
        # the snapshot to find that out helps nobody.
        s.settimeout(0.15)
        try:
            s.connect(("127.0.0.1", int(port)))
            ok = True
        except OSError:
            ok = False
        finally:
            s.close()
        self._port_probe[port] = (now, ok)
        return ok

    def _geoid(self):
        """The geoid separation ocs_client uses, read from the same file.

        Read rather than declared as our own parameter: two copies of a venue
        constant is exactly the drift check_config.py exists to stop, and the
        page showing a different HAE than the OCS receives would be worse than
        showing none.
        """
        try:
            return float(uav_config.node_params("ocs_client")["geoid_separation_m"])
        except Exception:
            return 0.0

    def _ocs_state(self):
        """What the OCS link is doing — from the graph, not from the node.

        ground_station does not import ocs_client and never will; it only knows
        whether that node exists. The detail the page shows beyond presence
        comes from /rosout, which is why the client logs its quiet reasons.
        """
        present = ("ocs_client" in self._graph
                   or bool(self._proc.get("ocs_client")))
        state = {"present": present}
        if present:
            # Best-effort read of the client's own last words on the subject.
            records, _newest, _dropped = self.logs.read(node="ocs_client",
                                                       limit=200)
            for line in reversed(records):
                m = line.get("msg", "")
                if "no heartbeat:" in m and "quiet_reason" not in state:
                    state["quiet_reason"] = m.split("no heartbeat:", 1)[1].strip()
                if "flight_phase source:" in m and "phase_source" not in state:
                    state["phase_source"] = m.rsplit(":", 1)[1].strip()
                if "connected to" in m:
                    state.setdefault("connected", True)
                if "cannot reach" in m or "closed by the OCS" in m:
                    state.setdefault("connected", False)
            state.setdefault("connected", False)
            state.setdefault("sent", "—")
            state.setdefault("skipped", 0)
        return state

    def _sys_state(self):
        # snapshot() returns *_gb / *_percent / uptime_s; the page wants short
        # names and a formatted uptime. Renaming here rather than in the page
        # keeps the units visible on this side, where they can be checked
        # against system_info, instead of implied by a JS label.
        raw = system_info.snapshot(self._cpu, self.p["disk_path"])
        up = raw.get("uptime_s")
        return {
            "hostname": self._hostname,
            "cpu": raw.get("cpu_percent"),
            "temp": raw.get("temp_c"),
            "mem_used": raw.get("mem_used_gb"),
            "mem_total": raw.get("mem_total_gb"),
            "disk_free": raw.get("disk_free_gb"),
            "disk_total": raw.get("disk_total_gb"),
            "disk_path": raw.get("disk_path"),
            "uptime": None if up is None else system_info.format_uptime(up),
            "host_time": raw.get("host_time"),
            "workspace": self._workspace,
        }

    def _power_state(self, known, armed):
        if not self.p["allow_power"]:
            return {"allowed": False,
                    "reason": "power is disabled by the allow_power parameter. "
                              "Install uav-shutdown.path / uav-reboot.path "
                              "(setup/install_jetson_host.sh) and set "
                              "allow_power: true if a browser may halt this "
                              "Jetson."}
        if not known:
            return {"allowed": False,
                    "reason": "armed state is UNKNOWN — no fresh FcuStatus. "
                              "That usually means telemetry_bridge is down, "
                              "which is not evidence the aircraft is safe to "
                              "reboot."}
        if armed:
            return {"allowed": False,
                    "reason": "vehicle is ARMED. Disarm before powering down."}
        ok, why = power_client.available(self.p["power_request_dir"])
        if not ok:
            # The client's own wording: it already explains the bind mount and
            # names the install script, and paraphrasing it here would give two
            # slightly different answers to the same question.
            return {"allowed": False, "reason": why}
        return {"allowed": True}

    # ---------- actions (every rule re-checked HERE) ----------

    def _action(self, path, payload):
        if path == "/node/start":
            return self._act_start(payload.get("name", ""))
        if path == "/node/stop":
            return self._act_stop(payload.get("name", ""), "stop")
        if path == "/node/restart":
            return self._act_stop(payload.get("name", ""), "restart")
        if path == "/logs":
            return self._act_logs(payload)
        if path == "/logs/clear":
            self.logs.clear()
            return {"ok": True, "message": "log buffer cleared"}
        if path == "/radio":
            return self._act_radio(payload)
        if path == "/radio/clear":
            self._radio.clear()
            return {"ok": True, "message": "radio log cleared"}
        if path == "/radio/send_test":
            return self._call_service(self._radio_test_cli, Trigger.Request(),
                                      "telemetry_bridge", "radio send_test")
        if path == "/map/clear_trail":
            self._trail.clear()
            return {"ok": True, "message": "trail cleared"}
        if path == "/map/clear_buoys":
            return self._call_service(self._clear_map_cli, Trigger.Request(),
                                      "buoy_mapper", "clear_buoy_map")
        if path == "/camera/capture":
            return self._act_capture(payload)
        if path == "/search/config":
            return self._act_search(payload)
        if path == "/power":
            return self._act_power(payload)
        return {"ok": False, "message": "unknown action %s" % path}

    def _act_capture(self, payload):
        """Turn CAPTURE on or off: the camera's 4K SD recording and the stills.

        Not the .mkv and not the frame index -- those are the flight record and
        always run, because a sortie nobody can diagnose is worse than a few
        hundred MB.
        """
        if "on" not in payload:
            return {"ok": False, "message": 'expected {"on": true} or {"on": false}'}
        req = SetBool.Request()
        req.data = bool(payload["on"])
        return self._call_service(self._capture_cli, req, "camera_node",
                                  "capture")

    def _act_search(self, payload):
        """The buoy search's page settings: {"enabled": bool} and/or
        {"buoys_to_find": int}.

        Both are safe to change at any time, and that is by design rather than
        by checking here: ON cannot start a flight (only the pilot's switch into
        GUIDED does), OFF makes the aircraft hold, and the count only decides
        when to RTL. The values are still validated, because anyone can curl
        this.
        """
        params = []
        if "enabled" in payload:
            if not isinstance(payload["enabled"], bool):
                return {"ok": False, "message": "enabled must be true or false"}
            params.append(Parameter(name="enabled", value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=payload["enabled"])))
        for name, allowed in (("task", TASKS), ("tier", TIERS)):
            if name in payload:
                if payload[name] not in allowed:
                    return {"ok": False, "message": "%s must be one of %s"
                            % (name, ", ".join(allowed))}
                params.append(Parameter(name=name, value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=payload[name])))
        if "buoys_to_find" in payload:
            n = payload["buoys_to_find"]
            lo, hi = BUOYS_TO_FIND_RANGE
            if isinstance(n, bool) or not isinstance(n, int) or not lo <= n <= hi:
                return {"ok": False,
                        "message": "buoys to find must be a whole number %d-%d" % (lo, hi)}
            params.append(Parameter(name="buoys_to_find", value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=n)))
        if not params:
            return {"ok": False,
                    "message": 'expected "enabled", "buoys_to_find", "task" '
                               'and/or "tier"'}
        tier_note = ""
        if "tier" in payload:
            tier_note = self._set_mapper_for_tier(payload["tier"])
        req = SetParameters.Request(parameters=params)

        def reply(res):
            bad = [r.reason for r in res.results if not r.successful]
            if bad:
                return False, "search_node refused: %s" % "; ".join(bad)
            parts = []
            if "enabled" in payload:
                parts.append("search %s" % ("ON: the pilot's switch into GUIDED "
                                            "starts it" if payload["enabled"]
                                            else "OFF"))
            if "buoys_to_find" in payload:
                parts.append("find %d buoys" % payload["buoys_to_find"])
            if "tier" in payload:
                parts.append("%s tier: %s" % (
                    payload["tier"],
                    "stay on station over the boat's gate once mapped"
                    if payload["tier"] == "disruptive" else "RTL once mapped"))
            if "task" in payload:
                parts.append(payload["task"])
            return True, ", ".join(parts) + tier_note
        return self._call_service(self._search_params_cli, req, "search_node",
                                  "set_parameters", reply)

    def _set_mapper_for_tier(self, tier):
        """Put buoy_mapper into the state the tier needs. -> a note for the page.

        The tier is one choice on one page; the two settings it implies are set
        here rather than left to the operator, because a Disruptive run with a
        frozen state decides the passage once and never notices it change.
        """
        want = TIER_MAPPER[tier]
        if not self._mapper_params_cli.service_is_ready():
            return (" — buoy_mapper is not running, so its state rules were "
                    "not set; start it and set the tier again")
        req = SetParameters.Request(parameters=[
            Parameter(name="lock_state", value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=want["lock_state"])),
            Parameter(name="decide_window_s", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=want["decide_window_s"]))])
        out = self._call_service(self._mapper_params_cli, req, "buoy_mapper",
                                 "set_parameters",
                                 lambda res: (all(x.successful for x in res.results),
                                              "; ".join(x.reason for x in res.results
                                                        if not x.successful)))
        if not out["ok"]:
            self.get_logger().error("buoy_mapper refused the tier settings: %s"
                                    % out["message"])
            return " — buoy_mapper refused: %s" % out["message"]
        return (" — buoy states %s"
                % ("can change, decided from the last %.0f s"
                   % want["decide_window_s"] if not want["lock_state"]
                   else "freeze once decided"))

    def _call_service(self, cli, req, who, what, reply=None):
        """Call a service from an HTTP action; -> {ok, message}.

        `reply` turns the response into (ok, message); by default the response
        is a std_srvs one with success and message.

        WAITING ON THE FUTURE HERE IS SAFE, AND ONLY HERE. This runs on the HTTP
        server's thread, never inside a ROS callback, so the executor spinning in
        the main thread is free to complete the call. That is also why it polls
        done() instead of spin_until_future_complete: spinning from this thread
        would fight the executor that already owns this node.
        """
        if not cli.service_is_ready():
            return {"ok": False,
                    "message": "%s is not offering %s — is it running?"
                               % (who, cli.srv_name)}
        fut = cli.call_async(req)
        deadline = time.monotonic() + SERVICE_TIMEOUT_S
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not fut.done():
            return {"ok": False,
                    "message": "%s did not answer %s within %.0fs; nothing has "
                               "changed as far as this page knows"
                               % (who, what, SERVICE_TIMEOUT_S)}
        res = fut.result()
        ok, message = reply(res) if reply else (res.success, res.message)
        return {"ok": bool(ok), "message": message}

    def _act_start(self, name):
        _items, running = self._node_items()
        why = reg.start_refusal(name, running, self._restart_t.get(name),
                                time.monotonic())
        if why:
            return {"ok": False, "message": why}
        ok, msg = self.procs.start(reg.BY_NAME[name])
        return {"ok": ok, "message": msg}

    def _act_stop(self, name, verb):
        """Stop an unsupervised node, or restart a supervised one.

        Both kill the process; what differs is who brings it back. For a node
        its systemd unit started, the unit does — so the kill IS the restart,
        and the time is recorded so a START in the gap is refused. For a copy
        this page started itself (the unit had given up), nothing would, so a
        restart starts it again here.
        """
        # THE gate. Re-checked here and not merely rendered, because anyone can
        # curl this endpoint.
        allowed, reason = reg.may_stop(name, verb)
        if not allowed:
            self.get_logger().warn("refused %s of %r: %s" % (verb, name, reason))
            return {"ok": False, "message": reason}
        spec = reg.BY_NAME[name]
        state, _detail = self.procs.status(name)
        if state == "running":
            ok, msg = self.procs.stop(name)
            if ok and verb == "restart":
                ok, msg = self.procs.start(spec)
                msg = "restarted %s (started from this page, so not by %s): %s" \
                      % (name, spec.unit, msg)
            return {"ok": ok, "message": msg}
        pids = self._proc.get(spec.executable)
        if not pids:
            return {"ok": False, "message": "%s is not running" % name}
        # By PID, never by process group: a node started by systemd or a
        # launch file shares its group with the whole unit, and signalling
        # that group would take everything else down with it.
        ok, msg = self.procs.stop_external(name, pids)
        if ok and verb == "restart":
            self._restart_t[name] = time.monotonic()
            msg = "restarting %s: %s brings it back in a few seconds" \
                  % (name, spec.unit)
        return {"ok": ok, "message": msg}

    def _act_logs(self, payload):
        try:
            since = int(payload.get("since", 0))
            limit = min(int(payload.get("limit", 300)), 1000)
        except (TypeError, ValueError):
            return {"ok": False, "message": "since/limit must be integers"}
        records, newest, dropped = self.logs.read(since_seq=since, limit=limit)
        # Shaped for the page: it wants a wall-clock string and short level
        # name, and building those here keeps the JS free of date formatting.
        lines = [{"seq": r["seq"],
                  "t": time.strftime("%H:%M:%S", time.localtime(r["t"])),
                  "name": r["node"], "level": r["level"],
                  "lvl": r["level_name"], "msg": r["msg"]}
                 for r in records]
        return {"ok": True, "message": "", "lines": lines, "newest": newest,
                "nodes": self.logs.nodes(), "dropped": dropped}

    def _act_radio(self, payload):
        """Radio frames the page has not seen, plus the per-system and boat
        link summaries, which the ground station keeps whether or not the tab
        is open."""
        try:
            since = int(payload.get("since", 0))
            limit = min(int(payload.get("limit", 300)), 1000)
        except (TypeError, ValueError):
            return {"ok": False, "message": "since/limit must be integers"}
        records, newest, dropped = self._radio.read(since_seq=since, limit=limit)
        rows = [dict(r, t=time.strftime("%H:%M:%S", time.localtime(r["t"])))
                for r in records]
        return {"ok": True, "message": "", "rows": rows, "newest": newest,
                "dropped": dropped, "systems": self._radio.systems(),
                "boat": self._radio.boat_link()}

    def _act_power(self, payload):
        verb = payload.get("verb", "")
        known, armed = self._armed()
        gate = self._power_state(known, armed)
        if not gate.get("allowed"):
            self.get_logger().warn("refused power %r: %s"
                                   % (verb, gate.get("reason")))
            return {"ok": False, "message": gate.get("reason")}
        if payload.get("confirm", "") != self._hostname:
            return {"ok": False,
                    "message": "type the hostname %r to confirm" % self._hostname}
        if verb not in ("shutdown", "reboot"):
            return {"ok": False, "message": "unknown verb %r" % verb}
        self.get_logger().warn("POWER %s requested from the ground station" % verb)
        try:
            reply = power_client.request(
                verb, self.p["power_request_dir"],
                reason="ground_station, confirmed by hostname")
        except power_client.PowerUnavailable as e:
            self.get_logger().error("power request: %s" % e)
            return {"ok": False, "message": str(e)}
        except ValueError as e:
            return {"ok": False, "message": str(e)}
        # The host deletes the request BEFORE it acts, so the file vanishing
        # means systemd picked it up -- accepted, not done.
        return {"ok": True, "message": "%s: %s" % (verb, reply)}

    # ---------- teardown ----------

    def destroy_node(self):
        # Deliberately does NOT stop the children. Closing the dashboard must
        # not stop the aircraft's stack — a ground station that killed the stack
        # on exit is one nobody would dare restart mid-session.
        self.server.stop()
        super().destroy_node()


def main(args=None):
    run_node(GroundStation, args=args)


if __name__ == "__main__":
    main()
