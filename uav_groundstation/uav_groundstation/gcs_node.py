"""ground_station — the whole aircraft in one browser tab.

NOTE: unverified in flight. It starts and stops real nodes and can power the
Jetson off, and none of that has been exercised on an airframe. It is started by
systemd (tools/systemd/uav-groundstation.service) because nobody will SSH into
this Jetson between flights; `systemctl disable uav-groundstation` is the way
back.

    ros2 run uav_groundstation ground_station
    # then, on the laptop:  http://<JETSON_IP>:8090

Subscribes (read-only): /uav/pose, /uav/attitude, /uav/fcu_status,
/uav/flight_state. Publishes nothing. Its only outward effects are the processes
it spawns and the two power verbs it can hand to the host helper — both gated,
both re-checked server-side.

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

from rcl_interfaces.msg import Log

from uav_msgs.msg import Attitude, FcuStatus, FlightState, GlobalPos

from uav_common import config as uav_config
from uav_common import geo
from uav_common.fence_core import polygon_from_flat
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config, make_set_callback
from uav_common.stream_cache import StreamCache

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
}

_LANDED_NAME = {0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF",
                4: "LANDING"}


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

        # The fence is the map's origin. Anchoring on it rather than on the
        # first fix means the polygon does not jump the moment GPS arrives, and
        # two sessions draw the same picture.
        # Flat [lat, lon, ...] in the params because ROS parameters cannot
        # nest; paired here by the one function that owns that conversion.
        self._fence = polygon_from_flat(p["geofence"])
        self._origin = self._fence_centroid()
        self._fence_xy = [list(geo.latlon_to_xy(a, b, self._origin))
                          for a, b in self._fence]

        self._trail = deque(maxlen=int(p["trail_length"]) or 1)
        self._cpu = system_info.CpuMeter()
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

    def _fence_centroid(self):
        pts = self._fence[:-1] if (len(self._fence) > 1
                                   and self._fence[0] == self._fence[-1]) \
            else self._fence
        if not pts:
            return (0.0, 0.0)
        return (sum(a for a, _ in pts) / len(pts),
                sum(b for _, b in pts) / len(pts))

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

    def _on_pose(self, msg: GlobalPos):
        # NaN heading is kept, not dropped: the readout says so and the operator
        # needs to know GPS yaw is unresolved. Only the trail skips it, because
        # a NaN cannot be plotted.
        x, y = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
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
            allowed, reason = reg.may_stop(spec.name)
            items.append({
                "name": spec.name, "label": spec.label, "group": spec.group,
                "running": running, "detail": detail, "note": spec.note,
                "may_stop": bool(allowed and running), "stop_reason": reason,
                "owned": state == "running", "pids": pids,
            })
        return items, running_names

    def _snapshot(self):
        now = time.monotonic()
        items, _running = self._node_items()
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
        return {
            "groups": groups,
            "tel": tel,
            "ocs": self._ocs_state(),
            "map": {
                "fence": self._fence_xy,
                "veh": (None if pose is None else
                        {"x": pose_e[1], "y": pose_e[2],
                         "heading": (0.0 if math.isnan(pose.heading)
                                     else pose.heading)}),
                "inside": tel.get("inside"),
                "trail_gate": self.p["trail_min_move_m"],
                "trail_max": int(self.p["trail_length"]),
            },
            "sys": self._sys_state(),
            "power": self._power_state(known, armed),
        }

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
            return self._act_stop(payload.get("name", ""))
        if path == "/logs":
            return self._act_logs(payload)
        if path == "/logs/clear":
            self.logs.clear()
            return {"ok": True, "message": "log buffer cleared"}
        if path == "/map/clear_trail":
            self._trail.clear()
            return {"ok": True, "message": "trail cleared"}
        if path == "/power":
            return self._act_power(payload)
        return {"ok": False, "message": "unknown action %s" % path}

    def _act_start(self, name):
        spec = reg.BY_NAME.get(name)
        if spec is None:
            return {"ok": False, "message": "unknown node %r" % name}
        _items, running = self._node_items()
        if name in running:
            return {"ok": False, "message": "%s is already running" % name}
        clash = reg.conflicts(name, running)
        if clash:
            return {"ok": False,
                    "message": "%s cannot start while %s is running (they "
                               "contend for one device)" % (name, ", ".join(clash))}
        ok, msg = self.procs.start(spec)
        return {"ok": ok, "message": msg}

    def _act_stop(self, name):
        # THE gate. Re-checked here and not merely rendered, because anyone can
        # curl this endpoint.
        allowed, reason = reg.may_stop(name)
        if not allowed:
            self.get_logger().warn("refused stop of %r: %s" % (name, reason))
            return {"ok": False, "message": reason}
        state, _detail = self.procs.status(name)
        if state == "running":
            ok, msg = self.procs.stop(name)
        else:
            spec = reg.BY_NAME.get(name)
            pids = self._proc.get(spec.executable) if spec else None
            if not pids:
                return {"ok": False, "message": "%s is not running" % name}
            # By PID, never by process group: a node started by systemd or a
            # launch file shares its group with the whole unit, and signalling
            # that group would take everything else down with it.
            ok, msg = self.procs.stop_external(name, pids)
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
