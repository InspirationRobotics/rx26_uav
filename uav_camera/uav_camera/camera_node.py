"""camera_node — the single owner of the SIYI A8 mini.

Four jobs, fused into one node because all four need the one camera and there
may only ever be one holder of it:

1. STREAM: hold the RTSP connection and hand decoded frames to whatever wants
   them (pipeline.py). Nothing else in the workspace opens the camera.
2. VIEW: serve the operator an MJPEG stream on its own port (mjpeg_server.py),
   which node_registry's unused `port`/`stream_path` fields were left for.
3. RECORD: to the Jetson AND to the camera's own SD card. Both, because they
   fail differently -- a Jetson crash costs the .mkv and keeps the card; a full
   card costs the card and keeps the .mkv.
4. POINT: command the gimbal to nadir once at startup, and publish the MEASURED
   angle (siyi_client.py).

WHAT IT PUBLISHES, AND WHY THAT IS SO LITTLE. One topic, /uav/camera/status, at
a low rate. Frames are NOT published: a 1080p25 BGR stream is ~150 MB/s, and
putting that through the ROS graph on a Jetson that also has to keep a telemetry
gateway alive would be a poor trade for a subscriber that does not exist yet.
When the perception node arrives it will run in-process against pipeline's frame
callback, or take a shared-memory transport -- either way that is a decision to
make with a real consumer in front of us, not to pre-build now.

THE FRAME INDEX IS THE POINT OF THE RECORDING. A video with no idea where the
aircraft was is training data for nothing. Every frame writes a row joining its
ROS timestamp to the pose, and writes BLANKS when the pose was stale -- see
recorder_core, which owns that rule and is benched on it.

NO MAVLINK. This node subscribes to /uav/pose and /uav/attitude like any other
consumer. telemetry_bridge is the sole MAVLink consumer and that is what keeps
the single-owner rule enforceable rather than merely stated.

NO CONTROL SURFACE ON THE VIEWER. The MJPEG server serves GET only. README
safety constraint 6 -- WiFi is a convenience, never a control path -- stays true
only if nobody adds an HTTP endpoint that would make it false, and nobody has.

THE GIMBAL TOPIC IS A DIFFERENT QUESTION, AND IT IS GATED. This node originally
had no way to move the gimbal at all: it was commanded to nadir at startup and
that was the whole story. That was right when the only requirement was a fixed
nadir view, and it stopped being right as soon as another node on this Jetson
needed to change the angle without a human running a script.

So there is now a subscription to /uav/camera/gimbal_cmd -- and it EXISTS ONLY
IF gimbal_control_enabled IS TRUE, which the flight config leaves false. The
distinction that makes this defensible is between an endpoint reachable from a
browser (still absent) and a DDS topic on the vehicle's own graph (present when
asked for). Be clear-eyed about the residual: DDS is not local-only, so on a
vehicle whose ROS graph rides the field WiFi, anything on that network can
publish to it. Ship it false; turn it on for bench work and for a mission node
that genuinely needs to re-aim.
"""
import math
import os
import shutil
import threading
import time
from datetime import datetime, timezone

from geometry_msgs.msg import Vector3
from rclpy.node import Node

from uav_msgs.msg import Attitude, CameraStatus, GlobalPos

from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
from uav_common.stream_cache import StreamCache

from uav_camera import recorder_core
from uav_camera.mjpeg_server import FrameSlot, MjpegServer
from uav_camera.pipeline import Pipeline
from uav_camera.siyi_client import NullSiyiClient, SiyiClient, SiyiUnavailable

# Every parameter here is SAFETY/STRUCTURAL CONFIG -> read_only. The change path
# is uav_params.yaml plus a node restart, same posture as telemetry_bridge. A
# camera whose recording directory or gimbal angle could be changed from
# `ros2 param set` mid-flight is a surface nobody asked for.
PARAM_SPEC = {
    "rtsp_url": dict(read_only=True,
                     description="camera main stream; rtsp:// only"),
    "rtsp_latency_ms": dict(read_only=True, lo=0, hi=2000,
                            description="rtspsrc jitter buffer; 0 for lowest "
                                        "latency on a wired link"),
    "want_frames": dict(read_only=True,
                        description="build the decode branch at all. False = "
                                    "record-only sortie, near-zero CPU"),
    "preview_fps": dict(read_only=True, lo=1, hi=15,
                        description="operator view rate; not the record rate"),
    "mjpeg_port": dict(read_only=True, lo=1024, hi=65535,
                       description="viewer port; MUST differ from "
                                   "ground_station.port and ocs_client.ocs_port"),
    "bind_host": dict(read_only=True,
                      description="must not be 127.0.0.1 or the laptop cannot "
                                  "reach the viewer"),
    "siyi_enabled": dict(read_only=True,
                         description="false = stream and record with no gimbal "
                                     "control, for a bench with no SDK"),
    "siyi_ip": dict(read_only=True, description="gimbal SDK address"),
    "siyi_port": dict(read_only=True, lo=1, hi=65535,
                      description="gimbal SDK UDP port"),
    "gimbal_pitch_deg": dict(read_only=True, lo=-180.0, hi=180.0,
                             description="pitch commanded at startup. NADIR, "
                                         "and the sign is unit-specific: SIYI "
                                         "documents -90 as full-down and units "
                                         "have been found where that aims up. "
                                         "Measure it, then write it here"),
    "gimbal_control_enabled": dict(read_only=True,
                                   description="subscribe to "
                                               "/uav/camera/gimbal_cmd at all. "
                                               "FALSE in the flight config: no "
                                               "subscriber means no way to "
                                               "re-aim in flight"),
    "record_dir": dict(read_only=True,
                       description="where .mkv and _frames.csv are written"),
    "record_on_start": dict(read_only=True,
                            description="begin a session as soon as frames "
                                        "flow, rather than waiting to be asked"),
    "record_sd": dict(read_only=True,
                      description="also drive the camera's own SD recording"),
    "min_free_mb": dict(read_only=True, lo=64.0, hi=1_000_000.0,
                        description="stop recording above this floor, so the "
                                    "muxer can still finalise the file"),
    "max_session_s": dict(read_only=True, lo=0.0, hi=7200.0,
                          description="roll to a new session this often; 0 "
                                      "disables. Bounds what a crash costs"),
    "frame_timeout_s": dict(read_only=True, lo=0.2, hi=30.0,
                            description="s without a frame before the stream is "
                                        "reported down"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s; older than this "
                                       "and the index writes BLANKS, not stale "
                                       "coordinates"),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                               description="attitude goes stale independently "
                                           "of pose; separate MAVLink streams"),
    "status_rate_hz": dict(read_only=True, lo=0.2, hi=10.0,
                           description="/uav/camera/status rate. A heartbeat "
                                       "about the pipeline, not per frame"),
}

# Rows accumulated between status ticks before being written. Bounded because a
# stalled disk must not turn into unbounded memory on a flight computer: at
# 25 fps and a 2 Hz tick this holds ~13 rows, so 4096 is four minutes of
# backlog and anything beyond that is a fault, not a hiccup.
MAX_PENDING_ROWS = 4096


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")
        p = declare_from_config(self, uav_config.node_params("camera_node"),
                                PARAM_SPEC)
        self.p = p

        # Validated here, not at first frame. A bad URL or an unwritable record
        # directory is a config error, and discovering it when someone presses
        # record -- which will be on a flight line -- is discovering it too late.
        if not str(p["rtsp_url"]).startswith("rtsp://"):
            raise ValueError(
                "rtsp_url %r is not an rtsp:// URL. The SIYI proprietary UDP "
                "protocol on the same port is a different thing and GStreamer "
                "does not speak it." % p["rtsp_url"])
        self.record_dir = str(p["record_dir"])
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            probe = os.path.join(self.record_dir, ".uav_camera_write_test")
            with open(probe, "w"):
                pass
            os.remove(probe)
        except OSError as e:
            raise ValueError(
                "record_dir %r is not writable: %s\n"
                "  It must exist inside the container and ride the workspace "
                "bind mount, or the footage vanishes with the container."
                % (self.record_dir, e)) from e

        # ---- state, all under one lock; the GStreamer streaming thread and the
        # ROS executor both touch it.
        self._lock = threading.Lock()
        self._session = ""
        self._session_started_at = None
        self._frame_idx = 0
        self._pending = []
        self._csv = None
        self._mkv_path = None
        self._recording_sd = False
        self._last_frame_count = 0
        self._last_fps_t = None
        self._fps = float("nan")
        self._gimbal_pitch = float("nan")
        self._gimbal_ok = False

        self.guard = recorder_core.DiskGuard(float(p["min_free_mb"]))

        # ---- telemetry in. Same freshness discipline as every other consumer:
        # a stale pose is not a pose.
        self.pose_cache = StreamCache(float(p["pose_timeout_s"]))
        self.att_cache = StreamCache(float(p["attitude_timeout_s"]))
        self.frame_cache = StreamCache(float(p["frame_timeout_s"]))
        self.create_subscription(GlobalPos, "/uav/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/uav/attitude", self._on_att, 10)

        self.status_pub = self.create_publisher(
            CameraStatus, "/uav/camera/status", 10)

        # ---- viewer
        self.slot = FrameSlot()
        self.viewer = MjpegServer(self.slot, self._viewer_state)
        self.viewer.start(int(p["mjpeg_port"]), str(p["bind_host"]))
        self.get_logger().info(
            "viewer on http://%s:%d/stream.mjpg"
            % (p["bind_host"], p["mjpeg_port"]))

        # ---- gimbal. A silent gimbal is degraded, not fatal: the camera still
        # streams and still records wherever it happens to be pointing, and
        # losing the footage over the pointing link would be the wrong trade.
        nadir = float(p["gimbal_pitch_deg"])
        if p["siyi_enabled"]:
            try:
                self.siyi = SiyiClient(str(p["siyi_ip"]), int(p["siyi_port"]),
                                       nadir_pitch_deg=nadir).connect()
                if self.siyi.set_nadir():
                    self.get_logger().info(
                        "gimbal commanded to nadir (pitch %+.1f)" % nadir)
                else:
                    self.get_logger().error("gimbal did not accept the nadir "
                                            "command; it is wherever it was")
            except SiyiUnavailable as e:
                self.get_logger().error("gimbal unavailable: %s" % e)
                self.siyi = NullSiyiClient(nadir_pitch_deg=nadir)
        else:
            self.get_logger().warn(
                "siyi_enabled is false: no gimbal control, no SD recording. "
                "The camera will stream and record from the Jetson only.")
            self.siyi = NullSiyiClient(nadir_pitch_deg=nadir)

        # ---- the gimbal command topic, created only when asked for.
        #
        # A Vector3 rather than a new message type: x is yaw, y is pitch, both
        # degrees, z unused. uav_msgs would mean editing CMakeLists.txt, and a
        # message left out of that file is not generated and fails at import
        # with no hint that the .msg was ever the problem. Two floats do not
        # justify that risk.
        #
        # Fire and forget, by design. The answer to "did it get there" is not a
        # service response, it is gimbal_pitch on /uav/camera/status -- the
        # MEASURED angle, which is the only one geo-projection may use. A
        # response echoing the commanded angle would be a worse answer wearing
        # a more official hat.
        if p["gimbal_control_enabled"]:
            self.create_subscription(Vector3, "/uav/camera/gimbal_cmd",
                                     self._on_gimbal_cmd, 10)
            self.get_logger().warn(
                "gimbal_control_enabled: /uav/camera/gimbal_cmd is live. "
                "Anything on this ROS graph can re-aim the camera.")

        # ---- pipeline last: everything it calls back into must already exist.
        self.pipe = Pipeline(
            str(p["rtsp_url"]),
            on_frame=self._on_frame,
            on_jpeg=self._on_jpeg,
            on_error=self._on_pipeline_error,
            want_frames=bool(p["want_frames"]),
            preview_fps=int(p["preview_fps"]),
            latency_ms=int(p["rtsp_latency_ms"]))

        if p["record_on_start"]:
            self._start_session()
        desc = self.pipe.start(self._mkv_path)
        self.get_logger().info("pipeline: %s" % desc)

        self.create_timer(1.0 / float(p["status_rate_hz"]), self._status_tick)

    # ------------------------------------------------------------ telemetry in

    def _on_pose(self, msg):
        self.pose_cache.set((msg.latitude, msg.longitude, msg.altitude_rel),
                            time.monotonic(), msg.header.stamp)

    def _on_att(self, msg):
        self.att_cache.set((msg.roll, msg.pitch, msg.yaw),
                           time.monotonic(), msg.header.stamp)

    # ----------------------------------------------------------- gimbal in

    def _on_gimbal_cmd(self, msg):
        """Point the gimbal. x = yaw, y = pitch, degrees. z ignored.

        Only ever subscribed when gimbal_control_enabled is true.

        NaN IS REJECTED, not passed through. A NaN reaching setGimbalRotation
        is a command with no defined meaning, and the gimbal's response to one
        is not something to discover in flight. An out-of-range angle IS passed
        through: the gimbal stops at its own limits, and refusing it here would
        mean encoding a sign convention that has already been observed to
        differ between units.
        """
        yaw, pitch = float(msg.x), float(msg.y)
        if math.isnan(yaw) or math.isnan(pitch) or \
                math.isinf(yaw) or math.isinf(pitch):
            self.get_logger().warn(
                "gimbal_cmd ignored: yaw=%r pitch=%r is not a finite angle"
                % (msg.x, msg.y))
            return
        if self.siyi.set_angles(yaw, pitch):
            self.get_logger().info("gimbal commanded to yaw %+.1f pitch %+.1f"
                                   % (yaw, pitch))
        else:
            # Not an exception: a silent gimbal is degraded, not fatal, and the
            # same rule applies to a command as to the startup nadir.
            self.get_logger().error(
                "gimbal did not accept yaw %+.1f pitch %+.1f; it is wherever "
                "it was. Watch gimbal_pitch on /uav/camera/status."
                % (yaw, pitch))

    # --------------------------------------------------------------- sessions

    def _start_session(self):
        """Open a new .mkv + _frames.csv pair sharing one UTC stem."""
        stem = recorder_core.session_stem(datetime.now(timezone.utc))
        csv_path = os.path.join(self.record_dir, stem + "_frames.csv")
        try:
            csv = open(csv_path, "w", buffering=1)
            csv.write(recorder_core.csv_header() + "\n")
        except OSError as e:
            self.get_logger().error("cannot open %s: %s" % (csv_path, e))
            return False
        with self._lock:
            self._session = stem
            self._session_started_at = time.monotonic()
            self._frame_idx = 0
            self._pending = []
            self._csv = csv
            self._mkv_path = os.path.join(self.record_dir, stem + ".mkv")
        self.guard.reset()
        if self.p["record_sd"] and self.siyi.start_recording():
            self._recording_sd = True
        self.get_logger().info("recording session %s" % stem)
        return True

    def _end_session(self):
        with self._lock:
            csv, stem = self._csv, self._session
            rows, self._pending = self._pending, []
            self._csv = None
            self._session = ""
            self._session_started_at = None
            self._mkv_path = None
        if csv is not None:
            try:
                for r in rows:
                    csv.write(r + "\n")
                csv.close()
            except OSError:
                pass
        if self._recording_sd:
            self.siyi.stop_recording()
            self._recording_sd = False
        if stem:
            self.get_logger().info("session %s closed" % stem)

    # -------------------------------------------------------------- callbacks

    def _on_frame(self, _buf, _w, _h, pts_ns):
        """GStreamer streaming thread. Keep this short -- upstream queues are
        bounded and a slow callback stalls the pipeline back to the socket.

        The frame BYTES are deliberately dropped here. Nothing subscribes to
        them yet, and holding them would be the memory cost of a consumer that
        does not exist. What is kept is the row that makes the frame findable
        later, which is the whole reason the recording is worth anything.
        """
        now = time.monotonic()
        ros_ns = self.get_clock().now().nanoseconds
        self.frame_cache.set(True, now)

        pose = self.pose_cache.get(now)
        att = self.att_cache.get(now)
        age = self.pose_cache.age(now)
        with self._lock:
            if self._csv is None:
                return
            row = recorder_core.csv_row(
                self._frame_idx, pts_ns, ros_ns,
                pose=pose, attitude=att,
                gimbal_pitch=(None if math.isnan(self._gimbal_pitch)
                              else self._gimbal_pitch),
                pose_age_s=age)
            self._frame_idx += 1
            if len(self._pending) < MAX_PENDING_ROWS:
                self._pending.append(row)
            else:
                # Dropping the row is better than growing without bound, but it
                # is a real hole in the index and must not be silent.
                self.pipe.frames_dropped += 1

    def _on_jpeg(self, jpeg):
        self.slot.put(jpeg)

    def _on_pipeline_error(self, message):
        self.get_logger().error(str(message))

    # ------------------------------------------------------------------ tick

    def _status_tick(self):
        now = time.monotonic()

        if self.frame_cache.went_stale(now):
            self.get_logger().error(
                "no frame for %.1fs -- video is down. The recording file stays "
                "open; it will resume if the stream comes back."
                % self.frame_cache.timeout_s)

        # Flush the index. Once per tick rather than per frame so the streaming
        # thread never touches the disk.
        with self._lock:
            rows, self._pending = self._pending, []
            csv = self._csv
            session = self._session
            started = self._session_started_at
            frame_idx = self._frame_idx
        if csv is not None and rows:
            try:
                csv.write("\n".join(rows) + "\n")
            except OSError as e:
                self.get_logger().error("index write failed: %s" % e)

        # fps over the interval actually elapsed, not the nominal one.
        total = self.pipe.frames_total
        if self._last_fps_t is not None and now > self._last_fps_t:
            self._fps = (total - self._last_frame_count) / (now - self._last_fps_t)
        self._last_fps_t, self._last_frame_count = now, total

        free_mb = self._free_mb()
        may_record, newly, reason = self.guard.check(free_mb)
        if newly:
            self.get_logger().error(reason)
            self._end_session()

        # Roll the session so a crash costs one segment, not the sortie.
        if (may_record and csv is not None and started is not None
                and recorder_core.should_rotate(
                    started, now, float(self.p["max_session_s"]))):
            self._end_session()
            self._start_session()

        att = self.siyi.attitude()
        if att is None:
            self._gimbal_ok = False
            self._gimbal_pitch = float("nan")
        else:
            self._gimbal_ok = True
            self._gimbal_pitch = att[1]

        msg = CameraStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stream_ok = self.frame_cache.fresh(now)
        msg.fps = float(self._fps)
        msg.frames_total = int(total)
        msg.frames_dropped = int(self.pipe.frames_dropped)
        msg.recording_local = csv is not None
        msg.recording_sd = bool(self._recording_sd)
        msg.session = session
        msg.frame_index = int(frame_idx)
        msg.disk_free_mb = float(free_mb)
        msg.gimbal_pitch = float(self._gimbal_pitch)
        msg.gimbal_ok = bool(self._gimbal_ok)
        self.status_pub.publish(msg)

    def _free_mb(self) -> float:
        try:
            return shutil.disk_usage(self.record_dir).free / (1024.0 * 1024.0)
        except OSError:
            return 0.0

    def _viewer_state(self):
        """Snapshot for the viewer's /state. Called on an HTTP thread."""
        with self._lock:
            session, frame_idx = self._session, self._frame_idx
        return {
            "stream_ok": self.frame_cache.fresh(time.monotonic()),
            "fps": None if math.isnan(self._fps) else round(self._fps, 1),
            "session": session,
            "frame_index": frame_idx,
            "recording_local": session != "",
            "recording_sd": self._recording_sd,
            "gimbal_pitch": (None if math.isnan(self._gimbal_pitch)
                             else round(self._gimbal_pitch, 1)),
            "gimbal_ok": self._gimbal_ok,
            "disk_free_mb": round(self._free_mb()),
        }

    # ---------------------------------------------------------------- teardown

    def destroy_node(self):
        """Deterministic on every exit path: SIGTERM, Ctrl-C, launch shutdown.

        Order matters. The pipeline stops first so EOS reaches the muxer and the
        matroska index lands -- a file whose index never landed may not seek,
        which is discovered later by whoever tries to label it.
        """
        try:
            self.pipe.stop()
        except Exception:
            pass
        self._end_session()
        try:
            self.viewer.stop()
        except Exception:
            pass
        try:
            self.siyi.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(CameraNode, args=args)


if __name__ == "__main__":
    main()
