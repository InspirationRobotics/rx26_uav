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
IF gimbal_control_enabled IS TRUE. The distinction that makes this defensible is
between an endpoint reachable from a browser (still absent) and a DDS topic on
the vehicle's own graph (present when asked for). Be clear-eyed about the
residual: DDS is not local-only, so on a vehicle whose ROS graph rides the field
WiFi, anything on that network can publish to it.

IT IS TRUE ON THIS AIRFRAME. Fitz is the development platform and the camera is
re-aimed by hand often enough that the surface earns its keep. Turning it off is
one parameter, and losing it costs less than it sounds: nadir is still commanded
at startup, and /uav/camera/set_nadir still works either way.

THE SERVICE IS NOT GATED, AND THAT IS NOT AN OVERSIGHT. /uav/camera/set_nadir
takes no angle. It can only send gimbal_pitch_deg from the params file -- the
same angle __init__ already commands unasked -- so it grants no authority the
node was not already exercising, and it cannot point the camera anywhere wrong.
Gating it would mean the one recovery action that is safe by construction is
also the one you cannot reach after the gimbal has lost nadir.
"""
import math
import os
import queue
import shutil
import threading
import time
from datetime import datetime, timezone

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from uav_msgs.msg import Attitude, CameraStatus, FcuStatus, GlobalPos

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
    "rtsp_codec": dict(read_only=True,
                       description="h264 or h265; MUST match what the "
                                   "camera is serving -- a mismatch "
                                   "shows as white frames, not an error"),
    "rtsp_latency_ms": dict(read_only=True, lo=0, hi=2000,
                            description="rtspsrc jitter buffer; 0 for lowest "
                                        "latency on a wired link"),
    "want_frames": dict(read_only=True,
                        description="build the decode branch at all. False = "
                                    "record-only sortie, near-zero CPU"),
    "preview_fps": dict(read_only=True, lo=1, hi=15,
                        description="operator view rate; not the record rate"),
    "stills_enabled": dict(read_only=True,
                           description="write JPEG stills beside the video, "
                                       "for labelling and training"),
    "stills_hz": dict(read_only=True, lo=0.1, hi=25.0,
                      description="stills per second; NOT the video rate"),
    "stills_quality": dict(read_only=True, lo=50, hi=100,
                           description="JPEG quality for stills, 50-100"),
    "gimbal_renadir_after_s": dict(read_only=True, lo=0.0, hi=600.0,
                                   description="seconds of gimbal_cmd silence "
                                               "before nadir is restored; "
                                               "0 disables the hold"),
    "gimbal_nadir_tol_deg": dict(read_only=True, lo=0.5, hi=30.0,
                                 description="how far off nadir counts as off"),
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
                                               "TRUE on the development "
                                               "airframe; false shuts off "
                                               "arbitrary-angle commands and "
                                               "leaves only startup nadir and "
                                               "the set_nadir service"),
    "record_dir": dict(read_only=True,
                       description="where .mkv and _frames.csv are written"),
    "record_on_start": dict(read_only=True,
                            description="begin a session as soon as frames "
                                        "flow, rather than waiting to be asked"),
    "capture_on_start": dict(read_only=True,
                             description="begin CAPTURE (camera SD recording + "
                                         "JPEG stills) as soon as a session "
                                         "opens. FALSE by default"),
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
    "gimbal_timeout_s": dict(read_only=True, lo=0.5, hi=30.0,
                             description="s without an answer from the gimbal "
                                         "before gimbal_ok goes false and "
                                         "gimbal_pitch goes NaN. Spans several "
                                         "status ticks so one lost datagram is "
                                         "not a dead gimbal"),
    "status_rate_hz": dict(read_only=True, lo=0.2, hi=10.0,
                           description="/uav/camera/status rate. A heartbeat "
                                       "about the pipeline, not per frame"),
}

# Rows accumulated between status ticks before being written. Bounded because a
# stalled disk must not turn into unbounded memory on a flight computer: at
# 25 fps and a 2 Hz tick this holds ~13 rows, so 4096 is four minutes of
# backlog and anything beyond that is a fault, not a hiccup.
MAX_PENDING_ROWS = 4096

# Stills waiting on the writer thread. SMALL ON PURPOSE. The queue exists to
# absorb a disk hiccup, not to buffer a sortie: each entry holds a full decoded
# BGR frame (~6 MB at 1080p), so 8 is ~50 MB of worst-case RAM. A backlog deeper
# than this means the disk cannot keep up with the requested rate, and the right
# answer is to drop stills -- the video and the index are the artifacts that
# must not be lost, and they share this process.
MAX_PENDING_STILLS = 8

# Minimum gap between nadir re-commands. The A8 mini takes a second or two to
# travel 90 degrees, and the status tick runs at 5 Hz: without this the node
# would re-command every tick of the slew and read "not at nadir" the whole way
# there, which looks like a gimbal fighting itself in the log.
RENADIR_MIN_INTERVAL_S = 4.0


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
        # What the OPERATOR wants, as distinct from what is happening right now.
        # Kept separate because sessions ROLL every max_session_s: _end_session
        # stops SD recording, and without a remembered intent the next session
        # would come up with it off. Someone who pressed REC once would get one
        # session of footage and silence after that, with the button still lit.
        self._capture_wanted = bool(p["capture_on_start"])

        # ---- what decides whether a session's files are KEPT
        #
        # Recording always runs. These decide whether it survives.
        #
        # _fcu_seen is deliberately separate from "armed right now". Never
        # having heard from the autopilot at all means telemetry_bridge is not
        # running, which on this aircraft means somebody is at the bench -- the
        # bridge has no systemd unit and is started by hand from the ground
        # station. Treating that as "not flying" is what makes the space saving
        # real; treating it as "unknown, keep everything" would preserve exactly
        # the bench sessions this exists to remove.
        #
        # Latched per session rather than sampled at close: a sortie that armed,
        # flew and disarmed before the session rolled must still be kept, and
        # the state at the closing instant would say disarmed.
        self._fcu_seen = False
        self._session_saw_armed = False
        # The LIVE arm state, as distinct from _session_saw_armed above. That
        # one latches for the whole session and answers "was this a flight";
        # this one tracks and answers "is it flying right now", which is what
        # the stills follow.
        self._armed_now = False
        self._session_saw_capture = bool(self._capture_wanted)
        self._discard_queue = []
        self._last_frame_count = 0
        self._last_fps_t = None
        self._fps = float("nan")
        self._gimbal_pitch = float("nan")
        self._gimbal_pitch_rate = float("nan")
        self._gimbal_ok = False
        # 0.0 means "no gimbal_cmd has ever arrived", which must read as
        # long-ago rather than just-now, so the hold is active from boot.
        self._last_gimbal_cmd_t = 0.0
        self._last_renadir_t = 0.0
        self._renadir_after_s = float(p["gimbal_renadir_after_s"])
        self._nadir_tol_deg = float(p["gimbal_nadir_tol_deg"])
        self._off_nadir_since = None

        self.guard = recorder_core.DiskGuard(float(p["min_free_mb"]))

        # ---- stills. A SECOND, INDEPENDENT record path, and that is the point.
        # Several 0-byte and index-less .mkv files exist in this directory from
        # earlier bench runs: matroska only becomes readable when EOS reaches the
        # muxer, so an unclean stop can cost the whole session. A JPEG is complete
        # the instant it is written. Stills are also what a labelling tool wants,
        # and the filename IS the frame_idx, which is the index CSV's join key --
        # no timestamp matching, no guessing.
        #
        # What they do NOT do: recover colour. The camera encodes H.264 4:2:0
        # before the Jetson ever sees the stream, so chroma is already halved by
        # the time these frames are decoded. Re-encoding at quality 95 preserves
        # what arrived; it cannot restore what the camera discarded. For real
        # colour fidelity use the camera's own 4K SD recording, or fly lower.
        self._stills_q = None
        self._stills_thread = None
        self._stills_dir = None
        self._stills_written = 0
        self._stills_dropped = 0
        self._last_still_t = 0.0
        self._stills_quality = int(p["stills_quality"])
        self._stills_period = 1.0 / max(float(p["stills_hz"]), 1e-6)
        self._cv2 = self._np = None
        if p["stills_enabled"]:
            try:
                import cv2
                import numpy
                self._cv2, self._np = cv2, numpy
            except ImportError as e:
                # Degraded, never fatal -- same posture as the gimbal. Losing the
                # sortie because an optional encoder is missing is the wrong trade.
                self.get_logger().error(
                    "stills_enabled but cv2/numpy are missing (%s) -- stills are "
                    "OFF. Video and the frame index are unaffected." % e)
        if self._cv2 is not None:
            self._stills_q = queue.Queue(maxsize=MAX_PENDING_STILLS)
            self._stills_thread = threading.Thread(
                target=self._stills_writer, daemon=True, name="stills")
            self._stills_thread.start()
            self.get_logger().info(
                "stills: %.1f Hz at quality %d -> <session>_stills/"
                % (float(p["stills_hz"]), self._stills_quality))

        # ---- telemetry in. Same freshness discipline as every other consumer:
        # a stale pose is not a pose.
        self.pose_cache = StreamCache(float(p["pose_timeout_s"]))
        self.att_cache = StreamCache(float(p["attitude_timeout_s"]))
        self.frame_cache = StreamCache(float(p["frame_timeout_s"]))
        # The gimbal gets the same treatment as every other stream, and for the
        # same reason. It is polled rather than subscribed -- siyi_client asks
        # the gimbal on each tick -- but the question a cache answers is
        # identical either way: is what I am holding still true? Without this
        # the node reported gimbal_ok on the mere fact that a call returned,
        # which was true even when the answer was minutes old.
        self.gimbal_cache = StreamCache(float(p["gimbal_timeout_s"]))
        self.create_subscription(GlobalPos, "/uav/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/uav/attitude", self._on_att, 10)
        self.create_subscription(FcuStatus, "/uav/fcu_status", self._on_fcu, 10)

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
                    self.get_logger().error(
                        "no ack for the startup nadir command -- it may still "
                        "have moved; the ack is a UDP reply, not a "
                        "confirmation. The nadir hold will retry if it did not.")
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

        # ---- re-command nadir. ALWAYS available, unlike the topic above.
        #
        # The difference is the whole reason this is a separate entry point:
        # the topic accepts ANY angle, so it is a way to point the camera
        # somewhere wrong and is gated accordingly. This service accepts NO
        # angle. It can only ever send gimbal_pitch_deg from the params file --
        # the same angle __init__ already commands at startup, unasked. It
        # therefore grants no authority the node was not already exercising,
        # and gating it would only mean that the one recovery action that
        # cannot aim the camera wrongly is the one you cannot reach.
        #
        # What it buys: getting nadir BACK after the gimbal has lost it -- a
        # power blip on the camera, a knock while handling the aircraft, a
        # re-centre from UniGCS -- without restarting the node, which would end
        # the recording session to fix a pointing problem.
        self.create_service(SetBool, "/uav/camera/capture", self._on_capture)
        self.create_service(Trigger, "/uav/camera/set_nadir",
                            self._on_set_nadir)

        # ---- pipeline last: everything it calls back into must already exist.
        self.pipe = Pipeline(
            str(p["rtsp_url"]),
            rtsp_codec=str(p["rtsp_codec"]),
            on_frame=self._on_frame,
            on_jpeg=self._on_jpeg,
            on_error=self._on_pipeline_error,
            want_frames=bool(p["want_frames"]),
            preview_fps=int(p["preview_fps"]),
            latency_ms=int(p["rtsp_latency_ms"]),
            next_record_path=self._next_record_path)

        if p["record_on_start"]:
            self._start_session()
        desc = self.pipe.start(self._mkv_path)
        self.get_logger().info(
            "pipeline [%s]: %s" % (self.pipe.rtsp_codec, desc))

        self.create_timer(1.0 / float(p["status_rate_hz"]), self._status_tick)

    # ------------------------------------------------------------ telemetry in

    def _on_pose(self, msg):
        self.pose_cache.set((msg.latitude, msg.longitude, msg.altitude_rel),
                            time.monotonic(), msg.header.stamp)

    def _on_att(self, msg):
        self.att_cache.set((msg.roll, msg.pitch, msg.yaw),
                           time.monotonic(), msg.header.stamp)

    # ----------------------------------------------------------- gimbal in

    def _stills_wanted(self):
        """Whether stills should be running at this instant.

        Two independent reasons -- the aircraft is ARMED, or the operator
        forced them on for a bench session -- resolved in ONE place, so a
        session roll, an arm transition and a button press can never disagree
        about it. Three callers previously each decided for themselves and a
        roll mid-flight would have silently dropped the stills.

        ARMED WINS. Pressing stop while airborne does not stop the stills of
        the sortie in progress; it only stops the 4K SD recording.

        Deliberately NOT gated on p["record_sd"]. That parameter governs the
        camera's own 4K SD recording, which stills do not use and which this
        airframe does not want started on every arm.
        """
        return bool(self._armed_now or self._capture_wanted)

    def _apply_stills_intent(self, want):
        """Start or stop stills on the CURRENTLY OPEN session.

        Reads and mutates _stills_dir under the lock because _on_frame consults
        it on the capture thread; this runs on the executor thread.
        """
        with self._lock:
            stem = self._session
            have = self._stills_dir is not None
        if stem is None or want == have:
            return                      # no session yet, or already as asked
        d = self._open_stills_dir(stem) if want else None
        with self._lock:
            self._stills_dir = d
            # Write the first still on the very next frame rather than waiting
            # out a stills_period that started counting long ago.
            self._last_still_t = 0.0
        self.get_logger().info(
            "stills %s for session %s" % ("ON" if d else "OFF", stem))

    def _on_capture(self, request, response):
        """Start or stop CAPTURE. std_srvs/SetBool.

        Capture means the two things that exist to collect a dataset and cost
        real storage:

          * the camera's own 4K SD recording   ~9 GB/hour
          * the Jetson's JPEG stills           ~0.9 GB/hour at 1080p

        It does NOT gate the .mkv or the frame index. Those are the flight
        record: ~0.7 GB/hour between them, and the index is what makes any of
        this geo-referenceable. They always run, so a sortie is never
        undiagnosable because somebody forgot to press a button.

        THE INTENT IS REMEMBERED, not just applied. Sessions roll every
        max_session_s and _end_session stops SD recording, so a request that
        only changed the current state would give the operator one session of
        footage and then silence, with the button still lit.

        THE UNDERLYING COMMAND IS A TOGGLE, NOT A SET. siyi_client's start and
        stop send the identical frame on this firmware, so sending it while
        already in the wanted state would flip the camera to the opposite of
        what was asked. Hence the early return when nothing needs to change --
        it is the difference between idempotent and actively wrong.
        """
        if not self.p["record_sd"]:
            response.success = False
            response.message = ("record_sd is false in uav_params.yaml; SD "
                                "recording is disabled for this airframe")
            return response
        want = bool(request.data)
        self._capture_wanted = want
        # Latch on the open session too. Pressing the button is the operator
        # saying "this one matters", and it has to save the session already in
        # progress -- not merely the next one -- or the footage they pressed it
        # for is the footage that gets discarded.
        if want:
            self._session_saw_capture = True
        # Stills follow the same intent and take effect ON THE LIVE SESSION.
        # They used to wait for the next session so a stills folder always
        # spanned its whole index; that cost the operator up to max_session_s
        # (600 s) standing at the field after pressing the button, which is a
        # far worse problem than a tidy index. Starting mid-session leaves the
        # folder covering only part of the frame range -- that is expected, not
        # dropped frames: every frame is still in the CSV, and stills are named
        # by frame_idx, so where they begin is exactly where capture was armed.
        self._apply_stills_intent(self._stills_wanted())
        if self._recording_sd == want:
            response.success = True
            response.message = "SD recording already %s" % ("ON" if want else "OFF")
            return response
        ok = self.siyi.start_recording() if want else self.siyi.stop_recording()
        if ok:
            self._recording_sd = want
        response.success = bool(ok)
        response.message = (
            "SD recording %s" % ("ON" if want else "OFF") if ok else
            "no ack for the SD record toggle -- it may still have taken. Watch "
            "recording_sd on /uav/camera/status, which is measured.")
        self.get_logger().info("capture request %s: %s" % (want, response.message))
        return response

    def _on_set_nadir(self, request, response):
        """Re-command the configured nadir angle. std_srvs/Trigger.

        success is whether the gimbal ACKNOWLEDGED the command -- NOT whether
        it has arrived. It has not: the A8 mini takes a few seconds to travel
        its sweep, and this returns straight away rather than blocking the
        executor while it does. Where it actually ended up is gimbal_pitch on
        /uav/camera/status, which is MEASURED, and is the only angle
        geo-projection may use. A service response echoing the commanded angle
        would be a worse answer wearing a more official hat.

        The last measured angle is reported anyway, because the useful question
        when you call this is usually "how far off was it", and that is the
        number that answers it.
        """
        target = self.siyi.nadir_pitch_deg
        ok = self.siyi.set_nadir()
        where = ("unknown -- the gimbal is not answering"
                 if math.isnan(self._gimbal_pitch)
                 else "%+.1f" % self._gimbal_pitch)
        if ok:
            response.message = (
                "commanded nadir (pitch %+.1f). Last measured pitch was %s; "
                "the gimbal takes a few seconds to travel. Watch gimbal_pitch "
                "on /uav/camera/status for where it settles."
                % (target, where))
            self.get_logger().info(response.message)
        else:
            response.message = (
                "no acknowledgement for the nadir command (pitch %+.1f). That "
                "is NOT proof it was ignored -- set_angles reports whether a "
                "reply frame came back over UDP, and the gimbal has been "
                "observed to move on a command whose ack was lost. Last "
                "measured pitch was %s; watch gimbal_pitch on "
                "/uav/camera/status, and the nadir hold will retry."
                % (target, where))
            self.get_logger().error(response.message)
        response.success = bool(ok)
        return response

    def _hold_nadir(self, now):
        """Put the camera back to nadir after anything moves it.

        WHY THIS EXISTS. The gimbal is powered by the AIRCRAFT, not the Jetson.
        A battery swap or an autopilot reboot power-cycles it, and the A8 mini
        comes back at 0 degrees -- pointing at the horizon -- while this node
        keeps running, never restarts, and so never re-sends the startup nadir
        command. Observed on Ekko 2026-09-02: a 48-second hover recorded
        ENTIRELY at the horizon while gimbal_ok was true, the stream was
        healthy, the index was complete, and nothing in any log said the sortie
        was worthless. The pictures are the only place it showed up.

        It also covers a lost acknowledgement. set_angles reports whether a
        reply frame came back over UDP, not whether the gimbal obeyed, so a
        dropped ack looks identical to a refused command. Re-commanding from
        the MEASURED angle makes that distinction stop mattering: if the gimbal
        moved, there is nothing to correct; if it did not, this retries.

        Gated on gimbal_cmd silence rather than switched off when
        gimbal_control_enabled is true: aiming the camera by hand still works,
        the aim just expires. Nadir is the resting state, not a startup event.
        """
        if self._renadir_after_s <= 0.0:
            return                          # hold disabled by config
        if now - self._last_gimbal_cmd_t < self._renadir_after_s:
            return                          # someone is aiming it; leave it
        if math.isnan(self._gimbal_pitch):
            # Angle unknown. Commanding blind would move a gimbal that may be
            # fine, and the stale-gimbal error above already says this loudly.
            self._off_nadir_since = None
            return
        target = self.siyi.nadir_pitch_deg
        if abs(self._gimbal_pitch - target) <= self._nadir_tol_deg:
            if self._off_nadir_since is not None:
                self.get_logger().info(
                    "gimbal back at nadir (%+.1f)" % self._gimbal_pitch)
                self._off_nadir_since = None
            return
        if now - self._last_renadir_t < RENADIR_MIN_INTERVAL_S:
            return                          # still travelling from the last one
        first = self._off_nadir_since is None
        if first:
            self._off_nadir_since = now
        self._last_renadir_t = now
        ok = self.siyi.set_nadir()
        if first:
            # Once per excursion, not per attempt: this fires after a power
            # cycle, and a line per tick would bury the one that matters.
            self.get_logger().warn(
                "gimbal is at %+.1f, not nadir (%+.1f) -- re-commanding. "
                "Usually means the aircraft was power-cycled under a running "
                "camera_node.%s"
                % (self._gimbal_pitch, target,
                   "" if ok else " No ack came back; will retry."))

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
        # Recorded BEFORE the send and regardless of the result: the operator
        # has expressed intent to aim the camera, and the nadir hold must back
        # off for the full window whether or not this particular frame was
        # acknowledged. Otherwise a lost ack lets the hold yank the camera back
        # while someone is still pointing it.
        self._last_gimbal_cmd_t = time.monotonic()
        if self.siyi.set_angles(yaw, pitch):
            self.get_logger().info("gimbal commanded to yaw %+.1f pitch %+.1f"
                                   % (yaw, pitch))
        else:
            # Not an exception: a silent gimbal is degraded, not fatal, and the
            # same rule applies to a command as to the startup nadir.
            self.get_logger().error(
                "no ack for gimbal yaw %+.1f pitch %+.1f -- it may still have "
                "moved; the ack is a UDP reply, not a confirmation. Watch "
                "gimbal_pitch on /uav/camera/status."
                % (yaw, pitch))

    # --------------------------------------------------------------- sessions

    def _open_stills_dir(self, stem):
        """Create (or reuse) the stills folder for `stem`. None if unusable.

        Shared by session start and the live capture toggle, which must agree:
        the operator pressing the button mid-session has to land in exactly the
        same folder the session would have opened for itself.
        """
        if self._stills_q is None:
            return None
        d = os.path.join(self.record_dir, stem + "_stills")
        try:
            os.makedirs(d, exist_ok=True)
        except OSError as e:
            # Degraded, not fatal: the video and index still record.
            self.get_logger().error(
                "cannot create %s: %s -- stills off for this session" % (d, e))
            return None
        return d

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
        stills_dir = None
        # Stills run while ARMED, or while the operator forced them on -- see
        # _stills_wanted. They are no longer tied to the 4K SD recording: this
        # airframe wants 1080p stills on every arm and does not want 4K at all.
        # Still not always-on: at ~0.9 GB/hour they would otherwise fill the
        # disk with pictures of whatever the aircraft was parked over.
        if self._stills_wanted():
            stills_dir = self._open_stills_dir(stem)
        with self._lock:
            self._session = stem
            self._session_started_at = time.monotonic()
            self._frame_idx = 0
            self._pending = []
            self._csv = csv
            self._mkv_path = os.path.join(self.record_dir, stem + ".mkv")
            self._stills_dir = stills_dir
            self._last_still_t = 0.0
        self.guard.reset()
        if self.p["record_sd"] and self._capture_wanted and self.siyi.start_recording():
            self._recording_sd = True
        self.get_logger().info("recording session %s" % stem)
        return True

    def _on_fcu(self, msg):
        """Autopilot heartbeat. The only thing read here is `armed`.

        Latches rather than tracks: once a session has seen the aircraft armed
        it is a flight, and nothing later in that session can make it bench
        footage again.
        """
        self._fcu_seen = True
        armed = bool(msg.armed)
        if armed:
            self._session_saw_armed = True
        # Stills FOLLOW the arm switch, and only on an actual CHANGE of it.
        # Edge-triggered for a reason that matters in flight: a telemetry link
        # that goes quiet produces no messages at all, so silence can never be
        # read as a disarm and stop the stills of the sortie in progress. It
        # also keeps one log line per arm instead of one per heartbeat.
        if armed != self._armed_now:
            self._armed_now = armed
            self._apply_stills_intent(self._stills_wanted())

    def _record_gate(self):
        """Why the open session will be kept, or why it will be discarded.

        Returns (keep, reason). The reason is published and shown to the
        operator, so it is written for someone standing at the field rather
        than for a log reader.
        """
        if self._session_saw_armed:
            return True, "keeping: aircraft armed"
        if self._session_saw_capture:
            return True, "keeping: capture requested"
        if not self._fcu_seen:
            return False, ("will discard: no FCU status -- is telemetry_bridge "
                           "running?")
        return False, "will discard: never armed"

    def _drain_discards(self):
        """Delete the files of sessions that closed as bench footage.

        Deferred by a tick on purpose. _end_session runs BEFORE splitmuxsink is
        told to split, so at that moment the .mkv is still open and being
        written; deleting it there removes a file the muxer then finalises,
        which on some filesystems resurrects a zero-length ghost and on all of
        them loses the error. One tick later the split has completed.
        """
        if not self._discard_queue:
            return
        stems, self._discard_queue = self._discard_queue, []
        for stem in stems:
            freed = 0
            for path in (os.path.join(self.record_dir, stem + ".mkv"),
                         os.path.join(self.record_dir, stem + "_frames.csv"),
                         os.path.join(self.record_dir, stem + "_stills")):
                try:
                    if os.path.isdir(path):
                        for root, _, files in os.walk(path):
                            for f in files:
                                fp = os.path.join(root, f)
                                freed += os.path.getsize(fp)
                        shutil.rmtree(path)
                    elif os.path.exists(path):
                        freed += os.path.getsize(path)
                        os.remove(path)
                except OSError as e:
                    self.get_logger().warning(
                        "could not discard %s: %s" % (path, e))
            self.get_logger().info(
                "discarded bench session %s (%.0f MB)" % (stem, freed / 1e6))

    def _next_record_path(self):
        """Filename for the recording file splitmuxsink is about to open.

        Called on GStreamer's thread, so it takes the lock and returns fast.
        The node is the only thing that knows the session stem, which is why
        the pipeline asks rather than deriving a name of its own -- two counters
        agreeing is a weaker guarantee than one value read twice.
        """
        with self._lock:
            return self._mkv_path

    def _end_session(self):
        # Decide BEFORE the lock clears the latches, and queue rather than
        # delete: at this instant splitmuxsink still has the .mkv open.
        keep, reason = self._record_gate()
        with self._lock:
            csv, stem = self._csv, self._session
            rows, self._pending = self._pending, []
            self._csv = None
            self._session = ""
            self._session_started_at = None
            self._mkv_path = None
            had_stills = self._stills_dir is not None
            self._stills_dir = None
            self._session_saw_armed = False
            # The operator's intent OUTLIVES the session -- a pressed button
            # stays pressed across a roll -- so this re-seeds from it rather
            # than clearing to False.
            self._session_saw_capture = bool(self._capture_wanted)
        if stem and not keep:
            self._discard_queue.append(stem)
            self.get_logger().info("%s -- %s" % (stem, reason))
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
            if had_stills:
                # Counted, because "the stills folder looked thin" is not
                # something anyone can reconstruct after the flight.
                self.get_logger().info(
                    "stills: %d written, %d dropped"
                    % (self._stills_written, self._stills_dropped))

    # -------------------------------------------------------------- callbacks

    def _on_frame(self, buf, w, h, pts_ns):
        """GStreamer streaming thread. Keep this short -- upstream queues are
        bounded and a slow callback stalls the pipeline back to the socket.

        The frame BYTES are handed to the stills writer thread and otherwise
        dropped. Nothing else subscribes to them. NO ENCODING HAPPENS HERE: a
        1080p JPEG costs milliseconds, and milliseconds spent on this thread are
        taken from the RTSP socket, which is shared with the recording branch.
        The queue put is the whole cost, and it never blocks.

        `buf` is safe to hand across threads -- pipeline._unpack copies it out
        of the GStreamer buffer with bytes() before unmapping.
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
            idx = self._frame_idx
            self._frame_idx += 1
            if len(self._pending) < MAX_PENDING_ROWS:
                self._pending.append(row)
            else:
                # Dropping the row is better than growing without bound, but it
                # is a real hole in the index and must not be silent.
                self.pipe.frames_dropped += 1
            # Rate-gated INSIDE the lock so two frames cannot both pass the test,
            # but enqueued outside it: queue.put must never be held under a lock
            # the status tick also wants.
            still_job = None
            if (self._stills_q is not None and self._stills_dir is not None
                    and now - self._last_still_t >= self._stills_period):
                self._last_still_t = now
                still_job = (self._stills_dir, idx, buf, w, h)
        if still_job is not None:
            try:
                self._stills_q.put_nowait(still_job)
            except queue.Full:
                # The disk is behind. Stills are the expendable artifact here --
                # see MAX_PENDING_STILLS.
                self._stills_dropped += 1

    def _stills_writer(self):
        """Own thread. Encodes and writes; never touches node state under a lock.

        Writes <frame_idx>.jpg, zero-padded so a directory listing sorts in
        capture order. The name is the index CSV's frame_idx on purpose: joining
        a still to its pose is a lookup, not a timestamp search.

        Written to a .part file and renamed. os.replace is atomic within a
        filesystem, so a power loss mid-write leaves a stray .part rather than a
        truncated JPEG that a labelling tool will happily load as a grey smear.
        That matters on an aircraft whose power can be cut by landing on it.
        """
        while True:
            job = self._stills_q.get()
            if job is None:                     # sentinel from destroy_node
                return
            stills_dir, idx, buf, w, h = job
            try:
                arr = self._np.frombuffer(buf, dtype=self._np.uint8)
                expected = w * h * 3
                if w <= 0 or h <= 0 or arr.size < expected:
                    self._stills_dropped += 1
                    continue
                if arr.size == expected:
                    img = arr.reshape((h, w, 3))
                else:
                    # GStreamer pads each row up to a 4-byte boundary, so a
                    # width whose stride is not w*3 arrives larger than
                    # w*h*3. Reshaping the flat buffer anyway produces an
                    # image that looks progressively sheared -- recognisable
                    # but useless, and easy to mistake for a camera fault.
                    stride = arr.size // h
                    if stride < w * 3:
                        self._stills_dropped += 1
                        continue
                    img = (arr[:stride * h].reshape((h, stride))[:, :w * 3]
                           .reshape((h, w, 3)))
                ok, enc = self._cv2.imencode(
                    ".jpg", img,
                    [int(self._cv2.IMWRITE_JPEG_QUALITY), self._stills_quality])
                if not ok:
                    self._stills_dropped += 1
                    continue
                path = os.path.join(stills_dir, "%08d.jpg" % idx)
                tmp = path + ".part"
                with open(tmp, "wb") as f:
                    f.write(enc.tobytes())
                os.replace(tmp, path)
                self._stills_written += 1
            except Exception as e:
                self._stills_dropped += 1
                self.get_logger().warn(
                    "still write failed: %s" % e, throttle_duration_sec=10.0)

    def _on_jpeg(self, jpeg):
        self.slot.put(jpeg)

    def _on_pipeline_error(self, message):
        self.get_logger().error(str(message))

    # ------------------------------------------------------------------ tick

    def _status_tick(self):
        now = time.monotonic()

        # First thing in the tick: by now any split requested last tick has
        # completed and the file is closed.
        self._drain_discards()

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
        #
        # split_now() AFTER _start_session(), never before: the new stem has to
        # be on _mkv_path when splitmuxsink asks for it via format-location, or
        # the video keeps the previous session's name and the pair stops
        # matching. That mismatch is the failure this whole rotation fix exists
        # to remove, so the ordering is the point rather than a detail.
        if (may_record and csv is not None and started is not None
                and recorder_core.should_rotate(
                    started, now, float(self.p["max_session_s"]))):
            self._end_session()
            if self._start_session():
                self.pipe.split_now()

        # Ask the gimbal, then read the answer back OUT OF THE CACHE rather
        # than using it directly. The extra hop is what separates a lost
        # datagram from a dead gimbal: a single miss leaves the last good angle
        # standing until gimbal_timeout_s has run out, and a gimbal that has
        # actually stopped answering goes NaN and says so exactly once.
        g = self.siyi.attitude_and_rates()
        if g is not None:
            self.gimbal_cache.set(g, now)
        if self.gimbal_cache.went_stale(now):
            self.get_logger().error(
                "gimbal has not answered for %.1fs. gimbal_pitch is NaN and the "
                "frame index will write blanks for it from here -- those frames "
                "cannot be geo-projected. Video and recording are unaffected; "
                "the camera is still pointing wherever it last was."
                % self.gimbal_cache.timeout_s)
        g = self.gimbal_cache.get(now)
        if g is None:
            self._gimbal_ok = False
            self._gimbal_pitch = float("nan")
            self._gimbal_pitch_rate = float("nan")
        else:
            self._gimbal_ok = True
            self._gimbal_pitch = g[1]
            self._gimbal_pitch_rate = g[4]

        self._hold_nadir(now)

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
        msg.record_gate = self._record_gate()[1] if session else ""
        msg.gimbal_pitch = float(self._gimbal_pitch)
        msg.gimbal_pitch_rate = float(self._gimbal_pitch_rate)
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
            "gimbal_pitch_rate": (None if math.isnan(self._gimbal_pitch_rate)
                                  else round(self._gimbal_pitch_rate, 1)),
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
        # After _end_session, so the counts it logs are final. The sentinel is
        # put with a timeout rather than blocking: a wedged writer must not hold
        # shutdown open, and the thread is a daemon so the process still exits.
        if self._stills_q is not None:
            try:
                self._stills_q.put(None, timeout=2.0)
                if self._stills_thread is not None:
                    self._stills_thread.join(timeout=5.0)
            except Exception:
                pass
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
