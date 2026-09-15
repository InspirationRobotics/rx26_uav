"""detector_node — what the buoy model sees, for the operator AND for the mapper.

    ros2 run uav_perception detector_node

It pulls frames from camera_node's EXISTING viewer stream, runs the trained
colour-buoy model on them, and does two things with the result:

  * serves an annotated MJPEG view on its own port (the Camera tab switches to
    it): boxes in the class colour, clipped boxes in grey, and a legend of the
    mapped buoys' decided states in the corner;
  * publishes every frame's boxes on /uav/perception/buoy_detections, WITH the
    pose and gimbal angles camera_node attached to that frame, for
    buoy_mapper_node to place on the map.

WHY IT CONSUMES MJPEG INSTEAD OF OPENING THE CAMERA. uav_camera's package.xml
already settled this:

    "It carries NO detection, deliberately. Perception belongs in a package that
    SUBSCRIBES to this one, so that nothing contends for the camera and a
    detector that crashes or runs the Jetson out of memory cannot stop the
    recording -- which is the artifact a flight exists to produce."

and node_registry names the specific failure: a second GStreamer client fighting
for the A8 mini. So this node holds no RTSP session, no SIYI socket and no
recording writer. Everything it touches is a copy. If it dies, hangs, or fills
the GPU, the .mkv, the frame index and the stills carry on.

WHY THE POSE COMES FROM THE STREAM. camera_node stamps each preview JPEG with the
same values it wrote to that frame's _frames.csv row (X-Frame-Meta). Copying
those onto the detections -- rather than reading /uav/pose when inference
finishes, a few hundred ms later -- places each box with the pose the aircraft
held when the frame arrived, and makes the live map and a replay of the CSV read
identical inputs.

IT WRITES NOTHING TO DISK, AND THAT IS DELIBERATE. The annotated frames exist
only in this stream. Burning boxes into saved imagery would poison the next
training round -- the model would learn that a buoy is a thing with a rectangle
drawn on it, score beautifully on our own data, and detect nothing at the
competition. The stills camera_node writes stay clean 1920x1080. Detections are
persisted AS DATA, by buoy_mapper_node's sightings log.

CUDA COMES FROM THE IMAGE, NOT FROM HERE. uav:ml carries torch, ultralytics and
a matched CUDA. On uav:latest this node will import-fail at startup, loudly,
which is the correct outcome: the alternative is a viewer that silently runs on
CPU at one frame every few seconds and looks like a broken camera.
"""
import os
import threading
import time
import urllib.request

from builtin_interfaces.msg import Time
from rclpy.node import Node

from uav_msgs.msg import BuoyDetection, BuoyDetections, BuoyMap

from uav_camera.mjpeg_server import FrameSlot, MjpegServer
from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
from uav_common.stream_cache import StreamCache
from uav_perception import detector_core as core

PARAM_SPEC = {
    "model_path": dict(read_only=True,
                       description="trained .pt or TensorRT .engine. Built on "
                                   "THIS device -- engines are not portable"),
    "source_url": dict(read_only=True,
                       description="camera_node's MJPEG stream. Consumed, "
                                   "never the camera itself"),
    "mjpeg_port": dict(read_only=True, lo=1024, hi=65535,
                       description="this node's viewer port; MUST differ from "
                                   "camera_node.mjpeg_port and ground_station.port"),
    "bind_host": dict(read_only=True, description="viewer bind address"),
    "infer_hz": dict(read_only=True, lo=0.2, hi=30.0,
                     description="inference rate. Sets how many colour samples "
                                 "the mapper gets per second of watching"),
    "imgsz": dict(read_only=True, lo=320, hi=1920,
                  description="inference input size. 1920 is native; smaller "
                              "shrinks the buoy and loses it before it loses speed"),
    "conf": dict(read_only=True, lo=0.05, hi=0.95,
                 description="detection confidence floor for the VIEW and the "
                             "topic; the mapper applies its own, higher one"),
    "edge_margin_px": dict(read_only=True, lo=0, hi=200,
                           description="a box within this many px of the image "
                                       "border is a clipped buoy: shown grey, "
                                       "never mapped"),
    "reconnect_s": dict(read_only=True, lo=0.5, hi=30.0,
                        description="wait before retrying a dropped source"),
}

# How long the mapper's last BuoyMap stays on the legend. Beyond this the legend
# says the mapper is silent rather than showing states it is no longer vouching for.
MAP_LEGEND_TIMEOUT_S = 3.0


def _stamp_from_ns(ns):
    t = Time()
    t.sec, t.nanosec = divmod(int(ns), 1_000_000_000)
    return t


def _f(meta, key):
    """A metadata number as float, NaN when absent. NaN, not a default: the
    message spells "unknown" that way and the mapper refuses it."""
    v = None if meta is None else meta.get(key)
    return float("nan") if v is None else float(v)


class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector_node")
        p = declare_from_config(self, uav_config.node_params("detector_node"),
                                PARAM_SPEC)
        self.p = p

        self.model_path = str(p["model_path"])
        # Checked here rather than on the first frame. A missing model is a
        # config error, and finding out when someone switches the tab on a
        # flight line is finding out too late.
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(
                "model not found: %s -- train one, or point model_path at the "
                "weights you copied to the aircraft" % self.model_path)

        self._period = 1.0 / float(p["infer_hz"])
        self._imgsz = int(p["imgsz"])
        self._conf = float(p["conf"])
        self._margin = int(p["edge_margin_px"])
        self._src = str(p["source_url"])
        self._reconnect_s = float(p["reconnect_s"])

        # Counters read by the viewer's state endpoint from another thread.
        # Plain ints under the GIL; only ever written by the worker.
        self._frames_in = 0
        self._infers = 0
        self._last_dets = 0
        self._last_ms = 0.0
        self._meta_missing = 0
        self._connected = False
        self._last_err = ""

        # Import cv2 and ultralytics HERE, not at module scope: on an image
        # without them the failure should name this node at startup rather than
        # break `ros2 pkg executables` for the whole workspace.
        import cv2
        from ultralytics import YOLO
        self._cv2 = cv2

        self.get_logger().info("loading %s ..." % self.model_path)
        t0 = time.monotonic()
        self._model = YOLO(self.model_path)
        # Class names come FROM THE WEIGHTS, never from a list written here. The
        # Roboflow export orders them alphabetically, and a hand-written list
        # would silently relabel every buoy the day someone retrains.
        self._names = dict(self._model.names)
        self.get_logger().info(
            "model ready in %.1fs, classes %s, inference at %.1f Hz, imgsz %d"
            % (time.monotonic() - t0, self._names, float(p["infer_hz"]),
               self._imgsz))

        self.det_pub = self.create_publisher(
            BuoyDetections, "/uav/perception/buoy_detections", 10)
        self._map = StreamCache(MAP_LEGEND_TIMEOUT_S)
        self.create_subscription(BuoyMap, "/uav/perception/buoy_map",
                                 self._on_map, 10)

        self.slot = FrameSlot()
        self.viewer = MjpegServer(self.slot, self._viewer_state)
        self.viewer.start(int(p["mjpeg_port"]), str(p["bind_host"]))
        self.get_logger().info(
            "annotated view on :%d -- the Camera tab switches to it"
            % int(p["mjpeg_port"]))

        self._stop = threading.Event()
        self._worker = threading.Thread(
            target=self._run, daemon=True, name="detector")
        self._worker.start()

    def _on_map(self, msg):
        self._map.set(msg, time.monotonic())

    # ------------------------------------------------------------------ loop

    def _run(self):
        """Pull, infer, draw, publish. Reconnects forever."""
        while not self._stop.is_set():
            try:
                self._pump()
            except Exception as e:
                self._connected = False
                self._last_err = str(e)
                self.get_logger().warning(
                    "source %s: %s -- retrying in %.1fs"
                    % (self._src, e, self._reconnect_s))
            self._stop.wait(self._reconnect_s)

    def _pump(self):
        """One connection's worth of frames. Returns when the stream ends."""
        buf = b""
        last = 0.0
        with urllib.request.urlopen(self._src, timeout=10) as r:
            self._connected = True
            self._last_err = ""
            self.get_logger().info("connected to %s" % self._src)
            while not self._stop.is_set():
                chunk = r.read(16384)
                if not chunk:
                    raise IOError("source closed the stream")
                buf += chunk
                if core.buffer_overflowed(buf):
                    raise IOError(
                        "no complete frame in %d bytes -- source is stalled"
                        % len(buf))
                parts, buf = core.split_parts(buf)
                if not parts:
                    continue
                # Only the NEWEST frame. Falling behind and then working through
                # a backlog would show the operator the past, and the whole
                # point of this view is what the model sees right now.
                self._frames_in += len(parts)
                now = time.monotonic()
                if not core.should_run(now, last, self._period):
                    continue
                last = now
                meta, jpeg = parts[-1]
                self._handle(jpeg, meta)

    def _handle(self, jpeg: bytes, meta):
        import numpy as np
        img = self._cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8), self._cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]

        t0 = time.monotonic()
        res = self._model.predict(img, imgsz=self._imgsz, conf=self._conf,
                                  verbose=False)[0]
        self._last_ms = (time.monotonic() - t0) * 1000.0
        self._infers += 1

        boxes = []
        for b, c, k in zip(res.boxes.xyxy.cpu().numpy(),
                           res.boxes.conf.cpu().numpy(),
                           res.boxes.cls.cpu().numpy()):
            box = (float(b[0]), float(b[1]), float(b[2]), float(b[3]))
            cid = int(k)
            boxes.append((box, float(c), cid, self._names.get(cid, str(cid)),
                          core.full_view(box, w, h, self._margin)))
        self._last_dets = len(boxes)

        self._publish(boxes, meta, w, h)
        self._draw(img, boxes)

        ok, enc = self._cv2.imencode(".jpg", img,
                                     [int(self._cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            self.slot.put(enc.tobytes())

    def _publish(self, boxes, meta, w, h):
        """Every inferred frame is published, including frames with no boxes.

        An empty frame is information too: it tells the mapper the camera looked
        and saw nothing, which is different from the detector having stopped.
        """
        msg = BuoyDetections()
        ok = meta is not None and meta.get("ros_time_ns") is not None
        if not ok:
            self._meta_missing += 1
        msg.header.stamp = (_stamp_from_ns(meta["ros_time_ns"]) if ok
                            else self.get_clock().now().to_msg())
        msg.frame_meta_ok = bool(ok)
        msg.session = str((meta or {}).get("session") or "")
        idx = (meta or {}).get("frame_idx")
        msg.frame_idx = int(idx) if idx is not None and idx >= 0 else 0
        msg.image_width, msg.image_height = int(w), int(h)
        msg.latitude = _f(meta, "lat")
        msg.longitude = _f(meta, "lon")
        msg.altitude_rel = _f(meta, "alt_rel")
        msg.roll, msg.pitch, msg.yaw = (_f(meta, "roll"), _f(meta, "pitch"),
                                        _f(meta, "yaw"))
        msg.pose_age_s = _f(meta, "pose_age_s")
        msg.gimbal_pitch = _f(meta, "gimbal_pitch")
        msg.gimbal_yaw = _f(meta, "gimbal_yaw")
        msg.gimbal_yaw_rate = _f(meta, "gimbal_yaw_rate")
        msg.gimbal_age_s = _f(meta, "gimbal_age_s")
        for (x0, y0, x1, y1), conf, cid, name, full in boxes:
            d = BuoyDetection()
            d.class_name, d.class_id, d.confidence = name, cid, conf
            d.x0, d.y0, d.x1, d.y1 = x0, y0, x1, y1
            d.full_view = bool(full)
            msg.detections.append(d)
        self.det_pub.publish(msg)

    def _draw(self, img, boxes):
        cv2 = self._cv2
        for (x0, y0, x1, y1), conf, _cid, name, full in boxes:
            colour = core.class_bgr(name) if full else core.PARTIAL_BGR
            p0, p1 = (int(x0), int(y0)), (int(x1), int(y1))
            cv2.rectangle(img, p0, p1, colour, 2 if full else 1)
            text = core.box_label(conf, name) + ("" if full else " (edge)")
            cv2.putText(img, text, (p0[0], max(14, p0[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)
        # A corner readout, because "no boxes" has two very different causes and
        # the operator cannot tell them apart from an empty frame: the model
        # ran and found nothing, or nothing is running at all.
        cv2.putText(img, "%d det  %.0f ms" % (len(boxes), self._last_ms),
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                    cv2.LINE_AA)
        self._draw_legend(img)

    def _draw_legend(self, img):
        """The mapper's decided state per buoy, top-left, under the readout.

        This is where FLASHING vs SOLID vs OFF is shown -- never on a box. A box
        is one frame, and one frame cannot tell those apart.
        """
        cv2 = self._cv2
        m = self._map.get(time.monotonic())
        y = 54
        if m is None:
            cv2.putText(img, "map: buoy_mapper silent", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2,
                        cv2.LINE_AA)
            return
        for b in sorted(m.buoys, key=lambda b: b.id)[:12]:
            text = "B%d %s  %.1fs  +/-%.1fm%s" % (
                b.id, b.label, b.observed_s, b.spread_m,
                "" if b.locked else "  watching")
            colour = core.class_bgr(b.colour.lower()) if b.colour else (
                (40, 40, 40) if b.state == "OFF" else (200, 200, 200))
            cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        colour, 2, cv2.LINE_AA)
            y += 24

    # ----------------------------------------------------------------- state

    def _viewer_state(self):
        return {
            "source": self._src,
            "connected": self._connected,
            "frames_in": self._frames_in,
            "inferences": self._infers,
            "detections": self._last_dets,
            "infer_ms": round(self._last_ms, 1),
            "model": os.path.basename(self.model_path),
            "classes": self._names,
            # Frames published without camera_node's pose metadata. Every one is
            # a frame the mapper cannot use; this should stay near zero.
            "frames_without_meta": self._meta_missing,
            "error": self._last_err,
        }

    def destroy_node(self):
        self._stop.set()
        try:
            self.viewer.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(DetectorNode, args=args)
