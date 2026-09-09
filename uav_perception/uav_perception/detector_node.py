"""detector_node — the operator's view of what the buoy model sees.

    ros2 run uav_perception detector_node

It pulls frames from camera_node's EXISTING viewer stream, runs the trained
YOLO11n buoy detector on them, draws boxes, and serves the annotated result as
its own MJPEG stream. The ground station's Camera tab switches its <img> between
the two.

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

IT WRITES NOTHING TO DISK, AND THAT IS DELIBERATE. The annotated frames exist
only in this stream. Burning boxes into saved imagery would poison the next
training round -- the model would learn that a buoy is a thing with a rectangle
drawn on it, score beautifully on our own data, and detect nothing at the
competition. The stills camera_node writes stay clean 1920x1080.

The useful thing to persist is detections AS DATA -- frame index, box,
confidence, beside _frames.csv. That is the shape mapping needs and it does not
touch a pixel. Not built yet.

CUDA COMES FROM THE IMAGE, NOT FROM HERE. uav:ml carries torch, ultralytics and
a matched CUDA. On uav:latest this node will import-fail at startup, loudly,
which is the correct outcome: the alternative is a viewer that silently runs on
CPU at one frame every few seconds and looks like a broken camera.
"""
import os
import threading
import time
import urllib.request

from rclpy.node import Node

from uav_camera.mjpeg_server import FrameSlot, MjpegServer
from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
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
                     description="inference rate. Deliberately low: this shares "
                                 "a GPU with the decoder that feeds the recording"),
    "imgsz": dict(read_only=True, lo=320, hi=1920,
                  description="inference input size. 1920 is native; smaller "
                              "shrinks the buoy and loses it before it loses speed"),
    "conf": dict(read_only=True, lo=0.05, hi=0.95,
                 description="detection confidence floor"),
    "reconnect_s": dict(read_only=True, lo=0.5, hi=30.0,
                        description="wait before retrying a dropped source"),
}


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
        self._src = str(p["source_url"])
        self._reconnect_s = float(p["reconnect_s"])

        # Counters read by the viewer's state endpoint from another thread.
        # Plain ints under the GIL; only ever written by the worker.
        self._frames_in = 0
        self._infers = 0
        self._last_dets = 0
        self._last_ms = 0.0
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
        self.get_logger().info(
            "model ready in %.1fs, inference at %.1f Hz, imgsz %d"
            % (time.monotonic() - t0, float(p["infer_hz"]), self._imgsz))

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
                frames, buf = core.split_jpegs(buf)
                if not frames:
                    continue
                # Only the NEWEST frame. Falling behind and then working through
                # a backlog would show the operator the past, and the whole
                # point of this view is what the model sees right now.
                self._frames_in += len(frames)
                now = time.monotonic()
                if not core.should_run(now, last, self._period):
                    continue
                last = now
                self._handle(frames[-1])

    def _handle(self, jpeg: bytes):
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
        for b, c in zip(res.boxes.xyxy.cpu().numpy(),
                        res.boxes.conf.cpu().numpy()):
            boxes.append((float(b[0]), float(b[1]), float(b[2]), float(b[3]),
                          float(c)))
        self._last_dets = len(boxes)

        # predict() ran on the image as given, so the boxes are already in its
        # coordinates -- scale_boxes is identity here. It stays in the call
        # because the moment someone resizes before inference for speed, the
        # boxes silently land in the wrong place, and a no-op that documents the
        # assumption is cheaper than the afternoon that costs.
        boxes = core.scale_boxes(boxes, (w, h), (w, h))
        self._draw(img, boxes)

        ok, enc = self._cv2.imencode(".jpg", img,
                                     [int(self._cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            self.slot.put(enc.tobytes())

    def _draw(self, img, boxes):
        cv2 = self._cv2
        for x0, y0, x1, y1, c in boxes:
            p0 = (int(x0), int(y0))
            p1 = (int(x1), int(y1))
            cv2.rectangle(img, p0, p1, (0, 255, 0), 2)
            cv2.putText(img, core.box_label(c), (p0[0], max(14, p0[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                        cv2.LINE_AA)
        # A corner readout, because "no boxes" has two very different causes and
        # the operator cannot tell them apart from an empty frame: the model
        # ran and found nothing, or nothing is running at all.
        cv2.putText(img, "%d det  %.0f ms" % (len(boxes), self._last_ms),
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                    cv2.LINE_AA)

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
