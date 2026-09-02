"""mjpeg_server — the operator's view of the camera, on its own port.

No ROS imports. It is handed a frame slot and a snapshot callable and knows
nothing else, which is what lets the whole thing be driven from a laptop with
invented JPEGs — the same arrangement gcs_server uses, for the same reason.

WHY ITS OWN PORT RATHER THAN A PROXY THROUGH :8090. node_registry.NodeSpec
already carries `port` and `stream_path` for exactly this, and tab_source()
exists so the page can tell "process running" from "port actually serving". A
proxy would put megabytes of video through the ground station's single-threaded
snapshot path and make a stalled camera look like a stalled ground station.

WHY LATEST-WINS RATHER THAN A QUEUE. A browser tab on a weak WiFi link consumes
frames slower than the camera produces them. A queue would grow until it ate the
Jetson's memory during exactly the moment the operator most wants the video; a
latest-wins slot makes a slow client skip frames instead, which is what a viewer
wants anyway. ocs_link.publish takes the same position for the same reason.

THIS IS A VIEWER, NOT A CONTROL PATH. It serves GET only. README safety
constraint 6 says WiFi is a convenience and never a control path, and a camera
page that could re-aim a gimbal or start a recording would quietly make that
false.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "uavframe"
# A viewer or two on a field laptop is the whole use case. The cap exists so a
# page left reloading in a broken tab cannot accumulate threads on a flight
# computer that also has to keep a telemetry gateway alive.
MAX_CLIENTS = 4
# How long a client blocks waiting for a frame before sending nothing and
# looping. Short enough that shutdown is prompt, long enough not to spin.
FRAME_WAIT_S = 1.0


class FrameSlot:
    """The newest JPEG, plus a counter so a reader can tell it apart.

    The sequence number is what makes a slow reader skip rather than repeat: it
    asks for "anything newer than N" and gets whatever is current, however many
    frames went past in between.
    """

    __slots__ = ("_cv", "_jpeg", "_seq")

    def __init__(self):
        self._cv = threading.Condition()
        self._jpeg = None
        self._seq = 0

    def put(self, jpeg: bytes):
        with self._cv:
            self._jpeg = jpeg
            self._seq += 1
            self._cv.notify_all()

    def wait_next(self, since_seq: int, timeout: float):
        """-> (seq, jpeg), or (since_seq, None) if nothing newer arrived.

        The `_jpeg is None` half of the test is not redundant with the sequence
        test: before the first frame the counter is 0 and a caller asking for
        "anything at all" passes -1, so the sequence comparison alone says a
        frame is available when none is. That read as an instant 503 on
        /snapshot.jpg for a freshly started server -- a camera that was fine but
        had not yet delivered its first frame looked like a camera that was not
        there.
        """
        with self._cv:
            if self._jpeg is None or self._seq <= since_seq:
                self._cv.wait(timeout)
            if self._jpeg is None or self._seq <= since_seq:
                return since_seq, None
            return self._seq, self._jpeg

    @property
    def seq(self) -> int:
        with self._cv:
            return self._seq


class MjpegServer:
    """Serves /stream.mjpg, /snapshot.jpg and /state.

    Args:
      slot: a FrameSlot the producer writes into.
      snapshot_fn: zero-arg callable -> JSON-serialisable dict of status.
    """

    def __init__(self, slot: FrameSlot, snapshot_fn):
        self.slot = slot
        self.snapshot_fn = snapshot_fn
        self._server = None
        self._clients = 0
        self._lock = threading.Lock()

    # ---- client accounting ----

    def _acquire(self) -> bool:
        with self._lock:
            if self._clients >= MAX_CLIENTS:
                return False
            self._clients += 1
            return True

    def _release(self):
        with self._lock:
            self._clients = max(0, self._clients - 1)

    @property
    def clients(self) -> int:
        with self._lock:
            return self._clients

    def handler(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            # HTTP/1.0 so a client that walks away does not leave a keep-alive
            # socket parked on a flight computer.
            protocol_version = "HTTP/1.0"

            def do_GET(self):
                if self.path.startswith("/stream"):
                    return self._stream()
                if self.path.startswith("/snapshot"):
                    return self._snapshot()
                if self.path == "/state":
                    return self._state()
                self.send_error(404)

            # No do_POST at all: see the module docstring. An endpoint that does
            # not exist cannot be repurposed into a control path later without
            # someone noticing they are adding one.

            def _state(self):
                try:
                    body = json.dumps(outer.snapshot_fn()).encode()
                except Exception as e:
                    body = json.dumps(
                        {"error": "snapshot failed: %s" % e}).encode()
                self._head(200, "application/json", len(body), nocache=True)
                self._write(body)

            def _snapshot(self):
                _, jpeg = outer.slot.wait_next(-1, FRAME_WAIT_S)
                if jpeg is None:
                    return self.send_error(503, "no frame yet")
                self._head(200, "image/jpeg", len(jpeg), nocache=True)
                self._write(jpeg)

            def _stream(self):
                if not outer._acquire():
                    return self.send_error(
                        503, "too many viewers (max %d)" % MAX_CLIENTS)
                try:
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    seq = -1
                    while True:
                        seq, jpeg = outer.slot.wait_next(seq, FRAME_WAIT_S)
                        if jpeg is None:
                            # No new frame. Keep the connection open rather than
                            # closing it: the camera coming back should resume
                            # the existing tab, not require a reload.
                            continue
                        self.wfile.write(
                            b"--%s\r\nContent-Type: image/jpeg\r\n"
                            b"Content-Length: %d\r\n\r\n"
                            % (BOUNDARY.encode(), len(jpeg)))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (ConnectionError, BrokenPipeError, OSError):
                    pass          # tab closed or link dropped; normal
                finally:
                    outer._release()

            def _head(self, code, ctype, length, nocache=False):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(length))
                if nocache:
                    self.send_header("Cache-Control", "no-store")
                self.end_headers()

            def _write(self, body):
                try:
                    self.wfile.write(body)
                except ConnectionError:
                    pass

            def log_message(self, *args):
                pass              # keep the console for ROS logs

        return Handler

    def start(self, port, host="0.0.0.0"):
        """Serve from a daemon thread. daemon_threads so an open viewer tab can
        never hold up node shutdown."""
        self._server = ThreadingHTTPServer((host, port), self.handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        return self

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
