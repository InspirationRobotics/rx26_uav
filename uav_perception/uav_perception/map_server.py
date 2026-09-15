"""map_server — the buoy map over HTTP: live state, and the files to download.

No ROS imports; handed two callables and nothing else, the same arrangement as
uav_camera.mjpeg_server and uav_groundstation.gcs_server, so a bench can drive it
over real HTTP on a laptop.

    GET /state        the map as JSON
    GET /buoys.csv    }
    GET /buoys.kml    }  the current map, rendered on request, sent as a download
    GET /buoys.plan   }  named <stem>_buoys.<fmt>
    GET /buoys.json   }

GET ONLY. Clearing the map is a ROS service, reached through the ground station,
not a URL: README safety constraint 6 (WiFi is a convenience, never a control
path) holds for a page only while nobody adds a POST to it.

Rendered on request rather than served from the files on disk, so a download is
always the map as it is now, not as it was at the last write.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class MapServer:
    """Args:
      snapshot_fn: zero-arg -> JSON-serialisable dict.
      export_fn:   fmt -> (content_type, text, filename), or None if unknown fmt.
    """

    def __init__(self, snapshot_fn, export_fn):
        self.snapshot_fn = snapshot_fn
        self.export_fn = export_fn
        self._server = None

    def handler(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path == "/state":
                    try:
                        body = json.dumps(outer.snapshot_fn()).encode()
                    except Exception as e:
                        body = json.dumps(
                            {"error": "snapshot failed: %s" % e}).encode()
                    return self._send(body, "application/json")
                if path.startswith("/buoys."):
                    try:
                        got = outer.export_fn(path[len("/buoys."):])
                    except Exception as e:
                        return self.send_error(500, "export failed: %s" % e)
                    if got is None:
                        return self.send_error(404)
                    ctype, text, filename = got
                    return self._send(text.encode("utf-8"), ctype, filename)
                self.send_error(404)

            # No do_POST: see the module docstring.

            def _send(self, body, ctype, filename=None):
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", ctype)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    if filename:
                        self.send_header(
                            "Content-Disposition",
                            'attachment; filename="%s"' % filename)
                    self.end_headers()
                    self.wfile.write(body)
                except ConnectionError:
                    pass

            def log_message(self, *args):
                pass

        return Handler

    def start(self, port, host="0.0.0.0"):
        self._server = ThreadingHTTPServer((host, port), self.handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        return self

    @property
    def port(self):
        return None if self._server is None else self._server.server_address[1]

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
