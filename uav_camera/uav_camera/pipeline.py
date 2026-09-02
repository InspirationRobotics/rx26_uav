"""pipeline — the one RTSP connection to the A8 mini.

WHY THE GRAPH SPLITS WHERE IT DOES. Three consumers want the same stream and
they want it in three different forms:

    rtspsrc ! rtph264depay ! h264parse ! tee name=enc
      enc. ! queue ! matroskamux ! filesink        <- the recording
      enc. ! queue ! avdec_h264 ! videoconvert ! tee name=dec
           dec. ! queue ! appsink                  <- frames for processing
           dec. ! queue ! videorate ! jpegenc      <- the operator's view

The split is AFTER h264parse and BEFORE the decoder on purpose. The file gets
the camera's own encoded bytes -- full quality, and near zero CPU, because
nothing re-encodes. Decoding first and re-encoding to disk would cost most of a
core and lose detail, to produce a worse file. On a Jetson also running a
telemetry gateway that is not a trade worth making, and the footage is the
artifact the flight exists to produce.

The decode branch is switchable off entirely (`want_frames=False`), so a sortie
flown purely to collect training data costs almost nothing.

THIS MODULE OWNS NO POLICY. It does not decide when to record, what to call the
file, or whether the disk is full -- recorder_core decides those and the node
applies them. What lives here is the part that cannot be tested without
GStreamer, kept as small as possible for that reason.

gi is imported inside start(), not at module scope, so this file imports on a
laptop with no GStreamer and the node's own import errors stay legible.
"""
import threading

# Frames are handed to the callback on GStreamer's streaming thread. That thread
# must not be blocked: the queue elements upstream are bounded, and a slow
# callback stalls the pipeline all the way back to the socket. The node's
# callback therefore does the minimum and hands off.
APPSINK_MAX_BUFFERS = 2


class Pipeline:
    """One RTSP connection, its tees, and their sinks.

    Args:
      rtsp_url:     the camera's main stream.
      on_frame:     callable(bytes, width, height, pts_ns) or None. Called on the
                    streaming thread. Keep it short.
      on_jpeg:      callable(bytes) or None. The operator's view.
      on_error:     callable(str). Pipeline-level failures, for the node to log
                    and act on. Never raises out of the bus thread.
      want_frames:  build the decode branch at all.
      preview_fps:  rate for the jpeg branch.
    """

    def __init__(self, rtsp_url, *, on_frame=None, on_jpeg=None, on_error=None,
                 want_frames=True, preview_fps=5, latency_ms=0):
        if not rtsp_url.startswith("rtsp://"):
            # Fail here rather than inside GStreamer, where a typo surfaces as
            # "could not link element" three elements away from the cause.
            raise ValueError(
                "rtsp_url must start with rtsp:// -- got %r" % (rtsp_url,))
        self.rtsp_url = rtsp_url
        self.on_frame = on_frame
        self.on_jpeg = on_jpeg
        self.on_error = on_error or (lambda msg: None)
        self.want_frames = want_frames
        self.preview_fps = int(preview_fps)
        self.latency_ms = int(latency_ms)

        self._gst = None
        self._pipeline = None
        self._loop = None
        self._loop_thread = None
        self._lock = threading.Lock()
        self._recording_to = None

        # Counters are read by the node's status tick from another thread. Plain
        # ints under the GIL are safe to read torn-free here; they are only ever
        # incremented from the streaming thread.
        self.frames_total = 0
        self.frames_dropped = 0

    # ------------------------------------------------------------------ graph

    def describe(self, record_path=None) -> str:
        """The gst-launch string. Pure -- built and asserted without GStreamer.

        Kept as a string rather than assembled element by element because this
        is the form a person can paste into gst-launch-1.0 on the Jetson when
        the node will not start, which is when they most need to bisect it.
        """
        parts = [
            "rtspsrc location=%s latency=%d protocols=tcp name=src"
            % (self.rtsp_url, self.latency_ms),
            "! rtph264depay ! h264parse config-interval=-1 ! tee name=enc",
        ]
        if record_path:
            parts.append(
                "enc. ! queue max-size-buffers=200 leaky=no "
                "! matroskamux ! filesink location=%s sync=false" % record_path)
        if self.want_frames:
            parts.append(
                "enc. ! queue max-size-buffers=8 leaky=downstream "
                "! avdec_h264 ! videoconvert ! video/x-raw,format=BGRx "
                "! tee name=dec")
            parts.append(
                "dec. ! queue max-size-buffers=%d leaky=downstream "
                "! videoconvert ! video/x-raw,format=BGR "
                "! appsink name=frames emit-signals=true max-buffers=%d drop=true"
                % (APPSINK_MAX_BUFFERS, APPSINK_MAX_BUFFERS))
            parts.append(
                "dec. ! queue max-size-buffers=2 leaky=downstream "
                "! videorate ! video/x-raw,framerate=%d/1 "
                "! videoconvert ! jpegenc quality=70 "
                "! appsink name=preview emit-signals=true max-buffers=1 drop=true"
                % self.preview_fps)
        elif not record_path:
            # Nothing would consume the tee. Say so plainly rather than letting
            # GStreamer fail with an unlinked-pad message.
            raise ValueError(
                "pipeline has no sink: want_frames is False and no record_path "
                "was given, so there is nothing to do with the stream")
        return " ".join(parts)

    # ------------------------------------------------------------------ run

    def start(self, record_path=None):
        """Build and run. Raises on anything that makes the graph unbuildable."""
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import GLib, Gst

        Gst.init(None)
        self._gst = Gst
        desc = self.describe(record_path)
        self._pipeline = Gst.parse_launch(desc)
        self._recording_to = record_path

        if self.want_frames:
            if self.on_frame is not None:
                sink = self._pipeline.get_by_name("frames")
                sink.connect("new-sample", self._on_frame_sample)
            if self.on_jpeg is not None:
                sink = self._pipeline.get_by_name("preview")
                sink.connect("new-sample", self._on_jpeg_sample)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::eos", self._on_bus_eos)

        self._pipeline.set_state(Gst.State.PLAYING)
        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(
            target=self._loop.run, daemon=True, name="gst-loop")
        self._loop_thread.start()
        return desc

    def stop(self):
        """Idempotent. Sends EOS first so the muxer can finalise the file.

        Without the EOS the matroska index never lands and the recording may not
        seek -- which is discovered later, by someone trying to label it.
        """
        with self._lock:
            if self._pipeline is not None:
                try:
                    if self._recording_to:
                        self._pipeline.send_event(self._gst.Event.new_eos())
                        bus = self._pipeline.get_bus()
                        bus.timed_pop_filtered(
                            3 * self._gst.SECOND,
                            self._gst.MessageType.EOS | self._gst.MessageType.ERROR)
                    self._pipeline.set_state(self._gst.State.NULL)
                except Exception:
                    pass
                self._pipeline = None
            if self._loop is not None:
                try:
                    self._loop.quit()
                except Exception:
                    pass
                self._loop = None
            if self._loop_thread is not None:
                self._loop_thread.join(timeout=2.0)
                self._loop_thread = None
            self._recording_to = None

    # ------------------------------------------------------------- callbacks

    def _on_frame_sample(self, sink):
        buf, w, h, pts = self._unpack(sink)
        if buf is None:
            return self._gst.FlowReturn.OK
        self.frames_total += 1
        try:
            self.on_frame(buf, w, h, pts)
        except Exception as e:
            # A raising callback must not tear down the pipeline: the recording
            # branch is still writing, and losing the footage because a
            # downstream consumer had a bad frame is the wrong trade.
            self.frames_dropped += 1
            self.on_error("frame callback raised: %s" % e)
        return self._gst.FlowReturn.OK

    def _on_jpeg_sample(self, sink):
        buf, _, _, _ = self._unpack(sink)
        if buf is None:
            return self._gst.FlowReturn.OK
        try:
            self.on_jpeg(buf)
        except Exception as e:
            self.on_error("preview callback raised: %s" % e)
        return self._gst.FlowReturn.OK

    def _unpack(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return None, 0, 0, 0
        buf = sample.get_buffer()
        caps = sample.get_caps()
        w = h = 0
        if caps is not None and caps.get_size() > 0:
            s = caps.get_structure(0)
            ok_w, w = s.get_int("width")
            ok_h, h = s.get_int("height")
            if not (ok_w and ok_h):
                w = h = 0
        ok, info = buf.map(self._gst.MapFlags.READ)
        if not ok:
            return None, 0, 0, 0
        try:
            data = bytes(info.data)
        finally:
            buf.unmap(info)
        pts = int(buf.pts) if buf.pts != self._gst.CLOCK_TIME_NONE else 0
        return data, w, h, pts

    def _on_bus_error(self, _bus, msg):
        err, debug = msg.parse_error()
        self.on_error("gstreamer: %s (%s)" % (err, debug))

    def _on_bus_eos(self, _bus, _msg):
        self.on_error("gstreamer: end of stream")
