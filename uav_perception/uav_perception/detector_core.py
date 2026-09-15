"""detector_core — the parts of the buoy viewer that need no camera and no GPU.

Split out for the same reason every *_core.py in this workspace is: the node
cannot be exercised without a running camera, a CUDA device and a trained model,
and none of those are available on a laptop. What lives here is the frame
splitting and the drawing, both of which are pure and both of which have been
wrong before in ways a unit test would have caught.

NOTHING HERE IMPORTS torch, ultralytics, rclpy OR cv2 AT MODULE SCOPE. The node
does; this file must stay importable anywhere.
"""
import json
import re

# camera_node's per-frame metadata header (uav_camera.mjpeg_server.META_HEADER).
# Matched case-insensitively, as HTTP header names are.
_META_RE = re.compile(rb"(?im)^x-frame-meta:[ \t]*(\{[^\r\n]*\})[ \t]*\r?$")

# Bytes kept while waiting for a frame to START. A part's headers arrive before
# its JPEG, and throwing them away while the image is still on the wire would
# lose the metadata of exactly the frame about to be detected. A header block is
# about a kilobyte; this is room for several.
MAX_PREAMBLE_BYTES = 16384

# JPEG framing markers. A multipart MJPEG stream is a sequence of complete JPEG
# files with HTTP part headers between them, and the only reliable way to find
# a frame boundary is these -- the Content-Length header is optional and the
# boundary string is chosen by the server.
SOI = b"\xff\xd8"   # start of image
EOI = b"\xff\xd9"   # end of image

# Refuse to accumulate forever. A stream that never yields EOI -- a half-open
# socket, a server writing headers and then stalling -- would otherwise grow the
# buffer until the Jetson runs out of memory, and the aircraft would lose the
# recording to a viewer nobody was watching.
MAX_BUFFER_BYTES = 8 * 1024 * 1024


def split_jpegs(buf: bytes):
    """Pull complete JPEG frames out of an MJPEG byte buffer.

    Returns (frames, remainder). `frames` may be empty; `remainder` is what to
    keep and prepend to the next read.

    Scanning for SOI *before* EOI matters: the part headers between frames are
    arbitrary bytes and a naive EOI-first split hands back a "frame" that begins
    with `--boundary\\r\\nContent-Type: ...`, which decodes to None and looks
    exactly like a corrupt camera.
    """
    frames = []
    while True:
        i = buf.find(SOI)
        if i < 0:
            # No frame started yet. Keep only a trailing byte in case a marker
            # straddles the read boundary.
            return frames, buf[-1:] if buf else b""
        j = buf.find(EOI, i + 2)
        if j < 0:
            return frames, buf[i:]
        frames.append(buf[i:j + 2])
        buf = buf[j + 2:]


def parse_meta(preamble: bytes):
    """The X-Frame-Meta object in a part's header block, or None.

    None for absent, for unparseable, and for anything that is not a JSON
    object. A frame without usable metadata is still a picture worth detecting
    on -- it just cannot be placed on a map, and the mapper is told so.
    """
    m = _META_RE.search(preamble)
    if m is None:
        return None
    try:
        meta = json.loads(m.group(1).decode("ascii"))
    except (ValueError, UnicodeDecodeError):
        return None
    return meta if isinstance(meta, dict) else None


def split_parts(buf: bytes):
    """Like split_jpegs, but keeps each frame's metadata: -> (parts, remainder).

    `parts` is a list of (meta_dict_or_None, jpeg_bytes). The header block that
    precedes a JPEG in the multipart stream belongs to THAT JPEG, so the bytes
    between one EOI and the next SOI are parsed before the image they introduce.
    """
    parts = []
    while True:
        i = buf.find(SOI)
        if i < 0:
            # Headers may have arrived with the image still on the wire. Keep
            # them (bounded) rather than the single byte split_jpegs keeps.
            return parts, buf[-MAX_PREAMBLE_BYTES:]
        j = buf.find(EOI, i + 2)
        if j < 0:
            return parts, buf            # preamble and partial image, together
        parts.append((parse_meta(buf[:i]), buf[i:j + 2]))
        buf = buf[j + 2:]


def full_view(box, width, height, margin_px) -> bool:
    """True when no side of the box is within margin_px of the image border.

    A buoy clipped by the frame edge usually has its beacon out of view, so the
    model calls it dark; and its box centre is the centre of the visible part,
    not of the buoy. So a clipped box is shown to the operator and used for
    nothing else.
    """
    x0, y0, x1, y1 = box
    return (x0 >= margin_px and y0 >= margin_px
            and x1 <= width - margin_px and y1 <= height - margin_px)


# BGR, for cv2 drawing. Unknown class names draw white.
_CLASS_BGR = {"red": (60, 60, 255), "green": (80, 230, 80),
              "blue": (255, 140, 30), "dark": (170, 170, 170)}
PARTIAL_BGR = (90, 90, 90)


def class_bgr(class_name):
    return _CLASS_BGR.get((class_name or "").lower(), (255, 255, 255))


def buffer_overflowed(buf: bytes) -> bool:
    """True when a stream has gone quiet mid-frame and the buffer must be reset.

    Separate from split_jpegs so the caller decides what to do about it -- the
    node logs and reconnects, a test just asserts.
    """
    return len(buf) > MAX_BUFFER_BYTES


def should_run(now: float, last: float, period_s: float) -> bool:
    """Rate gate for inference.

    The viewer's job is to show the operator what the model sees, not to run at
    frame rate. Inference shares a GPU with camera_node's hardware decoder, and
    that decoder is on the path that produces the recording -- the artifact the
    flight exists for. So this runs slowly on purpose.
    """
    return period_s <= 0.0 or (now - last) >= period_s


def box_label(conf: float, class_name: str = "") -> str:
    """Text drawn on a box: the model's per-frame class and confidence.

    Per-FRAME. A flashing buoy in its dark second is labelled "dark" here, which
    is correct for this frame and says nothing about the buoy's state -- the
    mapper's legend in the corner is where the state is shown.
    """
    return ("%s %.2f" % (class_name, conf)) if class_name else "%.2f" % conf


def scale_boxes(boxes, from_wh, to_wh):
    """Map boxes from inference resolution back to the display image.

    The viewer draws on the frame it received, which is whatever size the camera
    node's preview branch produced -- NOT the size inference ran at. Drawing
    unscaled boxes puts them in the right place only when those happen to match,
    which they did on the bench and would not have at 1920.
    """
    fw, fh = from_wh
    tw, th = to_wh
    if fw <= 0 or fh <= 0:
        return []
    sx, sy = float(tw) / fw, float(th) / fh
    return [(x0 * sx, y0 * sy, x1 * sx, y1 * sy, c) for x0, y0, x1, y1, c in boxes]
