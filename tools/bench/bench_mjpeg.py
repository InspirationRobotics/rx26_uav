#!/usr/bin/env python3
"""bench_mjpeg — drive the camera viewer over real HTTP, with no camera.

Spins the REAL MjpegServer on an ephemeral port against a FrameSlot fed with
invented JPEG bytes, then talks to it with urllib. No ROS, no GStreamer, no
A8 mini.

    python3 tools/bench/bench_mjpeg.py

Two of these cases are the point of the file:

  * POST must not work. The viewer is not a control path (README safety
    constraint 6), and "we never wrote a handler" is only true until someone
    does. This asserts the absence.
  * A slow reader must skip frames, not accumulate them. The failure that
    matters is not a dropped frame, it is a queue growing on a flight computer
    while the operator watches video.
"""
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "uav_camera"))

from uav_camera.mjpeg_server import (  # noqa: E402
    BOUNDARY, MAX_CLIENTS, FrameSlot, MjpegServer)

# Not a real JPEG; nothing in the server parses it, and inventing a decodable
# image here would test PIL rather than the server.
def jpeg(n):
    return b"\xff\xd8" + bytes([n % 256]) * 32 + b"\xff\xd9"


STATE = {"stream_ok": True, "fps": 24.9, "session": "20260901T142530Z"}


def check(name, passed, detail=""):
    print("%-28s %-4s %s" % (name, "PASS" if passed else "FAIL", detail[:66]))
    return passed


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def get(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.headers, r.read()


# ---------------------------------------------------------------- slot

def case_slot_latest_wins():
    """Ten frames produced, one reader: it must get the newest, not the oldest."""
    slot = FrameSlot()
    for i in range(10):
        slot.put(jpeg(i))
    seq, got = slot.wait_next(-1, 0.1)
    return check("slot hands out the newest",
                 got == jpeg(9) and seq == 10, "seq=%d" % seq)


def case_slot_no_repeat():
    slot = FrameSlot()
    slot.put(jpeg(1))
    seq, _ = slot.wait_next(-1, 0.1)
    seq2, got2 = slot.wait_next(seq, 0.05)
    return check("no new frame -> None", got2 is None and seq2 == seq)


def case_slot_blocks_then_wakes():
    slot = FrameSlot()
    out = {}

    def reader():
        out["seq"], out["jpeg"] = slot.wait_next(-1, 2.0)

    t = threading.Thread(target=reader)
    t.start()
    time.sleep(0.05)
    slot.put(jpeg(7))
    t.join(timeout=2.0)
    return check("waiter wakes on put", out.get("jpeg") == jpeg(7))


# ---------------------------------------------------------------- http

def case_state(base):
    st, hdr, body = get(base + "/state")
    d = json.loads(body)
    ok = (st == 200 and d["session"] == STATE["session"]
          and hdr.get("Cache-Control") == "no-store")
    return check("GET /state", ok, body[:44].decode())


def case_snapshot(base, slot):
    slot.put(jpeg(3))
    st, hdr, body = get(base + "/snapshot.jpg")
    ok = st == 200 and body == jpeg(3) and hdr["Content-Type"] == "image/jpeg"
    return check("GET /snapshot.jpg", ok, "%d bytes" % len(body))


def case_404(base):
    try:
        get(base + "/nope")
    except urllib.error.HTTPError as e:
        return check("unknown path -> 404", e.code == 404, "%d" % e.code)
    return check("unknown path -> 404", False, "served something")


def case_post_refused(base):
    """The viewer is not a control path. Assert POST has no handler at all."""
    req = urllib.request.Request(base + "/stream.mjpg", data=b"{}",
                                 method="POST")
    try:
        urllib.request.urlopen(req, timeout=3.0)
    except urllib.error.HTTPError as e:
        return check("POST refused", e.code in (404, 501), "%d" % e.code)
    except urllib.error.URLError as e:
        return check("POST refused", True, str(e)[:40])
    return check("POST refused", False, "server accepted a POST")


def case_stream(base, slot):
    """Open the stream, push frames, confirm multipart parts arrive."""
    stop = threading.Event()

    def producer():
        i = 0
        while not stop.is_set():
            slot.put(jpeg(i))
            i += 1
            time.sleep(0.01)

    t = threading.Thread(target=producer, daemon=True)
    t.start()
    try:
        r = urllib.request.urlopen(base + "/stream.mjpg", timeout=3.0)
        ctype = r.headers.get("Content-Type", "")
        chunk = r.read(2048)
        r.close()
    finally:
        stop.set()
        t.join(timeout=1.0)
    ok = (BOUNDARY in ctype and "multipart/x-mixed-replace" in ctype
          and b"--" + BOUNDARY.encode() in chunk
          and b"Content-Type: image/jpeg" in chunk)
    return check("GET /stream.mjpg", ok, ctype[:52])


def case_client_cap(base, slot):
    """More viewers than the cap must be refused, not queued."""
    slot.put(jpeg(1))
    opened = []
    refused = 0
    try:
        for _ in range(MAX_CLIENTS + 2):
            try:
                opened.append(urllib.request.urlopen(base + "/stream.mjpg",
                                                     timeout=3.0))
            except urllib.error.HTTPError as e:
                if e.code == 503:
                    refused += 1
    finally:
        for r in opened:
            try:
                r.close()
            except Exception:
                pass
    return check("viewer cap enforced", refused >= 1 and len(opened) <= MAX_CLIENTS,
                 "%d open, %d refused" % (len(opened), refused))


def main():
    slot = FrameSlot()
    srv = MjpegServer(slot, lambda: STATE)
    port = free_port()
    srv.start(port, "127.0.0.1")
    base = "http://127.0.0.1:%d" % port
    print("serving on %s\n" % base)
    try:
        results = [
            case_slot_latest_wins(),
            case_slot_no_repeat(),
            case_slot_blocks_then_wakes(),
            case_state(base),
            case_snapshot(base, slot),
            case_404(base),
            case_post_refused(base),
            case_stream(base, slot),
            case_client_cap(base, slot),
        ]
    finally:
        srv.stop()
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
