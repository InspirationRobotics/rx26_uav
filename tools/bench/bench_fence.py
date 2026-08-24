#!/usr/bin/env python3
"""bench_fence — drive the geofence upload dialog against a fake autopilot.

No ROS, no MAVProxy, no Pixhawk. FenceProtocol takes an injected transport, so
the entire mission-protocol exchange can be exercised on a laptop — including
the three failures that are the whole reason the readback exists and that a real
autopilot will not produce on demand.

    python3 tools/bench/bench_fence.py

A net you have never seen catch anything is a net you are guessing about. Each
case below prints PASS only when the driver did the strict thing.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "uav_common"))

from uav_common.fence_core import (  # noqa: E402
    ACK_ACCEPTED, FenceError, FenceProtocol, items_from_polygon,
    polygon_from_flat)

# Exactly as uav_params.yaml carries it: FLAT, because ROS parameters cannot
# nest. Written this way on purpose so the bench exercises the same conversion
# the node does rather than a tidier shape that never reaches production.
FENCE_FLAT = [
    1.28010, 103.85520,
    1.28010, 103.85625,
    1.28110, 103.85625,
    1.28110, 103.85520,
    1.28010, 103.85520,
]
FENCE = polygon_from_flat(FENCE_FLAT)


class FakeAutopilot:
    """Speaks the vehicle half of the FENCE mission protocol.

    `corrupt` injects one of the faults a healthy autopilot never produces:
      "reject"      NACK the upload
      "short"       ACK early, before every item was requested
      "count"       readback claims a different number of items
      "move"        readback shifts one vertex by ~30 m
      "vertexcount" readback returns the right points with the wrong param1
      "silent"      never answers at all
    """

    def __init__(self, corrupt=None):
        self.corrupt = corrupt
        self.stored = {}
        self._out = []
        self._expect = None
        self.acked = False

    # -- transport interface -------------------------------------------------

    def clear(self):
        self._out.clear()

    def send_count(self, n):
        self._expect = n
        self.stored.clear()
        if self.corrupt == "silent":
            return
        if self.corrupt == "reject":
            self._out.append({"type": "MISSION_ACK", "result": 1})
            return
        if self.corrupt == "short":
            # ACK immediately, having requested nothing
            self._out.append({"type": "MISSION_ACK", "result": ACK_ACCEPTED})
            return
        self._out.append({"type": "MISSION_REQUEST", "seq": 0})

    def send_item(self, item):
        self.stored[item.seq] = item
        nxt = item.seq + 1
        if nxt < self._expect:
            self._out.append({"type": "MISSION_REQUEST", "seq": nxt})
        else:
            self._out.append({"type": "MISSION_ACK", "result": ACK_ACCEPTED})

    def send_request_list(self):
        n = len(self.stored)
        if self.corrupt == "count":
            n -= 1
        self._out.append({"type": "MISSION_COUNT", "count": n})

    def send_request(self, seq):
        item = self.stored[seq]
        lat, lon = item.lat, item.lon
        n = item.vertex_count
        if self.corrupt == "move" and seq == 2:
            lat += 0.00027               # ~30 m north; well past the 1 m tol
        if self.corrupt == "vertexcount" and seq == 1:
            n += 1
        self._out.append({"type": "MISSION_ITEM", "seq": seq,
                          "lat": lat, "lon": lon, "param1": float(n)})

    def send_ack(self):
        self.acked = True

    def recv(self, timeout):
        return self._out.pop(0) if self._out else None


def case(name, corrupt, expect_ok):
    ap = FakeAutopilot(corrupt)
    # Short timeout: the "silent" case has to actually time out, and waiting the
    # production 5 s to prove it would make this bench annoying enough to skip.
    proto = FenceProtocol(ap, timeout_s=0.3)
    items = items_from_polygon(FENCE)
    try:
        proto.upload_and_verify(items)
        ok, detail = True, "accepted, %d vertices verified" % len(items)
    except FenceError as e:
        ok, detail = False, str(e).split("\n")[0]

    passed = (ok == expect_ok)
    # A refusal that never reached send_ack is the point: an upload that fails
    # verification must not leave the dialog looking completed.
    if expect_ok and not ap.acked:
        passed, detail = False, "verified but never sent the final ACK"
    print("%-14s %-4s %s" % (name, "PASS" if passed else "FAIL", detail[:96]))
    return passed


def main():
    print("geofence: %d flat values -> %d closed points -> %d uploaded vertices"
          % (len(FENCE_FLAT), len(FENCE), len(items_from_polygon(FENCE))))
    # An odd count is a dropped coordinate, which silently shifts every pair
    # after it; prove it is refused rather than quietly mis-paired.
    try:
        polygon_from_flat(FENCE_FLAT[:-1])
        print("odd flat list  FAIL  accepted a dropped coordinate\n")
    except FenceError as e:
        print("odd flat list  PASS  %s\n" % str(e).split("—")[0].strip()[:62])
    results = [
        case("happy path", None, True),
        case("reject", "reject", False),
        case("premature ack", "short", False),
        case("count wrong", "count", False),
        case("vertex moved", "move", False),
        case("param1 wrong", "vertexcount", False),
        case("no answer", "silent", False),
    ]
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
