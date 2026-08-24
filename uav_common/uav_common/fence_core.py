"""fence_core — the competition geofence -> an ArduPilot inclusion fence, with
readback verify.

Ported from the ASV's keep-out uploader (rx26_asv @ 8c4ffa5,
api/navigation/fence_core.py), which pushed MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
items. The protocol driver is unchanged in shape; what changed is the item type
and the closing-vertex rule, both explained below.

The fence is pushed over the MAVLink mission protocol (mission_type=FENCE) so
the AUTOPILOT enforces it — the aircraft stays inside the declared box even if
every ROS node on the companion computer dies, which is the only version of that
guarantee worth having.

TWO THINGS THIS GETS RIGHT THAT A NAIVE UPLOADER DOES NOT:

  * A fence the autopilot does not echo back DOES NOT EXIST. Upload without an
    ACCEPTED ack raises, and readback ALWAYS follows upload: the list is
    requested back and compared, count and geometry. "It didn't error" is not
    evidence that a fence is loaded.
  * THE CLOSING VERTEX IS STRIPPED. The geofence is stored closed (first point
    == last) because RoboNation's run declaration requires it that way, and
    their test_server rejects a declaration outright when a UAV's fence has
    fewer than 4 points. ArduPilot closes a polygon IMPLICITLY, so uploading the
    repeated point produces a degenerate zero-length final edge. Two different
    consumers, two different spellings of the same polygon, and exactly one
    place — items_from_polygon — that converts between them.

Every item carries the SAME param1: the total vertex count of the polygon.
ArduPilot uses it to know how many of the following items belong to this
polygon, so a count that disagrees with the number of items sent produces a
fence that is silently the wrong shape.

Required autopilot params — verify them, do NOT set them from code (the same
standing rule the ASV applies to ARMING_*): FENCE_ENABLE=1, FENCE_TYPE with the
polygon bit set, FENCE_ACTION per the flight plan. Enabling a fence is a
deliberate act at a ground station, not a side effect of a web button.

The protocol driver takes an INJECTED transport (send/recv callables), so the
full dialog — rejection, timeout and readback mismatch included — can be driven
against a fake autopilot with no hardware. See tools/bench/bench_fence.py.
"""
import time
from dataclasses import dataclass

from uav_common import geo

MISSION_TYPE_FENCE = 1                       # MAV_MISSION_TYPE_FENCE
CMD_FENCE_POLYGON_VERTEX_INCLUSION = 5001    # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
ACK_ACCEPTED = 0                             # MAV_MISSION_ACCEPTED

#: How far a readback vertex may sit from what we sent before it is a mismatch.
#: The wire carries lat/lon as int32 1e-7 degrees, so the round trip loses about
#: a centimetre; a metre of tolerance is far above that and far below any error
#: that would matter to a fence.
DEFAULT_TOLERANCE_M = 1.0


class FenceError(RuntimeError):
    pass


@dataclass
class FenceItem:
    seq: int
    lat: float
    lon: float
    vertex_count: int      # param1 — identical on every item of one polygon


def items_from_polygon(polygon):
    """[(lat, lon), ...] -> [FenceItem], stripping the closing duplicate.

    Accepts the polygon in the CLOSED form the params file and the OCS
    declaration both use, and returns the OPEN form ArduPilot wants. Raises if
    what is left is not a polygon at all, because a two-point "fence" uploads
    without complaint and encloses nothing.
    """
    pts = [(float(a), float(b)) for a, b in polygon]
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]           # ArduPilot closes it itself; see module header
    if len(pts) < 3:
        raise FenceError(
            "geofence has %d distinct vertices after removing the closing "
            "point; a polygon needs at least 3. Check the `geofence` param in "
            "uav_bringup/config/uav_params.yaml." % len(pts))
    n = len(pts)
    return [FenceItem(seq=i, lat=lat, lon=lon, vertex_count=n)
            for i, (lat, lon) in enumerate(pts)]


class FenceProtocol:
    """Drives the mission protocol over an injected transport.

    transport must provide:
      send_count(n)                 send_item(item: FenceItem)
      send_request_list()           send_request(seq)
      send_ack()                    clear()
      recv(timeout) -> dict with at least {"type": str} or None on timeout
        types used: MISSION_REQUEST {seq}, MISSION_ACK {result},
                    MISSION_COUNT {count},
                    MISSION_ITEM {seq, lat, lon, param1}
    """

    def __init__(self, transport, timeout_s: float = 5.0):
        self.t = transport
        self.timeout_s = timeout_s

    def _recv(self, want_types):
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            msg = self.t.recv(timeout=deadline - time.monotonic())
            if msg is None:
                break
            if msg["type"] in want_types:
                return msg
            # Anything else on the queue is another GCS's mission traffic or a
            # late frame from a previous dialog. Skipping rather than failing is
            # deliberate: the link is shared with MAVProxy's other clients.
        raise FenceError(
            "timeout after %.1fs waiting for %s — is the autopilot connected, "
            "and is mav_source_system distinct from MAVProxy's?"
            % (self.timeout_s, "/".join(sorted(want_types))))

    def upload(self, items):
        self.t.clear()
        self.t.send_count(len(items))
        remaining = {i.seq for i in items}
        while True:
            msg = self._recv({"MISSION_REQUEST", "MISSION_ACK"})
            if msg["type"] == "MISSION_ACK":
                if msg["result"] != ACK_ACCEPTED:
                    raise FenceError(
                        "fence upload rejected: MISSION_ACK result=%s "
                        "(0 = accepted). Check FENCE_TYPE has the polygon bit "
                        "set." % msg["result"])
                if remaining:
                    raise FenceError(
                        "premature ACK with %d items never requested: %s"
                        % (len(remaining), sorted(remaining)))
                return
            seq = msg["seq"]
            match = [i for i in items if i.seq == seq]
            if not match:
                raise FenceError("autopilot requested unknown seq %s" % seq)
            self.t.send_item(match[0])
            remaining.discard(seq)

    def readback_verify(self, items, tolerance_m: float = DEFAULT_TOLERANCE_M):
        """A fence the autopilot does not echo back does not exist."""
        self.t.send_request_list()
        count = self._recv({"MISSION_COUNT"})["count"]
        if count != len(items):
            raise FenceError(
                "readback count %d != uploaded %d — the autopilot holds a "
                "different fence than we just sent" % (count, len(items)))
        by_seq = {i.seq: i for i in items}
        for seq in range(count):
            self.t.send_request(seq)
            msg = self._recv({"MISSION_ITEM"})
            want = by_seq.get(msg["seq"])
            if want is None:
                raise FenceError(
                    "readback returned unexpected seq %s" % msg["seq"])
            # Compare in METRES about the expected point, so the tolerance means
            # something physical rather than being a number of degrees that is
            # 111 km at one latitude and less at another.
            dx, dy = geo.latlon_to_xy(msg["lat"], msg["lon"],
                                      (want.lat, want.lon))
            if abs(dx) > tolerance_m or abs(dy) > tolerance_m:
                raise FenceError(
                    "readback mismatch on seq %s: got (%.7f, %.7f), sent "
                    "(%.7f, %.7f) — %.1fm E, %.1fm N apart"
                    % (msg["seq"], msg["lat"], msg["lon"], want.lat, want.lon,
                       abs(dx), abs(dy)))
            got_n = int(msg.get("param1") or 0)
            if got_n != want.vertex_count:
                raise FenceError(
                    "readback vertex-count mismatch on seq %s: %d != %d. The "
                    "fence would be the wrong shape even though every point is "
                    "right." % (msg["seq"], got_n, want.vertex_count))
        self.t.send_ack()

    def upload_and_verify(self, items):
        self.upload(items)
        self.readback_verify(items)


class MavFenceTransport:
    """pymavlink binding.

    `mission_q` is fed by telemetry_bridge's RX loop with MISSION_* messages —
    this class NEVER owns a connection of its own. That is what keeps the
    single-MAVLink-owner rule true while still letting a second thread run a
    blocking request/response dialog.
    """

    def __init__(self, conn, mission_q, mavlink):
        self.conn = conn
        self.q = mission_q
        self.mav = mavlink

    def clear(self):
        """Drop anything left on the queue from a previous dialog.

        Called before an upload starts. Without it, a MISSION_ACK left over from
        a timed-out attempt is read as the answer to THIS one, and an upload
        that never happened reports success.
        """
        import queue as _q
        while True:
            try:
                self.q.get_nowait()
            except _q.Empty:
                return

    def send_count(self, n):
        self.conn.mav.mission_count_send(
            self.conn.target_system, self.conn.target_component, n,
            MISSION_TYPE_FENCE)

    def send_item(self, item):
        self.conn.mav.mission_item_int_send(
            self.conn.target_system, self.conn.target_component, item.seq,
            self.mav.MAV_FRAME_GLOBAL, CMD_FENCE_POLYGON_VERTEX_INCLUSION,
            0,                              # current
            0,                              # autocontinue
            float(item.vertex_count),       # param1: vertices in THIS polygon
            0, 0, 0,
            int(round(item.lat * 1e7)), int(round(item.lon * 1e7)), 0.0,
            MISSION_TYPE_FENCE)

    def send_request_list(self):
        self.conn.mav.mission_request_list_send(
            self.conn.target_system, self.conn.target_component,
            MISSION_TYPE_FENCE)

    def send_request(self, seq):
        self.conn.mav.mission_request_int_send(
            self.conn.target_system, self.conn.target_component, seq,
            MISSION_TYPE_FENCE)

    def send_ack(self):
        self.conn.mav.mission_ack_send(
            self.conn.target_system, self.conn.target_component,
            ACK_ACCEPTED, MISSION_TYPE_FENCE)

    def recv(self, timeout):
        import queue as _q
        try:
            msg = self.q.get(timeout=max(0.0, timeout))
        except _q.Empty:
            return None
        mtype = msg.get_type()
        if mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            return {"type": "MISSION_REQUEST", "seq": msg.seq}
        if mtype == "MISSION_ACK":
            return {"type": "MISSION_ACK", "result": msg.type}
        if mtype == "MISSION_COUNT":
            return {"type": "MISSION_COUNT", "count": msg.count}
        if mtype in ("MISSION_ITEM", "MISSION_ITEM_INT"):
            # MISSION_ITEM_INT carries 1e-7 degrees; MISSION_ITEM carries floats
            # already in degrees. Reading both the same way is how a readback
            # silently compares 47.0 against 470000000.
            scale = 1e-7 if mtype == "MISSION_ITEM_INT" else 1.0
            return {"type": "MISSION_ITEM", "seq": msg.seq,
                    "lat": msg.x * scale, "lon": msg.y * scale,
                    "param1": msg.param1}
        return {"type": mtype}
