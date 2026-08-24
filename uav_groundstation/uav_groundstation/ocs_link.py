"""ocs_link — the aircraft's end of the OCS link. One TCP connection, both
directions.

Telemetry goes up, commands come back down the same socket. No broker, no ROS in
this file: it is transport only, so it can be run and broken on a laptop with no
workspace sourced (see `python3 -m uav_groundstation.ocs_link --help` below).

WHY THERE IS NO PROTOBUF HERE. The OCS re-stamps seq and sent_at and
re-serialises every report before it reaches RoboCommand, so the format we send
was never coupled to the format RoboNation receives. That lets the aircraft send
plain JSON and carry no generated code, no protobuf dependency, and no schema to
regenerate. The OCS converts with protobuf's own ParseDict, which rejects a
misspelled key rather than silently dropping it.

THE FRAMING IS DUPLICATED FROM THE OCS REPO, deliberately.
`rx26_ocs/rx_bridge/framing.py` carries the same header format and names this
file as its counterpart. A ROS package must not import from that repo, so the
twenty lines are copied instead — and IF YOU CHANGE THE HEADER IN ONE PLACE YOU
MUST CHANGE IT IN THE OTHER IN THE SAME COMMIT. A mismatched length prefix is
indistinguishable from a corrupt payload, so the failure is silent garbage
rather than an error.

    4 bytes   big-endian uint32 length N
    N bytes   UTF-8 JSON

TWO THINGS THIS GETS RIGHT THAT THE OBVIOUS TCP CLIENT DOES NOT:

  * `recv()` returning b'' means the far end closed. Treated as "no data yet" —
    which `if data:` does — the client loops forever believing it is connected
    to a socket that is gone, and the aircraft looks healthy while saying
    nothing.
  * Telemetry is a SINGLE LATEST-WINS SLOT, not a queue. A queue that fills
    during a dropped link dumps every stale heartbeat on reconnect: the OCS rate
    governor then discards most of them as a rate problem when it was a link
    problem, and the survivors are re-stamped with the current time, so a
    position from ten seconds ago reaches RoboNation labelled as current. The
    newest fix is the only one worth sending, so it is the only one kept.
"""
from __future__ import annotations

import json
import math
import socket
import struct
import threading
import time
from datetime import datetime, timezone

_HEADER = struct.Struct(">I")
HEADER_LEN = _HEADER.size
MAX_FRAME = 1 << 20

#: How long a read blocks before we go round and check for something to send.
_READ_TIMEOUT_S = 0.05


# ---- framing (mirror of rx26_ocs/rx_bridge/framing.py) ----------------------

def encode(payload: bytes) -> bytes:
    if len(payload) > MAX_FRAME:
        raise ValueError("frame of %d bytes exceeds MAX_FRAME (%d)"
                         % (len(payload), MAX_FRAME))
    return _HEADER.pack(len(payload)) + payload


class FrameReader:
    """Accumulates whatever recv() gives you; hands back whole frames only."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list:
        self._buf.extend(chunk)
        out = []
        while len(self._buf) >= HEADER_LEN:
            (n,) = _HEADER.unpack_from(self._buf, 0)
            if n > MAX_FRAME:
                raise ValueError(
                    "framing desync: length prefix claims %d bytes (max %d)"
                    % (n, MAX_FRAME))
            if len(self._buf) < HEADER_LEN + n:
                break
            out.append(bytes(self._buf[HEADER_LEN:HEADER_LEN + n]))
            del self._buf[:HEADER_LEN + n]
        return out


# ---- JSON that protobuf will accept -----------------------------------------

def json_safe(obj):
    """Replace non-finite floats with the strings protobuf's JSON mapping wants.

    NaN is real data here, not a bug to scrub: telemetry_bridge emits it when
    GPS yaw is unresolved, and the OCS validator exists to catch exactly that
    and refuse the frame. So it must SURVIVE the trip — but `json.dumps` writes
    bare `NaN`, which is not valid JSON and which ParseDict rejects outright
    with "use quoted NaN instead". Send the quoted form and the OCS sees a real
    NaN.
    """
    if isinstance(obj, float):
        if math.isnan(obj):
            return "NaN"
        if math.isinf(obj):
            return "Infinity" if obj > 0 else "-Infinity"
        return obj
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj


def rfc3339(when: float | None = None) -> str:
    """Timestamp in the form protobuf parses into a Timestamp field."""
    dt = datetime.fromtimestamp(when if when is not None else time.time(),
                                tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


# ---- the link ---------------------------------------------------------------

class OcsLink:
    """Keeps one connection to the OCS alive, in the background.

    on_command(dict) is called for every RxCommand that arrives. It runs on the
    link thread, so it must not block — hand the work to ROS and return.
    """

    def __init__(self, host: str, port: int, *, on_command=None, log=print,
                 retry_s: float = 5.0) -> None:
        self.host = host
        self.port = port
        self.on_command = on_command
        self.log = log
        self.retry_s = retry_s

        self._sock: socket.socket | None = None
        self._reader = FrameReader()
        self._pending: dict | None = None      # the latest-wins slot
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sent = 0
        self._received = 0

    # ---- what the node calls -------------------------------------------

    def publish(self, report: dict) -> None:
        """Offer the newest report. Replaces any not yet sent — see the module
        docstring on why this must not be a queue."""
        with self._tx_lock:
            self._pending = report

    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def counts(self) -> tuple:
        return self._sent, self._received

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ocs-link",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.0)
        self._close()

    # ---- the thread ------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._sock is None:
                if not self._connect():
                    self._stop.wait(self.retry_s)
                    continue
            if not self._pump():
                self._close()
                # Straight round the loop: reconnect is attempted on the next
                # pass after the retry wait, not here, so one code path owns it.
        self._close()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=5.0)
        except OSError as exc:
            # Name the address. "connection refused" without it sends people
            # hunting through config files for which box they were even aiming
            # at — and on this fleet the answer is almost always that the OCS
            # laptop is not up yet, which is fine and self-healing.
            self.log("OCS link: cannot reach %s:%d — %s"
                     % (self.host, self.port, exc))
            return False
        sock.settimeout(_READ_TIMEOUT_S)
        self._sock = sock
        self._reader = FrameReader()
        self.log("OCS link: connected to %s:%d" % (self.host, self.port))
        return True

    def _pump(self) -> bool:
        """One read, then one send. False means the link is gone."""
        sock = self._sock
        if sock is None:
            return False

        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            chunk = b""
        except OSError as exc:
            self.log("OCS link: read failed — %s" % exc)
            return False
        else:
            if not chunk:
                # Clean close by the OCS. NOT "nothing to read": treating it as
                # that is how a client sits forever on a dead socket.
                self.log("OCS link: closed by the OCS")
                return False

        if chunk:
            try:
                frames = self._reader.feed(chunk)
            except ValueError as exc:
                self.log("OCS link: %s — reconnecting" % exc)
                return False
            for payload in frames:
                self._received += 1
                self._deliver(payload)

        with self._tx_lock:
            report, self._pending = self._pending, None
        if report is None:
            return True
        try:
            body = json.dumps(json_safe(report), allow_nan=False).encode("utf-8")
        except (TypeError, ValueError) as exc:
            # Ours to fix, not the link's. Drop the frame and keep the link up.
            self.log("OCS link: unserialisable report dropped — %s" % exc)
            return True
        try:
            sock.sendall(encode(body))   # sendall: a short write truncates
        except OSError as exc:
            self.log("OCS link: send failed — %s" % exc)
            return False
        self._sent += 1
        return True

    def _deliver(self, payload: bytes) -> None:
        try:
            cmd = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as exc:
            self.log("OCS link: undecodable command — %s" % exc)
            return
        self.log("OCS link: command %s" % sorted(cmd))
        if self.on_command is not None:
            try:
                self.on_command(cmd)
            except Exception as exc:            # noqa: BLE001
                # A throwing callback must not take the link down with it.
                self.log("OCS link: command handler raised — %s" % exc)

    def _close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass


# ---- standalone, for testing without ROS or an aircraft ---------------------

#: Venue geoid separation, metres: HAE = AMSL + this. Sarasota's is what
#: rx26_ocs/fake_vehicle.py uses; the real course is elsewhere. Only used by the
#: fake report below — the node reads its own param.
_GEOID_SEP_M = -24.6


def fake_report(vehicle_id: str, team_id: str, t0: float, fault: str = "",
                vtype: str = "TYPE_UAV") -> dict:
    """A slow circle with a sinusoidal climb, so every field moves.

    A stream of constants hides exactly the frozen-cache and stale-forwarding
    bugs this is meant to surface.

    The faults are the ones a bench cannot ask hardware for:
      "nan"          heading NaN, as telemetry_bridge emits it when GPS yaw is
                     unresolved (hdg == 65535)
      "unknown_task" current_task left at its proto zero value
      "no_phase"     flight_phase omitted. Legal from a boat, REFUSED from an
                     aircraft — the per-type policy in one flag.
      "mistype"      claim TYPE_USV, the signature of a reporter copy-pasted
                     off Crusader
    """
    theta = (time.time() - t0) * 0.05
    amsl = 42.0 + 3.0 * math.sin(theta * 2.0)
    hb = {
        "state": "STATE_AUTO",
        "position": {"latitude": 1.28060 + 0.0004 * math.sin(theta),
                     "longitude": 103.85570 + 0.0004 * math.cos(theta)},
        "spd_mps": 6.5,
        "heading_deg": (float("nan") if fault == "nan"
                        else math.degrees(theta) % 360.0),
        "roll_deg": 3.0 * math.sin(theta * 7.0),
        "pitch_deg": 1.5 * math.sin(theta * 5.0),
        "altitude_hae_m": amsl + _GEOID_SEP_M,
        "depth_m": 0.0,
        "vehicle_type": "TYPE_USV" if fault == "mistype" else vtype,
    }
    if fault != "unknown_task":
        hb["current_task"] = "TASK_NONE"
    # An aircraft MUST fill flight_phase: the OCS permits no UNKNOWN for
    # TYPE_UAV, because the field is relayed onward for Network Remote ID.
    if vtype == "TYPE_UAV" and fault != "no_phase":
        hb["flight_phase"] = "FLIGHT_PHASE_AIRBORNE"
    return {"team_id": team_id, "vehicle_id": vehicle_id,
            "sent_at": rfc3339(), "heartbeat": hb}


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        prog="ocs_link",
        description="Stream fake telemetry to the OCS. No ROS, no aircraft.")
    ap.add_argument("--host", default="192.168.8.107",
                    help="OCS address on the team subnet. NOT 127.0.0.1 unless "
                         "the OCS is on THIS machine.")
    ap.add_argument("--port", type=int, default=37564)
    ap.add_argument("--vehicle", default="UAV1")
    ap.add_argument("--team", default="ASTA")
    ap.add_argument("--type", dest="vtype", default="TYPE_UAV",
                    choices=["TYPE_UAV", "TYPE_USV", "TYPE_UUV"])
    ap.add_argument("--rate", type=float, default=2.0, help="Hz")
    ap.add_argument("--fault", default="",
                    choices=["", "nan", "unknown_task", "no_phase", "mistype"],
                    help="provoke a fault the OCS validator must refuse")
    args = ap.parse_args(argv)

    link = OcsLink(args.host, args.port)
    link.start()
    t0 = time.time()
    try:
        while True:
            link.publish(fake_report(args.vehicle, args.team, t0, args.fault,
                                     args.vtype))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        sent, received = link.counts
        print("\nsent %d, received %d" % (sent, received))
        link.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
