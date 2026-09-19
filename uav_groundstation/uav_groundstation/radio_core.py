"""radio_core -- the Radio tab's log and statistics, with no ROS in it.

Plain Python, like battery_core and preflight_core, so tools/bench/bench_radio.py
can drive it on a laptop with its own clock.

WHAT IT KEEPS. Every RadioFrame telemetry_bridge publishes -- frames Ekko sends
to other systems, and frames other systems put on the radio -- in a bounded ring
with the same cursor contract as LogBuffer, plus two summaries the page cannot
work out from a partial scroll-back:

  * per system: when it was last heard, and how many frames went each way;
  * the boat link: boat packets arriving per second, against the rate the boat
    is expected to send.

THE BOAT RATE IS AN ESTIMATE, AND SAYS SO. boat_link packets carry no sequence
number, so a lost packet cannot be counted -- only a rate below the expected one
can be seen. BOAT_EXPECTED_HZ is what tools/scripts/fake_crusader.py sends by
default. A real boat sending at another rate makes this number wrong, not the
link.
"""
import threading
import time

#: MAVLink system ids on the mesh (Fleet ICD), plus telemetry_bridge's own id,
#: which is the id Ekko's radio frames go out under.
NAMES = {1: "Ekko", 2: "Crusader", 3: "Graey", 42: "Crusader (link node)",
         200: "Ekko (bridge)", 255: "Ground station"}

#: Boat packets per second the boat is expected to send.
BOAT_EXPECTED_HZ = 1.0
#: How far back the boat rate and the longest silence look, in seconds.
RATE_WINDOW_S = 60.0

TX, RX = 1, 2
_DIR = {TX: "TX", RX: "RX"}


def name_of(sysid):
    """A readable name for a system id; 0 is a broadcast."""
    sysid = int(sysid)
    if sysid == 0:
        return "broadcast"
    return NAMES.get(sysid, "system %d" % sysid)


class RadioLog:
    """A bounded, thread-safe record of the radio.

    Written from a ROS callback and read from HTTP threads, so every access takes
    the lock. Bounded for the reason LogBuffer is: a chatty system on the radio
    must not become the reason the Jetson runs out of memory.
    """

    def __init__(self, capacity=1000, boat_sysid=42, clock=time.monotonic,
                 wall=time.time):
        self.capacity = capacity
        self.boat_sysid = boat_sysid
        self._clock = clock
        self._wall = wall
        self._lock = threading.Lock()
        self._records = []
        self._seq = 0
        self._dropped = 0
        self._systems = {}        # peer sysid -> counts and last times
        self._boat_times = []     # arrival times of boat packets, last RATE_WINDOW_S
        self._boat_since = None   # first boat packet since start or clear
        self._boat_last = None

    def add(self, direction, src, comp, dst, name, payload_type, summary,
            nbytes, stamp=None):
        """Record one frame. The PEER is who it was sent to (TX) or who sent it
        (RX), so a system's sent and heard counts land in one place."""
        now = self._clock()
        peer = int(dst) if direction == TX else int(src)
        with self._lock:
            self._seq += 1
            self._records.append({
                "seq": self._seq,
                "t": stamp if stamp is not None else self._wall(),
                "dir": _DIR.get(direction, "?"),
                "peer": peer,
                "who": name_of(peer),
                "src": int(src), "comp": int(comp), "dst": int(dst),
                "name": name, "ptype": int(payload_type),
                "summary": summary, "bytes": int(nbytes),
            })
            excess = len(self._records) - self.capacity
            if excess > 0:
                del self._records[:excess]
                self._dropped += excess

            s = self._systems.setdefault(
                peer, {"rx": 0, "tx": 0, "heard": None, "sent": None, "last": ""})
            if direction == TX:
                s["tx"] += 1
                s["sent"] = now
                return
            s["rx"] += 1
            s["heard"] = now
            s["last"] = name
            if peer == self.boat_sysid and name == "BOAT":
                if self._boat_since is None:
                    self._boat_since = now
                self._boat_last = now
                self._boat_times.append(now)
                cut = now - RATE_WINDOW_S
                while self._boat_times and self._boat_times[0] < cut:
                    self._boat_times.pop(0)

    def read(self, since_seq=0, limit=300):
        """Records newer than `since_seq`, oldest first, as (records, newest_seq,
        dropped) -- LogBuffer's contract, so the page only asks for what it has
        not seen and can say how much fell off the end."""
        with self._lock:
            out = [r for r in self._records if r["seq"] > since_seq]
            newest, dropped = self._seq, self._dropped
        return out[-limit:], newest, dropped

    def systems(self):
        """Every system the radio has carried frames to or from, by id."""
        now = self._clock()
        with self._lock:
            items = sorted(self._systems.items())
        return [{"sys": k, "name": name_of(k), "rx": v["rx"], "tx": v["tx"],
                 "heard_s": None if v["heard"] is None else now - v["heard"],
                 "sent_s": None if v["sent"] is None else now - v["sent"],
                 "last": v["last"]}
                for k, v in items]

    def boat_link(self):
        """Boat packets per second over the recent window, as an ESTIMATE.

        The window is the SHORTER of RATE_WINDOW_S and the time since the first
        boat packet, so a link heard for ten seconds is not scored against a
        minute it was never up for. The longest silence counts the gap up to
        now, so a boat that has gone quiet shows as quiet.
        """
        if not self.boat_sysid:
            return {"sys": None,
                    "why": "No boat system id: telemetry_bridge's boat_sysid is 0 "
                           "or could not be read, so the boat link is not scored."}
        now = self._clock()
        with self._lock:
            # Prune here too, not only in add(): a boat that has gone silent adds
            # nothing, and its last minute of packets must still age out.
            cut = now - RATE_WINDOW_S
            times = [t for t in self._boat_times if t >= cut]
            since, last = self._boat_since, self._boat_last
        out = {"sys": self.boat_sysid, "name": name_of(self.boat_sysid),
               "expected_hz": BOAT_EXPECTED_HZ, "window_s": RATE_WINDOW_S,
               "heard": len(times), "rate_hz": None, "pct": None,
               "longest_gap_s": None, "heard_s": None}
        if since is None:
            return out
        span = max(1.0, min(RATE_WINDOW_S, now - since))
        out["rate_hz"] = len(times) / span
        out["pct"] = min(100.0, 100.0 * out["rate_hz"] / BOAT_EXPECTED_HZ)
        out["heard_s"] = now - last
        edges = [max(now - RATE_WINDOW_S, since)] + times + [now]
        out["longest_gap_s"] = max(b - a for a, b in zip(edges, edges[1:]))
        return out

    def clear(self):
        """Forget everything. The sequence keeps counting, as LogBuffer's does,
        so a page holding an old cursor never re-reads a cleared record."""
        with self._lock:
            self._records = []
            self._dropped = 0
            self._systems = {}
            self._boat_times = []
            self._boat_since = None
            self._boat_last = None
