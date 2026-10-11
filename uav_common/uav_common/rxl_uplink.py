"""rxl_uplink -- what Ekko tells the OCS over the RFD radio, and when.

PURE: no ROS, no MAVLink, no clock (the caller passes `t`). telemetry_bridge does
the I/O -- it reads its caches, calls RxlUplink.tick(), and puts what comes back
on the air through its one TUNNEL sender. Everything that decides WHAT is said or
withheld lives here, where it can be exercised without a radio.

THE WIRE. MAVLink 2 TUNNEL, payload_type = 0x8000 | catalogue id, target_system 0
(a broadcast: measured to cross ArduPilot routing, where a frame addressed to the
OCS's own id crosses only after the Cube has heard that id on the radio port),
payload = the catalogue payload only. Ekko's RXL node is 3. The layouts and the
packers are the OCS's own, copied whole as uav_common/rxl.py; this file adds only
what is ours: turning Ekko's data into those fields. Specs: OCS/docs/
EKKO_RADIO_LINK.md ("The heartbeat", "Uplink") and VEHICLE_SIDE_PROPOSAL.md E1.

TWO MESSAGES

  HEARTBEAT 0x10, 24 B, 2 Hz (ocs_rxl_heartbeat_hz). Built by the SAME rules as
    the WiFi heartbeat -- heartbeat_core.build_heartbeat produces one body and
    this file only packs it -- so the two links cannot tell the OCS different
    things. Where heartbeat_core would send nothing, nothing is sent. Where a
    field is unknown the matching valid flag is CLEARED rather than a value
    invented; the OCS then refuses the heartbeat and Ekko's "I do not know" reaches
    RoboCommand as silence, which is the design: a stand-in guess must not
    contradict it.
  SAFE_PASSAGE 0x11, 21+9n B (n <= 10), ONLY while the current task is
    TASK_SAFE_PASSAGE, from the buoy map: sent at once when the set of buoys or any
    state changes (at most one a second), and refreshed every 5 s.

BUDGET. A TUNNEL frame is 17 B of envelope + payload: 41 B a heartbeat, 82 B/s at
2 Hz; a ten-buoy SAFE_PASSAGE is 128 B, 26 B/s at one per 5 s. About 108 B/s in
all, steady. (Changes are limited to one packet a second.) There is no
per-message rate table in the bridge to register them in.
"""
import math
from collections import namedtuple

from uav_common import rxl
from uav_common.task_core import TASK_SAFE_PASSAGE

PAYLOAD_HEARTBEAT = rxl.payload_type(rxl.HEARTBEAT)            # 0x8010
PAYLOAD_SAFE_PASSAGE = rxl.payload_type(rxl.SAFE_PASSAGE)      # 0x8011

#: The BuoyMap is published at 1 Hz (buoy_mapper map_rate_hz); three missed in a
#: row and it is no longer vouched for. Same figure the detector's legend uses
#: (detector_node.MAP_LEGEND_TIMEOUT_S). NOTE the boat_link path has NO such rule:
#: it keeps broadcasting the last map it was given. This one is for SAFE_PASSAGE.
BUOY_MAP_FRESH_S = 3.0

HEARTBEAT_MAX_HZ = 4.0          # 164 B/s; the proposal's ceiling (E1: 2-4 Hz)
PASSAGE_DEBOUNCE_S = 1.0        # at most one SAFE_PASSAGE a second on a change
PASSAGE_REFRESH_S = 5.0         # ...and a refresh this often regardless

#: The views telemetry_bridge's caches are adapted into, with the attribute names
#: heartbeat_core reads. The bridge keeps plain tuples (its caches pre-date this
#: file); the field order below is the order those tuples are written in.
Pose = namedtuple("Pose", "latitude longitude heading ground_speed "
                          "altitude_amsl altitude_rel climb")
Status = namedtuple("Status", "mode armed system_status")
Attitude = namedtuple("Attitude", "roll pitch yaw rollspeed pitchspeed yawspeed")


def _by_name(table):
    return {name: code for code, name in table.items()}


_STATE_CODE = _by_name(rxl.ROBOT_STATE)
_PHASE_CODE = _by_name(rxl.FLIGHT_PHASE)
_TASK_CODE = _by_name(rxl.RX_TASK)


def _code(table, what, name):
    try:
        return table[name]
    except KeyError:
        raise rxl.RxlError("%s %r has no RXL value" % (what, name)) from None


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


# ----------------------------------------------------------------------------- heartbeat

def heartbeat_payload(hb):
    """(payload, cleared) for the body heartbeat_core.build_heartbeat returned.

    `cleared` names what could not be vouched for -- "position", "attitude",
    "altitude" (their valid flags are clear) and "heading" (sent as the
    unresolved marker) -- and is empty for a heartbeat the OCS will relay.
    Raises rxl.RxlError for a body that cannot be packed at all (a speed that is
    not a number): the caller sends nothing.

    A position of exactly 0,0 is "the EKF has no origin", not a place; its flag is
    cleared, as the OCS's own stand-in treats it. (The WiFi heartbeat still sends
    it -- heartbeat_core passes it through -- so the two links differ there.)
    """
    pos = hb["position"]
    lat, lon = pos["latitude"], pos["longitude"]
    pos_valid = (_finite(lat, lon) and abs(lat) <= 90.0 and abs(lon) <= 180.0
                 and not (lat == 0.0 and lon == 0.0))
    roll, pitch = hb.get("roll_deg"), hb.get("pitch_deg")
    att_valid = _finite(roll, pitch)
    alt = hb["altitude_hae_m"]
    alt_valid = _finite(alt)
    heading = hb["heading_deg"]
    hdg = heading if _finite(heading) else None

    cleared = [name for name, ok in (("position", pos_valid), ("attitude", att_valid),
                                     ("altitude", alt_valid), ("heading", hdg is not None))
               if not ok]
    payload = rxl.pack_heartbeat(
        vehicle=rxl.NODE_UAV,
        robot_state=_code(_STATE_CODE, "robot state", hb["state"]),
        flight_phase=_code(_PHASE_CODE, "flight phase", hb["flight_phase"]),
        pos_valid=pos_valid, att_valid=att_valid, alt_valid=alt_valid,
        lat=lat if pos_valid else 0.0, lon=lon if pos_valid else 0.0,
        spd_mps=hb["spd_mps"], hdg_deg=hdg,
        roll_deg=roll if att_valid else 0.0, pitch_deg=pitch if att_valid else 0.0,
        alt_m=alt if alt_valid else 0.0,
        task=_code(_TASK_CODE, "task", hb["current_task"]),
        phase=rxl.PHASE_NONE)       # no source for HEARTBEAT.phase exists on Ekko
    return payload, cleared


# --------------------------------------------------------------------------- safe passage

#: The mapper's Buoy.label -> RXL BeaconState. The other labels the mapper can
#: produce (SOLID_RED, SOLID_GREEN) are not states the task defines, so they have
#: no value and are dropped, never relabelled. UNKNOWN is "not decided yet", which
#: the OCS refuses to publish: those buoys are left out, not sent as UNKNOWN.
BEACON_BY_LABEL = {"OFF": 1, "FLASHING_RED": 2, "FLASHING_GREEN": 3,
                   "FLASHING_BLUE": 4, "SOLID_BLUE": 5}
ENTRY_LABEL = "FLASHING_BLUE"       # handbook: safe ENTRY
EXIT_LABEL = "SOLID_BLUE"           # handbook: safe EXIT
#: Lower is kept first when more than MAX_BUOYS are mapped: the passage buoys,
#: then the gate (red, green), then the obstacles.
_KEEP_ORDER = {"FLASHING_BLUE": 0, "SOLID_BLUE": 0, "FLASHING_RED": 1,
               "FLASHING_GREEN": 1, "OFF": 2}

Passage = namedtuple("Passage", "entry exit buoys dropped notes")
Passage.__doc__ = """What a SAFE_PASSAGE would say.
entry, exit: (lat, lon) or None.  buoys: ((id, lat, lon, state code), ...) by id,
at most MAX_BUOYS.  dropped: ((id, label, why), ...) -- buoys left out that SHOULD
have been reported.  notes: other things worth a log line (ambiguous entry/exit)."""


def select_passage(buoys, limit=rxl.MAX_BUOYS):
    """The Passage for a buoy map given as [(id, lat, lon, label)].

    Entry is the position of THE buoy labelled FLASHING_BLUE, exit of THE buoy
    labelled SOLID_BLUE. If two buoys claim the same one the field is left
    unknown (0,0 on the wire) and a note says so: choosing between them would be
    a guess about which is real. Over `limit` buoys, the passage buoys, the gate
    and then the obstacles are kept, in that order and by id within each; the rest
    are returned in `dropped`.
    """
    usable, dropped = [], []
    for bid, lat, lon, label in buoys:
        if label == "UNKNOWN":
            continue
        if label not in BEACON_BY_LABEL:
            dropped.append((bid, label, "no RXL beacon state for it"))
        elif not (_finite(lat, lon) and abs(lat) <= 90.0 and abs(lon) <= 180.0):
            dropped.append((bid, label, "no usable position"))
        else:
            usable.append((bid, lat, lon, label))

    notes = []

    def place(label):
        hits = [b for b in usable if b[3] == label]
        if len(hits) == 1:
            return (hits[0][1], hits[0][2])
        if hits:
            notes.append("%d buoys are %s (%s): %s left unknown" % (
                len(hits), label, ", ".join("B%d" % b[0] for b in hits),
                "entry" if label == ENTRY_LABEL else "exit"))
        return None

    entry, exit_ = place(ENTRY_LABEL), place(EXIT_LABEL)
    ranked = sorted(usable, key=lambda b: (_KEEP_ORDER[b[3]], b[0]))
    for bid, _lat, _lon, label in ranked[limit:]:
        dropped.append((bid, label, "over the %d-buoy limit" % limit))
    kept = sorted(ranked[:limit], key=lambda b: b[0])
    return Passage(entry, exit_,
                   tuple((b[0], b[1], b[2], BEACON_BY_LABEL[b[3]]) for b in kept),
                   tuple(dropped), tuple(notes))


def passage_signature(passage):
    """What counts as a CHANGE: which buoys, and what each shows. Positions are
    left out on purpose -- the mapper keeps refining them by centimetres, and a
    packet a second for that would spend the radio on noise. They ride the
    refresh."""
    return tuple((b[0], b[3]) for b in passage.buoys)


def passage_payload(passage, time_boot_ms):
    return rxl.pack_safe_passage(
        time_boot_ms=int(time_boot_ms) & 0xFFFFFFFF, entry=passage.entry,
        exit_=passage.exit, buoys=[(b[1], b[2], b[3]) for b in passage.buoys])


# --------------------------------------------------------------------------- scheduling

class Periodic:
    """True once per period. The schedule is anchored, not 'period after the last
    send', so a 20 Hz tick that is a few ms late does not pull 2.0 Hz down to 1.8."""

    def __init__(self, period_s):
        self.period_s = period_s
        self._next = None

    def due(self, t):
        if self._next is None or t >= self._next:
            self._next = (t if self._next is None else self._next) + self.period_s
            if self._next <= t:                    # fell behind (a stall): re-anchor
                self._next = t + self.period_s
            return True
        return False


class PassageSender:
    """When a SAFE_PASSAGE goes out: on a change, then at most once per
    DEBOUNCE; refreshed every REFRESH. A change that arrives inside the debounce
    waits for it, and then the LATEST passage goes (never a stale one)."""

    def __init__(self, debounce_s=PASSAGE_DEBOUNCE_S, refresh_s=PASSAGE_REFRESH_S):
        self.debounce_s, self.refresh_s = debounce_s, refresh_s
        self._sig = None
        self._sent_t = None

    def reset(self):
        """Nothing to report right now (wrong task, no map). The next report
        is sent at once, not left to wait for a refresh -- but the debounce clock
        stays, so a task that flaps cannot flood the radio."""
        self._sig = None

    def due(self, t, passage):
        """'first', 'changed', 'refresh', or None."""
        if passage is None or not passage.buoys:
            self.reset()
            return None
        sig = passage_signature(passage)
        since = math.inf if self._sent_t is None else t - self._sent_t
        if sig != self._sig and since >= self.debounce_s:
            why = "first" if self._sig is None else "changed"
        elif since >= self.refresh_s:
            why = "refresh"
        else:
            return None
        self._sig, self._sent_t = sig, t
        return why


def _known(point):
    """'known' / 'unknown' for an entry or exit position, for log lines."""
    return "known" if point else "unknown"


def _view(cls, values):
    """A cache tuple as the named view heartbeat_core reads; None stays None."""
    return None if values is None else cls(*values)


class _Say:
    """The log. once() speaks only when a key's text CHANGES: a reason that holds
    for a minute is one line, not 1200. clear() re-arms the key; now() is for
    events, which always speak."""

    def __init__(self, log):
        self._log = log
        self._last = {}

    def once(self, key, level, text):
        if self._last.get(key) != text:
            self._last[key] = text
            self.now(level, text)

    def now(self, level, text):
        if self._log is not None:
            self._log(level, text)

    def state(self, key, level, text):
        """once() while `text` is a string; clear() when it is None -- for a
        condition that comes and goes."""
        if text is None:
            self.clear(key)
        else:
            self.once(key, level, text)

    def clear(self, key):
        self._last.pop(key, None)


class RxlUplink:
    """Everything Ekko's RXL radio uplink decides, one call per bridge tick.

    `build_heartbeat` is heartbeat_core.build_heartbeat, handed in rather than
    imported: heartbeat_core lives in the ground station's package and uav_common
    may not depend on that. `log(level, text)` takes "info" or "warning".
    """

    def __init__(self, *, heartbeat_hz, geoid_separation_m, airborne_alt_m,
                 build_heartbeat, log=None):
        if not 0.0 <= heartbeat_hz <= HEARTBEAT_MAX_HZ:
            raise ValueError("heartbeat_hz %r outside [0, %g]"
                             % (heartbeat_hz, HEARTBEAT_MAX_HZ))
        self._clock = Periodic(1.0 / heartbeat_hz) if heartbeat_hz > 0 else None
        self._geoid, self._airborne_alt = geoid_separation_m, airborne_alt_m
        self._build_heartbeat = build_heartbeat
        self._passage = PassageSender()
        self._say = _Say(log)
        self.sent = {PAYLOAD_HEARTBEAT: 0, PAYLOAD_SAFE_PASSAGE: 0}

    def tick(self, t, *, pose, status, attitude, landed, task, buoy_map):
        """[(payload_type, payload)] due now.

        pose, status, attitude: the bridge's FRESH cache tuples, or None when
        stale or never seen. landed: the autopilot's landed_state int, or None.
        task: the current task token. buoy_map: ([(id, lat, lon, label)],
        stamp_ms) from a FRESH BuoyMap, or None.
        """
        out = []
        if self._clock is not None and self._clock.due(t):
            payload = self._heartbeat(pose, status, attitude, landed, task)
            if payload is not None:
                out.append((PAYLOAD_HEARTBEAT, payload))
        payload = self._safe_passage(t, task, buoy_map)
        if payload is not None:
            out.append((PAYLOAD_SAFE_PASSAGE, payload))
        for ptype, _ in out:
            self.sent[ptype] += 1
        return out

    def _heartbeat(self, pose, status, attitude, landed, task):
        hb, info = self._build_heartbeat(
            pose=_view(Pose, pose), status=_view(Status, status),
            attitude=_view(Attitude, attitude), landed=landed, geoid_separation_m=self._geoid,
            airborne_alt_m=self._airborne_alt, current_task=task)
        if hb is None:
            self._say.once("hb", "warning",
                           "RXL heartbeat not sent: %s -- the OCS will see rising "
                           "silence, which is the truth" % info)
            self._say.clear("hb-flags")
            return None
        self._say.clear("hb")
        self._say.once("hb-phase", "info", "RXL heartbeat flight_phase source: %s" % info)
        try:
            payload, cleared = heartbeat_payload(hb)
        except rxl.RxlError as e:
            self._say.once("hb", "warning", "RXL heartbeat not sent: %s" % e)
            return None
        self._say.state("hb-flags", "warning", cleared and (
            "RXL heartbeat sent WITHOUT vouching for %s -- the OCS will not relay "
            "it, and nothing stands in for it" % ", ".join(cleared)) or None)
        return payload

    def _safe_passage(self, t, task, buoy_map):
        if task != TASK_SAFE_PASSAGE:
            self._passage.reset()
            for key in ("map", "drops", "notes", "empty"):
                self._say.clear(key)
            return None
        self._say.state("map", "warning", buoy_map is None and (
            "task is TASK_SAFE_PASSAGE but there is no fresh buoy map (none yet, or "
            "buoy_mapper stopped publishing): no SAFE_PASSAGE sent") or None)
        if buoy_map is None:
            self._passage.reset()
            return None
        buoys, stamp_ms = buoy_map
        passage = select_passage(buoys)
        self._say.state("drops", "warning", passage.dropped and (
            "SAFE_PASSAGE leaves out: " + "; ".join(
                "B%d %s (%s)" % d for d in passage.dropped)) or None)
        self._say.state("notes", "warning", passage.notes and (
            "SAFE_PASSAGE: " + "; ".join(passage.notes)) or None)
        self._say.state("empty", "info", not passage.buoys and (
            "task is TASK_SAFE_PASSAGE; no buoy has a decided state yet: "
            "no SAFE_PASSAGE sent") or None)
        why = self._passage.due(t, passage)
        if why is None:
            return None
        try:
            payload = passage_payload(passage, stamp_ms)
        except rxl.RxlError as e:
            self._say.once("pack", "warning", "SAFE_PASSAGE not sent: %s" % e)
            return None
        self._say.clear("pack")
        if why != "refresh":
            self._say.now("info", "SAFE_PASSAGE (%s): %d buoys, entry %s, exit %s" % (
                why, len(passage.buoys), _known(passage.entry), _known(passage.exit)))
        return payload


# -------------------------------------------------------------------------- for the log

def describe(payload_type, payload):
    """(name, one line) for a frame THIS file sends, for the Radio tab -- or None
    for any other payload type (boat_link.describe takes those). NEVER RAISES: a
    frame that does not decode is exactly the one worth seeing."""
    cid = rxl.catalogue_id(payload_type)
    if cid not in (rxl.HEARTBEAT, rxl.SAFE_PASSAGE):
        return None
    name = rxl.NAMES[cid]
    try:
        _, m = rxl.decode(payload_type, payload)
    except rxl.RxlError as e:
        return name, "does not decode (%d bytes): %s" % (len(payload), e)
    if cid == rxl.HEARTBEAT:
        where = ("%.6f %.6f" % (m["lat"], m["lon"]) if m["pos_valid"] else "position not vouched for")
        return name, "node %d, %s, %s, %s, %s" % (
            m["vehicle"], rxl.ROBOT_STATE[m["robot_state"]].replace("STATE_", ""),
            rxl.FLIGHT_PHASE[m["flight_phase"]].replace("FLIGHT_PHASE_", ""),
            rxl.RX_TASK.get(m["task"], "task %d" % m["task"]), where)
    states = ", ".join(rxl.BEACON_STATE[b["state"]].replace("BEACON_STATE_", "")
                       for b in m["buoys"])
    return name, "%d buoy(s)%s, entry %s, exit %s" % (
        len(m["buoys"]), " [%s]" % states if states else "",
        _known(m["entry"]), _known(m["exit"]))
