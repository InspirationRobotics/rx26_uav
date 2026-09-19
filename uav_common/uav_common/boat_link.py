"""boat_link -- the bytes Ekko and Crusader exchange over the RFD900ux mesh.

PURE: struct packing and the bookkeeping around it; no ROS, no MAVLink import.
Every radio on the mesh (Ekko, the ground laptop, Crusader, a laptop standing in
for Crusader) packs and unpacks with THIS file, so the ends cannot drift apart.

CARRIED BY MAVLINK `TUNNEL` (128 bytes of vendor payload per message): the mesh
also carries QGC and the autopilots' own telemetry, and Ekko's Pixhawk parses
every byte it hears, so anything we send must be a MAVLink frame. Each frame
costs about 17 bytes of envelope, which is why each packet below carries
everything of its kind at once rather than one fact per packet.

NO FIELD NAMES ON THE AIR: the order of the bytes IS the meaning. The shapes:

  POSITIONS  drone -> all   each buoy's position, ONCE, when its light is first
             decided:  count, then per buoy  slot(1) id(2) lat(4) lon(4).
             Re-sent only if the boat has not acknowledged it (see ACKS).
  LIGHTS     drone -> all   every second: the confirmed gate or exit (two slots,
             0 = none), then ONE BYTE per buoy -- slot in the top 5 bits, light
             in the bottom 3. Ten buoys is 12 bytes. Sent whole each time:
             a lost packet then costs a second, never a wrong light.
  BOAT       boat -> all    every second: lat(4) lon(4) activity(1)
             target slot(1), then a 32-bit mask of the slots whose positions
             the boat HOLDS. That mask is the acknowledgement.

SLOTS, NOT TRACKER IDS, ON THE AIR. buoy_mapper numbers every track it ever
starts, false ones included, so ids are not 1-10 in a real session. Each buoy
gets a radio slot 1-31 the first time its position is sent, and the POSITIONS
record carries its real id once, so every receiver still labels it "B7".
"""
import struct

#: TUNNEL payload types (vendor range, above 32767). 32769 was the old whole-map
#: format; it is not reused, so an old sender cannot be misread as a new one.
PAYLOAD_BOAT = 32768         # boat -> all
PAYLOAD_POSITIONS = 32770    # drone -> all
PAYLOAD_LIGHTS = 32771       # drone -> all
MAX_PAYLOAD = 128

#: Light states, by code. Index IS the wire value; append only, never reorder.
#: Three bits: at most eight.
STATES = ("UNKNOWN", "OFF", "FLASHING_RED", "FLASHING_GREEN", "FLASHING_BLUE",
          "SOLID_BLUE")
CODE = {name: i for i, name in enumerate(STATES)}

#: What the boat says it is doing. Same values as uav_msgs/BoatState.
ACTIVITY = ("unknown", "holding", "circling the entry buoy", "transiting",
            "circling the exit buoy", "done")

MAX_SLOT = 31                          # five bits
_POS = struct.Struct("<BHii")          # slot, id, lat 1e-7, lon 1e-7 = 11 bytes
_BOAT = struct.Struct("<iiBBI")        # lat, lon, activity, target slot, acked = 14
MAX_POSITIONS = (MAX_PAYLOAD - 1) // _POS.size   # 11 to a packet


def pad(payload):
    """A TUNNEL payload field is always 128 bytes; payload_length says how much
    of it is real. (MAVLink 2 trims the trailing zeros off the air.)"""
    body = bytearray(MAX_PAYLOAD)
    body[:len(payload)] = payload
    return bytes(body)


def body(msg):
    """The real bytes of a received TUNNEL, without the padding."""
    return bytes(msg.payload)[:msg.payload_length]


# ------------------------------------------------------------------ packets

def pack_positions(records):
    """[(slot, buoy_id, lat, lon)] -> [payload, ...], MAX_POSITIONS per packet."""
    out = []
    records = list(records)
    for i in range(0, len(records), MAX_POSITIONS):
        chunk = records[i:i + MAX_POSITIONS]
        buf = bytearray([len(chunk)])
        for slot, bid, lat, lon in chunk:
            buf += _POS.pack(int(slot), int(bid) & 0xFFFF,
                             int(round(lat * 1e7)), int(round(lon * 1e7)))
        out.append(bytes(buf))
    return out


def unpack_positions(payload):
    """-> [{"slot", "id", "lat", "lon"}]"""
    raw = bytes(payload)
    n = raw[0] if raw else 0
    out = []
    for i in range(min(n, MAX_POSITIONS)):
        off = 1 + i * _POS.size
        if off + _POS.size > len(raw):
            break
        slot, bid, lat, lon = _POS.unpack_from(raw, off)
        out.append({"slot": slot, "id": bid, "lat": lat / 1e7, "lon": lon / 1e7})
    return out


def pack_lights(lights, confirmed=()):
    """{slot: label} + confirmed slots ([red, green], [exit] or []) -> payload."""
    c = [int(s) for s in list(confirmed)[:2]]
    buf = bytearray(c + [0] * (2 - len(c)))
    for slot in sorted(lights):
        buf.append((int(slot) & MAX_SLOT) << 3 | CODE.get(lights[slot], 0))
    return bytes(buf)


def unpack_lights(payload):
    """-> {"confirmed": [slots], "lights": {slot: label}}"""
    raw = bytes(payload)
    confirmed = [s for s in raw[:2] if s]
    lights = {}
    for b in raw[2:]:
        slot, code = b >> 3, b & 0x07
        if slot:
            lights[slot] = STATES[code] if code < len(STATES) else "UNKNOWN"
    return {"confirmed": confirmed, "lights": lights}


def pack_boat(lat, lon, activity, target_slot=0, acked=()):
    """-> payload. `acked`: the slots whose positions this boat holds."""
    mask = 0
    for s in acked:
        if 1 <= s <= MAX_SLOT:
            mask |= 1 << s
    return _BOAT.pack(int(round(lat * 1e7)), int(round(lon * 1e7)),
                      int(activity) & 0xFF, int(target_slot) & 0xFF, mask)


def unpack_boat(payload):
    lat, lon, activity, target, mask = _BOAT.unpack(bytes(payload)[:_BOAT.size])
    return {"lat": lat / 1e7, "lon": lon / 1e7, "activity": activity,
            "target_slot": target,
            "acked": {s for s in range(1, MAX_SLOT + 1) if mask & (1 << s)}}


# ------------------------------------------------------------------ Ekko's side

class Sender:
    """What Ekko puts on the air, and when. Pure: the caller passes the clock.

    feed(buoys, confirmed_ids, now) with the map as [(id, lat, lon, label)] and
    the ids the search confirms; boat_heard(acked_slots, now) with each boat
    report; then due(now) -> [(payload_type, payload)] to transmit.
    """

    LIGHTS_PERIOD_S = 1.0
    #: How long to wait for the boat to acknowledge a position before sending
    #: it again. Only while a boat is actually being heard: re-sending to nobody
    #: fixes nothing.
    RESEND_S = 3.0
    BOAT_HEARD_S = 5.0

    def __init__(self):
        self.slots = {}            # buoy id -> slot
        self.sent = {}             # slot -> (id, lat, lon, when last sent)
        self.pending = []          # slots whose positions are not sent yet
        self.lights = {}           # slot -> label
        self.confirmed = []        # slots
        self.acked = set()
        self.boat_t = None
        self.lights_t = None
        self.overflow = []         # buoy ids with no slot left

    def slot_of(self, buoy_id):
        return self.slots.get(buoy_id)

    def id_of(self, slot):
        return next((b for b, s in self.slots.items() if s == slot), None)

    def feed(self, buoys, confirmed_ids, now):
        for bid, lat, lon, label in buoys:
            slot = self.slots.get(bid)
            if slot is None:
                if label == "UNKNOWN":
                    continue          # not decided yet: nothing worth a position
                if len(self.slots) >= MAX_SLOT:
                    if bid not in self.overflow:
                        self.overflow.append(bid)
                    continue
                slot = len(self.slots) + 1
                self.slots[bid] = slot
                self.sent[slot] = (bid, lat, lon, None)
                self.pending.append(slot)
            self.lights[slot] = label
        self.confirmed = [self.slots[b] for b in confirmed_ids if b in self.slots][:2]

    def boat_heard(self, acked_slots, now):
        self.acked = set(acked_slots)
        self.boat_t = now

    def boat_listening(self, now):
        return self.boat_t is not None and now - self.boat_t <= self.BOAT_HEARD_S

    def due(self, now):
        out = []
        resend = []
        if self.boat_listening(now):
            resend = [s for s, (_b, _la, _lo, t) in self.sent.items()
                      if t is not None and s not in self.acked
                      and now - t >= self.RESEND_S]
        send = self.pending + [s for s in resend if s not in self.pending]
        if send:
            records = []
            for s in send:
                bid, lat, lon, _t = self.sent[s]
                records.append((s, bid, lat, lon))
                self.sent[s] = (bid, lat, lon, now)
            out += [(PAYLOAD_POSITIONS, p) for p in pack_positions(records)]
            self.pending = []
        if self.lights and (self.lights_t is None
                            or now - self.lights_t >= self.LIGHTS_PERIOD_S):
            self.lights_t = now
            out.append((PAYLOAD_LIGHTS, pack_lights(self.lights, self.confirmed)))
        return out


# ------------------------------------------------------------------ the boat's side

class Receiver:
    """What a boat (or any laptop on the mesh) builds from what it hears."""

    def __init__(self):
        self.positions = {}        # slot -> {"slot","id","lat","lon"}
        self.lights = {}           # slot -> label
        self.confirmed_slots = []

    def hear(self, payload_type, payload):
        """Feed one TUNNEL payload. -> the packet kind it was, or None."""
        if payload_type == PAYLOAD_POSITIONS:
            for rec in unpack_positions(payload):
                self.positions[rec["slot"]] = rec
            return "positions"
        if payload_type == PAYLOAD_LIGHTS:
            rep = unpack_lights(payload)
            self.lights = rep["lights"]
            self.confirmed_slots = rep["confirmed"]
            return "lights"
        return None

    def acked(self):
        """The slots whose positions are held: sent back as the acknowledgement."""
        return set(self.positions)

    def slot_of(self, buoy_id):
        """The slot a buoy id travels as, or 0 when its position is not held."""
        return next((s for s, rec in self.positions.items() if rec["id"] == buoy_id), 0)

    def buoys(self):
        """[{"id", "lat", "lon", "label", "slot"}] for every buoy whose position
        is held. A light heard for a slot with no position yet is not a buoy
        anyone can steer by."""
        return [dict(rec, label=self.lights.get(s, "UNKNOWN"))
                for s, rec in sorted(self.positions.items())]

    def confirmed(self):
        """The confirmed gate or exit as buoy ids (not slots), or []."""
        ids = [self.positions[s]["id"] for s in self.confirmed_slots if s in self.positions]
        return ids if len(ids) == len(self.confirmed_slots) else []
