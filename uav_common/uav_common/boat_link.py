"""boat_link -- the bytes Ekko and Crusader exchange over the RFD900.

PURE: struct packing only, no ROS and no MAVLink import. Both ends use THIS
file's shapes, so the two vehicles cannot drift apart silently: a field with a
different width on one side is a field that decodes as garbage on the other.

CARRIED BY MAVLINK `TUNNEL`, which exists for exactly this -- 128 bytes of
vendor payload addressed to one system -- so the radio already plugged into the
Pixhawk's telemetry port carries it with no second link to debug, and the
autopilot routes it out to whoever the target system is.

WHAT CROSSES THE LINK, AND NOTHING ELSE:

  drone -> boat   every buoy: id, where it is, what its light is doing,
                  plus the ids of what the drone has CONFIRMED for the boat:
                  its next gate (red id, green id) or the exit (exit id, 0), or
                  nothing (0, 0). The boat drives only to what is confirmed; the
                  UAV never sends it a course, a waypoint or an instruction.

                  The confirmation is not decoration, and the map cannot stand
                  in for it. A buoy the drone has not reached yet is not in this
                  packet at all, and a light mapped a minute ago says nothing
                  about the light now -- so a boat steering by the map alone
                  drives a passage nobody has just looked at. Confirmed means
                  the aircraft is over it and has seen every light in it in the
                  last few seconds.
  boat -> drone   where the boat is and ONE BYTE for what it is doing, so the
                  aircraft can station over the gate the boat needs NEXT and
                  knows which buoys are already behind it.

IDS ARE THE CONTRACT. A buoy keeps its id for the life of the map, so "B4 went
dark" is unambiguous on the far end. Positions are 1e-7 degrees, the same
integers MAVLink uses everywhere.
"""
import struct

#: TUNNEL payload types. Above 32767 is the vendor-specific range.
PAYLOAD_BOAT = 32768         # boat -> drone
PAYLOAD_BUOYS = 32769        # drone -> boat
MAX_PAYLOAD = 128

#: Light states, by code. Index IS the wire value; append only, never reorder.
STATES = ("UNKNOWN", "OFF", "FLASHING_RED", "FLASHING_GREEN", "FLASHING_BLUE",
          "SOLID_BLUE")
CODE = {name: i for i, name in enumerate(STATES)}

#: What the boat says it is doing. Same values as uav_msgs/BoatState.
ACTIVITY = ("unknown", "holding", "circling the entry buoy", "transiting",
            "circling the exit buoy", "done")

_BUOY = struct.Struct("<BiiB")      # id, lat 1e-7, lon 1e-7, state   = 10 bytes
_BOAT = struct.Struct("<iiBB")      # lat, lon, activity, target buoy = 10 bytes
#: Count, the two confirmed ids, then the buoys. Task 1 has ten; the cap is 12.
_HEADER = 3
MAX_BUOYS = (MAX_PAYLOAD - _HEADER) // _BUOY.size


def pad(payload):
    """A TUNNEL payload field is always 128 bytes; payload_length says how much
    of it is real. Sending an un-padded buffer is a struct error on the wire,
    not an exception here."""
    body = bytearray(MAX_PAYLOAD)
    body[:len(payload)] = payload
    return bytes(body)


def body(msg):
    """The real bytes of a received TUNNEL, without the padding."""
    return bytes(msg.payload)[:msg.payload_length]


def pack_boat(lat, lon, activity, target=0):
    return _BOAT.pack(int(round(lat * 1e7)), int(round(lon * 1e7)),
                      int(activity) & 0xFF, int(target) & 0xFF)


def unpack_boat(payload):
    lat, lon, activity, target = _BOAT.unpack_from(bytes(payload), 0)
    return {"lat": lat / 1e7, "lon": lon / 1e7, "activity": activity,
            "target": target}


def pack_buoys(buoys, confirmed=()):
    """[(id, lat, lon, label)] + the confirmed ids -> one payload. Extras beyond
    MAX_BUOYS are cut.

    Cut rather than split: this is a 1 Hz snapshot of a ten-buoy field, and half
    a map arriving as two packets is a map the far end can assemble wrongly.
    """
    use = list(buoys)[:MAX_BUOYS]
    ids = [int(i) & 0xFF for i in list(confirmed)[:2]]
    out = bytearray([len(use)] + ids + [0] * (2 - len(ids)))
    for bid, lat, lon, label in use:
        out += _BUOY.pack(int(bid) & 0xFF, int(round(lat * 1e7)),
                          int(round(lon * 1e7)), CODE.get(label, 0))
    return bytes(out)


def unpack_buoys(payload):
    """-> {"buoys": [{"id", "lat", "lon", "label"}], "confirmed": [ids]}.

    A dict, not a bare list: every caller then has to look at the confirmation
    it would otherwise not know was there. `confirmed` is [] (nothing), [exit]
    or [red, green]. Buoy ids start at 1, so 0 always means "no id".
    """
    raw = bytes(payload)
    n = raw[0] if raw else 0
    confirmed = [i for i in raw[1:_HEADER] if i]
    out = []
    for i in range(min(n, MAX_BUOYS)):
        bid, lat, lon, state = _BUOY.unpack_from(raw, _HEADER + i * _BUOY.size)
        out.append({"id": bid, "lat": lat / 1e7, "lon": lon / 1e7,
                    "label": STATES[state] if state < len(STATES) else "UNKNOWN"})
    return {"buoys": out, "confirmed": confirmed}
