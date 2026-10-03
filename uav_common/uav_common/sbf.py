"""sbf -- the slice of Septentrio Binary Format that Ekko reads: PVTGeodetic.

Pure Python, no ROS, no serial port: bytes go in, decoded blocks come out, so
tools/bench/bench_gnss.py tests it with no receiver attached.

WHY THE GROUND STATION NEEDS THE RECEIVER'S OWN WORD. Ekko's main GNSS is a
Septentrio mosaic in Galileo HAS -- a PPP mode, SBF Mode 10. ArduPilot 4.7's SBF
driver has no fix type for mode 10, so it never says "HAS": after a standalone
start it keeps reporting "3D" (or "DGPS" when SBAS came first) for a 0.1 m fix,
and after an autopilot-only reboot it reports "no fix" for a fix that is fine.
Every PVTGeodetic block carries the mode the receiver is really in.
"""
import math
import struct

SYNC = b"$@"
PVT_GEODETIC = 4007

_HDR = struct.Struct("<HHH")           # CRC, ID, Length (after the 2 sync bytes)
_MAX_BLOCK = 4096                      # nothing Ekko reads is anywhere near this
_DNU_F8 = -2e10                        # "do not use" for the f8 fields


def crc16(data):
    """CRC-CCITT (XModem) over ID..end, as SBF defines it."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


class BlockReader:
    """Feed raw port bytes; get whole, CRC-checked SBF blocks back.

    The port also carries the receiver's ASCII command replies and, on other
    streams, NMEA -- anything that is not a valid block is skipped, and a block
    split across reads is held until the rest arrives.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        """-> [(block_number, revision, block_bytes)] for every block completed."""
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]              # keep a lone '$' that may start a sync
                return out
            if len(self.buf) < i + 8:
                del self.buf[:i]
                return out
            crc, bid, length = _HDR.unpack_from(self.buf, i + 2)
            if length < 8 or length % 4 or length > _MAX_BLOCK:
                del self.buf[:i + 2]           # not a block header: resync after it
                continue
            if len(self.buf) < i + length:
                del self.buf[:i]
                return out
            block = bytes(self.buf[i:i + length])
            del self.buf[:i + length]
            if crc16(block[4:]) != crc:
                self.bad_crc += 1
                continue
            out.append((bid & 0x1FFF, bid >> 13, block))


def pvt_geodetic(block):
    """PVTGeodetic -> {"mode", "error", "satellites", "h_acc_m", "v_acc_m",
    "lat", "lon"}; None when the block is too short to be one.

    mode: the receiver's PVT mode number (Mode bits 0-3): 0 no fix, 1
    standalone, 2 DGNSS, 4 RTK fixed, 5 RTK float, 6 SBAS, 10 PPP (HAS). Fields
    the receiver marks "do not use" come back None -- a blank, never a number.
    """
    if len(block) < 94:
        return None
    mode, error = block[14] & 0x0F, block[15]
    lat, lon = struct.unpack_from("<dd", block, 16)
    nsv = block[74]
    hacc, vacc = struct.unpack_from("<HH", block, 90)
    ok = lat > _DNU_F8 and lon > _DNU_F8
    return {
        "mode": mode, "error": error,
        "satellites": None if nsv == 255 else nsv,
        "h_acc_m": None if hacc == 65535 else hacc / 100.0,
        "v_acc_m": None if vacc == 65535 else vacc / 100.0,
        "lat": math.degrees(lat) if ok else None,
        "lon": math.degrees(lon) if ok else None,
    }


def build_pvt_geodetic(mode, satellites, h_acc_m, v_acc_m, lat=0.0, lon=0.0, tow_ms=0):
    """A valid PVTGeodetic block, for tests and the laptop preview. Blanks
    (None) are written as the receiver's own do-not-use values."""
    body = bytearray(96 - 8)
    struct.pack_into("<IHBB", body, 0, tow_ms, 0, mode, 0)
    struct.pack_into("<dd", body, 8, math.radians(lat), math.radians(lon))
    body[66] = 255 if satellites is None else satellites
    struct.pack_into("<HH", body, 82, 65535 if h_acc_m is None else int(round(h_acc_m * 100)),
                     65535 if v_acc_m is None else int(round(v_acc_m * 100)))
    head = struct.pack("<HH", PVT_GEODETIC | (2 << 13), 96)
    return SYNC + struct.pack("<H", crc16(head + body)) + head + bytes(body)
