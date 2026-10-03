#!/usr/bin/env python3
"""bench_gnss -- the GNSS receiver's own status, from its port to the radio.

    python3 tools/bench/bench_gnss.py

No receiver, no ROS, no radio. What it covers, and why:

  sbf          PVTGeodetic decodes to the receiver's mode, satellites and
               accuracy; the reader survives what a real USB port delivers --
               ASCII command replies between blocks, a block split across two
               reads, a corrupted block -- and "do not use" fields come back as
               blanks, never as numbers.
  boat_link    the GNSS packet round-trips; unknowns travel as unknowns; the
               Radio tab can describe it; a boat's receiver ignores it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "uav_common"))

from uav_common import boat_link, sbf  # noqa: E402


def check(name, passed, detail=""):
    print("%-56s %-4s %s" % (name, "PASS" if passed else "FAIL", str(detail)[:60]))
    return bool(passed)


def case_sbf():
    r = []
    r.append(check("CRC-CCITT of '123456789' is 0x31C3", sbf.crc16(b"123456789") == 0x31C3,
                   hex(sbf.crc16(b"123456789"))))
    has = sbf.build_pvt_geodetic(10, 11, 0.10, 0.71, lat=32.9238, lon=-117.0386)
    rd = sbf.BlockReader()
    got = rd.feed(has)
    pvt = sbf.pvt_geodetic(got[0][2]) if got else None
    r.append(check("a HAS block: mode 10, 11 sats, 0.10 m / 0.71 m",
                   pvt and pvt["mode"] == 10 and pvt["satellites"] == 11
                   and pvt["h_acc_m"] == 0.10 and pvt["v_acc_m"] == 0.71
                   and abs(pvt["lat"] - 32.9238) < 1e-9, pvt))
    r.append(check("...block number 4007, revision 2",
                   got and got[0][0] == sbf.PVT_GEODETIC and got[0][1] == 2, got and got[0][:2]))

    # what a real port delivers: the command reply, then blocks, split mid-block
    stream = (b"$R: sso, Stream10, USB2, PVTGeodetic, sec1\r\n  SBFOutput, ...\r\nUSB2>"
              + has + sbf.build_pvt_geodetic(1, 24, 1.2, 2.1))
    rd = sbf.BlockReader()
    first = rd.feed(stream[:70])
    rest = rd.feed(stream[70:])
    modes = [sbf.pvt_geodetic(b)["mode"] for _n, _r, b in first + rest]
    r.append(check("ASCII reply skipped, split block joined: modes [10, 1]",
                   modes == [10, 1], modes))
    bad = bytearray(sbf.build_pvt_geodetic(10, 11, 0.1, 0.7))
    bad[40] ^= 0xFF
    rd = sbf.BlockReader()
    out = rd.feed(bytes(bad) + has)
    r.append(check("a corrupted block is dropped, the next one still read",
                   len(out) == 1 and rd.bad_crc == 1, (len(out), rd.bad_crc)))
    dnu = sbf.pvt_geodetic(sbf.BlockReader().feed(
        sbf.build_pvt_geodetic(0, None, None, None))[0][2])
    r.append(check("do-not-use satellites/accuracy -> blanks",
                   dnu["satellites"] is None and dnu["h_acc_m"] is None
                   and dnu["v_acc_m"] is None, dnu))
    r.append(check("a short block is not a PVT", sbf.pvt_geodetic(has[:60]) is None))
    return r


def case_packet():
    r = []
    p = boat_link.pack_gnss(10, 11, 0.10, 0.71)
    g = boat_link.unpack_gnss(p)
    r.append(check("GNSS packet: 6 bytes, round-trips as HAS",
                   len(p) == 6 and g == {"mode": 10, "mode_name": "HAS", "satellites": 11,
                                         "h_acc_m": 0.10, "v_acc_m": 0.71}, g))
    u = boat_link.unpack_gnss(boat_link.pack_gnss(0, None, None, None))
    r.append(check("unknowns travel as unknowns",
                   u["satellites"] is None and u["h_acc_m"] is None and u["v_acc_m"] is None, u))
    big = boat_link.unpack_gnss(boat_link.pack_gnss(1, 300, 900.0, 999.0))
    r.append(check("out-of-range values clamp, never wrap",
                   big["satellites"] == 254 and big["h_acc_m"] == 655.34, big))
    name, text = boat_link.describe(boat_link.PAYLOAD_GNSS, p)
    r.append(check("Radio tab: 'GNSS: HAS, 11 sats, accuracy 0.10 m H / 0.71 m V'",
                   name == "GNSS" and text == "HAS, 11 sats, accuracy 0.10 m H / 0.71 m V",
                   text))
    name, text = boat_link.describe(boat_link.PAYLOAD_GNSS, b"\x0a")
    r.append(check("...a short one is described, not raised", name == "GNSS" and "short" in text,
                   text))
    rx = boat_link.Receiver()
    r.append(check("a boat's receiver ignores the GNSS packet",
                   rx.hear(boat_link.PAYLOAD_GNSS, p) is None and rx.buoys() == []))
    return r


def main():
    results = case_sbf() + case_packet()
    print("\n%d/%d" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
