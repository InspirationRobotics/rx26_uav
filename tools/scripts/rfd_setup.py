#!/usr/bin/env python3
"""rfd_setup — read, and optionally set, an RFD900ux for the three-radio mesh.

    python rfd_setup.py --port COM5                                  # READ ONLY
    python rfd_setup.py --port COM5 --apply --role master --nodes 3  # laptop
    python rfd_setup.py --port COM6 --apply --role node --node-id 2  # Ekko's
    python rfd_setup.py --port COM7 --apply --role node --node-id 3  # Crusader's

One radio at a time, plugged into this laptop over USB. Without --apply it
changes NOTHING: it reports the firmware, the board, and every setting, which is
also how to find out what someone else changed on a radio.

THE MESH (Chris, 16 Sep 2026): RFDesign's MULTIPOINT firmware, not "Asynchronous
Mesh" -- RFDesign does not recommend the async firmware for bands narrower than
915-928 MHz, and ours is 920-925. Multipoint has ONE master (NODEID 1, NETID 0):
every radio must stay in range of it, and the network stops without it. The
master is the ground laptop's radio. Every radio broadcasts (NODEDESTINATION
255), so every node hears everything -- QGC, Ekko and Crusader alike.

FLASHING FIRMWARE IS NOT DONE HERE. Use RFD Modem Tools with
RFD900ux(2)-MultipointRelease_V3.00MP from files.rfdesign.com.au/firmware/ --
the .gbl for V2 hardware, the .bin for V1. This script says which firmware a
radio is running and refuses to --apply to anything but Multipoint.

What --apply sets (all radios): AIR_SPEED, NETID, MIN_FREQ 920000, MAX_FREQ
925000, RXFRAME 1 (MAVLink framing, so the radio injects RADIO_STATUS),
NODEID, NODEDESTINATION 255. The master also gets NETCOUNT 1 and AT&M0=0,<nodes>.
TXPOWER and NUM_CHANNELS are changed only when asked (--power, --channels):
transmit power is a legal question for the venue, not a default.
"""
import argparse
import re
import sys
import time

#: Multipoint parameter names, exactly as the firmware prints them.
MULTIPOINT = "multipoint"
SIK = "sik"
ASYNC = "async"
UNKNOWN = "unknown"

BAND_KHZ = (920000, 925000)
AIR_SPEEDS = (12, 56, 64, 100, 125, 188, 200, 224, 500, 750)
BROADCAST = 255
MAX_NODE_ID = 16

_PARAM_LINE = re.compile(
    r"^\s*(?:\[\d+\]\s*)?([SR])(\d+):\s*([A-Za-z0-9_/ ]+?)\s*"
    r"(?:\([^)]*\))?\s*(?:\[[^\]]*\])?\s*=\s*(-?\d+)")


# ------------------------------------------------------------------ parsing

def parse_params(text):
    """ATI5 / ATI10:n output -> {name: (register, value)}.

    Handles both forms firmware prints: "S1:SERIAL_SPEED=57" and
    "S20:ANT_MODE(N)[0..3]=0{...}". Register is "S1" / "R0".
    """
    out = {}
    for line in text.splitlines():
        m = _PARAM_LINE.match(line)
        if m:
            kind, num, name, value = m.groups()
            out[name.strip().upper().replace(" ", "_")] = (kind + num, int(value))
    return out


def family(ati):
    """Which firmware, from the ATI banner. Multipoint banners carry "MP"."""
    t = ati.upper()
    if "ASYNC" in t:
        return ASYNC
    if "MULTIPOINT" in t or re.search(r"\bMP\b|\d+\.\d+MP", t):
        return MULTIPOINT
    if "SIK" in t or "RFD" in t:
        return SIK
    return UNKNOWN


def wanted(args):
    """{name: value} this radio should end up with, for its role."""
    want = {
        "AIR_SPEED": args.air,
        "NETID": args.netid,
        "MIN_FREQ": BAND_KHZ[0],
        "MAX_FREQ": BAND_KHZ[1],
        "RXFRAME": 1,
        "NODEID": 1 if args.role == "master" else args.node_id,
        "NODEDESTINATION": BROADCAST,
    }
    if args.role == "master":
        want["NETCOUNT"] = 1
    if args.power is not None:
        want["TXPOWER"] = args.power
    if args.channels is not None:
        want["NUM_CHANNELS"] = args.channels
    return want


def check_args(args):
    """-> an error message, or None."""
    if args.air not in AIR_SPEEDS:
        return "--air must be one of %s" % ", ".join(map(str, AIR_SPEEDS))
    if args.apply and args.role is None:
        return "--apply needs --role master or --role node"
    if args.role == "node" and not (2 <= (args.node_id or 0) <= MAX_NODE_ID):
        return "--role node needs --node-id 2..%d (1 is the master)" % MAX_NODE_ID
    if args.role == "master" and not (2 <= args.nodes <= MAX_NODE_ID):
        return "--nodes is the highest node id on the network, 2..%d" % MAX_NODE_ID
    if args.power is not None and not 0 <= args.power <= 30:
        return "--power is 0..30 dBm"
    return None


def spacing_khz(params):
    """Channel spacing from what the radio holds, for the report."""
    try:
        lo, hi = params["MIN_FREQ"][1], params["MAX_FREQ"][1]
        n = params["NUM_CHANNELS"][1]
        return (hi - lo) / float(n) if n else None
    except KeyError:
        return None


# ------------------------------------------------------------------ the link

class AtLink:
    """AT command mode over anything with write(bytes), read(n) and a timeout.

    The radio only enters command mode on "+++" framed by about a second of
    SILENCE either side -- send it mid-stream and it is just data.
    """

    GUARD_S = 1.2

    def __init__(self, port, sleep=time.sleep, clock=time.monotonic):
        self.port, self.sleep, self.clock = port, sleep, clock

    def _read_until(self, done, timeout_s):
        buf, end = b"", self.clock() + timeout_s
        while self.clock() < end:
            chunk = self.port.read(256)
            if chunk:
                buf += chunk
                if done(buf):
                    break
            else:
                self.sleep(0.05)
        return buf.decode("ascii", "replace")

    def enter(self):
        self.sleep(self.GUARD_S)
        self.port.reset_input_buffer()
        self.port.write(b"+++")
        reply = self._read_until(lambda b: b"OK" in b, 3.0)
        return "OK" in reply

    def cmd(self, text, timeout_s=1.5, until=b"\n"):
        self.port.reset_input_buffer()
        self.port.write(text.encode("ascii") + b"\r\n")
        reply = self._read_until(lambda b: b.rstrip().endswith((b"OK", b"ERROR"))
                                 or (until is not None and b.count(b"\n") > 1
                                     and not text.upper().startswith("ATI5")),
                                 timeout_s)
        # Drop the echo of the command itself.
        lines = [l for l in reply.replace("\r", "").split("\n")
                 if l.strip() and l.strip().upper() != text.upper()]
        return "\n".join(lines)

    def params(self, expect=()):
        """Every parameter, filling any ATI5 dropped (it can overflow its buffer
        on Multipoint firmware) with ATI10:n."""
        got = parse_params(self.cmd("ATI5", timeout_s=3.0))
        have = {reg for reg, _ in got.values()}
        for reg in expect:
            if reg not in have and reg.startswith("S"):
                got.update(parse_params(self.cmd("ATI10:%s" % reg[1:])))
        return got


# ------------------------------------------------------------------ the job

def run(link, args, out=print):
    """Report, and with args.apply set and verify. -> exit code."""
    if not link.enter():
        out("NO ANSWER to +++ : wrong port, wrong baud (try --baud 115200), or the "
            "radio is busy passing data (close QGC / MAVProxy on this port first)")
        return 2
    ati = link.cmd("ATI")
    fam = family(ati)
    out("firmware : %s  (%s)" % (ati.strip() or "?", fam))
    for q, label in (("ATI2", "board"), ("ATI3", "band"), ("ATI4", "board version")):
        out("%-9s: %s" % (label, link.cmd(q).strip() or "?"))

    params = link.params(expect=("S24", "S25", "S26") if fam == MULTIPOINT else ())
    out("\nsettings now:")
    for name, (reg, value) in sorted(params.items(), key=lambda kv: (kv[1][0][0], int(kv[1][0][1:]))):
        out("  %-5s %-18s %s" % (reg, name, value))
    sp = spacing_khz(params)
    if sp:
        out("  -> channel spacing %.0f kHz" % sp)
    lo_hi = (params.get("MIN_FREQ", (None, None))[1], params.get("MAX_FREQ", (None, None))[1])
    if None not in lo_hi and tuple(lo_hi) != BAND_KHZ:
        out("  !! band is %d-%d kHz, not the 920000-925000 we fly on" % lo_hi)
    power = params.get("TXPOWER", (None, None))[1]
    if power is not None:
        out("  -> transmit power %d dBm -- check it is legal where you fly" % power)

    if not args.apply:
        out("\nread only: nothing was changed (add --apply to set this radio)")
        link.cmd("ATO")
        return 0

    if fam != MULTIPOINT:
        out("\nREFUSED: this radio runs %s firmware. Flash "
            "RFD900ux(2)-MultipointRelease_V3.00MP with RFD Modem Tools first "
            "(.gbl for V2 hardware, .bin for V1), then run this again." % fam)
        link.cmd("ATO")
        return 3

    want = wanted(args)
    out("\nsetting %s:" % ("the MASTER (node 1)" if args.role == "master"
                          else "node %d" % args.node_id))
    missing = [n for n in want if n not in params]
    if missing:
        out("REFUSED: the radio did not report %s -- not writing blind"
            % ", ".join(missing))
        link.cmd("ATO")
        return 4
    for name, value in want.items():
        reg, now = params[name]
        if now == value:
            out("  %-18s %s (already)" % (name, value))
            continue
        reply = link.cmd("AT%s=%d" % (reg, value))
        ok = "OK" in reply
        out("  %-18s %s -> %s  %s" % (name, now, value, "ok" if ok else "REFUSED: " + reply))
        if not ok:
            link.cmd("ATO")
            return 5
    if args.role == "master":
        reply = link.cmd("AT&M0=0,%d" % args.nodes)
        out("  network table      AT&M0=0,%d  %s" % (args.nodes, "ok" if "OK" in reply else reply))
    reply = link.cmd("AT&W")
    if "OK" not in reply:
        out("REFUSED to save (AT&W): %s" % reply)
        return 6
    link.cmd("ATZ", timeout_s=0.5)
    out("saved; rebooting the radio to apply")
    link.sleep(4.0)

    if not link.enter():
        out("the radio did not come back into command mode after rebooting -- "
            "unplug it, plug it in, and run this again without --apply to check")
        return 7
    after = link.params(expect=tuple(params[n][0] for n in want))
    bad = [n for n, v in want.items() if after.get(n, (None, None))[1] != v]
    link.cmd("ATO")
    if bad:
        out("VERIFY FAILED: %s" % ", ".join("%s=%s (wanted %s)"
                                              % (n, after.get(n, (None, "?"))[1], want[n])
                                              for n in bad))
        return 8
    out("verified after reboot: %d settings as wanted" % len(want))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", required=True, help="COM5, /dev/ttyUSB0 ...")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--apply", action="store_true", help="write settings (default: read only)")
    ap.add_argument("--role", choices=("master", "node"))
    ap.add_argument("--node-id", type=int, help="2..16 for a node (the master is 1)")
    ap.add_argument("--nodes", type=int, default=3,
                    help="master only: the highest node id on the network (3 = laptop, Ekko, Crusader)")
    ap.add_argument("--air", type=int, default=64, help="air data rate, kbit/s")
    ap.add_argument("--netid", type=int, default=0)
    ap.add_argument("--power", type=int, help="TXPOWER dBm; unchanged unless given")
    ap.add_argument("--channels", type=int, help="NUM_CHANNELS; unchanged unless given")
    args = ap.parse_args(argv)
    err = check_args(args)
    if err:
        ap.error(err)
    import serial  # pyserial, installed with pymavlink
    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        return run(AtLink(port), args)


if __name__ == "__main__":
    sys.exit(main())
