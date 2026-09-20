#!/usr/bin/env python3
"""bench_rfd -- rfd_setup.py against a simulated RFD900ux, no radio needed.

The fake modem speaks the AT dialect the manuals document: "+++" answers OK,
ATI/ATI2-4 identify it, ATI5 lists parameters (and can be made to drop some, as
Multipoint firmware does when its buffer overflows -- ATI10:n then fills the
gap), ATSn=X sets, AT&W saves, ATZ reboots (unsaved changes are lost), AT&Mn=..
records the master's network table.

What is checked is the SAFETY of the script as much as its happy path: read-only
by default, refuses a radio that is not running Multipoint firmware, never
touches transmit power unless asked, verifies after the reboot.
"""
import glob
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))

import rfd_setup as rs  # noqa: E402

MULTIPOINT_PARAMS = [
    ("S0", "FORMAT", 69), ("S1", "SERIAL_SPEED", 57), ("S2", "AIR_SPEED", 64),
    ("S3", "NETID", 25), ("S4", "TXPOWER", 30), ("S5", "ECC", 0),
    ("S6", "RXFRAME", 1), ("S7", "OP_RESEND", 0), ("S8", "MIN_FREQ", 922000),
    ("S9", "MAX_FREQ", 928000), ("S10", "NUM_CHANNELS", 21), ("S11", "DUTY_CYCLE", 100),
    ("S12", "LBT_RSSI", 0), ("S13", "RTSCTS", 1), ("S14", "MAX_WINDOW", 80),
    ("S15", "ENCRYPTION_LEVEL", 0), ("S20", "ANT_MODE", 0), ("S24", "NODEID", 2),
    ("S25", "NODEDESTINATION", 255), ("S26", "NETCOUNT", 1), ("R0", "TARGET_RSSI", 0),
]
SIK_PARAMS = [
    ("S0", "FORMAT", 26), ("S1", "SERIAL_SPEED", 57), ("S2", "AIR_SPEED", 64),
    ("S3", "NETID", 25), ("S4", "TXPOWER", 30), ("S6", "MAVLINK", 1),
    ("S8", "MIN_FREQ", 920000), ("S9", "MAX_FREQ", 925000), ("S10", "NUM_CHANNELS", 20),
]


class FakeModem:
    """Just enough of an RFD900ux: serial in, serial out, a command mode."""

    def __init__(self, banner, params, long_form=False, drop=()):
        self.banner, self.long_form, self.drop = banner, long_form, set(drop)
        self.saved = {reg: [name, val] for reg, name, val in params}
        self.live = {reg: list(v) for reg, v in self.saved.items()}
        self.out = b""
        self.command_mode = False
        self.inbuf = b""
        self.log = []
        self.network = None
        self.reboots = 0
        self.silent = False

    # ---- the port interface rfd_setup uses
    def write(self, data):
        if self.silent:
            return
        self.inbuf += data
        if not self.command_mode:
            if self.inbuf.endswith(b"+++"):
                self.inbuf = b""
                self.command_mode = True
                self.out += b"OK\r\n"
            return
        while b"\r\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\r\n", 1)
            self._command(line.decode("ascii").strip())

    def read(self, n):
        chunk, self.out = self.out[:n], self.out[n:]
        return chunk

    def reset_input_buffer(self):
        self.out = b""

    # ---- the modem
    def _say(self, text):
        self.out += text.encode("ascii") + b"\r\n"

    def _param_line(self, reg, name, val):
        if self.long_form:
            return "%s:%s(N)[0..999999]=%d" % (reg, name, val)
        return "%s:%s=%d" % (reg, name, val)

    def _command(self, cmd):
        self.log.append(cmd)
        self._say(cmd)                      # echo, as the radios do
        u = cmd.upper()
        if u == "ATI":
            self._say(self.banner)
        elif u in ("ATI2", "ATI3", "ATI4"):
            self._say({"ATI2": "RFD900UX", "ATI3": "915", "ATI4": "V2.0"}[u])
        elif u == "ATI5":
            for reg, (name, val) in self.live.items():
                if reg not in self.drop:
                    self._say(self._param_line(reg, name, val))
        elif u.startswith("ATI10:"):
            reg = "S" + u.split(":", 1)[1]
            if reg in self.live:
                name, val = self.live[reg]
                self._say(self._param_line(reg, name, val))
        elif u.startswith("ATS") and "=" in u:
            reg, val = u[2:].split("=", 1)
            if reg in self.live and val.lstrip("-").isdigit():
                self.live[reg][1] = int(val)
                self._say("OK")
            else:
                self._say("ERROR")
        elif u.startswith("AT&M"):
            self.network = u
            self._say("OK")
        elif u == "AT&W":
            self.saved = {reg: list(v) for reg, v in self.live.items()}
            self._say("OK")
        elif u == "ATZ":
            self.reboots += 1
            self.live = {reg: list(v) for reg, v in self.saved.items()}
            self.command_mode = False
        elif u == "ATO":
            self.command_mode = False
            self._say("OK")
        else:
            self._say("ERROR")


def link(modem):
    return rs.AtLink(modem, sleep=lambda s: None)


def args(**kw):
    base = dict(apply=False, role=None, node_id=None, nodes=3, air=125, netid=0,
                power=None, channels=None)
    base.update(kw)
    return SimpleNamespace(**base)



def rs_defaults():
    """rfd_setup's argparse defaults, read from the parser itself, so this
    bench cannot drift away from the script it is guarding."""
    import argparse
    import contextlib
    import io as _io
    seen = {}
    real = argparse.ArgumentParser.add_argument

    def spy(self, *a, **kw):
        for name in a:
            if isinstance(name, str) and name.startswith("--"):
                seen[name[2:].replace("-", "_")] = kw.get("default")
        return real(self, *a, **kw)

    argparse.ArgumentParser.add_argument = spy
    try:
        with contextlib.redirect_stderr(_io.StringIO()):
            with contextlib.suppress(SystemExit):
                rs.main([])            # --port is required: it exits after parsing
    finally:
        argparse.ArgumentParser.add_argument = real
    return seen

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print("%-62s %s %s" % (name, "PASS" if ok else "FAIL", detail))


def main():
    print("parsing")
    p = rs.parse_params("S1:SERIAL_SPEED=57\r\nS20:ANT_MODE(N)[0..3]=0{Ant1&2,Ant1}\r\n"
                        "[2] S24:NODEID=2\r\nR0:TARGET_RSSI=0")
    check("short, long and remote-node ATI5 lines all parse",
          p.get("SERIAL_SPEED") == ("S1", 57) and p.get("ANT_MODE") == ("S20", 0)
          and p.get("NODEID") == ("S24", 2) and p.get("TARGET_RSSI") == ("R0", 0), p)
    check("firmware family from the banner",
          rs.family("RFD900x Multipoint V3.00MP") == rs.MULTIPOINT
          and rs.family("RFD SiK 3.57 on RFD900ux") == rs.SIK
          and rs.family("RFD Async V4.03") == rs.ASYNC,
          [rs.family(b) for b in ("RFD900x Multipoint V3.00MP", "RFD SiK 3.57 on RFD900ux",
                                  "RFD Async V4.03")])

    print("\nread only")
    m = FakeModem("RFD900ux Multipoint V3.00MP", MULTIPOINT_PARAMS)
    lines = []
    code = rs.run(link(m), args(), out=lines.append)
    writes = [c for c in m.log if c.upper().startswith(("ATS", "AT&W", "AT&M", "ATZ"))]
    check("without --apply nothing is written, saved or rebooted",
          code == 0 and not writes and m.reboots == 0, writes)
    check("the report names the wrong band and the transmit power",
          any("922000-928000" in l for l in lines) and any("30 dBm" in l for l in lines))

    print("\nthe defaults match the mesh that exists")
    # All three radios were set to SERIAL_SPEED 115200, AIR_SPEED 125 and
    # NETID 0 on 18 Sep. A default that disagrees is not a slower link: the
    # radio it is applied to goes DEAF, and a read at the wrong baud looks
    # exactly like a dead radio (it cost half an hour on 19 Sep).
    d = rs_defaults()
    check("--baud defaults to the mesh's serial speed", d["baud"] == 115200,
          d.get("baud"))
    check("--air defaults to the mesh's air speed", d["air"] == 125, d.get("air"))
    check("--netid defaults to multipoint's master net", d["netid"] == 0,
          d.get("netid"))

    print("\nthe laptop tools compile")
    # Nothing else checks these: no bench imports them and colcon does not
    # build tools/scripts, so a SyntaxError here surfaces at the field.
    scripts = sorted(glob.glob(os.path.join(HERE, '..', 'scripts', '*.py')))
    bad = []
    for f in scripts:
        # compile(), not py_compile: no .pyc is written anywhere, and it behaves
        # the same on the laptop and on the Jetson.
        try:
            with open(f, encoding='utf-8') as fh:
                compile(fh.read(), f, 'exec')
        except SyntaxError as e:
            bad.append('%s line %s: %s' % (os.path.basename(f), e.lineno, e.msg))
    check("every script in tools/scripts compiles (%d)" % len(scripts),
          scripts and not bad, bad)

    print("\nrefusals")
    m = FakeModem("RFD SiK 3.57 on RFD900ux", SIK_PARAMS)
    code = rs.run(link(m), args(apply=True, role="node", node_id=2), out=lambda s: None)
    check("stock SiK firmware: --apply refuses and writes nothing",
          code == 3 and not [c for c in m.log if c.upper().startswith("ATS")], code)
    m = FakeModem("x", MULTIPOINT_PARAMS)
    m.silent = True
    code = rs.run(link(m), args(), out=lambda s: None)
    check("no answer to +++ (wrong port, or QGC holding it): says so", code == 2, code)
    m = FakeModem("RFD900ux Multipoint V3.00MP",
                  [p for p in MULTIPOINT_PARAMS if p[1] != "NODEDESTINATION"])
    code = rs.run(link(m), args(apply=True, role="node", node_id=2), out=lambda s: None)
    check("a parameter the radio does not report: refuses to write blind",
          code == 4 and not [c for c in m.log if c.upper().startswith("ATS")], code)

    print("\napply")
    m = FakeModem("RFD900ux Multipoint V3.00MP", MULTIPOINT_PARAMS)
    code = rs.run(link(m), args(apply=True, role="master", nodes=3), out=lambda s: None)
    live = {name: val for name, val in m.saved.values()}
    check("master: node 1, broadcast, network 0 table for 3 nodes, verified",
          code == 0 and live["NODEID"] == 1 and live["NODEDESTINATION"] == 255
          and live["NETCOUNT"] == 1 and m.network == "AT&M0=0,3", (code, m.network))
    check("master: band 920000-925000, MAVLink framing, NETID 0",
          live["MIN_FREQ"] == 920000 and live["MAX_FREQ"] == 925000
          and live["RXFRAME"] == 1 and live["NETID"] == 0, live)
    check("transmit power and channel count untouched unless asked",
          live["TXPOWER"] == 30 and live["NUM_CHANNELS"] == 21,
          (live["TXPOWER"], live["NUM_CHANNELS"]))
    check("saved, then rebooted exactly once", m.reboots == 1, m.reboots)

    m = FakeModem("RFD900ux Multipoint V3.00MP", MULTIPOINT_PARAMS, long_form=True,
                  drop=("S24", "S25"))
    code = rs.run(link(m), args(apply=True, role="node", node_id=2, power=20, air=125),
                  out=lambda s: None)
    live = {name: val for name, val in m.saved.values()}
    check("node 2 on long-form output with ATI5 dropping NODEID: filled by ATI10",
          code == 0 and live["NODEID"] == 2 and m.network is None
          and any(c.upper().startswith("ATI10:") for c in m.log), code)
    check("--power and --air are applied when given",
          live["TXPOWER"] == 20 and live["AIR_SPEED"] == 125, live)

    print("\nverification catches a radio that does not keep a setting")

    class Forgetful(FakeModem):
        def _command(self, cmd):
            FakeModem._command(self, cmd)
            if cmd.upper() == "ATZ":
                self.live["S8"][1] = 922000      # reverts the band on reboot

    m = Forgetful("RFD900ux Multipoint V3.00MP", MULTIPOINT_PARAMS)
    lines = []
    code = rs.run(link(m), args(apply=True, role="node", node_id=3), out=lines.append)
    check("a setting lost across the reboot fails verification",
          code == 8 and any("VERIFY FAILED" in l and "MIN_FREQ" in l for l in lines), code)

    print("\nargument rules")
    check("--apply without a role is refused", rs.check_args(args(apply=True)) is not None)
    check("a node cannot be node 1 (that is the master)",
          rs.check_args(args(apply=True, role="node", node_id=1)) is not None)
    check("an air rate the radio does not support is refused",
          rs.check_args(args(air=80)) is not None)

    print("\n%d/%d" % (sum(RESULTS), len(RESULTS)))
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
