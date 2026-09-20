#!/usr/bin/env python3
"""radio_link — share the RFD900ux when QGroundControl is NOT running.

    python tools/scripts/radio_link.py --port COM28

NORMALLY YOU DO NOT WANT THIS. QGC owns the radio directly (a serial link on
COM28 at 115200, exactly as it owned the Holybro through every flight test) and
feeds the laptop's ground station through its own MAVLink Forwarding —
Application Settings > Telemetry > MAVLink Forwarding > localhost:14445 — which
is one less program in the path that carries the aircraft's telemetry:

    python tools/scripts/gcs_radio.py --from udpin:127.0.0.1:14445

This exists for the case where QGC is not in the picture and something still has
to share the radio, because A SERIAL PORT HAS EXACTLY ONE OWNER: the page and a
test tool would otherwise lock each other out. It holds the radio and
rebroadcasts over loopback UDP:

    127.0.0.1:14550  -> a GCS, if one is running
    127.0.0.1:14543  -> gcs_radio.py
    127.0.0.1:14544  -> ad-hoc diagnostics, LISTEN ONLY

Loopback, so the ports cannot collide with the aircraft's 1454x or the boat's
1455x on the field network. It forwards BYTES and counts them; it never parses,
rewrites or originates MAVLink.

THE PORT HAS ONE OWNER HERE TOO, and that is this program's main loop: reads and
writes both happen there, and everything a consumer sends is queued for it.
The first version wrote to the port from the consumer threads and swallowed the
failure with a bare `return` — the thread died on the first write error, the
uplink went silently dead for the rest of the session, and every request looked
like one the aircraft had ignored. That cost an afternoon on 20 Sep. A write
that fails now says so and the link carries on.
"""
import argparse
import queue
import socket
import sys
import threading
import time

try:
    import serial                                   # pyserial
except ImportError:                                 # pragma: no cover
    sys.exit("pyserial is missing:  pip install pyserial")

QGC = ("127.0.0.1", 14550)
GCS = ("127.0.0.1", 14543)
#: Diagnostics only, and deliberately LAST: the same role 14542 plays on Ekko.
#: A probe that binds a port someone else is being fed STEALS the datagrams and
#: the displaced consumer sees silence, not an error -- so probes get their own.
TEE = ("127.0.0.1", 14544)


class Counts:
    def __init__(self):
        self.down = 0          # radio -> consumers
        self.up = 0            # consumers -> radio, written
        self.queued = 0        # consumers -> radio, offered
        self.bytes = 0
        self.errors = 0


def consumer(sock, outbound, counts):
    """Everything a consumer sends is QUEUED for the radio.

    It does not touch the serial port. pyserial is not safe to write from one
    thread while another reads, and the first version did exactly that and
    swallowed the failure with a bare `return` -- the thread died, the uplink
    went silently dead for the rest of the session, and every request looked
    like an unanswered one. One owner for the port; everything else queues.
    """
    while True:
        try:
            data, _ = sock.recvfrom(4096)
        except OSError as e:
            print("  consumer socket closed: %s" % e, flush=True)
            return
        outbound.put(data)
        counts.queued += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", required=True, help="the radio's serial port (COM28)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="the mesh's SERIAL_SPEED (default 115200)")
    ap.add_argument("--qgc", type=int, default=QGC[1],
                    help="UDP port for QGroundControl (default 14550)")
    ap.add_argument("--gcs", type=int, default=GCS[1],
                    help="UDP port for the laptop ground station (default 14543)")
    ap.add_argument("--tee", type=int, default=TEE[1],
                    help="UDP port for ad-hoc diagnostics, listen-only (default "
                         "14544). Nothing in the flight picture may use it.")
    args = ap.parse_args()

    port = serial.Serial(args.port, args.baud, timeout=0.05)
    counts, outbound = Counts(), queue.Queue()

    outs = []
    for udp_port, talks_back in ((args.qgc, True), (args.gcs, True),
                                 (args.tee, False)):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))          # consumers reply to this port
        outs.append((s, ("127.0.0.1", udp_port)))
        print("  -> 127.0.0.1:%d   (replies accepted on 127.0.0.1:%d)"
              % (udp_port, s.getsockname()[1]), flush=True)
        if talks_back:
            threading.Thread(target=consumer, args=(s, outbound, counts),
                             daemon=True).start()

    print("radio %s @ %d  ->  QGC 127.0.0.1:%d   ground station 127.0.0.1:%d"
          "   diagnostics 127.0.0.1:%d"
          % (args.port, args.baud, args.qgc, args.gcs, args.tee), flush=True)
    print("ctrl-C to stop", flush=True)
    last = time.time()
    try:
        while True:
            data = port.read(1024)
            if data:
                counts.down += 1
                counts.bytes += len(data)
                for sock, addr in outs:
                    sock.sendto(data, addr)
            # The SAME thread writes: one owner for the port, as MAVProxy is for
            # the Pixhawk. A write that fails is reported and the link carries
            # on -- it must never take the uplink down with it.
            while True:
                try:
                    out = outbound.get_nowait()
                except queue.Empty:
                    break
                try:
                    port.write(out)
                    counts.up += 1
                except Exception as e:                # noqa: BLE001
                    counts.errors += 1
                    print("  WRITE FAILED (%d so far): %s" % (counts.errors, e),
                          flush=True)
            now = time.time()
            if now - last >= 10.0:
                print("  %6.0f B/s from the radio   %d reads   uplink %d sent"
                      "/%d offered%s"
                      % (counts.bytes / (now - last), counts.down, counts.up,
                         counts.queued,
                         "   %d WRITE ERRORS" % counts.errors if counts.errors
                         else ""), flush=True)
                counts.bytes = 0
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


if __name__ == "__main__":
    main()
