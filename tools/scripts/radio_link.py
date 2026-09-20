#!/usr/bin/env python3
"""radio_link — the laptop's RFD900ux, shared out to QGC and the ground station.

    python tools/scripts/radio_link.py --port COM28

A SERIAL PORT HAS EXACTLY ONE OWNER. QGroundControl and our own ground station
both want what the radio hears, and whichever opened it first would lock the
other out — the same rule that makes MAVProxy the sole owner of Ekko's Pixhawk.
So this owns the radio and rebroadcasts over loopback UDP, and both connect to
UDP instead:

    127.0.0.1:14550  -> QGroundControl (its automatic UDP link; nothing to set up)
    127.0.0.1:14543  -> gcs_radio.py, the ground station running on this laptop

Both directions: anything either consumer sends is written back to the radio, so
QGC can still change modes and read parameters. Loopback, so the ports cannot
collide with the aircraft's 1454x or the boat's 1455x on the field network.

It forwards BYTES and counts them. It never parses, rewrites or originates
MAVLink — a link that edits what crosses it is a link you cannot trust when the
two ends disagree.
"""
import argparse
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
        self.up = 0            # consumers -> radio
        self.bytes = 0


def consumer(sock, port, counts, lock):
    """Everything a consumer sends goes back out the radio."""
    while True:
        try:
            data, _ = sock.recvfrom(4096)
        except OSError:
            return
        with lock:
            try:
                port.write(data)
                counts.up += 1
            except Exception:
                return


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
    counts, lock = Counts(), threading.Lock()

    outs = []
    for udp_port, talks_back in ((args.qgc, True), (args.gcs, True),
                                 (args.tee, False)):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))          # consumers reply to this port
        outs.append((s, ("127.0.0.1", udp_port)))
        if talks_back:
            threading.Thread(target=consumer, args=(s, port, counts, lock),
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
            now = time.time()
            if now - last >= 10.0:
                print("  %6.0f B/s from the radio   %d reads, %d frames sent back"
                      % (counts.bytes / (now - last), counts.down, counts.up),
                      flush=True)
                counts.bytes = 0
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


if __name__ == "__main__":
    main()
