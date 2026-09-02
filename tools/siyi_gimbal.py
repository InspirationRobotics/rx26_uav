#!/usr/bin/env python3
"""siyi_gimbal — point the A8 mini and read it back, from any machine.

Needs nothing installed: no ROS, no siyi_sdk, no GStreamer. Just a network
route to the camera. Run it from a laptop on the camera subnet, or inside the
container on the aircraft.

    python3 tools/siyi_gimbal.py read                 # attitude + rates, 2 Hz
    python3 tools/siyi_gimbal.py point 0 -90          # yaw 0, pitch -90
    python3 tools/siyi_gimbal.py nadir                # find which sign aims down

WHAT THIS IS FOR. uav_params.yaml carries gimbal_pitch_deg, the angle the node
commands at startup to look straight down, and THE SIGN IS UNIT-SPECIFIC. SIYI
documents -90 as full-down; this airframe's unit aims at the SKY when sent -90,
so nadir on it is +90. A camera that films the clouds for an entire sortie
looks exactly like a camera that works -- the node reports healthy, the file is
the right size, and the footage is worthless. `nadir` below settles it in about
twenty seconds by trying both and reporting what came back.

It is also how you verify the gimbal readback after a rebuild, without waiting
for a flight: `read` prints live angles and rates, and prints nothing but
`no answer` when the gimbal stops talking -- which is the property the whole
readback path was rewritten to have.

    --ip / --port  point at a different camera (default 192.168.144.25:37260)
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "uav_camera"))

from uav_camera.siyi_client import SiyiClient  # noqa: E402

DEFAULT_IP = "192.168.144.25"
DEFAULT_PORT = 37260

# How long to let the gimbal travel before believing the angle it reports. Its
# full sweep is a little over 100 degrees and it is not fast; reading back too
# early catches it mid-move and reports an angle it is only passing through.
SETTLE_S = 3.0


def show(client):
    """One line of gimbal state, or a clear statement that there is none."""
    g = client.attitude_and_rates()
    if g is None:
        return None
    yaw, pitch, roll, yr, pr, rr = g
    print("  yaw %+7.1f  pitch %+7.1f  roll %+7.1f   "
          "rates %+6.1f %+6.1f %+6.1f deg/s" % (yaw, pitch, roll, yr, pr, rr))
    return g


def cmd_read(client, args):
    """Poll until interrupted. Silence is printed, not skipped."""
    print("reading %s:%d -- Ctrl-C to stop" % (args.ip, args.port))
    misses = 0
    try:
        while True:
            if show(client) is None:
                misses += 1
                print("  no answer (%d in a row)" % misses)
            else:
                misses = 0
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def cmd_point(client, args):
    print("commanding yaw %+.1f pitch %+.1f" % (args.yaw, args.pitch))
    if not client.set_angles(args.yaw, args.pitch):
        print("  the gimbal did not acknowledge the command.")
        print("  It may still have moved -- read it back before assuming.")
    print("  settling %.0fs..." % SETTLE_S)
    time.sleep(SETTLE_S)
    g = show(client)
    if g is None:
        print("  no answer. Nothing here can tell you where it is pointing.")
        return 1
    err = abs(g[1] - args.pitch)
    if err > 5.0:
        print("  NOTE: reported pitch is %.1f deg from what was commanded."
              % err)
        print("  Against a mechanical stop, or the sign convention is inverted"
              " -- run `nadir`.")
    return 0


def cmd_nadir(client, args):
    """Try both signs and report which one aims at the ground.

    Deliberately does not decide for you. It reports what the gimbal did with
    each command; you look at the camera, or at the stream, and write the
    answer into gimbal_pitch_deg. A tool that guessed here would be guessing
    about the one setting whose failure is invisible in every log.
    """
    print("Determining which sign aims DOWN on this unit.")
    print("Watch the camera, or the video stream, while this runs.\n")
    results = {}
    for pitch in (-90.0, 90.0):
        print("commanding pitch %+.1f" % pitch)
        client.set_angles(0.0, pitch)
        time.sleep(SETTLE_S)
        g = show(client)
        results[pitch] = None if g is None else g[1]
        print("")

    if all(v is None for v in results.values()):
        print("No answer to either command. The gimbal is not reachable at "
              "%s:%d." % (args.ip, args.port))
        return 1

    print("commanded -> reported:")
    for k, v in results.items():
        print("  %+.1f -> %s" % (k, "no answer" if v is None else "%+.1f" % v))
    print("\nWhichever of these had the camera looking at the GROUND is the")
    print("value for gimbal_pitch_deg in uav_bringup/config/uav_params.yaml.")
    print("Both may report a plausible angle; only one of them points down.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("read", help="print attitude and rates until interrupted")
    p = sub.add_parser("point", help="command an absolute yaw and pitch")
    p.add_argument("yaw", type=float)
    p.add_argument("pitch", type=float)
    sub.add_parser("nadir", help="find which pitch sign aims down on this unit")
    args = ap.parse_args()

    # No SDK needed: pointing and attitude are spoken directly on the wire.
    client = SiyiClient(ip=args.ip, port=args.port).connect_wire_only()
    return {"read": cmd_read, "point": cmd_point,
            "nadir": cmd_nadir}[args.cmd](client, args)


if __name__ == "__main__":
    raise SystemExit(main())
