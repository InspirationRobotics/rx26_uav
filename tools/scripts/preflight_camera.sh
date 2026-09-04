#!/bin/bash
# Pre-flight camera check, run on the JETSON HOST before a data sortie.
#
#   ssh ekko@ekko.local "~/robotx_ws/src/rx26_uav/tools/scripts/preflight_camera.sh"
#
# WHY THIS EXISTS: the camera's own settings are not evidence. This airframe has
# twice produced a ruined sortie while reporting perfectly correct settings --
# once serving a 720p main stream after we had set 1080p, and once serving H265
# into an H264-hardcoded GStreamer pipeline, which yields pure white frames. In
# both cases a settings read said everything was fine.
#
# So this checks a DECODED FRAME first -- the actual thing that lands in the
# dataset -- and prints settings last, labelled as context. Order is deliberate.
#
# Checks, in order of how much each has cost us:
#   1. decoded frame resolution        (the 720p revert)
#   2. decoded frame not blown out     (the H265/H264 white-frame failure)
#   3. the .mkv actually being written (recorder alive and at the right size)
#   4. gimbal at nadir                 (a battery swap silently returns it to 0)
set -u

EXPECT_W=${EXPECT_W:-1920}
EXPECT_H=${EXPECT_H:-1080}
CTR=${CTR:-uav_ekko}
VIDEO_DIR=${VIDEO_DIR:-$HOME/robotx_ws/logs/video}

FAIL=0
say(){ printf "  %-22s %s\n" "$1" "$2"; }
bad(){ FAIL=1; }

echo "=== Ekko camera preflight  ($(date -u +%Y-%m-%dT%H:%M:%SZ)) ==="
echo "--- what actually reaches the dataset ---"

# 1 + 2. a decoded frame off the live stream.
# The snapshot lands on the HOST; the decoder lives in the CONTAINER, and the
# container has its own /tmp -- hence the docker cp. Forgetting that silently
# reports a missing file as a resolution failure.
if curl -sf --max-time 8 localhost:8091/snapshot.jpg -o /tmp/pf_snap.jpg \
   && docker cp /tmp/pf_snap.jpg "$CTR":/tmp/pf_snap.jpg >/dev/null 2>&1; then
    OUT=$(docker exec -i "$CTR" python3 - <<'PY' 2>/dev/null
import cv2
im = cv2.imread("/tmp/pf_snap.jpg")
if im is None:
    print("ERR")
else:
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    print(im.shape[1], im.shape[0], int(g.mean()),
          int((g >= 250).mean() * 100), int(g.std()))
PY
)
    set -- $OUT
    if [ "${1:-ERR}" = "ERR" ] || [ $# -lt 5 ]; then
        say "decoded frame" "FAIL could not decode the snapshot"; bad
    else
        W=$1; H=$2; MEAN=$3; SAT=$4; SD=$5
        if [ "$W" = "$EXPECT_W" ] && [ "$H" = "$EXPECT_H" ]; then
            say "decoded frame" "OK   ${W}x${H}"
        else
            say "decoded frame" "FAIL ${W}x${H}, expected ${EXPECT_W}x${EXPECT_H}"; bad
        fi
        # A white frame is the codec-mismatch signature. A near-flat frame (tiny
        # std) is the lens-capped / dead-sensor case. Both ruin a sortie silently.
        if [ "$SAT" -ge 50 ] || [ "$MEAN" -ge 240 ]; then
            say "frame content" "FAIL blown out: mean ${MEAN}, ${SAT}% saturated -- CODEC MISMATCH?"; bad
        elif [ "$SD" -lt 8 ]; then
            say "frame content" "FAIL near-flat: std ${SD} -- lens covered or no image?"; bad
        else
            say "frame content" "OK   mean ${MEAN}, ${SAT}% saturated, std ${SD}"
        fi
    fi
else
    say "decoded frame" "FAIL no snapshot from :8091 -- is uav-camera up?"; bad
fi

# 3. what the recorder is putting on disk right now.
MKV=$(ls -t "$VIDEO_DIR"/*.mkv 2>/dev/null | head -1)
if [ -n "${MKV:-}" ]; then
    BASE=$(basename "$MKV")
    R=$(docker exec -i "$CTR" python3 - "$BASE" <<'PY' 2>/dev/null
import sys, cv2
c = cv2.VideoCapture("/root/robotx_ws/logs/video/" + sys.argv[1])
print("%dx%d" % (c.get(3), c.get(4)))
PY
)
    if [ "$R" = "${EXPECT_W}x${EXPECT_H}" ]; then
        say "recorder .mkv" "OK   $R  ($BASE)"
    else
        say "recorder .mkv" "FAIL $R  ($BASE)"; bad
    fi
else
    say "recorder .mkv" "WARN no session open yet"
fi

# 4. gimbal. Powered by the AIRCRAFT: any battery swap or FC reboot returns it
# to 0 (horizon). A whole sortie has been filmed at the horizon with gimbal_ok
# true, so this is checked every single time.
GP=$(curl -sf --max-time 5 localhost:8091/state 2>/dev/null \
     | python3 -c 'import json,sys; print(json.load(sys.stdin).get("gimbal_pitch"))' 2>/dev/null)
OK_NADIR=$(python3 -c "
try:
    v = float(\"${GP:-nan}\")
    print(1 if -93.0 <= v <= -87.0 else 0)
except Exception:
    print(0)
" 2>/dev/null)
if [ "$OK_NADIR" = "1" ]; then
    say "gimbal pitch" "OK   ${GP} (nadir)"
else
    say "gimbal pitch" "FAIL ${GP:-unknown}, expected about -90"; bad
fi

# Settings LAST, and explicitly not proof. The camera answers roughly two
# encoding queries in three; a "no reply" is normal flakiness, not a fault.
echo "--- camera settings (context only, NOT proof) ---"
docker exec -i "$CTR" python3 - <<'PY' 2>/dev/null | sed 's/^/  /'
import struct, sys
sys.path.insert(0, "/root/robotx_ws/src/rx26_uav/uav_camera/uav_camera")
try:
    from siyi_client import SiyiClient
except Exception as e:
    print("could not import siyi_client: %s" % e)
    raise SystemExit
c = SiyiClient("192.168.144.25", 37260)
CODEC = {1: "H264", 2: "H265"}
for st, name in ((0, "SD recording"), (1, "MAIN (Jetson sees)"), (2, "sub stream")):
    try:
        r = c._command(0x20, struct.pack("<B", st), timeout_s=1.5)
    except Exception as e:
        print("%-20s error %s" % (name, e)); continue
    if r and len(r) >= 9:
        _s, codec, w, h, kbps, _p = struct.unpack("<BBHHHB", r[:9])
        print("%-20s %4dx%-5d %-5s %6d kbps" % (name, w, h, CODEC.get(codec, codec), kbps))
    else:
        print("%-20s no reply (retry; ~1 in 3 is dropped)" % name)
PY

echo "---------------------------------------------------"
if [ $FAIL -eq 0 ]; then
    echo "PREFLIGHT PASS -- safe to collect data"
else
    echo "PREFLIGHT FAIL -- do not collect data yet"
fi
exit $FAIL
