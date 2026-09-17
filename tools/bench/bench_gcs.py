#!/usr/bin/env python3
"""bench_gcs — prove the ground station's rules hold at the ENDPOINT.

No ROS, no aircraft. Spins the real GcsServer with the real node_registry gate
behind it and drives it over real HTTP.

    python3 tools/bench/bench_gcs.py

WHY OVER HTTP AND NOT BY CALLING THE FUNCTIONS. The page disables the buttons it
should not offer, but anyone can edit JavaScript in a browser or curl the
endpoint directly, so a rule enforced only in the page is decoration. The thing
worth testing is what happens when the button is bypassed — which is exactly
what this does.
"""
import json
import os
import re
import sys
import urllib.error
import urllib.request

REPO = os.path.join(os.path.dirname(__file__), "..", "..")
sys.path.insert(0, os.path.join(REPO, "uav_groundstation"))

from uav_groundstation import node_registry as reg          # noqa: E402
from uav_groundstation.gcs_page import render               # noqa: E402
from uav_groundstation.gcs_server import GcsServer          # noqa: E402

# Stand-in for the node's live state. Armed, so the power interlock must bite.
STATE = {"tel": {"pose_ok": True, "fcu_ok": True, "armed": True,
                 "lat": 1.2806, "lon": 103.8557, "alt_rel": 30.0},
         "groups": [], "sys": {"hostname": "uav-jetson"}}


# {name: time of its restart}, as gcs_node._restart_t, and the bench's own
# clock, so the restart grace window can be crossed without sleeping.
RESTARTED = {}
NOW = [1000.0]


def action(path, payload):
    """Mirrors gcs_node._action, minus the ROS parts.

    The RULES are the real ones -- node_registry.may_stop and start_refusal --
    so what is proved here is the rule itself, not a copy of it. Only the
    process handling is invented, and nothing is running.
    """
    name = payload.get("name", "")
    if path in ("/node/stop", "/node/restart"):
        verb = path.rsplit("/", 1)[1]
        allowed, reason = reg.may_stop(name, verb)
        if allowed and verb == "restart":
            RESTARTED[name] = NOW[0]
        return {"ok": allowed, "message": reason or "would %s" % verb}
    if path == "/node/start":
        why = reg.start_refusal(name, set(), RESTARTED.get(name), NOW[0])
        return {"ok": not why, "message": why or "would start %s" % name}
    if path == "/power":
        if STATE["tel"]["armed"]:
            return {"ok": False, "message": "vehicle is ARMED. Disarm before "
                                            "powering down."}
        if payload.get("confirm") != STATE["sys"]["hostname"]:
            return {"ok": False, "message": "type the hostname to confirm"}
        return {"ok": True, "message": "accepted"}
    return {"ok": False, "message": "unknown action %s" % path}


def io_read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def post(base, path, body):
    req = urllib.request.Request(base + path, method="POST",
                                 data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def get(base, path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return r.read(), r.status


def check(name, ok, detail=""):
    print("  %-46s %-4s %s" % (name, "PASS" if ok else "FAIL", detail[:60]))
    return ok


def main():
    page = render(200.0)
    srv = GcsServer(page, lambda: STATE, action).start(0, "127.0.0.1")
    base = "http://127.0.0.1:%d" % srv._server.server_address[1]
    r = []

    print("\npage")
    r.append(check("poll period substituted", b"__POLL_MS__" not in page,
                   "template placeholder left in" if b"__POLL_MS__" in page else ""))
    r.append(check("has all six tabs",
                   all(t in page for t in (b"'nodes'", b"'tel'", b"'map'",
                                           b"'cam'", b"'logs'", b"'sys'"))))
    r.append(check("self-contained (no external fetch)",
                   b"http://" not in page.replace(b"http://<JETSON_IP>", b"")
                   and b"cdn" not in page.lower()))
    body, status = get(base, "/")
    r.append(check("GET / serves it", status == 200 and body == page))
    r.append(check("light/dark toggle and a light palette",
                   b"toggleTheme" in page and b"data-theme=light" in page))
    r.append(check("grid replaces the fixed 50 m scale bar",
                   b"gridStep" in page and b"'50 m'" not in page))
    # A literal colour on the canvas is a colour the theme toggle cannot reach.
    lit = re.findall(rb"(?:strokeStyle|fillStyle)='(?:#|rgba)", page)
    r.append(check("no hard-coded canvas colours", not lit,
                   "%d found" % len(lit) if lit else ""))
    for name, needle in (("battery readout in the header", b'id="battery"'),
                         ("pre-flight strip in the header", b'id="preflight"'),
                         ("tape measure", b"toggleMeasure"),
                         ("camera footprint drawn", b"m.footprint"),
                         ("lock beep", b"checkLocks"),
                         ("search controls on the map", b'id="searchbar"'),
                         ("search path drawn", b"drawSearch"),
                         ("search tile in the header", b'id="searchtile"')):
        r.append(check(name, needle in page))
    # The page's search controls post SETTINGS and nothing else: on/off, the
    # count, the task and the tier. Anything that could name a position, a mode
    # or a waypoint would be a web page flying the aircraft.
    keys = set(re.findall(rb"post\('/search/config',\{?(\w+)", page))
    r.append(check("search posts only settings",
                   keys <= {b"enabled", b"buoys_to_find", b"b"},
                   b",".join(sorted(keys))))
    picks = set(re.findall(rb"setPick\('(\w+)'", page))
    r.append(check("the two selectors set only task and tier",
                   picks == {b"task", b"tier"}, b",".join(sorted(picks))))

    print("\nthe protected-node rule, bypassing the page")
    j = post(base, "/node/stop", {"name": "telemetry_bridge"})
    r.append(check("POST stop telemetry_bridge -> refused", j["ok"] is False,
                   j["message"]))
    r.append(check("  ...and says why", "cannot be stopped" in j["message"]))
    j = post(base, "/node/start", {"name": "telemetry_bridge"})
    r.append(check("POST start telemetry_bridge -> allowed", j["ok"] is True,
                   "starting can only move toward observable"))
    j = post(base, "/node/restart", {"name": "telemetry_bridge"})
    r.append(check("POST restart telemetry_bridge -> refused", j["ok"] is False,
                   "a restart is a stop for 5 s"))
    j = post(base, "/node/stop", {"name": "../../etc/passwd"})
    r.append(check("POST stop unknown node -> refused", j["ok"] is False,
                   j["message"]))

    print("\nrestart, not stop, for what systemd brings back")
    j = post(base, "/node/stop", {"name": "camera_node"})
    r.append(check("POST stop camera_node -> refused", j["ok"] is False,
                   j["message"]))
    r.append(check("  ...and names the unit to stop instead",
                   "systemctl stop uav-camera" in j["message"]))
    j = post(base, "/node/restart", {"name": "camera_node"})
    r.append(check("POST restart camera_node -> allowed", j["ok"] is True))
    j = post(base, "/node/start", {"name": "camera_node"})
    r.append(check("start inside the respawn gap -> refused", j["ok"] is False,
                   "would run a second camera_node"))
    NOW[0] += reg.RESTART_GRACE_S + 1
    j = post(base, "/node/start", {"name": "camera_node"})
    r.append(check("start after the gap -> allowed", j["ok"] is True,
                   "the unit gave up; the page may start it"))
    j = post(base, "/node/restart", {"name": "ocs_client"})
    r.append(check("POST restart ocs_client -> allowed", j["ok"] is True))
    j = post(base, "/node/restart", {"name": "detector_node"})
    r.append(check("POST restart detector_node -> refused", j["ok"] is False,
                   "no unit: nothing would bring it back"))
    j = post(base, "/node/stop", {"name": "detector_node"})
    r.append(check("POST stop detector_node -> allowed", j["ok"] is True))

    # The button label is only honest if `unit` matches the unit files. Checked
    # both ways: a declared unit must exist and restart the node, and a node
    # declared unsupervised must not be started by any unit.
    units = os.path.join(REPO, "tools", "systemd")
    texts = {f: io_read(os.path.join(units, f)) for f in os.listdir(units)
             if f.endswith(".service")}
    for spec in reg.REGISTRY:
        word = re.compile(r"\b%s\b" % re.escape(spec.executable))
        if spec.unit:
            t = texts.get(spec.unit + ".service", "")
            r.append(check("%s: %s restarts it" % (spec.name, spec.unit),
                           "Restart=on-failure" in t and bool(word.search(t)),
                           "" if t else "no such unit file"))
        else:
            hits = [f for f, t in texts.items() if word.search(t)]
            r.append(check("%s: no unit runs it" % spec.name, not hits,
                           ", ".join(hits)))

    print("\nthe power interlock, bypassing the page")
    j = post(base, "/power", {"verb": "reboot", "confirm": "uav-jetson"})
    r.append(check("reboot while ARMED -> refused", j["ok"] is False, j["message"]))
    STATE["tel"]["armed"] = False
    j = post(base, "/power", {"verb": "reboot", "confirm": "wrong-host"})
    r.append(check("reboot with wrong hostname -> refused", j["ok"] is False))
    j = post(base, "/power", {"verb": "reboot", "confirm": "uav-jetson"})
    r.append(check("reboot disarmed + correct hostname -> ok", j["ok"] is True))

    print("\nactions are POST, state is GET")
    try:
        get(base, "/node/stop")
        r.append(check("GET /node/stop -> 404", False, "it answered a GET"))
    except urllib.error.HTTPError as e:
        r.append(check("GET /node/stop -> 404", e.code == 404,
                       "a link preview or prefetch must not stop a node"))
    body, status = get(base, "/state")
    r.append(check("GET /state serves JSON", status == 200
                   and json.loads(body)["tel"]["lat"] == 1.2806))

    print("\nmalformed input")
    req = urllib.request.Request(base + "/node/stop", method="POST",
                                 data=b"not json",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as resp:
        j = json.loads(resp.read())
    r.append(check("garbage body -> refused, not a crash", j["ok"] is False,
                   j["message"]))

    srv.stop()
    print("\n%d/%d" % (sum(r), len(r)))
    return 0 if all(r) else 1


if __name__ == "__main__":
    raise SystemExit(main())
