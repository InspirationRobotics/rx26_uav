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
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "uav_groundstation"))

from uav_groundstation import node_registry as reg          # noqa: E402
from uav_groundstation.gcs_page import render               # noqa: E402
from uav_groundstation.gcs_server import GcsServer          # noqa: E402

# Stand-in for the node's live state. Armed, so the power interlock must bite.
STATE = {"tel": {"pose_ok": True, "fcu_ok": True, "armed": True,
                 "lat": 1.2806, "lon": 103.8557, "alt_rel": 30.0},
         "groups": [], "sys": {"hostname": "uav-jetson"}}


def action(path, payload):
    """Mirrors gcs_node._action's gates, minus the ROS parts."""
    if path == "/node/stop":
        allowed, reason = reg.may_stop(payload.get("name", ""))
        return {"ok": allowed, "message": reason or "would stop"}
    if path == "/node/start":
        name = payload.get("name", "")
        if name not in reg.BY_NAME:
            return {"ok": False, "message": "unknown node %r" % name}
        return {"ok": True, "message": "would start %s" % name}
    if path == "/power":
        if STATE["tel"]["armed"]:
            return {"ok": False, "message": "vehicle is ARMED. Disarm before "
                                            "powering down."}
        if payload.get("confirm") != STATE["sys"]["hostname"]:
            return {"ok": False, "message": "type the hostname to confirm"}
        return {"ok": True, "message": "accepted"}
    return {"ok": False, "message": "unknown action %s" % path}


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
    r.append(check("has all five tabs",
                   all(t in page for t in (b"'nodes'", b"'tel'", b"'map'",
                                           b"'logs'", b"'sys'"))))
    r.append(check("self-contained (no external fetch)",
                   b"http://" not in page.replace(b"http://<JETSON_IP>", b"")
                   and b"cdn" not in page.lower()))
    body, status = get(base, "/")
    r.append(check("GET / serves it", status == 200 and body == page))

    print("\nthe protected-node rule, bypassing the page")
    j = post(base, "/node/stop", {"name": "telemetry_bridge"})
    r.append(check("POST stop telemetry_bridge -> refused", j["ok"] is False,
                   j["message"]))
    r.append(check("  ...and says why", "cannot be stopped" in j["message"]))
    j = post(base, "/node/start", {"name": "telemetry_bridge"})
    r.append(check("POST start telemetry_bridge -> allowed", j["ok"] is True,
                   "starting can only move toward observable"))
    j = post(base, "/node/stop", {"name": "ocs_client"})
    r.append(check("POST stop ocs_client -> allowed", j["ok"] is True))
    j = post(base, "/node/stop", {"name": "../../etc/passwd"})
    r.append(check("POST stop unknown node -> refused", j["ok"] is False,
                   j["message"]))

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
