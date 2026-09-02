#!/usr/bin/env python3
"""check_config — guard the config file that can ground the aircraft.

    python3 tools/scripts/check_config.py [--params PATH] [--bridge-toml PATH]

Runs without ROS, so CI and a laptop can both run it. Exit nonzero = do not fly.

WHAT IT GUARDS, AND WHY EACH ONE IS HERE

  rcl's two format rules. Break either and EVERY node dies at rclpy.init()
  before a line of node code runs. PyYAML accepts both, which is exactly why a
  human reading the file does not catch them.

  The pinned duplicates. rcl forbids YAML anchors, so values that must stay
  equal across node sections are written out literally. That makes drift
  invisible: two numbers that disagree look like two settings, not one broken
  one. This is the only thing that notices.

  The geofence, hardest of all. It appears three times in the params AND once
  more in the OCS repo's bridge.toml, and all four must agree. If the params and
  the OCS disagree we upload one polygon to the autopilot and declare a
  different one to RoboNation — flying a box we did not declare, which is a
  protest, not a bug. It must also be CLOSED and have at least 4 points, because
  RoboNation's test_server rejects a run declaration outright when a UAV's fence
  has fewer than 4.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DEFAULT_PARAMS = os.path.join(REPO, "uav_bringup", "config", "uav_params.yaml")

#: Where to look for the OCS repo, if it is checked out beside this one. Not an
#: error when absent — most machines have one repo — but when it IS there the
#: geofence cross-check is the most valuable thing in this file.
DEFAULT_BRIDGE_TOML = os.path.join(os.path.dirname(REPO), "rx26_ocs",
                                   "bridge.toml")

problems = []
notes = []


def fail(msg):
    problems.append(msg)


def note(msg):
    notes.append(msg)


def check_format_rules(raw):
    """rcl's parser is stricter than PyYAML. These are the two that bite."""
    for i, line in enumerate(raw.splitlines(), 1):
        code = line.split("#", 1)[0]
        if re.search(r"\s[&*]\w", code):
            fail("line %d: YAML anchor/alias — rcl_yaml_param_parser rejects "
                 "these outright and every node dies at rclpy.init(). Write "
                 "the value out literally and add it to PINNED below.\n"
                 "    %s" % (i, line.strip()))


def check_structure(cfg):
    for key, val in cfg.items():
        if not isinstance(val, dict) or list(val) != ["ros__parameters"]:
            fail("top-level key %r must have `ros__parameters` as its ONLY "
                 "child (rcl: \"Cannot have a value before ros__parameters\"). "
                 "Got: %s" % (key, list(val) if isinstance(val, dict) else type(val).__name__))


def get(cfg, section, name):
    try:
        return cfg[section]["ros__parameters"][name]
    except KeyError:
        fail("%s.%s is missing" % (section, name))
        return None


#: (canonical, [(section, param), ...]) — every copy that must stay equal.
PINNED = [
    (("shared", "pose_timeout_s"), [
        ("telemetry_bridge", "stream_timeout_s"),
        ("ocs_client", "pose_timeout_s"),
        ("ground_station", "pose_timeout_s"),
        # camera_node uses it to decide when a frame's pose is too old to
        # write into the frame index. A camera that trusted a pose for longer
        # than the bridge vouches for it would stamp coordinates onto frames
        # the bridge had already stopped standing behind — and unlike a stale
        # readout, that one is written to disk and read back months later as
        # if it were measured.
        ("camera_node", "pose_timeout_s"),
    ]),
    (("shared", "geoid_separation_m"), [
        ("ocs_client", "geoid_separation_m"),
    ]),
    (("shared", "geofence"), [
        ("telemetry_bridge", "geofence"),
        ("ground_station", "geofence"),
    ]),
]


def check_pinned(cfg):
    for (csec, cname), copies in PINNED:
        canonical = get(cfg, csec, cname)
        if canonical is None:
            continue
        for sec, name in copies:
            v = get(cfg, sec, name)
            if v is None:
                continue
            if v != canonical:
                fail("%s.%s = %r but %s.%s = %r — these must be equal. rcl "
                     "forbids anchors, so the copies are written out literally "
                     "and this check is what notices when one drifts."
                     % (csec, cname, canonical, sec, name, v))


def _pairs(flat):
    """Flat [lat, lon, ...] -> [(lat, lon), ...]. None if it cannot be paired.

    Mirrors uav_common.fence_core.polygon_from_flat, deliberately: this script
    runs with no ROS workspace sourced (CI, a laptop), so it cannot import from
    the packages it checks. Six lines duplicated is the price of that.
    """
    if any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in flat):
        return None
    if len(flat) % 2:
        return None
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]


def check_geofence(cfg):
    fence = get(cfg, "shared", "geofence")
    if not fence:
        return None
    if any(isinstance(v, (list, tuple)) for v in fence):
        fail("geofence is NESTED. ROS 2 parameters cannot nest — rclpy rejects "
             "a list of pairs at declare_parameter() and the node dies before "
             "any of its code runs. Write it FLAT: [lat, lon, lat, lon, ...]. "
             "uav_common.fence_core.polygon_from_flat pairs them back up.")
        return None
    pts = _pairs(fence)
    if pts is None:
        fail("geofence must be a FLAT list of an EVEN number of plain numbers "
             "(lat, lon, lat, lon, ...); got %d entries. A dropped coordinate "
             "shifts every pair after it." % len(fence))
        return None
    if len(pts) < 2 or pts[0] != pts[-1]:
        fail("geofence is not CLOSED (first point must equal last). The OCS "
             "declaration requires a closed ring; the closing point is stripped "
             "on the way to the autopilot, not here.")
    distinct = pts[:-1] if len(pts) >= 2 and pts[0] == pts[-1] else pts
    if len(distinct) < 4:
        fail("geofence has %d distinct vertices; RoboNation's test_server "
             "REJECTS a run declaration when a vehicle id starts with 'UAV' and "
             "the fence has fewer than 4 points (rc_test/test_server.py:104). A "
             "wrong fence here fails a RUN, not just a flight."
             % len(distinct))
    for lat, lon in distinct:
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            fail("geofence vertex (%s, %s) is not a valid lat/lon — check the "
                 "order; these are [lat, lon], not [lon, lat]." % (lat, lon))
    return pts


def check_against_ocs(pts, toml_path):
    """The cross-repo check: our fence vs the one the OCS declares."""
    if pts is None:
        return
    if not os.path.isfile(toml_path):
        note("OCS bridge.toml not found at %s — skipping the cross-repo "
             "geofence check. Clone rx26_ocs beside this repo to enable it; it "
             "is the check that catches declaring one box and flying another."
             % toml_path)
        return
    try:
        import tomllib
    except ImportError:                     # Python < 3.11
        note("no tomllib (needs Python 3.11+) — skipped the OCS cross-check")
        return
    with open(toml_path, "rb") as f:
        raw = tomllib.load(f)
    theirs = [tuple(p) for p in raw.get("uav_geofence", [])]
    if not theirs:
        fail("%s has an EMPTY uav_geofence. The OCS cannot declare a run for a "
             "UAV without one." % toml_path)
        return
    if theirs != pts:
        fail("GEOFENCE MISMATCH between repos — we would upload one polygon to "
             "the autopilot and declare a different one to RoboNation.\n"
             "    ours  (uav_params.yaml): %s\n"
             "    theirs (%s): %s"
             % (pts, os.path.basename(toml_path), theirs))
    else:
        note("geofence matches the OCS bridge.toml (%d points)" % len(pts))


def check_ocs_identity(cfg, toml_path):
    """vehicle_id and team_id must exist in the OCS fleet, or every frame drops."""
    vid = get(cfg, "ocs_client", "vehicle_id")
    tid = get(cfg, "ocs_client", "team_id")
    if vid and not str(vid).upper().startswith("UAV"):
        note("vehicle_id %r does not start with 'UAV' — legal, but RoboNation's "
             "geofence requirement keys on that prefix." % vid)
    if not os.path.isfile(toml_path):
        return
    try:
        import tomllib
    except ImportError:
        return
    with open(toml_path, "rb") as f:
        raw = tomllib.load(f)
    if tid and raw.get("team_id") != tid:
        fail("team_id %r != the OCS's %r — the OCS drops frames it cannot "
             "attribute." % (tid, raw.get("team_id")))
    fleet = {v.get("id"): v.get("type") for v in raw.get("vehicle", [])}
    if vid and vid not in fleet:
        fail("vehicle_id %r is not a [[vehicle]] in %s (fleet: %s) — the OCS "
             "drops every frame with 'not a configured vehicle'."
             % (vid, os.path.basename(toml_path), sorted(fleet)))
    elif vid and fleet.get(vid) != "TYPE_UAV":
        fail("the OCS has %r as %s, but ocs_client reports TYPE_UAV. The bridge "
             "drops any heartbeat whose type contradicts its config."
             % (vid, fleet.get(vid)))


def check_ports(cfg):
    """Ports must be distinct locally, and clear of the ASV's on a shared subnet."""
    # Every port this vehicle binds or dials, checked pairwise. Listed rather
    # than compared ad hoc so adding a service means adding one line here, and
    # the failure names both halves instead of leaving someone to find the
    # other one.
    ports = [
        ("ground_station.port", get(cfg, "ground_station", "port")),
        ("ocs_client.ocs_port", get(cfg, "ocs_client", "ocs_port")),
        ("camera_node.mjpeg_port", get(cfg, "camera_node", "mjpeg_port")),
        ("camera_node.siyi_port", get(cfg, "camera_node", "siyi_port")),
    ]
    seen = {}
    for name, value in ports:
        if value is None:
            continue
        if value in seen:
            fail("%s and %s are both %s — two servers cannot bind one socket "
                 "and the loser dies with an address-in-use that reads like a "
                 "crash." % (seen[value], name, value))
        seen[value] = name
    mjpeg = get(cfg, "camera_node", "mjpeg_port")
    if mjpeg is not None and 14540 <= mjpeg <= 14549:
        fail("camera_node.mjpeg_port = %s is inside the aircraft's MAVLink "
             "range (1454x). MAVProxy binds those; see the port table in the "
             "README." % mjpeg)
    endpoint = get(cfg, "telemetry_bridge", "mav_endpoint")
    if endpoint and not (endpoint.startswith("udp") or endpoint.startswith("tcp")):
        fail("telemetry_bridge.mav_endpoint = %r is not udp/tcp. MAVProxy is "
             "the sole Pixhawk owner; this node consumes its rebroadcast and "
             "refuses a serial device at startup." % endpoint)
    if endpoint and ":1455" in endpoint:
        fail("mav_endpoint %r uses the ASV's port range (1455x). Both vehicles "
             "share one subnet; this aircraft is on 1454x. See the port table "
             "in the README." % endpoint)
    sysid = get(cfg, "telemetry_bridge", "mav_source_system")
    if sysid == 255:
        fail("mav_source_system = 255 collides with MAVProxy's default. The "
             "geofence upload's MISSION_REQUEST_INT replies become ambiguous "
             "between us and MAVProxy.")


def check_host(cfg):
    host = get(cfg, "ocs_client", "ocs_host")
    if host in ("127.0.0.1", "localhost"):
        fail("ocs_client.ocs_host = %r is THIS Jetson. The OCS is a separate "
             "laptop on the team subnet (192.168.8.107); the connect will fail "
             "forever with a retry every 5 s." % host)
    if get(cfg, "ocs_client", "fake_telemetry"):
        note("fake_telemetry is TRUE — ocs_client will publish INVENTED "
             "positions. Never leave this set for a scored run.")
    if get(cfg, "ground_station", "bind_host") in ("127.0.0.1", "localhost"):
        fail("ground_station.bind_host is loopback — no laptop can reach the "
             "page. Use 0.0.0.0.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=DEFAULT_PARAMS)
    ap.add_argument("--bridge-toml", default=DEFAULT_BRIDGE_TOML,
                    help="the OCS repo's bridge.toml, for the geofence "
                         "cross-check. Skipped when absent.")
    args = ap.parse_args()

    import yaml
    with open(args.params, encoding="utf-8") as f:
        raw = f.read()
    check_format_rules(raw)
    try:
        cfg = yaml.safe_load(raw)
    except yaml.YAMLError as e:
        print("FAIL: %s is not valid YAML: %s" % (args.params, e))
        return 1

    check_structure(cfg)
    check_pinned(cfg)
    pts = check_geofence(cfg)
    check_against_ocs(pts, args.bridge_toml)
    check_ocs_identity(cfg, args.bridge_toml)
    check_ports(cfg)
    check_host(cfg)

    print("checked %s" % args.params)
    for n in notes:
        print("  note: %s" % n)
    if problems:
        print("\n%d PROBLEM(S):\n" % len(problems))
        for p in problems:
            print("  * %s\n" % p)
        return 1
    print("  OK — %d sections, all pinned values agree" % len(cfg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
