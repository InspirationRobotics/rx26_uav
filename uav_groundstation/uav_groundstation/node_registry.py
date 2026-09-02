"""node_registry — what the aircraft can run, and what the page may do about it.

No ROS, no subprocess, no I/O: this is the catalogue and the rules, so both can
be read in one place and exercised on a laptop. Starting things is
process_manager's job; deciding what is startable is this file's.

THE PROTECTION RULE, which is the reason this file exists at all. WiFi is a
convenience, never a safety mechanism — and a web page that can stop the
telemetry gateway inverts that. telemetry_bridge is the only thing that speaks
MAVLink: stopping it takes down the pose the OCS heartbeat is built from, the
autonomy-drop latch that gates RC overrides, and the only path by which a
geofence can be uploaded. Anyone who can reach the Jetson over WiFi could switch
all of that off, from a laptop, silently.

So the core stack is PROTECTED. The page shows whether it is up, and may start
it if it is down, but nothing served over HTTP can take it down. Stopping it is
a decision for someone at a terminal who has thought about it, which is exactly
the friction that should exist. Note the asymmetry is deliberate: starting the
gateway can only ever move the aircraft toward observable, so it needs no guard.

NO EXCLUSION RULE HERE. The ASV's registry carries an `exclusive` tag because
its two camera nodes contend for one OAK-D. Nothing on this airframe contends
for a device yet, so the field is kept (the page and process_manager both read
it) and no entry sets it. Add a tag rather than a special case when a second
consumer of one device arrives.

BOTH ENTRIES ARE ALSO STARTED BY SYSTEMD (tools/systemd/), which is why
presence has to come from the ROS graph and /proc rather than from a Popen
handle this process owns — see proc_scan.
"""
from dataclasses import dataclass

# Groups, in the order the page lists them. Ordering is not cosmetic: the stack
# reads top-down the way it is brought up, so a group above another is one you
# want running first.
GROUPS = (
    ("core", "Core",
     "The MAVLink gateway. Protected: startable here, not stoppable."),
    ("comms", "Comms",
     "The link to the Operator Control Station."),
    ("perception", "Perception",
     "The gimbal camera. Serves its own video port; see the Camera tab."),
)


@dataclass(frozen=True)
class NodeSpec:
    """One launchable thing.

    kind:
      "ros"    -> ros2 run <package> <executable>. Presence is detected from the
                  ROS graph AND /proc, so a node started by core.launch.py, by
                  its systemd unit, or by hand in another terminal shows as
                  running here too.
      "script" -> python3 <tools_dir>/<executable>. A bench tool, not a ROS node
                  we can name in the graph, so it is only visible while THIS
                  process is its parent. Nothing uses it yet; kept so
                  process_manager's two branches stay honest.
    """
    name: str                      # ROS node name (kind="ros") or a unique id
    label: str
    package: str
    executable: str
    group: str
    kind: str = "ros"
    protected: bool = False        # startable here, never stoppable here
    exclusive: str = ""            # tag; two nodes sharing one cannot co-run
    port: int = 0                  # serves a browser view on this port, if any
    stream_path: str = ""          # MJPEG path on that port, if any
    note: str = ""


REGISTRY = (
    NodeSpec("telemetry_bridge", "telemetry_bridge", "uav_fcu",
             "telemetry_bridge", "core", protected=True,
             note="the only thing that speaks MAVLink; owns the geofence "
                  "upload and the RC-override gate"),
    NodeSpec("ocs_client", "ocs_client", "uav_groundstation",
             "ocs_client", "comms",
             note="2 Hz heartbeat to the OCS at 192.168.8.107:37564"),
    # port/stream_path are what the Camera tab points its <img> at — the video
    # is served by THIS node on its own socket, not proxied through :8090.
    # Proxying would put megabytes of MJPEG through the ground station's
    # single-threaded snapshot path and make a stalled camera look like a
    # stalled ground station.
    #
    # exclusive="camera" is set now, while nothing else wants the A8 mini,
    # because the moment something does — a recorder, a detector that opens its
    # own RTSP session — the failure is a second GStreamer client fighting for
    # the stream, and that reads as a flaky camera rather than a design mistake.
    NodeSpec("camera_node", "camera_node", "uav_camera",
             "camera_node", "perception", exclusive="camera",
             port=8091, stream_path="/stream.mjpg",
             note="owns the A8 mini: RTSP in, MJPEG out on :8091, records to "
                  "the Jetson and the camera's SD card, holds the gimbal at "
                  "nadir"),
)

BY_NAME = {n.name: n for n in REGISTRY}

# One-click profiles. A profile is a claim about what a session needs, and
# naming them here rather than in the page keeps that claim reviewable.
PROFILES = {
    # camera_node is in the flight profile because an unrecorded sortie is a
    # sortie flown twice, and the recordings are the training data the
    # perception work depends on. It is not in the bench profile: that one runs
    # without a Pixhawk, and usually without a camera too.
    "flight": ("Flight profile",
               ("telemetry_bridge", "ocs_client", "camera_node")),
    "bench": ("Bench profile", ("telemetry_bridge",)),
}


def conflicts(name: str, running) -> list:
    """Names that must stop before `name` may start.

    `running` is any iterable of currently-running node names. Returns the
    subset that shares an `exclusive` tag with the requested node. Nothing sets
    that tag today, so this returns empty — kept because the alternative is
    discovering a device conflict from a driver traceback the first time two
    nodes want one sensor.
    """
    spec = BY_NAME.get(name)
    if spec is None or not spec.exclusive:
        return []
    return [other for other in running
            if other != name
            and BY_NAME.get(other)
            and BY_NAME[other].exclusive == spec.exclusive]


def may_stop(name: str) -> tuple:
    """(allowed, reason). The single gate every stop request passes through.

    Returning a reason rather than a bare False is what lets the page say why a
    button is absent instead of just not having one — an unexplained missing
    control reads as a bug and invites someone to go around it.
    """
    spec = BY_NAME.get(name)
    if spec is None:
        return False, "unknown node %r" % name
    if spec.protected:
        return False, (
            "%s is the MAVLink gateway and cannot be stopped from a web page. "
            "Stopping it blinds the OCS heartbeat, the RC-override gate and "
            "the geofence uploader at once, and WiFi is never a control path "
            "for that. Stop it from a terminal, or with "
            "`systemctl stop uav-groundstation`'s sibling unit, if you really "
            "mean to." % name)
    return True, ""


def tab_source(sources, running, serving=None):
    """Which candidate is filling a viewer tab: (source, starting).

    `running` is the set of node names that exist as processes. `serving` is the
    subset whose HTTP port is actually accepting connections — pass None to skip
    that distinction.

    THE DISTINCTION IS THE POINT, and it is why this survives into a repo with
    no viewers yet. A process appears in the table within a second of being
    started and may spend far longer before its server binds. A tab that trusted
    "running" alone points an iframe at a port nothing is listening on, gets
    connection-refused, and — because the page will not reload a stream it
    thinks is already correct — stays on that error page permanently.

    Returns:
      (name, False)  a viewer is up and serving; show it.
      (name, True)   the process exists but its port is not open YET.
      (None, False)  nothing is running; offer to start it.
    """
    for name in sources:
        if name not in running:
            continue
        if serving is None or name in serving:
            return name, False
        return name, True
    return None, False
