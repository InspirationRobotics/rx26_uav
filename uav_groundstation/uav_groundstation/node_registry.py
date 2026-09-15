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

ONE EXCLUSION TAG. The ASV's registry carries an `exclusive` tag because its two
camera nodes contend for one OAK-D. Here only camera_node sets one ("camera"),
ahead of the day a second consumer of the A8 mini arrives; detector_node reads
camera_node's MJPEG instead of the camera, so it does not need the tag. Add a tag
rather than a special case when a second consumer of one device arrives.

SOME ENTRIES ARE SUPERVISED BY SYSTEMD (tools/systemd/): telemetry_bridge,
ocs_client and camera_node each have a unit with Restart=on-failure. That is
why presence has to come from the ROS graph and /proc rather than from a Popen
handle this process owns — see proc_scan — and why those nodes get RESTART
rather than STOP. Killing one brings it straight back after the unit's
RestartSec, so a "stop" button for it is a restart button wearing the wrong
label, and a "start" pressed in the gap before it returns runs a second copy.
bench_gcs checks `unit` against the unit files, so the label cannot drift.
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
     "The gimbal camera, the buoy detector and the buoy mapper. Video on the "
     "Camera tab; the buoy map on the Map tab. To map: start all three."),
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
    unit: str = ""                 # systemd unit that restarts it, if any


REGISTRY = (
    NodeSpec("telemetry_bridge", "telemetry_bridge", "uav_fcu",
             "telemetry_bridge", "core", protected=True,
             unit="uav-telemetry-bridge",
             note="the only thing that speaks MAVLink; owns the geofence "
                  "upload and the RC-override gate"),
    NodeSpec("ocs_client", "ocs_client", "uav_groundstation",
             "ocs_client", "comms", unit="uav-ocs-client",
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
             port=8091, stream_path="/stream.mjpg", unit="uav-camera",
             note="owns the A8 mini: RTSP in, MJPEG out on :8091, records to "
                  "the Jetson and the camera's SD card, holds the gimbal at "
                  "nadir"),
    # NOT tagged exclusive="camera", and that is the whole design. This node
    # consumes camera_node's MJPEG rather than opening the A8 mini, so the two
    # co-run by construction -- which is what the exclusive tag on camera_node
    # exists to protect. Stoppable, unprotected: nothing safety-related reads
    # it, and an operator who wants the GPU back should be able to take it.
    NodeSpec("detector_node", "detector_node", "uav_perception",
             "detector_node", "perception",
             port=8092, stream_path="/stream.mjpg",
             note="runs the colour-buoy model on camera_node's stream, serves "
                  "an annotated view on :8092, publishes each frame's boxes "
                  "with its pose; writes nothing"),
    # Consumes detector_node's topic, so it needs that node running to map
    # anything, but holds no device and co-runs with everything. port is its
    # GET-only download server, not a video stream, so stream_path stays empty.
    NodeSpec("buoy_mapper", "buoy_mapper", "uav_perception",
             "buoy_mapper", "perception", port=8093,
             note="turns detections into the Task 1 buoy map: positions, and "
                  "each buoy's state decided over 4 s of full-view watching. "
                  "Map tab; downloads on :8093"),
)

BY_NAME = {n.name: n for n in REGISTRY}

# One-click profiles. A profile is a claim about what a session needs, and
# naming them here rather than in the page keeps that claim reviewable.
PROFILES = {
    # camera_node is in the flight profile because an unrecorded sortie is a
    # sortie flown twice, and the recordings are the training data the
    # perception work depends on. It is not in the bench profile: that one runs
    # without a Pixhawk, and usually without a camera too.
    # detector_node is NOT in the flight profile. It is a view, not a
    # requirement: a sortie with no detector still produces the recording and
    # the stills, and starting it by default would put inference on the GPU for
    # every flight whether anyone is watching or not.
    "flight": ("Flight profile",
               ("telemetry_bridge", "ocs_client", "camera_node")),
    # Everything a buoy-mapping sortie needs. Separate from "flight" on purpose:
    # it puts inference on the GPU for the whole flight, which a data-collection
    # sortie should not pay for.
    "mapping": ("Mapping profile",
                ("telemetry_bridge", "ocs_client", "camera_node",
                 "detector_node", "buoy_mapper")),
    "bench": ("Bench profile", ("telemetry_bridge",)),
}


def conflicts(name: str, running) -> list:
    """Names that must stop before `name` may start.

    `running` is any iterable of currently-running node names. Returns the
    subset that shares an `exclusive` tag with the requested node. Only
    camera_node sets a tag today and nothing shares it, so in practice this
    returns empty — kept because the alternative is discovering a device
    conflict from a driver traceback the first time two nodes want one sensor.
    """
    spec = BY_NAME.get(name)
    if spec is None or not spec.exclusive:
        return []
    return [other for other in running
            if other != name
            and BY_NAME.get(other)
            and BY_NAME[other].exclusive == spec.exclusive]


#: How long after a restart the page refuses START for that node. The unit waits
#: RestartSec (5 s) and `docker exec` takes a moment more before the node is back
#: in /proc; a START inside that gap runs a SECOND copy beside the one systemd is
#: about to bring up. That has already happened with camera_node, and two
#: camera_nodes fight over one RTSP stream. Long enough to cover the respawn,
#: short enough that a unit which has given up (StartLimitBurst) is offered a
#: start again within the minute.
RESTART_GRACE_S = 20.0


def stop_verb(name: str) -> str:
    """"restart" for a node a systemd unit brings back, "stop" for the rest."""
    spec = BY_NAME.get(name)
    return "restart" if spec is not None and spec.unit else "stop"


def may_stop(name: str, verb: str = "stop") -> tuple:
    """(allowed, reason). The single gate every stop AND restart passes through.

    `verb` is what the caller asked for. It must be the verb this node actually
    has: a supervised node cannot be stopped from here (its unit would bring it
    straight back), and an unsupervised one cannot be "restarted" by killing it
    (nothing would bring it back).

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
            "for that. Stop it from a terminal with `sudo systemctl stop %s` "
            "if you really mean to." % (name, spec.unit or "its unit"))
    if verb != stop_verb(name):
        if spec.unit:
            return False, (
                "%s is supervised by %s, which brings it back within seconds "
                "of stopping, so from here it can only be restarted. To really "
                "stop it: `sudo systemctl stop %s` in a terminal."
                % (name, spec.unit, spec.unit))
        return False, ("%s has no systemd unit, so nothing would bring it "
                       "back: stop it, then start it." % name)
    return True, ""


def restarting(name: str, running, restarted_at, now: float) -> bool:
    """Is this node down only because it was restarted moments ago?

    True while a supervised node is absent and still inside RESTART_GRACE_S of
    its restart — the gap in which its unit is about to bring it back.
    """
    spec = BY_NAME.get(name)
    return bool(spec is not None and spec.unit and name not in running
                and restarted_at is not None
                and now - restarted_at < RESTART_GRACE_S)


def start_refusal(name: str, running, restarted_at=None, now: float = 0.0) -> str:
    """Why `name` may not start right now, or "" if it may.

    The single gate every start request passes through, kept here rather than
    in the node so bench_gcs exercises the rule itself, not a copy of it.
    """
    spec = BY_NAME.get(name)
    if spec is None:
        return "unknown node %r" % name
    if name in running:
        return "%s is already running" % name
    if restarting(name, running, restarted_at, now):
        return ("%s was restarted %.0f s ago and %s is bringing it back. "
                "Starting it now would run a second copy beside that one; "
                "wait for it to reappear." % (name, now - restarted_at,
                                              spec.unit))
    clash = conflicts(name, running)
    if clash:
        return ("%s cannot start while %s is running (they contend for one "
                "device)" % (name, ", ".join(clash)))
    return ""


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
