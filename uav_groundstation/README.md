# `uav_groundstation` — the operator's page, and the OCS heartbeat

```bash
ros2 run uav_groundstation ground_station    # then http://<JETSON_IP>:8090
```
```bash
ros2 run uav_groundstation ocs_client
```

Both are started by systemd in normal operation. **Neither has flown.**

## Why this is its own package

It is the one package allowed to **know about** every other — it names their
nodes in order to launch them — while **importing** none. It depends on `rclpy`,
`uav_msgs` and `uav_common` and nothing else, which keeps it off the dependency
graph of the code it starts.

## `ground_station` — five tabs

| Tab | Shows | Can do |
|---|---|---|
| Nodes | every registry node, running or not, from the ROS graph **and** `/proc` | start anything; stop what is not protected |
| Telemetry | lat/lon, three altitudes, climb, speed, heading, roll/pitch/yaw, mode, armed, landed state, OCS link | — |
| Map | aircraft, trail, **the geofence**, inside/outside, altitude | pan, zoom, follow, clear trail |
| Logs | every node's `/rosout`, filterable by level and node | clear |
| System | CPU, temp, memory, disk, uptime, **workspace mount** | shut down / reboot the host (gated) |

### The two rules it holds

**The MAVLink gateway cannot be stopped from here.** Stopping `telemetry_bridge`
blinds the OCS heartbeat, the RC-override gate and the geofence uploader at once,
and WiFi is never a control path for that. Starting it is allowed — that can only
move the aircraft toward observable.

**Power is locked while armed**, and while armed is *unknown*: a missing
`FcuStatus` usually means the bridge is down, which is not evidence the aircraft
is safe to reboot. Then it needs the hostname typed.

Both are re-checked **server-side on every request**, because anyone can edit
JavaScript or curl the endpoint. Verified by calling them directly
(`tools/bench/bench_gcs.py`), not by clicking — stop of the gateway is refused
with its reason, power while armed is refused, and a GET to an action endpoint
404s, because a GET that stops a node can be fired by a link preview or a
bookmark sync.

### Presence comes from the graph *and* `/proc`

Both nodes are normally started by **systemd**, so this process has no `Popen`
handle for either and would otherwise report them down. The graph sees ROS nodes
across the DDS domain; `/proc` sees processes whatever started them. A thing is
running if either says so. Matching is on **whole path components** — a substring
test passes every casual check and then reports the wrong node on the day it
matters.

Stopping differs by owner: our own children get SIGTERM to the process **group**
(`ros2 run` execs the node as a child, so signalling only the parent orphans it);
a foreign process gets SIGTERM **by PID**, because a node started by systemd
shares its group with the whole unit.

### The map draws the uploaded fence

Not a copy — the same `geofence` parameter `telemetry_bridge` sends to the
autopilot. Two sources would drift silently, and the drift would only show as an
unexplained fence breach. The map is a **readout**; the autopilot enforces.

The map's origin is the fence centroid, not the first GPS fix, so the polygon
does not jump when GPS arrives and two sessions draw the same picture.

### Power is a file, watched by the host

A container has no init to ask, so `shutdown` inside it fails or does worse. The
alternative — `--privileged` with the host PID namespace — buys one button by
granting every process in the container permanent root. Instead the node writes
`<ws>/logs/shutdown.request` into the workspace bind mount, and
`tools/systemd/uav-shutdown.path` runs a oneshot that deletes the file and calls
`systemctl poweroff`. The privileged surface is a set of two filenames; nothing
written is ever read back, parsed, or executed.

This replaced a Unix-socket helper on 2026-08-24. The socket needed its own bind
mount, and a bind mount of a *file* pins an inode — so every helper restart
silently orphaned the container's end, and a socket path that did not exist at
create time made Docker invent a directory that stopped the container starting
at all. The request now rides the mount that must work anyway for this code to
be here. Borrowed from `robotx_graey_2026`, where it has flown.

The acknowledgement is the file **disappearing**: the host deletes it before
acting, so a consumed request proves systemd picked it up. A request still there
after five seconds means the `.path` unit is down, and the node says so instead
of reporting a shutdown that will never come.

`allow_power` defaults to **false**: the host-side units must be installed
first, and a button that silently does nothing is worse than one that is openly
off.

## `ocs_client` — the heartbeat

2 Hz to the OCS at `192.168.8.107:37564`. Inbound commands are republished to
`/uav/ocs_command` and **nothing else** — this node never acts.

What it says, and when it must say nothing, lives in **`heartbeat_core.py`**,
which has no ROS in it. The node's own job is deciding what is *fresh*; the
core's is deciding what is *true*. Staleness needs a clock and a subscription;
truthfulness does not.

### It will not invent

No heartbeat at all if pose is stale, autopilot status is stale, **or the flight
phase cannot be determined**. A gap shows as rising silence, which is the truth.

`flight_phase` is on that list because `rx26_ocs/rx_bridge/config.py` permits
**no** UNKNOWN for `TYPE_UAV` — the field is relayed to Garuda Robotics for
Singapore's Network Remote ID, so a guess misinforms a regulator, not a
scoreboard. Resolution order:

1. `/uav/flight_state` — the autopilot's own answer.
2. `armed AND altitude_rel > airborne_alt_m` — a real derivation, a worse answer;
   logged loudly the whole time it is in use.
3. Neither → **send nothing**.

`LANDED_STATE_UNDEFINED` falls through to the fallback rather than reading as
GROUNDED. `TAKEOFF` and `LANDING` are AIRBORNE — the aircraft is off the ground
in both, and reporting mid-takeoff as grounded is the more dangerous rounding.

### `altitude_hae_m` is not the altitude you have

`GLOBAL_POSITION_INT.alt` is above **mean sea level**; the report wants above the
**WGS84 ellipsoid**. `geoid_separation_m` is a venue constant (−24.6 m at
Sarasota; Singapore differs) and is silently wrong by tens of metres until
someone checks it against a known field elevation. The Telemetry tab shows the
sum so that check is possible without reading code.

### `ocs_link.py` is duplicated on purpose

`rx26_ocs/rx_bridge/framing.py` carries the same 4-byte length prefix and names
this file as its counterpart. A ROS package must not import from that repo, so
twenty lines are copied — and **a change in one must land in the other in the
same commit.** A mismatched length prefix is indistinguishable from a corrupt
payload, so the failure is silent garbage rather than an error.

It runs standalone, with no ROS and no aircraft:

```bash
python3 -m uav_groundstation.ocs_link --host 192.168.8.107 --type uav
```

## Change impact

| You changed | Re-run |
|---|---|
| `heartbeat_core.py` | `tools/bench/bench_heartbeat.py` — 24 cases, most of them "must go quiet" |
| `node_registry.py` | `tools/bench/bench_gcs.py`; it is pure, exercise it directly |
| a protection or power rule | `curl` the endpoint, not the button — the page is not where the rule lives |
| `ocs_link.py` framing | **`rx26_ocs/rx_bridge/framing.py` in the same commit** |
| `gcs_page.py` | open every tab, and check the **stale** paths: stop the bridge and confirm the banner fires |
| `system_info.py` | it must return `None`, never raise — one reader that throws blanks every tab |
| `process_manager.py` | restart a systemd-started node and confirm `pgrep -fc` returns exactly 1 |
