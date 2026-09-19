# `uav_fcu` — the autopilot gateway

```bash
ros2 launch uav_bringup core.launch.py
```

One node, `telemetry_bridge`. The single ROS-side consumer of MAVProxy's
rebroadcast and the single sender back to it. Nothing else in the workspace may
open a MAVLink connection.

**Flown on Ekko** (ArduCopter 4.7.0, CubeOrange+).

## Six jobs

**RX** — republishes `/uav/pose`, `/uav/attitude`, `/uav/fcu_status`,
`/uav/flight_state`, `/uav/rc_channels`, `/uav/battery`, `/uav/gps`,
`/uav/autonomy_drop`, each **only while fresh**, with the stamp captured at
receipt. Also reads eight autopilot parameters back (`PARAM_REQUEST_READ`,
read-only) and publishes them latched on `/uav/fcu_params`, so the ground
station's battery and fence checks — and the buoy search — use the autopilot's
real thresholds instead of a copy in our YAML.

**RX (the fence the autopilot holds)** — reads the fence back over the mission
protocol at startup, every 15 s while disarmed (so a fence drawn in QGC shows
up), on each arm, and every 30 s in the air, and publishes it latched on
`/uav/fence`. It keeps reading while armed on purpose: the search refuses to
start on a fence older than 90 s, and a single read taken at arming that failed
used to block the search for the whole flight. Valid only
when the autopilot holds exactly one inclusion polygon and no exclusion zones or
circles; otherwise `problem` says what is there. On its own thread. Mission
frames are only taken if addressed to this node's sysid and of type FENCE, so
QGC fetching its own copy is never read as our answer.

**TX (autonomy)** — the only sanctioned RC-override path, gated by the
autonomy-drop latch. Nothing publishes to `/uav/rc_override` yet; the enforcement
point exists before anything needs it.

**TX (guided targets, for the buoy search)** — `/uav/guided_target`
(`uav_msgs/GuidedTarget`) becomes `SET_POSITION_TARGET_GLOBAL_INT` at the
target's speed (`MAV_CMD_DO_CHANGE_SPEED`), and `/uav/rtl_from_guided`
(`std_srvs/Trigger`) a mode change to RTL. **Both refused unless the autopilot
freshly reports GUIDED and armed**; a target must also be under a second old,
inside the fence read back above, at least 2 m up and no higher than
`FENCE_ALT_MAX - FENCE_MARGIN`. The rules are `uav_common/guided_gate.py`.
An unchanged target is only re-sent every 5 s or after a fresh entry into GUIDED,
because re-sending restarts the autopilot's leg.

GUIDED is the pilot's switch. The autopilot ignores position targets in every
other mode, so taking control back never depends on this node.

**TX (geofence)** — `/uav/fence_upload` (`std_srvs/Trigger`) uploads the
configured polygon and verifies the readback.

**RX/TX (Crusader, over the RFD900ux on the telemetry port)** — MAVLink
`TUNNEL`, two vendor payload types (`uav_common/boat_link.py`): the boat sends
where it is and one byte for what it is doing (`/uav/boat`), and this node sends
the WHOLE buoy map back at `boat_report_hz`, never deltas — one packet resyncs
the boat completely, so a lost packet costs a second of staleness instead of a
buoy the boat never hears about. Ten bytes per buoy, 12 buoys to a 128-byte
payload, behind a count byte and **two confirmed ids**.

Those ids are the one thing the boat cannot work out for itself: what Ekko has
just CONFIRMED for it — its next gate (red, green), the exit (exit, 0), or
nothing (0, 0). A buoy the aircraft has not reached yet is missing from the map,
and a light mapped a minute ago is not a light seen now, so a boat steering by
the map alone drives a passage nobody has just looked at. The ids come from
`confirmed` on `/uav/search/status`; two seconds of silence from `search_node`
sends nothing confirmed, because a search that is not reporting is not a search
that is still checking. Nothing else goes to the boat.

The path is **through the autopilot**, not a second link of our own: this node
talks to MAVProxy on 14541, MAVProxy to the USB, and the autopilot routes a
message addressed to the boat's sysid out the telemetry port the radio is on —
which is why the boat must send heartbeats, or the autopilot has no route to it.
We are **sysid 200** (`mav_source_system`) and address the boat at `boat_sysid`
(42: Crusader's `rxl_link_node`, not its autopilot, which is not on the radio).
Verified end to end in SITL on 16 Sep, using SITL's own SERIAL2 as the
radio port — the same routing the RFD900ux relies on. In `sim_search` that port
is UDP 14555 and a second one, SERIAL1 on 14556, is free for the stand-in boat:

    python3 tools/scripts/fake_crusader.py --udp udpin:0.0.0.0:14556 --sysid 42

and at the park, the same tool on a laptop with `--port COM5`.

**The radio, as a record.** Every frame that crosses it — the map out, the
boat's packets in, and any other system's frames in — is also published on
`/uav/radio/traffic` (`uav_msgs/RadioFrame`) for the ground station's Radio tab,
after the fact; nothing acts on that topic. `/uav/radio/send_test` puts one
text TUNNEL (`boat_link.PAYLOAD_TEST`, `0x80FE`) on the air, addressed to the
boat, that neither end acts on.

## There is no disarm path, deliberately

The ASV's version carries a force-disarm (`MAV_CMD_COMPONENT_ARM_DISARM` with the
21196 force magic) driven by RC loss. **Do not port it.** On a boat that stops
the thrusters and the hull floats; on a multirotor it stops the motors and the
aircraft falls. RC loss belongs to the autopilot's own failsafe params, which
work without a companion computer being alive to have an opinion.

## The geofence dialog

Runs on a **service callback thread**, not the RX thread — it blocks on each
autopilot reply, and the RX thread keeps every other stream alive. `MISSION_*`
frames are routed from the RX loop into a `queue.Queue` the transport drains.

`mav_source_system` is **200**, not pymavlink's default 255, because MAVProxy is
also 255. While the node only reads, the sysid is cosmetic; the moment it runs a
mission dialog it decides whether `MISSION_REQUEST_INT` replies are addressed to
us or to MAVProxy.

Refuses while armed. Never sets `FENCE_ENABLE` — enabling a fence is a deliberate
act at a ground station.

## Autopilot params this node depends on

Set in QGC on `MAV1_*` (USB = SERIAL0; `SR0_*` before ArduPilot 4.7). MAVProxy
runs `--streamrate=-1` precisely so it does not stomp them.

| Param | Why |
|---|---|
| `MAV1_POSITION` > 0 | `GLOBAL_POSITION_INT` → `/uav/pose` |
| `MAV1_EXTRA1` = 30 | `ATTITUDE` → `/uav/attitude` |
| `FENCE_TYPE` polygon bit | or the upload is NACKed, and the search will not start |
| `FENCE_ALT_MAX - FENCE_MARGIN` ≥ the search altitude | or every search target is refused (12 − 2 = 10 m for a 10 m search) |

`/uav/flight_state` needs no parameter. `EXTENDED_SYS_STATE` is in **no** stream group on any firmware, so no parameter turns it on: `telemetry_bridge` requests it itself with `SET_MESSAGE_INTERVAL` and re-asks every 30 s, because an autopilot reboot discards the request.

## Change impact

| You changed | Re-run |
|---|---|
| the fence path | `tools/bench/bench_fence.py`, then upload to a real Pixhawk and diff the polygon in QGC **vertex for vertex** |
| a published topic | `ocs_client` and `ground_station` both subscribe; `grep` before renaming |
| the staleness rule | stop MAVProxy and confirm **one** loud line per stream and that republishing stops |
| `mav_source_system` | the fence upload; the symptom of a collision is a timeout, not an error |
| the guided-target gate | `tools/bench/bench_search.py`, then the SITL search test before anything flies |
