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

**RX/TX (Crusader and the ground laptop, over the RFD900ux mesh)** — MAVLink
`TUNNEL`, three vendor payload types, all in `uav_common/boat_link.py`. No field
names go on the air: the order of the bytes is the meaning.

| Packet | From | When | Bytes |
|---|---|---|---|
| POSITIONS | Ekko | once per buoy, when its light is first decided; re-sent only if the boat has not acknowledged it | 1 + 11 per buoy (slot, id, lat, lon) |
| LIGHTS | Ekko | every `boat_report_hz` | 2 (the confirmed gate or exit) + 1 per buoy (slot in 5 bits, light in 3) — 12 for Task 1 |
| BOAT | Crusader | every second | 14: lat, lon, activity, target slot, and a 32-bit mask of the positions it holds — the acknowledgement |

Buoys travel as **radio slots 1–31**, not tracker ids: `buoy_mapper` numbers every
track it ever starts, false ones included, so ids climb past what 5 bits hold. A
buoy's POSITIONS record carries its real id once, so every receiver still calls
it B7. LIGHTS go out whole every period: a lost packet costs a second, never a
wrong light.

**The confirmation is the one thing the boat cannot work out for itself**: the
gate (or exit) Ekko is over and has just seen. A buoy the aircraft has not
reached yet is missing from the map, and a light mapped a minute ago is not a
light seen now. It comes from `confirmed` on `/uav/search/status`; two seconds of
silence from `search_node` sends nothing confirmed. Nothing else goes to the boat.

Ekko addresses these to **everyone** (target 0), so the autopilot forwards them
out every link and the ground laptop on the mesh hears them even before a boat
has said hello. Boat reports are read only from `boat_sysid` (2); `boat_sysid 0`
turns the link off. We are **sysid 200** (`mav_source_system`). The path is
**through the autopilot** — this node talks to MAVProxy on 14541, MAVProxy to the
USB, the autopilot to the telemetry port the radio is on — so the boat must send
heartbeats, or nothing it sends back is routed to us.

Verified in SITL with a stand-in boat on SITL's own SERIAL2 (UDP 14555 in
`sim_search`); SERIAL1 on 14556 is free for the team-laptop tool:

    python3 tools/scripts/fake_crusader.py --udp udpin:0.0.0.0:14556

and on the mesh, the same tool on a laptop with `--port COM5`. `tools/scripts/
rfd_setup.py` reads a radio and sets it up for the mesh (Multipoint firmware,
920–925 MHz, the laptop as master).

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
