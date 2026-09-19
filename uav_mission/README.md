# `uav_mission` — the aircraft doing a task on its own

```bash
ros2 run uav_mission search_node      # or Start on the ground station's Nodes tab
```

One node, `search_node`: the **Task 1 buoy search** (Advanced tier). No systemd
unit; start it from the Nodes tab with `detector_node` and `buoy_mapper`.

**Tested in ArduCopter 4.7.0 SITL** with the real `telemetry_bridge`,
`ground_station` and this node (a stand-in for the camera and mapper). Not yet
flown.

## What it does

1. Sweeps the area inside **the fence the autopilot holds** (read back by
   `telemetry_bridge`, so at practice it is whatever was drawn in QGC) at
   10 m, in lines 10 m apart, keeping every point it flies to 2 m inside the
   fence. **The nose points where it is going** (Chris, 19 Sep), and freezes
   within 3 m of wherever that is (`FACE_HOLD_M`), so it never turns while
   hovering over a buoy or the boat's gate. It used to hold one heading per
   pass and fly the return lines backwards so the gimbal never swung; the cost
   now is a turn at each line end, while the gimbal catches up and
   `buoy_mapper` refuses the frames (`max_gimbal_yaw_rate_dps`).
2. The moment the buoy map has an **UNKNOWN** buoy inside the fence, it flies
   over it and hovers until `buoy_mapper` locks its state. Unknown buoys within
   4 m of each other (a 3 m gate) share one hover.
3. Confirmed: it flies back to where it left the line and carries on.
4. **Not confirmed after 10 s overhead:** given up on for the rest of that pass.
5. A pass that ends short of the count starts another with the lines turned
   90°; every other pair of passes also shifts them half a line.
6. **Once the ground station's "buoys to find" are confirmed inside the fence,
   it asks for RTL.**

## Disruptive

**Crusader moves only to what Ekko confirms.** `confirmed` on
`/uav/search/status` (and in the radio packet) is the boat's next gate as
`[red, green]`, or the exit as `[id]`, and it is set only when all of these are
true on that tick:

- the search is escorting (a pilot who takes the aircraft back takes the
  confirmation with it, on the same tick);
- the aircraft is within `hover_radius_m` of the gate (or the exit);
- every light in it was actually seen in the last `FRESH_S` (4 s) and still
  reads right — a buoy with no age at all is not fresh;
- nothing between the boat and that gate is still UNKNOWN. If something is, the
  aircraft reads it first; one that will not settle is given up on after
  `give_up_s`, like any other.

`watching` is only where the aircraft is going. The difference is the whole
point: the boat once waited while Ekko hovered over its gate (nothing on the
wire said "go"), and once ran the passage while Ekko was still mapping (it
trusted the map). The confirmation is what closes both.

The stand-in boat in `sim_search` laps the entry buoy once from its gate side,
stops there, and after every gate stops again until the next confirmation. tier: staying with the boat

Set the tier on the Map tab. **Advanced** is the above: map it, RTL. In
**Disruptive** the passage may change while Crusader transits, and only the UAV
can see it (handbook 3.3.2), so once the field is mapped the aircraft does not
go home:

1. It hovers over **the boat's next element** — its next gate, then the exit —
   using the boat's position over the radio (`/uav/boat`).
2. A gate **holds** while both lights still read flashing red and green on a
   fresh look. The buoys never move; only the lights change.
3. When one changes it **re-reads known buoys**: pairable ones first, ahead of
   the boat first, and among those the earliest along the passage — the first
   one the boat will reach. It stops at the first valid new gate.
4. **The 4 m rule.** Gates are 3 m apart, so a buoy with no neighbour within 4 m
   can never be half of a gate whatever its light does. Those are LONE and are
   checked only when everything pairable has been. That is what keeps the
   re-check quick enough to beat the boat.

The drone sends the boat buoy ids, positions and states — nothing else; the boat
plans from that. While no gate ahead is confirmed, the boat holds.

**Disruptive also needs `buoy_mapper.lock_state` false**, or a state decided once
never changes again — and the mapper needs to decide from RECENT samples, which
it does not do yet (see `buoy_tracker.evidence`: it weighs every sample ever
taken, so a buoy watched as red for a minute cannot flip quickly).

## Who is flying

- **Only the pilot starts it**: on a *change* into GUIDED (the SC switch), while
  switched on from the page and ready. Being in GUIDED already is not a start,
  so switching it on from the page, or restarting the node in the air, never
  moves the aircraft.
- **Any other mode pauses it** — SB to Loiter or Brake, SC off, an RTL — with
  its place kept. SC off and on again resumes it.
- **Switched off from the page** while in GUIDED: it holds position, and needs
  the SC switch again to resume.
- **Buoy map lost** while flying: it holds, and resumes by itself when the map
  returns.
- **The node dies**: the autopilot finishes the leg it was on (inside the fence,
  at 10 m) and holds there in GUIDED until the pilot flips a switch.
- The only mode it asks for is RTL, and only from GUIDED.

`telemetry_bridge` checks every target again before it reaches the autopilot —
GUIDED and armed, recent, inside the fence, under `FENCE_ALT_MAX - FENCE_MARGIN`
— so a bug here can ask for a bad point but not fly one.

## What it needs before it will start

The page's Map tab and the checklist say which of these is missing, in words.

| | |
|---|---|
| Autopilot | armed, in the air, GPS position |
| Fence | one polygon, no exclusion zones or circles, **no inward corners** (a vertex within 0.75 m of straight is fine), read from the autopilot in the last 90 s |
| `FENCE_ENABLE` 1, `FENCE_TYPE` with the polygon bit | the fence must actually be enforced |
| `FENCE_ALT_MAX - FENCE_MARGIN` ≥ 10 m | with the altitude bit set. Saturday setup: 12 and 2 |
| Buoy map | `detector_node` and `buoy_mapper` running |
| The count | not already met by the map. **Buoys confirmed on an earlier flight count** — clear the buoys on the Map tab before a fresh run |

The convex-fence rule is deliberate: inside a convex polygon every straight leg
stays inside, so there is no path planner to get wrong. See `sweep_core.py`.

## Files

| | |
|---|---|
| `sweep_core.py` | fence shrinking, sweep lines, waypoint order. Pure |
| `search_core.py` | the state machine and every rule above. Pure |
| `search_node.py` | ROS wiring only |

`tools/bench/bench_search.py` drives both cores, and the bridge's gate, with no
aircraft.

## Parameters

In `uav_params.yaml` under `search_node`. Two are set from the page and are the
only dynamic ones: `enabled`, `buoys_to_find`. The rest are read-only.
`search_alt_m` is pinned to `buoy_mapper.waypoint_alt_m` by `check_config.py`.
