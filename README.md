# RobotX 2026 — UAV Software Package (v0.1)

Team Inspiration's codebase for the aircraft in the 2026 RobotX system-of-systems.
A Pixhawk-based multirotor with a Jetson Orin Nano companion computer running
ROS 2 Humble in the `uav` Docker container, reporting to the team's Operator
Control Station over field WiFi.

**What this is:** the minimum that makes the UAV a real member of the fleet —
stable device names, the single MAVLink gateway, a geofence the autopilot
enforces, an operator page, and the 2 Hz heartbeat the OCS is already waiting
for. **What it is not:** perception, a world model, missions, or anything that
has flown. Nothing in this repo has been in the air.

The fleet's other two vehicles are built:
[rx26_asv](https://github.com/InspirationRobotics/rx26_asv) (the boat, whose
patterns this repo follows) and
[rx26_ocs](https://github.com/InspirationRobotics/rx26_ocs) (the shore laptop,
which already lists us as `UAV1`).

## Packages

Five, laid out the way the ASV's eight are: a message package that depends on
almost nothing, a shared library under everything, one node that owns the
autopilot, one that faces the operator, and a data-only package on top.

| Package | Contains | State |
|---|---|---|
| [`uav_msgs`](uav_msgs/) | Message definitions. Depends on nothing but `std_msgs` | 5 msgs |
| [`uav_common`](uav_common/) | Params loader, node lifecycle, stream cache, drop latch, geodesy, the geofence protocol. No nodes | library |
| [`uav_fcu`](uav_fcu/) | `telemetry_bridge` — the only thing that speaks MAVLink. Also uploads the geofence | **untested in flight** |
| [`uav_groundstation`](uav_groundstation/) | `ground_station` (one web page on `:8090`) and `ocs_client` (the OCS heartbeat) | **untested in flight** |
| [`uav_bringup`](uav_bringup/) | Launch file + the params YAML. Ships no code; build entry point | — |

**A new package must be added to `uav_bringup/package.xml`'s exec_depends** or it
silently stops being built by `--packages-up-to uav_bringup`.

## Ports — read this before touching a `--out` line

The boat and this aircraft are on **one subnet**. If both broadcast MAVLink to
the same port, a GCS bound to it receives both interleaved and shows one
vehicle's telemetry under the other's name — which looks like wild GPS noise,
not like two vehicles.

| | ASV | **UAV** |
|---|---|---|
| MAVProxy → ROS bridge (loopback) | 14551 | **14541** |
| MAVProxy → GCS (loopback + subnet broadcast) | 14550 | **14540** |
| Ground station HTTP | 8090 | 8090 (different machine) |
| OCS vehicle link | 37564 | 37564 (outbound to `192.168.8.107`) |

**The boat lives on 1455x, the aircraft on 1454x.**
`tools/scripts/check_config.py` fails if `mav_endpoint` drifts back.

## The stack

| Node | Package | Role |
|---|---|---|
| `telemetry_bridge` | `uav_fcu` | THE single consumer of MAVProxy's rebroadcast and THE single sender back to it; uploads the geofence |
| `ground_station` | `uav_groundstation` | one web page: nodes, telemetry, map, logs, system |
| `ocs_client` | `uav_groundstation` | 2 Hz heartbeat to the OCS |

MAVProxy owns the Pixhawk serial link and is started by systemd outside ROS —
nothing else may open `/dev/uav-pixhawk`.

`core.launch.py` starts **only** `telemetry_bridge`. The other two have their own
systemd units rather than a place in the launch, so a crash-looping display or
reporting link cannot take the telemetry gateway down with it.

## Boot chain

`sudo bash setup/install_jetson_host.sh` installs the units, in order:

```
uav-mavproxy      host    sole Pixhawk owner; 14541 -> ROS, 14540 -> GCS
uav-container     host    docker start; ROS 2 Humble
uav-groundstation host    docker exec -> ground_station     (unproven)
uav-ocs-client    host    docker exec -> ocs_client         (unproven)
uav-shutdown.path host    watches <ws>/logs/shutdown.request -> poweroff
uav-reboot.path   host    watches <ws>/logs/reboot.request   -> reboot
```

The two `.path` units are how the System tab powers the Jetson down. The
container has no init to ask, so it drops a file in the workspace bind mount it
already has and systemd acts on it — no socket, no daemon, no extra mount. The
oneshot each one triggers (`uav-shutdown.service`, `uav-reboot.service`) has no
`[Install]` section on purpose: enabling it would power the machine off at every
boot. **Never `systemctl start uav-shutdown.service`** — that *is* the shutdown.

The last two are auto-started because nobody will SSH into this Jetson between
flights. Neither has carried a run; `systemctl disable uav-groundstation
uav-ocs-client` is the way back. The failure mode is benign — `ocs_client`
retries the OCS every 5 s and logs the address it is aiming at.

### Restarting cleanly

**`docker exec` does not propagate termination.** Kill the exec client and the
process keeps running inside the container, so a naive `systemctl restart` starts
a second copy while the first still holds `:8090` — and the new one dies with an
address-in-use that reads like a crash rather than like a leftover.

Three layers handle it (`scripts/run_in_container.sh` and the unit files):
`ExecStartPre` sweeps orphans **before** starting — the one that saves you, since
it works even when the previous stop failed; the wrapper traps `SIGTERM` and
forwards the kill inward; `ExecStop` is the backstop. The kill target is the
**install-space path**, not the node name, because `ground_station` alone would
also match neighbouring names and any editor left open in the container.

Check it with `pgrep`, not with "the page loaded":

```bash
systemctl restart uav-groundstation && docker exec uav pgrep -fc install/uav_groundstation/lib/uav_groundstation/ground_station
```

must print `1`. Two means the sweep is not matching; zero means the marker is wrong.

## Working on it — the everyday loop

The **whole colcon workspace** is bind-mounted, so the repo is pulled on the host
and the container sees the same bytes:

```
HOST                                  CONTAINER
~/robotx_ws/                    <->   /root/robotx_ws/
  src/rx26_uav/   <- git pull here      src/rx26_uav/    <- appears instantly
  build/ install/ log/                  build/ install/ log/  <- written by the build
```

```bash
cd ~/robotx_ws/src/rx26_uav && git pull
```
```bash
docker exec uav bash -lc '/root/robotx_ws/src/rx26_uav/tools/scripts/rebuild.sh'
```
```bash
sudo systemctl restart uav-groundstation uav-ocs-client
```

**A `git pull` alone changes nothing that is running.** colcon installs Python
into `install/`; the node does not import from `src/`. "The change did nothing"
is almost always a skipped rebuild or a skipped restart. The System tab reports
whether the workspace really is a bind mount, because when it is not, all three
commands succeed and the behaviour still does not change.

Three things about the mount that are easy to get wrong:

- **`docker start` cannot add mounts** — they are fixed at *create* time. The
  install script creates the container when absent, and only *checks* and prints
  the fix when it exists, because `docker rm` discards anything living only
  inside it.
- **Never run `colcon build` from the host.** One `build/` shared by two
  toolchains produces artifacts that fail at node start with a missing symbol.
- `build/`, `install/` and `log/` appear on the host owned by **root**, because
  the container builds as root. That is expected, not a broken mount — `rm -rf
  build` on the host needs `sudo`.

## Troubleshooting

Everything below is a failure that actually happened during the first bring-up
on Ekko (2026-08-24). `setup/install_jetson_host.sh` now checks for most of
them up front, so a fresh Jetson should not repeat this list — but when a unit
does fail, systemd's status line names a code and nothing else, and that code is
the fastest way in.

### Start here

```bash
systemctl status uav-mavproxy uav-container uav-groundstation uav-ocs-client --no-pager
```

```bash
journalctl -u uav-mavproxy -n 40 --no-pager -o cat
```

`-o cat` matters: the default format ellipsizes long lines, and the useful half
of a script's error message is usually the half that gets cut. Running the
script by hand is even more direct, and any difference from the service run
*is* the bug — it runs as you, with your groups and your PATH:

```bash
/home/$USER/robotx_ws/src/rx26_uav/scripts/start_mavproxy.sh 192.168.8.255
```

### systemd status codes

| Status | Meaning | Cause here | Fix |
|---|---|---|---|
| `203/EXEC` | kernel could not `execve` the file | script has no `+x`. Git records the bit, but zip/scp/cloud-sync drop it | `chmod +x scripts/*.sh setup/*.sh tools/udev/*.sh tools/scripts/*.sh`, or re-run the installer (step 2 does it) |
| `127` | command not found *inside* the script | `mavproxy.py` is not on PATH | `sudo pip3 install MAVProxy` — see the PATH trap below |
| `1/FAILURE`, ~1.4 s CPU | the program ran, then failed | MAVProxy imported (Python startup is what the CPU time is) then could not **open** the autopilot — the service user is not in `dialout` | `sudo usermod -aG dialout $USER && sudo systemctl restart uav-mavproxy` |
| `1/FAILURE`, milliseconds | a guard in the script fired | it printed a reason — read the journal | as printed |
| `activating (auto-restart)` forever | `Restart=on-failure` looping | whatever the first failure was; the loop hides it | `journalctl -u <unit> -n 40 -o cat` |

**The PATH trap**, because it costs an hour: `pip3 install --user MAVProxy`
lands in `~/.local/bin`, which is **not** on systemd's default PATH
(`/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin`). It then works
perfectly when you type `mavproxy.py` in a shell and keeps failing with `127`
under systemd. Install it with `sudo` so it lands in `/usr/local/bin`.

**Group membership needs a new process, not a new login** — for the *service*.
`usermod -aG` takes effect for anything spawned afterwards, and systemd forks
fresh on restart, so the unit picks it up immediately. Your own shell does not,
until you log out and back in.

### The device

| Symptom | Cause | Fix |
|---|---|---|
| `ERROR: Pixhawk device /dev/uav-pixhawk not found` | the udev rule does not match this board. `99-uav.rules` says outright that its four Pixhawk lines are a **candidate list**, not a confirmed one | `udevadm info -a -n /dev/ttyACM0 \| grep -E 'idVendor\|idProduct' \| head -4`, add the line, `sudo bash tools/udev/install_udev.sh` |
| Works under `sudo`, fails as the service user | device is `MODE=0660 GROUP=dialout` and the user is not in `dialout` | the `usermod` above |
| `install_udev.sh` refuses with a collision | two names resolve to one tty — **this aliases the autopilot** | prune the candidate rules to the board actually aboard |
| Symlink exists but dangles | stale rule or unplugged device | replug, then `sudo udevadm trigger` |

### The container

| Symptom | Cause | Fix |
|---|---|---|
| `uav-container` loops on `status=1/FAILURE` after **milliseconds** | the service user is not in the `docker` group, so `docker start` cannot open `/var/run/docker.sock`. Works under `sudo`, fails as the unit — the installer creates the container as root | `sudo usermod -aG docker $USER && sudo systemctl restart uav-container`, or re-run the installer (step 3 does it) |
| container is down at boot **despite** `--restart unless-stopped` | `uav-container`'s `ExecStop` runs `docker stop` on every failed restart cycle, and a manual stop clears Docker's auto-start flag | fix the `ExecStart` failure first, then `docker start uav` once to re-arm it |
| `docker attach uav` hangs, shows nothing | you attached to PID 1, which is `tail -f /dev/null` — silent by design. **Ctrl+C there kills the container** and takes the nodes with it | `docker exec -it uav bash`. To escape an attach: **Ctrl+P Ctrl+Q** |
| `docker exec -it uav bash` has no `ros2` | `docker exec` does **not** run the image ENTRYPOINT, so `/ros_entrypoint.sh` never fires | rebuild the image (it now sources ROS in `.bashrc`), or `source /opt/ros/humble/setup.bash` by hand |
| `systemctl restart` leaves **two** copies | `docker exec` does not propagate termination; the old process survives and still holds `:8090` | the `ExecStartPre` sweep handles it. Verify with `docker exec uav pgrep -fc install/uav_groundstation/lib/uav_groundstation/ground_station` — must be `1` |
| container `Exited (127)`, **`docker logs` completely empty** | almost never a missing command. `dockerd` stamps 127 on a container whose task never started, and a bind mount it cannot satisfy is the usual reason — the process never ran, so there is nothing to log. An image fault would leave the shell's own error in the log | treat it as a **start** failure, not a command failure: `docker start uav` once by hand and read the daemon's error. Confirm the image is fine with `docker run --rm --entrypoint /bin/bash uav -c 'command -v tail; ls -l /ros_entrypoint.sh'` |
| System tab: "power request directory … does not exist" | `<ws>/logs` is missing, or the workspace is not bind-mounted | re-run the installer (step 6 creates it); if the mount is the problem the System tab already says so separately |
| power button reports the request was **withdrawn** after 5 s | the `.path` unit is not running, so nothing consumed the request file. The node takes it back rather than leaving a live request for the next boot to find | `systemctl status uav-shutdown.path uav-reboot.path` — `enable` is what arms them |
| Jetson powers off again immediately after every boot | a `shutdown.request` outlived an interrupted shutdown, and `PathExists=` fires at unit start on a file that is already there | `/etc/tmpfiles.d/uav.conf` sweeps both request files at boot, before the `.path` units arm. Re-run the installer if it is missing |
| container still mounts `/run/uav` or `/run/uav-power.sock` | left from the retired socket helper. Harmless as a directory, **fatal** as a file — `docker start` fails with "not a directory" | recreate it: `docker rm -f uav && sudo bash setup/install_jetson_host.sh`. The power request needs no mount of its own now |
| `git pull` changed nothing | colcon installs into `install/`; the node does not import from `src/` | `rebuild.sh`, then restart the units |
| Rebuild changed nothing either | the workspace is not actually bind-mounted | the System tab says so explicitly. Recreate with `-v ~/robotx_ws:/root/robotx_ws` |

### Build and startup

| Symptom | Cause | Fix |
|---|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` | ROS's `setup.bash` reads variables it never sets, and the script runs `set -u` | fixed in `rebuild.sh` / `install_container.sh` (nounset is lifted only across the `source`). If you hit it in your own script, do the same |
| `TypeError: The given value is not a list of one of the allowed types` | a **nested** value in `uav_params.yaml`. ROS 2 parameters cannot nest — scalars and flat homogeneous arrays only | write it flat. The geofence is `[lat, lon, lat, lon, …]`; `fence_core.polygon_from_flat` pairs it back |
| `Couldn't parse params file`, every node dies at `rclpy.init()` | a YAML anchor/alias, or a top-level key without `ros__parameters` | `python3 tools/scripts/check_config.py` |
| `could not declare parameter … outside [lo, hi]` | the YAML and the node's `PARAM_SPEC` are different ages | almost always a stale install space — `rebuild.sh` |
| `still no heartbeat — is MAVProxy running?` | the bridge is up, MAVProxy is not, or `SR0_*` are all 0 | check the unit; then check stream rates in QGC |

### Reporting to the OCS

| Symptom | Cause | Fix |
|---|---|---|
| `cannot reach 192.168.8.107:37564`, every 5 s | the OCS laptop is not up | **benign and self-healing** — the link retries and names the address. Not an error |
| OCS shows rising silence | `ocs_client` is refusing to invent telemetry | the log says which input is missing: pose, `fcu_status`, or flight phase |
| `flight_phase source: fallback` in the log | `/uav/flight_state` is stale or absent | set `SR0_EXT_STAT > 0` in QGC. The fallback is a real derivation but a worse answer than the autopilot's own |
| OCS drops every frame: "not a configured vehicle" | `vehicle_id` / `team_id` disagree with `bridge.toml` | `check_config.py` compares them when `rx26_ocs` is checked out beside this repo |
| OCS refuses a frame as `UNKNOWN` | a UAV may not send `FLIGHT_PHASE_UNKNOWN` | that is the net working. Find why the phase was unknown |

### Nothing works and you want a clean read

These need no ROS, no aircraft, and no container — run them on any laptop:

```bash
python3 tools/scripts/check_config.py
```

```bash
python3 tools/bench/bench_fence.py && python3 tools/bench/bench_heartbeat.py && python3 tools/bench/bench_gcs.py
```

`7/7`, `24/24`, `15/15`, `OK`. If those pass, the logic is sound and the problem
is the host, the device, or the wiring — which is the whole list above.

## Safety constraints (non-negotiable)

1. **One Pixhawk owner.** MAVProxy holds the serial link; everything else
   consumes its UDP rebroadcast. `uav_fcu` rejects a non-udp/tcp endpoint at
   startup, and no other package may open a MAVLink connection at all.
2. **There is no disarm path in this repo, deliberately.** The ASV's
   `telemetry_bridge` carries a force-disarm driven by RC loss. Do not port it.
   On a boat a force-disarm stops the thrusters and the hull floats; on a
   multirotor it stops the motors and the aircraft falls. RC loss belongs to the
   autopilot's own failsafe parameters, which work without a companion computer
   being alive to have an opinion.
3. **The autonomy-drop latch is not a disarm.** Tripping it releases overridden
   RC channels back to the pilot and leaves the aircraft flying.
4. **The geofence is enforced by the autopilot, not by us.** We upload it and
   verify the readback; `FENCE_ENABLE` is never set from code. A fence the
   autopilot does not echo back does not exist.
5. **Never invent telemetry.** If pose, autopilot status, or the flight phase
   cannot be determined, `ocs_client` sends *nothing*. A gap shows on the OCS as
   rising silence; a fabricated value is indistinguishable from a real one — and
   for a UAV it is relayed onward for Singapore's Network Remote ID.
6. **WiFi is a convenience, never a control path.** The ground station cannot
   stop the MAVLink gateway, and `ocs_client` never acts on an inbound command.

## The two external contracts

Neither is negotiable from inside this repo.

**The OCS vehicle link** — mirrors `rx26_ocs/rx_bridge/framing.py`:

```
TCP to 192.168.8.107:37564
4 bytes  big-endian uint32 length N
N bytes  UTF-8 JSON of an RxReport (protobuf json_format.ParseDict)
```

Non-finite floats must be the **quoted** strings `"NaN"`/`"Infinity"`. The OCS's
`framing.py` names our `ocs_link.py` as its deliberate duplicate — **change the
header in one repo and you must change it in the other in the same commit.**

**UAV-specific validation** — `rx26_ocs/rx_bridge/config.py`:

```python
"TYPE_USV": ("flight_phase",)     # may be UNKNOWN
"TYPE_UUV": ("flight_phase",)     # may be UNKNOWN
"TYPE_UAV": ()                    # NOTHING may be UNKNOWN
```

So `flight_phase` is **mandatory** for us and optional for the boat. A hull has
no honest flight phase; an aircraft has no such excuse. `vehicle_type` must be
`TYPE_UAV` or the bridge drops the frame, and `altitude_hae_m` must be real
altitude.

**The geofence appears in four places** — three in `uav_params.yaml` and one as
`uav_geofence` in the OCS's `bridge.toml` — and all four must agree, or we upload
one polygon to the autopilot and declare a different one to RoboNation.
`check_config.py` pins them, including across repos when `rx26_ocs` is checked
out beside this one.

## Verifying it

Nothing here has flown. What *has* been exercised, and how to repeat it:

```bash
python3 tools/bench/bench_fence.py
```
```bash
python3 tools/bench/bench_heartbeat.py
```
```bash
python3 tools/bench/bench_gcs.py
```
```bash
python3 tools/scripts/check_config.py
```

No ROS, no aircraft, no simulator. They cover the geofence upload dialog
including every failure the readback exists to catch, the heartbeat suppression
rules, and the ground station's rules driven **over HTTP with the page
bypassed** — because a rule enforced only in the page is decoration.

Then, with no aircraft, against a real OCS:

```bash
python3 -m uav_groundstation.ocs_link --host 192.168.8.107 --type uav --vehicle UAV1 --team ASTA
```

and the two faults it must be *refused* for: `--fault no_phase`, `--fault nan`.

On the airframe, props off, see [docs/BRINGUP.md](docs/BRINGUP.md).

## Versioning

**0.1.0.** Plain semver. This becomes `1.0.0` at competition freeze. The version
counts capability proven, not code written — which is why it is 0.1 despite the
package count.

## Dependencies

**Hardware:** Jetson Orin Nano; Pixhawk (HAWK'S WORK 2.4.8-class board — **its
USB VID/PID is not yet confirmed**, see `tools/udev/99-uav.rules`); RadioMaster
Pocket + RP3 ELRS receiver (CRSF to the Pixhawk, not USB to the Jetson); WiFi to
the team subnet.

**Software:** ROS 2 Humble + pymavlink inside the `uav` container; MAVProxy on
the **host**, never in the container. Python ≥3.10 + pyyaml for the off-board
tools. No device SDKs, by design — there is no camera or LiDAR on this airframe.

## What is deliberately absent

Perception, world model, missions, LED status stack, session recording, camera
and LiDAR viewer tabs, and any simulator. Each is a real gap, not an oversight;
naming them here is cheaper than a stub that looks like progress.

## Next

- **Confirm the Pixhawk's VID/PID** on the real airframe and prune
  `99-uav.rules` to the board actually aboard. One `udevadm info` away.
- **Set `SR0_EXT_STAT > 0`** in QGC. Without it `/uav/flight_state` never
  publishes and `ocs_client` falls back to an armed+altitude guess for
  `flight_phase` — it says so loudly, but the fallback is a worse answer than
  the autopilot's own.
- **Check `geoid_separation_m` against the real course.** −24.6 m is Sarasota;
  Singapore differs. It is silently wrong by tens of metres until someone
  measures it.
- **Replace the placeholder geofence** in both repos, together.
- Run the link against RoboNation's own stub. Ours agrees with itself; theirs is
  the authority.
