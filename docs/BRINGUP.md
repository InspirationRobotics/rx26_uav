# Bringing the aircraft up

Ordered so each step needs only what the one before it proved. Nothing here has
flown; every step below is bench or ground.

---

## 0. Off-board, no hardware at all

Runs on any laptop with Python ≥3.10. Do this before touching the airframe.

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

Expect `7/7`, `24/24`, `15/15`, and `OK`. These cover the geofence upload dialog
including every failure the readback exists to catch, the heartbeat suppression
rules, and the ground station's rules **driven over HTTP with the page
bypassed**.

With `rx26_ocs` cloned beside this repo, `check_config.py` also compares our
geofence against the one the OCS declares. That is the check worth having.

---

## 1. The OCS link, no aircraft

With the OCS laptop up at `192.168.8.107`:

```bash
python3 -m uav_groundstation.ocs_link --host 192.168.8.107 --type uav --vehicle UAV1 --team ASTA
```

The OCS log should file the socket as `UAV1` and forward heartbeats. Then prove
the nets catch what they exist for — **each must be refused, visibly**:

```bash
python3 -m uav_groundstation.ocs_link --host 192.168.8.107 --type uav --fault no_phase
```
```bash
python3 -m uav_groundstation.ocs_link --host 192.168.8.107 --type uav --fault nan
```
```bash
python3 -m uav_groundstation.ocs_link --host 192.168.8.107 --type uav --fault mistype
```

A net you have never seen catch anything is a net you are guessing about.

---

## 2. Host install

```bash
sudo bash setup/install_jetson_host.sh
```

It creates `~/robotx_ws/src/`, installs the udev rules and all five units, and
creates the container if absent. It **refuses** to move the repo or delete a
container — where either is needed it prints the command.

Then, inside the container:

```bash
docker exec uav bash /root/robotx_ws/src/rx26_uav/setup/install_container.sh
```

### Confirm the Pixhawk symlink

```bash
ls -l /dev/uav-pixhawk
```

If it is missing, the VID/PID in `tools/udev/99-uav.rules` does not match this
board — the rules file says outright that its four Pixhawk lines are a candidate
list, not a confirmed one. The installer prints every attached serial device with
its descriptors; read the right one off that and add a line.

The collision guard must also be quiet. If two names resolve to one tty, **stop**:
that aliases the autopilot, and anything opening the other name becomes a second
owner of a link MAVProxy already holds.

---

## 3. Autopilot parameters, in QGC

MAVProxy runs `--streamrate=-1` so it does not stomp these. Set them once; they
persist in EEPROM.

| Param | Value | Without it |
|---|---|---|
| `SR0_POSITION` | > 0 | no `/uav/pose` at all |
| `SR0_EXTRA1` | 30 | attitude at whatever rate, or none |
| `SR0_EXT_STAT` | > 0 | **no `/uav/flight_state`** — `ocs_client` falls back to an armed+altitude guess for `flight_phase` |
| `FENCE_TYPE` | polygon bit set | the fence upload is NACKed |
| `FENCE_ENABLE` | your call | never set from code |

`SR0_EXT_STAT` is the one people miss. Everything keeps working and the log fills
with a warning that the flight phase is a fallback.

---

## 4. Boot chain, props off

```bash
sudo systemctl start uav-mavproxy uav-container uav-groundstation uav-ocs-client uav-power
```

Then reboot and let it come up unattended.

```bash
systemctl status uav-mavproxy uav-container uav-groundstation uav-ocs-client
```

Check, in order:

- `ros2 topic echo /uav/pose` — live `altitude_amsl` and `altitude_rel`.
- `ros2 topic echo /uav/attitude` — moves when the airframe is tilted by hand.
- `ros2 topic echo /uav/flight_state` — `landed_state: 1` (ON_GROUND). Silent
  means `SR0_EXT_STAT` is still 0.
- `sudo systemctl stop uav-mavproxy` → **one** loud stale line per stream, and
  republishing **stops**. Restart and confirm recovery. This is the whole
  staleness rule; if a topic keeps publishing, something is replaying a cache.

### The restart check — do not skip this

```bash
sudo systemctl restart uav-groundstation
```
```bash
docker exec uav pgrep -fc install/uav_groundstation/lib/uav_groundstation/ground_station
```

Must print **`1`**. Repeat three times — the orphan case only appears once a stop
has failed. Then force the bad case deliberately: suspend the node
(`docker exec uav pkill -STOP -f .../ground_station`), restart the unit, and
confirm the `ExecStartPre` sweep still leaves exactly one.

Two means the sweep pattern is not matching; zero means the marker is wrong.
`run_in_container.sh` prints the marker it derived at startup.

---

## 5. The geofence, on the real Pixhawk

Disarmed:

```bash
ros2 service call /uav/fence_upload std_srvs/srv/Trigger
```

Then **read it back in QGC and compare vertex for vertex** against `uav_geofence`
in `rx26_ocs/bridge.toml`. The service verifies its own readback, but that proves
the autopilot agrees with what we sent — not that what we sent is what we
declared.

Arm and call it again: **it must refuse.**

---

## 6. Ground station

`http://<JETSON_IP>:8090`

Exercise the rules by calling the endpoints directly, since the page is not where
a rule lives:

```bash
curl -X POST http://<JETSON_IP>:8090/node/stop -d '{"name":"telemetry_bridge"}'
```

Refused, with the reason. Then:

- The Nodes tab shows both systemd-started nodes as **running** — that path is
  `/proc`, not a `Popen` handle.
- The Map draws the geofence; walking the airframe across an edge flips the
  inside/outside readout.
- The System tab says the workspace **is** a bind mount. If it says otherwise,
  fix that before writing any more code — a `git pull` is invisible in the
  container and a rebuild silently changes nothing.

### The shared-workspace loop

```bash
cd ~/robotx_ws/src/rx26_uav && git pull
```
```bash
docker exec uav bash -lc '/root/robotx_ws/src/rx26_uav/tools/scripts/rebuild.sh'
```
```bash
sudo systemctl restart uav-groundstation uav-ocs-client
```

Confirm a pull **without** a rebuild visibly changes nothing. Understanding that
now is cheaper than discovering it at a flight line.

---

## Still not proven by any of the above

- Anything at all in the air.
- The link against RoboNation's own stub. Ours agrees with itself; theirs is the
  authority.
- `geoid_separation_m` against the real venue.
- The competition geofence — what ships is a placeholder at the right venue.
