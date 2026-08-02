# Public-release runtime validation

**These are not thesis results.** They record what happened when the code in
*this repository* — not the private development tree — was pointed at a live
PX4 SITL stack, on one ordinary laptop, in August 2026.

Read them as a compatibility check on the release: does the extracted code
still talk to PX4, does the obstacle really appear in Gazebo, does the vehicle
really arm and hold position. Nothing here replaces or updates
[`results/stop_replan_resume/`](../stop_replan_resume/) or
[`results/nominal_mission/`](../nominal_mission/), which are the canonical
single-run evidence the thesis reports.

| | Canonical bundles | This directory |
|---|---|---|
| Produced by | the private development tree | this public repository |
| Purpose | thesis evidence | release compatibility check |
| Status | frozen, cited | disposable, re-runnable |
| Statistical weight | none (single runs) | none (single runs) |

## What was validated

### Physical Gazebo obstacle spawn — verified

[`obstacle_spawn/spawn_record.yaml`](obstacle_spawn/spawn_record.yaml)

`tools/spawn_srr_obstacle.py` put `srr_unknown_obstacle` into the running
`forest_map2` world and then read the pose back out of Gazebo:

| | |
|---|---|
| Models before / after | 88 → 89 |
| Occurrences of the model | exactly 1 |
| Requested pose | (−21.330, −43.710, 1.500) m |
| Measured pose | (−21.330, −43.710, 1.500) m, delta (0.000, 0.000, 0.000) |
| Second invocation | `already_present`, nothing spawned, exit 0 |
| `--fail-if-exists` on a populated world | exit 4 |

### Armed perception-driven Stop–Replan–Resume — **not completed**

[`srr_armed_attempt/`](srr_armed_attempt/)

The run reached, in order:

```
WAIT_STATUS → WAIT_LPOS → WAIT_EKF_READY → PRE_STREAM
→ DECIDE_ARM → WAIT_OFFBOARD → WAIT_ARMED → SETTLE_HOLD
```

It armed, entered OFFBOARD, and held the settle setpoint well — **lateral error
max 0.060 m, mean 0.034 m** over 107 samples while climbing from 0.63 m to
1.45 m. Then it aborted on `lpos_stale`, landed and disarmed cleanly with no
force-disarm.

It never reached MISSION, so **no detection, stop, replan, resume or rejoin was
exercised**. `verdict.yaml` records exactly that: every condition from
`perception_sensed_obstacle_built` onward is `false`.

**The blocker, precisely.** The executor aborts when telemetry goes stale:
`--lpos-watchdog-s 0.5` on `/fmu/out/vehicle_local_position` and
`--watchdog-s 3.0` on `/fmu/out/vehicle_status_v1`. On this machine the
uXRCE-DDS path delivers in bursts:

| Topic | PX4 publishes every | Delivered, median gap | p95 | max |
|---|---|---|---|---|
| `vehicle_status_v1` | 0.503 s (measured from its own timestamps) | 0.74 s | 2.82 s | **6.21 s** |
| `vehicle_local_position` | 50 ms | 0.001 s | 0.33 s | **1.24 s** |

Messages are late, not lost: across a 6.21 s wall-clock gap the two PX4
timestamps either side differ by 0.509 s.

The likely mechanism, stated as the observed runtime diagnostic it is. In a
separate 60 s correlation trace taken earlier in the session, on the stack
configuration that still predated the world-file fixes below, the four largest
`vehicle_status_v1` delivery gaps each fell in a window where the Gazebo
real-time factor dipped to 0.002–0.004 against a 0.99 baseline; PX4's own DDS
client reported a 3.146 s worst-case cycle. A Gazebo stall pausing PX4's
lockstep scheduler, and with it the DDS client, fits that. **That trace is not
part of these artifacts and the correlation is therefore not independently
recomputable here.** It was measured on a different stack configuration from
the armed attempt recorded in this directory, so it is offered as a diagnostic
lead rather than as a property of this run.

The thresholds were **not** raised. A watchdog that fires on stale telemetry is
doing its job; making it tolerant enough to pass here would make it useless.

## What was fixed to get this far

Four defects in this repository were found by running it, and are fixed:

1. **`sim/worlds/forest_map2.sdf` declared no `<atmosphere>`, `<magnetic_field>`,
   `<gravity>` or `<physics>`.** PX4 reported `No valid data from Baro 0` and
   `No valid data from Compass 0`; the EKF height random-walked by up to 13 m
   with the vehicle stationary on the ground. Adding them took barometer errors
   from 112 to 0 and, comparing pre-arm sampling windows before and after,
   the RTF fifth percentile from 0.005 to 0.991.
2. **The magnetic-field vector has to match the world's own coordinates.**
   Copying PX4's default-world vector gave a 114° EKF heading error and the
   vehicle flew away from its hold point. Derived correctly from PX4's world
   magnetic model at 51.239583 N, 7.162194 E, the heading reads 95.08°, against
   96.12° in the canonical run.
3. **`<physics type="ode">` crashes Gazebo on this world.** A 400 m ground box
   with 80 mesh trees overflows ODE's collision-space bounds
   (`ODE INTERNAL ERROR 1 ... collide()`), the server aborts mid-flight, PX4
   loses its accelerometer and the vehicle departs at 6 m/s. The engine is DART.
4. **The launch order and the environment.** See `tools/sitl_stack.sh`: the DDS
   agent must precede PX4; PX4 must own Gazebo's lifecycle; a snap-packaged
   terminal breaks Gazebo's GPU driver loading; and the offscreen EGL path picks
   an unusable GPU on hybrid graphics.

The occupancy grid derived from the world file is **byte-identical** before and
after these changes (SHA-256 of the rasterised image unchanged), so no planning
result moved. `tests/test_world_and_obstacle.py` asserts both halves of that.

## Environment

| | |
|---|---|
| Date | 2 August 2026 |
| PX4 | v1.16.0-rc1-449-gbfbc76189e |
| Gazebo | Sim 8.10.0 (Harmonic) |
| ROS 2 | Jazzy, `rmw_fastrtps_cpp` |
| Physics RTF, pre-arm sampling window | median 0.994, p05 0.991, min 0.990 (8 s, `sitl_stack.sh rtf`, taken on the corrected stack before the obstacle spawn and before the armed node started) |
| Physics RTF during the armed attempt | not recorded. The bundle carries no RTF series |
| Code version | this repository; no git checkout, so the run records `unknown (not a git checkout)` |
| Hardware | one laptop, hybrid graphics, 16 threads |

## Reproducing

```bash
bash tools/sitl_stack.sh start
bash tools/sitl_stack.sh params
bash tools/sitl_stack.sh prearm
python3 tools/spawn_srr_obstacle.py --world forest_map2 --out-dir /tmp/srr_run
python3 tools/gcs_heartbeat.py &
# then the armed command in docs/installation.md
bash tools/sitl_stack.sh stop
```

Expect different numbers. A live SITL run is not deterministic, and on a
machine with more headroom the watchdogs may well not fire at all.
