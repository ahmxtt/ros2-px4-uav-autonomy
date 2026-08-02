# Installation

Two paths. The **offline** path needs only Python and reproduces every
planning and trajectory number in [results.md](results.md). The **SITL** path
additionally runs the flight stack and is required only to re-fly a mission.

---

## Offline (recommended first)

```bash
git clone <this repository>
cd ros2-px4-uav-autonomy
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
python3 tools/reproduce_offline_demo.py --output-dir /tmp/uav_demo
```

No ROS, no PX4, no Gazebo.

Run the reproduction commands from the repository root. Internally the tools
resolve repository inputs independently of the caller's current working
directory, so nothing breaks if you invoke them by absolute path; CI does
exactly that, from a foreign working directory, as a standing check.

### Environment of record

The published results were produced with:

| Component | Version |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| Python | 3.12.3 |
| NumPy | 1.26.4 |
| SciPy | 1.11.4 |
| Matplotlib | 3.6.3 |
| Pillow | 10.2.0 |
| PyYAML | 6.x |

`pyproject.toml` declares looser floors (`numpy>=1.24`, `scipy>=1.10`) because
the pipeline does not depend on version-specific behaviour. If you need
bit-exact agreement with the published trajectory metrics rather than
agreement to 1e-9, pin the versions above: floating-point summation order in
SciPy and BLAS can move the last few digits of the arc-length and KD-tree
reductions.

### Tests

```bash
pip install -e ".[test]"
pytest                     # everything, including the slow regression
pytest -m "not slow"       # fast invariant tests only
```

The `slow` test rebuilds the full 2913 × 3911 grid and replans the published
leg (~230k node expansions). It is the test that ties the code to the numbers
in the documentation.

---

## SITL (PX4 + Gazebo + ROS 2)

### Environment of record

| Component | Version / commit |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy |
| RMW | `rmw_fastrtps_cpp` |
| Gazebo | Sim 8.10.0 (Harmonic) |
| PX4-Autopilot | `v1.16.0-rc1-449-gbfbc76189e` (`bfbc76189e48fbb1250667eaa89cafb9d8aa8b9e`) |
| `px4_msgs` | 2.0.1 (`4f22059bc4a27cff5d06411ff2e478811431df1a`) |
| Micro XRCE-DDS Agent | source build |
| QGroundControl | optional, for manual inspection |

### 1 · ROS 2 Jazzy and Gazebo Harmonic

Follow the upstream instructions for
[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) and
[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install).

### 2 · PX4

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout bfbc76189e48fbb1250667eaa89cafb9d8aa8b9e
make px4_sitl gz_x500_lidar_2d
export PX4_AUTOPILOT_DIR=$PWD
```

The tools read `PX4_AUTOPILOT_DIR` where they need the PX4 tree; nothing
assumes a fixed install location.

### 3 · `px4_msgs` and the DDS agent

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs && git checkout 4f22059bc4a27cff5d06411ff2e478811431df1a
cd ~/ros2_ws && colcon build && source install/setup.bash
```

Then build and install the Micro XRCE-DDS Agent from source.

> **RMW consistency matters.** The PX4 bridge topics are only discoverable
> when every participant uses the same RMW implementation. Mixing
> `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp` across shells makes `/fmu/*`
> appear absent — which looks exactly like "the simulator is not publishing".
> Export `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in every shell.

### 4 · Gazebo Fuel models — required

The world references two Fuel models by `model://` URI and **this repository
does not vendor their meshes**:

| Model | Owner | Version | Licence |
|---|---|---|---|
| `Pine Tree` | OpenRobotics | 6 | CC0 1.0 Universal (public domain) |
| `Oak tree` | OpenRobotics | 7 | CC0 1.0 Universal (public domain) |

Without them Gazebo cannot load the world. Fetch them once:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree"
```

Then set `GZ_FUEL_OFFLINE=1` for subsequent runs so no network fetch can
silently change the scene. Full attribution: [../NOTICE.md](../NOTICE.md).

The **offline** pipeline does not need these meshes: the canopy radii were
measured once and are carried as documented constants.
### 5 · Bring the stack up

One script does the whole sequence and refuses to continue past any layer that
is not actually alive:

```bash
bash tools/sitl_stack.sh start
```

It starts four things, in this order, and the order is not cosmetic:

| # | Component | Why here |
|---|---|---|
| 1 | Micro XRCE-DDS Agent | PX4's DDS client tries to connect once, early. If nothing is listening it gives up and no `/fmu/...` topic ever appears, which looks exactly like a simulator that is not publishing. |
| 2 | PX4 SITL, which starts Gazebo itself | PX4's simulated IMU, barometer, magnetometer and GPS are driven by Gazebo's sensor systems through PX4's own bridge, and that pipeline expects PX4 to own the simulator's lifecycle. Attaching PX4 to a Gazebo server that was already free-running produces `No valid data from Baro 0` and an EKF that diverges by hundreds of metres while the vehicle sits still. |
| 3 | Gazebo → ROS 2 bridge | It can only discover the LiDAR topic once the vehicle model exists. |
| 4 | LiDAR self-filter | Publishes `/scan_filtered`, the perception input the SRR node subscribes to by default. |

**Nothing is written into the PX4 tree.** PX4 resolves worlds from
`$PX4_GZ_WORLDS`, which its own `gz_env.sh` normally overwrites, so the script
gives PX4 a scratch working directory with its own `gz_env.sh` beside it,
pointing at this repository's `sim/worlds`. Confirm it in the PX4 log:

```bash
grep 'Starting gazebo with world' /tmp/sitl_stack/px4.log
# .../ros2-px4-uav-autonomy/sim/worlds/forest_map2.sdf
```

The vehicle spawns at `PX4_GZ_MODEL_POSE="-50,-70,0.5,0,0,0"`. That world
position is also the PX4 local-NED origin; changing it invalidates the frame
constants in `uav_autonomy.frames`.

The script also repairs two environment problems rather than leaving them to
you. A terminal launched from a snap-packaged editor exports `SNAP*`, `GTK_*`,
`LOCPATH` and a snap-prefixed `LD_LIBRARY_PATH`, which make Gazebo load the
wrong GUI libraries; and on hybrid graphics the offscreen EGL path picks a GPU
whose driver may be unusable, so `EGL_PLATFORM=x11` is forced when a display
exists. Both otherwise show up as a collapsed real-time factor.

By default the bridge relays **only** `/scan`. Set `BRIDGE_FULL=1` to add
`/clock` and `/gz_pose_array`, which RViz and a ground-truth recorder need.
Neither ROS node in this repository reads them, and relaying Gazebo's 1 kHz
clock costs simulator time.

### 6 · Check what is running

```bash
bash tools/sitl_stack.sh status
bash tools/sitl_stack.sh rtf
```

`status` subscribes to each topic rather than listing it: with the ROS CLI
daemon disabled, `ros2 topic list` can return an empty list while a topic is
publishing at full rate.

`rtf` reports two different things. The **physics real-time factor**, read from
Gazebo's own statistics topic, is the one that matters: the executor paces on
wall-clock time, so below about 0.95 both the settle window and the tracking
budget are at risk. The `/clock` relay figure below it measures bridge
throughput, not simulator speed, and a low value there is not by itself a
problem.

A healthy stack on this world reports a median physics RTF of about 0.99 with a
fifth percentile above 0.99.

### 7 · Apply the PX4 parameters

```bash
bash tools/sitl_stack.sh params
```

| Parameter | Value | Why |
|---|---|---|
| `MPC_TKO_RAMP_T` | 1.0 | The default takeoff ramp keeps the aircraft on the ground long enough to consume the settle budget. |
| `MPC_TKO_SPEED` | 1.0 | Takeoff climb rate, for the same reason. |
| `COM_RCL_EXCEPT` | 4 | No RC transmitter exists in SITL. Without the Offboard exception PX4 declares RC loss the moment it arms. |

The command sets all three through `px4-param`, saves them, and prints the
values back for you to read.

### 8 · Leave AUTO_LOITER for Position mode

```bash
bash tools/sitl_stack.sh prearm
```

A freshly booted PX4 SITL sits in `AUTO_LOITER` (`nav_state` 4). The executor
refuses to arm from `AUTO_LOITER`, `AUTO_RTL`, `AUTO_LAND` or an already-active
`OFFBOARD`, and it is right to: arming on top of a residual state is how a run
ends up doing something other than what was asked. The canonical published run
observed `nav_state` transitions `[2, 14, 18]` — POSCTL before arming, OFFBOARD
during the mission, AUTO_LAND at the end.

### 9 · Give PX4 a ground station

```bash
python3 tools/gcs_heartbeat.py &
```

PX4 expects a GCS to be connected. Without one the pre-flight checks report
`No connection to the ground control station` and `vehicle_status` is
republished irregularly, which can trip the executor's status watchdog while
the vehicle is still on the ground. Running QGroundControl fixes both, but a
GUI costs real-time factor an armed run cannot spare. This sends `HEARTBEAT`
and nothing else: it cannot arm, command or configure the vehicle.

### 10 · Spawn the unknown obstacle

```bash
python3 tools/spawn_srr_obstacle.py --world forest_map2 --out-dir /tmp/srr_run
```

**When, and why then.** The obstacle must exist in Gazebo before the vehicle
reaches that part of the mission, which is about 39 m along segment 1. Spawning
it before the armed process starts is simplest and is what the canonical run
did. It changes nothing for the planner: the obstacle is deliberately absent
from the world file and therefore from the prior map, so it stays novel.

Expected output, and what to check:

```
[spawn] models before   = 88 (obstacle present: False)
[spawn] models after    = 89 (obstacle present: True)
[spawn] measured pose   = (-21.330, -43.710, 1.500) m  delta=(+0.000, +0.000, +0.000)  match=True
[spawn] status          = spawned
```

The tool reads the pose back out of Gazebo and fails if it disagrees by more
than 5 cm. It spawns **exactly once**: on a world that already has the model it
reports `already_present` and exits 0, or exits 4 under `--fail-if-exists`.
`spawn_record.yaml` records `spawned_this_invocation` and the before/after
model counts, so exactly-once is checkable afterwards rather than assumed.

Verify independently at any time:

```bash
python3 tools/spawn_srr_obstacle.py --verify-only --out-dir /tmp/srr_run
gz model -m srr_unknown_obstacle
```

**Five things called "the obstacle". Keep them apart:**

| Representation | What it is | Who reads it |
|---|---|---|
| Physical Gazebo model | The cylinder spawned above | The 2D LiDAR, and nothing else |
| Prior occupancy map | Does **not** contain it | The replanner's A\* |
| Sensed obstacle | Built online from LiDAR clusters plus a novelty test against the prior | The local ROI replanner |
| Ground truth (`sim/obstacles/obstacle.yaml`) | The true pose and radius | In perception mode: held in a separate evaluation-only namespace. It gates no detection, obstacle construction, replanning or control decision, but it is read for a per-tick diagnostic distance and for the post-run metrics. The oracle modes read it for decisions too |
| Patched map | Prior with ground truth stamped in | Only `--oracle-mode replan` |

Perception-driven Gate 6D-B needs the physical model **present** and the ground
truth **unused**. Both at once, or the result means nothing.

### 11 · Build the mission reference and setpoints

`results/mission_reference/mission_reference.csv` ships with this repository:
the complete five-segment route, 16 157 samples, starting at the vehicle spawn
point. Turn it into a PX4 local-NED setpoint stream:

```bash
python3 tools/build_ned_setpoints.py \
    --trajectory-csv results/mission_reference/mission_reference.csv \
    --output-csv /tmp/setpoints_ned.csv \
    --require-segments start_to_poi1,orbit_poi1,poi1_to_poi2,orbit_poi2,poi2_to_start
```

5725 rows at 20 Hz, 286.16 s, first setpoint 0.035 m from the NED origin. The
tool refuses to write a stream whose first setpoint is more than 1 m from the
origin, because the settle guard cannot fly there from a standing start. It
also writes `<output>.validation.yaml` and `<output>.hashes.txt`.

For the SRR node, one command assembles all five inputs instead:

```bash
python3 tools/build_srr_inputs.py --output-dir /tmp/srr_inputs
```

That writes `prior/map.yaml`, `patched/map.yaml`, `obstacle.yaml`,
`mission_reference.csv` and `setpoints_ned.csv`, and prints the run command.

### 12 · Nominal Offboard executor

**Dry run first.**

```bash
python3 ros2/nodes/offboard_mission_executor.py \
    --setpoints-csv /tmp/setpoints_ned.csv \
    --out-dir /tmp/run_nominal \
    --multi-segment
# expect: VERDICT=DRY_RUN_OK
```

What that actually covers: the dry run verifies ROS/PX4 connectivity, LiDAR
message reception, argument handling, pre-arm Offboard heartbeat and
hold-setpoint streaming, the two-factor arm gate and output initialisation. It
does not execute obstacle detection, stop, local replanning, resume or rejoin,
because those states are reached only during an armed mission. Concretely it
reaches six of the node's thirteen states — `WAIT_STATUS`, `WAIT_LPOS`,
`WAIT_EKF_READY`, `PRE_STREAM`, `DECIDE_ARM`, `DONE` — and stops there. It
publishes `OffboardControlMode` and a hold `TrajectorySetpoint`, and never
publishes a `VehicleCommand` of any kind. See
[safety.md](safety.md#what-dry-run-actually-does).

Armed run, **simulation only**, and only with eyes on the simulator:

```bash
python3 ros2/nodes/offboard_mission_executor.py \
    --setpoints-csv /tmp/setpoints_ned.csv \
    --out-dir /tmp/run_nominal \
    --multi-segment --use-velocity-feedforward \
    --takeoff-altitude-m 2.0 --max-altitude-m 2.5 \
    --max-tracking-error-m 1.0 --rate-hz 20 --max-run-s 600 \
    --arm --i-have-physical-eyes-on-the-sim
```

Outputs in `--out-dir`: `run_log.csv` (per tick), `run_metrics.yaml`,
`verdict.yaml`, `manifest.yaml`, `hashes.txt`.

### 13 · Stop–Replan–Resume node

Dry run:

```bash
python3 ros2/nodes/stop_replan_resume_node.py \
    --setpoints-csv    /tmp/srr_inputs/setpoints_ned.csv \
    --mission-csv      /tmp/srr_inputs/mission_reference.csv \
    --prior-map-yaml   /tmp/srr_inputs/prior/map.yaml \
    --patched-map-yaml /tmp/srr_inputs/patched/map.yaml \
    --obstacle-yaml    /tmp/srr_inputs/obstacle.yaml \
    --oracle-mode off \
    --out-dir /tmp/run_srr
```

`--oracle-mode off` is the perception-driven variant (Gate 6D-B): ground truth
is confined to the evaluation-only namespace and `--patched-map-yaml` is not
read. The oracle modes
(`detection`, `replan`) do read it, and exist to validate the mechanics rather
than the perception.

The same dry-run limits apply: this confirms the node connects, subscribes to
`/scan_filtered`, parses all five inputs and initialises its outputs. Detection,
stop, hover, replan, resume and rejoin are unreachable without arming — the
guard is an explicit `state != MISSION` early return, and MISSION is only
entered from SETTLE_HOLD after a confirmed arm.

Armed run:

```bash
python3 ros2/nodes/stop_replan_resume_node.py \
    --setpoints-csv    /tmp/srr_inputs/setpoints_ned.csv \
    --mission-csv      /tmp/srr_inputs/mission_reference.csv \
    --prior-map-yaml   /tmp/srr_inputs/prior/map.yaml \
    --patched-map-yaml /tmp/srr_inputs/patched/map.yaml \
    --obstacle-yaml    /tmp/srr_inputs/obstacle.yaml \
    --oracle-mode off --use-velocity-feedforward \
    --max-run-s 600 --write-plot \
    --out-dir /tmp/run_srr \
    --arm --i-have-physical-eyes-on-the-sim
```

What to verify in the output, in order: `scan_topic_first_msg` and
`perception_mode_no_ground_truth_online` in `verdict.yaml` (the key name is
kept for schema compatibility; it asserts that no online *decision* read ground
truth, not that ground truth was discarded);
`detection_confirmed` and `stop_command_sent` in `event_log.csv`;
`hover_stable_achieved`; `replan_completed` with its runtime, length and
clearance; the resume anchor and rejoin figures in `resume_join_metrics.yaml`;
and finally `mission_completed` with `land_sent` and `disarm_seen`.

Outputs additionally include `event_log.csv`, `sensed_obstacle_metrics.yaml`,
`replan_runtime_metrics.yaml`, `resume_join_metrics.yaml` and the two
local-replan CSVs.

> **State of this run in the public release.** An armed SRR mission has not been
> completed with this code. On the reference machine it arms, enters OFFBOARD
> and holds the settle setpoint to 0.06 m, then aborts on a telemetry-freshness
> watchdog before reaching MISSION. The cause is measured and documented in
> [../results/public_runtime_validation/README.md](../results/public_runtime_validation/README.md),
> together with the four world-file and launch defects that were fixed along the
> way. The watchdog thresholds were not relaxed. Everything up to and including
> ARM, OFFBOARD and SETTLE_HOLD is demonstrated; MISSION and the SRR sequence
> are not.

### 14 · One armed run per stack

Restart everything between armed runs:

```bash
bash tools/sitl_stack.sh stop
bash tools/sitl_stack.sh start
```

This is not optional hygiene. A run that ended in a force-disarm leaves the
vehicle and the estimator in a state that reliably makes the *next* run abort
during SETTLE, on altitude or on tracking. PX4 runs with its working directory
inside the scratch `RUN_DIR` (`/tmp/sitl_stack/px4_run` by default), so its
parameter store and every other runtime file live there. `stop` removes that
directory and `start` recreates it, which is what makes the next stack
genuinely virgin. Nothing under the PX4 build tree is touched. It is a current
limitation of this workflow, not something fundamental.

### 15 · Shutdown

```bash
bash tools/sitl_stack.sh stop
```

It kills the self-filter, bridge, PX4, Gazebo and the DDS agent, waits,
escalates to `SIGKILL`, removes the scratch run directory, and then **verifies**
nothing survived, listing anything that did and returning non-zero. An orphaned Gazebo server silently
degrades the next run's real-time factor, so confirm the shutdown is clean.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `/fmu/*` topics missing | RMW mismatch between shells, the DDS agent is not running, or it was started *after* PX4 |
| `abort requested: status_stale` or `lpos_stale` | Telemetry arriving in bursts. Check the physics RTF first (`sitl_stack.sh rtf`): in the one correlation trace taken here, the largest delivery gaps fell in windows where the RTF had collapsed. Start the GCS heartbeat (step 9), and do not run `gz topic` or `gz model` probes during a flight. Do **not** raise the watchdog thresholds |
| `REFUSING TO ARM: dirty pre-arm state 'nav_state_failsafe_like:4'` | PX4 is still in its AUTO_LOITER boot state. Run `sitl_stack.sh prearm` |
| `Preflight Fail: No valid data from Baro 0` / `Compass 0` | The world is missing `<atmosphere>` or `<magnetic_field>`. Both are declared in `sim/worlds/forest_map2.sdf`; if you swapped in another world, add them |
| EKF height drifts metres while the vehicle sits on the ground | Same cause as above: no barometer |
| Vehicle flies away from the hold point on takeoff | The world's `<magnetic_field>` does not match its `<spherical_coordinates>`, so the EKF heading is biased. `tests/test_world_and_obstacle.py` checks the two agree |
| Gazebo aborts with `ODE INTERNAL ERROR 1 ... collide()` | The physics engine was switched to ODE. This world needs DART |
| Second armed run misbehaves during SETTLE | One armed run per stack; see step 14 |
| `WARNING: Gazebo fell back to software rendering` | Check the physics RTF. A couple of EGL probe warnings at startup are normal; a low RTF is not |
| Gazebo fails to load the world | Fuel models not in the cache; see step 4 |
| Tracking error grows steadily | Physics real-time factor below ~0.95; see step 6 |
| `sha256sum -c hashes.txt` fails after regeneration | Check you are not hashing `manifest.yaml`, which carries a timestamp |
| `pytest` fails with `ModuleNotFoundError: No module named 'lark'` | A sourced ROS 2 environment puts `/opt/ros/.../site-packages` on `PYTHONPATH`, and pytest auto-loads the `launch_testing` plugin it finds there. Run the offline tests in a shell without ROS sourced, or use `env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest`. |
| Trajectory metrics differ in the last digits | NumPy/SciPy version difference; see the offline environment note |
