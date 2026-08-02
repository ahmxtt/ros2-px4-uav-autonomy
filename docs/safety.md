# Safety

## Status

This is **research software validated in simulation only**: deterministic
offline computation plus controlled PX4 SITL / Gazebo runs. It is not
safety-certified, has undergone no airworthiness or functional-safety
assessment, and must not be flown on real hardware without an independent
safety case.

The planning and collision geometry is sized toward the Holybro X650 from its
published wheelbase and propeller diameter; mass, inertia, propulsion and
runtime flight dynamics remain those of the PX4 X500 SITL model. Any safety
argument derived from the stopping distances, hover transitions or tracking
bounds reported here is an argument about that simulation, not about an X650
in flight.

## Arming is guarded by two independent flags

The Offboard executor commands mode change and arming only when **both**
`--arm` and `--i-have-physical-eyes-on-the-sim` are supplied. Either flag alone
keeps the run in dry-run mode: the state machine reaches `DECIDE_ARM` and
branches to `DONE` before any `VehicleCommand` can be constructed. The default
is dry-run.

The second flag is deliberately awkward to type. It exists so that arming
cannot be reached by shell history or by copy-pasting a command without
reading it.

### What dry-run actually does

Dry-run is **not silent**. Without arm permission the executor still publishes
the pre-arm Offboard heartbeat and a hold setpoint, because PX4 requires a
setpoint stream to be present *before* an OFFBOARD mode change would be
accepted. What it withholds are the commands that change vehicle state.

| | Topic / message | Dry-run |
|---|---|---|
| Published | `/fmu/in/offboard_control_mode` (`OffboardControlMode`) | **yes** |
| Published | `/fmu/in/trajectory_setpoint` (hold setpoint at the takeoff position) | **yes** |
| Subscribed | `vehicle_status_v1`, `vehicle_local_position`, failsafe flags, battery | yes |
| **Withheld** | `VehicleCommand` `DO_SET_MODE` (OFFBOARD entry) | **never sent** |
| **Withheld** | `VehicleCommand` `COMPONENT_ARM_DISARM` (arm / disarm) | **never sent** |
| **Withheld** | `VehicleCommand` `NAV_LAND` | **never sent** |

It ends with `VERDICT=DRY_RUN_OK`.

**What a dry run does and does not cover.** The dry run verifies ROS/PX4
connectivity, LiDAR message reception, argument handling, pre-arm Offboard
heartbeat and hold-setpoint streaming, the two-factor arm gate and output
initialisation. It does not execute obstacle detection, stop, local
replanning, resume or rejoin, because these states are reached only during an
armed mission.

Concretely: a dry run is any run without **both** `--arm` and
`--i-have-physical-eyes-on-the-sim`. It reaches exactly 6 of the parent node's
13 states,

```
WAIT_STATUS → WAIT_LPOS → WAIT_EKF_READY → PRE_STREAM → DECIDE_ARM → DONE
```

stopping at `offboard_mission_executor.py:1373-1384`. It publishes
`OffboardControlMode` and a hold `TrajectorySetpoint` after EKF-ready for about
`--pre-stream-s` seconds. It never publishes any `VehicleCommand` at all: no
`DO_SET_MODE`, no `COMPONENT_ARM_DISARM`, no `NAV_LAND`, no force-disarm. The
publisher is advertised and never used.

For `stop_replan_resume_node`, obstacle detection, `STOP_HOVER`, replan, resume
and rejoin are all unreachable in a dry run. The detection callback begins with
`if self._state != "MISSION": return`
(`stop_replan_resume_node.py:521-522`), and `MISSION` is entered only from
`SETTLE_HOLD`, which is entered only after a confirmed arm. Any statement that
the dry run exercises the full state machine, or the perception path, is false.

Every validation command in this repository is **SITL-only**. None of them is
intended for, or safe on, real hardware.

## Two support tools that cannot command the vehicle

A run needs two things that are not the executor, and both are deliberately
incapable of moving the aircraft.

**`tools/spawn_srr_obstacle.py`** puts the obstacle into the running Gazebo
world, because the scenario is meaningless if the 2D LiDAR cannot see it. It
talks to Gazebo over `gz service /world/<world>/create` and to nothing else: it
never contacts PX4, never publishes a ROS topic, never touches the vehicle, and
cannot arm anything. It spawns at most once per world, verifies the resulting
pose, and records what it did in `spawn_record.yaml`. `--verify-only` checks
presence and pose without spawning at all.

**`tools/gcs_heartbeat.py`** exists because PX4 expects a ground control
station to be connected, and without one the pre-flight checks complain and
`vehicle_status` is republished irregularly enough to trip the executor's 3.0 s
status watchdog while the vehicle is still on the ground. It sends **only**
`HEARTBEAT`, with component type `MAV_TYPE_GCS`. It sends no command, does not
arm, does not change mode, and uploads no mission. It cannot make the vehicle
do anything. It is the part of a GCS that matters here, without the real-time
factor cost of a GUI.

## Continuous supervision, not end-of-run checking

While armed, the executor monitors on every tick:

| Watch | Action on violation |
|---|---|
| Navigation state left OFFBOARD | abort, controlled land |
| Rolling tracking-error window exceeded | abort, controlled land |
| Absolute altitude cap exceeded | abort, controlled land |
| PX4 failsafe flag raised | recorded, abort path |
| Arming state changed unexpectedly | recorded, abort path |

An abort holds the vehicle's current position as the setpoint rather than
continuing along the reference, then commands land.

**The disarm path, as configured and as observed.** The configured sequence is:
send Land, enter `WAIT_DISARM`, and if no disarm is observed within 30 s of the
Land command, send the force-disarm fallback. That is the configuration; it is
not a statement about how often the fallback fires. In the best armed attempt
recorded for this release, the abort path ran to completion: land and disarm
were both observed and **no force-disarm was sent**
(`results/public_runtime_validation/srr_armed_attempt/verdict.yaml` records
`land_sent: true`, `disarm_seen: true`, `force_disarm_not_sent: true`). No
broader claim about the fallback is made, because no run in this release has
exercised it.

A run that aborts is recorded as a failure **with its reason**. It is never
silently truncated into a shorter successful run.

## One armed run per simulator stack

A run that ends in a force-disarm leaves the vehicle and the estimator in a
state that reliably makes the next run abort during settle. Restart PX4,
Gazebo and the DDS agent between armed runs and clear the persisted parameter
store. `bash tools/sitl_stack.sh stop` does both, then start the stack again.
The executor's dirty-state guard refuses to arm from a residual `OFFBOARD`,
`AUTO_LOITER`, `AUTO_RTL` or `AUTO_LAND` navigation state, but it can only
refuse — it cannot repair the stack. PX4 boots into `AUTO_LOITER`, so
`bash tools/sitl_stack.sh prearm` switches to Position mode before an armed
run; without it the guard correctly reports a dirty pre-arm state.

## Real-time factor is a safety-relevant precondition

The executor advances its setpoint cursor on wall-clock time. If the simulator
runs slower than real time, the reference advances faster than the vehicle,
position error accumulates, and the controller over-speeds to catch up. Verify
that the real-time factor is near 1.0 **before** arming, with
`bash tools/sitl_stack.sh rtf`; a low factor produces tracking failures that
look like control problems but are not. It appears to produce telemetry stalls
too: the armed attempt recorded in `results/public_runtime_validation/` aborted
on a stale-position watchdog, and a separate earlier trace on a different stack
configuration found the largest delivery gaps falling in windows where the
real-time factor had collapsed to 0.002 to 0.004. That trace is not published
and the link is a diagnostic lead rather than a measured property of the
recorded run. See
[limitations.md](limitations.md#execution-constraints).

## Obstacle reaction is stop-first

On a confirmed detection the vehicle stops and establishes a stable hover
before any replanning begins. Planning does not run while the vehicle is
moving toward the obstacle. Resume happens only after the replanned path has
passed clearance and motion validation.

If validation fails, the replan is rejected. The unsmoothed polyline fallback
that the offline refinement layer allows is **not** accepted during a live
run: a
controlled abort is preferred to continuing on a degraded reference.

## Clearance margins

| Symbol | Value | Meaning |
|---|---|---|
| `r_eff` | 0.65 m | Effective planning radius: the 0.5155 m propeller-tip envelope of the X650 plus a 0.1345 m safety margin. Hard floor. Not an airframe dimension. |
| `r_pref` | 0.90 m | Preferred radius: `r_eff` + tracking margin. |
| `r_plan_min` | 1.10 m | Planning radius: `r_pref` + budget the spline may consume. |
| `r_comfort` | 1.20 m | Upper edge of the comfort band. |

The nominal plan is searched at `r_plan_min`, above the radius the trajectory
must finally satisfy, which gives refinement room to consume. This is a
budget, not a guarantee: the refined result is validated on dense samples and
classified by its resulting clearance. The published nominal trajectory
reached 1.082 m (`robust_smoothed`); the published local detour reached only
0.808 m (`boundary_smoothed`) — above `r_eff` but below `r_pref`.

The hard floor is `r_eff`. A candidate below it is rejected outright.

## Before any real-hardware use

Nothing in this repository has been validated on real hardware. At minimum a
real deployment would need: a source of the prior map other than the
simulation world; validation of the LiDAR mount geometry against the actual
airframe; a real-time execution path that does not pace on wall clock; an
independent geofence and failsafe layer; and a safety case appropriate to the
operating environment.
