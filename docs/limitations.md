# Limitations

Stated up front so a reader can calibrate every number in
[results.md](results.md) before reading it.

## Validation scope

**Two classes of result, with different standing.** The planning and
trajectory-refinement results are deterministic offline computations based on
the Gazebo world. Tracking, state-consistency and Stop–Replan–Resume results
originate from controlled PX4 SITL / Gazebo runs. Neither class is a
real-flight result.

**X650 geometry, X500 dynamics.** The planning and collision geometry is sized
toward the Holybro X650 target platform. Mass, inertia, propulsion and runtime
flight dynamics remain those of the PX4 X500 SITL model. The reported execution
results are therefore **X650-geometry-informed SITL results**, not an
autonomous real-flight validation of the X650.

Adapted to the X650: the planning radius `r_eff` = 0.65 m, derived from the
vendor's published 650 mm wheelbase and 15 inch propellers as
0.3250 + 0.1905 + 0.1345 m, that is the propeller-tip envelope of 0.5155 m plus
a 0.1345 m safety margin. `r_eff` is a **safety and planning margin, not a
physical airframe dimension**.

**The 0.46 m footprint half-extent has no measurement provenance.** It has
been described as a measured platform value and as the X650 arm-tip
half-extent. It is neither. A search of the development record found only
assertions, and the change that introduced it says the value was pinned to the
X650, not measured. It is an **assumed** axis-aligned planning footprint. The
X650 arm tip in an X configuration is at 0.3250 / √2 = 0.2298 m and the
propeller-swept half-extent is 0.4203 m, so the ±0.46 m square is conservative
as a square, but that is the only defensible claim for it. No code in this
repository reads the footprint polygon.

Two further corrections belong here. An earlier nominal radius of 0.50 m was
rejected for public planning because it does not enclose the 0.5155 m
propeller-tip envelope; all published planning results use `r_eff` = 0.65 m
from `sim/config/robot_map2_safe.yaml`, and the 0.50 m value is not shipped as
a configuration. And the invariant
`robot_radius ≥ inscribed_circle_radius(footprint)` has been removed: the
inscribed circle is the non-conservative bound for a square, the conservative
bound is the circumscribed circle at 0.650538 m, and `robot_radius` = 0.50 m
does not satisfy it. The derivation as it now stands lives in
`sim/config/robot_map2_safe.yaml`, the only robot geometry this release ships.

Not adapted: mass, inertia, propulsion, motor layout, closed-loop flight
dynamics and the LiDAR mount offset. Everything that depends on closed-loop
dynamics — tracking error, stopping behaviour, hover transitions and the rejoin
transient — carries the X500 model's dynamics and must be read as such.

**Evidence is single controlled runs, not repeated trials.** Each reported
outcome — the nominal mission, the estimator-consistency audit, the
stop–replan–resume run — comes from one run under one configuration. The
development history contains further runs, but those were iterative runs under
changing software and configuration, not repetitions under fixed conditions.
They therefore cannot be pooled into a success rate, and none is claimed.

**No statistical robustness claim.** No confidence interval, no variance
estimate, no Monte Carlo study. Establishing robustness would require repeated
trials under a fixed configuration with randomised initial conditions.

## Method scope

**No custom SLAM or sensor-fusion method.** State estimation is PX4's EKF2,
used as supplied. The ground-truth comparison in
[results.md](results.md#4--ground-truth-vs-px4-estimate-estimation) characterises the
transform and state consistency of the integration; it is not an
estimator-accuracy benchmark and no estimation contribution is claimed.

**Global planning is 2D at a constant reference altitude.** The evaluated
mission flies at a fixed 2 m AGL reference. The altitude adaptation is handled
by rasterising tree canopies at that altitude into the prior map, not by
planning in three dimensions. A different reference altitude requires a
regenerated map.

**One obstacle in the stop–replan–resume scenario.** Multi-obstacle handling,
false-positive policy, observation gating and moving obstacles are out of
scope. The detector confirms a single cluster that is novel with respect to
the prior map.

**Detection is 2D.** A single-plane LiDAR at the flight altitude. Obstacles
that do not intersect that plane are invisible to it.

**The prior map is generated from the world description, not sensed.** This is
a deliberate choice for reproducibility: the planning input is exactly
recoverable from a text file. It means the planning results do not include
mapping error, and a real deployment would have to supply an equivalent map
from another source.

## Execution constraints

**No armed SITL run has been completed with this release.** The canonical
thesis evidence in `results/nominal_mission/` and `results/stop_replan_resume/`
comes from the private development tree. When the code in *this* repository was
pointed at a live stack, ten armed attempts were made and none reached the end
of the mission. The best of them reached, in order, `WAIT_STATUS`, `WAIT_LPOS`,
`WAIT_EKF_READY`, `PRE_STREAM`, `DECIDE_ARM`, `WAIT_OFFBOARD`, `WAIT_ARMED`,
`SETTLE_HOLD`. It armed, entered OFFBOARD and held the settle setpoint to a
lateral error of max 0.060 m and mean 0.034 m over 107 samples while climbing
from 0.63 m to 1.45 m, then aborted on `lpos_stale`, landed and disarmed
cleanly with no force-disarm. It never reached `MISSION`, so no detection,
stop, replan, resume or rejoin was exercised. The record is
`results/public_runtime_validation/srr_armed_attempt/`, whose `verdict.yaml`
is `FAIL` with every condition from `perception_sensed_obstacle_built` onward
`false`. There is likewise no successful public nominal armed smoke test.

**The blocker is telemetry delivery, and it is measured.** The executor aborts
on stale telemetry: `--lpos-watchdog-s 0.5` on
`/fmu/out/vehicle_local_position` and `--watchdog-s 3.0` on
`/fmu/out/vehicle_status_v1`. On the machine used, the uXRCE-DDS path delivers
in bursts.

| Topic | PX4 publishes every | Delivered, median gap | p95 | max |
|---|---|---|---|---|
| `vehicle_status_v1` | 0.503 s, from its own timestamps | 0.74 s | 2.82 s | 6.21 s |
| `vehicle_local_position` | 50 ms | 0.001 s | 0.33 s | 1.24 s |

Messages are late, not lost: across a 6.21 s wall-clock gap the PX4 timestamps
either side differ by 0.509 s. A separate correlation trace, taken earlier in
the session on a different stack configuration and **not** included in the
published artifacts, found the four largest delivery gaps each falling in a
window where the Gazebo real-time factor dipped to 0.002 to 0.004 against a
0.99 baseline, with PX4's own DDS client reporting a 3.146 s worst-case cycle.
That is a diagnostic lead, not a recomputable property of the recorded run.
**No threshold was raised.** A watchdog that fires on stale telemetry is doing its job, and
loosening it enough to pass on this machine would make it useless as a safety
device.

**The executor paces on wall-clock time.** A valid run requires the simulator
to run at a real-time factor near 1.0. Below that, the setpoint cursor
advances faster than the simulated vehicle, position error accumulates, the
controller over-speeds to catch up, and tracking degrades — a simulation
artefact, not a control-design property. On the public-release stack, after the
world file was given an explicit `<physics>` block, a pre-arm sampling window
measured median 0.994, fifth percentile 0.991, minimum 0.990; the equivalent
window before the fix gave a fifth percentile of 0.005. Both are pre-arm
measurements: no real-time factor was recorded during the armed attempt itself,
and none was recorded for the canonical runs, so none is quoted for either.

**One armed run per simulator stack, at present.** After an armed run the
vehicle, the estimator and the persisted PX4 parameter store are left in a
state that reliably makes the next run abort during settle. The remedy is
exact and cheap: `bash tools/sitl_stack.sh stop`, which also removes the
scratch run directory holding PX4's parameter store, then start the stack
again, which recreates it. This is a **current runtime
limitation of the tooling, not a fundamental property** of the approach; it
reflects state that is not reset between runs, and resetting it properly is
ordinary engineering work.

**The local replan is accepted at a lower margin than the nominal plan.** The
published detour is classified `boundary_smoothed`: its minimum clearance
(0.808 m) exceeds the effective radius `r_eff` = 0.65 m but not the preferred
radius `r_pref` = 0.90 m. It is collision-free with margin, but it has less
margin than the nominal plan, and that difference is a property of the local
geometry, not a general result.

**Reproducibility is asymmetric.** The offline chain is deterministic by
construction and verified byte-identical in this release, and the
Stop–Replan–Resume replan *geometry* reproduces exactly with it. SITL
execution is not deterministic and is not claimed to be, so the timing and
tracking figures of a flown run cannot be reproduced. See
[reproducibility.md](reproducibility.md) and
[data_availability.md](data_availability.md).

## Not certified

This is research software. It is **not safety-certified**, has not undergone
any airworthiness or functional-safety assessment, and is not approved for
flight on real hardware. See [safety.md](safety.md).

## What is explicitly not claimed

- A novel SLAM method.
- A novel sensor-fusion or state-estimation method.
- Full autonomous real-flight validation.
- A completed armed SITL run using the code in this repository. The armed
  evidence reported in [results.md](results.md) comes from the private
  development tree; see `results/public_runtime_validation/` for what this
  repository has and has not been observed to do.
- Statistical robustness or a quantified success rate.
- Safety certification or airworthiness.
- Deterministic SITL execution.
- That the smoothing budget guarantees a refined trajectory above `r_pref`.
  It does not. The planner introduces a smoothing budget above `r_pref`
  because refinement consumes margin; every refined candidate is then
  validated and classified by its **resulting** clearance and motion
  feasibility. The published nominal trajectory reached `robust_smoothed`; the
  published local detour reached only `boundary_smoothed`.
- A general anti-monotonicity result for clearance under smoothing. The
  budget is a design decision in this pipeline, not a proved general property.
- A "clearance-preserving reformulation". No such reformulation is claimed.
- A general global-versus-local replanning speedup. The offline global replan
  and the in-flight local replan solved different problems over different
  goal distances on different code paths, so their runtimes are not a
  controlled comparison and no ratio is quoted.
