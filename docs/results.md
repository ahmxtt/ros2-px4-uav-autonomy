# Results

Read [limitations.md](limitations.md) first. There is no real-flight validation
of any result on this page. The simulated dynamics are those of the PX4 X500
model with X650-informed planning geometry, see
[X650 geometry, X500 dynamics](#x650-geometry-x500-dynamics) below.

## Five result classes that must not be mixed

Everything published under `results/` belongs to exactly one of these classes.
They differ in what produced them and in what may be claimed about reproducing
them.

**Class 1, offline nominal planning.** `results/planning/`. Occupancy grid,
A\* path, line-of-sight reduction and cubic B-spline refinement, computed
offline from the Gazebo world file. Sections 1 and 2 below. The A\* cell path
is asserted cell for cell against the published `path.csv`; the remaining
geometry and the derived floating-point metrics reproduce within the tolerances
tabulated under [Provenance](#provenance).

**Class 2, offline Stop-Replan-Resume (SRR) geometry.** The replan geometry in
`results/stop_replan_resume/`, together with `tools/reproduce_srr_geometry.py`.
The detour is re-derived offline from the world file, the published sensed
obstacle and the published mission segment window. The A\* node count is
compared exactly; the geometry and the metrics within the documented
tolerances.

**Class 3, canonical controlled SITL evidence.** `results/nominal_mission/` and
the runtime parts of `results/stop_replan_resume/`: detection, stop, online
replan timing, resume, rejoin and mission outcome. Sections 3 to 5 below. These
are single PX4 SITL runs recorded with the private development tree. They are
measurements of specific flights, they are not regenerable, and they carry no
statistical weight.

**Class 4, public-release runtime validation.**
[`results/public_runtime_validation/`](../results/public_runtime_validation/README.md).
Produced by *this* repository rather than the private development tree, on one
laptop, on 2 August 2026, as a compatibility check on the release rather than
as thesis evidence. Two things were attempted and the outcome was mixed. The
physical Gazebo obstacle spawn is verified: `tools/spawn_srr_obstacle.py` took
the world from 88 to 89 models with exactly one occurrence of
`srr_unknown_obstacle`, the requested pose (−21.330, −43.710, 1.500) m was read
back out of Gazebo with a delta of (0.000, 0.000, 0.000) m, and a second
invocation reported `already_present` and spawned nothing. The armed
perception-driven SRR run did **not** complete. It reached, in order,
WAIT_STATUS, WAIT_LPOS, WAIT_EKF_READY, PRE_STREAM, DECIDE_ARM, WAIT_OFFBOARD,
WAIT_ARMED and SETTLE_HOLD; it armed, entered OFFBOARD and held the settle
setpoint to a lateral error of 0.060 m max and 0.034 m mean over the 107
SETTLE_HOLD samples; it then aborted on `lpos_stale`, landed and disarmed with
no force-disarm. It never entered MISSION, so no detection, stop, replan,
resume or rejoin was exercised, and `srr_armed_attempt/verdict.yaml` records
`false` for every condition from `perception_sensed_obstacle_built` onward. The
cause was measured rather than guessed: uXRCE-DDS delivered in bursts, with
`vehicle_local_position` inter-arrival gaps of 0.001 s median, 0.33 s p95 and
1.24 s max against the 0.5 s `--lpos-watchdog-s` budget, and
`vehicle_status_v1` gaps of 0.74 s median, 2.82 s p95 and 6.21 s max against
`--watchdog-s 3.0`. The messages were late, not lost. A separate correlation
trace, taken earlier in the session on a different stack configuration and not
included in the published artifacts, found the four largest gaps each falling
in a window where the Gazebo real-time factor had dipped to 0.002 to 0.004
against a 0.99 baseline; that is a diagnostic lead, not a recomputable property
of this run. The healthy figures quoted elsewhere (median 0.994, p05 0.991)
come from a pre-arm sampling window, not from the armed attempt, for which no
real-time factor was recorded. No watchdog threshold was raised.

**Class 5, there is no successful public nominal armed smoke test.** Nothing in
this repository demonstrates a complete armed mission flown by the released
code. The completed missions reported in sections 3 to 5 are class 3, recorded
with the private development tree. Class 4 is not a public replication of them
and must not be presented as one.

## The five measurements below

The sections below measure different things on different runs. They are kept
separate because combining them would be wrong:

| # | Measures | Class | Source | Regenerable |
|---|---|---|---|---|
| 1 | Global planning geometry | 1, offline | Gazebo world file | cell path exactly, scalars to tolerance |
| 2 | Trajectory refinement geometry | 1, offline | the plan above | geometry and metrics to tolerance |
| 3 | **Control**: setpoint vs estimate | 3, SITL | live run A | no |
| 4 | **Estimation**: ground truth vs estimate | 3, SITL | live run B | no |
| 5 | Obstacle reaction | 3, SITL; geometry is class 2 | live run C | geometry yes, timing no |

Sections 3 and 4 are the pairing most easily confused. Section 3 asks *did the
vehicle follow the commanded reference*. Section 4 asks *did the vehicle know
where it was*. They come from different runs and different quantities; neither
bounds the other.

Policy radii used throughout: `r_eff` = 0.65 m, `r_pref` = 0.90 m,
`r_plan_min` = 1.10 m, `r_comfort` = 1.20 m, from
`sim/config/robot_map2_safe.yaml` and `sim/config/mission_policy.yaml`. What
each of them is, and is not, is set out under
[X650 geometry, X500 dynamics](#x650-geometry-x500-dynamics).

## The mission reference (an input, not a measurement)

`results/mission_reference/` holds the complete multi-POI mission the vehicle
flies in sections 3 to 5: out from the takeoff point, two points of interest
with a two-turn orbit at each, and back. It is what a run consumes, not
evidence about a run.

| Quantity | Value |
|---|---|
| Samples | 16 157 |
| Segments | 5, three legs and two orbits |
| 1 `start_to_poi1`, leg | 5179 samples, 101.989 m |
| 2 `orbit_poi1`, orbit | 1685 samples, 88.179 m, r = 6 m, 2 turns, ccw, about (15, 5) |
| 3 `poi1_to_poi2`, leg | 2572 samples, 50.785 m |
| 4 `orbit_poi2`, orbit | 1673 samples, 87.558 m, r = 6 m, 2 turns, ccw, about (−30, 24) |
| 5 `poi2_to_start`, leg | 5048 samples, 100.141 m |
| Total length | **428.651 m** |
| Reference duration | 286.159 s |
| Minimum clearance | 1.040 m |
| Frame | Gazebo world ENU, planar; the 2.0 m reference altitude enters downstream as `z_ned_m` = −2.0 |
| First sample | world (−50.025, −69.975), 0.035 m from the vehicle spawn point |

Columns: `segment_id, segment_type, segment_name, s_m, x_m, y_m, yaw_rad,
curvature_1_per_m, v_ref_mps, t_ref_s`. `s_m` and `t_ref_s` are cumulative over
the whole mission, not per segment. The Stop-Replan-Resume event of section 5
happens on segment 1, about 39.4 m along it.

This file is a **curated public copy** of the canonical artifact, byte-identical
to it (`sha256 5870ffd2f0e89b92a7794d858e7757fdf88f4001980b0375b7219a2f8ab4864e`),
and it is a derived mission product rather than raw data. This repository does
**not** regenerate it: the multi-leg policy planner, the orbit generator and the
mission stitcher are not part of the release. Treat the CSV as a fixed input and
verify it against `hashes.txt`.

Two files are easily confused with it.
`results/stop_replan_resume/mission_segment_window.csv` is an exact row-for-row
slice of it, segment 1 from s = 15.019014 m to s = 71.995748 m.
`results/planning/path.csv` is a different, standalone single-leg planning
result; it is not segment 1 of this mission and the two must not be compared.

Data: [`results/mission_reference/manifest.yaml`](../results/mission_reference/manifest.yaml),
`mission_metrics.yaml`, `segment_metrics.yaml`, `transition_metrics.yaml` and
[`README.md`](../results/mission_reference/README.md).

---

## 1 · Nominal planning

Single leg (start → orbit-1 entry) on the execution prior map at the 2 m
reference altitude. Grid 2913 × 3911 at 0.05 m/cell, origin (−80.30, −100.25).

| Quantity | Value |
|---|---|
| Planner | A\*, 8-connected, octile, no corner cutting, hard inflation at `r_plan_min`, comfort cost in [1.10, 1.20) |
| Path length | **102.139 m** |
| Straight-line distance | 97.541 m (ratio α = 1.047) |
| Path points | 1543 |
| Nodes expanded | 230 047 |
| Minimum clearance to raw obstacles | **1.200 m** |
| Mean clearance | 2.820 m |
| Classification | `robust` |

Clearance is measured to the **raw, un-inflated** obstacles — the real margin,
not the inflated proxy the search ran on.

Data: [`results/planning/metrics.yaml`](../results/planning/metrics.yaml),
`path.csv`.

### Line-of-sight reduction

Applied to the same path on the grid inflated at the radius the path was
planned with, `r_plan_min` = 1.10 m: **1543 → 8 waypoints**, length
102.139 m → 98.561 m, all four invariants hold (endpoints preserved,
full-segment line of sight, length monotone, clearance floor respected).
`tools/reproduce_offline_demo.py` computes this as its step 3 and writes the
reduced list to `02_plan/path_reduced.csv`; `tools/reduce_path_los.py` gives the
same result with `--inflation-radius-m 1.10`. Run with its default radius
(`robot_radius` = 0.65 m, which is not the radius this path was planned with),
the same tool returns 1543 → 5 waypoints with a 0.652 m clearance minimum.

**The reduction is a separate, diagnostic stage. It is not the input to the
B-spline refinement below**, which is fitted to the full A\* path
(`tools/reproduce_offline_demo.py`, `tools/refine_trajectory.py`), and it is not
part of the flown mission reference. The reduced waypoint list is not one of
the published files: `results/planning/` ships the A\* path and the refined
trajectory, and no published result file records the 1543 → 8 reduction.

This reduction is reported for the leg above. Do not transfer the ratio to a
different map: the input point count depends on the grid and the path.

---

## 2 · Trajectory refinement

Cubic B-spline refinement of the path from section 1. This is the **single
validated leg**, not the flown multi-POI mission.

| Quantity | Value |
|---|---|
| Selected attempt | `bspline_s0_2.00m` (first of the cascade) |
| Classification | `robust_smoothed` |
| Control points | 53 |
| Dense samples | 5108 |
| Length | **100.557 m** (−1.549 % vs A\*) |
| Minimum clearance | **1.082 m** (≥ `r_pref` 0.90 m) |
| Mean clearance | 2.930 m |
| 10th-percentile clearance | 1.204 m |
| Max deviation from A\* | 0.252 m |
| Mean deviation from A\* | 0.029 m |
| Max curvature | 0.545 1/m |
| Collision-free | yes |

### Speed profile

| Quantity | Value |
|---|---|
| Nominal speed | 1.5 m/s |
| Observed speed range | 1.355 – 1.500 m/s |
| Lateral-acceleration limit | 1.0 m/s² |
| Observed maximum lateral acceleration | **1.000 m/s²** — at the limit, not above |

The profile saturates its lateral-acceleration budget. Any future relaxation
of `a_lat_max` requires re-validating the smoother envelope.

### Why clearance drops from 1.200 m to 1.082 m

The spline cuts corners, so it consumes margin. The planner therefore searches
at `r_plan_min` = 1.10 m rather than at `r_pref` = 0.90 m: the 0.20 m surplus
is a **smoothing budget** the refinement is permitted to spend.

This budget does **not** guarantee that a refined candidate stays above
`r_pref`. It makes that outcome likely for well-conditioned geometry, nothing
more. Every refined candidate is subsequently validated on dense samples and
classified by its resulting clearance and motion feasibility. Here the
selected candidate landed at 1.082 m and was classified `robust_smoothed`; in
the Stop–Replan–Resume case (section 5) the same mechanism produced
`boundary_smoothed`, below `r_pref`. Both outcomes are reported as measured.

Data: [`results/planning/trajectory_metrics.yaml`](../results/planning/trajectory_metrics.yaml),
`trajectory.csv`, `control_points.csv`.

---

## 3 · Nominal mission tracking (control)

Commanded setpoint vs PX4/EKF local position, MISSION phase only (takeoff,
settle and land excluded). One PX4 SITL run, verdict PASS, 5/5 segments.

| Quantity | Value |
|---|---|
| Samples | 2922 |
| Horizontal error, mean | **0.220 m** |
| Horizontal error, RMSE | **0.234 m** |
| Horizontal error, median | 0.220 m |
| Horizontal error, p95 | **0.378 m** |
| Horizontal error, max | **0.609 m** |
| 3D error, RMSE / max | 0.235 m / 0.609 m |
| Tracking budget | 1.0 m — no window breach |
| Setpoint rate | 20.0 Hz median (p95 20.1 Hz) |
| Altitude | 1.998 m mean (reference 2.0 m), range 1.813 – 2.059 m |
| Horizontal speed | 1.583 m/s mean, 1.832 m/s max |
| Failsafe / OFFBOARD dropout | none |

**Definition matters, and precision matters.** The figures above are the
*horizontal Euclidean* error `sqrt(err_x² + err_y²)`. A max-absolute-component
definition `max(|err_x|, |err_y|, |err_z|)` over the same samples gives
mean 0.197 m, p95 0.339 m, max 0.600 m. Both are correct; they are not
interchangeable, and any comparison must state which is meant.

All values are stored at full float precision in
`results/nominal_mission/tracking_metrics.yaml` and quoted here rounded to
three decimals. The horizontal maximum is `0.60945917 m` → **0.609 m**. The
thesis quotes `0.600 m` for the same run, which is the *max-absolute-component*
maximum (`0.599829 m`). The two numbers describe the same flight under two
definitions; neither supersedes the other.

Data: [`results/nominal_mission/tracking_metrics.yaml`](../results/nominal_mission/tracking_metrics.yaml).

---

## 4 · Ground truth vs PX4 estimate (estimation)

Gazebo ground-truth pose against PX4/EKF local position over a complete
nominal mission. **Separate run** from section 3, with ground-truth recording
enabled. Airborne subset (height > 0.5 m).

| Quantity | Value |
|---|---|
| Samples | 14 869 |
| Horizontal RMSE | **0.226 m** |
| Horizontal mean / median | 0.200 m / 0.200 m |
| Horizontal p95 / max | 0.396 m / 0.445 m |
| 3D RMSE / max | 0.228 m / 0.4475 m |
| Mean offset (dx, dy, dz) | 0.060 / 0.123 / 0.019 m |

Transform: `x_NED = y_world − Y0`, `y_NED = x_world − X0`, `z_NED = −z_world`,
with X0 = −50.0, Y0 = −70.0.

Time alignment is by wall clock (median Δt 4.3 ms, p95 8.2 ms). Simulation-time
alignment is **invalid** here and was not used: PX4 `lpos` timestamps are Unix
microseconds while the ground-truth stamp is simulation-relative, and their
offset is not constant (IQR 3.1 s), so a single-offset alignment would be
wrong.

**Interpretation.** Ground truth and the PX4 estimate agree at the decimetre
level across the mission. This supports the transform and state-consistency
argument for the integration. It is **not** an estimator-accuracy benchmark,
and **no SLAM or sensor-fusion contribution is claimed** — the estimator is
PX4's EKF2 used as supplied.

Data: [`results/nominal_mission/state_consistency_metrics.yaml`](../results/nominal_mission/state_consistency_metrics.yaml).

---

## 5 · Stop–replan–resume

One obstacle absent from the prior map, encountered during the nominal
mission. Detection from the 2D LiDAR only. Ground truth was retained in a
separate evaluation-only namespace: it did not gate detection, obstacle
construction, replanning or control decisions, but it was used to compute the
diagnostic and post-run accuracy figures reported below. One PX4 SITL run,
verdict PASS.

### Detection

| Quantity | Value |
|---|---|
| Sensor / mode | `/scan_filtered`, LiDAR cluster detection, perception only |
| Confirmation | 4-frame hysteresis streak |
| Sensed centroid | (−21.274, −42.772) m, radius 1.953 m |
| Novelty distance to prior map | 3.734 m |
| Ground-truth centroid *(evaluation only)* | (−21.330, −43.710) m, radius 1.5 m |
| Centroid error | 0.940 m |
| Radius error | +0.453 m — conservative (over-estimates the obstacle) |

### Stop and hover

| Quantity | Value |
|---|---|
| Detection → stop command | 0.015 s |
| Stop → stable hover | 6.4 s (stability window 3.1 s) |
| Max position error during the transition | 0.61 m |

### Local replan

| Quantity | Value |
|---|---|
| Total runtime, measured online during the controlled SITL mission | **0.251 s** (budget 3 s) |
| of which A\* | 0.137 s |
| of which smoother | 0.009 s |
| A\* nodes expanded | 16 746 |
| A\* path length | 23.525 m |
| Refined path length | **23.178 m** |
| Minimum clearance | **0.808 m** |
| Classification | **`boundary_smoothed`** |
| Max curvature | 0.734 1/m |
| Max lateral acceleration | 1.000 m/s² |
| Collision-free | yes |

**Scope of the runtime figure.** 0.251 s is `t_total_s` in
`replan_runtime_metrics.yaml`: crop, patch, clearance field, inflation, A\*,
smoothing and validation. It excludes loading the mission segment and the prior
map, which happen before it. The `started_wall_s` → `completed_wall_s` span in
the same file, which includes that loading, is **0.322 s**. In the released
code the replan runs synchronously inside the control tick, so the setpoint
stream does not tick during it; see
[limitations.md](limitations.md#known-runtime-defects-documented-not-fixed).

**`boundary_smoothed` is a real qualification, not a formality.** The detour
clears `r_eff` = 0.65 m but not `r_pref` = 0.90 m. It is collision-free with
margin, and it has *less* margin than the nominal plan. This is a property of
the local geometry under a tight ROI, and it is why the classification is
reported rather than reduced to "collision-free".

### Resume and rejoin

| Quantity | Value |
|---|---|
| Resume anchor | (−14.909, −30.767) m, index 737 in the segment |
| Position jump at handover | **0.245 m** |
| Yaw jump at handover | **11.07°** |
| Rejoin distance to the original reference | **0.006 m** |
| Detection → rejoin | 22.5 s |

**Two different quantities.** The *position jump* is the discontinuity between
the end of the local detour and the resume anchor. The *rejoin distance* is
how far the vehicle actually was from the original mission reference once it
had rejoined. 0.245 m and 0.006 m are not comparable and must not be quoted
as if one refined the other.

### Mission outcome

| Quantity | Value |
|---|---|
| Mission | completed |
| Max tracking error | 0.802 m (budget 1.0 m, no breach) |
| OFFBOARD dropout / failsafe | none |
| Land and disarm | commanded and observed, no force-disarm |

Data: [`results/stop_replan_resume/srr_metrics.yaml`](../results/stop_replan_resume/srr_metrics.yaml)
and the raw `event_log.csv`, `replan_runtime_metrics.yaml`,
`resume_join_metrics.yaml`, `sensed_obstacle_metrics.yaml`, `verdict.yaml`.

---

## X650 geometry, X500 dynamics

The planning geometry is sized toward the Holybro X650 target platform. Mass,
inertia, propulsion and runtime flight dynamics remain those of the PX4 X500
SITL model. The reported execution results are therefore **X650-geometry-informed
SITL results**, not an autonomous real-flight validation of the X650.

### What is actually known about the airframe

Two published vendor figures, and nothing else: a **650 mm wheelbase**
(diagonal, motor to motor) and **15 inch propellers**. Everything below follows
from them.

| Derived quantity | Arithmetic | Value |
|---|---|---|
| Frame radius, motor hub to centre | 0.650 / 2 | 0.3250 m |
| Propeller radius | 15 × 0.0254 / 2 | 0.1905 m |
| **Propeller-tip envelope radius** | 0.3250 + 0.1905 | **0.5155 m** |
| Arm-tip half-extent, X configuration | 0.3250 / √2 | 0.2298 m |
| Propeller-swept half-extent, X configuration | 0.2298 + 0.1905 | 0.4203 m |

0.5155 m is the smallest circle that actually contains the turning aircraft.
Any planning radius has to exceed it.

### Every radius, and what it is

| Value | Geometric meaning | Status | Used by | Includes a safety margin |
|---|---|---|---|---|
| **0.46 m** | Half-extent of the square footprint `[±0.46, ±0.46]` | **Assumed planning footprint. No measurement exists** — see below | Nothing. Documentation only; no code reads the polygon | Implicitly: it exceeds the 0.4203 m propeller-swept half-extent |
| **0.50 m** | An earlier nominal radius | **Rejected for public planning**, because 0.0155 m *smaller* than the 0.5155 m propeller-tip envelope means it is not a conservative collision radius | Nothing. Not shipped as a configuration; recorded here as history | No |
| **0.65 m** | `robot_radius` in the planning config = `r_eff` | **The planning envelope.** 0.3250 + 0.1905 + 0.1345 margin. Encloses the propeller tips with 0.1345 m to spare. Not an airframe dimension | Every published planning result | Yes, 0.1345 m over the propeller tips |
| **0.90 m** | `r_pref` = `r_eff` + 0.25 tracking margin | Clearance policy, not geometry | Candidate classification | Yes |
| **1.10 m** | `r_plan_min` = `r_pref` + 0.20 spline budget | Planning abstraction: what the comfort planner inflates to | Comfort A\* | Yes |
| **1.20 m** | `r_comfort` = `r_pref` + 0.30 | Soft preference in the comfort cost | Comfort A\* | Yes |

**On the 0.46 m footprint.** No measurement of it exists. An exhaustive search
of the development archive finds only assertions; the commit that introduced it
says the value was *pinned* to the X650, not measured. It is also not the arm
tip: that would be 0.3250 m in a plus configuration or 0.2298 m in an X
configuration. Treat it as a conservative axis-aligned planning footprint of
unverified origin, and note that nothing in this repository reads it.

**On the removed invariant.** The configuration files used to assert
`robot_radius >= inscribed_circle_radius(footprint)`, satisfied as 0.50 ≥ 0.46.
That invariant has been deleted. For a square the inscribed circle is the
**non-conservative** bound, so satisfying it says nothing about whether the
aircraft fits. The conservative bound is the circumscribed circle:

```
circumscribed_radius = sqrt(0.46² + 0.46²) = 0.650538 m
```

`robot_radius` = 0.50 m does **not** satisfy it. `r_eff` = 0.65 m matches it to
within 0.54 mm, but that is not independent corroboration: 0.46 is itself
0.65 / √2 to two decimals, so the two numbers encode the same choice twice.

**Which file the results came from.** Every published planning result was
computed from `sim/config/robot_map2_safe.yaml` — `tools/plan_astar.py`,
`tools/reduce_path_los.py`, `tools/refine_trajectory.py`,
`tools/reproduce_offline_demo.py`, `tools/make_docs_figures.py` and
`tests/test_regression_published_result.py` all read `robot_radius` from it.
It is the only robot geometry configuration this release ships, and
`tests/test_geometry_semantics.py` enforces that every planning tool reads it.

Not adapted, and therefore X500 model values: mass, inertia, propulsion, motor
layout and closed-loop flight dynamics, plus the LiDAR mount offset.

Sections 1–2 depend only on geometry and are X650-geometry-informed. Sections
3–5 depend on closed-loop dynamics — tracking error, stopping behaviour, hover
transitions and the rejoin transient are all dynamics-dependent — and carry the
X500 model's dynamics.

## Offline reproduction of the SRR geometry

The replan **geometry** of section 5 is reproducible offline:

```bash
python3 tools/reproduce_srr_geometry.py --output-dir /tmp/srr
```

It rebuilds the map from the world file, reads the *sensed* obstacle from the
published bundle, and re-derives the detour. A* node count and path length
match exactly; the refined path matches to the six decimals the recorded CSV
stores; the recomputed rejoin continuity reproduces 0.245 m and 11.07°.

The **timing** of section 5 is not reproducible offline and the tool does not
pretend otherwise: it prints its own offline recomputation time separately and
labels it as a different quantity from the online figure.

## Provenance

Sections 1–2, and section 5's geometry, reproduce from this repository, with
`pytest` enforcing all three. Be precise about what "reproduce" means here:

- the occupancy grid rebuilds deterministically, and byte-identity is asserted
  in `tests/test_regression_published_result.py` against a PGM the fixture
  itself writes. No PGM is published, so byte-identity against a *published*
  artifact is not demonstrated;
- the A\* cell path reproduces **cell for cell** against
  `results/planning/path.csv`, asserted in the same test file. The quickstart
  tool `tools/reproduce_offline_demo.py` does not do this — it compares five
  scalars;
- integer counters (nodes expanded, sample counts) are exact;
- B-spline geometry and derived floating-point metrics reproduce within the
  documented tolerances: 1e-9 in the tests, 1e-6 for scalar geometry in the
  demo, 1e-6 m on the SRR refined path.

Sections 3–4 and the timing of section 5 are measurements of specific live runs
and are not regenerable: SITL execution is not deterministic and no claim to
the contrary is made anywhere. Each bundle ships its own `manifest.yaml` and
`hashes.txt`.

One further exact reproduction, added in this release: the PX4 setpoint stream.
`tools/build_ned_setpoints.py` regenerates the stream the canonical armed run
consumed **byte for byte** from `results/mission_reference/mission_reference.csv`
— 5725 rows, SHA-256
`bcc70fb896454d5e0a98fd46812bbd4513670891bac2708298871ec611e5d81e`, the hash
recorded as that run's input.

See [reproducibility.md](reproducibility.md) and
[data_availability.md](data_availability.md).
