# Architecture

A sense–plan–act stack split into an algorithmic core with no ROS dependency
and a thin runtime layer that consumes it.

The system was developed through an **incremental validation-gate framework**;
gate labels appear throughout the source and the result bundles. They are
defined in [validation_gates.md](validation_gates.md), in short:

| Gate | Scope |
|---|---|
| **Gate 4** | Coordinate transforms, communication and basic PX4 Offboard execution |
| **Gate 5** | Nominal mission preparation and execution |
| **Gate 6** | Obstacle response and Stop–Replan–Resume validation |

*Stop–Replan–Resume* (SRR) is this project's name for the obstacle-response
sequence: stop, plan a local detour, resume the original mission. The
abbreviation is used freely below.

```
                    sim/worlds/forest_map2.sdf
                                │
                    ┌───────────▼───────────┐
                    │ uav_autonomy.mapping  │  SDF → occupancy grid
                    └───────────┬───────────┘  (PGM + map.yaml)
                                │
                    ┌───────────▼───────────┐
                    │ uav_autonomy.grid     │  inflation, world↔cell,
                    └───────────┬───────────┘  clearance field
                                │
                    ┌───────────▼───────────┐
                    │ uav_autonomy.planning │  deterministic A*
                    └───────────┬───────────┘  (3 cost variants)
                                │
              ┌─────────────────┼─────────────────┐
              │                                   │
  ┌───────────▼───────────┐          ┌────────────▼──────────┐
  │ uav_autonomy.smoothing│          │ uav_autonomy.trajectory│
  │ line-of-sight         │          │ B-spline cascade +     │
  │ reduction (polyline)  │          │ speed profile          │
  └───────────────────────┘          └────────────┬──────────┘
                                                  │
                                     ┌────────────▼──────────┐
                                     │ uav_autonomy.frames   │  ENU → NED
                                     └────────────┬──────────┘
                                                  │
   ═══════════════════════════ ROS 2 boundary ════╪══════════════════════
                                                  │
                                  ┌───────────────▼──────────────┐
                                  │ ros2/nodes/                  │
                                  │   offboard_mission_executor  │  20 Hz
                                  └───────────────┬──────────────┘
                                                  │
        2D LiDAR ──────────────►  ┌───────────────▼──────────────┐
        /scan_filtered            │   stop_replan_resume_node    │
                                  └───────────────┬──────────────┘
                                                  │
                                  ┌───────────────▼──────────────┐
                                  │ uav_autonomy.replanning      │  local ROI
                                  └──────────────────────────────┘
```

## Why the split

The core is importable and testable from a plain Python environment. That is
what makes the planning results reproducible without a simulator, and it is
why `tests/` can pin the planner invariants directly instead of inferring them
from a flight log.

The ROS nodes hold everything that genuinely needs a running autopilot:
QoS-matched publishers and subscribers, the arming handshake, the state
machine, the safety supervision, and the timing.

## Layers

### `mapping` — SDF to occupancy grid

Parses the Gazebo world in document order and rasterises each obstacle as a
filled disc with a per-species footprint radius. No RNG anywhere, so identical
SDF input yields a byte-identical PGM.

Two footprint sets are used. The **trunk** radii (pine 0.25 m, oak 0.30 m) give
the ground-level prior map. The **canopy** radii at the 2 m reference altitude
(pine 1.5 m, oak 4.0 m) give the *execution prior* — the map the mission is
actually planned on, because at flight altitude it is the canopy the vehicle
must clear. Both share one grid geometry, derived from the trunk bounding box,
so the two maps are comparable cell by cell.

An unknown `model://` include raises rather than being skipped: silently
dropping an obstacle would produce a map that is optimistic exactly where it
matters.

### `grid` — representation and clearance

Two encodings coexist deliberately. The **semantic** grid
(`FREE`/`OCCUPIED`/`UNKNOWN`) drives inflation. The **raw PGM** drives the
distance transform used for clearance, so clearance is always measured to real
obstacles rather than to the inflated proxy the search ran on.

Cell indexing: columns increase with +x, rows increase as y decreases (PGM
stores the top row first, while `origin` refers to the bottom-left corner).
Inflation pads the array with *blocked*, so the map border behaves as an
obstacle and a path can never leave the grid.

### `planning` — deterministic A*

One search core, three edge costs:

| Variant | Hard inflation | Soft cost | Role |
|---|---|---|---|
| `a_star` | `r_eff` | none | reference behaviour |
| `a_star_clearance` | `r_eff` | penalty in `[r_eff, r_pref)` | boundary candidate; local replan |
| `a_star_comfort` | `r_plan_min` | penalty in `[r_plan_min, r_comfort)` | robust candidate; nominal mission |

Shared invariants: 8-connected with a fixed neighbour order, no corner cutting
(a diagonal step requires both adjacent orthogonals clear), octile heuristic
measured to the *boundary* of the goal disc, and a total tie-break on
`(f, g, row, col)` so repeated runs are byte-identical.

Both soft costs are non-negative, so every penalised edge costs at least its
geometric length and the octile heuristic remains admissible.

**The smoothing budget.** The robust candidate searches at
`r_plan_min = r_pref + spline_shrinkage_budget = 1.10 m`, above the 0.90 m the
finished trajectory is expected to satisfy. The surplus is a budget the
B-spline may consume when it cuts corners; without it, a plan that exactly met
`r_pref` would fall below it after refinement.

The budget makes a good outcome likely; it does not guarantee one. Every
refined candidate is validated afterwards and classified by its **resulting**
clearance — see the refinement cascade below.

### `smoothing` — line-of-sight reduction

Greedy string pulling on the same inflated grid the planner used, so any
waypoint it accepts is by construction acceptable to the planner. Output is
still a polyline. Four invariants are checked and a violation is an error, not
a warning: endpoint preservation, full-segment line of sight, length
monotonicity, and the clearance floor.

Clearance is evaluated over the *rasterised interior* of each segment, not at
the waypoints: after reduction a segment spans many cells and an interior cell
can sit closer to an obstacle than either endpoint.

### `trajectory` — B-spline cascade and speed profile

Three candidates in fixed order — interpolating cubic B-spline at 2.0 m
control spacing, the same at 1.0 m, then an unsmoothed 0.5 m polyline
fallback. Each is validated on dense 0.02 m samples against the raw clearance
field; the first to clear `r_eff` is selected. Validation is on samples rather
than control points because a spline bulges *between* control points, and that
bulge is what has to clear the obstacle.

Classification is load-bearing: `robust_smoothed` (≥ `r_pref`),
`boundary_smoothed` (≥ `r_eff`), `conservative_fallback`, `fail`. The fallback
keeps its marker even when its clearance happens to exceed `r_pref`, so a
degraded run can never be reported as a nominal one.

The speed profile is curvature-bounded — `v = sqrt(a_lat_max / kappa)` clamped
to `[v_min, v_nominal]` — then limited by forward and backward longitudinal
passes.

### `frames` — ENU to NED

Planning works in the Gazebo world frame (ENU). PX4 Offboard setpoints are in
the vehicle's local NED frame, origin at the EKF origin:

```
x_ned =  y_world - Y0        y_ned =  x_world - X0
z_ned = -z_ref               yaw_ned = wrap_pi(pi/2 - yaw_world)
```

The yaw relation is an involution, so the same expression serves both
directions. The velocity mapping follows from differentiating the position
mapping along the path tangent.

### `replanning` — local ROI detour

On a confirmed detection: derive a path-aligned ROI from the motion envelope,
crop the prior map to it, stamp in the sensed obstacle, build the clearance
field and inflated grid on the crop, run clearance-aware A* from the hover
pose to a downstream resume anchor, refine, and slice back to the
hover→resume span.

ROI extents are derived, not tuned: minimum turn radius `v²/a_lat`, stopping
distance `v²/(2·a_stop)`, the replanning time budget, and the obstacle radius
plus preferred clearance.

**Lead-in / tail-in.** A few samples of the *original* reference are prepended
before the hover pose and appended after the resume anchor before smoothing.
This anchors the spline tangent at both ends to the original mission heading,
which collapses the yaw discontinuity at the join. Those samples belong to the
already-validated reference, so this neither weakens the clearance argument
nor introduces synthetic geometry.

Acceptance during a live run is stricter than offline: the polyline fallback is not
accepted, because a controlled abort beats continuing on a degraded reference.

### `artifacts` — provenance

Every stage writes content files, `metrics.yaml`, a `manifest.yaml` recording
inputs and parameters, and a `sha256sum`-compatible `hashes.txt`. Timestamped
files are deliberately excluded from the hash list, so
`sha256sum -c hashes.txt` stays meaningful on a correct regeneration.

## ROS runtime

`offboard_mission_executor` streams a pre-computed NED reference at 20 Hz and
runs the mission state machine, with continuous safety supervision. See
[safety.md](safety.md). The parent node has thirteen states:

```
WAIT_STATUS → WAIT_LPOS → WAIT_EKF_READY → PRE_STREAM → DECIDE_ARM →
WAIT_OFFBOARD → WAIT_ARMED → SETTLE_HOLD → MISSION → OBSERVE_END →
WAIT_DISARM → DONE
```

with `ABORT_LANDING` entered from the abort path in place of `WAIT_DISARM`. A
dry run reaches only the first five of these and then `DONE`; see
[safety.md](safety.md#what-dry-run-actually-does).

`stop_replan_resume_node` subclasses it and inserts the obstacle-reaction
states (`DETECTED → STOP_HOVER → REPLAN_LOCAL_ROI → RESUME_REPLAN_REFERENCE →
REJOIN_ORIGINAL_MISSION → OBSERVE_END`). Detection runs on `/scan_filtered`
within a corridor ahead of the vehicle, requires a hysteresis streak, and
tests novelty against the prior map. The detection callback returns
immediately unless the state is `MISSION`
(`stop_replan_resume_node.py:521-522`), and `MISSION` is entered only from
`SETTLE_HOLD`, which follows a confirmed arm.

Two variants exist so that mechanism and perception are falsifiable
separately: **Gate 6D-A** (`--oracle-mode replan`) feeds the state machine
from ground truth on a patched map and validates the mechanics only; **Gate
6D-B** (`--oracle-mode off`) is perception-driven, with ground truth retained
in a separate evaluation-only namespace: it does not gate detection, obstacle
construction, replanning or control decisions, but it may be used to compute
diagnostic metrics and post-run evaluation values. The published run is
Gate 6D-B.

### Two monolithic nodes, deliberately

`offboard_mission_executor.py` is 2943 lines and `stop_replan_resume_node.py`
is 2061 lines. These are large modules by any normal standard, and no attempt
is made here to present that as a design virtue.

Release 0.1 keeps them as they are because they are the runtime state machines
that produced the reported evidence, and splitting a validated state machine
during a public-release extraction changes timing, callback order and
initialisation order, which is exactly what would invalidate the runtime
results the release is meant to accompany. Decomposition into separate
supervision, handshake, streaming and reaction units is future engineering
work, not something already done and hidden behind an interface. The layers
above the ROS 2 boundary carry no such constraint, and they are split.

### The physical obstacle at run time

The Stop-Replan-Resume scenario only means anything if the obstacle physically
exists in the simulation, because the 2D LiDAR has to be able to see it. It is
not in the world file. It is spawned into the running world before the run:

```
sim/obstacles/obstacle.yaml   (pose, radius, height: single source of truth)
sim/obstacles/srr_unknown_obstacle.sdf
              │
   tools/spawn_srr_obstacle.py
              │   gz service /world/<world>/create
              ▼
        running Gazebo world  ──►  2D LiDAR  ──►  /scan  ──►  /scan_filtered
                                                                    │
                                                    stop_replan_resume_node
```

The tool lists the world's models first, spawns only if the model is absent,
re-lists to confirm it appeared, and reads the pose back. A second invocation
reports `already_present` and spawns nothing; `--fail-if-exists` turns that
into exit 4. It writes `spawn_record.yaml` with `spawned_this_invocation` and
the before and after model counts, so "exactly once" is checkable rather than
assumed. It talks to Gazebo and nothing else: no PX4 command, no ROS topic,
and no path by which it could arm anything.

### Five representations of one obstacle

These are five different objects and mixing any two of them invalidates the
perception argument.

| Representation | Where it lives | Who reads it |
|---|---|---|
| **Physical Gazebo model** | spawned at run time from `sim/obstacles/srr_unknown_obstacle.sdf` | the simulated LiDAR, and only it |
| **Prior occupancy map** | rasterised from `sim/worlds/forest_map2.sdf`; does **not** contain the obstacle | the replanner searches this |
| **Sensed obstacle** | built online from LiDAR clusters plus a novelty test against the prior | stamped into the local ROI crop |
| **Ground truth** | `sim/obstacles/obstacle.yaml` | in perception mode, held in a separate evaluation-only namespace: it gates no decision, but it is read for a per-tick diagnostic distance and for the post-run metrics |
| **Patched map** | prior with ground truth stamped in | `--oracle-mode replan` only |

Gate 6D-B needs the physical model present **and** ground truth kept out of
every decision. The prior map never contains the obstacle in either variant.

## Simulated platform

The planning and collision geometry is sized toward the Holybro X650 target
platform. Mass, inertia, propulsion and runtime flight dynamics remain those of
the PX4 X500 SITL model. The reported execution results are therefore
**X650-geometry-informed SITL results**, not an autonomous real-flight
validation of the X650.

The planner is circular and consumes exactly one geometric number:
`robot_radius` = `r_eff` = 0.65 m in `sim/config/robot_map2_safe.yaml`. It is
derived from the vendor's published figures, a 650 mm wheelbase measured
diagonally from motor to motor and 15 inch propellers:

| Quantity | Value |
|---|---|
| Frame radius, motor hub to centre, 0.650 / 2 | 0.3250 m |
| Propeller radius, 15 × 0.0254 / 2 | 0.1905 m |
| Propeller-tip envelope radius | 0.5155 m |
| Safety margin | 0.1345 m |
| `r_eff` | 0.6500 m |

So `r_eff` encloses the smallest circle that actually contains the turning
aircraft, with 0.1345 m to spare.

Three readings of the geometry that appear elsewhere are wrong and are
corrected here:

* The square footprint `[±0.46, ±0.46] m` is an **assumed** axis-aligned
  planning footprint. There is no measurement on record for the 0.46 m
  half-extent, and it is not an arm-tip half-extent: in an X configuration the
  arm tip sits at 0.3250 / √2 = 0.2298 m and the propeller-swept half-extent is
  0.2298 + 0.1905 = 0.4203 m. Because 0.46 m exceeds 0.4203 m, the square is
  conservative as a footprint. No code in this repository reads the polygon; it
  is documentation only.
* An earlier nominal radius of **0.50 m** was rejected for public planning
  because it does not enclose the 0.5155 m propeller-tip envelope. All
  published planning results use `r_eff` = 0.65 m from
  `sim/config/robot_map2_safe.yaml`. The 0.50 m value is recorded here as
  history; it is not shipped as a configuration and must not be described as
  an available one.
* The invariant `robot_radius ≥ inscribed_circle_radius(footprint)` has been
  removed from the configurations. The inscribed circle is the
  non-conservative bound for a square. The conservative bound is the
  circumscribed circle, √2 × 0.46 = 0.650538 m, which `robot_radius` = 0.50 m
  does not satisfy. That 0.650538 m agrees with `r_eff` to within 0.54 mm, but
  this is not independent corroboration: 0.46 is itself 0.65 / √2 to two
  decimals.

`r_pref` = 0.90 m, `r_plan_min` = 1.10 m and `r_comfort` = 1.20 m are clearance
policy rather than airframe dimensions: a classification threshold, what the
comfort planner inflates to, and the soft preference in the comfort cost.
`r_plan_min` − `r_pref` = 0.20 m is the spline budget.

Not adapted to the X650, and therefore X500 model values: mass, inertia,
propulsion, motor layout, closed-loop flight dynamics, and the LiDAR mount
offset in `lidar_mount.xyz`.

`sim/config/robot_map2_safe.yaml` carries this derivation in full and is the
authoritative wording. It is the planning configuration every published
planning result was computed from, and the only robot geometry this release
ships.
