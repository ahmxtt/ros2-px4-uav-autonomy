# Modular UAV Autonomy with ROS 2 and PX4

[![CI](https://github.com/ahmxtt/ros2-px4-uav-autonomy/actions/workflows/ci.yml/badge.svg)](https://github.com/ahmxtt/ros2-px4-uav-autonomy/actions/workflows/ci.yml)

**Planning, PX4 Offboard execution and Stop–Replan–Resume validation**

A modular sense–plan–act autonomy stack for a multirotor UAV in cluttered
forest terrain, built on ROS 2 Jazzy, PX4 and Gazebo Harmonic.

The system plans a collision-free reference through a prior occupancy map,
refines it into a curvature-continuous trajectory, executes it through PX4
Offboard at 20 Hz, and reacts to an obstacle absent from the map by stopping,
replanning locally, and rejoining the original mission.

> Master's thesis work on UAV autonomy for search-and-rescue, disaster response
> and wildlife monitoring. This repository is the curated implementation and
> evidence release, not the full development history.

![Architecture](docs/figures/architecture.png)

---

## Two classes of result — read this first

| | |
|---|---|
| **Offline, deterministic** | Map generation, global planning, path reduction, B-spline refinement, feasibility evaluation. Computed from the Gazebo world file. The occupancy grid and the A\* cell path reproduce **exactly** in the verified environment. Refined trajectory geometry and derived floating-point metrics reproduce within the documented numerical tolerances. |
| **Controlled PX4 SITL / Gazebo runs** | Nominal mission tracking, ground-truth versus PX4 state consistency, online Stop–Replan–Resume behaviour. Measurements of specific flights. Live SITL is not deterministic and is not claimed to be. Not regenerable. |

Nothing here is a real-flight result.

## What this repository contributes

An **integration and validation** contribution with explicit, checkable
contracts — not a new estimation or SLAM method.

- **Deterministic 2D grid planning.** 8-connected, octile heuristic, no corner
  cutting, hard binary inflation, total tie-break on `(f, g, row, col)`.
  `tests/test_regression_published_result.py` asserts the A\* result cell for
  cell against `results/planning/path.csv`, and rebuilds the rasterised grid
  byte-identically inside the same test. The quickstart tool compares five
  scalars, not cells.
- **A smoothing budget, followed by validation.** The planner searches above
  the clearance the finished trajectory should hold. That is a budget, not a
  guarantee: every refined candidate is validated on dense samples and
  classified by its clearance. The nominal trajectory reached
  `robust_smoothed`; the local detour reached only `boundary_smoothed`.
- **Local ROI replanning** over a motion-envelope-sized neighbourhood,
  computed online in 0.25 s during the controlled SITL mission, rejoining the
  untouched original reference.
- **Reproducibility as infrastructure.** Every stage emits a manifest, metrics
  and SHA-256 hashes; both offline chains rerun in CI.

## Incremental validation-gate framework

Each gate defines a capability, the evidence that would demonstrate it, and its
failure conditions; a gate had to close before the next opened. A gate is a
**scope boundary**: a claim carries the scope of the gate that produced it.

| Gate | Scope |
|---|---|
| **Gate 4** | Coordinate transforms, communication and basic PX4 Offboard execution |
| **Gate 5** | Nominal mission preparation and execution |
| **Gate 6** | Obstacle response and Stop–Replan–Resume validation |

Gates 0–3 and the Gate 6D-A / 6D-B split:
[docs/validation_gates.md](docs/validation_gates.md).

*Stop–Replan–Resume* (SRR) is this project's name for the obstacle-response
sequence: stop, plan a local detour, resume the original mission.

## X650 geometry, X500 dynamics

The planning geometry is sized toward the Holybro X650 target platform. Mass,
inertia, propulsion, motor layout, closed-loop dynamics and the LiDAR mount
offset remain those of the PX4 X500 SITL model. The reported execution results
are therefore **X650-geometry-informed SITL results**, not an autonomous
real-flight validation of the X650.

`r_eff` = 0.65 m is the only radius any code reads. It is a **planning
envelope, not an airframe dimension**, built from the vendor's published
650 mm wheelbase and 15 inch propellers:

| Quantity | Value | Where it comes from |
|---|---|---|
| Frame radius, motor hub to centre | 0.3250 m | vendor figure, 0.650 / 2 |
| Propeller radius | 0.1905 m | vendor figure, 15 × 0.0254 / 2 |
| Propeller-tip envelope radius | 0.5155 m | sum of the two above |
| Safety margin | 0.1345 m | chosen, not derived |
| **`r_eff` = `robot_radius` = 0.6500 m** | the planning envelope | `sim/config/robot_map2_safe.yaml`; read by every planning tool and by the regression test |
| Footprint `±0.46 m` | assumed | **No measurement backs it.** Not an arm-tip half-extent. Documentation only: no code reads the footprint polygon. |
| `r_pref` 0.90 m, `r_plan_min` 1.10 m, `r_comfort` 1.20 m | clearance policy | thresholds, not geometry |

An earlier nominal radius of 0.50 m was rejected for public planning because
it does not enclose the 0.5155 m propeller-tip envelope. All published planning
results use `r_eff` = 0.65 m from `sim/config/robot_map2_safe.yaml`; the 0.50 m
value is not shipped as a configuration. The conservative bound for a ±0.46 m
square is its circumscribed circle, 0.650538 m, not its inscribed circle.

Tracking, stopping, hover transitions and the rejoin transient are
**dynamics-dependent** and carry the X500 model's dynamics. Full provenance:
[docs/results.md](docs/results.md#x650-geometry-x500-dynamics).

## Quickstart (offline — no ROS, PX4 or Gazebo)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
python3 tools/reproduce_offline_demo.py --output-dir /tmp/uav_demo
python3 tools/reproduce_srr_geometry.py --output-dir /tmp/uav_srr
```

Under a minute each. Run them from the repository root; internally the tools
resolve repository inputs independently of the caller's current working
directory, and CI additionally invokes them by absolute path from a foreign
working directory. The first reproduces the
nominal planning and trajectory chain, the second the Stop–Replan–Resume detour
geometry. Both rebuild the map from the world file, rerun the canonical
computation and **compare their output against the published numbers**, exiting
non-zero on a regression.

The SITL path is in [docs/installation.md](docs/installation.md).

## Selected results

The groups measure **different things** and must not be combined. Each is
labelled with its result class: **1** offline nominal planning, **2** offline
SRR geometry, **3** canonical controlled SITL evidence (single runs, not
regenerable), **4** public-release runtime validation.

**1 · Nominal planning** *(class 1, offline)* — single leg on the execution
prior map

| | |
|---|---|
| A\* path length | 102.139 m (straight line 97.541 m, ratio 1.047) |
| Nodes expanded | 230 047 |
| Minimum clearance to raw obstacles | 1.200 m |

**2 · Trajectory refinement** *(class 1, offline)*

| | |
|---|---|
| Length | 100.557 m (−1.549 % vs A\*) |
| Minimum clearance | 1.082 m → `robust_smoothed` (≥ `r_pref` 0.90 m) |
| Max deviation from A\* | 0.252 m |

![Nominal planning](docs/figures/nominal_planning.png)

**3 · Nominal mission tracking** *(class 3)* — setpoint vs PX4 estimate,
MISSION phase, n = 2922

| | |
|---|---|
| Horizontal error, mean / RMSE | 0.220 m / 0.234 m |
| Horizontal error, p95 / max | 0.378 m / 0.609 m |
| Budget | 1.0 m, no window breach |

**4 · Ground truth vs PX4 estimate** *(class 3)* — airborne, n = 14 869

| | |
|---|---|
| Horizontal RMSE | 0.226 m |
| Mean offset (dx, dy, dz) | 0.060 / 0.123 / 0.019 m |

**5 · Stop–Replan–Resume** *(class 3 for the run, class 2 for the geometry)*

| | |
|---|---|
| Local replan runtime | 0.251 s online during the controlled SITL mission (budget 3 s) |
| Replanned path length | 23.178 m |
| Minimum clearance | 0.808 m → `boundary_smoothed` (≥ `r_eff` 0.65 m, **below** `r_pref` 0.90 m) |
| Position jump at handover / yaw jump | 0.245 m / 11.07° |
| Rejoin distance to original reference | 0.006 m |
| Mission | completed, no failsafe, tracking max 0.802 m |

![Stop-Replan-Resume geometry](docs/figures/srr_geometry.png)

Groups 3 to 5 come from the canonical runs in `results/nominal_mission/` and
`results/stop_replan_resume/`, produced by the private development tree.

**Class 4**, [`results/public_runtime_validation/`](results/public_runtime_validation/),
records what happened when *this* repository drove a live PX4 SITL stack. The
physical obstacle spawn is **verified**: models 88 → 89, exactly one
occurrence, measured pose equal to the requested (−21.330, −43.710, 1.500) m.
The armed Stop–Replan–Resume run is **not completed**: it armed, entered
Offboard, held the settle setpoint, then aborted on stale local-position
telemetry and landed cleanly. It never reached MISSION, so no detection, stop,
replan, resume or rejoin was exercised. There is no public armed nominal run
either.

Full tables, definitions and provenance: [docs/results.md](docs/results.md).

## Repository structure

| Path | Contents |
|---|---|
| `src/uav_autonomy/` | Algorithmic core: mapping, grid, planning, smoothing, trajectory, replanning, frames, artifacts. No ROS. |
| `tools/` | Thin CLIs over the core: the two reproduction tools, map/plan/refine, `build_ned_setpoints.py`, the figure generator. Plus the SITL helpers `sitl_stack.sh` (`start`, `status`, `params`, `prearm`, `rtf`, `stop`), `spawn_srr_obstacle.py`, `gcs_heartbeat.py` and `build_srr_inputs.py`. |
| `ros2/nodes/` | PX4 Offboard mission executor and the Stop–Replan–Resume node. Requires ROS 2. |
| `ros2/rviz/`, `ros2/bridge/` | RViz config and the Gazebo↔ROS bridge template. |
| `sim/` | Gazebo world, scenario definition, robot and mission-policy configuration. |
| `sim/obstacles/` | The unknown obstacle: `srr_unknown_obstacle.sdf` (static cylinder, r = 1.5 m, h = 3.0 m, world (−21.33, −43.71)) and `obstacle.yaml`, its ground truth. Absent from the prior map by design. |
| `results/mission_reference/` | The complete multi-POI mission reference: 16 157 samples, 5 segments, 428.651 m, 286.159 s. The input an armed run consumes. |
| `results/` | Curated result bundles with metrics, manifests and hashes, including `public_runtime_validation/`. |
| `tests/` | Invariant and regression tests (`pytest`). |
| `docs/` | Architecture, validation gates, installation, reproducibility, results, data availability, limitations, safety. |

## Limitations

- Validated in **offline computation and PX4 SITL / Gazebo only**. No
  autonomous real-flight validation.
- **No custom SLAM or sensor-fusion method.** State estimation is PX4's EKF2.
- Global planning is **2D at a constant reference altitude** in the evaluated
  mission.
- Evidence is **single controlled runs**, not repeated trials under a fixed
  protocol. **No statistical robustness claim**, and no success rate.
- SRR was validated for **one** obstacle. Multi-obstacle handling and
  false-positive policy are out of scope.
- **Not safety-certified.**

Full list: [docs/limitations.md](docs/limitations.md) ·
[docs/safety.md](docs/safety.md).

## Data availability

This is a curated research-software release with fully reproducible offline
planning and SRR-geometry results, together with selected result summaries and
evidence artifacts from controlled PX4 SITL runs.

It also publishes the complete mission reference,
`results/mission_reference/mission_reference.csv`, which is what makes an armed
run possible: it begins 0.035 m from the vehicle spawn point, so the settle
guard will accept it, and `tools/build_ned_setpoints.py` turns it into the
canonical 5725-row, 20 Hz PX4 setpoint stream. It is a byte-identical copy of
the canonical artifact and a derived mission product, not raw data; the
multi-leg policy planner, orbit generator and mission stitcher that produced it
are not part of this release, so it is a fixed input rather than a regenerable
one.

The offline computations are independently reproducible: the occupancy grid
and the A\* cell path reproduce exactly, and the Stop–Replan–Resume detour
geometry with them, within the documented tolerances. The canonical live-run
summaries are not independently recomputable from this release, because the
complete raw per-tick time series behind them are not included; they stay in
the private research archive, available on reasonable request, and what is
published is curated evidence from those thesis runs. Published hashes prove
the integrity of the published files, not reconstruction from absent raw logs.
Separately, the public armed runtime did not complete an end-to-end mission,
which is recorded in
[results/public_runtime_validation/](results/public_runtime_validation/README.md).
Full breakdown: [docs/data_availability.md](docs/data_availability.md).

## Citation

See [CITATION.cff](CITATION.cff), or GitHub's "Cite this repository" button.

## Third-party dependencies

ROS 2 Jazzy, PX4, Gazebo Harmonic, NumPy, SciPy, Matplotlib, Pillow, PyYAML.
The Gazebo world references the Fuel models *Pine Tree* and *Oak tree*
(OpenRobotics, **CC0 1.0**); their meshes are not included and must be fetched
separately. Full attribution: [NOTICE.md](NOTICE.md).

## Licence

Code under [LICENSE](LICENSE) (Apache-2.0); data, figures and result bundles
under [LICENSE-DATA](LICENSE-DATA) (CC BY 4.0).
