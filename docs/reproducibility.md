# Reproducibility

Reproducibility here is **asymmetric**, and the split is the point.

| Layer | Class | Status |
|---|---|---|
| Map generation → A\* → B-spline (LoS reduction as a side branch on the A\* path) | offline | **Deterministic by construction, verified in this release** |
| Stop–Replan–Resume *geometry* | offline | **Deterministic; A\* exactly, refined detour to 1e-6 m** |
| World-frame reference → PX4 NED setpoint stream | offline | **Byte-identical to the canonical run input** |
| PX4 SITL execution (tracking, timing, mission outcome) | SITL | **Not deterministic, not claimed to be** |

Conflating the offline layers with the SITL layer is the easiest way to
overstate what this repository shows.

---

## What is verified

The offline chain contains no RNG. Grid rasterisation, grid A\*, Bresenham
string pulling and cubic B-spline interpolation through arc-length-resampled
control points are all deterministic algorithms, and the A\* tie-break is total, so the expansion
order — and therefore the returned path — is unique.

That is the argument. This release also checks it:

| Property | Check | Result |
|---|---|---|
| Grid rasterisation is byte-identical on a repeat build | `tests/test_regression_published_result.py:135-143` | identical arrays over 2913 × 3911 |
| Published A\* path regenerates cell for cell | `tests/test_regression_published_result.py:152-158` | 1543 / 1543 cells identical |
| A\* is stable across repeated runs | `tests/test_planning.py:126-138` | 15 repeats, identical |
| Trajectory metrics match the published values | `tests/test_regression_published_result.py` | equal to within 1e-9 |
| Canonical PX4 setpoint stream regenerates byte-identically | `tools/build_ned_setpoints.py` | SHA-256 equals the recorded run input |
| SRR local detour reproduces from public inputs | `tools/reproduce_srr_geometry.py`, `tests/test_replanning.py` | A\* nodes and length exact; refined path to 1e-6 m |
| SRR rejoin continuity reproduces | same | 0.245 m and 11.07°, matching the live run |
| Result bundles are self-consistent | `sha256sum -c` on each `hashes.txt` | pass |

Reproduce all of it:

```bash
pip install -e ".[test]"
pytest
python3 tools/reproduce_offline_demo.py --output-dir /tmp/uav_demo
```

> If ROS 2 is sourced in your shell, run the tests as
> `env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest` — otherwise
> pytest picks up ROS's `launch_testing` plugin from `PYTHONPATH`. The offline
> tests themselves need nothing from ROS.

### What the byte-identity check does and does not cover

The map check rasterises the world a second time and compares the result
against the `map.pgm` that the same test fixture wrote a moment earlier
(`tests/test_regression_published_result.py:135-143`). **No occupancy grid and
no grid hash ship with this release**, so byte-identity against a *published*
artifact is not demonstrated and is not claimed. What is demonstrated is that
the rasteriser is deterministic, and that the grid on which the published A\*
result is replanned is the one this repository builds from
`sim/worlds/forest_map2.sdf`.

### Which check lives where

The two entry points are not interchangeable, and the difference matters.

**The cell-for-cell A\* comparison is in the test suite**, at
`tests/test_regression_published_result.py:152-158`. It reads every
`(row, col)` pair out of `results/planning/path.csv` and asserts list equality
against the replanned cells. Exact, no tolerance.

**`tools/reproduce_offline_demo.py` does not do this.** It compares **five
scalars** against `results/planning/metrics.yaml` and
`results/planning/trajectory_metrics.yaml`:

| Scalar | Tolerance |
|---|---|
| A\* path length | 1e-6 m |
| A\* nodes expanded | 0, exact |
| A\* minimum clearance | 1e-9 |
| B-spline trajectory length | 1e-6 m |
| B-spline minimum clearance | 1e-9 |

It prints each delta and the worst of them, and exits non-zero if any value
drifts. That is a useful quickstart check, but it is not the path comparison.
Cite the test when you need the exact claim.

### Library versions

The published results were produced in the environment recorded in
[installation.md](installation.md#environment-of-record). The 1e-9 tolerance on
the trajectory metrics exists because floating-point summation order in BLAS
and SciPy moves the last digits of the arc-length and KD-tree reductions. No
cross-version reproduction run is published with this release, so no
cross-version agreement figure is quoted here.

### Why the map is regenerated rather than shipped

The occupancy grid is not committed. It is rebuilt from
`sim/worlds/forest_map2.sdf` on every run, and `tools/build_prior_map.py`
writes a `hashes.txt` over its own output so the rebuild can be checked for
self-consistency. This turns the determinism claim into something a reader
executes rather than something they take on trust, and it keeps the repository
small. It also means there is no published grid to compare against; see the
preceding section.

### Tolerances, and why they differ

- **Path cells: exact.** Any deviation means the search changed behaviour.
- **Path geometry: 1e-6 m.** Far below the 0.05 m cell size.
- **Trajectory metrics: 1e-9.** This absorbs floating-point summation-order
  differences that BLAS and SciPy versions introduce in the arc-length and
  KD-tree reductions, while still catching any real behavioural change. If you
  need agreement tighter than that, pin the versions in
  [installation.md](installation.md#environment-of-record).

### The canonical setpoint stream regenerates byte-identically

This is the one exact reproduction that reaches into the flown runs, and it is
worth stating precisely. `tools/build_ned_setpoints.py` rebuilds the PX4
local-NED setpoint stream that the canonical armed run consumed, from the
published mission reference:

```bash
python3 tools/build_ned_setpoints.py \
    --trajectory-csv results/mission_reference/mission_reference.csv \
    --output-csv /tmp/setpoints_ned.csv
sha256sum /tmp/setpoints_ned.csv
```

The output is 5725 rows at 20 Hz spanning 286.16 s, with all five mission
segments in order and their labels preserved, and its SHA-256 is

```
bcc70fb896454d5e0a98fd46812bbd4513670891bac2708298871ec611e5d81e
```

which is the hash recorded as the setpoint-stream input of the canonical armed
run. This is a byte comparison, not a tolerance. The run manifest that records
that hash on the other side stays in the private archive, so what a reader can
check unaided is agreement with the value printed above; what the check
establishes is that the world-frame-to-NED conversion is deterministic and that
the published mission reference is the file the canonical run actually flew.

The tool also writes `<out>.manifest.yaml`, `<out>.validation.yaml` and
`<out>.hashes.txt` beside the CSV. It refuses to emit a stream whose first
setpoint is more than `--max-start-offset-m` (default 1.0 m) from the NED
origin, because the executor's settle guard cannot fly there from a standing
start; `--allow-offset-start` downgrades that to a warning.

The stream is reproducible; the *flight* is not. See below.

---

## What is not deterministic

**Live PX4 SITL execution is not deterministic, and no timing figure from a
flown run is expected to reproduce.** A flown run depends on wall-clock pacing,
simulator physics stepping, the EKF, the achieved real-time factor, and the
delivery behaviour of the uXRCE-DDS transport, none of which is fixed. Every
latency, duration and tracking figure taken from a live run is a property of
that run on that machine. Re-flying the nominal mission will not reproduce the
tracking numbers exactly, and this repository does not claim it will.

What *is* reproducible about a flown run is its provenance: each result bundle
records the inputs, their hashes, the parameters and the verdict conditions,
so the run can be audited even though it cannot be replayed bit-for-bit.

**The complete mission reference is published as data, not regenerated.**
`results/mission_reference/` is a curated public copy of the canonical
multi-POI reference, byte-identical to the archived artifact. The public
offline chain reproduces a single-leg planning result exactly; it does not
rebuild the multi-POI mission, because the multi-leg policy planner, the orbit
generator and the mission stitcher were not part of this extraction. Treat
`mission_reference.csv` as a fixed input and verify it with its `hashes.txt`.
`tests/test_mission_reference.py` pins its structure, its start point, and the
fact that `results/stop_replan_resume/mission_segment_window.csv` is an exact
row-for-row slice of it. See
[data_availability.md](data_availability.md).

**Consequence for the reported numbers.** Sections 1–2 of
[results.md](results.md) are regenerable, and so is the *geometry* of section
5. The tracking and estimator-consistency figures in sections 3–4, and the
online timing of section 5, are measurements of specific runs and are
reproducible only in distribution — and this release contains a single run
each, so not even that is characterised. See [limitations.md](limitations.md).

### Stop–Replan–Resume: what reproduces and what does not

```bash
python3 tools/reproduce_srr_geometry.py --output-dir /tmp/srr
```

**Reproduces** the ROI extents, the cropped and patched map, the
clearance-aware A* detour, the B-spline refinement, the resulting clearance,
curvature and lateral acceleration, and the rejoin continuity. The tool reads
the *sensed* obstacle from the published bundle — the perception output the
vehicle acted on, not the ground truth — and exits non-zero on a regression.

**Does not reproduce** detection latency, the stop-to-hover transition, the
online replanning runtime, tracking error or mission outcome. The tool prints
its own offline recomputation time separately and labels it as a different
quantity from the 0.251 s measured online during the SITL mission.

---

## Regenerating individual stages

```bash
# Occupancy grid from the world file
python3 tools/build_prior_map.py \
    --world-sdf sim/worlds/forest_map2.sdf \
    --output-dir /tmp/prior
cd /tmp/prior && sha256sum -c hashes.txt && cd -

# A* on that grid
python3 tools/plan_astar.py \
    --map-yaml /tmp/prior/map.yaml \
    --candidate robust \
    --output-dir /tmp/plan

# Line-of-sight reduction
python3 tools/reduce_path_los.py \
    --path-csv /tmp/plan/path.csv \
    --map-yaml /tmp/prior/map.yaml \
    --inflation-radius-m 1.10 \
    --output-dir /tmp/reduced

# B-spline refinement and speed profile
python3 tools/refine_trajectory.py \
    --path-csv /tmp/plan/path.csv \
    --map-yaml /tmp/prior/map.yaml \
    --output-dir /tmp/traj
```

`tools/build_prior_map.py` uses trunk footprints and reproduces the
ground-level prior map. The execution prior that the mission is planned on
uses canopy footprints at the 2 m reference altitude;
`tools/reproduce_offline_demo.py` builds that one.

> **The `map.yaml` round trip is load-bearing.** The in-memory grid origin is
> `-80.30000000000001`, a float artefact of snapping the origin to a cell
> multiple, while `write_map_yaml` formats it to three decimals as `-80.300`.
> That 1e-14 difference is enough to shift the start cell by one and change
> the A* path length by 0.05 m. Every entry point in this repository reads a
> written `map.yaml` rather than an in-memory geometry, and
> `tools/make_docs_figures.py` verifies its own output against the published
> numbers for exactly this reason.

## ROS 2 runtime verification

The nodes under `ros2/nodes/` are not covered by CI, which installs no ROS.
They were verified locally against a live PX4 SITL stack:

| Check | Result |
|---|---|
| Import and argument parsing against real `rclpy` + `px4_msgs` | pass, both nodes |
| Nominal executor, dry run against live SITL | `VERDICT=DRY_RUN_OK`; reaches `DECIDE_ARM` and stops; two-flag arm gate refused |
| SRR node, dry run against live SITL, perception mode | output bundle written; the online-decision obstacle is empty before the first control tick, with ground truth confined to the evaluation-only namespace |
| Output-schema compatibility with the published run | the release code emits the same 20 verdict conditions, under the same keys, as `results/stop_replan_resume/verdict.yaml` |

**What a dry run covers, exactly.** A dry run is any run without *both*
`--arm` and `--i-have-physical-eyes-on-the-sim`. It reaches at most 6 of the
executor's 13 states (fewer if it aborts early) (`WAIT_STATUS`, `WAIT_LPOS`, `WAIT_EKF_READY`, `PRE_STREAM`,
`DECIDE_ARM`, `DONE`) and stops at
`ros2/nodes/offboard_mission_executor.py:1373-1384`. It does publish
`OffboardControlMode` and a hold `TrajectorySetpoint` for about
`--pre-stream-s` seconds after EKF-ready, and it never publishes any
`VehicleCommand`: no mode change, no arm, no land, no force-disarm.

The dry run therefore verifies ROS/PX4 connectivity, LiDAR message reception,
argument handling, pre-arm Offboard heartbeat and hold-setpoint streaming, the
two-factor arm gate and output initialisation. **It does not execute obstacle
detection, stop, local replanning, resume or rejoin**, because these states are
reached only during an armed mission: the SRR handler returns immediately
unless the node is in `MISSION`
(`ros2/nodes/stop_replan_resume_node.py:528-529`), and `MISSION` is entered
only from `SETTLE_HOLD`, which follows a confirmed arm.

**Armed flight.** The armed nominal mission was not re-flown for this release;
the published flight results come from the recorded canonical runs. An armed
perception-driven SRR run *was* attempted with this repository and did not
complete: it armed, entered OFFBOARD and held the settle setpoint, then aborted
on stale local-position telemetry before reaching `MISSION`. The evidence and
the measured cause are in `results/public_runtime_validation/`. No armed run of
this public code has completed a mission. See
[data_availability.md](data_availability.md).

---

## Artifact convention

Each bundle contains:

| File | Role |
|---|---|
| content (`*.csv`, `*.pgm`) | deterministic output |
| `metrics.yaml` | quantitative results |
| `manifest.yaml` | inputs, input hashes, parameters, tool versions |
| `hashes.txt` | `sha256sum`-compatible digests of the content files |

`manifest.yaml` carries a generation timestamp and is deliberately **excluded**
from `hashes.txt`, as is any metrics file holding wall-clock timings.
Otherwise `sha256sum -c` would fail on a correct regeneration and the check
would be worthless.

Verify a bundle:

```bash
cd results/planning && sha256sum -c hashes.txt
```
