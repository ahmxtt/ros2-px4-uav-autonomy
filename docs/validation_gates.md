# The incremental validation-gate framework

The system was not built and then tested. It was built through a sequence of
**validation gates**: each gate defines a capability, the evidence that would
demonstrate it, and the conditions under which it fails. A gate had to close
before the next one opened.

This matters for reading the results. A gate is a scope boundary, so a claim
carries the scope of the gate that produced it — an offline planning gate
cannot support a statement about flight behaviour, and a live execution gate
cannot support a statement about statistical robustness.

Gate labels appear in the source comments, in run verdicts and in the result
bundles. They are recorded here so those labels are never unexplained.

*Stop-Replan-Resume* (SRR) is this project's name for the obstacle-response
sequence: stop, plan a local detour, resume the original mission. It is the
subject of Gate 6 and the abbreviation is used below.

## The gates

| Gate | Scope | Closed by |
|---|---|---|
| **Gate 0** | Communication and Offboard smoke test. PX4 ↔ ROS 2 visibility, message deserialisation, setpoint streaming, the OFFBOARD handshake, hover, and the setpoint-loss failsafe. | Offboard smoke evidence |
| **Gate 1** | Prior map. Occupancy grid generated deterministically from the world description. | Prior map bundle |
| **Gate 2** | Global planning. 2A: geometric A* baseline. 2B: the candidate policy — a robust candidate and a boundary candidate, with an explicit classification rule. | Planning bundle |
| **Gate 3** | Trajectory refinement and mission reference. Clearance-validated B-spline refinement; multi-POI mission reference with tangent-compatible orbit entry and exit. | Trajectory and mission-reference bundles |
| **Gate 4** | Coordinate transforms, communication and basic Offboard execution. 4A: offline ENU/world → PX4 local NED transform validation. 4B: no-arm setpoint-streamer validation. 4C: first armed, defensively bounded Offboard segment. | Transform validation, streamer validation, short armed segment |
| **Gate 5** | Nominal mission preparation and execution. 5A: altitude-aware execution prior (tree canopies projected at the flight altitude). 5B: full single-segment tracking. 5C: complete multi-POI mission executed end to end. | Execution prior, segment tracking, full-mission run |
| **Gate 6** | Obstacle response and Stop–Replan–Resume validation. 6A: offline global replan, the geometric correctness reference. 6B: detection replay. 6C.5: offline local-ROI replan validator. 6D: the live run, in two variants below. | Offline replan validators, then the live run |

## Gate 6D: two variants

Gate 6D is the live Stop–Replan–Resume run. It has two variants because
mechanism and perception must be falsifiable separately — otherwise a working
state machine fed by ground truth could be mistaken for working perception.

| Variant | `--oracle-mode` | What supplies the obstacle | What it demonstrates |
|---|---|---|---|
| **Gate 6D-A** — oracle mechanics | `replan` | Ground truth, on the patched prior map | The state machine: stop, replan, resume, rejoin. Says nothing about perception. |
| **Gate 6D-B** — perception-driven | `off` | LiDAR clustering plus a novelty test against the *unpatched* prior. Ground truth is retained in a separate evaluation-only namespace and gates no decision. | The full sense–plan–act loop. |

**Gate 6D-B has four preconditions, and all four must hold at once:**

1. **The physical obstacle is present in Gazebo.** It is not in the world file;
   `tools/spawn_srr_obstacle.py` spawns `srr_unknown_obstacle` from
   `sim/obstacles/srr_unknown_obstacle.sdf` into the running world before the
   run. If it is not there, the LiDAR has nothing to return and the run is
   vacuous.
2. **The prior map does not contain it.** The execution prior is rasterised
   from `sim/worlds/forest_map2.sdf`, which does not describe the obstacle, so
   the replanner searches a map in which the obstacle does not exist.
3. **Detection is LiDAR plus novelty.** Clusters from `/scan_filtered` inside a
   corridor ahead of the vehicle, a hysteresis streak, and a novelty test
   against that unpatched prior. Only the sensed obstacle is stamped into the
   local ROI.
4. **No ground truth in any decision.** `sim/obstacles/obstacle.yaml` is read
   once at startup into a separate evaluation-only namespace. It does not gate
   detection, obstacle construction, replanning or control decisions: the
   online-decision obstacle is left empty, so an accidental decision use fails
   loud instead of quietly assisting the planner. Ground truth may still be
   read for diagnostics while the run is in progress, and for the post-run
   metrics, so detection accuracy can be scored offline. The node runs
   correctly with the file absent, and that absence is the oracle-leak test.

Gate 6D-A deliberately violates 3 and 4, which is why it can validate the
mechanics and nothing else.

**The published run is Gate 6D-B.** A perception pass additionally requires
`detection_mode == "lidar_cluster_detection"`; a synthetic detection event is
logged but can never produce one.

Run verdicts record this as a self-describing pair, so the label never appears
alone:

```yaml
scenario_id: stop_replan_resume_perception
validation_gate: "Gate 6D-B"
validation_gate_note: "Gate 6D-B: perception-driven. Detection from LiDAR
  clustering plus a novelty test against the prior map; the replan uses the
  sensed obstacle on the unpatched prior. Ground truth is retained in a
  separate evaluation-only namespace: it does not gate detection, obstacle
  construction, replanning or control decisions, but it may be used to compute
  diagnostic metrics and post-run evaluation values."
```

## Which gates this release publishes

| Gate | In this release |
|---|---|
| Gate 1, 2, 3 | **Fully reproducible offline.** `tools/reproduce_offline_demo.py` regenerates the map, the plan and the refined trajectory and checks them against `results/planning/`. |
| Gate 5C | **Curated summary** of one controlled SITL run: `results/nominal_mission/tracking_metrics.yaml`. |
| Gate 5C + ground-truth audit | **Curated summary** of a second controlled SITL run: `results/nominal_mission/state_consistency_metrics.yaml`. |
| Gate 6C.5 geometry | **Fully reproducible offline.** `tools/reproduce_srr_geometry.py` re-derives the published detour exactly. |
| Gate 6D-B | **Curated summary** of one controlled SITL run: `results/stop_replan_resume/srr_metrics.yaml`. |
| Gate 0, 4A, 4B, 4C, 5A, 5B, 6A, 6B | Not published as separate bundles. They are intermediate stages whose outcome is carried by the later gates that depend on them. |

See [data_availability.md](data_availability.md) for exactly what each bundle
contains and what its hashes do and do not prove.
