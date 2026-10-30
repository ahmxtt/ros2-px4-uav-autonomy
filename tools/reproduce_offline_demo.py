#!/usr/bin/env python3
"""End-to-end offline reproduction of the planning pipeline.

Runs the canonical offline chain with no ROS, no PX4 and no Gazebo::

    SDF world
      -> occupancy grid          (uav_autonomy.mapping)
      -> deterministic A* path   (uav_autonomy.planning)
           |-> line-of-sight reduction (uav_autonomy.smoothing)
           |     diagnostic side branch; its output is not used below
      -> B-spline + speed profile(uav_autonomy.trajectory), fitted to the A* path
      -> motion-limit validation (uav_autonomy.trajectory)

Usage::

    python3 tools/reproduce_offline_demo.py --output-dir /tmp/uav_demo

Run it from the repository root. All repository inputs are resolved relative to
this file rather than to the shell's working directory, so invoking it by
absolute path from elsewhere works too; CI does that as a standing check.

By default the demo plans the first mission leg on the *execution prior* map
(the altitude-adapted map used by the evaluated mission). The prior map is
regenerated from the SDF rather than shipped, which demonstrates the
determinism claim instead of merely asserting it: rebuild the grid and check
its hash.

The demo also re-checks the published planning and trajectory figures from
``results/`` when it is run with the default parameters, so a reviewer sees
immediately whether their environment reproduces the recorded numbers.
"""

from __future__ import annotations

import argparse
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import mapping as M  # noqa: E402
from uav_autonomy import planning as P  # noqa: E402
from uav_autonomy import smoothing as S  # noqa: E402
from uav_autonomy import trajectory as T  # noqa: E402
from uav_autonomy.artifacts import (  # noqa: E402
    format_float, sha256_of_file, write_csv, write_hashes, write_yaml,
)

# Endpoints of the canonical first leg (start -> orbit-1 entry), world frame.
DEFAULT_START_XY = (-50.0, -70.0)
DEFAULT_GOAL_XY = (18.080636, -0.148755)
DEFAULT_GOAL_RADIUS_M = 0.3

# Tree-canopy footprints at the 2 m reference altitude, in metres.
#
# The prior map rasterises trunk radii; at flight altitude it is the canopy the
# vehicle must clear, so the execution prior uses these larger radii instead.
# The values were measured once from the Collada meshes of the Fuel models in
# the z = [1.5, 2.5] m slab (max radius from the SDF pose anchor, rounded up:
# oak 3.975 -> 4.0, pine 1.470 -> 1.5). They are carried here as documented
# constants because this repository does not vendor those meshes; see NOTICE.md.
CANOPY_FOOTPRINTS_M = {"Pine Tree": 1.5, "Oak tree": 4.0}


def _rule(title: str) -> None:
    print(f"\n{'=' * 72}\n{title}\n{'=' * 72}")


def build_execution_prior(out_dir: Path, world_sdf: Path) -> tuple[Path, dict]:
    """Rasterise the SDF using canopy footprints at the reference altitude.

    The grid *geometry* is derived from the trunk footprints, so the execution
    prior shares an identical cell grid with the trunk-level prior map and the
    two are directly comparable cell by cell. Only the rasterised radii differ.
    """
    obstacles = M.parse_obstacles(world_sdf)
    geom = M.compute_grid(obstacles, M.RESOLUTION_M, M.PADDING_M)
    canopy = [
        M.Obstacle(
            name=o.name, model_type=o.model_type, x=o.x, y=o.y,
            footprint_radius_m=CANOPY_FOOTPRINTS_M.get(o.model_type, o.footprint_radius_m),
        )
        for o in obstacles
    ]
    image = M.rasterize(canopy, geom)

    map_dir = out_dir / "01_map"
    map_dir.mkdir(parents=True, exist_ok=True)
    pgm = map_dir / "map.pgm"
    map_yaml = map_dir / "map.yaml"
    M.write_pgm(image, pgm)
    M.write_map_yaml(map_yaml, geom, pgm.name)
    write_hashes(map_dir / "hashes.txt", [pgm, map_yaml])

    info = {
        "n_obstacles": len(canopy),
        "grid_cells": [int(geom.width_cells), int(geom.height_cells)],
        "resolution_m": float(geom.resolution),
        "origin_xy_m": [float(geom.origin_x), float(geom.origin_y)],
        "occupied_cells": int((image == M.PGM_OCCUPIED).sum()),
        "map_pgm_sha256": sha256_of_file(pgm),
        "footprints_m": CANOPY_FOOTPRINTS_M,
    }
    print(f"  obstacles       : {len(canopy)}")
    print(f"  grid            : {geom.width_cells} x {geom.height_cells} @ {geom.resolution} m")
    print(f"  origin          : ({geom.origin_x:.3f}, {geom.origin_y:.3f})")
    print(f"  occupied cells  : {info['occupied_cells']}")
    print(f"  map.pgm sha256  : {info['map_pgm_sha256']}")
    return map_yaml, info


def write_figure(out_png: Path, *, image, geom_origin, resolution, height,
                 astar_xy, reduced_xy, traj_xy, start_xy, goal_xy) -> bool:
    """Render an overview figure. Returns False if matplotlib is unavailable."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"  matplotlib unavailable ({exc}); skipping figure")
        return False

    x0 = geom_origin[0]
    y0 = geom_origin[1]
    x1 = x0 + image.shape[1] * resolution
    y1 = y0 + image.shape[0] * resolution

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    ax = axes[0]
    ax.imshow(image, cmap="gray", origin="upper", extent=[x0, x1, y0, y1],
              interpolation="nearest")
    ax.plot(astar_xy[:, 0], astar_xy[:, 1], lw=1.2, color="tab:blue",
            label=f"A* path ({len(astar_xy)} pts)")
    ax.plot(traj_xy[:, 0], traj_xy[:, 1], lw=1.4, color="tab:red",
            label=f"B-spline reference ({len(traj_xy)} samples)")
    ax.plot(reduced_xy[:, 0], reduced_xy[:, 1], "o--", ms=4, lw=0.9,
            color="tab:green", label=f"LoS reduced ({len(reduced_xy)} wpts)")
    ax.plot(*start_xy, "o", ms=9, color="tab:cyan", label="start")
    ax.plot(*goal_xy, "*", ms=15, color="gold", markeredgecolor="k", label="goal")
    ax.set_xlabel("x [m] (world ENU)")
    ax.set_ylabel("y [m] (world ENU)")
    ax.set_title("Planning on the execution prior map")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal")

    ax = axes[1]
    ax.set_xlabel("arc length s [m]")
    ax.set_ylabel("clearance [m]", color="tab:blue")
    ax.plot(_S_CACHE["s"], _S_CACHE["clearance"], lw=1.0, color="tab:blue")
    ax.axhline(_S_CACHE["r_pref"], ls="--", lw=1.0, color="tab:green",
               label=f"r_pref = {_S_CACHE['r_pref']:.2f} m")
    ax.axhline(_S_CACHE["r_eff"], ls=":", lw=1.0, color="tab:red",
               label=f"r_eff = {_S_CACHE['r_eff']:.2f} m")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    ax.legend(loc="upper right", fontsize=8)

    ax2 = ax.twinx()
    ax2.plot(_S_CACHE["s"], _S_CACHE["v"], lw=1.0, color="tab:orange")
    ax2.set_ylabel("v_ref [m/s]", color="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_title("Clearance and reference speed along the trajectory")

    fig.tight_layout()
    fig.savefig(out_png, dpi=130, metadata={"Software": None})
    plt.close(fig)
    return True


_S_CACHE: dict = {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--world-sdf", type=Path, default=REPO_ROOT / "sim/worlds/forest_map2.sdf")
    ap.add_argument("--robot", type=Path, default=REPO_ROOT / "sim/config/robot_map2_safe.yaml")
    ap.add_argument("--policy", type=Path, default=REPO_ROOT / "sim/config/mission_policy.yaml")
    ap.add_argument("--start-x", type=float, default=DEFAULT_START_XY[0])
    ap.add_argument("--start-y", type=float, default=DEFAULT_START_XY[1])
    ap.add_argument("--goal-x", type=float, default=DEFAULT_GOAL_XY[0])
    ap.add_argument("--goal-y", type=float, default=DEFAULT_GOAL_XY[1])
    ap.add_argument("--goal-radius", type=float, default=DEFAULT_GOAL_RADIUS_M)
    ap.add_argument("--no-figure", action="store_true")
    args = ap.parse_args()

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    t_start = time.perf_counter()

    print(f"repository : {REPO_ROOT}")
    print(f"output     : {out.resolve()}")
    print(f"python     : {platform.python_version()}   numpy {np.__version__}")

    # ---------------------------------------------------------- policy
    robot = yaml.safe_load(args.robot.read_text(encoding="utf-8"))
    policy = yaml.safe_load(args.policy.read_text(encoding="utf-8"))["planning_policy"]
    r_eff = float(robot["robot_radius"])
    r_pref = r_eff + float(policy["tracking_margin_m"])
    r_plan_min = r_pref + float(policy["spline_shrinkage_budget_m"])
    r_comfort = r_pref + float(policy["comfort_margin_m"])
    w_comfort = float(policy["w_comfort"])

    _rule("Step 1/5  Rasterise the SDF world into an occupancy grid")
    map_yaml, map_info = build_execution_prior(out, args.world_sdf)

    _rule("Step 2/5  Deterministic A* (8-connected, octile, no corner cutting)")
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    from PIL import Image
    image = np.array(Image.open(map_yaml.parent / meta["image"]).convert("L"))
    res = float(meta["resolution"])
    origin = list(meta["origin"])
    grid = G.classify_map(image, int(meta.get("negate", 0)),
                          float(meta.get("occupied_thresh", 0.65)),
                          float(meta.get("free_thresh", 0.196)))
    height = grid.shape[0]
    clear_grid = G.inflated_clear_grid(grid, int(math.ceil(r_plan_min / res)),
                                       unknown_is_blocked=False)
    clearance_cells = G.compute_clearance_metres(grid, res)

    start_xy = (args.start_x, args.start_y)
    goal_xy = (args.goal_x, args.goal_y)
    start_cell = G.world_to_cell(start_xy, origin, res, height)
    goal_cells, goal_centre, goal_radius_cells = G.collect_goal_cells(
        goal_xy, args.goal_radius, origin, res, grid.shape, clear_grid,
    )
    if not clear_grid[start_cell] or not goal_cells:
        print("ERROR: start or goal region is not inflation-safe", file=sys.stderr)
        return 2

    astar = P.a_star_comfort(
        clear_grid=clear_grid, clearance_m_field=clearance_cells,
        start=start_cell, goal_cells_set=frozenset(goal_cells),
        goal_centre=goal_centre, goal_radius_cells=goal_radius_cells,
        r_plan_min=r_plan_min, r_comfort=r_comfort, w_comfort=w_comfort,
    )
    if not astar.success:
        print("ERROR: A* found no path", file=sys.stderr)
        return 1

    astar_len = P.path_length_metres(astar.path_cells, res)
    astar_clr_min, astar_clr_mean = P.path_clearances(astar.path_cells, grid, res)
    astar_xy = P.path_cells_to_world(astar.path_cells, origin, res, height)
    print(f"  path points     : {len(astar.path_cells)}")
    print(f"  nodes expanded  : {astar.nodes_expanded}")
    print(f"  path length     : {astar_len:.3f} m")
    print(f"  clearance min   : {astar_clr_min:.3f} m   mean {astar_clr_mean:.3f} m")

    # Determinism is demonstrated, not asserted.
    repeat = P.a_star_comfort(
        clear_grid=clear_grid, clearance_m_field=clearance_cells,
        start=start_cell, goal_cells_set=frozenset(goal_cells),
        goal_centre=goal_centre, goal_radius_cells=goal_radius_cells,
        r_plan_min=r_plan_min, r_comfort=r_comfort, w_comfort=w_comfort,
    )
    deterministic = list(repeat.path_cells) == list(astar.path_cells)
    print(f"  re-run identical: {deterministic}")

    _rule("Step 3/5  Line-of-sight reduction and invariant check")
    from scipy.ndimage import distance_transform_edt
    dist_m = distance_transform_edt(~(grid == G.OCCUPIED)) * res
    reduced = S.string_pull(astar.path_cells, clear_grid)
    los_errors = S.validate_invariants(
        astar.path_cells, reduced, clear_grid, dist_m, res, r_plan_min,
    )
    reduced_xy = np.array(
        [G.cell_to_world(c, origin, res, height) for c in reduced], dtype=float,
    )
    print(f"  waypoints       : {len(astar.path_cells)} -> {len(reduced)}")
    print(f"  length          : {astar_len:.3f} m -> "
          f"{S.polyline_length_metres(reduced, res):.3f} m")
    print(f"  invariants      : {'all hold' if not los_errors else los_errors}")

    _rule("Step 4/5  Cubic B-spline refinement with clearance validation")
    map_data = G.load_map(map_yaml)
    clearance = G.clearance_field(map_data)
    sp = T.SpeedParams(v_nominal=1.5, v_min=0.3, a_lat_max=1.0, a_long_max=1.0)
    sel = T.refine_trajectory(
        astar_xy=astar_xy, map_data=map_data, clearance_m_field=clearance,
        r_eff=r_eff, r_pref=r_pref, sp=sp,
    )
    a = sel.selected
    print(f"  attempts run    : {[at.name for at in sel.attempts]}")
    print(f"  selected        : {a.name}  ({a.classification})")
    print(f"  samples         : {a.n_samples} from {a.n_control_points} control points")
    print(f"  length          : {a.trajectory_length_m:.3f} m")
    print(f"  clearance min   : {a.clearance_min_m:.4f} m  "
          f"(r_eff {r_eff}, r_pref {r_pref})")
    print(f"  max deviation   : {a.max_deviation_from_astar_m:.4f} m")

    _rule("Step 5/5  Motion-limit validation")
    seg_ds = np.diff(a.s)
    limits = T.check_motion_limits(
        v_ref=sel.v_ref, lateral_accel=sel.lateral_accel, segment_ds=seg_ds, sp=sp,
    )
    for k, v in limits.items():
        print(f"  {k:24s}: {v}")
    all_ok = all(v for k, v in limits.items() if isinstance(v, bool))

    # ------------------------------------------------------------ outputs
    plan_dir = out / "02_plan"
    traj_dir = out / "03_trajectory"
    plan_dir.mkdir(exist_ok=True)
    traj_dir.mkdir(exist_ok=True)

    path_csv = plan_dir / "path.csv"
    write_csv(path_csv, ["idx", "x_m", "y_m", "row", "col"],
              [(i, format_float(x), format_float(y), c[0], c[1])
               for i, ((x, y), c) in enumerate(zip(astar_xy, astar.path_cells))])
    reduced_csv = plan_dir / "path_reduced.csv"
    write_csv(reduced_csv, ["idx", "x_m", "y_m", "row", "col"],
              [(i, format_float(x), format_float(y), c[0], c[1])
               for i, ((x, y), c) in enumerate(zip(reduced_xy, reduced))])
    write_hashes(plan_dir / "hashes.txt", [path_csv, reduced_csv])

    traj_csv = traj_dir / "trajectory.csv"
    write_csv(
        traj_csv,
        ["idx", "s_m", "x_m", "y_m", "yaw_rad", "curvature_1_per_m",
         "v_ref_mps", "a_lat_mps2", "clearance_m"],
        [(i, format_float(a.s[i]), format_float(a.samples_xy[i, 0]),
          format_float(a.samples_xy[i, 1]), format_float(a.yaw[i]),
          format_float(a.curvature[i]), format_float(sel.v_ref[i]),
          format_float(sel.lateral_accel[i]), format_float(a.clearance[i]))
         for i in range(a.n_samples)],
    )
    write_hashes(traj_dir / "hashes.txt", [traj_csv])

    metrics = {
        "demo": "offline planning pipeline",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
        "map": map_info,
        "policy": {
            "r_eff_m": r_eff, "r_pref_m": r_pref,
            "r_plan_min_m": r_plan_min, "r_comfort_m": r_comfort,
            "w_comfort": w_comfort,
        },
        "query": {
            "start_xy_world": list(start_xy),
            "goal_xy_world": list(goal_xy),
            "goal_radius_m": args.goal_radius,
        },
        "astar": {
            "n_points": len(astar.path_cells),
            "nodes_expanded": int(astar.nodes_expanded),
            "path_length_m": float(astar_len),
            "clearance_min_m": float(astar_clr_min),
            "clearance_mean_m": float(astar_clr_mean),
            "repeat_run_identical": bool(deterministic),
        },
        "los_reduction": {
            "waypoints_before": len(astar.path_cells),
            "waypoints_after": len(reduced),
            "length_after_m": float(S.polyline_length_metres(reduced, res)),
            "invariant_errors": los_errors,
        },
        "trajectory": {
            "selected_attempt": a.name,
            "classification": a.classification,
            "n_control_points": int(a.n_control_points),
            "n_samples": int(a.n_samples),
            "trajectory_length_m": float(a.trajectory_length_m),
            "clearance_min_m": float(a.clearance_min_m),
            "clearance_mean_m": float(a.clearance_mean_m),
            "max_deviation_from_astar_m": float(a.max_deviation_from_astar_m),
            "curvature_max_1_per_m": float(a.curvature_max_1_per_m),
            "estimated_travel_time_s": float(sel.estimated_travel_time_s),
        },
        "motion_limits": limits,
        "wall_clock_s": None,  # filled in below
    }

    # ------------------------------------------- compare with published results
    comparison = None
    default_query = (
        (args.start_x, args.start_y) == DEFAULT_START_XY
        and (args.goal_x, args.goal_y) == DEFAULT_GOAL_XY
        and args.goal_radius == DEFAULT_GOAL_RADIUS_M
    )
    published = REPO_ROOT / "results/planning/metrics.yaml"
    published_traj = REPO_ROOT / "results/planning/trajectory_metrics.yaml"
    if default_query and published.exists() and published_traj.exists():
        _rule("Comparison with the published results in results/planning/")
        ref = yaml.safe_load(published.read_text(encoding="utf-8"))
        ref_t = yaml.safe_load(published_traj.read_text(encoding="utf-8"))
        checks = [
            ("A* path length [m]", astar_len, ref["path_length_m"], 1e-6),
            ("A* nodes expanded", astar.nodes_expanded, ref["nodes_expanded"], 0),
            ("A* clearance min [m]", astar_clr_min, ref["clearance_min_m"], 1e-9),
            ("B-spline length [m]", a.trajectory_length_m,
             ref_t["trajectory_length_m"], 1e-6),
            ("B-spline clearance min [m]", a.clearance_min_m,
             ref_t["clearance_min_m"], 1e-9),
        ]
        comparison = {}
        worst = 0.0
        for name, mine, refv, tol in checks:
            delta = abs(float(mine) - float(refv))
            ok = delta <= tol
            worst = max(worst, delta)
            comparison[name] = {"reproduced": float(mine), "published": float(refv),
                                "abs_delta": float(delta), "match": bool(ok)}
            print(f"  {name:30s} {float(mine):>14.6f}  vs published "
                  f"{float(refv):>14.6f}   delta {delta:.3e}  {'OK' if ok else 'MISMATCH'}")
        print(f"  worst absolute delta: {worst:.3e}")
        metrics["comparison_with_published"] = comparison

    # ------------------------------------------------------------- figure
    figure_written = False
    if not args.no_figure:
        _rule("Figure")
        _S_CACHE.update(s=a.s, clearance=a.clearance, v=sel.v_ref,
                        r_eff=r_eff, r_pref=r_pref)
        figure_written = write_figure(
            out / "overview.png",
            image=image, geom_origin=origin, resolution=res, height=height,
            astar_xy=astar_xy, reduced_xy=reduced_xy, traj_xy=a.samples_xy,
            start_xy=start_xy, goal_xy=goal_xy,
        )
        if figure_written:
            print(f"  wrote {out / 'overview.png'}")

    metrics["wall_clock_s"] = round(time.perf_counter() - t_start, 3)
    metrics["figure_written"] = figure_written
    write_yaml(out / "metrics.yaml", metrics)

    _rule("Summary")
    ok = bool(astar.success and not los_errors and a.classification != "fail"
              and all_ok and deterministic)
    if comparison is not None:
        ok = ok and all(c["match"] for c in comparison.values())
    print(f"  wall clock      : {metrics['wall_clock_s']} s")
    print(f"  outputs         : {out.resolve()}")
    print(f"  RESULT          : {'PASS' if ok else 'CHECK OUTPUT'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
