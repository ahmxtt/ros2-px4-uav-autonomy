#!/usr/bin/env python3
"""Regenerate the three documentation figures from public code and data only.

The figures are committed under ``docs/figures/`` because a README must render
on GitHub without the reader running anything. This script is how they are
produced, so they are auditable rather than opaque:

1. ``architecture.png``  - the pipeline overview (drawn, not measured).
2. ``nominal_planning.png`` - the A* path and the refined trajectory on the
   execution prior map, with the clearance profile. Regenerated from the SDF.
3. ``srr_geometry.png`` - the Stop-Replan-Resume detour, sensed obstacle and
   rejoin, reproduced offline from the published result bundle.

Figures 2 and 3 are derived from the same computations the reproduction tools
run, so a figure can never drift away from the published numbers without those
tools failing first.

Usage::

    python3 tools/make_docs_figures.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch  # noqa: E402
from PIL import Image  # noqa: E402

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import mapping as M  # noqa: E402
from uav_autonomy import planning as P  # noqa: E402
from uav_autonomy import replanning as R  # noqa: E402
from uav_autonomy import trajectory as T  # noqa: E402
from uav_autonomy.frames import ned_to_world  # noqa: E402

OUT = REPO_ROOT / "docs" / "figures"
SRR = REPO_ROOT / "results" / "stop_replan_resume"
CANOPY_M = {"Pine Tree": 1.5, "Oak tree": 4.0}
SAVE = dict(dpi=140, bbox_inches="tight", metadata={"Software": None})

INK = "#1b2430"
MUTED = "#6b7785"
ACCENT = "#c2410c"
BLUE = "#1d4ed8"
GREEN = "#15803d"


def build_execution_prior():
    """Rebuild the execution prior and return it via a written ``map.yaml``.

    The round trip through ``map.yaml`` is load-bearing, not incidental. The
    in-memory grid origin is ``-80.30000000000001`` (a float artefact of
    snapping the origin to a cell multiple), while ``write_map_yaml`` formats
    it to three decimals as ``-80.300``. That 1e-14 difference is enough to
    shift the start cell by one, which changes the A* path by one cell and the
    reported length by 0.05 m.

    Every other entry point -- the offline demo, the CLI tools and the
    regression test -- reads a written ``map.yaml``, so the figures must too,
    or they would show numbers that disagree with the documentation.
    """
    import tempfile
    obstacles = M.parse_obstacles(REPO_ROOT / "sim/worlds/forest_map2.sdf")
    geom = M.compute_grid(obstacles, M.RESOLUTION_M, M.PADDING_M)
    canopy = [M.Obstacle(o.name, o.model_type, o.x, o.y, CANOPY_M[o.model_type])
              for o in obstacles]
    image = M.rasterize(canopy, geom)
    d = Path(tempfile.mkdtemp(prefix="uav_figs_"))
    M.write_pgm(image, d / "map.pgm")
    M.write_map_yaml(d / "map.yaml", geom, "map.pgm")
    map_data = G.load_map(d / "map.yaml")
    meta = yaml.safe_load((d / "map.yaml").read_text())
    return geom, image, map_data, meta


# ---------------------------------------------------------------- figure 1

def figure_architecture(path: Path) -> None:
    """System data flow.

    Three lanes, matching how the system actually runs:

      offline / pre-mission : world -> map -> A* -> LoS -> B-spline ->
                              mission reference -> frame transform
      online execution      : mission reference -> Offboard executor -> PX4 SITL
      obstacle response     : LiDAR -> detection -> stop/hover -> local ROI
                              replan -> resume anchor + rejoin -> executor

    Deliberate correctness constraints, because the obvious "boxes and arrows"
    layout gets all four of these wrong:
      * A* never reaches PX4 directly; it feeds refinement, then the reference.
      * The frame transform converts the reference; it does not replan.
      * LiDAR reaches the executor only through the obstacle-response chain.
      * The local replan rejoins the mission reference; it does not bypass it.
    """
    fig, ax = plt.subplots(figsize=(13.6, 9.4))
    ax.set_xlim(0, 136)
    ax.set_ylim(0, 96)
    ax.axis("off")

    def box(x, y, w, h, title, sub, ec, ls="-", tfs=9.4):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                                    boxstyle="round,pad=0.45,rounding_size=1.1",
                                    fc="#ffffff", ec=ec, lw=1.5, ls=ls, zorder=3))
        ax.text(x + w / 2, y + h * 0.62, title, ha="center", va="center",
                fontsize=tfs, fontweight="bold", color=INK, zorder=4)
        ax.text(x + w / 2, y + h * 0.25, sub, ha="center", va="center",
                fontsize=7.2, color=MUTED, zorder=4)
        return (x, y, w, h)

    def arrow(p1, p2, color=MUTED, rad=0.0, lw=1.4):
        ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="-|>", mutation_scale=12,
                                     lw=lw, color=color, shrinkA=3, shrinkB=3,
                                     connectionstyle=f"arc3,rad={rad}", zorder=2))

    def note(x, y, text, color):
        ax.text(x, y, text, fontsize=7.3, color=color, ha="center", va="center",
                zorder=6, bbox=dict(fc="white", ec="none", pad=1.4))

    def R(b): return (b[0] + b[2], b[1] + b[3] / 2)
    def L(b): return (b[0], b[1] + b[3] / 2)
    def T(b): return (b[0] + b[2] / 2, b[1] + b[3])
    def B(b): return (b[0] + b[2] / 2, b[1])

    # ---- lane backgrounds -------------------------------------------------
    ax.add_patch(plt.Rectangle((0.5, 56.0), 135, 39.0, fc="#f0fdf4",
                               ec="#bbf7d0", lw=1.2, zorder=0))
    ax.add_patch(plt.Rectangle((0.5, 31.0), 135, 21.0, fc="#fff7ed",
                               ec="#fed7aa", lw=1.2, zorder=0))
    ax.add_patch(plt.Rectangle((0.5, 1.5), 135, 25.5, fc="#eff6ff",
                               ec="#bfdbfe", lw=1.2, zorder=0))
    ax.text(2.2, 92.2, "OFFLINE / PRE-MISSION    deterministic, no simulator",
            fontsize=9.4, fontweight="bold", color=GREEN, zorder=5)
    ax.text(2.2, 49.0, "ONLINE EXECUTION    controlled PX4 SITL / Gazebo run",
            fontsize=9.4, fontweight="bold", color=ACCENT, zorder=5)
    ax.text(2.2, 24.2, "OBSTACLE RESPONSE    Stop-Replan-Resume (SRR)",
            fontsize=9.4, fontweight="bold", color=BLUE, zorder=5)

    # ---- offline lane: two rows so nothing is cramped ---------------------
    h = 12.5
    r1, r2 = 74.5, 58.5
    b_world = box(4.0, r1, 26.0, h, "Gazebo world", "sim/worlds/forest_map2.sdf", GREEN)
    b_map   = box(36.0, r1, 26.0, h, "occupancy map", "SDF rasterisation", GREEN)
    b_astar = box(68.0, r1, 26.0, h, "A* planning", "octile, no corner cutting", GREEN)
    b_los   = box(100.0, r1, 30.0, h, "line-of-sight reduction", "string pulling", GREEN)
    for a, b in ((b_world, b_map), (b_map, b_astar), (b_astar, b_los)):
        arrow(R(a), L(b), GREEN)

    b_bspl = box(36.0, r2, 30.0, h, "B-spline refinement",
                 "clearance-validated cascade", GREEN)
    b_ref  = box(72.0, r2, 26.0, h, "mission reference", "world ENU + v_ref", GREEN)
    b_tf   = box(104.0, r2, 26.0, h, "frame transform", "ENU \u2192 NED", GREEN)
    # Wrap from row 1 to row 2.
    arrow(B(b_los), T(b_bspl), GREEN, rad=0.18)
    arrow(R(b_bspl), L(b_ref), GREEN)
    arrow(R(b_ref), L(b_tf), GREEN)

    # ---- online lane ------------------------------------------------------
    ye, he = 33.5, 13.5
    b_exec = box(40.0, ye, 34.0, he, "Offboard mission executor",
                 "20 Hz setpoints \u00b7 safety state machine", ACCENT)
    b_px4  = box(88.0, ye, 30.0, he, "PX4 SITL + Gazebo",
                 "uXRCE-DDS \u00b7 EKF2 \u00b7 physics", ACCENT)
    _tf_x = b_tf[0] + b_tf[2] * 0.5
    _ex_x = b_exec[0] + b_exec[2] * 0.78
    arrow((b_exec[0] + b_exec[2], ye + he * 0.66),
          (b_px4[0], ye + he * 0.66), ACCENT)
    note(81.0, ye + he * 0.66 + 2.5, "TrajectorySetpoint", ACCENT)
    arrow((b_px4[0], ye + he * 0.30),
          (b_exec[0] + b_exec[2], ye + he * 0.30), ACCENT)
    note(81.0, ye + he * 0.30 - 2.6, "status \u00b7 local position", ACCENT)

    # Mission reference, transformed to NED, feeds the executor.
    def _route_pre(points, color, lw=1.4):
        ax.plot([q[0] for q in points], [q[1] for q in points], color=color,
                lw=lw, zorder=2, solid_capstyle="round")
        ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>",
                                     mutation_scale=12, lw=lw, color=color,
                                     shrinkA=0, shrinkB=2, zorder=2))
    _route_pre([(_tf_x, b_tf[1]), (_tf_x, 54.0), (_ex_x, 54.0), (_ex_x, ye + he)],
               ACCENT)
    note(96.0, 54.0, "NED setpoint stream", ACCENT)

    # ---- obstacle-response lane ------------------------------------------
    yr, hr = 5.0, 13.5
    b_lidar  = box(3.0, yr, 20.0, hr, "2D LiDAR", "/scan_filtered", MUTED,
                   ls="--", tfs=9.0)
    b_det    = box(26.0, yr, 24.0, hr, "obstacle detection",
                   "clustering + prior novelty", BLUE, tfs=9.0)
    b_stop   = box(53.0, yr, 20.0, hr, "stop and hover", "stability gate", BLUE, tfs=9.0)
    b_replan = box(76.0, yr, 24.0, hr, "local ROI replan",
                   "crop \u00b7 patch \u00b7 A* \u00b7 refine", BLUE, tfs=9.0)
    b_join   = box(103.0, yr, 29.0, hr, "resume anchor + rejoin",
                   "reintegrate mission reference", BLUE, tfs=9.0)
    for a, b in ((b_lidar, b_det), (b_det, b_stop), (b_stop, b_replan),
                 (b_replan, b_join)):
        arrow(R(a), L(b), BLUE)

    # ---- long-haul connections -------------------------------------------
    # arc3 curves are unmanageable over these spans and land on the wrong box,
    # so route them explicitly as orthogonal polylines with a single arrowhead.
    def route(points, color, lw=1.4, ls="-"):
        xs = [q[0] for q in points]
        ys = [q[1] for q in points]
        ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=2,
                solid_capstyle="round")
        ax.add_patch(FancyArrowPatch(points[-2], points[-1], arrowstyle="-|>",
                                     mutation_scale=12, lw=lw, color=color,
                                     shrinkA=0, shrinkB=2, zorder=2))

    # PX4 produces the simulated LiDAR scan: down the far right, along the
    # bottom gutter, into the LiDAR box from below.
    route([(b_px4[0] + b_px4[2] - 3.0, b_px4[1]),
           (b_px4[0] + b_px4[2] - 3.0, 29.2),
           (b_lidar[0] + b_lidar[2] * 0.5, 29.2),
           (b_lidar[0] + b_lidar[2] * 0.5, yr + hr)], MUTED, ls="--")
    note(72.0, 29.2, "simulated scan", MUTED)

    # The unpatched prior map is what the local replanner searches: down the
    # far left, then right into the replan box.
    # Leave from the map's left edge and use the clear gutter at x = 32.5, so
    # the line does not cross the B-spline box sitting directly below the map.
    route([(b_map[0], b_map[1] + b_map[3] * 0.5),
           (32.5, b_map[1] + b_map[3] * 0.5),
           (32.5, 21.0),
           (b_replan[0] + b_replan[2] * 0.5, 21.0),
           (b_replan[0] + b_replan[2] * 0.5, yr + hr)], GREEN)
    note(57.0, 21.0, "unpatched prior map", GREEN)

    # The response chain returns to the executor: up the right edge of the
    # rejoin box, then left into the executor from below.
    route([(b_join[0] + b_join[2] * 0.5, yr + hr),
           (b_join[0] + b_join[2] * 0.5, 25.0),
           (b_exec[0] + b_exec[2] * 0.35, 25.0),
           (b_exec[0] + b_exec[2] * 0.35, ye)], BLUE)
    note(96.0, 25.0, "stitched reference \u2192 executor", BLUE)

    ax.text(135.0, 0.2,
            "green: src/uav_autonomy (offline)    orange: ros2/nodes (online)"
            "    blue: Stop-Replan-Resume path",
            ha="right", fontsize=7.6, color=MUTED)
    fig.savefig(path, **SAVE)
    plt.close(fig)


# ---------------------------------------------------------------- figure 2

def figure_nominal(path: Path) -> None:
    """A* path and refined trajectory, regenerated from the SDF."""
    geom, image, map_data, meta = build_execution_prior()
    res = float(meta["resolution"])
    origin = list(meta["origin"])
    sem = G.classify_map(image, 0, 0.65, 0.196)
    height = sem.shape[0]

    robot = yaml.safe_load((REPO_ROOT / "sim/config/robot_map2_safe.yaml").read_text())
    pol = yaml.safe_load((REPO_ROOT / "sim/config/mission_policy.yaml").read_text())["planning_policy"]
    r_eff = float(robot["robot_radius"])
    r_pref = r_eff + float(pol["tracking_margin_m"])
    r_plan_min = r_pref + float(pol["spline_shrinkage_budget_m"])
    r_comfort = r_pref + float(pol["comfort_margin_m"])

    clear = G.inflated_clear_grid(sem, int(math.ceil(r_plan_min / res)), False)
    field = G.compute_clearance_metres(sem, res)
    start_xy, goal_xy = (-50.0, -70.0), (18.080636, -0.148755)
    sc = G.world_to_cell(start_xy, origin, res, height)
    gc, gcen, grad = G.collect_goal_cells(goal_xy, 0.3, origin, res, sem.shape, clear)
    a = P.a_star_comfort(clear_grid=clear, clearance_m_field=field, start=sc,
                         goal_cells_set=frozenset(gc), goal_centre=gcen,
                         goal_radius_cells=grad, r_plan_min=r_plan_min,
                         r_comfort=r_comfort, w_comfort=float(pol["w_comfort"]))
    xy = P.path_cells_to_world(a.path_cells, origin, res, height)

    sel = T.refine_trajectory(astar_xy=xy, map_data=map_data,
                              clearance_m_field=G.clearance_field(map_data),
                              r_eff=r_eff, r_pref=r_pref,
                              sp=T.SpeedParams(1.5, 0.3, 1.0, 1.0))
    tr = sel.selected

    fig = plt.figure(figsize=(13.5, 6.6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.15, 1.0], wspace=0.22)

    ax = fig.add_subplot(gs[0, 0])
    ox, oy = float(origin[0]), float(origin[1])
    x1 = ox + geom.width_cells * res
    y1 = oy + geom.height_cells * res
    ax.imshow(image, cmap="gray", origin="upper",
              extent=[ox, x1, oy, y1], interpolation="nearest")
    ax.plot(xy[:, 0], xy[:, 1], lw=1.6, color=BLUE,
            label=f"A* path — {P.path_length_metres(a.path_cells, res):.3f} m")
    ax.plot(tr.samples_xy[:, 0], tr.samples_xy[:, 1], lw=1.8, color=ACCENT,
            label=f"B-spline — {tr.trajectory_length_m:.3f} m")
    ax.plot(*start_xy, "o", ms=10, color="#0891b2", mec="k", label="start")
    ax.plot(*goal_xy, "*", ms=17, color="#facc15", mec="k", label="goal")
    pad = 12
    ax.set_xlim(min(xy[:, 0].min(), start_xy[0]) - pad, max(xy[:, 0].max(), goal_xy[0]) + pad)
    ax.set_ylim(min(xy[:, 1].min(), start_xy[1]) - pad, max(xy[:, 1].max(), goal_xy[1]) + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] — world ENU")
    ax.set_ylabel("y [m] — world ENU")
    ax.set_title("Nominal planning on the execution prior map (2 m reference altitude)",
                 fontsize=11)
    ax.legend(loc="upper left", fontsize=8.5, framealpha=0.92)

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(tr.s, tr.clearance, lw=1.2, color=BLUE, label="clearance along the trajectory")
    for val, col, lab in ((r_plan_min, GREEN, f"$r_{{plan\\,min}}$ = {r_plan_min:.2f} m"),
                          (r_pref, "#ca8a04", f"$r_{{pref}}$ = {r_pref:.2f} m"),
                          (r_eff, ACCENT, f"$r_{{eff}}$ = {r_eff:.2f} m")):
        ax2.axhline(val, ls="--", lw=1.1, color=col, label=lab)
    ax2.plot(tr.s[np.argmin(tr.clearance)], tr.clearance.min(), "v", ms=9,
             color=ACCENT, mec="k",
             label=f"minimum {tr.clearance_min_m:.3f} m")
    ax2.set_xlabel("arc length s [m]")
    ax2.set_ylabel("clearance to raw obstacles [m]")
    ax2.set_ylim(0, min(6.0, float(tr.clearance.max()) * 1.05))
    ax2.set_title("Clearance of the refined trajectory", fontsize=11)
    ax2.legend(loc="upper right", fontsize=8.5, framealpha=0.92)
    ax2.grid(alpha=0.25)

    fig.savefig(path, **SAVE)
    plt.close(fig)
    return tr


# ---------------------------------------------------------------- figure 3

def figure_srr(path: Path) -> None:
    """Stop-Replan-Resume detour, reproduced offline from public data."""
    geom, image, _map_data, meta = build_execution_prior()

    sensed = yaml.safe_load((SRR / "sensed_obstacle_metrics.yaml").read_text())
    join = yaml.safe_load((SRR / "resume_join_metrics.yaml").read_text())
    obs_xy = tuple(sensed["perception_sensed_obstacle"]["centroid_world_xy"])
    obs_r = float(sensed["perception_sensed_obstacle"]["radius_m"])
    gt = sensed["ground_truth_eval_only"]
    resume_xy = tuple(join["resume_xy_world"])

    hover_xy = None
    with (SRR / "event_log.csv").open() as f:
        for row in csv.DictReader(f):
            if row["event"] == "stop_command_sent":
                inner = row["info"].split("hover_ned=(", 1)[1].split(")", 1)[0]
                n, e, _d = (float(v) for v in inner.split(","))
                hover_xy = ned_to_world(n, e)

    xs, ys, ss = [], [], []
    with (SRR / "mission_segment_window.csv").open() as f:
        for row in csv.DictReader(f):
            ss.append(float(row["s_m"])); xs.append(float(row["x_m"])); ys.append(float(row["y_m"]))
    xs, ys, ss = np.asarray(xs), np.asarray(ys), np.asarray(ss)

    roi = R.derive_roi_spec(geom.resolution)
    ih = int(np.argmin(np.hypot(xs - hover_xy[0], ys - hover_xy[1])))
    ir = int(np.argmin(np.hypot(xs - resume_xy[0], ys - resume_xy[1])))
    cand = R.run_local_roi_replan(
        full_image=image, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
        obstacle_xy=obs_xy, obs_r=obs_r,
        s_min=float(ss[ih]) - roi.upstream_margin_m,
        s_max=float(ss[ir]) + roi.smoothing_margin_m,
        lateral_half_width=roi.lateral_half_width_m,
        hover_xy=hover_xy, resume_xy=resume_xy, hover_idx=ih, resume_idx=ir)

    fig, ax = plt.subplots(figsize=(9.6, 8.2))
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    x1 = ox + geom.width_cells * geom.resolution
    y1 = oy + geom.height_cells * geom.resolution
    ax.imshow(image, cmap="gray", origin="upper",
              extent=[ox, x1, oy, y1], interpolation="nearest")

    ax.plot(xs, ys, lw=1.3, ls="--", color=MUTED, label="original mission reference")
    ax.plot(cand.astar_xy[:, 0], cand.astar_xy[:, 1], lw=1.2, color=BLUE,
            label=f"local A* detour — {cand.astar_path_length_m:.2f} m")
    ax.plot(cand.smoothed_xy[:, 0], cand.smoothed_xy[:, 1], lw=2.1, color=ACCENT,
            label=f"refined detour — {cand.smoothed_length_m:.2f} m, "
                  f"clearance {cand.clearance_min_m:.3f} m")
    ax.add_patch(Circle(obs_xy, obs_r, fc="#fb923c", ec="#c2410c", alpha=0.55,
                        label=f"sensed obstacle (r = {obs_r:.2f} m)"))
    ax.add_patch(Circle(tuple(gt["centroid_world_xy"]), float(gt["radius_m"]),
                        fc="none", ec=INK, ls=":", lw=1.5,
                        label=f"ground truth (evaluation only, r = {gt['radius_m']:.1f} m)"))
    ax.plot(*hover_xy, "o", ms=11, color="#0891b2", mec="k", label="stop / hover")
    ax.plot(*resume_xy, "*", ms=18, color="#facc15", mec="k", label="resume anchor")

    pad = 13
    allx = np.concatenate([cand.smoothed_xy[:, 0], [obs_xy[0], hover_xy[0], resume_xy[0]]])
    ally = np.concatenate([cand.smoothed_xy[:, 1], [obs_xy[1], hover_xy[1], resume_xy[1]]])
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] — world ENU")
    ax.set_ylabel("y [m] — world ENU")
    ax.set_title("Stop–Replan–Resume: sensed obstacle, local detour and rejoin\n"
                 "(geometry reproduced offline from the published run)", fontsize=11)
    ax.legend(loc="upper left", fontsize=8.4, framealpha=0.93)
    fig.savefig(path, **SAVE)
    plt.close(fig)
    return cand


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--output-dir", type=Path, default=OUT)
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    print("1/3 architecture ...")
    figure_architecture(args.output_dir / "architecture.png")
    print("2/3 nominal planning (regenerating map + plan + trajectory) ...")
    tr = figure_nominal(args.output_dir / "nominal_planning.png")
    print(f"    trajectory {tr.trajectory_length_m:.6f} m, "
          f"clearance {tr.clearance_min_m:.6f} m, {tr.classification}")
    print("3/3 stop-replan-resume geometry ...")
    c = figure_srr(args.output_dir / "srr_geometry.png")
    print(f"    detour {c.smoothed_length_m:.6f} m, "
          f"clearance {c.clearance_min_m:.6f} m, {c.classification}")
    print("\nchecking the figures against the published results ...")
    pub_t = yaml.safe_load((REPO_ROOT / "results/planning/trajectory_metrics.yaml").read_text())
    pub_s = yaml.safe_load((REPO_ROOT / "results/stop_replan_resume/replan_runtime_metrics.yaml").read_text())
    bad = []
    for label, mine, ref in (
        ("nominal trajectory length [m]", tr.trajectory_length_m, pub_t["trajectory_length_m"]),
        ("nominal clearance min [m]", tr.clearance_min_m, pub_t["clearance_min_m"]),
        ("SRR detour length [m]", c.smoothed_length_m, pub_s["smoothed_length_m"]),
        ("SRR clearance min [m]", c.clearance_min_m, pub_s["clearance_min_m"]),
    ):
        ok = abs(float(mine) - float(ref)) <= 1e-6
        print(f"  {'OK  ' if ok else 'FAIL'} {label:32s} {float(mine):.6f} vs {float(ref):.6f}")
        if not ok:
            bad.append(label)
    if bad:
        print(f"\nERROR: figures disagree with the published results: {bad}", file=sys.stderr)
        return 1

    for f in sorted(args.output_dir.glob("*.png")):
        print(f"  wrote {f}  ({f.stat().st_size / 1024:.0f} kB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
