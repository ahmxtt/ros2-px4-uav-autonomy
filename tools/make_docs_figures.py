#!/usr/bin/env python3
"""Regenerate the two computed documentation figures from public code and data.

The figures are committed under ``docs/figures/`` because a README must render
on GitHub without the reader running anything. This script is how they are
produced, so they are auditable rather than opaque:

1. ``nominal_planning.png`` - the A* path and the refined trajectory on the
   execution prior map, with the clearance profile. Regenerated from the SDF.
2. ``srr_geometry.png`` - the Stop-Replan-Resume detour, sensed obstacle and
   rejoin, with the refined detour's speed and lateral acceleration,
   reproduced offline from the published result bundle.

Both are derived from the same computations the reproduction tools run, so a
figure can never drift away from the published numbers without those tools
failing first.

The architecture overview, ``architecture.png``, is drawn, not computed. Its
source is ``docs/figures/architecture.drawio``; edit it in draw.io and export
with draw.io Desktop::

    draw.io -x -f png -s 2 -b 10 -o docs/figures/architecture.png \
        docs/figures/architecture.drawio

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
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
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

# Same palette as docs/figures/architecture.drawio: green is the offline
# result, blue the Stop-Replan-Resume result, grey an intermediate or a
# reference, orange what the vehicle sensed.
INK = "#1A1A1A"
MUTED = "#6B7280"
GREEN = "#2E7D32"
BLUE = "#2F4FC9"
ORANGE = "#C0501F"
LIMIT = "#B91C1C"     # hard limit (r_eff, a_lat_max)
PREF = "#B45309"      # soft target (r_pref)
MAP_CMAP = LinearSegmentedColormap.from_list("prior_map", ["#242A33", "#FFFFFF"])
FIGSIZE = (13.5, 6.0)   # both figures share it, so they line up in the README

plt.rcParams.update({
    "font.size": 10.5, "axes.titlesize": 12.5, "axes.titleweight": "bold",
    "axes.titlelocation": "left", "axes.titlepad": 10, "axes.labelsize": 10.5,
    "axes.edgecolor": "#9CA3AF", "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": "#4B5563", "ytick.color": "#4B5563",
    "legend.fontsize": 9.5, "legend.frameon": False,
    "axes.spines.top": False, "axes.spines.right": False,
})


def _prior_map(ax, image, extent) -> None:
    ax.imshow(image, cmap=MAP_CMAP, vmin=0, vmax=255, origin="upper",
              extent=extent, interpolation="nearest")
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_edgecolor("#4B5563")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m], world ENU")
    ax.set_ylabel("y [m], world ENU")


def _hline(ax, y, color, text, ls="--") -> None:
    """Reference line labelled at its right end instead of in a legend."""
    ax.axhline(y, ls=ls, lw=1.2, color=color, zorder=1)
    ax.annotate(text, xy=(1.0, y), xycoords=("axes fraction", "data"),
                xytext=(4, 0), textcoords="offset points", va="center",
                fontsize=9.5, color=color)


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

    fig = plt.figure(figsize=FIGSIZE)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.25], wspace=0.28)

    ax = fig.add_subplot(gs[0, 0])
    ox, oy = float(origin[0]), float(origin[1])
    _prior_map(ax, image, [ox, ox + geom.width_cells * res, oy, oy + geom.height_cells * res])
    ax.plot(xy[:, 0], xy[:, 1], lw=1.2, ls="--", color=MUTED,
            label=f"A* path, {P.path_length_metres(a.path_cells, res):.3f} m")
    ax.plot(tr.samples_xy[:, 0], tr.samples_xy[:, 1], lw=2.4, color=GREEN,
            label=f"B-spline trajectory, {tr.trajectory_length_m:.3f} m")
    ax.plot(*start_xy, "o", ms=9, color="white", mec=INK, mew=1.6, label="start")
    ax.plot(*goal_xy, "*", ms=16, color="#FACC15", mec=INK, label="goal")
    pad = 6
    ax.set_xlim(min(xy[:, 0].min(), start_xy[0]) - pad, max(xy[:, 0].max(), goal_xy[0]) + pad)
    ax.set_ylim(min(xy[:, 1].min(), start_xy[1]) - pad, max(xy[:, 1].max(), goal_xy[1]) + pad)
    ax.set_title("Nominal plan on the execution prior map")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2)

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(tr.s, tr.clearance, lw=1.6, color=GREEN)
    _hline(ax2, r_plan_min, MUTED, f"r_plan_min {r_plan_min:.2f} m")
    _hline(ax2, r_pref, PREF, f"r_pref {r_pref:.2f} m")
    _hline(ax2, r_eff, LIMIT, f"r_eff {r_eff:.2f} m")
    i_min = int(np.argmin(tr.clearance))
    ax2.plot(tr.s[i_min], tr.clearance[i_min], "v", ms=9, color=GREEN, mec=INK)
    ax2.annotate(f"minimum {tr.clearance_min_m:.3f} m", xy=(tr.s[i_min], tr.clearance[i_min]),
                 xytext=(-14, 26), textcoords="offset points", ha="right", fontsize=9.5,
                 arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ax2.set_xlim(0, float(tr.s[-1]))
    ax2.set_ylim(0, min(6.0, float(tr.clearance.max()) * 1.05))
    ax2.set_xlabel("arc length s [m]")
    ax2.set_ylabel("clearance to the nearest obstacle [m]")
    ax2.set_title(f"Clearance along the trajectory: {tr.classification}")
    ax2.grid(axis="y", alpha=0.25)

    fig.savefig(path, **SAVE)
    plt.close(fig)
    return tr


# ---------------------------------------------------------------- figure 2

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

    fig = plt.figure(figsize=FIGSIZE)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.25], wspace=0.28, hspace=0.35)

    ax = fig.add_subplot(gs[:, 0])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    _prior_map(ax, image, [ox, ox + geom.width_cells * geom.resolution,
                           oy, oy + geom.height_cells * geom.resolution])
    ax.plot(xs, ys, lw=1.4, ls="--", color=GREEN, label="original mission reference")
    ax.plot(cand.astar_xy[:, 0], cand.astar_xy[:, 1], lw=1.1, color=MUTED,
            label=f"local A* detour, {cand.astar_path_length_m:.2f} m")
    ax.plot(cand.smoothed_xy[:, 0], cand.smoothed_xy[:, 1], lw=2.4, color=BLUE,
            label=f"refined detour, {cand.smoothed_length_m:.2f} m")
    ax.add_patch(Circle(obs_xy, obs_r, fc="#FDBA74", ec=ORANGE, lw=1.4, alpha=0.75,
                        label=f"sensed obstacle, r = {obs_r:.2f} m"))
    ax.add_patch(Circle(tuple(gt["centroid_world_xy"]), float(gt["radius_m"]),
                        fc="none", ec=INK, ls=":", lw=1.5,
                        label=f"ground truth (eval. only), r = {gt['radius_m']:.1f} m"))
    ax.plot(*hover_xy, "o", ms=9, color="white", mec=INK, mew=1.6, label="stop / hover")
    ax.plot(*resume_xy, "*", ms=16, color="#FACC15", mec=INK, label="resume anchor")
    pad = 6
    allx = np.concatenate([cand.smoothed_xy[:, 0], [obs_xy[0], hover_xy[0], resume_xy[0]]])
    ally = np.concatenate([cand.smoothed_xy[:, 1], [obs_xy[1], hover_xy[1], resume_xy[1]]])
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_title("Stop–Replan–Resume detour")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2)

    s_d = cand.smoothed_s - cand.smoothed_s[0]
    sp = R.SPEED_PARAMS
    axv = fig.add_subplot(gs[0, 1])
    axv.plot(s_d, cand.smoothed_v, lw=1.6, color=BLUE)
    _hline(axv, sp.v_nominal, MUTED, f"v_nominal {sp.v_nominal:.1f} m/s")
    axv.set_ylim(0, sp.v_nominal * 1.25)
    axv.set_xlim(0, float(s_d[-1]))
    axv.set_ylabel("speed [m/s]")
    axv.set_title(f"Refined detour profile: {cand.classification}, "
                  f"clearance {cand.clearance_min_m:.3f} m")
    axv.grid(axis="y", alpha=0.25)
    axa = fig.add_subplot(gs[1, 1], sharex=axv)
    axa.plot(s_d, np.abs(cand.smoothed_lat_accel), lw=1.6, color=BLUE)
    _hline(axa, sp.a_lat_max, LIMIT, f"a_lat_max {sp.a_lat_max:.1f} m/s²")
    axa.set_ylim(0, sp.a_lat_max * 1.35)
    axa.set_xlabel("arc length along the detour [m]")
    axa.set_ylabel("|lateral acceleration| [m/s²]")
    axa.grid(axis="y", alpha=0.25)

    fig.savefig(path, **SAVE)
    plt.close(fig)
    return cand


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--output-dir", type=Path, default=OUT)
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    print("1/2 nominal planning (regenerating map + plan + trajectory) ...")
    tr = figure_nominal(args.output_dir / "nominal_planning.png")
    print(f"    trajectory {tr.trajectory_length_m:.6f} m, "
          f"clearance {tr.clearance_min_m:.6f} m, {tr.classification}")
    print("2/2 stop-replan-resume geometry ...")
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
        ("SRR max lateral accel [m/s^2]", c.max_lateral_accel_mps2, pub_s["max_lateral_accel_mps2"]),
        ("SRR max curvature [1/m]", c.max_curvature_1_per_m, pub_s["max_curvature_1_per_m"]),
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
