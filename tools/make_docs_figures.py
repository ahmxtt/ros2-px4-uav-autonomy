#!/usr/bin/env python3
"""Regenerate the two computed documentation figures from public code and data.

The figures are committed under ``docs/figures/`` because a README must render
on GitHub without the reader running anything. This script is how they are
produced, so they are auditable rather than opaque:

1. ``nominal_planning.png`` - the A* path and the refined trajectory on the
   execution prior map, with the clearance profile. Regenerated from the SDF.
2. ``srr_geometry.png`` - the Stop-Replan-Resume detour, sensed obstacle and
   rejoin, reproduced offline from the published result bundle.

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
