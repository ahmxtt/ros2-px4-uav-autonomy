"""Regression against the published planning result.

This is the test that ties the code to the numbers in ``docs/results.md``. It
rebuilds the occupancy grid from the SDF, replans the published leg, refines
it, and compares against ``results/planning/``.

It is the slowest test in the suite (the A* expands ~230k nodes on a
2913 x 3911 grid) and is marked ``slow`` so it can be deselected with
``pytest -m "not slow"`` during development. It is not optional for a release:
if this fails, the published numbers no longer describe the shipped code.

Tolerances are asymmetric on purpose:

* the path is compared **cell for cell** -- exact, no tolerance;
* scalar geometry is compared to 1e-6 m, far below the 0.05 m cell size;
* the trajectory metrics allow 1e-9, which absorbs the summation-order
  differences that BLAS and library versions introduce in the arc-length and
  KD-tree reductions while still catching any real behavioural change.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from uav_autonomy import grid as G
from uav_autonomy import mapping as M
from uav_autonomy import planning as P
from uav_autonomy import trajectory as T

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "results" / "planning"

# The published leg (start -> orbit-1 entry) in the world frame.
START_XY = (-50.0, -70.0)
GOAL_XY = (18.080636, -0.148755)
GOAL_RADIUS_M = 0.3

# Canopy footprints of the execution prior at the 2 m reference altitude.
CANOPY_M = {"Pine Tree": 1.5, "Oak tree": 4.0}

pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def policy():
    robot = yaml.safe_load((REPO / "sim/config/robot_map2_safe.yaml").read_text())
    pol = yaml.safe_load(
        (REPO / "sim/config/mission_policy.yaml").read_text())["planning_policy"]
    r_eff = float(robot["robot_radius"])
    r_pref = r_eff + float(pol["tracking_margin_m"])
    return {
        "r_eff": r_eff,
        "r_pref": r_pref,
        "r_plan_min": r_pref + float(pol["spline_shrinkage_budget_m"]),
        "r_comfort": r_pref + float(pol["comfort_margin_m"]),
        "w_comfort": float(pol["w_comfort"]),
    }


@pytest.fixture(scope="module")
def execution_prior(tmp_path_factory):
    """Rebuild the execution prior map from the SDF."""
    obstacles = M.parse_obstacles(REPO / "sim/worlds/forest_map2.sdf")
    # Grid geometry from the trunk footprints, canopy radii rasterised onto it.
    geom = M.compute_grid(obstacles, M.RESOLUTION_M, M.PADDING_M)
    canopy = [
        M.Obstacle(o.name, o.model_type, o.x, o.y, CANOPY_M[o.model_type])
        for o in obstacles
    ]
    image = M.rasterize(canopy, geom)
    out = tmp_path_factory.mktemp("prior")
    M.write_pgm(image, out / "map.pgm")
    M.write_map_yaml(out / "map.yaml", geom, "map.pgm")
    return out / "map.yaml", geom


@pytest.fixture(scope="module")
def planned(execution_prior, policy):
    map_yaml, _ = execution_prior
    meta = yaml.safe_load(map_yaml.read_text())
    image = np.array(Image.open(map_yaml.parent / meta["image"]).convert("L"))
    res = float(meta["resolution"])
    origin = list(meta["origin"])
    grid = G.classify_map(image, int(meta.get("negate", 0)),
                          float(meta.get("occupied_thresh", 0.65)),
                          float(meta.get("free_thresh", 0.196)))
    height = grid.shape[0]
    clear = G.inflated_clear_grid(
        grid, int(math.ceil(policy["r_plan_min"] / res)), unknown_is_blocked=False)
    field = G.compute_clearance_metres(grid, res)

    start_cell = G.world_to_cell(START_XY, origin, res, height)
    goal_cells, goal_centre, goal_radius_cells = G.collect_goal_cells(
        GOAL_XY, GOAL_RADIUS_M, origin, res, grid.shape, clear)

    result = P.a_star_comfort(
        clear_grid=clear, clearance_m_field=field, start=start_cell,
        goal_cells_set=frozenset(goal_cells), goal_centre=goal_centre,
        goal_radius_cells=goal_radius_cells,
        r_plan_min=policy["r_plan_min"], r_comfort=policy["r_comfort"],
        w_comfort=policy["w_comfort"])
    return {
        "result": result, "grid": grid, "res": res, "origin": origin,
        "height": height, "map_yaml": map_yaml,
    }


@pytest.fixture(scope="module")
def published():
    return yaml.safe_load((RESULTS / "metrics.yaml").read_text())


@pytest.fixture(scope="module")
def published_traj():
    return yaml.safe_load((RESULTS / "trajectory_metrics.yaml").read_text())


# --------------------------------------------------------------------- map

def test_execution_prior_grid_matches_the_published_geometry(execution_prior, published):
    _, geom = execution_prior
    assert [geom.width_cells, geom.height_cells] == [2913, 3911]
    assert geom.resolution == pytest.approx(0.05)
    assert geom.origin_x == pytest.approx(-80.30)
    assert geom.origin_y == pytest.approx(-100.25)


def test_map_rebuild_is_byte_identical(execution_prior):
    """Rebuilding the grid twice must produce identical bytes."""
    map_yaml, geom = execution_prior
    obstacles = M.parse_obstacles(REPO / "sim/worlds/forest_map2.sdf")
    canopy = [M.Obstacle(o.name, o.model_type, o.x, o.y, CANOPY_M[o.model_type])
              for o in obstacles]
    again = M.rasterize(canopy, geom)
    first = np.array(Image.open(map_yaml.parent / "map.pgm"), dtype=np.uint8)
    assert np.array_equal(first, again)


# ------------------------------------------------------------------ plan

def test_astar_succeeds(planned):
    assert planned["result"].success


def test_astar_path_is_cell_for_cell_identical_to_published(planned):
    """Exact comparison. Any deviation means the planner changed behaviour."""
    reference = []
    with (RESULTS / "path.csv").open() as f:
        for row in csv.DictReader(f):
            reference.append((int(row["row"]), int(row["col"])))
    assert list(planned["result"].path_cells) == reference


def test_astar_scalars_match_published(planned, published):
    r = planned["result"]
    length = P.path_length_metres(r.path_cells, planned["res"])
    clr_min, clr_mean = P.path_clearances(r.path_cells, planned["grid"], planned["res"])

    assert r.nodes_expanded == published["nodes_expanded"]
    assert length == pytest.approx(published["path_length_m"], abs=1e-6)
    assert clr_min == pytest.approx(published["clearance_min_m"], abs=1e-9)
    assert clr_mean == pytest.approx(published["clearance_mean_m"], abs=1e-9)
    assert len(r.path_cells) == published["n_path_points"]


def test_astar_clearance_respects_the_planning_radius(planned, published, policy):
    """The path must never come closer than the hard inflation radius."""
    assert published["clearance_min_m"] >= policy["r_plan_min"] - 1e-9


# ------------------------------------------------------------ trajectory

@pytest.fixture(scope="module")
def refined(planned, policy):
    xy = P.path_cells_to_world(
        planned["result"].path_cells, planned["origin"],
        planned["res"], planned["height"])
    map_data = G.load_map(planned["map_yaml"])
    return T.refine_trajectory(
        astar_xy=xy, map_data=map_data,
        clearance_m_field=G.clearance_field(map_data),
        r_eff=policy["r_eff"], r_pref=policy["r_pref"],
        sp=T.SpeedParams(v_nominal=1.5, v_min=0.3, a_lat_max=1.0, a_long_max=1.0),
    )


def test_trajectory_selects_the_published_attempt(refined, published_traj):
    a = refined.selected
    assert a.name == published_traj["selected_attempt"]
    assert a.classification == published_traj["classification"]
    assert a.n_control_points == published_traj["n_control_points"]
    assert a.n_samples == published_traj["n_samples"]


def test_trajectory_scalars_match_published(refined, published_traj):
    a = refined.selected
    for key, mine in [
        ("trajectory_length_m", a.trajectory_length_m),
        ("clearance_min_m", a.clearance_min_m),
        ("clearance_mean_m", a.clearance_mean_m),
        ("max_deviation_from_astar_m", a.max_deviation_from_astar_m),
        ("mean_deviation_from_astar_m", a.mean_deviation_from_astar_m),
        ("curvature_max_1_per_m", a.curvature_max_1_per_m),
    ]:
        assert mine == pytest.approx(published_traj[key], abs=1e-9), key


def test_refined_trajectory_clears_the_preferred_radius(refined, policy):
    assert refined.selected.clearance_min_m >= policy["r_pref"] - 1e-9
    assert refined.selected.collision_free
    assert refined.selected.in_bounds_all


def test_speed_profile_respects_its_limits(refined, published_traj):
    sp = refined.speed_params
    limits = T.check_motion_limits(
        v_ref=refined.v_ref, lateral_accel=refined.lateral_accel,
        segment_ds=np.diff(refined.selected.s), sp=sp)
    assert limits["v_within_nominal"]
    assert limits["a_lat_within_limit"]
    assert limits["a_long_within_limit"]
    ref = published_traj["speed_profile"]
    assert limits["v_max_mps"] == pytest.approx(ref["v_max_observed_mps"], abs=1e-9)
    assert limits["a_lat_max_mps2"] == pytest.approx(
        ref["a_lat_max_observed_mps2"], abs=1e-9)


def test_smoothing_shortens_the_path_but_stays_close_to_it(refined, published):
    """The spline cuts corners, so it is shorter -- but bounded deviation."""
    a = refined.selected
    assert a.trajectory_length_m < published["path_length_m"]
    assert a.max_deviation_from_astar_m < 0.5


# --------------------------------------------------------------- integrity

def test_published_hashes_verify():
    from uav_autonomy.artifacts import verify_hashes
    for bundle in ("planning", "nominal_mission", "stop_replan_resume"):
        h = REPO / "results" / bundle / "hashes.txt"
        assert h.exists(), f"missing {h}"
        assert verify_hashes(h) == [], f"hash mismatch in results/{bundle}"
