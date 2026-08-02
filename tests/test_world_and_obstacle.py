"""The simulation world and the Stop-Replan-Resume obstacle.

Two separate concerns, tested together because they share the world file.

The world needs four environment declarations that Gazebo does not supply by
default. Without them PX4 has no barometer and no compass, its height estimate
random-walks by metres on the ground, and an armed run cannot settle. They are
easy to lose in a merge and the symptom points nowhere near the cause, so they
are pinned here, together with proof that adding them did not move the
occupancy grid.

The obstacle has to be described identically in two places, and must not be in
the world file at all: the planner searches a prior map rasterised from that
file, and an obstacle already in the map is not novel.
"""

from __future__ import annotations

import hashlib
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from uav_autonomy import mapping as M

REPO = Path(__file__).resolve().parents[1]
WORLD = REPO / "sim" / "worlds" / "forest_map2.sdf"
OBSTACLE_YAML = REPO / "sim" / "obstacles" / "obstacle.yaml"
OBSTACLE_SDF = REPO / "sim" / "obstacles" / "srr_unknown_obstacle.sdf"
SCENARIO = REPO / "sim" / "scenarios" / "forest_map2.yaml"

# The rasterised execution prior, as a hash of the raw image bytes. Adding the
# physics and environment elements to the world must not change this.
EXPECTED_GRID_SHA256 = (
    "608d8a78e990766ef13505ce8a0930ec94769c8e1700ddb6d75fcd0cacc37988")


@pytest.fixture(scope="module")
def world():
    return ET.parse(WORLD).getroot().find("world")


# --------------------------------------------------------------------------
# World environment
# --------------------------------------------------------------------------

def test_world_declares_physics_at_px4s_rate(world):
    physics = world.find("physics")
    assert physics is not None, (
        "no <physics>: Gazebo defaults to 1 ms steps, four times PX4's rate")
    assert physics.findtext("max_step_size") == "0.004"
    assert physics.findtext("real_time_update_rate") == "250"


def test_physics_engine_is_dart_not_ode(world):
    """ODE aborts on this world: a 400 m ground box with 80 mesh trees
    overflows its collision-space integer bounds and the server dies
    mid-flight, taking PX4's accelerometer stream with it."""
    assert world.find("physics").get("type") == "dart"


def test_world_declares_gravity_magnetic_field_and_atmosphere(world):
    assert world.findtext("gravity") == "0 0 -9.8"
    assert world.find("atmosphere") is not None, (
        "no <atmosphere>: the barometer has nothing to sample and PX4 reports "
        "'No valid data from Baro 0'")
    assert world.find("magnetic_field") is not None, (
        "no <magnetic_field>: PX4 reports 'No valid data from Compass 0'")


def test_magnetic_field_matches_the_declared_site(world):
    """A field copied from another location biases the EKF heading, and the
    vehicle then flies away from its hold point instead of towards it.

    Checked against PX4's own world magnetic model at the world's declared
    coordinates: declination 3.119 deg, inclination 66.578 deg, 49.304 uT.
    """
    import math
    east, north, up = (float(v) for v in world.findtext("magnetic_field").split())
    total = math.sqrt(east ** 2 + north ** 2 + up ** 2)
    declination = math.degrees(math.atan2(east, north))
    inclination = math.degrees(math.atan2(-up, math.hypot(east, north)))
    assert total == pytest.approx(49.304e-6, rel=1e-3)
    assert declination == pytest.approx(3.119, abs=0.05)
    assert inclination == pytest.approx(66.578, abs=0.05)

    coords = world.find("spherical_coordinates")
    assert float(coords.findtext("latitude_deg")) == pytest.approx(51.239583)
    assert float(coords.findtext("longitude_deg")) == pytest.approx(7.162194)


def test_the_occupancy_grid_did_not_move():
    """The environment declarations touch no tree pose, so the derived map is
    byte-identical. This is what makes the change safe for every published
    planning result."""
    image, grid, obstacles = M.execution_prior(WORLD)
    assert len(obstacles) == 80
    assert (grid.width_cells, grid.height_cells) == (2913, 3911)
    digest = hashlib.sha256(image.tobytes()).hexdigest()
    assert digest == EXPECTED_GRID_SHA256


def test_scenario_pins_the_current_world_hash():
    pinned = yaml.safe_load(SCENARIO.read_text())["world"]["sha256"]
    actual = hashlib.sha256(WORLD.read_bytes()).hexdigest()
    assert pinned == actual, (
        "sim/scenarios/forest_map2.yaml pins a stale world hash")


# --------------------------------------------------------------------------
# The obstacle
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def obstacle():
    return yaml.safe_load(OBSTACLE_YAML.read_text())


def test_obstacle_definition_has_what_the_spawn_tool_needs(obstacle):
    for key in ("model_name", "position_world_xy_m", "radius_m", "height_m",
                "z_base_world_m", "sdf", "verdict"):
        assert key in obstacle, f"missing {key}"
    assert obstacle["verdict"] == "PASS", (
        "the node refuses a non-PASS verdict, and so does the spawn tool")


def test_obstacle_geometry_matches_the_sdf(obstacle):
    """Two files describe one cylinder. If they disagree, the model that gets
    spawned is not the model that was documented."""
    text = OBSTACLE_SDF.read_text()
    pose = [float(v) for v in
            re.search(r"<pose>([^<]+)</pose>", text).group(1).split()[:3]]
    z_center = obstacle["z_base_world_m"] + 0.5 * obstacle["height_m"]
    assert pose[0] == pytest.approx(obstacle["position_world_xy_m"][0])
    assert pose[1] == pytest.approx(obstacle["position_world_xy_m"][1])
    assert pose[2] == pytest.approx(z_center)

    radius = float(re.search(r"<radius>([^<]+)</radius>", text).group(1))
    length = float(re.search(r"<length>([^<]+)</length>", text).group(1))
    assert radius == pytest.approx(obstacle["radius_m"])
    assert length == pytest.approx(obstacle["height_m"])
    assert f'name="{obstacle["model_name"]}"' in text
    assert "<static>true</static>" in text


def test_obstacle_sdf_parses():
    root = ET.parse(OBSTACLE_SDF).getroot()
    model = root.find("model")
    assert model is not None and model.get("name") == "srr_unknown_obstacle"
    assert model.find("link/collision") is not None, (
        "no collision geometry: the LiDAR would see nothing")


def test_obstacle_spans_the_reference_altitude(obstacle):
    lo, hi = obstacle["altitude_band_m"]
    assert lo <= 2.0 <= hi
    assert obstacle["z_base_world_m"] + obstacle["height_m"] >= 2.0


def test_obstacle_is_absent_from_the_world_file():
    """The prior map is rasterised from the world. An obstacle already in it is
    not novel, and the whole scenario collapses."""
    text = WORLD.read_text()
    assert "srr_unknown_obstacle" not in text
    assert "cylinder" not in text.lower()


def test_obstacle_is_not_in_the_rasterised_prior(obstacle):
    """Stronger than the text check: nothing occupies its cell."""
    image, grid, _ = M.execution_prior(WORLD)
    ox, oy = obstacle["position_world_xy_m"]
    col = int((ox - grid.origin_x) / grid.resolution)
    row = int((grid.y_max - oy) / grid.resolution)
    from uav_autonomy.grid import PGM_FREE
    assert image[row, col] == PGM_FREE


def test_spawn_tool_rejects_a_pose_disagreement(tmp_path):
    """Editing one file and not the other must not silently spawn the wrong
    model. Tested without Gazebo: the check runs before any service call."""
    import subprocess
    import sys
    bad_yaml = tmp_path / "obstacle.yaml"
    data = yaml.safe_load(OBSTACLE_YAML.read_text())
    data["position_world_xy_m"] = [0.0, 0.0]
    bad_yaml.write_text(yaml.safe_dump(data))
    proc = subprocess.run(
        [sys.executable, str(REPO / "tools" / "spawn_srr_obstacle.py"),
         "--obstacle-yaml", str(bad_yaml), "--out-dir", str(tmp_path)],
        capture_output=True, text=True)
    assert proc.returncode != 0
    assert "pose disagreement" in proc.stderr


def test_spawn_tool_rejects_a_failed_verdict(tmp_path):
    import subprocess
    import sys
    bad_yaml = tmp_path / "obstacle.yaml"
    data = yaml.safe_load(OBSTACLE_YAML.read_text())
    data["verdict"] = "FAIL"
    bad_yaml.write_text(yaml.safe_dump(data))
    proc = subprocess.run(
        [sys.executable, str(REPO / "tools" / "spawn_srr_obstacle.py"),
         "--obstacle-yaml", str(bad_yaml), "--out-dir", str(tmp_path)],
        capture_output=True, text=True)
    assert proc.returncode != 0
    assert "not 'PASS'" in proc.stderr
