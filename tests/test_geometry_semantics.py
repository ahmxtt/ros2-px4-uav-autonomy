"""What the vehicle radii mean, and what they must not be claimed to mean.

The numbers themselves are frozen: every published planning result depends on
r_eff = 0.65 m. What these tests protect is their *interpretation*, which has
been wrong in this repository before.

The airframe is a Holybro X650: 650 mm wheelbase (diagonal motor to motor) and
15 inch propellers. From those two published figures alone:

    frame radius, motor hub to centre    0.650 / 2         = 0.3250 m
    propeller radius                     15 * 0.0254 / 2   = 0.1905 m
    propeller-tip envelope radius        sum               = 0.5155 m

0.5155 m is the smallest circle that actually contains the turning aircraft.
Any planning radius has to exceed it. r_eff = 0.65 m does, by 0.1345 m. An
earlier nominal radius of 0.50 m does not, which is why it was rejected for
public planning and is not shipped as a configuration.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
PLANNING = REPO / "sim" / "config" / "robot_map2_safe.yaml"
POLICY = REPO / "sim" / "config" / "mission_policy.yaml"

WHEELBASE_M = 0.650
PROP_INCHES = 15.0
FRAME_RADIUS_M = WHEELBASE_M / 2.0                    # 0.3250
PROP_RADIUS_M = PROP_INCHES * 0.0254 / 2.0            # 0.1905
PROP_TIP_ENVELOPE_M = FRAME_RADIUS_M + PROP_RADIUS_M  # 0.5155


def load(path):
    return yaml.safe_load(path.read_text())


def test_derived_airframe_geometry_is_what_the_docs_say():
    assert FRAME_RADIUS_M == pytest.approx(0.3250)
    assert PROP_RADIUS_M == pytest.approx(0.1905)
    assert PROP_TIP_ENVELOPE_M == pytest.approx(0.5155)


def test_planning_radius_encloses_the_propeller_tips():
    r_eff = float(load(PLANNING)["robot_radius"])
    assert r_eff == 0.65
    margin = r_eff - PROP_TIP_ENVELOPE_M
    assert margin == pytest.approx(0.1345, abs=1e-4)
    assert margin > 0, "the planning radius does not contain the aircraft"


def test_the_rejected_nominal_radius_is_not_shipped():
    """0.50 m is smaller than the 0.5155 m propeller-tip envelope, so it is not
    a conservative collision radius. It was rejected for public planning and
    the file that carried it is not part of the release. The arithmetic is kept
    here so the reason survives even though the file does not."""
    assert 0.50 < PROP_TIP_ENVELOPE_M
    assert not (REPO / "sim" / "config" / "robot.yaml").exists(), (
        "the rejected nominal configuration is back in the release")
    shipped = sorted(p.name for p in (REPO / "sim" / "config").glob("*.yaml"))
    assert shipped == ["mission_policy.yaml", "robot_map2_safe.yaml",
                       "trajectory_feasibility.yaml"], shipped


def test_circumscribed_not_inscribed_is_the_conservative_bound():
    """The old invariant compared robot_radius against the inscribed circle of
    the square footprint. For a square the inscribed circle is the
    NON-conservative bound: satisfying it says nothing about whether the
    aircraft fits."""
    half = 0.46
    inscribed = half
    circumscribed = math.hypot(half, half)
    assert circumscribed == pytest.approx(0.650538, abs=1e-6)
    assert circumscribed > inscribed

    r_eff = float(load(PLANNING)["robot_radius"])
    # The rejected 0.50 m value satisfied the weak bound and failed the strong
    # one, which is exactly why the weak bound was the wrong test to apply.
    assert 0.50 >= inscribed
    assert 0.50 < circumscribed
    # r_eff agrees with the circumscribed radius to within 0.54 mm. That is a
    # coincidence of rounding, not corroboration: 0.46 is itself 0.65/sqrt(2).
    assert abs(r_eff - circumscribed) < 6e-4


def test_the_footprint_is_the_documented_square():
    fp = load(PLANNING)["footprint"]
    assert sorted(tuple(p) for p in fp) == sorted(
        [(-0.46, -0.46), (-0.46, 0.46), (0.46, -0.46), (0.46, 0.46)])


def test_no_config_claims_a_measurement_that_does_not_exist():
    """No measurement of the 0.46 m half-extent exists anywhere. Wording that
    calls it measured, or calls it the arm tip, is a false provenance claim."""
    text = PLANNING.read_text().lower()
    assert "measured" not in text or "no measurement" in text
    for phrase in ("arm tip plus", "arm-tip half-extent",
                   "measured platform value"):
        assert phrase not in text, f"{PLANNING.name} still says {phrase!r}"


def test_the_inscribed_circle_invariant_is_gone():
    assert "INVARIANT: robot_radius >= inscribed" not in PLANNING.read_text()


def test_the_clearance_policy_derives_the_documented_radii():
    """mission_policy.yaml stores margins, not radii. The published values are
    what those margins produce when applied to r_eff, so the chain is what has
    to hold, not four literals."""
    policy = load(POLICY)["planning_policy"]
    r_eff = float(load(PLANNING)["robot_radius"])

    r_pref = r_eff + float(policy["tracking_margin_m"])
    r_plan_min = r_pref + float(policy["spline_shrinkage_budget_m"])
    r_comfort = r_pref + float(policy["comfort_margin_m"])

    assert r_eff == pytest.approx(0.65)
    assert r_pref == pytest.approx(0.90)
    assert r_plan_min == pytest.approx(1.10)
    assert r_comfort == pytest.approx(1.20)

    assert r_eff < r_pref < r_plan_min < r_comfort
    assert r_plan_min - r_pref == pytest.approx(0.20), "spline budget changed"
    # A comfort band narrower than the spline budget would collapse to zero
    # width and the comfort cost would never fire.
    assert r_comfort > r_plan_min


PLANNING_TOOLS = (
    "tools/plan_astar.py",
    "tools/reduce_path_los.py",
    "tools/refine_trajectory.py",
    "tools/reproduce_offline_demo.py",
    "tools/make_docs_figures.py",
    "tests/test_regression_published_result.py",
)


def test_every_planning_tool_reads_the_active_config():
    """The published numbers depend on r_eff coming from one file. A tool that
    quietly sourced a different radius would move the results without moving
    any test."""
    missing = [rel for rel in PLANNING_TOOLS
               if "robot_map2_safe.yaml" not in (REPO / rel).read_text()]
    assert missing == [], f"these do not read the planning config: {missing}"


def test_no_code_reads_a_robot_config_other_than_the_active_one():
    hits = []
    for tree in ("src", "tools", "tests", "ros2"):
        for py in (REPO / tree).rglob("*.py"):
            if py.resolve() == Path(__file__).resolve():
                continue
            text = py.read_text()
            if "robot.yaml" in text and "robot_map2_safe.yaml" not in text:
                hits.append(str(py.relative_to(REPO)))
    assert hits == [], f"these reference a non-shipped robot config: {hits}"
