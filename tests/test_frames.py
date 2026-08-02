"""ENU <-> NED transformation.

A sign or axis-swap error here is the classic PX4 Offboard failure: the
vehicle flies a mirrored or rotated version of the intended path while every
log looks internally consistent. The tests pin the axis mapping explicitly,
not only the round trip -- a round trip alone would also pass if both
directions were wrong in the same way.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uav_autonomy.frames import (
    X_ORIGIN_WORLD, Y_ORIGIN_WORLD, Z_REF_M,
    apply_enu_to_ned, ned_to_world, world_to_ned, wrap_pi,
    yaw_ned_to_world, yaw_world_to_ned,
)


def test_axis_mapping_is_explicit():
    """north = y_world - Y0, east = x_world - X0."""
    n, e = world_to_ned(10.0, 20.0, x_origin=1.0, y_origin=2.0)
    assert n == pytest.approx(18.0)  # from y
    assert e == pytest.approx(9.0)   # from x


def test_origin_maps_to_zero():
    n, e = world_to_ned(X_ORIGIN_WORLD, Y_ORIGIN_WORLD)
    assert n == pytest.approx(0.0)
    assert e == pytest.approx(0.0)


def test_position_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(5000):
        xw, yw = rng.uniform(-500.0, 500.0, 2)
        xb, yb = ned_to_world(*world_to_ned(xw, yw))
        assert xb == pytest.approx(xw, abs=1e-12)
        assert yb == pytest.approx(yw, abs=1e-12)


def test_position_roundtrip_with_custom_origin():
    xb, yb = ned_to_world(*world_to_ned(3.0, -7.0, 11.0, -13.0), 11.0, -13.0)
    assert xb == pytest.approx(3.0)
    assert yb == pytest.approx(-7.0)


def test_yaw_mapping_at_cardinal_headings():
    """World heading +x (east) is NED heading +pi/2; world +y (north) is 0."""
    assert yaw_world_to_ned(0.0) == pytest.approx(math.pi / 2)
    assert yaw_world_to_ned(math.pi / 2) == pytest.approx(0.0)
    assert yaw_world_to_ned(-math.pi / 2) == pytest.approx(math.pi)


def test_yaw_roundtrip_across_the_wrap():
    rng = np.random.default_rng(1)
    for yw in np.concatenate([
        rng.uniform(-math.pi, math.pi, 5000),
        np.array([-math.pi, math.pi, 0.0, math.pi - 1e-12, -math.pi + 1e-12]),
    ]):
        back = yaw_ned_to_world(yaw_world_to_ned(float(yw)))
        diff = abs((back - yw + math.pi) % (2 * math.pi) - math.pi)
        assert diff < 1e-12, f"yaw roundtrip failed at {yw}: {back}"


def test_yaw_output_is_always_wrapped():
    rng = np.random.default_rng(2)
    for yw in rng.uniform(-20.0, 20.0, 2000):
        out = yaw_world_to_ned(float(yw))
        assert -math.pi - 1e-12 <= out <= math.pi + 1e-12


def _circ_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def test_wrap_pi_scalar_and_array():
    # +/-pi sit on the branch cut, so compare on the circle.
    assert _circ_diff(wrap_pi(3 * math.pi), math.pi) < 1e-12
    assert _circ_diff(wrap_pi(-3 * math.pi), math.pi) < 1e-12
    assert wrap_pi(0.5) == pytest.approx(0.5)
    assert wrap_pi(0.5 + 2 * math.pi) == pytest.approx(0.5)
    arr = wrap_pi(np.array([0.0, 2 * math.pi, -2 * math.pi, 0.25]))
    assert np.allclose(arr, [0.0, 0.0, 0.0, 0.25], atol=1e-12)
    assert np.all(np.abs(wrap_pi(np.array([5 * math.pi]))) == pytest.approx(math.pi))


def test_vectorised_transform_matches_the_scalar_one():
    rng = np.random.default_rng(3)
    xw = rng.uniform(-100, 100, 200)
    yw = rng.uniform(-100, 100, 200)
    yaw = rng.uniform(-math.pi, math.pi, 200)
    v = rng.uniform(0.0, 2.0, 200)
    out = apply_enu_to_ned(
        x_world=xw, y_world=yw, yaw_world=yaw, v_world=v,
        origin_xy_world=(X_ORIGIN_WORLD, Y_ORIGIN_WORLD), z_ref_m=Z_REF_M,
    )
    for i in range(0, 200, 17):
        n, e = world_to_ned(float(xw[i]), float(yw[i]))
        assert out["x_ned"][i] == pytest.approx(n)
        assert out["y_ned"][i] == pytest.approx(e)
        assert out["yaw_ned"][i] == pytest.approx(yaw_world_to_ned(float(yaw[i])))


def test_level_flight_is_a_negative_constant_in_ned():
    out = apply_enu_to_ned(
        x_world=np.zeros(4), y_world=np.zeros(4),
        yaw_world=np.zeros(4), v_world=np.ones(4),
        origin_xy_world=(0.0, 0.0), z_ref_m=2.0,
    )
    assert np.allclose(out["z_ned"], -2.0)
    assert np.allclose(out["vz_ned"], 0.0)


def test_velocity_is_consistent_with_the_position_mapping():
    """Integrating the NED velocity must match transforming the moved point."""
    dt = 1e-6
    for yaw in (0.0, 0.4, math.pi / 2, -2.0, math.pi):
        speed = 1.5
        x0, y0 = 3.0, -4.0
        x1 = x0 + speed * math.cos(yaw) * dt
        y1 = y0 + speed * math.sin(yaw) * dt

        out = apply_enu_to_ned(
            x_world=np.array([x0]), y_world=np.array([y0]),
            yaw_world=np.array([yaw]), v_world=np.array([speed]),
            origin_xy_world=(0.0, 0.0), z_ref_m=2.0,
        )
        n0, e0 = world_to_ned(x0, y0, 0.0, 0.0)
        n1, e1 = world_to_ned(x1, y1, 0.0, 0.0)
        assert out["vx_ned"][0] == pytest.approx((n1 - n0) / dt, abs=1e-6)
        assert out["vy_ned"][0] == pytest.approx((e1 - e0) / dt, abs=1e-6)


def test_speed_magnitude_is_preserved():
    rng = np.random.default_rng(4)
    yaw = rng.uniform(-math.pi, math.pi, 500)
    v = rng.uniform(0.0, 3.0, 500)
    out = apply_enu_to_ned(
        x_world=np.zeros(500), y_world=np.zeros(500),
        yaw_world=yaw, v_world=v,
        origin_xy_world=(0.0, 0.0), z_ref_m=2.0,
    )
    mag = np.hypot(out["vx_ned"], out["vy_ned"])
    assert np.allclose(mag, v, atol=1e-12)
