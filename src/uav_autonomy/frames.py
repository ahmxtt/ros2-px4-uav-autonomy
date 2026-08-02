"""World-ENU <-> PX4-local-NED transformation.

The planning stack works in the Gazebo world frame (ENU: +x east, +y north,
+z up). PX4 Offboard setpoints are expressed in the vehicle's local NED frame
(+x north, +y east, +z down), whose origin is the EKF origin — in this project
the vehicle spawn pose.

With ``(X0, Y0)`` the spawn position in world coordinates and a constant
reference altitude ``z_ref`` above ground::

    x_ned =  y_world - Y0          (north)
    y_ned =  x_world - X0          (east)
    z_ned = -z_ref                 (down, so level flight is a negative constant)
    yaw_ned = wrap_pi(pi/2 - yaw_world)

The velocity mapping follows from differentiating the position mapping along
the path tangent::

    d/dt x_ned = d/dt y_world = v * sin(yaw_world)
    d/dt y_ned = d/dt x_world = v * cos(yaw_world)

The yaw relation is an involution: applying it twice returns the input, so the
same expression serves both directions.

``Z_REF_M``, ``X_ORIGIN_WORLD`` and ``Y_ORIGIN_WORLD`` are the constants of the
evaluated mission. They are exposed as module defaults but every function takes
them as arguments, so the transform is not tied to one scenario.
"""

from __future__ import annotations

import math
from typing import Tuple, Union

import numpy as np

__all__ = [
    "X_ORIGIN_WORLD", "Y_ORIGIN_WORLD", "Z_REF_M",
    "wrap_pi", "world_to_ned", "ned_to_world",
    "yaw_world_to_ned", "yaw_ned_to_world", "apply_enu_to_ned",
]

# Constants of the evaluated mission (vehicle spawn pose / EKF origin).
X_ORIGIN_WORLD = -50.0
Y_ORIGIN_WORLD = -70.0
Z_REF_M = 2.0

ArrayLike = Union[np.ndarray, float]


def wrap_pi(a: ArrayLike) -> ArrayLike:
    """Wrap an angle (or array of angles) to ``[-pi, pi]``.

    Implemented as ``atan2(sin a, cos a)``. At the branch cut the sign follows
    atan2, so an input of exactly +/-pi may come back as either; callers must
    compare wrapped angles by their circular difference, not by equality.
    """
    if isinstance(a, np.ndarray):
        return np.arctan2(np.sin(a), np.cos(a))
    return math.atan2(math.sin(a), math.cos(a))


def world_to_ned(
    xw: float,
    yw: float,
    x_origin: float = X_ORIGIN_WORLD,
    y_origin: float = Y_ORIGIN_WORLD,
) -> Tuple[float, float]:
    """World ENU (x, y) -> PX4 local NED (north, east)."""
    return (yw - y_origin, xw - x_origin)


def ned_to_world(
    xned: float,
    yned: float,
    x_origin: float = X_ORIGIN_WORLD,
    y_origin: float = Y_ORIGIN_WORLD,
) -> Tuple[float, float]:
    """PX4 local NED (north, east) -> world ENU (x, y)."""
    return (yned + x_origin, xned + y_origin)


def yaw_world_to_ned(yaw_world: float) -> float:
    """World-ENU heading -> NED heading, wrapped to ``(-pi, pi]``."""
    return wrap_pi((math.pi / 2.0) - yaw_world)


def yaw_ned_to_world(yaw_ned: float) -> float:
    """NED heading -> world-ENU heading. Same expression as the forward map:
    ``yaw -> pi/2 - yaw`` is its own inverse."""
    return wrap_pi((math.pi / 2.0) - yaw_ned)


def apply_enu_to_ned(
    *,
    x_world: np.ndarray,
    y_world: np.ndarray,
    yaw_world: np.ndarray,
    v_world: np.ndarray,
    origin_xy_world: Tuple[float, float],
    z_ref_m: float,
) -> dict:
    """Vectorised transform of a full world-frame reference into NED.

    Returns position, yaw and velocity components ready to be streamed as PX4
    ``TrajectorySetpoint`` values. ``vz_ned`` is zero: the evaluated mission
    flies at a constant reference altitude.
    """
    ox, oy = origin_xy_world
    x_ned = y_world - oy
    y_ned = x_world - ox
    z_ned = np.full_like(x_world, -z_ref_m)
    yaw_ned = wrap_pi(math.pi / 2.0 - yaw_world)
    vx_ned = v_world * np.sin(yaw_world)
    vy_ned = v_world * np.cos(yaw_world)
    vz_ned = np.zeros_like(x_world)
    return dict(
        x_ned=x_ned, y_ned=y_ned, z_ned=z_ned, yaw_ned=yaw_ned,
        vx_ned=vx_ned, vy_ned=vy_ned, vz_ned=vz_ned,
    )
