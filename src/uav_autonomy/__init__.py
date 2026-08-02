"""Offline planning, trajectory refinement and local replanning for a
ROS 2 / PX4 UAV.

The package holds the algorithmic core only: no ROS, no PX4, no Gazebo. Every
module here runs from a plain Python environment, which is what makes the
planning pipeline testable and reproducible without a simulator.

Layers, in pipeline order::

    mapping      SDF world       -> occupancy grid (PGM + map.yaml)
    grid         occupancy grid  -> inflation, world/cell mapping, clearance
    planning     grid + endpoints-> deterministic A* path (cells)
    smoothing    A* path         -> line-of-sight reduced polyline
    trajectory   path            -> cubic B-spline + speed profile
    replanning   sensed obstacle -> local ROI detour + rejoin
    frames       world ENU       <-> PX4 local NED
    artifacts    any stage       -> CSV / YAML / SHA-256 bundle

The ROS 2 runtime nodes that consume this package live under ``ros2/nodes/``
and are intentionally kept outside it.
"""

__version__ = "0.1.0"

__all__ = [
    "artifacts", "frames", "grid", "mapping", "planning", "replanning",
    "smoothing", "trajectory", "__version__",
]
