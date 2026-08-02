"""Occupancy-grid representation, world/cell mapping, inflation, clearance.

Shared substrate for the planning, smoothing and replanning layers. Two
grid encodings coexist and are deliberately kept distinct:

``classify_map`` -> semantic grid
    ``FREE`` / ``OCCUPIED`` / ``UNKNOWN`` derived from a ROS ``map.yaml``
    (``negate``, ``occupied_thresh``, ``free_thresh``). Used by the planner
    for inflation and by the clearance metric.

``MapData.image`` -> raw PGM
    The unmodified PGM byte image (``254`` free, ``0`` occupied). Used by the
    trajectory layer, which evaluates clearance on dense sub-cell samples via
    a distance transform rather than on discrete cells.

Cell indexing convention (identical in both encodings)::

    col = floor((x - origin_x) / resolution)
    row = height - 1 - floor((y - origin_y) / resolution)

that is: columns increase with +x, rows increase as y decreases (PGM stores
the top row first while ``origin`` refers to the bottom-left corner).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt

__all__ = [
    "FREE", "OCCUPIED", "UNKNOWN", "PGM_FREE", "PGM_OCCUPIED",
    "Cell", "MapData",
    "classify_map", "disk_kernel", "inflated_clear_grid",
    "world_to_cell", "cell_to_world", "in_bounds", "collect_goal_cells",
    "load_map_yaml", "load_map", "load_full_pgm",
    "clearance_field", "compute_clearance_metres", "sample_clearances",
]

# Semantic grid values (classify_map output).
FREE = 0
OCCUPIED = 1
UNKNOWN = 2

# Raw PGM byte convention used by the map generator and the trajectory layer.
PGM_FREE = 254
PGM_OCCUPIED = 0

Cell = Tuple[int, int]


# ---------------------------------------------------------------------------
# Map container
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MapData:
    """Raw PGM image plus its world georeference.

    ``origin_x`` / ``origin_y`` are the world coordinates of the *bottom-left*
    corner, matching the ROS ``map.yaml`` ``origin`` field.
    """

    image: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    height: int
    width: int

    @property
    def y_max(self) -> float:
        return self.origin_y + self.height * self.resolution

    @property
    def x_max(self) -> float:
        return self.origin_x + self.width * self.resolution


def load_map_yaml(path: Path) -> Dict:
    with Path(path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_map(map_yaml_path: Path) -> MapData:
    """Load a ROS ``map.yaml`` and its PGM into a :class:`MapData`."""
    map_yaml_path = Path(map_yaml_path)
    meta = load_map_yaml(map_yaml_path)
    image_path = (map_yaml_path.parent / meta["image"]).resolve()
    arr = np.array(Image.open(image_path), dtype=np.uint8)
    h, w = arr.shape
    origin = meta["origin"]
    return MapData(
        image=arr,
        resolution=float(meta["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        height=h,
        width=w,
    )


def load_full_pgm(map_yaml_path: Path) -> Tuple[np.ndarray, Dict]:
    """Return ``(pgm_array, map_yaml_dict)`` without wrapping in MapData.

    Used by the replanning layer, which crops the array before georeferencing.
    """
    map_yaml_path = Path(map_yaml_path)
    meta = load_map_yaml(map_yaml_path)
    arr = np.array(Image.open(map_yaml_path.parent / meta["image"]), dtype=np.uint8)
    return arr, meta


# ---------------------------------------------------------------------------
# Semantic classification and inflation
# ---------------------------------------------------------------------------

def classify_map(
    image: np.ndarray,
    negate: int,
    occupied_thresh: float,
    free_thresh: float,
) -> np.ndarray:
    """Map a grayscale image to FREE / OCCUPIED / UNKNOWN via ROS thresholds."""
    if image.ndim != 2:
        raise ValueError(f"expected grayscale map image, got shape={image.shape}")
    pixels = image.astype(np.float32)
    occupancy = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    grid = np.full(image.shape, UNKNOWN, dtype=np.uint8)
    grid[occupancy > occupied_thresh] = OCCUPIED
    grid[occupancy < free_thresh] = FREE
    return grid


def disk_kernel(radius_cells: int) -> np.ndarray:
    y, x = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    return (x * x + y * y) <= radius_cells * radius_cells


def inflated_clear_grid(
    grid: np.ndarray,
    radius_cells: int,
    unknown_is_blocked: bool,
) -> np.ndarray:
    """Hard binary inflation by a disk of ``radius_cells``.

    Returns a boolean array that is True where the robot centre may be placed.
    Padding with ``True`` (blocked) makes the map border behave as an obstacle,
    so a path can never leave the grid.
    """
    blocked = grid == OCCUPIED
    if unknown_is_blocked:
        blocked |= grid == UNKNOWN
    if radius_cells <= 0:
        return ~blocked
    padded = np.pad(blocked, radius_cells, mode="constant", constant_values=True)
    inflated = binary_dilation(padded, structure=disk_kernel(radius_cells))
    return ~inflated[radius_cells:-radius_cells, radius_cells:-radius_cells]


# ---------------------------------------------------------------------------
# World <-> cell
# ---------------------------------------------------------------------------

def world_to_cell(
    point: Tuple[float, float],
    origin: List[float],
    resolution: float,
    height: int,
) -> Cell:
    x, y = point
    col = int(math.floor((x - float(origin[0])) / resolution))
    row_from_bottom = int(math.floor((y - float(origin[1])) / resolution))
    row = height - 1 - row_from_bottom
    return row, col


def cell_to_world(
    cell: Cell,
    origin: List[float],
    resolution: float,
    height: int,
) -> Tuple[float, float]:
    """World coordinate of the *centre* of ``cell``."""
    row, col = cell
    x = (col + 0.5) * resolution + float(origin[0])
    row_from_bottom = height - 1 - row
    y = (row_from_bottom + 0.5) * resolution + float(origin[1])
    return x, y


def in_bounds(cell: Cell, shape: Tuple[int, int]) -> bool:
    r, c = cell
    h, w = shape
    return 0 <= r < h and 0 <= c < w


def collect_goal_cells(
    center: Tuple[float, float],
    radius_m: float,
    origin: List[float],
    resolution: float,
    shape: Tuple[int, int],
    clear_grid: np.ndarray,
) -> Tuple[List[Cell], Cell, int]:
    """Inflation-safe cells inside the goal disc.

    Returns ``(cells, goal_centre_cell, radius_in_cells)``. ``cells`` may be
    empty, which the planner treats as an unreachable goal region.
    """
    height, _ = shape
    radius_cells = int(math.ceil(radius_m / resolution))
    cr, cc = world_to_cell(center, origin, resolution, height)
    cells: List[Cell] = []
    r2 = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc > r2:
                continue
            cell = (cr + dr, cc + dc)
            if in_bounds(cell, shape) and clear_grid[cell]:
                cells.append(cell)
    return cells, (cr, cc), radius_cells


# ---------------------------------------------------------------------------
# Clearance
# ---------------------------------------------------------------------------

def clearance_field(map_data: MapData) -> np.ndarray:
    """Metric distance to the nearest occupied cell, from the raw PGM image.

    Operates on ``PGM_OCCUPIED`` so it measures the true margin to real
    obstacles, not to the inflated proxy the planner searches on.
    """
    occupied = map_data.image == PGM_OCCUPIED
    dist_cells = distance_transform_edt(~occupied)
    return dist_cells.astype(np.float64) * map_data.resolution


def compute_clearance_metres(grid: np.ndarray, resolution_m: float) -> np.ndarray:
    """Metric distance to the nearest ``OCCUPIED`` cell of a *semantic* grid."""
    raw_occ = grid == OCCUPIED
    if not raw_occ.any():
        return np.full(grid.shape, np.inf, dtype=np.float64)
    return distance_transform_edt(~raw_occ).astype(np.float64) * resolution_m


def sample_clearances(
    samples_xy: np.ndarray,
    clearance_m: np.ndarray,
    map_data: MapData,
) -> Tuple[np.ndarray, np.ndarray]:
    """Look up clearance at continuous world points.

    Returns ``(clearances, in_bounds_mask)``. Out-of-bounds samples get
    clearance 0.0 and a False mask entry; callers must treat a partially
    out-of-bounds path as invalid rather than averaging over the valid part.
    """
    h, w = clearance_m.shape
    clearances = np.zeros(len(samples_xy), dtype=np.float64)
    in_bounds_mask = np.zeros(len(samples_xy), dtype=bool)
    for i, (x, y) in enumerate(samples_xy):
        col = int(math.floor((x - map_data.origin_x) / map_data.resolution))
        row = int(math.floor((map_data.y_max - y) / map_data.resolution))
        if 0 <= row < h and 0 <= col < w:
            in_bounds_mask[i] = True
            clearances[i] = clearance_m[row, col]
    return clearances, in_bounds_mask
