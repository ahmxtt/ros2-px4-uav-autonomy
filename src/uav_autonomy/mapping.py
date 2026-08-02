"""Deterministic SDF -> 2D occupancy-grid rasterisation.

Builds the prior occupancy map directly from the Gazebo world description
rather than from a SLAM run, so the planning input is exactly reproducible
from a text file under version control.

Obstacles are ground-projected as filled discs with a per-species footprint
radius. The radii are deliberately conservative constants, not measurements
taken from the tree collision meshes: the mesh geometry lives in third-party
Fuel models that this repository does not vendor (see ``NOTICE.md``), and a
documented constant is reproducible where a mesh-derived value would silently
depend on which model version happens to be installed.

Determinism:

* ``<world>`` children are visited in document order.
* No RNG anywhere in the pipeline.
* The grid origin is snapped *down* to a whole cell multiple, so small edits
  to the world extent shift the origin in discrete steps rather than
  continuously.
* The PGM is written as headerless-metadata P5 bytes.

Identical SDF input therefore yields a byte-identical PGM.

An unknown ``model://`` include raises rather than being skipped: silently
dropping an obstacle would produce a map that is optimistic in exactly the
place where it matters.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from .grid import PGM_FREE, PGM_OCCUPIED

__all__ = [
    "RESOLUTION_M", "PADDING_M", "TREE_FOOTPRINTS_M", "CANOPY_FOOTPRINTS_M",
    "Obstacle", "GridGeometry",
    "parse_obstacles", "compute_grid", "rasterize", "execution_prior",
    "write_pgm", "write_map_yaml", "world_to_pixel", "cell_is_free",
]

RESOLUTION_M = 0.05
PADDING_M = 5.0  # buffer around the obstacle bounding box, both sides

# Conservative ground-projected trunk footprints, in metres.
TREE_FOOTPRINTS_M: Dict[str, float] = {
    "Pine Tree": 0.25,
    "Oak tree": 0.30,
}

# Canopy footprints, in metres: what the tree occupies at the 2.0 m reference
# altitude, where the vehicle actually flies and the 2D LiDAR sweeps. Much
# larger than the trunk, because at 2 m the branches are what you hit.
#
# The grid geometry is still derived from the trunk positions, so the map
# extent is unchanged; only the discs grow. This pair - trunk-derived geometry,
# canopy-sized discs - is the "execution prior", the map every published
# planning result was computed on.
CANOPY_FOOTPRINTS_M: Dict[str, float] = {
    "Pine Tree": 1.5,
    "Oak tree": 4.0,
}

# Models excluded from the occupancy map:
#   green_ground / ground / ground_plane - the world ground plane
#   forstweg_*                           - visual-only track polyline, no collision
EXCLUDED_MODEL_NAMES_LITERAL = {"green_ground", "ground", "ground_plane"}
EXCLUDED_MODEL_NAME_PREFIXES = ("forstweg_",)


@dataclass(frozen=True)
class Obstacle:
    """A 2D ground-projected obstacle."""

    name: str
    model_type: str  # e.g. "Pine Tree", "Oak tree", or "<model>"
    x: float
    y: float
    footprint_radius_m: float


@dataclass(frozen=True)
class GridGeometry:
    origin_x: float
    origin_y: float
    width_cells: int
    height_cells: int
    resolution: float

    @property
    def x_max(self) -> float:
        return self.origin_x + self.width_cells * self.resolution

    @property
    def y_max(self) -> float:
        return self.origin_y + self.height_cells * self.resolution


# ---------------------------------------------------------------------------
# SDF parsing
# ---------------------------------------------------------------------------

def _parse_pose(pose_text: str) -> Tuple[float, float, float]:
    """Parse an SDF ``<pose>`` ``x y z roll pitch yaw`` into ``(x, y, yaw)``."""
    parts = pose_text.strip().split()
    if len(parts) < 6:
        raise ValueError(f"malformed <pose>: {pose_text!r}")
    return float(parts[0]), float(parts[1]), float(parts[5])


def _model_type_from_uri(uri: str) -> str:
    """``'model://Pine Tree'`` -> ``'Pine Tree'``."""
    if uri.startswith("model://"):
        return uri[len("model://"):].strip()
    return uri.strip()


def _is_excluded(name: str) -> bool:
    if name in EXCLUDED_MODEL_NAMES_LITERAL:
        return True
    return any(name.startswith(p) for p in EXCLUDED_MODEL_NAME_PREFIXES)


def _max_collision_half_extent(model_el: ET.Element) -> float:
    """Largest box/cylinder half-extent of an in-place model. 0 if unusable."""
    best = 0.0
    for box in model_el.findall(".//collision/geometry/box/size"):
        parts = (box.text or "").split()
        if len(parts) >= 2:
            best = max(best, max(float(parts[0]) * 0.5, float(parts[1]) * 0.5))
    for cyl in model_el.findall(".//collision/geometry/cylinder/radius"):
        try:
            best = max(best, float((cyl.text or "0").strip()))
        except ValueError:
            pass
    return best


def parse_obstacles(sdf_path: Path) -> List[Obstacle]:
    """Parse an SDF world and return its obstacles in document order."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("no <world> element in SDF")

    obstacles: List[Obstacle] = []

    for child in list(world):
        tag = child.tag
        if tag == "include":
            uri_el = child.find("uri")
            name_el = child.find("name")
            pose_el = child.find("pose")
            if uri_el is None or pose_el is None:
                continue
            model_type = _model_type_from_uri(uri_el.text or "")
            name = (name_el.text or model_type).strip()
            if _is_excluded(name):
                continue
            radius = TREE_FOOTPRINTS_M.get(model_type)
            if radius is None:
                raise RuntimeError(
                    f"unknown model type {model_type!r} for include named {name!r}; "
                    "add a conservative footprint to TREE_FOOTPRINTS_M and document it."
                )
            x, y, _yaw = _parse_pose(pose_el.text or "0 0 0 0 0 0")
            obstacles.append(
                Obstacle(name=name, model_type=model_type, x=x, y=y,
                         footprint_radius_m=radius)
            )
        elif tag == "model":
            name_attr = child.attrib.get("name", "").strip()
            if _is_excluded(name_attr):
                continue
            # An in-place <model> must declare <collision> to block. Visual-only
            # models are already filtered above; anything left without collision
            # is non-blocking.
            if child.find(".//collision") is None:
                continue
            pose_el = child.find("pose")
            x, y, _yaw = (
                (0.0, 0.0, 0.0) if pose_el is None
                else _parse_pose(pose_el.text or "0 0 0 0 0 0")
            )
            half_extent = _max_collision_half_extent(child)
            if half_extent <= 0.0:
                continue
            obstacles.append(
                Obstacle(name=name_attr, model_type="<model>", x=x, y=y,
                         footprint_radius_m=half_extent)
            )
    return obstacles


# ---------------------------------------------------------------------------
# Rasterisation
# ---------------------------------------------------------------------------

def compute_grid(
    obstacles: List[Obstacle],
    resolution: float,
    padding: float,
) -> GridGeometry:
    """Derive grid extent from the obstacle bounding box plus padding."""
    if not obstacles:
        raise RuntimeError("no obstacles parsed; refuse to build empty map")
    xs_min = min(o.x - o.footprint_radius_m for o in obstacles)
    xs_max = max(o.x + o.footprint_radius_m for o in obstacles)
    ys_min = min(o.y - o.footprint_radius_m for o in obstacles)
    ys_max = max(o.y + o.footprint_radius_m for o in obstacles)

    xs_min -= padding
    xs_max += padding
    ys_min -= padding
    ys_max += padding

    # Snap the origin down to a whole cell multiple for stability across edits.
    origin_x = float(np.floor(xs_min / resolution) * resolution)
    origin_y = float(np.floor(ys_min / resolution) * resolution)
    width_cells = int(np.ceil((xs_max - origin_x) / resolution))
    height_cells = int(np.ceil((ys_max - origin_y) / resolution))

    return GridGeometry(
        origin_x=origin_x, origin_y=origin_y,
        width_cells=width_cells, height_cells=height_cells,
        resolution=resolution,
    )


def rasterize(obstacles: List[Obstacle], grid: GridGeometry) -> np.ndarray:
    """Rasterise obstacle discs into an ``(H, W)`` uint8 PGM array.

    Row 0 is the top of the image, corresponding to ``y = y_max``.
    """
    h = grid.height_cells
    w = grid.width_cells
    image = np.full((h, w), PGM_FREE, dtype=np.uint8)

    for o in obstacles:
        r_pix = o.footprint_radius_m / grid.resolution
        col_center = (o.x - grid.origin_x) / grid.resolution - 0.5
        row_center = (grid.y_max - o.y) / grid.resolution - 0.5

        rmin = int(np.floor(row_center - r_pix))
        rmax = int(np.ceil(row_center + r_pix))
        cmin = int(np.floor(col_center - r_pix))
        cmax = int(np.ceil(col_center + r_pix))

        rmin = max(0, rmin)
        rmax = min(h - 1, rmax)
        cmin = max(0, cmin)
        cmax = min(w - 1, cmax)
        if rmin > rmax or cmin > cmax:
            continue

        rs, cs = np.ogrid[rmin:rmax + 1, cmin:cmax + 1]
        dr = rs - row_center
        dc = cs - col_center
        mask = (dr * dr + dc * dc) <= r_pix * r_pix
        image[rmin:rmax + 1, cmin:cmax + 1][mask] = PGM_OCCUPIED

    return image


def execution_prior(world_sdf: Path):
    """Rasterise a world into the execution prior: canopy discs, trunk geometry.

    Returns ``(image, grid, obstacles)``. This is the map every published
    planning result was computed on, so every tool that needs it must build it
    the same way - hence one function rather than a constant copied around.

    Note what is *not* in it: the Stop-Replan-Resume obstacle. That model is
    spawned into Gazebo at run time and is deliberately absent here, which is
    what makes it detectable as novel.
    """
    obstacles = parse_obstacles(world_sdf)
    grid = compute_grid(obstacles, RESOLUTION_M, PADDING_M)
    canopy = [
        Obstacle(o.name, o.model_type, o.x, o.y,
                 CANOPY_FOOTPRINTS_M.get(o.model_type, o.footprint_radius_m))
        for o in obstacles
    ]
    return rasterize(canopy, grid), grid, obstacles


# ---------------------------------------------------------------------------
# Writers and lookups
# ---------------------------------------------------------------------------

def write_pgm(image: np.ndarray, path: Path) -> None:
    """Write a binary P5 PGM with no metadata, so the bytes are deterministic."""
    h, w = image.shape
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(image.tobytes(order="C"))


def write_map_yaml(path: Path, grid: GridGeometry, pgm_name: str) -> None:
    """Write a nav2 ``map_server``-compatible ``map.yaml``."""
    lines = [
        f"image: {pgm_name}",
        "mode: trinary",
        f"resolution: {grid.resolution:.3f}",
        f"origin: [{grid.origin_x:.3f}, {grid.origin_y:.3f}, 0.000]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def world_to_pixel(grid: GridGeometry, x: float, y: float) -> Tuple[int, int]:
    """World coordinate -> ``(row, col)`` in the rasterised image."""
    col = int((x - grid.origin_x) / grid.resolution)
    row = int((grid.y_max - y) / grid.resolution)
    return row, col


def cell_is_free(image: np.ndarray, grid: GridGeometry, x: float, y: float) -> bool:
    """True if the world point falls on a free cell inside the grid."""
    row, col = world_to_pixel(grid, x, y)
    h, w = image.shape
    if not (0 <= row < h and 0 <= col < w):
        return False
    return bool(image[row, col] == PGM_FREE)
