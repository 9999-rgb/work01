#!/usr/bin/env python3
"""Generate Nav2 occupancy-grid maps by slicing static scene STL meshes.

This is a development-time tool: it reads a binary STL scene mesh, projects it
onto a horizontal occupancy grid, and writes a PGM + YAML pair that Nav2's
``map_server`` can serve.  The generated maps are committed, so this script
(and its ``vtk``/``numpy`` dependencies) is not needed at runtime.

Classification (per grid cell, via a downward vertical ray cast):
  * ray hits a surface at height ~0      -> free floor        -> 254 (white)
  * ray hits a surface above the floor   -> equipment         ->   0 (black)
  * ray misses the mesh entirely         -> hole / off-floor  ->   0 (black)

Usage:
    python3 scripts/generate_scene_maps.py [--resolution 0.05] [--margin 1.0]
                                           [--floor-threshold 0.3]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import vtk
except ImportError as error:  # pragma: no cover - only affects this dev tool
    raise SystemExit(
        "This script requires the 'vtk' Python module "
        "(already present in the ROS 2 Humble environment)."
    ) from error


# The nav2 occupancy convention written to the PGM (see map_io.cpp).
OCCUPIED = 0
FREE = 254


def _scene_registry() -> dict[str, dict[str, object]]:
    root = Path(__file__).resolve().parent.parent
    description = root / "xczs_inspection_robot_description"
    nav2 = root / "xczs_inspection_robot_nav2" / "maps"
    return {
        "electrical_mezzanine": {
            "stl": description
            / "meshes/scenes/electrical_mezzanine/dianqi_ground.STL",
            "basename": "inspection_map_electrical_mezzanine",
            "out_dir": nav2,
        },
        "generator_plant": {
            "stl": description / "meshes/scenes/generator_plant/ground.STL",
            "basename": "inspection_map_generator_plant",
            "out_dir": nav2,
        },
    }


def build_occupancy(
    mesh_path: Path,
    *,
    resolution: float,
    margin: float,
    floor_threshold: float,
) -> tuple[np.ndarray, dict[str, object]]:
    """Return a 2-D uint8 occupancy array and its YAML metadata.

    The array uses the nav2 PGM values (0=occupied, 254=free).  ``metadata``
    contains ``image``/``resolution``/``origin``/``negate`` and thresholds.
    """
    if not mesh_path.is_file():
        raise FileNotFoundError(f"Scene mesh not found: {mesh_path}")

    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(mesh_path))
    reader.Update()
    polydata = reader.GetOutput()
    bounds = polydata.GetBounds()  # xmin, xmax, ymin, ymax, zmin, zmax

    locator = vtk.vtkStaticCellLocator()
    locator.SetDataSet(polydata)
    locator.BuildLocator()

    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    # Snap the origin to a clean value; the STL xmin/ymin are ~0 up to a tiny
    # floating epsilon that would otherwise leak into the map-server origin.
    origin_x = round(xmin - margin, 3)
    origin_y = round(ymin - margin, 3)
    world_width = (xmax + margin) - origin_x
    world_height = (ymax + margin) - origin_y
    cols = int(np.ceil(world_width / resolution))
    rows = int(np.ceil(world_height / resolution))

    z_high = zmax + margin
    z_low = zmin - margin

    occupancy = np.full((rows, cols), OCCUPIED, dtype=np.uint8)
    t = vtk.reference(0.0)
    hit_point = [0.0, 0.0, 0.0]
    pcoords = [0.0, 0.0, 0.0]
    sub_id = vtk.reference(0)

    for row in range(rows):
        # PGM row 0 is the top of the image (highest y).
        y = origin_y + (rows - 1 - row + 0.5) * resolution
        for col in range(cols):
            x = origin_x + (col + 0.5) * resolution
            hit = locator.IntersectWithLine(
                [x, y, z_high],
                [x, y, z_low],
                1.0e-6,
                t,
                hit_point,
                pcoords,
                sub_id,
            )
            if hit and hit_point[2] <= floor_threshold:
                occupancy[row, col] = FREE

    metadata = {
        "image": None,  # filled in by the caller with the basename
        "resolution": resolution,
        "origin": [origin_x, origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    return occupancy, metadata


def _write_pgm(path: Path, occupancy: np.ndarray) -> None:
    rows, cols = occupancy.shape
    header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
    path.write_bytes(header + occupancy.tobytes())


def _write_yaml(path: Path, metadata: dict[str, object]) -> None:
    lines = [
        f"image: {metadata['image']}",
        f"resolution: {metadata['resolution']}",
        f"origin: {metadata['origin']}",
        f"negate: {metadata['negate']}",
        f"occupied_thresh: {metadata['occupied_thresh']}",
        f"free_thresh: {metadata['free_thresh']}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def generate(
    *,
    resolution: float = 0.05,
    margin: float = 1.0,
    floor_threshold: float = 0.3,
) -> None:
    for name, scene in _scene_registry().items():
        mesh_path = Path(scene["stl"])
        basename = str(scene["basename"])
        out_dir = Path(scene["out_dir"])
        out_dir.mkdir(parents=True, exist_ok=True)

        occupancy, metadata = build_occupancy(
            mesh_path,
            resolution=resolution,
            margin=margin,
            floor_threshold=floor_threshold,
        )
        metadata["image"] = f"{basename}.pgm"

        _write_pgm(out_dir / f"{basename}.pgm", occupancy)
        _write_yaml(out_dir / f"{basename}.yaml", metadata)

        free = int(np.count_nonzero(occupancy == FREE))
        total = occupancy.size
        print(
            f"{name}: {occupancy.shape[1]}x{occupancy.shape[0]} cells "
            f"({occupancy.shape[1] * resolution:.1f}m x "
            f"{occupancy.shape[0] * resolution:.1f}m), "
            f"free={free}/{total} ({100.0 * free / total:.1f}%)"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--floor-threshold", type=float, default=0.3)
    args = parser.parse_args()
    generate(
        resolution=args.resolution,
        margin=args.margin,
        floor_threshold=args.floor_threshold,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
