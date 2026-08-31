#!/usr/bin/env python3
"""Generate Nav2 occupancy-grid maps by slicing static scene STL meshes.

This is a development-time tool: it reads a binary STL scene mesh, projects it
onto a horizontal occupancy grid, and writes a PGM + YAML pair that Nav2's
``map_server`` can serve.  The generated maps are committed, so this script
(and its ``vtk``/``numpy`` dependencies) is not needed at runtime.

Classification (per grid cell, via a downward vertical ray-cast interval test):

The scene meshes are *closed building solids* (floor slab at ~0 m, walls,
columns and equipment casings rising to a roof at up to ~5 m).  A robot with
a folded height of ~``clearance_height`` must be able to drive through the
building interior wherever nothing solid occupies its body band.  A vertical
ray pierces the closed mesh in enter/exit pairs; each pair bounds a solid
interval.  The cell is blocked exactly when one of those solid intervals
overlaps the robot's body band ``[floor_threshold, clearance_height]``:

  * no floor surface below ``floor_threshold``
                                -> hole / off-floor -> 0 (black)
  * some solid interval overlaps the body band
                                -> wall / column / equipment -> 0 (black)
  * floor below, open air in the body band
                                -> drivable floor -> 254 (white)

Roofs and mezzanine decks sitting above ``clearance_height`` do not overlap
the band, so their interiors stay free; walls, columns, casings and any
equipment reaching into the band block the cell.

Usage:
    python3 scripts/tools/generate_scene_maps.py [--resolution 0.05] [--margin 1.0]
                                           [--floor-threshold 0.3]
                                           [--clearance-height 0.9]
                                           [--scenes electrical_mezzanine,generator_plant]
                                           [--decimate-below 200000]
                                           [--progress]
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

from _scene_ray_classify import build_locator  # noqa: E402  (dev-tool sibling)
from _scene_ray_classify import classify_cell  # noqa: E402
from _scene_ray_classify import extract_triangles  # noqa: E402


# The nav2 occupancy convention written to the PGM (see map_io.cpp).
OCCUPIED = 0
FREE = 254


def _scene_registry() -> dict[str, dict[str, object]]:
    # scripts/tools/<this file> -> repo root.
    root = Path(__file__).resolve().parent.parent.parent
    description = root / "xczs_inspection_robot_description"
    nav2 = root / "xczs_inspection_robot_nav2" / "maps"
    return {
        "electrical_mezzanine": {
            "stl": description
            / "meshes/scenes/electrical_mezzanine/dianqiground.STL",
            "basename": "inspection_map_electrical_mezzanine",
            "out_dir": nav2,
        },
        "generator_plant": {
            "stl": description
            / "meshes/scenes/generator_plant/fadianground.STL",
            "basename": "inspection_map_generator_plant",
            "out_dir": nav2,
        },
    }


def _load_polydata(mesh_path: Path, *, decimate_below: int | None) -> vtk.vtkPolyData:
    """Load an STL mesh, decimating it in-place when it is excessively large.

    Map generation ray-casts one vertical line per grid cell; a 1.67M-triangle
    ground mesh makes that impractically slow (estimated 15+ min).  The
    occupancy classification only needs the *solid intervals* a ray passes
    through -- surface detail well below the ``resolution`` adds nothing, so a
    uniform decimation for oversized meshes is lossless for our purposes.  This
    decimates only the dev-time copy in memory; the committed visual mesh is
    untouched.
    """
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(mesh_path))
    reader.Update()
    polydata = reader.GetOutput()

    if decimate_below is not None:
        n_triangles = polydata.GetNumberOfCells()
        if n_triangles > decimate_below:
            target_reduction = 1.0 - decimate_below / n_triangles
            decimate = vtk.vtkDecimatePro()
            decimate.SetInputData(polydata)
            decimate.SetTargetReduction(target_reduction)
            # Topology-preserving decimation cannot remove isolated detail and
            # may not reach the target; the map is already fast with
            # vtkStaticCellLocator, so whatever reduction lands is a bonus.
            decimate.PreserveTopologyOn()
            decimate.SplittingOff()
            decimate.Update()
            polydata = decimate.GetOutput()
            actual = 1.0 - polydata.GetNumberOfCells() / n_triangles
            print(
                f"    decimated {n_triangles} -> "
                f"{polydata.GetNumberOfCells()} triangles "
                f"({100.0 * actual:.1f}% actual reduction)"
            )
    return polydata


def build_occupancy(
    mesh_path: Path,
    *,
    resolution: float,
    margin: float,
    floor_threshold: float,
    clearance_height: float,
    decimate_below: int | None = None,
    progress: bool = False,
) -> tuple[np.ndarray, dict[str, object]]:
    """Return a 2-D uint8 occupancy array and its YAML metadata.

    The array uses the nav2 PGM values (0=occupied, 254=free).  ``metadata``
    contains ``image``/``resolution``/``origin``/``negate`` and thresholds.

    Each cell is classified with a vertical ray-cast *interval* test: cast a
    ray from above the mesh down to below the floor, collect every surface it
    crosses, sort top-down, and pair them up (enter, exit) into the solid
    intervals the ray passes through.  A cell is free when a floor lies below
    ``floor_threshold`` and none of the solid intervals overlaps the robot's
    body band ``[floor_threshold, clearance_height]`` -- so walls, columns,
    casings and low equipment block it, while roofs and decks sitting above
    ``clearance_height`` leave the interior drivable.  Cells without a floor
    (holes / off-floor) stay occupied.

    Performance: the scene grounds are large, flat, mostly-coplanar meshes on
    which ``vtkOBBTree``'s "collect all intersections" degenerates (measured
    1.5 ms/cell).  A ``vtkStaticCellLocator`` uniform-grid buckets the mesh
    instead and is ~17x faster; candidate triangles from ``FindCellsAlongLine``
    are then intersected exactly in vectorized numpy (~85 us/cell).
    """
    if not mesh_path.is_file():
        raise FileNotFoundError(f"Scene mesh not found: {mesh_path}")

    polydata = _load_polydata(mesh_path, decimate_below=decimate_below)
    bounds = polydata.GetBounds()  # xmin, xmax, ymin, ymax, zmin, zmax

    locator = build_locator(polydata)
    tri_pts = extract_triangles(polydata)

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
    cell_ids = vtk.vtkIdList()

    for row in range(rows):
        # PGM row 0 is the top of the image (highest y).
        y = origin_y + (rows - 1 - row + 0.5) * resolution
        for col in range(cols):
            x = origin_x + (col + 0.5) * resolution
            if (
                classify_cell(
                    locator,
                    tri_pts,
                    x,
                    y,
                    z_high=z_high,
                    z_low=z_low,
                    floor_threshold=floor_threshold,
                    clearance_height=clearance_height,
                    cell_ids=cell_ids,
                )
                == "floor"
            ):
                occupancy[row, col] = FREE
        if progress and (row % max(1, rows // 20) == 0 or row == rows - 1):
            print(
                f"    row {row + 1}/{rows} "
                f"({100.0 * (row + 1) / rows:.0f}%)",
                flush=True,
            )

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
    clearance_height: float = 0.9,
    scenes: list[str] | None = None,
    decimate_below: int | None = None,
    progress: bool = False,
) -> None:
    registry = _scene_registry()
    if scenes is not None:
        unknown = sorted(set(scenes) - set(registry))
        if unknown:
            raise SystemExit(f"Unknown scene(s): {', '.join(unknown)}")
        registry = {name: registry[name] for name in scenes}

    for name, scene in registry.items():
        mesh_path = Path(scene["stl"])
        basename = str(scene["basename"])
        out_dir = Path(scene["out_dir"])
        out_dir.mkdir(parents=True, exist_ok=True)

        occupancy, metadata = build_occupancy(
            mesh_path,
            resolution=resolution,
            margin=margin,
            floor_threshold=floor_threshold,
            clearance_height=clearance_height,
            decimate_below=decimate_below,
            progress=progress,
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
    parser.add_argument("--clearance-height", type=float, default=0.9)
    parser.add_argument("--scenes", type=str, default=None,
                        help="comma-separated scene names to generate (default: all)")
    parser.add_argument("--decimate-below", type=int, default=200_000,
                        help="decimate meshes with more triangles than this "
                             "(default 200000; pass 0 to disable)")
    parser.add_argument("--progress", action="store_true",
                        help="print per-row progress while classifying")
    args = parser.parse_args()
    scenes = [s.strip() for s in args.scenes.split(",")] if args.scenes else None
    generate(
        resolution=args.resolution,
        margin=args.margin,
        floor_threshold=args.floor_threshold,
        clearance_height=args.clearance_height,
        scenes=scenes,
        decimate_below=args.decimate_below or None,
        progress=args.progress,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
