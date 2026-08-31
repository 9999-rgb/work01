#!/usr/bin/env python3
"""Shared vertical ray-cast classification for the scene dev tools.

Both ``generate_scene_maps.py`` and ``build_scene_sdf.py`` need to answer the
same question about a scene's ground mesh: *for a map cell / collision cell,
is there solid material inside the robot's body band?*  This module factors the
exact ray-cast machinery both use so the two tools cannot drift apart.

The scene grounds are large, flat, mostly-coplanar meshes (SolidWorks exports)
on which ``vtkOBBTree``'s "collect all intersections" call degenerates (measured
~1.5 ms/cell).  A ``vtkStaticCellLocator`` uniform grid buckets the mesh
instead (~85 us/cell): candidate triangles from ``FindCellsAlongLine`` are
intersected exactly (Möller–Trumbore) against a downward ray in vectorized
numpy, giving every surface's z on the ray.
"""

from __future__ import annotations

import numpy as np
import vtk

try:
    from vtk.util.numpy_support import vtk_to_numpy
except ImportError:  # pragma: no cover - vtk ships numpy_support
    vtk_to_numpy = None  # type: ignore[assignment]


def build_locator(polydata: vtk.vtkPolyData) -> vtk.vtkStaticCellLocator:
    """Build the fast ray-cast locator over ``polydata``."""
    locator = vtk.vtkStaticCellLocator()
    locator.SetDataSet(polydata)
    locator.BuildLocator()
    return locator


def extract_triangles(polydata: vtk.vtkPolyData) -> np.ndarray:
    """Return the mesh triangles as one ``(n, 3, 3)`` vertex array."""
    points = vtk_to_numpy(polydata.GetPoints().GetData())
    polys = vtk_to_numpy(polydata.GetPolys().GetData())
    return points[polys.reshape(-1, 4)[:, 1:]].astype(np.float64)


def vertical_ray_zs(
    cell_ids: np.ndarray,
    tri_pts: np.ndarray,
    x: float,
    y: float,
    z_high: float,
    z_low: float,
) -> np.ndarray:
    """Return the z of every surface a downward vertical ray hits, top-down.

    ``cell_ids`` are the candidate triangle ids the locator returned for this
    cell; each is intersected exactly against the ray.  ``tri_pts`` is the
    ``(n, 3, 3)`` vertex array of the whole mesh (indexed by ``cell_ids``).
    """
    t = tri_pts[cell_ids]
    d = np.array([0.0, 0.0, -1.0])
    o = np.array([x, y, z_high])
    e1 = t[:, 1] - t[:, 0]
    e2 = t[:, 2] - t[:, 0]
    tvec = o - t[:, 0]
    pvec = np.cross(d, e2)
    det = (pvec * e1).sum(axis=1)
    ok = np.abs(det) > 1e-12
    inv = 1.0 / np.where(np.abs(det) > 1e-12, det, 1.0)
    u = (tvec * pvec).sum(axis=1) * inv
    ok &= (u >= -1e-9) & (u <= 1 + 1e-9)
    qvec = np.cross(tvec, e1)
    v = (d * qvec).sum(axis=1) * inv
    ok &= (v >= -1e-9) & (u + v <= 1 + 1e-9)
    tpar = (e2 * qvec).sum(axis=1) * inv
    zs = z_high + tpar[ok] * d[2]
    return np.sort(zs)[::-1]


def classify_cell(
    locator: vtk.vtkStaticCellLocator,
    tri_pts: np.ndarray,
    x: float,
    y: float,
    *,
    z_high: float,
    z_low: float,
    floor_threshold: float,
    clearance_height: float,
    cell_ids: vtk.vtkIdList,
) -> str:
    """Classify one grid cell against the robot body band.

    Returns one of:

      * ``"hole"``  -- no surface below ``floor_threshold`` (off the floor)
      * ``"wall"``  -- a solid interval overlaps ``[floor_threshold,
                       clearance_height]`` (blocked by walls / equipment)
      * ``"floor"`` -- floor below, open air in the body band (drivable)

    The interval test pairs the ray crossings (enter, exit) top-down; an
    unpaired last crossing (non-watertight mesh) extends to the bottom of the
    ray.  Roofs and decks sitting above ``clearance_height`` do not overlap
    the band, so their interiors stay drivable.
    """
    p1 = [x, y, z_high]
    p2 = [x, y, z_low]
    cell_ids.Reset()
    locator.FindCellsAlongLine(p1, p2, 1e-6, cell_ids)
    n = cell_ids.GetNumberOfIds()
    if n == 0:
        return "hole"
    ids_arr = np.fromiter((cell_ids.GetId(i) for i in range(n)), dtype=np.int64)
    zs = vertical_ray_zs(ids_arr, tri_pts, x, y, z_high, z_low)
    if zs.size == 0 or not np.any(zs <= floor_threshold):
        return "hole"
    band_bottom = floor_threshold
    band_top = clearance_height
    for i in range(0, zs.size, 2):
        hi = zs[i]
        lo = zs[i + 1] if i + 1 < zs.size else z_low
        if hi > band_bottom and lo < band_top:
            return "wall"
    return "floor"
