#!/usr/bin/env python3
"""Analyze electrical-mezzanine drawer STL meshes with VTK.

Compute bbox, centroid, front-face normal, and handle-riser regions
for b1 / m1 drawer meshes and their sub-parts (b1r/b1p/m1p).

Frames:
  - STL-local frame: as stored in the mesh file.
  - b1-local frame:  b1.STL is attached at identity; sub-parts merged at xacro pose.
  - world frame:      add b1 root pose (0.03, 4.047, 0.831870606) (no rotation).
"""
import vtk
import numpy as np
import os

MESH_DIR = "/home/live/work01/xczs_inspection_robot_description/meshes/scenes/electrical_mezzanine"

# xacro merge offsets (b1-local -> child mesh origin)
OFFSETS = {
    "b1.STL": (0.0, 0.0, 0.0),
    "b1r.STL": (0.03, 0.16661, 0.12017),
    "b1p.STL": (0.069, 0.638, 0.168),
    "m1.STL": (0.0, 0.0, 0.0),
    "m1r.STL": (0.03, 0.16661, 0.060175),
    "m1p.STL": (0.069, 0.638, 0.108),
}
ROOT_POSES = {
    "b1": (0.03, 4.047, 0.831870606),
    "m1": (0.03, 4.847, 0.895),
}


def read_stl(path):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(path)
    reader.Update()
    return reader.GetOutput()


def points_of(pd):
    pts = pd.GetPoints()
    n = pts.GetNumberOfPoints()
    arr = np.zeros((n, 3))
    for i in range(n):
        arr[i] = pts.GetPoint(i)
    return arr


def triangles_of(pd):
    polys = pd.GetPolys()
    polys.InitTraversal()
    tris = []
    while True:
        ids = vtk.vtkIdList()
        r = polys.GetNextCell(ids)
        if r == 0:
            break
        tris.append([ids.GetId(j) for j in range(ids.GetNumberOfIds())])
    return np.array(tris)


def bbox(pts):
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    return mn, mx, (mx - mn)


def centroid(pts):
    return pts.mean(axis=0)


def area_weighted_normal(pd, pts, tris):
    """Return dominant outward-ish normal via area-weighted triangle normals."""
    n = np.zeros(3)
    total_area = 0.0
    for t in tris:
        a = pts[t[0]]
        b = pts[t[1]]
        c = pts[t[2]]
        v1 = b - a
        v2 = c - a
        cr = np.cross(v1, v2)
        ar = 0.5 * np.linalg.norm(cr)
        if ar < 1e-12:
            continue
        nn = cr / np.linalg.norm(cr)
        # orient consistently: make it point +x-ish by convention? No, keep raw.
        n += ar * nn
        total_area += ar
    if total_area > 0:
        n /= total_area
    n /= np.linalg.norm(n)
    return n, total_area


def fmt(v, prec=4):
    return "[" + ", ".join(f"{x:.{prec}f}" for x in v) + "]"


def analyze_mesh(name, pd, offset, root_pose, label=""):
    print("=" * 78)
    print(f"{name}  {label}")
    pts = points_of(pd)
    tris = triangles_of(pd)
    mn, mx, sz = bbox(pts)
    c = centroid(pts)
    norm, area = area_weighted_normal(pd, pts, tris)
    print(f"  n_points = {len(pts)}, n_triangles = {len(tris)}")
    print(f"  STL-local bbox  min={fmt(mn)} max={fmt(mx)} size={fmt(sz)}")
    print(f"  STL-local centroid = {fmt(c)}")
    print(f"  area-weighted normal (raw) = {fmt(norm, 3)}")
    # local frame (offset applied = actual geometry position in b1 local frame)
    lmn = mn + np.array(offset)
    lmx = mx + np.array(offset)
    lc = c + np.array(offset)
    print(f"  b1-local   bbox  min={fmt(lmn)} max={fmt(lmx)}")
    print(f"  b1-local   centroid = {fmt(lc)}")
    if root_pose is not None:
        rp = np.array(root_pose)
        wmn = lmn + rp
        wmx = lmx + rp
        wc = lc + rp
        print(f"  world      bbox  min={fmt(wmn)} max={fmt(wmx)}")
        print(f"  world      centroid = {fmt(wc)}")
    return pts, tris, mn, mx, sz


def cluster_front_faces(pts, xmax, x_tol=0.002):
    """Points on the front plane(s) near max x; cluster by y into groups."""
    mask = pts[:, 0] >= (xmax - x_tol)
    fp = pts[mask]
    return fp


def main():
    # ---- 1. b1.STL: full drawer front panel ----
    pd = read_stl(os.path.join(MESH_DIR, "b1.STL"))
    pts, tris, mn, mx, sz = analyze_mesh("b1.STL", pd, OFFSETS["b1.STL"], ROOT_POSES["b1"], "drawer front panel")

    xmax = mx[0]
    print(f"\n  b1.STL xmax = {xmax:.4f}")
    # x distribution summary: unique x levels
    xs = np.unique(np.round(pts[:, 0], 3))
    print(f"  distinct x values (rounded 3): {xs}")
    # x histogram
    hist, edges = np.histogram(pts[:, 0], bins=40)
    print("  x histogram (bins 40):")
    for i, h in enumerate(hist):
        if h > 0:
            print(f"    x[{edges[i]:.3f}-{edges[i+1]:.3f}): {h} pts")

    # front-face points near max x
    fp = cluster_front_faces(pts, xmax, x_tol=0.003)
    print(f"\n  points on front plane x >= {xmax-0.003:.4f}: {len(fp)}")
    if len(fp):
        ymin = fp[:, 1].min()
        ymax = fp[:, 1].max()
        print(f"  front-plane y range [{ymin:.4f}, {ymax:.4f}], z range [{fp[:,2].min():.4f}, {fp[:,2].max():.4f}]")
        # cluster by y
        # simple: find gaps in y
        ys = np.sort(fp[:, 1])
        gaps = np.diff(ys)
        big_gap = gaps.max()
        print(f"  max y gap between consecutive front pts = {big_gap:.4f}")

        # KMeans-like simple split via gap
        thresh = 0.05
        # split into connected components by y distance
        groups = []
        cur = [fp[0]]
        for p in fp[1:]:
            if p[1] - cur[-1][1] > thresh:
                groups.append(np.array(cur))
                cur = [p]
            else:
                cur.append(p)
        groups.append(np.array(cur))
        print(f"  y-connected groups (gap>{thresh}): {len(groups)}")
        for gi, g in enumerate(groups):
            gmn = g.min(axis=0)
            gmx = g.max(axis=0)
            gc = g.mean(axis=0)
            print(f"    group {gi}: n={len(g)} bbox min={fmt(gmn)} max={fmt(gmx)}")
            print(f"       centroid={fmt(gc)}  front_x_max={gmx[0]:.4f} ycenter={gc[1]:.4f} z[{gmn[2]:.4f},{gmx[2]:.4f}]")

    # ---- 2. sub-parts ----
    for sub in ["b1r.STL", "b1p.STL", "m1p.STL"]:
        pd2 = read_stl(os.path.join(MESH_DIR, sub))
        analyze_mesh(sub, pd2, OFFSETS[sub], ROOT_POSES["b1"] if sub.startswith("b1") else ROOT_POSES["m1"])

    # ---- m1.STL for reference ----
    pd3 = read_stl(os.path.join(MESH_DIR, "m1.STL"))
    analyze_mesh("m1.STL", pd3, OFFSETS["m1.STL"], ROOT_POSES["m1"], "drawer front panel (reference)")

    print("=" * 78)


if __name__ == "__main__":
    main()
