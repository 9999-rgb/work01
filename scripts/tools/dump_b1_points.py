#!/usr/bin/env python3
"""Dump all points of b1.STL and identify handle-riser regions by y."""
import vtk
import numpy as np
import os

MESH_DIR = "/home/live/work01/xczs_inspection_robot_description/meshes/scenes/electrical_mezzanine"
ROOT = np.array([0.03, 4.047, 0.831870606])


def read_stl(path):
    r = vtk.vtkSTLReader()
    r.SetFileName(path)
    r.Update()
    return r.GetOutput()


def points_of(pd):
    pts = pd.GetPoints()
    n = pts.GetNumberOfPoints()
    arr = np.zeros((n, 3))
    for i in range(n):
        arr[i] = pts.GetPoint(i)
    return arr


def pfmt(v):
    return f"({v[0]:+.4f},{v[1]:+.4f},{v[2]:+.4f})"


pd = read_stl(os.path.join(MESH_DIR, "b1.STL"))
pts = points_of(pd)
print(f"total points: {len(pts)}")
print("all points (local frame, sorted by x then y):")
idx = np.lexsort((pts[:, 2], pts[:, 1], pts[:, 0]))
for i in idx:
    print(f"  {pfmt(pts[i])}   world {pfmt(pts[i] + ROOT)}")

# Riser regions
print("\n---- left riser candidate y in [0.02, 0.10] ----")
mask_l = (pts[:, 1] >= 0.02) & (pts[:, 1] <= 0.10)
sub = pts[mask_l]
if len(sub):
    mn = sub.min(axis=0); mx = sub.max(axis=0)
    print(f"  n={len(sub)} bbox min={pfmt(mn)} max={pfmt(mx)} size={pfmt(mx-mn)}")
    print(f"  world bbox min={pfmt(mn+ROOT)} max={pfmt(mx+ROOT)}")

print("\n---- right riser candidate y in [0.60, 0.70] ----")
mask_r = (pts[:, 1] >= 0.60) & (pts[:, 1] <= 0.70)
sub = pts[mask_r]
if len(sub):
    mn = sub.min(axis=0); mx = sub.max(axis=0)
    print(f"  n={len(sub)} bbox min={pfmt(mn)} max={pfmt(mx)} size={pfmt(mx-mn)}")
    print(f"  world bbox min={pfmt(mn+ROOT)} max={pfmt(mx+ROOT)}")

# central panel region
print("\n---- central panel y in [0.10, 0.60], x < 0.03 ----")
mask_c = (pts[:, 1] >= 0.10) & (pts[:, 1] <= 0.60) & (pts[:, 0] < 0.03)
sub = pts[mask_c]
if len(sub):
    mn = sub.min(axis=0); mx = sub.max(axis=0)
    print(f"  n={len(sub)} bbox min={pfmt(mn)} max={pfmt(mx)} size={pfmt(mx-mn)}")

# x-distinct levels detail
print("\n---- points grouped by rounded x ----")
for xv in sorted(set(np.round(pts[:, 0], 3))):
    m = np.abs(pts[:, 0] - xv) < 0.002
    sub = pts[m]
    ys = np.unique(np.round(sub[:, 1], 3))
    zs = np.unique(np.round(sub[:, 2], 3))
    print(f"  x={xv:.3f}: {len(sub)} pts  y={ys}  z={zs}")
