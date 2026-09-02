#!/usr/bin/env python3
"""Verify m1 riser geometry, b1p orientation, b1r knob axis, riser front normals."""
import vtk
import numpy as np
import os

MESH_DIR = "/home/live/work01/xczs_inspection_robot_description/meshes/scenes/electrical_mezzanine"
ROOT_B1 = np.array([0.03, 4.047, 0.831870606])
ROOT_M1 = np.array([0.03, 4.847, 0.895])


def read_stl(path):
    r = vtk.vtkSTLReader()
    r.SetFileName(path)
    r.Update()
    return r.GetOutput()


def pts_of(pd):
    pts = pd.GetPoints()
    n = pts.GetNumberOfPoints()
    a = np.zeros((n, 3))
    for i in range(n):
        a[i] = pts.GetPoint(i)
    return a


def tris_of(pd):
    polys = pd.GetPolys()
    polys.InitTraversal()
    tris = []
    while True:
        ids = vtk.vtkIdList()
        if polys.GetNextCell(ids) == 0:
            break
        tris.append([ids.GetId(j) for j in range(ids.GetNumberOfIds())])
    return np.array(tris)


def tri_normals(pts, tris):
    ns = []
    areas = []
    for t in tris:
        a, b, c = pts[t[0]], pts[t[1]], pts[t[2]]
        cr = np.cross(b - a, c - a)
        ar = 0.5 * np.linalg.norm(cr)
        if ar < 1e-12:
            continue
        ns.append(cr / np.linalg.norm(cr))
        areas.append(ar)
    return np.array(ns), np.array(areas)


def fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"


def pf(v):
    return f"({v[0]:+.4f},{v[1]:+.4f},{v[2]:+.4f})"


print("========== m1.STL riser regions ==========")
pd = read_stl(os.path.join(MESH_DIR, "m1.STL"))
pts = pts_of(pd)
for label, lo, hi in [("left riser y[0.02,0.10]", 0.02, 0.10), ("right riser y[0.60,0.70]", 0.60, 0.70)]:
    m = (pts[:, 1] >= lo) & (pts[:, 1] <= hi)
    sub = pts[m]
    if len(sub):
        mn = sub.min(axis=0); mx = sub.max(axis=0)
        print(f"{label}: n={len(sub)} bbox min={pf(mn)} max={pf(mx)} size={pf(mx-mn)}")
        print(f"    world bbox min={pf(mn+ROOT_M1)} max={pf(mx+ROOT_M1)}")

print("\n========== b1p.STL triangle normals (orientation) ==========")
pd = read_stl(os.path.join(MESH_DIR, "b1p.STL"))
pts = pts_of(pd)
tris = tris_of(pd)
ns, areas = tri_normals(pts, tris)
print(f"n_triangles={len(tris)}")
for n, a in zip(ns, areas):
    print(f"  normal [{n[0]:+.3f},{n[1]:+.3f},{n[2]:+.3f}]  area {a:.6f}")

print("\n========== b1r.STL (knob) bbox & normals ==========")
pd = read_stl(os.path.join(MESH_DIR, "b1r.STL"))
pts = pts_of(pd)
tris = tris_of(pd)
mn = pts.min(axis=0); mx = pts.max(axis=0)
print(f"STL-local bbox min={pf(mn)} max={pf(mx)} size={pf(mx-mn)}")
print(f"b1-local bbox min={pf(mn+np.array([0.03,0.16661,0.12017]))} max={pf(mx+np.array([0.03,0.16661,0.12017]))}")
print(f"world bbox min={pf(mn+np.array([0.03,0.16661,0.12017])+ROOT_B1)} max={pf(mx+np.array([0.03,0.16661,0.12017])+ROOT_B1)}")
ns, areas = tri_normals(pts, tris)
# histogram of |nx| to see axis direction
ax = np.abs(ns)
print("triangle normal dominant axis counts (|nx|>0.8, |ny|>0.8, |nz|>0.8):",
      int((ax[:,0]>0.8).sum()), int((ax[:,1]>0.8).sum()), int((ax[:,2]>0.8).sum()))

print("\n========== riser front-face normals in b1.STL ==========")
pd = read_stl(os.path.join(MESH_DIR, "b1.STL"))
pts = pts_of(pd)
tris = tris_of(pd)
# find triangles whose all 3 verts have x near max (0.069)
mask_tris = np.array([ (pts[t,0].min() > 0.068) for t in tris ])
sel = tris[mask_tris]
print(f"triangles on front plane x>0.068: {len(sel)}")
if len(sel):
    ns2, areas2 = tri_normals(pts, sel)
    for n, a in zip(ns2, areas2):
        print(f"  normal [{n[0]:+.3f},{n[1]:+.3f},{n[2]:+.3f}]  area {a:.6f}")
