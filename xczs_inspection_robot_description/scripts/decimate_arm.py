#!/usr/bin/env python3
"""Decimate an arm-link visual STL into a collision STL using VTK's quadric
edge-collapse decimation.

Convex hulls are unsuitable for the arm links: the wrist (arm_6) is genuinely
non-convex, so its hull bulges into the elbow (arm_4) and creates phantom
self-collisions that make every MoveIt plan fail with START_STATE_INVALID.
A decimated surface preserves the concavity while keeping the triangle count
(and therefore MoveIt FCL / Gazebo cost) low.

Usage: decimate_arm.py <input_visual.stl> <output_collision.stl> [target_faces]
"""
import sys

import vtk

DEFAULT_TARGET = 2500


def read_stl(path):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(path)
    reader.Update()
    poly = reader.GetOutput()
    nf = poly.GetNumberOfCells()
    nv = poly.GetNumberOfPoints()
    return poly, nv, nf


def decimate(poly, nf, target):
    reduction = max(0.0, 1.0 - target / float(nf))
    dec = vtk.vtkDecimatePro()
    dec.SetInputData(poly)
    dec.SetTargetReduction(reduction)
    dec.SetPreserveTopology(0)       # collision soup meshes: topology not needed
    dec.SetBoundaryVertexDeletion(1) # keep outer boundary vertices stable
    dec.SetSplitting(1)
    dec.SetPreSplitMesh(1)
    dec.Update()
    return dec.GetOutput()


def write_stl(poly, path):
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(path)
    writer.SetFileTypeToBinary()
    writer.SetInputData(poly)
    writer.Write()


def main():
    src, dst = sys.argv[1], sys.argv[2]
    target = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TARGET

    poly, nv, nf = read_stl(src)
    if nf <= target:
        # Already below target: use the visual mesh verbatim as collision.
        write_stl(poly, dst)
        print(f"{dst}: {nf} faces (unchanged, below target {target})")
        return 0

    out = decimate(poly, nf, target)
    onf = out.GetNumberOfCells()
    onv = out.GetNumberOfPoints()
    if onf < 1:
        print(f"ERROR: {src}: decimation produced 0 faces", file=sys.stderr)
        return 1

    # Sanity: the decimated hull must bound the visual bbox (rough check).
    bv = poly.GetBounds()
    bo = out.GetBounds()
    for i, (a, b) in enumerate(zip(bv, bo)):
        if abs(a - b) > 0.02:  # 2 cm drift on any bound is suspicious
            print(f"WARNING: {src} bound {i}: visual {a:.3f} vs dec {b:.3f}",
                  file=sys.stderr)

    write_stl(out, dst)
    print(f"{dst}: {nf} -> {onf} faces, {nv} -> {onv} verts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
