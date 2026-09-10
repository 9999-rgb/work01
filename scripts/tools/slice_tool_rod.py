#!/usr/bin/env python3
"""按局部 z 切片解析作业工具杆件 STL，量出杆身直径与端部钩爪的真实形状。

背景（fix#25 后续，2026-09-10）：要设计「钩爪从把手外侧绕过去再内收卡住」，
必须知道杆端那 55mm 侧爪到底是刀片、钩子还是斜面，以及杆身直径 —— 因为把手
立筋在 y 向只有 16mm，杆身若比它粗就永远进不了那个 53×16mm 的腔，只能从
立筋上/下方绕到前板背后去卡。本脚本输出每 2mm 一片的 (x,y) 包围盒 + 点数，
并在关键片打印顶点，供人肉判形。

用法：
  python3 scripts/tools/slice_tool_rod.py \
      xczs_inspection_robot_description/meshes/tools/two_cylinder/endlink1.STL
"""

import struct
import sys


def read_stl(path):
    with open(path, "rb") as handle:
        head = handle.read(84)
        if len(head) < 84:
            raise RuntimeError("file too short")
        count = struct.unpack("<I", head[80:84])[0]
        body = handle.read(count * 50)
        if len(body) < count * 50:
            raise RuntimeError("truncated binary STL")
    tris = []
    for i in range(count):
        off = i * 50
        cells = struct.unpack("<12fH", body[off:off + 50])
        tris.append(((cells[3], cells[4], cells[5]),
                     (cells[6], cells[7], cells[8]),
                     (cells[9], cells[10], cells[11])))
    return tris


def main():
    path = sys.argv[1]
    tris = read_stl(path)
    vs = [v for t in tris for v in t]
    lo = [min(v[i] for v in vs) for i in range(3)]
    hi = [max(v[i] for v in vs) for i in range(3)]
    print(f"file      : {path}")
    print(f"triangles : {len(tris)}")
    print(f"bbox lo   : {lo[0]:+.4f} {lo[1]:+.4f} {lo[2]:+.4f}")
    print(f"bbox hi   : {hi[0]:+.4f} {hi[1]:+.4f} {hi[2]:+.4f}")
    print(f"size      : {hi[0]-lo[0]:.4f} {hi[1]-lo[1]:.4f} {hi[2]-lo[2]:.4f}")

    # 只统计"杆身柱体"：取 z 中段，看 (x,y) 半径随 z 的变化
    print()
    print(" z_lo   z_hi |  n |    x_lo    x_hi    y_lo    y_hi | dx     dy"
          "   cx     cy")
    z = lo[2]
    step = 0.002
    while z < hi[2] - 1e-6:
        z2 = min(z + step, hi[2])
        sel = [v for v in vs if z - 1e-6 <= v[2] < z2 - 1e-6 + 1e-9]
        if sel:
            xs = [v[0] for v in sel]
            ys = [v[1] for v in sel]
            print(f"{z:+.4f} {z2:+.4f} | {len(sel):3d} | "
                  f"{min(xs):+.4f} {max(xs):+.4f} {min(ys):+.4f} {max(ys):+.4f} | "
                  f"{max(xs)-min(xs):.4f} {max(ys)-min(ys):.4f} "
                  f"{(min(xs)+max(xs))/2:+.4f} {(min(ys)+max(ys))/2:+.4f}")
        z = z2


if __name__ == "__main__":
    main()
