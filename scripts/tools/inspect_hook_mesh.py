#!/usr/bin/env python3
"""用「平面×三角形求交」精确解析钩爪杆件端部形状，并渲染三视图。

为什么不用顶点切片：STL 是车削/倒角面片，长三角形跨多个 z 带，按顶点分箱会
漏带、把包围盒算飞。必须用平面与三角形真求交，得到该 z 截面的真实轮廓。

设计目标（fix#25 后续）：判定「钩爪从把手外侧绕过去再内收卡住」到底能不能
靠杆的伸缩完成 —— 需要知道 (a) 杆身直径，(b) 端部侧爪在世界 z 向的真实高度
跨度（因为它决定钩爪能否从把手立筋的上/下方探进 53mm 通腔再回勾）。

用法：
  python3 scripts/tools/inspect_hook_mesh.py \
      xczs_inspection_robot_description/meshes/tools/two_cylinder/endlink1.STL \
      --out /tmp/hook_mesh
"""

import argparse
import struct

import numpy as np


def read_stl(path):
    with open(path, "rb") as handle:
        head = handle.read(84)
        count = struct.unpack("<I", head[80:84])[0]
        body = handle.read(count * 50)
    tris = np.frombuffer(body, dtype=np.uint8).reshape(count, 50)
    floats = tris[:, :48].copy().view("<f4").reshape(count, 12)
    return floats[:, 3:12].reshape(count, 3, 3).astype(float)


def cross_section(tris, z):
    """返回 z 平面与网格的交线段列表 [(x0,y0,x1,y1), ...]。"""
    segs = []
    for tri in tris:
        zs = tri[:, 2]
        if zs.max() < z or zs.min() > z:
            continue
        pts = []
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            if (a[2] - z) * (b[2] - z) < 0:
                t = (z - a[2]) / (b[2] - a[2])
                pts.append(a[:2] + t * (b[:2] - a[:2]))
            elif abs(a[2] - z) < 1e-12:
                pts.append(a[:2])
        if len(pts) >= 2:
            segs.append((pts[0][0], pts[0][1], pts[1][0], pts[1][1]))
    return segs


def rasterize(segs, cell):
    """把交线段栅格化成 ASCII 位图。返回 (rows, cols, origin_x, origin_y)。

    用奇偶规则扫描：每个采样点沿 +x 打射线数交点，奇数=实体。这样能直接看
    出截面的**孔/缝**在哪 —— 只看包围盒看不出叉齿之间的空隙。
    """
    xs = [s[0] for s in segs] + [s[2] for s in segs]
    ys = [s[1] for s in segs] + [s[3] for s in segs]
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = min(ys), max(ys)
    cols = max(int((x_hi - x_lo) / cell) + 1, 1)
    rows = max(int((y_hi - y_lo) / cell) + 1, 1)

    def inside(px, py):
        hits = 0
        for x0, y0, x1, y1 in segs:
            if (y0 > py) != (y1 > py):
                xi = x0 + (py - y0) / (y1 - y0) * (x1 - x0)
                if xi > px:
                    hits += 1
        return hits % 2 == 1

    grid = []
    for r in range(rows):
        # 行从上往下 = y 从大到小
        py = y_hi - (r + 0.5) * cell
        grid.append("".join(
            "#" if inside(x_lo + (c + 0.5) * cell, py) else "."
            for c in range(cols)))
    return grid, x_lo, y_hi


def print_ascii(segs, cell, label):
    grid, x_lo, y_hi = rasterize(segs, cell)
    print(f"\n--- section {label}  (origin x{x_lo:+.4f} y{y_hi:+.4f}, "
          f"cell {cell*1000:.1f}mm, row0 = y_max) ---")
    for r, line in enumerate(grid):
        print(f"y{y_hi - (r + 0.5) * cell:+.4f} |{line}|")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh")
    ap.add_argument("--out", default=None)
    ap.add_argument("--step", type=float, default=0.005)
    ap.add_argument("--ascii", default=None,
                    help="逗号分隔的 z 站位，逐站打印 (x,y) 截面 ASCII 位图")
    ap.add_argument("--cell", type=float, default=0.002,
                    help="ASCII 位图栅格边长（米）")
    args = ap.parse_args()

    tris = read_stl(args.mesh)
    vs = tris.reshape(-1, 3)
    lo, hi = vs.min(axis=0), vs.max(axis=0)
    print(f"mesh {args.mesh}: {len(tris)} tris")
    print(f"bbox lo {lo[0]:+.4f} {lo[1]:+.4f} {lo[2]:+.4f}")
    print(f"bbox hi {hi[0]:+.4f} {hi[1]:+.4f} {hi[2]:+.4f}")

    print()
    print("    z   | seg |    x_lo    x_hi    y_lo    y_hi | dx     dy")
    z = lo[2] + args.step / 2
    while z < hi[2]:
        segs = cross_section(tris, z)
        if segs:
            xs = [s[0] for s in segs] + [s[2] for s in segs]
            ys = [s[1] for s in segs] + [s[3] for s in segs]
            print(f"{z:+.4f} | {len(segs):3d} | {min(xs):+.4f} {max(xs):+.4f} "
                  f"{min(ys):+.4f} {max(ys):+.4f} | "
                  f"{max(xs)-min(xs):.4f} {max(ys)-min(ys):.4f}")
        z += args.step

    if args.ascii:
        for token in args.ascii.split(","):
            z = float(token)
            segs = cross_section(tris, z)
            if not segs:
                print(f"\n--- section z{z:+.4f}: empty ---")
                continue
            print_ascii(segs, args.cell, f"z={z:+.4f}")

    if args.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        views = [
            ("front (look -z, x-y)", 0, 1, "X", "Y"),
            ("side  (look -x, y-z)", 1, 2, "Y", "Z"),
            ("top   (look -y, x-z)", 0, 2, "X", "Z"),
        ]
        fig, axes = plt.subplots(1, 3, figsize=(19, 6))
        order = np.argsort(-vs[:, 1])
        for ax, (title, i, j, li, lj) in zip(axes, views):
            for tri in tris:
                pts = tri[:, [i, j]]
                ax.fill(pts[:, 0], pts[:, 1], facecolor="#c8d8ea",
                        edgecolor="#4a6fa5", linewidth=0.15)
            ax.set_title(title)
            ax.set_xlabel(li)
            ax.set_ylabel(lj)
            ax.set_aspect("equal")
            ax.grid(alpha=0.25, linewidth=0.3)
        fig.suptitle(f"{args.mesh.split('/')[-1]} — claw fin +Y up to "
                     f"{hi[1]*1000:.1f}mm, thickness X {hi[0]-lo[0]:.3f}..")
        fig.tight_layout()
        fig.savefig(args.out + ".png", dpi=110)
        print(f"\nwrote {args.out}.png")


if __name__ == "__main__":
    main()
