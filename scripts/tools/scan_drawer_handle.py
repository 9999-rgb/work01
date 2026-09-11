#!/usr/bin/env python3
"""沿 +X 对 b1.STL（电气夹层 db1 抽屉）做射线扫描，弄清把手立筋附近到底
哪一段 x 有实体、哪一段是空腔。

判定「钩爪能不能绕到把手后面」只需要一件事：在立筋的 y 带上，抽屉前脸
沿 x 的实体分层是什么。本脚本把每一层打印出来，不做任何假设。

用法：
  python3 scripts/tools/scan_drawer_handle.py                 # 默认立筋中心扫描
  python3 scripts/tools/scan_drawer_handle.py --z 0.120 --y 0.060 --span 0.05
"""

import argparse
import struct

import numpy as np

B1_ORIGIN = np.array([0.03, 4.047, 0.831870606])


def read_stl(path):
    with open(path, "rb") as handle:
        head = handle.read(84)
        count = struct.unpack("<I", head[80:84])[0]
        body = handle.read(count * 50)
    tris = np.frombuffer(body, dtype=np.uint8).reshape(count, 50)
    floats = tris[:, :48].copy().view("<f4").reshape(count, 12)
    return floats[:, 3:12].reshape(count, 3, 3).astype(float)


def ray_x_intervals(tris, y0, z0):
    """在 (y=y0, z=z0) 处沿 +x 打射线，返回按 x 排序的「进/出」交替列表。

    做法：把三角形投影到 (y,z) 平面，做二维点在三角形内测试，命中则按重心
    坐标插值出该处的 x。射线与三角形共面/退化的情形直接跳过。
    """
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    # 二维重心坐标（投影到 y,z）
    y0v, z0v = y0, z0
    d = (b[:, 1] - a[:, 1]) * (c[:, 2] - a[:, 2]) - \
        (c[:, 1] - a[:, 1]) * (b[:, 2] - a[:, 2])
    good = np.abs(d) > 1e-12
    w0 = ((b[:, 1] - y0v) * (c[:, 2] - z0v) -
          (c[:, 1] - y0v) * (b[:, 2] - z0v)) / np.where(good, d, 1.0)
    w1 = ((c[:, 1] - y0v) * (a[:, 2] - z0v) -
          (a[:, 1] - y0v) * (c[:, 2] - z0v)) / np.where(good, d, 1.0)
    w2 = 1.0 - w0 - w1
    tol = -1e-9
    inside = good & (w0 >= tol) & (w1 >= tol) & (w2 >= tol)
    if not np.any(inside):
        return []
    xs = (w0 * a[:, 0] + w1 * b[:, 0] + w2 * c[:, 0])[inside]
    xs = np.sort(xs)
    # 相邻近重合的命中合并（共边重复计数）
    merged = []
    for value in xs:
        if merged and abs(value - merged[-1]) < 1e-6:
            continue
        merged.append(float(value))
    return merged


def format_intervals(merged, origin_x):
    if not merged:
        return "        (无实体)"
    parts = []
    for index in range(0, len(merged) - 1, 2):
        parts.append(f"x[{merged[index] + origin_x:+.4f},{merged[index + 1] + origin_x:+.4f}]"
                     f"({(merged[index + 1] - merged[index]) * 1000:.0f}mm)")
    tail = f" 尾单点 {merged[-1] + origin_x:+.4f}" if len(merged) % 2 else ""
    return " ".join(parts) + tail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", default="xczs_inspection_robot_description/meshes/"
                                    "scenes/electrical_mezzanine/b1.STL")
    ap.add_argument("--z", type=float, default=0.120,
                    help="扫描所在的 b1 局部 z（0.120 = 把手中心）")
    ap.add_argument("--y", type=float, default=0.060,
                    help="扫描经过的 b1 局部 y（0.060 = 左立筋中心，0.646 = 右）")
    ap.add_argument("--span", type=float, default=0.060,
                    help="以 --y 为中心、两侧各扫多少 y")
    ap.add_argument("--step", type=float, default=0.004)
    args = ap.parse_args()

    tris = read_stl(args.mesh)
    print(f"mesh {args.mesh}  tris {len(tris)}  "
          f"local bbox x[{tris[...,0].min():+.4f},{tris[...,0].max():+.4f}] "
          f"y[{tris[...,1].min():+.4f},{tris[...,1].max():+.4f}] "
          f"z[{tris[...,2].min():+.4f},{tris[...,2].max():+.4f}]")
    print(f"b1 link 世界位 {B1_ORIGIN}，下方 x 已换算成世界坐标")
    print(f"z 局部 {args.z:+.4f} (世界 {args.z + B1_ORIGIN[2]:.4f})，"
          f"沿 +x 射线，y 步进 {args.step * 1000:.0f} mm\n")

    count = int(args.span / args.step)
    for index in range(-count, count + 1):
        y_local = args.y + index * args.step
        merged = ray_x_intervals(tris, y_local, args.z)
        print(f"  y {y_local:+.4f} (世界 {y_local + B1_ORIGIN[1]:.4f})  "
              f"{format_intervals(merged, B1_ORIGIN[0])}")


if __name__ == "__main__":
    main()
