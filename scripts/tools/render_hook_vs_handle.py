#!/usr/bin/env python3
"""把「钩爪」和「把手」按真实相对位姿叠在一起渲染，回答一个只靠数字吵不清的问题：

    钩爪的叉口到底能不能绕过把手立板、卡到立板背面？

做法：读真实 STL（b1.STL 的把手区域 + endlink1.STL 钩爪），按 adapter 的工作
位姿（tool_roll_offset、钩杆落点 y=4.693、杆端压在立板前面）摆好，再正交投影
出三个视图。**前视图（沿 -x 看，即钩爪逼近方向）就是判定图** —— 叉口开在哪个
世界轴、净空多大、立板多宽，一眼可见。

用法：
  python3 scripts/tools/render_hook_vs_handle.py --roll-deg 0   --out /tmp/wrap_r0
  python3 scripts/tools/render_hook_vs_handle.py --roll-deg 90  --out /tmp/wrap_r90
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


# --- 世界坐标下的把手合同盒（与 fix#25 的 xacro collision 一一对应）---------
# b1 link 世界位姿 (0.03, 4.047, 0.831870606)，右把手在 b1 局部 y=0.646。
B1_ORIGIN = np.array([0.03, 4.047, 0.831870606])
RIGHT_RING_Y = 4.047 + 0.646  # 4.693

def ring_boxes():
    """右把手的六个合同盒：前立板 / 后立板 / 上梁 / 下梁（各 x,y,z 半尺寸）。"""
    def box(centre_local, half):
        return np.array(centre_local) + B1_ORIGIN, np.array(half)
    boxes = [
        # 前立板 flange: x 局部 0.065±0.004, y 0.646±0.008, z 0.120±0.048
        box((0.065, 0.646, 0.120), (0.004, 0.008, 0.048)),
        # 后立板 riser: x 0.004±0.004, z 0.120±0.040
        box((0.004, 0.646, 0.120), (0.004, 0.008, 0.040)),
        # 上梁: x 0.038..0.091, z 0.9959..1.0039
        box((0.0645, 0.646, 0.160), (0.0265, 0.008, 0.004)),
        # 下梁: z 0.9079..0.9159
        box((0.0645, 0.646, 0.080), (0.0265, 0.008, 0.004)),
    ]
    return boxes


def claw_to_world(tris, roll_deg, origin_x, origin_y, origin_z):
    """把钩爪局部坐标搬到世界：局部 x -> (cos,sin) 于 (y,z)，局部 y -> (-sin,cos)，局部 z -> x。

    roll_deg=0 即 adapter 现在的 tool_roll_offset(-90deg) 状态。
    """
    phi = np.deg2rad(roll_deg)
    c, s = np.cos(phi), np.sin(phi)
    # 局部 (a,b,cc) -> 世界偏移 (cc, a*c - b*s, a*s + b*c)
    a, b, cc = tris[..., 0], tris[..., 1], tris[..., 2]
    out = np.empty_like(tris)
    out[..., 0] = cc + origin_x
    out[..., 1] = a * c - b * s + origin_y
    out[..., 2] = a * s + b * c + origin_z
    return out


def draw(ax, tris, i, j, colour, alpha, linewidth, label_i, label_j, title):
    for tri in tris:
        pts = tri[:, [i, j]]
        ax.fill(pts[:, 0], pts[:, 1], facecolor=colour, edgecolor=colour,
                alpha=alpha, linewidth=linewidth)
    ax.set_title(title)
    ax.set_xlabel(label_i)
    ax.set_ylabel(label_j)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, linewidth=0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roll-deg", type=float, default=0.0,
                    help="0 = adapter 当前 roll；90 = 叉口转到世界 y")
    ap.add_argument("--out", default="/tmp/hook_vs_handle")
    ap.add_argument("--tip-x", type=float, default=0.097,
                    help="杆端（局部 z=-0.075）落在的世界 x")
    ap.add_argument("--tip-z", type=float, default=0.9519,
                    help="钩杆轴心的世界 z")
    ap.add_argument("--hook-y", type=float, default=RIGHT_RING_Y,
                    help="钩杆轴心（局部 x=-0.034）落在的世界 y")
    args = ap.parse_args()

    drawer = read_stl("xczs_inspection_robot_description/meshes/scenes/"
                      "electrical_mezzanine/b1.STL")
    claw = read_stl("xczs_inspection_robot_description/meshes/tools/"
                    "two_cylinder/endlink1.STL")

    # b1.STL 是 b1 link 的局部网格，先搬到世界
    drawer = drawer + B1_ORIGIN
    # 只留把手附近的抽屉面片，图才看得清
    lo = np.array([-0.02, 4.63, 0.86])
    hi = np.array([0.12, 4.76, 1.05])
    keep = np.all((drawer.reshape(-1, 3) > lo) &
                  (drawer.reshape(-1, 3) < hi), axis=1).reshape(-1, 3).all(axis=1)
    drawer = drawer[keep]

    # endlink1.STL 画在自己的杆轴系里（杆轴 = 局部 z 轴）。它在 tool base 系里的
    # 位置 = 手指 link 原点 [-0.034, -0.061735, -0.2935]（adapter 的
    # gripper_contact_point_base_at_zero [-0.034,-0.061735,-0.3685] 减去局部
    # 接触点 [0,0,-0.075] 得到）。
    claw = claw + np.array([-0.034, -0.061735, -0.2935])
    # 杆端（base 系 [-0.034,-0.061735,-0.3685]）落在 tip_x；杆轴落在 (hook_y, tip_z)
    phi = np.deg2rad(args.roll_deg)
    c, s = np.cos(phi), np.sin(phi)
    axis_local = np.array([-0.034, -0.061735])
    origin_y = args.hook_y - (axis_local[0] * c - axis_local[1] * s)
    origin_z = args.tip_z - (axis_local[0] * s + axis_local[1] * c)
    origin_x = args.tip_x + 0.3685
    claw_w = claw_to_world(claw, args.roll_deg, origin_x, origin_y, origin_z)

    print(f"roll {args.roll_deg:+.0f} deg  ->  tool origin "
          f"({origin_x:+.4f}, {origin_y:+.4f}, {origin_z:+.4f})")
    for name, arr in (("claw", claw_w), ("drawer", drawer)):
        v = arr.reshape(-1, 3)
        print(f"  {name:6s} world bbox "
              f"x[{v[:,0].min():+.4f},{v[:,0].max():+.4f}] "
              f"y[{v[:,1].min():+.4f},{v[:,1].max():+.4f}] "
              f"z[{v[:,2].min():+.4f},{v[:,2].max():+.4f}]")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    boxes = ring_boxes()
    views = [
        ("front: look -x  (approach dir)", 1, 2, "Y", "Z"),
        ("top:   look -z", 1, 0, "Y", "X"),
        ("side:  look -y", 2, 0, "Z", "X"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(19, 6))
    for ax, (title, i, j, li, lj) in zip(axes, views):
        for centre, half in boxes:
            xs = [centre[i] - half[i], centre[i] + half[i]]
            ys = [centre[j] - half[j], centre[j] + half[j]]
            ax.fill([xs[0], xs[1], xs[1], xs[0]],
                    [ys[0], ys[0], ys[1], ys[1]],
                    facecolor="#d8d8d8", edgecolor="#8a8a8a",
                    linewidth=0.6, alpha=0.9)
        draw(ax, drawer, i, j, "#b8c4d0", 0.5, 0.2, li, lj, title)
        draw(ax, claw_w, i, j, "#1f4fd8", 0.85, 0.3, li, lj, title)
    fig.suptitle(
        f"claw endlink1 vs right handle ring — roll {args.roll_deg:+.0f} deg "
        f"(adapter value = 0 deg); tip at world x={args.tip_x:.3f}")
    fig.tight_layout()
    fig.savefig(args.out + ".png", dpi=110)
    print(f"wrote {args.out}.png")


if __name__ == "__main__":
    main()
