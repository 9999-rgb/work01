#!/usr/bin/env python3
"""采样作业工具基座/杆端在世界系下的位姿与轴向（db1 钩爪动作设计用）。

背景（fix#25，2026-09-10）：把手立筋是**空心框**（b1.STL 前板 x0.061..0.069、
背板 x0.000..0.008、中间 53mm 通孔沿 ±y 通透）。要让钩爪「绕过去内收卡住」，
必须先知道杆端端部那个 55mm 侧爪在作业位姿下朝哪个世界方向、离把手立筋多远。
本探针按固定节奏把 map→各工具链的 TF 打印成 TSV，便于与杆端真值探针互证。

用法：
  python3 scripts/tools/probe_tool_frame_axes.py --out /tmp/tool_axes.tsv \
      --every 0.5 --seconds 200
每行：t <link>_px.._pz <link>_xx.._xz <link>_yx.._yz <link>_zx.._zz
（x/y/z_hat 为该 link 三轴在世界系下的单位向量；基座与 finger link 同姿态，
侧爪 = finger link 的 +Y 方向。）
"""

import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

LINKS = [
    "l_two_cyl_base", "l_two_cyl_finger1", "l_two_cyl_finger2",
    "r_three_cyl_base", "r_three_cyl_finger1", "r_three_cyl_finger2",
    "r_three_cyl_finger3",
]


class ToolFrameAxesProbe(Node):
    def __init__(self, every: float, seconds: float, out_path: str):
        super().__init__("xczs_tool_frame_axes_probe")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.every = every
        self.seconds = seconds
        self.out_path = out_path

    def run(self) -> int:
        deadline = time.time() + self.seconds
        rows = 0
        with open(self.out_path, "w", encoding="utf-8") as sink:
            sink.write("t\t" + "\t".join(
                [f"{l}_{f}" for l in LINKS
                 for f in ("px", "py", "pz",
                           "xx", "xy", "xz",
                           "yx", "yy", "yz",
                           "zx", "zy", "zz")]) + "\n")
            sink.flush()
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                cells = [f"{time.time():.3f}"]
                complete = True
                for link in LINKS:
                    try:
                        tr = self.buffer.lookup_transform(
                            "map", link, rclpy.time.Time())
                    except Exception:  # noqa: BLE001 - TF 尚未就绪
                        complete = False
                        break
                    q = tr.transform.rotation
                    x, y, z, w = q.x, q.y, q.z, q.w
                    rot = [
                        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                         2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                         2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w),
                         1 - 2 * (x * x + y * y)],
                    ]
                    t = tr.transform.translation
                    cells += [f"{t.x:.6f}", f"{t.y:.6f}", f"{t.z:.6f}"]
                    # 列向量 = 该轴在世界系下的方向
                    for col in range(3):
                        cells += [f"{rot[r][col]:.6f}" for r in range(3)]
                if complete:
                    sink.write("\t".join(cells) + "\n")
                    sink.flush()
                    rows += 1
                time.sleep(self.every)
        print(f"wrote {rows} rows to {self.out_path}")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/tool_axes.tsv")
    parser.add_argument("--every", type=float, default=0.5)
    parser.add_argument("--seconds", type=float, default=200.0)
    args = parser.parse_args()
    rclpy.init()
    node = ToolFrameAxesProbe(args.every, args.seconds, args.out)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
