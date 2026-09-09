#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""017c 贯通拉杆跨杆取证探针（cap2 驻留窗运行）——移杆裁决版（2026-09-09）。

双臂停在 db1 工作位姿、钩爪推进压贴时采样 TF，回答“两叉是否真跨住拉杆”：
拉杆上移 +24mm 后杆带 z 0.968..0.984（心 0.976）。实测叉缝不在叉心上下对称
（旧 HALF_GAP ±14mm 假定已证伪——缝全在杆轴/叉心上侧：L v_y 15.4..34.6mm、
R v_y 10.3..31.7mm，以叉尖为 0）。0.5mm 离线扫掠（STL 顶点 vs Ø16+0.6mm，
活验互证）给出每侧真跨杆窗：L 钩尖 z ∈ [0.950,0.952]、R 钩尖 z ∈ [0.953,0.957]。
叉尖 z 落窗内 → 杆在两齿间缝内不相撞；且尖 x ≤ 0.102（越杆前凸面 0.115、
贴/近背衬 0.099）才证明推进到位——两者皆真 = 真跨杆压贴。
插件抓握门（grasp_contact_threshold 0.02，3D，锚 grasp_point z0.952）与两窗
同兼容（窗中心距锚 1-3mm）。

角色（config-B 现役, 用户指令）:
  L 钩 = l_two_cyl_finger1 (端深 -0.075)   R 钩 = r_three_cyl_finger2 (端深 -0.075)
  L 支 = l_two_cyl_finger2 (端深 -0.095)   R 支 = r_three_cyl_finger1 (端深 -0.090)
  R 解锁 = r_three_cyl_finger3 (端深 -0.013)

用法: python3 probe_hook_straddle_017c.py [--every 1.0] [--max-samples 300]
  TF listener 由独立线程 spin;每周期采样一次并打印(flush),无输出积压。
"""
import argparse
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import tf2_ros

BAR = {  # 017c 拉杆几何（世界, 抽屉 closed 时）: 带 z（心 0.976 = 上移 +24mm 裁决）
    "z_lo": 0.968, "z_hi": 0.984, "x_front": 0.099, "x_proud": 0.115,
    "y_lo": 4.060, "y_hi": 4.740,
}
WINDOWS = {  # 每侧真跨杆窗（移杆后 0.5mm 离线扫掠, tip z 世界）: L/R 钩
    "L_hook": (0.950, 0.952),  # L 缝 v_y 15.4..34.6mm → 杆带相对尖 +17..+33mm
    "R_hook": (0.953, 0.957),  # R 缝 v_y 10.3..31.7mm → 杆带相对尖 +11..+31mm
}
PRESS_X = 0.102  # 尖 ≤ 此 x 才算越过杆前凸面 0.115 并贴/近背衬前缘 0.099
HOOKS = {
    "L_hook": ("l_two_cyl_finger1", 0.075),
    "R_hook": ("r_three_cyl_finger2", 0.075),
    "L_support": ("l_two_cyl_finger2", 0.095),
    "R_support": ("r_three_cyl_finger1", 0.090),
    "R_unlock": ("r_three_cyl_finger3", 0.013002),
}


def quat_rot(q, v):
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


class StraddleProbe(Node):
    def __init__(self):
        super().__init__("hook_straddle_probe_017c")
        self._buf = tf2_ros.Buffer()
        self._lst = tf2_ros.TransformListener(self._buf, self)

    def _root(self):
        # TF 根帧按已见帧列表回退: odom(默认) → map → world
        seen = self._buf.all_frames_as_string() or ""
        for f in ("odom", "map", "world"):
            if f in seen:
                return f
        return "odom"

    def sample(self, root):
        out = {}
        for name, (link, depth) in HOOKS.items():
            try:
                tf = self._buf.lookup_transform(
                    root, link, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
                t = tf.transform.translation
                q = (tf.transform.rotation.x, tf.transform.rotation.y,
                     tf.transform.rotation.z, tf.transform.rotation.w)
                # 杆系 Y 轴（叉分列轴）世界方向
                y_axis = quat_rot(q, (0.0, 1.0, 0.0))
                tip = quat_rot(q, (0.0, 0.0, -depth))
                out[name] = {
                    "link": link, "depth": depth,
                    "tip": (t.x + tip[0], t.y + tip[1], t.z + tip[2]),
                    "y_axis": y_axis,
                }
            except Exception as e:
                out[name] = {"link": link, "err": str(e)[:120]}
        return out

    def report(self, s, root, tick):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts} t={tick}] root={root} " + "; ".join(
            self._one(name, r) for name, r in s.items())
        print(line)
        sys.stdout.flush()

    def _one(self, name, r):
        if "err" in r:
            return f"{name}:ERR({r['err'][:40]})"
        tx, ty, tz = r["tip"]
        ya = r["y_axis"]
        w = WINDOWS.get(name)
        if w is None:
            # 支撑/解锁杆无跨杆窗,只报位（入缝/贴面由其它探针判）
            return f"{name}@({tx:.4f},{ty:.4f},{tz:.4f}) yz={ya[2]:+.3f}"
        lo, hi = w
        in_z = lo - 1e-6 <= tz <= hi + 1e-6
        pressed = tx <= PRESS_X
        ok = in_z and pressed
        tag = " STRADDLE_OK" if ok else " STRADDLE_FAIL"
        return (f"{name}@({tx:.4f},{ty:.4f},{tz:.4f}) yz={ya[2]:+.3f} "
                f"z-w[{lo:.3f},{hi:.3f}]{'IN' if in_z else 'OUT'} "
                f"x<={PRESS_X:.3f}{'Y' if pressed else 'N'}{tag}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--every", type=float, default=1.0)
    ap.add_argument("--max-samples", type=int, default=300)
    args = ap.parse_args()

    rclpy.init()
    node = StraddleProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    root = None
    try:
        for tick in range(args.max_samples):
            if root is None or tick % 30 == 0:
                root = node._root()
            s = node.sample(root)
            if all("err" not in r for r in s.values()) or tick % 3 == 0:
                node.report(s, root, tick)
            time.sleep(args.every)
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
