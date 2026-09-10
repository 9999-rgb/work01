#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""017d 前脸压贴取证探针（无杆基线重验, 2026-09-09）——cap1 驻留窗运行。

017c 贯通拉杆已按用户目视裁决移除（commit 2365fad），几何退回 016/017 基线
「钩爪自 +X 侧正面压贴前脸受力背衬」。本探针在 db1 cap1 单工作位姿就位期间
采样双臂钩爪/支撑杆尖端 TF，回答两个问题：
  1. 钩尖是否真贴前脸合同面 x≈0.099（而非 bar 时代的挡停 0.109 或 L 叉深入
     异常 ~0.03）——PRESS_OK: tip_x ≤ 0.103 且未随抽屉被推动；
  2. 抽屉是否被压开——由 driver 侧 drawer 位置/unsafe-movement 事件判（本
     探针只报尖端坐标，driver 输出互补）。
叉尖 x 判据 0.103 与插件 grasp_contact_threshold 0.02（锚 grasp_point 面
x0.099）同族；z 只报原值不做窗判（自对中 L≈0.951/R≈0.955，±2mm 内均合法）。

角色（config-B 现役, 用户指令）:
  L 钩 = l_two_cyl_finger1 (端深 -0.075)   R 钩 = r_three_cyl_finger2 (端深 -0.075)
  L 支 = l_two_cyl_finger2 (端深 -0.095)   R 支 = r_three_cyl_finger1 (端深 -0.090)
  R 解锁 = r_three_cyl_finger3 (端深 -0.013)

用法: python3 probe_hook_press_017d.py [--every 1.0] [--max-samples 300]
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

PRESS_X = 0.103  # 尖 ≤ 此 x 才算压上前脸面 0.099（016/017 压贴族 ≈0.099）
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


class PressProbe(Node):
    def __init__(self):
        super().__init__("hook_press_probe_017d")
        self._buf = tf2_ros.Buffer()
        self._lst = tf2_ros.TransformListener(self._buf, self)

    def _root(self):
        # TF 根帧必须优先 map: 柜体/把手合同点 (0.099, 4.107/4.693, 0.952) 定义在
        # map 下, 且 electrical_mezzanine_frame 在 map 下为单位位姿。用 odom 会
        # 得到随机器人漂移(且 boot 后可能被重设)的坐标, 事后无法换算 —— 2026-09-10
        # 实测: 探针 odom 采样 z≈2.24(TF 里工具实际在 0.95), 说明 odom 中途被重设,
        # 整段记录作废。故 map 优先, 缺失时才退回 world/odom。
        seen = self._buf.all_frames_as_string() or ""
        for f in ("map", "world", "odom"):
            if f in seen:
                return f
        return "map"

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
                tip = quat_rot(q, (0.0, 0.0, -depth))
                out[name] = {
                    "tip": (t.x + tip[0], t.y + tip[1], t.z + tip[2]),
                }
            except Exception as e:
                out[name] = {"err": str(e)[:120]}
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
        if name.endswith("_hook"):
            ok = tx <= PRESS_X
            tag = " PRESS_OK" if ok else " PRESS_FAIL"
            return (f"{name}@({tx:.4f},{ty:.4f},{tz:.4f}) "
                    f"x<={PRESS_X:.3f}{'Y' if ok else 'N'}{tag}")
        # 支撑/解锁杆无压贴判据,只报位
        return f"{name}@({tx:.4f},{ty:.4f},{tz:.4f})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--every", type=float, default=1.0)
    ap.add_argument("--max-samples", type=int, default=300)
    args = ap.parse_args()

    rclpy.init()
    node = PressProbe()
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
