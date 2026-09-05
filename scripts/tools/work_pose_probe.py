#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Probe the tool-link poses at the db1 work pose during a cap run.

Waits for the operator's "arrival dwell active" line in the console log (the
bimanual work-pose dwell), then samples map -> {l_two_cyl_base, r_three_cyl_base}
plus rod joint states for the dwell + seal windows.  This answers where the
hook/support rod tips actually sit relative to the handle plate at press time.

Tip world pose is computed offline as: base_link ⊗ (base_at_zero − q*ẑ_base).
"""
import argparse
import time
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from sensor_msgs.msg import JointState
from tf2_ros import LookupException, ExtrapolationException

ARMS = [f"l_arm_{i}_joint" for i in range(7)] + \
       [f"r_arm_{i}_joint" for i in range(7)]
RODS = ["l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
        "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint"]


class WorkPoseProbe(Node):
    def __init__(self, out_path, console_log):
        super().__init__("work_pose_probe")
        self._out = open(out_path, "w", buffering=1)
        self._lock = threading.Lock()
        self._pos = {}
        self._tfbuf = tf2_ros.Buffer()
        self._tf = tf2_ros.TransformListener(self._tfbuf, self)
        self._js = self.create_subscription(
            JointState, "/xczs/joint_states", self._on_js, 50)
        # 控制面：等 console 出现 dwell 行 → 采样 work-pose 20 s
        self._spin_executor = rclpy.executors.SingleThreadedExecutor()
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()
        self._console = console_log
        self._seen = False

    def _on_js(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in RODS or name in ARMS:
                    self._pos[name] = msg.position[i]

    def _wait_dwell(self, timeout_s):
        end = time.time() + timeout_s
        marker = "arrival dwell active"
        while time.time() < end:
            try:
                with open(self._console, "rb") as f:
                    data = f.read()
                if marker in data.decode("utf-8", "replace"):
                    return True
            except OSError:
                pass
            time.sleep(0.5)
        return False

    def sample(self, seconds):
        """Sample map -> base-link poses + rod joints at 20 Hz."""
        with self._lock:
            stamp = self.get_clock().now().to_msg()
            pos = dict(self._pos)
        # 用 lookup_transform 拿 pose（含 map->odom 修正）
        poses = {}
        for link in ("l_two_cyl_base", "r_three_cyl_base"):
            try:
                t = self._tfbuf.lookup_transform(
                    "map", link, rclpy.time.Time(), timeout=Duration(seconds=1.0))
                poses[link] = (t.transform.translation, t.transform.rotation)
            except (LookupException, ExtrapolationException) as e:
                poses[link] = None
        with self._lock:
            now = time.monotonic()
            for link, p in poses.items():
                if p is None:
                    self._out.write(f"{now:9.2f} {link} NONE\n")
                    continue
                tr, quat = p
                self._out.write(
                    f"{now:9.2f} {link} {tr.x:.6f} {tr.y:.6f} {tr.z:.6f} "
                    f"{quat.x:.5f} {quat.y:.5f} {quat.z:.5f} {quat.w:.5f}\n")
            rod = " ".join(f"{self._pos.get(r, float('nan')):.5f}" for r in RODS)
            arm = " ".join(f"{self._pos.get(r, float('nan')):.5f}" for r in ARMS)
            self._out.write(f"{now:9.2f} JOINTS {arm} | {rod}\n")
            self._out.flush()

    def run(self, dwell_timeout_s, sample_s):
        if not self._wait_dwell(dwell_timeout_s):
            self._out.write("NO_DWELL_MARKER\n")
            self._out.close()
            return
        t_end = time.monotonic() + sample_s
        while time.monotonic() < t_end:
            self.sample(0.05)
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/work_pose_probe.log")
    parser.add_argument("--console", default="/tmp/xczs_restart_v11_console.log")
    parser.add_argument("--dwell-wait", type=float, default=600.0)
    parser.add_argument("--sample", type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    probe = WorkPoseProbe(args.out, args.console)
    try:
        probe.run(args.dwell_wait, args.sample)
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
