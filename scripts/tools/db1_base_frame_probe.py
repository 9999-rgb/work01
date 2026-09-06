#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""db1 底盘命令帧标定探针（可视化后端排障用）。

在给定站位（preposition 或 spawn 位）对底盘发一串已知 body/base_link 系命令，
量化"命令轴 → 实测世界位移"的映射，判断物理 mover（gazebo_ros_planar_move，
robot_base_frame=body）到底按哪个系执行 /xczs/cmd_vel：

  - 若按 body：body +X 命令 → 世界位移 ≈ body +X 方向 (cos,sin)*speed*t。
  - 若按 base_link：body +X 命令 → 世界位移 ≈ base_link +X 方向（相差固定 −90°）。
  - 若命令后世界位移与命令轴投影一致且能停住，则系是稳定的，可标定旋转 R。

每条命令发完后先显式 stop + 长驻停，避免残余滑行污染下一条（实测 stop 后仍有
~0.4s 尾迹，故 settle 给足）。测量取 odom→body（mover 里程/控制同系）与
map→body（AMCL 真值）双份，便于判断 TF 是否被真值覆盖。

用法:
  python3 scripts/tools/db1_base_frame_probe.py          # 标定四向
  python3 scripts/tools/db1_base_frame_probe.py --once fwd   # fwd|back|left|right
"""
import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

CMD = 0.06
SECONDS = 2.0


class FrameProbe(Node):
    def __init__(self):
        super().__init__("db1_base_frame_probe",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(Twist, "/xczs/manual_cmd_vel", 10)
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self.ex.add_node(self)
        self._spin = threading.Thread(target=self.ex.spin, daemon=True)
        self._spin.start()

    def pose(self, frame, tries=20):
        last = None
        for _ in range(tries):
            try:
                t = self.buf.lookup_transform(frame, "body", Time(),
                                              Duration(seconds=1.0))
                tr = t.transform.translation
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
                return (tr.x, tr.y, yaw)
            except Exception as exc:  # noqa: BLE001
                last = exc
                time.sleep(0.2)
        raise last

    def bl_yaw(self, tries=20):
        last = None
        for _ in range(tries):
            try:
                t = self.buf.lookup_transform("odom", "base_link", Time(),
                                              Duration(seconds=1.0))
                q = t.transform.rotation
                return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
            except Exception as exc:  # noqa: BLE001
                last = exc
                time.sleep(0.2)
        raise last

    def poke(self, lx, ly):
        time.sleep(0.8)  # ensure prior residual stopped
        a_odom = self.pose("odom")
        a_map = self.pose("map")
        bl0 = self.bl_yaw()
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        for _ in range(int(SECONDS * 20.0)):
            self.pub.publish(msg)
            time.sleep(0.05)
        for _ in range(8):
            self.pub.publish(Twist())
            time.sleep(0.08)
        time.sleep(1.2)
        b_odom = self.pose("odom")
        b_map = self.pose("map")
        bl1 = self.bl_yaw()
        return (a_odom, b_odom, a_map, b_map, bl0, bl1)

    def report(self, label, a, b):
        ca, sa = math.cos(a[2]), math.sin(a[2])
        dx, dy = b[0] - a[0], b[1] - a[1]
        fwd = dx * ca + dy * sa          # 沿 body +X（起测 yaw）
        left = -dx * sa + dy * ca        # 沿 body +Y（起测 yaw）
        speed = math.hypot(dx, dy) / SECONDS
        bearing = math.degrees(math.atan2(dy, dx))
        return ("%s world d=(%+.4f,%+.4f) speed=%.3f m/s bearing=%.1f deg | "
                "body_fwd=%+.4f body_left=%+.4f dyaw=%+.3f rad"
                % (label, dx, dy, speed, bearing, fwd, left, b[2] - a[2]))

    def stop(self):
        try:
            self.ex.shutdown()
        except Exception:  # noqa: BLE001
            pass


def main():
    global CMD, SECONDS
    ap = argparse.ArgumentParser()
    ap.add_argument("--once", choices=["fwd", "back", "left", "right"], default=None)
    ap.add_argument("--cmd", type=float, default=CMD)
    ap.add_argument("--seconds", type=float, default=SECONDS)
    args = ap.parse_args()
    CMD, SECONDS = args.cmd, args.seconds

    rclpy.init()
    n = FrameProbe()
    try:
        o = n.pose("odom")
        m = n.pose("map")
        bl = n.bl_yaw()
        print("start odom->body (%.3f, %.3f) yaw=%.3f rad (%.1f deg)" %
              (o[0], o[1], o[2], math.degrees(o[2])))
        print("start map ->body (%.3f, %.3f) yaw=%.3f rad (%.1f deg)" %
              (m[0], m[1], m[2], math.degrees(m[2])))
        print("start base_link yaw=%.3f rad (%.1f deg); body-bl offset=%.1f deg\n" %
              (bl, math.degrees(bl), math.degrees(bl - o[2])))

        axes = [("fwd", CMD, 0.0), ("back", -CMD, 0.0),
                ("left", 0.0, CMD), ("right", 0.0, -CMD)]
        if args.once:
            axes = [a for a in axes if a[0] == args.once]
        for label, lx, ly in axes:
            a_odom, b_odom, a_map, b_map, bl0, bl1 = n.poke(lx, ly)
            print("CMD %-6s (lx=%+.2f, ly=%+.2f)" % (label, lx, ly))
            print("    " + n.report("odom", a_odom, b_odom))
            print("    " + n.report("map ", a_map, b_map))
    finally:
        n.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
