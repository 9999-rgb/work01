#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""db1 播放租约"崩溃 owner"模拟器（2026-09-07 AGENT T3 看门狗活验）。

故意 START 一段小行程播放调度后**不 RELEASE 直接退出**，模拟操作方在持有
SetCabinetPlayback 租约期间崩溃/被 SIGKILL 的现场——这正是把 db1 播放层永久占
锁（"owned by another operation lease"，直至 gzserver 重启）的泄漏源头。

插件侧看门狗（cabinet_state_plugin.cpp kPlaybackLeaseIdleTimeoutSeconds=60）应
在该调度 finished 后闲置 60s 时把会话按 RELEASE 相同路径收尾（闭位复闩 / 开位
idle-hold），使后续任何新租约能再次 START。用本脚本制造泄漏后：

  1) 观察 world log（/tmp/xczs_world.log）出现
     "abandoned (lease idle timeout)"
  2) 运行 db1_hold_probe.py relatch / open —— 必须成功（修复前此处永远被拒）

用法:
  python3 db1_playback_leak_probe.py            # 0 -> 0.06 小行程后退出
  python3 db1_playback_leak_probe.py --to 0.12  # 自定义泄漏停留轨位
"""
import argparse
import sys
import time
import uuid

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy)

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xczs_inspection_robot_interfaces.srv import SetCabinetPlayback
from xczs_inspection_robot_interfaces.msg import CabinetControlState

CONTROL_ID = "db1"
PLAYBACK_SERVICE = "/xczs/cabinet/electrical_mezzanine/playback"
STATE_TOPIC = "/xczs/cabinet/electrical_mezzanine/db1/state"


class LeakProbe(Node):
    def __init__(self):
        super().__init__("db1_playback_leak_probe")
        self.lease = "leak-" + uuid.uuid4().hex[:10]
        self.position = None
        self.state_id = None
        self.client = self.create_client(
            SetCabinetPlayback, PLAYBACK_SERVICE)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            CabinetControlState, STATE_TOPIC, self._on_state, qos)

    def _on_state(self, msg):
        self.position = msg.position
        self.state_id = msg.state_id

    def start_only(self, to_q, secs):
        """START 一段行程后返回；调用方刻意不 RELEASE 即退出。"""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.client.service_is_ready() and self.position is not None:
                break
        req = SetCabinetPlayback.Request()
        req.command = SetCabinetPlayback.Request.COMMAND_START
        req.control_id = CONTROL_ID
        req.operation_lease_id = self.lease
        req.trajectory.joint_names = [CONTROL_ID]
        n = max(2, int(secs * 10.0))
        for i in range(n + 1):
            t = secs * i / n
            p = JointTrajectoryPoint()
            p.time_from_start = Duration(seconds=t).to_msg()
            p.positions = [self.position + (to_q - self.position) * i / n]
            p.velocities = [0.0]
            req.trajectory.points.append(p)
        future = self.client.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        resp = future.result()
        print("START -> %.3f: success=%s lease=%s  %s" %
              (to_q, resp.success, self.lease,
               "" if resp.success else resp.message))
        if not resp.success:
            return False
        # 等调度跑完（finished），确认抽屉到位后**不 RELEASE 退出**。
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs + 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.position is not None and abs(self.position - to_q) < 0.006:
                break
        print("rail now=%.4f state=%s -- leaving playback lease '%s' held "
              "(simulated crashed owner); exiting WITHOUT RELEASE."
              % (self.position, self.state_id, self.lease))
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--to", type=float, default=0.06,
                        help="泄漏停留轨位 (m)，默认 0.06")
    parser.add_argument("--move-secs", type=float, default=1.5,
                        help="行程调度秒数")
    args = parser.parse_args()

    rclpy.init()
    probe = LeakProbe()
    try:
        ok = probe.start_only(args.to, args.move_secs)
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
