#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""db1 运动学播放探针（可视化快速落地 切片A 活验）。

直接调用电气夹层 operator 命名空间下的 /xczs/cabinet/electrical_mezzanine/playback
（SetCabinetPlayback，插件按 grasp_service_name 所在目录实例化注册），不经过
Web/operator 鉴权，只验证抽屉能否被运动学调度命令 开→等→关→释放，并用
db1/state（CabinetControlState.position，单位 m）读回验证。

用法:
  python3 db1_playback_probe.py open
  python3 db1_playback_probe.py close        # 从当前开位拉回
  python3 db1_playback_probe.py full         # 开→0.3→关→释放 完整链路
  python3 db1_playback_probe.py hold          # START 开 + HOLD 冻结 + 释放
默认 target 0.30 m / 行程 3.0 s，可用 --distance/--seconds 覆盖。
"""
import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy)
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xczs_inspection_robot_interfaces.srv import SetCabinetPlayback
from xczs_inspection_robot_interfaces.msg import CabinetControlState

CONTROL_ID = "db1"
PLAYBACK_SERVICE = "/xczs/cabinet/electrical_mezzanine/playback"
STATE_TOPIC = "/xczs/cabinet/electrical_mezzanine/db1/state"


class Db1PlaybackProbe(Node):
    def __init__(self, distance, seconds, samples_per_second):
        super().__init__("db1_playback_probe")
        self.distance = float(distance)
        self.seconds = float(seconds)
        self.sps = float(samples_per_second)
        self.lease = "visual-probe-" + uuid.uuid4().hex[:12]
        self.position = None
        self.stamp = None
        self.client = self.create_client(
            SetCabinetPlayback, PLAYBACK_SERVICE)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.state_sub = self.create_subscription(
            CabinetControlState, STATE_TOPIC, self.on_state, qos)

    def on_state(self, msg):
        self.position = msg.position
        self.stamp = msg.header.stamp

    def wait_service(self, timeout=10.0):
        if not self.client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("playback 服务不可用: %s" % PLAYBACK_SERVICE)

    def _schedule(self, start_q, end_q, seconds):
        traj = JointTrajectory()
        traj.joint_names = [CONTROL_ID]
        n = max(2, int(seconds * self.sps))
        for i in range(n + 1):
            t = seconds * i / n
            q = start_q + (end_q - start_q) * i / n
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=t).to_msg()
            point.positions = [q]
            point.velocities = [0.0]
            traj.points.append(point)
        return traj

    def call(self, command, start_q, end_q, seconds=None):
        request = SetCabinetPlayback.Request()
        request.command = command
        request.control_id = CONTROL_ID
        request.operation_lease_id = self.lease
        if seconds is None:
            seconds = self.seconds
        if command == SetCabinetPlayback.Request.COMMAND_START:
            request.trajectory = self._schedule(start_q, end_q, seconds)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done():
            raise RuntimeError("playback 请求超时")
        return future.result()

    def wait_position(self, target, tolerance, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.position is not None and \
                    abs(self.position - target) <= tolerance:
                return self.position
        raise RuntimeError(
            "position 未达 %.3f±%.3f（最后 %.4f）" %
            (target, tolerance, self.position))

    def open(self):
        now = self.position
        if now is None:
            raise RuntimeError("尚未收到 db1/state")
        return self.call(SetCabinetPlayback.Request.COMMAND_START,
                         now, self.distance)

    def close(self):
        now = self.position
        if now is None:
            raise RuntimeError("尚未收到 db1/state")
        return self.call(SetCabinetPlayback.Request.COMMAND_START,
                         now, 0.0)


def main():
    parser = argparse.ArgumentParser(description="db1 playback probe")
    parser.add_argument("action", nargs="?", default="full",
                        choices=["open", "close", "full", "hold"])
    parser.add_argument("--distance", type=float, default=0.30)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--sps", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = Db1PlaybackProbe(args.distance, args.seconds, args.sps)
    try:
        node.wait_service()
        # wait a first state sample
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.position is not None:
                break
        print("lease=%s  current=%.4f m" %
              (node.lease, node.position if node.position is not None else float("nan")))

        if args.action == "open":
            resp = node.open()
        elif args.action == "close":
            resp = node.close()
        elif args.action == "hold":
            resp = node.open()
            print("OPEN  resp success=%s pos=%.4f finished=%s msg=%s" %
                  (resp.success, resp.position, resp.finished, resp.message))
            node.wait_position(node.distance, 0.015)
            print("OPEN 到达 %.4f m" % node.position)
            hold = node.call(SetCabinetPlayback.Request.COMMAND_HOLD, 0.0, 0.0)
            print("HOLD  resp success=%s pos=%.4f msg=%s" %
                  (hold.success, hold.position, hold.message))
            frozen = node.position
            time.sleep(1.0)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
            rel = node.call(SetCabinetPlayback.Request.COMMAND_RELEASE, 0.0, 0.0)
            print("RELEASE resp success=%s pos=%.4f finished=%s msg=%s" %
                  (rel.success, rel.position, rel.finished, rel.message))
            print("HOLD 冻结位 %.4f -> 释放后 %.4f m（应几乎不变）" %
                  (frozen, node.position))
            return 0
        else:  # full
            resp = node.open()
            print("OPEN  resp success=%s pos=%.4f finished=%s msg=%s" %
                  (resp.success, resp.position, resp.finished, resp.message))
            node.wait_position(node.distance, 0.015)
            print("OPEN 到达 %.4f m" % node.position)
            resp2 = node.close()
            print("CLOSE resp success=%s pos=%.4f msg=%s" %
                  (resp2.success, resp2.position, resp2.message))
            node.wait_position(0.0, 0.012)
            print("CLOSE回到 %.4f m" % node.position)
            rel = node.call(SetCabinetPlayback.Request.COMMAND_RELEASE, 0.0, 0.0)
            print("RELEASE resp success=%s pos=%.4f finished=%s msg=%s" %
                  (rel.success, rel.position, rel.finished, rel.message))
            return 0

        print("%s resp success=%s pos=%.4f finished=%s msg=%s" %
              (args.action.upper(), resp.success, resp.position,
               resp.finished, resp.message))
        return 0 if resp.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
