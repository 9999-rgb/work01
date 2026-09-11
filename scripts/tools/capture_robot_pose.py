#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取机器人当前位姿并存成 YAML（现场教学标定位姿用）。

一次抓取包含四部分，缺一不可——单有臂关节角没法复现，因为同一个末端位姿在
7 自由度冗余臂上有多组解（实测同一 TCP 不同次运行 l_arm_0 可为 -1.234 或
-2.468 rad，TCP 差 < 0.1 mm）；反过来只有 TF 也没法直接下发。所以关节角、
工具坐标系、底盘、抽屉状态一起存。

  1. joints        全部关节名→位置（含 19 个臂/电缸关节；含轮子，便于事后判
                   断底盘有没有被挪过）
  2. frames        odom → 底盘 / 双臂末端 / 全部工具连杆 的位姿
  3. base          odom → body
  4. controls      各可控机构（抽屉等）的 state_id / position / effort

用法：
  python3 scripts/tools/capture_robot_pose.py --out /tmp/pose.yaml --label "下压后"
  python3 scripts/tools/capture_robot_pose.py --label "x" --stdout   # 只打印不落盘

先决条件：活栈已就绪。本工具只读，不下发任何指令。
"""
import argparse
import sys
import threading
import time

import rclpy
import rclpy.executors
import tf2_ros
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy)
from rclpy.time import Time
from sensor_msgs.msg import JointState

from xczs_inspection_robot_interfaces.msg import CabinetControlState

WORLD_FRAME = "odom"
JOINT_STATES_TOPIC = "/xczs/joint_states"
NAMESPACE = "/xczs/cabinet/electrical_mezzanine"
# 关注的可控机构（抽屉）；要扩到别的场景在这里加即可。
CONTROLS = ("db1",)
# 关心位姿的连杆：底盘、双臂末端、全部工具杆（按前缀扫 TF，见 collect_frames）。
KEY_FRAMES = ("body", "l_arm_6", "r_arm_6", "l_two_cyl_base", "r_three_cyl_base")
TOOL_FRAME_MARKER = "cyl"


class PoseCapture(Node):
    def __init__(self):
        super().__init__("capture_robot_pose", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self._joint_state = None
        # joint_states 是 best-effort 发布，默认 RELIABLE 订阅收不到消息。
        state_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._state_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state, state_qos)
        self._controls = {}
        for control in CONTROLS:
            self.create_subscription(
                CabinetControlState, "%s/%s/state" % (NAMESPACE, control),
                self._make_control_cb(control), 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # rclpy Node.executor 只存弱引用：必须用不同名强属性保住 executor，
        # 否则立即被 GC，回调全停（与 db1_stage_cap_driver.py 同一坑）。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)

    # ------------------------------------------------------------------ infra
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _make_control_cb(self, control):
        def callback(msg):
            self._controls[control] = msg
        return callback

    def _wait(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        raise RuntimeError("timeout waiting for %s" % what)

    def _lookup(self, frame):
        transform = self._tf_buffer.lookup_transform(
            WORLD_FRAME, frame, Time(), Duration(seconds=1.0))
        p = transform.transform.translation
        q = transform.transform.rotation
        return {
            "position": [round(p.x, 6), round(p.y, 6), round(p.z, 6)],
            "orientation_xyzw": [round(q.x, 6), round(q.y, 6),
                                 round(q.z, 6), round(q.w, 6)],
        }

    def _tf_frames(self):
        try:
            return sorted(yaml.safe_load(
                self._tf_buffer.all_frames_as_yaml()).keys())
        except Exception:  # noqa: BLE001
            return []

    def _tool_frames(self):
        """TF 树里**除 KEY_FRAMES 之外**的杆坐标系。

        判据必须排除 KEY_FRAMES：`l_two_cyl_base` 本身就含 "cyl"，拿它当"杆帧
        已到齐"的信号会在第一个轮询就误判通过，等于没等（2026-09-11 实测踩过，
        抓出来的存档只剩 5 个坐标系）。
        """
        return [f for f in self._tf_frames()
                if TOOL_FRAME_MARKER in f and f not in KEY_FRAMES]

    def _wait_for_tool_frames(self, timeout=15.0):
        """等 TF 树里出现工具杆坐标系再抓。

        `all_frames_as_yaml()` 只列**已经收到过**的帧：刚起节点就抓，动态帧
        还在路上，结果会把工具杆整批丢掉（而"记成 error"的兜底根本轮不到，
        因为它们连列表都没进）。所以先等，等不到才降级。
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._tool_frames():
                return True
            time.sleep(0.2)
        return False

    def collect_frames(self):
        """KEY_FRAMES + TF 树上所有工具连杆。缺失的帧记成 error，不静默跳过。"""
        if not self._wait_for_tool_frames():
            print("警告: %.0f s 内 TF 树未出现含 %r 的工具杆帧，本次抓取会缺这些"
                  "坐标系。" % (15.0, TOOL_FRAME_MARKER))
        wanted = list(KEY_FRAMES) + self._tool_frames()
        frames = {}
        for frame in wanted:
            try:
                frames[frame] = self._lookup(frame)
            except Exception as exc:  # noqa: BLE001
                frames[frame] = {"error": str(exc)}
        return frames

    def capture(self, label):
        self._wait(lambda: self._joint_state is not None, 20.0,
                   JOINT_STATES_TOPIC)
        self._wait(lambda: any(c in self._controls for c in CONTROLS), 20.0,
                   "control state topics")
        state = self._joint_state
        payload = {
            "label": label,
            "captured_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "world_frame": WORLD_FRAME,
            "joints": {
                name: round(float(position), 6)
                for name, position in zip(state.name, state.position)
            },
            "joint_efforts": {
                name: round(float(effort), 4)
                for name, effort in zip(state.name, state.effort)
            },
            "frames": self.collect_frames(),
            "controls": {
                control: {
                    "state_id": msg.state_id,
                    "position": round(msg.position, 6),
                    "effort": round(msg.effort, 4),
                    "in_motion": bool(msg.in_motion),
                }
                for control, msg in sorted(self._controls.items())
            },
        }
        return payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=None, help="输出 YAML 路径")
    parser.add_argument("--label", default="", help="这个位姿的名字/备注")
    parser.add_argument("--stdout", action="store_true", help="只打印不写文件")
    args = parser.parse_args()
    if not args.out and not args.stdout:
        parser.error("需要 --out 或 --stdout")

    rclpy.init()
    node = PoseCapture()
    try:
        payload = node.capture(args.label)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    text = yaml.safe_dump(payload, allow_unicode=True, sort_keys=False,
                          default_flow_style=False)
    if args.stdout:
        sys.stdout.write(text)
    if args.out and not args.stdout:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write("# 机器人标定位姿（capture_robot_pose.py 抓取）\n")
            handle.write(text)
        print("已写入 %s（%d 个关节，%d 个坐标系，%d 个机构）"
              % (args.out, len(payload["joints"]), len(payload["frames"]),
                 len(payload["controls"])))


if __name__ == "__main__":
    main()
