#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""由参考位姿 + 平移量，自动求出一份新的教学位姿存档（不需要人工教学）。

为什么需要它：把一套教学好的位姿推广到**另一扇抽屉**时，两扇的把手高度/横向
位置不同。用"直线平移"去够是行不通的——实测要把手抬 0.57 m 时，笛卡尔路径
完整度从 1.0 掉到 0.14（那条直线上手臂够不着）。**改走 IK** 就没有这个限制：
把参考位姿里的末端位姿整体平移 (Δx, Δy, Δz)，交给 MoveIt 的 /compute_ik 解出
一组新的关节角，落到存档里即可。

做法：
  1. 读参考存档，取双臂末端 tip link 的世界位姿（位置 + 姿态）
  2. 目标位姿 = 参考位姿 **整体平移** (Δx, Δy, Δz)，**姿态照抄**（工具朝向不变）
  3. 对每侧调用 /compute_ik 求该 tip 位姿的关节角（seed 用参考位姿的关节角，
     尽量落在同一构型分支上）
  4. 电缸杆沿用参考存档的值（杆相对工具基座是固定的）
  5. 写出一份与 capture_robot_pose.py 同格式的存档，可直接喂给
     restore_robot_pose.py / 教学序列的 restore_pose 步骤

用法：
  python3 scripts/tools/derive_taught_pose.py \\
      --reference xczs_inspection_robot_control/config/taught_poses/db1_teach_node001_raise30.yaml \\
      --out xczs_inspection_robot_control/config/taught_poses/ds2_raised.yaml \\
      --dy 0.20 --dz 0.57 --label "ds2 举起位姿"

先决条件：活栈就绪；机器人**已在该抽屉的工位**（IK 用的是当前世界系）。
"""
import argparse
import sys
import time
from pathlib import Path

import rclpy
import rclpy.executors
import tf2_ros
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xczs_controllers import (  # noqa: E402
    ARMS, JOINT_STATES_TOPIC, WORLD_FRAME, SpinNode, best_effort_qos, wait_for)

IK_SERVICE = "/compute_ik"
ROD_JOINTS = ("l_two_cyl_finger1_joint", "l_two_cyl_finger2_joint",
              "r_three_cyl_finger1_joint", "r_three_cyl_finger2_joint",
              "r_three_cyl_finger3_joint")


class PoseDeriver(SpinNode):
    def __init__(self):
        super().__init__("derive_taught_pose", num_threads=4,
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._joint_state = None
        self.create_subscription(JointState, JOINT_STATES_TOPIC,
                                 self._on_joint_state, best_effort_qos())
        self._ik_cli = self.create_client(GetPositionIK, IK_SERVICE)

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _wait(self, predicate, timeout, what):
        wait_for(predicate, timeout, what)

    def tip_pose(self, tip):
        deadline = time.monotonic() + 10.0
        last = None
        while time.monotonic() < deadline:
            try:
                t = self._tf_buffer.lookup_transform(WORLD_FRAME, tip, Time(),
                                                     Duration(seconds=0.5))
                p, q = t.transform.translation, t.transform.rotation
                return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
            except Exception as exc:  # noqa: BLE001
                last = exc
                time.sleep(0.1)
        raise RuntimeError("TF %s→%s 不可用: %s" % (WORLD_FRAME, tip, last))

    def solve(self, group, tip, position, quat, seed):
        if not self._ik_cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("%s 不可用" % IK_SERVICE)
        request = GetPositionIK.Request()
        request.ik_request.group_name = group
        request.ik_request.ik_link_name = tip
        request.ik_request.pose_stamped.header.frame_id = WORLD_FRAME
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = position[0]
        request.ik_request.pose_stamped.pose.position.y = position[1]
        request.ik_request.pose_stamped.pose.position.z = position[2]
        request.ik_request.pose_stamped.pose.orientation.x = quat[0]
        request.ik_request.pose_stamped.pose.orientation.y = quat[1]
        request.ik_request.pose_stamped.pose.orientation.z = quat[2]
        request.ik_request.pose_stamped.pose.orientation.w = quat[3]
        request.ik_request.avoid_collisions = True
        # seed：用参考位姿的关节角，尽量落在同一构型分支
        seed_state = JointState()
        seed_state.name = list(seed.keys())
        seed_state.position = [float(v) for v in seed.values()]
        request.ik_request.robot_state.joint_state = seed_state
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        future = self._ik_cli.call_async(request)
        deadline = time.monotonic() + 20.0
        while self.context.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError("IK 未在 20s 内返回")
        response = future.result()
        if response.error_code.val != 1:
            raise RuntimeError("IK 失败，error_code=%s" % response.error_code.val)
        solved = response.solution.joint_state
        return {n: float(p) for n, p in zip(solved.name, solved.position)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, help="参考存档路径")
    parser.add_argument("--out", required=True, help="输出存档路径")
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    with open(args.reference, "r", encoding="utf-8") as handle:
        reference = yaml.safe_load(handle)
    ref_joints = {k: float(v) for k, v in reference["joints"].items()}

    rclpy.init()
    node = PoseDeriver()
    try:
        node._wait(lambda: node._joint_state is not None, 20.0, JOINT_STATES_TOPIC)
        joints = {}
        for side, cfg in ARMS.items():
            tip = cfg["tip"]
            position, quat = node.tip_pose(tip)
            # 参考存档里的 tip 位姿（存档时抓的），用它算"平移后应该在哪"
            ref_pos = reference["frames"][tip]["position"]
            target = (ref_pos[0] + args.dx, ref_pos[1] + args.dy, ref_pos[2] + args.dz)
            print("%-6s tip %-8s 实测 (%+.4f %+.4f %+.4f) → 目标 (%+.4f %+.4f %+.4f)"
                  % (side, tip, position[0], position[1], position[2],
                     target[0], target[1], target[2]))
            seed = {j: ref_joints[j] for j in cfg["joints"] if j in ref_joints}
            solved = node.solve(cfg["group"], tip, target, quat, seed)
            for joint in cfg["joints"]:
                joints[joint] = solved[joint]
        for joint in ROD_JOINTS:
            joints[joint] = ref_joints[joint]
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    payload = {
        "label": args.label or ("%s + (%+.3f %+.3f %+.3f)"
                                % (Path(args.reference).stem, args.dx, args.dy, args.dz)),
        "captured_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "world_frame": WORLD_FRAME,
        "derived_from": str(args.reference),
        "offset": {"dx": args.dx, "dy": args.dy, "dz": args.dz},
        "joints": joints,
        "joint_efforts": {j: 0.0 for j in joints},
        "frames": {},
        "controls": reference.get("controls", {}),
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write("# 由 derive_taught_pose.py 自动生成（IK 求得，非人工教学）\n")
        yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)
    print("已写入 %s（%d 个关节）" % (args.out, len(joints)))


if __name__ == "__main__":
    main()
