#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电缸杆行程微动（抽屉现场教学用）。

一次只动**一根**杆，同侧其余电缸保持实测当前值不动，机械臂一个关节都不碰。
杆的角色与关节名来自场景适配 YAML 的 drawer_tools 段（跨层合同），不在本脚本里
重复硬编码；`--fraction` 的分母就是合同里的 `*_contact_point_local` 的 z 分量，
即"杆自移动连杆原点伸出的长度"——和现场说"钩爪长度/支撑杆长度"是同一个数。

  钩爪（gripper） 左 `l_two_cyl_finger1`(75 mm)  右 `r_three_cyl_finger2`(75 mm)
  支撑杆（support）左 `l_two_cyl_finger2`(95 mm)  右 `r_three_cyl_finger1`(90 mm)

直接下发到电缸控制器（two_cylinder / three_cylinder 的 follow_joint_trajectory），
不经 MoveIt、不经 operator。注意：**因此没有碰撞检查**，杆顶到柜体/面板时会被
物理挡停，实测行程会小于指令行程——这正是现场要观察的接触现象，不是故障。

用法：
  # 两侧支撑杆各向前伸出自身长度的 1/5（左 19 mm / 右 18 mm）
  python3 scripts/tools/nudge_rod_stroke.py --rod support --fraction 0.2
  # 两侧钩爪各向前伸出 18.75 mm（= 75 mm 的 1/4）
  python3 scripts/tools/nudge_rod_stroke.py --rod gripper --extend 0.01875
  # 只动左手、缩回去
  python3 scripts/tools/nudge_rod_stroke.py --rod support --fraction -0.2 --side left
  # 只算不动
  python3 scripts/tools/nudge_rod_stroke.py --rod support --fraction 0.2 --dry-run

先决条件：活栈已就绪、电缸控制器可用。行程上限 0.12 m，越界自动钳位并提示。
"""
import argparse
import sys
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xczs_controllers import (  # noqa: E402
    JOINT_STATES_TOPIC, ROD_SIDES, SpinNode, best_effort_qos, wait_for)

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_ADAPTER = (
    WORKSPACE / "xczs_inspection_robot_control" / "config" / "scene_controls"
    / "electrical_mezzanine_adapter.yaml"
)
ROLES = ("gripper", "support")
STROKE_LIMIT = 0.12  # 与 URDF prismatic joint limit 一致


def find_drawer_tools(node):
    """在 adapter 里递归找 drawer_tools 段。

    该段实际位于 /**/xczs_cabinet_button_operator/ros__parameters/ 下，但层级
    属于 ROS 参数文件的组织方式、不是跨层合同的一部分，写死路径会在别人调整
    节点名时静默失配，所以按名字找。
    """
    if isinstance(node, dict):
        if "drawer_tools" in node:
            return node["drawer_tools"]
        for value in node.values():
            found = find_drawer_tools(value)
            if found:
                return found
    return None


def load_rod_contract(adapter_path, control_id):
    """从 adapter 取每侧每个角色的 (关节名, 杆长)。

    杆长 = |contact_point_local 的 z 分量|，即杆自移动连杆原点伸出的长度。
    """
    with open(adapter_path, "r", encoding="utf-8") as handle:
        adapter = yaml.safe_load(handle)
    tools = find_drawer_tools(adapter)
    if not tools:
        raise RuntimeError("%s 里找不到 drawer_tools" % adapter_path)
    contract = {}
    for side in ROD_SIDES:
        if side not in tools:
            raise RuntimeError("adapter 缺少 drawer_tools.%s" % side)
        contract[side] = {}
        for role in ROLES:
            entry = tools[side].get("%s_joint" % role)
            point = tools[side].get("%s_contact_point_local" % role)
            if not entry or not point:
                raise RuntimeError(
                    "adapter 缺少 drawer_tools.%s.%s_joint / _contact_point_local"
                    % (side, role))
            contract[side][role] = (entry, abs(float(point[2])))
    return contract


class RodStrokeNudge(SpinNode):
    def __init__(self):
        super().__init__("rod_stroke_nudge", num_threads=3)
        self._joint_state = None
        self._state_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state,
            best_effort_qos())
        self._stroke_clients = {
            side: ActionClient(self, FollowJointTrajectory, cfg["action"])
            for side, cfg in ROD_SIDES.items()
        }

    # ------------------------------------------------------------------ infra
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _measured(self, joint):
        if self._joint_state is None:
            raise RuntimeError("%s not received yet" % JOINT_STATES_TOPIC)
        try:
            index = list(self._joint_state.name).index(joint)
        except ValueError:
            raise RuntimeError("joint %s missing from %s"
                               % (joint, JOINT_STATES_TOPIC))
        return self._joint_state.position[index]

    def _settled(self, joints, timeout=15.0, tolerance=0.0002, window=0.5):
        """等这些关节连续 window 秒内的变动都小于 tolerance。

        控制器 action 返回 ≠ 关节到位：同控制器下各杆收尾速度不同（实测钩爪
        左杆比右杆慢数十毫秒量级），固定延时后取样会把"还在收尾"误报成
        "落点误差"。
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            first = {j: self._measured(j) for j in joints}
            time.sleep(window)
            if all(abs(self._measured(j) - first[j]) < tolerance
                   for j in joints):
                return True
        return False

    # ------------------------------------------------------------- execution
    def command(self, side, targets, duration):
        """targets: {joint_name: position}，一次轨迹点下发（无中间插补）。"""
        cfg = ROD_SIDES[side]
        joints = list(cfg["joints"])
        trajectory = JointTrajectory()
        trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [targets[j] for j in joints]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]

        client = self._stroke_clients[side]
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("%s action server unavailable" % cfg["action"])
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = 5
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("%s rejected the trajectory goal" % side)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + max(30.0, duration * 5.0)
        while rclpy.ok() and not result_future.done() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            raise RuntimeError("%s trajectory did not finish" % side)
        return result_future.result().result

    # ------------------------------------------------------------------- main
    def run(self, role, contract, offsets, dry_run, duration, step):
        wait_for(lambda: self._joint_state is not None, 15.0,
                 JOINT_STATES_TOPIC)
        print("\n==== 电缸杆行程微动（%s）====" % role)
        targets = {}
        before = {}
        for side, offset in offsets.items():
            joint = contract[side][role][0]
            current = self._measured(joint)
            before[side] = current
            wanted = current + offset
            clamped = max(0.0, min(STROKE_LIMIT, wanted))
            note = "" if abs(clamped - wanted) < 1e-9 else \
                "  ← 越界钳位到 [0, %.3f]" % STROKE_LIMIT
            print("%-5s %-26s %.5f -> %.5f m  (行程 %+.5f)%s"
                  % (side, joint, current, clamped, clamped - current, note))
            targets[side] = {joint: clamped}
            for other in ROD_SIDES[side]["joints"]:
                if other == joint:
                    continue
                value = self._measured(other)
                targets[side][other] = value
                print("      保持 %-26s %.5f m" % (other, value))

        if dry_run:
            print("--dry-run：未下发。")
            return

        stalled = {}
        if step and step > 0.0:
            # 分步推进：每步重新以实测值为基准下发，走不动就停。
            # 杆顶到柜体/墙体时会被物理挡停，一次发满行程会硬压进刚性体里
            # （求解器上表现为大接触力），所以"伸长抵住"要一步步试探。
            for side in targets:
                joint = contract[side][role][0]
                stalled[side] = self._advance(
                    side, role, joint, targets[side], before[side],
                    offsets[side], step, duration)
        else:
            for side, values in targets.items():
                print("下发 %-5s ..." % side)
                result = self.command(side, values, duration)
                print("%-5s 控制器结果 error_code=%s" % (side, result.error_code))
            for side in targets:
                stalled[side] = None

        all_joints = [contract[s][role][0] for s in targets] + \
            [j for s in targets for j in ROD_SIDES[s]["joints"]
             if j != contract[s][role][0]]
        if not self._settled(all_joints):
            print("警告: 15 s 内未观测到全部电缸静止，下面的实测值可能仍在动。")
        print("\n---- 实测行程 ----")
        for side, offset in offsets.items():
            joint = contract[side][role][0]
            after = self._measured(joint)
            short = (after - before[side]) - offset
            verdict = ""
            if stalled.get(side):
                verdict = "  ← 顶住：走到 %.2f mm 就不再前进" % (
                    (after - before[side]) * 1000)
            elif short < -0.0005:
                verdict = "  ← 比指令短 %.2f mm（未走完）" % (-short * 1000)
            print("%-5s %-26s %.5f -> %.5f m  实测行程 %+.5f m"
                  "  (指令 %+.5f m, 误差 %+.2f mm)%s"
                  % (side, joint, before[side], after, after - before[side],
                     offset, (after - targets[side][joint]) * 1000.0, verdict))
        print("=============================")

    def _advance(self, side, role, joint, target_map, start, offset, step,
                 duration):
        """分步推进到 start+offset。返回 True 表示中途顶住而提前停止。

        判据：单步指令 step，实测只走出 < 40% 即认为被挡停。阈值取 40% 是留出
        接触初期"压紧"的余地——完全用 0 判据会把启动迟滞误判成顶住。
        """
        remaining = offset
        position = start
        while abs(remaining) > 1e-6:
            this = max(-step, min(step, remaining))
            wanted = max(0.0, min(STROKE_LIMIT, position + this))
            if abs(wanted - position) < 1e-6:
                print("%-5s 已到行程边界 %.5f m，停止推进。" % (side, wanted))
                return True
            point = dict(target_map)
            point[joint] = wanted
            result = self.command(side, point, duration)
            if result.error_code != 0:
                print("%-5s 第 %+.1f mm 步控制器返回 error_code=%s，停止推进。"
                      % (side, this * 1000, result.error_code))
                return True
            self._settled([joint], timeout=8.0)
            moved = self._measured(joint) - position
            print("%-5s 步进 %+.2f mm -> 实测 %+.2f mm (目标 %.5f m)%s"
                  % (side, this * 1000, moved * 1000, wanted,
                     "" if abs(moved) >= abs(this) * 0.4 else "   ← 顶住"))
            if abs(moved) < abs(this) * 0.4:
                return True
            position = self._measured(joint)
            remaining -= moved
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rod", choices=list(ROLES), default="gripper",
                        help="动哪根杆：gripper=钩爪，support=支撑杆（默认 gripper）")
    amount = parser.add_mutually_exclusive_group(required=True)
    amount.add_argument("--extend", type=float,
                        help="行程增量（m，正 = 向前伸出，负 = 缩回），两侧同值")
    amount.add_argument("--fraction", type=float,
                        help="行程增量 = 该侧杆长的这个倍数（0.2 = 1/5）。"
                             "杆长取自 adapter 的 *_contact_point_local，两侧各自算")
    parser.add_argument("--side", choices=["left", "right", "both"],
                        default="both", help="只动哪一侧，默认两侧")
    parser.add_argument("--adapter", default=str(DEFAULT_ADAPTER),
                        help="场景适配 YAML 路径（取杆长用）")
    parser.add_argument("--control", default="db1",
                        help="抽屉控制 id，用于在 adapter 里定位（默认 db1）")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印当前值/目标值，不下发")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="轨迹时长（s），默认 3.0（放慢便于观察）")
    parser.add_argument("--step", type=float, default=0.0,
                        help="分步推进的步长（m），0 = 一次性发满行程（默认）。"
                             ">0 时每步以实测值为基准重发，实测走不出 40%% 即"
                             "判定顶住并停止——杆前方有柜体/墙体时用这个，"
                             "避免把杆硬压进刚性体")
    args = parser.parse_args()

    sides = ["left", "right"] if args.side == "both" else [args.side]
    contract = load_rod_contract(args.adapter, args.control)
    offsets = {}
    for side in sides:
        joint, length = contract[side][args.rod]
        if args.extend is not None:
            offsets[side] = args.extend
        else:
            offsets[side] = args.fraction * length
    if args.fraction is not None:
        for side in sides:
            _joint, length = contract[side][args.rod]
            print("%-5s 杆长 %.4f m × %.3f = %+.5f m"
                  % (side, length, args.fraction, args.fraction * length))

    rclpy.init()
    node = RodStrokeNudge()
    try:
        node.run(args.rod, contract, offsets, args.dry_run, args.duration,
                 args.step)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
