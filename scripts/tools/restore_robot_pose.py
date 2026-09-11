#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把机器人恢复到某个已存档的教学位姿（现场教学节点回退用）。

读 `capture_robot_pose.py` 落盘的 YAML，按控制器整组下发关节角。下发顺序是刻意的，
三步不可调换：

  ① 电缸杆先退到 min(当前值, 目标值)  —— 杆还伸着时就动机械臂，会把杆别在柜体上
  ② 双臂下发目标关节角                —— 此时杆已经不在外面，臂可以自由走
  ③ 电缸杆下发目标值                  —— 到这儿才允许重新伸出

若目标杆位比当前还长，第 ① 步就是空操作，等价于"先动臂、后伸杆"，同样安全。

轮子关节**不下发**：那会驱动底盘。本工具只负责回退末端位姿，底盘由导航负责。

用法：
  python3 scripts/tools/restore_robot_pose.py \\
      --pose xczs_inspection_robot_control/config/taught_poses/db1_teach_node1_press48.yaml
  python3 scripts/tools/restore_robot_pose.py --pose ... --dry-run   # 只看差值

先决条件：活栈已就绪、臂与电缸控制器可用。
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
    ARMS, JOINT_STATES_TOPIC, ROD_SIDES, SpinNode, best_effort_qos, control_groups,
    wait_for)

# 退杆判据：差 1 mm 以上就认为没退到位。电缸是 0.12 m 行程，1 mm 已远大于噪声。
RETREAT_TOLERANCE = 0.001

# 电缸关节的物理行程（URDF <limit>）。下发前必须把存档值钳进来：
# 2026-09-11 实测，早期 effort 接口留下的存档里有**越限的负值**（支撑杆 −13.8mm），
# 位置伺服会一直往 −13.8 推、限位再挡回来，两边对顶 → 杆振到 34mm 并带抖整条臂。
ROD_LIMITS = {
    "default": (0.0, 0.12),
    "r_three_cyl_finger3_joint": (0.0, 0.0254),
    "l_rocker_rotor_joint": (0.0, 0.0254),
}


def clamp_rod(joint, value):
    """把电缸目标钳到行程内，返回 (钳位后的值, 是否被钳)。"""
    low, high = ROD_LIMITS.get(joint, ROD_LIMITS["default"])
    clamped = max(low, min(high, value))
    return clamped, abs(clamped - value) > 1e-9


class PoseRestorer(SpinNode):
    def __init__(self):
        super().__init__("restore_robot_pose", num_threads=4)
        self._joint_state = None
        self._state_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state,
            best_effort_qos())
        groups = control_groups()
        # 名字不能叫 _clients / _subscriptions：那是 rclpy Node 的内部列表，
        # 覆盖掉会让 destroy_node() 读到错的东西直接 KeyError。
        self._action_clients = {
            action: ActionClient(self, FollowJointTrajectory, action)
            for action in groups
        }
        self._groups = groups

    # ------------------------------------------------------------------ infra
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _measured(self, joint):
        if self._joint_state is None:
            raise RuntimeError("%s not received yet" % JOINT_STATES_TOPIC)
        try:
            return self._joint_state.position[
                list(self._joint_state.name).index(joint)]
        except ValueError:
            raise RuntimeError("joint %s missing from %s"
                               % (joint, JOINT_STATES_TOPIC))

    def _settled(self, joints, timeout=20.0, tolerance=0.0002, window=0.5):
        """等这些关节连续 window 秒内的变动都小于 tolerance。

        控制器 action 返回 ≠ 关节到位（末段减速仍在走），固定延时取样会把
        "还在收尾"读成"到位误差"。
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
    def send(self, action, joints, targets, duration):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joints)
        point = JointTrajectoryPoint()
        point.positions = [targets[j] for j in joints]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]

        client = self._action_clients[action]
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("%s action server unavailable" % action)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = 5
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("%s rejected the goal" % action)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + max(30.0, duration * 6.0)
        while rclpy.ok() and not result_future.done() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            raise RuntimeError("%s did not finish in %.0f s"
                               % (action, max(30.0, duration * 6.0)))
        return result_future.result().result

    # ------------------------------------------------------------------- main
    def run(self, targets, dry_run, arm_duration, rod_duration):
        wait_for(lambda: self._joint_state is not None, 15.0,
                 JOINT_STATES_TOPIC)

        rod_joints = {j for cfg in ROD_SIDES.values() for j in cfg["joints"]}
        arm_joints = {j for cfg in ARMS.values() for j in cfg["joints"]}
        known = rod_joints | arm_joints
        # 末端关节一个都不能少：缺了就不是"回到那个位姿"，宁可报错也别动一半。
        missing = sorted(known - set(targets))
        if missing:
            raise RuntimeError("存档缺少关节: %s" % ", ".join(missing))
        # 其余的（轮子）是存档为了事后判断底盘有没有被挪过而记的，不下发。
        skipped = sorted(n for n in targets if n not in known)
        if skipped:
            print("跳过非末端关节（含轮子，不下发）: %s" % ", ".join(skipped))

        # 按 action 归组，并且只保留存档里有的关节。
        plan = []
        for action, joints in self._groups.items():
            present = [j for j in joints if j in targets]
            if present:
                plan.append((action, present))

        current = {j: self._measured(j) for j in targets if j in known}

        # 杆目标先钳进行程：存档里的越限值会让位置伺服和限位对顶（见 ROD_LIMITS 注释）。
        clamped_any = False
        for joint in sorted(rod_joints):
            fixed, was_clamped = clamp_rod(joint, targets[joint])
            if was_clamped:
                clamped_any = True
                print("钳位 %-28s 存档 %+.6f → %+.6f（行程上限/下限外）"
                      % (joint, targets[joint], fixed))
            targets[joint] = fixed
        if clamped_any:
            print("（存档里的杆越限值是早期 effort 接口留下的产物，物理上到不了）")

        print("\n==== 恢复存档位姿 ====")
        for action, joints in plan:
            delta = max(abs(targets[j] - current[j]) for j in joints)
            print("%-58s %d 个关节，最大偏差 %.5f" % (action, len(joints), delta))

        if dry_run:
            print("--dry-run：未下发。")
            return

        # ① 所有杆一律先收回到 0，绝不在杆伸着的时候动臂。
        # 早先写成"退到 min(当前, 目标)"，但目标本身可能就是伸出的（钩爪 18.75mm），
        # 那样等于带着伸出的钩爪去动臂——2026-09-11 就是这么把两根支撑杆贴着面板
        # 横刮 40mm 别死的。全收到 0 在任何情形下都更安全：多收一段不会伤机构，
        # 少收一段会拖杆。
        retreat = {}
        for action, joints in plan:
            if not set(joints) & rod_joints:
                continue
            values = {j: 0.0 for j in joints}
            if all(abs(values[j] - current[j]) < 1e-6 for j in joints):
                continue
            print("① 退杆 %-52s ..." % action)
            result = self.send(action, joints, values, rod_duration)
            print("   控制器结果 error_code=%s" % result.error_code)
            retreat[action] = (joints, values)
        if retreat:
            self._settled([j for joints, _v in retreat.values() for j in joints])
            # 电缸会"报告成功但实际没动"——顶住时 error_code 照样是 0，所以必须回读
            # 确认。2026-09-11 就是因为少了这一步，杆还伸着就动了臂，把两根支撑杆
            # 贴着面板横刮 40 mm 别死了。宁可不恢复，也不能把杆别坏。
            stuck = [(j, self._measured(j), values[j])
                     for joints, values in retreat.values() for j in joints
                     if abs(self._measured(j) - values[j]) > RETREAT_TOLERANCE]
            for joint, actual, wanted in stuck:
                print("   !! %s 没退到位: 目标 %.5f 实测 %.5f（差 %+.2f mm）"
                      % (joint, wanted, actual, (actual - wanted) * 1000.0))
            if stuck:
                raise RuntimeError(
                    "电缸没退到位（多半是顶住了），已中止，**未动机械臂**")

        # ② 双臂到位。
        for action, joints in plan:
            if not set(joints) & arm_joints:
                continue
            print("② 动臂 %-52s ..." % action)
            result = self.send(action, joints, targets, arm_duration)
            print("   控制器结果 error_code=%s" % result.error_code)
        self._settled([j for a, joints in plan if set(joints) & arm_joints
                       for j in joints])

        # ③ 杆伸到目标值。
        for action, joints in plan:
            if not set(joints) & rod_joints:
                continue
            values = {j: targets[j] for j in joints}
            if all(abs(values[j] - self._measured(j)) < 1e-6 for j in joints):
                continue
            print("③ 伸杆 %-52s ..." % action)
            result = self.send(action, joints, values, rod_duration)
            print("   控制器结果 error_code=%s" % result.error_code)

        all_joints = [j for _a, joints in plan for j in joints]
        if not self._settled(all_joints):
            print("警告: 20 s 内未观测到全部关节静止，下面的实测值可能仍在动。")

        print("\n---- 到位核对 ----")
        worst = 0.0
        for joint in sorted(all_joints):
            after = self._measured(joint)
            error = after - targets[joint]
            worst = max(worst, abs(error))
            print("  %-28s 目标 %+.6f  实测 %+.6f  误差 %+.2f mm"
                  % (joint, targets[joint], after, error * 1000.0))
        print("最大误差 %.2f mm" % (worst * 1000.0))
        print("======================")


def load_targets(pose_path):
    with open(pose_path, "r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle)
    if "joints" not in payload:
        raise RuntimeError("%s 里没有 joints 段，不是 capture_robot_pose 的输出？"
                           % pose_path)
    print("位姿: %s" % payload.get("label", "(无 label)"))
    print("抓取时间: %s" % payload.get("captured_wall_time", "?"))
    return {name: float(value) for name, value in payload["joints"].items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", required=True, help="存档 YAML 路径")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印各控制器最大偏差，不下发")
    parser.add_argument("--arm-duration", type=float, default=5.0,
                        help="机械臂轨迹时长（s），默认 5.0")
    parser.add_argument("--rod-duration", type=float, default=3.0,
                        help="电缸轨迹时长（s），默认 3.0")
    args = parser.parse_args()

    targets = load_targets(args.pose)
    rclpy.init()
    node = PoseRestorer()
    try:
        node.run(targets, args.dry_run, args.arm_duration, args.rod_duration)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
