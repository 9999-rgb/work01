#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AGENT §3.4 解锁故障测试驱动（fail-loud 逐项验证）。

对电气夹层抽屉 db1，向插件解锁服务 /xczs/cabinet/electrical_mezzanine/unlock
提交"预置条件被破坏"的解锁请求，逐项验证插件响亮失败且轨道保持锁止。

两类用例：
  * 无会话（停车场即可执行，无需机械臂运动）：
      - case 3a  空操作租约 -> 拒绝"requires a non-empty operation lease identity"
      - case 3b  外来/从未授权的租约（无 active_control+heartbeat 会话）
                  -> 拒绝"requires a fresh heartbeat ... session"
  * 有会话 + 真实解锁电机伸出（仅需末端 FJT，无需机械臂到抽屉）：
      - case 2   解锁电机已伸出（pressed=true）但工具不在解锁区
                  -> 拒绝"not inside the unlock zone（距离超阈值）"，轨道保持锁止

依赖机械臂到达抽屉解锁区才能构造的用例（1 仅机械臂靠近但电机未伸、
4 按压 *p 指示灯、5 电机过冲/达安全上限）在 §7.2 阶段 2/3 现场按序补测，
本文件保留同一会话/请求骨架。

用法：
  python3 scripts/tools/s34_unlock_fault_test.py          # 跑全部 park 用例
  python3 scripts/tools/s34_unlock_fault_test.py --case 2 # 只跑指定用例
先决条件：活栈（run_all.sh）已就绪、无人任务占用、db1 处于 closed。
"""
import argparse
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import String
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration as BuiltinDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from xczs_inspection_robot_interfaces.srv import (
    ManageOperationLease, SetCabinetUnlock)
from xczs_inspection_robot_interfaces.msg import CabinetControlState

# --- 合同常量（来自 electrical_mezzanine.xacro / 场景适配 YAML）--------------
NS = "/xczs/cabinet/electrical_mezzanine"
CONTROL = "db1"
ROBOT_MODEL = "xczs_inspection_robot"
RIGHT_TOOL_LINK = "r_three_cyl_base"
# r_three_cyl_base 局部系下的有效工具尖端（适配器 drawer_tools.right.tool_tip_position）
TIP = {"x": 0.0265, "y": -0.061735, "z": -0.3885}
# 解锁逻辑区中心（operator 镜像值；插件实际用 SDF 局部点，请求字段仅为保真填写）
PRESS_POINT = {"x": 0.115, "y": 4.693, "z": 0.952}

UNLOCK_MOTOR = "r_three_cyl_finger3_joint"
FINGER1 = "r_three_cyl_finger1_joint"
FINGER2 = "r_three_cyl_finger2_joint"
THREE_CYL_JOINTS = [FINGER1, FINGER2, UNLOCK_MOTOR]
FJT_TOPIC = "/xczs/three_cylinder_controller/follow_joint_trajectory"
JOINT_STATES_TOPIC = "/xczs/joint_states"
# 插件周期发布本控制名下关节（含抽屉滑轨 db1），是抽屉真实位置的物证。
DB1_JOINT_STATES = NS + "/db1/joint_states"
DRAWER_JOINT = "db1"

LEASE_SERVICE = "/xczs/operation_lease"
OWNER_PREFIX = "s34_fault_test"
HB_PERIOD = 0.4            # s；插件看门狗 2.0 s
RENEW_PERIOD = 0.7         # s；协调器租约最大 5.0 s（本驱动请 3.0 s）
MOTOR_EXTEND = 0.008       # = controls.db1.unlock_pressed_position（现场标定前占位）
MOTOR_TRAJECTORY_SEC = 2.0

PASS_MSG = {
    1: "Unlock contact link is not inside the unlock zone",
    2: "Unlock contact link is not inside the unlock zone",
    "3a": "Drawer unlock requires a non-empty operation lease identity",
    "3b": "Drawer unlock requires a fresh heartbeat from the exact active "
          "control and global operation lease session",
}


def point(xyz):
    return Point(x=float(xyz["x"]), y=float(xyz["y"]), z=float(xyz["z"]))


class S34FaultTest(Node):
    def __init__(self, owner_seq):
        super().__init__("s34_unlock_fault_test")
        # active_control 订阅是 transient_local：发布端必须同 durability，否则
        # QoS 不兼容静默丢弃（与真实 operator 一致）。
        self.active_pub = self.create_publisher(
            String, NS + "/active_control",
            QoSProfile(depth=10,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))
        self.hb_pub = self.create_publisher(String, NS + "/operation_heartbeat", 10)
        self.unlock_cli = self.create_client(SetCabinetUnlock, NS + "/unlock")
        self.lease_cli = self.create_client(ManageOperationLease, LEASE_SERVICE)
        self.state = None
        self.state_sub = self.create_subscription(
            CabinetControlState, f"{NS}/{CONTROL}/state", self._on_state, 10)
        self._joints = {}
        self.js_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10)
        # 抽屉滑轨实测（plugin 以 BEST_EFFORT 周期发布，位置变化即抽屉被动过）。
        self._drawer_joints = {}
        self.drawer_js_sub = self.create_subscription(
            JointState, DB1_JOINT_STATES, self._on_drawer_joint_states,
            QoSProfile(depth=5,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self._hb_stop = False
        self._renew_stop = False
        # rclpy Node.executor 只存弱引用：必须用不同名强属性保住 executor，否则
        # 立即被 GC，回调全停（与 p2_drawer_test.py 同一坑）。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

        self.owner_id = f"{OWNER_PREFIX}:{owner_seq}"
        self.lease_id = None

    # ---- 通用工具 ----------------------------------------------------------
    @staticmethod
    def _call(cli, req, timeout=20.0):
        future = cli.call_async(req)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise TimeoutError(f"service call timed out: {cli.srv_name}")
            time.sleep(0.02)
        return future.result()

    def _on_state(self, msg):
        self.state = msg

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            self._joints[name] = msg.position[i]

    def _on_drawer_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            self._drawer_joints[name] = msg.position[i]

    def joint_position(self, name, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline and name not in self._joints:
            time.sleep(0.05)
        return self._joints.get(name)

    def drawer_position(self, timeout=3.0):
        """实测抽屉滑轨位置；抽屉全程不得因解锁尝试而移动。"""
        deadline = time.time() + timeout
        while time.time() < deadline and DRAWER_JOINT not in self._drawer_joints:
            time.sleep(0.05)
        return self._drawer_joints.get(DRAWER_JOINT)

    def wait_service(self, timeout=15.0):
        for cli in (self.unlock_cli, self.lease_cli):
            if not cli.wait_for_service(timeout):
                raise RuntimeError(f"service {cli.srv_name} unavailable")

    # ---- 全局操作租约（协调器单主）-----------------------------------------
    def acquire_lease(self):
        req = ManageOperationLease.Request()
        req.command = ManageOperationLease.Request.ACQUIRE
        req.owner_id = self.owner_id
        req.requested_duration = 3.0
        resp = self._call(self.lease_cli, req)
        if not resp.success:
            raise RuntimeError(f"lease ACQUIRE rejected: {resp.message}")
        self.lease_id = resp.lease_id
        print(f"[lease] ACQUIRE ok owner={self.owner_id} lease={self.lease_id} "
              f"remaining={resp.remaining_duration:.2f}s")

    def _renew_loop(self):
        req = ManageOperationLease.Request()
        req.command = ManageOperationLease.Request.RENEW
        req.owner_id = self.owner_id
        req.lease_id = self.lease_id
        req.requested_duration = 3.0  # 续期同样要给出时长（0.0 会出允许区间）
        while not self._renew_stop:
            try:
                resp = self._call(self.lease_cli, req, timeout=3.0)
                if not resp.success:
                    print(f"[lease] RENEW lost: {resp.message}")
                    return
            except Exception as exc:  # noqa: BLE001
                print(f"[lease] RENEW error: {exc}")
            time.sleep(RENEW_PERIOD)

    def release_lease(self):
        if not self.lease_id:
            return
        req = ManageOperationLease.Request()
        req.command = ManageOperationLease.Request.RELEASE
        req.owner_id = self.owner_id
        req.lease_id = self.lease_id
        try:
            resp = self._call(self.lease_cli, req, timeout=10.0)
            print(f"[lease] RELEASE ok={resp.success} msg={resp.message}")
        except Exception as exc:  # noqa: BLE001
            print(f"[lease] RELEASE error: {exc}")
        self.lease_id = None

    # ---- 会话（active_control + heartbeat）---------------------------------
    def start_session(self):
        self.active_pub.publish(String(data=CONTROL))
        self.hb_pub.publish(String(data=self.lease_id))
        t = threading.Thread(target=self._hb_loop, daemon=True)
        t.start()
        # 等会话在插件内生效（心跳窗口 2 s，提前一拍即可）。
        time.sleep(0.5)

    def _hb_loop(self):
        while not self._hb_stop:
            self.active_pub.publish(String(data=CONTROL))
            if self.lease_id:
                self.hb_pub.publish(String(data=self.lease_id))
            time.sleep(HB_PERIOD)

    def stop_session(self):
        self._hb_stop = True
        self.active_pub.publish(String(data=""))

    # ---- 末端 FJT：解锁电机有界伸出/收回 ------------------------------------
    def drive_unlock_motor(self, target):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(THREE_CYL_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0, float(target)]  # finger1/2 保持标定位 0
        pt.time_from_start = BuiltinDuration(
            sec=int(MOTOR_TRAJECTORY_SEC),
            nanosec=int((MOTOR_TRAJECTORY_SEC - int(MOTOR_TRAJECTORY_SEC)) * 1e9))
        goal.trajectory.points.append(pt)
        client = ActionClient(self, FollowJointTrajectory, FJT_TOPIC)
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"FJT server unavailable: {FJT_TOPIC}")
        send_future = client.send_goal_async(goal)
        deadline = time.time() + 12.0
        while rclpy.ok() and not send_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            client.destroy()
            raise TimeoutError("FJT send_goal timed out")
        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        deadline = time.time() + 12.0
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            time.sleep(0.05)
        client.destroy()
        if not result_future.done():
            raise TimeoutError("FJT execution timed out")
        status = result_future.result().status
        # 电机驱动态以实测电机关节为准（operator drive_unlock_motor 同判据）。
        measured = self.joint_position(UNLOCK_MOTOR, timeout=3.0)
        print(f"[motor] target={target} FJT status={status} "
              f"measured {UNLOCK_MOTOR}={measured}")
        return measured

    # ---- 解锁服务请求 -------------------------------------------------------
    def unlock_request(self, lease_id):
        req = SetCabinetUnlock.Request()
        req.control_id = CONTROL
        req.operation_lease_id = lease_id
        req.robot_model = ROBOT_MODEL
        req.right_robot_link = RIGHT_TOOL_LINK
        req.right_robot_grasp_point = point(TIP)
        req.unlock_press_point = point(PRESS_POINT)
        req.unlock = True
        resp = self._call(self.unlock_cli, req)
        print(f"[unlock] success={resp.success} distance={resp.distance:.4f} "
              f"drawer_unlocked={resp.drawer_unlocked} pressed={resp.pressed} "
              f"right_tool_contact={resp.right_tool_contact}")
        print(f"[unlock] message: {resp.message}")
        return resp

    def check(self, name, cond, detail=""):
        status = "PASS" if cond else "FAIL"
        print(f"[{status}] {name}" + (f"  ({detail})" if detail else ""))
        return cond


def run_baseline(driver):
    """P0：无会话用例（停车场即可，无机器人运动）。"""
    results = []
    # §3.4-3a：空操作租约。
    resp = driver.unlock_request("")
    results.append(driver.check(
        "§3.4-3a 空租约拒绝",
        (not resp.success and not resp.drawer_unlocked
         and PASS_MSG["3a"] in resp.message), resp.message))
    # §3.4-3b：非空但从未授权的外来租约（未建立 active_control+heartbeat）。
    resp = driver.unlock_request("lease-not-mine")
    results.append(driver.check(
        "§3.4-3b 外来租约拒绝",
        (not resp.success and not resp.drawer_unlocked
         and PASS_MSG["3b"] in resp.message), resp.message))
    return results


def run_case2(driver):
    """P1：真实会话 + 解锁电机伸出但工具不在解锁区 -> 距离门拒绝。"""
    results = []
    driver.acquire_lease()
    driver.start_session()
    renew_thread = threading.Thread(target=driver._renew_loop, daemon=True)
    renew_thread.start()
    try:
        # 前置断言：确认会话已授权（否则会先撞心跳门而不是距离门）。
        measured = driver.joint_position(UNLOCK_MOTOR, timeout=3.0)
        base_motor = measured if measured is not None else 0.0
        slide_before = driver.drawer_position(timeout=3.0)
        measured = driver.drive_unlock_motor(MOTOR_EXTEND)
        extended_ok = (measured is not None and
                       measured >= 0.001 and measured < 0.0254)
        resp = driver.unlock_request(driver.lease_id)
        results.append(driver.check(
            "§3.4-2 电机已伸出但工具不在解锁区拒绝",
            (not resp.success and not resp.drawer_unlocked
             and resp.right_tool_contact is False
             and resp.pressed is True
             and resp.distance is not None and resp.distance > 0.03
             and PASS_MSG[2] in resp.message),
            f"extended_ok={extended_ok} motor={base_motor:.4f}->"
            f"{measured if measured is not None else float('nan'):.4f}"))
        # 抽屉滑轨必须纹丝不动（轨道保持锁止）。
        time.sleep(0.3)
        slide_after = driver.drawer_position(timeout=3.0)
        slide_still = (slide_before is not None and slide_after is not None
                       and abs(slide_after - slide_before) < 1e-3
                       and abs(slide_after) < 0.05)
        results.append(driver.check(
            "§3.4-2 db1 滑轨保持 closed",
            slide_still,
            f"slide {slide_before if slide_before is not None else float('nan'):.4f}"
            f" -> {slide_after if slide_after is not None else float('nan'):.4f} m"))
        # 收回电机，不留半伸探杆。
        try:
            driver.drive_unlock_motor(0.0)
        except Exception as exc:  # noqa: BLE001
            print(f"[motor] retract error: {exc}")
    finally:
        driver.stop_session()
        driver._renew_stop = True
        if renew_thread.is_alive():
            renew_thread.join(timeout=3.0)
        driver.release_lease()
    return results


def main():
    parser = argparse.ArgumentParser(description="AGENT §3.4 解锁故障测试")
    parser.add_argument(
        "--case", choices=["3a", "3b", "2", "all"], default="all")
    args = parser.parse_args()

    rclpy.init()
    driver = S34FaultTest(owner_seq=int(time.time() * 1000))
    driver.wait_service()
    results = []
    try:
        if args.case == "2":
            results += run_case2(driver)
        elif args.case == "3a":
            resp = driver.unlock_request("")
            results.append(driver.check(
                "§3.4-3a 空租约拒绝",
                (not resp.success and not resp.drawer_unlocked
                 and PASS_MSG["3a"] in resp.message), resp.message))
        elif args.case == "3b":
            resp = driver.unlock_request("lease-not-mine")
            results.append(driver.check(
                "§3.4-3b 外来租约拒绝",
                (not resp.success and not resp.drawer_unlocked
                 and PASS_MSG["3b"] in resp.message), resp.message))
        else:  # all：无会话用例在前（不污染会话），有会话 case 2 在后。
            results += run_baseline(driver)
            results += run_case2(driver)
    except Exception as exc:  # noqa: BLE001
        print(f"[driver] ERROR: {exc}")
        results.append(False)
    finally:
        driver.stop_session()
        driver.destroy_node()
        rclpy.shutdown()

    passed = sum(1 for r in results if r)
    print(f"\n{passed}/{len(results)} checks passed")
    sys.exit(0 if passed == len(results) and results else 1)


if __name__ == "__main__":
    main()
