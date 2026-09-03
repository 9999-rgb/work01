#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AGENT §7.2 db1 分级封顶现场调测驱动（双臂拉拽封顶验收前置）。

对电气夹层抽屉 db1 跑一次"打开"任务，但把 operator 的 debug_stage_cap 参数
临时设为 N，使执行在 §7.2 指定阶段边界后按成功收尾（收尾含脱离/电缸回位/
退让收起/轨道闩重锁），并打印完整物证。

用法：
  python3 scripts/tools/db1_stage_cap_driver.py --cap 1   # 仅 ready 就位
  python3 scripts/tools/db1_stage_cap_driver.py --cap 3   # 解锁电机全链
  python3 scripts/tools/db1_stage_cap_driver.py --cap 4   # + 支撑段
  python3 scripts/tools/db1_stage_cap_driver.py --cap 5   # + 夹爪闭合段
  python3 scripts/tools/db1_stage_cap_driver.py --cap 6   # + 真实附着/脱离取证
  python3 scripts/tools/db1_stage_cap_driver.py --cap 7   # + 拉距封顶 0.03 并推回
  python3 scripts/tools/db1_stage_cap_driver.py --cap 8   # + 拉距封顶 0.10 并推回

先决条件：活栈（run_all.sh）已就绪、无人任务占用、db1 处于 closed+闩定
（本驱动每次跑完 cap>=3 后都会把抽屉重新闩定，可连续递增跑）。跑完后参数
复位为 0（全流程）；--keep-cap 可保留。

导航由任务层负责：每次运行前先
  python3 scripts/tools/preposition_base.py --control db1 \\
      --cabinet electrical_mezzanine --toolset A \\
      --adapter xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_adapter.yaml
把底盘遥移到 db1 工位（操作员嵌入式导航关闭，navigate_to_staging_pose 必须为
False，否则目标立即以 NAVIGATION_FAILED 结束）。
"""
import argparse
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from xczs_inspection_robot_interfaces.action import OperateCabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControlState

NS = "/xczs/cabinet/electrical_mezzanine"
OPERATOR_NODE = "xczs_cabinet_button_operator"
CONTROL = "db1"
PARAM = "debug_stage_cap"
ACTION = NS + "/operate_cabinet_control"


class Db1StageCapDriver(Node):
    def __init__(self, cap, keep_cap):
        super().__init__("db1_stage_cap_driver")
        self._cap = cap
        self._keep_cap = keep_cap
        self._param_cli = self.create_client(
            SetParameters, f"{NS}/{OPERATOR_NODE}/set_parameters")
        self._action = ActionClient(self, OperateCabinetControl, ACTION)
        self._drawer_state = None
        self._state_sub = self.create_subscription(
            CabinetControlState, f"{NS}/{CONTROL}/state", self._on_state, 10)
        # rclpy Node.executor 只存弱引用：必须用不同名强属性保住 executor，否则
        # 立即被 GC，回调全停（与 s34_unlock_fault_test.py 同一坑）。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def stop(self):
        """停 spin 线程后再销毁节点：rclpy 退出时若 spin 线程仍存活会触发
        'terminate called without an active exception' 核心转储（实测）。"""
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)

    # ------------------------------------------------------------------ infra
    def _on_state(self, msg):
        self._drawer_state = msg

    def _on_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        stage = getattr(fb, "stage", None)
        progress = getattr(fb, "progress", None)
        detail = getattr(fb, "detail", "")
        print(f"[fb] stage={stage} progress={progress} {detail}")

    def _wait(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        raise RuntimeError(f"timeout waiting for {what}")

    def _set_param(self, value):
        self._wait(lambda: self._param_cli.service_is_ready(), 10.0,
                   "operator parameter service")
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name=PARAM,
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=value))]
        future = self._param_cli.call_async(request)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError(
                f"could not set {OPERATOR_NODE}.{PARAM}={value}: no reply "
                "within 10 s")
        result = future.result()
        if not result.results or not result.results[0].successful:
            raise RuntimeError(
                f"could not set {OPERATOR_NODE}.{PARAM}={value}; "
                f"reply={result.results[0].reason if result.results else 'empty results'}")

    def _operate_open(self, timeout_s=600.0):
        if not self._action.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("operate action server unavailable")
        goal = OperateCabinetControl.Goal()
        goal.control_id = CONTROL
        goal.command = OperateCabinetControl.Goal.COMMAND_SET_STATE
        goal.target_state = "open"
        goal.use_target_position = False
        # 操作员嵌入式导航被任务层禁用（profile_contract 强制 false）；底盘由
        # preposition_base.py 预先遥移到 db1 工位。
        goal.navigate_to_staging_pose = False
        future = self._action.send_goal_async(goal, feedback_callback=self._on_fb)
        # rclpy Future.result() 不阻塞（未完成时立即返回 None）：必须轮询
        # done() 等待服务器应答，否则把尚在途中的应答误读成传输层拒绝。
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        goal_handle = future.result() if future.done() else None
        print(f"[diag] goal_handle={goal_handle!r}")
        if goal_handle is None or not goal_handle.accepted:
            print("[diag] goal NOT accepted at transport layer; "
                  "waiting 15 s for a result anyway...")
            result_future = None
            if goal_handle is not None:
                try:
                    result_future = goal_handle.get_result_async()
                except Exception as exc:  # noqa: BLE001
                    print(f"[diag] get_result_async failed: {exc}")
            if result_future is not None:
                outcome = result_future.result(15.0)
                if outcome is not None:
                    return outcome
            raise RuntimeError(
                "operate goal was rejected by the operator "
                "(check navigate_to_staging_pose and resource state)")
        self.get_logger().info(f"operate goal accepted (cap={self._cap}); "
                               "waiting for completion...")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not result_future.done() and \
                time.monotonic() < deadline:
            time.sleep(0.1)
        if not result_future.done():
            raise RuntimeError(
                f"operate goal did not finish within {timeout_s} s")
        outcome = result_future.result()
        return outcome

    # ------------------------------------------------------------------- main
    def run(self):
        self._wait(lambda: self._drawer_state is not None, 10.0,
                   "drawer state topic")
        before = self._drawer_state
        try:
            self._set_param(self._cap)
            self.get_logger().info(
                f"set {OPERATOR_NODE}.{PARAM}={self._cap}; before: "
                f"state={before.state_id} position={before.position:.4f}")
            outcome = self._operate_open()
        finally:
            if not self._keep_cap:
                self._set_param(0)
                self.get_logger().info(f"reset {OPERATOR_NODE}.{PARAM}=0")

        status = outcome.status
        result = outcome.result
        after = self._drawer_state
        print("\n==== AGENT §7.2 capped run evidence ====")
        print(f"goal status      : {status}")
        print(f"success          : {result.success}")
        print(f"error_code       : {result.error_code}")
        print(f"message          : {result.message}")
        print(f"diagnostic_stage : {result.diagnostic_stage}")
        print(f"final_state      : {result.final_state}")
        print(f"final_position   : {result.final_position:.4f} m")
        print(f"peak_position    : {result.peak_position:.4f} m")
        print(f"physical_outcome : {result.physical_outcome_confirmed}")
        print(f"final_verified   : {result.final_state_verified}")
        print(f"grasp_released   : {result.grasp_released}")
        print(f"transport_ok     : {result.transport_succeeded}")
        if before and after:
            print(f"drawer topic     : state {before.state_id}@{before.position:.4f}"
                  f" -> {after.state_id}@{after.position:.4f} m")
        print("=====================================")
        if not result.success or result.error_code != OperateCabinetControl.Result.SUCCESS:
            raise SystemExit(f"capped stage {self._cap} did not finish SUCCESS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cap", type=int, required=True,
                        choices=[1, 3, 4, 5, 6, 7, 8],
                        help="AGENT §7.2 stage cap (1/3/4/5/6/7/8)")
    parser.add_argument("--keep-cap", action="store_true",
                        help="leave debug_stage_cap set after the run")
    args = parser.parse_args()

    rclpy.init()
    driver = Db1StageCapDriver(args.cap, args.keep_cap)
    try:
        driver.run()
    finally:
        driver.stop()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
