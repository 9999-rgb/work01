#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电气夹层 db1 可视化后端（drawer_execution_backend=visual）现场调测驱动。

对抽屉 db1 跑一次"双手同步开/合"任务，走操作员 operator 的 visual 分派：
双臂钉单工作位姿 → 抽屉插件运动学播放（主）→ 底盘 1:1 跟随（从）。
本驱动不经 Web/HTTP（绕开管理员鉴权），直接经 ROS action 调 operator，
并在整段动作中以 5 Hz 采样抽屉轨位（/…/db1/state.position）与底盘位姿
（TF odom→body），用于核验"手沿轨道跟随 + 底盘与抽屉同轴位移 1:1"。

用法：
  python3 scripts/tools/db1_visual_roundtrip.py            # 预置底盘点位+开+合
  python3 scripts/tools/db1_visual_roundtrip.py --once open   # 只开（预置+开）
  python3 scripts/tools/db1_visual_roundtrip.py --once close  # 只合（需抽屉已开位）
  python3 scripts/tools/db1_visual_roundtrip.py --no-preposition   # 不遥移底盘
  # 现场逐帧观察：跑到某阶段后原地停住不动（Ctrl-C 取消并正常收尾）
  python3 scripts/tools/db1_visual_roundtrip.py --once open --stop-after work_pose
  python3 scripts/tools/db1_visual_roundtrip.py --once open --stop-after hook --hold-sec 60

先决条件：活栈（run_all.sh）已就绪、toolset A、无人任务占用。开合之间基座
保持跟随后的位置，close 由该位置直接拉回，不需要重新停靠。
"""
import argparse
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from xczs_inspection_robot_interfaces.action import OperateCabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControlState

NS = "/xczs/cabinet/electrical_mezzanine"
CONTROL = "db1"
ACTION = NS + "/operate_cabinet_control"
STATE_TOPIC = NS + "/" + CONTROL + "/state"

# visual 后端阶段驻留调试开关（operator 侧参数；见
# cabinet_button_operator.cpp 中 execute_drawer_visual_backend 的注释）。
# --stop-after 命中阶段后原地保持不动，--hold-sec 0 = 保持到取消/租约丢失。
OPERATOR_NODE = "xczs_cabinet_button_operator"
STOP_STAGE_PARAM = "visual_stop_after_stage"
STOP_HOLD_PARAM = "visual_stop_hold_seconds"
STOP_STAGES = ("work_pose", "self_center", "hook", "support", "unlock")

# 阶段名（open/close，供 CLI 与日志）→ 抽屉 state_id（控制 enum 校验用）。
# 抽屉插件的 detent 命名是 'closed'（state_ids=['closed','open']），CLOSE 阶段
# 若把 'close' 直接当 target_state 发给 operator，resolve_operation_target 的
# std::find 找不到 → error_code 2（UNSUPPORTED_COMMAND）。
STAGE_TO_STATE_ID = {"open": "open", "close": "closed"}
WORKSPACE = Path(__file__).resolve().parents[2]
ADAPTER = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "scene_controls"
    / "electrical_mezzanine_adapter.yaml"
)
BASE_FRAME = "body"
WORLD_FRAME = "odom"


class Db1VisualRoundtrip(Node):
    def __init__(self, stop_after="", hold_sec=0.0):
        super().__init__("db1_visual_roundtrip", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self._stop_after = stop_after
        self._hold_sec = hold_sec
        self._goal_handle = None
        self._param_cli = self.create_client(
            SetParameters, f"{NS}/{OPERATOR_NODE}/set_parameters")
        self._action = ActionClient(self, OperateCabinetControl, ACTION)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._drawer_state = None
        self._state_sub = self.create_subscription(
            CabinetControlState, STATE_TOPIC, self._on_state, 10)
        # rclpy Node.executor 只存弱引用：用不同名强属性保住 executor，否则
        # 立即被 GC，回调全停（与 s34_unlock_fault_test.py 同一坑）。
        self._spin_executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        self._spin_executor.add_node(self)
        self._spin = threading.Thread(target=self._spin_executor.spin, daemon=True)
        self._spin.start()

    def stop(self):
        if getattr(self, "_spin", None) and self._spin.is_alive():
            self._spin_executor.shutdown()
            self._spin.join(timeout=3.0)

    # ------------------------------------------------------------------ infra
    def _on_state(self, msg):
        self._drawer_state = msg

    def _on_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        print("[fb] phase=%d stage=%s current=%.4f target=%.4f %s" % (
            fb.phase,
            getattr(fb, "stage", ""),
            getattr(fb, "current_position", float("nan")),
            getattr(fb, "target_position", float("nan")),
            getattr(fb, "message", "")))

    def _wait(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        raise RuntimeError("timeout waiting for %s" % what)

    def _set_param_string(self, name, value):
        self._set_param(name, ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=value))

    def _set_param_double(self, name, value):
        self._set_param(name, ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=value))

    def _set_param(self, name, pvalue):
        if not self._param_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("operator parameter service unavailable")
        request = SetParameters.Request()
        request.parameters = [ParameterMsg(name=name, value=pvalue)]
        future = self._param_cli.call_async(request)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError(f"could not set {OPERATOR_NODE}.{name}")
        result = future.result()
        if not result.results or not result.results[0].successful:
            raise RuntimeError(
                f"could not set {OPERATOR_NODE}.{name}: "
                f"{result.results[0].reason if result.results else 'no reply'}")

    def _apply_stop_params(self, stop_after, hold_sec):
        """设置/复位 visual 后端阶段驻留参数（空串 + 0 = 关闭，零回归）。"""
        self._set_param_string(STOP_STAGE_PARAM, stop_after)
        self._set_param_double(STOP_HOLD_PARAM, hold_sec)
        self.get_logger().info(
            f"set {OPERATOR_NODE}.{STOP_STAGE_PARAM}='{stop_after}', "
            f"{STOP_HOLD_PARAM}={hold_sec}"
            + (" (0 = hold until canceled)" if stop_after else ""))

    def reset_stop_params(self):
        """复位阶段驻留参数（空串 + 0 = 关闭）。context 失效时显式报错，
        避免"复位成功"的假象把调试态留给下一个任务。"""
        if not rclpy.ok():
            raise RuntimeError("rclpy context 已失效，无法复位驻留参数")
        self._apply_stop_params("", 0.0)

    def cancel(self):
        """取消在途目标 —— 驻留窗内调用即让 operator 走取消收尾。"""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _base_pose(self):
        """odom→body 平动 (x, y, z)；TF 时间取最新（0 = latest）。"""
        try:
            t = self._tf_buffer.lookup_transform(
                WORLD_FRAME, BASE_FRAME, Time(), Duration(seconds=1.0))
        except Exception as exc:  # noqa: BLE001
            return None, "tf odom→body 不可用: %s" % exc
        p = t.transform.translation
        return (p.x, p.y, p.z), None

    def _operate(self, target_state, timeout_s=600.0):
        if not self._action.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("operate action server unavailable")
        goal = OperateCabinetControl.Goal()
        goal.control_id = CONTROL
        goal.command = OperateCabinetControl.Goal.COMMAND_SET_STATE
        goal.target_state = STAGE_TO_STATE_ID[target_state]
        goal.use_target_position = False
        # 操作员嵌入式导航被任务层禁用；底盘由 preposition_base.py 预置。
        goal.navigate_to_staging_pose = False
        future = self._action.send_goal_async(goal, feedback_callback=self._on_fb)
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        goal_handle = future.result() if future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(
                "operate goal rejected at transport layer "
                "(check resource/toolset state)")
        self._goal_handle = goal_handle
        self.get_logger().info(
            "operate goal accepted (%s); sampling drawer/base..." % target_state)

        samples = []
        started = time.monotonic()
        last_s = None
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done() and \
                time.monotonic() - started < timeout_s:
            mono = time.monotonic()
            if last_s is None or mono - last_s >= 0.2:
                last_s = mono
                base, err = self._base_pose()
                q = self._drawer_state.position if self._drawer_state else None
                # 样本首列为绝对 monotonic 秒，供 --csv-out 跨阶段拼一条时间轴。
                samples.append((mono, q, base))
            time.sleep(0.01)
        if not result_future.done():
            # 不中断任务：留给操作员收尾，采样已知的还差多少。
            raise RuntimeError(
                "operate goal did not finish within %s s" % timeout_s)
        outcome = result_future.result()
        return outcome, samples

    # ------------------------------------------------------------------- main
    def run(self, once, preposition, csv_out=None):
        self._wait(lambda: self._drawer_state is not None, 10.0,
                   "drawer state topic")
        self._wait(lambda: self._base_pose()[1] is None, 15.0,
                   "odom→body transform")

        self._csv = None
        if csv_out:
            self._csv = open(csv_out, "w", encoding="utf-8")
            self._csv.write("stage,t_s,rail_m,base_x_m,base_y_m,base_z_m\n")

        if preposition:
            self.get_logger().info("teleporting base to db1 station ...")
            cmd = [
                sys.executable,
                str(WORKSPACE / "scripts" / "tools" / "preposition_base.py"),
                "--control", CONTROL, "--cabinet", "electrical_mezzanine",
                "--toolset", "A", "--adapter", str(ADAPTER),
            ]
            subprocess.run(cmd, check=True, cwd=str(WORKSPACE))
            # 遥移后 odom→body 立即生效；等一段让 AMCL/位姿权威同步。
            time.sleep(1.0)

        before = self._drawer_state
        base_before, _ = self._base_pose()
        print("\n==== db1 visual backend run ====")
        print("drawer before: state=%s position=%.4f m" %
              (before.state_id, before.position))
        print("base before  : odom→body %.4f %.4f %.4f m" % base_before)

        if once == "close":
            stages = ["close"]
        else:
            stages = (["open"] if once == "open" else ["open", "close"])
        run_started = time.monotonic()
        t0_run = None  # 全 run 首个样本的绝对 monotonic 秒（跨阶段连续时间轴锚点）

        # 阶段驻留：只在单段 open 上生效（close 无对应阶段，且驻留期间不该继续）。
        stop_after = self._stop_after if stages == ["open"] else ""
        if self._stop_after and not stop_after:
            print("--stop-after 只在 --once open 上生效；本次忽略。")
        self._apply_stop_params(stop_after, self._hold_sec)
        # 驻留期间目标不会结束：超时给足（hold=0 = 保持到手动取消）。
        timeout_s = 600.0 if not stop_after else (
            (self._hold_sec + 600.0) if self._hold_sec > 0.0 else 86400.0)

        try:
            for stage in stages:
                self._wait(lambda: self._drawer_state is not None, 10.0,
                           "drawer state before %s" % stage)
                q_start = self._drawer_state.position
                outcome, samples = self._operate(stage, timeout_s)
                result = outcome.result
                print("\n---- %s result ----" % stage.upper())
                print("goal status    : %s" % outcome.status)
                print("success        : %s" % result.success)
                print("error_code     : %s" % result.error_code)
                print("message        : %s" % result.message)
                print("diagnostic_stg : %s" % result.diagnostic_stage)
                print("initial_pos    : %.4f m" % result.initial_position)
                print("final_position : %.4f m" % result.final_position)
                print("final_state    : %s" % result.final_state)
                print("execution_back : %s" %
                      getattr(result, "execution_backend", ""))
                print("phys_outcome   : %s" % result.physical_outcome_confirmed)
                print("final_verified : %s" % result.final_state_verified)
                print("transport_ok   : %s" % result.transport_succeeded)
                print("grasp_released : %s" % result.grasp_released)
                base_now, _ = self._base_pose()
                print("drawer rail    : %.4f -> %.4f m (samples=%d)" %
                      (q_start,
                       self._drawer_state.position
                       if self._drawer_state else float("nan"),
                       len(samples)))
                if base_now:
                    print("base after     : odom→body %.4f %.4f %.4f m "
                          "(Δ %.4f %.4f %.4f)" %
                          (base_now[0], base_now[1], base_now[2],
                           base_now[0] - base_before[0],
                           base_now[1] - base_before[1],
                           base_now[2] - base_before[2]))

                # 5 Hz (rail, odom→body) 样本落盘：可视化切片用真实遥测回放，
                # t 以本次 run 首次 sample 为 0（跨 open/close 拼一条连续时间轴）。
                if self._csv is not None:
                    if t0_run is None:
                        t0_run = samples[0][0] if samples else run_started
                    for mono, q, base in samples:
                        if q is None or base is None:
                            continue
                        self._csv.write("%s,%.3f,%.4f,%.4f,%.4f,%.4f\n" % (
                            stage, mono - t0_run, q, base[0], base[1], base[2]))
                    self._csv.flush()

                if not result.success:
                    raise SystemExit(
                        "visual %s did not finish SUCCESS" % stage)
                base_before = base_now
        finally:
            # 驻留参数必须复位，否则下一次任务会继承这个调试态。
            if stop_after:
                try:
                    self._apply_stop_params("", 0.0)
                except Exception as exc:  # noqa: BLE001
                    print("警告: 驻留参数复位失败: %s" % exc)
            if self._csv is not None:
                self._csv.close()
        print("====================================")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", choices=["open", "close"], default=None,
                        help="only run one direction (default: open then close)")
    parser.add_argument("--no-preposition", action="store_true",
                        help="skip the base teleport (base already at station)")
    parser.add_argument("--csv-out", metavar="PATH", default=None,
                        help="write 5 Hz (rail, odom->body) samples as CSV "
                             "(stage,t_s,rail_m,base_x_m,base_y_m,base_z_m)")
    parser.add_argument("--stop-after", choices=[""] + list(STOP_STAGES),
                        default="",
                        help="visual 后端跑到该阶段后原地保持不动再收起："
                             "work_pose/self_center（单工作位姿，电缸全收拢）、"
                             "hook（钩爪闭合压贴把手）、support（支撑杆入缝）、"
                             "unlock（解锁杆伸出）。只在 --once open 上生效；"
                             "与 --hold-sec 0 合用 = 一直停住直到 Ctrl-C。")
    parser.add_argument("--hold-sec", type=float, default=0.0,
                        help="阶段驻留秒数。0（默认）= 一直保持到本进程被 "
                             "Ctrl-C（会取消目标、让 operator 正常收尾）。")
    args = parser.parse_args()
    if args.hold_sec < 0.0:
        parser.error("--hold-sec 不能为负")
    if args.hold_sec > 0.0 and not args.stop_after:
        parser.error("--hold-sec 需要与 --stop-after 一起用")

    # 自装 SIGINT 处理器，不用 rclpy 默认的：默认处理器会先把 context shutdown
    # 再抛 KeyboardInterrupt，于是 run() 收尾里的驻留参数复位一律变成
    # "rcl node's context is invalid" 的空动作（实测），调试态（stop-after N 秒
    # 驻留）会被下一个任务继承。自装后可自己定顺序：复位 → 取消 → shutdown。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def _raise_keyboard_interrupt(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    driver = Db1VisualRoundtrip(args.stop_after, args.hold_sec)
    try:
        driver.run(args.once, not args.no_preposition, args.csv_out)
    except KeyboardInterrupt:
        # 驻留窗内 Ctrl-C：取消目标让 operator 走正常取消收尾（退让收起），
        # 而不是把目标遗留在服务端。
        print("\n收到中断，正在取消在途目标...")
        driver.cancel()
        time.sleep(8.0)
    finally:
        if args.stop_after:
            # 兜底：run() 的 finally 已复位过一次，这里在 context 仍有效时再确认。
            try:
                driver.reset_stop_params()
            except Exception as exc:  # noqa: BLE001
                print("警告: 驻留参数复位失败(请手动 ros2 param set 复位): %s" % exc)
        driver.stop()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
