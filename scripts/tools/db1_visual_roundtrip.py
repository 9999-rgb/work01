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

先决条件：活栈（run_all.sh）已就绪、toolset A、无人任务占用。开合之间基座
保持跟随后的位置，close 由该位置直接拉回，不需要重新停靠。
"""
import argparse
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
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from xczs_inspection_robot_interfaces.action import OperateCabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControlState

NS = "/xczs/cabinet/electrical_mezzanine"
CONTROL = "db1"
ACTION = NS + "/operate_cabinet_control"
STATE_TOPIC = NS + "/" + CONTROL + "/state"

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
    def __init__(self):
        super().__init__("db1_visual_roundtrip", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
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

        for stage in stages:
            self._wait(lambda: self._drawer_state is not None, 10.0,
                       "drawer state before %s" % stage)
            q_start = self._drawer_state.position
            outcome, samples = self._operate(stage)
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
            print("execution_back : %s" % getattr(result, "execution_backend", ""))
            print("phys_outcome   : %s" % result.physical_outcome_confirmed)
            print("final_verified : %s" % result.final_state_verified)
            print("transport_ok   : %s" % result.transport_succeeded)
            print("grasp_released : %s" % result.grasp_released)
            base_now, _ = self._base_pose()
            print("drawer rail    : %.4f -> %.4f m (samples=%d)" %
                  (q_start,
                   self._drawer_state.position if self._drawer_state else float("nan"),
                   len(samples)))
            if base_now:
                print("base after     : odom→body %.4f %.4f %.4f m (Δ %.4f %.4f %.4f)" %
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
    args = parser.parse_args()

    rclpy.init()
    driver = Db1VisualRoundtrip()
    try:
        driver.run(args.once, not args.no_preposition, args.csv_out)
    finally:
        driver.stop()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
