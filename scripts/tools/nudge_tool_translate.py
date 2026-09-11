#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双手末端沿世界坐标轴平行微移（抽屉现场教学用）。

从"当前实测位姿"出发，把左/右末端工具沿指定世界轴平移给定距离，姿态保持不变
（纯平移、不转向），用 MoveIt 的 /compute_cartesian_path 算直线路径，再把轨迹
直接下发到对应臂的 ros2_control 控制器执行。

为什么不用 MoveIt 的 ExecuteTrajectory：项目里 move_group.execute() 存在
"结果丢失即永久占用"的已知坑（见 operator 的 execute_motion_bounded 封装），
本工具只借用 MoveIt 的路径规划，执行走控制器 action，不经 move_group 执行器。

作业位姿下的工具轴映射（实测，见 README）：
  工具 X (160.0 mm 指列分离方向) → 世界 y   ——  "宽度"
  工具 Y (194.2 mm 电缸体方向)   → 世界 z   ——  "高度"
  工具 Z (313.5 mm 杆伸出方向)   → 世界 −x  ——  "进给深度"（伸出朝柜体）
所以"左右"= 世界 ±y，"上下"= 世界 ±z，"前后"= 世界 ±x。

用法：
  # 左右各向外平移 40 mm（0.160 m 工具宽度的 1/4）
  python3 scripts/tools/nudge_tool_translate.py --spread 0.040
  # 双手一起下压 48.6 mm（0.1942 m 工具高度的 1/4）
  python3 scripts/tools/nudge_tool_translate.py --axis z --left -0.0486 --right -0.0486
  # 双手一起往前推 19 mm（世界 −x，朝柜体）
  python3 scripts/tools/nudge_tool_translate.py --axis x --left -0.019 --right -0.019
  # 分步内收，走不动就停（每步 2 mm，累计上限 60 mm）
  python3 scripts/tools/nudge_tool_translate.py --search inward --step 0.002
  # 只算路径、不动（先看可不可达、fraction 多少）
  python3 scripts/tools/nudge_tool_translate.py --axis z --left -0.0486 --right -0.0486 --dry-run

先决条件：活栈已就绪、臂控制器可用。工具只读 TF 与关节状态，不改任何参数。
"""
import argparse
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xczs_controllers import (  # noqa: E402
    ARMS, JOINT_STATES_TOPIC, WORLD_FRAME, SpinNode, best_effort_qos, wait_for)
from xczs_inspection_robot_interfaces.msg import CabinetControlState  # noqa: E402

# 受控机构（抽屉等）状态话题前缀；--watch-control 会拼成
# <PREFIX>/<id>/state。场景前缀属于跨层合同，允许命令行覆盖。
CONTROL_STATE_PREFIX = "/xczs/cabinet/electrical_mezzanine"

CARTESIAN_SERVICE = "/compute_cartesian_path"
AXES = ("x", "y", "z")
# 分步搜索模式：{模式: (沿哪个世界轴, 左符号, 右符号)}
#   inward  内收 = 双手相向（y）
#   outward 外张 = 双手相背（y）
#   retract 往回缩 = 双手一起远离柜体（+x）
#   advance 往前伸 = 双手一起朝柜体（−x）
SEARCH_MODES = {
    "inward": ("y", +1.0, -1.0),
    "outward": ("y", -1.0, +1.0),
    "retract": ("x", +1.0, +1.0),
    "advance": ("x", -1.0, -1.0),
}
# 单步实测位移低于指令的这个比例，就认为被挡住了不再前进（与电缸工具同一判据：
# 留出接触初期"压紧"的余地，完全用 0 判据会把启动迟滞误判成顶住）。
STALL_RATIO = 0.4


class ToolTranslateNudge(SpinNode):
    def __init__(self):
        super().__init__("tool_translate_nudge", num_threads=4,
                         parameter_overrides=[
                             Parameter("use_sim_time", value=True)])
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._joint_state = None
        # joint_states 是 best-effort 发布，默认 RELIABLE 订阅收不到消息。
        self._state_sub = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state,
            best_effort_qos())
        self._cartesian_cli = self.create_client(
            GetCartesianPath, CARTESIAN_SERVICE)
        self._arm_clients = {
            side: ActionClient(self, FollowJointTrajectory, cfg["action"])
            for side, cfg in ARMS.items()
        }
        # 受控机构（抽屉）位移。--watch-control 用它判定"是否已经咬住把手"：
        # 钩爪一咬上，继续回缩就会把抽屉带动，position 立刻变化。
        self._watch_control = None
        self._watched_position = None
        self._watch_sub = None

    def watch_control(self, control_id, timeout=10.0):
        """订阅 <PREFIX>/<id>/state，用于判定抽屉是否被带动。"""
        if not control_id:
            return
        self._watch_control = control_id

        def callback(msg):
            self._watched_position = msg.position

        self._watch_sub = self.create_subscription(
            CabinetControlState, "%s/%s/state" % (CONTROL_STATE_PREFIX, control_id),
            callback, 10)
        wait_for(lambda: self._watched_position is not None, timeout,
                 "%s/%s/state" % (CONTROL_STATE_PREFIX, control_id))

    # ------------------------------------------------------------------ infra
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _tip_pose(self, tip, timeout=10.0):
        """世界系(odom)下末端 tip link 的位姿；TF 时间取最新（0 = latest）。"""
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                t = self._tf_buffer.lookup_transform(
                    WORLD_FRAME, tip, Time(), Duration(seconds=0.5))
                p = t.transform.translation
                q = t.transform.rotation
                return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
            except Exception as exc:  # noqa: BLE001
                last = exc
                time.sleep(0.1)
        raise RuntimeError("tf %s->%s unavailable: %s" % (WORLD_FRAME, tip, last))

    def _tip_settled(self, tip, timeout=10.0, tolerance=0.0002, window=0.4):
        """等末端位姿连续 window 秒内变动都小于 tolerance。

        控制器 action 返回 ≠ 机械臂停稳：末段减速仍在走，固定延时后取样会把
        "还在收尾"读成"到位误差"（钩爪电缸那次实测踩过这个坑）。
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            first, _ = self._tip_pose(tip)
            time.sleep(window)
            now, _ = self._tip_pose(tip)
            if all(abs(now[i] - first[i]) < tolerance for i in range(3)):
                return True
        return False

    def _call(self, client, request, timeout, what):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("%s unavailable" % what)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError("%s did not reply within %.0f s" % (what, timeout))
        return future.result()

    # ------------------------------------------------------------- planning
    def plan_translate(self, side, axis, distance, max_step, timeout,
                       avoid_collisions=True):
        """算一条"沿世界 axis 平移 distance、姿态不变"的直线路径。

        返回 (solution_joint_trajectory, fraction, start_xyz, goal_xyz)。
        不填 start_state：move_group 已把 joint_states 重映射到
        /xczs/joint_states（规划帧 odom），其监视状态就是真机状态。
        """
        cfg = ARMS[side]
        (sx, sy, sz), quat = self._tip_pose(cfg["tip"])
        target = Pose()
        target.position.x = sx + (distance if axis == "x" else 0.0)
        target.position.y = sy + (distance if axis == "y" else 0.0)
        target.position.z = sz + (distance if axis == "z" else 0.0)
        target.orientation.x = quat[0]
        target.orientation.y = quat[1]
        target.orientation.z = quat[2]
        target.orientation.w = quat[3]

        request = GetCartesianPath.Request()
        request.header.frame_id = WORLD_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = cfg["group"]
        request.link_name = cfg["tip"]
        request.waypoints = [target]
        request.max_step = max_step
        # 0.0 = 关闭关节跳变检查（单点路径用不到，且会误杀正常解）。
        request.jump_threshold = 0.0
        request.avoid_collisions = avoid_collisions
        response = self._call(
            self._cartesian_cli, request, timeout, CARTESIAN_SERVICE)
        solution = response.solution.joint_trajectory
        goal = (target.position.x, target.position.y, target.position.z)
        return solution, response.fraction, (sx, sy, sz), goal

    # ------------------------------------------------------------- execution
    def execute(self, side, solution, slowdown, timeout):
        """把 MoveIt 算出的轨迹裁到本臂 7 个关节后下发控制器（不经 move_group）。"""
        cfg = ARMS[side]
        wanted = cfg["joints"]
        index = {name: i for i, name in enumerate(solution.joint_names)}
        missing = [j for j in wanted if j not in index]
        if missing:
            raise RuntimeError(
                "%s trajectory is missing joints: %s" % (side, missing))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(wanted)
        scale = max(1.0, slowdown)
        for point in solution.points:
            new_point = type(point)()
            new_point.positions = [point.positions[index[j]] for j in wanted]
            new_point.velocities = (
                [point.velocities[index[j]] for j in wanted]
                if len(point.velocities) == len(solution.joint_names) else [])
            new_point.accelerations = []
            new_point.effort = []
            new_point.time_from_start = self._scaled_time(
                point.time_from_start, scale)
            trajectory.points.append(new_point)

        client = self._arm_clients[side]
        if not client.wait_for_server(timeout_sec=timeout):
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
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not result_future.done() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            raise RuntimeError("%s did not finish the trajectory in %.0f s"
                               % (side, timeout))
        return result_future.result().result

    @staticmethod
    def _scaled_time(stamp, scale):
        """把 time_from_start 放大 scale 倍（>1 = 更慢），返回新 Duration。"""
        total = stamp.sec + stamp.nanosec * 1e-9
        total *= scale
        out = type(stamp)()
        out.sec = int(total)
        out.nanosec = int((total - int(total)) * 1e9)
        return out

    # ------------------------------------------------------------------- main
    def _ready(self, timeout):
        wait_for(lambda: self._joint_state is not None, 15.0,
                 JOINT_STATES_TOPIC)
        wait_for(lambda: self._cartesian_cli.service_is_ready()
                 or self._cartesian_cli.wait_for_service(timeout_sec=1.0),
                 20.0, CARTESIAN_SERVICE)
        for side in ARMS:
            self._tip_pose(ARMS[side]["tip"])

    def run(self, offsets, axis, dry_run, max_step, slowdown, min_fraction,
            avoid_collisions, timeout):
        self._ready(timeout)
        before = {}
        plans = {}
        print("\n==== 末端沿世界 %s 轴平行微移 ====" % axis)
        for side, distance in offsets.items():
            if abs(distance) < 1e-9:
                continue
            solution, fraction, start, goal = self.plan_translate(
                side, axis, distance, max_step, timeout, avoid_collisions)
            before[side] = start
            plans[side] = (solution, fraction)
            print("%-5s 起点 odom (%+.4f %+.4f %+.4f) -> 目标 (%+.4f %+.4f %+.4f)"
                  "  Δ%s %+.4f m  路径完整度 %.4f  轨迹点 %d" %
                  (side, start[0], start[1], start[2],
                   goal[0], goal[1], goal[2], axis, distance, fraction,
                   len(solution.points)))

        if dry_run:
            print("--dry-run：只算路径，未执行。")
            return

        # 完整度不够就整批拒绝：宁可不动，也不要走半截停在半路。
        short = {s: f for s, (_sol, f) in plans.items() if f < min_fraction}
        if short:
            raise RuntimeError(
                "cartesian path incomplete (min %.3f): %s"
                % (min_fraction,
                   ", ".join("%s %.4f" % (s, f) for s, f in short.items())))
        for side, (solution, _fraction) in plans.items():
            if len(solution.points) == 0:
                raise RuntimeError("%s: empty trajectory" % side)
            print("执行 %-5s ..." % side)
            result = self.execute(side, solution, slowdown, timeout)
            print("%-5s 控制器结果 error_code=%s" % (side, result.error_code))

        index = AXES.index(axis)
        print("\n---- 实测位移 ----")
        for side, start in before.items():
            if not self._tip_settled(ARMS[side]["tip"]):
                print("%-5s 警告: 10 s 内未观测到末端静止，实测值可能仍在动。"
                      % side)
            (nx, ny, nz), _ = self._tip_pose(ARMS[side]["tip"])
            delta = (nx - start[0], ny - start[1], nz - start[2])
            others = ", ".join("%s %+.4f" % (ax, delta[i])
                               for i, ax in enumerate(AXES) if i != index)
            print("%-5s odom (%+.4f %+.4f %+.4f) -> (%+.4f %+.4f %+.4f)"
                  "  Δ%s %+.4f m (误差 %+.2f mm)  其余轴: %s" %
                  (side, start[0], start[1], start[2], nx, ny, nz, axis,
                   delta[index],
                   (delta[index] - offsets[side]) * 1000.0, others))
        print("=================================")

    def run_search(self, mode, step, max_travel, dry_run, max_step,
                   slowdown, min_fraction, avoid_collisions, timeout,
                   watch_threshold):
        """分步走，直到走不动、或把受控机构带起来为止。

        停止判据（任一命中即停）：
          · 规划完整度 < min_fraction —— MoveIt 直接说前面过不去
          · 单步实测位移 < 40% 指令 —— 走出去了但被物理挡住
          · 受控机构位移 > watch_threshold —— 钩爪咬住把手、抽屉被带动
          · 累计行程到 --max-travel —— 安全阀
        两侧同步走：一侧被挡就两侧一起停，保持双手平行（现场一直要求"平行"）。
        """
        axis, left_sign, right_sign = SEARCH_MODES[mode]
        sign = {"left": left_sign, "right": right_sign}
        self._ready(timeout)
        max_iters = max(1, int(max_travel / step) + 4)
        watch = self._watch_control
        start_position = self._watched_position if watch else None
        print("\n==== 末端分步%s（世界 %s 轴，步长 %.1f mm，累计上限 %.1f mm）===="
              % (mode, axis, step * 1000, max_travel * 1000))
        if watch:
            print("监视受控机构 %s：起始 position %s，位移超过 %.1f mm 即判定咬住。"
                  % (watch,
                     "%.6f" % start_position if start_position is not None
                     else "(未就绪)", watch_threshold * 1000))

        travelled = {side: 0.0 for side in ARMS}
        index = AXES.index(axis)
        stopped = None
        for iteration in range(1, max_iters + 1):
            plans = {}
            for side in ARMS:
                solution, fraction, start, _goal = self.plan_translate(
                    side, axis, sign[side] * step, max_step, timeout,
                    avoid_collisions)
                plans[side] = (solution, fraction, start)
            blocked = {s: f for s, (_sol, f, _st) in plans.items()
                       if f < min_fraction}
            if blocked:
                stopped = "规划失败（前面过不去）：%s" % ", ".join(
                    "%s 完整度 %.4f" % (s, f) for s, f in blocked.items())
                break
            if dry_run:
                print("--dry-run：两侧完整度 %s，未执行。"
                      % ", ".join("%s %.4f" % (s, plans[s][1]) for s in ARMS))
                return

            for side, (solution, _f, _st) in plans.items():
                if len(solution.points) == 0:
                    raise RuntimeError("%s: empty trajectory" % side)
                result = self.execute(side, solution, slowdown, timeout)
                if result.error_code != 0:
                    stopped = "%s 控制器 error_code=%s" % (side,
                                                          result.error_code)
                    break
            if stopped:
                break

            moved = {}
            for side, (_sol, _f, start) in plans.items():
                if not self._tip_settled(ARMS[side]["tip"]):
                    print("  警告: %s 末端 10 s 内未静止，读数可能仍在动。" % side)
                now, _ = self._tip_pose(ARMS[side]["tip"])
                moved[side] = now[index] - start[index]
                travelled[side] += moved[side]
            line = "  第 %2d 步  左 Δ%s %+.2f mm（累计 %+.2f）   右 Δ%s %+.2f mm（累计 %+.2f）" % (
                iteration, axis, moved["left"] * 1000, travelled["left"] * 1000,
                axis, moved["right"] * 1000, travelled["right"] * 1000)
            if watch and self._watched_position is not None:
                delta = self._watched_position - start_position
                line += "   %s Δ %.2f mm" % (watch, delta * 1000)
            print(line)

            if watch and self._watched_position is not None:
                delta = self._watched_position - start_position
                if abs(delta) > watch_threshold:
                    stopped = "咬住：%s 已被带动 %.2f mm（判定阈值 %.1f mm）" % (
                        watch, delta * 1000, watch_threshold * 1000)
                    break

            short = {s: m for s, m in moved.items()
                     if abs(m) < abs(step) * STALL_RATIO}
            if short:
                stopped = "顶住：%s" % ", ".join(
                    "%s 本步只走出 %.2f mm（指令 %.2f mm）"
                    % (s, m * 1000, step * 1000) for s, m in short.items())
                break

        if stopped:
            print("\n停止 —— %s" % stopped)
        else:
            print("\n停止 —— 到达累计上限 %.1f mm。" % (max_travel * 1000))
        print("---- 累计位移 ----")
        for side in ARMS:
            print("  %-5s Δ%s %+.4f m (%+.1f mm)"
                  % (side, axis, travelled[side], travelled[side] * 1000))
        if watch and self._watched_position is not None:
            print("  %s position %.6f -> %.6f（Δ %+.2f mm）"
                  % (watch, start_position, self._watched_position,
                     (self._watched_position - start_position) * 1000))
        print("=================================")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--axis", choices=list(AXES), default="y",
                        help="平移沿哪个世界轴，默认 y（左右）")
    parser.add_argument("--spread", type=float, default=None,
                        help="仅 --axis y 可用：左右各向外平移的距离（m），"
                             "左 -spread，右 +spread。例：0.160 的 1/4 = 0.040")
    parser.add_argument("--left", type=float, default=None,
                        help="左末端沿 --axis 的位移（m）")
    parser.add_argument("--right", type=float, default=None,
                        help="右末端沿 --axis 的位移（m）")
    parser.add_argument("--search", choices=list(SEARCH_MODES), default=None,
                        help="分步走直到走不动或把机构带起来：inward=内收、"
                             "outward=外张、retract=往回缩、advance=往前伸。"
                             "与 --spread/--left/--right 互斥")
    parser.add_argument("--step", type=float, default=0.002,
                        help="--search 的步长（m），默认 0.002")
    parser.add_argument("--max-travel", type=float, default=0.060,
                        help="--search 的累计行程上限（m），默认 0.060")
    parser.add_argument("--watch-control", default=None, metavar="ID",
                        help="监视某个受控机构（如 db1）的 position：位移超过 "
                             "--watch-threshold 就判定咬住并停止。--search 用")
    parser.add_argument("--watch-threshold", type=float, default=0.003,
                        help="判定咬住的机构位移阈值（m），默认 0.003")
    parser.add_argument("--dry-run", action="store_true",
                        help="只算笛卡尔路径并报告完整度，不执行")
    parser.add_argument("--max-step", type=float, default=0.005,
                        help="笛卡尔插补步长（m），默认 0.005")
    parser.add_argument("--slowdown", type=float, default=1.0,
                        help="时间轴放慢倍数（>1 = 更慢），默认 1.0")
    parser.add_argument("--min-fraction", type=float, default=0.99,
                        help="路径完整度下限，低于此值拒绝执行，默认 0.99")
    parser.add_argument("--allow-collisions", action="store_true",
                        help="规划时不做碰撞检查（默认检查）")
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="单臂规划/执行超时（s），默认 60")
    args = parser.parse_args()

    explicit = [args.spread is not None, args.left is not None,
                args.right is not None]
    if args.search is not None:
        if any(explicit):
            parser.error("--search 与 --spread/--left/--right 不能同时给")
        offsets = None
    else:
        if args.spread is not None:
            if args.axis != "y":
                parser.error("--spread 只在 --axis y 下有意义")
            if args.left is not None or args.right is not None:
                parser.error("--spread 与 --left/--right 不能同时给")
            left, right = -args.spread, args.spread
        else:
            left, right = args.left, args.right
        if left is None and right is None:
            parser.error("需要 --spread、--left/--right，或 --search")
        offsets = {"left": left or 0.0, "right": right or 0.0}

    rclpy.init()
    node = ToolTranslateNudge()
    try:
        if args.search is not None:
            node.watch_control(args.watch_control)
            node.run_search(args.search, args.step, args.max_travel,
                            args.dry_run, args.max_step, args.slowdown,
                            args.min_fraction, not args.allow_collisions,
                            args.timeout, args.watch_threshold)
        else:
            node.run(offsets, args.axis, args.dry_run, args.max_step,
                     args.slowdown, args.min_fraction,
                     not args.allow_collisions, args.timeout)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
