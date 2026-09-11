#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""教学动作序列的 ROS 执行节点（被 jiang/control_gateway/taught_sequence.py 调用）。

只做三件事，逐一对应序列里的步骤类型：

  ``step_preposition_base``  送底盘到控件工位（复用 preposition_base.py 的做法）
  ``step_restore_pose``      按 taught_poses/*.yaml 下发臂 + 全部电缸杆
  ``step_pull_drawer``       联动抽拉：抽屉轨道运动学播放 + 双臂同距同时长后拉

为什么单独一个文件：``taught_sequence.py`` 属于 Web 任务层，不直接碰 ROS；
ROS 细节集中在这里，两边职责清楚。

三个现场踩过的坑都固化在本文件里（详见各处注释）：
  ① 操作租约单次上限 5s → 必须后台续租；
  ② 播放的 ``header.stamp`` 必须为 0，插件用节点墙钟判定；
  ③ 轨道起点会读成负的微小值 → 下发前必须钳进关节限位。
"""
from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional

import rclpy
import rclpy.executors
import rclpy.time
import tf2_ros
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xczs_inspection_robot_interfaces.msg import CabinetControlState
from xczs_inspection_robot_interfaces.srv import (
    ManageOperationLease, SetCabinetPlayback)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xczs_controllers import (  # noqa: E402
    ARMS, JOINT_STATES_TOPIC, ROD_SIDES, WORLD_FRAME, SpinNode, best_effort_qos,
    wait_for)
from sensor_msgs.msg import JointState  # noqa: E402

WORKSPACE = Path(__file__).resolve().parents[2]
TAUGHT_POSES_DIR = (
    WORKSPACE / "xczs_inspection_robot_control" / "config" / "taught_poses"
)
ADAPTER = (
    WORKSPACE / "xczs_inspection_robot_control" / "config" / "scene_controls"
    / "electrical_mezzanine_adapter.yaml"
)
# 机器人"初始姿势"的权威来源：机器人启动时的姿态就是这份文件定义的。
INITIAL_POSITIONS = (
    WORKSPACE / "xczs_inspection_robot_description" / "config"
    / "initial_positions.yaml"
)
CARTESIAN_SERVICE = "/compute_cartesian_path"
LEASE_SERVICE = "/xczs/operation_lease"
OWNER_ID = "taught_sequence"
LEASE_SECONDS = 5.0          # 坑①：协调器把单次租约限死在 5s，长动作靠续租
LEASE_RENEW_PERIOD = 1.5
RAIL_LIMIT = 0.38
# 杆的物理行程（URDF <limit>）；存档里的越限值必须先钳进来，否则位置伺服
# 会和限位对顶，杆振几十毫米并把整条臂带抖。
ROD_LIMITS = {"default": (0.0, 0.12),
              "r_three_cyl_finger3_joint": (0.0, 0.0254),
              "l_rocker_rotor_joint": (0.0, 0.0254)}
RETREAT_TOLERANCE = 0.001


def _clamp_rod(joint: str, value: float) -> float:
    low, high = ROD_LIMITS.get(joint, ROD_LIMITS["default"])
    return max(low, min(high, value))


class TaughtRosWorker(SpinNode):
    def __init__(self, cabinet: str, cancel_event: threading.Event,
                 context: Optional[Any] = None):
        # Web 任务层用的是**私有 rclpy Context**（runner 里 Context() +
        # rclpy.init(context=...)），节点必须建在同一个 context 上，
        # 否则报 "rclpy.init() has not been called"。独立工具调用时传 None
        # 就用默认 context（它们自己做过 rclpy.init()）。
        super().__init__("taught_sequence_worker", num_threads=4,
                         context=context,
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self._cabinet = cabinet
        self._cancel = cancel_event
        self._joint_state = None
        self._rail = None
        self._controls: Dict[str, Any] = {}
        self.create_subscription(JointState, JOINT_STATES_TOPIC,
                                 self._on_joint_state, best_effort_qos())
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cartesian_cli = self.create_client(GetCartesianPath,
                                                 CARTESIAN_SERVICE)
        self._lease_cli = self.create_client(ManageOperationLease, LEASE_SERVICE)
        self._arm_clients = {
            side: ActionClient(self, FollowJointTrajectory, cfg["action"])
            for side, cfg in ARMS.items()
        }
        self._rod_clients = {
            side: ActionClient(self, FollowJointTrajectory, cfg["action"])
            for side, cfg in ROD_SIDES.items()
        }
        self._playback_cli: Optional[Any] = None
        self._renew_stop: Optional[threading.Event] = None
        self._renew_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ infra
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _state_cb(self, control_id):
        def callback(msg):
            self._controls[control_id] = msg
        return callback

    def wait_ready(self, timeout: float = 30.0) -> None:
        wait_for(lambda: self._joint_state is not None, timeout,
                 JOINT_STATES_TOPIC)
        wait_for(lambda: self._cartesian_cli.service_is_ready()
                 or self._cartesian_cli.wait_for_service(timeout_sec=1.0),
                 timeout, CARTESIAN_SERVICE)

    def shutdown(self) -> None:
        self.stop_renewing()
        self.stop()

    def _check_cancel(self) -> None:
        if self._cancel.is_set():
            raise RuntimeError("已取消")

    def _measured(self, joint: str) -> float:
        names = list(self._joint_state.name)
        if joint not in names:
            raise RuntimeError("关节 %s 不在 %s 里" % (joint, JOINT_STATES_TOPIC))
        return float(self._joint_state.position[names.index(joint)])

    def _settled(self, joints: List[str], timeout: float = 15.0,
                 tolerance: float = 0.0002, window: float = 0.5) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            first = {j: self._measured(j) for j in joints}
            time.sleep(window)
            if all(abs(self._measured(j) - first[j]) < tolerance
                   for j in joints):
                return True
        return False

    # --------------------------------------------------------- 控制器 / 规划
    def _send(self, client, joints, targets, duration) -> int:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joints)
        point = JointTrajectoryPoint()
        point.positions = [float(targets[j]) for j in joints]
        point.time_from_start = Duration(seconds=float(duration)).to_msg()
        trajectory.points = [point]
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("控制器 action 不可用")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = 5
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 15.0
        while self.context.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            # JTC 的 goal 响应只有 accepted=false，没有原因字段。把下发内容与
            # 控制器当前的可用性一并抛出，否则现场只能看到"被拒"三个字。
            available = client.server_is_ready()
            raise RuntimeError(
                "控制器拒绝了目标（action=%s, server_is_ready=%s, "
                "joints=%s, targets=%s, duration=%.2f）"
                % (client._action_name if hasattr(client, "_action_name") else "?",
                   available, list(joints),
                   ["%.5f" % float(targets[j]) for j in joints], float(duration)))
        result_future = handle.get_result_async()
        deadline = time.monotonic() + max(40.0, duration * 6.0)
        while self.context.ok() and not result_future.done() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            raise RuntimeError("控制器未在时限内完成")
        return int(result_future.result().result.error_code)

    def _call(self, client, request, timeout, what):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("%s 不可用" % what)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while self.context.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError("%s 未在 %.0fs 内回复" % (what, timeout))
        return future.result()

    def _tip_pose(self, tip: str, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                t = self._tf_buffer.lookup_transform(WORLD_FRAME, tip,
                                                     rclpy.time.Time(),
                                                     Duration(seconds=0.5))
                p = t.transform.translation
                q = t.transform.rotation
                return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
            except Exception as exc:  # noqa: BLE001
                last = exc
                time.sleep(0.1)
        raise RuntimeError("TF %s→%s 不可用: %s" % (WORLD_FRAME, tip, last))

    def _plan_translate(self, side: str, axis: str, distance: float,
                        max_step: float = 0.005, timeout: float = 60.0):
        cfg = ARMS[side]
        (sx, sy, sz), quat = self._tip_pose(cfg["tip"])
        target = Pose()
        target.position.x = sx + (distance if axis == "x" else 0.0)
        target.position.y = sy + (distance if axis == "y" else 0.0)
        target.position.z = sz + (distance if axis == "z" else 0.0)
        target.orientation.x, target.orientation.y = quat[0], quat[1]
        target.orientation.z, target.orientation.w = quat[2], quat[3]
        request = GetCartesianPath.Request()
        request.header.frame_id = WORLD_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = cfg["group"]
        request.link_name = cfg["tip"]
        request.waypoints = [target]
        request.max_step = max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        response = self._call(self._cartesian_cli, request, timeout,
                              CARTESIAN_SERVICE)
        return response.solution.joint_trajectory, response.fraction, (sx, sy, sz)

    def _execute_plan(self, side: str, solution, scale: float,
                      timeout: float) -> int:
        """把 MoveIt 算出的臂轨迹裁到本臂 7 关节，时间轴缩放后下发控制器。"""
        cfg = ARMS[side]
        wanted = cfg["joints"]
        index = {name: i for i, name in enumerate(solution.joint_names)}
        missing = [j for j in wanted if j not in index]
        if missing:
            raise RuntimeError("%s 轨迹缺少关节 %s" % (side, missing))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(wanted)
        for point in solution.points:
            new_point = JointTrajectoryPoint()
            new_point.positions = [point.positions[index[j]] for j in wanted]
            total = (point.time_from_start.sec
                     + point.time_from_start.nanosec * 1e-9) * max(1.0, scale)
            new_point.time_from_start = Duration(seconds=total).to_msg()
            trajectory.points.append(new_point)
        return self._send_trajectory(side, trajectory, timeout)

    def _send_trajectory(self, side: str, trajectory, timeout: float) -> int:
        client = self._arm_clients[side]
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("%s 控制器不可用" % ARMS[side]["action"])
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = 5
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 15.0
        while self.context.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("%s 拒绝了轨迹" % side)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while self.context.ok() and not result_future.done() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            raise RuntimeError("%s 未在 %.0fs 内完成轨迹" % (side, timeout))
        return int(result_future.result().result.error_code)

    # ------------------------------------------------------------- 步骤 1 送底盘
    def step_preposition_base(self, step: Mapping[str, Any],
                              on_progress: Callable) -> None:
        control = str(step["control"])
        cabinet = str(step.get("cabinet") or self._cabinet)
        toolset = str(step.get("toolset") or "A")
        on_progress(0.1, "把底盘送到 %s 工位" % control)
        command = [
            sys.executable,
            str(WORKSPACE / "scripts" / "tools" / "preposition_base.py"),
            "--control", control, "--cabinet", cabinet,
            "--toolset", toolset, "--adapter", str(ADAPTER),
        ]
        completed = subprocess.run(command, cwd=str(WORKSPACE),
                                   capture_output=True, text=True)
        if completed.returncode != 0:
            raise RuntimeError("送底盘失败: %s"
                               % (completed.stderr or completed.stdout).strip())
        time.sleep(1.0)          # 让 AMCL / 位姿权威同步
        on_progress(1.0, "底盘已就位")

    # ------------------------------------------------------------- 步骤 2 还原位姿
    def step_restore_pose(self, step: Mapping[str, Any],
                          on_progress: Callable) -> None:
        name = str(step["pose"])
        pose_path = Path(name)
        if not pose_path.is_absolute():
            pose_path = TAUGHT_POSES_DIR / name
        with pose_path.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle)
        targets = {k: float(v) for k, v in payload["joints"].items()}
        rod_joints = {j for cfg in ROD_SIDES.values() for j in cfg["joints"]}
        arm_joints = {j for cfg in ARMS.values() for j in cfg["joints"]}
        for joint in sorted(rod_joints):
            targets[joint] = _clamp_rod(joint, targets[joint])

        on_progress(0.1, "退杆到 0")
        for side, cfg in ROD_SIDES.items():
            values = {j: 0.0 for j in cfg["joints"]}
            if all(abs(values[j] - self._measured(j)) < 1e-6
                   for j in cfg["joints"]):
                continue
            self._send(self._rod_clients[side], cfg["joints"], values, 3.0)
        self._settled(sorted(rod_joints))
        stuck = [j for j in sorted(rod_joints)
                 if abs(self._measured(j) - 0.0) > RETREAT_TOLERANCE]
        if stuck:
            raise RuntimeError("电缸没退到位（多半是顶住了），已中止且未动机械臂: %s"
                               % ", ".join(stuck))

        self._check_cancel()
        on_progress(0.4, "双臂到位")
        for side, cfg in ARMS.items():
            self._send(self._arm_clients[side], cfg["joints"], targets, 5.0)
        self._settled(sorted(arm_joints), timeout=20.0)

        self._check_cancel()
        on_progress(0.8, "电缸伸到目标值")
        for side, cfg in ROD_SIDES.items():
            self._send(self._rod_clients[side], cfg["joints"], targets, 3.0)
        self._settled(sorted(rod_joints))
        on_progress(1.0, "已还原教学位姿 %s" % pose_path.name)

    # ------------------------------------------------- 步骤 3b 末端整体平移
    def step_translate_tool(self, step: Mapping[str, Any],
                            on_progress: Callable) -> None:
        """双臂一起沿某个世界轴平移一段（姿态不变）。

        用途：闭合时抽屉已开着、把手在 +0.25 m 处，而 001 位姿的手在 0 处——
        先把手平移到把手上，再执行推回，视觉上就是"手搭上把手往里推"。
        """
        axis = str(step["axis"])
        distance = float(step["distance"])
        if axis not in ("x", "y", "z"):
            raise RuntimeError("translate_tool 的 axis 只能是 x/y/z，收到 %r" % axis)
        duration = float(step.get("duration") or 0.0)
        on_progress(0.05, "末端沿世界 %s 平移 %+.3f m" % (axis, distance))
        plans = {}
        for side in ARMS:
            solution, fraction, tip = self._plan_translate(side, axis, distance)
            if fraction < 0.99 or not solution.points:
                raise RuntimeError("%s 平移路径完整度仅 %.4f" % (side, fraction))
            plans[side] = solution
        scale = 1.0
        if duration > 0.0:
            plan_time = max(
                (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9)
                for solution in plans.values() for p in solution.points)
            scale = max(1.0, duration / plan_time) if plan_time > 0 else 1.0
        for index, (side, solution) in enumerate(plans.items()):
            self._check_cancel()
            on_progress(0.1 + 0.85 * index / float(len(plans)),
                        "平移（%s）" % side)
            code = self._execute_plan(side, solution, scale,
                                      max(120.0, duration * 4 or 120.0))
            if code != 0:
                raise RuntimeError("%s 控制器 error_code=%s" % (side, code))
        for side in ARMS:
            self._tip_settled(ARMS[side]["tip"])
        on_progress(1.0, "末端平移完成")

    # ------------------------------------------------------- 步骤 4 收回电缸杆
    def step_retract_rods(self, step: Mapping[str, Any],
                          on_progress: Callable) -> None:
        """把所有电缸杆收到 0。动臂之前必须先把杆收回，顺序不能反。"""
        on_progress(0.1, "收回全部电缸杆")
        rod_joints = {j for cfg in ROD_SIDES.values() for j in cfg["joints"]}
        for side, cfg in ROD_SIDES.items():
            values = {j: 0.0 for j in cfg["joints"]}
            if all(abs(values[j] - self._measured(j)) < 1e-6
                   for j in cfg["joints"]):
                continue
            self._send(self._rod_clients[side], cfg["joints"], values, 3.0)
        self._settled(sorted(rod_joints))
        stuck = [j for j in sorted(rod_joints)
                 if abs(self._measured(j)) > RETREAT_TOLERANCE]
        if stuck:
            raise RuntimeError("有电缸没收到位（多半是顶住了）: %s"
                               % ", ".join(stuck))
        on_progress(1.0, "电缸杆已全部收回")

    # ------------------------------------------------- 步骤 5 机械臂回初始姿势
    def step_go_home(self, step: Mapping[str, Any],
                     on_progress: Callable) -> None:
        """双臂回到 URDF 的初始位姿（``initial_positions.yaml``）。

        机器人刚启动时的姿态就是这个文件定义的，所以"回初始姿势"用它最权威，
        不需要另外抓一份存档。
        """
        with INITIAL_POSITIONS.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle)
        targets = payload["initial_positions"]
        if not isinstance(targets, Mapping) or not targets:
            raise RuntimeError("%s 里没有 initial_positions" % INITIAL_POSITIONS)
        on_progress(0.1, "机械臂回到初始姿势")
        for side, cfg in ARMS.items():
            joints = [j for j in cfg["joints"] if j in targets]
            missing = [j for j in cfg["joints"] if j not in targets]
            if missing:
                raise RuntimeError("初始位姿缺少 %s 的关节: %s"
                                   % (side, ", ".join(missing)))
            if all(abs(float(targets[j]) - self._measured(j)) < 1e-6
                   for j in joints):
                continue
            self._send(self._arm_clients[side], joints,
                       {j: float(targets[j]) for j in joints}, 5.0)
        self._settled([j for cfg in ARMS.values() for j in cfg["joints"]],
                      timeout=20.0)
        on_progress(1.0, "机械臂已回到初始姿势")

    # ------------------------------------------------------------- 步骤 3 联动抽拉
    def step_pull_drawer(self, step: Mapping[str, Any],
                         on_progress: Callable) -> None:
        control = str(step["control"])
        distance = float(step["distance"])
        duration = float(step["duration"])
        topic = "/xczs/cabinet/%s/%s/state" % (self._cabinet, control)
        self.create_subscription(CabinetControlState, topic,
                                 self._state_cb(control), 10)
        wait_for(lambda: control in self._controls, 15.0, topic)
        # 坑③：关到位时轨道位置实测是负的微小值，而下限是 0 → 必须钳进限位，
        # 否则插件按 `q < lower` 直接拒收。
        start = max(0.0, min(RAIL_LIMIT, self._controls[control].position))
        if start + distance > RAIL_LIMIT:
            raise RuntimeError("目标 %.4f 超过轨道上限 %.2f"
                               % (start + distance, RAIL_LIMIT))

        on_progress(0.05, "规划双臂后拉路径")
        plans = {}
        for side in ARMS:
            solution, fraction, tip = self._plan_translate(side, "x", distance)
            if fraction < 0.99 or not solution.points:
                raise RuntimeError("%s 后拉路径完整度仅 %.4f" % (side, fraction))
            plans[side] = (solution, tip)
        plan_time = max(
            (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9)
            for solution, _tip in plans.values() for p in solution.points)
        scale = max(1.0, duration / plan_time) if plan_time > 0 else 1.0

        playback = self.create_client(
            SetCabinetPlayback, "/xczs/cabinet/%s/playback" % self._cabinet)
        self._playback_cli = playback
        lease_id = self._acquire_lease()
        self._start_renewing(lease_id)
        try:
            on_progress(0.15, "启动抽屉轨道播放")
            self._playback_control = control
            self._start_playback(playback, lease_id, control, start,
                                 distance, duration)
            # **两条臂必须并发**：抽屉的时间表是时长 duration 的一条线，
            # 若串行执行（左 6 s 再右 6 s），抽屉 6 s 就走完而臂要 12 s——
            # 全程错位，钩爪（跨在把手两侧）会被抽屉拖着穿过把手，实测表现为
            # "抽拉过程中不断穿模"。各起一个线程，与抽屉共用同一条时间线。
            results: Dict[str, Any] = {}

            def run_arm(side_name, arm_solution):
                try:
                    results[side_name] = self._execute_plan(
                        side_name, arm_solution, scale, max(120.0, duration * 4))
                except Exception as error:  # noqa: BLE001
                    results[side_name] = error

            threads = []
            for index, (side, (solution, _tip)) in enumerate(plans.items()):
                self._check_cancel()
                on_progress(0.25 + 0.6 * index / float(len(plans)),
                            "双臂后拉（%s）" % side)
                thread = threading.Thread(target=run_arm, args=(side, solution),
                                          name="taught-pull-%s" % side,
                                          daemon=True)
                thread.start()
                threads.append(thread)
            for thread in threads:
                thread.join(timeout=max(180.0, duration * 6))
            for side in plans:
                outcome = results.get(side)
                if isinstance(outcome, Exception):
                    raise RuntimeError("%s 后拉失败: %s" % (side, outcome))
                if outcome != 0:
                    raise RuntimeError("%s 控制器 error_code=%s" % (side, outcome))
            for side in ARMS:
                self._tip_settled(ARMS[side]["tip"])
            time.sleep(1.0)
            on_progress(1.0, "抽屉轨道 %.4f → %.4f m"
                        % (start, self._controls[control].position))
        finally:
            self._stop_playback(playback, lease_id)
            self.stop_renewing()
            self._release_lease(lease_id)

    def _tip_settled(self, tip: str, timeout: float = 15.0,
                     tolerance: float = 0.0002, window: float = 0.4) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            first, _ = self._tip_pose(tip)
            time.sleep(window)
            now, _ = self._tip_pose(tip)
            if all(abs(now[i] - first[i]) < tolerance for i in range(3)):
                return True
        return False

    # ------------------------------------------------------------- 租约 / 播放
    def _acquire_lease(self) -> str:
        request = ManageOperationLease.Request()
        request.command = ManageOperationLease.Request.ACQUIRE
        request.owner_id = OWNER_ID
        request.lease_id = ""
        request.requested_duration = LEASE_SECONDS
        response = self._call(self._lease_cli, request, 15.0, LEASE_SERVICE)
        if not response.success:
            raise RuntimeError("获取操作租约失败: %s" % response.message)
        return str(response.lease_id)

    def _start_renewing(self, lease_id: str) -> None:
        self._renew_stop = threading.Event()

        def loop():
            while not self._renew_stop.wait(LEASE_RENEW_PERIOD):
                request = ManageOperationLease.Request()
                request.command = ManageOperationLease.Request.RENEW
                request.owner_id = OWNER_ID
                request.lease_id = lease_id
                request.requested_duration = LEASE_SECONDS
                try:
                    self._call(self._lease_cli, request, 5.0, LEASE_SERVICE)
                except Exception:  # noqa: BLE001
                    pass

        self._renew_thread = threading.Thread(target=loop, daemon=True)
        self._renew_thread.start()

    def stop_renewing(self) -> None:
        if self._renew_stop is not None:
            self._renew_stop.set()
        if self._renew_thread is not None and self._renew_thread.is_alive():
            self._renew_thread.join(timeout=3.0)

    def _release_lease(self, lease_id: str) -> None:
        request = ManageOperationLease.Request()
        request.command = ManageOperationLease.Request.RELEASE
        request.owner_id = OWNER_ID
        request.lease_id = lease_id
        request.requested_duration = 0.0
        try:
            self._call(self._lease_cli, request, 10.0, LEASE_SERVICE)
        except Exception:  # noqa: BLE001
            pass

    def _start_playback(self, client, lease_id, control, start, distance,
                        duration) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = [control]
        # 坑②：header.stamp 必须为 0（立即执行）。插件用**节点墙钟**判定，
        # 填仿真时间会被算成"过去 17 亿秒"而拒收。
        trajectory.header.stamp.sec = 0
        trajectory.header.stamp.nanosec = 0
        for elapsed, value in ((0.0, start), (duration, start + distance)):
            point = JointTrajectoryPoint()
            point.positions = [value]
            point.time_from_start = Duration(seconds=elapsed).to_msg()
            trajectory.points.append(point)
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_START
        request.control_id = control
        request.operation_lease_id = lease_id
        request.trajectory = trajectory
        response = self._call(client, request, 15.0, "playback")
        if not response.success:
            raise RuntimeError("抽屉播放被拒: %s" % response.message)

    def _stop_playback(self, client, lease_id) -> None:
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_RELEASE
        request.control_id = getattr(self, "_playback_control", "")
        request.operation_lease_id = lease_id
        try:
            self._call(client, request, 10.0, "playback")
        except Exception:  # noqa: BLE001
            pass
