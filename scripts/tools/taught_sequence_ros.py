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

import json
import math
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional

import rclpy
import rclpy.executors
import rclpy.time
import tf2_ros
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionFK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive
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
# 解锁点的几何在 controls（按控件给），杆端坐标在 adapter（按工具给）——
# 两份都是跨层合同，各从各自的地方读，不混。
CONTROLS = (
    WORKSPACE / "xczs_inspection_robot_control" / "config" / "scene_controls"
    / "electrical_mezzanine_controls.yaml"
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
# 退杆守卫的**硬**上限：超过它才中止，<= 它只警告。物理杆偶有 1~2mm 顶滞，
# 按 1mm 判死会让整套动作白跑（实测闭合因此反复失败）。
ROD_RETREAT_HARD_LIMIT = 0.004


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
        # 轨道位置：优先取 joint_states（每控件都有），state 话题只 db1 有
        self._rails: Dict[str, float] = {}
        self._rail_subscriptions = {}
        self.create_subscription(JointState, JOINT_STATES_TOPIC,
                                 self._on_joint_state, best_effort_qos())
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cartesian_cli = self.create_client(GetCartesianPath,
                                                 CARTESIAN_SERVICE)
        self._fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self._motion_plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._planning_scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
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
        self._current_lease_id: Optional[str] = None
        self._renew_stop: Optional[threading.Event] = None
        self._renew_thread: Optional[threading.Thread] = None

    def _ensure_rail_subscription(self, control):
        # MultiThreadedExecutor 可能正准备读取订阅，不能在任务线程提前销毁。
        # 持有至 shutdown 停止 executor 后，再由 destroy_node 统一释放。
        if control not in self._rail_subscriptions:
            topic = "/xczs/cabinet/%s/%s/joint_states" % (self._cabinet, control)
            self._rail_subscriptions[control] = self.create_subscription(
                JointState, topic, self._rail_cb(control), best_effort_qos())

    def drawer_already_at_target(self, control, target):
        """整条动作开始前检查位置，重复点击不再驱动底盘与机械臂。"""
        self._ensure_rail_subscription(control)
        wait_for(lambda: control in self._rails, 5.0, "抽屉轨道反馈")
        position = self._rails[control]
        if not math.isfinite(position):
            raise RuntimeError("抽屉位置反馈无效")
        if abs(position - target) > 0.003:
            return False
        self.drawer_result = {
            "simulation_outcome_confirmed": True,
            "final_position": position, "target_position": target,
            "position_tolerance": 0.003, "already_at_target": True,
        }
        return True

    def can_continue_drawer_close(self, control, support_enabled):
        """仅实测仍处在扣手开位时省略重新准备，不依赖上次任务成功标志。"""
        path = TAUGHT_POSES_DIR / (control + "_opened.yaml")
        if not path.is_file():
            return False
        targets = yaml.safe_load(path.read_text())["joints"]
        contract = self._rod_contract()
        for side in ROD_SIDES:
            gripper = 0.00375 if support_enabled else 0.01875
            if abs(self._measured(contract[side]["gripper"]) - gripper) > 0.003:
                return False
            support = 0.115 if support_enabled else 0.0
            if abs(self._measured(contract[side]["support"]) - support) > 0.003:
                return False
        if abs(self._rails.get(control, float("inf")) - 0.05) > 0.003:
            return False
        # 七轴臂有冗余自由度：关节角不同也可能是同一个扣手姿态。
        # 用实测末端位置与方向确认接合，避免 Web 新任务误走整套准备。
        client = self._fk_cli
        try:
            request = GetPositionFK.Request()
            request.header.frame_id = WORLD_FRAME
            request.fk_link_names = [cfg["tip"] for cfg in ARMS.values()]
            request.robot_state.joint_state.name = list(targets)
            request.robot_state.joint_state.position = [float(v) for v in targets.values()]
            response = self._call(client, request, 10.0, "末端姿态校验")
            if response.error_code.val != 1 or len(response.pose_stamped) != len(ARMS):
                return False
            for tip, expected in zip(request.fk_link_names, response.pose_stamped):
                position, quaternion = self._tip_pose(tip)
                p, q = expected.pose.position, expected.pose.orientation
                if math.dist(position, (p.x, p.y, p.z)) > 0.006:
                    return False
                dot = abs(sum(a*b for a, b in zip(quaternion, (q.x, q.y, q.z, q.w))))
                if 2.0 * math.acos(min(1.0, dot)) > 0.03:
                    return False
            return True
        except RuntimeError as error:
            raise RuntimeError("扣手姿态校验失败，保持当前位置: %s" % error) from error

    # ------------------------------------------------------------------ infra
    def _on_joint_state(self, msg):
        self._joint_state = msg

    def _rail_cb(self, control_id):
        """从 <ns>/<control>/joint_states 取该抽屉的轨道位置（第一个关节）。"""
        def callback(msg):
            if msg.position:
                self._rails[control_id] = float(msg.position[0])
        return callback

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
        # 播放服务的**发现在实测里占 ~9s**（它出现在"双臂后拉"那一段的等待里）。
        # 提前在这里建客户端、发起发现，让它与前面十几秒的动作重叠，而不是等到
        # 最后一刻才等。控制 id 未知时按 db1 兜底——真实 id 在 pull 步骤里再建。
        self._warm_playback_service("db1")

    def _warm_playback_service(self, control: str) -> None:
        try:
            client = self.create_client(
                SetCabinetPlayback, "/xczs/cabinet/%s/playback" % self._cabinet)
            self._playback_warm = client      # 保住强引用，别被 GC
            client.wait_for_service(timeout_sec=0.1)
        except Exception:  # noqa: BLE001
            pass

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
        value = float(self._joint_state.position[names.index(joint)])
        if not math.isfinite(value):
            raise RuntimeError("关节位置反馈无效: " + joint)
        return value

    def _settled(self, joints: List[str], timeout: float = 12.0,
                 tolerance: float = 0.0002, window: float = 0.15) -> bool:
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
        point.velocities = [0.0] * len(joints)
        point.accelerations = [0.0] * len(joints)
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
            if self._cancel.is_set():
                handle.cancel_goal_async()
                self._check_cancel()
            time.sleep(0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            raise RuntimeError("控制器未在时限内完成")
        code = int(result_future.result().result.error_code)
        if code != 0:
            raise RuntimeError("控制器执行失败 error_code=%s" % code)
        return code

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
                        max_step: float = 0.02, timeout: float = 60.0,
                        avoid_collisions: bool = True):
        cfg = ARMS[side]
        # TF 的末端变换可能落后于刚完成的关节轨迹。起点和目标必须由同一份
        # 关节快照计算，否则“最新起点 + 旧末端目标”会产生额外横移和高度偏差。
        start_joints = self._joint_state
        fk = GetPositionFK.Request()
        fk.header.frame_id = WORLD_FRAME
        fk.fk_link_names = [cfg["tip"]]
        fk.robot_state.joint_state = start_joints
        pose_response = self._call(self._fk_cli, fk, timeout, "直线起点正解")
        if pose_response.error_code.val != 1 or not pose_response.pose_stamped:
            raise RuntimeError("直线起点正解失败: " + side)
        start_pose = pose_response.pose_stamped[0].pose
        sx, sy, sz = start_pose.position.x, start_pose.position.y, start_pose.position.z
        q = start_pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in (sx, sy, sz, *quat)):
            raise RuntimeError("直线起点正解包含无效读数: " + side)
        target = Pose()
        target.position.x = sx + (distance if axis == "x" else 0.0)
        target.position.y = sy + (distance if axis == "y" else 0.0)
        target.position.z = sz + (distance if axis == "z" else 0.0)
        target.orientation.x, target.orientation.y = quat[0], quat[1]
        target.orientation.z, target.orientation.w = quat[2], quat[3]
        request = GetCartesianPath.Request()
        request.header.frame_id = WORLD_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.joint_state = start_joints
        request.group_name = cfg["group"]
        request.link_name = cfg["tip"]
        request.waypoints = [target]
        request.max_step = max_step
        request.jump_threshold = 2.0
        request.avoid_collisions = avoid_collisions
        response = self._call(self._cartesian_cli, request, timeout,
                              CARTESIAN_SERVICE)
        trajectory = response.solution.joint_trajectory
        indices = {name: i for i, name in enumerate(trajectory.joint_names)}
        previous = {joint: self._measured(joint) for joint in cfg["joints"]}
        for point in trajectory.points:
            for joint in cfg["joints"]:
                current = point.positions[indices[joint]]
                if not math.isfinite(current) or abs(current - previous[joint]) > 0.35:
                    raise RuntimeError("直线轨迹发生关节跳变，停止执行: " + joint)
                previous[joint] = current
        return trajectory, response.fraction, (sx, sy, sz)

    def _execute_plan(self, side: str, solution, scale: float,
                      timeout: float, lead: float = 0.0) -> int:
        """把 MoveIt 算出的臂轨迹裁到本臂 7 关节，时间轴缩放后下发控制器。

        ``lead`` > 0 时前置一段**保持点**（t=0 与 t=lead 同一位置），让本臂晚
        ``lead`` 秒起步——用于与抽屉播放对齐起跑时刻（见 step_pull_drawer）。
        """
        cfg = ARMS[side]
        wanted = cfg["joints"]
        index = {name: i for i, name in enumerate(solution.joint_names)}
        missing = [j for j in wanted if j not in index]
        if missing:
            raise RuntimeError("%s 轨迹缺少关节 %s" % (side, missing))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(wanted)
        if lead > 0.0 and solution.points:
            hold = JointTrajectoryPoint()
            hold.positions = [solution.points[0].positions[index[j]]
                              for j in wanted]
            hold.time_from_start = Duration(seconds=0.0).to_msg()
            trajectory.points.append(hold)
        for point in solution.points:
            new_point = JointTrajectoryPoint()
            new_point.positions = [point.positions[index[j]] for j in wanted]
            total = (point.time_from_start.sec
                     + point.time_from_start.nanosec * 1e-9) * max(1.0, scale)
            new_point.time_from_start = Duration(seconds=total + lead).to_msg()
            trajectory.points.append(new_point)
        return self._send_trajectory(side, trajectory, timeout)

    def _send_trajectory(self, side: str, trajectory, timeout: float, *, rods=False) -> int:
        client = (self._rod_clients if rods else self._arm_clients)[side]
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("%s 控制器不可用" % (ROD_SIDES if rods else ARMS)[side]["action"])
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
            if self._cancel.is_set():
                handle.cancel_goal_async()
                self._check_cancel()
            time.sleep(0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            raise RuntimeError("%s 未在 %.0fs 内完成轨迹" % (side, timeout))
        code = int(result_future.result().result.error_code)
        if code != 0:
            raise RuntimeError("控制器执行失败 error_code=%s" % code)
        return code

    def _move_arm_to(self, side, targets):
        """准备/归位走规划路径，末端基座始终留在柜前净空内。"""
        cfg = ARMS[side]
        tool_base = "l_two_cyl_base" if side == "left" else "r_three_cyl_base"
        fk_client = self._fk_cli
        plan_client = self._motion_plan_cli
        fk = GetPositionFK.Request()
        fk.header.frame_id = WORLD_FRAME
        fk.fk_link_names = [tool_base]
        goal_state = {j: self._measured(j) for j in self._joint_state.name}
        goal_state.update({j: float(targets[j]) for j in cfg["joints"]})
        fk.robot_state.joint_state.name = list(goal_state)
        fk.robot_state.joint_state.position = list(goal_state.values())
        goal_fk = self._call(fk_client, fk, 10.0, "准备姿态正解")
        if goal_fk.error_code.val != 1 or not goal_fk.pose_stamped:
            raise RuntimeError("准备姿态正解失败: " + side)
        current, _ = self._tip_pose(tool_base)
        destination = goal_fk.pose_stamped[0].pose.position
        # 仅在准备/归位期间加一块柜前净空障碍，直接检查全部机器人网格。
        # 比位置约束的 IK 区域采样更稳定，也能覆盖杆件没有变形的穿模。
        guard = CollisionObject()
        guard.header.frame_id = WORLD_FRAME
        guard.id = "taught_drawer_transit_clearance"
        guard.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[10.0, 20.0, 6.0])
        center = Pose()
        center.position.x = min(current[0], destination.x) - 0.43 - 5.0
        center.position.y = destination.y
        center.position.z = 3.0
        center.orientation.w = 1.0
        guard.primitives = [box]
        guard.primitive_poses = [center]
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = True
        update.scene.world.collision_objects = [guard]
        if not self._call(self._planning_scene_cli, update, 10.0, "设置柜前净空").success:
            raise RuntimeError("设置柜前净空失败")
        try:
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = cfg["group"]
            motion.planner_id = "RRTConnect"
            motion.allowed_planning_time = 5.0
            motion.num_planning_attempts = 3
            motion.max_velocity_scaling_factor = 0.4
            motion.max_acceleration_scaling_factor = 0.4
            motion.start_state.joint_state = self._joint_state
            goal = Constraints()
            for joint in cfg["joints"]:
                goal.joint_constraints.append(JointConstraint(
                    joint_name=joint, position=float(targets[joint]),
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
            motion.goal_constraints = [goal]
            # 随机采样路径经插值细查后偶尔会触碰净空边界（INVALID_MOTION_PLAN）。
            # 保留障碍且不执行该路径，最多重新规划两次；其他错误立即停止。
            for attempt in range(3):
                self._check_cancel()
                response = self._call(plan_client, request, 30.0, "柜外准备路径规划")
                result = response.motion_plan_response
                if result.error_code.val != -2 or attempt == 2:
                    break
            if result.error_code.val != 1 or not result.trajectory.joint_trajectory.points:
                raise RuntimeError("无法规划柜外准备路径: %s (%s)" % (side, result.error_code.val))
            self._execute_plan(side, result.trajectory.joint_trajectory, 1.0, 120.0)
        finally:
            guard.operation = CollisionObject.REMOVE
            update.scene.world.collision_objects = [guard]
            if not self._call(self._planning_scene_cli, update, 10.0, "清除柜前净空").success:
                raise RuntimeError("清除柜前净空失败")

    def _retime_drawer_path(self, side, trajectory, duration):
        """按真实笛卡尔位移配速，而不是把 MoveIt 返回的点当成等距点。"""
        positions = []
        for point in trajectory.points:
            request = GetPositionFK.Request()
            request.header.frame_id = WORLD_FRAME
            request.fk_link_names = [ARMS[side]["tip"]]
            request.robot_state.joint_state.name = list(trajectory.joint_names)
            request.robot_state.joint_state.position = list(point.positions)
            response = self._call(self._fk_cli, request, 10.0, "抽拉轨迹位移校验")
            if response.error_code.val != 1 or not response.pose_stamped:
                raise RuntimeError("抽拉轨迹正解失败: " + side)
            positions.append(response.pose_stamped[0].pose.position.x)
        if len(positions) < 2 or not all(math.isfinite(x) for x in positions):
            raise RuntimeError("抽拉轨迹采样不足或无效")
        travel = positions[-1] - positions[0]
        if abs(travel) < 1e-6:
            raise RuntimeError("抽拉轨迹没有有效位移")
        retimed = []
        previous = -1.0
        for index, (point, position) in enumerate(zip(trajectory.points, positions)):
            fraction = (position - positions[0]) / travel
            if fraction < previous - 1e-6 or not -1e-6 <= fraction <= 1.0 + 1e-6:
                raise RuntimeError("抽拉轨迹中途反向，停止执行")
            fraction = min(1.0, max(0.0, fraction))
            point.time_from_start = Duration(seconds=duration * fraction).to_msg()
            if fraction <= previous + 1e-6:
                if index == len(positions) - 1:
                    retimed[-1] = point
                continue
            retimed.append(point)
            previous = fraction
        trajectory.points = retimed

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
            targets[joint] = (0.0 if step.get("retract_only", False)
                              else _clamp_rod(joint, targets[joint]))

        on_progress(0.1, "退杆到 0")
        for side, cfg in ROD_SIDES.items():
            values = {j: 0.0 for j in cfg["joints"]}
            if all(abs(values[j] - self._measured(j)) < 1e-6
                   for j in cfg["joints"]):
                continue
            self._send(self._rod_clients[side], cfg["joints"], values, 1.2)
        self._settled(sorted(rod_joints))
        stuck = [j for j in sorted(rod_joints)
                 if abs(self._measured(j) - 0.0) > RETREAT_TOLERANCE]
        if stuck:
            worst = max(abs(self._measured(j)) for j in stuck)
            if worst > ROD_RETREAT_HARD_LIMIT:
                raise RuntimeError(
                    "电缸没退到位（顶住 %.2f mm），已中止且未动机械臂: %s"
                    % (worst * 1000, ", ".join(stuck)))
            print("注意：%s 未完全收到 0（最大 %.2f mm，在 %.0f mm 硬限内）——继续。"
                  % (", ".join(stuck), worst * 1000,
                     ROD_RETREAT_HARD_LIMIT * 1000), flush=True)

        self._check_cancel()
        on_progress(0.4, "双臂到位")
        for side, cfg in ARMS.items():
            self._move_arm_to(side, targets)
        self._settled(sorted(arm_joints), timeout=12.0)

        self._check_cancel()
        on_progress(0.8, "电缸伸到目标值")
        for side, cfg in ROD_SIDES.items():
            if all(abs(targets[j] - self._measured(j)) < 0.0005 for j in cfg["joints"]):
                continue
            self._send(self._rod_clients[side], cfg["joints"], targets, 3.0)
        self._settled(sorted(rod_joints))
        on_progress(1.0, "已还原教学位姿 %s" % pose_path.name)

    # ------------------------------------------------- 步骤 3a 电缸杆伸缩
    def _rod_contract(self):
        """从场景适配 YAML 的 drawer_tools 段读"角色→关节名"（跨层合同）。

        与现场工具 nudge_rod_stroke.py 同一份来源、同一套约定，不在这里另立一份。
        """
        if getattr(self, "_rod_contract_cache", None) is None:
            with ADAPTER.open("r", encoding="utf-8") as handle:
                adapter = yaml.safe_load(handle)

            def find(node):
                # 同上：适配器 YAML 有锚点，可能自引用，必须环安全。
                seen = set()
                stack = [node]
                while stack:
                    current = stack.pop()
                    if isinstance(current, dict):
                        if id(current) in seen:
                            continue
                        seen.add(id(current))
                        if "drawer_tools" in current:
                            return current["drawer_tools"]
                        stack.extend(current.values())
                    elif isinstance(current, list):
                        if id(current) in seen:
                            continue
                        seen.add(id(current))
                        stack.extend(current)
                return None

            tools = find(adapter)
            if not tools:
                raise RuntimeError("%s 里找不到 drawer_tools" % ADAPTER)
            contract = {}
            for side in ROD_SIDES:
                contract[side] = {}
                for role in ("gripper", "support", "unlock"):
                    joint = tools[side].get("%s_joint" % role)
                    if not joint:
                        continue      # unlock 只有右臂有，左侧缺是正常的
                    contract[side][role] = joint
            self._rod_contract_cache = contract
        return self._rod_contract_cache

    def step_rod_stroke(self, step: Mapping[str, Any],
                        on_progress: Callable) -> None:
        """按角色伸缩电缸杆。``distance`` 为正=伸出、为负=缩回（米）。

        现场教学里的"钩爪前伸/收缩卡住"就是这一步。同侧其余杆保持实测值不动。
        """
        role = str(step["role"])
        if role not in ("gripper", "support", "unlock"):
            raise RuntimeError(
                "rod_stroke 的 role 只能是 gripper/support/unlock，收到 %r" % role)
        distance = float(step["distance"])
        contract = self._rod_contract()
        sides = ROD_SIDES.keys() if step.get("side", "both") == "both" \
            else [str(step["side"])]
        duration = float(step.get("duration") or 3.0)
        on_progress(0.1, "电缸 %s %+.4f m" % (role, distance))
        for side in sides:
            # 某些角色只有单侧有（unlock 只有右臂），缺的一侧跳过而不是报错。
            joint = contract.get(side, {}).get(role)
            if not joint:
                continue
            cfg = ROD_SIDES[side]
            current = self._measured(joint)
            low, high = ROD_LIMITS.get(joint, ROD_LIMITS["default"])
            target = max(low, min(high, float(step.get("target", current + distance))))
            values = {j: self._measured(j) for j in cfg["joints"]}
            values[joint] = target
            code = self._send(self._rod_clients[side], cfg["joints"], values,
                              duration)
            if code != 0:
                raise RuntimeError("%s 电缸控制器 error_code=%s" % (side, code))
        done = [contract[s][role] for s in sides if contract.get(s, {}).get(role)]
        if not done:
            raise RuntimeError("role=%s 在所有侧都没有对应关节" % role)
        self._settled(done)
        on_progress(1.0, "电缸 %s 完成" % role)

    # ------------------------------------------------- 步骤 3c 释放轨道闩锁
    def step_unlock(self, step: Mapping[str, Any],
                    on_progress: Callable) -> None:
        """释放抽屉的轨道闩锁（SetCabinetUnlock）。

        **为什么必须有这一步**：插件里闩住的抽屉会被弹簧**一直压在关闭档位**
        （源码：*While a drawer is latched (locked) it is parked at the closed
        detent*）。不释放闩锁，抽拉只是在跟弹簧较劲——播放一结束就被拉回 0，
        实测"开了又自己合上"。释放后抽屉才停在播放交给它的位置。

        服务要求（缺一即拒）：解锁电缸的真实杆端要落在解锁逻辑区容差内、且该
        电缸实测伸出量在工作区间内——所以调用前必须先把它伸出去（见序列里
        rod_stroke role=unlock 那一步）。这里的杆端坐标与解锁点全部从跨层合同
        读（适配器 drawer_tools 的 unlock_* 与 controls 的 unlock_press_point），
        不在这里另立一份。
        """
        from xczs_inspection_robot_interfaces.srv import SetCabinetUnlock
        control = str(step.get("control") or self._control_id)
        lease_id = self.ensure_lease()

        with ADAPTER.open("r", encoding="utf-8") as handle:
            adapter = yaml.safe_load(handle)

        def find(node, key):
            # 适配器 YAML 用了锚点/别名，safe_load 出来可能是**自引用**结构，
            # 朴素递归会 maximum recursion depth exceeded。带访问集做环安全。
            seen = set()
            stack = [node]
            while stack:
                current = stack.pop()
                if isinstance(current, dict):
                    if id(current) in seen:
                        continue
                    seen.add(id(current))
                    if key in current:
                        return current[key]
                    stack.extend(current.values())
                elif isinstance(current, list):
                    if id(current) in seen:
                        continue
                    seen.add(id(current))
                    stack.extend(current)
            return None

        tools = find(adapter, "drawer_tools") or {}
        right = tools.get("right") or {}
        link = right.get("unlock_joint")
        point = right.get("unlock_contact_point_local")
        if link and link.endswith("_joint"):
            link = link[: -len("_joint")]
        with CONTROLS.open("r", encoding="utf-8") as handle:
            controls_doc = yaml.safe_load(handle)
        press = find(controls_doc, "unlock_press_point")
        if not (link and point and press):
            raise RuntimeError(
                "解锁合同不全：unlock_joint=%r contact=%r press_point=%r"
                % (link, point, press))

        on_progress(0.2, "释放轨道闩锁（解锁电缸杆端需在解锁区内）")
        request = SetCabinetUnlock.Request()
        request.control_id = control
        request.operation_lease_id = lease_id
        request.robot_model = "xczs_inspection_robot"
        request.right_robot_link = str(link)
        request.right_robot_grasp_point.x = float(point[0])
        request.right_robot_grasp_point.y = float(point[1])
        request.right_robot_grasp_point.z = float(point[2])
        request.unlock_press_point.x = float(press[0])
        request.unlock_press_point.y = float(press[1])
        request.unlock_press_point.z = float(press[2])
        request.unlock = True

        client = self.create_client(
            SetCabinetUnlock, "/xczs/cabinet/%s/unlock" % self._cabinet)
        response = self._call(client, request, 15.0, "unlock")
        print("解锁结果: success=%s mode=%s pressed=%s contact=%s msg=%s"
              % (response.success, response.unlock_mode, response.pressed,
                 response.right_tool_contact, response.message), flush=True)
        if not response.success:
            raise RuntimeError("释放轨道闩锁被拒: %s" % response.message)
        on_progress(1.0, "轨道闩锁已释放")

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
        # **分段平移**：大位移（实测 ds2/ds3 需要上抬 0.57m）一步算不出完整
        # 直线路径（完整度只有 0.72），而拆成 <= `chunk` 的小段后每段都能算满
        # （0.99+）。所以先按 chunk 切段、逐段规划执行，任一段不满即如实报错。
        chunk = abs(float(step.get("chunk") or 0.10))
        segments = []
        remaining = distance
        while abs(remaining) > 1e-9:
            this = max(-chunk, min(chunk, remaining))
            segments.append(this)
            remaining -= this
        on_progress(0.05, "末端沿世界 %s 平移 %+.3f m（分 %d 段）"
                    % (axis, distance, len(segments)))
        for index, seg in enumerate(segments):
            self._check_cancel()
            plans = {}
            for side in ARMS:
                side_distance = (-seg if step.get("mirror", False) and side == "left"
                                 else seg)
                solution, fraction, tip = self._plan_translate(side, axis, side_distance)
                if fraction < 0.99 or not solution.points:
                    raise RuntimeError(
                        "%s 平移第 %d/%d 段（%+.3f m）路径完整度仅 %.4f"
                        % (side, index + 1, len(segments), seg, fraction))
                plans[side] = solution
            times = {side: max(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                               for p in solution.points)
                     for side, solution in plans.items()}
            common_duration = max(duration / len(segments), *times.values())
            with ThreadPoolExecutor(max_workers=2) as executor:
                futures = {
                    side: executor.submit(self._execute_plan, side, solution,
                                          common_duration / times[side] if times[side] else 1.0,
                                          90.0)
                    for side, solution in plans.items()
                }
                for side, future in futures.items():
                    if future.result() != 0:
                        raise RuntimeError("%s 平移失败" % side)
            for side in ARMS:
                self._tip_settled(ARMS[side]["tip"])
            on_progress(0.05 + 0.9 * (index + 1) / float(len(segments)),
                        "平移 %d/%d 段完成" % (index + 1, len(segments)))
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
            self._send(self._rod_clients[side], cfg["joints"], values, 1.2)
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
            self._move_arm_to(side, targets)
        self._settled([j for cfg in ARMS.values() for j in cfg["joints"]],
                      timeout=12.0)
        on_progress(1.0, "机械臂已回到初始姿势")

    # ------------------------------------------------------------- 步骤 3 联动抽拉
    def step_pull_drawer(self, step: Mapping[str, Any],
                         on_progress: Callable) -> None:
        control = str(step["control"])
        arm_share = 0.7 if step.get("support_enabled", True) else 1.0
        duration = float(step["duration"])
        distance = float(step.get("distance") or 0.0)
        # 每个抽屉均从插件的位置反馈确定本次位移。
        joint_topic = "/xczs/cabinet/%s/%s/joint_states" % (self._cabinet, control)
        state_topic = "/xczs/cabinet/%s/%s/state" % (self._cabinet, control)
        self._ensure_rail_subscription(control)
        self.create_subscription(CabinetControlState, state_topic,
                                 self._state_cb(control), best_effort_qos())
        # 插件使用 SensorDataQoS；必须读到当前位置，避免重复打开时累计错位。
        wait_for(lambda: control in self._rails or control in self._controls,
                 5.0, joint_topic)
        raw = self._rails.get(control)
        if raw is None and control in self._controls:
            raw = self._controls[control].position
        if raw is None or not math.isfinite(float(raw)):
            raise RuntimeError("抽屉位置反馈缺失或无效: " + control)
        start = max(0.0, min(RAIL_LIMIT, float(raw)))
        # **支持绝对目标位**（给 target 就按"当前位置 → target"算位移）。
        # 只用相对距离会在异常后累积：实测闭合失败一次、抽屉停在 0.07，
        # 下一次 open 又加 0.07 变成 0.14，越开越大。绝对目标天然不会。
        if step.get("target") is not None:
            target = max(0.0, min(RAIL_LIMIT, float(step["target"])))
            distance = target - start
            print("绝对目标位 %.4f m，当前 %.4f → 位移 %+.4f m"
                  % (target, start, distance), flush=True)
        if start + distance > RAIL_LIMIT + 1e-9 or start + distance < -1e-9:
            raise RuntimeError("目标 %.4f m 超出轨道 [0, %.2f]"
                               % (start + distance, RAIL_LIMIT))

        if abs(distance) <= 0.001:
            self.drawer_result = {
                "simulation_outcome_confirmed": True,
                "final_position": start,
                "target_position": start + distance,
                "position_tolerance": 0.003,
            }
            on_progress(1.0, "抽屉已在目标位置，无需重复抽拉")
            return

        on_progress(0.05, "规划双臂后拉路径")
        plans = {}
        for side in ARMS:
            # 宽柜双臂分担 35 mm；小柜双臂完成全部 50 mm。
            solution, fraction, tip = self._plan_translate(
                side, "x", distance * arm_share, max_step=0.002)
            if fraction < 0.99 or not solution.points:
                raise RuntimeError("%s 后拉路径完整度仅 %.4f" % (side, fraction))
            plans[side] = (solution, tip)
        # MoveIt 的时间参数化会重新采样，点序号并不代表笛卡尔路程。
        # 以正解得到的实际路程定时，支撑杆才不会在中途伸得过快或缩得过慢。
        for side, (solution, _tip) in plans.items():
            self._retime_drawer_path(side, solution, duration)
        plan_time = duration
        scale = 1.0

        # 双臂承担 70% 行程，钩杆回缩承担 30%；支撑杆等量伸长，端点留在墙面。
        # 5 cm 行程下支撑杆从 8 cm 到 11.5 cm，保持在原有 12 cm 限位内。
        rod_plans = {}
        contract = self._rod_contract()
        for side, cfg in ROD_SIDES.items():
            trajectory = JointTrajectory()
            trajectory.joint_names = list(cfg["joints"])
            initial = {j: self._measured(j) for j in cfg["joints"]}
            target_rods = dict(initial)
            if step.get("support_enabled", True):
                target_rods[contract[side]["support"]] += distance * arm_share
            else:
                initial[contract[side]["support"]] = 0.0
                target_rods[contract[side]["support"]] = 0.0
            target_rods[contract[side]["gripper"]] -= distance * (1.0 - arm_share)
            for joint, value in target_rods.items():
                if abs(_clamp_rod(joint, value) - value) > 0.001:
                    raise RuntimeError("抽拉所需电缸行程超限: %s=%.4f" % (joint, value))
            lead = float(step.get("lead") or 0.3)
            for elapsed, values in ((0.0, initial), (lead, initial),
                                    (lead + plan_time * scale, target_rods)):
                point = JointTrajectoryPoint()
                point.positions = [values[j] for j in cfg["joints"]]
                point.time_from_start = Duration(seconds=elapsed).to_msg()
                trajectory.points.append(point)
            if step.get("support_enabled", True):
                rod_plans[side] = trajectory

        # 复用预热好的客户端（见 wait_ready），不再新建——新建要重新发现，
        # 实测那一次发现要等好几秒。
        playback = getattr(self, "_playback_warm", None) or self.create_client(
            SetCabinetPlayback, "/xczs/cabinet/%s/playback" % self._cabinet)
        self._playback_cli = playback
        lease_id = self.ensure_lease()
        try:
            on_progress(0.15, "启动双臂后拉")
            self._playback_control = control
            # 双臂并行执行并预留控制器接收时间，抽屉跟随两侧实际位移。
            lead = float(step.get("lead") or 0.3)
            on_progress(0.28, "起跑对齐：臂前置保持 %.2f s，与抽屉同刻起跑" % lead)
            results: Dict[str, Any] = {}

            def run_arm(side_name, arm_solution):
                try:
                    results[side_name] = self._execute_plan(
                        side_name, arm_solution,
                        (plan_time * scale) / max(
                            p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                            for p in arm_solution.points),
                        max(120.0, duration * 4), lead=lead)
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


            def run_rods(side_name, trajectory):
                try:
                    results["rods_" + side_name] = self._send_trajectory(
                        side_name, trajectory, max(120.0, duration * 4), rods=True)
                except Exception as error:
                    results["rods_" + side_name] = error

            for side, trajectory in rod_plans.items():
                thread = threading.Thread(target=run_rods, args=(side, trajectory),
                                          daemon=True)
                thread.start()
                threads.append(thread)

            self.release_previous_hold(playback)
            on_progress(0.3, "启动抽屉轨道播放")
            self._start_playback(playback, lease_id, control, start,
                                 distance, plan_time * scale, lead=lead)
            for thread in threads:
                thread.join(timeout=max(180.0, duration * 6))
            for side in list(plans) + ["rods_" + s for s in rod_plans]:
                outcome = results.get(side)
                if isinstance(outcome, Exception):
                    raise RuntimeError("%s 后拉失败: %s" % (side, outcome))
                if outcome != 0:
                    raise RuntimeError("%s 控制器 error_code=%s" % (side, outcome))
            for side in ARMS:
                self._tip_settled(ARMS[side]["tip"])
            time.sleep(1.0)
            final = self._rails.get(control)
            if final is None and control in self._controls:
                final = self._controls[control].position
            if (final is None or not math.isfinite(float(final))
                    or abs(float(final) - (start + distance)) > 0.003):
                raise RuntimeError("抽屉未到目标位置: 实测 %s，目标 %.4f"
                                   % (final, start + distance))
            self.drawer_result = {
                "simulation_outcome_confirmed": True,
                "final_position": float(final),
                "target_position": start + distance,
                "position_tolerance": 0.003,
            }
            on_progress(1.0, "抽屉轨道 %.4f → %.4f m"
                        % (start, float(final) if final is not None else start))
        finally:
            self._stop_playback(playback, lease_id)
            # 插件 RELEASE 后会保持轨道当前位置，可以释放任务租约。
            self.stop_renewing()
            self._release_lease(lease_id)
            self._current_lease_id = None

    def _wait_for_arms_moving(self, plans, timeout: float = 8.0,
                              epsilon: float = 0.0015) -> bool:
        """等双臂末端真的开始移动（或超时）。

        用于把抽屉播放的启动时刻对齐到臂的起步——不等的话抽屉会先跑 0.5~1 s，
        全程与手错位。
        """
        baseline = {}
        for side in plans:
            try:
                baseline[side], _ = self._tip_pose(ARMS[side]["tip"])
            except RuntimeError:
                baseline[side] = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            moved = False
            for side, origin in baseline.items():
                if origin is None:
                    continue
                try:
                    now, _ = self._tip_pose(ARMS[side]["tip"], timeout=0.5)
                except RuntimeError:
                    continue
                if max(abs(now[i] - origin[i]) for i in range(3)) >= epsilon:
                    moved = True
            if moved:
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------- 冻结抽屉的持有租约
    def _hold_record(self) -> Path:
        return Path("/tmp/xczs_taught_hold_%s.json" % self._cabinet)

    def release_previous_hold(self, client) -> None:
        """开工前释放上一次任务留下的"冻结抽屉"。

        HOLD 是由**那一次任务**的租约持有的，而插件规则是"被别的租约持有的
        播放会被拒"。所以新任务（例如闭合）在启动播放前，必须先拿回并释放
        上一份持有——否则闭合永远被拒。持有租约随冻结一起落在 /tmp 的小文件里。
        """
        record = self._hold_record()
        if not record.is_file():
            return
        try:
            data = json.loads(record.read_text())
            request = SetCabinetPlayback.Request()
            request.command = SetCabinetPlayback.Request.COMMAND_RELEASE
            request.control_id = str(data.get("control") or "")
            request.operation_lease_id = str(data.get("lease_id") or "")
            self._call(client, request, 10.0, "playback")
            print("已释放上一次的抽屉冻结（租约 %s）"
                  % request.operation_lease_id, flush=True)
        except Exception as error:  # noqa: BLE001
            print("释放上一次冻结失败（继续尝试启动）: %s" % error, flush=True)
        finally:
            try:
                record.unlink()
            except OSError:
                pass

    def remember_hold(self, lease_id: str, control: str) -> None:
        try:
            self._hold_record().write_text(
                json.dumps({"lease_id": lease_id, "control": control}))
        except OSError as error:  # noqa: BLE001
            print("记录冻结租约失败: %s" % error, flush=True)

    def _tip_settled(self, tip: str, timeout: float = 12.0,
                     tolerance: float = 0.0002, window: float = 0.15) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            first, _ = self._tip_pose(tip)
            time.sleep(window)
            now, _ = self._tip_pose(tip)
            if all(abs(now[i] - first[i]) < tolerance for i in range(3)):
                return True
        return False

    # ------------------------------------------------------------- 租约 / 播放
    def ensure_lease(self) -> str:
        """按需申请操作租约（解锁与抽拉都用它，谁先用到谁申请）。

        租约必须在**整条序列**内有效：unlock 服务与 playback 服务都要求
        operation_lease_id 有效，而解锁步骤排在抽拉之前，所以不能等到
        pull_drawer 才申请。
        """
        if self._current_lease_id:
            return self._current_lease_id
        lease_id = self._acquire_lease()
        self._current_lease_id = lease_id
        self._start_renewing(lease_id)
        return lease_id

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
                        duration, lead: float = 0.0) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = [control]
        # header.stamp 契约（插件侧）：0 = 立即开始；**未来值 = 停住等到那一刻
        # 才开始**（插件源码 AGENT T3 段：a future stamp parks the drawer until
        # its start time is reached）。判定用的是**节点墙钟**，不是 world SimTime
        # ——所以这里必须用 time.time() 的墙钟基准，填仿真时间会被算成"过去
        # 17 亿秒"而拒收（本会话踩过）。
        #
        # 起跑对齐就靠这个：臂的目标要走 action 受理才起步，抽屉的播放要等插件
        # 处理——两条独立时间线。给抽屉钉一个"预计双臂真正起步的时刻"，两边
        # 同刻开跑，钩爪与把手在全过程中保持固定相对关系（001 位姿里那 2.4 mm
        # 的缝因此不会被磨掉）。
        if lead > 0.0:
            target = time.time() + lead
            trajectory.header.stamp.sec = int(target)
            trajectory.header.stamp.nanosec = int((target % 1.0) * 1e9)
        else:
            trajectory.header.stamp.sec = 0
            trajectory.header.stamp.nanosec = 0
        for elapsed, value in ((0.0, start), (duration, start + distance)):
            # **终点也必须钳进限位**：关到位时 start 常是 0.24999977 这类值，
            # start + distance(−0.25) 会算出 −2.3e-07 这种极小的负数，而关节
            # 下限是 0 → 插件按 `q < lower` 直接拒收。2026-09-11 闭合任务反复
            # 失败就是这个：只钳了起点没钳终点，报错文案又是四条规则共用的，
            # 光看"被拒"根本看不出是负了 2e-07。
            point = JointTrajectoryPoint()
            point.positions = [max(0.0, min(RAIL_LIMIT, value))]
            point.time_from_start = Duration(seconds=elapsed).to_msg()
            trajectory.points.append(point)
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_START
        request.control_id = control
        request.operation_lease_id = lease_id
        request.trajectory = trajectory
        response = self._call(client, request, 15.0, "playback")
        if not response.success:
            # 插件的报错文案是四条规则共用的，光看"被拒"定位不到是哪一条。
            # 把下发的原始样本与插件返回的位置一并抛出。
            print("播放被拒诊断：command=%s control=%s 样本=%s 时长=%.3f "
                  "返回位置=%s" % (request.command, control,
                                 [(round(p.time_from_start.sec
                                         + p.time_from_start.nanosec * 1e-9, 3),
                                   p.positions[0]) for p in trajectory.points],
                                 duration, response.position), flush=True)
            raise RuntimeError("抽屉播放被拒: %s" % response.message)

    def _stop_playback(self, client, lease_id, hold: bool = False) -> None:
        """释放播放会话；插件以当前位置进入轨道保持模式。"""
        request = SetCabinetPlayback.Request()
        request.command = SetCabinetPlayback.Request.COMMAND_RELEASE
        request.control_id = getattr(self, "_playback_control", "")
        request.operation_lease_id = lease_id
        try:
            self._call(client, request, 10.0, "playback")
        except Exception:  # noqa: BLE001
            pass
