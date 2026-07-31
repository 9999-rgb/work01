"""ROS 2 publishers, services and actions used by the browser gateway."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PositionConstraint
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .velocity_profile import VelocityProfile


class ControlRequestError(RuntimeError):
    """Validated request error that can be returned through the HTTP API."""

    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


class RosControlNode(Node):
    """Own manual publishers and autonomous ROS 2 clients."""

    JOINT_NAMES = [
        "body_arm1",
        "arm1_arm2",
        "arm2_arm3",
        "arm3_arm4",
        "arm4_arm5",
        "arm5_end",
        "end_worklink1",
        "end_worklink2",
    ]
    NAMED_TARGETS = {
        ("manipulator", "home"): {
            "body_arm1": 0.0,
            "arm1_arm2": 0.0,
            "arm2_arm3": 0.0,
            "arm3_arm4": 0.0,
            "arm4_arm5": 0.0,
            "arm5_end": 0.0,
        },
        ("gripper", "open"): {
            "end_worklink1": 0.35,
            "end_worklink2": -0.35,
        },
        ("gripper", "closed"): {
            "end_worklink1": 0.0,
            "end_worklink2": 0.0,
        },
    }
    ACTIVE_NAVIGATION_STATES = {
        "enabling",
        "sending",
        "navigating",
        "canceling",
    }
    ACTIVE_MOTION_STATES = {
        "sending",
        "planning",
        "executing",
        "canceling",
    }

    def __init__(
        self,
        cmd_vel_topic: str,
        joint_trajectory_topic: str,
        max_linear_speed: float,
        max_angular_speed: float,
        command_timeout: float,
        context: Context,
    ) -> None:
        super().__init__(
            "xczs_web_control_server",
            context=context,
        )

        # Enable sim time so the control node synchronises with the
        # Gazebo /clock topic in simulation.
        sim_time_param = rclpy.parameter.Parameter(
            "use_sim_time",
            rclpy.parameter.Parameter.Type.BOOL,
            True,
        )
        self.set_parameters([sim_time_param])

        self._lock = threading.RLock()
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            joint_trajectory_topic,
            10,
        )
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._command_timeout = command_timeout
        self._linear_profile = VelocityProfile(0.50, 2.00)
        self._angular_profile = VelocityProfile(1.20, 4.80)
        self._target_linear_y = 0.0
        self._target_angular_z = 0.0
        self._last_command_time = time.monotonic()
        self._last_update_time = time.monotonic()
        self._pending_trajectory: Optional[JointTrajectory] = None
        self._pending_trajectory_repeats = 0

        self._navigation_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self._navigation_mode_client = self.create_client(
            SetBool,
            "/xczs/set_navigation_mode",
        )
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )
        self._controller_cancel_clients = [
            self.create_client(
                CancelGoal,
                (
                    "/xczs/arm_controller/follow_joint_trajectory/"
                    "_action/cancel_goal"
                ),
            ),
            self.create_client(
                CancelGoal,
                (
                    "/xczs/gripper_controller/follow_joint_trajectory/"
                    "_action/cancel_goal"
                ),
            ),
        ]

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            "/xczs/navigation_mode",
            self._navigation_mode_callback,
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            transient_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._pose_callback,
            10,
        )
        self.create_subscription(
            Path,
            "/plan",
            self._plan_callback,
            10,
        )

        self._navigation_mode: Optional[bool] = None
        self._navigation_goal_handle: Any = None
        self._navigation_state: Dict[str, Any] = {
            "state": "idle",
            "message": "No navigation goal has been sent.",
            "goal": None,
            "distance_remaining": None,
            "eta_seconds": None,
            "navigation_time_seconds": None,
            "recoveries": 0,
            "updated_at": time.time(),
        }
        self._map_state: Optional[Dict[str, Any]] = None
        self._robot_pose: Optional[Dict[str, float]] = None
        self._global_plan: List[Dict[str, float]] = []

        self._motion_goal_handle: Any = None
        self._motion_cancel_requested = False
        self._last_controller_cancel_time = 0.0
        self._motion_state: Dict[str, Any] = {
            "state": "idle",
            "message": "No MoveIt goal has been sent.",
            "target": None,
            "execute": False,
            "feedback": "",
            "planning_time": None,
            "error_code": None,
            "updated_at": time.time(),
        }
        self.create_timer(0.02, self._update_manual_control)

    def set_base_target(
        self,
        linear_y: float,
        angular_z: float,
    ) -> Tuple[float, float]:
        """Set the smoothed manual base target."""
        linear_y = max(
            -self._max_linear_speed,
            min(self._max_linear_speed, linear_y),
        )
        angular_z = max(
            -self._max_angular_speed,
            min(self._max_angular_speed, angular_z),
        )
        with self._lock:
            self._target_linear_y = linear_y
            self._target_angular_z = angular_z
            self._last_command_time = time.monotonic()
        return linear_y, angular_z

    def set_joint_target(self, positions: List[float]) -> List[float]:
        """Queue one legacy manual joint target."""
        safe_positions = [
            max(-2.80, min(2.80, value))
            for value in positions[:6]
        ]
        safe_positions.append(max(0.0, min(0.35, positions[6])))
        safe_positions.append(max(-0.35, min(0.0, positions[7])))

        trajectory = JointTrajectory()
        trajectory.header.frame_id = "world"
        trajectory.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = safe_positions
        # Give the controller a 0.5 s window to reach the target so it
        # can interpolate a smooth trajectory.  Without this the
        # ros2_control JointTrajectoryController has no timing
        # reference and may reject the command outright.
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500_000_000
        trajectory.points.append(point)

        with self._lock:
            self._pending_trajectory = trajectory
            self._pending_trajectory_repeats = 6
        return safe_positions

    def emergency_stop(self) -> None:
        """Stop only the manual base publisher."""
        with self._lock:
            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
        self._cmd_vel_publisher.publish(Twist())

    def navigation_snapshot(self) -> Dict[str, Any]:
        """Return current Nav2 availability, feedback and display overlays."""
        with self._lock:
            snapshot = dict(self._navigation_state)
            snapshot.update(
                {
                    "available": self._action_server_ready(
                        self._navigation_client
                    ),
                    "mode": self._navigation_mode,
                    "current_pose": (
                        dict(self._robot_pose)
                        if self._robot_pose is not None
                        else None
                    ),
                    "plan": [dict(point) for point in self._global_plan],
                }
            )
        return snapshot

    def map_snapshot(self) -> Dict[str, Any]:
        """Return the latest transient-local occupancy map."""
        with self._lock:
            if self._map_state is None:
                raise ControlRequestError(
                    "Nav2 occupancy map is not available.",
                    503,
                )
            return {
                **self._map_state,
                "data": list(self._map_state["data"]),
            }

    def motion_snapshot(self) -> Dict[str, Any]:
        """Return current MoveIt availability and action state."""
        with self._lock:
            snapshot = dict(self._motion_state)
            snapshot["available"] = self._action_server_ready(
                self._move_group_client
            )
        return snapshot

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        """Switch the base command router through its ROS 2 service."""
        if not self._navigation_mode_client.service_is_ready():
            raise ControlRequestError(
                "Base navigation mode service is unavailable.",
                503,
            )
        if not enabled:
            self.cancel_navigation(allow_idle=True)
            self.emergency_stop()
        self._request_navigation_mode(enabled)
        return {
            "status": "accepted",
            "enabled": enabled,
        }

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Dict[str, Any]:
        """Enable navigation mode and submit one NavigateToPose goal."""
        if not self._action_server_ready(self._navigation_client):
            raise ControlRequestError(
                "Nav2 NavigateToPose action server is unavailable.",
                503,
            )
        if not self._navigation_mode_client.service_is_ready():
            raise ControlRequestError(
                "Base navigation mode service is unavailable.",
                503,
            )
        self._validate_navigation_goal(x, y)
        goal_description = {
            "x": x,
            "y": y,
            "yaw": yaw,
        }
        with self._lock:
            if (
                self._navigation_state["state"]
                in self.ACTIVE_NAVIGATION_STATES
            ):
                raise ControlRequestError(
                    "A navigation goal is already active.",
                    409,
                )
            self._navigation_state.update(
                {
                    "state": "enabling",
                    "message": "Enabling Nav2 base command mode.",
                    "goal": goal_description,
                    "distance_remaining": None,
                    "eta_seconds": None,
                    "navigation_time_seconds": None,
                    "recoveries": 0,
                    "updated_at": time.time(),
                }
            )

        request = SetBool.Request()
        request.data = True
        future = self._navigation_mode_client.call_async(request)
        future.add_done_callback(
            lambda result_future: self._navigation_mode_for_goal_callback(
                result_future,
                goal_description,
            )
        )
        return {
            "status": "accepted",
            "goal": goal_description,
        }

    def cancel_navigation(self, allow_idle: bool = False) -> Dict[str, Any]:
        """Cancel the currently accepted Nav2 goal."""
        with self._lock:
            goal_handle = self._navigation_goal_handle
            active = (
                self._navigation_state["state"]
                in self.ACTIVE_NAVIGATION_STATES
            )
            if goal_handle is None:
                if allow_idle or not active:
                    return {"status": "idle"}
                self._navigation_state.update(
                    {
                        "state": "canceled",
                        "message": "Navigation canceled before goal acceptance.",
                        "updated_at": time.time(),
                    }
                )
                return {"status": "canceling"}
            self._navigation_state.update(
                {
                    "state": "canceling",
                    "message": "Canceling the active navigation goal.",
                    "updated_at": time.time(),
                }
            )
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self._navigation_cancel_callback)
        return {"status": "canceling"}

    def send_named_motion(
        self,
        group: str,
        target: str,
        execute: bool,
    ) -> Dict[str, Any]:
        """Submit a collision-checked MoveIt named joint target."""
        key = (group, target)
        if key not in self.NAMED_TARGETS:
            raise ControlRequestError(
                f"Unsupported MoveIt named target: {group}/{target}."
            )
        constraints = Constraints()
        constraints.name = target
        for joint_name, position in self.NAMED_TARGETS[key].items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        description = {
            "type": "named",
            "group": group,
            "name": target,
        }
        return self._send_move_group_goal(
            group,
            constraints,
            description,
            execute,
        )

    def send_pose_motion(
        self,
        frame_id: str,
        position: List[float],
        orientation: List[float],
        execute: bool,
    ) -> Dict[str, Any]:
        """Submit a MoveIt end-effector pose target."""
        if frame_id not in {"body", "odom", "map"}:
            raise ControlRequestError(
                "frame_id must be body, odom or map."
            )
        if len(position) != 3 or len(orientation) != 4:
            raise ControlRequestError(
                "MoveIt pose requires 3 position and 4 orientation values."
            )
        if not all(math.isfinite(value) for value in position + orientation):
            raise ControlRequestError("MoveIt pose values must be finite.")
        quaternion_norm = math.sqrt(
            sum(value * value for value in orientation)
        )
        if quaternion_norm < 1.0e-6:
            raise ControlRequestError(
                "MoveIt orientation quaternion must be non-zero."
            )
        normalized = [
            value / quaternion_norm
            for value in orientation
        ]

        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.005]
        region_pose = Pose()
        region_pose.position.x = position[0]
        region_pose.position.y = position[1]
        region_pose.position.z = position[2]
        region_pose.orientation.w = 1.0

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = "end"
        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(
            region_pose
        )
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.header.stamp = (
            position_constraint.header.stamp
        )
        orientation_constraint.link_name = "end"
        orientation_constraint.orientation.x = normalized[0]
        orientation_constraint.orientation.y = normalized[1]
        orientation_constraint.orientation.z = normalized[2]
        orientation_constraint.orientation.w = normalized[3]
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "web_end_pose"
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        description = {
            "type": "pose",
            "group": "manipulator",
            "frame_id": frame_id,
            "position": position,
            "orientation": normalized,
        }
        return self._send_move_group_goal(
            "manipulator",
            constraints,
            description,
            execute,
        )

    def cancel_motion(self, allow_idle: bool = False) -> Dict[str, Any]:
        """Cancel the currently accepted MoveIt goal."""
        with self._lock:
            goal_handle = self._motion_goal_handle
            active = self._motion_state["state"] in self.ACTIVE_MOTION_STATES
            if goal_handle is None:
                if allow_idle or not active:
                    return {"status": "idle"}
                self._motion_state.update(
                    {
                        "state": "canceled",
                        "message": "MoveIt goal canceled before acceptance.",
                        "updated_at": time.time(),
                    }
                )
                return {"status": "canceling"}
            self._motion_state.update(
                {
                    "state": "canceling",
                    "message": "Canceling the active MoveIt goal.",
                    "updated_at": time.time(),
                }
            )
            self._motion_cancel_requested = True
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self._motion_cancel_callback)
        return {"status": "canceling"}

    def _update_manual_control(self) -> None:
        now = time.monotonic()
        period = now - self._last_update_time
        self._last_update_time = now

        with self._lock:
            if now - self._last_command_time >= self._command_timeout:
                self._target_linear_y = 0.0
                self._target_angular_z = 0.0
            linear_y = self._linear_profile.update(
                self._target_linear_y,
                period,
            )
            angular_z = self._angular_profile.update(
                self._target_angular_z,
                period,
            )
            trajectory = self._pending_trajectory
            if self._pending_trajectory_repeats > 0:
                self._pending_trajectory_repeats -= 1
            else:
                trajectory = None
                self._pending_trajectory = None

        command = Twist()
        command.linear.y = linear_y
        command.angular.z = angular_z
        self._cmd_vel_publisher.publish(command)
        if trajectory is not None:
            self._trajectory_publisher.publish(trajectory)
        self._enforce_motion_cancel(now)

    def _request_navigation_mode(self, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        future = self._navigation_mode_client.call_async(request)
        future.add_done_callback(
            lambda result_future: self._navigation_mode_callback_result(
                result_future,
                enabled,
            )
        )

    def _navigation_mode_callback_result(
        self,
        future: Any,
        enabled: bool,
    ) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            with self._lock:
                self._navigation_mode = enabled
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to switch base command mode: {error}"
            )

    def _navigation_mode_for_goal_callback(
        self,
        future: Any,
        goal_description: Dict[str, float],
    ) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
        except Exception as error:  # noqa: BLE001
            with self._lock:
                self._navigation_state.update(
                    {
                        "state": "failed",
                        "message": (
                            "Failed to enable navigation mode: "
                            f"{error}"
                        ),
                        "updated_at": time.time(),
                    }
                )
            return

        with self._lock:
            self._navigation_mode = True
            if self._navigation_state["state"] == "canceled":
                return
            self._navigation_state.update(
                {
                    "state": "sending",
                    "message": "Sending the Nav2 goal.",
                    "updated_at": time.time(),
                }
            )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_description["x"]
        goal.pose.pose.position.y = goal_description["y"]
        goal.pose.pose.orientation.z = math.sin(
            goal_description["yaw"] / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            goal_description["yaw"] / 2.0
        )
        future = self._navigation_client.send_goal_async(
            goal,
            feedback_callback=self._navigation_feedback_callback,
        )
        future.add_done_callback(self._navigation_goal_response_callback)

    def _navigation_goal_response_callback(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self._set_navigation_terminal(
                "failed",
                f"Failed to send Nav2 goal: {error}",
            )
            return
        if not goal_handle.accepted:
            self._set_navigation_terminal(
                "rejected",
                "Nav2 rejected the navigation goal.",
            )
            return
        with self._lock:
            canceled_before_acceptance = (
                self._navigation_state["state"] == "canceled"
            )
            self._navigation_goal_handle = goal_handle
            self._navigation_state.update(
                {
                    "state": (
                        "canceling"
                        if canceled_before_acceptance
                        else "navigating"
                    ),
                    "message": (
                        "Canceling the accepted navigation goal."
                        if canceled_before_acceptance
                        else "Nav2 is driving to the selected goal."
                    ),
                    "updated_at": time.time(),
                }
            )
        if canceled_before_acceptance:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                self._navigation_cancel_callback
            )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result_callback)

    def _navigation_feedback_callback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        with self._lock:
            self._navigation_state.update(
                {
                    "distance_remaining": float(
                        feedback.distance_remaining
                    ),
                    "eta_seconds": self._duration_seconds(
                        feedback.estimated_time_remaining
                    ),
                    "navigation_time_seconds": self._duration_seconds(
                        feedback.navigation_time
                    ),
                    "recoveries": int(feedback.number_of_recoveries),
                    "updated_at": time.time(),
                }
            )

    def _navigation_result_callback(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
            state, message = self._goal_status(
                wrapped_result.status,
                "Navigation",
            )
        except Exception as error:  # noqa: BLE001
            state = "failed"
            message = f"Navigation result failed: {error}"
        self._set_navigation_terminal(state, message)

    def _navigation_cancel_callback(self, future: Any) -> None:
        try:
            response = future.result()
            if not response.goals_canceling:
                raise RuntimeError("Nav2 did not accept the cancel request.")
        except Exception as error:  # noqa: BLE001
            with self._lock:
                self._navigation_state.update(
                    {
                        "state": "failed",
                        "message": f"Navigation cancel failed: {error}",
                        "updated_at": time.time(),
                    }
                )

    def _set_navigation_terminal(
        self,
        state: str,
        message: str,
    ) -> None:
        with self._lock:
            self._navigation_goal_handle = None
            self._navigation_state.update(
                {
                    "state": state,
                    "message": message,
                    "updated_at": time.time(),
                }
            )

    def _send_move_group_goal(
        self,
        group: str,
        constraints: Constraints,
        description: Dict[str, Any],
        execute: bool,
    ) -> Dict[str, Any]:
        if not self._action_server_ready(self._move_group_client):
            raise ControlRequestError(
                "MoveIt MoveGroup action server is unavailable.",
                503,
            )
        with self._lock:
            if self._motion_state["state"] in self.ACTIVE_MOTION_STATES:
                raise ControlRequestError(
                    "A MoveIt goal is already active.",
                    409,
                )
            self._motion_state.update(
                {
                    "state": "sending",
                    "message": "Sending the MoveIt planning request.",
                    "target": description,
                    "execute": execute,
                    "feedback": "",
                    "planning_time": None,
                    "error_code": None,
                    "updated_at": time.time(),
                }
            )
            self._motion_cancel_requested = False

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = not execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = execute
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.25

        future = self._move_group_client.send_goal_async(
            goal,
            feedback_callback=self._motion_feedback_callback,
        )
        future.add_done_callback(self._motion_goal_response_callback)
        return {
            "status": "accepted",
            "target": description,
            "execute": execute,
        }

    def _motion_goal_response_callback(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self._set_motion_terminal(
                "failed",
                f"Failed to send MoveIt goal: {error}",
            )
            return
        if not goal_handle.accepted:
            self._set_motion_terminal(
                "rejected",
                "MoveIt rejected the planning request.",
            )
            return
        with self._lock:
            canceled_before_acceptance = (
                self._motion_state["state"] == "canceled"
            )
            self._motion_goal_handle = goal_handle
            state = (
                "canceling"
                if canceled_before_acceptance
                else (
                    "executing"
                    if self._motion_state["execute"]
                    else "planning"
                )
            )
            self._motion_state.update(
                {
                    "state": state,
                    "message": (
                        "Canceling the accepted MoveIt goal."
                        if canceled_before_acceptance
                        else (
                            "MoveIt is planning and executing the trajectory."
                            if self._motion_state["execute"]
                            else "MoveIt is planning without execution."
                        )
                    ),
                    "updated_at": time.time(),
                }
            )
            if canceled_before_acceptance:
                self._motion_cancel_requested = True
        if canceled_before_acceptance:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._motion_cancel_callback)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._motion_result_callback)

    def _motion_feedback_callback(self, feedback_message: Any) -> None:
        with self._lock:
            self._motion_state.update(
                {
                    "feedback": str(feedback_message.feedback.state),
                    "updated_at": time.time(),
                }
            )

    def _motion_result_callback(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
            error_code = int(wrapped_result.result.error_code.val)
            planning_time = float(wrapped_result.result.planning_time)
            with self._lock:
                cancel_requested = self._motion_cancel_requested
            if cancel_requested:
                state = "canceled"
                message = "MoveIt motion was canceled."
            elif (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and error_code == 1
            ):
                state = "succeeded"
                message = (
                    "MoveIt trajectory execution succeeded."
                    if self._motion_state["execute"]
                    else "MoveIt plan succeeded."
                )
            else:
                state, message = self._goal_status(
                    wrapped_result.status,
                    "MoveIt",
                )
                if state == "succeeded":
                    state = "failed"
                    message = (
                        "MoveIt planning failed with error code "
                        f"{error_code}."
                    )
        except Exception as error:  # noqa: BLE001
            state = "failed"
            message = f"MoveIt result failed: {error}"
            error_code = None
            planning_time = None
        self._set_motion_terminal(
            state,
            message,
            error_code,
            planning_time,
        )

    def _motion_cancel_callback(self, future: Any) -> None:
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warning(
                    "MoveIt did not accept direct action cancellation; "
                    "canceling active trajectory controllers."
                )
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(
                f"MoveIt action cancellation failed: {error}"
            )

    def _set_motion_terminal(
        self,
        state: str,
        message: str,
        error_code: Optional[int] = None,
        planning_time: Optional[float] = None,
    ) -> None:
        with self._lock:
            self._motion_goal_handle = None
            self._motion_cancel_requested = False
            self._motion_state.update(
                {
                    "state": state,
                    "message": message,
                    "error_code": error_code,
                    "planning_time": planning_time,
                    "updated_at": time.time(),
                }
            )

    def _enforce_motion_cancel(self, now: float) -> None:
        with self._lock:
            should_cancel = (
                self._motion_cancel_requested
                and self._motion_state["state"]
                in self.ACTIVE_MOTION_STATES
                and now - self._last_controller_cancel_time >= 0.2
            )
            if should_cancel:
                self._last_controller_cancel_time = now
        if not should_cancel:
            return
        for client in self._controller_cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())

    def _navigation_mode_callback(self, message: Bool) -> None:
        with self._lock:
            self._navigation_mode = bool(message.data)

    def _map_callback(self, message: OccupancyGrid) -> None:
        with self._lock:
            self._map_state = {
                "frame_id": message.header.frame_id,
                "resolution": float(message.info.resolution),
                "width": int(message.info.width),
                "height": int(message.info.height),
                "origin": {
                    "x": float(message.info.origin.position.x),
                    "y": float(message.info.origin.position.y),
                    "yaw": self._quaternion_yaw(
                        message.info.origin.orientation
                    ),
                },
                "data": list(message.data),
            }

    def _pose_callback(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        with self._lock:
            self._robot_pose = {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": self._quaternion_yaw(
                    message.pose.pose.orientation
                ),
            }

    def _plan_callback(self, message: Path) -> None:
        poses = message.poses
        step = max(1, math.ceil(len(poses) / 500))
        points = [
            {
                "x": float(pose.pose.position.x),
                "y": float(pose.pose.position.y),
            }
            for pose in poses[::step]
        ]
        with self._lock:
            self._global_plan = points

    def _validate_navigation_goal(self, x: float, y: float) -> None:
        with self._lock:
            map_state = self._map_state
        if map_state is None:
            return
        resolution = map_state["resolution"]
        column = math.floor((x - map_state["origin"]["x"]) / resolution)
        row = math.floor((y - map_state["origin"]["y"]) / resolution)
        if (
            column < 0
            or row < 0
            or column >= map_state["width"]
            or row >= map_state["height"]
        ):
            raise ControlRequestError(
                "Navigation goal is outside the loaded map."
            )
        occupancy = map_state["data"][
            row * map_state["width"] + column
        ]
        if occupancy < 0:
            raise ControlRequestError(
                "Navigation goal is in an unknown map cell."
            )
        if occupancy >= 65:
            raise ControlRequestError(
                "Navigation goal is inside an occupied map cell."
            )

    @staticmethod
    def _action_server_ready(client: Any) -> bool:
        try:
            return bool(client.server_is_ready())
        except Exception:  # noqa: BLE001
            return False

    @staticmethod
    def _duration_seconds(duration: Any) -> float:
        return float(duration.sec) + float(duration.nanosec) / 1.0e9

    @staticmethod
    def _quaternion_yaw(quaternion: Any) -> float:
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def _goal_status(status: int, label: str) -> Tuple[str, str]:
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "succeeded", f"{label} succeeded."
        if status == GoalStatus.STATUS_CANCELED:
            return "canceled", f"{label} was canceled."
        if status == GoalStatus.STATUS_ABORTED:
            return "failed", f"{label} was aborted."
        return "failed", f"{label} ended with action status {status}."
