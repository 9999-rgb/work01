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
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from xczs_inspection_robot_control.action import PressCabinetButton
from xczs_inspection_robot_control.msg import CabinetControlCatalog

from .velocity_profile import VelocityProfile


class ControlRequestError(RuntimeError):
    """Validated request error that can be returned through the HTTP API."""

    def __init__(self, message: str, status: int = 400) -> None:
        super().__init__(message)
        self.status = status


class RosControlNode(Node):
    """Own manual publishers and autonomous ROS 2 clients."""

    CABINET_BUTTON_ID = "box_10_button_1"
    CABINET_BUTTON_JOINT_NAME = "box_10_box_10_button_1"
    CABINET_CONTROL_TYPE_BUTTON = 0
    DEFAULT_CABINET_CONTROL = {
        "control_id": CABINET_BUTTON_ID,
        "display_name": "10 号模块红色按钮",
        "control_type": CABINET_CONTROL_TYPE_BUTTON,
        "joint_name": CABINET_BUTTON_JOINT_NAME,
        "joint_state_topic": (
            "/xczs/cabinet/box_10_button_1/joint_states"
        ),
        "pressed_topic": "/xczs/cabinet/box_10_button_1/pressed",
    }
    NAVIGATION_MODE_MAX_ATTEMPTS = 3

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
        "taking_over",
    }
    ACTIVE_MOTION_STATES = {
        "sending",
        "planning",
        "executing",
        "canceling",
    }
    ACTIVE_CABINET_STATES = {
        "sending",
        "operating",
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
        self._cabinet_catalog_received = False
        self._cabinet_controls: Dict[str, Dict[str, Any]] = {
            self.CABINET_BUTTON_ID: dict(self.DEFAULT_CABINET_CONTROL)
        }
        self._cabinet_control_states: Dict[str, Dict[str, Any]] = {
            self.CABINET_BUTTON_ID: self._new_cabinet_control_state()
        }
        self._cabinet_control_subscriptions: Dict[
            str, Dict[str, Any]
        ] = {}

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
        self._cabinet_button_client = ActionClient(
            self,
            PressCabinetButton,
            "/xczs/press_cabinet_button",
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
        self.create_subscription(
            CabinetControlCatalog,
            "/xczs/cabinet/control_catalog",
            self._cabinet_control_catalog_callback,
            transient_qos,
        )
        self._ensure_cabinet_control_subscriptions(
            self.DEFAULT_CABINET_CONTROL,
            transient_qos,
        )

        self._navigation_mode: Optional[bool] = None
        self._navigation_mode_acknowledged: Optional[bool] = None
        self._navigation_generation = 0
        self._navigation_goal_handle: Any = None
        self._navigation_goal_token: Any = None
        self._navigation_cancel_requested = False
        self._manual_takeover_generation: Optional[int] = None
        self._navigation_mode_desired: Optional[Tuple[bool, int]] = None
        self._navigation_mode_request: Optional[Tuple[bool, int, int]] = None
        self._navigation_mode_request_sequence = 0
        self._navigation_mode_attempt = 0
        self._navigation_retirements: Dict[
            Tuple[int, Any], Dict[str, Any]
        ] = {}
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

        self._cabinet_goal_handle: Any = None
        self._cabinet_cancel_requested = False
        self._cabinet_generation = 0
        self._cabinet_terminal_event = threading.Event()
        self._cabinet_terminal_event.set()
        self._cabinet_state: Dict[str, Any] = {
            "state": "idle",
            "message": "No cabinet button operation has been sent.",
            "button_id": self.CABINET_BUTTON_ID,
            "navigate_to_staging_pose": True,
            "phase": None,
            "progress": 0.0,
            "button_pressed": None,
            "button_travel": None,
            "max_travel": None,
            "success": None,
            "error_code": None,
            "updated_at": time.time(),
            "button_state_updated_at": None,
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
            self._reject_if_cabinet_active_locked()
            if not self._manual_control_ready_locked():
                raise ControlRequestError(
                    "Manual base control is unavailable until Nav2 "
                    "takeover reaches manual mode.",
                    409,
                )
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
            self._reject_if_cabinet_active_locked()
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
            manual_control_ready = self._manual_control_ready_locked()
            snapshot.update(
                {
                    "available": self._action_server_ready(
                        self._navigation_client
                    ),
                    "mode": self._navigation_mode,
                    "mode_acknowledged": (
                        self._navigation_mode_acknowledged
                    ),
                    "retiring_goals": len(self._navigation_retirements),
                    "retirement_errors": [
                        retirement["error"]
                        for retirement in self._navigation_retirements.values()
                        if retirement.get("error")
                    ],
                    "manual_control_ready": manual_control_ready,
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

    def cabinet_snapshot(self) -> Dict[str, Any]:
        """Return cabinet action availability, feedback and button state."""
        with self._lock:
            snapshot = dict(self._cabinet_state)
            control_state = self._cabinet_control_states.get(
                str(snapshot.get("button_id", ""))
            )
            if control_state is not None:
                snapshot.update(control_state)
            snapshot.update(
                {
                    "available": self._action_server_ready(
                        self._cabinet_button_client
                    ),
                    "active": self._cabinet_active_locked(),
                }
            )
        return snapshot

    def cabinet_controls_snapshot(self) -> Dict[str, Any]:
        """Return the current operator-provided cabinet control catalog."""
        with self._lock:
            controls = []
            for control_id, control in self._cabinet_controls.items():
                snapshot = dict(control)
                state = self._cabinet_control_states.get(control_id)
                if state is not None:
                    snapshot.update(state)
                controls.append(snapshot)
            selected_control_id = str(
                self._cabinet_state.get(
                    "button_id",
                    self.CABINET_BUTTON_ID,
                )
            )
            catalog_received = self._cabinet_catalog_received
            available = self._action_server_ready(
                self._cabinet_button_client
            )
        return {
            "available": available,
            "catalog_received": catalog_received,
            "source": (
                "operator_catalog"
                if catalog_received
                else "compatibility_default"
            ),
            "selected_control_id": selected_control_id,
            "controls": controls,
        }

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        """Submit one complete cabinet button operation."""
        if not isinstance(button_id, str) or not button_id.strip():
            raise ControlRequestError(
                "button_id must be a non-empty string."
            )
        button_id = button_id.strip()
        if not isinstance(navigate_to_staging_pose, bool):
            raise ControlRequestError(
                "navigate_to_staging_pose must be a boolean."
            )
        with self._lock:
            self._validate_cabinet_button_locked(button_id)
            self._reject_cabinet_start_conflicts_locked()
        if not self._action_server_ready(self._cabinet_button_client):
            raise ControlRequestError(
                "Cabinet button action server is unavailable.",
                503,
            )

        with self._lock:
            self._validate_cabinet_button_locked(button_id)
            self._reject_cabinet_start_conflicts_locked()

            self._cabinet_generation += 1
            generation = self._cabinet_generation
            self._cabinet_cancel_requested = False
            self._cabinet_goal_handle = None
            self._cabinet_terminal_event.clear()
            self._cabinet_state.update(
                {
                    "state": "sending",
                    "message": "Sending the cabinet button goal.",
                    "button_id": button_id,
                    "navigate_to_staging_pose": navigate_to_staging_pose,
                    "phase": None,
                    "progress": 0.0,
                    "max_travel": 0.0,
                    "success": None,
                    "error_code": None,
                    "updated_at": time.time(),
                }
            )

            # The cabinet operator shares the manual base topic during
            # precision docking.  Stop once, then suppress this gateway's
            # periodic publishers until the cabinet action is terminal.
            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
            self._pending_trajectory = None
            self._pending_trajectory_repeats = 0
            self._cmd_vel_publisher.publish(Twist())

        goal = PressCabinetButton.Goal()
        goal.button_id = button_id
        goal.navigate_to_staging_pose = navigate_to_staging_pose
        try:
            future = self._cabinet_button_client.send_goal_async(
                goal,
                feedback_callback=(
                    lambda feedback_message: self._cabinet_feedback_callback(
                        feedback_message,
                        generation,
                    )
                ),
            )
            future.add_done_callback(
                lambda goal_future: self._cabinet_goal_response_callback(
                    goal_future,
                    generation,
                )
            )
        except Exception as error:  # noqa: BLE001
            self._set_cabinet_terminal(
                "failed",
                f"Failed to send cabinet button goal: {error}",
                False,
                None,
                generation=generation,
            )
            raise ControlRequestError(
                f"Failed to send cabinet button goal: {error}",
                503,
            ) from error

        return {
            "status": "accepted",
            "button_id": button_id,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }

    def cancel_cabinet_button(
        self,
        allow_idle: bool = False,
    ) -> Dict[str, Any]:
        """Cancel the active cabinet operation, including pre-acceptance."""
        with self._lock:
            if not self._cabinet_active_locked():
                if allow_idle:
                    return {"status": "idle"}
                return {"status": "idle"}
            if self._cabinet_state["state"] == "canceling":
                return {"status": "canceling"}

            self._cabinet_cancel_requested = True
            self._cabinet_state.update(
                {
                    "state": "canceling",
                    "message": "Canceling the cabinet button operation.",
                    "updated_at": time.time(),
                }
            )
            goal_handle = self._cabinet_goal_handle
            generation = self._cabinet_generation
            if goal_handle is None:
                return {"status": "canceling"}

        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda cancel_future: self._cabinet_cancel_callback(
                    cancel_future,
                    generation,
                    goal_handle,
                )
            )
        except Exception as error:  # noqa: BLE001
            with self._lock:
                if (
                    generation == self._cabinet_generation
                    and self._cabinet_goal_handle is goal_handle
                    and self._cabinet_active_locked()
                ):
                    self._cabinet_cancel_requested = False
                    self._cabinet_state.update(
                        {
                            "state": "operating",
                            "message": (
                                "Cabinet operation cancellation failed: "
                                f"{error}"
                            ),
                            "updated_at": time.time(),
                        }
                    )
            raise ControlRequestError(
                f"Cabinet operation cancellation failed: {error}",
                503,
            ) from error
        return {"status": "canceling"}

    def wait_for_cabinet_idle(self, timeout_sec: float) -> bool:
        """Wait for an in-flight cabinet goal to reach a terminal state."""
        return self._cabinet_terminal_event.wait(
            timeout=max(0.0, timeout_sec)
        )

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        """Switch the base command router through its ROS 2 service."""
        if not enabled:
            result = self.takeover_navigation()
            return {
                **result,
                "enabled": False,
            }

        with self._lock:
            self._reject_if_cabinet_active_locked()
            if self._navigation_state["state"] == "taking_over":
                raise ControlRequestError(
                    "Manual takeover is still in progress.",
                    409,
                )
            if self._navigation_retirements:
                raise ControlRequestError(
                    "A previous Nav2 goal is still retiring.",
                    409,
                )
            if not self._navigation_mode_client.service_is_ready():
                raise ControlRequestError(
                    "Base navigation mode service is unavailable.",
                    503,
                )
            generation = self._navigation_generation
            self._request_navigation_mode_locked(True, generation)
        return {
            "status": "accepted",
            "enabled": True,
        }

    def takeover_navigation(self) -> Dict[str, Any]:
        """Cancel Nav2 and switch to manual mode while holding zero speed.

        This is deliberately a two-phase interface.  It never stores a
        manual motion command: callers must wait until the reported router
        mode is ``False`` and then send a fresh command through ``/cmd_vel``.
        """
        with self._lock:
            self._reject_if_cabinet_active_locked()
            service_ready = self._navigation_mode_client.service_is_ready()
            takeover_in_progress = (
                self._manual_takeover_generation
                == self._navigation_generation
                and self._navigation_state["state"] == "taking_over"
            )
            cancel_targets: List[Tuple[Any, int, Any]] = []
            if not takeover_in_progress:
                old_generation = self._navigation_generation
                old_goal_handle = self._navigation_goal_handle
                old_goal_token = self._navigation_goal_token
                if old_goal_token is not None:
                    retirement_key = (old_generation, old_goal_token)
                    self._navigation_retirements[retirement_key] = {
                        "handle": old_goal_handle,
                        "state": (
                            "canceling"
                            if old_goal_handle is not None
                            else "awaiting_acceptance"
                        ),
                        "error": None,
                        "result_channel_failed": False,
                    }
                    if old_goal_handle is not None:
                        cancel_targets.append(
                            (
                                old_goal_handle,
                                old_generation,
                                old_goal_token,
                            )
                        )

                self._navigation_generation += 1
                generation = self._navigation_generation
                self._manual_takeover_generation = generation
                self._navigation_cancel_requested = True
                self._navigation_goal_handle = None
                self._navigation_goal_token = None
                self._navigation_state.update(
                    {
                        "state": "taking_over",
                        "message": (
                            "Canceling Nav2 and switching to manual mode."
                        ),
                        "updated_at": time.time(),
                    }
                )
                # Queue the manual request even when discovery currently says
                # the service is unavailable.  This invalidates any older
                # enable transaction before the HTTP caller receives 503.
                self._request_navigation_mode_locked(False, generation)

            for (retirement_generation, retirement_token), retirement in (
                self._navigation_retirements.items()
            ):
                if (
                    retirement.get("handle") is not None
                    and retirement["state"] == "failed"
                ):
                    retirement["state"] = "canceling"
                    if not retirement.get("result_channel_failed", False):
                        retirement["error"] = None
                    cancel_targets.append(
                        (
                            retirement["handle"],
                            retirement_generation,
                            retirement_token,
                        )
                    )

            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
            self._last_command_time = time.monotonic()

        # A zero command is harmless in navigation mode and prevents an old
        # manual profile from surviving until the router changes ownership.
        self._cmd_vel_publisher.publish(Twist())
        canceled_handles = set()
        for goal_handle, goal_generation, goal_token in cancel_targets:
            if id(goal_handle) in canceled_handles:
                continue
            canceled_handles.add(id(goal_handle))
            self._cancel_navigation_goal(
                goal_handle,
                goal_generation,
                goal_token,
            )
        if not service_ready:
            raise ControlRequestError(
                "Base navigation mode service is unavailable; Nav2 was "
                "invalidated and cancellation was requested, but manual "
                "mode is not yet confirmed.",
                503,
            )
        with self._lock:
            status = (
                "manual"
                if self._manual_takeover_generation is None
                and self._navigation_mode is False
                and self._navigation_mode_acknowledged is False
                else "taking_over"
            )
            mode = self._navigation_mode
        return {
            "status": status,
            "mode": mode,
        }

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Dict[str, Any]:
        """Enable navigation mode and submit one NavigateToPose goal."""
        with self._lock:
            self._reject_if_cabinet_active_locked()
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
            self._reject_if_cabinet_active_locked()
            if self._navigation_retirements:
                raise ControlRequestError(
                    "A previous Nav2 goal is still retiring.",
                    409,
                )
            if (
                self._navigation_state["state"]
                in self.ACTIVE_NAVIGATION_STATES
            ):
                raise ControlRequestError(
                    "A navigation goal is already active.",
                    409,
                )
            self._navigation_generation += 1
            generation = self._navigation_generation
            self._navigation_cancel_requested = False
            self._manual_takeover_generation = None
            self._navigation_goal_handle = None
            self._navigation_goal_token = None
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
            self._request_navigation_mode_locked(True, generation)
        return {
            "status": "accepted",
            "goal": goal_description,
        }

    def cancel_navigation(self, allow_idle: bool = False) -> Dict[str, Any]:
        """Cancel the currently accepted Nav2 goal."""
        with self._lock:
            generation = self._navigation_generation
            goal_handle = self._navigation_goal_handle
            goal_token = self._navigation_goal_token
            active = (
                self._navigation_state["state"]
                in self.ACTIVE_NAVIGATION_STATES
                or goal_handle is not None
            )
            if not active:
                return {"status": "idle"}
            if self._navigation_state["state"] == "taking_over":
                return {"status": "taking_over"}
            self._navigation_cancel_requested = True
            self._navigation_state.update(
                {
                    "state": "canceling",
                    "message": (
                        "Canceling the active navigation goal."
                        if goal_handle is not None
                        else "Canceling navigation before goal acceptance."
                    ),
                    "updated_at": time.time(),
                }
            )
        if goal_handle is not None:
            self._cancel_navigation_goal(
                goal_handle,
                generation,
                goal_token,
            )
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
            if not self._cabinet_active_locked():
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

                # Publish while holding the state lock so a cabinet goal
                # cannot become active between the active check and send.
                command = Twist()
                command.linear.y = linear_y
                command.angular.z = angular_z
                self._cmd_vel_publisher.publish(command)
                if trajectory is not None:
                    self._trajectory_publisher.publish(trajectory)
        self._enforce_motion_cancel(now)

    def _request_navigation_mode_locked(
        self,
        enabled: bool,
        generation: int,
    ) -> None:
        """Record the latest desired mode and serialize service calls."""
        desired = (enabled, generation)
        if self._navigation_mode_desired != desired:
            self._navigation_mode_attempt = 0
        self._navigation_mode_desired = desired
        self._drive_navigation_mode_request_locked()

    def _drive_navigation_mode_request_locked(self) -> None:
        if (
            self._navigation_mode_request is not None
            or self._navigation_mode_desired is None
        ):
            return
        enabled, generation = self._navigation_mode_desired
        self._navigation_mode_request_sequence += 1
        self._navigation_mode_attempt += 1
        sequence = self._navigation_mode_request_sequence
        request_identity = (enabled, generation, sequence)
        self._navigation_mode_request = request_identity
        request = SetBool.Request()
        request.data = enabled
        try:
            future = self._navigation_mode_client.call_async(request)
        except Exception as error:  # noqa: BLE001
            self._navigation_mode_request = None
            if self._navigation_mode_desired == (enabled, generation):
                if (
                    self._navigation_mode_attempt
                    >= self.NAVIGATION_MODE_MAX_ATTEMPTS
                ):
                    self._navigation_mode_desired = None
                    self._navigation_mode_failed_locked(
                        enabled,
                        generation,
                        error,
                    )
            self._drive_navigation_mode_request_locked()
            return
        future.add_done_callback(
            lambda result_future: self._navigation_mode_request_callback(
                result_future,
                enabled,
                generation,
                sequence,
            )
        )

    def _navigation_mode_request_callback(
        self,
        future: Any,
        enabled: bool,
        generation: int,
        sequence: int,
    ) -> None:
        error: Optional[Exception] = None
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
        except Exception as caught_error:  # noqa: BLE001
            error = caught_error

        with self._lock:
            request_identity = (enabled, generation, sequence)
            if self._navigation_mode_request != request_identity:
                return
            self._navigation_mode_request = None
            desired_is_request = self._navigation_mode_desired == (
                enabled,
                generation,
            )
            if error is None:
                self._navigation_mode_acknowledged = enabled
                if desired_is_request:
                    self._navigation_mode_desired = None
                    self._navigation_mode_attempt = 0
                    self._navigation_mode_reached_locked(
                        enabled,
                        generation,
                    )
            elif desired_is_request:
                if (
                    self._navigation_mode_attempt
                    >= self.NAVIGATION_MODE_MAX_ATTEMPTS
                ):
                    self._navigation_mode_desired = None
                    self._navigation_mode_failed_locked(
                        enabled,
                        generation,
                        error,
                    )
            self._drive_navigation_mode_request_locked()

    def _navigation_mode_reached_locked(
        self,
        enabled: bool,
        generation: int,
    ) -> None:
        if generation != self._navigation_generation:
            return
        if not enabled:
            if self._manual_takeover_generation != generation:
                return
            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
            self._last_command_time = time.monotonic()
            self._manual_takeover_generation = None
            self._navigation_cancel_requested = False
            self._navigation_state.update(
                {
                    "state": "canceled",
                    "message": (
                        "Manual takeover complete; base speed remains zero."
                        if not self._navigation_retirements
                        else (
                            "Manual routing is active at zero speed; waiting "
                            "for the previous Nav2 goal to terminate."
                        )
                    ),
                    "updated_at": time.time(),
                }
            )
            return

        if self._navigation_state["state"] == "enabling":
            if self._navigation_cancel_requested:
                self._navigation_state.update(
                    {
                        "state": "canceled",
                        "message": (
                            "Navigation canceled before goal submission."
                        ),
                        "updated_at": time.time(),
                    }
                )
                return
            self._send_navigation_goal_locked(generation)
        elif (
            self._navigation_state["state"] == "canceling"
            and self._navigation_goal_token is None
        ):
            self._navigation_state.update(
                {
                    "state": "canceled",
                    "message": "Navigation canceled before goal submission.",
                    "updated_at": time.time(),
                }
            )

    def _navigation_mode_failed_locked(
        self,
        enabled: bool,
        generation: int,
        error: Exception,
    ) -> None:
        if generation != self._navigation_generation:
            return
        if not enabled and self._manual_takeover_generation == generation:
            self._manual_takeover_generation = None
            self._navigation_cancel_requested = False
            message = f"Failed to switch to manual mode: {error}"
        elif (
            enabled
            and self._navigation_state["state"] == "canceling"
            and self._navigation_goal_token is None
        ):
            self._navigation_cancel_requested = False
            self._navigation_state.update(
                {
                    "state": "canceled",
                    "message": "Navigation canceled before goal submission.",
                    "updated_at": time.time(),
                }
            )
            return
        elif enabled and self._navigation_state["state"] == "enabling":
            message = f"Failed to enable navigation mode: {error}"
        else:
            self.get_logger().error(
                f"Failed to switch base command mode: {error}"
            )
            return
        self._navigation_state.update(
            {
                "state": "failed",
                "message": message,
                "updated_at": time.time(),
            }
        )

    def _send_navigation_goal_locked(self, generation: int) -> None:
        if (
            generation != self._navigation_generation
            or self._navigation_state["state"] != "enabling"
        ):
            return
        goal_description = self._navigation_state["goal"]
        if goal_description is None:
            return
        goal_token = object()
        self._navigation_goal_token = goal_token
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
        try:
            future = self._navigation_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: (
                    self._navigation_feedback_callback(
                        feedback,
                        generation,
                        goal_token,
                    )
                ),
            )
        except Exception as error:  # noqa: BLE001
            self._set_navigation_terminal_locked(
                "failed",
                f"Failed to send Nav2 goal: {error}",
                generation,
                goal_token,
            )
            return
        future.add_done_callback(
            lambda result_future: self._navigation_goal_response_callback(
                result_future,
                generation,
                goal_token,
            )
        )

    def _navigation_goal_response_callback(
        self,
        future: Any,
        generation: int,
        goal_token: Any,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self._finish_navigation_retirement(generation, goal_token)
            self._set_navigation_terminal(
                "failed",
                f"Failed to send Nav2 goal: {error}",
                generation,
                goal_token,
            )
            return
        if not goal_handle.accepted:
            self._finish_navigation_retirement(generation, goal_token)
            self._set_navigation_terminal(
                "rejected",
                "Nav2 rejected the navigation goal.",
                generation,
                goal_token,
            )
            return

        with self._lock:
            is_current = (
                generation == self._navigation_generation
                and goal_token is self._navigation_goal_token
            )
            should_cancel = (
                not is_current or self._navigation_cancel_requested
            )
            if is_current:
                self._navigation_goal_handle = goal_handle
                self._navigation_state.update(
                    {
                        "state": (
                            "canceling" if should_cancel else "navigating"
                        ),
                        "message": (
                            "Canceling the accepted navigation goal."
                            if should_cancel
                            else "Nav2 is driving to the selected goal."
                        ),
                        "updated_at": time.time(),
                    }
                )
            else:
                retirement_key = (generation, goal_token)
                retirement = self._navigation_retirements.setdefault(
                    retirement_key,
                    {},
                )
                retirement.update(
                    {
                        "handle": goal_handle,
                        "state": "canceling",
                        "error": None,
                        "result_channel_failed": False,
                    }
                )

        if should_cancel:
            self._cancel_navigation_goal(
                goal_handle,
                generation,
                goal_token,
            )
        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:  # noqa: BLE001
            self._set_navigation_result_channel_failed(
                error,
                generation,
                goal_handle,
                goal_token,
            )
            return
        result_future.add_done_callback(
            lambda completed_future: self._navigation_result_callback(
                completed_future,
                generation,
                goal_handle,
                goal_token,
            )
        )

    def _navigation_feedback_callback(
        self,
        feedback_message: Any,
        generation: int,
        goal_token: Any,
    ) -> None:
        feedback = feedback_message.feedback
        with self._lock:
            if (
                generation != self._navigation_generation
                or goal_token is not self._navigation_goal_token
            ):
                return
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

    def _navigation_result_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
        goal_token: Any,
    ) -> None:
        try:
            wrapped_result = future.result()
            state, message = self._goal_status(
                wrapped_result.status,
                "Navigation",
            )
        except Exception as error:  # noqa: BLE001
            self._set_navigation_result_channel_failed(
                error,
                generation,
                goal_handle,
                goal_token,
            )
            return
        self._finish_navigation_retirement(
            generation,
            goal_token,
            goal_handle,
        )
        self._set_navigation_terminal(
            state,
            message,
            generation,
            goal_token,
            goal_handle,
        )

    def _set_navigation_result_channel_failed(
        self,
        error: Exception,
        generation: int,
        goal_handle: Any,
        goal_token: Any,
    ) -> None:
        """Keep an unconfirmed goal blocking after result-channel failure."""
        with self._lock:
            is_current = (
                generation == self._navigation_generation
                and goal_handle is self._navigation_goal_handle
                and goal_token is self._navigation_goal_token
            )
            retirement_key = (generation, goal_token)
            retirement = self._navigation_retirements.get(retirement_key)
            if not is_current and retirement is None:
                # A stale accepted goal is still unsafe even if its earlier
                # bookkeeping was lost; reconstruct the retirement guard.
                retirement = {}
                self._navigation_retirements[retirement_key] = retirement
            elif is_current and retirement is None:
                retirement = {}
                self._navigation_retirements[retirement_key] = retirement
            if retirement is None:
                return
            retirement.update(
                {
                    "handle": goal_handle,
                    "state": "failed",
                    "error": f"Navigation result channel failed: {error}",
                    "result_channel_failed": True,
                }
            )
            if not is_current:
                return
            self._navigation_goal_handle = None
            self._navigation_goal_token = None
            self._navigation_cancel_requested = False
            self._navigation_state.update(
                {
                    "state": "failed",
                    "message": (
                        "Navigation result channel failed; goal termination "
                        f"is unconfirmed: {error}"
                    ),
                    "updated_at": time.time(),
                }
            )

    def _cancel_navigation_goal(
        self,
        goal_handle: Any,
        generation: int,
        goal_token: Any,
    ) -> None:
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001
            self._set_navigation_cancel_failed(
                error,
                generation,
                goal_handle,
                goal_token,
            )
            return
        future.add_done_callback(
            lambda result_future: self._navigation_cancel_callback(
                result_future,
                generation,
                goal_handle,
                goal_token,
            )
        )

    def _navigation_cancel_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
        goal_token: Any,
    ) -> None:
        try:
            response = future.result()
            if not response.goals_canceling:
                raise RuntimeError("Nav2 did not accept the cancel request.")
        except Exception as error:  # noqa: BLE001
            self._set_navigation_cancel_failed(
                error,
                generation,
                goal_handle,
                goal_token,
            )
            return
        with self._lock:
            retirement = self._navigation_retirements.get(
                (generation, goal_token)
            )
            if (
                retirement is not None
                and retirement.get("handle") is goal_handle
            ):
                if not retirement.get("result_channel_failed", False):
                    retirement["state"] = "cancel_accepted"
                    retirement["error"] = None

    def _set_navigation_cancel_failed(
        self,
        error: Exception,
        generation: int,
        goal_handle: Any,
        goal_token: Any,
    ) -> None:
        with self._lock:
            retirement = self._navigation_retirements.get(
                (generation, goal_token)
            )
            if (
                retirement is not None
                and retirement.get("handle") is goal_handle
            ):
                retirement["state"] = "failed"
                if retirement.get("result_channel_failed", False):
                    retirement["error"] = (
                        f"{retirement['error']}; cancellation also failed: "
                        f"{error}"
                    )
                else:
                    retirement["error"] = str(error)
                if self._navigation_state["state"] in {
                    "taking_over",
                    "canceled",
                }:
                    self._navigation_state.update(
                        {
                            "message": (
                                "Manual routing is safe, but the previous "
                                "Nav2 goal did not accept cancellation: "
                                f"{error}"
                            ),
                            "updated_at": time.time(),
                        }
                    )
                return
            if (
                generation != self._navigation_generation
                or goal_handle is not self._navigation_goal_handle
                or goal_token is not self._navigation_goal_token
            ):
                return
            self._navigation_retirements[(generation, goal_token)] = {
                "handle": goal_handle,
                "state": "failed",
                "error": str(error),
                "result_channel_failed": False,
            }
            self._navigation_state.update(
                {
                    "state": "failed",
                    "message": f"Navigation cancel failed: {error}",
                    "updated_at": time.time(),
                }
            )

    def _finish_navigation_retirement(
        self,
        generation: int,
        goal_token: Any,
        goal_handle: Any = None,
    ) -> None:
        with self._lock:
            retirement = self._navigation_retirements.get(
                (generation, goal_token)
            )
            if retirement is None:
                return
            if (
                goal_handle is not None
                and retirement.get("handle") is not None
                and retirement.get("handle") is not goal_handle
            ):
                return
            self._navigation_retirements.pop(
                (generation, goal_token),
                None,
            )

    def _set_navigation_terminal(
        self,
        state: str,
        message: str,
        generation: int,
        goal_token: Any,
        goal_handle: Any = None,
    ) -> None:
        with self._lock:
            self._set_navigation_terminal_locked(
                state,
                message,
                generation,
                goal_token,
                goal_handle,
            )

    def _set_navigation_terminal_locked(
        self,
        state: str,
        message: str,
        generation: int,
        goal_token: Any,
        goal_handle: Any = None,
    ) -> None:
        if (
            generation != self._navigation_generation
            or goal_token is not self._navigation_goal_token
            or (
                goal_handle is not None
                and goal_handle is not self._navigation_goal_handle
            )
        ):
            return
        self._navigation_goal_handle = None
        self._navigation_goal_token = None
        self._navigation_cancel_requested = False
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
        with self._lock:
            self._reject_if_cabinet_active_locked()
        if not self._action_server_ready(self._move_group_client):
            raise ControlRequestError(
                "MoveIt MoveGroup action server is unavailable.",
                503,
            )
        with self._lock:
            self._reject_if_cabinet_active_locked()
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

    def _cabinet_goal_response_callback(
        self,
        future: Any,
        generation: int,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            with self._lock:
                if generation != self._cabinet_generation:
                    return
                canceled_before_acceptance = (
                    self._cabinet_cancel_requested
                )
            self._set_cabinet_terminal(
                "canceled" if canceled_before_acceptance else "failed",
                (
                    "Cabinet button operation was canceled before "
                    "goal acceptance."
                    if canceled_before_acceptance
                    else f"Failed to send cabinet button goal: {error}"
                ),
                False,
                (
                    PressCabinetButton.Result.CANCELED
                    if canceled_before_acceptance
                    else None
                ),
                generation=generation,
            )
            return

        if not goal_handle.accepted:
            self._set_cabinet_terminal(
                "rejected",
                "Cabinet button action server rejected the goal.",
                False,
                None,
                generation=generation,
            )
            return

        with self._lock:
            if generation != self._cabinet_generation:
                return
            canceled_before_acceptance = self._cabinet_cancel_requested
            self._cabinet_goal_handle = goal_handle
            self._cabinet_state.update(
                {
                    "state": (
                        "canceling"
                        if canceled_before_acceptance
                        else "operating"
                    ),
                    "message": (
                        "Canceling the accepted cabinet button goal."
                        if canceled_before_acceptance
                        else "Cabinet button operation is running."
                    ),
                    "updated_at": time.time(),
                }
            )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._cabinet_result_callback(
                result,
                generation,
                goal_handle,
            )
        )
        if canceled_before_acceptance:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda result: self._cabinet_cancel_callback(
                        result,
                        generation,
                        goal_handle,
                    )
                )
            except Exception as error:  # noqa: BLE001
                with self._lock:
                    if (
                        generation == self._cabinet_generation
                        and self._cabinet_goal_handle is goal_handle
                        and self._cabinet_active_locked()
                    ):
                        self._cabinet_cancel_requested = False
                        self._cabinet_state.update(
                            {
                                "state": "operating",
                                "message": (
                                    "Cabinet operation cancellation "
                                    f"failed: {error}"
                                ),
                                "updated_at": time.time(),
                            }
                        )

    def _cabinet_feedback_callback(
        self,
        feedback_message: Any,
        generation: int,
    ) -> None:
        feedback = feedback_message.feedback
        travel = max(0.0, float(feedback.button_travel))
        with self._lock:
            if (
                generation != self._cabinet_generation
                or not self._cabinet_active_locked()
            ):
                return
            previous_max = self._cabinet_state["max_travel"]
            max_travel = max(
                float(previous_max) if previous_max is not None else 0.0,
                travel,
            )
            update = {
                "phase": int(feedback.phase),
                "progress": float(feedback.progress),
                "max_travel": max_travel,
                "updated_at": time.time(),
            }
            if self._cabinet_state["state"] != "canceling":
                update["message"] = str(feedback.message)
            self._cabinet_state.update(update)

    def _cabinet_result_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
    ) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            action_status = int(wrapped_result.status)
            error_code = int(result.error_code)
            max_travel = max(0.0, float(result.max_travel))
            result_message = str(result.message)

            if (
                action_status == GoalStatus.STATUS_CANCELED
                or error_code == PressCabinetButton.Result.CANCELED
            ):
                state = "canceled"
                success = False
                message = (
                    result_message
                    or "Cabinet button operation was canceled."
                )
            elif (
                action_status == GoalStatus.STATUS_SUCCEEDED
                and bool(result.success)
                and error_code == PressCabinetButton.Result.SUCCESS
            ):
                state = "succeeded"
                success = True
                message = (
                    result_message
                    or "Cabinet button operation succeeded."
                )
            else:
                state = "failed"
                success = False
                _, fallback_message = self._goal_status(
                    action_status,
                    "Cabinet button operation",
                )
                message = result_message or fallback_message
        except Exception as error:  # noqa: BLE001
            state = "failed"
            success = False
            message = f"Cabinet button result failed: {error}"
            error_code = None
            max_travel = None

        self._set_cabinet_terminal(
            state,
            message,
            success,
            error_code,
            max_travel,
            generation=generation,
            expected_goal_handle=goal_handle,
        )

    def _cabinet_cancel_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
    ) -> None:
        try:
            response = future.result()
            cancellation_accepted = bool(response.goals_canceling)
            error_message = ""
        except Exception as error:  # noqa: BLE001
            cancellation_accepted = False
            error_message = str(error)

        with self._lock:
            if (
                generation != self._cabinet_generation
                or self._cabinet_goal_handle is not goal_handle
                or not self._cabinet_active_locked()
            ):
                return
            if cancellation_accepted:
                self._cabinet_state.update(
                    {
                        "message": (
                            "Cabinet button cancellation was accepted."
                        ),
                        "updated_at": time.time(),
                    }
                )
                return

            self._cabinet_cancel_requested = False
            self._cabinet_state.update(
                {
                    "state": "operating",
                    "message": (
                        "Cabinet action server did not accept cancellation."
                        if not error_message
                        else (
                            "Cabinet operation cancellation failed: "
                            f"{error_message}"
                        )
                    ),
                    "updated_at": time.time(),
                }
            )

    def _set_cabinet_terminal(
        self,
        state: str,
        message: str,
        success: bool,
        error_code: Optional[int],
        max_travel: Optional[float] = None,
        generation: Optional[int] = None,
        expected_goal_handle: Any = None,
    ) -> None:
        with self._lock:
            if (
                generation is not None
                and generation != self._cabinet_generation
            ):
                return
            if (
                expected_goal_handle is not None
                and self._cabinet_goal_handle is not expected_goal_handle
            ):
                return
            observed_max = self._cabinet_state["max_travel"]
            if max_travel is not None:
                observed_max = max(
                    float(observed_max)
                    if observed_max is not None
                    else 0.0,
                    max_travel,
                )
            self._cabinet_goal_handle = None
            self._cabinet_cancel_requested = False
            self._cabinet_state.update(
                {
                    "state": state,
                    "message": message,
                    "progress": (
                        1.0
                        if state == "succeeded"
                        else self._cabinet_state["progress"]
                    ),
                    "max_travel": observed_max,
                    "success": success,
                    "error_code": error_code,
                    "updated_at": time.time(),
                }
            )
            self._cabinet_terminal_event.set()

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
            # This topic is observation only.  Transaction progress is driven
            # by the matching SetBool response so a delayed retained sample
            # can never acknowledge a newer command.
            self._navigation_mode = bool(message.data)

    @staticmethod
    def _new_cabinet_control_state() -> Dict[str, Any]:
        return {
            "button_pressed": None,
            "button_travel": None,
            "button_state_updated_at": None,
        }

    def _cabinet_control_catalog_callback(
        self,
        message: CabinetControlCatalog,
    ) -> None:
        controls: Dict[str, Dict[str, Any]] = {}
        for entry in message.controls:
            control_id = str(entry.control_id).strip()
            if not control_id or control_id in controls:
                continue
            joint_name = str(entry.joint_name).strip()
            joint_state_topic = str(entry.joint_state_topic).strip()
            pressed_topic = str(entry.pressed_topic).strip()
            control_type = int(entry.control_type)
            if (
                control_type != self.CABINET_CONTROL_TYPE_BUTTON
                or not joint_name
                or not joint_state_topic
                or not pressed_topic
            ):
                continue
            controls[control_id] = {
                "control_id": control_id,
                "display_name": (
                    str(entry.display_name).strip() or control_id
                ),
                "control_type": control_type,
                "joint_name": joint_name,
                "joint_state_topic": joint_state_topic,
                "pressed_topic": pressed_topic,
            }

        if not controls:
            self.get_logger().warning(
                "Ignored an empty or invalid cabinet control catalog."
            )
            return

        with self._lock:
            previous_controls = self._cabinet_controls
            previous_states = self._cabinet_control_states
            self._cabinet_controls = controls
            self._cabinet_control_states = {
                control_id: (
                    dict(previous_states[control_id])
                    if (
                        control_id in previous_states
                        and self._cabinet_control_signature(
                            previous_controls.get(control_id)
                        ) == self._cabinet_control_signature(control)
                    )
                    else self._new_cabinet_control_state()
                )
                for control_id, control in controls.items()
            }
            removed_control_ids = set(previous_controls) - set(controls)
            self._cabinet_catalog_received = True

        for control_id in removed_control_ids:
            self._remove_cabinet_control_subscriptions(control_id)

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for control in controls.values():
            self._ensure_cabinet_control_subscriptions(
                control,
                transient_qos,
            )

    @staticmethod
    def _cabinet_control_signature(
        control: Optional[Dict[str, Any]],
    ) -> Optional[Tuple[str, str, str]]:
        if control is None:
            return None
        return (
            str(control.get("joint_name", "")),
            str(control.get("joint_state_topic", "")),
            str(control.get("pressed_topic", "")),
        )

    def _remove_cabinet_control_subscriptions(
        self,
        control_id: str,
    ) -> None:
        subscriptions = self._cabinet_control_subscriptions.pop(
            control_id,
            None,
        )
        if subscriptions is None:
            return
        for subscription_name in ("joint", "pressed"):
            subscription = subscriptions.get(subscription_name)
            if subscription is not None:
                self.destroy_subscription(subscription)

    def _ensure_cabinet_control_subscriptions(
        self,
        control: Dict[str, Any],
        transient_qos: QoSProfile,
    ) -> None:
        control_id = str(control["control_id"])
        joint_name = str(control["joint_name"])
        joint_state_topic = str(control["joint_state_topic"])
        pressed_topic = str(control["pressed_topic"])

        signature = (joint_name, joint_state_topic, pressed_topic)
        existing = self._cabinet_control_subscriptions.get(control_id)
        if existing is not None and existing.get("signature") == signature:
            return

        joint_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            lambda message, selected_id=control_id, selected_joint=(
                joint_name
            ), selected_topic=(
                joint_state_topic
            ): self._cabinet_button_joint_state_callback(
                selected_id,
                selected_joint,
                selected_topic,
                message,
            ),
            10,
        )
        try:
            pressed_subscription = self.create_subscription(
                Bool,
                pressed_topic,
                lambda message, selected_id=control_id, selected_topic=(
                    pressed_topic
                ): (
                    self._cabinet_button_pressed_callback(
                        selected_id,
                        selected_topic,
                        message,
                    )
                ),
                transient_qos,
            )
        except Exception:
            self.destroy_subscription(joint_subscription)
            raise

        self._remove_cabinet_control_subscriptions(control_id)
        self._cabinet_control_subscriptions[control_id] = {
            "signature": signature,
            "joint": joint_subscription,
            "pressed": pressed_subscription,
        }

    def _cabinet_button_joint_state_callback(
        self,
        control_id: str,
        joint_name: str,
        joint_state_topic: str,
        message: JointState,
    ) -> None:
        try:
            joint_index = message.name.index(joint_name)
            travel = float(message.position[joint_index])
        except (ValueError, IndexError, TypeError):
            return
        if not math.isfinite(travel):
            return
        with self._lock:
            control = self._cabinet_controls.get(control_id)
            if (
                control is None
                or control.get("joint_name") != joint_name
                or control.get("joint_state_topic") != joint_state_topic
            ):
                return
            state = self._cabinet_control_states.setdefault(
                control_id,
                self._new_cabinet_control_state(),
            )
            state.update(
                {
                    "button_travel": max(0.0, travel),
                    "button_state_updated_at": time.time(),
                }
            )
            if self._cabinet_state.get("button_id") == control_id:
                self._cabinet_state.update(state)

    def _cabinet_button_pressed_callback(
        self,
        control_id: str,
        pressed_topic: str,
        message: Bool,
    ) -> None:
        with self._lock:
            control = self._cabinet_controls.get(control_id)
            if (
                control is None
                or control.get("pressed_topic") != pressed_topic
            ):
                return
            state = self._cabinet_control_states.setdefault(
                control_id,
                self._new_cabinet_control_state(),
            )
            state.update(
                {
                    "button_pressed": bool(message.data),
                    "button_state_updated_at": time.time(),
                }
            )
            if self._cabinet_state.get("button_id") == control_id:
                self._cabinet_state.update(state)

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

    def _cabinet_active_locked(self) -> bool:
        return self._cabinet_state["state"] in self.ACTIVE_CABINET_STATES

    def _manual_control_ready_locked(self) -> bool:
        return (
            self._navigation_mode is False
            and self._navigation_mode_acknowledged is False
            and self._manual_takeover_generation is None
            and not self._navigation_mode_transition_pending_locked()
            and self._navigation_state["state"]
            not in self.ACTIVE_NAVIGATION_STATES
            and not self._cabinet_active_locked()
        )

    def _navigation_mode_transition_pending_locked(self) -> bool:
        return (
            self._navigation_mode_request is not None
            or self._navigation_mode_desired is not None
        )

    def _reject_if_cabinet_active_locked(self) -> None:
        if self._cabinet_active_locked():
            raise ControlRequestError(
                "A cabinet button operation is active.",
                409,
            )

    def _validate_cabinet_button_locked(self, button_id: str) -> None:
        control = self._cabinet_controls.get(button_id)
        if (
            control is None
            or int(control.get("control_type", -1))
            != self.CABINET_CONTROL_TYPE_BUTTON
        ):
            raise ControlRequestError(
                f"Unsupported cabinet button: {button_id}."
            )

    def _reject_cabinet_start_conflicts_locked(self) -> None:
        if self._cabinet_active_locked():
            raise ControlRequestError(
                "A cabinet button operation is already active.",
                409,
            )
        if self._navigation_state["state"] in self.ACTIVE_NAVIGATION_STATES:
            raise ControlRequestError(
                "Navigation is active; cancel it before operating "
                "the cabinet.",
                409,
            )
        if self._navigation_mode_transition_pending_locked():
            raise ControlRequestError(
                "A base mode transition is pending; wait before operating "
                "the cabinet.",
                409,
            )
        if self._navigation_retirements:
            raise ControlRequestError(
                "A previous Nav2 goal is still retiring; wait before "
                "operating the cabinet.",
                409,
            )
        if self._motion_state["state"] in self.ACTIVE_MOTION_STATES:
            raise ControlRequestError(
                "MoveIt is active; cancel it before operating the cabinet.",
                409,
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
