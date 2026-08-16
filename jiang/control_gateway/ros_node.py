"""ROS 2 publishers, services and actions used by the browser gateway."""

from __future__ import annotations

import math
import json
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time as RosTime
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from xczs_inspection_robot_control.action import OperateCabinetControl
from xczs_inspection_robot_control.action import PressCabinetButton
from xczs_inspection_robot_control.msg import CabinetControl
from xczs_inspection_robot_control.msg import CabinetControlCatalog
from xczs_inspection_robot_control.msg import CabinetControlState
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from .inventory import NavigationStation
from .inventory import NavigationStationSpec
from .robot_adapter import ManualJointConfig
from .velocity_profile import VelocityProfile


class ControlRequestError(RuntimeError):
    """Validated request error that can be returned through the HTTP API."""

    def __init__(
        self,
        message: str,
        status: int = 400,
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.details = dict(details or {})


class RosControlNode(Node):
    """Own manual publishers and autonomous ROS 2 clients."""

    CABINET_CONTROL_TYPE_BUTTON = CabinetControl.TYPE_BUTTON
    CABINET_CONTROL_TYPE_KNOB = CabinetControl.TYPE_KNOB
    CABINET_CONTROL_TYPE_SWITCH = CabinetControl.TYPE_SWITCH
    CABINET_CONTROL_TYPE_DOOR = CabinetControl.TYPE_DOOR
    CABINET_CONTROL_TYPES = {
        CABINET_CONTROL_TYPE_BUTTON,
        CABINET_CONTROL_TYPE_KNOB,
        CABINET_CONTROL_TYPE_SWITCH,
        CABINET_CONTROL_TYPE_DOOR,
    }
    CABINET_COMMANDS = {
        "press": OperateCabinetControl.Goal.COMMAND_PRESS,
        "set_state": OperateCabinetControl.Goal.COMMAND_SET_STATE,
        "set_position": OperateCabinetControl.Goal.COMMAND_SET_POSITION,
        "toggle": OperateCabinetControl.Goal.COMMAND_TOGGLE,
    }
    CABINET_COMMAND_NAMES = {
        value: name for name, value in CABINET_COMMANDS.items()
    }
    CABINET_COMMAND_SUPPORT = {
        OperateCabinetControl.Goal.COMMAND_PRESS: CabinetControl.SUPPORT_PRESS,
        OperateCabinetControl.Goal.COMMAND_SET_STATE: (
            CabinetControl.SUPPORT_SET_STATE
        ),
        OperateCabinetControl.Goal.COMMAND_SET_POSITION: (
            CabinetControl.SUPPORT_SET_POSITION
        ),
        OperateCabinetControl.Goal.COMMAND_TOGGLE: CabinetControl.SUPPORT_TOGGLE,
    }
    CABINET_DETENT_TOLERANCE = 0.035
    NAVIGATION_MODE_MAX_ATTEMPTS = 3
    NAVIGATION_READINESS_POLL_SECONDS = 0.25
    NAVIGATION_READINESS_REQUEST_TIMEOUT_SECONDS = 2.0
    # A discovered-but-unresponsive action server leaves a goal stuck in
    # 'sending' (or 'canceling' before acceptance) forever; the watchdog below
    # fails such goals so the state machine returns to idle instead of every
    # later operation being rejected with 409 until restart.
    GOAL_ACCEPTANCE_TIMEOUT_SEC = 5.0
    GOAL_ACCEPTANCE_WATCHDOG_PERIOD_SEC = 1.0

    TASK_PUBLISHER_HISTORY_LIMIT = 256
    DEFAULT_MANUAL_TRAJECTORY_DURATION_SECONDS = 0.50
    MANUAL_TRAJECTORY_SETTLE_MARGIN_SECONDS = 0.10
    ACTIVE_NAVIGATION_STATES = {
        "enabling",
        "sending",
        "navigating",
        "canceling",
        "taking_over",
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
        navigation_frame: str = "map",
        navigation_base_frame: str = "base_link",
        navigation_action: str = "/navigate_to_pose",
        navigation_readiness_service: Optional[str] = None,
        navigation_mode_service: str = "/xczs/set_navigation_mode",
        navigation_mode_topic: str = "/xczs/navigation_mode",
        map_topic: str = "/map",
        localization_pose_topic: str = "/amcl_pose",
        map_load_service: str = "/map_server/load_map",
        initial_pose_topic: str = "/initialpose",
        manual_linear_axis: str = "y",
        manual_joints: Sequence[ManualJointConfig] = (),
        joint_state_topic: str = "/xczs/joint_states",
        cabinet_pose_valid_topics: Optional[Mapping[str, str]] = None,
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
        self._navigation_frame = navigation_frame
        self._navigation_base_frame = navigation_base_frame
        self._manual_joints = tuple(manual_joints)
        if not self._manual_joints:
            raise ValueError("manual_joints must not be empty.")
        if not isinstance(joint_state_topic, str) or not joint_state_topic.strip():
            raise ValueError("joint_state_topic must be a non-empty ROS name.")
        if manual_linear_axis not in {"x", "y"}:
            raise ValueError("manual_linear_axis must be either x or y.")
        self._manual_linear_axis = manual_linear_axis
        self._robot_joint_state: Dict[str, Any] = {
            "available": False,
            "positions": {},
            "stamp_ros_nanoseconds": None,
            "received_monotonic": None,
        }
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        self._linear_profile = VelocityProfile(0.50, 2.00)
        self._angular_profile = VelocityProfile(1.20, 4.80)
        self._target_linear_y = 0.0
        self._target_angular_z = 0.0
        self._last_command_time = time.monotonic()
        self._last_update_time = time.monotonic()
        self._pending_trajectory: Optional[JointTrajectory] = None
        self._pending_trajectory_repeats = 0
        self._pending_trajectory_duration_sec = 0.0
        self._manual_trajectory_active_until = 0.0
        self._cabinet_catalog_received = False
        self._cabinet_controls: Dict[str, Dict[str, Any]] = {}
        self._cabinet_control_states: Dict[str, Dict[str, Any]] = {}
        self._cabinet_control_subscriptions: Dict[
            str, Dict[str, Any]
        ] = {}
        self._task_publishers: Dict[str, Dict[str, Any]] = {}

        self._navigation_client = ActionClient(
            self,
            NavigateToPose,
            navigation_action,
        )
        self._navigation_readiness_service = (
            navigation_readiness_service
            or self._navigation_readiness_service_for_action(navigation_action)
        )
        self._navigation_readiness_client = self.create_client(
            Trigger,
            self._navigation_readiness_service,
        )
        self._navigation_readiness_active = False
        self._navigation_readiness_state = "checking"
        self._navigation_readiness_message = (
            "Waiting for the Nav2 lifecycle manager readiness response."
        )
        self._navigation_readiness_future: Any = None
        self._navigation_readiness_future_started: Optional[float] = None
        self._navigation_readiness_generation = 0
        self._navigation_mode_client = self.create_client(
            SetBool,
            navigation_mode_service,
        )
        self._map_load_client = self.create_client(
            LoadMap,
            map_load_service,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            10,
        )
        self._cabinet_button_client = ActionClient(
            self,
            PressCabinetButton,
            "/xczs/press_cabinet_button",
        )
        self._cabinet_operation_client = ActionClient(
            self,
            OperateCabinetControl,
            "/xczs/operate_cabinet_control",
        )
        self._cabinet_reset_client = self.create_client(
            Trigger,
            "/xczs/cabinet/reset_controls",
        )
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            navigation_mode_topic,
            self._navigation_mode_callback,
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._map_callback,
            transient_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            localization_pose_topic,
            self._pose_callback,
            10,
        )
        self._robot_joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic.strip(),
            self._robot_joint_state_callback,
            qos_profile_sensor_data,
        )
        self._cabinet_pose_validity: Dict[str, Optional[bool]] = {}
        self._cabinet_pose_validity_subscriptions = []
        for cabinet, topic in dict(cabinet_pose_valid_topics or {}).items():
            if not isinstance(cabinet, str) or not cabinet.strip():
                raise ValueError("cabinet pose-validity names must be non-empty.")
            if not isinstance(topic, str) or not topic.strip():
                raise ValueError("cabinet pose-validity topics must be non-empty.")
            normalized_cabinet = cabinet.strip()
            self._cabinet_pose_validity[normalized_cabinet] = None
            self._cabinet_pose_validity_subscriptions.append(
                self.create_subscription(
                    Bool,
                    topic.strip(),
                    lambda message, name=normalized_cabinet: (
                        self._cabinet_pose_validity_callback(name, message)
                    ),
                    transient_qos,
                )
            )
        self.create_subscription(
            CabinetControlCatalog,
            "/xczs/cabinet/control_catalog",
            self._cabinet_control_catalog_callback,
            transient_qos,
        )
        self._navigation_mode: Optional[bool] = None
        self._navigation_mode_acknowledged: Optional[bool] = None
        self._navigation_generation = 0
        self._navigation_goal_handle: Any = None
        self._navigation_goal_token: Any = None
        self._navigation_goal_sent_monotonic: Optional[float] = None
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
            "goal_sent_ros_nanoseconds": None,
            "distance_remaining": None,
            "eta_seconds": None,
            "navigation_time_seconds": None,
            "recoveries": 0,
            "updated_at": time.time(),
        }
        self._map_state: Optional[Dict[str, Any]] = None
        self._map_error = "Nav2 occupancy map has not been received."
        self._robot_pose: Optional[Dict[str, Any]] = None
        self._robot_pose_sequence = 0

        self._cabinet_goal_handle: Any = None
        self._cabinet_cancel_requested = False
        self._cabinet_generation = 0
        self._cabinet_goal_sent_monotonic: Optional[float] = None
        self._cabinet_terminal_event = threading.Event()
        self._cabinet_terminal_event.set()
        self._cabinet_state: Dict[str, Any] = {
            "state": "idle",
            "message": "No cabinet operation has been sent.",
            "control_id": "",
            "control_type": None,
            "type": None,
            "command": None,
            "command_code": None,
            "target_state": None,
            "target_position": None,
            "target": {"state": None, "position": None},
            "action_interface": None,
            "button_id": "",
            "navigate_to_staging_pose": True,
            "phase": None,
            "progress": 0.0,
            "button_pressed": None,
            "button_travel": None,
            "max_travel": None,
            "initial_position": None,
            "final_position": None,
            "peak_position": None,
            "final_state": None,
            "success": None,
            "error_code": None,
            "updated_at": time.time(),
            "button_state_updated_at": None,
        }
        self.create_timer(0.02, self._update_manual_control)
        self.create_timer(
            self.NAVIGATION_READINESS_POLL_SECONDS,
            self._poll_navigation_readiness,
        )
        self.create_timer(
            self.GOAL_ACCEPTANCE_WATCHDOG_PERIOD_SEC,
            self._check_goal_acceptance,
        )

    def publish_task_event(self, event: Dict[str, Any]) -> None:
        """Mirror one task event to its ROS 2 progress or result topic."""
        data = event.get("data")
        task_id = data.get("task_id") if isinstance(data, dict) else None
        if not isinstance(task_id, str) or not task_id:
            return
        with self._lock:
            publishers = self._task_publishers.get(task_id)
            if publishers is None:
                if (
                    len(self._task_publishers)
                    >= self.TASK_PUBLISHER_HISTORY_LIMIT
                ):
                    expired_task_id, expired = next(
                        iter(self._task_publishers.items())
                    )
                    self._task_publishers.pop(expired_task_id)
                    self.destroy_publisher(expired["progress"])
                    self.destroy_publisher(expired["result"])
                result_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                publishers = {
                    "progress": self.create_publisher(
                        String,
                        f"/xczs/task/{task_id}/progress",
                        10,
                    ),
                    "result": self.create_publisher(
                        String,
                        f"/xczs/task/{task_id}/result",
                        result_qos,
                    ),
                }
                self._task_publishers[task_id] = publishers
        message = String()
        message.data = json.dumps(
            event,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        event_type = event.get("event")
        if event_type == "task_completed":
            publishers["result"].publish(message)
        elif event_type == "task_reservation_released":
            # Replace the transient-local terminal snapshot as well as notify
            # live progress listeners. A late subscriber must not believe the
            # old reservation_active=true result still owns the robot.
            publishers["progress"].publish(message)
            publishers["result"].publish(message)
        else:
            publishers["progress"].publish(message)

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

    def set_joint_target(
        self,
        positions: List[float],
        duration_sec: float = DEFAULT_MANUAL_TRAJECTORY_DURATION_SECONDS,
    ) -> List[float]:
        """Queue one legacy manual joint target."""
        if (
            isinstance(duration_sec, bool)
            or not isinstance(duration_sec, (int, float))
            or not math.isfinite(float(duration_sec))
            or float(duration_sec) <= 0.0
        ):
            raise ControlRequestError(
                "duration_sec must be a positive finite number."
            )
        duration_sec = float(duration_sec)
        duration_nanoseconds = round(duration_sec * 1_000_000_000)
        duration_seconds, duration_remainder = divmod(
            duration_nanoseconds,
            1_000_000_000,
        )
        if duration_nanoseconds <= 0 or duration_seconds > 2_147_483_647:
            raise ControlRequestError(
                "duration_sec is outside the ROS duration range."
            )
        if len(positions) != len(self._manual_joints):
            raise ControlRequestError(
                "positions must match the configured manual joint order "
                f"({len(self._manual_joints)} values)."
            )
        joint_names = [joint.name for joint in self._manual_joints]
        safe_positions = [
            max(joint.min_position, min(joint.max_position, value))
            for joint, value in zip(self._manual_joints, positions)
        ]

        trajectory = JointTrajectory()
        trajectory.header.frame_id = "world"
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = safe_positions
        # Give the controller an explicit interpolation window. Reset callers
        # can select a slower, safer trajectory while legacy requests retain
        # the historical 0.5 s default.
        point.time_from_start.sec = duration_seconds
        point.time_from_start.nanosec = duration_remainder
        trajectory.points.append(point)

        with self._lock:
            self._reject_if_cabinet_active_locked()
            self._pending_trajectory = trajectory
            self._pending_trajectory_repeats = 6
            self._pending_trajectory_duration_sec = duration_sec
        return safe_positions

    def emergency_stop(self) -> None:
        """Stop only the manual base publisher."""
        with self._lock:
            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
        self._cmd_vel_publisher.publish(Twist())

    def quiesce_manual_outputs(self) -> float:
        """Stop queued manual writes and return remaining settle time.

        A trajectory already accepted through the legacy topic cannot be
        recalled. Replay admission therefore keeps the gateway write lock until
        its configured interpolation window plus a small margin expires.
        Pending repeats are discarded immediately.
        """
        now = time.monotonic()
        with self._lock:
            self._target_linear_y = 0.0
            self._target_angular_z = 0.0
            self._linear_profile.reset()
            self._angular_profile.reset()
            self._pending_trajectory = None
            self._pending_trajectory_repeats = 0
            self._pending_trajectory_duration_sec = 0.0
            settle_seconds = max(
                0.0,
                float(getattr(self, "_manual_trajectory_active_until", 0.0))
                - now,
            )
        self._cmd_vel_publisher.publish(Twist())
        return settle_seconds

    def robot_joint_state_snapshot(self) -> Dict[str, Any]:
        """Return an isolated snapshot of the latest manual-joint positions."""
        with self._lock:
            state = self._robot_joint_state
            return {
                "available": bool(state["available"]),
                "positions": dict(state["positions"]),
                "stamp_ros_nanoseconds": state["stamp_ros_nanoseconds"],
                "received_monotonic": state["received_monotonic"],
            }

    def navigation_snapshot(self) -> Dict[str, Any]:
        """Return current Nav2 availability, feedback and display overlays."""
        # AMCL publishes ``amcl_pose`` only when its particle-filter estimate
        # changes.  A robot that has stopped at the goal can therefore have a
        # perfectly current map->base transform while the last amcl_pose
        # message is many seconds old.  Query TF at snapshot time so task
        # completion validates the pose Nav2 itself is using.
        self._refresh_robot_pose_from_tf()
        readiness = self._navigation_availability_snapshot()
        with self._lock:
            snapshot = dict(self._navigation_state)
            manual_control_ready = self._manual_control_ready_locked()
            snapshot.update(
                {
                    **readiness,
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
                }
            )
        return snapshot

    def load_map(self, map_url: str, *, timeout_sec: float = 20.0) -> None:
        """Request the Nav2 ``map_server`` to serve a different occupancy map.

        Scene switching swaps the map while the robot and map_server stay
        alive, so the change is runtime-only (mirrors the synchronous
        service-call pattern used by :meth:`reset_cabinet_controls`).
        """
        if not isinstance(map_url, str) or not map_url.strip():
            raise ControlRequestError(
                "map_url must be a non-empty string.",
                400,
            )
        map_client = getattr(self, "_map_load_client", None)
        if not self._service_ready(map_client):
            raise ControlRequestError(
                "Nav2 map_server LoadMap service is unavailable.",
                503,
            )
        request = LoadMap.Request()
        request.map_url = map_url.strip()
        try:
            future = map_client.call_async(request)
        except Exception as error:  # noqa: BLE001
            raise ControlRequestError(
                f"Failed to request map load: {error}",
                503,
            ) from error
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=max(0.1, float(timeout_sec))):
            raise ControlRequestError("Nav2 map load request timed out.", 503)
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            raise ControlRequestError(
                f"Nav2 map load failed: {error}",
                503,
            ) from error
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            raise ControlRequestError(
                f"Nav2 map load was rejected (result {response.result}).",
                409,
            )

    def publish_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        frame_id: str = "map",
    ) -> None:
        """Publish an AMCL initial-pose hypothesis on ``/initialpose``.

        The covariance encodes a modest uncertainty so AMCL relocalises
        around the teleported pose rather than trusting it exactly.
        """
        x_value = self._validated_station_number(x, "x", status=400)
        y_value = self._validated_station_number(y, "y", status=400)
        yaw_value = self._validated_station_number(yaw, "yaw", status=400)
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ControlRequestError(
                "frame_id must be a non-empty string.",
                400,
            )
        message = PoseWithCovarianceStamped()
        message.header.frame_id = frame_id.strip()
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x_value
        message.pose.pose.position.y = y_value
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(yaw_value / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw_value / 2.0)
        covariance = [0.0] * 36
        covariance[0] = 0.25  # x variance (0.5 m std)
        covariance[7] = 0.25  # y variance (0.5 m std)
        covariance[35] = 0.06853891945200942  # yaw variance (~0.26 rad std)
        message.pose.covariance = covariance
        self._initial_pose_publisher.publish(message)

    def navigation_station_from_tf(
        self,
        cabinet: str,
        cabinet_frame: str,
        station_spec: NavigationStationSpec,
    ) -> NavigationStation:
        """Transform cabinet-local station geometry into the Nav2 frame.

        Inventory poses describe the intended simulation layout, but Nav2
        plans in the localized navigation frame.  Resolving the latest
        navigation-frame-to-cabinet TF here includes any current map/odom
        correction instead of silently treating inventory coordinates as map
        coordinates.
        """
        if not isinstance(cabinet, str) or not cabinet.strip():
            raise ControlRequestError(
                "cabinet must be a non-empty string.",
                400,
            )
        if (
            not isinstance(cabinet_frame, str)
            or not cabinet_frame.strip()
            or any(character.isspace() for character in cabinet_frame)
        ):
            raise ControlRequestError(
                "cabinet_frame must be a non-empty ROS frame without "
                "whitespace.",
                400,
            )
        if not isinstance(station_spec, NavigationStationSpec):
            raise ControlRequestError(
                "station_spec must be a NavigationStationSpec.",
                400,
            )
        if station_spec.frame_id != self._navigation_frame:
            raise ControlRequestError(
                "Navigation station frame must match the configured "
                f"navigation frame {self._navigation_frame}.",
                400,
            )
        pose_validity = getattr(self, "_cabinet_pose_validity", {})
        if cabinet in pose_validity:
            with self._lock:
                pose_valid = self._cabinet_pose_validity.get(cabinet)
            if pose_valid is not True:
                state = "invalid" if pose_valid is False else "not confirmed"
                raise ControlRequestError(
                    f"Live pose for {cabinet} is {state}; refusing to use a "
                    "possibly stale TF transform.",
                    503,
                )

        anchor = self._validated_station_vector(
            station_spec.local_anchor,
            "station_spec.local_anchor",
        )
        outward_axis = self._validated_station_vector(
            station_spec.outward_axis,
            "station_spec.outward_axis",
        )
        standoff = self._validated_station_number(
            station_spec.standoff,
            "station_spec.standoff",
            status=400,
        )
        yaw_offset = self._validated_station_number(
            station_spec.base_yaw_offset,
            "station_spec.base_yaw_offset",
            status=400,
        )
        if standoff <= 0.0:
            raise ControlRequestError(
                "station_spec.standoff must be positive.",
                400,
            )
        axis_norm = math.hypot(*outward_axis)
        if axis_norm <= 1.0e-12:
            raise ControlRequestError(
                "station_spec.outward_axis must not be zero.",
                400,
            )
        outward_axis = tuple(
            component / axis_norm for component in outward_axis
        )

        tf_buffer = getattr(self, "_tf_buffer", None)
        if tf_buffer is None:
            raise ControlRequestError(
                f"Live transform for {cabinet_frame} is not available.",
                503,
            )
        try:
            transform = tf_buffer.lookup_transform(
                self._navigation_frame,
                cabinet_frame,
                RosTime(),
            )
        except TransformException as error:
            raise ControlRequestError(
                f"Live transform from {cabinet_frame} to "
                f"{self._navigation_frame} is not available: {error}",
                503,
            ) from error

        try:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            translation_values = tuple(
                self._validated_station_number(
                    getattr(translation, field),
                    f"live transform translation.{field}",
                    status=503,
                )
                for field in ("x", "y", "z")
            )
            quaternion = tuple(
                self._validated_station_number(
                    getattr(rotation, field),
                    f"live transform rotation.{field}",
                    status=503,
                )
                for field in ("x", "y", "z", "w")
            )
        except (AttributeError, TypeError) as error:
            raise ControlRequestError(
                f"Live transform for {cabinet_frame} is malformed.",
                503,
            ) from error

        quaternion_norm = math.hypot(*quaternion)
        if (
            quaternion_norm <= 1.0e-12
            or abs(quaternion_norm - 1.0) > 5.0e-4
        ):
            raise ControlRequestError(
                f"Live transform for {cabinet_frame} has an invalid "
                "rotation quaternion.",
                503,
            )
        unit_quaternion = tuple(
            value / quaternion_norm for value in quaternion
        )
        local_position = tuple(
            anchor[index] + outward_axis[index] * standoff
            for index in range(3)
        )
        rotated_position = self._rotate_vector_by_unit_quaternion(
            local_position,
            unit_quaternion,
        )
        navigation_axis = self._rotate_vector_by_unit_quaternion(
            outward_axis,
            unit_quaternion,
        )
        horizontal_norm = math.hypot(
            navigation_axis[0],
            navigation_axis[1],
        )
        if not math.isfinite(horizontal_norm) or horizontal_norm <= 1.0e-9:
            raise ControlRequestError(
                f"Navigation outward axis for {cabinet} has no finite "
                "horizontal component.",
                400,
            )
        unnormalized_yaw = (
            math.atan2(-navigation_axis[1], -navigation_axis[0])
            + yaw_offset
        )
        yaw = math.atan2(
            math.sin(unnormalized_yaw),
            math.cos(unnormalized_yaw),
        )
        position = tuple(
            translation_values[index] + rotated_position[index]
            for index in range(3)
        )
        if not all(math.isfinite(value) for value in (*position, yaw)):
            raise ControlRequestError(
                f"Live navigation station for {cabinet} is not finite.",
                503,
            )
        return NavigationStation(
            cabinet=cabinet,
            frame_id=self._navigation_frame,
            x=position[0],
            y=position[1],
            z=position[2],
            yaw=yaw,
        )

    def _refresh_robot_pose_from_tf(self) -> None:
        """Refresh the localized SE(2) pose from the latest TF transform."""
        tf_buffer = getattr(self, "_tf_buffer", None)
        if tf_buffer is None:
            return
        try:
            transform = tf_buffer.lookup_transform(
                self._navigation_frame,
                self._navigation_base_frame,
                RosTime(),
            )
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        timing = self._localized_pose_timing(transform.header.stamp)
        with self._lock:
            self._robot_pose_sequence += 1
            self._robot_pose = {
                "x": float(translation.x),
                "y": float(translation.y),
                "yaw": self._quaternion_yaw(rotation),
                "frame_id": str(transform.header.frame_id),
                "stamp": {
                    "sec": int(transform.header.stamp.sec),
                    "nanosec": int(transform.header.stamp.nanosec),
                },
                **timing,
                "received_at": time.time(),
                "received_monotonic": time.monotonic(),
                "sequence": self._robot_pose_sequence,
                "source": "tf",
                "child_frame_id": str(transform.child_frame_id),
            }

    def _localized_pose_timing(self, stamp: Any) -> Dict[str, Any]:
        """Describe a localization stamp using this node's ROS clock.

        Wall and monotonic time cannot be compared with timestamps produced
        under ``use_sim_time``.  Keep both the source timestamp and the ROS
        time at which it was observed so terminal navigation checks can
        identify an old transform even when it was just read from TF cache.
        """
        stamp_ros_nanoseconds = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        )
        observed_ros_nanoseconds = self._ros_clock_now_nanoseconds()
        stamp_age_seconds = (
            observed_ros_nanoseconds - stamp_ros_nanoseconds
        ) / 1_000_000_000.0
        return {
            "stamp_ros_nanoseconds": stamp_ros_nanoseconds,
            "observed_ros_nanoseconds": observed_ros_nanoseconds,
            "stamp_age_seconds": stamp_age_seconds,
        }

    def _ros_clock_now_nanoseconds(self) -> int:
        """Return the active ROS clock, including Gazebo simulation time."""
        return int(self.get_clock().now().nanoseconds)

    def map_snapshot(self) -> Dict[str, Any]:
        """Return the latest transient-local occupancy map."""
        with self._lock:
            if self._map_state is None:
                raise ControlRequestError(
                    str(
                        getattr(
                            self,
                            "_map_error",
                            "Nav2 occupancy map is not available.",
                        )
                    ),
                    503,
                )
            return {
                **self._map_state,
                "data": list(self._map_state["data"]),
            }

    def cabinet_snapshot(self) -> Dict[str, Any]:
        """Return cabinet action availability, feedback and control state."""
        with self._lock:
            snapshot = dict(self._cabinet_state)
            selected_control_id = str(
                snapshot.get("control_id")
                or snapshot.get("button_id")
                or ""
            )
            control_state = self._cabinet_control_states.get(
                selected_control_id
            )
            if control_state is not None:
                snapshot.update(
                    {
                        key: value
                        for key, value in control_state.items()
                        if value is not None
                        and not (key == "current_state" and value == "")
                    }
                )
            operation_available = self._action_server_ready(
                getattr(self, "_cabinet_operation_client", None)
            )
            legacy_button_available = self._action_server_ready(
                getattr(self, "_cabinet_button_client", None)
            )
            catalog_received = self._cabinet_catalog_received
            snapshot.update(
                {
                    "available": (
                        catalog_received
                        and (operation_available or legacy_button_available)
                    ),
                    "operation_available": operation_available,
                    "legacy_button_available": legacy_button_available,
                    "catalog_received": catalog_received,
                    "reset_available": self._service_ready(
                        getattr(self, "_cabinet_reset_client", None)
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
            selected_control_id = self._selected_cabinet_control_id_locked()
            if selected_control_id not in self._cabinet_controls:
                selected_control_id = next(
                    (
                        control_id
                        for control_id, control in self._cabinet_controls.items()
                        if bool(control.get("operable", True))
                    ),
                    next(iter(self._cabinet_controls), ""),
                )
            catalog_received = self._cabinet_catalog_received
            operation_available = self._action_server_ready(
                getattr(self, "_cabinet_operation_client", None)
            )
            legacy_button_available = self._action_server_ready(
                getattr(self, "_cabinet_button_client", None)
            )
        return {
            "available": (
                catalog_received
                and (operation_available or legacy_button_available)
            ),
            "operation_available": operation_available,
            "legacy_button_available": legacy_button_available,
            "reset_available": self._service_ready(
                getattr(self, "_cabinet_reset_client", None)
            ),
            "catalog_received": catalog_received,
            "source": (
                "operator_catalog"
                if catalog_received
                else "waiting_for_catalog"
            ),
            "selected_control_id": selected_control_id,
            "controls": controls,
        }

    def operate_cabinet_control(
        self,
        control_id: str,
        command: Any,
        target_state: Optional[str] = None,
        target_position: Optional[float] = None,
        navigate_to_staging_pose: bool = True,
        force: float = 5.0,
    ) -> Dict[str, Any]:
        """Submit one generic, physically verified cabinet operation."""
        if not isinstance(control_id, str) or not control_id.strip():
            raise ControlRequestError(
                "control_id must be a non-empty string."
            )
        control_id = control_id.strip()
        command_code, command_name = self._normalize_cabinet_command(command)
        if not isinstance(navigate_to_staging_pose, bool):
            raise ControlRequestError(
                "navigate_to_staging_pose must be a boolean."
            )
        if isinstance(force, bool):
            raise ControlRequestError("force must be a positive finite number.")
        try:
            force = float(force)
        except (TypeError, ValueError) as error:
            raise ControlRequestError(
                "force must be a positive finite number."
            ) from error
        if not math.isfinite(force) or force <= 0.0:
            raise ControlRequestError("force must be a positive finite number.")
        if target_state is not None:
            if not isinstance(target_state, str) or not target_state.strip():
                raise ControlRequestError(
                    "target_state must be a non-empty string when provided."
                )
            target_state = target_state.strip()
        if target_position is not None:
            if isinstance(target_position, bool):
                raise ControlRequestError("target_position must be a number.")
            try:
                target_position = float(target_position)
            except (TypeError, ValueError) as error:
                raise ControlRequestError(
                    "target_position must be a number."
                ) from error
            if not math.isfinite(target_position):
                raise ControlRequestError("target_position must be finite.")

        with self._lock:
            control = self._validate_cabinet_operation_locked(
                control_id,
                command_code,
                target_state,
                target_position,
            )
            self._reject_cabinet_start_conflicts_locked()
        operation_client = getattr(
            self,
            "_cabinet_operation_client",
            None,
        )
        if not self._action_server_ready(operation_client):
            raise ControlRequestError(
                "Generic cabinet operation action server is unavailable.",
                503,
            )

        with self._lock:
            control = self._validate_cabinet_operation_locked(
                control_id,
                command_code,
                target_state,
                target_position,
            )
            self._reject_cabinet_start_conflicts_locked()
            self._cabinet_generation += 1
            generation = self._cabinet_generation
            self._cabinet_cancel_requested = False
            self._cabinet_goal_handle = None
            self._cabinet_goal_sent_monotonic = time.monotonic()
            self._cabinet_terminal_event.clear()
            physical_state = self._cabinet_control_states.get(control_id, {})
            current_position = physical_state.get("current_position")
            self._cabinet_state.update(
                {
                    "state": "sending",
                    "message": "Sending the cabinet operation goal.",
                    "control_id": control_id,
                    "control_type": int(control["control_type"]),
                    "type": int(control["control_type"]),
                    "command": command_name,
                    "command_code": command_code,
                    "target_state": target_state,
                    "target_position": target_position,
                    "target": {
                        "state": target_state,
                        "position": target_position,
                    },
                    "action_interface": "generic",
                    # Compatibility aliases used by existing clients.
                    "button_id": control_id,
                    "navigate_to_staging_pose": navigate_to_staging_pose,
                    "phase": None,
                    "progress": 0.0,
                    "max_travel": 0.0,
                    "initial_position": current_position,
                    "final_position": None,
                    "peak_position": current_position,
                    "final_state": None,
                    "success": None,
                    "error_code": None,
                    "requested_force": force,
                    "estimated_force": None,
                    "button_triggered": None,
                    "updated_at": time.time(),
                }
            )
            self._prepare_for_cabinet_operation_locked()

        goal = OperateCabinetControl.Goal()
        goal.control_id = control_id
        goal.command = command_code
        goal.target_state = target_state or ""
        goal.use_target_position = target_position is not None
        goal.target_position = (
            target_position if target_position is not None else 0.0
        )
        goal.force = force
        goal.navigate_to_staging_pose = navigate_to_staging_pose
        try:
            future = operation_client.send_goal_async(
                goal,
                feedback_callback=(
                    lambda feedback_message: (
                        self._cabinet_operation_feedback_callback(
                            feedback_message,
                            generation,
                        )
                    )
                ),
            )
            future.add_done_callback(
                lambda goal_future: self._cabinet_goal_response_callback(
                    goal_future,
                    generation,
                    generic=True,
                )
            )
        except Exception as error:  # noqa: BLE001
            self._set_cabinet_terminal(
                "failed",
                f"Failed to send cabinet operation goal: {error}",
                False,
                None,
                generation=generation,
            )
            raise ControlRequestError(
                f"Failed to send cabinet operation goal: {error}",
                503,
            ) from error

        return {
            "status": "accepted",
            "control_id": control_id,
            "command": command_name,
            "target_state": target_state,
            "target_position": target_position,
            "force": force,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        """Submit a button press through the generic API, with legacy fallback."""
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
        if self._action_server_ready(
            getattr(self, "_cabinet_operation_client", None)
        ):
            result = self.operate_cabinet_control(
                button_id,
                "press",
                navigate_to_staging_pose=navigate_to_staging_pose,
            )
            return {**result, "button_id": button_id}

        return self._press_cabinet_button_legacy(
            button_id,
            navigate_to_staging_pose,
        )

    def _press_cabinet_button_legacy(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        """Use the retained PressCabinetButton action for verified buttons."""
        with self._lock:
            self._validate_cabinet_button_locked(button_id)
            control = self._cabinet_controls[button_id]
            if not bool(control.get("operable", True)):
                raise ControlRequestError(
                    "The generic cabinet operation action is required for "
                    "live planning validation of an unverified button, but "
                    "that backend is unavailable. The deprecated legacy "
                    "PressCabinetButton action only supports verified "
                    "operable buttons.",
                    503,
                )
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
            self._cabinet_goal_sent_monotonic = time.monotonic()
            self._cabinet_terminal_event.clear()
            self._cabinet_state.update(
                {
                    "state": "sending",
                    "message": "Sending the cabinet button goal.",
                    "control_id": button_id,
                    "control_type": self.CABINET_CONTROL_TYPE_BUTTON,
                    "type": self.CABINET_CONTROL_TYPE_BUTTON,
                    "command": "press",
                    "command_code": (
                        OperateCabinetControl.Goal.COMMAND_PRESS
                    ),
                    "target_state": None,
                    "target_position": None,
                    "target": {"state": None, "position": None},
                    "action_interface": "legacy",
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

            self._prepare_for_cabinet_operation_locked()

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
            "control_id": button_id,
            "command": "press",
            "button_id": button_id,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }

    def reset_cabinet_controls(self, timeout_sec: float = 5.0) -> Dict[str, Any]:
        """Reset all simulated cabinet controls while the operator is idle."""
        with self._lock:
            self._reject_cabinet_start_conflicts_locked()
        reset_client = getattr(self, "_cabinet_reset_client", None)
        if not self._service_ready(reset_client):
            raise ControlRequestError(
                "Cabinet reset service is unavailable.",
                503,
            )

        try:
            future = reset_client.call_async(Trigger.Request())
        except Exception as error:  # noqa: BLE001
            raise ControlRequestError(
                f"Failed to request cabinet reset: {error}",
                503,
            ) from error
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=max(0.1, float(timeout_sec))):
            raise ControlRequestError("Cabinet reset request timed out.", 503)
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            raise ControlRequestError(
                f"Cabinet reset failed: {error}",
                503,
            ) from error
        if not bool(response.success):
            raise ControlRequestError(
                str(response.message) or "Cabinet reset was rejected.",
                409,
            )
        with self._lock:
            self._cabinet_state.update(
                {
                    "state": "idle",
                    "message": str(response.message) or "Cabinet controls reset.",
                    "command": None,
                    "command_code": None,
                    "target_state": None,
                    "target_position": None,
                    "target": {"state": None, "position": None},
                    "phase": None,
                    "progress": 0.0,
                    "success": None,
                    "error_code": None,
                    "updated_at": time.time(),
                }
            )
        return {
            "status": "reset",
            "message": str(response.message) or "Cabinet controls reset.",
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

    def cancel_cabinet_operation(
        self,
        allow_idle: bool = False,
    ) -> Dict[str, Any]:
        """Generic name for canceling either cabinet action interface."""
        return self.cancel_cabinet_button(allow_idle=allow_idle)

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
            if not self._service_ready(self._navigation_mode_client):
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
            service_ready = self._service_ready(self._navigation_mode_client)
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
        with self._lock:
            if not service_ready:
                # Nav2 不在时直接标记为手动模式。内部已将所有导航目标
                # 失效并清零速度，前端拿到 mode=false 即可正常发 cmd_vel。
                self._navigation_mode = False
                self._navigation_mode_acknowledged = False
                self._manual_takeover_generation = None
                self._navigation_mode_desired = None
                self._navigation_mode_request = None
                status = "manual"
            else:
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
        readiness = self._navigation_availability_snapshot()
        if not readiness["action_server_available"]:
            raise ControlRequestError(
                "Nav2 NavigateToPose action server is unavailable.",
                503,
            )
        if not readiness["lifecycle_active"]:
            raise ControlRequestError(
                "Nav2 navigation stack is not active yet.",
                503,
                {
                    "navigation_lifecycle_state": readiness[
                        "lifecycle_state"
                    ],
                    "navigation_lifecycle_message": readiness[
                        "lifecycle_message"
                    ],
                },
            )
        if not self._service_ready(self._navigation_mode_client):
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
                    "goal_sent_ros_nanoseconds": None,
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
                trajectory_duration_sec = float(
                    getattr(
                        self,
                        "_pending_trajectory_duration_sec",
                        self.DEFAULT_MANUAL_TRAJECTORY_DURATION_SECONDS,
                    )
                )
                if self._pending_trajectory_repeats > 0:
                    self._pending_trajectory_repeats -= 1
                else:
                    trajectory = None
                    self._pending_trajectory = None
                    self._pending_trajectory_duration_sec = 0.0

                # Publish while holding the state lock so a cabinet goal
                # cannot become active between the active check and send.
                command = Twist()
                if self._manual_linear_axis == "x":
                    command.linear.x = linear_y
                else:
                    command.linear.y = linear_y
                command.angular.z = angular_z
                self._cmd_vel_publisher.publish(command)
                if trajectory is not None:
                    self._trajectory_publisher.publish(trajectory)
                    self._manual_trajectory_active_until = max(
                        float(
                            getattr(
                                self,
                                "_manual_trajectory_active_until",
                                0.0,
                            )
                        ),
                        now
                        + trajectory_duration_sec
                        + self.MANUAL_TRAJECTORY_SETTLE_MARGIN_SECONDS,
                    )

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
        self._navigation_goal_sent_monotonic = time.monotonic()
        self._navigation_state.update(
            {
                "state": "sending",
                "message": "Sending the Nav2 goal.",
                "updated_at": time.time(),
            }
        )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._navigation_frame
        goal_stamp = self.get_clock().now()
        goal.pose.header.stamp = goal_stamp.to_msg()
        self._navigation_state["goal_sent_ros_nanoseconds"] = int(
            goal_stamp.nanoseconds
        )
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
                self._navigation_goal_sent_monotonic = None
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
        self._navigation_goal_sent_monotonic = None
        self._navigation_cancel_requested = False
        self._navigation_state.update(
            {
                "state": state,
                "message": message,
                "updated_at": time.time(),
            }
        )

    def _cabinet_goal_response_callback(
        self,
        future: Any,
        generation: int,
        generic: bool = False,
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
                    "Cabinet operation was canceled before "
                    "goal acceptance."
                    if canceled_before_acceptance
                    else f"Failed to send cabinet operation goal: {error}"
                ),
                False,
                (
                    (
                        OperateCabinetControl.Result.CANCELED
                        if generic
                        else PressCabinetButton.Result.CANCELED
                    )
                    if canceled_before_acceptance
                    else None
                ),
                generation=generation,
            )
            return

        if not goal_handle.accepted:
            self._set_cabinet_terminal(
                "rejected",
                "Cabinet operation action server rejected the goal.",
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
            self._cabinet_goal_sent_monotonic = None
            self._cabinet_state.update(
                {
                    "state": (
                        "canceling"
                        if canceled_before_acceptance
                        else "operating"
                    ),
                    "message": (
                        "Canceling the accepted cabinet operation goal."
                        if canceled_before_acceptance
                        else "Cabinet operation is running."
                    ),
                    "updated_at": time.time(),
                }
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:  # noqa: BLE001
            # Mirror the navigation path: if the result channel cannot be
            # opened (goal handle invalidated after acceptance), route to a
            # terminal state so the cabinet returns to idle instead of being
            # left wedged in 'operating' with the terminal event never set.
            self._set_cabinet_terminal(
                "failed",
                f"Cabinet result channel failed: {error}",
                False,
                None,
                generation=generation,
                expected_goal_handle=goal_handle,
            )
            return
        result_future.add_done_callback(
            (
                lambda result: self._cabinet_operation_result_callback(
                    result,
                    generation,
                    goal_handle,
                )
            )
            if generic
            else (
                lambda result: self._cabinet_result_callback(
                    result,
                    generation,
                    goal_handle,
                )
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

    def _cabinet_operation_feedback_callback(
        self,
        feedback_message: Any,
        generation: int,
    ) -> None:
        """Store generic operation feedback without crossing generations."""
        feedback = feedback_message.feedback
        current_position = float(feedback.current_position)
        target_position = float(feedback.target_position)
        if not math.isfinite(current_position):
            current_position = None
        if not math.isfinite(target_position):
            target_position = None
        with self._lock:
            if (
                generation != self._cabinet_generation
                or not self._cabinet_active_locked()
            ):
                return
            previous_peak = self._cabinet_state.get("peak_position")
            peak_position = current_position
            if current_position is not None and previous_peak is not None:
                peak_position = max(
                    float(previous_peak),
                    current_position,
                )
            update = {
                "phase": int(feedback.phase),
                "progress": float(feedback.progress),
                "current_position": current_position,
                "current_state": str(feedback.current_state),
                "target_position": target_position,
                "peak_position": peak_position,
                "updated_at": time.time(),
            }
            if (
                self._cabinet_state.get("control_type")
                == self.CABINET_CONTROL_TYPE_BUTTON
                and current_position is not None
            ):
                previous_max = self._cabinet_state.get("max_travel")
                update["max_travel"] = max(
                    float(previous_max)
                    if previous_max is not None
                    else 0.0,
                    max(0.0, current_position),
                )
            if self._cabinet_state["state"] != "canceling":
                update["message"] = str(feedback.message)
            self._cabinet_state.update(update)

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

    def _cabinet_operation_result_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
    ) -> None:
        """Finish a generic cabinet goal from its physical result."""
        result_updates: Dict[str, Any] = {}
        max_travel: Optional[float] = None
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            action_status = int(wrapped_result.status)
            error_code = int(result.error_code)
            result_message = str(result.message)
            result_updates = {
                "initial_position": float(result.initial_position),
                "final_position": float(result.final_position),
                "peak_position": float(result.peak_position),
                "final_state": str(result.final_state),
                "current_position": float(result.final_position),
                "current_state": str(result.final_state),
                "requested_force": float(result.requested_force),
                "estimated_force": float(result.estimated_force),
                "button_triggered": bool(result.button_triggered),
            }
            with self._lock:
                if (
                    self._cabinet_state.get("control_type")
                    == self.CABINET_CONTROL_TYPE_BUTTON
                ):
                    max_travel = max(0.0, float(result.peak_position))

            if (
                action_status == GoalStatus.STATUS_CANCELED
                or error_code == OperateCabinetControl.Result.CANCELED
            ):
                state = "canceled"
                success = False
                message = result_message or "Cabinet operation was canceled."
            elif (
                action_status == GoalStatus.STATUS_SUCCEEDED
                and bool(result.success)
                and error_code == OperateCabinetControl.Result.SUCCESS
            ):
                state = "succeeded"
                success = True
                message = result_message or "Cabinet operation succeeded."
            else:
                state = "failed"
                success = False
                _, fallback_message = self._goal_status(
                    action_status,
                    "Cabinet operation",
                )
                message = result_message or fallback_message
        except Exception as error:  # noqa: BLE001
            state = "failed"
            success = False
            message = f"Cabinet operation result failed: {error}"
            error_code = None
            result_updates = {}

        self._set_cabinet_terminal(
            state,
            message,
            success,
            error_code,
            max_travel,
            generation=generation,
            expected_goal_handle=goal_handle,
            result_updates=result_updates,
        )

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
        result_updates: Optional[Dict[str, Any]] = None,
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
            self._cabinet_goal_sent_monotonic = None
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
            if result_updates:
                self._cabinet_state.update(result_updates)
            self._cabinet_terminal_event.set()

    def _navigation_mode_callback(self, message: Bool) -> None:
        with self._lock:
            # This topic is observation only.  Transaction progress is driven
            # by the matching SetBool response so a delayed retained sample
            # can never acknowledge a newer command.
            self._navigation_mode = bool(message.data)

    @staticmethod
    def _new_cabinet_control_state() -> Dict[str, Any]:
        return {
            "control_type": None,
            "type": None,
            "valid": None,
            "current_position": None,
            "current_state": "",
            "velocity": None,
            "effort": None,
            "normalized_position": None,
            "activated": None,
            "in_motion": None,
            "transition_sequence": None,
            "state_updated_at": None,
            # Compatibility aliases retained for the original button UI/API.
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
            state_topic = str(getattr(entry, "state_topic", "")).strip()
            control_type = int(entry.control_type)
            operable = bool(getattr(entry, "operable", True))
            if (
                control_type not in self.CABINET_CONTROL_TYPES
                or (
                    operable
                    and not state_topic
                    and not (joint_name and joint_state_topic)
                )
            ):
                continue
            min_position = float(getattr(entry, "min_position", 0.0))
            max_position = float(getattr(entry, "max_position", 0.0))
            if (
                not math.isfinite(min_position)
                or not math.isfinite(max_position)
                or min_position > max_position
            ):
                continue
            state_ids = [str(value) for value in getattr(entry, "state_ids", [])]
            state_labels = [
                str(value) for value in getattr(entry, "state_labels", [])
            ]
            state_positions = [
                float(value)
                for value in getattr(entry, "state_positions", [])
            ]
            if (
                len(state_labels) not in {0, len(state_ids)}
                or len(state_positions) not in {0, len(state_ids)}
                or not all(math.isfinite(value) for value in state_positions)
            ):
                continue
            supported_commands = int(
                getattr(entry, "supported_commands", 0)
            )
            if (
                supported_commands == 0
                and control_type == self.CABINET_CONTROL_TYPE_BUTTON
            ):
                supported_commands = self.CABINET_COMMAND_SUPPORT[
                    OperateCabinetControl.Goal.COMMAND_PRESS
                ]
            controls[control_id] = {
                "control_id": control_id,
                "display_name": (
                    str(entry.display_name).strip() or control_id
                ),
                "control_type": control_type,
                "joint_name": joint_name,
                "joint_state_topic": joint_state_topic,
                "pressed_topic": pressed_topic,
                "state_topic": state_topic,
                "supported_commands": supported_commands,
                "unit": str(getattr(entry, "unit", "")),
                "min_position": min_position,
                "max_position": max_position,
                "state_ids": state_ids,
                "state_labels": state_labels,
                "state_positions": state_positions,
                "requires_grasp": bool(
                    getattr(entry, "requires_grasp", False)
                ),
                "operable": operable,
                "unavailable_reason": str(
                    getattr(entry, "unavailable_reason", "")
                ),
                "default_force": float(
                    getattr(entry, "default_force", 0.0)
                ),
                "min_trigger_force": float(
                    getattr(entry, "min_trigger_force", 0.0)
                ),
                "max_force": float(getattr(entry, "max_force", 0.0)),
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
            for control_id, control in controls.items():
                control_type = int(control["control_type"])
                self._cabinet_control_states[control_id].update(
                    {
                        "control_type": control_type,
                        "type": control_type,
                    }
                )
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
    ) -> Optional[Tuple[str, str, str, str, int]]:
        if control is None:
            return None
        return (
            str(control.get("joint_name", "")),
            str(control.get("joint_state_topic", "")),
            str(control.get("pressed_topic", "")),
            str(control.get("state_topic", "")),
            int(control.get("control_type", -1)),
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
        for subscription_name in ("state", "joint", "pressed"):
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
        state_topic = str(control.get("state_topic", ""))
        control_type = int(control.get("control_type", -1))

        signature = (
            joint_name,
            joint_state_topic,
            pressed_topic,
            state_topic,
            control_type,
        )
        existing = self._cabinet_control_subscriptions.get(control_id)
        if existing is not None and existing.get("signature") == signature:
            return

        subscriptions: Dict[str, Any] = {"signature": signature}
        try:
            if state_topic:
                # The aggregate state contains every field consumed below.
                # Avoid also consuming its high-frequency compatibility
                # mirrors; with dozens of controls per cabinet those duplicate
                # subscriptions can monopolize the Python executor.
                subscriptions["state"] = self.create_subscription(
                    CabinetControlState,
                    state_topic,
                    lambda message, selected_id=control_id, selected_topic=(
                        state_topic
                    ): self._cabinet_control_state_callback(
                        selected_id,
                        selected_topic,
                        message,
                    ),
                    transient_qos,
                )
            else:
                if joint_name and joint_state_topic:
                    subscriptions["joint"] = self.create_subscription(
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
                        qos_profile_sensor_data,
                    )
                if pressed_topic:
                    subscriptions["pressed"] = self.create_subscription(
                        Bool,
                        pressed_topic,
                        lambda message, selected_id=control_id, selected_topic=(
                            pressed_topic
                        ): self._cabinet_button_pressed_callback(
                            selected_id,
                            selected_topic,
                            message,
                        ),
                        transient_qos,
                    )
        except Exception:
            for name in ("state", "joint", "pressed"):
                subscription = subscriptions.get(name)
                if subscription is not None:
                    self.destroy_subscription(subscription)
            raise

        self._remove_cabinet_control_subscriptions(control_id)
        self._cabinet_control_subscriptions[control_id] = subscriptions

    def _cabinet_control_state_callback(
        self,
        control_id: str,
        state_topic: str,
        message: CabinetControlState,
    ) -> None:
        """Store the authoritative generic state for one catalog control."""
        message_control_id = str(message.control_id).strip()
        if message_control_id and message_control_id != control_id:
            return
        position = float(message.position)
        velocity = float(message.velocity)
        effort = float(message.effort)
        normalized_position = float(message.normalized_position)
        if not all(
            math.isfinite(value)
            for value in (position, velocity, effort, normalized_position)
        ):
            return
        with self._lock:
            control = self._cabinet_controls.get(control_id)
            if (
                control is None
                or control.get("state_topic") != state_topic
                or int(message.control_type) != int(control["control_type"])
            ):
                return
            state = self._cabinet_control_states.setdefault(
                control_id,
                self._new_cabinet_control_state(),
            )
            now = time.time()
            state.update(
                {
                    "control_type": int(message.control_type),
                    "type": int(message.control_type),
                    "valid": bool(message.valid),
                    "current_position": position,
                    "current_state": str(message.state_id),
                    "velocity": velocity,
                    "effort": effort,
                    "normalized_position": normalized_position,
                    "activated": bool(message.activated),
                    "in_motion": bool(message.in_motion),
                    "transition_sequence": int(
                        message.transition_sequence
                    ),
                    "state_updated_at": now,
                }
            )
            if int(message.control_type) == self.CABINET_CONTROL_TYPE_BUTTON:
                state.update(
                    {
                        "button_pressed": bool(message.activated),
                        "button_travel": max(0.0, position),
                        "button_state_updated_at": now,
                    }
                )
            if self._selected_cabinet_control_id_locked() == control_id:
                self._cabinet_state.update(state)

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
            now = time.time()
            state.update(
                {
                    "current_position": travel,
                    "state_updated_at": now,
                    "button_travel": max(0.0, travel),
                    "button_state_updated_at": now,
                }
            )
            if self._selected_cabinet_control_id_locked() == control_id:
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
            now = time.time()
            state.update(
                {
                    "activated": bool(message.data),
                    "button_pressed": bool(message.data),
                    "state_updated_at": now,
                    "button_state_updated_at": now,
                }
            )
            if self._selected_cabinet_control_id_locked() == control_id:
                self._cabinet_state.update(state)

    def _map_callback(self, message: OccupancyGrid) -> None:
        try:
            frame_id = str(message.header.frame_id).strip()
            resolution = float(message.info.resolution)
            width = int(message.info.width)
            height = int(message.info.height)
            origin_x = float(message.info.origin.position.x)
            origin_y = float(message.info.origin.position.y)
            orientation = message.info.origin.orientation
            quaternion = tuple(
                float(getattr(orientation, field))
                for field in ("x", "y", "z", "w")
            )
            data = [int(value) for value in message.data]
        except (AttributeError, TypeError, ValueError) as error:
            map_error = f"Nav2 occupancy map metadata is malformed: {error}"
            with self._lock:
                self._map_state = None
                self._map_error = map_error
            return

        quaternion_norm = math.hypot(*quaternion)
        if not frame_id:
            map_error = "Nav2 occupancy map frame_id is empty."
        elif frame_id != self._navigation_frame:
            map_error = (
                "Nav2 occupancy map frame does not match the navigation frame; "
                f"expected {self._navigation_frame}, received {frame_id}."
            )
        elif (
            not math.isfinite(resolution)
            or resolution <= 0.0
            or width <= 0
            or height <= 0
            or not math.isfinite(origin_x)
            or not math.isfinite(origin_y)
            or not all(math.isfinite(value) for value in quaternion)
            or quaternion_norm <= 1.0e-12
        ):
            map_error = "Nav2 occupancy map geometry is invalid."
        elif len(data) != width * height:
            map_error = (
                "Nav2 occupancy map data length does not match width * height."
            )
        elif any(value < -1 or value > 100 for value in data):
            map_error = "Nav2 occupancy map contains an invalid occupancy value."
        else:
            map_error = None

        if map_error is not None:
            with self._lock:
                self._map_state = None
                self._map_error = map_error
            return

        unit_x, unit_y, unit_z, unit_w = (
            value / quaternion_norm for value in quaternion
        )
        origin_yaw = math.atan2(
            2.0 * (unit_w * unit_z + unit_x * unit_y),
            1.0 - 2.0 * (unit_y * unit_y + unit_z * unit_z),
        )
        with self._lock:
            self._map_state = {
                "frame_id": frame_id,
                "resolution": resolution,
                "width": width,
                "height": height,
                "origin": {
                    "x": origin_x,
                    "y": origin_y,
                    "yaw": origin_yaw,
                },
                "data": data,
            }
            self._map_error = None

    def _cabinet_pose_validity_callback(
        self,
        cabinet: str,
        message: Bool,
    ) -> None:
        """Track the authority signal that invalidates cached cabinet TF."""
        with self._lock:
            if cabinet in self._cabinet_pose_validity:
                self._cabinet_pose_validity[cabinet] = bool(message.data)

    def _robot_joint_state_callback(self, message: JointState) -> None:
        """Store only finite positions for the configured manual joints."""
        required_names = tuple(joint.name for joint in self._manual_joints)
        required_set = set(required_names)
        positions: Dict[str, float] = {}
        seen = set()
        invalid = set()
        for index, raw_name in enumerate(message.name):
            name = str(raw_name)
            if name not in required_set:
                continue
            if name in seen:
                invalid.add(name)
                positions.pop(name, None)
                continue
            seen.add(name)
            if index >= len(message.position):
                invalid.add(name)
                continue
            position = float(message.position[index])
            if not math.isfinite(position):
                invalid.add(name)
                continue
            positions[name] = position

        ordered_positions = {
            name: positions[name]
            for name in required_names
            if name in positions and name not in invalid
        }
        stamp_ros_nanoseconds = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        snapshot = {
            "available": len(ordered_positions) == len(required_names),
            "positions": ordered_positions,
            "stamp_ros_nanoseconds": stamp_ros_nanoseconds,
            "received_monotonic": time.monotonic(),
        }
        with self._lock:
            self._robot_joint_state = snapshot

    def _pose_callback(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        timing = self._localized_pose_timing(message.header.stamp)
        with self._lock:
            self._robot_pose_sequence += 1
            self._robot_pose = {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": self._quaternion_yaw(
                    message.pose.pose.orientation
                ),
                "frame_id": str(message.header.frame_id),
                "stamp": {
                    "sec": int(message.header.stamp.sec),
                    "nanosec": int(message.header.stamp.nanosec),
                },
                **timing,
                "received_at": time.time(),
                "received_monotonic": time.monotonic(),
                "sequence": self._robot_pose_sequence,
                "source": "amcl",
            }

    def _validate_navigation_goal(self, x: float, y: float) -> None:
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in (x, y)
        ):
            raise ControlRequestError(
                "Navigation goal coordinates must be finite numbers."
            )
        with self._lock:
            map_state = self._map_state
        if map_state is None:
            raise ControlRequestError(
                "Nav2 occupancy map is not available.",
                503,
            )
        map_frame = map_state.get("frame_id")
        if map_frame != self._navigation_frame:
            raise ControlRequestError(
                "Nav2 occupancy map frame does not match the navigation frame.",
                503,
            )
        resolution = map_state["resolution"]
        delta_x = x - map_state["origin"]["x"]
        delta_y = y - map_state["origin"]["y"]
        origin_yaw = map_state["origin"]["yaw"]
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = math.floor(local_x / resolution)
        row = math.floor(local_y / resolution)
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
                "A cabinet operation is active.",
                409,
            )

    def _selected_cabinet_control_id_locked(self) -> str:
        return str(
            self._cabinet_state.get("control_id")
            or self._cabinet_state.get("button_id")
            or ""
        )

    def _prepare_for_cabinet_operation_locked(self) -> None:
        """Stop local publishers before the operator assumes base/arm control."""
        self._target_linear_y = 0.0
        self._target_angular_z = 0.0
        self._linear_profile.reset()
        self._angular_profile.reset()
        self._pending_trajectory = None
        self._pending_trajectory_repeats = 0
        self._pending_trajectory_duration_sec = 0.0
        self._cmd_vel_publisher.publish(Twist())

    @classmethod
    def _normalize_cabinet_command(cls, command: Any) -> Tuple[int, str]:
        if isinstance(command, str):
            command_name = command.strip().lower().replace("-", "_")
            if command_name in cls.CABINET_COMMANDS:
                return cls.CABINET_COMMANDS[command_name], command_name
        elif isinstance(command, int) and not isinstance(command, bool):
            if command in cls.CABINET_COMMAND_NAMES:
                return command, cls.CABINET_COMMAND_NAMES[command]
        raise ControlRequestError(
            "command must be one of: press, set_state, set_position, toggle."
        )

    def _validate_cabinet_operation_locked(
        self,
        control_id: str,
        command_code: int,
        target_state: Optional[str],
        target_position: Optional[float],
    ) -> Dict[str, Any]:
        if not self._cabinet_catalog_received:
            raise ControlRequestError(
                "Cabinet control catalog is not available yet.",
                503,
            )
        control = self._cabinet_controls.get(control_id)
        if control is None:
            raise ControlRequestError(
                f"Unsupported cabinet control: {control_id}."
            )
        default_support = (
            self.CABINET_COMMAND_SUPPORT[
                OperateCabinetControl.Goal.COMMAND_PRESS
            ]
            if int(control.get("control_type", -1))
            == self.CABINET_CONTROL_TYPE_BUTTON
            else 0
        )
        supported_commands = int(
            control.get("supported_commands", default_support)
        )
        required_support = self.CABINET_COMMAND_SUPPORT[command_code]
        if supported_commands & required_support == 0:
            raise ControlRequestError(
                f"Control {control_id} does not support command "
                f"{self.CABINET_COMMAND_NAMES[command_code]}."
            )
        if (
            command_code == OperateCabinetControl.Goal.COMMAND_SET_STATE
            and target_state is None
        ):
            raise ControlRequestError(
                "target_state is required for set_state."
            )
        if (
            command_code == OperateCabinetControl.Goal.COMMAND_SET_POSITION
            and target_position is None
        ):
            raise ControlRequestError(
                "target_position is required for set_position."
            )
        state_ids = [str(value) for value in control.get("state_ids", [])]
        if target_state is not None and state_ids and target_state not in state_ids:
            raise ControlRequestError(
                f"target_state must be one of: {', '.join(state_ids)}."
            )
        if target_position is not None:
            min_position = float(control.get("min_position", target_position))
            max_position = float(control.get("max_position", target_position))
            if not min_position <= target_position <= max_position:
                raise ControlRequestError(
                    "target_position is outside the control range "
                    f"[{min_position}, {max_position}]."
                )
            if command_code == OperateCabinetControl.Goal.COMMAND_SET_POSITION:
                state_positions = [
                    float(value)
                    for value in control.get("state_positions", [])
                ]
                if not state_positions or min(
                    abs(target_position - preset)
                    for preset in state_positions
                ) > self.CABINET_DETENT_TOLERANCE:
                    raise ControlRequestError(
                        "target_position must match a configured physical "
                        "detent. Use set_state for the listed states."
                    )
        return control

    def _validate_cabinet_button_locked(self, button_id: str) -> None:
        if not self._cabinet_catalog_received:
            raise ControlRequestError(
                "Cabinet control catalog is not available yet.",
                503,
            )
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
                "A cabinet operation is already active.",
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

    @staticmethod
    def _action_server_ready(client: Any) -> bool:
        try:
            return bool(client.server_is_ready())
        except Exception:  # noqa: BLE001
            return False

    @staticmethod
    def _navigation_readiness_service_for_action(
        navigation_action: str,
    ) -> str:
        """Derive the standard Nav2 lifecycle-manager service namespace."""
        namespace, _, _action_name = navigation_action.rstrip("/").rpartition(
            "/"
        )
        if not namespace:
            return "/lifecycle_manager_navigation/is_active"
        return f"{namespace}/lifecycle_manager_navigation/is_active"

    def _navigation_availability_snapshot(self) -> Dict[str, Any]:
        """Combine action discovery with Nav2's managed lifecycle state."""
        action_available = self._action_server_ready(self._navigation_client)
        lifecycle_service_available = self._service_ready(
            self._navigation_readiness_client
        )
        with self._lock:
            lifecycle_active = bool(
                self._navigation_readiness_active
                and lifecycle_service_available
            )
            lifecycle_state = self._navigation_readiness_state
            lifecycle_message = self._navigation_readiness_message
        return {
            "available": bool(action_available and lifecycle_active),
            "action_server_available": action_available,
            "lifecycle_service_available": lifecycle_service_available,
            "lifecycle_active": lifecycle_active,
            "lifecycle_state": lifecycle_state,
            "lifecycle_message": lifecycle_message,
            "lifecycle_service": self._navigation_readiness_service,
        }

    def _poll_navigation_readiness(self) -> None:
        """Refresh Nav2 lifecycle readiness without submitting a motion goal."""
        action_available = self._action_server_ready(self._navigation_client)
        service_available = self._service_ready(
            self._navigation_readiness_client
        )
        now = time.monotonic()
        stale_future: Any = None
        with self._lock:
            future = self._navigation_readiness_future
            started = self._navigation_readiness_future_started
            timed_out = bool(
                future is not None
                and started is not None
                and now - started
                >= self.NAVIGATION_READINESS_REQUEST_TIMEOUT_SECONDS
            )
            if not action_available or not service_available or timed_out:
                self._navigation_readiness_generation += 1
                stale_future = future
                self._navigation_readiness_future = None
                self._navigation_readiness_future_started = None
                self._navigation_readiness_active = False
                if not action_available:
                    self._navigation_readiness_state = "unavailable"
                    self._navigation_readiness_message = (
                        "Nav2 NavigateToPose action server is unavailable."
                    )
                elif not service_available:
                    self._navigation_readiness_state = "unavailable"
                    self._navigation_readiness_message = (
                        "Nav2 lifecycle manager readiness service is "
                        "unavailable."
                    )
                else:
                    self._navigation_readiness_state = "timeout"
                    self._navigation_readiness_message = (
                        "Nav2 lifecycle manager readiness request timed out."
                    )
            elif future is not None:
                return

        if stale_future is not None:
            try:
                self._navigation_readiness_client.remove_pending_request(
                    stale_future
                )
            except Exception:  # noqa: BLE001
                pass
        if not action_available or not service_available:
            return

        with self._lock:
            self._navigation_readiness_generation += 1
            generation = self._navigation_readiness_generation
            if not self._navigation_readiness_active:
                self._navigation_readiness_state = "checking"
                self._navigation_readiness_message = (
                    "Checking the Nav2 lifecycle manager readiness state."
                )
        try:
            future = self._navigation_readiness_client.call_async(
                Trigger.Request()
            )
        except Exception as error:  # noqa: BLE001
            with self._lock:
                if generation == self._navigation_readiness_generation:
                    self._navigation_readiness_active = False
                    self._navigation_readiness_state = "error"
                    self._navigation_readiness_message = str(error)
            return

        with self._lock:
            if generation != self._navigation_readiness_generation:
                try:
                    self._navigation_readiness_client.remove_pending_request(
                        future
                    )
                except Exception:  # noqa: BLE001
                    pass
                return
            self._navigation_readiness_future = future
            self._navigation_readiness_future_started = now
        future.add_done_callback(
            lambda completed, token=generation: (
                self._navigation_readiness_response(completed, token)
            )
        )

    def _navigation_readiness_response(
        self,
        future: Any,
        generation: int,
    ) -> None:
        try:
            response = future.result()
            active = bool(response is not None and response.success)
            state = "active" if active else "inactive"
            message = str(getattr(response, "message", "") or "").strip()
            if not message:
                message = (
                    "Nav2 managed lifecycle nodes are active."
                    if active
                    else "Nav2 managed lifecycle nodes are not active."
                )
        except Exception as error:  # noqa: BLE001
            active = False
            state = "error"
            message = str(error)
        with self._lock:
            if (
                generation != self._navigation_readiness_generation
                or future is not self._navigation_readiness_future
            ):
                return
            self._navigation_readiness_future = None
            self._navigation_readiness_future_started = None
            self._navigation_readiness_active = active
            self._navigation_readiness_state = state
            self._navigation_readiness_message = message

    def _check_goal_acceptance(self) -> None:
        """Fail a goal whose action server never returns an acceptance response.

        ``send_goal_async``'s done-callback only fires when the server accepts
        or rejects the goal, so a discovered-but-unresponsive server leaves the
        state machine in 'sending' (or 'canceling' before acceptance) forever.
        Without this watchdog the cabinet/navigation stays active and every
        later operation is rejected with 409 until the process restarts.
        """
        now = time.monotonic()
        with self._lock:
            cabinet_sent = self._cabinet_goal_sent_monotonic
            if (
                self._cabinet_state["state"] in {"sending", "canceling"}
                and isinstance(cabinet_sent, (int, float))
                and now - float(cabinet_sent)
                >= self.GOAL_ACCEPTANCE_TIMEOUT_SEC
            ):
                self._set_cabinet_terminal(
                    "failed",
                    "Cabinet action server did not respond to the goal in "
                    "time.",
                    False,
                    None,
                    generation=self._cabinet_generation,
                )
            navigation_sent = self._navigation_goal_sent_monotonic
            if (
                self._navigation_state["state"] in {"sending", "canceling"}
                and isinstance(navigation_sent, (int, float))
                and now - float(navigation_sent)
                >= self.GOAL_ACCEPTANCE_TIMEOUT_SEC
            ):
                self._set_navigation_terminal_locked(
                    "failed",
                    "Nav2 action server did not respond to the goal in time.",
                    self._navigation_generation,
                    self._navigation_goal_token,
                )

    @staticmethod
    def _service_ready(client: Any) -> bool:
        try:
            return bool(client.service_is_ready())
        except Exception:  # noqa: BLE001
            return False

    @staticmethod
    def _duration_seconds(duration: Any) -> float:
        return float(duration.sec) + float(duration.nanosec) / 1.0e9

    @staticmethod
    def _validated_station_number(
        value: Any,
        label: str,
        *,
        status: int,
    ) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ControlRequestError(
                f"{label} must be a finite number.",
                status,
            )
        try:
            converted = float(value)
        except (OverflowError, TypeError, ValueError) as error:
            raise ControlRequestError(
                f"{label} must be a finite number.",
                status,
            ) from error
        if not math.isfinite(converted):
            raise ControlRequestError(
                f"{label} must be a finite number.",
                status,
            )
        return converted

    @staticmethod
    def _validated_station_vector(
        value: Any,
        label: str,
    ) -> Tuple[float, float, float]:
        if (
            not isinstance(value, Sequence)
            or isinstance(value, (str, bytes))
            or len(value) != 3
        ):
            raise ControlRequestError(
                f"{label} must contain exactly three finite numbers.",
                400,
            )
        return tuple(
            RosControlNode._validated_station_number(
                component,
                f"{label}[{index}]",
                status=400,
            )
            for index, component in enumerate(value)
        )  # type: ignore[return-value]

    @staticmethod
    def _rotate_vector_by_unit_quaternion(
        vector: Tuple[float, float, float],
        quaternion: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
        x, y, z, w = quaternion
        rotation = (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        )
        return tuple(
            sum(row[index] * vector[index] for index in range(3))
            for row in rotation
        )  # type: ignore[return-value]

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
