#!/usr/bin/env python3
"""World-preserving supervisor for the mutually exclusive A/B toolsets.

Gazebo Classic cannot add or remove links, joints, or ros2_control interfaces
from a live model.  The two XCZS end-effector sets therefore cannot be swapped
inside one robot entity.  This node keeps the Gazebo *world* alive and replaces
only the robot child launch after it has captured the robot root pose.  It
exposes the transition exclusively through ROS 2:

* ``/xczs/toolset/switch`` (:class:`SwitchToolset`) accepts an asynchronous
  request; and
* ``/xczs/toolset/status`` (transient-local JSON) is the single runtime source
  of truth for the Web gateway.

The supervisor never edits URDF meshes, materials, collision geometry or model
appearance.  It owns just one child ``ros2 launch`` process group and performs
a bounded rollback to the old toolset if the target cannot become ready.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Iterable, Mapping
from uuid import uuid4

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import DeleteEntity, GetEntityState, GetModelList
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from xczs_inspection_robot_interfaces.action import OperateCabinetControl
from xczs_inspection_robot_interfaces.srv import SwitchToolset


VALID_TOOLSETS = frozenset({"A", "B"})
DEFAULT_SWITCH_SERVICE = "/xczs/toolset/switch"
DEFAULT_STATUS_TOPIC = "/xczs/toolset/status"
DEFAULT_JOINT_STATE_TOPIC = "/xczs/joint_states"
DEFAULT_NAV2_LIFECYCLE_ACTIVE_SERVICE = "/lifecycle_manager_navigation/is_active"
DEFAULT_READY_TIMEOUT_SEC = 120.0
DEFAULT_SERVICE_TIMEOUT_SEC = 10.0
DEFAULT_CHILD_STOP_TIMEOUT_SEC = 20.0
POLL_INTERVAL_SEC = 0.10
# The global-args guard lives inside gzserver and only appears once the Gazebo
# world has loaded, which can take longer than the service timeout during a
# cold start.  The very first robot spawn starts from a fresh gzserver whose
# global rcl arguments are already clean, so the guard is only a requirement
# for switch/rollback spawns, never for the initial one.
GUARD_INITIAL_WAIT_SEC = 3.0
# A Nav2 lifecycle manager can advertise ``is_active`` while its single
# executor is busy activating a managed node.  Such a request is legitimately
# queued without a response; do not let it block an A/B readiness worker for
# the generic Gazebo service timeout.
NAV2_LIFECYCLE_REQUEST_TIMEOUT_SEC = 1.0
# Gazebo's DeleteEntity reply may precede completion of the model/plugin
# teardown on the simulation thread.  Require two consecutive negative entity
# lookups before spawning the replacement, so the next child does not race a
# still-live plugin instance.
ENTITY_REMOVAL_STABLE_CYCLES = 2
# The lifecycle endpoint is inexpensive, but polling it for every joint-state
# iteration needlessly hammers Nav2 while it is still starting.  A short cache
# still requires a positive response from the child stack before readiness can
# be reported.
NAV2_LIFECYCLE_PROBE_INTERVAL_SEC = 0.50

TOOLSET_ACTIONS: Mapping[str, tuple[tuple[str, str], ...]] = {
    "A": (
        (
            "three-cylinder tool",
            "/xczs/three_cylinder_controller/follow_joint_trajectory",
        ),
        (
            "two-cylinder tool",
            "/xczs/two_cylinder_controller/follow_joint_trajectory",
        ),
    ),
    "B": (
        (
            "rotate-button tool",
            "/xczs/rotate_button_controller/follow_joint_trajectory",
        ),
        (
            "rocker tool",
            "/xczs/rocker_controller/follow_joint_trajectory",
        ),
    ),
}
TOOLSET_JOINTS: Mapping[str, tuple[str, ...]] = {
    "A": (
        "r_three_cyl_finger1_joint",
        "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint",
        "l_two_cyl_finger1_joint",
        "l_two_cyl_finger2_joint",
    ),
    "B": (
        "r_rotbtn_rotate_joint",
        "r_rotbtn_jaw1_joint",
        "r_rotbtn_jaw2_joint",
        "l_rocker_rotor_joint",
    ),
}
ARM_ACTIONS = (
    ("left arm", "/xczs/left_arm_controller/follow_joint_trajectory"),
    ("right arm", "/xczs/right_arm_controller/follow_joint_trajectory"),
)
ARM_JOINTS = tuple(
    f"{side}_arm_{index}_joint" for side in ("l", "r") for index in range(7)
)

# These arguments are deliberately owned by the supervisor.  Allowing the
# caller's common launch list to override one would create a second Gazebo,
# re-spawn cabinets after every tool change, or produce a topology mismatch.
_SUPERVISOR_OWNED_LAUNCH_ARGUMENTS = frozenset(
    {
        "gazebo",
        "gui",
        "robot_bringup",
        "teleop",
        "control_gui",
        "moveit_rviz",
        "nav2_rviz",
        "toolset",
        "cabinet_bringup",
        "spawn_cabinet",
        "robot_spawn_x",
        "robot_spawn_y",
        "robot_spawn_z",
        "robot_spawn_yaw",
        "gazebo_plugin_instance_id",
    }
)

# The launch file derives these three paths from ``toolset`` when they are
# omitted.  The unified shell launcher supplies absolute paths instead so it
# can validate selected assets before starting ROS.  A fixed A-path in that
# static list would quietly make a later B switch start MoveIt with A's SRDF
# or controller map, so explicitly require a small, literal template when a
# caller overrides one of them.  It is expanded by this supervisor only after
# the requested toolset has been normalized to A/B; no shell interpolation is
# involved.
_TOOLSET_TEMPLATE_LAUNCH_ARGUMENTS = frozenset(
    {
        "moveit_srdf",
        "moveit_joint_limits",
        "moveit_controllers",
    }
)
_TOOLSET_TEMPLATE_TOKEN = "{toolset}"
_GAZEBO_PLUGIN_INSTANCE_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,63}$")

STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ToolsetSupervisorError(RuntimeError):
    """A controlled failure while replacing the robot child stack."""


def normalize_toolset(value: object) -> str:
    """Return A/B in uppercase or raise a stable request error."""
    if not isinstance(value, str):
        raise ToolsetSupervisorError("toolset must be A or B")
    normalized = value.strip().upper()
    if normalized not in VALID_TOOLSETS:
        raise ToolsetSupervisorError("toolset must be A or B")
    return normalized


def _finite_positive(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ToolsetSupervisorError(f"{label} must be a positive finite number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ToolsetSupervisorError(f"{label} must be a positive finite number")
    return result


def _quaternion_yaw(orientation: Any) -> float:
    """Return planar yaw, rejecting a malformed Gazebo orientation."""
    try:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
    except (AttributeError, TypeError, ValueError) as error:
        raise ToolsetSupervisorError("Gazebo returned an invalid robot orientation") from error
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ToolsetSupervisorError("Gazebo returned a non-finite robot orientation")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        raise ToolsetSupervisorError("Gazebo returned a zero-length robot orientation")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _parse_launch_arguments(values: Iterable[str]) -> tuple[str, ...]:
    """Validate the caller-owned static child-launch arguments once."""
    parsed: dict[str, str] = {}
    for raw in values:
        if not isinstance(raw, str) or ":=" not in raw:
            raise ToolsetSupervisorError(
                "every --launch-arg must use launch syntax name:=value"
            )
        name, value = raw.split(":=", 1)
        name = name.strip()
        if not name or any(character.isspace() for character in name):
            raise ToolsetSupervisorError(
                f"invalid launch argument name in {raw!r}"
            )
        if name in _SUPERVISOR_OWNED_LAUNCH_ARGUMENTS:
            raise ToolsetSupervisorError(
                f"--launch-arg must not override supervisor-owned '{name}'"
            )
        if name in parsed:
            raise ToolsetSupervisorError(
                f"duplicate --launch-arg '{name}'"
            )
        if (
            name in _TOOLSET_TEMPLATE_LAUNCH_ARGUMENTS
            and _TOOLSET_TEMPLATE_TOKEN not in value
        ):
            raise ToolsetSupervisorError(
                f"--launch-arg '{name}' must contain "
                f"{_TOOLSET_TEMPLATE_TOKEN!r} or be omitted so the launch "
                "file can derive it from toolset"
            )
        parsed[name] = value
    return tuple(f"{name}:={value}" for name, value in parsed.items())


class ToolsetSupervisor(Node):
    """Own one replaceable robot child while keeping Gazebo alive."""

    def __init__(
        self,
        *,
        initial_toolset: str,
        robot_name: str,
        launch_package: str,
        launch_file: str,
        launch_arguments: Iterable[str],
        initial_spawn_cabinet: bool,
        cabinet_bringup: bool,
        require_nav2: bool,
        require_cabinet_action: bool,
        ready_timeout_sec: float,
        service_timeout_sec: float,
        child_stop_timeout_sec: float,
        joint_state_topic: str = DEFAULT_JOINT_STATE_TOPIC,
        switch_service: str = DEFAULT_SWITCH_SERVICE,
        status_topic: str = DEFAULT_STATUS_TOPIC,
        cabinet_action_names: Iterable[str] = (),
    ) -> None:
        super().__init__("xczs_toolset_supervisor")
        self._initial_toolset = normalize_toolset(initial_toolset)
        if not isinstance(robot_name, str) or not robot_name.strip():
            raise ToolsetSupervisorError("robot_name must be non-empty")
        self._robot_name = robot_name.strip()
        if not isinstance(launch_package, str) or not launch_package.strip():
            raise ToolsetSupervisorError("launch_package must be non-empty")
        if not isinstance(launch_file, str) or not launch_file.strip():
            raise ToolsetSupervisorError("launch_file must be non-empty")
        if not isinstance(joint_state_topic, str) or not joint_state_topic.startswith("/"):
            raise ToolsetSupervisorError("joint_state_topic must be an absolute ROS name")
        if not isinstance(switch_service, str) or not switch_service.startswith("/"):
            raise ToolsetSupervisorError("switch_service must be an absolute ROS name")
        if not isinstance(status_topic, str) or not status_topic.startswith("/"):
            raise ToolsetSupervisorError("status_topic must be an absolute ROS name")
        self._launch_package = launch_package.strip()
        self._launch_file = launch_file.strip()
        self._launch_arguments = _parse_launch_arguments(launch_arguments)
        self._initial_spawn_cabinet = bool(initial_spawn_cabinet)
        self._cabinet_bringup = bool(cabinet_bringup)
        if self._initial_spawn_cabinet and not self._cabinet_bringup:
            raise ToolsetSupervisorError(
                "initial_spawn_cabinet=true requires cabinet_bringup=true"
            )
        self._require_nav2 = bool(require_nav2)
        self._require_cabinet_action = bool(require_cabinet_action)
        if self._require_cabinet_action and not self._cabinet_bringup:
            raise ToolsetSupervisorError(
                "require_cabinet_action=true requires cabinet_bringup=true"
            )
        if isinstance(cabinet_action_names, str):
            raise ToolsetSupervisorError(
                "cabinet_action_names must be an iterable of absolute ROS names"
            )
        cabinet_actions: list[str] = []
        for action_name in cabinet_action_names:
            if (
                not isinstance(action_name, str)
                or not action_name.startswith("/")
                or any(character.isspace() for character in action_name)
            ):
                raise ToolsetSupervisorError(
                    "every cabinet action name must be an absolute ROS name"
                )
            if action_name not in cabinet_actions:
                cabinet_actions.append(action_name)
        if self._require_cabinet_action and not cabinet_actions:
            raise ToolsetSupervisorError(
                "require_cabinet_action=true requires at least one cabinet action"
            )
        self._cabinet_action_names = tuple(cabinet_actions)
        self._ready_timeout_sec = _finite_positive(
            ready_timeout_sec, "ready_timeout_sec"
        )
        self._service_timeout_sec = _finite_positive(
            service_timeout_sec, "service_timeout_sec"
        )
        self._child_stop_timeout_sec = _finite_positive(
            child_stop_timeout_sec, "child_stop_timeout_sec"
        )

        self._lock = threading.RLock()
        self._stopping = False
        # A worker may be between two irreversible switch stages when the
        # outer launch asks us to stop.  This event is checked both outside
        # and inside the process-start lock so shutdown can never race a new
        # robot child into existence after the supervisor has begun exiting.
        self._stop_requested = threading.Event()
        self._intentional_child_stop = False
        self._fatal_error: str | None = None
        self._child_process: subprocess.Popen[bytes] | None = None
        self._child_process_group_id: int | None = None
        # Gazebo Classic can defer a deleted model's plugin destruction by one
        # or more simulation updates.  Every child therefore gets a fresh
        # plugin node suffix, including a rollback to the same toolset.  This
        # only changes private ROS node names; all published/subscribed topic
        # names and the robot entity name stay unchanged.
        self._child_instance_sequence = 0
        self._operation_thread: threading.Thread | None = None
        self._latest_joint_positions: dict[str, float] = {}
        self._latest_joint_received_monotonic: float | None = None

        self._status: dict[str, Any] = {
            "schema_version": 1,
            "managed": True,
            "state": "starting",
            "stage": "starting_robot_child",
            "ready": False,
            "active_toolset": self._initial_toolset,
            "target_toolset": self._initial_toolset,
            "generation": 1,
            "operation_id": None,
            "last_operation_id": None,
            "last_error": None,
            "message": "Starting the initial robot child stack.",
            "updated_at": time.time(),
        }

        self._status_publisher = self.create_publisher(
            String,
            status_topic,
            STATUS_QOS,
        )
        self._switch_service = self.create_service(
            SwitchToolset,
            switch_service,
            self._switch_service_callback,
        )
        self._delete_entity_client = self.create_client(
            DeleteEntity,
            "/delete_entity",
        )
        self._get_entity_state_client = self.create_client(
            GetEntityState,
            "/get_entity_state",
        )
        self._get_model_list_client = self.create_client(
            GetModelList,
            "/get_model_list",
        )
        self._global_args_guard_client = self.create_client(
            Trigger,
            "/xczs/global_args_guard/reset",
        )
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            20,
        )
        self._trajectory_clients = {
            action_name: ActionClient(self, FollowJointTrajectory, action_name)
            for _label, action_name in ARM_ACTIONS
            + TOOLSET_ACTIONS["A"]
            + TOOLSET_ACTIONS["B"]
        }
        self._navigation_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        # Create this client at each robot-child boundary rather than once for
        # the supervisor lifetime.  A/B replacement tears down and recreates
        # Nav2 under the same service name; preserving the old client can leave
        # a request routed to the retiring lifecycle manager or accept its late
        # response as evidence that the replacement is ready.
        self._nav2_lifecycle_active_client: Any | None = None
        self._nav2_lifecycle_client_generation = 0
        self._nav2_lifecycle_active = False
        self._next_nav2_lifecycle_probe_monotonic = 0.0
        self._nav2_lifecycle_status_detail = "Nav2 lifecycle manager is not active"
        self._cabinet_action_clients = {
            action_name: ActionClient(
                self,
                OperateCabinetControl,
                action_name,
            )
            for action_name in self._cabinet_action_names
        }
        self._child_monitor = self.create_timer(0.5, self._monitor_child)
        self._publish_status()

    @property
    def fatal_error(self) -> str | None:
        with self._lock:
            return self._fatal_error

    def start(self) -> None:
        """Start the initial robot child and verify it asynchronously."""
        with self._lock:
            if self._stopping:
                raise ToolsetSupervisorError("supervisor is stopping")
            if self._operation_thread is not None:
                raise ToolsetSupervisorError("supervisor has already started")
            operation_id = self._new_operation_id()
            self._set_status_locked(
                state="starting",
                stage="starting_robot_child",
                ready=False,
                target_toolset=self._initial_toolset,
                operation_id=operation_id,
                message="Starting the initial robot child stack.",
            )
            worker = threading.Thread(
                target=self._initial_start_worker,
                args=(operation_id,),
                name="xczs-toolset-initial-start",
                daemon=True,
            )
            self._operation_thread = worker
            worker.start()

    def shutdown(self) -> None:
        """Stop the owned child group during normal supervisor teardown."""
        self.request_shutdown()
        with self._lock:
            worker = self._operation_thread

        # A switch worker can otherwise observe the old child, be preempted
        # here, then spawn a new child after this method has already stopped
        # its stale snapshot.  Do not join ourselves when shutdown is reached
        # from a worker fault path.
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=self._child_stop_timeout_sec + 5.0)

        with self._lock:
            child = self._child_process
        if child is not None:
            try:
                self._stop_child_process(child)
            except ToolsetSupervisorError as error:
                self.get_logger().warning(f"Failed to stop robot child: {error}")
        with self._lock:
            if self._child_process is child:
                self._child_process = None
                self._child_process_group_id = None

    def request_shutdown(self) -> None:
        """Prevent workers from advancing or spawning another child."""
        self._stop_requested.set()
        with self._lock:
            self._stopping = True

    def _is_stopping(self) -> bool:
        return self._stop_requested.is_set()

    def _ensure_not_stopping(self) -> None:
        if self._is_stopping():
            raise ToolsetSupervisorError("supervisor is stopping")

    def status_snapshot(self) -> dict[str, Any]:
        """Return a detached, JSON-safe runtime status snapshot."""
        with self._lock:
            return dict(self._status)

    def _new_operation_id(self) -> str:
        return f"toolset-{uuid4().hex}"

    def _set_status_locked(self, **changes: Any) -> None:
        self._status.update(changes)
        self._status["updated_at"] = time.time()
        self._publish_status_locked()

    def _publish_status(self) -> None:
        with self._lock:
            self._publish_status_locked()

    def _publish_status_locked(self) -> None:
        message = String()
        message.data = json.dumps(
            self._status,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        try:
            self._status_publisher.publish(message)
        except Exception:
            # SIGINT/SIGTERM may invalidate rclpy before the background
            # worker observes the stop token.  During orderly shutdown status
            # publication is no longer useful and must not mask cleanup.
            if not self._is_stopping():
                raise

    def _switch_service_callback(
        self,
        request: SwitchToolset.Request,
        response: SwitchToolset.Response,
    ) -> SwitchToolset.Response:
        try:
            target = normalize_toolset(request.toolset)
        except ToolsetSupervisorError as error:
            return self._reject_switch_response(response, str(error))
        expected_generation = int(request.expected_generation)
        with self._lock:
            status = dict(self._status)
            if self._stopping:
                return self._reject_switch_response(response, "supervisor is stopping")
            if status["state"] in {"starting", "switching"}:
                return self._reject_switch_response(
                    response,
                    "a robot/toolset transition is already in progress",
                )
            if status.get("state") != "ready" or not status.get("ready"):
                return self._reject_switch_response(
                    response,
                    "supervisor is not ready; resolve the current fault before retrying",
                )
            if expected_generation and expected_generation != int(status["generation"]):
                return self._reject_switch_response(
                    response,
                    "toolset status generation is stale; refresh status and retry",
                )
            active = str(status.get("active_toolset") or "")
            if status.get("ready") and target == active:
                response.accepted = True
                response.state = str(status["state"])
                response.active_toolset = active
                response.target_toolset = target
                response.generation = int(status["generation"])
                response.message = "requested toolset is already active"
                return response

            generation = int(status["generation"]) + 1
            operation_id = self._new_operation_id()
            self._set_status_locked(
                state="switching",
                stage="queued",
                ready=False,
                target_toolset=target,
                generation=generation,
                operation_id=operation_id,
                message=(
                    f"Switching robot end-effector set from {active or 'unknown'} "
                    f"to {target}; Gazebo world remains running."
                ),
            )
            worker = threading.Thread(
                target=self._switch_worker,
                args=(active, target, generation, operation_id),
                name=f"xczs-toolset-switch-{generation}",
                daemon=True,
            )
            self._operation_thread = worker
            worker.start()

        response.accepted = True
        response.state = "switching"
        response.active_toolset = active
        response.target_toolset = target
        response.generation = generation
        response.message = "toolset switch accepted"
        return response

    def _reject_switch_response(
        self,
        response: SwitchToolset.Response,
        message: str,
    ) -> SwitchToolset.Response:
        status = self.status_snapshot()
        response.accepted = False
        response.state = str(status.get("state", "unavailable"))
        response.active_toolset = str(status.get("active_toolset") or "")
        response.target_toolset = str(status.get("target_toolset") or "")
        response.generation = int(status.get("generation", 0))
        response.message = message
        return response

    def _initial_start_worker(self, operation_id: str) -> None:
        try:
            self._ensure_not_stopping()
            self._start_robot_child(
                self._initial_toolset,
                pose=None,
                spawn_cabinet=self._initial_spawn_cabinet,
                guard_reset_required=False,
            )
            self._wait_for_robot_ready(self._initial_toolset)
        except Exception as error:  # noqa: BLE001 - process fault boundary
            if self._is_stopping():
                return
            message = f"Initial robot child did not become ready: {error}"
            self.get_logger().error(message)
            with self._lock:
                if self._stopping:
                    return
                self._fatal_error = message
                self._set_status_locked(
                    state="failed",
                    stage="initial_start_failed",
                    ready=False,
                    target_toolset=None,
                    operation_id=None,
                    last_operation_id=operation_id,
                    last_error=message,
                    message=message,
                )
            self.get_logger().error(
                "Keeping the Gazebo world alive after the failed initial "
                "start; restart the system to retry the robot child."
            )
            return
        if self._is_stopping():
            return
        with self._lock:
            if self._stopping:
                return
            self._set_status_locked(
                state="ready",
                stage="ready",
                ready=True,
                active_toolset=self._initial_toolset,
                target_toolset=None,
                operation_id=None,
                last_operation_id=operation_id,
                last_error=None,
                message=f"Toolset {self._initial_toolset} is ready.",
            )

    def _switch_worker(
        self,
        previous: str,
        target: str,
        generation: int,
        operation_id: str,
    ) -> None:
        pose: dict[str, float] | None = None
        target_started = False
        old_stack_mutation_started = False
        try:
            self._ensure_not_stopping()
            self._set_switch_stage(generation, "capturing_pose", "Capturing robot pose.")
            pose = self._capture_robot_pose()
            self._set_switch_stage(
                generation,
                "stopping_old_stack",
                f"Stopping the {previous} robot control stack.",
            )
            # Everything before this point is a read-only preflight.  Record
            # the irreversible boundary immediately before signalling the old
            # child so a pose/service failure can never trigger a destructive
            # rollback against an otherwise healthy robot.
            old_stack_mutation_started = True
            self._stop_current_robot_child()
            self._set_switch_stage(
                generation,
                "removing_old_robot",
                "Removing the old robot entity while keeping the Gazebo world.",
            )
            self._delete_robot_entity()
            self._set_switch_stage(
                generation,
                "starting_target_stack",
                f"Starting the {target} robot control stack.",
            )
            self._start_robot_child(target, pose=pose, spawn_cabinet=False)
            target_started = True
            self._set_switch_stage(
                generation,
                "verifying_target",
                f"Verifying controllers and joint state for toolset {target}.",
            )
            self._wait_for_robot_ready(target)
        except Exception as error:  # noqa: BLE001 - rollback boundary
            if self._is_stopping():
                return
            failure = f"Toolset {target} could not become ready: {error}"
            self.get_logger().error(failure)
            if not old_stack_mutation_started:
                try:
                    self._wait_for_robot_ready(previous)
                    previous_ready = True
                    readiness_error = None
                except Exception as readiness_failure:  # noqa: BLE001
                    previous_ready = False
                    readiness_error = str(readiness_failure)
                with self._lock:
                    if self._stopping:
                        return
                    if previous_ready:
                        self._set_status_locked(
                            state="ready",
                            stage="preflight_failed",
                            ready=True,
                            active_toolset=previous,
                            target_toolset=None,
                            operation_id=None,
                            last_operation_id=operation_id,
                            last_error=failure,
                            message=(
                                f"Toolset {target} switch failed before the "
                                f"robot was modified; toolset {previous} "
                                "remains ready."
                            ),
                        )
                    else:
                        fatal = (
                            failure
                            + "; the unchanged previous robot stack could not "
                            + f"be verified ready: {readiness_error}"
                        )
                        self._fatal_error = fatal
                        self._set_status_locked(
                            state="failed",
                            stage="preflight_failed",
                            ready=False,
                            active_toolset=previous or None,
                            target_toolset=None,
                            operation_id=None,
                            last_operation_id=operation_id,
                            last_error=fatal,
                            message=fatal,
                        )
                return
            restored = self._rollback(previous, pose, target_started, generation)
            with self._lock:
                if self._stopping:
                    return
                if restored:
                    self._set_status_locked(
                        state="ready",
                        stage="rollback_ready",
                        ready=True,
                        active_toolset=previous,
                        target_toolset=None,
                        operation_id=None,
                        last_operation_id=operation_id,
                        last_error=failure,
                        message=(
                            f"Toolset {target} switch failed; toolset {previous} "
                            "was restored."
                        ),
                    )
                else:
                    fatal = failure + "; rollback to the previous toolset failed"
                    self._fatal_error = fatal
                    self._set_status_locked(
                        state="failed",
                        stage="rollback_failed",
                        ready=False,
                        active_toolset=previous or None,
                        target_toolset=None,
                        operation_id=None,
                        last_operation_id=operation_id,
                        last_error=fatal,
                        message=fatal,
                    )
            if not restored:
                # The Gazebo world must survive an internal fault: the shell's
                # process monitor tears everything down when this supervisor
                # exits, so staying alive keeps the world recoverable.  The
                # failed state rejects further switches; restart to relaunch.
                self.get_logger().error(
                    "Keeping the Gazebo world alive after the failed switch; "
                    "restart the system to recover the robot child."
                )
            return

        if self._is_stopping():
            return
        with self._lock:
            if self._stopping:
                return
            self._set_status_locked(
                state="ready",
                stage="ready",
                ready=True,
                active_toolset=target,
                target_toolset=None,
                operation_id=None,
                last_operation_id=operation_id,
                last_error=None,
                message=(
                    f"Toolset {target} is ready; Gazebo world was preserved."
                ),
            )

    def _set_switch_stage(self, generation: int, stage: str, message: str) -> None:
        self._ensure_not_stopping()
        with self._lock:
            if self._stopping:
                raise ToolsetSupervisorError("supervisor is stopping")
            if int(self._status.get("generation", -1)) != generation:
                raise ToolsetSupervisorError("toolset transition was superseded")
            self._set_status_locked(stage=stage, message=message)

    def _rollback(
        self,
        previous: str,
        pose: dict[str, float] | None,
        target_started: bool,
        generation: int,
    ) -> bool:
        if not previous or self._is_stopping():
            return False
        try:
            self._set_switch_stage(
                generation,
                "rolling_back",
                f"Restoring previous toolset {previous}.",
            )
            with self._lock:
                current_child = self._child_process
            # A child may have been assigned just before its startup helper
            # raised, leaving ``target_started`` false.  Always stop any
            # residual child before deleting the entity and respawning the
            # previous topology.
            if target_started or current_child is not None:
                self._stop_current_robot_child()
            self._delete_robot_entity(ignore_missing=True)
            self._start_robot_child(previous, pose=pose, spawn_cabinet=False)
            self._wait_for_robot_ready(previous)
            return True
        except Exception as error:  # noqa: BLE001 - preserve original failure
            if not self._is_stopping():
                self.get_logger().error(f"Toolset rollback failed: {error}")
            return False

    def _reset_gazebo_global_args(self, *, required: bool = True) -> None:
        """Reset gzserver's process-global rcl arguments before a robot spawn.

        gazebo_ros2_control rewrites the gzserver global rcl arguments on
        every robot spawn, injecting a ``--remap __ns:=/xczs`` default remap
        (verified live: the second robot's lidar appears on ``/xczs/scan``
        instead of ``/xczs/lidar/scan``).  rcl_node_init applies that default
        remap to every node the gzserver process creates afterwards, so the
        sensor nodes of the NEXT spawn, which load before the ros2_control
        plugin, have their ``/xczs/lidar/scan`` and ``/xczs/camera/*``
        namespaces truncated to ``/xczs`` -- breaking Nav2, the Web sensor
        feeds and the toolset switch readiness check.  Calling the guard
        service here, before any child launch issues a spawn_entity, gives the
        new robot a clean default namespace so its sensor plugins register
        under their full paths again.

        The guard service lives in gzserver and only appears after the Gazebo
        world has loaded.  During a cold start the world can still be loading
        when the *initial* spawn begins, but a fresh gzserver always starts
        with clean global arguments, so ``required=False`` (the initial spawn)
        skips the reset with a warning if the guard is not up yet.  Switch and
        rollback spawns re-enter a long-lived gzserver whose arguments are
        polluted by the previous robot, so they pass ``required=True`` and fail
        fast if the guard cannot be reached.
        """
        request = Trigger.Request()
        try:
            response = self._call_service(
                self._global_args_guard_client,
                request,
                "reset gzserver global rcl arguments",
                timeout_sec=(
                    self._service_timeout_sec
                    if required
                    else GUARD_INITIAL_WAIT_SEC
                ),
            )
        except ToolsetSupervisorError:
            if required:
                raise
            self.get_logger().warn(
                "gazebo global-args guard is not ready yet; skipping the reset "
                "for the initial spawn (a fresh gzserver starts with clean "
                "global arguments, so the first spawn is unaffected by the "
                "namespace truncation)."
            )
            return
        if not bool(getattr(response, "success", False)):
            message = (
                "gazebo global-args guard could not reset rcl arguments: "
                f"{getattr(response, 'message', '')}"
            )
            if required:
                raise ToolsetSupervisorError(message)
            self.get_logger().warn(
                "%s (continuing with the initial spawn)", message
            )
            return

    def _start_robot_child(
        self,
        toolset: str,
        *,
        pose: Mapping[str, float] | None,
        spawn_cabinet: bool,
        guard_reset_required: bool = True,
    ) -> None:
        target = normalize_toolset(toolset)
        with self._lock:
            if self._stopping:
                raise ToolsetSupervisorError("supervisor is stopping")
            existing = self._child_process
            if existing is not None and self._process_group_is_alive(
                self._child_process_group_id or existing.pid
            ):
                raise ToolsetSupervisorError("robot child is already running")
            command = [
                "ros2",
                "launch",
                self._launch_package,
                self._launch_file,
                *self._expanded_launch_arguments(target),
                "gazebo:=false",
                "gui:=false",
                "robot_bringup:=true",
                "teleop:=false",
                "control_gui:=false",
                "moveit_rviz:=false",
                "nav2_rviz:=false",
                f"toolset:={target}",
                f"cabinet_bringup:={'true' if self._cabinet_bringup else 'false'}",
                f"spawn_cabinet:={'true' if spawn_cabinet else 'false'}",
            ]
            self._child_instance_sequence += 1
            plugin_instance_id = self._gazebo_plugin_instance_id(
                target, self._child_instance_sequence
            )
            command.append(f"gazebo_plugin_instance_id:={plugin_instance_id}")
            if pose is not None:
                command.extend(
                    f"robot_spawn_{field}:={float(pose[field]):.12g}"
                    for field in ("x", "y", "z", "yaw")
                )
            # gazebo_ros2_control pollutes the gzserver global rcl arguments
            # (default namespace remap to /xczs) on every robot spawn, which
            # truncates the sensor namespaces of every later spawn.  Reset them
            # right before this spawn so the new robot's sensors register under
            # their full /xczs/lidar/scan and /xczs/camera/* paths again.  Only
            # switch/rollback spawns require the guard; the initial spawn starts
            # from a clean gzserver and must not be held hostage by the world
            # still loading.
            self._reset_gazebo_global_args(required=guard_reset_required)
            environment = dict(os.environ)
            environment["TOOLSET"] = target
            environment["XCZS_ACTIVE_TOOLSET"] = target
            try:
                child = subprocess.Popen(
                    command,
                    env=environment,
                    start_new_session=True,
                )
            except OSError as error:
                raise ToolsetSupervisorError(
                    f"could not start robot child: {error}"
                ) from error
            self._child_process = child
            self._child_process_group_id = child.pid
            self._intentional_child_stop = False
            self._latest_joint_positions = {}
            self._latest_joint_received_monotonic = None
            self._reset_nav2_lifecycle_probe_locked()
        self.get_logger().info(
            "Started robot child for toolset %s (PID %s, plugin instance %s)."
            % (target, child.pid, plugin_instance_id)
        )

    @staticmethod
    def _gazebo_plugin_instance_id(toolset: str, sequence: int) -> str:
        """Return a bounded, ROS-safe unique name suffix for one child launch."""
        target = normalize_toolset(toolset)
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 1:
            raise ToolsetSupervisorError("robot child instance sequence is invalid")
        instance_id = f"robot_{target.lower()}_{sequence}"
        if _GAZEBO_PLUGIN_INSTANCE_ID_PATTERN.fullmatch(instance_id) is None:
            raise ToolsetSupervisorError("robot child plugin instance id is invalid")
        return instance_id

    def _expanded_launch_arguments(self, toolset: str) -> tuple[str, ...]:
        """Expand only the audited per-toolset path token for one child."""
        target = normalize_toolset(toolset)
        expanded: list[str] = []
        for raw in self._launch_arguments:
            name, value = raw.split(":=", 1)
            if name in _TOOLSET_TEMPLATE_LAUNCH_ARGUMENTS:
                value = value.replace(_TOOLSET_TEMPLATE_TOKEN, target)
            expanded.append(f"{name}:={value}")
        return tuple(expanded)

    def _stop_current_robot_child(self) -> None:
        with self._lock:
            child = self._child_process
        if child is None:
            return
        self._stop_child_process(child)
        with self._lock:
            if self._child_process is child:
                self._child_process = None
                self._child_process_group_id = None

    def _stop_child_process(self, child: subprocess.Popen[bytes]) -> None:
        with self._lock:
            self._intentional_child_stop = True
            process_group_id = (
                self._child_process_group_id
                if self._child_process is child
                else child.pid
            )
        if process_group_id is None or not self._process_group_is_alive(
            process_group_id
        ):
            return
        self._signal_child_process_group(process_group_id, signal.SIGINT)
        if self._wait_for_process_group_exit(
            child,
            process_group_id,
            self._child_stop_timeout_sec,
        ):
            return
        self.get_logger().warning("Robot child ignored SIGINT; sending SIGTERM.")
        self._signal_child_process_group(process_group_id, signal.SIGTERM)
        if self._wait_for_process_group_exit(child, process_group_id, 5.0):
            return
        self.get_logger().error("Robot child ignored SIGTERM; sending SIGKILL.")
        self._signal_child_process_group(process_group_id, signal.SIGKILL)
        if not self._wait_for_process_group_exit(child, process_group_id, 2.0):
            raise ToolsetSupervisorError(
                "robot child process group did not exit after SIGKILL"
            )

    @staticmethod
    def _process_group_is_alive(process_group_id: int) -> bool:
        try:
            os.killpg(process_group_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    @staticmethod
    def _signal_child_process_group(process_group_id: int, signum: int) -> None:
        try:
            os.killpg(process_group_id, signum)
        except ProcessLookupError:
            return

    @classmethod
    def _wait_for_process_group_exit(
        cls,
        child: subprocess.Popen[bytes],
        process_group_id: int,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            # Reap the launch leader while waiting.  A dead-but-unreaped
            # session leader still keeps its process group visible to
            # killpg(…, 0), which previously made a clean shutdown look like
            # a stubborn residual child group.
            child.poll()
            if not cls._process_group_is_alive(process_group_id):
                return True
            time.sleep(POLL_INTERVAL_SEC)
        child.poll()
        return not cls._process_group_is_alive(process_group_id)

    def _capture_robot_pose(self) -> dict[str, float]:
        request = GetEntityState.Request()
        request.name = self._robot_name
        request.reference_frame = "world"
        response = self._call_service(
            self._get_entity_state_client,
            request,
            "get robot entity state",
        )
        if not bool(getattr(response, "success", False)):
            detail = str(getattr(response, "status_message", "")).strip()
            raise ToolsetSupervisorError(
                detail or f"Gazebo could not find robot '{self._robot_name}'"
            )
        pose = getattr(response, "state", None)
        pose = getattr(pose, "pose", None)
        if pose is None:
            raise ToolsetSupervisorError("Gazebo returned no robot pose")
        try:
            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)
        except (AttributeError, TypeError, ValueError) as error:
            raise ToolsetSupervisorError("Gazebo returned an invalid robot position") from error
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise ToolsetSupervisorError("Gazebo returned a non-finite robot position")
        # ``spawn_entity`` receives the body/root yaw while scenes express the
        # base_link yaw.  The launch file applies the inverse +pi/2 transform,
        # so convert it back here to preserve the robot's real map heading.
        base_yaw = _quaternion_yaw(pose.orientation) - math.pi / 2.0
        return {"x": x, "y": y, "z": z, "yaw": base_yaw}

    def _delete_robot_entity(self, *, ignore_missing: bool = False) -> None:
        request = DeleteEntity.Request()
        request.name = self._robot_name
        response = self._call_service(
            self._delete_entity_client,
            request,
            "delete robot entity",
        )
        if bool(getattr(response, "success", False)):
            self._wait_for_robot_entity_absent()
            return
        detail = str(getattr(response, "status_message", "")).strip()
        absent = self._entity_status_means_absent(detail)
        if ignore_missing and absent:
            self._wait_for_robot_entity_absent()
            return
        raise ToolsetSupervisorError(
            detail or f"Gazebo could not delete robot '{self._robot_name}'"
        )

    @staticmethod
    def _entity_status_means_absent(detail: str) -> bool:
        normalized = detail.lower()
        return any(
            token in normalized
            for token in ("does not exist", "doesn't exist", "not found", "no such")
        )

    def _wait_for_robot_entity_absent(self) -> None:
        """Wait until Gazebo has stably removed the old robot entity.

        A successful ``/delete_entity`` response only acknowledges the request.
        The model's sensor plugins can otherwise still occupy their
        ``gazebo_ros::Node`` names while the next child starts.  Requiring two
        consecutive absent observations spans at least one poll interval and
        gives Gazebo's deletion queue a deterministic handoff point.

        Query the model list instead of deliberately calling
        ``/get_entity_state`` for a missing model.  ``gazebo_ros_state`` logs
        every such negative lookup as an ERROR even though absence is the
        expected post-delete state; ``/get_model_list`` represents the same
        check without manufacturing an alarming runtime error.
        """
        deadline = time.monotonic() + self._service_timeout_sec
        consecutive_absent = 0
        request = GetModelList.Request()
        while True:
            self._ensure_not_stopping()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ToolsetSupervisorError(
                    f"Gazebo did not remove robot '{self._robot_name}' before timeout"
                )
            response = self._call_service(
                self._get_model_list_client,
                request,
                "verify robot entity removal",
                timeout_sec=remaining,
            )
            if not bool(getattr(response, "success", False)):
                raise ToolsetSupervisorError(
                    "Gazebo could not list models while verifying removal of "
                    f"robot '{self._robot_name}'"
                )
            model_names = {
                str(name) for name in getattr(response, "model_names", ())
            }
            if self._robot_name in model_names:
                consecutive_absent = 0
            else:
                consecutive_absent += 1
                if consecutive_absent >= ENTITY_REMOVAL_STABLE_CYCLES:
                    return
            if time.monotonic() >= deadline:
                raise ToolsetSupervisorError(
                    f"Gazebo did not remove robot '{self._robot_name}' before timeout"
                )
            time.sleep(POLL_INTERVAL_SEC)

    def _call_service(
        self,
        client: Any,
        request: Any,
        label: str,
        *,
        timeout_sec: float | None = None,
    ) -> Any:
        """Call one ROS service without retaining an unresponsive request."""
        request_timeout_sec = (
            self._service_timeout_sec
            if timeout_sec is None
            else _finite_positive(timeout_sec, "service request timeout")
        )
        deadline = time.monotonic() + request_timeout_sec
        while not self._client_ready(client):
            self._ensure_not_stopping()
            if time.monotonic() >= deadline:
                raise ToolsetSupervisorError(f"{label} service is unavailable")
            time.sleep(POLL_INTERVAL_SEC)
        self._ensure_not_stopping()
        try:
            future = client.call_async(request)
        except Exception as error:  # noqa: BLE001
            raise ToolsetSupervisorError(
                f"failed to request {label}: {error}"
            ) from error
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        try:
            while not completed.is_set():
                self._ensure_not_stopping()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise ToolsetSupervisorError(f"{label} request timed out")
                completed.wait(timeout=min(POLL_INTERVAL_SEC, remaining))
            self._ensure_not_stopping()
            return future.result()
        except ToolsetSupervisorError:
            if not completed.is_set():
                self._remove_pending_service_request(client, future)
            raise
        except Exception as error:  # noqa: BLE001
            raise ToolsetSupervisorError(f"{label} failed: {error}") from error

    @staticmethod
    def _remove_pending_service_request(client: Any, future: Any) -> None:
        """Best-effort removal for a request whose server never responded."""
        remove = getattr(client, "remove_pending_request", None)
        if not callable(remove):
            return
        try:
            remove(future)
        except Exception:  # noqa: BLE001 - the client may already be destroyed
            pass

    @staticmethod
    def _client_ready(client: Any) -> bool:
        try:
            return bool(client.service_is_ready())
        except Exception:  # noqa: BLE001
            return False

    def _wait_for_robot_ready(self, toolset: str) -> None:
        target = normalize_toolset(toolset)
        required_actions = ARM_ACTIONS + TOOLSET_ACTIONS[target]
        required_joints = ARM_JOINTS + TOOLSET_JOINTS[target]
        started = time.monotonic()
        deadline = started + self._ready_timeout_sec
        last_unavailable: list[str] = []
        last_missing: list[str] = []
        last_report = started - 5.0
        while time.monotonic() < deadline:
            self._ensure_not_stopping()
            with self._lock:
                child = self._child_process
                received = self._latest_joint_received_monotonic
                positions = dict(self._latest_joint_positions)
            if child is None:
                raise ToolsetSupervisorError("robot child is absent during readiness check")
            return_code = child.poll()
            if return_code is not None:
                raise ToolsetSupervisorError(
                    f"robot child exited before ready (code {return_code})"
                )
            last_unavailable = [
                label
                for label, action_name in required_actions
                if not self._action_ready(action_name)
            ]
            if self._require_nav2:
                if not self._action_client_ready(self._navigation_client):
                    last_unavailable.append("Nav2 navigation")
                nav2_active, nav2_detail = self._nav2_lifecycle_is_active()
                if not nav2_active:
                    last_unavailable.append(nav2_detail)
            if self._require_cabinet_action:
                for action_name, client in self._cabinet_action_clients.items():
                    if not self._action_client_ready(client):
                        last_unavailable.append(
                            f"cabinet operation ({action_name})"
                        )
            last_missing = [
                name for name in required_joints if name not in positions
            ]
            fresh = received is not None and received >= started
            if not last_unavailable and not last_missing and fresh:
                return
            now = time.monotonic()
            if now - last_report >= 5.0:
                details: list[str] = []
                if last_unavailable:
                    details.append("actions=" + ", ".join(last_unavailable))
                if last_missing:
                    details.append("joints=" + ", ".join(last_missing))
                if not fresh:
                    details.append("no fresh joint-state sample")
                self.get_logger().info(
                    "Waiting for toolset %s readiness: %s"
                    % (target, "; ".join(details))
                )
                last_report = now
            time.sleep(POLL_INTERVAL_SEC)
        detail = []
        if last_unavailable:
            detail.append("actions unavailable: " + ", ".join(last_unavailable))
        if last_missing:
            detail.append("joint state missing: " + ", ".join(last_missing))
        if not detail:
            detail.append("no fresh joint-state sample received")
        raise ToolsetSupervisorError("; ".join(detail))

    def _action_ready(self, action_name: str) -> bool:
        client = self._trajectory_clients[action_name]
        return self._action_client_ready(client)

    @staticmethod
    def _action_client_ready(client: Any) -> bool:
        try:
            return bool(client.server_is_ready())
        except Exception:  # noqa: BLE001
            return False

    def _nav2_lifecycle_is_active(self) -> tuple[bool, str]:
        """Return whether this child stack's Nav2 lifecycle manager is active.

        Nav2 can create ``/navigate_to_pose`` before all managed lifecycle
        nodes are active.  Its standard Trigger endpoint is the authoritative
        readiness signal.  The probe client is replaced for every robot child,
        so a queued reply from an old toolset cannot make its replacement look
        ready.
        """
        now = time.monotonic()
        with self._lock:
            if now < self._next_nav2_lifecycle_probe_monotonic:
                return (
                    self._nav2_lifecycle_active,
                    self._nav2_lifecycle_status_detail,
                )
            client = self._nav2_lifecycle_active_client
            client_generation = self._nav2_lifecycle_client_generation
            self._next_nav2_lifecycle_probe_monotonic = (
                now + NAV2_LIFECYCLE_PROBE_INTERVAL_SEC
            )
        if not self._client_ready(client):
            return self._set_nav2_lifecycle_probe_result(
                client,
                client_generation,
                False,
                "Nav2 lifecycle manager is unavailable",
            )

        try:
            response = self._call_service(
                client,
                Trigger.Request(),
                "Nav2 lifecycle active",
                timeout_sec=min(
                    self._service_timeout_sec,
                    NAV2_LIFECYCLE_REQUEST_TIMEOUT_SEC,
                ),
            )
        except ToolsetSupervisorError as error:
            return self._set_nav2_lifecycle_probe_result(
                client,
                client_generation,
                False,
                f"Nav2 lifecycle manager is unavailable ({error})",
            )

        active = bool(getattr(response, "success", False))
        detail = str(getattr(response, "message", "")).strip()
        if active:
            status_detail = "Nav2 lifecycle manager"
        elif detail:
            status_detail = (
                f"Nav2 lifecycle manager is not active ({detail})"
            )
        else:
            status_detail = "Nav2 lifecycle manager is not active"
        return self._set_nav2_lifecycle_probe_result(
            client,
            client_generation,
            active,
            status_detail,
        )

    def _set_nav2_lifecycle_probe_result(
        self,
        client: Any,
        client_generation: int,
        active: bool,
        detail: str,
    ) -> tuple[bool, str]:
        """Commit a readiness result only when it belongs to this child."""
        with self._lock:
            if (
                client is not self._nav2_lifecycle_active_client
                or client_generation != self._nav2_lifecycle_client_generation
            ):
                return False, "Nav2 lifecycle manager changed during probe"
            self._nav2_lifecycle_active = active
            self._nav2_lifecycle_status_detail = detail
            return active, detail

    def _reset_nav2_lifecycle_probe_locked(self) -> None:
        """Bind Nav2 readiness to the newly launched robot child.

        The caller holds ``_lock`` and has stopped the old child before this
        call.  Destroying the old client also drops any pending request routed
        to the retiring lifecycle manager.
        """
        previous_client = self._nav2_lifecycle_active_client
        if previous_client is not None:
            try:
                self.destroy_client(previous_client)
            except Exception as error:  # noqa: BLE001 - cleanup must not block swap
                self.get_logger().warning(
                    "Could not destroy the previous Nav2 lifecycle client: "
                    f"{error}"
                )
        self._nav2_lifecycle_active_client = self.create_client(
            Trigger,
            DEFAULT_NAV2_LIFECYCLE_ACTIVE_SERVICE,
        )
        self._nav2_lifecycle_client_generation += 1
        self._nav2_lifecycle_active = False
        self._next_nav2_lifecycle_probe_monotonic = 0.0
        self._nav2_lifecycle_status_detail = (
            "Nav2 lifecycle manager has not confirmed active"
        )

    def _joint_state_callback(self, message: JointState) -> None:
        positions: dict[str, float] = {}
        for index, raw_name in enumerate(message.name):
            if index >= len(message.position):
                continue
            try:
                value = float(message.position[index])
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                positions[str(raw_name)] = value
        with self._lock:
            self._latest_joint_positions = positions
            self._latest_joint_received_monotonic = time.monotonic()

    def _monitor_child(self) -> None:
        with self._lock:
            if self._stopping or self._fatal_error is not None:
                return
            child = self._child_process
            state = str(self._status.get("state", ""))
            intentional = self._intentional_child_stop
        if child is None or child.poll() is None or intentional:
            return
        if state in {"starting", "switching"}:
            # The active worker reports the failure and attempts rollback.
            return
        message = f"Robot child exited unexpectedly with code {child.returncode}."
        self.get_logger().error(message)
        with self._lock:
            self._fatal_error = message
            self._set_status_locked(
                state="failed",
                stage="robot_child_exited",
                ready=False,
                target_toolset=None,
                operation_id=None,
                last_error=message,
                message=message,
            )
        try:
            self._stop_child_process(child)
        except ToolsetSupervisorError as error:
            self.get_logger().warning(
                f"Failed to stop residual robot child group: {error}"
            )
        self.get_logger().error(
            "Keeping the Gazebo world alive after the robot child exited; "
            "restart the system to relaunch the robot."
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="World-preserving XCZS end-effector toolset supervisor",
    )
    parser.add_argument("--toolset", default="A", help="initial toolset A/B")
    parser.add_argument("--robot-name", default="xczs_inspection_robot")
    parser.add_argument(
        "--launch-package",
        default="xczs_inspection_robot_bringup",
    )
    parser.add_argument(
        "--launch-file",
        default="inspection_robot.launch.py",
    )
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="static robot-child launch argument in name:=value form; repeatable",
    )
    parser.add_argument(
        "--initial-spawn-cabinet",
        choices=("true", "false"),
        default="true",
    )
    parser.add_argument(
        "--cabinet-bringup",
        choices=("true", "false"),
        default="true",
    )
    parser.add_argument("--require-nav2", action="store_true")
    parser.add_argument("--require-cabinet-action", action="store_true")
    parser.add_argument(
        "--cabinet-action",
        action="append",
        default=[],
        help="absolute namespaced cabinet OperateCabinetControl action; repeatable",
    )
    parser.add_argument(
        "--joint-state-topic",
        default=DEFAULT_JOINT_STATE_TOPIC,
    )
    parser.add_argument("--switch-service", default=DEFAULT_SWITCH_SERVICE)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=DEFAULT_READY_TIMEOUT_SEC,
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=DEFAULT_SERVICE_TIMEOUT_SEC,
    )
    parser.add_argument(
        "--child-stop-timeout",
        type=float,
        default=DEFAULT_CHILD_STOP_TIMEOUT_SEC,
    )
    return parser


def _install_shutdown_signal_handlers(supervisor: ToolsetSupervisor) -> None:
    """Turn outer-launch TERM/INT into orderly child-stack shutdown.

    The supervisor's robot child deliberately runs in its own session so a
    replacement can signal it without touching the Gazebo world.  Consequently
    the shell's process-group TERM reaches this supervisor but not that child.
    Python's default SIGTERM action would exit before ``main``'s ``finally``
    block had a chance to stop it.  Requesting rclpy shutdown instead lets the
    spin loop return and the normal bounded ``supervisor.shutdown()`` path run.
    """
    requested = threading.Event()

    def _handle(signum: int, _frame: Any) -> None:
        if requested.is_set():
            return
        requested.set()
        try:
            signal_name = signal.Signals(signum).name
        except ValueError:
            signal_name = str(signum)
        supervisor.get_logger().info(
            f"Received {signal_name}; stopping the robot child cleanly."
        )
        supervisor.request_shutdown()
        if rclpy.ok(context=supervisor.context):
            rclpy.shutdown(context=supervisor.context)

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)


def main(args: list[str] | None = None) -> int:
    options = _parser().parse_args(args)
    # All supported CLI options belong to this script and have already been
    # parsed.  Do not let launch arguments such as ``use_sim_time:=true`` leak
    # into rcl as deprecated global remap rules.
    rclpy.init(args=[])
    supervisor: ToolsetSupervisor | None = None
    try:
        supervisor = ToolsetSupervisor(
            initial_toolset=options.toolset,
            robot_name=options.robot_name,
            launch_package=options.launch_package,
            launch_file=options.launch_file,
            launch_arguments=options.launch_arg,
            initial_spawn_cabinet=options.initial_spawn_cabinet == "true",
            cabinet_bringup=options.cabinet_bringup == "true",
            require_nav2=options.require_nav2,
            require_cabinet_action=options.require_cabinet_action,
            ready_timeout_sec=options.ready_timeout,
            service_timeout_sec=options.service_timeout,
            child_stop_timeout_sec=options.child_stop_timeout,
            joint_state_topic=options.joint_state_topic,
            switch_service=options.switch_service,
            status_topic=options.status_topic,
            cabinet_action_names=options.cabinet_action,
        )
        _install_shutdown_signal_handlers(supervisor)
        supervisor.start()
        rclpy.spin(supervisor)
        return 1 if supervisor.fatal_error is not None else 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:  # noqa: BLE001 - command-line fault boundary
        print(f"Toolset supervisor failed: {error}", file=sys.stderr)
        return 1
    finally:
        if supervisor is not None:
            supervisor.shutdown()
            supervisor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
