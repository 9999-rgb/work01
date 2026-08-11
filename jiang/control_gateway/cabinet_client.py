"""ROS 2 client for one namespaced simulated control cabinet.

The Web gateway creates one :class:`CabinetClient` for each configured
cabinet.  Keeping the catalog, dynamic state subscriptions and action
generation inside the instance prevents controls with the same ID in two
cabinets from sharing state or accepting one another's delayed callbacks.
"""

from __future__ import annotations

import copy
import math
import re
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from xczs_inspection_robot_control.action import OperateCabinetControl
from xczs_inspection_robot_control.msg import CabinetControl
from xczs_inspection_robot_control.msg import CabinetControlCatalog
from xczs_inspection_robot_control.msg import CabinetControlState


_CABINET_NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


class CabinetClientError(RuntimeError):
    """A request that cannot be submitted to a cabinet client."""

    def __init__(
        self,
        message: str,
        *,
        code: str = "invalid_request",
        status: int = 400,
        details: Optional[Mapping[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status = status
        self.details = dict(details or {})


class CabinetClient(Node):
    """Own the ROS interfaces and observable state for one cabinet."""

    COMMANDS = {
        "press": OperateCabinetControl.Goal.COMMAND_PRESS,
        "set_state": OperateCabinetControl.Goal.COMMAND_SET_STATE,
        "set_position": OperateCabinetControl.Goal.COMMAND_SET_POSITION,
        "toggle": OperateCabinetControl.Goal.COMMAND_TOGGLE,
    }
    COMMAND_NAMES = {value: key for key, value in COMMANDS.items()}
    COMMAND_SUPPORT = {
        OperateCabinetControl.Goal.COMMAND_PRESS: CabinetControl.SUPPORT_PRESS,
        OperateCabinetControl.Goal.COMMAND_SET_STATE: (
            CabinetControl.SUPPORT_SET_STATE
        ),
        OperateCabinetControl.Goal.COMMAND_SET_POSITION: (
            CabinetControl.SUPPORT_SET_POSITION
        ),
        OperateCabinetControl.Goal.COMMAND_TOGGLE: (
            CabinetControl.SUPPORT_TOGGLE
        ),
    }
    CONTROL_TYPES = frozenset(
        {
            CabinetControl.TYPE_BUTTON,
            CabinetControl.TYPE_KNOB,
            CabinetControl.TYPE_SWITCH,
            CabinetControl.TYPE_DOOR,
        }
    )
    ACTIVE_STATES = frozenset({"sending", "operating", "canceling"})
    DETENT_TOLERANCE = 0.035

    def __init__(
        self,
        cabinet_name: str,
        listener: Callable[[Dict[str, Any]], None],
        *,
        context: Optional[Context] = None,
        action_client_factory: Callable[..., Any] = ActionClient,
        reset_client_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        if (
            not isinstance(cabinet_name, str)
            or _CABINET_NAME_PATTERN.fullmatch(cabinet_name) is None
        ):
            raise ValueError(
                "cabinet_name must start with a letter and contain only "
                "letters, digits, and underscores."
            )
        if not callable(listener):
            raise TypeError("listener must be callable.")

        self.cabinet_name = cabinet_name
        self.interface_namespace = f"/xczs/cabinet/{cabinet_name}"
        super().__init__(
            f"xczs_cabinet_client_{cabinet_name}",
            namespace=self.interface_namespace,
            context=context,
        )

        self.action_name = f"{self.interface_namespace}/operate_cabinet_control"
        self.catalog_topic = f"{self.interface_namespace}/control_catalog"
        self.reset_service = f"{self.interface_namespace}/reset_controls"
        self._listener = listener
        self._lock = threading.RLock()
        self._catalog_generation = 0
        self._operation_generation = 0
        self._catalog_received = False
        self._controls: Dict[str, Dict[str, Any]] = {}
        self._control_states: Dict[str, Dict[str, Any]] = {}
        # ``rclpy.node.Node`` owns an internal ``_subscriptions`` list.
        # Keep the dynamic catalog subscriptions under a distinct name.
        self._control_subscriptions: Dict[str, Dict[str, Any]] = {}
        self._goal_handle: Any = None
        self._cancel_requested = False
        self._status: Dict[str, Any] = self._new_status()

        self._action_client = action_client_factory(
            self,
            OperateCabinetControl,
            self.action_name,
        )
        self._reset_client = (
            reset_client_factory(self, Trigger, self.reset_service)
            if reset_client_factory is not None
            else self.create_client(Trigger, self.reset_service)
        )
        self._transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._catalog_subscription = self.create_subscription(
            CabinetControlCatalog,
            self.catalog_topic,
            self._catalog_callback,
            self._transient_qos,
        )

    def snapshot_controls(self) -> Dict[str, Any]:
        """Return a detached snapshot of this cabinet's catalog and state."""
        with self._lock:
            operation_available = self._action_server_ready()
            reset_available = self._service_ready()
            controls = []
            for control_id, control in self._controls.items():
                value = dict(control)
                value.update(self._control_states.get(control_id, {}))
                controls.append(value)
            result = {
                "cabinet": self.cabinet_name,
                "namespace": self.interface_namespace,
                "catalog_received": self._catalog_received,
                "available": self._catalog_received and operation_available,
                "operation_available": operation_available,
                "reset_available": reset_available,
                "controls": controls,
            }
        return copy.deepcopy(result)

    def snapshot_status(self) -> Dict[str, Any]:
        """Return action/service availability and the latest operation state."""
        with self._lock:
            result = dict(self._status)
            result.update(
                {
                    "cabinet": self.cabinet_name,
                    "namespace": self.interface_namespace,
                    "catalog_received": self._catalog_received,
                    "operation_available": self._action_server_ready(),
                    "reset_available": self._service_ready(),
                    "active": self._status["state"] in self.ACTIVE_STATES,
                }
            )
        return copy.deepcopy(result)

    def submit_operation(
        self,
        control_id: str,
        command: Any,
        target_state: Optional[str] = None,
        target_position: Optional[float] = None,
        force: Optional[float] = None,
        navigate: bool = False,
    ) -> Dict[str, Any]:
        """Submit one catalog-valid operation to the cabinet action server.

        ``operable=false`` is a safety-policy hint, not a cached execution
        result.  The action server owns the real-time planning validation and
        returns its measured stage, path fraction and failure reason.
        """
        control_id, command_code, command_name = self._validate_request_shape(
            control_id,
            command,
            target_state,
            target_position,
            force,
            navigate,
        )
        normalized_state = target_state.strip() if target_state is not None else None
        normalized_position = (
            float(target_position) if target_position is not None else None
        )
        normalized_force = float(force) if force is not None else None

        with self._lock:
            if self._status["state"] in self.ACTIVE_STATES:
                raise CabinetClientError(
                    f"Cabinet {self.cabinet_name} already has an active operation.",
                    code="operation_active",
                    status=409,
                )
            self._operation_generation += 1
            generation = self._operation_generation
            control = self._controls.get(control_id)
            self._begin_operation_locked(
                generation,
                control_id,
                command_code,
                command_name,
                normalized_state,
                normalized_position,
                normalized_force,
                navigate,
            )

        local_failure = self._validate_against_catalog(
            control,
            control_id,
            command_code,
            normalized_state,
            normalized_position,
            normalized_force,
        )
        if local_failure is not None:
            code, message, details = local_failure
            terminal = self._finish_terminal(
                generation,
                "failed",
                message,
                error_code=None,
                failure_code=code,
                result=details,
            )
            return self._submission_response(terminal)

        assert control is not None
        is_button = (
            int(control["control_type"]) == CabinetControl.TYPE_BUTTON
        )
        requested_force = (
            normalized_force
            if is_button and normalized_force is not None
            else (
                float(control.get("default_force", 0.0))
                if is_button
                else 0.0
            )
        )
        with self._lock:
            if generation != self._operation_generation:
                return {"status": "superseded", "cabinet": self.cabinet_name}
            self._status["force"] = requested_force

        if not self._action_server_ready():
            terminal = self._finish_terminal(
                generation,
                "failed",
                "Cabinet operation action server is unavailable.",
                error_code=None,
                failure_code="not_ready",
            )
            return self._submission_response(terminal)

        goal = OperateCabinetControl.Goal()
        goal.control_id = control_id
        goal.command = command_code
        goal.target_state = normalized_state or ""
        goal.use_target_position = normalized_position is not None
        goal.target_position = normalized_position or 0.0
        goal.force = requested_force
        goal.navigate_to_staging_pose = navigate
        self._emit(
            self._feedback_event(
                generation,
                phase="sending",
                phase_code=None,
                progress=0.0,
                message="Sending the cabinet operation goal.",
            )
        )
        try:
            future = self._action_client.send_goal_async(
                goal,
                feedback_callback=lambda message: self._feedback_callback(
                    message,
                    generation,
                ),
            )
            future.add_done_callback(
                lambda completed: self._goal_response_callback(
                    completed,
                    generation,
                )
            )
        except Exception as error:  # noqa: BLE001
            terminal = self._finish_terminal(
                generation,
                "failed",
                f"Failed to send cabinet operation goal: {error}",
                error_code=None,
                failure_code="not_ready",
            )
            return self._submission_response(terminal)
        return {
            "status": "accepted",
            "cabinet": self.cabinet_name,
            "generation": generation,
            "control_id": control_id,
            "command": command_name,
            "target_state": normalized_state,
            "target_position": normalized_position,
            "force": requested_force,
            "navigate": navigate,
        }

    def cancel(self) -> Dict[str, Any]:
        """Request cancellation of this cabinet's active action goal."""
        with self._lock:
            if self._status["state"] not in self.ACTIVE_STATES:
                return {"status": "idle", "cabinet": self.cabinet_name}
            if self._status["state"] == "canceling":
                return {"status": "canceling", "cabinet": self.cabinet_name}
            self._cancel_requested = True
            self._status.update(
                {
                    "state": "canceling",
                    "message": "Canceling the cabinet operation.",
                    "updated_at": time.time(),
                }
            )
            generation = self._operation_generation
            goal_handle = self._goal_handle

        self._emit(
            self._feedback_event(
                generation,
                phase="canceling",
                phase_code=None,
                progress=float(self.snapshot_status().get("progress", 0.0)),
                message="Canceling the cabinet operation.",
            )
        )
        if goal_handle is None:
            return {"status": "canceling", "cabinet": self.cabinet_name}
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._cancel_callback(
                    completed,
                    generation,
                    goal_handle,
                )
            )
        except Exception as error:  # noqa: BLE001
            with self._lock:
                if generation == self._operation_generation:
                    self._cancel_requested = False
                    self._status.update(
                        {
                            "state": "operating",
                            "message": f"Cancellation request failed: {error}",
                            "updated_at": time.time(),
                        }
                    )
            raise CabinetClientError(
                f"Cancellation request failed: {error}",
                code="cancel_failed",
                status=503,
            ) from error
        return {"status": "canceling", "cabinet": self.cabinet_name}

    def reset(self, timeout_sec: float = 5.0) -> Dict[str, Any]:
        """Reset this cabinet's simulated controls while no action is active."""
        with self._lock:
            if self._status["state"] in self.ACTIVE_STATES:
                raise CabinetClientError(
                    "Cannot reset a cabinet while an operation is active.",
                    code="operation_active",
                    status=409,
                )
        if not self._service_ready():
            raise CabinetClientError(
                "Cabinet reset service is unavailable.",
                code="not_ready",
                status=503,
            )
        completed = threading.Event()
        try:
            future = self._reset_client.call_async(Trigger.Request())
            future.add_done_callback(lambda _future: completed.set())
        except Exception as error:  # noqa: BLE001
            raise CabinetClientError(
                f"Failed to request cabinet reset: {error}",
                code="reset_failed",
                status=503,
            ) from error
        if not completed.wait(max(0.1, float(timeout_sec))):
            raise CabinetClientError(
                "Cabinet reset request timed out.",
                code="reset_timeout",
                status=503,
            )
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            raise CabinetClientError(
                f"Cabinet reset failed: {error}",
                code="reset_failed",
                status=503,
            ) from error
        if not bool(response.success):
            raise CabinetClientError(
                str(response.message) or "Cabinet reset was rejected.",
                code="reset_rejected",
                status=409,
            )
        return {
            "status": "reset",
            "cabinet": self.cabinet_name,
            "message": str(response.message) or "Cabinet controls reset.",
        }

    def _catalog_callback(self, message: CabinetControlCatalog) -> None:
        controls: Dict[str, Dict[str, Any]] = {}
        for entry in message.controls:
            parsed = self._parse_control(entry)
            if parsed is None or parsed["control_id"] in controls:
                continue
            controls[parsed["control_id"]] = parsed
        if not controls:
            self.get_logger().warning(
                f"Ignored an empty or invalid catalog for {self.cabinet_name}."
            )
            return

        with self._lock:
            self._catalog_generation += 1
            generation = self._catalog_generation
            old_subscriptions = self._control_subscriptions
            old_controls = self._controls
            old_states = self._control_states
            self._control_subscriptions = {}
            self._controls = controls
            self._control_states = {
                control_id: (
                    dict(old_states[control_id])
                    if (
                        control_id in old_states
                        and self._control_signature(old_controls.get(control_id))
                        == self._control_signature(controls[control_id])
                    )
                    else self._new_control_state()
                )
                for control_id in controls
            }
            for control_id, control in controls.items():
                self._control_states[control_id].update(
                    {
                        "control_type": control["control_type"],
                        "type": control["control_type"],
                    }
                )
            self._catalog_received = True

        self._destroy_subscription_map(old_subscriptions)
        for control in controls.values():
            subscriptions = self._create_control_subscriptions(
                control,
                generation,
            )
            with self._lock:
                if generation == self._catalog_generation:
                    self._control_subscriptions[
                        control["control_id"]
                    ] = subscriptions
                    subscriptions = {}
            self._destroy_subscription_map({"stale": subscriptions})

    def _create_control_subscriptions(
        self,
        control: Mapping[str, Any],
        generation: int,
    ) -> Dict[str, Any]:
        control_id = str(control["control_id"])
        subscriptions: Dict[str, Any] = {}
        state_topic = str(control.get("resolved_state_topic", ""))
        joint_topic = str(control.get("resolved_joint_state_topic", ""))
        pressed_topic = str(control.get("resolved_pressed_topic", ""))
        try:
            if state_topic:
                subscriptions["state"] = self.create_subscription(
                    CabinetControlState,
                    state_topic,
                    lambda message: self._state_callback(
                        control_id,
                        state_topic,
                        generation,
                        message,
                    ),
                    self._transient_qos,
                )
            if joint_topic and control.get("joint_name"):
                joint_name = str(control["joint_name"])
                subscriptions["joint"] = self.create_subscription(
                    JointState,
                    joint_topic,
                    lambda message: self._joint_callback(
                        control_id,
                        joint_name,
                        joint_topic,
                        generation,
                        message,
                    ),
                    qos_profile_sensor_data,
                )
            if pressed_topic:
                subscriptions["pressed"] = self.create_subscription(
                    Bool,
                    pressed_topic,
                    lambda message: self._pressed_callback(
                        control_id,
                        pressed_topic,
                        generation,
                        message,
                    ),
                    self._transient_qos,
                )
        except Exception:  # noqa: BLE001
            self._destroy_subscription_map({"partial": subscriptions})
            raise
        return subscriptions

    def _state_callback(
        self,
        control_id: str,
        topic: str,
        generation: int,
        message: CabinetControlState,
    ) -> None:
        if str(message.control_id).strip() not in {"", control_id}:
            return
        values = (
            float(message.position),
            float(message.velocity),
            float(message.effort),
            float(message.normalized_position),
        )
        if not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            control = self._controls.get(control_id)
            if (
                generation != self._catalog_generation
                or control is None
                or control.get("resolved_state_topic") != topic
                or int(message.control_type) != int(control["control_type"])
            ):
                return
            state = self._control_states.setdefault(
                control_id,
                self._new_control_state(),
            )
            state.update(
                {
                    "valid": bool(message.valid),
                    "current_position": values[0],
                    "velocity": values[1],
                    "effort": values[2],
                    "normalized_position": values[3],
                    "current_state": str(message.state_id),
                    "activated": bool(message.activated),
                    "in_motion": bool(message.in_motion),
                    "transition_sequence": int(message.transition_sequence),
                    "state_updated_at": time.time(),
                }
            )

    def _joint_callback(
        self,
        control_id: str,
        joint_name: str,
        topic: str,
        generation: int,
        message: JointState,
    ) -> None:
        try:
            index = message.name.index(joint_name)
            position = float(message.position[index])
        except (ValueError, IndexError, TypeError):
            return
        if not math.isfinite(position):
            return
        with self._lock:
            control = self._controls.get(control_id)
            if (
                generation != self._catalog_generation
                or control is None
                or control.get("joint_name") != joint_name
                or control.get("resolved_joint_state_topic") != topic
            ):
                return
            self._control_states.setdefault(
                control_id,
                self._new_control_state(),
            ).update(
                {
                    "current_position": position,
                    "state_updated_at": time.time(),
                }
            )

    def _pressed_callback(
        self,
        control_id: str,
        topic: str,
        generation: int,
        message: Bool,
    ) -> None:
        with self._lock:
            control = self._controls.get(control_id)
            if (
                generation != self._catalog_generation
                or control is None
                or control.get("resolved_pressed_topic") != topic
            ):
                return
            self._control_states.setdefault(
                control_id,
                self._new_control_state(),
            ).update(
                {
                    "activated": bool(message.data),
                    "state_updated_at": time.time(),
                }
            )

    def _goal_response_callback(self, future: Any, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            with self._lock:
                canceled = (
                    generation == self._operation_generation
                    and self._cancel_requested
                )
            self._finish_terminal(
                generation,
                "canceled" if canceled else "failed",
                (
                    "Cabinet operation was canceled before goal acceptance."
                    if canceled
                    else f"Failed to submit cabinet operation: {error}"
                ),
                error_code=None,
                failure_code="canceled" if canceled else "not_ready",
            )
            return
        with self._lock:
            if generation != self._operation_generation:
                return
            canceled = self._cancel_requested
            if bool(goal_handle.accepted):
                self._goal_handle = goal_handle
                self._status.update(
                    {
                        "state": "canceling" if canceled else "operating",
                        "message": (
                            "Canceling the accepted cabinet operation."
                            if canceled
                            else "Cabinet operation is running."
                        ),
                        "updated_at": time.time(),
                    }
                )
        if not bool(goal_handle.accepted):
            self._finish_terminal(
                generation,
                "canceled" if canceled else "failed",
                (
                    "Cabinet operation was canceled before goal acceptance."
                    if canceled
                    else "Cabinet operation action server rejected the goal."
                ),
                error_code=None,
                failure_code=(
                    "canceled"
                    if canceled
                    else "rejected"
                ),
            )
            return
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed: self._result_callback(
                    completed,
                    generation,
                    goal_handle,
                )
            )
            if canceled:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda completed: self._cancel_callback(
                        completed,
                        generation,
                        goal_handle,
                    )
                )
        except Exception as error:  # noqa: BLE001
            self._finish_terminal(
                generation,
                "failed",
                f"Cabinet operation result channel failed: {error}",
                error_code=None,
                failure_code="result_channel_failed",
                expected_goal_handle=goal_handle,
            )

    def _feedback_callback(self, feedback_message: Any, generation: int) -> None:
        feedback = feedback_message.feedback
        phase_code = int(feedback.phase)
        event = self._feedback_event(
            generation,
            phase=self._feedback_phase_name(phase_code),
            phase_code=phase_code,
            progress=float(feedback.progress),
            current_position=float(feedback.current_position),
            target_position=float(feedback.target_position),
            current_state=str(feedback.current_state),
            message=str(feedback.message),
        )
        with self._lock:
            if (
                generation != self._operation_generation
                or self._status["state"] not in self.ACTIVE_STATES
            ):
                return
            self._status.update(
                {
                    key: value
                    for key, value in event.items()
                    if key
                    in {
                        "phase",
                        "phase_code",
                        "progress",
                        "current_position",
                        "target_position",
                        "current_state",
                        "message",
                    }
                }
            )
            self._status["updated_at"] = time.time()
        self._emit(event)

    def _result_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
    ) -> None:
        try:
            wrapped = future.result()
            result_message = wrapped.result
            action_status = int(wrapped.status)
            error_code = int(result_message.error_code)
            failure_code = self._error_code_name(error_code)
            result = {
                "initial_position": float(result_message.initial_position),
                "final_position": float(result_message.final_position),
                "peak_position": float(result_message.peak_position),
                "final_state": str(result_message.final_state),
                "requested_force": float(result_message.requested_force),
                "estimated_force": float(result_message.estimated_force),
                "button_triggered": bool(result_message.button_triggered),
                "validation_performed": bool(
                    getattr(result_message, "validation_performed", False)
                ),
                "operation_executed": bool(
                    getattr(result_message, "operation_executed", False)
                ),
                "diagnostic_stage": str(
                    getattr(result_message, "diagnostic_stage", "")
                ),
                "path_fraction": float(
                    getattr(result_message, "path_fraction", 0.0)
                ),
                "required_fraction": float(
                    getattr(result_message, "required_fraction", 0.0)
                ),
                "moveit_error_code": int(
                    getattr(result_message, "moveit_error_code", 0)
                ),
                "policy_reason": str(
                    getattr(result_message, "policy_reason", "")
                ),
            }
            if (
                action_status == GoalStatus.STATUS_CANCELED
                or failure_code == "canceled"
            ):
                outcome = "canceled"
                failure_code = "canceled"
            elif (
                action_status == GoalStatus.STATUS_SUCCEEDED
                and bool(result_message.success)
                and failure_code == "success"
            ):
                outcome = "success"
                failure_code = None
            else:
                outcome = "failed"
            message = str(result_message.message).strip() or (
                "Cabinet operation succeeded."
                if outcome == "success"
                else "Cabinet operation failed."
            )
        except Exception as error:  # noqa: BLE001
            outcome = "failed"
            message = f"Cabinet operation result failed: {error}"
            error_code = None
            failure_code = "result_channel_failed"
            result = {}
        self._finish_terminal(
            generation,
            outcome,
            message,
            error_code=error_code,
            failure_code=failure_code,
            result=result,
            expected_goal_handle=goal_handle,
        )

    def _cancel_callback(
        self,
        future: Any,
        generation: int,
        goal_handle: Any,
    ) -> None:
        try:
            accepted = bool(future.result().goals_canceling)
            error_message = ""
        except Exception as error:  # noqa: BLE001
            accepted = False
            error_message = str(error)
        with self._lock:
            if (
                generation != self._operation_generation
                or self._goal_handle is not goal_handle
                or self._status["state"] not in self.ACTIVE_STATES
            ):
                return
            if accepted:
                message = "Cabinet operation cancellation was accepted."
            else:
                self._cancel_requested = False
                self._status["state"] = "operating"
                message = (
                    f"Cabinet operation cancellation failed: {error_message}"
                    if error_message
                    else "Cabinet action server rejected cancellation."
                )
            self._status.update({"message": message, "updated_at": time.time()})
            progress = float(self._status.get("progress", 0.0))
        self._emit(
            self._feedback_event(
                generation,
                phase="canceling" if accepted else "operating",
                phase_code=None,
                progress=progress,
                message=message,
            )
        )

    def _finish_terminal(
        self,
        generation: int,
        outcome: str,
        message: str,
        *,
        error_code: Optional[int],
        failure_code: Optional[str],
        result: Optional[Mapping[str, Any]] = None,
        expected_goal_handle: Any = None,
    ) -> Dict[str, Any]:
        result_data = dict(result or {})
        with self._lock:
            if generation != self._operation_generation:
                return {}
            if (
                expected_goal_handle is not None
                and expected_goal_handle is not self._goal_handle
            ):
                return {}
            now = time.time()
            self._goal_handle = None
            self._cancel_requested = False
            self._status.update(
                {
                    "state": outcome,
                    "message": message,
                    "success": outcome == "success",
                    "failure_code": failure_code,
                    "error_code": error_code,
                    "result": result_data,
                    "progress": (
                        1.0 if outcome == "success" else self._status["progress"]
                    ),
                    "updated_at": now,
                }
            )
            event = {
                "event": "terminal",
                "cabinet": self.cabinet_name,
                "generation": generation,
                "timestamp": now,
                "outcome": outcome,
                "success": outcome == "success",
                "failure_code": failure_code,
                "error_code": error_code,
                "message": message,
                "result": result_data,
            }
        self._emit(event)
        return event

    def _feedback_event(
        self,
        generation: int,
        *,
        phase: str,
        phase_code: Optional[int],
        progress: float,
        message: str,
        current_position: Optional[float] = None,
        target_position: Optional[float] = None,
        current_state: str = "",
    ) -> Dict[str, Any]:
        return {
            "event": "feedback",
            "cabinet": self.cabinet_name,
            "generation": generation,
            "timestamp": time.time(),
            "phase": phase,
            "phase_code": phase_code,
            "progress": progress,
            "current_position": current_position,
            "target_position": target_position,
            "current_state": current_state,
            "message": message,
        }

    def _emit(self, event: Dict[str, Any]) -> None:
        if not event:
            return
        try:
            self._listener(copy.deepcopy(event))
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Cabinet event listener failed: {error}")

    def _begin_operation_locked(
        self,
        generation: int,
        control_id: str,
        command_code: int,
        command_name: str,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
        navigate: bool,
    ) -> None:
        self._goal_handle = None
        self._cancel_requested = False
        self._status = {
            "state": "sending",
            "message": "Sending the cabinet operation goal.",
            "generation": generation,
            "control_id": control_id,
            "command": command_name,
            "command_code": command_code,
            "target_state": target_state,
            "target_position": target_position,
            "force": force,
            "navigate": navigate,
            "phase": "sending",
            "phase_code": None,
            "progress": 0.0,
            "current_position": None,
            "current_state": "",
            "success": None,
            "failure_code": None,
            "error_code": None,
            "result": {},
            "updated_at": time.time(),
        }

    def _validate_request_shape(
        self,
        control_id: Any,
        command: Any,
        target_state: Any,
        target_position: Any,
        force: Any,
        navigate: Any,
    ) -> Tuple[str, int, str]:
        if not isinstance(control_id, str) or not control_id.strip():
            raise CabinetClientError("control_id must be a non-empty string.")
        if isinstance(command, str):
            command_name = command.strip().lower().replace("-", "_")
            if command_name not in self.COMMANDS:
                raise CabinetClientError(
                    "command must be press, set_state, set_position, or toggle."
                )
            command_code = self.COMMANDS[command_name]
        elif (
            isinstance(command, int)
            and not isinstance(command, bool)
            and command in self.COMMAND_NAMES
        ):
            command_code = command
            command_name = self.COMMAND_NAMES[command]
        else:
            raise CabinetClientError(
                "command must be press, set_state, set_position, or toggle."
            )
        if target_state is not None and (
            not isinstance(target_state, str) or not target_state.strip()
        ):
            raise CabinetClientError(
                "target_state must be a non-empty string when provided."
            )
        for name, value in (
            ("target_position", target_position),
            ("force", force),
        ):
            if value is None:
                continue
            if isinstance(value, bool):
                raise CabinetClientError(f"{name} must be a finite number.")
            try:
                converted = float(value)
            except (TypeError, ValueError) as error:
                raise CabinetClientError(
                    f"{name} must be a finite number."
                ) from error
            if not math.isfinite(converted):
                raise CabinetClientError(f"{name} must be a finite number.")
        if not isinstance(navigate, bool):
            raise CabinetClientError("navigate must be a boolean.")
        return control_id.strip(), command_code, command_name

    def _validate_against_catalog(
        self,
        control: Optional[Mapping[str, Any]],
        control_id: str,
        command_code: int,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Optional[Tuple[str, str, Dict[str, Any]]]:
        with self._lock:
            catalog_received = self._catalog_received
        if not catalog_received:
            return (
                "not_ready",
                "Cabinet control catalog is not available yet.",
                {},
            )
        if control is None:
            return (
                "invalid_control",
                f"Unsupported cabinet control: {control_id}.",
                {"control_id": control_id},
            )
        required_support = self.COMMAND_SUPPORT[command_code]
        if int(control.get("supported_commands", 0)) & required_support == 0:
            return (
                "unsupported_command",
                f"Control {control_id} does not support command "
                f"{self.COMMAND_NAMES[command_code]}.",
                {},
            )
        if command_code == OperateCabinetControl.Goal.COMMAND_SET_STATE:
            if target_state is None:
                return (
                    "invalid_target",
                    "target_state is required for set_state.",
                    {},
                )
            state_ids = [str(value) for value in control.get("state_ids", [])]
            if state_ids and target_state not in state_ids:
                return (
                    "invalid_target",
                    f"target_state must be one of: {', '.join(state_ids)}.",
                    {"state_ids": state_ids},
                )
        if command_code == OperateCabinetControl.Goal.COMMAND_SET_POSITION:
            if target_position is None:
                return (
                    "invalid_target",
                    "target_position is required for set_position.",
                    {},
                )
            minimum = float(control["min_position"])
            maximum = float(control["max_position"])
            if target_position < minimum or target_position > maximum:
                return (
                    "invalid_target",
                    f"target_position is outside [{minimum}, {maximum}].",
                    {"min_position": minimum, "max_position": maximum},
                )
            detents = [float(value) for value in control.get("state_positions", [])]
            if not detents or min(
                abs(target_position - item) for item in detents
            ) > self.DETENT_TOLERANCE:
                return (
                    "invalid_target",
                    "target_position does not match a configured detent.",
                    {"state_positions": detents},
                )
        if (
            int(control["control_type"]) == CabinetControl.TYPE_BUTTON
            and force is not None
            and (
                force <= 0.0
                or (
                    float(control.get("max_force", 0.0)) > 0.0
                    and force > float(control["max_force"])
                )
            )
        ):
            return (
                "invalid_force",
                "Button force must be greater than zero and no larger than "
                f"{float(control.get('max_force', 0.0))} N.",
                {
                    "requested_force": force,
                    "max_force": float(control.get("max_force", 0.0)),
                },
            )
        return None

    def _parse_control(self, entry: Any) -> Optional[Dict[str, Any]]:
        control_id = str(entry.control_id).strip()
        control_type = int(entry.control_type)
        if not control_id or control_type not in self.CONTROL_TYPES:
            return None
        minimum = float(getattr(entry, "min_position", 0.0))
        maximum = float(getattr(entry, "max_position", 0.0))
        force_values = (
            float(getattr(entry, "default_force", 0.0)),
            float(getattr(entry, "min_trigger_force", 0.0)),
            float(getattr(entry, "max_force", 0.0)),
        )
        if (
            not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum > maximum
            or not all(math.isfinite(value) for value in force_values)
        ):
            return None
        state_ids = [str(value) for value in getattr(entry, "state_ids", [])]
        state_labels = [str(value) for value in getattr(entry, "state_labels", [])]
        state_positions = [
            float(value) for value in getattr(entry, "state_positions", [])
        ]
        if (
            len(state_labels) not in {0, len(state_ids)}
            or len(state_positions) not in {0, len(state_ids)}
            or not all(math.isfinite(value) for value in state_positions)
        ):
            return None
        state_topic = str(getattr(entry, "state_topic", "")).strip()
        joint_topic = str(getattr(entry, "joint_state_topic", "")).strip()
        pressed_topic = str(getattr(entry, "pressed_topic", "")).strip()
        resolved_topics = {
            "resolved_state_topic": self._resolve_control_topic(state_topic),
            "resolved_joint_state_topic": self._resolve_control_topic(joint_topic),
            "resolved_pressed_topic": self._resolve_control_topic(pressed_topic),
        }
        operable = bool(getattr(entry, "operable", True))
        if operable and not resolved_topics["resolved_state_topic"] and not (
            str(getattr(entry, "joint_name", "")).strip()
            and resolved_topics["resolved_joint_state_topic"]
        ):
            return None
        supported = int(getattr(entry, "supported_commands", 0))
        if supported == 0 and control_type == CabinetControl.TYPE_BUTTON:
            supported = CabinetControl.SUPPORT_PRESS
        return {
            "control_id": control_id,
            "display_name": str(getattr(entry, "display_name", "")).strip()
            or control_id,
            "control_type": control_type,
            "type": control_type,
            "joint_name": str(getattr(entry, "joint_name", "")).strip(),
            "joint_state_topic": joint_topic,
            "pressed_topic": pressed_topic,
            "state_topic": state_topic,
            **resolved_topics,
            "supported_commands": supported,
            "unit": str(getattr(entry, "unit", "")),
            "min_position": minimum,
            "max_position": maximum,
            "state_ids": state_ids,
            "state_labels": state_labels,
            "state_positions": state_positions,
            "requires_grasp": bool(getattr(entry, "requires_grasp", False)),
            "operable": operable,
            "unavailable_reason": str(
                getattr(entry, "unavailable_reason", "")
            ).strip(),
            "default_force": force_values[0],
            "min_trigger_force": force_values[1],
            "max_force": force_values[2],
        }

    @staticmethod
    def _control_signature(
        control: Optional[Mapping[str, Any]],
    ) -> Optional[Tuple[Any, ...]]:
        if control is None:
            return None
        return (
            control.get("control_type"),
            control.get("joint_name"),
            control.get("resolved_joint_state_topic"),
            control.get("resolved_pressed_topic"),
            control.get("resolved_state_topic"),
        )

    def _resolve_control_topic(self, topic: str) -> str:
        if not topic:
            return ""
        if topic.startswith("/"):
            normalized = "/" + "/".join(part for part in topic.split("/") if part)
            if not (
                normalized == self.interface_namespace
                or normalized.startswith(f"{self.interface_namespace}/")
            ):
                self.get_logger().warning(
                    f"Ignoring cross-cabinet topic '{topic}' in catalog for "
                    f"{self.cabinet_name}."
                )
                return ""
            return normalized
        return f"{self.interface_namespace}/{topic.lstrip('/')}"

    @classmethod
    def _error_code_name(cls, code: int) -> str:
        names = (
            ("SUCCESS", "success"),
            ("INVALID_CONTROL", "invalid_control"),
            ("UNSUPPORTED_COMMAND", "unsupported_command"),
            ("NOT_READY", "not_ready"),
            ("NAVIGATION_FAILED", "navigation_failed"),
            ("PLANNING_FAILED", "planning_failed"),
            ("EXECUTION_FAILED", "execution_failed"),
            ("GRASP_FAILED", "grasp_failed"),
            ("TARGET_NOT_REACHED", "target_not_reached"),
            ("RELEASE_FAILED", "release_failed"),
            ("CANCELED", "canceled"),
            ("INTERNAL_ERROR", "internal_error"),
            ("INVALID_FORCE", "invalid_force"),
            ("INSUFFICIENT_FORCE", "insufficient_force"),
            ("UNREACHABLE", "unreachable"),
            ("CONTACT_DETECTION_TIMEOUT", "contact_detection_timeout"),
            ("RESOURCE_BUSY", "resource_busy"),
            ("LEASE_LOST", "lease_lost"),
        )
        for constant, name in names:
            if getattr(OperateCabinetControl.Result, constant, None) == code:
                return name
        return "unknown_error"

    @classmethod
    def _feedback_phase_name(cls, phase: int) -> str:
        names = (
            ("WAITING_FOR_SYSTEM", "waiting_for_system"),
            ("NAVIGATING", "navigating"),
            ("DOCKING", "docking"),
            ("MOVING_TO_READY", "moving_to_ready"),
            ("APPROACHING", "approaching"),
            ("GRASPING", "grasping"),
            ("MANIPULATING", "manipulating"),
            ("VERIFYING", "verifying"),
            ("RELEASING", "releasing"),
            ("RETREATING", "retreating"),
        )
        for constant, name in names:
            if getattr(OperateCabinetControl.Feedback, constant, None) == phase:
                return name
        return "unknown"

    def _action_server_ready(self) -> bool:
        try:
            return bool(self._action_client.server_is_ready())
        except (AttributeError, RuntimeError):
            return False

    def _service_ready(self) -> bool:
        try:
            return bool(self._reset_client.service_is_ready())
        except (AttributeError, RuntimeError):
            return False

    def _destroy_subscription_map(
        self,
        groups: Mapping[str, Mapping[str, Any]],
    ) -> None:
        for subscriptions in groups.values():
            for subscription in subscriptions.values():
                if subscription is not None:
                    self.destroy_subscription(subscription)

    @staticmethod
    def _new_control_state() -> Dict[str, Any]:
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
        }

    @staticmethod
    def _new_status() -> Dict[str, Any]:
        return {
            "state": "idle",
            "message": "No cabinet operation has been sent.",
            "generation": 0,
            "control_id": "",
            "command": None,
            "command_code": None,
            "target_state": None,
            "target_position": None,
            "force": None,
            "navigate": False,
            "phase": None,
            "phase_code": None,
            "progress": 0.0,
            "current_position": None,
            "current_state": "",
            "success": None,
            "failure_code": None,
            "error_code": None,
            "result": {},
            "updated_at": time.time(),
        }

    @staticmethod
    def _submission_response(terminal: Mapping[str, Any]) -> Dict[str, Any]:
        return {
            "status": str(terminal.get("outcome", "failed")),
            "cabinet": terminal.get("cabinet"),
            "generation": terminal.get("generation"),
            "failure_code": terminal.get("failure_code"),
            "message": terminal.get("message"),
            "result": copy.deepcopy(terminal.get("result", {})),
        }
