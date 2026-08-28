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
from xczs_inspection_robot_interfaces.action import OperateCabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControl
from xczs_inspection_robot_interfaces.msg import CabinetControlCatalog
from xczs_inspection_robot_interfaces.msg import CabinetControlState


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
        failure_reason: Optional[str] = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status = status
        self.details = dict(details or {})
        self.failure_reason = (
            str(failure_reason).strip()
            if failure_reason is not None
            else None
        ) or None


class CabinetClient(Node):
    """Own the ROS interfaces and observable state for one cabinet."""

    COMMANDS = {
        "press": OperateCabinetControl.Goal.COMMAND_PRESS,
        "set_state": OperateCabinetControl.Goal.COMMAND_SET_STATE,
        "set_position": OperateCabinetControl.Goal.COMMAND_SET_POSITION,
        "toggle": OperateCabinetControl.Goal.COMMAND_TOGGLE,
    }
    COMMAND_NAMES = {value: key for key, value in COMMANDS.items()}
    CONTROL_TYPES = frozenset(
        {
            CabinetControl.TYPE_BUTTON,
            CabinetControl.TYPE_KNOB,
            CabinetControl.TYPE_SWITCH,
            CabinetControl.TYPE_DOOR,
        }
    )
    BUTTON_CONTROL_TYPE = CabinetControl.TYPE_BUTTON
    ARTICULATED_CONTROL_TYPES = frozenset(
        {
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
        toolset_status_provider: Optional[
            Callable[[], Mapping[str, Any]]
        ] = None,
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
        # The local Zenoh/DDS bridge can replay a transient catalog published
        # by the retiring robot child.  In managed A/B deployments the
        # supervisor status is therefore an independent freshness fence: a
        # catalog is usable only when its capability metadata agrees with the
        # supervisor's ready active_toolset.  Keep the provider optional so
        # legacy/static deployments and isolated unit tests retain their
        # historical behavior.
        self._toolset_status_provider = toolset_status_provider
        self._catalog_coherent = toolset_status_provider is None
        self._catalog_coherence_reason = (
            "Waiting for the toolset supervisor to report a ready active toolset."
            if toolset_status_provider is not None
            else ""
        )
        self._catalog_active_toolset: Optional[str] = None
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

    def _toolset_status_snapshot(self) -> Optional[Mapping[str, Any]]:
        """Read the supervisor status without allowing a bad provider to crash ROS."""
        provider = self._toolset_status_provider
        if provider is None:
            return None
        try:
            status = provider()
        except Exception as error:  # noqa: BLE001 - status is a safety fence
            self.get_logger().warning(
                f"Toolset status provider failed for {self.cabinet_name}: {error}"
            )
            return None
        return status if isinstance(status, Mapping) else None

    def _catalog_coherence_locked(
        self,
        controls: Optional[Mapping[str, Mapping[str, Any]]] = None,
    ) -> Tuple[bool, str, Optional[str]]:
        """Check that a catalog belongs to the ready active A/B child.

        The catalog message has no publisher generation.  Comparing every
        control's required toolset and compatibility bit with the supervisor
        status gives the gateway a deterministic fence against a transient
        stale DDS/Zenoh sample during a hot replacement.
        """
        if self._toolset_status_provider is None:
            return True, "", None
        status = self._toolset_status_snapshot()
        if status is None or status.get("managed") is not True:
            return (
                False,
                "Toolset supervisor status is unavailable; cabinet controls "
                "are locked.",
                None,
            )
        active = status.get("active_toolset")
        if isinstance(active, str):
            active = active.strip().upper()
        else:
            active = None
        state = status.get("state")
        if (
            state != "ready"
            or status.get("ready") is not True
            or active not in {"A", "B"}
        ):
            stage = str(status.get("stage") or state or "unknown")
            return (
                False,
                f"End-effector toolset is not ready (stage: {stage}); "
                "cabinet controls are locked until the fresh catalog arrives.",
                active,
            )

        candidate = self._controls if controls is None else controls
        if not candidate:
            return (
                False,
                "Cabinet control catalog is empty while the active toolset is ready.",
                active,
            )
        missing_metadata = [
            control_id
            for control_id, control in candidate.items()
            if not bool(control.get("capability_metadata_present", False))
        ]
        if missing_metadata:
            return (
                False,
                "Cabinet catalog has no toolset capability metadata; "
                "waiting for a fresh publisher from the active robot child.",
                active,
            )
        mismatches = []
        for control_id, control in candidate.items():
            required = str(control.get("required_toolset") or "").strip().upper()
            compatible = bool(control.get("toolset_compatible", False))
            expected = required in {"A", "B"} and required == active
            if required not in {"A", "B"} or compatible != expected:
                mismatches.append(str(control_id))
        if mismatches:
            preview = ", ".join(mismatches[:4])
            if len(mismatches) > 4:
                preview += ", ..."
            return (
                False,
                "Cabinet catalog does not match the active end-effector "
                f"toolset {active}; waiting for a fresh catalog "
                f"(controls: {preview}).",
                active,
            )
        return True, "", active

    def _refresh_catalog_coherence_locked(self) -> None:
        """Refresh the catalog fence just before exposing or using state."""
        coherent, reason, active = self._catalog_coherence_locked()
        self._catalog_coherent = coherent
        self._catalog_coherence_reason = reason
        self._catalog_active_toolset = active

    def _catalog_coherence_failure_locked(
        self,
    ) -> Optional[Tuple[str, str, int, Dict[str, Any]]]:
        """Return a structured failure for a stale/transitioning catalog.

        Keeping the status mapping in one place is important because both the
        read-only preflight and the actual submit path use the same fence.  A
        transition is a temporary conflict (409); a ready supervisor with a
        stale or incomplete catalog is an unavailable backend (503).
        """
        self._refresh_catalog_coherence_locked()
        if self._catalog_coherent:
            return None
        status = self._toolset_status_snapshot()
        state = status.get("state") if status is not None else None
        active = status.get("active_toolset") if status is not None else None
        if isinstance(active, str):
            active = active.strip().upper()
        code = (
            "toolset_transition"
            if (
                status is None
                or state != "ready"
                or status.get("ready") is not True
                or active not in {"A", "B"}
            )
            else "catalog_stale"
        )
        details: Dict[str, Any] = {
            "catalog_coherent": False,
            "catalog_active_toolset": self._catalog_active_toolset,
        }
        if status is not None:
            details["toolset_status"] = dict(status)
        return (
            code,
            self._catalog_coherence_reason
            or "Cabinet catalog is not synchronized with the active toolset.",
            409 if code == "toolset_transition" else 503,
            details,
        )

    def snapshot_controls(self) -> Dict[str, Any]:
        """Return a detached snapshot of this cabinet's catalog and state."""
        with self._lock:
            self._refresh_catalog_coherence_locked()
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
                "catalog_coherent": self._catalog_coherent,
                "catalog_active_toolset": self._catalog_active_toolset,
                "catalog_coherence_reason": self._catalog_coherence_reason,
                "available": (
                    self._catalog_received
                    and self._catalog_coherent
                    and operation_available
                ),
                "operation_available": operation_available,
                "reset_available": reset_available,
                "controls": controls,
            }
        return copy.deepcopy(result)

    def snapshot_status(self) -> Dict[str, Any]:
        """Return action/service availability and the latest operation state."""
        with self._lock:
            self._refresh_catalog_coherence_locked()
            result = dict(self._status)
            result.update(
                {
                    "cabinet": self.cabinet_name,
                    "namespace": self.interface_namespace,
                    "catalog_received": self._catalog_received,
                    "catalog_coherent": self._catalog_coherent,
                    "catalog_active_toolset": self._catalog_active_toolset,
                    "catalog_coherence_reason": self._catalog_coherence_reason,
                    "available": (
                        self._catalog_received
                        and self._catalog_coherent
                        and self._action_server_ready()
                    ),
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
            catalog_generation = self._catalog_generation
            control_signature = self._control_signature(control)
            self._begin_operation_locked(
                generation,
                control_id,
                (
                    int(control["control_type"])
                    if control is not None
                    else None
                ),
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
            expected_catalog_generation=catalog_generation,
            expected_control_signature=control_signature,
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
            self._status["force"] = requested_force

        goal = OperateCabinetControl.Goal()
        goal.control_id = control_id
        goal.command = command_code
        goal.target_state = normalized_state or ""
        goal.use_target_position = normalized_position is not None
        goal.target_position = normalized_position or 0.0
        goal.force = requested_force
        goal.navigate_to_staging_pose = navigate

        # A retiring A/B robot child can publish a replacement catalog while
        # this request is being prepared. Re-check the exact generation and
        # control signature immediately before dispatch so a validated stale
        # entry can never reach the action server.
        with self._lock:
            catalog_changed = (
                catalog_generation != self._catalog_generation
                or self._control_signature(self._controls.get(control_id))
                != control_signature
            )
            self._refresh_catalog_coherence_locked()
            catalog_ready = self._catalog_received and self._catalog_coherent
            catalog_reason = self._catalog_coherence_reason
            action_ready = self._action_server_ready()
            current_catalog_generation = self._catalog_generation
        if catalog_changed:
            terminal = self._finish_terminal(
                generation,
                "failed",
                "Cabinet control catalog changed while the operation was "
                "being prepared; refresh the catalog and retry.",
                error_code=None,
                failure_code="catalog_stale",
                result={
                    "catalog_generation": catalog_generation,
                    "current_catalog_generation": current_catalog_generation,
                },
            )
            return self._submission_response(terminal)
        if not catalog_ready:
            terminal = self._finish_terminal(
                generation,
                "failed",
                catalog_reason
                or "Cabinet catalog is not synchronized with the active toolset.",
                error_code=None,
                failure_code="catalog_stale",
            )
            return self._submission_response(terminal)
        if not action_ready:
            terminal = self._finish_terminal(
                generation,
                "failed",
                "Cabinet operation action server is unavailable.",
                error_code=None,
                failure_code="not_ready",
            )
            return self._submission_response(terminal)
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
            # Keep dispatch under the same lock used for catalog replacement;
            # rclpy queues the request and invokes callbacks asynchronously.
            with self._lock:
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
            "failure_code": None,
            "failure_reason": None,
        }

    def preflight_operation(
        self,
        control_id: str,
        command: Any,
        target_state: Optional[str] = None,
        target_position: Optional[float] = None,
        force: Optional[float] = None,
        navigate: bool = False,
    ) -> Dict[str, Any]:
        """Validate an operation without changing robot or cabinet state.

        This is deliberately read-only and is used by the task runner before
        homing or navigating the robot.  A catalog/toolset mismatch therefore
        becomes a deterministic failure at the API boundary instead of a
        pointless arm motion followed by an action rejection.
        """
        (
            normalized_control_id,
            command_code,
            command_name,
        ) = self._validate_request_shape(
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
            control = self._controls.get(normalized_control_id)
            catalog_generation = self._catalog_generation
            control_signature = self._control_signature(control)
        failure = self._validate_against_catalog(
            control,
            normalized_control_id,
            command_code,
            normalized_state,
            normalized_position,
            normalized_force,
            expected_catalog_generation=catalog_generation,
            expected_control_signature=control_signature,
        )
        if failure is not None:
            code, message, details = failure
            status = (
                503
                if code == "not_ready"
                else 404
                if code == "invalid_control"
                else 409
                if code in {"toolset_mismatch", "toolset_transition"}
                else 503
                if code == "catalog_stale"
                else 400
            )
            raise CabinetClientError(
                message,
                code=code,
                status=status,
                details=details,
                failure_reason=message,
            )
        if not self._action_server_ready():
            raise CabinetClientError(
                "Cabinet operation action server is unavailable.",
                code="not_ready",
                status=503,
                failure_reason="Cabinet operation action server is unavailable.",
            )
        with self._lock:
            if (
                catalog_generation != self._catalog_generation
                or self._control_signature(
                    self._controls.get(normalized_control_id)
                )
                != control_signature
            ):
                raise CabinetClientError(
                    "Cabinet control catalog changed while preflight was "
                    "completing; refresh the catalog and retry.",
                    code="catalog_stale",
                    status=503,
                    details={
                        "catalog_generation": catalog_generation,
                        "current_catalog_generation": self._catalog_generation,
                    },
                )
            coherence_failure = self._catalog_coherence_failure_locked()
        if coherence_failure is not None:
            code, reason, status, details = coherence_failure
            raise CabinetClientError(
                reason,
                code=code,
                status=status,
                details=details,
                failure_reason=reason,
            )
        return {
            "status": "ready",
            "cabinet": self.cabinet_name,
            "control_id": normalized_control_id,
            "command": command_name,
            "target_state": normalized_state,
            "target_position": normalized_position,
            "force": normalized_force,
            "navigate": bool(navigate),
            "toolset_compatible": bool(
                control.get("toolset_compatible", True)
            ) if control is not None else None,
            "adapter_validated": bool(
                control.get("adapter_validated", control.get("operable", False))
            ) if control is not None else None,
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
        entries = getattr(message, "controls", None)
        if entries is None:
            self.get_logger().warning(
                f"Ignored a catalog without controls for {self.cabinet_name}."
            )
            return
        try:
            iterator = iter(entries)
        except TypeError:
            self.get_logger().warning(
                f"Ignored a catalog with a non-iterable controls field for "
                f"{self.cabinet_name}."
            )
            return
        for entry in iterator:
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
            self._refresh_catalog_coherence_locked()

        self._destroy_subscription_map(old_subscriptions)
        for control in controls.values():
            try:
                subscriptions = self._create_control_subscriptions(
                    control,
                    generation,
                )
            except Exception as error:  # noqa: BLE001
                # One malformed control must not abort subscription for the
                # rest of the catalog.  _create_control_subscriptions already
                # tears down its own partial subscriptions before re-raising.
                self.get_logger().error(
                    f"Failed to subscribe to control "
                    f"{control.get('control_id')!r}: {error}"
                )
                continue
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
                # CabinetControlState is the authoritative aggregate and
                # already carries position, velocity, effort and activation.
                # Subscribing to the legacy joint/pressed mirrors as well
                # triples both DDS entities and Python callbacks for every
                # control (hundreds of subscriptions for a multi-cabinet
                # scene), which can starve unrelated sensor callbacks.
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
            else:
                # Legacy catalogs may expose only the split topics.  Keep
                # those fallbacks when no aggregate state topic is available.
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
        try:
            accepted = bool(goal_handle.accepted)
        except Exception as error:  # noqa: BLE001 - malformed action handle
            self._finish_terminal(
                generation,
                "failed",
                f"Cabinet action returned an invalid goal handle: {error}",
                error_code=None,
                failure_code="invalid_backend_result",
                failure_reason=(
                    f"Cabinet action returned an invalid goal handle: {error}"
                ),
            )
            return
        with self._lock:
            if generation != self._operation_generation:
                return
            canceled = self._cancel_requested
            if accepted:
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
        if not accepted:
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
            explicit_failure_reason = str(
                getattr(result_message, "failure_reason", "") or ""
            ).strip()
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
            evidence: Dict[str, bool] = {}
            for name in (
                "physical_outcome_confirmed",
                "final_state_verified",
                "transport_succeeded",
                "recovery_succeeded",
                "grasp_released",
            ):
                if hasattr(result_message, name):
                    evidence[name] = bool(getattr(result_message, name))
                    result[name] = evidence[name]
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
            # A successful terminal must not carry a failure reason; that is
            # another malformed backend result rather than a successful
            # operation with an ignorable warning.
            if outcome == "success" and explicit_failure_reason:
                result.update(
                    {
                        "backend_result_success": True,
                        "backend_action_status": action_status,
                        "backend_error_code": error_code,
                        "backend_failure_reason": explicit_failure_reason,
                    }
                )
                outcome = "failed"
                failure_code = "invalid_backend_result"
                message = (
                    "Cabinet operation returned a success result with a "
                    "non-empty failure reason."
                )
            if outcome == "success" and evidence:
                missing_evidence = [
                    name for name, confirmed in evidence.items()
                    if not confirmed
                ]
                if missing_evidence:
                    result.update(
                        {
                            "backend_result_success": True,
                            "backend_action_status": action_status,
                            "backend_error_code": error_code,
                            "missing_terminal_evidence": missing_evidence,
                        }
                    )
                    outcome = "failed"
                    failure_code = "invalid_backend_result"
                    message = (
                        "Cabinet operation reported success before all physical "
                        "terminal evidence was confirmed: "
                        + ", ".join(missing_evidence)
                    )
            # A terminal failure must never be represented by the SUCCESS
            # symbolic code.  Treat contradictory action/result pairs as a
            # malformed backend result and retain the raw values in the
            # diagnostic payload for operators.
            if outcome != "success" and failure_code == "success":
                result.update(
                    {
                        "backend_result_success": bool(result_message.success),
                        "backend_action_status": action_status,
                        "backend_error_code": error_code,
                    }
                )
                failure_code = "invalid_backend_result"
                message = (
                    "Cabinet operation returned an inconsistent terminal "
                    "result (failure was reported with SUCCESS code)."
                )
            failure_reason = (
                None
                if outcome == "success"
                else explicit_failure_reason or message
            )
        except Exception as error:  # noqa: BLE001
            outcome = "failed"
            message = f"Cabinet operation result failed: {error}"
            error_code = None
            failure_code = "result_channel_failed"
            result = {}
            failure_reason = message
        self._finish_terminal(
            generation,
            outcome,
            message,
            error_code=error_code,
            failure_code=failure_code,
            failure_reason=failure_reason,
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
        failure_reason: Optional[str] = None,
        result: Optional[Mapping[str, Any]] = None,
        expected_goal_handle: Any = None,
    ) -> Dict[str, Any]:
        result_data = dict(result or {})
        if outcome == "success":
            failure_code = None
            failure_reason = None
        else:
            failure_reason = str(failure_reason or message).strip() or (
                "Cabinet operation failed."
            )
            if not failure_code:
                failure_code = "operation_failed"
        with self._lock:
            if generation != self._operation_generation:
                return {}
            if (
                expected_goal_handle is not None
                and expected_goal_handle is not self._goal_handle
            ):
                return {}
            now = time.time()
            control_type = self._status.get("control_type")
            if control_type in self.CONTROL_TYPES:
                result_data["control_type"] = int(control_type)
            self._goal_handle = None
            self._cancel_requested = False
            self._status.update(
                {
                    "state": outcome,
                    "message": message,
                    "success": outcome == "success",
                    "failure_code": failure_code,
                    "failure_reason": failure_reason,
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
                "failure_reason": failure_reason,
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
        control_type: Optional[int],
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
            "control_type": control_type,
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
            "failure_reason": None,
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
        *,
        expected_catalog_generation: Optional[int] = None,
        expected_control_signature: Optional[Tuple[Any, ...]] = None,
    ) -> Optional[Tuple[str, str, Dict[str, Any]]]:
        with self._lock:
            if (
                expected_catalog_generation is not None
                and expected_catalog_generation != self._catalog_generation
            ):
                return (
                    "catalog_stale",
                    "Cabinet control catalog changed while the operation was "
                    "being validated; refresh the catalog and retry.",
                    {
                        "catalog_generation": expected_catalog_generation,
                        "current_catalog_generation": self._catalog_generation,
                    },
                )
            if (
                expected_control_signature is not None
                and self._control_signature(self._controls.get(control_id))
                != expected_control_signature
            ):
                return (
                    "catalog_stale",
                    "The requested cabinet control changed while the operation "
                    "was being validated; refresh the catalog and retry.",
                    {"control_id": control_id},
                )
            catalog_received = self._catalog_received
            coherence_failure = self._catalog_coherence_failure_locked()
        if not catalog_received:
            return (
                "not_ready",
                "Cabinet control catalog is not available yet.",
                {},
            )
        if coherence_failure is not None:
            code, reason, _status, details = coherence_failure
            return (
                code,
                reason,
                details,
            )
        if control is None:
            return (
                "invalid_control",
                f"Unsupported cabinet control: {control_id}.",
                {"control_id": control_id},
            )
        try:
            control_type = int(control.get("control_type", -1))
        except (TypeError, ValueError, OverflowError):
            return (
                "invalid_control",
                f"Cabinet control {control_id} has an invalid control type.",
                {"control_id": control_id},
            )
        if control_type not in self.CONTROL_TYPES:
            return (
                "invalid_control",
                f"Cabinet control {control_id} has an unsupported control type.",
                {"control_id": control_id, "control_type": control_type},
            )
        if command_code != OperateCabinetControl.Goal.COMMAND_SET_STATE and (
            target_state is not None
        ):
            return (
                "invalid_target",
                "target_state is only valid for the set_state command.",
                {"command": self.COMMAND_NAMES.get(command_code, command_code)},
            )
        if command_code != OperateCabinetControl.Goal.COMMAND_SET_POSITION and (
            target_position is not None
        ):
            return (
                "invalid_target",
                "target_position is only valid for the set_position command.",
                {"command": self.COMMAND_NAMES.get(command_code, command_code)},
            )
        if force is not None and (
            command_code != OperateCabinetControl.Goal.COMMAND_PRESS
            or control_type != CabinetControl.TYPE_BUTTON
        ):
            return (
                "invalid_force",
                "force is only valid for the press command on button controls; "
                "knobs, switches, and doors use their configured detent "
                "transition.",
                {"command": self.COMMAND_NAMES.get(command_code, command_code)},
            )
        if (
            bool(control.get("capability_metadata_present", False))
            and not bool(control.get("toolset_compatible", True))
        ):
            required_toolset = str(
                control.get("required_toolset") or "another toolset"
            ).strip()
            reason = str(control.get("unavailable_reason") or "").strip()
            message = (
                f"Control {control_id} requires toolset {required_toolset}; "
                "the currently mounted toolset cannot operate it."
            )
            if reason:
                message = f"{message} {reason}"
            return (
                "toolset_mismatch",
                message,
                {
                    "control_id": control_id,
                    "required_toolset": required_toolset,
                    "toolset_compatible": False,
                    "adapter_validated": bool(
                        control.get("adapter_validated", False)
                    ),
                    "reason": reason,
                },
            )
        # NOTE: We intentionally do NOT reject ``unsupported_command`` here.
        # When the command does not match the control's ``supported_commands``
        # bitmask, the request is forwarded to the C++ action server which
        # performs real-time planning validation and returns a structured
        # ``UNSUPPORTED_COMMAND`` failure with diagnostic details.  This
        # ensures every reachable task goes through simulation testing before
        # reporting failure, rather than being rejected outright at the Python
        # layer.
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
        if entry is None:
            return None
        try:
            control_id = str(entry.control_id).strip()
            control_type = int(entry.control_type)
        except (AttributeError, TypeError, ValueError, OverflowError):
            # A malformed DDS sample must not abort the catalog callback or
            # accidentally leave a partially parsed control writable.
            return None
        if not control_id or control_type not in self.CONTROL_TYPES:
            return None
        try:
            minimum = float(getattr(entry, "min_position", 0.0))
            maximum = float(getattr(entry, "max_position", 0.0))
            force_values = (
                float(getattr(entry, "default_force", 0.0)),
                float(getattr(entry, "min_trigger_force", 0.0)),
                float(getattr(entry, "max_force", 0.0)),
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        if (
            not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum > maximum
            or not all(math.isfinite(value) for value in force_values)
        ):
            return None
        try:
            state_ids = [
                str(value) for value in getattr(entry, "state_ids", [])
            ]
            state_labels = [
                str(value) for value in getattr(entry, "state_labels", [])
            ]
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        try:
            state_positions = [
                float(value) for value in getattr(entry, "state_positions", [])
            ]
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
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
        # Capability is a positive allowlist.  Old or malformed catalogs that
        # omit the field must never silently acquire physical-motion rights.
        operable = bool(getattr(entry, "operable", False))
        if operable and not resolved_topics["resolved_state_topic"] and not (
            str(getattr(entry, "joint_name", "")).strip()
            and resolved_topics["resolved_joint_state_topic"]
        ):
            return None
        try:
            supported = int(getattr(entry, "supported_commands", 0))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        if supported == 0 and control_type == CabinetControl.TYPE_BUTTON:
            supported = CabinetControl.SUPPORT_PRESS
        required_toolset = str(
            getattr(entry, "required_toolset", "")
        ).strip().upper()
        capability_metadata_present = bool(required_toolset)
        adapter_validated = bool(
            getattr(entry, "adapter_validated", operable)
        )
        toolset_compatible = bool(
            getattr(entry, "toolset_compatible", True)
        )
        if capability_metadata_present:
            # ``operable`` is a derived field, never an independent grant.
            operable = adapter_validated and toolset_compatible
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
            "required_toolset": required_toolset,
            "toolset_compatible": toolset_compatible,
            "adapter_validated": adapter_validated,
            # Required toolset is the version marker for the capability
            # metadata.  Old catalogs have no such field and remain
            # compatible, while newly published catalogs fail closed.
            "capability_metadata_present": capability_metadata_present,
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
            ("TOOLSET_MISMATCH", "toolset_mismatch"),
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
        except Exception:  # noqa: BLE001 - readiness is a fail-closed probe
            return False

    def _service_ready(self) -> bool:
        try:
            return bool(self._reset_client.service_is_ready())
        except Exception:  # noqa: BLE001 - readiness is a fail-closed probe
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
            "control_type": None,
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
            "failure_reason": None,
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
            "failure_reason": terminal.get("failure_reason") or terminal.get(
                "message"
            ),
            "error_code": terminal.get("error_code"),
            "success": terminal.get("success") is True,
            "message": terminal.get("message"),
            "result": copy.deepcopy(terminal.get("result", {})),
        }
