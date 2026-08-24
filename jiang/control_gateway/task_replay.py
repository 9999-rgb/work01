"""Safe orchestration of recorded high-level cabinet task scenarios."""

from __future__ import annotations

import copy
import math
import re
import threading
import time
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple


_RECORDING_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
_TERMINAL_TASK_STATUSES = frozenset({"success", "failed", "canceled"})
_ACTIVE_REPLAY_STATUSES = frozenset({"running", "canceling"})
_MAX_SCENARIO_STEPS = 1000
_MAX_CANCEL_ATTEMPTS = 3
_OPERATION_COMMANDS = frozenset(
    {"press", "set_state", "set_position", "toggle"}
)


class TaskReplayError(RuntimeError):
    """Base error for task-scenario replay."""


class TaskReplayConflictError(TaskReplayError):
    """Raised when another task replay already owns the orchestrator."""


class TaskReplayValidationError(TaskReplayError):
    """Raised when a recorded scenario is not safe to execute."""


SubmitNavigation = Callable[[str, Optional[str]], Mapping[str, Any]]
SubmitOperation = Callable[
    [str, str, Any, Optional[str], Optional[float], Optional[float]],
    Mapping[str, Any],
]
SubmitReset = Callable[[str], Mapping[str, Any]]
TaskStatus = Callable[[str], Mapping[str, Any]]
CancelTask = Callable[[str], Mapping[str, Any]]
LoadScenario = Callable[[str], Mapping[str, Any]]


class TaskReplayOrchestrator:
    """Replay semantic tasks without republishing recorded motion commands."""

    def __init__(
        self,
        *,
        load_scenario: LoadScenario,
        submit_navigation: SubmitNavigation,
        submit_operation: SubmitOperation,
        submit_reset: SubmitReset,
        task_status: TaskStatus,
        cancel_task: CancelTask,
        poll_period: float = 0.10,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not all(
            callable(callback)
            for callback in (
                load_scenario,
                submit_navigation,
                submit_operation,
                submit_reset,
                task_status,
                cancel_task,
                wall_clock,
                monotonic_clock,
            )
        ):
            raise TypeError("Task replay callbacks must be callable.")
        if (
            isinstance(poll_period, bool)
            or not isinstance(poll_period, (int, float))
            or not math.isfinite(float(poll_period))
            or float(poll_period) <= 0.0
        ):
            raise ValueError("poll_period must be a positive finite number.")
        self._load_scenario = load_scenario
        self._submit_navigation = submit_navigation
        self._submit_operation = submit_operation
        self._submit_reset = submit_reset
        self._task_status = task_status
        self._cancel_task = cancel_task
        self._poll_period = float(poll_period)
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock
        self._condition = threading.Condition(threading.RLock())
        self._cancel_event = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._current_task_id: Optional[str] = None
        self._cancel_sent_for_task: Optional[str] = None
        self._cancel_in_progress_for_task: Optional[str] = None
        self._cancel_attempts_for_task = 0
        self._submission_in_progress = False
        self._state = self._idle_state()

    @property
    def is_active(self) -> bool:
        """Return whether a scenario currently owns robot task execution."""
        with self._condition:
            return self._state["status"] in _ACTIVE_REPLAY_STATUSES

    def status(self) -> Dict[str, Any]:
        """Return a detached state snapshot for HTTP/Web consumers."""
        with self._condition:
            return copy.deepcopy(self._state)

    def start(self, recording_id: str) -> Dict[str, Any]:
        """Validate and asynchronously execute one recorded scenario."""
        recording_id = _recording_id(recording_id)
        raw_scenario = self._load_scenario(recording_id)
        steps = _validated_steps(raw_scenario)
        with self._condition:
            if (
                self._state["status"] in _ACTIVE_REPLAY_STATUSES
                or self._submission_in_progress
                or self._cancel_in_progress_for_task is not None
            ):
                raise TaskReplayConflictError(
                    "A task replay is already active."
                )
            self._cancel_event.clear()
            self._current_task_id = None
            self._cancel_sent_for_task = None
            self._cancel_in_progress_for_task = None
            self._cancel_attempts_for_task = 0
            self._submission_in_progress = False
            now = _finite_time(self._wall_clock(), "wall clock")
            self._state = {
                "status": "running",
                "recording_id": recording_id,
                "step_index": 0,
                "step_count": len(steps),
                "progress": 0.0,
                "current_step": None,
                "current_task_id": None,
                "message": "Task replay started.",
                "error": None,
                "started_at": now,
                "updated_at": now,
                "completed_at": None,
                "duration_seconds": None,
            }
            worker = threading.Thread(
                target=self._run,
                args=(steps,),
                name=f"xczs-task-replay-{recording_id}",
                daemon=True,
            )
            self._worker = worker
            try:
                worker.start()
            except Exception:
                self._worker = None
                self._finish_locked(
                    "failed",
                    "Failed to start the task replay worker.",
                )
                raise
            return copy.deepcopy(self._state)

    def cancel(self) -> Dict[str, Any]:
        """Asynchronously request cancellation of the active scenario."""
        with self._condition:
            if self._state["status"] not in _ACTIVE_REPLAY_STATUSES:
                return copy.deepcopy(self._state)
            self._request_cancel_locked()
            return copy.deepcopy(self._state)

    def shutdown(self, timeout: float = 5.0) -> bool:
        """Cancel and boundedly join the replay worker."""
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not math.isfinite(float(timeout))
            or float(timeout) < 0.0
        ):
            raise ValueError(
                "shutdown timeout must be finite and nonnegative."
            )
        deadline = time.monotonic() + float(timeout)
        # The replay worker owns backend cancellation delivery.  A wedged
        # callback therefore cannot make this caller exceed the shutdown
        # deadline, and ownership remains active until the callback stops.
        with self._condition:
            if self._state["status"] in _ACTIVE_REPLAY_STATUSES:
                self._request_cancel_locked()
            worker = self._worker
            stopped = (
                self._state["status"] not in _ACTIVE_REPLAY_STATUSES
                and not self._submission_in_progress
                and self._cancel_in_progress_for_task is None
            )
        if worker is None:
            return stopped
        if worker is threading.current_thread():
            return stopped
        worker.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._condition:
            return (
                not worker.is_alive()
                and self._state["status"] not in _ACTIVE_REPLAY_STATUSES
                and not self._submission_in_progress
                and self._cancel_in_progress_for_task is None
            )

    def _run(
        self,
        steps: Tuple[Dict[str, Any], ...],
    ) -> None:
        started = _finite_time(self._monotonic_clock(), "monotonic clock")
        try:
            for index, step in enumerate(steps):
                if not self._begin_submission(index, step):
                    self._finish(
                        "canceled",
                        "Task replay was canceled.",
                        started,
                    )
                    return
                try:
                    accepted = self._submit_step(step)
                except Exception:
                    with self._condition:
                        self._submission_in_progress = False
                        self._condition.notify_all()
                    raise
                task_id = accepted.get("task_id")
                if not isinstance(task_id, str) or not task_id:
                    with self._condition:
                        self._submission_in_progress = False
                        self._condition.notify_all()
                    raise TaskReplayError(
                        "Task submission did not return a task_id."
                    )
                with self._condition:
                    self._submission_in_progress = False
                    self._current_task_id = task_id
                    self._cancel_sent_for_task = None
                    self._cancel_in_progress_for_task = None
                    self._cancel_attempts_for_task = 0
                    self._state["current_task_id"] = task_id
                    if self._cancel_event.is_set():
                        self._state["status"] = "canceling"
                        self._state["message"] = (
                            "Task replay is canceling the task returned by "
                            "an in-flight submission."
                        )
                    else:
                        self._state["message"] = (
                            f"Waiting for replay step {index + 1}/"
                            f"{len(steps)}."
                        )
                    self._state["updated_at"] = _finite_time(
                        self._wall_clock(),
                        "wall clock",
                    )
                    self._condition.notify_all()
                if self._cancel_event.is_set():
                    self._try_cancel_task(task_id)
                terminal = self._wait_for_task(task_id)
                with self._condition:
                    while self._cancel_in_progress_for_task == task_id:
                        self._condition.wait(timeout=self._poll_period)
                    self._current_task_id = None
                    self._state["current_task_id"] = None
                if self._cancel_event.is_set():
                    self._finish(
                        "canceled",
                        "Task replay was canceled.",
                        started,
                    )
                    return
                outcome = str(terminal.get("status", ""))
                if outcome != "success":
                    reason = str(
                        terminal.get("failure_reason")
                        or terminal.get("message")
                        or f"Replay step ended as {outcome or 'unknown'}."
                    )
                    code = str(terminal.get("failure_code") or outcome)
                    raise TaskReplayError(f"{code}: {reason}")
                with self._condition:
                    self._state["step_index"] = index + 1
                    self._state["progress"] = (index + 1) / len(steps)
                    self._state["updated_at"] = _finite_time(
                        self._wall_clock(),
                        "wall clock",
                    )
            self._finish(
                "success",
                "Task replay completed successfully.",
                started,
            )
        except Exception as error:  # noqa: BLE001
            if self._cancel_event.is_set():
                self._finish("canceled", "Task replay was canceled.", started)
            else:
                self._finish(
                    "failed",
                    str(error) or type(error).__name__,
                    started,
                )
        finally:
            with self._condition:
                if self._worker is threading.current_thread():
                    self._worker = None
                self._condition.notify_all()

    def _submit_step(self, step: Mapping[str, Any]) -> Mapping[str, Any]:
        request = step["request"]
        if step["type"] == "navigate":
            return self._submit_navigation(
                request["cabinet"],
                request.get("control_id"),
            )
        if step["type"] == "reset":
            return self._submit_reset(request["cabinet"])
        return self._submit_operation(
            request["cabinet"],
            request["control_id"],
            request["command"],
            request.get("target_state"),
            request.get("target_position"),
            request.get("force"),
        )

    def _wait_for_task(self, task_id: str) -> Mapping[str, Any]:
        while True:
            try:
                task = self._task_status(task_id)
            except Exception as error:  # noqa: BLE001
                with self._condition:
                    if self._cancel_event.is_set():
                        self._state["message"] = (
                            "Task replay remains canceling while task "
                            f"status is unavailable: {error}"
                        )
                    else:
                        self._state["message"] = (
                            "Task replay is waiting while task status is "
                            f"temporarily unavailable: {error}"
                        )
                    self._state["updated_at"] = _finite_time(
                        self._wall_clock(),
                        "wall clock",
                    )
                if self._cancel_event.is_set():
                    self._try_cancel_task(task_id)
                time.sleep(self._poll_period)
                continue
            status = str(task.get("status", ""))
            reservation_active = bool(task.get("reservation_active", False))
            if status in _TERMINAL_TASK_STATUSES and not reservation_active:
                with self._condition:
                    cancel_in_progress = (
                        self._cancel_in_progress_for_task == task_id
                    )
                if not cancel_in_progress:
                    return task
            if self._cancel_event.is_set():
                self._try_cancel_task(task_id)
            time.sleep(self._poll_period)

    def _begin_submission(
        self,
        index: int,
        step: Mapping[str, Any],
    ) -> bool:
        with self._condition:
            if self._cancel_event.is_set():
                return False
            self._submission_in_progress = True
            count = self._state["step_count"]
            self._state["step_index"] = index
            self._state["progress"] = index / count
            self._state["current_step"] = copy.deepcopy(dict(step))
            self._state["message"] = (
                f"Submitting replay step {index + 1}/{count}."
            )
            self._state["updated_at"] = _finite_time(
                self._wall_clock(),
                "wall clock",
            )
            self._condition.notify_all()
            return True

    def _request_cancel_locked(self) -> None:
        already_canceling = self._state["status"] == "canceling"
        self._cancel_event.set()
        self._state["status"] = "canceling"
        if (
            self._submission_in_progress
            and self._current_task_id is None
            and not already_canceling
        ):
            self._state["message"] = (
                "Task replay cancellation requested; waiting for the "
                "in-flight submission so its task can be canceled."
            )
        elif not already_canceling:
            self._state["message"] = "Task replay cancellation requested."
        self._state["updated_at"] = _finite_time(
            self._wall_clock(),
            "wall clock",
        )
        self._condition.notify_all()

    def _claim_cancel_locked(self, task_id: Optional[str]) -> bool:
        if task_id is None or self._current_task_id != task_id:
            return False
        if self._cancel_sent_for_task == task_id:
            return False
        if self._cancel_in_progress_for_task == task_id:
            return False
        if self._cancel_attempts_for_task >= _MAX_CANCEL_ATTEMPTS:
            return False
        self._cancel_attempts_for_task += 1
        self._cancel_in_progress_for_task = task_id
        return True

    def _try_cancel_task(self, task_id: str) -> bool:
        with self._condition:
            should_cancel = self._claim_cancel_locked(task_id)
        if not should_cancel:
            return False
        return self._deliver_cancel(task_id) is None

    def _deliver_cancel(self, task_id: str) -> Optional[Exception]:
        try:
            self._cancel_task(task_id)
        except Exception as error:  # noqa: BLE001
            with self._condition:
                if self._cancel_in_progress_for_task == task_id:
                    self._cancel_in_progress_for_task = None
                    attempts = self._cancel_attempts_for_task
                    if attempts >= _MAX_CANCEL_ATTEMPTS:
                        self._state["message"] = (
                            "Task replay remains canceling after "
                            f"{attempts} backend cancellation attempts: "
                            f"{error}"
                        )
                    else:
                        self._state["message"] = (
                            "Task replay remains canceling; backend "
                            f"cancellation attempt {attempts}/"
                            f"{_MAX_CANCEL_ATTEMPTS} failed: {error}"
                        )
                    self._state["updated_at"] = _finite_time(
                        self._wall_clock(),
                        "wall clock",
                    )
                    self._condition.notify_all()
            return error
        with self._condition:
            if self._cancel_in_progress_for_task == task_id:
                self._cancel_in_progress_for_task = None
                self._cancel_sent_for_task = task_id
                self._state["message"] = (
                    "Task replay cancellation was sent to the active task."
                )
                self._state["updated_at"] = _finite_time(
                    self._wall_clock(),
                    "wall clock",
                )
                self._condition.notify_all()
        return None

    def _finish(self, status: str, message: str, started: float) -> None:
        elapsed = max(
            0.0,
            _finite_time(self._monotonic_clock(), "monotonic clock") - started,
        )
        with self._condition:
            self._finish_locked(status, message, duration=elapsed)

    def _finish_locked(
        self,
        status: str,
        message: str,
        *,
        duration: Optional[float] = None,
    ) -> None:
        now = _finite_time(self._wall_clock(), "wall clock")
        self._state["status"] = status
        self._state["message"] = message
        self._state["error"] = message if status == "failed" else None
        self._state["progress"] = (
            1.0 if status == "success" else self._state["progress"]
        )
        self._state["updated_at"] = now
        self._state["completed_at"] = now
        self._state["duration_seconds"] = duration
        self._state["current_task_id"] = None
        self._current_task_id = None
        self._condition.notify_all()

    @staticmethod
    def _idle_state() -> Dict[str, Any]:
        return {
            "status": "idle",
            "recording_id": None,
            "step_index": 0,
            "step_count": 0,
            "progress": 0.0,
            "current_step": None,
            "current_task_id": None,
            "message": "No task replay is active.",
            "error": None,
            "started_at": None,
            "updated_at": None,
            "completed_at": None,
            "duration_seconds": None,
        }


def _recording_id(value: Any) -> str:
    if (
        not isinstance(value, str)
        or not _RECORDING_ID_PATTERN.fullmatch(value)
    ):
        raise TaskReplayValidationError(
            "recording_id must contain only lowercase letters, digits, "
            "underscores, or hyphens and be at most 64 characters."
        )
    return value


def _validated_steps(scenario: Any) -> Tuple[Dict[str, Any], ...]:
    if not isinstance(scenario, Mapping):
        raise TaskReplayValidationError("Scenario must be a mapping.")
    allowed_document = {"schema_version", "recording_id", "steps"}
    unknown_document = set(scenario) - allowed_document
    if unknown_document:
        raise TaskReplayValidationError(
            "Scenario has unknown fields: "
            + ", ".join(sorted(str(key) for key in unknown_document))
        )
    schema_version = scenario.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version not in {1, 2}
    ):
        raise TaskReplayValidationError(
            "Scenario schema_version must be 1 or 2."
        )
    raw_steps = scenario.get("steps")
    if (
        not isinstance(raw_steps, Sequence)
        or isinstance(raw_steps, (str, bytes))
        or not raw_steps
    ):
        raise TaskReplayValidationError(
            "Scenario steps must be a non-empty list."
        )
    if len(raw_steps) > _MAX_SCENARIO_STEPS:
        raise TaskReplayValidationError(
            f"Scenario may contain at most {_MAX_SCENARIO_STEPS} steps."
        )
    return tuple(
        _validated_step(index, step, schema_version)
        for index, step in enumerate(raw_steps)
    )


def _validated_step(
    index: int,
    raw_step: Any,
    schema_version: int,
) -> Dict[str, Any]:
    context = f"steps[{index}]"
    if not isinstance(raw_step, Mapping):
        raise TaskReplayValidationError(f"{context} must be a mapping.")
    allowed_step = {
        "type",
        "request",
        "recorded_task_id",
        "recorded_outcome",
        "recorded_result",
        "recorded_failure_code",
        "recorded_failure_reason",
    }
    unknown_step = set(raw_step) - allowed_step
    if unknown_step:
        raise TaskReplayValidationError(
            f"{context} has unknown fields: "
            + ", ".join(sorted(str(key) for key in unknown_step))
        )
    step_type = raw_step.get("type")
    allowed_types = (
        {"navigate", "operate"}
        if schema_version == 1
        else {"navigate", "operate", "reset"}
    )
    if step_type not in allowed_types:
        raise TaskReplayValidationError(
            f"{context}.type is not allowed by scenario schema version "
            f"{schema_version}."
        )
    request = raw_step.get("request")
    if not isinstance(request, Mapping):
        raise TaskReplayValidationError(
            f"{context}.request must be a mapping."
        )
    if step_type == "navigate":
        normalized_request = _navigation_request(context, request)
    elif step_type == "reset":
        normalized_request = _reset_request(context, request)
    else:
        normalized_request = _operation_request(context, request)
    return {"type": step_type, "request": normalized_request}


def _navigation_request(
    context: str,
    request: Mapping[str, Any],
) -> Dict[str, Any]:
    allowed = {"cabinet", "control_id", "station"}
    unknown = set(request) - allowed
    if unknown:
        raise TaskReplayValidationError(
            f"{context}.request has unknown fields: "
            + ", ".join(sorted(str(key) for key in unknown))
        )
    normalized = {
        "cabinet": _nonempty_string(request.get("cabinet"), "cabinet")
    }
    control_id = request.get("control_id")
    if control_id is not None:
        normalized["control_id"] = _nonempty_string(control_id, "control_id")
    # A recorded absolute station is deliberately ignored.  Replaying through
    # the task API recomputes the current adapter's safe station.
    return normalized


def _reset_request(
    context: str,
    request: Mapping[str, Any],
) -> Dict[str, Any]:
    allowed = {"cabinet"}
    unknown = set(request) - allowed
    if unknown:
        raise TaskReplayValidationError(
            f"{context}.request has unknown fields: "
            + ", ".join(sorted(str(key) for key in unknown))
        )
    return {
        "cabinet": _nonempty_string(request.get("cabinet"), "cabinet")
    }


def _operation_request(
    context: str,
    request: Mapping[str, Any],
) -> Dict[str, Any]:
    allowed = {
        "cabinet",
        "control_id",
        "command",
        "target_state",
        "target_position",
        "force",
    }
    unknown = set(request) - allowed
    if unknown:
        raise TaskReplayValidationError(
            f"{context}.request has unknown fields: "
            + ", ".join(sorted(str(key) for key in unknown))
        )
    normalized: Dict[str, Any] = {
        "cabinet": _nonempty_string(request.get("cabinet"), "cabinet"),
        "control_id": _nonempty_string(
            request.get("control_id"),
            "control_id",
        ),
        "command": request.get("command"),
        "target_state": request.get("target_state"),
        "target_position": _optional_finite(
            request.get("target_position"),
            "target_position",
        ),
        "force": _optional_positive(request.get("force"), "force"),
    }
    command = normalized["command"]
    if not isinstance(command, str):
        raise TaskReplayValidationError(
            "command must be one of: press, set_state, set_position, toggle."
        )
    normalized_command = command.strip()
    if normalized_command not in _OPERATION_COMMANDS:
        raise TaskReplayValidationError(
            "command must be one of: press, set_state, set_position, toggle."
        )
    normalized["command"] = normalized_command
    target_state = normalized["target_state"]
    if target_state is not None:
        normalized["target_state"] = _nonempty_string(
            target_state,
            "target_state",
        )
    target_position = normalized["target_position"]
    # Recording happens after the live Pydantic request has been normalized:
    # TaskManager stores both optional target keys and therefore loses
    # ``model_fields_set``.  A non-applicable recorded null is compatible;
    # only a non-null target for the wrong command is unsafe.
    if normalized_command == "set_state":
        if normalized["target_state"] is None:
            raise TaskReplayValidationError(
                "target_state is required for set_state."
            )
        if target_position is not None:
            raise TaskReplayValidationError(
                "target_position is only valid for set_position."
            )
    elif normalized_command == "set_position":
        if target_position is None:
            raise TaskReplayValidationError(
                "target_position is required for set_position."
            )
        if normalized["target_state"] is not None:
            raise TaskReplayValidationError(
                "target_state is only valid for set_state."
            )
    elif normalized["target_state"] is not None or target_position is not None:
        raise TaskReplayValidationError(
            "target_state and target_position are not valid for "
            f"{normalized_command}."
        )
    return normalized


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TaskReplayValidationError(
            f"{label} must be a non-empty string."
        )
    return value.strip()


def _optional_finite(value: Any, label: str) -> Optional[float]:
    if value is None:
        return None
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise TaskReplayValidationError(f"{label} must be finite or null.")
    return float(value)


def _optional_positive(value: Any, label: str) -> Optional[float]:
    result = _optional_finite(value, label)
    if result is not None and result <= 0.0:
        raise TaskReplayValidationError(f"{label} must be positive or null.")
    return result


def _finite_time(value: Any, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError(f"{label} must return a finite number.")
    return float(value)
