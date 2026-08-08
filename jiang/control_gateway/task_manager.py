"""Thread-safe task lifecycle and replayable event stream for the Web API.

This module contains no ROS imports.  ROS adapters create a task, report
progress from action feedback, and complete it from the action result.  The
same events can therefore be consumed by both the HTTP SSE endpoint and a ROS
``std_msgs/String`` publisher.
"""

from __future__ import annotations

import copy
import math
import queue
import re
import secrets
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass
from typing import Any, Callable, Deque, Dict, Iterable, Mapping, Optional


_TASK_TYPE_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,31}$")
_RANDOM_SUFFIX_PATTERN = re.compile(r"^[a-z0-9]{6}$")
TERMINAL_STATUSES = frozenset({"success", "failed", "canceled"})


class TaskManagerError(RuntimeError):
    """Base task-lifecycle error."""


class TaskNotFoundError(TaskManagerError):
    """Raised when a task ID is not present in retained history."""

    def __init__(self, task_id: str) -> None:
        super().__init__(f"Unknown task: {task_id}")
        self.task_id = task_id
        self.status = 404


class TaskConflictError(TaskManagerError):
    """Raised when the global one-active-task invariant would be violated."""

    def __init__(self, active_task_id: str) -> None:
        super().__init__(f"Task {active_task_id} is already active.")
        self.active_task_id = active_task_id
        self.status = 409
        self.details = {"active_task_id": active_task_id}


class InvalidTaskTransitionError(TaskManagerError):
    """Raised for a stale or invalid lifecycle update."""


class TaskCancellationError(TaskManagerError):
    """Raised when an executor rejects or cannot deliver cancellation."""


class TaskManagerClosedError(TaskManagerError):
    """Raised when new work is submitted after shutdown has started."""

    def __init__(self) -> None:
        super().__init__("Task manager is shutting down.")
        self.status = 503


class TaskExecutionError(TaskManagerError):
    """Structured failure an executor can return through ``submit``."""

    def __init__(
        self,
        reason: str,
        *,
        code: str = "execution_failed",
        details: Optional[Mapping[str, Any]] = None,
        result: Optional[Mapping[str, Any]] = None,
    ) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("Task failure reason must be a non-empty string.")
        super().__init__(reason.strip())
        self.reason = reason.strip()
        self.code = code
        self.details = dict(details or {})
        self.result = dict(result or {})


class TaskCanceledError(TaskManagerError):
    """Signal cooperative cancellation from a submitted task executor."""


class EventHubClosed(RuntimeError):
    """Raised when a closed subscription has no buffered events left."""


class _SubscriptionBuffer:
    """Bound progress traffic while retaining every terminal task event."""

    def __init__(self, max_pending: int) -> None:
        self._max_pending = max_pending
        self._events: Deque[Dict[str, Any]] = deque()
        self._condition = threading.Condition()
        self._closed = False

    def put(self, event: Dict[str, Any]) -> None:
        with self._condition:
            if self._closed:
                return
            if len(self._events) >= self._max_pending:
                removable = next(
                    (
                        index
                        for index, pending in enumerate(self._events)
                        if pending.get("event") != "task_completed"
                    ),
                    None,
                )
                if removable is not None:
                    del self._events[removable]
                elif event.get("event") != "task_completed":
                    return
                # If the buffer consists only of completion events, allow a
                # completion event to overflow instead of losing a terminal.
            self._events.append(copy.deepcopy(event))
            self._condition.notify()

    def get(self, timeout: Optional[float]) -> Dict[str, Any]:
        if timeout is not None and timeout < 0.0:
            raise ValueError("timeout must not be negative")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while not self._events:
                if self._closed:
                    raise EventHubClosed("Event subscription is closed.")
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise queue.Empty
                self._condition.wait(remaining)
            return self._events.popleft()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed


class EventSubscription:
    """One EventHub subscription suitable for a streaming HTTP handler."""

    def __init__(
        self,
        hub: "EventHub",
        subscription_id: int,
        buffer: _SubscriptionBuffer,
    ) -> None:
        self._hub = hub
        self._subscription_id = subscription_id
        self._buffer = buffer
        self._close_lock = threading.Lock()

    def get(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Get the next event; raises ``queue.Empty`` on heartbeat timeout."""
        return self._buffer.get(timeout)

    def close(self) -> None:
        with self._close_lock:
            if self._buffer.closed:
                return
            self._hub._unsubscribe(self._subscription_id)  # noqa: SLF001

    @property
    def closed(self) -> bool:
        return self._buffer.closed

    def __enter__(self) -> "EventSubscription":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()


class EventHub:
    """Replayable event ring with race-free live subscriptions."""

    def __init__(self, *, capacity: int = 512) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("Event capacity must be a positive integer.")
        self._capacity = capacity
        self._events: Deque[Dict[str, Any]] = deque(maxlen=capacity)
        self._subscriptions: Dict[int, _SubscriptionBuffer] = {}
        self._next_subscription_id = 1
        self._next_sequence = 1
        self._lock = threading.RLock()
        self._closed = False

    def publish(
        self,
        event_type: str,
        data: Mapping[str, Any],
        *,
        timestamp: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Store and fan out an event, returning its JSON-safe envelope."""
        if not isinstance(event_type, str) or not event_type.strip():
            raise ValueError("event_type must be a non-empty string.")
        if not isinstance(data, Mapping):
            raise ValueError("event data must be a mapping.")
        event_timestamp = time.time() if timestamp is None else timestamp
        if (
            isinstance(event_timestamp, bool)
            or not isinstance(event_timestamp, (int, float))
            or not math.isfinite(float(event_timestamp))
        ):
            raise ValueError("event timestamp must be finite.")
        with self._lock:
            if self._closed:
                raise EventHubClosed("Event hub is closed.")
            sequence = self._next_sequence
            self._next_sequence += 1
            event = {
                "id": str(sequence),
                "sequence": sequence,
                "event": event_type.strip(),
                "timestamp": float(event_timestamp),
                "data": copy.deepcopy(dict(data)),
            }
            self._events.append(event)
            subscriptions = tuple(self._subscriptions.values())
        for subscription in subscriptions:
            subscription.put(event)
        return copy.deepcopy(event)

    emit = publish

    def subscribe(
        self,
        *,
        last_event_id: Optional[str | int] = None,
        max_pending: int = 128,
    ) -> EventSubscription:
        """Atomically replay retained events and attach a live subscriber.

        Replay is exclusive of ``last_event_id``, matching SSE
        ``Last-Event-ID`` semantics.  An omitted ID starts with new events.
        """
        if (
            isinstance(max_pending, bool)
            or not isinstance(max_pending, int)
            or max_pending <= 0
        ):
            raise ValueError("max_pending must be a positive integer.")
        after_sequence = _parse_event_id(last_event_id)
        with self._lock:
            if self._closed:
                raise EventHubClosed("Event hub is closed.")
            subscription_id = self._next_subscription_id
            self._next_subscription_id += 1
            buffer = _SubscriptionBuffer(max_pending)
            if after_sequence is not None:
                for event in self._events:
                    if event["sequence"] > after_sequence:
                        buffer.put(event)
            self._subscriptions[subscription_id] = buffer
        return EventSubscription(self, subscription_id, buffer)

    def events_after(
        self,
        last_event_id: Optional[str | int] = None,
    ) -> list[Dict[str, Any]]:
        """Return retained events after an ID for polling/debugging."""
        after_sequence = _parse_event_id(last_event_id)
        if after_sequence is None:
            after_sequence = 0
        with self._lock:
            return [
                copy.deepcopy(event)
                for event in self._events
                if event["sequence"] > after_sequence
            ]

    def close(self) -> None:
        """Wake streaming clients and reject future publications."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            subscriptions = tuple(self._subscriptions.values())
            self._subscriptions.clear()
        for subscription in subscriptions:
            subscription.close()

    def _unsubscribe(self, subscription_id: int) -> None:
        with self._lock:
            buffer = self._subscriptions.pop(subscription_id, None)
        if buffer is not None:
            buffer.close()


@dataclass(frozen=True)
class TaskContext:
    """Cooperative interface passed to a ``TaskManager.submit`` executor."""

    manager: "TaskManager"
    task_id: str

    def progress(
        self,
        phase: str,
        progress: float,
        *,
        message: Optional[str] = None,
        data: Optional[Mapping[str, Any]] = None,
    ) -> Dict[str, Any]:
        return self.manager.report_progress(
            self.task_id,
            phase,
            progress,
            message=message,
            data=data,
        )

    @property
    def cancellation_requested(self) -> bool:
        return self.manager.is_cancel_requested(self.task_id)

    def raise_if_canceled(self) -> None:
        if self.cancellation_requested:
            raise TaskCanceledError("Task was canceled.")

    @property
    def shutdown_requested(self) -> bool:
        return self.manager.shutdown_requested

    @property
    def shutdown_elapsed(self) -> float:
        return self.manager.shutdown_elapsed

    def fail_retaining_reservation(
        self,
        reason: str,
        *,
        code: str,
        details: Optional[Mapping[str, Any]] = None,
        result: Optional[Mapping[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Publish a terminal failure while retaining backend ownership."""
        return self.manager.fail_task(
            self.task_id,
            reason,
            code=code,
            details=details,
            result=result,
            retain_reservation=True,
        )

    def release_reservation(
        self,
        *,
        backend_termination_confirmed: bool,
        details: Optional[Mapping[str, Any]] = None,
    ) -> Dict[str, Any]:
        return self.manager.release_reservation(
            self.task_id,
            backend_termination_confirmed=backend_termination_confirmed,
            details=details,
        )


CancelCallback = Callable[[str], Any]
TaskExecutor = Callable[[TaskContext], Optional[Mapping[str, Any]]]


class TaskManager:
    """Own task history and enforce one active task across all task types."""

    def __init__(
        self,
        *,
        event_hub: Optional[EventHub] = None,
        history_limit: int = 256,
        clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
        random_suffix: Callable[[], str] = lambda: secrets.token_hex(3),
    ) -> None:
        if (
            isinstance(history_limit, bool)
            or not isinstance(history_limit, int)
            or history_limit <= 0
        ):
            raise ValueError("history_limit must be a positive integer.")
        self.events = event_hub if event_hub is not None else EventHub()
        self._history_limit = history_limit
        self._clock = clock
        self._monotonic_clock = monotonic_clock
        self._random_suffix = random_suffix
        self._condition = threading.Condition(threading.RLock())
        self._tasks: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
        self._task_monotonic: Dict[str, float] = {}
        self._cancel_callbacks: Dict[str, CancelCallback] = {}
        self._workers: Dict[str, threading.Thread] = {}
        self._shutdown_cancel_workers: Dict[str, threading.Thread] = {}
        self._shutdown_cancel_errors: Dict[str, str] = {}
        self._active_task_id: Optional[str] = None
        self._shutdown_requested = False
        self._shutdown_requested_at: Optional[float] = None

    @property
    def active_task_id(self) -> Optional[str]:
        with self._condition:
            return self._active_task_id

    @property
    def shutdown_requested(self) -> bool:
        with self._condition:
            return self._shutdown_requested

    @property
    def shutdown_elapsed(self) -> float:
        with self._condition:
            requested_at = self._shutdown_requested_at
        if requested_at is None:
            return 0.0
        return max(0.0, time.monotonic() - requested_at)

    def create_task(
        self,
        task_type: str,
        request: Mapping[str, Any],
        *,
        cancel_callback: Optional[CancelCallback] = None,
    ) -> Dict[str, Any]:
        """Atomically accept a task or raise ``TaskConflictError``."""
        task_type = _validate_task_type(task_type)
        if not isinstance(request, Mapping):
            raise ValueError("Task request must be a mapping.")
        if cancel_callback is not None and not callable(cancel_callback):
            raise ValueError("cancel_callback must be callable.")
        with self._condition:
            if self._shutdown_requested:
                raise TaskManagerClosedError()
            if self._active_task_id is not None:
                raise TaskConflictError(self._active_task_id)
            created_at = _finite_timestamp(self._clock(), "task clock")
            created_monotonic = _finite_timestamp(
                self._monotonic_clock(),
                "task monotonic clock",
            )
            task_id = self._new_task_id_locked(task_type, created_at)
            task = {
                "task_id": task_id,
                "type": task_type,
                "status": "accepted",
                "request": copy.deepcopy(dict(request)),
                "phase": "accepted",
                "progress": 0.0,
                "message": "Task accepted.",
                "business_data": {},
                "result": None,
                "failure_code": None,
                "failure_reason": None,
                "failure_details": None,
                "cancel_requested": False,
                "cancel_acknowledgement": None,
                "cancel_callback_in_progress": False,
                "created_at": created_at,
                "started_at": None,
                "updated_at": created_at,
                "completed_at": None,
                "duration_seconds": None,
                "reservation_active": True,
                "reservation_released_at": None,
                "backend_termination_confirmed": None,
                "backend_completed_at": None,
            }
            self._tasks[task_id] = task
            self._task_monotonic[task_id] = created_monotonic
            if cancel_callback is not None:
                self._cancel_callbacks[task_id] = cancel_callback
            self._active_task_id = task_id
            self._trim_history_locked()
            snapshot = _snapshot(task)
            self._publish_event_locked(
                "task_accepted",
                {"task_id": task_id, "task": snapshot},
                timestamp=created_at,
            )
            self._condition.notify_all()
            return snapshot

    def submit(
        self,
        task_type: str,
        request: Mapping[str, Any],
        executor: TaskExecutor,
        *,
        cancel_callback: Optional[CancelCallback] = None,
        thread_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Accept a task and run a blocking executor on a daemon thread."""
        if not callable(executor):
            raise ValueError("executor must be callable.")
        # Hold the manager lock through worker registration/start so shutdown
        # cannot miss a task between create_task() and thread tracking.
        with self._condition:
            task = self.create_task(
                task_type,
                request,
                cancel_callback=cancel_callback,
            )
            task_id = task["task_id"]
            worker = threading.Thread(
                target=self._run_executor,
                args=(task_id, executor),
                name=thread_name or f"xczs-task-{task_id}",
                daemon=True,
            )
            self._workers[task_id] = worker
            try:
                worker.start()
            except Exception:
                self._workers.pop(task_id, None)
                self.fail_task(
                    task_id,
                    "Failed to start the task worker.",
                    code="worker_start_failed",
                )
                raise
        return task

    def request_shutdown(self, *, cancel_active: bool = True) -> Dict[str, Any]:
        """Reject new tasks, signal workers, and request backend cancellation."""
        with self._condition:
            if not self._shutdown_requested:
                self._shutdown_requested = True
                self._shutdown_requested_at = time.monotonic()
            active_task_id = self._active_task_id
            worker_task_ids = list(self._workers)
            self._condition.notify_all()

        cancellation_error: Optional[str] = None
        if cancel_active and active_task_id is not None:
            cancellation_error = self._cancel_active_for_shutdown(
                active_task_id
            )
        return {
            "active_task_id": active_task_id,
            "worker_task_ids": worker_task_ids,
            "cancellation_error": cancellation_error,
        }

    def shutdown(
        self,
        *,
        timeout: float,
        cancel_active: bool = True,
    ) -> Dict[str, Any]:
        """Boundedly join tracked workers without closing the event stream."""
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not math.isfinite(float(timeout))
            or timeout < 0.0
        ):
            raise ValueError("shutdown timeout must be a finite nonnegative number.")
        deadline = time.monotonic() + float(timeout)
        # Backend cancel callbacks may enter rclpy and are not guaranteed to
        # return.  Run them as tracked daemon workers so the caller's timeout
        # covers cancellation as well as the task executor itself.
        requested = self.request_shutdown(cancel_active=False)
        active_task_id = requested["active_task_id"]
        if cancel_active and active_task_id is not None:
            self._start_shutdown_cancel_worker(active_task_id)
        current = threading.current_thread()
        while True:
            with self._condition:
                workers = {
                    task_id: worker
                    for task_id, worker in self._workers.items()
                    if worker is not current
                }
                cancel_workers = {
                    task_id: worker
                    for task_id, worker in self._shutdown_cancel_workers.items()
                    if worker is not current
                }
            all_workers = list(workers.values()) + list(cancel_workers.values())
            if not all_workers:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            # Join in short slices so a worker finishing wakes the next audit
            # promptly while the total wait remains bounded by one deadline.
            slice_timeout = min(0.05, remaining)
            for worker in all_workers:
                worker.join(timeout=slice_timeout)
                if time.monotonic() >= deadline:
                    break

        with self._condition:
            pending = [
                task_id
                for task_id, worker in self._workers.items()
                if worker is not current and worker.is_alive()
            ]
            pending_cancel = [
                task_id
                for task_id, worker in self._shutdown_cancel_workers.items()
                if worker is not current and worker.is_alive()
            ]
            active_task_id = self._active_task_id
            cancellation_errors = copy.deepcopy(self._shutdown_cancel_errors)
        return {
            **requested,
            "pending_worker_task_ids": pending,
            "pending_cancellation_task_ids": pending_cancel,
            "cancellation_errors": cancellation_errors,
            "workers_stopped": not pending and not pending_cancel,
            "active_task_id": active_task_id,
        }

    def _cancel_active_for_shutdown(self, task_id: str) -> Optional[str]:
        try:
            task = self.get_task(task_id)
            if task["status"] not in TERMINAL_STATUSES:
                self.cancel_task(task_id)
            else:
                callback = self._cancel_callback(task_id)
                if callback is not None:
                    callback(task_id)
        except Exception as error:  # noqa: BLE001
            # Shutdown remains best effort. The adapter retains ownership and
            # reports an unconfirmed backend termination when appropriate.
            return str(error) or type(error).__name__
        return None

    def _start_shutdown_cancel_worker(self, task_id: str) -> None:
        with self._condition:
            existing = self._shutdown_cancel_workers.get(task_id)
            if existing is not None and existing.is_alive():
                return

            def cancel_backend() -> None:
                error = self._cancel_active_for_shutdown(task_id)
                with self._condition:
                    if error is not None:
                        self._shutdown_cancel_errors[task_id] = error
                    current = self._shutdown_cancel_workers.get(task_id)
                    if current is threading.current_thread():
                        self._shutdown_cancel_workers.pop(task_id, None)
                    self._condition.notify_all()

            worker = threading.Thread(
                target=cancel_backend,
                name=f"xczs-task-shutdown-cancel-{task_id}",
                daemon=True,
            )
            self._shutdown_cancel_workers[task_id] = worker
            try:
                worker.start()
            except Exception:
                self._shutdown_cancel_workers.pop(task_id, None)
                raise

    def _cancel_callback(self, task_id: str) -> Optional[CancelCallback]:
        with self._condition:
            return self._cancel_callbacks.get(task_id)

    def start_task(
        self,
        task_id: str,
        *,
        phase: str = "starting",
        message: str = "Task started.",
    ) -> Dict[str, Any]:
        phase = _nonempty_string(phase, "phase")
        message = _nonempty_string(message, "message")
        with self._condition:
            task = self._task_locked(task_id)
            if task["status"] == "canceling":
                return _snapshot(task)
            if task["status"] != "accepted":
                raise InvalidTaskTransitionError(
                    f"Cannot start task {task_id} from {task['status']}."
                )
            now = _finite_timestamp(self._clock(), "task clock")
            task.update(
                status="running",
                phase=phase,
                message=message,
                started_at=now,
                updated_at=now,
            )
            snapshot = _snapshot(task)
            self._publish_progress_locked(snapshot, now)
            self._condition.notify_all()
            return snapshot

    def report_progress(
        self,
        task_id: str,
        phase: str,
        progress: float,
        *,
        message: Optional[str] = None,
        data: Optional[Mapping[str, Any]] = None,
    ) -> Dict[str, Any]:
        phase = _nonempty_string(phase, "phase")
        progress = _progress_value(progress)
        if message is not None:
            message = _nonempty_string(message, "message")
        if data is not None and not isinstance(data, Mapping):
            raise ValueError("progress data must be a mapping.")
        with self._condition:
            task = self._task_locked(task_id)
            if task["status"] in TERMINAL_STATUSES:
                raise InvalidTaskTransitionError(
                    f"Cannot update completed task {task_id}."
                )
            if progress < task["progress"]:
                raise ValueError("Task progress must not decrease.")
            now = _finite_timestamp(self._clock(), "task clock")
            if task["status"] == "accepted":
                task["status"] = "running"
                task["started_at"] = now
            task["phase"] = phase
            task["progress"] = progress
            task["updated_at"] = now
            if message is not None:
                task["message"] = message
            if data is not None:
                task["business_data"] = copy.deepcopy(dict(data))
            snapshot = _snapshot(task)
            self._publish_progress_locked(snapshot, now)
            self._condition.notify_all()
            return snapshot

    def succeed_task(
        self,
        task_id: str,
        result: Optional[Mapping[str, Any]] = None,
        *,
        message: str = "Task completed successfully.",
    ) -> Dict[str, Any]:
        return self._finish_task(
            task_id,
            "success",
            result=result,
            message=message,
        )

    def fail_task(
        self,
        task_id: str,
        reason: str,
        *,
        code: str = "execution_failed",
        details: Optional[Mapping[str, Any]] = None,
        result: Optional[Mapping[str, Any]] = None,
        retain_reservation: bool = False,
    ) -> Dict[str, Any]:
        reason = _nonempty_string(reason, "failure reason")
        code = _nonempty_string(code, "failure code")
        if details is not None and not isinstance(details, Mapping):
            raise ValueError("failure details must be a mapping.")
        if not isinstance(retain_reservation, bool):
            raise ValueError("retain_reservation must be a boolean.")
        return self._finish_task(
            task_id,
            "failed",
            result=result,
            message=reason,
            failure_code=code,
            failure_reason=reason,
            failure_details=details,
            retain_reservation=retain_reservation,
        )

    def mark_canceled(
        self,
        task_id: str,
        result: Optional[Mapping[str, Any]] = None,
        *,
        reason: str = "Task was canceled.",
    ) -> Dict[str, Any]:
        reason = _nonempty_string(reason, "cancellation reason")
        return self._finish_task(
            task_id,
            "canceled",
            result=result,
            message=reason,
            failure_code="canceled",
            failure_reason=reason,
        )

    def release_reservation(
        self,
        task_id: str,
        *,
        backend_termination_confirmed: bool,
        details: Optional[Mapping[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Release retained global ownership after backend cleanup finishes."""
        if not isinstance(backend_termination_confirmed, bool):
            raise ValueError("backend_termination_confirmed must be a boolean.")
        if details is not None and not isinstance(details, Mapping):
            raise ValueError("reservation details must be a mapping.")
        with self._condition:
            task = self._task_locked(task_id)
            if task["status"] not in TERMINAL_STATUSES:
                raise InvalidTaskTransitionError(
                    f"Cannot release reservation for non-terminal task {task_id}."
                )
            if not task.get("reservation_active", False):
                return _snapshot(task)
            now = _finite_timestamp(self._clock(), "task clock")
            task.update(
                reservation_active=False,
                reservation_released_at=now,
                backend_termination_confirmed=backend_termination_confirmed,
                backend_completed_at=(
                    now if backend_termination_confirmed else None
                ),
                updated_at=now,
            )
            if details:
                existing = dict(task.get("failure_details") or {})
                existing.update(copy.deepcopy(dict(details)))
                task["failure_details"] = existing
            self._cancel_callbacks.pop(task_id, None)
            task["cancel_callback_in_progress"] = False
            if self._active_task_id == task_id:
                self._active_task_id = None
            snapshot = _snapshot(task)
            self._publish_event_locked(
                "task_reservation_released",
                {
                    "task_id": task_id,
                    "backend_termination_confirmed": (
                        backend_termination_confirmed
                    ),
                    "task": snapshot,
                },
                timestamp=now,
            )
            self._trim_history_locked()
            self._condition.notify_all()
            return snapshot

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        """Request cancellation, leaving terminal confirmation to the adapter."""
        retained_terminal = False
        with self._condition:
            task = self._task_locked(task_id)
            if task["status"] in TERMINAL_STATUSES:
                if not task.get("reservation_active", False):
                    return _snapshot(task)
                retained_terminal = True
                callback = self._cancel_callbacks.get(task_id)
                if callback is not None:
                    task["cancel_callback_in_progress"] = True
                snapshot = _snapshot(task)
            elif task["status"] == "canceling":
                return _snapshot(task)
            else:
                previous_status = task["status"]
                now = _finite_timestamp(self._clock(), "task clock")
                task.update(
                    status="canceling",
                    phase="canceling",
                    message="Cancellation requested.",
                    cancel_requested=True,
                    updated_at=now,
                )
                callback = self._cancel_callbacks.get(task_id)
                if callback is not None:
                    task["cancel_callback_in_progress"] = True
                snapshot = _snapshot(task)
                self._publish_progress_locked(snapshot, now)
                self._condition.notify_all()
        if callback is None:
            return snapshot if retained_terminal else self.mark_canceled(task_id)
        try:
            acknowledgement = callback(task_id)
            rejected = acknowledgement is False or (
                isinstance(acknowledgement, Mapping)
                and acknowledgement.get("accepted") is False
            )
            if rejected:
                raise TaskCancellationError("Task executor rejected cancellation.")
        except Exception as error:
            release_after_error = False
            with self._condition:
                task = self._task_locked(task_id)
                task["cancel_callback_in_progress"] = False
                if task["status"] == "canceling":
                    task.update(
                        status=previous_status,
                        phase="cancel_failed",
                        message=str(error),
                        cancel_requested=False,
                        updated_at=_finite_timestamp(self._clock(), "task clock"),
                    )
                    self._publish_progress_locked(
                        _snapshot(task),
                        task["updated_at"],
                    )
                    self._condition.notify_all()
                elif not retained_terminal and task["status"] in TERMINAL_STATUSES:
                    release_after_error = bool(task.get("reservation_active"))
            if release_after_error:
                self.release_reservation(
                    task_id,
                    backend_termination_confirmed=True,
                    details={"cancel_callback_error": str(error)},
                )
            if isinstance(error, TaskCancellationError):
                raise
            raise TaskCancellationError(str(error)) from error
        if retained_terminal:
            with self._condition:
                task = self._task_locked(task_id)
                task["cancel_callback_in_progress"] = False
                task["cancel_acknowledgement"] = copy.deepcopy(
                    acknowledgement
                )
                task["updated_at"] = _finite_timestamp(
                    self._clock(),
                    "task clock",
                )
                self._condition.notify_all()
                return _snapshot(task)
        release_after_callback = False
        with self._condition:
            task = self._task_locked(task_id)
            task["cancel_callback_in_progress"] = False
            if task["status"] == "canceling":
                task["cancel_acknowledgement"] = copy.deepcopy(acknowledgement)
                task["updated_at"] = _finite_timestamp(
                    self._clock(),
                    "task clock",
                )
            elif task["status"] in TERMINAL_STATUSES:
                release_after_callback = bool(task.get("reservation_active"))
            snapshot = _snapshot(task)
        if release_after_callback:
            return self.release_reservation(
                task_id,
                backend_termination_confirmed=True,
                details={"cancel_callback_completed": True},
            )
        return snapshot

    def set_cancel_callback(
        self,
        task_id: str,
        callback: Optional[CancelCallback],
    ) -> None:
        """Register a callback after an asynchronous backend creates a goal."""
        if callback is not None and not callable(callback):
            raise ValueError("cancel callback must be callable.")
        with self._condition:
            task = self._task_locked(task_id)
            if task["status"] in TERMINAL_STATUSES:
                raise InvalidTaskTransitionError(
                    f"Cannot configure completed task {task_id}."
                )
            if callback is None:
                self._cancel_callbacks.pop(task_id, None)
            else:
                self._cancel_callbacks[task_id] = callback

    def is_cancel_requested(self, task_id: str) -> bool:
        with self._condition:
            return bool(self._task_locked(task_id)["cancel_requested"])

    def get_task(self, task_id: str) -> Dict[str, Any]:
        with self._condition:
            return _snapshot(self._task_locked(task_id))

    status = get_task

    def list_tasks(self, *, limit: Optional[int] = None) -> list[Dict[str, Any]]:
        if limit is not None and (
            isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0
        ):
            raise ValueError("limit must be a positive integer.")
        with self._condition:
            tasks: Iterable[Dict[str, Any]] = reversed(self._tasks.values())
            snapshots = [_snapshot(task) for task in tasks]
            return snapshots if limit is None else snapshots[:limit]

    def wait(
        self,
        task_id: str,
        *,
        timeout: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Wait for a terminal state, primarily for orchestration and tests."""
        if timeout is not None and timeout < 0.0:
            raise ValueError("timeout must not be negative")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while True:
                task = self._task_locked(task_id)
                if task["status"] in TERMINAL_STATUSES:
                    return _snapshot(task)
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"Task {task_id} did not finish in time.")
                self._condition.wait(remaining)

    def _finish_task(
        self,
        task_id: str,
        status: str,
        *,
        result: Optional[Mapping[str, Any]],
        message: str,
        failure_code: Optional[str] = None,
        failure_reason: Optional[str] = None,
        failure_details: Optional[Mapping[str, Any]] = None,
        retain_reservation: bool = False,
    ) -> Dict[str, Any]:
        if result is not None and not isinstance(result, Mapping):
            raise ValueError("Task result must be a mapping.")
        message = _nonempty_string(message, "message")
        with self._condition:
            task = self._task_locked(task_id)
            if task["status"] in TERMINAL_STATUSES:
                if task["status"] == status:
                    return _snapshot(task)
                raise InvalidTaskTransitionError(
                    f"Task {task_id} already finished as {task['status']}."
                )
            now = _finite_timestamp(self._clock(), "task clock")
            elapsed = max(
                0.0,
                _finite_timestamp(self._monotonic_clock(), "task monotonic clock")
                - self._task_monotonic[task_id],
            )
            retain_reservation = bool(
                retain_reservation
                or task.get("cancel_callback_in_progress", False)
            )
            task.update(
                status=status,
                phase="completed",
                progress=1.0,
                message=message,
                result=copy.deepcopy(dict(result or {})),
                failure_code=failure_code,
                failure_reason=failure_reason,
                failure_details=(
                    copy.deepcopy(dict(failure_details))
                    if failure_details is not None
                    else None
                ),
                updated_at=now,
                completed_at=now,
                duration_seconds=elapsed,
                reservation_active=retain_reservation,
                reservation_released_at=(None if retain_reservation else now),
                backend_termination_confirmed=(
                    False if retain_reservation else True
                ),
                backend_completed_at=(None if retain_reservation else now),
            )
            if not retain_reservation:
                self._cancel_callbacks.pop(task_id, None)
                if self._active_task_id == task_id:
                    self._active_task_id = None
            snapshot = _snapshot(task)
            self._publish_event_locked(
                "task_completed",
                {
                    "task_id": task_id,
                    "outcome": status,
                    "task": snapshot,
                },
                timestamp=now,
            )
            self._trim_history_locked()
            self._condition.notify_all()
            return snapshot

    def _publish_progress_locked(
        self,
        snapshot: Dict[str, Any],
        timestamp: float,
    ) -> None:
        self._publish_event_locked(
            "task_progress",
            {
                "task_id": snapshot["task_id"],
                "phase": snapshot["phase"],
                "progress": snapshot["progress"],
                "business_data": snapshot["business_data"],
                "task": snapshot,
            },
            timestamp=timestamp,
        )

    def _publish_event_locked(
        self,
        event_type: str,
        data: Mapping[str, Any],
        *,
        timestamp: float,
    ) -> None:
        """Keep task state authoritative if streaming has already closed."""
        try:
            self.events.publish(
                event_type,
                data,
                timestamp=timestamp,
            )
        except EventHubClosed:
            return

    def _task_locked(self, task_id: str) -> Dict[str, Any]:
        try:
            return self._tasks[task_id]
        except (KeyError, TypeError) as error:
            raise TaskNotFoundError(str(task_id)) from error

    def _new_task_id_locked(self, task_type: str, timestamp: float) -> str:
        timestamp_ms = int(timestamp * 1000.0)
        for _attempt in range(64):
            suffix = self._random_suffix()
            if not isinstance(suffix, str) or not _RANDOM_SUFFIX_PATTERN.fullmatch(
                suffix
            ):
                raise ValueError(
                    "random_suffix must return six lowercase alphanumeric characters."
                )
            task_id = f"{task_type}_{timestamp_ms}_{suffix}"
            if task_id not in self._tasks:
                return task_id
        raise RuntimeError("Could not allocate a unique task ID.")

    def _trim_history_locked(self) -> None:
        while len(self._tasks) > self._history_limit:
            removable = next(
                (
                    task_id
                    for task_id, task in self._tasks.items()
                    if task["status"] in TERMINAL_STATUSES
                    and task_id != self._active_task_id
                ),
                None,
            )
            if removable is None:
                return
            self._tasks.pop(removable, None)
            self._task_monotonic.pop(removable, None)
            self._cancel_callbacks.pop(removable, None)

    def _run_executor(self, task_id: str, executor: TaskExecutor) -> None:
        context = TaskContext(self, task_id)
        try:
            started = self.start_task(task_id)
            if started["status"] == "canceling":
                self.mark_canceled(task_id)
                return
            result = executor(context)
            if result is not None and not isinstance(result, Mapping):
                raise TaskExecutionError(
                    "Task executor returned a non-mapping result.",
                    code="invalid_executor_result",
                )
            if context.cancellation_requested:
                self.mark_canceled(task_id, result)
            else:
                self.succeed_task(task_id, result)
        except TaskCanceledError as error:
            try:
                self.mark_canceled(
                    task_id,
                    reason=str(error) or "Task was canceled.",
                )
            except InvalidTaskTransitionError:
                return
        except TaskExecutionError as error:
            try:
                self.fail_task(
                    task_id,
                    error.reason,
                    code=error.code,
                    details=error.details,
                    result=error.result,
                )
            except InvalidTaskTransitionError:
                return
        except InvalidTaskTransitionError:
            # A result callback may already have completed this task.
            return
        except Exception as error:  # noqa: BLE001
            try:
                self.fail_task(
                    task_id,
                    str(error) or type(error).__name__,
                    code="internal_error",
                    details={"exception_type": type(error).__name__},
                )
            except InvalidTaskTransitionError:
                # Cancellation can become terminal while an executor unwinds.
                return
        finally:
            with self._condition:
                self._workers.pop(task_id, None)
                self._condition.notify_all()


def _snapshot(task: Mapping[str, Any]) -> Dict[str, Any]:
    return copy.deepcopy(dict(task))


def _validate_task_type(task_type: Any) -> str:
    if not isinstance(task_type, str) or not _TASK_TYPE_PATTERN.fullmatch(task_type):
        raise ValueError(f"Task type must match {_TASK_TYPE_PATTERN.pattern}.")
    return task_type


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string.")
    return value.strip()


def _progress_value(value: Any) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError("Task progress must be a finite number.")
    value = float(value)
    if not 0.0 <= value <= 1.0:
        raise ValueError("Task progress must be between 0 and 1.")
    return value


def _finite_timestamp(value: Any, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError(f"{label} must return a finite number.")
    return float(value)


def _parse_event_id(value: Optional[str | int]) -> Optional[int]:
    if value is None or value == "":
        return None
    if isinstance(value, bool):
        raise ValueError("Last-Event-ID must be a non-negative integer.")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError("Last-Event-ID must be a non-negative integer.") from error
    if parsed < 0 or str(parsed) != str(value).strip():
        raise ValueError("Last-Event-ID must be a non-negative integer.")
    return parsed
