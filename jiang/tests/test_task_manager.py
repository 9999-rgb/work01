"""Unit tests for task mutual exclusion, events, and cancellation."""

from __future__ import annotations

import json
import queue
import re
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from typing import Any, Dict, List


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
# ``control_gateway.__init__`` imports ROS; these lifecycle tests stay pure.
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.task_manager import EventHub  # noqa: E402
from control_gateway.task_manager import EventHubClosed  # noqa: E402
from control_gateway.task_manager import TaskConflictError  # noqa: E402
from control_gateway.task_manager import TaskCanceledError  # noqa: E402
from control_gateway.task_manager import TaskExecutionError  # noqa: E402
from control_gateway.task_manager import TaskManager  # noqa: E402
from control_gateway.task_manager import TaskManagerClosedError  # noqa: E402
from control_gateway.task_manager import TaskNotFoundError  # noqa: E402


class _Clock:
    def __init__(self, value: float, increment: float = 0.001) -> None:
        self.value = value
        self.increment = increment
        self.lock = threading.Lock()

    def __call__(self) -> float:
        with self.lock:
            value = self.value
            self.value += self.increment
            return value


class TaskManagerTest(unittest.TestCase):
    def _manager(self, *, history_limit: int = 8) -> TaskManager:
        return TaskManager(
            history_limit=history_limit,
            clock=_Clock(1_700_000_000.123, 0.001),
            monotonic_clock=_Clock(100.0, 0.25),
            random_suffix=lambda: "abc123",
        )

    def test_task_id_lifecycle_and_structured_failure(self) -> None:
        manager = self._manager()
        task = manager.create_task(
            "operate",
            {"cabinet": "cabinet_a", "control_id": "button_1"},
        )
        self.assertRegex(
            task["task_id"],
            re.compile(r"^operate_\d{13}_abc123$"),
        )
        manager.start_task(task["task_id"], phase="planning")
        manager.report_progress(
            task["task_id"],
            "approaching",
            0.4,
            data={"distance": 0.12},
        )
        completed = manager.fail_task(
            task["task_id"],
            "力度不足",
            code="insufficient_force",
            details={"requested_force": 4.0, "minimum_force": 4.8},
            result={"actual_displacement": 0.005},
        )

        self.assertEqual("failed", completed["status"])
        self.assertEqual("力度不足", completed["failure_reason"])
        self.assertEqual("insufficient_force", completed["failure_code"])
        self.assertEqual(0.005, completed["result"]["actual_displacement"])
        self.assertIsNone(manager.active_task_id)
        events = manager.events.events_after()
        self.assertEqual(
            [
                "task_accepted",
                "task_progress",
                "task_progress",
                "task_completed",
            ],
            [event["event"] for event in events],
        )
        self.assertEqual("failed", events[-1]["data"]["outcome"])

    def test_only_one_concurrent_create_wins_atomically(self) -> None:
        manager = self._manager()
        barrier = threading.Barrier(8)
        accepted: List[Dict[str, Any]] = []
        conflicts: List[str] = []
        result_lock = threading.Lock()

        def create(index: int) -> None:
            barrier.wait()
            try:
                task = manager.create_task("navigate", {"index": index})
                with result_lock:
                    accepted.append(task)
            except TaskConflictError as error:
                with result_lock:
                    conflicts.append(error.active_task_id)

        threads = [threading.Thread(target=create, args=(index,)) for index in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)

        self.assertEqual(1, len(accepted))
        self.assertEqual(7, len(conflicts))
        self.assertEqual({accepted[0]["task_id"]}, set(conflicts))
        self.assertEqual(accepted[0]["task_id"], manager.active_task_id)

    def test_cancel_callback_waits_for_terminal_confirmation(self) -> None:
        manager = self._manager()
        callback_calls: List[str] = []
        task = manager.create_task(
            "navigate",
            {"cabinet": "cabinet_a"},
            cancel_callback=lambda task_id: callback_calls.append(task_id)
            or {"accepted": True},
        )
        manager.start_task(task["task_id"])

        canceling = manager.cancel_task(task["task_id"])
        self.assertEqual("canceling", canceling["status"])
        self.assertTrue(canceling["cancel_requested"])
        self.assertEqual([task["task_id"]], callback_calls)
        self.assertEqual(task["task_id"], manager.active_task_id)
        canceled = manager.mark_canceled(task["task_id"], {"backend": "nav2"})
        self.assertEqual("canceled", canceled["status"])
        self.assertEqual("canceled", canceled["failure_code"])
        self.assertIsNone(manager.active_task_id)

    def test_cancel_callback_in_flight_keeps_global_reservation(self) -> None:
        manager = self._manager()
        callback_started = threading.Event()
        release_callback = threading.Event()

        def cancel_backend(_task_id: str) -> Dict[str, bool]:
            callback_started.set()
            release_callback.wait(timeout=2.0)
            return {"accepted": True}

        task = manager.create_task(
            "navigate",
            {"cabinet": "cabinet_a"},
            cancel_callback=cancel_backend,
        )
        manager.start_task(task["task_id"])
        cancel_thread = threading.Thread(
            target=manager.cancel_task,
            args=(task["task_id"],),
        )
        cancel_thread.start()
        self.assertTrue(callback_started.wait(timeout=1.0))

        terminal = manager.mark_canceled(task["task_id"])
        self.assertTrue(terminal["reservation_active"])
        self.assertEqual(task["task_id"], manager.active_task_id)
        with self.assertRaises(TaskConflictError):
            manager.create_task("operate", {"cabinet": "cabinet_b"})

        release_callback.set()
        cancel_thread.join(timeout=1.0)
        self.assertFalse(cancel_thread.is_alive())
        released = manager.get_task(task["task_id"])
        self.assertFalse(released["reservation_active"])
        self.assertTrue(released["backend_termination_confirmed"])
        self.assertIsNone(manager.active_task_id)

    def test_cancel_without_backend_finishes_immediately(self) -> None:
        manager = self._manager()
        task = manager.create_task("operate", {"control_id": "button_1"})
        canceled = manager.cancel_task(task["task_id"])
        self.assertEqual("canceled", canceled["status"])
        self.assertEqual(canceled, manager.cancel_task(task["task_id"]))

    def test_submitted_executor_maps_success_and_known_failure(self) -> None:
        manager = self._manager()

        success = manager.submit(
            "navigate",
            {"cabinet": "cabinet_a"},
            lambda context: (
                context.progress("navigating", 0.5, data={"remaining": 1.0})
                and {"pose_error": 0.02}
            ),
        )
        succeeded = manager.wait(success["task_id"], timeout=2.0)
        self.assertEqual("success", succeeded["status"])
        self.assertEqual(0.02, succeeded["result"]["pose_error"])

        def fail(_context: Any) -> None:
            raise TaskExecutionError(
                "目标不可达",
                code="target_unreachable",
                details={"cabinet": "cabinet_b"},
            )

        failure = manager.submit(
            "operate",
            {"cabinet": "cabinet_b"},
            fail,
        )
        failed = manager.wait(failure["task_id"], timeout=2.0)
        self.assertEqual("failed", failed["status"])
        self.assertEqual("target_unreachable", failed["failure_code"])

    def test_json_boundaries_reject_non_finite_state_transactionally(self) -> None:
        manager = self._manager()
        with self.assertRaisesRegex(ValueError, "finite"):
            manager.create_task("operate", {"force": float("nan")})
        self.assertIsNone(manager.active_task_id)
        self.assertEqual([], manager.events.events_after())

        task = manager.create_task("operate", {"force": 5.0})
        manager.start_task(task["task_id"])
        with self.assertRaisesRegex(ValueError, "finite"):
            manager.report_progress(
                task["task_id"],
                "contact",
                0.5,
                data={"measured_force": float("inf")},
            )
        unchanged = manager.get_task(task["task_id"])
        self.assertEqual({}, unchanged["business_data"])
        self.assertEqual("running", unchanged["status"])

        with self.assertRaisesRegex(ValueError, "finite"):
            manager.succeed_task(
                task["task_id"],
                {"measured_force": float("nan")},
            )
        self.assertEqual("running", manager.get_task(task["task_id"])["status"])
        with self.assertRaisesRegex(ValueError, "finite"):
            manager.fail_task(
                task["task_id"],
                "backend failed",
                details={"measured_force": float("-inf")},
            )
        self.assertEqual("running", manager.get_task(task["task_id"])["status"])
        manager.fail_task(task["task_id"], "safe failure")

    def test_progress_text_rejects_non_utf8_before_mutation(self) -> None:
        manager = self._manager()
        task = manager.create_task("operate", {"force": 5.0})
        manager.start_task(task["task_id"])
        before = manager.get_task(task["task_id"])

        with self.assertRaisesRegex(ValueError, "UTF-8"):
            manager.report_progress(
                task["task_id"],
                "invalid\ud800phase",
                0.5,
            )
        self.assertEqual(before, manager.get_task(task["task_id"]))
        manager.fail_task(task["task_id"], "safe failure")

    def test_reservation_details_are_validated_before_release(self) -> None:
        manager = self._manager()
        task = manager.create_task("navigate", {"cabinet": "cabinet_a"})
        manager.fail_task(
            task["task_id"],
            "backend cleanup pending",
            retain_reservation=True,
        )
        with self.assertRaisesRegex(ValueError, "finite"):
            manager.release_reservation(
                task["task_id"],
                backend_termination_confirmed=True,
                details={"elapsed": float("nan")},
            )
        retained = manager.get_task(task["task_id"])
        self.assertTrue(retained["reservation_active"])
        self.assertEqual(task["task_id"], manager.active_task_id)
        manager.release_reservation(
            task["task_id"],
            backend_termination_confirmed=True,
            details={"elapsed": 0.5},
        )

    def test_invalid_executor_results_fail_with_encodable_events(self) -> None:
        for invalid_result in (
            {"measurement": float("nan")},
            {"value": object()},
            ["not", "a", "mapping"],
        ):
            with self.subTest(result=type(invalid_result).__name__):
                manager = self._manager()
                task = manager.submit(
                    "operate",
                    {"cabinet": "cabinet_a"},
                    lambda _context, value=invalid_result: value,
                )
                failed = manager.wait(task["task_id"], timeout=1.0)
                self.assertEqual("failed", failed["status"])
                self.assertEqual(
                    "invalid_executor_result",
                    failed["failure_code"],
                )
                for event in manager.events.events_after():
                    json.dumps(event, allow_nan=False).encode("utf-8")

    def test_invalid_executor_exception_text_still_finishes_safely(self) -> None:
        manager = self._manager()

        def fail_with_invalid_text(_context: Any) -> None:
            raise RuntimeError("invalid\ud800text")

        task = manager.submit(
            "operate",
            {"cabinet": "cabinet_a"},
            fail_with_invalid_text,
        )
        failed = manager.wait(task["task_id"], timeout=1.0)
        self.assertEqual("failed", failed["status"])
        self.assertEqual("internal_error", failed["failure_code"])
        self.assertEqual("RuntimeError", failed["failure_reason"])
        for event in manager.events.events_after():
            json.dumps(event, allow_nan=False).encode("utf-8")

    def test_invalid_structured_failure_text_still_finishes_safely(self) -> None:
        manager = self._manager()

        def fail_with_invalid_text(_context: Any) -> None:
            raise TaskExecutionError("invalid\ud800text")

        task = manager.submit(
            "operate",
            {"cabinet": "cabinet_a"},
            fail_with_invalid_text,
        )
        failed = manager.wait(task["task_id"], timeout=1.0)
        self.assertEqual("failed", failed["status"])
        self.assertEqual("internal_error", failed["failure_code"])
        for event in manager.events.events_after():
            json.dumps(event, allow_nan=False).encode("utf-8")

    def test_non_json_cancellation_acknowledgement_cannot_poison_task(self) -> None:
        manager = self._manager()
        task = manager.create_task(
            "navigate",
            {"cabinet": "cabinet_a"},
            cancel_callback=lambda _task_id: {
                "accepted": True,
                "latency": float("nan"),
            },
        )
        manager.start_task(task["task_id"])
        canceling = manager.cancel_task(task["task_id"])
        self.assertEqual("canceling", canceling["status"])
        self.assertEqual(
            {
                "accepted": True,
                "acknowledgement_discarded": "not_json_safe",
            },
            canceling["cancel_acknowledgement"],
        )
        terminal = manager.mark_canceled(task["task_id"])
        json.dumps(terminal, allow_nan=False).encode("utf-8")
        for event in manager.events.events_after():
            json.dumps(event, allow_nan=False).encode("utf-8")

    def test_backend_success_is_authoritative_over_late_cancel_request(
        self,
    ) -> None:
        manager = self._manager()
        backend_terminal = threading.Event()
        return_result = threading.Event()

        def execute(context: Any) -> Any:
            backend_terminal.set()
            return_result.wait(timeout=1.0)
            return context.backend_succeeded({"backend_state": "succeeded"})

        task = manager.submit(
            "navigate",
            {"cabinet": "cabinet_a"},
            execute,
            cancel_callback=lambda _task_id: {"accepted": True},
        )
        self.assertTrue(backend_terminal.wait(timeout=1.0))

        canceling = manager.cancel_task(task["task_id"])
        self.assertEqual("canceling", canceling["status"])
        return_result.set()
        terminal = manager.wait(task["task_id"], timeout=1.0)

        self.assertEqual("success", terminal["status"])
        self.assertTrue(terminal["cancel_requested"])
        self.assertEqual("succeeded", terminal["result"]["backend_state"])

    def test_plain_executor_result_remains_cooperatively_canceled(self) -> None:
        manager = self._manager()
        executor_started = threading.Event()
        return_result = threading.Event()

        def execute(_context: Any) -> Dict[str, bool]:
            executor_started.set()
            return_result.wait(timeout=1.0)
            return {"ignored_cancel": True}

        task = manager.submit(
            "operate",
            {"cabinet": "cabinet_a"},
            execute,
            cancel_callback=lambda _task_id: {"accepted": True},
        )
        self.assertTrue(executor_started.wait(timeout=1.0))
        manager.cancel_task(task["task_id"])
        return_result.set()
        terminal = manager.wait(task["task_id"], timeout=1.0)

        self.assertEqual("canceled", terminal["status"])
        self.assertTrue(terminal["result"]["ignored_cancel"])

    def test_plain_executor_completion_linearizes_with_concurrent_cancel(
        self,
    ) -> None:
        for attempt in range(64):
            with self.subTest(attempt=attempt):
                manager = self._manager()
                finish_barrier = threading.Barrier(2)

                def execute(_context: Any) -> Dict[str, int]:
                    finish_barrier.wait(timeout=1.0)
                    return {"attempt": attempt}

                task = manager.submit(
                    "operate",
                    {"attempt": attempt},
                    execute,
                    cancel_callback=lambda _task_id: {"accepted": True},
                )
                cancel_result: Dict[str, Any] = {}

                def cancel() -> None:
                    finish_barrier.wait(timeout=1.0)
                    cancel_result.update(manager.cancel_task(task["task_id"]))

                cancel_thread = threading.Thread(target=cancel)
                cancel_thread.start()
                terminal = manager.wait(task["task_id"], timeout=1.0)
                cancel_thread.join(timeout=1.0)

                self.assertFalse(cancel_thread.is_alive())
                self.assertIn(terminal["status"], {"success", "canceled"})
                if terminal["status"] == "success":
                    self.assertFalse(terminal["cancel_requested"])
                    self.assertEqual("success", cancel_result["status"])
                else:
                    self.assertTrue(terminal["cancel_requested"])

    def test_explicit_canceled_error_overrides_backend_success_marker_path(
        self,
    ) -> None:
        manager = self._manager()

        def execute(_context: Any) -> None:
            raise TaskCanceledError("Backend confirmed cancellation.")

        task = manager.submit(
            "navigate",
            {"cabinet": "cabinet_a"},
            execute,
        )
        terminal = manager.wait(task["task_id"], timeout=1.0)

        self.assertEqual("canceled", terminal["status"])
        self.assertEqual(
            "Backend confirmed cancellation.",
            terminal["failure_reason"],
        )

    def test_history_is_bounded_and_snapshots_are_isolated(self) -> None:
        manager = self._manager(history_limit=2)
        first = manager.create_task("navigate", {"cabinet": "cabinet_a"})
        manager.succeed_task(first["task_id"], {"value": 1})
        second = manager.create_task("navigate", {"cabinet": "cabinet_b"})
        manager.succeed_task(second["task_id"], {"value": 2})
        third = manager.create_task("navigate", {"cabinet": "cabinet_c"})
        manager.succeed_task(third["task_id"], {"value": 3})

        with self.assertRaises(TaskNotFoundError):
            manager.get_task(first["task_id"])
        snapshots = manager.list_tasks()
        snapshots[0]["result"]["value"] = 999
        self.assertEqual(3, manager.get_task(third["task_id"])["result"]["value"])

    def test_terminal_timeout_retains_global_reservation_until_backend_exit(
        self,
    ) -> None:
        manager = self._manager()
        task = manager.create_task("navigate", {"cabinet": "cabinet_a"})
        manager.start_task(task["task_id"])

        failed = manager.fail_task(
            task["task_id"],
            "navigation timeout",
            code="navigation_timeout",
            retain_reservation=True,
        )

        self.assertEqual("failed", failed["status"])
        self.assertTrue(failed["reservation_active"])
        self.assertEqual(task["task_id"], manager.active_task_id)
        with self.assertRaises(TaskConflictError):
            manager.create_task("operate", {"cabinet": "cabinet_b"})

        released = manager.release_reservation(
            task["task_id"],
            backend_termination_confirmed=True,
            details={"backend_terminal_state": "canceled"},
        )
        self.assertFalse(released["reservation_active"])
        self.assertTrue(released["backend_termination_confirmed"])
        self.assertIsNone(manager.active_task_id)
        self.assertEqual(
            ["task_completed", "task_reservation_released"],
            [
                event["event"]
                for event in manager.events.events_after()
                if event["event"].startswith("task_")
            ][-2:],
        )

    def test_exception_after_terminal_retention_does_not_leak_slot(self) -> None:
        """An exception escaping a monitor that already went terminal-retained
        must release the global slot, otherwise every later /task/* submission
        raises TaskConflictError forever."""
        manager = self._manager()

        def fail_then_escape(context: Any) -> None:
            context.fail_retaining_reservation(
                "navigation timeout",
                code="navigation_timeout",
            )
            raise RuntimeError("bug escaped the navigation monitor loop")

        failed = manager.submit(
            "navigate",
            {"cabinet": "cabinet_a"},
            fail_then_escape,
        )
        terminal = manager.wait(failed["task_id"], timeout=2.0)
        self.assertEqual("failed", terminal["status"])
        # The terminal snapshot may be captured before the unwinding executor
        # releases the retained slot, so poll for the release rather than
        # asserting on the wait snapshot.
        deadline = time.monotonic() + 2.0
        while manager.active_task_id is not None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIsNone(manager.active_task_id)
        self.assertFalse(manager.get_task(failed["task_id"])["reservation_active"])
        # The slot must be free: a fresh submission is accepted and runs.
        following = manager.submit(
            "operate",
            {"cabinet": "cabinet_b"},
            lambda context: context.progress("operating", 0.25)
            and {"ok": True},
        )
        self.assertEqual(
            "success", manager.wait(following["task_id"], timeout=2.0)["status"]
        )

    def test_task_execution_error_after_terminal_retention_releases_slot(self) -> None:
        """The TaskExecutionError unwinding path must also release a retained
        slot; without it cancel_retaining_reservation followed by an escaped
        execution error strands the gateway."""
        manager = self._manager()

        def cancel_then_raise(context: Any) -> None:
            context.cancel_retaining_reservation(
                "cancellation unconfirmed by Nav2",
                details={"cancel_grace_seconds": 10.0},
            )
            raise TaskExecutionError(
                "backend wedged",
                code="backend_wedged",
            )

        canceled = manager.submit(
            "navigate",
            {"cabinet": "cabinet_a"},
            cancel_then_raise,
        )
        terminal = manager.wait(canceled["task_id"], timeout=2.0)
        self.assertEqual("canceled", terminal["status"])
        self.assertFalse(terminal["reservation_active"])
        self.assertIsNone(manager.active_task_id)
        manager.submit(
            "operate",
            {"cabinet": "cabinet_b"},
            lambda context: {"ok": True},
        )

    def test_cancel_requested_wins_over_executor_failure(self) -> None:
        """A user cancellation, once requested, must be terminal 'canceled'
        even when the executor fails afterwards (e.g. homing timing out after
        the user canceled), never 'failed'."""
        manager = self._manager()
        executor_started = threading.Event()
        release = threading.Event()

        def execute(_context: Any) -> None:
            executor_started.set()
            release.wait(timeout=2.0)
            raise TaskExecutionError(
                "Robot joints did not reach their configured defaults "
                "before the reset timeout.",
                code="robot_joint_reset_timeout",
            )

        task = manager.submit(
            "operate",
            {"cabinet": "cabinet_a"},
            execute,
            cancel_callback=lambda _task_id: {"accepted": True},
        )
        self.assertTrue(executor_started.wait(timeout=1.0))
        manager.cancel_task(task["task_id"])
        release.set()
        terminal = manager.wait(task["task_id"], timeout=2.0)

        self.assertEqual("canceled", terminal["status"])
        self.assertEqual("canceled", terminal["failure_code"])
        # The underlying backend failure is preserved in the cancel reason.
        self.assertIn("robot_joint_reset_timeout", terminal["failure_reason"])

    def test_closed_event_stream_does_not_corrupt_task_state(self) -> None:
        manager = self._manager()
        manager.events.close()

        task = manager.create_task("operate", {"control_id": "button_1"})
        manager.start_task(task["task_id"])
        completed = manager.succeed_task(task["task_id"], {"ok": True})

        self.assertEqual("success", completed["status"])
        self.assertIsNone(manager.active_task_id)
        self.assertEqual(
            "success",
            manager.wait(task["task_id"], timeout=0.1)["status"],
        )

    def test_shutdown_cancels_and_joins_cooperative_worker(self) -> None:
        manager = self._manager()
        started = threading.Event()

        def execute(context: Any) -> None:
            started.set()
            while True:
                context.raise_if_canceled()
                threading.Event().wait(0.005)

        task = manager.submit(
            "navigate",
            {"cabinet": "cabinet_a"},
            execute,
            cancel_callback=lambda _task_id: {"accepted": True},
        )
        self.assertTrue(started.wait(timeout=1.0))

        report = manager.shutdown(timeout=1.0)

        self.assertTrue(report["workers_stopped"])
        self.assertEqual([], report["pending_worker_task_ids"])
        self.assertEqual(
            "canceled",
            manager.get_task(task["task_id"])["status"],
        )
        with self.assertRaises(TaskManagerClosedError):
            manager.create_task("navigate", {"cabinet": "cabinet_b"})

    def test_shutdown_is_bounded_for_uncooperative_worker(self) -> None:
        manager = self._manager()
        started = threading.Event()
        release = threading.Event()

        def execute(_context: Any) -> Dict[str, bool]:
            started.set()
            release.wait(timeout=2.0)
            return {"released": True}

        task = manager.submit(
            "operate",
            {"cabinet": "cabinet_a"},
            execute,
            cancel_callback=lambda _task_id: {"accepted": True},
        )
        self.assertTrue(started.wait(timeout=1.0))

        report = manager.shutdown(timeout=0.02)

        self.assertFalse(report["workers_stopped"])
        self.assertEqual([task["task_id"]], report["pending_worker_task_ids"])
        release.set()
        final_report = manager.shutdown(timeout=1.0)
        self.assertTrue(final_report["workers_stopped"])

    def test_shutdown_retries_cancel_for_retained_terminal_reservation(
        self,
    ) -> None:
        manager = self._manager()
        canceled: List[str] = []
        task = manager.create_task(
            "navigate",
            {"cabinet": "cabinet_a"},
            cancel_callback=lambda task_id: canceled.append(task_id),
        )
        manager.fail_task(
            task["task_id"],
            "backend did not stop",
            code="navigation_timeout",
            retain_reservation=True,
        )

        report = manager.shutdown(timeout=0.0)

        if not canceled:
            self.assertEqual(
                [task["task_id"]],
                report["pending_cancellation_task_ids"],
            )
        manager.shutdown(timeout=1.0)
        self.assertTrue(canceled)
        self.assertEqual({task["task_id"]}, set(canceled))
        self.assertEqual(task["task_id"], report["active_task_id"])

    def test_shutdown_timeout_covers_blocking_cancel_callback(self) -> None:
        manager = self._manager()
        callback_started = threading.Event()
        release_callback = threading.Event()

        def cancel_backend(_task_id: str) -> Dict[str, bool]:
            callback_started.set()
            release_callback.wait(timeout=2.0)
            return {"accepted": True}

        task = manager.create_task(
            "navigate",
            {"cabinet": "cabinet_a"},
            cancel_callback=cancel_backend,
        )
        started_at = time.monotonic()
        report = manager.shutdown(timeout=0.05)
        elapsed = time.monotonic() - started_at

        self.assertLess(elapsed, 0.5)
        self.assertTrue(callback_started.wait(timeout=1.0))
        self.assertFalse(report["workers_stopped"])
        self.assertEqual(
            [task["task_id"]],
            report["pending_cancellation_task_ids"],
        )
        self.assertEqual(task["task_id"], manager.active_task_id)
        release_callback.set()
        final_report = manager.shutdown(timeout=1.0)
        self.assertTrue(final_report["workers_stopped"])


class EventHubTest(unittest.TestCase):
    def test_publish_rejects_non_json_safe_data_without_consuming_sequence(
        self,
    ) -> None:
        hub = EventHub()
        with self.assertRaisesRegex(ValueError, "finite"):
            hub.publish("task_progress", {"value": float("nan")})
        event = hub.publish("task_progress", {"value": 1.0})
        self.assertEqual("1", event["id"])

    def test_publish_rejects_non_utf8_or_multiline_event_type_without_sequence_gap(
        self,
    ) -> None:
        hub = EventHub()
        for event_type in ("bad\ud800event", "task_progress\nevent"):
            with self.assertRaisesRegex(ValueError, "event_type must match"):
                hub.publish(event_type, {"value": 1.0})
        event = hub.publish("task_progress", {"value": 2.0})
        self.assertEqual("1", event["id"])

    def test_async_notification_covers_replay_live_data_and_close(self) -> None:
        hub = EventHub()
        replay = hub.publish("task_progress", {"value": 1})
        subscription = hub.subscribe(last_event_id=0)
        notified = threading.Event()
        subscription.set_notify(notified.set)
        self.assertTrue(notified.wait(timeout=0.1))
        self.assertEqual(subscription.get_nowait()["id"], replay["id"])

        notified.clear()
        live = hub.publish("task_completed", {"value": 2})
        self.assertTrue(notified.wait(timeout=0.1))
        self.assertEqual(subscription.get_nowait()["id"], live["id"])

        notified.clear()
        hub.close()
        self.assertTrue(notified.wait(timeout=0.1))
        with self.assertRaises(EventHubClosed):
            subscription.get_nowait()

    def test_replay_is_exclusive_and_live_delivery_has_no_gap(self) -> None:
        hub = EventHub(capacity=4)
        first = hub.publish("task_accepted", {"task_id": "one"})
        second = hub.publish("task_progress", {"task_id": "one"})
        subscription = hub.subscribe(last_event_id=first["id"])
        third = hub.publish("task_completed", {"task_id": "one"})

        self.assertEqual(second["id"], subscription.get(timeout=0.1)["id"])
        self.assertEqual(third["id"], subscription.get(timeout=0.1)["id"])
        with self.assertRaises(queue.Empty):
            subscription.get(timeout=0.01)
        subscription.close()
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)

    def test_slow_subscriber_never_drops_terminal_event(self) -> None:
        hub = EventHub(capacity=8)
        subscription = hub.subscribe(max_pending=2)
        hub.publish("task_progress", {"value": 1})
        terminal = hub.publish("task_completed", {"value": 2})
        hub.publish("task_progress", {"value": 3})
        hub.publish("task_progress", {"value": 4})

        pending = [
            subscription.get(timeout=0.1),
            subscription.get(timeout=0.1),
        ]
        self.assertIn(terminal["id"], {event["id"] for event in pending})

    def test_slow_subscriber_coalesces_progress_before_task_acceptance(self) -> None:
        hub = EventHub(capacity=8)
        subscription = hub.subscribe(max_pending=2)
        accepted = hub.publish("task_accepted", {"task_id": "one"})
        hub.publish("task_progress", {"task_id": "one", "value": 1})
        hub.publish("task_progress", {"task_id": "one", "value": 2})

        pending = [
            subscription.get(timeout=0.1),
            subscription.get(timeout=0.1),
        ]
        self.assertIn(accepted["id"], {event["id"] for event in pending})

    def test_lifecycle_overflow_is_bounded_and_explicit(self) -> None:
        hub = EventHub(capacity=8)
        subscription = hub.subscribe(max_pending=1)
        accepted = hub.publish("task_accepted", {"task_id": "one"})
        hub.publish("task_completed", {"task_id": "one"})

        self.assertEqual(
            subscription.get(timeout=0.1)["id"],
            accepted["id"],
        )
        overflow = subscription.get(timeout=0.1)
        self.assertEqual(overflow["event"], "stream_overflow")
        self.assertTrue(overflow["data"]["reconnect_required"])
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)

    def test_progress_coalescing_preserves_global_sequence_order(self) -> None:
        hub = EventHub(capacity=8)
        subscription = hub.subscribe(max_pending=2)
        hub.publish("task_progress", {"task_id": "one", "value": 1})
        canceling = hub.publish(
            "task_cancel_requested",
            {"task_id": "one"},
        )
        latest = hub.publish(
            "task_progress",
            {"task_id": "one", "value": 2},
        )

        pending = [
            subscription.get(timeout=0.1),
            subscription.get(timeout=0.1),
        ]
        self.assertEqual(
            [event["id"] for event in pending],
            [canceling["id"], latest["id"]],
        )

    def test_replay_overflow_does_not_leave_a_dead_hub_subscription(self) -> None:
        hub = EventHub(capacity=8)
        accepted = hub.publish("task_accepted", {"task_id": "one"})
        hub.publish("task_completed", {"task_id": "one"})
        subscription = hub.subscribe(last_event_id=0, max_pending=1)

        self.assertEqual(len(hub._subscriptions), 0)
        self.assertEqual(
            subscription.get(timeout=0.1)["id"],
            accepted["id"],
        )
        self.assertEqual(
            subscription.get(timeout=0.1)["event"],
            "stream_overflow",
        )
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)

    def test_replay_history_gap_is_explicit_instead_of_silent(self) -> None:
        hub = EventHub(capacity=2)
        hub.publish("task_accepted", {"task_id": "one"})
        hub.publish("task_progress", {"task_id": "one"})
        hub.publish("task_completed", {"task_id": "one"})

        subscription = hub.subscribe(last_event_id=0)
        notice = subscription.get(timeout=0.1)
        self.assertEqual(notice["event"], "stream_overflow")
        self.assertEqual(notice["data"]["reason"], "replay_history_gap")
        self.assertEqual(notice["data"]["oldest_available_sequence"], 2)
        self.assertEqual(len(hub._subscriptions), 0)
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)

    def test_concurrent_publishers_preserve_subscription_sequence_order(self) -> None:
        hub = EventHub(capacity=8)
        subscription = hub.subscribe(max_pending=8)
        buffer = next(iter(hub._subscriptions.values()))
        original_put = buffer.put
        first_in_fanout = threading.Event()
        release_first = threading.Event()

        def controlled_put(event):
            if event["id"] == "1":
                first_in_fanout.set()
                self.assertTrue(release_first.wait(timeout=1.0))
            return original_put(event)

        buffer.put = controlled_put
        workers = [
            threading.Thread(
                target=lambda value=value: hub.publish(
                    "task_progress",
                    {"task_id": str(value)},
                )
            )
            for value in (1, 2)
        ]
        workers[0].start()
        self.assertTrue(first_in_fanout.wait(timeout=1.0))
        workers[1].start()
        # Publisher two must not fan out around publisher one's blocked put.
        workers[1].join(timeout=0.02)
        self.assertTrue(workers[1].is_alive())
        release_first.set()
        for worker in workers:
            worker.join(timeout=1.0)
            self.assertFalse(worker.is_alive())

        delivered = [
            subscription.get(timeout=0.1),
            subscription.get(timeout=0.1),
        ]
        self.assertEqual(
            [event["id"] for event in delivered],
            [event["id"] for event in hub.events_after()],
        )
        self.assertEqual([event["id"] for event in delivered], ["1", "2"])

    def test_notify_reentrant_publish_reaches_all_subscribers_in_order(self) -> None:
        hub = EventHub(capacity=8)
        first = hub.subscribe(max_pending=8)
        second = hub.subscribe(max_pending=8)
        republished = False

        def notify() -> None:
            nonlocal republished
            if republished:
                return
            republished = True
            hub.publish("task_progress", {"task_id": "nested"})

        first.set_notify(notify)
        worker = threading.Thread(
            target=lambda: hub.publish(
                "task_accepted",
                {"task_id": "outer"},
            )
        )
        worker.start()
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(
            [
                first.get(timeout=0.1)["id"],
                first.get(timeout=0.1)["id"],
            ],
            ["1", "2"],
        )
        self.assertEqual(
            [
                second.get(timeout=0.1)["id"],
                second.get(timeout=0.1)["id"],
            ],
            ["1", "2"],
        )

    def test_hub_close_wakes_subscription(self) -> None:
        hub = EventHub()
        subscription = hub.subscribe()
        hub.close()
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)
        with self.assertRaises(EventHubClosed):
            hub.publish("task_progress", {})

    def test_close_waits_for_in_flight_publication_fanout(self) -> None:
        hub = EventHub()
        subscription = hub.subscribe()
        buffer = next(iter(hub._subscriptions.values()))
        original_put = buffer.put
        fanout_started = threading.Event()
        release_fanout = threading.Event()

        def controlled_put(event):
            fanout_started.set()
            self.assertTrue(release_fanout.wait(timeout=1.0))
            return original_put(event)

        buffer.put = controlled_put
        published = []
        publisher = threading.Thread(
            target=lambda: published.append(
                hub.publish("task_completed", {"task_id": "one"})
            )
        )
        publisher.start()
        self.assertTrue(fanout_started.wait(timeout=1.0))

        closed = threading.Event()

        def close_hub() -> None:
            hub.close()
            closed.set()

        closer = threading.Thread(target=close_hub)
        closer.start()
        self.assertFalse(closed.wait(timeout=0.02))
        release_fanout.set()
        publisher.join(timeout=1.0)
        closer.join(timeout=1.0)
        self.assertFalse(publisher.is_alive())
        self.assertFalse(closer.is_alive())

        delivered = subscription.get(timeout=0.1)
        self.assertEqual(delivered["id"], published[0]["id"])
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)


if __name__ == "__main__":
    unittest.main()
