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

    def test_hub_close_wakes_subscription(self) -> None:
        hub = EventHub()
        subscription = hub.subscribe()
        hub.close()
        with self.assertRaises(EventHubClosed):
            subscription.get(timeout=0.1)
        with self.assertRaises(EventHubClosed):
            hub.publish("task_progress", {})


if __name__ == "__main__":
    unittest.main()
