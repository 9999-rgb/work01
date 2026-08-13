"""Unit tests for safe high-level task scenario replay."""

from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.task_replay import TaskReplayConflictError  # noqa: E402
from control_gateway.task_replay import TaskReplayOrchestrator  # noqa: E402
from control_gateway.task_replay import TaskReplayValidationError  # noqa: E402


def _scenario(
    *steps: Mapping[str, Any],
    schema_version: int = 1,
) -> Dict[str, Any]:
    return {
        "schema_version": schema_version,
        "recording_id": "sample",
        "steps": list(steps),
    }


class _Backend:
    def __init__(self, scenario: Mapping[str, Any]) -> None:
        self.scenario = scenario
        self.submissions: List[Dict[str, Any]] = []
        self.tasks: Dict[str, Dict[str, Any]] = {}
        self.canceled: List[str] = []
        self.submitted = threading.Event()

    def load(self, recording_id: str) -> Mapping[str, Any]:
        if recording_id == "missing":
            raise FileNotFoundError(recording_id)
        return self.scenario

    def navigate(
        self,
        cabinet: str,
        control_id: Optional[str],
    ) -> Mapping[str, Any]:
        return self._submit(
            {"type": "navigate", "cabinet": cabinet, "control_id": control_id}
        )

    def operate(
        self,
        cabinet: str,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Mapping[str, Any]:
        return self._submit(
            {
                "type": "operate",
                "cabinet": cabinet,
                "control_id": control_id,
                "command": command,
                "target_state": target_state,
                "target_position": target_position,
                "force": force,
            }
        )

    def reset(self, cabinet: str) -> Mapping[str, Any]:
        return self._submit({"type": "reset", "cabinet": cabinet})

    def _submit(self, value: Dict[str, Any]) -> Mapping[str, Any]:
        task_id = f"task_{len(self.submissions)}"
        self.submissions.append(value)
        self.tasks[task_id] = {
            "task_id": task_id,
            "status": "running",
            "reservation_active": True,
        }
        self.submitted.set()
        return {"task_id": task_id, "status": "accepted"}

    def status(self, task_id: str) -> Mapping[str, Any]:
        return dict(self.tasks[task_id])

    def finish(
        self,
        task_id: str,
        status: str = "success",
        *,
        reason: Optional[str] = None,
    ) -> None:
        self.tasks[task_id].update(
            status=status,
            reservation_active=False,
            failure_code=(None if status == "success" else "test_failure"),
            failure_reason=reason,
        )

    def cancel(self, task_id: str) -> Mapping[str, Any]:
        self.canceled.append(task_id)
        self.finish(task_id, "canceled", reason="canceled")
        return dict(self.tasks[task_id])


class _BlockingCancelBackend(_Backend):
    def __init__(self, scenario: Mapping[str, Any]) -> None:
        super().__init__(scenario)
        self.cancel_entered = threading.Event()
        self.cancel_release = threading.Event()

    def cancel(self, task_id: str) -> Mapping[str, Any]:
        self.canceled.append(task_id)
        self.cancel_entered.set()
        self.cancel_release.wait(timeout=2.0)
        self.finish(task_id, "canceled", reason="canceled")
        return dict(self.tasks[task_id])


class _BlockingSubmitBackend(_Backend):
    def __init__(self, scenario: Mapping[str, Any]) -> None:
        super().__init__(scenario)
        self.submit_entered = threading.Event()
        self.submit_release = threading.Event()

    def _submit(self, value: Dict[str, Any]) -> Mapping[str, Any]:
        self.submit_entered.set()
        self.submit_release.wait(timeout=2.0)
        return super()._submit(value)


class _FailingCancelBackend(_Backend):
    def cancel(self, task_id: str) -> Mapping[str, Any]:
        self.canceled.append(task_id)
        raise RuntimeError("injected cancellation failure")


def _orchestrator(backend: _Backend) -> TaskReplayOrchestrator:
    return TaskReplayOrchestrator(
        load_scenario=backend.load,
        submit_navigation=backend.navigate,
        submit_operation=backend.operate,
        submit_reset=backend.reset,
        task_status=backend.status,
        cancel_task=backend.cancel,
        poll_period=0.005,
    )


class TaskReplayTest(unittest.TestCase):
    def test_replays_navigation_then_operation_through_callbacks(self) -> None:
        backend = _Backend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "box_10_button_1",
                        "station": {"x": 999.0},
                    },
                },
                {
                    "type": "operate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "box_10_button_1",
                        "command": "press",
                        "target_state": None,
                        "target_position": None,
                        "force": 5.0,
                    },
                },
            )
        )
        replay = _orchestrator(backend)

        accepted = replay.start("sample")
        self.assertEqual("running", accepted["status"])
        self.assertTrue(backend.submitted.wait(timeout=1.0))
        backend.finish("task_0")
        deadline = time.monotonic() + 1.0
        while len(backend.submissions) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        backend.finish("task_1")
        deadline = time.monotonic() + 1.0
        while replay.is_active and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(replay.shutdown(timeout=1.0))

        self.assertEqual(
            [
                {
                    "type": "navigate",
                    "cabinet": "cabinet_a",
                    "control_id": "box_10_button_1",
                },
                {
                    "type": "operate",
                    "cabinet": "cabinet_a",
                    "control_id": "box_10_button_1",
                    "command": "press",
                    "target_state": None,
                    "target_position": None,
                    "force": 5.0,
                },
            ],
            backend.submissions,
        )
        self.assertEqual("success", replay.status()["status"])
        self.assertEqual(1.0, replay.status()["progress"])

    def test_schema_v2_replays_reset_before_following_task(self) -> None:
        backend = _Backend(
            _scenario(
                {
                    "type": "reset",
                    "request": {"cabinet": "cabinet_a"},
                },
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                },
                schema_version=2,
            )
        )
        replay = _orchestrator(backend)

        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))
        self.assertEqual(
            [{"type": "reset", "cabinet": "cabinet_a"}],
            backend.submissions,
        )
        backend.finish("task_0")
        deadline = time.monotonic() + 1.0
        while len(backend.submissions) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual("navigate", backend.submissions[1]["type"])
        backend.finish("task_1")
        deadline = time.monotonic() + 1.0
        while replay.is_active and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(replay.shutdown(timeout=1.0))
        self.assertEqual("success", replay.status()["status"])

    def test_failure_stops_following_steps_and_preserves_reason(self) -> None:
        backend = _Backend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                },
                {
                    "type": "operate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "button",
                        "command": "press",
                    },
                },
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))
        backend.finish("task_0", "failed", reason="target blocked")
        deadline = time.monotonic() + 1.0
        while replay.is_active and time.monotonic() < deadline:
            time.sleep(0.005)

        self.assertEqual(1, len(backend.submissions))
        self.assertEqual("failed", replay.status()["status"])
        self.assertIn("target blocked", replay.status()["error"])

    def test_cancel_routes_to_current_task_and_prevents_next_step(
        self,
    ) -> None:
        backend = _Backend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                },
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_b"},
                },
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))

        canceling = replay.cancel()
        self.assertEqual("canceling", canceling["status"])
        self.assertTrue(replay.shutdown(timeout=1.0))

        self.assertEqual(["task_0"], backend.canceled)
        self.assertEqual(1, len(backend.submissions))
        self.assertEqual("canceled", replay.status()["status"])

    def test_shutdown_timeout_is_not_blocked_by_backend_cancel(self) -> None:
        backend = _BlockingCancelBackend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                }
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))

        started = time.monotonic()
        self.assertFalse(replay.shutdown(timeout=0.03))
        elapsed = time.monotonic() - started

        self.assertLess(elapsed, 0.25)
        self.assertTrue(backend.cancel_entered.wait(timeout=1.0))
        self.assertTrue(replay.is_active)
        self.assertEqual("canceling", replay.status()["status"])
        self.assertEqual("task_0", replay.status()["current_task_id"])
        with self.assertRaises(TaskReplayConflictError):
            replay.start("sample")

        backend.cancel_release.set()
        self.assertTrue(replay.shutdown(timeout=1.0))
        self.assertEqual("canceled", replay.status()["status"])

    def test_cancel_during_submission_targets_returned_task(self) -> None:
        backend = _BlockingSubmitBackend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                },
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_b"},
                },
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submit_entered.wait(timeout=1.0))

        canceling = replay.cancel()
        self.assertEqual("canceling", canceling["status"])
        self.assertIn("in-flight submission", canceling["message"])
        self.assertEqual([], backend.canceled)
        self.assertFalse(replay.shutdown(timeout=0.02))

        backend.submit_release.set()
        self.assertTrue(replay.shutdown(timeout=1.0))
        self.assertEqual(["task_0"], backend.canceled)
        self.assertEqual(1, len(backend.submissions))
        self.assertEqual("canceled", replay.status()["status"])

    def test_cancel_failure_retries_boundedly_and_keeps_ownership(
        self,
    ) -> None:
        backend = _FailingCancelBackend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                }
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))

        canceling = replay.cancel()
        self.assertEqual("canceling", canceling["status"])
        deadline = time.monotonic() + 1.0
        while len(backend.canceled) < 3 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(3, len(backend.canceled))
        time.sleep(0.03)
        self.assertEqual(3, len(backend.canceled))
        self.assertFalse(replay.shutdown(timeout=0.02))
        self.assertTrue(replay.is_active)
        self.assertIn(
            "3 backend cancellation attempts",
            replay.status()["message"],
        )
        with self.assertRaises(TaskReplayConflictError):
            replay.start("sample")

        backend.finish("task_0", "canceled", reason="manual cleanup")
        self.assertTrue(replay.shutdown(timeout=1.0))
        self.assertEqual("canceled", replay.status()["status"])

    def test_blocking_cancel_retains_ownership_until_callback_returns(
        self,
    ) -> None:
        backend = _BlockingCancelBackend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                }
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))

        started = time.monotonic()
        canceling = replay.cancel()
        elapsed = time.monotonic() - started

        self.assertLess(elapsed, 0.25)
        self.assertEqual("canceling", canceling["status"])
        self.assertTrue(backend.cancel_entered.wait(timeout=1.0))
        backend.finish("task_0", "canceled", reason="already stopped")
        time.sleep(0.03)

        self.assertTrue(replay.is_active)
        self.assertFalse(replay.shutdown(timeout=0.02))
        with self.assertRaises(TaskReplayConflictError):
            replay.start("sample")

        backend.cancel_release.set()
        self.assertTrue(replay.shutdown(timeout=1.0))
        self.assertEqual(["task_0"], backend.canceled)

    def test_rejects_concurrent_replay(self) -> None:
        backend = _Backend(
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a"},
                }
            )
        )
        replay = _orchestrator(backend)
        replay.start("sample")
        self.assertTrue(backend.submitted.wait(timeout=1.0))
        with self.assertRaises(TaskReplayConflictError):
            replay.start("sample")
        replay.cancel()
        self.assertTrue(replay.shutdown(timeout=1.0))

    def test_strictly_rejects_unsafe_or_malformed_scenarios(self) -> None:
        invalid = [
            {"schema_version": 2, "steps": []},
            {
                "schema_version": True,
                "steps": [
                    {
                        "type": "navigate",
                        "request": {"cabinet": "cabinet_a"},
                    }
                ],
            },
            _scenario(),
            _scenario({"type": "cmd_vel", "request": {}}),
            _scenario(
                {"type": "reset", "request": {"cabinet": "cabinet_a"}}
            ),
            _scenario(
                {
                    "type": "reset",
                    "request": {"cabinet": "cabinet_a", "all": True},
                },
                schema_version=2,
            ),
            _scenario(
                {
                    "type": "navigate",
                    "request": {"cabinet": "cabinet_a", "x": 1.0},
                }
            ),
            _scenario(
                {
                    "type": "operate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "button",
                        "command": "press",
                        "force": 0.0,
                    },
                }
            ),
            _scenario(
                {
                    "type": "operate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "button",
                        "command": 1,
                    },
                }
            ),
            _scenario(
                {
                    "type": "operate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "button",
                        "command": "set_state",
                    },
                }
            ),
            _scenario(
                {
                    "type": "operate",
                    "request": {
                        "cabinet": "cabinet_a",
                        "control_id": "button",
                        "command": "press",
                        "target_position": 0.5,
                    },
                }
            ),
        ]
        for scenario in invalid:
            with self.subTest(scenario=scenario):
                backend = _Backend(scenario)
                with self.assertRaises(TaskReplayValidationError):
                    _orchestrator(backend).start("sample")

        backend = _Backend(_scenario({"type": "navigate", "request": {}}))
        replay = _orchestrator(backend)
        with self.assertRaises(TaskReplayValidationError):
            replay.start("../escape")


if __name__ == "__main__":
    unittest.main()
