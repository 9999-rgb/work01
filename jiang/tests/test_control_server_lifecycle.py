"""Regression tests for Web gateway shutdown request draining."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from typing import Any, Dict, List
from unittest.mock import MagicMock, patch


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402


class _BlockingNode:
    def __init__(self) -> None:
        self.press_started = threading.Event()
        self.release_press = threading.Event()
        self.operation_started = threading.Event()
        self.release_operation = threading.Event()
        self.press_count = 0
        self.operation_count = 0
        self.cabinet_cancel_count = 0
        self.destroyed = False

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        self.press_count += 1
        self.press_started.set()
        if not self.release_press.wait(timeout=2.0):
            raise RuntimeError("test did not release cabinet press")
        return {
            "status": "accepted",
            "button_id": button_id,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }

    def cancel_cabinet_button(
        self,
        allow_idle: bool = False,
    ) -> Dict[str, str]:
        del allow_idle
        self.cabinet_cancel_count += 1
        return {"status": "idle"}

    def operate_cabinet_control(
        self,
        control_id: str,
        command: Any,
        target_state: Any,
        target_position: Any,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        self.operation_count += 1
        self.operation_started.set()
        if not self.release_operation.wait(timeout=2.0):
            raise RuntimeError("test did not release cabinet operation")
        return {
            "status": "accepted",
            "control_id": control_id,
            "command": command,
            "target_state": target_state,
            "target_position": target_position,
            "navigate_to_staging_pose": navigate_to_staging_pose,
        }

    def cancel_cabinet_operation(
        self,
        allow_idle: bool = False,
    ) -> Dict[str, str]:
        return self.cancel_cabinet_button(allow_idle=allow_idle)

    def cancel_navigation(self, allow_idle: bool = False) -> Dict[str, str]:
        del allow_idle
        return {"status": "idle"}

    def emergency_stop(self) -> None:
        return

    def destroy_node(self) -> None:
        self.destroyed = True


class _Executor:
    def __init__(self) -> None:
        self.shutdown_called = False
        self.wake_called = False

    def wake(self) -> None:
        self.wake_called = True

    def shutdown(self, timeout_sec: float) -> None:
        del timeout_sec
        self.shutdown_called = True


class _Thread:
    def join(self, timeout: float) -> None:
        del timeout


class _Context:
    def __init__(self) -> None:
        self.shutdown_called = False

    def ok(self) -> bool:
        return not self.shutdown_called

    def shutdown(self) -> None:
        self.shutdown_called = True


class _RetiringNode(_BlockingNode):
    def __init__(self) -> None:
        super().__init__()
        self.idle = False

    def cancel_cabinet_button(
        self,
        allow_idle: bool = False,
    ) -> Dict[str, str]:
        del allow_idle
        self.cabinet_cancel_count += 1
        return {"status": "idle" if self.idle else "canceling"}

    def wait_for_cabinet_idle(self, timeout_sec: float) -> bool:
        del timeout_sec
        return self.idle


def _make_server(node: _BlockingNode) -> ControlServer:
    server = object.__new__(ControlServer)
    server._node = node
    server._request_condition = threading.Condition()
    server._active_requests = 0
    server._stopping = False
    server._http_server = None
    server._http_thread = None
    server._executor = _Executor()
    server._executor_stop_event = threading.Event()
    server._executor_thread = _Thread()
    server._executor_fatal_lock = threading.Lock()
    server._executor_fatal_error = None
    server._executor_fatal_callback = None
    server._context = _Context()
    return server


class ControlServerLifecycleTest(unittest.TestCase):
    def test_stop_joins_spin_thread_before_executor_cleanup(self) -> None:
        node = _BlockingNode()
        server = _make_server(node)

        class _BlockingExecutor(_Executor):
            def __init__(self) -> None:
                super().__init__()
                self.spin_started = threading.Event()
                self.spin_released = threading.Event()
                self.cleanup_overlapped_spin = False

            def spin_once(self, timeout_sec: float) -> None:
                del timeout_sec
                self.spin_started.set()
                self.spin_released.wait(timeout=2.0)
                raise RuntimeError("executor wake during intentional stop")

            def wake(self) -> None:
                super().wake()
                self.spin_released.set()

            def shutdown(self, timeout_sec: float) -> None:
                if server._executor_thread.is_alive():
                    self.cleanup_overlapped_spin = True
                super().shutdown(timeout_sec)

        executor = _BlockingExecutor()
        server._executor = executor
        fatal_callback = MagicMock()
        server._executor_fatal_callback = fatal_callback
        server._executor_thread = threading.Thread(
            target=server._spin_executor,
        )
        server._executor_thread.start()
        self.assertTrue(executor.spin_started.wait(timeout=1.0))

        server.stop()

        self.assertTrue(executor.wake_called)
        self.assertFalse(server._executor_thread.is_alive())
        self.assertTrue(executor.shutdown_called)
        self.assertFalse(executor.cleanup_overlapped_spin)
        self.assertTrue(server._shutdown_report["executor_stopped"])
        fatal_callback.assert_not_called()
        self.assertTrue(server.executor_health()["healthy"])

    def test_executor_exception_records_fatal_and_calls_callback_once(self) -> None:
        node = _BlockingNode()
        server = _make_server(node)
        callback = MagicMock()
        server._executor_fatal_callback = callback

        class _FailingExecutor(_Executor):
            def spin_once(self, timeout_sec: float) -> None:
                del timeout_sec
                raise RuntimeError("DDS wait-set failed")

        server._executor = _FailingExecutor()
        server._spin_executor()
        # A second report cannot overwrite the root cause or signal twice.
        self.assertFalse(server._record_executor_fatal(ValueError("later")))

        health = server.executor_health()
        self.assertEqual(health["status"], "error")
        self.assertFalse(health["healthy"])
        self.assertEqual(health["error"]["type"], "RuntimeError")
        self.assertIn("DDS wait-set failed", health["error"]["message"])
        callback.assert_called_once()
        self.assertEqual(callback.call_args.args[0], "control_ros_executor")

    def test_second_stop_finishes_teardown_after_backend_retires(self) -> None:
        node = _RetiringNode()
        server = _make_server(node)

        with patch(
            "control_gateway.runner.CABINET_SHUTDOWN_TIMEOUT_SEC",
            0.01,
        ):
            server.stop()
            self.assertFalse(node.destroyed)
            self.assertFalse(server._shutdown_report["ros_teardown_completed"])

            node.idle = True
            server.stop()

        self.assertTrue(node.destroyed)
        self.assertTrue(server._context.shutdown_called)
        self.assertTrue(server._shutdown_report["ros_teardown_completed"])

    def test_stop_accepts_an_executor_thread_that_was_never_started(self) -> None:
        node = _BlockingNode()
        server = _make_server(node)
        server._executor_thread = threading.Thread(target=lambda: None)

        server.stop()

        self.assertTrue(node.destroyed)
        self.assertTrue(server._shutdown_report["executor_stopped"])

    def test_stop_drains_entered_press_and_rejects_later_press(self) -> None:
        node = _BlockingNode()
        server = _make_server(node)
        press_results: List[Dict[str, Any]] = []
        thread_errors: List[BaseException] = []

        def press() -> None:
            try:
                press_results.append(
                    server.press_cabinet_button(
                        "box_10_button_1",
                        True,
                    )
                )
            except BaseException as error:  # noqa: BLE001
                thread_errors.append(error)

        def stop() -> None:
            try:
                server.stop()
            except BaseException as error:  # noqa: BLE001
                thread_errors.append(error)

        press_thread = threading.Thread(target=press)
        press_thread.start()
        self.assertTrue(node.press_started.wait(timeout=1.0))

        stop_thread = threading.Thread(target=stop)
        stop_thread.start()
        with server._request_condition:
            self.assertTrue(
                server._request_condition.wait_for(
                    lambda: server._stopping,
                    timeout=1.0,
                )
            )

        self.assertEqual(0, node.cabinet_cancel_count)
        self.assertTrue(stop_thread.is_alive())
        with self.assertRaises(ControlRequestError) as raised:
            server.press_cabinet_button("box_10_button_1", True)
        self.assertEqual(503, raised.exception.status)
        self.assertEqual(1, node.press_count)

        node.release_press.set()
        press_thread.join(timeout=2.0)
        stop_thread.join(timeout=2.0)

        self.assertFalse(press_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertFalse(thread_errors)
        self.assertEqual("accepted", press_results[0]["status"])
        self.assertEqual(1, node.cabinet_cancel_count)
        self.assertTrue(server._executor.shutdown_called)
        self.assertTrue(node.destroyed)
        self.assertTrue(server._context.shutdown_called)

    def test_stop_drains_generic_operation_and_rejects_later_work(self) -> None:
        node = _BlockingNode()
        server = _make_server(node)
        operation_results: List[Dict[str, Any]] = []
        thread_errors: List[BaseException] = []

        def operate() -> None:
            try:
                operation_results.append(
                    server.operate_cabinet_control(
                        "box_03_knob_1",
                        "set_position",
                        None,
                        1.0,
                        False,
                    )
                )
            except BaseException as error:  # noqa: BLE001
                thread_errors.append(error)

        def stop() -> None:
            try:
                server.stop()
            except BaseException as error:  # noqa: BLE001
                thread_errors.append(error)

        operation_thread = threading.Thread(target=operate)
        operation_thread.start()
        self.assertTrue(node.operation_started.wait(timeout=1.0))
        stop_thread = threading.Thread(target=stop)
        stop_thread.start()
        with server._request_condition:
            self.assertTrue(
                server._request_condition.wait_for(
                    lambda: server._stopping,
                    timeout=1.0,
                )
            )

        with self.assertRaises(ControlRequestError) as raised:
            server.operate_cabinet_control(
                "cabinet_door",
                "toggle",
                None,
                None,
                True,
            )
        self.assertEqual(503, raised.exception.status)
        self.assertTrue(stop_thread.is_alive())
        self.assertEqual(1, node.operation_count)

        node.release_operation.set()
        operation_thread.join(timeout=2.0)
        stop_thread.join(timeout=2.0)

        self.assertFalse(operation_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertFalse(thread_errors)
        self.assertEqual("accepted", operation_results[0]["status"])
        self.assertEqual(1, node.cabinet_cancel_count)
        self.assertTrue(node.destroyed)


if __name__ == "__main__":
    unittest.main()
