"""Regression tests for Web gateway shutdown request draining."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from typing import Any, Dict, List


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402


class _BlockingNode:
    def __init__(self) -> None:
        self.press_started = threading.Event()
        self.release_press = threading.Event()
        self.press_count = 0
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

    def cancel_navigation(self, allow_idle: bool = False) -> Dict[str, str]:
        del allow_idle
        return {"status": "idle"}

    def cancel_motion(self, allow_idle: bool = False) -> Dict[str, str]:
        del allow_idle
        return {"status": "idle"}

    def emergency_stop(self) -> None:
        return

    def destroy_node(self) -> None:
        self.destroyed = True


class _Executor:
    def __init__(self) -> None:
        self.shutdown_called = False

    def shutdown(self, timeout_sec: float) -> None:
        del timeout_sec
        self.shutdown_called = True


class _Thread:
    def join(self, timeout: float) -> None:
        del timeout


class _Context:
    def __init__(self) -> None:
        self.shutdown_called = False

    def shutdown(self) -> None:
        self.shutdown_called = True


def _make_server(node: _BlockingNode) -> ControlServer:
    server = object.__new__(ControlServer)
    server._node = node
    server._request_condition = threading.Condition()
    server._active_requests = 0
    server._stopping = False
    server._http_server = None
    server._http_thread = None
    server._executor = _Executor()
    server._executor_thread = _Thread()
    server._context = _Context()
    return server


class ControlServerLifecycleTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
