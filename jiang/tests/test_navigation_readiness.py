"""Regression tests for side-effect-free Nav2 lifecycle readiness."""

from __future__ import annotations

import threading
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.ros_node import ControlRequestError
from control_gateway.ros_node import RosControlNode


class _DeferredFuture:
    def __init__(self) -> None:
        self._callbacks: list[Callable[[Any], None]] = []
        self._result: Any = None

    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        self._callbacks.append(callback)

    def result(self) -> Any:
        return self._result

    def complete(self, result: Any) -> None:
        self._result = result
        for callback in list(self._callbacks):
            callback(self)


class _ActionClient:
    def __init__(self, ready: bool = True) -> None:
        self.ready = ready

    def server_is_ready(self) -> bool:
        return self.ready


class _ReadinessClient:
    def __init__(self, ready: bool = True) -> None:
        self.ready = ready
        self.requests: list[Any] = []
        self.futures: list[_DeferredFuture] = []
        self.removed: list[_DeferredFuture] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request: Any) -> _DeferredFuture:
        future = _DeferredFuture()
        self.requests.append(request)
        self.futures.append(future)
        return future

    def remove_pending_request(self, future: _DeferredFuture) -> None:
        self.removed.append(future)


def _make_node() -> RosControlNode:
    node = object.__new__(RosControlNode)
    node._lock = threading.RLock()
    node._navigation_client = _ActionClient()
    node._navigation_readiness_service = (
        "/lifecycle_manager_navigation/is_active"
    )
    node._navigation_readiness_client = _ReadinessClient()
    node._navigation_readiness_active = False
    node._navigation_readiness_state = "checking"
    node._navigation_readiness_message = "waiting"
    node._navigation_readiness_future = None
    node._navigation_readiness_future_started = None
    node._navigation_readiness_generation = 0
    return node


class NavigationReadinessTests(unittest.TestCase):
    def test_lifecycle_service_follows_navigation_action_namespace(self) -> None:
        self.assertEqual(
            "/lifecycle_manager_navigation/is_active",
            RosControlNode._navigation_readiness_service_for_action(
                "/navigate_to_pose"
            ),
        )
        self.assertEqual(
            "/robot/navigation/lifecycle_manager_navigation/is_active",
            RosControlNode._navigation_readiness_service_for_action(
                "/robot/navigation/navigate_to_pose"
            ),
        )

    def test_action_discovery_alone_does_not_report_navigation_ready(
        self,
    ) -> None:
        node = _make_node()

        before = node._navigation_availability_snapshot()
        self.assertTrue(before["action_server_available"])
        self.assertFalse(before["lifecycle_active"])
        self.assertFalse(before["available"])

        node._poll_navigation_readiness()
        client = node._navigation_readiness_client
        self.assertEqual(1, len(client.requests))
        self.assertFalse(node._navigation_availability_snapshot()["available"])

        client.futures[0].complete(
            SimpleNamespace(success=False, message="nodes inactive")
        )
        inactive = node._navigation_availability_snapshot()
        self.assertFalse(inactive["available"])
        self.assertEqual("inactive", inactive["lifecycle_state"])

    def test_successful_lifecycle_response_enables_navigation(self) -> None:
        node = _make_node()
        node._poll_navigation_readiness()
        node._navigation_readiness_client.futures[0].complete(
            SimpleNamespace(success=True, message="all nodes active")
        )

        ready = node._navigation_availability_snapshot()
        self.assertTrue(ready["available"])
        self.assertTrue(ready["lifecycle_active"])
        self.assertEqual("active", ready["lifecycle_state"])

        node._navigation_readiness_client.ready = False
        unavailable = node._navigation_availability_snapshot()
        self.assertFalse(unavailable["available"])
        self.assertFalse(unavailable["lifecycle_active"])

    def test_inactive_lifecycle_rejects_navigation_without_sending_goal(
        self,
    ) -> None:
        node = _make_node()
        node._reject_if_cabinet_active_locked = lambda: None

        with self.assertRaisesRegex(
            ControlRequestError,
            "navigation stack is not active",
        ) as raised:
            node.send_navigation_goal(1.0, 2.0, 0.0)

        self.assertEqual(503, raised.exception.status)
        self.assertEqual(
            "checking",
            raised.exception.details["navigation_lifecycle_state"],
        )

    def test_pending_request_does_not_accumulate_and_timeout_is_safe(
        self,
    ) -> None:
        node = _make_node()
        node._poll_navigation_readiness()
        client = node._navigation_readiness_client
        first = client.futures[0]

        node._poll_navigation_readiness()
        self.assertEqual(1, len(client.requests))

        node._navigation_readiness_future_started = (
            time.monotonic()
            - node.NAVIGATION_READINESS_REQUEST_TIMEOUT_SECONDS
            - 0.1
        )
        node._poll_navigation_readiness()
        self.assertEqual([first], client.removed)
        self.assertEqual(2, len(client.requests))
        second = client.futures[1]

        first.complete(SimpleNamespace(success=True, message="late active"))
        after_late_response = node._navigation_availability_snapshot()
        self.assertFalse(after_late_response["available"])
        self.assertEqual("checking", after_late_response["lifecycle_state"])

        second.complete(SimpleNamespace(success=True, message="current active"))
        self.assertTrue(node._navigation_availability_snapshot()["available"])


if __name__ == "__main__":
    unittest.main()
