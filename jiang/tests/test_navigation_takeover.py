"""Regression tests for atomic Web-to-manual navigation takeover."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, List, Optional


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from action_msgs.msg import GoalStatus  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.ros_node import RosControlNode  # noqa: E402


class _DeferredFuture:
    def __init__(self) -> None:
        self._callbacks: List[Callable[[Any], None]] = []
        self._result: Any = None
        self._error: Optional[Exception] = None
        self._done = False

    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        if self._done:
            callback(self)
            return
        self._callbacks.append(callback)

    def result(self) -> Any:
        if self._error is not None:
            raise self._error
        return self._result

    def complete(self, result: Any) -> None:
        self._result = result
        self._done = True
        for callback in list(self._callbacks):
            callback(self)

    def fail(self, error: Exception) -> None:
        self._error = error
        self._done = True
        for callback in list(self._callbacks):
            callback(self)


class _ModeClient:
    def __init__(self) -> None:
        self.ready = True
        self.requests: List[bool] = []
        self.futures: List[_DeferredFuture] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request: Any) -> _DeferredFuture:
        future = _DeferredFuture()
        self.requests.append(bool(request.data))
        self.futures.append(future)
        return future


class _Publisher:
    def __init__(self) -> None:
        self.messages: List[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class _Profile:
    def __init__(self) -> None:
        self.reset_count = 0

    def reset(self) -> None:
        self.reset_count += 1


class _NavigationClient:
    def __init__(self) -> None:
        self.send_count = 0

    def send_goal_async(self, *_args: Any, **_kwargs: Any) -> Any:
        self.send_count += 1
        return _DeferredFuture()

    def server_is_ready(self) -> bool:
        return True


class _GoalHandle:
    accepted = True

    def __init__(self) -> None:
        self.cancel_count = 0
        self.cancel_future = _DeferredFuture()
        self.result_future = _DeferredFuture()

    def cancel_goal_async(self) -> _DeferredFuture:
        self.cancel_count += 1
        return self.cancel_future

    def get_result_async(self) -> _DeferredFuture:
        return self.result_future


class _BrokenResultGoalHandle(_GoalHandle):
    def get_result_async(self) -> _DeferredFuture:
        raise RuntimeError("result service unavailable")


def _make_node(mode: Optional[bool] = False) -> RosControlNode:
    node = object.__new__(RosControlNode)
    node._lock = threading.RLock()
    node._navigation_mode = mode
    node._navigation_mode_acknowledged = mode
    node._navigation_generation = 1
    node._navigation_goal_handle = None
    node._navigation_goal_token = None
    node._navigation_cancel_requested = False
    node._manual_takeover_generation = None
    node._navigation_mode_desired = None
    node._navigation_mode_request = None
    node._navigation_mode_request_sequence = 0
    node._navigation_mode_attempt = 0
    node._navigation_retirements = {}
    node._navigation_mode_client = _ModeClient()
    node._navigation_client = _NavigationClient()
    node._navigation_state = {
        "state": "idle",
        "message": "idle",
        "goal": None,
        "distance_remaining": None,
        "eta_seconds": None,
        "navigation_time_seconds": None,
        "recoveries": 0,
        "updated_at": 0.0,
    }
    node._cabinet_state = {"state": "idle"}
    node._map_state = None
    node._target_linear_y = 0.2
    node._target_angular_z = 0.4
    node._last_command_time = 0.0
    node._linear_profile = _Profile()
    node._angular_profile = _Profile()
    node._cmd_vel_publisher = _Publisher()
    node._max_linear_speed = 0.25
    node._max_angular_speed = 0.60
    return node


class NavigationTakeoverTest(unittest.TestCase):
    def test_old_enable_finishes_before_queued_manual_switch(self) -> None:
        node = _make_node(mode=False)
        node._navigation_state.update(
            {
                "state": "enabling",
                "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            }
        )
        with node._lock:
            node._request_navigation_mode_locked(True, generation=1)

        result = node.takeover_navigation()

        self.assertEqual("taking_over", result["status"])
        self.assertEqual([True], node._navigation_mode_client.requests)
        node._navigation_mode_client.futures[0].complete(
            SimpleNamespace(success=True, message="enabled")
        )
        self.assertEqual(
            [True, False],
            node._navigation_mode_client.requests,
        )
        self.assertEqual(0, node._navigation_client.send_count)

        node._navigation_mode_client.futures[1].complete(
            SimpleNamespace(success=True, message="manual")
        )
        self.assertFalse(node._navigation_mode_acknowledged)
        node._navigation_mode_callback(SimpleNamespace(data=False))
        self.assertFalse(node._navigation_mode)
        self.assertEqual("canceled", node._navigation_state["state"])
        self.assertIsNone(node._manual_takeover_generation)
        self.assertEqual(0.0, node._target_linear_y)
        self.assertEqual(0.0, node._target_angular_z)

    def test_stale_goal_acceptance_is_canceled_without_state_mutation(
        self,
    ) -> None:
        node = _make_node(mode=True)
        node._navigation_generation = 2
        node._manual_takeover_generation = 2
        node._navigation_state["state"] = "taking_over"
        stale_token = object()
        goal_handle = _GoalHandle()
        response = _DeferredFuture()
        response.complete(goal_handle)

        node._navigation_goal_response_callback(
            response,
            generation=1,
            goal_token=stale_token,
        )

        self.assertEqual(1, goal_handle.cancel_count)
        self.assertEqual("taking_over", node._navigation_state["state"])
        self.assertIsNone(node._navigation_goal_handle)
        goal_handle.cancel_future.complete(
            SimpleNamespace(goals_canceling=[object()])
        )
        self.assertEqual("taking_over", node._navigation_state["state"])
        retirement = node._navigation_retirements[(1, stale_token)]
        self.assertEqual("cancel_accepted", retirement["state"])

        goal_handle.result_future.complete(
            SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
        )
        self.assertEqual({}, node._navigation_retirements)

    def test_stale_feedback_and_result_cannot_finish_current_goal(
        self,
    ) -> None:
        node = _make_node(mode=True)
        current_token = object()
        current_handle = _GoalHandle()
        node._navigation_generation = 2
        node._navigation_goal_token = current_token
        node._navigation_goal_handle = current_handle
        node._navigation_state["state"] = "navigating"
        feedback = SimpleNamespace(
            feedback=SimpleNamespace(
                distance_remaining=9.0,
                estimated_time_remaining=SimpleNamespace(sec=4, nanosec=0),
                navigation_time=SimpleNamespace(sec=2, nanosec=0),
                number_of_recoveries=3,
            )
        )

        node._navigation_feedback_callback(
            feedback,
            generation=1,
            goal_token=object(),
        )
        stale_result = _DeferredFuture()
        stale_result.complete(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
        )
        node._navigation_result_callback(
            stale_result,
            generation=1,
            goal_handle=object(),
            goal_token=object(),
        )

        self.assertEqual("navigating", node._navigation_state["state"])
        self.assertIsNone(node._navigation_state["distance_remaining"])
        self.assertIs(current_handle, node._navigation_goal_handle)

    def test_takeover_already_in_manual_mode_still_holds_zero(self) -> None:
        node = _make_node(mode=False)

        result = node.takeover_navigation()

        self.assertEqual("taking_over", result["status"])
        self.assertEqual([False], node._navigation_mode_client.requests)
        self.assertEqual(0.0, node._target_linear_y)
        self.assertEqual(0.0, node._target_angular_z)
        self.assertEqual("taking_over", node._navigation_state["state"])
        self.assertEqual(1, len(node._cmd_vel_publisher.messages))

    def test_takeover_rejects_active_cabinet_operation(self) -> None:
        node = _make_node(mode=True)
        node._cabinet_state["state"] = "operating"

        with self.assertRaises(ControlRequestError) as context:
            node.takeover_navigation()

        self.assertEqual(409, context.exception.status)

    def test_observed_mode_does_not_acknowledge_takeover(self) -> None:
        node = _make_node(mode=True)

        node.takeover_navigation()
        node._navigation_mode_callback(SimpleNamespace(data=False))

        self.assertFalse(node._navigation_mode)
        self.assertTrue(node._navigation_mode_acknowledged)
        self.assertEqual("taking_over", node._navigation_state["state"])
        with self.assertRaises(ControlRequestError):
            node.set_base_target(0.1, 0.0)
        with node._lock:
            with self.assertRaises(ControlRequestError):
                node._reject_cabinet_start_conflicts_locked()
        node._navigation_mode_client.futures[0].complete(
            SimpleNamespace(success=True, message="manual")
        )
        self.assertFalse(node._navigation_mode_acknowledged)
        self.assertEqual("canceled", node._navigation_state["state"])
        self.assertEqual((0.1, 0.0), node.set_base_target(0.1, 0.0))

    def test_mode_switch_failure_retries_then_stays_failed_and_zero(
        self,
    ) -> None:
        node = _make_node(mode=True)

        node.takeover_navigation()
        for index in range(node.NAVIGATION_MODE_MAX_ATTEMPTS):
            node._navigation_mode_client.futures[index].complete(
                SimpleNamespace(success=False, message="router unavailable")
            )

        self.assertEqual(
            [False] * node.NAVIGATION_MODE_MAX_ATTEMPTS,
            node._navigation_mode_client.requests,
        )
        self.assertEqual("failed", node._navigation_state["state"])
        self.assertTrue(node._navigation_mode_acknowledged)
        self.assertEqual(0.0, node._target_linear_y)
        self.assertEqual(0.0, node._target_angular_z)
        with self.assertRaises(ControlRequestError):
            node.set_base_target(0.1, 0.0)

    def test_pending_mode_enable_blocks_manual_command(self) -> None:
        node = _make_node(mode=False)
        with node._lock:
            node._request_navigation_mode_locked(True, generation=1)

        with self.assertRaises(ControlRequestError):
            node.set_base_target(0.1, 0.0)
        with node._lock:
            with self.assertRaises(ControlRequestError):
                node._reject_cabinet_start_conflicts_locked()

    def test_unavailable_mode_service_still_invalidates_and_cancels(
        self,
    ) -> None:
        node = _make_node(mode=True)
        token = object()
        goal_handle = _GoalHandle()
        node._navigation_goal_token = token
        node._navigation_goal_handle = goal_handle
        node._navigation_state["state"] = "navigating"
        node._navigation_mode_client.ready = False

        with self.assertRaises(ControlRequestError) as context:
            node.takeover_navigation()

        self.assertEqual(503, context.exception.status)
        self.assertEqual(2, node._navigation_generation)
        self.assertEqual(1, goal_handle.cancel_count)
        self.assertIn((1, token), node._navigation_retirements)
        self.assertEqual(0.0, node._target_linear_y)
        self.assertEqual(0.0, node._target_angular_z)
        self.assertEqual(1, len(node._cmd_vel_publisher.messages))

    def test_failed_retirement_blocks_automatic_modes_and_cabinet(
        self,
    ) -> None:
        node = _make_node(mode=True)
        token = object()
        goal_handle = _GoalHandle()
        node._navigation_retirements[(1, token)] = {
            "handle": goal_handle,
            "state": "failed",
            "error": "cancel rejected",
        }

        with self.assertRaises(ControlRequestError):
            node.set_navigation_mode(True)
        with self.assertRaises(ControlRequestError):
            node.send_navigation_goal(0.0, 0.0, 0.0)
        with node._lock:
            with self.assertRaises(ControlRequestError):
                node._reject_cabinet_start_conflicts_locked()

    def test_current_cancel_failure_becomes_blocking_retirement(self) -> None:
        node = _make_node(mode=True)
        token = object()
        goal_handle = _GoalHandle()
        node._navigation_goal_token = token
        node._navigation_goal_handle = goal_handle
        node._navigation_state["state"] = "navigating"

        node.cancel_navigation()
        goal_handle.cancel_future.complete(
            SimpleNamespace(goals_canceling=[])
        )

        self.assertEqual("failed", node._navigation_state["state"])
        retirement = node._navigation_retirements[(1, token)]
        self.assertEqual("failed", retirement["state"])
        with self.assertRaises(ControlRequestError):
            node.set_navigation_mode(True)

        result = _DeferredFuture()
        result.complete(
            SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
        )
        node._navigation_result_callback(
            result,
            generation=1,
            goal_handle=goal_handle,
            goal_token=token,
        )
        self.assertEqual({}, node._navigation_retirements)
        self.assertEqual("canceled", node._navigation_state["state"])

    def test_stale_result_exception_keeps_retirement_blocking(self) -> None:
        node = _make_node(mode=False)
        node._navigation_generation = 2
        node._navigation_state["state"] = "canceled"
        stale_token = object()
        stale_handle = _GoalHandle()
        node._navigation_retirements[(1, stale_token)] = {
            "handle": stale_handle,
            "state": "cancel_accepted",
            "error": None,
        }
        failed_result = _DeferredFuture()
        failed_result.fail(RuntimeError("result response lost"))

        node._navigation_result_callback(
            failed_result,
            generation=1,
            goal_handle=stale_handle,
            goal_token=stale_token,
        )

        retirement = node._navigation_retirements[(1, stale_token)]
        self.assertEqual("failed", retirement["state"])
        self.assertIn("result response lost", retirement["error"])
        self.assertEqual("canceled", node._navigation_state["state"])

    def test_current_result_exception_moves_goal_to_retirement(self) -> None:
        node = _make_node(mode=True)
        token = object()
        goal_handle = _GoalHandle()
        node._navigation_goal_token = token
        node._navigation_goal_handle = goal_handle
        node._navigation_state["state"] = "navigating"
        failed_result = _DeferredFuture()
        failed_result.fail(RuntimeError("result response lost"))

        node._navigation_result_callback(
            failed_result,
            generation=1,
            goal_handle=goal_handle,
            goal_token=token,
        )

        self.assertIsNone(node._navigation_goal_handle)
        self.assertIsNone(node._navigation_goal_token)
        self.assertEqual("failed", node._navigation_state["state"])
        retirement = node._navigation_retirements[(1, token)]
        self.assertEqual("failed", retirement["state"])
        with self.assertRaises(ControlRequestError):
            node.set_navigation_mode(True)

        node.takeover_navigation()
        self.assertEqual(1, goal_handle.cancel_count)

    def test_synchronous_get_result_error_stays_blocking(self) -> None:
        node = _make_node(mode=True)
        token = object()
        goal_handle = _BrokenResultGoalHandle()
        node._navigation_goal_token = token
        node._navigation_state["state"] = "sending"
        response = _DeferredFuture()
        response.complete(goal_handle)

        node._navigation_goal_response_callback(
            response,
            generation=1,
            goal_token=token,
        )

        self.assertIsNone(node._navigation_goal_handle)
        self.assertEqual("failed", node._navigation_state["state"])
        retirement = node._navigation_retirements[(1, token)]
        self.assertEqual("failed", retirement["state"])
        self.assertIn("result service unavailable", retirement["error"])
        goal_handle.cancel_future.complete(
            SimpleNamespace(goals_canceling=[object()])
        )
        retirement = node._navigation_retirements[(1, token)]
        self.assertEqual("failed", retirement["state"])
        self.assertIn("result service unavailable", retirement["error"])


if __name__ == "__main__":
    unittest.main()
