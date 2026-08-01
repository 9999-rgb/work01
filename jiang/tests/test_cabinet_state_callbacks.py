"""Focused regression tests for cabinet action callback isolation."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from action_msgs.msg import GoalStatus  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.ros_node import RosControlNode  # noqa: E402
from xczs_inspection_robot_control.action import (  # noqa: E402
    PressCabinetButton,
)


class _CompletedFuture:
    def __init__(self, result: object) -> None:
        self._result = result

    def result(self) -> object:
        return self._result


def _make_node() -> RosControlNode:
    node = object.__new__(RosControlNode)
    node._lock = threading.RLock()
    node._cabinet_generation = 2
    node._cabinet_goal_handle = object()
    node._cabinet_cancel_requested = False
    node._cabinet_terminal_event = threading.Event()
    node._cabinet_state = {
        "state": "operating",
        "message": "current operation",
        "phase": 1,
        "progress": 0.2,
        "max_travel": 0.001,
        "success": None,
        "error_code": None,
        "updated_at": 0.0,
    }
    return node


class CabinetStateCallbackTest(unittest.TestCase):
    """Ensure delayed callbacks cannot mutate a newer cabinet goal."""

    def test_stale_feedback_is_ignored(self) -> None:
        node = _make_node()
        feedback = SimpleNamespace(
            feedback=SimpleNamespace(
                phase=4,
                progress=0.75,
                button_travel=0.0065,
                message="stale feedback",
            )
        )

        node._cabinet_feedback_callback(feedback, generation=1)

        self.assertEqual("current operation", node._cabinet_state["message"])
        self.assertEqual(0.001, node._cabinet_state["max_travel"])

    def test_stale_cancel_response_is_ignored(self) -> None:
        node = _make_node()
        current_handle = node._cabinet_goal_handle
        response = _CompletedFuture(
            SimpleNamespace(goals_canceling=[])
        )

        node._cabinet_cancel_callback(
            response,
            generation=1,
            goal_handle=object(),
        )

        self.assertIs(current_handle, node._cabinet_goal_handle)
        self.assertEqual("operating", node._cabinet_state["state"])
        self.assertEqual("current operation", node._cabinet_state["message"])

    def test_stale_result_does_not_finish_new_goal(self) -> None:
        node = _make_node()
        old_handle = object()
        action_result = SimpleNamespace(
            success=True,
            error_code=PressCabinetButton.Result.SUCCESS,
            message="old goal succeeded",
            max_travel=0.0065,
        )
        future = _CompletedFuture(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=action_result,
            )
        )

        node._cabinet_result_callback(
            future,
            generation=1,
            goal_handle=old_handle,
        )

        self.assertEqual("operating", node._cabinet_state["state"])
        self.assertIsNone(node._cabinet_state["success"])
        self.assertFalse(node._cabinet_terminal_event.is_set())

    def test_current_result_sets_terminal_event(self) -> None:
        node = _make_node()
        action_result = SimpleNamespace(
            success=True,
            error_code=PressCabinetButton.Result.SUCCESS,
            message="current goal succeeded",
            max_travel=0.0065,
        )
        future = _CompletedFuture(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=action_result,
            )
        )

        node._cabinet_result_callback(
            future,
            generation=2,
            goal_handle=node._cabinet_goal_handle,
        )

        self.assertEqual("succeeded", node._cabinet_state["state"])
        self.assertTrue(node._cabinet_state["success"])
        self.assertEqual(0.0065, node._cabinet_state["max_travel"])
        self.assertTrue(node._cabinet_terminal_event.is_set())

    def test_old_synchronous_cancel_error_does_not_mutate_new_goal(
        self,
    ) -> None:
        node = _make_node()
        new_handle = object()

        class _OldGoalHandle:
            def cancel_goal_async(self) -> object:
                # Model the old result callback completing and a new goal
                # entering cancellation before this thread reacquires the
                # node lock to handle the synchronous client error.
                with node._lock:
                    node._cabinet_generation += 1
                    node._cabinet_goal_handle = new_handle
                    node._cabinet_cancel_requested = True
                    node._cabinet_state.update(
                        {
                            "state": "canceling",
                            "message": "new operation is canceling",
                        }
                    )
                raise RuntimeError("old action client is unavailable")

        old_handle = _OldGoalHandle()
        node._cabinet_goal_handle = old_handle

        with self.assertRaises(ControlRequestError):
            node.cancel_cabinet_button()

        self.assertIs(new_handle, node._cabinet_goal_handle)
        self.assertTrue(node._cabinet_cancel_requested)
        self.assertEqual("canceling", node._cabinet_state["state"])
        self.assertEqual(
            "new operation is canceling",
            node._cabinet_state["message"],
        )


if __name__ == "__main__":
    unittest.main()
