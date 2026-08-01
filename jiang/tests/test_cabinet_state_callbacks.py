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
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
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
    node._cabinet_catalog_received = True
    node._cabinet_controls = {
        "box_10_button_1": {
            "control_id": "box_10_button_1",
            "display_name": "10 号模块红色按钮",
            "control_type": 0,
            "joint_name": "box_10_box_10_button_1",
            "joint_state_topic": (
                "/xczs/cabinet/box_10_button_1/joint_states"
            ),
            "pressed_topic": "/xczs/cabinet/box_10_button_1/pressed",
        },
        "box_10_button_2": {
            "control_id": "box_10_button_2",
            "display_name": "10 号模块绿色按钮",
            "control_type": 0,
            "joint_name": "box_10_box_10_button_2",
            "joint_state_topic": (
                "/xczs/cabinet/box_10_button_2/joint_states"
            ),
            "pressed_topic": "/xczs/cabinet/box_10_button_2/pressed",
        },
    }
    node._cabinet_control_states = {
        control_id: node._new_cabinet_control_state()
        for control_id in node._cabinet_controls
    }
    node._cabinet_button_client = SimpleNamespace(
        server_is_ready=lambda: True
    )
    node._cabinet_state = {
        "state": "operating",
        "message": "current operation",
        "button_id": "box_10_button_2",
        "phase": 1,
        "progress": 0.2,
        "button_pressed": None,
        "button_travel": None,
        "button_state_updated_at": None,
        "max_travel": 0.001,
        "success": None,
        "error_code": None,
        "updated_at": 0.0,
    }
    return node


class CabinetStateCallbackTest(unittest.TestCase):
    """Ensure delayed callbacks cannot mutate a newer cabinet goal."""

    def test_changed_catalog_topics_replace_dynamic_subscriptions(
        self,
    ) -> None:
        node = object.__new__(RosControlNode)
        node._cabinet_control_subscriptions = {}
        created = []
        destroyed = []

        def create_subscription(
            message_type: object,
            topic: str,
            callback: object,
            qos: object,
        ) -> object:
            subscription = SimpleNamespace(
                message_type=message_type,
                topic=topic,
                callback=callback,
                qos=qos,
            )
            created.append(subscription)
            return subscription

        node.create_subscription = create_subscription
        node.destroy_subscription = destroyed.append
        control = dict(node.DEFAULT_CABINET_CONTROL)

        node._ensure_cabinet_control_subscriptions(control, object())
        node._ensure_cabinet_control_subscriptions(control, object())
        changed_control = {
            **control,
            "joint_state_topic": "/replacement/joint_states",
            "pressed_topic": "/replacement/pressed",
        }
        node._ensure_cabinet_control_subscriptions(
            changed_control,
            object(),
        )

        self.assertEqual(4, len(created))
        self.assertEqual(created[:2], destroyed)
        subscriptions = node._cabinet_control_subscriptions[
            control["control_id"]
        ]
        self.assertEqual(
            (
                control["joint_name"],
                "/replacement/joint_states",
                "/replacement/pressed",
            ),
            subscriptions["signature"],
        )

    def test_physical_state_is_kept_separate_for_each_button(self) -> None:
        node = _make_node()
        red_joint = JointState()
        red_joint.name = ["box_10_box_10_button_1"]
        red_joint.position = [0.001]
        green_joint = JointState()
        green_joint.name = ["box_10_box_10_button_2"]
        green_joint.position = [0.006]

        node._cabinet_button_joint_state_callback(
            "box_10_button_1",
            "box_10_box_10_button_1",
            "/xczs/cabinet/box_10_button_1/joint_states",
            red_joint,
        )
        node._cabinet_button_pressed_callback(
            "box_10_button_1",
            "/xczs/cabinet/box_10_button_1/pressed",
            Bool(data=False),
        )
        node._cabinet_button_joint_state_callback(
            "box_10_button_2",
            "box_10_box_10_button_2",
            "/xczs/cabinet/box_10_button_2/joint_states",
            green_joint,
        )
        node._cabinet_button_pressed_callback(
            "box_10_button_2",
            "/xczs/cabinet/box_10_button_2/pressed",
            Bool(data=True),
        )

        snapshot = node.cabinet_snapshot()
        self.assertEqual("box_10_button_2", snapshot["button_id"])
        self.assertAlmostEqual(0.006, snapshot["button_travel"])
        self.assertTrue(snapshot["button_pressed"])
        self.assertAlmostEqual(
            0.001,
            node._cabinet_control_states["box_10_button_1"][
                "button_travel"
            ],
        )
        self.assertFalse(
            node._cabinet_control_states["box_10_button_1"][
                "button_pressed"
            ]
        )

    def test_controls_snapshot_contains_both_targets(self) -> None:
        node = _make_node()
        node._cabinet_control_states["box_10_button_1"].update(
            {"button_pressed": False, "button_travel": 0.001}
        )
        node._cabinet_control_states["box_10_button_2"].update(
            {"button_pressed": True, "button_travel": 0.006}
        )

        snapshot = node.cabinet_controls_snapshot()

        self.assertTrue(snapshot["available"])
        self.assertTrue(snapshot["catalog_received"])
        self.assertEqual("box_10_button_2", snapshot["selected_control_id"])
        self.assertEqual(
            ["box_10_button_1", "box_10_button_2"],
            [control["control_id"] for control in snapshot["controls"]],
        )
        self.assertTrue(snapshot["controls"][1]["button_pressed"])

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
