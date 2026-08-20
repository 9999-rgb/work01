"""Focused regression tests for cabinet action callback isolation."""

from __future__ import annotations

import sys
import threading
import unittest
from math import pi
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from action_msgs.msg import GoalStatus  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.ros_node import RosControlNode  # noqa: E402
from control_gateway.robot_adapter import ManualJointConfig  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from xczs_inspection_robot_control.action import (  # noqa: E402
    OperateCabinetControl,
    PressCabinetButton,
)
from xczs_inspection_robot_control.msg import CabinetControl  # noqa: E402
from xczs_inspection_robot_control.msg import (  # noqa: E402
    CabinetControlCatalog,
)
from xczs_inspection_robot_control.msg import (  # noqa: E402
    CabinetControlState,
)


class _CompletedFuture:
    def __init__(self, result: object) -> None:
        self._result = result

    def result(self) -> object:
        return self._result

    def add_done_callback(self, callback: object) -> None:
        callback(self)


class _PendingFuture:
    def __init__(self) -> None:
        self.callback = None

    def add_done_callback(self, callback: object) -> None:
        self.callback = callback


class _ActionClient:
    def __init__(self) -> None:
        self.goal = None
        self.feedback_callback = None
        self.future = _PendingFuture()

    def server_is_ready(self) -> bool:
        return True

    def send_goal_async(
        self,
        goal: object,
        feedback_callback: object,
    ) -> _PendingFuture:
        self.goal = goal
        self.feedback_callback = feedback_callback
        return self.future


def _button_control(
    control_id: str = "box_10_button_1",
    display_name: str = "10 号模块红色按钮",
) -> dict[str, object]:
    return {
        "control_id": control_id,
        "display_name": display_name,
        "control_type": CabinetControl.TYPE_BUTTON,
        "joint_name": f"box_10_{control_id}",
        "joint_state_topic": f"/xczs/cabinet/{control_id}/joint_states",
        "pressed_topic": f"/xczs/cabinet/{control_id}/pressed",
        "state_topic": f"/xczs/cabinet/{control_id}/state",
        "supported_commands": CabinetControl.SUPPORT_PRESS,
        "unit": "m",
        "min_position": 0.0,
        "max_position": 0.008,
        "state_ids": ["released", "pressed"],
        "state_labels": ["已释放", "已按下"],
        "state_positions": [0.0, 0.006],
        "requires_grasp": False,
        "operable": True,
        "unavailable_reason": "",
    }


def _make_node() -> RosControlNode:
    node = object.__new__(RosControlNode)
    node._lock = threading.RLock()
    node._cabinet_generation = 2
    node._cabinet_goal_handle = object()
    node._cabinet_cancel_requested = False
    node._cabinet_terminal_event = threading.Event()
    node._cabinet_catalog_received = True
    node._cabinet_controls = {
        "box_10_button_1": _button_control(),
        "box_10_button_2": _button_control(
            "box_10_button_2",
            "10 号模块绿色按钮",
        ),
    }
    node._cabinet_control_states = {
        control_id: node._new_cabinet_control_state()
        for control_id in node._cabinet_controls
    }
    node._cabinet_button_client = SimpleNamespace(
        server_is_ready=lambda: True
    )
    node._cabinet_operation_client = SimpleNamespace(
        server_is_ready=lambda: False
    )
    node._cabinet_reset_client = SimpleNamespace(
        service_is_ready=lambda: False
    )
    node._cabinet_state = {
        "state": "operating",
        "message": "current operation",
        "control_id": "box_10_button_2",
        "control_type": CabinetControl.TYPE_BUTTON,
        "type": CabinetControl.TYPE_BUTTON,
        "command": "press",
        "target_state": None,
        "target_position": None,
        "target": {"state": None, "position": None},
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


def _add_knob(node: RosControlNode) -> None:
    control_id = "box_03_knob_1"
    node._cabinet_controls[control_id] = {
        "control_id": control_id,
        "display_name": "3 号模块旋钮",
        "control_type": CabinetControl.TYPE_KNOB,
        "joint_name": "box_03_box_03_knob_1",
        "joint_state_topic": "/xczs/cabinet/box_03_knob_1/joint_states",
        "pressed_topic": "",
        "state_topic": "/xczs/cabinet/box_03_knob_1/state",
        "supported_commands": (
            CabinetControl.SUPPORT_SET_STATE
            | CabinetControl.SUPPORT_SET_POSITION
            | CabinetControl.SUPPORT_TOGGLE
        ),
        "unit": "rad",
        "min_position": -pi,
        "max_position": pi,
        "state_ids": ["left", "center", "right"],
        "state_labels": ["左", "中", "右"],
        "state_positions": [-pi / 2, 0.0, pi / 2],
        "requires_grasp": True,
        "operable": True,
        "unavailable_reason": "",
    }
    node._cabinet_control_states[control_id] = (
        node._new_cabinet_control_state()
    )
    node._cabinet_control_states[control_id].update(
        {
            "control_type": CabinetControl.TYPE_KNOB,
            "type": CabinetControl.TYPE_KNOB,
        }
    )


class CabinetStateCallbackTest(unittest.TestCase):
    """Ensure delayed callbacks cannot mutate a newer cabinet goal."""

    def test_catalog_is_empty_and_operations_wait_before_first_message(
        self,
    ) -> None:
        node = object.__new__(RosControlNode)
        node._lock = threading.RLock()
        node._cabinet_catalog_received = False
        node._cabinet_controls = {}
        node._cabinet_control_states = {}
        node._cabinet_state = {
            "state": "idle",
            "control_id": "",
            "button_id": "",
        }
        node._cabinet_button_client = SimpleNamespace(
            server_is_ready=lambda: True
        )
        node._cabinet_operation_client = SimpleNamespace(
            server_is_ready=lambda: True
        )
        node._cabinet_reset_client = SimpleNamespace(
            service_is_ready=lambda: True
        )

        snapshot = node.cabinet_controls_snapshot()

        self.assertFalse(snapshot["available"])
        self.assertFalse(snapshot["catalog_received"])
        self.assertEqual("waiting_for_catalog", snapshot["source"])
        self.assertEqual("", snapshot["selected_control_id"])
        self.assertEqual([], snapshot["controls"])
        with self.assertRaises(ControlRequestError) as operation_error:
            node.operate_cabinet_control("box_10_button_1", "press")
        self.assertEqual(503, operation_error.exception.status)
        self.assertIn("catalog", str(operation_error.exception).lower())
        with self.assertRaises(ControlRequestError) as button_error:
            node.press_cabinet_button("box_10_button_1", True)
        self.assertEqual(503, button_error.exception.status)

    def test_manual_joint_target_supports_six_axis_arm(self) -> None:
        node = object.__new__(RosControlNode)
        node._lock = threading.RLock()
        node._cabinet_state = {"state": "idle"}
        node._pending_trajectory = None
        node._pending_trajectory_repeats = 0
        node._manual_joints = (
            ManualJointConfig("body_arm1", "arm", -2.8, 2.8, 0.0, None),
            ManualJointConfig("arm1_arm2", "arm", -2.8, 2.8, 0.0, None),
            ManualJointConfig("arm2_arm3", "arm", -2.8, 2.8, 0.0, None),
            ManualJointConfig("arm3_arm4", "arm", -2.8, 2.8, 0.0, None),
            ManualJointConfig("arm4_arm5", "arm", -2.8, 2.8, 0.0, None),
            ManualJointConfig("arm5_end", "arm", -2.8, 2.8, 0.0, None),
            ManualJointConfig(
                "end_worklink1", "gripper", 0.0, 0.35, 0.0, 0.35
            ),
            ManualJointConfig(
                "end_worklink2", "gripper", -0.35, 0.0, 0.0, -0.35
            ),
        )

        result = node.set_joint_target(
            [3.0, -3.0, 0.1, 0.2, 0.3, 0.4, 0.5, -0.5]
        )

        self.assertEqual(2.8, result[0])
        self.assertEqual(-2.8, result[1])
        self.assertEqual(0.35, result[6])
        self.assertEqual(-0.35, result[7])
        self.assertEqual(
            [joint.name for joint in node._manual_joints],
            node._pending_trajectory.joint_names,
        )
        default_point = node._pending_trajectory.points[0]
        self.assertEqual(0, default_point.time_from_start.sec)
        self.assertEqual(500_000_000, default_point.time_from_start.nanosec)

        zero = node.set_joint_target([0.0] * 8, duration_sec=4.25)
        self.assertEqual([0.0] * 8, zero)
        reset_point = node._pending_trajectory.points[0]
        self.assertEqual(4, reset_point.time_from_start.sec)
        self.assertEqual(250_000_000, reset_point.time_from_start.nanosec)
        self.assertEqual(4.25, node._pending_trajectory_duration_sec)
        self.assertNotIn(
            "body_arm_lift",
            node._pending_trajectory.joint_names,
        )

        with self.assertRaisesRegex(ControlRequestError, "manual joint order"):
            node.set_joint_target([0.0] * 7)
        with self.assertRaisesRegex(ControlRequestError, "manual joint order"):
            node.set_joint_target([0.0] * 9)
        for invalid_duration in (
            True,
            "1.0",
            0.0,
            -1.0,
            float("nan"),
            float("inf"),
        ):
            with self.subTest(duration_sec=invalid_duration):
                with self.assertRaisesRegex(
                    ControlRequestError,
                    "duration_sec",
                ):
                    node.set_joint_target(
                        [0.0] * 8,
                        duration_sec=invalid_duration,
                    )
        with self.assertRaisesRegex(ControlRequestError, "ROS duration range"):
            node.set_joint_target(
                [0.0] * 8,
                duration_sec=2_147_483_648.0,
            )

    def test_robot_joint_state_snapshot_requires_all_finite_manual_joints(
        self,
    ) -> None:
        node = object.__new__(RosControlNode)
        node._lock = threading.RLock()
        node._manual_joints = (
            ManualJointConfig("joint_a", "arm", -2.0, 2.0, 0.0, None),
            ManualJointConfig("joint_b", "arm", -2.0, 2.0, 0.0, None),
        )
        node._robot_joint_state = {
            "available": False,
            "positions": {},
            "stamp_ros_nanoseconds": None,
            "received_monotonic": None,
        }

        self.assertEqual(
            {
                "available": False,
                "positions": {},
                "stamp_ros_nanoseconds": None,
                "received_monotonic": None,
            },
            node.robot_joint_state_snapshot(),
        )

        message = JointState()
        message.header.stamp.sec = 12
        message.header.stamp.nanosec = 345
        message.name = ["unrelated", "joint_b", "joint_a"]
        message.position = [99.0, -0.25, 0.75]
        with patch("control_gateway.ros_node.time.monotonic", return_value=8.5):
            node._robot_joint_state_callback(message)

        snapshot = node.robot_joint_state_snapshot()
        self.assertEqual(
            {
                "available": True,
                "positions": {"joint_a": 0.75, "joint_b": -0.25},
                "stamp_ros_nanoseconds": 12_000_000_345,
                "received_monotonic": 8.5,
            },
            snapshot,
        )
        snapshot["positions"]["joint_a"] = 100.0
        self.assertEqual(
            0.75,
            node.robot_joint_state_snapshot()["positions"]["joint_a"],
        )

        incomplete = JointState()
        incomplete.header.stamp.sec = 13
        incomplete.name = ["joint_a", "joint_b"]
        incomplete.position = [0.1, float("nan")]
        node._robot_joint_state_callback(incomplete)
        snapshot = node.robot_joint_state_snapshot()
        self.assertFalse(snapshot["available"])
        self.assertEqual({"joint_a": 0.1}, snapshot["positions"])
        self.assertEqual(13_000_000_000, snapshot["stamp_ros_nanoseconds"])

        duplicated = JointState()
        duplicated.name = ["joint_a", "joint_a", "joint_b"]
        duplicated.position = [0.1, 0.2, 0.3]
        node._robot_joint_state_callback(duplicated)
        snapshot = node.robot_joint_state_snapshot()
        self.assertFalse(snapshot["available"])
        self.assertEqual({"joint_b": 0.3}, snapshot["positions"])

    def test_replay_quiescence_clears_pending_manual_outputs(self) -> None:
        resets = []
        published = []
        node = object.__new__(RosControlNode)
        node._lock = threading.RLock()
        node._target_linear_y = 0.2
        node._target_angular_z = -0.3
        node._linear_profile = SimpleNamespace(reset=lambda: resets.append("linear"))
        node._angular_profile = SimpleNamespace(
            reset=lambda: resets.append("angular")
        )
        node._pending_trajectory = object()
        node._pending_trajectory_repeats = 4
        node._manual_trajectory_active_until = 10.4
        node._cmd_vel_publisher = SimpleNamespace(publish=published.append)

        with patch("control_gateway.ros_node.time.monotonic", return_value=10.0):
            settle = node.quiesce_manual_outputs()

        self.assertAlmostEqual(0.4, settle)
        self.assertEqual(0.0, node._target_linear_y)
        self.assertEqual(0.0, node._target_angular_z)
        self.assertIsNone(node._pending_trajectory)
        self.assertEqual(0, node._pending_trajectory_repeats)
        self.assertEqual(["linear", "angular"], resets)
        self.assertEqual(1, len(published))

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
        control = _button_control()

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

        # Aggregate CabinetControlState is authoritative; changing only its
        # legacy mirror topics may replace the generation, but must never add
        # duplicate high-frequency subscriptions.
        self.assertEqual(2, len(created))
        self.assertEqual(created[:1], destroyed)
        self.assertEqual(["state", "state"], [
            item.topic.rsplit("/", 1)[-1] for item in created
        ])
        subscriptions = node._cabinet_control_subscriptions[
            control["control_id"]
        ]
        self.assertEqual(
            (
                control["joint_name"],
                "/replacement/joint_states",
                "/replacement/pressed",
                control["state_topic"],
                control["control_type"],
            ),
            subscriptions["signature"],
        )

    def test_catalog_accepts_button_knob_switch_and_door(self) -> None:
        node = object.__new__(RosControlNode)
        node._lock = threading.RLock()
        node._cabinet_controls = {}
        node._cabinet_control_states = {}
        node._cabinet_control_subscriptions = {}
        node._cabinet_catalog_received = False
        created = []
        node.create_subscription = (
            lambda message_type, topic, callback, qos: created.append(
                SimpleNamespace(
                    message_type=message_type,
                    topic=topic,
                    callback=callback,
                    qos=qos,
                )
            ) or created[-1]
        )
        node.destroy_subscription = lambda subscription: None

        entries = []
        specifications = [
            ("button", CabinetControl.TYPE_BUTTON, CabinetControl.SUPPORT_PRESS),
            (
                "knob",
                CabinetControl.TYPE_KNOB,
                CabinetControl.SUPPORT_SET_POSITION,
            ),
            (
                "switch",
                CabinetControl.TYPE_SWITCH,
                CabinetControl.SUPPORT_SET_STATE,
            ),
            ("door", CabinetControl.TYPE_DOOR, CabinetControl.SUPPORT_TOGGLE),
        ]
        for control_id, control_type, supported_commands in specifications:
            entry = CabinetControl()
            entry.control_id = control_id
            entry.display_name = control_id
            entry.control_type = control_type
            entry.state_topic = f"/cabinet/{control_id}/state"
            entry.supported_commands = supported_commands
            entry.unit = "rad"
            entry.min_position = -1.0
            entry.max_position = 1.0
            entry.state_ids = ["off", "on"]
            entry.state_labels = ["关", "开"]
            entry.state_positions = [-1.0, 1.0]
            entry.operable = True
            entries.append(entry)
        catalog = CabinetControlCatalog()
        catalog.controls = entries

        node._cabinet_control_catalog_callback(catalog)

        self.assertEqual(
            ["button", "knob", "switch", "door"],
            list(node._cabinet_controls),
        )
        self.assertEqual(4, len(created))
        self.assertTrue(node._cabinet_catalog_received)
        self.assertEqual(
            CabinetControl.TYPE_DOOR,
            node._cabinet_control_states["door"]["type"],
        )

    def test_generic_control_state_updates_selected_knob(self) -> None:
        node = _make_node()
        _add_knob(node)
        node._cabinet_state.update(
            {
                "control_id": "box_03_knob_1",
                "button_id": "box_03_knob_1",
                "control_type": CabinetControl.TYPE_KNOB,
                "type": CabinetControl.TYPE_KNOB,
            }
        )
        state = CabinetControlState()
        state.control_id = "box_03_knob_1"
        state.control_type = CabinetControl.TYPE_KNOB
        state.valid = True
        state.position = -pi / 2
        state.velocity = 0.1
        state.effort = 0.2
        state.normalized_position = 0.25
        state.state_id = "left"
        state.activated = True
        state.in_motion = False
        state.transition_sequence = 7

        node._cabinet_control_state_callback(
            "box_03_knob_1",
            "/xczs/cabinet/box_03_knob_1/state",
            state,
        )

        snapshot = node.cabinet_snapshot()
        self.assertEqual("box_03_knob_1", snapshot["control_id"])
        self.assertEqual(CabinetControl.TYPE_KNOB, snapshot["type"])
        self.assertAlmostEqual(-pi / 2, snapshot["current_position"])
        self.assertEqual("left", snapshot["current_state"])
        self.assertEqual(7, snapshot["transition_sequence"])

    def test_generic_operation_builds_goal_and_exposes_generic_state(self) -> None:
        node = _make_node()
        _add_knob(node)
        action_client = _ActionClient()
        node._cabinet_operation_client = action_client
        node._cabinet_goal_handle = None
        node._cabinet_state["state"] = "idle"
        node._navigation_state = {"state": "idle"}
        node._navigation_mode_request = None
        node._navigation_mode_desired = None
        node._navigation_retirements = {}
        node._linear_profile = SimpleNamespace(reset=lambda: None)
        node._angular_profile = SimpleNamespace(reset=lambda: None)
        node._pending_trajectory = None
        node._pending_trajectory_repeats = 0
        published = []
        node._cmd_vel_publisher = SimpleNamespace(publish=published.append)

        response = node.operate_cabinet_control(
            "box_03_knob_1",
            "set_position",
            target_position=pi / 2,
            navigate_to_staging_pose=False,
        )

        self.assertEqual("accepted", response["status"])
        self.assertEqual(
            OperateCabinetControl.Goal.COMMAND_SET_POSITION,
            action_client.goal.command,
        )
        self.assertTrue(action_client.goal.use_target_position)
        self.assertAlmostEqual(pi / 2, action_client.goal.target_position)
        self.assertFalse(action_client.goal.navigate_to_staging_pose)
        self.assertEqual("generic", node._cabinet_state["action_interface"])
        self.assertEqual(
            CabinetControl.TYPE_KNOB,
            node._cabinet_state["type"],
        )
        self.assertEqual(
            {"state": None, "position": pi / 2},
            node._cabinet_state["target"],
        )
        self.assertEqual(1, len(published))

    def test_generic_operation_sends_unverified_control_for_validation(self) -> None:
        node = _make_node()
        node._cabinet_controls["box_10_button_1"].update(
            {"operable": False, "unavailable_reason": "workspace limit"}
        )
        action_client = _ActionClient()
        node._cabinet_operation_client = action_client
        node._cabinet_goal_handle = None
        node._cabinet_state["state"] = "idle"
        node._navigation_state = {"state": "idle"}
        node._navigation_mode_request = None
        node._navigation_mode_desired = None
        node._navigation_retirements = {}
        node._linear_profile = SimpleNamespace(reset=lambda: None)
        node._angular_profile = SimpleNamespace(reset=lambda: None)
        node._pending_trajectory = None
        node._pending_trajectory_repeats = 0
        node._cmd_vel_publisher = SimpleNamespace(publish=lambda _msg: None)

        response = node.operate_cabinet_control(
            "box_10_button_1",
            "press",
        )

        self.assertEqual("accepted", response["status"])
        self.assertEqual("box_10_button_1", action_client.goal.control_id)

    def test_legacy_action_requires_generic_backend_for_unverified_button(
        self,
    ) -> None:
        node = _make_node()
        node._cabinet_controls["box_10_button_1"].update(
            {"operable": False, "unavailable_reason": "workspace limit"}
        )
        node._cabinet_goal_handle = None
        node._cabinet_state["state"] = "idle"
        node._navigation_state = {"state": "idle"}
        node._navigation_mode_request = None
        node._navigation_mode_desired = None
        node._navigation_retirements = {}

        with self.assertRaises(ControlRequestError) as rejected:
            node.press_cabinet_button("box_10_button_1", False)

        self.assertEqual(503, rejected.exception.status)
        self.assertIn("generic", str(rejected.exception).lower())
        self.assertIsNone(node._cabinet_goal_handle)

    def test_legacy_action_rejects_missing_operable_capability(self) -> None:
        node = _make_node()
        node._cabinet_controls["box_10_button_1"].pop("operable", None)
        node._cabinet_goal_handle = None
        node._cabinet_state["state"] = "idle"
        node._navigation_state = {"state": "idle"}
        node._navigation_mode_request = None
        node._navigation_mode_desired = None
        node._navigation_retirements = {}

        with self.assertRaises(ControlRequestError) as rejected:
            node.press_cabinet_button("box_10_button_1", False)

        self.assertEqual(503, rejected.exception.status)
        self.assertIsNone(node._cabinet_goal_handle)

    def test_generic_operation_validates_command_targets(self) -> None:
        node = _make_node()
        _add_knob(node)

        with self.assertRaisesRegex(ControlRequestError, "outside"):
            node.operate_cabinet_control(
                "box_03_knob_1",
                "set_position",
                target_position=4.0,
            )
        with self.assertRaisesRegex(ControlRequestError, "detent"):
            node.operate_cabinet_control(
                "box_03_knob_1",
                "set_position",
                target_position=1.0,
            )
        with self.assertRaisesRegex(ControlRequestError, "target_state"):
            node.operate_cabinet_control(
                "box_03_knob_1",
                "set_state",
                target_state="unknown",
            )
        with self.assertRaisesRegex(ControlRequestError, "does not support"):
            node.operate_cabinet_control(
                "box_10_button_1",
                "toggle",
            )

    def test_generic_feedback_and_result_report_physical_outcome(self) -> None:
        node = _make_node()
        _add_knob(node)
        node._cabinet_state.update(
            {
                "control_id": "box_03_knob_1",
                "control_type": CabinetControl.TYPE_KNOB,
                "type": CabinetControl.TYPE_KNOB,
                "peak_position": 0.0,
            }
        )
        feedback = SimpleNamespace(
            feedback=SimpleNamespace(
                phase=OperateCabinetControl.Feedback.MANIPULATING,
                progress=0.6,
                current_position=0.8,
                target_position=pi / 2,
                current_state="between",
                message="rotating",
            )
        )
        node._cabinet_operation_feedback_callback(feedback, generation=2)
        self.assertEqual("rotating", node._cabinet_state["message"])
        self.assertAlmostEqual(0.8, node._cabinet_state["current_position"])
        self.assertEqual("between", node._cabinet_state["current_state"])

        goal_handle = node._cabinet_goal_handle
        result = SimpleNamespace(
            success=True,
            error_code=OperateCabinetControl.Result.SUCCESS,
            message="knob reached right detent",
            initial_position=0.0,
            final_position=pi / 2,
            peak_position=pi / 2,
            final_state="right",
            requested_force=0.0,
            estimated_force=0.0,
            button_triggered=False,
        )
        future = _CompletedFuture(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=result,
            )
        )
        node._cabinet_operation_result_callback(
            future,
            generation=2,
            goal_handle=goal_handle,
        )

        self.assertEqual("succeeded", node._cabinet_state["state"])
        self.assertAlmostEqual(pi / 2, node._cabinet_state["final_position"])
        self.assertEqual("right", node._cabinet_state["current_state"])
        self.assertTrue(node._cabinet_terminal_event.is_set())

    def test_reset_service_clears_operation_target(self) -> None:
        node = _make_node()
        node._cabinet_state["state"] = "idle"
        node._navigation_state = {"state": "idle"}
        node._navigation_mode_request = None
        node._navigation_mode_desired = None
        node._navigation_retirements = {}
        node._cabinet_reset_client = SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request: _CompletedFuture(
                SimpleNamespace(success=True, message="reset complete")
            ),
        )

        response = node.reset_cabinet_controls(timeout_sec=0.1)

        self.assertEqual("reset", response["status"])
        self.assertEqual("reset complete", response["message"])
        self.assertIsNone(node._cabinet_state["command"])
        self.assertEqual(
            {"state": None, "position": None},
            node._cabinet_state["target"],
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
