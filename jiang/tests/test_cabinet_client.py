"""Tests for per-cabinet ROS client isolation using fake ROS entities."""

from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional
from unittest.mock import patch


JIANG_DIR = Path(__file__).resolve().parents[1]


class _Logger:
    def warning(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class _Subscription:
    def __init__(self, topic: str, callback: Callable[[Any], None]) -> None:
        self.topic = topic
        self.callback = callback
        self.destroyed = False


class _Node:
    def __init__(
        self,
        name: str,
        *,
        namespace: str,
        context: Any = None,
    ) -> None:
        self.name = name
        self.namespace = namespace
        self.context = context
        # Match the relevant rclpy.Node implementation detail so tests catch
        # accidental collisions with its internal subscription registry.
        self._subscriptions: List[_Subscription] = []

    @property
    def subscriptions(self) -> List[_Subscription]:
        return self._subscriptions

    def create_subscription(
        self,
        _message_type: Any,
        topic: str,
        callback: Callable[[Any], None],
        _qos: Any,
    ) -> _Subscription:
        subscription = _Subscription(topic, callback)
        self._subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription: _Subscription) -> None:
        subscription.destroyed = True

    def create_client(self, *_args: Any) -> Any:
        return _ResetClient()

    def get_logger(self) -> _Logger:
        return _Logger()


class _QoS:
    def __init__(self, **values: Any) -> None:
        self.values = values


class _DeferredFuture:
    def __init__(self) -> None:
        self._callbacks: List[Callable[[Any], None]] = []
        self._result: Any = None
        self._error: Optional[Exception] = None
        self._done = False

    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        if self._done:
            callback(self)
        else:
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


class _ActionClient:
    def __init__(self, ready: bool = True) -> None:
        self.ready = ready
        self.sent: List[Any] = []
        self.feedback_callbacks: List[Callable[[Any], None]] = []
        self.goal_futures: List[_DeferredFuture] = []

    def server_is_ready(self) -> bool:
        return self.ready

    def send_goal_async(
        self,
        goal: Any,
        *,
        feedback_callback: Callable[[Any], None],
    ) -> _DeferredFuture:
        future = _DeferredFuture()
        self.sent.append(goal)
        self.feedback_callbacks.append(feedback_callback)
        self.goal_futures.append(future)
        return future


class _GoalHandle:
    accepted = True

    def __init__(self) -> None:
        self.result_future = _DeferredFuture()
        self.cancel_future = _DeferredFuture()

    def get_result_async(self) -> _DeferredFuture:
        return self.result_future

    def cancel_goal_async(self) -> _DeferredFuture:
        return self.cancel_future


class _ResetClient:
    def service_is_ready(self) -> bool:
        return True


class _Goal:
    COMMAND_PRESS = 0
    COMMAND_SET_STATE = 1
    COMMAND_SET_POSITION = 2
    COMMAND_TOGGLE = 3

    def __init__(self) -> None:
        self.control_id = ""
        self.command = 0
        self.target_state = ""
        self.target_position = 0.0
        self.use_target_position = False
        self.force = 0.0
        self.navigate_to_staging_pose = False


class _Result:
    SUCCESS = 0
    INVALID_CONTROL = 1
    UNSUPPORTED_COMMAND = 2
    NOT_READY = 3
    NAVIGATION_FAILED = 4
    PLANNING_FAILED = 5
    EXECUTION_FAILED = 6
    GRASP_FAILED = 7
    TARGET_NOT_REACHED = 8
    RELEASE_FAILED = 9
    CANCELED = 10
    INTERNAL_ERROR = 11
    INVALID_FORCE = 12
    INSUFFICIENT_FORCE = 13
    UNREACHABLE = 14
    CONTACT_DETECTION_TIMEOUT = 15
    RESOURCE_BUSY = 16
    LEASE_LOST = 17


class _Feedback:
    WAITING_FOR_SYSTEM = 0
    NAVIGATING = 1
    DOCKING = 2
    MOVING_TO_READY = 3
    APPROACHING = 4
    GRASPING = 5
    MANIPULATING = 6
    VERIFYING = 7
    RELEASING = 8
    RETREATING = 9


class _Operate:
    Goal = _Goal
    Result = _Result
    Feedback = _Feedback


class _Control:
    TYPE_BUTTON = 0
    TYPE_KNOB = 1
    TYPE_SWITCH = 2
    TYPE_DOOR = 3
    SUPPORT_PRESS = 1
    SUPPORT_SET_STATE = 2
    SUPPORT_SET_POSITION = 4
    SUPPORT_TOGGLE = 8


def _module(name: str, **attributes: Any) -> types.ModuleType:
    result = types.ModuleType(name)
    result.__dict__.update(attributes)
    return result


def _load_cabinet_client() -> types.ModuleType:
    fake_modules = {
        "action_msgs.msg": _module(
            "action_msgs.msg",
            GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_CANCELED=5),
        ),
        "rclpy.action": _module("rclpy.action", ActionClient=object),
        "rclpy.context": _module("rclpy.context", Context=object),
        "rclpy.node": _module("rclpy.node", Node=_Node),
        "rclpy.qos": _module(
            "rclpy.qos",
            DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
            ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
            QoSProfile=_QoS,
            qos_profile_sensor_data=object(),
        ),
        "sensor_msgs.msg": _module("sensor_msgs.msg", JointState=object),
        "std_msgs.msg": _module("std_msgs.msg", Bool=object),
        "std_srvs.srv": _module(
            "std_srvs.srv",
            Trigger=type("Trigger", (), {"Request": type("Request", (), {})}),
        ),
        "xczs_inspection_robot_control.action": _module(
            "xczs_inspection_robot_control.action",
            OperateCabinetControl=_Operate,
        ),
        "xczs_inspection_robot_control.msg": _module(
            "xczs_inspection_robot_control.msg",
            CabinetControl=_Control,
            CabinetControlCatalog=object,
            CabinetControlState=object,
        ),
    }
    path = JIANG_DIR / "control_gateway" / "cabinet_client.py"
    specification = importlib.util.spec_from_file_location(
        "_test_cabinet_client_module",
        path,
    )
    assert specification is not None and specification.loader is not None
    loaded = importlib.util.module_from_spec(specification)
    with patch.dict(sys.modules, fake_modules):
        specification.loader.exec_module(loaded)
    return loaded


CABINET_MODULE = _load_cabinet_client()
CabinetClient = CABINET_MODULE.CabinetClient


def _control(
    *,
    state_topic: str = "button/state",
    operable: bool = True,
    reason: str = "",
    default_force: float = 5.0,
) -> Any:
    return SimpleNamespace(
        control_id="button_1",
        display_name="按钮 1",
        control_type=_Control.TYPE_BUTTON,
        joint_name="button_joint",
        joint_state_topic="button/joint_states",
        pressed_topic="button/pressed",
        state_topic=state_topic,
        supported_commands=_Control.SUPPORT_PRESS,
        unit="m",
        min_position=0.0,
        max_position=0.008,
        state_ids=["released", "pressed"],
        state_labels=["释放", "按下"],
        state_positions=[0.0, 0.006],
        requires_grasp=False,
        operable=operable,
        unavailable_reason=reason,
        default_force=default_force,
        min_trigger_force=4.8,
        max_force=6.4,
    )


def _knob_control() -> Any:
    return SimpleNamespace(
        control_id="knob_1",
        display_name="旋钮 1",
        control_type=_Control.TYPE_KNOB,
        joint_name="knob_joint",
        joint_state_topic="knob/joint_states",
        pressed_topic="",
        state_topic="knob/state",
        supported_commands=(
            _Control.SUPPORT_SET_STATE
            | _Control.SUPPORT_SET_POSITION
            | _Control.SUPPORT_TOGGLE
        ),
        unit="rad",
        min_position=-1.0,
        max_position=1.0,
        state_ids=["left", "center", "right"],
        state_labels=["左", "中", "右"],
        state_positions=[-1.0, 0.0, 1.0],
        requires_grasp=True,
        operable=True,
        unavailable_reason="",
        default_force=0.0,
        min_trigger_force=0.0,
        max_force=0.0,
    )


def _state(position: float) -> Any:
    return SimpleNamespace(
        control_id="button_1",
        control_type=_Control.TYPE_BUTTON,
        valid=True,
        position=position,
        velocity=0.0,
        effort=0.0,
        normalized_position=position / 0.008,
        state_id="pressed" if position >= 0.006 else "released",
        activated=position >= 0.006,
        in_motion=False,
        transition_sequence=1,
    )


class CabinetClientTest(unittest.TestCase):
    def _client(
        self,
        name: str,
        events: List[Dict[str, Any]],
        action: Optional[_ActionClient] = None,
    ) -> tuple[Any, _ActionClient]:
        action = action or _ActionClient()
        return (
            CabinetClient(
                name,
                events.append,
                action_client_factory=lambda *_args: action,
                reset_client_factory=lambda *_args: _ResetClient(),
            ),
            action,
        )

    def test_absolute_interfaces_and_same_id_state_are_isolated(self) -> None:
        events_a: List[Dict[str, Any]] = []
        events_b: List[Dict[str, Any]] = []
        client_a, _ = self._client("cabinet_a", events_a)
        client_b, _ = self._client("cabinet_b", events_b)
        client_a._catalog_callback(SimpleNamespace(controls=[_control()]))
        client_b._catalog_callback(
            SimpleNamespace(
                controls=[
                    _control(
                        state_topic=(
                            "/xczs/cabinet/cabinet_b/absolute/state"
                        )
                    )
                ]
            )
        )

        self.assertEqual(
            "/xczs/cabinet/cabinet_a/operate_cabinet_control",
            client_a.action_name,
        )
        callback_a = next(
            item.callback
            for item in client_a.subscriptions
            if item.topic == "/xczs/cabinet/cabinet_a/button/state"
        )
        callback_b = next(
            item.callback
            for item in client_b.subscriptions
            if item.topic == "/xczs/cabinet/cabinet_b/absolute/state"
        )
        callback_a(_state(0.007))
        callback_b(_state(0.002))

        availability = client_a.snapshot_controls()
        self.assertTrue(availability["available"])
        self.assertTrue(availability["operation_available"])
        self.assertTrue(availability["reset_available"])
        self.assertEqual(
            0.007,
            availability["controls"][0]["current_position"],
        )
        self.assertEqual(
            0.002,
            client_b.snapshot_controls()["controls"][0]["current_position"],
        )

    def test_catalog_generation_blocks_retired_subscription_callback(self) -> None:
        client, _ = self._client("cabinet_a", [])
        client._catalog_callback(SimpleNamespace(controls=[_control()]))
        retired = next(
            item
            for item in client.subscriptions
            if item.topic == "/xczs/cabinet/cabinet_a/button/state"
        )
        retired.callback(_state(0.004))
        client._catalog_callback(
            SimpleNamespace(controls=[_control(state_topic="new/state")])
        )
        retired.callback(_state(0.008))
        self.assertTrue(retired.destroyed)
        self.assertIsNone(
            client.snapshot_controls()["controls"][0]["current_position"]
        )

    def test_aggregate_state_avoids_duplicate_legacy_subscriptions(self) -> None:
        client, _ = self._client("cabinet_a", [])
        client._catalog_callback(SimpleNamespace(controls=[_control()]))

        dynamic_topics = {
            item.topic
            for item in client.subscriptions
            if item.topic != client.catalog_topic
        }
        self.assertEqual(
            dynamic_topics,
            {"/xczs/cabinet/cabinet_a/button/state"},
        )

    def test_split_state_topics_remain_a_legacy_fallback(self) -> None:
        client, _ = self._client("cabinet_a", [])
        client._catalog_callback(
            SimpleNamespace(controls=[_control(state_topic="")])
        )

        dynamic_topics = {
            item.topic
            for item in client.subscriptions
            if item.topic != client.catalog_topic
        }
        self.assertEqual(
            dynamic_topics,
            {
                "/xczs/cabinet/cabinet_a/button/joint_states",
                "/xczs/cabinet/cabinet_a/button/pressed",
            },
        )

    def test_force_goal_feedback_and_insufficient_force_result(self) -> None:
        events: List[Dict[str, Any]] = []
        client, action = self._client("cabinet_a", events)
        client._catalog_callback(SimpleNamespace(controls=[_control()]))

        accepted = client.submit_operation(
            "button_1",
            "press",
            force=4.0,
        )
        self.assertEqual("accepted", accepted["status"])
        self.assertEqual(4.0, action.sent[0].force)
        self.assertFalse(action.sent[0].navigate_to_staging_pose)
        handle = _GoalHandle()
        action.goal_futures[0].complete(handle)
        action.feedback_callbacks[0](
            SimpleNamespace(
                feedback=SimpleNamespace(
                    phase=_Feedback.MANIPULATING,
                    progress=0.7,
                    current_position=0.005,
                    target_position=0.005,
                    current_state="released",
                    message="按压力度测量中",
                )
            )
        )
        handle.result_future.complete(
            SimpleNamespace(
                status=4,
                result=SimpleNamespace(
                    success=False,
                    error_code=_Result.INSUFFICIENT_FORCE,
                    message="力度不足",
                    initial_position=0.0,
                    final_position=0.0,
                    peak_position=0.005,
                    final_state="released",
                    requested_force=4.0,
                    estimated_force=4.0,
                    button_triggered=False,
                ),
            )
        )

        terminal = events[-1]
        self.assertEqual("terminal", terminal["event"])
        self.assertEqual("insufficient_force", terminal["failure_code"])
        self.assertEqual(4.0, terminal["result"]["requested_force"])
        self.assertEqual(4.0, terminal["result"]["estimated_force"])
        self.assertFalse(terminal["result"]["button_triggered"])
        self.assertEqual(0.005, terminal["result"]["peak_position"])

    def test_omitted_force_uses_catalog_default(self) -> None:
        client, action = self._client("cabinet_a", [])
        client._catalog_callback(
            SimpleNamespace(controls=[_control(default_force=5.6)])
        )

        client.submit_operation("button_1", "press", force=None)

        self.assertEqual(5.6, action.sent[0].force)

    def test_contact_detection_timeout_has_stable_failure_code(self) -> None:
        self.assertEqual(
            "contact_detection_timeout",
            CabinetClient._error_code_name(
                _Result.CONTACT_DETECTION_TIMEOUT
            ),
        )

    def test_global_operation_lease_has_stable_failure_codes(self) -> None:
        self.assertEqual(
            "resource_busy",
            CabinetClient._error_code_name(_Result.RESOURCE_BUSY),
        )
        self.assertEqual(
            "lease_lost",
            CabinetClient._error_code_name(_Result.LEASE_LOST),
        )

    def test_policy_limited_control_is_sent_for_live_validation(self) -> None:
        events: List[Dict[str, Any]] = []
        client, action = self._client("cabinet_a", events)
        reason = "当前机器人工作空间无法到达该按钮"
        client._catalog_callback(
            SimpleNamespace(controls=[_control(operable=False, reason=reason)])
        )

        result = client.submit_operation("button_1", "press", force=5.0)

        self.assertEqual("accepted", result["status"])
        self.assertEqual(1, len(action.sent))
        self.assertEqual("button_1", action.sent[0].control_id)
        handle = _GoalHandle()
        action.goal_futures[0].complete(handle)
        handle.result_future.complete(
            SimpleNamespace(
                status=4,
                result=SimpleNamespace(
                    success=False,
                    error_code=_Result.UNREACHABLE,
                    message="实时规划验证通过，但安全策略禁止物理执行",
                    initial_position=0.0,
                    final_position=0.0,
                    peak_position=0.0,
                    final_state="released",
                    requested_force=5.0,
                    estimated_force=0.0,
                    button_triggered=False,
                    validation_performed=True,
                    operation_executed=False,
                    diagnostic_stage="retreat",
                    path_fraction=1.0,
                    required_fraction=0.98,
                    moveit_error_code=1,
                    policy_reason=reason,
                ),
            )
        )

        terminal = events[-1]
        self.assertEqual("terminal", terminal["event"])
        self.assertEqual("failed", terminal["outcome"])
        self.assertEqual("unreachable", terminal["failure_code"])
        self.assertTrue(terminal["result"]["validation_performed"])
        self.assertFalse(terminal["result"]["operation_executed"])
        self.assertEqual("retreat", terminal["result"]["diagnostic_stage"])
        self.assertEqual(1.0, terminal["result"]["path_fraction"])
        self.assertEqual(0.98, terminal["result"]["required_fraction"])
        self.assertEqual(1, terminal["result"]["moveit_error_code"])
        self.assertEqual(reason, terminal["result"]["policy_reason"])

    def test_non_button_ignores_web_force_and_reports_zero_force(self) -> None:
        events: List[Dict[str, Any]] = []
        client, action = self._client("cabinet_a", events)
        client._catalog_callback(
            SimpleNamespace(controls=[_knob_control()])
        )

        accepted = client.submit_operation(
            "knob_1",
            "set_state",
            target_state="right",
            force=99.0,
        )

        self.assertEqual("accepted", accepted["status"])
        self.assertEqual(0.0, accepted["force"])
        self.assertEqual(0.0, action.sent[0].force)
        handle = _GoalHandle()
        action.goal_futures[0].complete(handle)
        handle.result_future.complete(
            SimpleNamespace(
                status=4,
                result=SimpleNamespace(
                    success=True,
                    error_code=_Result.SUCCESS,
                    message="旋钮操作成功",
                    initial_position=0.0,
                    final_position=1.0,
                    peak_position=1.0,
                    final_state="right",
                    requested_force=0.0,
                    estimated_force=0.0,
                    button_triggered=False,
                ),
            )
        )

        terminal = events[-1]
        self.assertEqual("success", terminal["outcome"])
        self.assertEqual(0.0, terminal["result"]["requested_force"])
        self.assertEqual(0.0, terminal["result"]["estimated_force"])
        self.assertFalse(terminal["result"]["button_triggered"])
        self.assertEqual("right", terminal["result"]["final_state"])

    def test_cancel_and_old_generation_result_do_not_mutate_new_goal(self) -> None:
        events: List[Dict[str, Any]] = []
        client, action = self._client("cabinet_a", events)
        client._catalog_callback(SimpleNamespace(controls=[_control()]))
        client.submit_operation("button_1", "press", force=5.0)
        old_handle = _GoalHandle()
        action.goal_futures[0].complete(old_handle)
        self.assertEqual("canceling", client.cancel()["status"])
        old_handle.cancel_future.complete(
            SimpleNamespace(goals_canceling=[object()])
        )
        old_handle.result_future.complete(
            SimpleNamespace(
                status=5,
                result=SimpleNamespace(
                    success=False,
                    error_code=_Result.CANCELED,
                    message="已取消",
                    initial_position=0.0,
                    final_position=0.0,
                    peak_position=0.0,
                    final_state="released",
                    requested_force=5.0,
                    estimated_force=0.0,
                    button_triggered=False,
                ),
            )
        )
        client.submit_operation("button_1", "press", force=5.0)
        terminal_count = sum(event["event"] == "terminal" for event in events)
        client._result_callback(
            SimpleNamespace(
                result=lambda: SimpleNamespace(
                    status=4,
                    result=SimpleNamespace(
                        success=True,
                        error_code=_Result.SUCCESS,
                        message="stale",
                        initial_position=0.0,
                        final_position=0.0,
                        peak_position=0.006,
                        final_state="pressed",
                        requested_force=5.0,
                        estimated_force=5.0,
                        button_triggered=True,
                    ),
                )
            ),
            1,
            old_handle,
        )
        self.assertEqual(
            terminal_count,
            sum(event["event"] == "terminal" for event in events),
        )
        self.assertEqual("sending", client.snapshot_status()["state"])


if __name__ == "__main__":
    unittest.main()
