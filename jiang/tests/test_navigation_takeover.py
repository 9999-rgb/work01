"""Regression tests for atomic Web-to-manual navigation takeover."""

from __future__ import annotations

import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, List, Optional


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from action_msgs.msg import GoalStatus  # noqa: E402
from builtin_interfaces.msg import Time as TimeMessage  # noqa: E402
from control_gateway.inventory import NavigationStationSpec  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.ros_node import RosControlNode  # noqa: E402
from tf2_ros import TransformException  # noqa: E402


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

    def update(self, target: float, period: float) -> float:
        del period
        return target


class _NavigationClient:
    def __init__(self) -> None:
        self.send_count = 0
        self.goal: Any = None

    def send_goal_async(self, *args: Any, **_kwargs: Any) -> Any:
        self.send_count += 1
        self.goal = args[0]
        return _DeferredFuture()

    def server_is_ready(self) -> bool:
        return True


class _TransformBuffer:
    def __init__(self, transform: Any) -> None:
        self.transform = transform
        self.requests = []

    def lookup_transform(self, target: str, source: str, stamp: Any) -> Any:
        self.requests.append((target, source, stamp))
        return self.transform


class _MissingTransformBuffer:
    def lookup_transform(self, target: str, source: str, stamp: Any) -> Any:
        del target, source, stamp
        raise TransformException("frame is not connected")


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
    node._robot_pose = None
    node._robot_pose_sequence = 0
    node._cabinet_pose_validity = {}
    node._navigation_frame = "map"
    node._navigation_base_frame = "base_link"
    node._manual_linear_axis = "y"
    node._ros_clock_now_nanoseconds = lambda: 23_500_000_000
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
    def test_navigation_goal_uses_configured_frame(self) -> None:
        node = _make_node(mode=True)
        node._navigation_frame = "robot_map"
        node._navigation_state.update(
            {
                "state": "enabling",
                "goal": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
        now = SimpleNamespace(
            nanoseconds=123,
            to_msg=lambda: TimeMessage(sec=0, nanosec=123),
        )
        node.get_clock = lambda: SimpleNamespace(now=lambda: now)

        with node._lock:
            node._send_navigation_goal_locked(generation=1)

        self.assertEqual(
            "robot_map",
            node._navigation_client.goal.pose.header.frame_id,
        )

    def test_manual_linear_axis_selects_twist_component(self) -> None:
        node = _make_node()
        node._manual_linear_axis = "x"
        node._command_timeout = 10.0
        node._last_command_time = time.monotonic()
        node._last_update_time = time.monotonic()
        node._pending_trajectory = None
        node._pending_trajectory_repeats = 0

        node._update_manual_control()

        command = node._cmd_vel_publisher.messages[-1]
        self.assertAlmostEqual(0.2, command.linear.x)
        self.assertAlmostEqual(0.0, command.linear.y)

    def test_rotated_occupancy_grid_goal_uses_inverse_origin_rotation(
        self,
    ) -> None:
        node = _make_node()
        node._map_state = {
            "frame_id": "map",
            "resolution": 1.0,
            "width": 2,
            "height": 2,
            "origin": {"x": 10.0, "y": 20.0, "yaw": math.pi / 2.0},
            "data": [0, 100, 0, 0],
        }

        node._validate_navigation_goal(9.8, 20.2)
        with self.assertRaisesRegex(ControlRequestError, "occupied"):
            node._validate_navigation_goal(9.8, 21.2)

    def test_map_callback_rejects_wrong_frame_and_truncated_data(self) -> None:
        node = _make_node()

        def message(frame_id: str, data: list[int]) -> Any:
            return SimpleNamespace(
                header=SimpleNamespace(frame_id=frame_id),
                info=SimpleNamespace(
                    resolution=1.0,
                    width=2,
                    height=1,
                    origin=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0),
                        orientation=SimpleNamespace(
                            x=0.0, y=0.0, z=0.0, w=1.0
                        ),
                    ),
                ),
                data=data,
            )

        node._map_callback(message("other_map", [0, 0]))
        with self.assertRaisesRegex(ControlRequestError, "does not match"):
            node.map_snapshot()

        node._map_callback(message("map", [0]))
        with self.assertRaisesRegex(ControlRequestError, "data length"):
            node.map_snapshot()

        node._map_callback(message("map", [0, 100]))
        snapshot = node.map_snapshot()
        self.assertEqual("map", snapshot["frame_id"])
        self.assertEqual([0, 100], snapshot["data"])

    def test_pose_snapshot_retains_frame_stamp_and_freshness(self) -> None:
        node = _make_node()
        message = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="map",
                stamp=SimpleNamespace(sec=12, nanosec=34),
            ),
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=2.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            ),
        )

        node._pose_callback(message)

        self.assertEqual("map", node._robot_pose["frame_id"])
        self.assertEqual({"sec": 12, "nanosec": 34}, node._robot_pose["stamp"])
        self.assertEqual(
            12_000_000_034,
            node._robot_pose["stamp_ros_nanoseconds"],
        )
        self.assertEqual(
            23_500_000_000,
            node._robot_pose["observed_ros_nanoseconds"],
        )
        self.assertAlmostEqual(
            11.499999966,
            node._robot_pose["stamp_age_seconds"],
        )
        self.assertEqual("amcl", node._robot_pose["source"])
        self.assertEqual(1, node._robot_pose["sequence"])
        self.assertGreater(node._robot_pose["received_monotonic"], 0.0)

    def test_tf_refresh_supplies_current_navigation_pose(self) -> None:
        node = _make_node()
        node._navigation_frame = "robot_map"
        transform = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="robot_map",
                stamp=SimpleNamespace(sec=23, nanosec=45),
            ),
            child_frame_id="base_link",
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.25, y=-0.5),
                rotation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(0.2 / 2.0),
                    w=math.cos(0.2 / 2.0),
                ),
            ),
        )
        node._tf_buffer = _TransformBuffer(transform)

        node._refresh_robot_pose_from_tf()

        self.assertEqual(
            ("robot_map", "base_link"),
            node._tf_buffer.requests[0][:2],
        )
        self.assertAlmostEqual(1.25, node._robot_pose["x"])
        self.assertAlmostEqual(-0.5, node._robot_pose["y"])
        self.assertAlmostEqual(0.2, node._robot_pose["yaw"])
        self.assertEqual("tf", node._robot_pose["source"])
        self.assertEqual("base_link", node._robot_pose["child_frame_id"])
        self.assertEqual(
            23_000_000_045,
            node._robot_pose["stamp_ros_nanoseconds"],
        )
        self.assertEqual(
            23_500_000_000,
            node._robot_pose["observed_ros_nanoseconds"],
        )
        self.assertAlmostEqual(
            0.499999955,
            node._robot_pose["stamp_age_seconds"],
        )

    def test_live_station_tf_includes_current_map_odom_correction(self) -> None:
        node = _make_node()
        cabinet_yaw = math.pi / 2.0
        transform = SimpleNamespace(
            transform=SimpleNamespace(
                # This is the live map->cabinet result after TF has composed
                # the current map->odom localization correction.
                translation=SimpleNamespace(x=10.0, y=-4.0, z=1.0),
                rotation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(cabinet_yaw / 2.0),
                    w=math.cos(cabinet_yaw / 2.0),
                ),
            )
        )
        node._tf_buffer = _TransformBuffer(transform)
        spec = NavigationStationSpec(
            local_anchor=(1.0, 2.0, 0.5),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.2,
            frame_id="map",
        )

        station = node.navigation_station_from_tf(
            "cabinet_a",
            "cabinet_a_frame",
            spec,
        )

        self.assertEqual(
            ("map", "cabinet_a_frame"),
            node._tf_buffer.requests[0][:2],
        )
        self.assertAlmostEqual(8.0, station.x)
        self.assertAlmostEqual(-2.5, station.y)
        self.assertAlmostEqual(1.5, station.z)
        self.assertAlmostEqual(-math.pi / 2.0 + 0.2, station.yaw)
        self.assertEqual("map", station.frame_id)

    def test_live_station_requires_available_cabinet_tf(self) -> None:
        node = _make_node()
        node._tf_buffer = _MissingTransformBuffer()
        spec = NavigationStationSpec(
            local_anchor=(0.0, 0.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=1.0,
            frame_id="map",
        )

        with self.assertRaises(ControlRequestError) as unavailable:
            node.navigation_station_from_tf(
                "cabinet_a",
                "cabinet_a_frame",
                spec,
            )

        self.assertEqual(503, unavailable.exception.status)
        self.assertIn("not available", str(unavailable.exception))

    def test_live_station_rejects_invalid_or_unconfirmed_pose_authority(
        self,
    ) -> None:
        node = _make_node()
        node._cabinet_pose_validity = {"cabinet_a": None}
        node._tf_buffer = _TransformBuffer(
            SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
        spec = NavigationStationSpec(
            local_anchor=(0.0, 0.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=1.0,
            frame_id="map",
        )

        with self.assertRaisesRegex(ControlRequestError, "not confirmed"):
            node.navigation_station_from_tf(
                "cabinet_a", "cabinet_a_frame", spec
            )
        node._cabinet_pose_validity["cabinet_a"] = False
        with self.assertRaisesRegex(ControlRequestError, "invalid"):
            node.navigation_station_from_tf(
                "cabinet_a", "cabinet_a_frame", spec
            )

        node._cabinet_pose_validity_callback(
            "cabinet_a", SimpleNamespace(data=True)
        )
        station = node.navigation_station_from_tf(
            "cabinet_a", "cabinet_a_frame", spec
        )
        self.assertAlmostEqual(2.0, station.x)

    def test_navigation_goal_requires_map_and_matching_frame(self) -> None:
        node = _make_node()
        with self.assertRaisesRegex(ControlRequestError, "not available") as missing:
            node._validate_navigation_goal(0.0, 0.0)
        self.assertEqual(503, missing.exception.status)

        node._map_state = {
            "frame_id": "other_map",
            "resolution": 1.0,
            "width": 1,
            "height": 1,
            "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "data": [0],
        }
        with self.assertRaisesRegex(ControlRequestError, "does not match") as frame:
            node._validate_navigation_goal(0.0, 0.0)
        self.assertEqual(503, frame.exception.status)

    def test_navigation_goal_rejects_nonfinite_coordinates(self) -> None:
        node = _make_node()
        with self.assertRaisesRegex(ControlRequestError, "finite"):
            node._validate_navigation_goal(float("nan"), 0.0)

    def test_live_station_rejects_nonfinite_tf_and_spec_values(self) -> None:
        node = _make_node()
        node._tf_buffer = _TransformBuffer(
            SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(
                        x=float("nan"),
                        y=0.0,
                        z=0.0,
                    ),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
        valid_spec = NavigationStationSpec(
            local_anchor=(0.0, 0.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=1.0,
            frame_id="map",
        )

        with self.assertRaises(ControlRequestError) as invalid_tf:
            node.navigation_station_from_tf(
                "cabinet_a",
                "cabinet_a_frame",
                valid_spec,
            )
        self.assertEqual(503, invalid_tf.exception.status)
        self.assertIn("finite", str(invalid_tf.exception))

        invalid_spec = NavigationStationSpec(
            local_anchor=(0.0, 0.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=float("inf"),
            frame_id="map",
        )
        with self.assertRaises(ControlRequestError) as invalid_geometry:
            node.navigation_station_from_tf(
                "cabinet_a",
                "cabinet_a_frame",
                invalid_spec,
            )
        self.assertEqual(400, invalid_geometry.exception.status)
        self.assertIn("finite", str(invalid_geometry.exception))

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

        # Nav2 不可用时 takeover 不再抛异常，而是直接进入手动模式。
        result = node.takeover_navigation()

        self.assertEqual("manual", result["status"])
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
