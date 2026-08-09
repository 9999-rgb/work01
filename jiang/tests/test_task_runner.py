"""Task orchestration tests for the multi-cabinet Web gateway."""

from __future__ import annotations

import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Mapping, Optional
from unittest.mock import patch


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.cabinet_client import CabinetClientError  # noqa: E402
from control_gateway.inventory import CabinetInstance  # noqa: E402
from control_gateway.inventory import CabinetInventory  # noqa: E402
from control_gateway.inventory import NavigationStationSpec  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway import runner as runner_module  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402
from control_gateway.task_manager import TaskManager  # noqa: E402
from control_gateway.task_manager import TaskExecutionError  # noqa: E402


class _NavigationNode:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.goal_event = threading.Event()
        self.goal: Optional[Dict[str, float]] = None
        self.navigation_goal_count = 0
        self.cancel_count = 0
        self.takeover_count = 0
        self.base_targets = []
        self.joint_targets = []
        self.navigation_mode_requests = []
        self.task_events = []
        self.ros_time_nanoseconds = 100_000_000_000
        self.state: Dict[str, Any] = {
            "available": True,
            "state": "idle",
            "message": "idle",
            "goal": None,
            "goal_sent_ros_nanoseconds": None,
            "current_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "distance_remaining": None,
            "eta_seconds": None,
            "recoveries": 0,
        }

    def map_snapshot(self) -> Dict[str, Any]:
        return {
            "width": 200,
            "height": 200,
            "resolution": 0.05,
            "origin": {"x": -5.0, "y": -5.0, "yaw": 0.0},
            "data": [],
        }

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Dict[str, Any]:
        with self._lock:
            self.navigation_goal_count += 1
            self.goal = {"x": x, "y": y, "yaw": yaw}
            self.state.update(
                {
                    "state": "navigating",
                    "message": "driving",
                    "goal": dict(self.goal),
                    "goal_sent_ros_nanoseconds": (
                        self.ros_time_nanoseconds
                    ),
                    "distance_remaining": 1.0,
                }
            )
            self.goal_event.set()
        return {"status": "accepted", "goal": dict(self.goal)}

    def navigation_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.state)

    def finish_navigation(
        self,
        state: str,
        *,
        pose: Optional[Mapping[str, float]] = None,
        message: str = "done",
    ) -> None:
        with self._lock:
            self.state.update(
                {
                    "state": state,
                    "message": message,
                    "distance_remaining": 0.0,
                }
            )
            if pose is not None:
                self.ros_time_nanoseconds += 100_000_000
                current_pose = dict(pose)
                current_pose.setdefault("frame_id", "map")
                current_pose.setdefault(
                    "stamp_ros_nanoseconds",
                    self.ros_time_nanoseconds,
                )
                current_pose.setdefault(
                    "observed_ros_nanoseconds",
                    self.ros_time_nanoseconds,
                )
                current_pose.setdefault(
                    "received_monotonic",
                    time.monotonic(),
                )
                self.state["current_pose"] = current_pose

    def cancel_navigation(self, allow_idle: bool = False) -> Dict[str, Any]:
        del allow_idle
        with self._lock:
            self.cancel_count += 1
            if self.state["state"] in {"idle", "canceled", "succeeded"}:
                return {"status": "idle"}
            self.state.update(state="canceling", message="canceling")
        return {"status": "canceling"}

    def takeover_navigation(self) -> Dict[str, Any]:
        with self._lock:
            self.takeover_count += 1
            self.state.update(state="canceled", message="manual takeover")
        return {"status": "taking_over", "mode": True}

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        self.navigation_mode_requests.append(enabled)
        return {"status": "accepted", "enabled": enabled}

    def set_base_target(
        self,
        linear_y: float,
        angular_z: float,
    ) -> tuple[float, float]:
        self.base_targets.append((linear_y, angular_z))
        return linear_y, angular_z

    def set_joint_target(self, positions: list[float]) -> list[float]:
        self.joint_targets.append(list(positions))
        return list(positions)

    def publish_task_event(self, event: Dict[str, Any]) -> None:
        self.task_events.append(event)


class _CabinetClient:
    def __init__(self, name: str, listener: Any) -> None:
        self.name = name
        self.listener = listener
        self.submit_event = threading.Event()
        self.submissions = []
        self.cancel_count = 0
        self.submit_error: Optional[Exception] = None
        self.status = {
            "available": True,
            "active": False,
            "state": "idle",
        }

    def snapshot_controls(self) -> Dict[str, Any]:
        return {
            "available": True,
            "catalog_received": True,
            "controls": [
                {
                    "control_id": "button_1",
                    "control_type": 0,
                    "default_force": 5.0,
                    "min_trigger_force": 4.8,
                    "max_force": 6.4,
                    "operable": True,
                }
            ],
        }

    def snapshot_status(self) -> Dict[str, Any]:
        return dict(self.status)

    def submit_operation(self, *args: Any, **kwargs: Any) -> Dict[str, Any]:
        if self.submit_error is not None:
            raise self.submit_error
        self.submissions.append((args, kwargs))
        self.status.update(active=True, state="operating")
        self.submit_event.set()
        self.listener(
            {
                "event": "feedback",
                "cabinet": self.name,
                "generation": 1,
                "timestamp": time.time(),
                "phase": "approaching",
                "phase_code": 4,
                "progress": 0.4,
                "current_position": 0.001,
                "target_position": 0.00625,
                "current_state": "released",
                "message": "approaching",
            }
        )
        return {"status": "accepted"}

    def finish(
        self,
        outcome: str,
        *,
        failure_code: Optional[str] = None,
        message: str = "done",
    ) -> None:
        self.status.update(active=False, state=outcome)
        self.listener(
            {
                "event": "terminal",
                "cabinet": self.name,
                "generation": 1,
                "timestamp": time.time(),
                "outcome": outcome,
                "success": outcome == "success",
                "failure_code": failure_code,
                "error_code": 0 if outcome == "success" else 13,
                "message": message,
                "result": {
                    "initial_position": 0.0,
                    "final_position": 0.0,
                    "peak_position": 0.00625,
                    "final_state": "released",
                    "requested_force": 5.0,
                    "estimated_force": 5.0,
                    "button_triggered": outcome == "success",
                },
            }
        )

    def cancel(self) -> Dict[str, Any]:
        self.cancel_count += 1
        return {"status": "canceling", "cabinet": self.name}

    def reset(self) -> Dict[str, Any]:
        return {"status": "reset", "cabinet": self.name}


def _inventory(count: int = 2) -> CabinetInventory:
    instances = [
        CabinetInstance(
            name=f"cabinet_{letter}",
            x=float(index),
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
        for index, letter in enumerate(("a", "b")[:count])
    ]
    station = NavigationStationSpec(
        local_anchor=(0.0, 0.0, 0.0),
        outward_axis=(1.0, 0.0, 0.0),
        standoff=1.0,
        base_yaw_offset=0.0,
        frame_id="map",
    )
    return CabinetInventory(instances, station)


def _server(count: int = 2) -> tuple[ControlServer, _NavigationNode]:
    server = object.__new__(ControlServer)
    server._inventory = _inventory(count)
    server._robot_adapter = SimpleNamespace(navigation_frame="map")
    server._node = _NavigationNode()
    server._task_manager = TaskManager()
    server._task_interlock_lock = threading.RLock()
    server._operation_bindings_lock = threading.RLock()
    server._operation_event_queues = {}
    server._cabinet_clients = {
        name: _CabinetClient(name, server._cabinet_event_listener)
        for name in server._inventory.names
    }
    server._request_condition = threading.Condition()
    server._active_requests = 0
    server._stopping = False
    return server, server._node


class TaskRunnerTest(unittest.TestCase):
    def assert_task_conflict(
        self,
        task_id: str,
        callback: Any,
    ) -> None:
        with self.assertRaises(ControlRequestError) as conflict:
            callback()
        self.assertEqual(409, conflict.exception.status)
        self.assertEqual(
            task_id,
            conflict.exception.details["active_task_id"],
        )

    def test_control_gateway_rejects_non_loopback_bind(self) -> None:
        with self.assertRaisesRegex(ValueError, "loopback"):
            ControlServer(host="0.0.0.0")

    def test_loopback_host_validation_accepts_supported_forms(self) -> None:
        self.assertTrue(ControlServer._is_loopback_host("127.0.0.1"))
        self.assertTrue(ControlServer._is_loopback_host("::1"))
        self.assertTrue(ControlServer._is_loopback_host("localhost"))
        self.assertFalse(ControlServer._is_loopback_host("192.0.2.10"))

    def test_inventory_and_scoped_catalog(self) -> None:
        server, _node = _server()

        listed = server.cabinets()
        self.assertEqual(2, listed["count"])
        self.assertEqual(
            ["cabinet_a", "cabinet_b"],
            [value["name"] for value in listed["cabinets"]],
        )
        controls = server.cabinet_controls("cabinet_b")
        self.assertEqual("cabinet_b", controls["cabinet"])
        self.assertEqual("button_1", controls["controls"][0]["control_id"])

        with self.assertRaises(ControlRequestError) as unknown:
            server.cabinet_controls("missing")
        self.assertEqual(404, unknown.exception.status)
        with self.assertRaises(ControlRequestError) as legacy:
            server.cabinet_controls()
        self.assertEqual(410, legacy.exception.status)

    def test_navigation_success_reports_final_pose_and_error(self) -> None:
        server, node = _server()
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertRegex(accepted["task_id"], r"^navigate_\d+_[a-z0-9]{6}$")
        self.assertTrue(node.goal_event.wait(timeout=1.0))
        assert node.goal is not None
        self.assertAlmostEqual(1.0, node.goal["x"])
        self.assertAlmostEqual(math.pi, abs(node.goal["yaw"]))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.05, "y": 0.0, "yaw": math.pi - 0.03},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertAlmostEqual(0.05, task["result"]["error"]["position_m"])
        self.assertAlmostEqual(0.03, task["result"]["error"]["yaw_rad"])
        self.assertEqual("cabinet_a", task["result"]["cabinet"])

    def test_navigation_result_rejects_pose_that_stopped_updating(self) -> None:
        station = {
            "cabinet": "cabinet_a",
            "frame_id": "map",
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
        }
        snapshot = {
            "current_pose": {
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "frame_id": "map",
                "stamp_ros_nanoseconds": 20_000_000_000,
                "observed_ros_nanoseconds": 20_100_000_000,
                "received_monotonic": 10.0,
            }
        }

        with self.assertRaises(TaskExecutionError) as stale:
            ControlServer._navigation_result(
                station,
                snapshot,
                2.0,
                minimum_pose_stamp_ros_nanoseconds=19_000_000_000,
                minimum_pose_received_monotonic=9.0,
                observation_monotonic=11.1,
            )

        self.assertEqual("final_pose_stale", stale.exception.code)

    def test_navigation_result_rejects_old_tf_cache_timestamp(self) -> None:
        station = {
            "cabinet": "cabinet_a",
            "frame_id": "map",
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
        }
        snapshot = {
            "current_pose": {
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
                "frame_id": "map",
                # The cache was queried now, but its transform is old.
                "stamp_ros_nanoseconds": 20_000_000_000,
                "observed_ros_nanoseconds": 21_100_000_000,
                "received_monotonic": 10.0,
                "source": "tf",
            }
        }

        with self.assertRaises(TaskExecutionError) as stale:
            ControlServer._navigation_result(
                station,
                snapshot,
                2.0,
                minimum_pose_stamp_ros_nanoseconds=19_000_000_000,
                minimum_pose_received_monotonic=9.0,
                observation_monotonic=10.1,
            )

        self.assertEqual("final_pose_stale", stale.exception.code)
        self.assertGreater(
            stale.exception.details["pose_ros_age_seconds"],
            runner_module.NAVIGATION_FINAL_POSE_MAX_AGE_SEC,
        )

    def test_navigation_result_rejects_zero_or_predating_ros_stamp(
        self,
    ) -> None:
        station = {
            "cabinet": "cabinet_a",
            "frame_id": "map",
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
        }
        pose = {
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "frame_id": "map",
            "stamp_ros_nanoseconds": 0,
            "observed_ros_nanoseconds": 20_100_000_000,
            "received_monotonic": 10.0,
            "source": "tf",
        }

        with self.assertRaises(TaskExecutionError) as zero_stamp:
            ControlServer._navigation_result(
                station,
                {"current_pose": pose},
                2.0,
                minimum_pose_stamp_ros_nanoseconds=20_000_000_000,
                minimum_pose_received_monotonic=9.0,
                observation_monotonic=10.1,
            )
        self.assertEqual("final_pose_stale", zero_stamp.exception.code)

        pose.update(
            stamp_ros_nanoseconds=19_900_000_000,
            observed_ros_nanoseconds=20_000_000_000,
        )
        with self.assertRaises(TaskExecutionError) as predating:
            ControlServer._navigation_result(
                station,
                {"current_pose": pose},
                2.0,
                minimum_pose_stamp_ros_nanoseconds=20_000_000_000,
                minimum_pose_received_monotonic=9.0,
                observation_monotonic=10.1,
            )
        self.assertEqual("final_pose_stale", predating.exception.code)
        self.assertEqual(
            20_000_000_000,
            predating.exception.details["goal_sent_ros_nanoseconds"],
        )

    def test_navigation_failure_is_honestly_classified_unreachable(
        self,
    ) -> None:
        server, node = _server()
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertTrue(node.goal_event.wait(timeout=1.0))
        node.finish_navigation("failed", message="Nav2 aborted")

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("failed", task["status"])
        self.assertEqual("target_unreachable", task["failure_code"])
        self.assertIn(
            "does not expose",
            task["failure_details"]["classification"],
        )

    def test_navigation_timeout_is_terminal_but_retains_backend_lock(
        self,
    ) -> None:
        server, node = _server()
        with patch.object(
            runner_module,
            "NAVIGATION_TIMEOUT_SEC",
            0.02,
        ), patch.object(
            runner_module,
            "NAVIGATION_CANCEL_GRACE_SEC",
            0.02,
        ):
            accepted = server.submit_navigation_task("cabinet_a")
            failed = server._task_manager.wait(
                accepted["task_id"],
                timeout=1.0,
            )
            self.assertEqual("failed", failed["status"])
            self.assertEqual("navigation_timeout", failed["failure_code"])
            self.assertTrue(failed["reservation_active"])
            self.assertEqual(
                accepted["task_id"],
                server._task_manager.active_task_id,
            )
            self.assert_task_conflict(
                accepted["task_id"],
                lambda: server.submit_navigation_task("cabinet_b"),
            )

            node.finish_navigation("canceled")
            deadline = time.monotonic() + 1.0
            while (
                server._task_manager.active_task_id is not None
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertIsNone(server._task_manager.active_task_id)
            released = server._task_manager.get_task(accepted["task_id"])
            self.assertFalse(released["reservation_active"])
            self.assertTrue(released["backend_termination_confirmed"])

    def test_cancel_holds_global_lock_until_nav2_terminal(self) -> None:
        server, node = _server()
        first = server.submit_navigation_task("cabinet_a")
        self.assertTrue(node.goal_event.wait(timeout=1.0))

        canceling = server.cancel_task(first["task_id"])
        self.assertEqual("canceling", canceling["status"])
        with self.assertRaises(ControlRequestError) as conflict:
            server.submit_navigation_task("cabinet_b")
        self.assertEqual(409, conflict.exception.status)
        self.assertEqual(
            first["task_id"],
            conflict.exception.details["active_task_id"],
        )

        node.finish_navigation("canceled")
        terminal = server._task_manager.wait(first["task_id"], timeout=2.0)
        self.assertEqual("canceled", terminal["status"])
        self.assertIsNone(server._task_manager.active_task_id)

    def test_operation_forces_no_navigation_and_maps_result(self) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_b"]
        accepted = server.submit_operation_task(
            "cabinet_b",
            "button_1",
            "press",
            None,
            None,
            5.0,
        )
        self.assertTrue(client.submit_event.wait(timeout=1.0))
        _args, kwargs = client.submissions[0]
        self.assertFalse(kwargs["navigate"])
        self.assertEqual(5.0, kwargs["force"])
        client.finish("success")

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertAlmostEqual(0.00625, task["result"]["actual_displacement"])
        self.assertTrue(task["result"]["button_triggered"])

    def test_operation_cancel_waits_for_action_terminal(self) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_a"]
        accepted = server.submit_operation_task(
            "cabinet_a",
            "button_1",
            "press",
            None,
            None,
            5.0,
        )
        self.assertTrue(client.submit_event.wait(timeout=1.0))

        canceling = server.cancel_task(accepted["task_id"])
        self.assertEqual("canceling", canceling["status"])
        self.assertEqual(1, client.cancel_count)
        with self.assertRaises(ControlRequestError) as conflict:
            server.submit_navigation_task("cabinet_b")
        self.assertEqual(409, conflict.exception.status)

        client.finish("canceled", failure_code="canceled")
        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])

    def test_operation_timeout_is_terminal_but_retains_backend_lock(
        self,
    ) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_a"]
        with patch.object(
            runner_module,
            "OPERATION_TIMEOUT_SEC",
            0.02,
        ), patch.object(
            runner_module,
            "OPERATION_CANCEL_GRACE_SEC",
            0.02,
        ):
            accepted = server.submit_operation_task(
                "cabinet_a",
                "button_1",
                "press",
                None,
                None,
                5.0,
            )
            failed = server._task_manager.wait(
                accepted["task_id"],
                timeout=1.0,
            )
            self.assertEqual("operation_timeout", failed["failure_code"])
            self.assertTrue(failed["reservation_active"])
            self.assert_task_conflict(
                accepted["task_id"],
                lambda: server.submit_navigation_task("cabinet_b"),
            )

            client.finish("canceled", failure_code="canceled")
            deadline = time.monotonic() + 1.0
            while (
                server._task_manager.active_task_id is not None
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertIsNone(server._task_manager.active_task_id)

    def test_operation_task_interlocks_legacy_motion_routes(self) -> None:
        server, node = _server()
        client = server._cabinet_clients["cabinet_a"]
        accepted = server.submit_operation_task(
            "cabinet_a",
            "button_1",
            "press",
            None,
            None,
            5.0,
        )
        self.assertTrue(client.submit_event.wait(timeout=1.0))

        blocked_calls = (
            lambda: server.publish_cmd_vel(0.1, 0.2),
            lambda: server.publish_joint_trajectory([0.0] * 8),
            lambda: server.set_navigation_mode(True),
            lambda: server.set_navigation_mode(False),
            server.takeover_navigation,
            lambda: server.press_cabinet_button("button_1", False),
            lambda: server.operate_cabinet_control(
                "button_1",
                "press",
                None,
                None,
                False,
            ),
            server.reset_cabinet_controls,
        )
        for callback in blocked_calls:
            with self.subTest(callback=callback):
                self.assert_task_conflict(accepted["task_id"], callback)

        canceling = server.cancel_cabinet_operation()
        self.assertEqual("canceling", canceling["status"])
        for callback in blocked_calls:
            with self.subTest(canceling=True, callback=callback):
                self.assert_task_conflict(accepted["task_id"], callback)

        self.assertEqual([], node.base_targets)
        self.assertEqual([], node.joint_targets)
        self.assertEqual([], node.navigation_mode_requests)
        self.assertEqual(0, node.navigation_goal_count)
        self.assertEqual(0, node.takeover_count)

        client.finish("canceled", failure_code="canceled")
        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])

    def test_navigation_task_blocks_legacy_mode_but_allows_cancel(
        self,
    ) -> None:
        server, node = _server()
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertTrue(node.goal_event.wait(timeout=1.0))

        self.assert_task_conflict(
            accepted["task_id"],
            lambda: server.set_navigation_mode(True),
        )
        self.assert_task_conflict(
            accepted["task_id"],
            lambda: server.set_navigation_mode(False),
        )
        self.assert_task_conflict(
            accepted["task_id"],
            lambda: server.publish_cmd_vel(0.1, 0.0),
        )
        self.assert_task_conflict(
            accepted["task_id"],
            lambda: server.press_cabinet_button("button_1", False),
        )
        self.assert_task_conflict(
            accepted["task_id"],
            server.reset_cabinet_controls,
        )
        self.assertEqual([], node.navigation_mode_requests)
        self.assertEqual(1, node.navigation_goal_count)

        canceling = server.cancel_navigation()
        self.assertEqual("canceling", canceling["status"])
        self.assertEqual(1, node.cancel_count)
        node.finish_navigation("canceled")
        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])

    def test_navigation_task_allows_takeover_and_cancels_task_owner(self) -> None:
        server, node = _server()
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertTrue(node.goal_event.wait(timeout=1.0))

        takeover = server.takeover_navigation()
        self.assertEqual("taking_over", takeover["status"])
        self.assertEqual(1, node.cancel_count)
        self.assertEqual(1, node.takeover_count)
        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])

    def test_immediate_navigation_cancel_never_sends_late_goal(self) -> None:
        server, node = _server()
        with server._task_interlock_lock:
            accepted = server.submit_navigation_task("cabinet_a")
            canceling = server.cancel_task(accepted["task_id"])
            self.assertEqual("canceling", canceling["status"])

        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])
        self.assertEqual(0, node.navigation_goal_count)

    def test_immediate_operation_cancel_never_sends_late_action(self) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_a"]
        with server._task_interlock_lock:
            accepted = server.submit_operation_task(
                "cabinet_a",
                "button_1",
                "press",
                None,
                None,
                5.0,
            )
            canceling = server.cancel_task(accepted["task_id"])
            self.assertEqual("canceling", canceling["status"])

        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])
        self.assertEqual([], client.submissions)

    def test_task_events_are_mirrored_to_ros_topics(self) -> None:
        server, node = _server()
        server._event_bridge_subscription = (
            server._task_manager.events.subscribe(last_event_id=0)
        )
        server._event_bridge_thread = threading.Thread(
            target=server._bridge_task_events,
            daemon=True,
        )
        server._event_bridge_thread.start()
        task = server._task_manager.create_task("operate", {"cabinet": "a"})
        server._task_manager.fail_task(
            task["task_id"],
            "test failure",
            code="test_failure",
        )
        deadline = time.monotonic() + 1.0
        while len(node.task_events) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        server._task_manager.events.close()
        server._event_bridge_thread.join(timeout=1.0)

        self.assertEqual(
            ["task_accepted", "task_completed"],
            [event["event"] for event in node.task_events],
        )
        self.assertEqual(
            task["task_id"],
            node.task_events[-1]["data"]["task_id"],
        )

    def test_insufficient_force_and_not_ready_are_terminal_task_failures(
        self,
    ) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_a"]
        low_force = server.submit_operation_task(
            "cabinet_a",
            "button_1",
            "press",
            None,
            None,
            4.0,
        )
        self.assertTrue(client.submit_event.wait(timeout=1.0))
        client.finish(
            "failed",
            failure_code="insufficient_force",
            message="力度不足",
        )
        failed = server._task_manager.wait(low_force["task_id"], timeout=2.0)
        self.assertEqual("failed", failed["status"])
        self.assertEqual("insufficient_force", failed["failure_code"])
        self.assertEqual("力度不足", failed["failure_reason"])

        client.submit_event.clear()
        client.submit_error = CabinetClientError(
            "action server unavailable",
            code="not_ready",
            status=503,
        )
        unavailable = server.submit_operation_task(
            "cabinet_a",
            "button_1",
            "press",
            None,
            None,
            None,
        )
        terminal = server._task_manager.wait(
            unavailable["task_id"],
            timeout=2.0,
        )
        self.assertEqual("failed", terminal["status"])
        self.assertEqual("backend_unavailable", terminal["failure_code"])


if __name__ == "__main__":
    unittest.main()
