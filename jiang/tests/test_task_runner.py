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
from control_gateway.inventory import MapBounds  # noqa: E402
from control_gateway.inventory import NavigationStation  # noqa: E402
from control_gateway.inventory import NavigationStationSpec  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway import runner as runner_module  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402
from control_gateway.task_manager import TaskManager  # noqa: E402
from control_gateway.task_manager import TaskExecutionError  # noqa: E402


class _NavigationNode:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._goal_condition = threading.Condition(self._lock)
        self.goal_event = threading.Event()
        self.goal: Optional[Dict[str, float]] = None
        self.navigation_goals: list[Dict[str, float]] = []
        self.navigation_goal_count = 0
        self.cancel_count = 0
        self.takeover_count = 0
        self.quiesce_count = 0
        self.base_targets = []
        self.joint_targets = []
        self.joint_target_durations = []
        self.joint_target_event = threading.Event()
        self.joint_state_available = True
        self.joint_positions: Dict[str, float] = {}
        self.joint_state_received_monotonic: Optional[float] = None
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
        with self._goal_condition:
            self.navigation_goal_count += 1
            self.goal = {"x": x, "y": y, "yaw": yaw}
            self.navigation_goals.append(dict(self.goal))
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
            self._goal_condition.notify_all()
        return {"status": "accepted", "goal": dict(self.goal)}

    def wait_for_navigation_goal(
        self,
        count: int,
        timeout: float = 1.0,
    ) -> Optional[Dict[str, float]]:
        deadline = time.monotonic() + timeout
        with self._goal_condition:
            while len(self.navigation_goals) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._goal_condition.wait(remaining)
            return dict(self.navigation_goals[count - 1])

    def navigation_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            current_pose = self.state.get("current_pose")
            if (
                isinstance(current_pose, Mapping)
                and "stamp_ros_nanoseconds" in current_pose
            ):
                # Production RosControlNode refreshes map->base from TF on
                # every snapshot. Mirror its fresh monotonic receipt time so
                # post-station-refresh handoff checks exercise the same rule.
                refreshed_pose = dict(current_pose)
                refreshed_pose["received_monotonic"] = (
                    runner_module.time.monotonic()
                )
                self.state["current_pose"] = refreshed_pose
            return dict(self.state)

    def set_navigation_ros_time(self, nanoseconds: int) -> None:
        with self._lock:
            self.ros_time_nanoseconds = nanoseconds
            current_pose = dict(self.state["current_pose"])
            current_pose["observed_ros_nanoseconds"] = nanoseconds
            self.state["current_pose"] = current_pose

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

    def quiesce_manual_outputs(self) -> float:
        self.quiesce_count += 1
        return 0.0

    def set_joint_target(
        self,
        positions: list[float],
        duration_sec: float = 0.5,
    ) -> list[float]:
        self.joint_targets.append(list(positions))
        self.joint_target_durations.append(duration_sec)
        self.joint_positions = {
            name: position
            for name, position in zip(
                (
                    "body_arm1",
                    "arm1_arm2",
                    "arm2_arm3",
                    "arm3_arm4",
                    "arm4_arm5",
                    "arm5_end",
                    "end_worklink1",
                    "end_worklink2",
                ),
                positions,
            )
        }
        self.joint_state_received_monotonic = time.monotonic()
        self.joint_target_event.set()
        return list(positions)

    def robot_joint_state_snapshot(self) -> Dict[str, Any]:
        return {
            "available": self.joint_state_available,
            "positions": dict(self.joint_positions),
            "stamp_ros_nanoseconds": self.ros_time_nanoseconds,
            "received_monotonic": self.joint_state_received_monotonic,
        }

    def publish_task_event(self, event: Dict[str, Any]) -> None:
        self.task_events.append(event)


class _CabinetClient:
    def __init__(self, name: str, listener: Any) -> None:
        self.name = name
        self.listener = listener
        self.submit_event = threading.Event()
        self.submissions = []
        self.cancel_count = 0
        self.reset_count = 0
        self.reset_event = threading.Event()
        self.reset_gate: Optional[threading.Event] = None
        self.reset_error: Optional[CabinetClientError] = None
        self.submit_error: Optional[Exception] = None
        self.submission_response: Optional[Dict[str, Any]] = None
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
        if self.submission_response is not None:
            self.submit_event.set()
            return dict(self.submission_response)
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
        self.reset_count += 1
        self.reset_event.set()
        if self.reset_gate is not None:
            self.reset_gate.wait(timeout=1.0)
        if self.reset_error is not None:
            raise self.reset_error
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
    manual_joints = tuple(
        SimpleNamespace(name=name, default_position=position)
        for name, position in (
            ("body_arm1", 0.0),
            ("arm1_arm2", -math.pi / 2.0),
            ("arm2_arm3", 0.0),
            ("arm3_arm4", 0.0),
            ("arm4_arm5", 0.0),
            ("arm5_end", 0.0),
            ("end_worklink1", 0.0),
            ("end_worklink2", 0.0),
        )
    )
    server._robot_adapter = SimpleNamespace(
        navigation_frame="map",
        control_navigation_station=lambda _control_id: None,
        manual_joints=manual_joints,
        reset_base_pose=SimpleNamespace(
            frame_id="map",
            x=0.0,
            y=0.0,
            yaw=math.pi / 2.0,
        ),
        reset_joint_tolerance=0.02,
        reset_joint_timeout_sec=0.5,
        reset_joint_duration_sec=0.2,
    )
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

    def test_control_gateway_accepts_non_loopback_bind(self) -> None:
        # FastAPI 已启用 JWT 鉴权，非 loopback 绑定不再被拒绝。
        self.assertFalse(ControlServer._is_loopback_host("0.0.0.0"))
        self.assertFalse(ControlServer._is_loopback_host("192.168.1.100"))
        # 验证 init 不在 loopback 检查处抛异常（会在后面因缺 ROS 失败，
        # 但 loopback 本身不再是阻塞点）。
        try:
            ControlServer(host="0.0.0.0")
        except ValueError as e:
            self.assertNotIn("loopback", str(e))
        except Exception:
            pass  # 缺 ROS 环境是预期的

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

    def test_scene_reset_verifies_cabinet_joints_and_base(self) -> None:
        server, node = _server()
        client = server._cabinet_clients["cabinet_a"]

        accepted = server.submit_reset_task("cabinet_a")
        self.assertRegex(accepted["task_id"], r"^reset_\d+_[a-z0-9]{6}$")
        self.assertTrue(client.reset_event.wait(timeout=1.0))
        self.assertTrue(node.joint_target_event.wait(timeout=1.0))
        goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(1, client.reset_count)
        self.assertEqual(1, node.quiesce_count)
        self.assertAlmostEqual(-math.pi / 2.0, node.joint_targets[0][1])
        self.assertAlmostEqual(0.2, node.joint_target_durations[0])
        self.assertAlmostEqual(0.0, goal["x"])
        self.assertAlmostEqual(0.0, goal["y"])
        self.assertAlmostEqual(math.pi / 2.0, goal["yaw"])

        node.finish_navigation(
            "succeeded",
            pose={"x": 0.0, "y": 0.0, "yaw": math.pi / 2.0},
        )
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("success", task["status"])
        self.assertEqual("verified", task["result"]["components"][
            "robot_joints"
        ]["status"])
        self.assertEqual("map_x_then_y", task["result"]["components"][
            "robot_base"
        ]["route"]["policy"])

    def test_scene_reset_rejects_unknown_cabinet_before_task_acceptance(
        self,
    ) -> None:
        server, node = _server()

        with self.assertRaises(ControlRequestError) as unknown:
            server.submit_reset_task("missing")

        self.assertEqual(404, unknown.exception.status)
        self.assertEqual(0, node.navigation_goal_count)
        self.assertIsNone(server._task_manager.active_task_id)

    def test_scene_reset_requires_idle_navigation_before_acceptance(
        self,
    ) -> None:
        server, node = _server()
        node.state["state"] = "navigating"

        with self.assertRaises(ControlRequestError) as busy:
            server.submit_reset_task("cabinet_a")

        self.assertEqual(409, busy.exception.status)
        self.assertEqual(
            0,
            server._cabinet_clients["cabinet_a"].reset_count,
        )
        self.assertIsNone(server._task_manager.active_task_id)

    def test_scene_reset_cabinet_failure_is_a_terminal_task_failure(
        self,
    ) -> None:
        server, node = _server()
        client = server._cabinet_clients["cabinet_a"]
        client.reset_error = CabinetClientError(
            "physics reset unavailable",
            code="not_ready",
            status=503,
        )

        accepted = server.submit_reset_task("cabinet_a")
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("failed", task["status"])
        self.assertEqual("not_ready", task["failure_code"])
        self.assertEqual([], node.joint_targets)
        self.assertEqual(0, node.navigation_goal_count)

    def test_scene_reset_requires_fresh_joint_state_verification(self) -> None:
        server, node = _server()
        server._robot_adapter.reset_joint_timeout_sec = 0.05
        server._robot_adapter.reset_joint_duration_sec = 0.02
        node.joint_state_available = False

        accepted = server.submit_reset_task("cabinet_a")
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("failed", task["status"])
        self.assertEqual("robot_joint_reset_timeout", task["failure_code"])
        self.assertEqual(0, node.navigation_goal_count)

    def test_scene_reset_rejects_cancel_during_noncancelable_phase(
        self,
    ) -> None:
        server, node = _server()
        client = server._cabinet_clients["cabinet_a"]
        client.reset_gate = threading.Event()

        accepted = server.submit_reset_task("cabinet_a")
        self.assertTrue(client.reset_event.wait(timeout=1.0))
        with self.assertRaises(ControlRequestError) as rejected:
            server.cancel_task(accepted["task_id"])
        self.assertEqual(409, rejected.exception.status)
        self.assertEqual(
            accepted["task_id"],
            server._task_manager.active_task_id,
        )

        client.reset_gate.set()
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.0, "y": 0.0, "yaw": math.pi / 2.0},
        )
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])

    def test_scene_reset_base_homing_can_be_canceled(self) -> None:
        server, node = _server()
        accepted = server.submit_reset_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))

        canceling = server.cancel_task(accepted["task_id"])
        self.assertEqual("canceling", canceling["status"])
        self.assertEqual(1, node.cancel_count)
        node.finish_navigation("canceled")
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("canceled", task["status"])

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
        self.assertEqual(1, node.navigation_goal_count)
        self.assertEqual("map_x_then_y", task["result"]["route"]["policy"])
        self.assertEqual(
            ["x"],
            [leg["axis"] for leg in task["result"]["route"]["legs"]],
        )

    def test_navigation_uses_live_tf_station_when_node_supports_it(self) -> None:
        server, node = _server()
        calls = []

        def live_station(
            cabinet: str,
            cabinet_frame: str,
            spec: NavigationStationSpec,
        ) -> NavigationStation:
            calls.append((cabinet, cabinet_frame, spec))
            # The static inventory station is x=1.0.  This shifted value
            # represents the latest composed map->odom->cabinet transform.
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=2.5,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station

        accepted = server.submit_navigation_task("cabinet_a")
        goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertAlmostEqual(2.5, goal["x"])
        self.assertAlmostEqual(2.5, accepted["request"]["station"]["x"])
        self.assertEqual("cabinet_a", calls[0][0])
        self.assertEqual("cabinet_a_frame", calls[0][1])
        self.assertIsInstance(calls[0][2], NavigationStationSpec)
        node.finish_navigation(
            "succeeded",
            pose={"x": 2.5, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])

    def test_navigation_refreshes_live_station_and_corrects_x_then_y(
        self,
    ) -> None:
        server, node = _server()
        calls = []

        def live_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            calls.append(cabinet)
            if len(calls) == 1:
                x, y = 1.0, 0.0
            else:
                x, y = 1.3, 0.4
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=x,
                y=y,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station
        accepted = server.submit_navigation_task("cabinet_a")

        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        )
        correction_x = node.wait_for_navigation_goal(2)
        self.assertIsNotNone(correction_x)
        assert correction_x is not None
        self.assertAlmostEqual(1.3, correction_x["x"])
        self.assertAlmostEqual(0.0, correction_x["y"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.3, "y": 0.0, "yaw": math.pi},
        )
        correction_y = node.wait_for_navigation_goal(3)
        self.assertIsNotNone(correction_y)
        assert correction_y is not None
        self.assertAlmostEqual(1.3, correction_y["x"])
        self.assertAlmostEqual(0.4, correction_y["y"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.3, "y": 0.4, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(3, len(calls))
        self.assertEqual(1, task["result"]["route"]["correction_count"])
        self.assertEqual(
            ["x", "x", "y"],
            [leg["axis"] for leg in task["result"]["route"]["legs"]],
        )
        self.assertNotIn("correction", task["result"]["route"]["legs"][0])
        self.assertTrue(task["result"]["route"]["legs"][1]["correction"])
        self.assertTrue(task["result"]["route"]["legs"][2]["correction"])
        self.assertAlmostEqual(1.3, task["result"]["station"]["x"])
        self.assertAlmostEqual(0.4, task["result"]["station"]["y"])

    def test_navigation_live_station_refresh_never_falls_back_to_static(
        self,
    ) -> None:
        server, node = _server()
        calls = 0

        def live_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            if calls > 1:
                raise ControlRequestError("cabinet TF unavailable", 503)
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("failed", task["status"])
        self.assertEqual(
            "navigation_station_refresh_failed",
            task["failure_code"],
        )
        self.assertEqual(1, node.navigation_goal_count)
        self.assertIn("TF unavailable", task["message"])

    def test_navigation_corrects_when_latest_handoff_pose_is_outside_margin(
        self,
    ) -> None:
        server, node = _server()
        calls = 0

        def live_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0 if calls == 1 else 1.019,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        # The first goal is valid against its own station (0.11 m error), but
        # the sub-threshold 0.019 m TF shift puts it outside the stricter
        # 0.12 m operation handoff margin.
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.89, "y": 0.0, "yaw": math.pi},
        )
        correction = node.wait_for_navigation_goal(2)
        self.assertIsNotNone(correction)
        assert correction is not None
        self.assertAlmostEqual(1.019, correction["x"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.019, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(1, task["result"]["route"]["correction_count"])
        self.assertAlmostEqual(1.019, task["result"]["station"]["x"])

    def test_navigation_records_station_drift_without_a_redundant_goal(
        self,
    ) -> None:
        server, node = _server()
        calls = 0

        def live_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0 if calls == 1 else 1.1,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        # The localized base and cabinet station moved together.  The station
        # drift is diagnostically meaningful, but the live handoff pose is
        # already exact and must not cause a redundant correction goal.
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.1, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(1, node.navigation_goal_count)
        self.assertEqual(0, task["result"]["route"]["correction_count"])
        self.assertTrue(
            task["result"]["route"]["significant_station_drift"]
        )
        self.assertAlmostEqual(1.1, task["result"]["station"]["x"])

    def test_navigation_fails_on_unsafe_live_localization_jump(self) -> None:
        server, node = _server()
        calls = 0

        def jumping_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0 if calls == 1 else 1.6,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = jumping_station
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("failed", task["status"])
        self.assertEqual("localization_jump", task["failure_code"])
        self.assertEqual(1, node.navigation_goal_count)
        self.assertAlmostEqual(
            0.6,
            task["failure_details"]["station_drift"]["position_m"],
        )

    def test_navigation_live_station_corrections_are_bounded(self) -> None:
        server, node = _server()
        calls = 0

        def moving_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0 + 0.25 * (calls - 1),
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = moving_station
        accepted = server.submit_navigation_task("cabinet_a")
        for goal_count, goal_x in enumerate((1.0, 1.25, 1.5), start=1):
            goal = node.wait_for_navigation_goal(goal_count)
            self.assertIsNotNone(goal)
            assert goal is not None
            self.assertAlmostEqual(goal_x, goal["x"])
            node.finish_navigation(
                "succeeded",
                pose={"x": goal_x, "y": 0.0, "yaw": math.pi},
            )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("failed", task["status"])
        self.assertEqual(
            "navigation_station_unstable",
            task["failure_code"],
        )
        self.assertEqual(3, node.navigation_goal_count)
        self.assertAlmostEqual(1.75, task["result"]["station"]["x"])

    def test_navigation_correction_reuses_original_ros_time_budget(self) -> None:
        server, node = _server()
        node.set_navigation_ros_time(100_000_000_000)
        calls = 0

        def live_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            if calls > 1:
                # Advance the shared active ROS clock beyond the patched task
                # budget while the post-arrival station is being refreshed.
                node.set_navigation_ros_time(100_700_000_000)
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0 if calls == 1 else 1.3,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station
        with patch.object(runner_module, "NAVIGATION_TIMEOUT_SEC", 0.5):
            accepted = server.submit_navigation_task("cabinet_a")
            self.assertIsNotNone(node.wait_for_navigation_goal(1))
            node.finish_navigation(
                "succeeded",
                pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
            )
            task = server._task_manager.wait(
                accepted["task_id"],
                timeout=2.0,
            )

        self.assertEqual("failed", task["status"])
        self.assertEqual("navigation_timeout", task["failure_code"])
        self.assertEqual(1, node.navigation_goal_count)

    def test_navigation_refresh_enforces_budget_without_correction(
        self,
    ) -> None:
        server, node = _server()
        node.set_navigation_ros_time(100_000_000_000)
        calls = 0

        def stable_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            if calls > 1:
                # The refreshed station and base pose are already aligned,
                # but their handoff validation must not bypass the task's
                # shared active-ROS-time budget.
                node.set_navigation_ros_time(100_700_000_000)
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = stable_station
        with patch.object(runner_module, "NAVIGATION_TIMEOUT_SEC", 0.5):
            accepted = server.submit_navigation_task("cabinet_a")
            self.assertIsNotNone(node.wait_for_navigation_goal(1))
            node.finish_navigation(
                "succeeded",
                pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
            )
            task = server._task_manager.wait(
                accepted["task_id"],
                timeout=2.0,
            )

        self.assertEqual("failed", task["status"])
        self.assertEqual("navigation_timeout", task["failure_code"])
        self.assertEqual(1, node.navigation_goal_count)

    def test_navigation_cancel_still_applies_to_live_station_correction(
        self,
    ) -> None:
        server, node = _server()
        calls = 0

        def live_station(
            cabinet: str,
            _cabinet_frame: str,
            _spec: NavigationStationSpec,
        ) -> NavigationStation:
            nonlocal calls
            calls += 1
            return NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=1.0 if calls == 1 else 1.3,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )

        node.navigation_station_from_tf = live_station
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        )
        self.assertIsNotNone(node.wait_for_navigation_goal(2))

        canceling = server.cancel_task(accepted["task_id"])
        self.assertEqual("canceling", canceling["status"])
        node.finish_navigation("canceled")
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("canceled", task["status"])
        self.assertEqual(1, node.cancel_count)

    def test_navigation_keeps_static_fallback_for_tf_less_fake_node(self) -> None:
        server, node = _server()
        self.assertFalse(hasattr(node, "navigation_station_from_tf"))

        accepted = server.submit_navigation_task("cabinet_a")
        goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertAlmostEqual(1.0, goal["x"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])

    def test_navigation_ignores_sub_cell_cross_axis_localization_noise(
        self,
    ) -> None:
        server, node = _server()
        node.state["current_pose"] = {
            "x": 0.0,
            "y": -0.0044,
            "yaw": math.pi / 2.0,
            "frame_id": "map",
        }

        accepted = server.submit_navigation_task("cabinet_a")
        goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(goal)
        assert goal is not None
        # The exact final station remains authoritative; only the pointless
        # corner goal and its 90-degree turn are omitted.
        self.assertAlmostEqual(1.0, goal["x"])
        self.assertAlmostEqual(0.0, goal["y"])
        self.assertAlmostEqual(math.pi, abs(goal["yaw"]))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(1, node.navigation_goal_count)
        self.assertEqual("x", task["result"]["route"]["legs"][0]["axis"])

    def test_navigation_accepts_goal_checker_yaw_settle_margin(self) -> None:
        server, node = _server()
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi - 0.253},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertAlmostEqual(0.253, task["result"]["error"]["yaw_rad"])

    def test_navigation_waits_for_fresh_pose_to_settle_after_nav2_success(
        self,
    ) -> None:
        server, node = _server()
        node.set_navigation_ros_time(100_000_000_000)
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi - 0.266},
        )

        deadline = time.monotonic() + 1.0
        phase = ""
        while time.monotonic() < deadline:
            phase = str(
                server._task_manager.get_task(accepted["task_id"])["phase"]
            )
            if phase == "navigation_settling":
                break
            time.sleep(0.005)
        self.assertEqual("navigation_settling", phase)

        # A newer TF sample inside the active-ROS-time settling window is the
        # authoritative terminal pose, not the first sample after the action.
        node.ros_time_nanoseconds = 101_000_000_000
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi - 0.24},
        )
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("success", task["status"])
        self.assertAlmostEqual(0.24, task["result"]["error"]["yaw_rad"])
        self.assertEqual(1, node.navigation_goal_count)

    def test_navigation_fails_if_pose_remains_outside_settling_tolerance(
        self,
    ) -> None:
        server, node = _server()
        node.set_navigation_ros_time(100_000_000_000)
        accepted = server.submit_navigation_task("cabinet_a")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi - 0.266},
        )

        deadline = time.monotonic() + 1.0
        phase = ""
        while time.monotonic() < deadline:
            phase = str(
                server._task_manager.get_task(accepted["task_id"])["phase"]
            )
            if phase == "navigation_settling":
                break
            time.sleep(0.005)
        self.assertEqual("navigation_settling", phase)

        # Advance beyond the two-second active ROS window while supplying a
        # fresh transform that remains outside the verified yaw tolerance.
        node.ros_time_nanoseconds = 102_200_000_000
        node.finish_navigation(
            "succeeded",
            pose={"x": 1.0, "y": 0.0, "yaw": math.pi - 0.266},
        )
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)

        self.assertEqual("failed", task["status"])
        self.assertEqual("pose_deviation_exceeded", task["failure_code"])
        self.assertAlmostEqual(0.266, task["result"]["error"]["yaw_rad"])
        self.assertEqual(1, node.navigation_goal_count)

    def test_navigation_uses_map_x_then_positive_y_legs(self) -> None:
        server, node = _server()
        node.state["current_pose"] = {
            "x": 0.0,
            "y": 0.0,
            "yaw": math.pi / 2.0,
            "frame_id": "map",
        }
        station = NavigationStationSpec(
            local_anchor=(0.0, 2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )

        accepted = server.submit_navigation_task("cabinet_a", "button_1")
        first_goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(first_goal)
        assert first_goal is not None
        self.assertAlmostEqual(0.5, first_goal["x"])
        self.assertAlmostEqual(0.0, first_goal["y"])
        self.assertAlmostEqual(math.pi / 2.0, first_goal["yaw"])
        self.assertEqual(1, node.navigation_goal_count)
        self.assertEqual("button_1", accepted["request"]["control_id"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.5, "y": 0.0, "yaw": math.pi / 2.0},
        )

        second_goal = node.wait_for_navigation_goal(2)
        self.assertIsNotNone(second_goal)
        assert second_goal is not None
        self.assertAlmostEqual(0.5, second_goal["x"])
        self.assertAlmostEqual(2.0, second_goal["y"])
        self.assertAlmostEqual(math.pi, abs(second_goal["yaw"]))
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.5, "y": 2.0, "yaw": math.pi},
        )
        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(
            ["x", "y"],
            [leg["axis"] for leg in task["result"]["route"]["legs"]],
        )
        self.assertEqual(
            [first_goal, second_goal],
            [leg["target"] for leg in task["result"]["route"]["legs"]],
        )

    def test_navigation_preserves_heading_at_negative_y_corner(self) -> None:
        server, node = _server()
        node.state["current_pose"] = {
            "x": 0.0,
            "y": 0.0,
            "yaw": math.pi / 2.0,
            "frame_id": "map",
        }
        station = NavigationStationSpec(
            local_anchor=(0.0, -2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )

        accepted = server.submit_navigation_task("cabinet_a", "button_1")
        first_goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(first_goal)
        assert first_goal is not None
        self.assertAlmostEqual(0.5, first_goal["x"])
        self.assertAlmostEqual(0.0, first_goal["y"])
        self.assertAlmostEqual(math.pi / 2.0, first_goal["yaw"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.5, "y": 0.0, "yaw": math.pi / 2.0},
        )

        second_goal = node.wait_for_navigation_goal(2)
        self.assertIsNotNone(second_goal)
        assert second_goal is not None
        self.assertAlmostEqual(0.5, second_goal["x"])
        self.assertAlmostEqual(-2.0, second_goal["y"])
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.5, "y": -2.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(2, node.navigation_goal_count)
        self.assertEqual(
            ["x", "y"],
            [leg["axis"] for leg in task["result"]["route"]["legs"]],
        )

    def test_navigation_skips_x_leg_when_already_on_target_x(self) -> None:
        server, node = _server()
        station = NavigationStationSpec(
            local_anchor=(0.0, 2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )
        node.state["current_pose"] = {
            "x": 0.5,
            "y": 0.0,
            "yaw": math.pi / 2.0,
            "frame_id": "map",
        }

        accepted = server.submit_navigation_task("cabinet_a", "button_1")
        only_goal = node.wait_for_navigation_goal(1)
        self.assertIsNotNone(only_goal)
        assert only_goal is not None
        self.assertAlmostEqual(0.5, only_goal["x"])
        self.assertAlmostEqual(2.0, only_goal["y"])
        self.assertAlmostEqual(math.pi, abs(only_goal["yaw"]))
        node.finish_navigation(
            "succeeded",
            pose={"x": 0.5, "y": 2.0, "yaw": math.pi},
        )

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("success", task["status"])
        self.assertEqual(1, node.navigation_goal_count)
        self.assertEqual("y", task["result"]["route"]["legs"][0]["axis"])

    def test_navigation_first_leg_failure_never_submits_second_leg(self) -> None:
        server, node = _server()
        station = NavigationStationSpec(
            local_anchor=(0.0, 2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )

        accepted = server.submit_navigation_task("cabinet_a", "button_1")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))
        node.finish_navigation("failed", message="X-axis route blocked")

        task = server._task_manager.wait(accepted["task_id"], timeout=2.0)
        self.assertEqual("failed", task["status"])
        self.assertEqual("target_unreachable", task["failure_code"])
        self.assertEqual(1, node.navigation_goal_count)

    def test_navigation_retries_transient_nav2_goal_rejection(self) -> None:
        server, node = _server()
        station = NavigationStationSpec(
            local_anchor=(0.0, 2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )
        node.state["current_pose"] = {
            "x": 0.5,
            "y": 0.0,
            "yaw": math.pi / 2.0,
            "frame_id": "map",
        }

        with patch.object(
            runner_module,
            "NAVIGATION_REJECT_RETRY_DELAY_SEC",
            0.001,
        ):
            accepted = server.submit_navigation_task(
                "cabinet_a",
                "button_1",
            )
            self.assertIsNotNone(node.wait_for_navigation_goal(1))
            node.finish_navigation("rejected", message="not active yet")
            retry_goal = node.wait_for_navigation_goal(2)
            self.assertIsNotNone(retry_goal)
            node.finish_navigation(
                "succeeded",
                pose={"x": 0.5, "y": 2.0, "yaw": math.pi},
            )

            task = server._task_manager.wait(
                accepted["task_id"],
                timeout=2.0,
            )

        self.assertEqual("success", task["status"])
        self.assertEqual(2, node.navigation_goal_count)

    def test_navigation_rejection_retries_are_bounded(self) -> None:
        server, node = _server()

        with patch.object(
            runner_module,
            "NAVIGATION_REJECT_RETRY_LIMIT",
            2,
        ), patch.object(
            runner_module,
            "NAVIGATION_REJECT_RETRY_DELAY_SEC",
            0.001,
        ):
            accepted = server.submit_navigation_task("cabinet_a")
            for goal_count in range(1, 4):
                self.assertIsNotNone(
                    node.wait_for_navigation_goal(goal_count)
                )
                node.finish_navigation(
                    "rejected",
                    message="Nav2 lifecycle is inactive",
                )

            task = server._task_manager.wait(
                accepted["task_id"],
                timeout=2.0,
            )

        self.assertEqual("failed", task["status"])
        self.assertEqual("target_unreachable", task["failure_code"])
        self.assertEqual(3, node.navigation_goal_count)
        self.assertEqual(2, task["failure_details"]["goal_retries"])

    def test_control_navigation_rejects_unknown_catalog_id(self) -> None:
        server, _node = _server()

        with self.assertRaises(ControlRequestError) as unknown:
            server.submit_navigation_task("cabinet_a", "missing")

        self.assertEqual(404, unknown.exception.status)

    def test_control_navigation_requires_ready_catalog(self) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_a"]
        client.snapshot_controls = lambda: {
            "catalog_received": False,
            "controls": [],
        }

        with self.assertRaises(ControlRequestError) as not_ready:
            server.submit_navigation_task("cabinet_a", "button_1")

        self.assertEqual(503, not_ready.exception.status)

    def test_control_navigation_rejects_station_outside_live_map(self) -> None:
        server, _node = _server()
        station = NavigationStationSpec(
            local_anchor=(100.0, 0.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=1.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda _control_id: station
        )
        server._live_map_bounds = lambda: MapBounds(-5.0, -5.0, 5.0, 5.0)

        with self.assertRaises(ControlRequestError) as outside:
            server.submit_navigation_task("cabinet_a", "button_1")

        self.assertEqual(400, outside.exception.status)
        self.assertIn("outside the map", str(outside.exception))

    def test_live_tf_station_is_checked_against_live_map_bounds(self) -> None:
        server, node = _server()
        node.navigation_station_from_tf = (
            lambda cabinet, _cabinet_frame, _spec: NavigationStation(
                cabinet=cabinet,
                frame_id="map",
                x=5.0,
                y=0.0,
                z=0.0,
                yaw=math.pi,
            )
        )

        with self.assertRaises(ControlRequestError) as outside:
            server.submit_navigation_task("cabinet_a")

        self.assertEqual(400, outside.exception.status)
        self.assertIn("outside the map", str(outside.exception))
        self.assertIsNone(server._task_manager.active_task_id)

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

    def test_navigation_result_allows_only_bounded_future_tf_skew(self) -> None:
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
            "stamp_ros_nanoseconds": 20_050_000_000,
            "observed_ros_nanoseconds": 20_000_000_000,
            "received_monotonic": 10.0,
            "source": "tf",
        }

        result = ControlServer._navigation_result(
            station,
            {"current_pose": pose},
            2.0,
            minimum_pose_stamp_ros_nanoseconds=19_000_000_000,
            minimum_pose_received_monotonic=9.0,
            observation_monotonic=10.1,
        )
        self.assertEqual(0.0, result["error"]["position_m"])

        pose["stamp_ros_nanoseconds"] = 20_101_000_000
        with self.assertRaises(TaskExecutionError) as inconsistent:
            ControlServer._navigation_result(
                station,
                {"current_pose": pose},
                2.0,
                minimum_pose_stamp_ros_nanoseconds=19_000_000_000,
                minimum_pose_received_monotonic=9.0,
                observation_monotonic=10.1,
            )

        self.assertEqual("final_pose_stale", inconsistent.exception.code)
        self.assertLess(
            inconsistent.exception.details["pose_ros_age_seconds"],
            -runner_module.NAVIGATION_FINAL_POSE_MAX_FUTURE_SKEW_SEC,
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

    def test_navigation_timeout_uses_active_ros_time_not_slow_sim_wall(
        self,
    ) -> None:
        server, node = _server()
        node.set_navigation_ros_time(100_000_000_000)
        wall_offset = [0.0]

        def navigation_monotonic() -> float:
            return time.monotonic() + wall_offset[0]

        navigation_time = SimpleNamespace(
            monotonic=navigation_monotonic,
            sleep=time.sleep,
        )
        with patch.object(
            runner_module,
            "time",
            navigation_time,
        ), patch.object(
            runner_module,
            "NAVIGATION_TIMEOUT_SEC",
            10.0,
        ):
            accepted = server.submit_navigation_task("cabinet_a")
            self.assertIsNotNone(node.wait_for_navigation_goal(1))

            # Simulate a low real-time factor: twenty wall seconds pass while
            # the active ROS clock advances only five seconds.
            wall_offset[0] = 20.0
            node.ros_time_nanoseconds = 104_900_000_000
            node.finish_navigation(
                "succeeded",
                pose={
                    "x": 1.0,
                    "y": 0.0,
                    "yaw": math.pi,
                    "received_monotonic": navigation_monotonic(),
                },
            )

            task = server._task_manager.wait(
                accepted["task_id"],
                timeout=1.0,
            )
            self.assertEqual("success", task["status"])
            self.assertGreater(task["result"]["duration_seconds"], 10.0)

    def test_navigation_ros_clock_stall_uses_wall_watchdog(self) -> None:
        server, node = _server()
        node.set_navigation_ros_time(100_000_000_000)
        with patch.object(
            runner_module,
            "NAVIGATION_TIMEOUT_SEC",
            10.0,
        ), patch.object(
            runner_module,
            "NAVIGATION_CLOCK_STALL_TIMEOUT_SEC",
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
            self.assertEqual("ros", failed["failure_details"]["timeout_clock"])
            self.assertEqual(
                "ros_clock_stalled",
                failed["failure_details"]["timeout_cause"],
            )
            self.assertTrue(failed["reservation_active"])

            node.finish_navigation("canceled")
            deadline = time.monotonic() + 1.0
            while (
                server._task_manager.active_task_id is not None
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertIsNone(server._task_manager.active_task_id)

    def test_axis_navigation_timeout_budget_is_shared_by_both_legs(
        self,
    ) -> None:
        server, node = _server()
        station = NavigationStationSpec(
            local_anchor=(0.0, 2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )
        node.set_navigation_ros_time(100_000_000_000)
        with patch.object(
            runner_module,
            "NAVIGATION_TIMEOUT_SEC",
            10.0,
        ), patch.object(
            runner_module,
            "NAVIGATION_CANCEL_GRACE_SEC",
            0.0,
        ):
            accepted = server.submit_navigation_task(
                "cabinet_a",
                "button_1",
            )
            self.assertIsNotNone(node.wait_for_navigation_goal(1))

            # Spend nine active ROS seconds of the one task-level budget on X.
            node.ros_time_nanoseconds = 109_000_000_000
            node.finish_navigation(
                "succeeded",
                pose={
                    "x": 0.5,
                    "y": 0.0,
                    "yaw": 0.0,
                },
            )
            self.assertIsNotNone(node.wait_for_navigation_goal(2))

            # Crossing ten active seconds on Y must time out immediately; a
            # per-leg timeout would incorrectly grant another ten seconds.
            node.set_navigation_ros_time(111_000_000_000)
            failed = server._task_manager.wait(
                accepted["task_id"],
                timeout=1.0,
            )
            self.assertEqual("failed", failed["status"])
            self.assertEqual("navigation_timeout", failed["failure_code"])
            self.assertTrue(failed["reservation_active"])
            self.assertEqual("ros", failed["failure_details"]["timeout_clock"])
            self.assertEqual(
                "active_time_limit",
                failed["failure_details"]["timeout_cause"],
            )
            self.assertEqual(2, node.navigation_goal_count)
            self.assertGreaterEqual(node.cancel_count, 1)

            node.finish_navigation("canceled")
            deadline = time.monotonic() + 1.0
            while (
                server._task_manager.active_task_id is not None
                and time.monotonic() < deadline
            ):
                time.sleep(0.005)
            self.assertIsNone(server._task_manager.active_task_id)

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

    def test_cancel_between_axis_legs_never_submits_second_goal(self) -> None:
        server, node = _server()
        station = NavigationStationSpec(
            local_anchor=(0.0, 2.0, 0.0),
            outward_axis=(1.0, 0.0, 0.0),
            standoff=0.5,
            base_yaw_offset=0.0,
            frame_id="map",
        )
        server._robot_adapter.control_navigation_station = (
            lambda control_id: station if control_id == "button_1" else None
        )
        accepted = server.submit_navigation_task("cabinet_a", "button_1")
        self.assertIsNotNone(node.wait_for_navigation_goal(1))

        # Hold the same admission lock used for each goal submission.  This
        # creates the exact boundary after X succeeds but before Y can start.
        with server._task_interlock_lock:
            node.finish_navigation(
                "succeeded",
                pose={"x": 0.5, "y": 0.0, "yaw": 0.0},
            )
            deadline = time.monotonic() + 1.0
            phase = ""
            while time.monotonic() < deadline:
                phase = str(
                    server._task_manager.get_task(accepted["task_id"])[
                        "phase"
                    ]
                )
                if phase == "navigation_x_complete":
                    break
                time.sleep(0.005)
            self.assertEqual("navigation_x_complete", phase)
            canceling = server.cancel_task(accepted["task_id"])
            self.assertEqual("canceling", canceling["status"])

        terminal = server._task_manager.wait(
            accepted["task_id"],
            timeout=2.0,
        )
        self.assertEqual("canceled", terminal["status"])
        self.assertEqual(1, node.navigation_goal_count)

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

    def test_synchronous_operation_failure_does_not_wait_for_listener(self) -> None:
        server, _node = _server()
        client = server._cabinet_clients["cabinet_a"]
        client.submission_response = {
            "status": "failed",
            "generation": 1,
            "failure_code": "unreachable",
            "message": "robot workspace cannot reach this control",
            "result": {"unavailable_reason": "workspace limit"},
        }

        accepted = server.submit_operation_task(
            "cabinet_a",
            "button_1",
            "press",
            None,
            None,
            5.0,
        )
        task = server._task_manager.wait(accepted["task_id"], timeout=1.0)

        self.assertEqual("failed", task["status"])
        self.assertEqual("unreachable", task["failure_code"])
        self.assertEqual(
            "robot workspace cannot reach this control",
            task["failure_reason"],
        )
        self.assertEqual("workspace limit", task["result"]["unavailable_reason"])

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
