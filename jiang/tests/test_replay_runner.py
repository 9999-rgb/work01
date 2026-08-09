"""Runner integration tests for recording and replay safety interlocks."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Mapping, Optional
from unittest.mock import patch


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.recording_manager import RecordingError  # noqa: E402
from control_gateway.recording_manager import (  # noqa: E402
    RecordingBackendUnavailableError,
)
from control_gateway.recording_manager import RecordingNotFoundError  # noqa: E402
from control_gateway.recording_manager import RecordingValidationError  # noqa: E402
from control_gateway.ros_node import ControlRequestError  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402
from control_gateway.task_manager import EventHubClosed  # noqa: E402
from control_gateway.task_manager import TaskManager  # noqa: E402


class _RecordingManager:
    def __init__(self) -> None:
        self.recording_state = "idle"
        self.playback_state = "idle"
        self.playback_rate = 1.0
        self.recording_id: Optional[str] = None
        self.playback_id: Optional[str] = None
        self.calls: list[tuple[Any, ...]] = []
        self.task_events: list[Dict[str, Any]] = []
        self.errors: Dict[str, RecordingError] = {}
        self.cancel_count = 0
        self.shutdown_count = 0
        self.start_result: Optional[Dict[str, Any]] = None
        self.stop_result: Optional[Dict[str, Any]] = None

    @property
    def active_mode(self) -> str:
        if self.recording_state == "recording":
            return "recording"
        if self.playback_state in {"playing", "paused"}:
            return "playback"
        return "idle"

    def _raise(self, operation: str) -> None:
        error = self.errors.get(operation)
        if error is not None:
            raise error

    def list_recordings(self) -> list[Dict[str, Any]]:
        self._raise("list_recordings")
        return [{"recording_id": "sample", "status": "completed"}]

    def get_recording(self, recording_id: str) -> Dict[str, Any]:
        self._raise("get_recording")
        return {"recording_id": recording_id, "status": "completed"}

    def timeline(self, recording_id: str) -> Dict[str, Any]:
        self._raise("timeline")
        return {"recording_id": recording_id, "events": []}

    def recording_status(self) -> Dict[str, Any]:
        self._raise("recording_status")
        return {
            "state": self.recording_state,
            "recording_id": self.recording_id,
            "duration_seconds": 1.25,
        }

    def playback_status(self) -> Dict[str, Any]:
        self._raise("playback_status")
        return {
            "state": self.playback_state,
            "recording_id": self.playback_id,
            "rate": self.playback_rate,
            "progress": 0.5,
        }

    def start_recording(
        self,
        recording_id: Optional[str],
        *,
        include_sensors: bool,
    ) -> Dict[str, Any]:
        self._raise("start_recording")
        self.calls.append(("start_recording", recording_id, include_sensors))
        self.recording_state = "recording"
        self.recording_id = recording_id or "generated"
        return dict(self.start_result or self.recording_status())

    def stop_recording(self) -> Dict[str, Any]:
        self._raise("stop_recording")
        self.calls.append(("stop_recording",))
        self.recording_state = "idle"
        self.recording_id = None
        return dict(self.stop_result or self.recording_status())

    def start_playback(
        self,
        recording_id: str,
        *,
        rate: float,
    ) -> Dict[str, Any]:
        self._raise("start_playback")
        self.calls.append(("start_playback", recording_id, rate))
        self.playback_state = "playing"
        self.playback_id = recording_id
        self.playback_rate = rate
        return self.playback_status()

    def pause_playback(self) -> Dict[str, Any]:
        self._raise("pause_playback")
        self.playback_state = "paused"
        return self.playback_status()

    def resume_playback(self) -> Dict[str, Any]:
        self._raise("resume_playback")
        self.playback_state = "playing"
        return self.playback_status()

    def set_playback_rate(self, rate: float) -> Dict[str, Any]:
        self._raise("set_playback_rate")
        self.playback_rate = rate
        return self.playback_status()

    def cancel_playback(self) -> Dict[str, Any]:
        self._raise("cancel_playback")
        self.cancel_count += 1
        self.playback_state = "canceled"
        return self.playback_status()

    def record_task_event(self, event: Mapping[str, Any]) -> Dict[str, Any]:
        self.task_events.append(dict(event))
        return dict(event)

    def shutdown(self) -> Dict[str, Any]:
        self.shutdown_count += 1
        self.recording_state = "idle"
        self.recording_id = None
        self.playback_state = "idle"
        self.playback_id = None
        return {"active_mode": "idle"}


class _TaskReplay:
    def __init__(self) -> None:
        self.state = "idle"
        self.recording_id: Optional[str] = None
        self.start_error: Optional[Exception] = None
        self.start_calls: list[str] = []
        self.cancel_count = 0
        self.shutdown_count = 0

    @property
    def is_active(self) -> bool:
        return self.state in {"running", "canceling"}

    def status(self) -> Dict[str, Any]:
        return {
            "status": self.state,
            "recording_id": self.recording_id,
            "progress": 0.25 if self.is_active else 0.0,
        }

    def start(self, recording_id: str) -> Dict[str, Any]:
        if self.start_error is not None:
            raise self.start_error
        self.start_calls.append(recording_id)
        self.state = "running"
        self.recording_id = recording_id
        return self.status()

    def cancel(self) -> Dict[str, Any]:
        self.cancel_count += 1
        self.state = "canceled"
        return self.status()

    def shutdown(self, timeout: float) -> bool:
        del timeout
        self.shutdown_count += 1
        if self.is_active:
            self.cancel()
        return True


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _Node:
    def __init__(self) -> None:
        self.navigation = {
            "available": True,
            "state": "idle",
            "retiring_goals": 0,
        }
        self.base_targets: list[tuple[float, float]] = []
        self.joint_targets: list[list[float]] = []
        self.mode_requests: list[bool] = []
        self.navigation_cancel_count = 0
        self.takeover_count = 0
        self.emergency_stop_count = 0
        self.quiesce_count = 0
        self.quiesce_settle_seconds = 0.0
        self.task_events: list[Dict[str, Any]] = []
        self.destroyed = False
        self.logger = _Logger()

    def navigation_snapshot(self) -> Dict[str, Any]:
        return dict(self.navigation)

    def map_snapshot(self) -> Dict[str, Any]:
        return {
            "width": 100,
            "height": 100,
            "resolution": 0.1,
            "origin": {"x": -5.0, "y": -5.0, "yaw": 0.0},
        }

    def set_base_target(self, linear: float, angular: float) -> tuple[float, float]:
        self.base_targets.append((linear, angular))
        return linear, angular

    def set_joint_target(self, positions: list[float]) -> list[float]:
        self.joint_targets.append(list(positions))
        return list(positions)

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        self.mode_requests.append(enabled)
        return {"status": "accepted", "enabled": enabled}

    def cancel_navigation(self, allow_idle: bool = False) -> Dict[str, Any]:
        del allow_idle
        self.navigation_cancel_count += 1
        return {"status": "idle"}

    def takeover_navigation(self) -> Dict[str, Any]:
        self.takeover_count += 1
        return {"status": "taking_over"}

    def emergency_stop(self) -> None:
        self.emergency_stop_count += 1

    def quiesce_manual_outputs(self) -> float:
        self.quiesce_count += 1
        return self.quiesce_settle_seconds

    def publish_task_event(self, event: Mapping[str, Any]) -> None:
        self.task_events.append(dict(event))

    def get_logger(self) -> _Logger:
        return self.logger

    def destroy_node(self) -> None:
        self.destroyed = True


class _Client:
    def __init__(self) -> None:
        self.active = False
        self.submit_count = 0
        self.cancel_count = 0
        self.reset_count = 0
        self.destroyed = False

    def snapshot_status(self) -> Dict[str, Any]:
        return {"active": self.active, "state": "operating" if self.active else "idle"}

    def snapshot_controls(self) -> Dict[str, Any]:
        return {
            "catalog_received": True,
            "controls": [{"control_id": "button_1", "operable": True}],
        }

    def submit_operation(self, *args: Any, **kwargs: Any) -> Dict[str, Any]:
        del args, kwargs
        self.submit_count += 1
        return {"status": "accepted"}

    def cancel(self) -> Dict[str, Any]:
        self.cancel_count += 1
        self.active = False
        return {"status": "idle"}

    def reset(self) -> Dict[str, Any]:
        self.reset_count += 1
        return {"status": "reset"}

    def destroy_node(self) -> None:
        self.destroyed = True


class _Station:
    frame_id = "map"

    @staticmethod
    def to_dict() -> Dict[str, Any]:
        return {
            "cabinet": "cabinet_a",
            "frame_id": "map",
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
        }


class _Inventory:
    names = ("cabinet_a",)

    @staticmethod
    def get(name: str) -> SimpleNamespace:
        if name != "cabinet_a":
            raise KeyError(name)
        return SimpleNamespace(name=name)

    @staticmethod
    def station_for(*args: Any, **kwargs: Any) -> _Station:
        del args, kwargs
        return _Station()

    @staticmethod
    def list_cabinets(include_station: bool) -> list[Dict[str, Any]]:
        del include_station
        return [{"name": "cabinet_a"}]


class _Subscription:
    def __init__(self, *events: Mapping[str, Any]) -> None:
        self.events = [dict(event) for event in events]

    def get(self, timeout: float) -> Dict[str, Any]:
        del timeout
        if not self.events:
            raise EventHubClosed("test complete")
        return self.events.pop(0)


class _Executor:
    def __init__(self) -> None:
        self.wake_count = 0
        self.shutdown_count = 0

    def wake(self) -> None:
        self.wake_count += 1

    def shutdown(self, timeout_sec: float) -> bool:
        del timeout_sec
        self.shutdown_count += 1
        return True


class _Context:
    def __init__(self) -> None:
        self.shutdown_count = 0

    def shutdown(self) -> None:
        self.shutdown_count += 1


def _server() -> ControlServer:
    server = object.__new__(ControlServer)
    server._recording_manager = _RecordingManager()
    server._task_replay = _TaskReplay()
    server._replay_internal = threading.local()
    server._task_manager = TaskManager()
    server._task_interlock_lock = threading.RLock()
    server._operation_bindings_lock = threading.RLock()
    server._operation_event_queues = {}
    server._request_condition = threading.Condition()
    server._active_requests = 0
    server._stopping = False
    server._inventory = _Inventory()
    server._robot_adapter = SimpleNamespace(
        navigation_frame="map",
        control_navigation_station=lambda _control_id: None,
    )
    server._node = _Node()
    server._cabinet_clients = {"cabinet_a": _Client()}
    return server


class ReplayRunnerTest(unittest.TestCase):
    def test_status_is_normalized_and_shared_by_all_facade_operations(self) -> None:
        server = _server()
        manager = server._recording_manager
        manager.recording_state = "recording"
        manager.recording_id = "capture"
        manager.playback_state = "paused"
        manager.playback_id = "sample"

        status = server.replay_status()

        self.assertEqual("data_playback", status["mode"])
        self.assertTrue(status["read_only"])
        self.assertEqual("recording", status["recording"]["status"])
        self.assertEqual("paused", status["playback"]["status"])
        self.assertNotIn("state", status["recording"])
        self.assertNotIn("state", status["playback"])
        self.assertEqual("idle", status["task_replay"]["status"])

        self.assertEqual(1, server.recordings()["count"])
        self.assertEqual("sample", server.recording_detail("sample")["recording_id"])
        self.assertEqual([], server.recording_timeline("sample")["events"])

    def test_recording_is_passive_and_does_not_lock_live_controls(self) -> None:
        server = _server()

        started = server.start_recording("capture", True)
        self.assertEqual("recording", started["recording"]["status"])
        self.assertFalse(started["read_only"])
        self.assertEqual((0.1, -0.2), server.publish_cmd_vel(0.1, -0.2))
        self.assertEqual([0.0, 0.5], server.publish_joint_trajectory([0.0, 0.5]))
        self.assertEqual([(0.1, -0.2)], server._node.base_targets)
        self.assertEqual([[0.0, 0.5]], server._node.joint_targets)

        stopped = server.stop_recording()
        self.assertEqual("idle", stopped["recording"]["status"])

    def test_recording_mutations_preserve_their_own_terminal_result(self) -> None:
        server = _server()
        manager = server._recording_manager
        manager.start_result = {
            "state": "failed",
            "recording_id": "capture",
            "error": "recorder exited during startup",
        }

        started = server.start_recording("capture", False)

        self.assertEqual("failed", started["recording"]["status"])
        self.assertEqual(
            "recorder exited during startup",
            started["recording"]["error"],
        )

        manager.start_result = None
        server.start_recording("capture", False)
        manager.stop_result = {
            "state": "failed",
            "recording_id": "capture",
            "error": "metadata finalization failed",
        }

        stopped = server.stop_recording()

        self.assertEqual("failed", stopped["recording"]["status"])
        self.assertEqual(
            "metadata finalization failed",
            stopped["recording"]["error"],
        )

    def test_successful_replay_starts_and_data_controls_share_status(self) -> None:
        server = _server()

        started = server.start_data_playback("sample", 2.0)
        self.assertEqual("data_playback", started["mode"])
        self.assertTrue(started["read_only"])
        self.assertEqual("playing", started["playback"]["status"])
        self.assertEqual(2.0, started["playback"]["rate"])
        self.assertEqual(1, server._node.quiesce_count)
        self.assertEqual(0, server._node.emergency_stop_count)
        self.assertIn(
            ("start_playback", "sample", 2.0),
            server._recording_manager.calls,
        )

        paused = server.pause_data_playback()
        self.assertEqual("paused", paused["playback"]["status"])
        self.assertTrue(paused["read_only"])
        rated = server.set_data_playback_rate(0.5)
        self.assertEqual(0.5, rated["playback"]["rate"])
        resumed = server.resume_data_playback()
        self.assertEqual("playing", resumed["playback"]["status"])
        server.cancel_replay()

        task_started = server.start_task_replay("sample")
        self.assertEqual("task_replay", task_started["mode"])
        self.assertTrue(task_started["read_only"])
        self.assertEqual("running", task_started["task_replay"]["status"])
        self.assertEqual(["sample"], server._task_replay.start_calls)
        self.assertEqual(2, server._node.quiesce_count)

    def test_both_replay_modes_require_global_robot_idle(self) -> None:
        blockers = ("task", "navigation", "cabinet")
        replay_types = ("data", "task")
        for replay_type in replay_types:
            for blocker in blockers:
                with self.subTest(replay_type=replay_type, blocker=blocker):
                    server = _server()
                    if blocker == "task":
                        server._task_manager.create_task("operate", {})
                    elif blocker == "navigation":
                        server._node.navigation["state"] = "navigating"
                    else:
                        server._cabinet_clients["cabinet_a"].active = True

                    with self.assertRaises(ControlRequestError) as raised:
                        if replay_type == "data":
                            server.start_data_playback("sample", 1.0)
                        else:
                            server.start_task_replay("sample")
                    self.assertEqual(409, raised.exception.status)
                    self.assertEqual(0, server._node.quiesce_count)
                    self.assertEqual(0, server._node.emergency_stop_count)
                    self.assertFalse(server._task_replay.start_calls)
                    self.assertFalse(
                        any(
                            call[0] == "start_playback"
                            for call in server._recording_manager.calls
                        )
                    )
                    server._task_manager.shutdown(
                        timeout=0.1,
                        cancel_active=True,
                    )

    def test_replay_waits_for_legacy_manual_trajectory_to_settle(self) -> None:
        server = _server()
        server._node.quiesce_settle_seconds = 0.42

        with patch("control_gateway.runner.time.sleep") as sleeper:
            server.start_data_playback("sample", 1.0)

        sleeper.assert_called_once_with(0.42)
        self.assertEqual(1, server._node.quiesce_count)
        invalid = _server()
        invalid._node.quiesce_settle_seconds = float("nan")
        with self.assertRaises(ControlRequestError) as raised:
            invalid._quiesce_manual_outputs_for_replay()
        self.assertEqual(503, raised.exception.status)

    def test_replay_read_only_blocks_every_live_mutation(self) -> None:
        for replay_mode in ("data_playback", "task_replay"):
            with self.subTest(replay_mode=replay_mode):
                server = _server()
                if replay_mode == "data_playback":
                    server._recording_manager.playback_state = "playing"
                    server._recording_manager.playback_id = "sample"
                else:
                    server._task_replay.state = "running"
                    server._task_replay.recording_id = "sample"

                blocked = (
                    lambda: server.publish_cmd_vel(0.1, 0.0),
                    lambda: server.publish_joint_trajectory([0.0]),
                    lambda: server.set_navigation_mode(True),
                    server.cancel_navigation,
                    server.takeover_navigation,
                    lambda: server.press_cabinet_button("button_1", False),
                    lambda: server.operate_cabinet_control(
                        "button_1", "press", None, None, False
                    ),
                    server.reset_cabinet_controls,
                    server.cancel_cabinet_operation,
                    lambda: server.submit_navigation_task("cabinet_a"),
                    lambda: server.submit_operation_task(
                        "cabinet_a",
                        "button_1",
                        "press",
                        None,
                        None,
                        5.0,
                    ),
                    lambda: server.cancel_task("unknown"),
                )
                for callback in blocked:
                    with self.subTest(callback=callback):
                        with self.assertRaises(ControlRequestError) as raised:
                            callback()
                        self.assertEqual(409, raised.exception.status)
                        self.assertTrue(raised.exception.details["read_only"])
                        self.assertEqual(
                            replay_mode,
                            raised.exception.details["replay_mode"],
                        )

                self.assertFalse(server._node.base_targets)
                self.assertFalse(server._node.joint_targets)
                self.assertFalse(server._node.mode_requests)
                self.assertEqual(0, server._node.navigation_cancel_count)
                self.assertEqual(0, server._node.takeover_count)
                self.assertEqual(
                    0,
                    server._cabinet_clients["cabinet_a"].submit_count,
                )

    def test_internal_task_replay_authorization_reaches_task_workers(self) -> None:
        server = _server()
        server._task_replay.state = "running"
        server._task_replay.recording_id = "sample"
        authorization: list[tuple[str, bool]] = []

        def execute_navigation(context: Any, station: Mapping[str, Any]) -> Dict[str, Any]:
            del context, station
            with server._task_interlock_scope():
                authorization.append(
                    ("navigate", server._replay_internal_authorized())
                )
            return {"replayed": True}

        def execute_operation(context: Any, *args: Any) -> Dict[str, Any]:
            del context, args
            with server._task_interlock_scope():
                authorization.append(
                    ("operate", server._replay_internal_authorized())
                )
            return {"replayed": True}

        server._execute_navigation_task = execute_navigation
        server._execute_operation_task = execute_operation

        navigation = server._replay_submit_navigation("cabinet_a", None)
        navigation_result = server._task_manager.wait(
            navigation["task_id"],
            timeout=1.0,
        )
        operation = server._replay_submit_operation(
            "cabinet_a",
            "button_1",
            "press",
            None,
            None,
            5.0,
        )
        operation_result = server._task_manager.wait(
            operation["task_id"],
            timeout=1.0,
        )

        self.assertEqual("success", navigation_result["status"])
        self.assertEqual("success", operation_result["status"])
        self.assertEqual(
            [("navigate", True), ("operate", True)],
            authorization,
        )

    def test_cancel_replay_routes_to_the_active_owner(self) -> None:
        server = _server()
        server._recording_manager.playback_state = "playing"
        server._recording_manager.playback_id = "sample"

        data_result = server.cancel_replay()
        self.assertEqual(1, server._recording_manager.cancel_count)
        self.assertEqual(0, server._task_replay.cancel_count)
        self.assertEqual("idle", data_result["mode"])
        self.assertFalse(data_result["read_only"])

        server._task_replay.state = "running"
        server._task_replay.recording_id = "sample"
        task_result = server.cancel_replay()
        self.assertEqual(1, server._recording_manager.cancel_count)
        self.assertEqual(1, server._task_replay.cancel_count)
        self.assertEqual("idle", task_result["mode"])
        self.assertFalse(task_result["read_only"])

    def test_recording_errors_are_mapped_to_control_request_errors(self) -> None:
        server = _server()
        manager = server._recording_manager
        manager.errors["list_recordings"] = RecordingValidationError(
            "manifest invalid",
            code="manifest_invalid",
        )
        with self.assertRaises(ControlRequestError) as invalid:
            server.recordings()
        self.assertEqual(400, invalid.exception.status)
        self.assertEqual("manifest_invalid", invalid.exception.details["code"])

        manager.errors.clear()
        manager.errors["start_playback"] = RecordingBackendUnavailableError(
            "rosbag2 unavailable",
            code="player_start_failed",
        )
        with self.assertRaises(ControlRequestError) as unavailable:
            server.start_data_playback("sample", 1.0)
        self.assertEqual(503, unavailable.exception.status)
        self.assertEqual(
            "player_start_failed",
            unavailable.exception.details["code"],
        )

        manager.errors.clear()
        server._task_replay.start_error = RecordingNotFoundError("missing")
        with self.assertRaises(ControlRequestError) as missing:
            server.start_task_replay("missing")
        self.assertEqual(404, missing.exception.status)
        self.assertEqual(
            "recording_not_found",
            missing.exception.details["code"],
        )

        server._task_replay.start_error = None
        manager.errors["playback_status"] = RecordingValidationError(
            "playback state unavailable",
            code="playback_state_invalid",
        )
        with self.assertRaises(ControlRequestError) as state_error:
            server.start_task_replay("sample")
        self.assertEqual(400, state_error.exception.status)
        self.assertEqual(
            "playback_state_invalid",
            state_error.exception.details["code"],
        )

    def test_unknown_playback_state_fails_closed_for_live_writes(self) -> None:
        server = _server()
        server._recording_manager.errors["playback_status"] = (
            RecordingValidationError(
                "playback state unavailable",
                code="playback_state_invalid",
            )
        )

        with self.assertRaises(ControlRequestError):
            server.publish_cmd_vel(0.2, 0.0)
        self.assertFalse(server._node.base_targets)

    def test_task_events_are_written_to_timeline_before_ros_publication(self) -> None:
        server = _server()
        event = {
            "id": "12",
            "event": "task_completed",
            "timestamp": 1_700_000_000.0,
            "data": {"task_id": "operate_1", "outcome": "success"},
        }
        server._event_bridge_subscription = _Subscription(event)

        server._bridge_task_events()

        self.assertEqual([event], server._recording_manager.task_events)
        self.assertEqual([event], server._node.task_events)

    def test_shutdown_stops_replays_before_ros_teardown(self) -> None:
        server = _server()
        server._task_replay.state = "running"
        server._recording_manager.playback_state = "playing"
        server._recording_manager.playback_id = "sample"
        server._shutdown_lock = threading.Lock()
        server._ros_teardown_completed = False
        server._http_server = None
        server._http_thread = None
        server._event_bridge_thread = None
        server._executor_stop_event = threading.Event()
        server._executor_thread = None
        server._executor = _Executor()
        server._context = _Context()

        server.stop()

        self.assertEqual(2, server._task_replay.shutdown_count)
        self.assertEqual(1, server._recording_manager.cancel_count)
        self.assertEqual(1, server._recording_manager.shutdown_count)
        self.assertTrue(server._shutdown_report["task_replay_stopped"])
        self.assertTrue(server._shutdown_report["recording_manager_stopped"])
        self.assertTrue(server._shutdown_report["ros_teardown_completed"])
        self.assertTrue(server._node.destroyed)
        self.assertTrue(server._cabinet_clients["cabinet_a"].destroyed)
        self.assertEqual(1, server._context.shutdown_count)

    def test_shutdown_rechecks_replay_owner_after_admitted_requests_drain(
        self,
    ) -> None:
        server = _server()
        server._shutdown_lock = threading.Lock()
        server._ros_teardown_completed = False
        server._http_server = None
        server._http_thread = None
        server._event_bridge_thread = None
        server._executor_stop_event = threading.Event()
        server._executor_thread = None
        server._executor = _Executor()
        server._context = _Context()
        server._active_requests = 1
        first_shutdown = threading.Event()
        original_shutdown = server._recording_manager.shutdown

        def observed_shutdown() -> Dict[str, Any]:
            result = original_shutdown()
            first_shutdown.set()
            return result

        server._recording_manager.shutdown = observed_shutdown

        def finish_admitted_request() -> None:
            self.assertTrue(first_shutdown.wait(timeout=1.0))
            server._recording_manager.start_playback("sample", rate=1.0)
            with server._request_condition:
                server._active_requests = 0
                server._request_condition.notify_all()

        request_thread = threading.Thread(target=finish_admitted_request)
        request_thread.start()

        server.stop()
        request_thread.join(timeout=1.0)

        self.assertFalse(request_thread.is_alive())
        self.assertEqual(2, server._recording_manager.shutdown_count)
        self.assertEqual("idle", server._recording_manager.active_mode)
        self.assertTrue(server._shutdown_report["recording_manager_stopped"])
        self.assertTrue(server._shutdown_report["ros_teardown_completed"])

    def test_shutdown_uses_final_owner_state_instead_of_stale_timeouts(
        self,
    ) -> None:
        server = _server()
        server._task_replay.state = "running"
        server._shutdown_lock = threading.Lock()
        server._ros_teardown_completed = False
        server._http_server = None
        server._http_thread = None
        server._event_bridge_thread = None
        server._executor_stop_event = threading.Event()
        server._executor_thread = None
        server._executor = _Executor()
        server._context = _Context()
        replay_shutdown_calls = 0
        task_shutdown_calls = 0
        original_task_shutdown = server._task_manager.shutdown

        def staged_replay_shutdown(timeout: float) -> bool:
            nonlocal replay_shutdown_calls
            del timeout
            replay_shutdown_calls += 1
            server._task_replay.state = "canceled"
            return replay_shutdown_calls > 1

        def staged_task_shutdown(**kwargs: Any) -> Dict[str, Any]:
            nonlocal task_shutdown_calls
            task_shutdown_calls += 1
            result = original_task_shutdown(**kwargs)
            if task_shutdown_calls == 1:
                return {
                    **result,
                    "workers_stopped": False,
                    "pending_worker_task_ids": ["late_worker"],
                }
            return result

        server._task_replay.shutdown = staged_replay_shutdown
        server._task_manager.shutdown = staged_task_shutdown

        server.stop()

        self.assertEqual(2, replay_shutdown_calls)
        self.assertEqual(2, task_shutdown_calls)
        self.assertTrue(server._shutdown_report["task_replay_stopped"])
        self.assertTrue(
            server._shutdown_report["task_manager"]["workers_stopped"]
        )
        self.assertTrue(server._shutdown_report["ros_teardown_completed"])


if __name__ == "__main__":
    unittest.main()
