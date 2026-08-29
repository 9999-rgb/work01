"""Pure fake-process tests for safe rosbag recording and replay."""

from __future__ import annotations

import hashlib
import signal
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Optional


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.recording_manager import PlaybackSafetyError  # noqa: E402
from control_gateway.recording_manager import (  # noqa: E402
    RecordingBackendUnavailableError,
)
from control_gateway.recording_manager import RecordingConflictError  # noqa: E402
from control_gateway.recording_manager import RecordingManager  # noqa: E402
from control_gateway.recording_manager import RecordingNotFoundError  # noqa: E402
from control_gateway.recording_manager import RecordingValidationError  # noqa: E402


class _Clock:
    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


class _FakeProcess:
    _next_pid = 50_000

    def __init__(self, command: list[str], kwargs: dict[str, Any]) -> None:
        self.command = command
        self.kwargs = kwargs
        self.returncode: Optional[int] = None
        self.signals: list[int] = []
        self.pid = self._next_pid
        type(self)._next_pid += 1

    def poll(self) -> Optional[int]:
        return self.returncode

    def wait(self, timeout: Optional[float] = None) -> int:
        if self.returncode is None:
            raise subprocess.TimeoutExpired(self.command, timeout)
        return self.returncode


class _ProcessFactory:
    def __init__(self) -> None:
        self.processes: list[_FakeProcess] = []
        self.fail_with: Optional[Exception] = None

    def __call__(self, command: list[str], **kwargs: Any) -> _FakeProcess:
        if self.fail_with is not None:
            raise self.fail_with
        process = _FakeProcess(command, kwargs)
        self.processes.append(process)
        return process


class _SignalController:
    def __init__(self) -> None:
        self.terminate = True
        self.exit_code = 0

    def __call__(self, process: _FakeProcess, signal_number: int) -> None:
        process.signals.append(signal_number)
        if (
            self.terminate
            and signal_number in {signal.SIGINT, signal.SIGTERM, signal.SIGKILL}
        ):
            process.returncode = self.exit_code


class _CloseFailure:
    def __init__(self, delegate: Any) -> None:
        self.delegate = delegate
        self.attempted = False

    def close(self) -> None:
        self.attempted = True
        self.delegate.close()
        raise OSError("injected close failure")


def _signal_process(process: _FakeProcess, signal_number: int) -> None:
    process.signals.append(signal_number)
    if signal_number in {signal.SIGINT, signal.SIGTERM, signal.SIGKILL}:
        process.returncode = 0


def _raising_signal_only(signal_number: int):
    """Signal sender that raises ProcessLookupError for one signal only.

    Everything else delegates to ``_signal_process`` so recorder stop (SIGINT /
    SIGTERM / SIGKILL) and playback lifecycle keep working in the test.
    """

    def sender(process: _FakeProcess, sig: int) -> None:
        if sig == signal_number:
            raise ProcessLookupError(3, "No such process")
        _signal_process(process, sig)

    return sender


def _no_process_group(_process: _FakeProcess) -> bool:
    return False


def _metadata(topics: list[str], duration_seconds: float = 12.5) -> str:
    topic_blocks = "\n".join(
        """    - topic_metadata:
        name: {topic}
        type: std_msgs/msg/String
        serialization_format: cdr
      message_count: 1""".format(topic=topic)
        for topic in topics
    )
    duration_ns = int(duration_seconds * 1_000_000_000)
    return f"""rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration:
    nanoseconds: {duration_ns}
  starting_time:
    nanoseconds_since_epoch: 1700000000000000000
  message_count: {len(topics)}
  topics_with_message_count:
{topic_blocks}
  relative_file_paths:
    - data_0.db3
"""


def _flow_metadata(topics: list[str], duration_seconds: float = 12.5) -> str:
    topic_entries = ", ".join(
        "{topic_metadata: {name: "
        f"{topic}, type: std_msgs/msg/String, serialization_format: cdr"
        "}, message_count: 1}"
        for topic in topics
    )
    duration_ns = int(duration_seconds * 1_000_000_000)
    return f"""rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration: {{nanoseconds: {duration_ns}}}
  topics_with_message_count: [{topic_entries}]
  relative_file_paths: [data_0.db3]
"""


class RecordingManagerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.workspace = Path(self.temporary.name)
        self.root = self.workspace / "recordings"
        self.config = self.workspace / "robot.yaml"
        self.config.write_text("robot: xczs\n", encoding="utf-8")
        self.wall_clock = _Clock(1_700_000_000.0)
        self.monotonic = _Clock(100.0)
        self.factory = _ProcessFactory()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _manager(
        self,
        *,
        record_topics: tuple[str, ...] = ("/map",),
        signal_sender: Any = _signal_process,
        process_group_probe: Any = _no_process_group,
    ) -> RecordingManager:
        return RecordingManager(
            self.root,
            self.workspace,
            (self.config,),
            {"cabinets": [{"name": "cabinet_a"}]},
            record_topics,
            popen_factory=self.factory,
            clock=self.wall_clock,
            monotonic_clock=self.monotonic,
            signal_sender=signal_sender,
            process_group_probe=process_group_probe,
            git_commit="0123456789abcdef",
        )

    def _write_metadata(
        self,
        recording_id: str,
        topics: Optional[list[str]] = None,
        *,
        duration_seconds: float = 12.5,
    ) -> None:
        bag = self.root / recording_id / "bag"
        bag.mkdir(parents=True, exist_ok=True)
        (bag / "data_0.db3").touch()
        (bag / "metadata.yaml").write_text(
            _metadata(
                topics or ["/tf", "/clock", "/xczs/task/example/result"],
                duration_seconds,
            ),
            encoding="utf-8",
        )

    def _completed_recording(
        self,
        manager: RecordingManager,
        recording_id: str = "run_001",
        topics: Optional[list[str]] = None,
    ) -> None:
        manager.start_recording(recording_id)
        self._write_metadata(recording_id, topics)
        status = manager.stop_recording({"outcome": "success"})
        self.assertEqual("completed", status["state"])

    def test_start_recording_uses_safe_regex_and_writes_manifest(self) -> None:
        manager = self._manager()
        status = manager.start_recording(
            "run_001",
            include_sensors=("lidar",),
        )

        self.assertEqual("recording", manager.active_mode)
        self.assertEqual("recording", status["state"])
        command = self.factory.processes[-1].command
        self.assertEqual(["ros2", "bag", "record"], command[:3])
        regex = command[command.index("--regex") + 1]
        self.assertIn(r"/xczs/task/[^/]+/(?:progress|result)", regex)
        self.assertIn(r"/xczs/cabinet/[^/]+", regex)
        self.assertIn("/map", regex)
        self.assertIn("/scan", regex)
        self.assertIn("/xczs/lidar/scan", regex)
        self.assertNotIn("cmd_vel", regex)
        self.assertNotIn("joint_trajectory", regex)
        self.assertTrue(self.factory.processes[-1].kwargs["start_new_session"])

        manifest = manager.get_recording("run_001")
        self.assertEqual(1, manager.load_scenario("run_001")["schema_version"])
        expected_hash = hashlib.sha256(self.config.read_bytes()).hexdigest()
        self.assertEqual("0123456789abcdef", manifest["git_commit"])
        self.assertEqual(expected_hash, manifest["config_sha256"]["robot.yaml"])
        self.assertEqual("cabinet_a", manifest["inventory"]["cabinets"][0]["name"])
        self.assertEqual(["lidar"], manifest["sensor_groups"])
        with self.assertRaises(RecordingConflictError):
            manager.start_playback("run_001")

        self._write_metadata("run_001", ["/tf", "/clock"])
        self.wall_clock.advance(13.0)
        self.monotonic.advance(13.0)
        stopped = manager.stop_recording()
        self.assertEqual("completed", stopped["state"])
        self.assertEqual(12.5, stopped["duration_seconds"])
        self.assertEqual(["/tf", "/clock"], stopped["topics"])
        self.assertEqual("idle", manager.active_mode)

        manager.start_recording("run_camera", include_sensors=("camera",))
        camera_regex = self.factory.processes[-1].command[-1]
        self.assertIn("/xczs/camera/arm_camera/image_raw", camera_regex)
        self.assertIn("/xczs/camera/arm_camera/camera_info", camera_regex)
        self._write_metadata("run_camera", ["/tf"])
        manager.stop_recording()

    def test_task_timeline_extracts_ordered_replay_scenario(self) -> None:
        manager = self._manager()
        manager.start_recording("scenario_1")
        reset = {
            "task_id": "reset_0",
            "type": "reset",
            "request": {"cabinet": "cabinet_a"},
        }
        navigate = {
            "task_id": "navigate_1",
            "type": "navigate",
            "request": {"cabinet": "cabinet_a", "control_id": "box_10_button_1"},
        }
        operation = {
            "task_id": "operate_2",
            "type": "operate",
            "request": {
                "cabinet": "cabinet_a",
                "control_id": "box_10_button_1",
                "command": "press",
                "force": 5.0,
            },
        }
        manager.record_task_event(
            "task_accepted",
            {"task_id": "reset_0", "task": reset},
            timestamp=1_700_000_000.1,
        )
        manager.record_task_event(
            "task_completed",
            {
                "task_id": "reset_0",
                "outcome": "success",
                "task": {
                    **reset,
                    "result": {
                        "cabinet": "cabinet_a",
                        "scene_reset": True,
                    },
                },
            },
            timestamp=1_700_000_000.2,
        )
        manager.record_task_event(
            "task_accepted",
            {"task_id": "navigate_1", "task": navigate},
            timestamp=1_700_000_001.0,
        )
        manager.record_task_event(
            {
                "event": "task_progress",
                "timestamp": 1_700_000_002.0,
                "data": {"task_id": "navigate_1", "progress": 0.5},
            }
        )
        manager.record_task_event(
            "task_completed",
            {
                "task_id": "navigate_1",
                "outcome": "success",
                "task": {**navigate, "result": {"distance_error": 0.01}},
            },
            timestamp=1_700_000_003.0,
        )
        manager.record_task_event(
            "task_accepted",
            {"task_id": "operate_2", "task": operation},
            timestamp=1_700_000_004.0,
        )
        manager.record_task_event(
            "task_completed",
            {
                "task_id": "operate_2",
                "outcome": "failed",
                "task": {
                    **operation,
                    "result": {"estimated_force": 4.0},
                    "failure_code": "insufficient_force",
                    "failure_reason": "力度不足",
                },
            },
            timestamp=1_700_000_005.0,
        )
        manager.record_task_event(
            "task_accepted",
            {
                "task_id": "manual_3",
                "task": {
                    "task_id": "manual_3",
                    "type": "manual",
                    "request": {"linear": 1.0},
                },
            },
            timestamp=1_700_000_006.0,
        )

        page = manager.timeline("scenario_1", offset=1, limit=2)
        self.assertEqual(2, len(page["events"]))
        self.assertEqual(3, page["next_offset"])
        self.assertTrue(page["has_more"])
        scenario = manager.load_scenario("scenario_1")
        self.assertEqual(2, scenario["schema_version"])
        self.assertEqual(
            ["reset", "navigate", "operate"],
            [step["type"] for step in scenario["steps"]],
        )
        self.assertEqual("reset_0", scenario["steps"][0]["recorded_task_id"])
        self.assertEqual("success", scenario["steps"][0]["recorded_outcome"])
        self.assertEqual(
            {"cabinet": "cabinet_a"},
            scenario["steps"][0]["request"],
        )
        self.assertEqual("navigate_1", scenario["steps"][1]["recorded_task_id"])
        self.assertEqual("success", scenario["steps"][1]["recorded_outcome"])
        self.assertEqual(
            "insufficient_force",
            scenario["steps"][2]["recorded_failure_code"],
        )
        self.assertEqual(
            "力度不足",
            scenario["steps"][2]["recorded_failure_reason"],
        )
        self.assertEqual(5.0, scenario["steps"][2]["request"]["force"])

        self._write_metadata("scenario_1")
        manager.stop_recording()
        loaded_again = manager.load_scenario("scenario_1")
        self.assertEqual(scenario["steps"], loaded_again["steps"])

    def test_timeline_paginates_with_exact_total_and_has_more(self) -> None:
        """Pages fill via break (not EOF scan); total stays exact via the
        writer-maintained count cache; has_more flips to False exactly at EOF."""
        manager = self._manager()
        manager.start_recording("page_001")
        for index in range(6):
            manager.record_task_event(
                "task_progress",
                {"step": index},
                timestamp=1_700_000_000.0 + index,
            )

        pages = [
            manager.timeline("page_001", offset=0, limit=2),
            manager.timeline("page_001", offset=2, limit=2),
            manager.timeline("page_001", offset=4, limit=2),
        ]
        self.assertEqual([2, 2, 2], [len(p["events"]) for p in pages])
        self.assertEqual([2, 4, 6], [p["next_offset"] for p in pages])
        self.assertEqual([True, True, False], [p["has_more"] for p in pages])
        self.assertEqual([6, 6, 6], [p["total"] for p in pages])
        # The cache tracks the exact line count under the writer lock.
        self.assertEqual(6, manager._timeline_totals["page_001"])

        # A page past the end reports empty but keeps the exact total.
        past_end = manager.timeline("page_001", offset=6, limit=2)
        self.assertEqual([], past_end["events"])
        self.assertEqual(False, past_end["has_more"])
        self.assertEqual(6, past_end["total"])

        # Appending more events advances the cached total on the next read.
        for index in range(6, 8):
            manager.record_task_event(
                "task_progress",
                {"step": index},
                timestamp=1_700_000_000.0 + index,
            )
        tail = manager.timeline("page_001", offset=6, limit=2)
        self.assertEqual(2, len(tail["events"]))
        self.assertEqual(False, tail["has_more"])
        self.assertEqual(8, tail["total"])
        self.assertEqual(8, manager._timeline_totals["page_001"])

    def test_timeline_counts_uncached_recording_in_one_pass(self) -> None:
        """A recording written by another manager (empty count cache) still
        reports an exact total/has_more: the page-break fallback does one cheap
        line count to EOF and caches it, so later pages stay O(1)."""
        writer = self._manager()
        writer.start_recording("ext_001")
        for index in range(6):
            writer.record_task_event(
                "task_progress",
                {"step": index},
                timestamp=1_700_000_000.0 + index,
            )

        # Fresh instance over the same recordings root: no writer cache.
        reader = self._manager()
        self.assertEqual({}, reader._timeline_totals)
        first = reader.timeline("ext_001", offset=0, limit=2)
        self.assertEqual(2, len(first["events"]))
        self.assertEqual(True, first["has_more"])
        self.assertEqual(6, first["total"])
        # The one-pass count was cached for the fresh instance.
        self.assertEqual(6, reader._timeline_totals["ext_001"])

        middle = reader.timeline("ext_001", offset=2, limit=2)
        self.assertEqual(True, middle["has_more"])
        self.assertEqual(6, middle["total"])
        last = reader.timeline("ext_001", offset=4, limit=2)
        self.assertEqual(False, last["has_more"])
        self.assertEqual(6, last["total"])

    def test_ids_root_confinement_and_read_only_topic_validation(self) -> None:
        manager = self._manager()
        for invalid in ("../escape", "/absolute", "UPPER", "", "a" * 65):
            with self.subTest(invalid=invalid):
                with self.assertRaises(RecordingValidationError):
                    manager.start_recording(invalid)
        with self.assertRaises(RecordingNotFoundError):
            manager.get_recording("missing")

        outside = self.workspace / "outside"
        outside.mkdir()
        self.root.mkdir(exist_ok=True)
        (self.root / "linked").symlink_to(outside, target_is_directory=True)
        with self.assertRaises(RecordingValidationError):
            manager.get_recording("linked")

        for dangerous in (
            "/cmd_vel",
            "/xczs/manual_cmd_vel",
            "/xczs/joint_trajectory",
            "/navigate_to_pose/_action/goal",
        ):
            with self.subTest(topic=dangerous):
                with self.assertRaises(PlaybackSafetyError):
                    self._manager(record_topics=(dangerous,))
        with self.assertRaises(RecordingValidationError):
            self._manager(record_topics=("relative_topic",))

        with self.assertRaises(RecordingValidationError):
            manager.start_recording(
                "invalid_sensor",
                include_sensors=("unknown",),
            )
        self.assertFalse((self.root / "invalid_sensor").exists())

    def test_backend_start_error_is_typed_and_manifest_is_retained(self) -> None:
        manager = self._manager()
        self.factory.fail_with = FileNotFoundError("ros2 not installed")
        with self.assertRaises(RecordingBackendUnavailableError) as raised:
            manager.start_recording("failed_start")
        self.assertEqual(503, raised.exception.status)
        self.assertEqual("recorder_start_failed", raised.exception.code)
        retained = manager.get_recording("failed_start")
        self.assertEqual("failed", retained["status"])
        self.assertIn("ros2 not installed", retained["result"]["reason"])
        self.assertEqual("idle", manager.active_mode)

    def test_failed_backend_start_reclaims_same_id_on_retry(self) -> None:
        manager = self._manager()
        self.factory.fail_with = FileNotFoundError("ros2 not installed")
        with self.assertRaises(RecordingBackendUnavailableError):
            manager.start_recording("mission_1")
        retained = manager.get_recording("mission_1")
        self.assertEqual("failed", retained["status"])

        # A transient backend failure must not burn the explicit id forever:
        # a retry reclaims the dead (failed) slot instead of returning 409.
        self.factory.fail_with = None
        started = manager.start_recording("mission_1")
        self.assertEqual("recording", started["state"])
        self.assertEqual("recording", manager.active_mode)
        manifest = manager.get_recording("mission_1")
        self.assertEqual("recording", manifest["status"])
        self.assertIsNone(manifest["result"])

    def test_completed_recording_id_is_not_reclaimable(self) -> None:
        manager = self._manager()
        self._completed_recording(manager, recording_id="done_1")
        with self.assertRaises(RecordingConflictError):
            manager.start_recording("done_1")

    def test_pause_playback_oserror_is_a_graceful_conflict(self) -> None:
        manager = self._manager(
            signal_sender=_raising_signal_only(signal.SIGSTOP)
        )
        self._completed_recording(manager)
        manager.start_playback("run_001")
        with self.assertRaises(RecordingConflictError) as raised:
            manager.pause_playback()
        self.assertIn("Playback ended", str(raised.exception))

    def test_resume_playback_oserror_is_a_graceful_conflict(self) -> None:
        manager = self._manager(
            signal_sender=_raising_signal_only(signal.SIGCONT)
        )
        self._completed_recording(manager)
        manager.start_playback("run_001")
        manager.pause_playback()
        with self.assertRaises(RecordingConflictError) as raised:
            manager.resume_playback()
        self.assertIn("Playback ended", str(raised.exception))

    def test_playback_remaps_every_metadata_topic_and_tracks_controls(self) -> None:
        manager = self._manager()
        topics = ["/tf", "/tf_static", "/clock", "/amcl_pose"]
        self._completed_recording(manager, topics=topics)

        started = manager.start_playback("run_001", rate=2.0, start_offset=1.0)
        self.assertEqual("playing", started["state"])
        self.assertEqual("playback", manager.active_mode)
        command = self.factory.processes[-1].command
        remap_index = command.index("--remap")
        self.assertEqual(
            [
                f"{topic}:=/xczs/replay/run_001{topic}"
                for topic in topics
            ],
            command[remap_index + 1:],
        )
        self.assertIn("/clock:=/xczs/replay/run_001/clock", command)

        self.monotonic.advance(2.0)
        playing = manager.playback_status()
        self.assertEqual(5.0, playing["position_seconds"])
        paused = manager.pause_playback()
        self.assertEqual("paused", paused["state"])
        self.assertEqual(signal.SIGSTOP, self.factory.processes[-1].signals[-1])
        self.monotonic.advance(10.0)
        self.assertEqual(5.0, manager.playback_status()["position_seconds"])

        resumed = manager.resume_playback()
        self.assertEqual("playing", resumed["state"])
        self.assertEqual(signal.SIGCONT, self.factory.processes[-1].signals[-1])
        self.monotonic.advance(1.0)
        changed = manager.set_playback_rate(0.5)
        self.assertEqual(0.5, changed["rate"])
        restarted_command = self.factory.processes[-1].command
        offset = restarted_command[restarted_command.index("--start-offset") + 1]
        self.assertEqual("7", offset)
        canceled = manager.cancel_playback()
        self.assertEqual("canceled", canceled["state"])
        self.assertEqual("idle", manager.active_mode)

    def test_flow_style_metadata_is_fully_filtered_and_remapped(self) -> None:
        manager = self._manager()
        manager.start_recording("flow_style")
        bag = self.root / "flow_style" / "bag"
        bag.mkdir(parents=True, exist_ok=True)
        (bag / "data_0.db3").touch()
        topics = ["/tf", "/clock", "/joint_states"]
        (bag / "metadata.yaml").write_text(
            _flow_metadata(topics),
            encoding="utf-8",
        )
        stopped = manager.stop_recording()
        self.assertEqual("completed", stopped["state"])
        self.assertEqual(topics, stopped["topics"])

        manager.start_playback("flow_style")
        command = self.factory.processes[-1].command
        topics_index = command.index("--topics")
        remap_index = command.index("--remap")
        self.assertEqual(topics, command[topics_index + 1:remap_index])
        self.assertEqual(
            [
                f"{topic}:=/xczs/replay/flow_style{topic}"
                for topic in topics
            ],
            command[remap_index + 1:],
        )
        manager.cancel_playback()

        (bag / "metadata.yaml").write_text(
            _flow_metadata(["/tf", "/cmd_vel"]),
            encoding="utf-8",
        )
        process_count = len(self.factory.processes)
        with self.assertRaises(PlaybackSafetyError) as unsafe:
            manager.start_playback("flow_style")
        self.assertEqual("/cmd_vel", unsafe.exception.topic)
        self.assertEqual(process_count, len(self.factory.processes))

    def test_metadata_rejects_unparsed_or_ambiguous_topic_entries(self) -> None:
        manager = self._manager()
        self._completed_recording(manager)
        metadata = self.root / "run_001" / "bag" / "metadata.yaml"
        metadata.write_text(
            """rosbag2_bagfile_information:
  duration: {nanoseconds: 1000000000}
  topics_with_message_count:
    - topic_metadata: {type: std_msgs/msg/String, serialization_format: cdr}
      message_count: 1
""",
            encoding="utf-8",
        )
        with self.assertRaises(RecordingValidationError) as missing_name:
            manager.start_playback("run_001")
        self.assertEqual("metadata_corrupted", missing_name.exception.code)

        metadata.write_text(
            """rosbag2_bagfile_information:
  duration: {nanoseconds: 1000000000}
  topics_with_message_count:
    - topic_metadata:
        name: /tf
        name: /cmd_vel
        type: std_msgs/msg/String
        serialization_format: cdr
      message_count: 1
""",
            encoding="utf-8",
        )
        with self.assertRaises(RecordingValidationError) as duplicate_name:
            manager.start_playback("run_001")
        self.assertEqual("metadata_corrupted", duplicate_name.exception.code)

    def test_playback_natural_completion_and_backend_failure_are_explicit(self) -> None:
        manager = self._manager()
        self._completed_recording(manager)
        manager.start_playback("run_001")
        self.factory.processes[-1].returncode = 0
        completed = manager.playback_status()
        self.assertEqual("completed", completed["state"])
        self.assertEqual(1.0, completed["progress"])

        manager.start_playback("run_001", start_offset=2.0)
        self.factory.processes[-1].returncode = 7
        failed = manager.playback_status()
        self.assertEqual("failed", failed["state"])
        self.assertIn("code 7", failed["error"])

        manager.start_playback("run_001")
        self.factory.processes[-1].returncode = -signal.SIGINT
        signaled = manager.playback_status()
        self.assertEqual("failed", signaled["state"])
        self.assertIn("code -2", signaled["error"])

    def test_requested_sigint_exit_is_a_clean_playback_stop(self) -> None:
        signals = _SignalController()
        manager = self._manager(signal_sender=signals)
        self._completed_recording(manager)
        manager.start_playback("run_001")
        process_count = len(self.factory.processes)
        signals.exit_code = -signal.SIGINT

        restarted = manager.set_playback_rate(2.0)

        self.assertEqual("playing", restarted["state"])
        self.assertEqual(2.0, restarted["rate"])
        self.assertIsNone(restarted["error"])
        self.assertEqual(process_count + 1, len(self.factory.processes))

        canceled = manager.cancel_playback()

        self.assertEqual("canceled", canceled["state"])
        self.assertIsNone(canceled["error"])
        self.assertEqual("idle", manager.active_mode)

    def test_unexpected_recorder_exit_is_finalized_and_releases_owner(self) -> None:
        manager = self._manager()
        manager.start_recording("recorder_crash")
        self._write_metadata("recorder_crash", ["/joint_states"])
        self.factory.processes[-1].returncode = 7

        failed = manager.recording_status()

        self.assertEqual("failed", failed["state"])
        self.assertIn("code 7", failed["error"])
        self.assertEqual("idle", manager.active_mode)
        manifest = manager.get_recording("recorder_crash")
        self.assertEqual("failed", manifest["status"])
        self.assertIn("code 7", manifest["result"]["reason"])

    def test_unterminated_recorder_retains_owner_until_exit_is_confirmed(self) -> None:
        signals = _SignalController()
        signals.terminate = False
        manager = self._manager(signal_sender=signals)
        manager.start_recording("stubborn_recorder")
        self._write_metadata("stubborn_recorder", ["/joint_states"])
        process = self.factory.processes[-1]

        with self.assertRaises(RecordingBackendUnavailableError) as raised:
            manager.stop_recording()
        self.assertEqual("recorder_stop_failed", raised.exception.code)
        self.assertIn("ownership was retained", str(raised.exception))
        self.assertEqual("recording", manager.active_mode)
        self.assertFalse(process.kwargs["stdout"].closed)
        with self.assertRaises(RecordingConflictError):
            manager.start_recording("second_recorder")
        with self.assertRaises(RecordingConflictError):
            manager.start_playback("stubborn_recorder")

        signals.terminate = True
        stopped = manager.stop_recording()
        self.assertEqual("completed", stopped["state"])
        self.assertTrue(process.kwargs["stdout"].closed)
        self.assertEqual("idle", manager.active_mode)

    def test_unterminated_player_blocks_rate_restart_and_cancel(self) -> None:
        signals = _SignalController()
        manager = self._manager(signal_sender=signals)
        self._completed_recording(manager)
        manager.start_playback("run_001")
        manager.pause_playback()
        process = self.factory.processes[-1]
        process_count = len(self.factory.processes)
        signals.terminate = False

        with self.assertRaises(RecordingBackendUnavailableError) as rate_error:
            manager.set_playback_rate(2.0)
        self.assertEqual("playback_stop_failed", rate_error.exception.code)
        self.assertEqual(process_count, len(self.factory.processes))
        self.assertEqual("paused", manager.playback_status()["state"])
        self.assertEqual("playback", manager.active_mode)
        self.assertFalse(process.kwargs["stdout"].closed)
        with self.assertRaises(RecordingConflictError):
            manager.start_playback("run_001")

        with self.assertRaises(RecordingBackendUnavailableError) as cancel_error:
            manager.cancel_playback()
        self.assertEqual("playback_stop_failed", cancel_error.exception.code)
        self.assertEqual("paused", manager.playback_status()["state"])
        self.assertIs(process, manager._playback_process)

        signals.terminate = True
        canceled = manager.cancel_playback()
        self.assertEqual("canceled", canceled["state"])
        self.assertEqual("idle", manager.active_mode)
        self.assertTrue(process.kwargs["stdout"].closed)

    def test_stopped_recorder_finalizes_failure_after_artifact_corruption(self) -> None:
        manager = self._manager()
        manager.start_recording("broken_timeline")
        self._write_metadata("broken_timeline", ["/joint_states"])
        timeline = self.root / "broken_timeline" / "timeline.jsonl"
        timeline.write_text("{broken\n", encoding="utf-8")

        stopped = manager.stop_recording()
        self.assertEqual("failed", stopped["state"])
        self.assertIn("Failed to finalize task scenario", stopped["error"])
        self.assertIn("corrupted", stopped["error"])
        self.assertEqual("idle", manager.active_mode)
        retained = manager.get_recording("broken_timeline")
        self.assertEqual("failed", retained["status"])

        manager.start_recording("broken_manifest")
        self._write_metadata("broken_manifest", ["/joint_states"])
        manifest = self.root / "broken_manifest" / "manifest.json"
        manifest.write_text("{broken", encoding="utf-8")
        stopped = manager.stop_recording()
        self.assertEqual("failed", stopped["state"])
        self.assertIn("Manifest", stopped["error"])
        self.assertEqual("idle", manager.active_mode)
        self.assertEqual("failed", manager.get_recording("broken_manifest")["status"])

    def test_storage_files_are_confined_present_and_retained(self) -> None:
        manager = self._manager()
        self._completed_recording(manager, "safe_storage")
        manifest = manager.get_recording("safe_storage")
        self.assertEqual(["data_0.db3"], manifest["storage_files"])

        metadata = self.root / "safe_storage" / "bag" / "metadata.yaml"
        outside = self.root / "safe_storage" / "outside.db3"
        outside.touch()
        metadata.write_text(
            _metadata(["/tf"]).replace("data_0.db3", "../outside.db3"),
            encoding="utf-8",
        )
        process_count = len(self.factory.processes)
        with self.assertRaises(RecordingValidationError) as escaped:
            manager.start_playback("safe_storage")
        self.assertEqual("metadata_corrupted", escaped.exception.code)
        self.assertEqual(process_count, len(self.factory.processes))

        metadata.write_text(_metadata(["/tf"]), encoding="utf-8")
        storage = metadata.parent / "data_0.db3"
        storage.unlink()
        storage.symlink_to(outside)
        with self.assertRaises(RecordingValidationError) as linked:
            manager.start_playback("safe_storage")
        self.assertEqual("metadata_corrupted", linked.exception.code)

        manager.start_recording("missing_storage")
        self._write_metadata("missing_storage", ["/joint_states"])
        (self.root / "missing_storage" / "bag" / "data_0.db3").unlink()
        stopped = manager.stop_recording()
        self.assertEqual("failed", stopped["state"])
        self.assertIn("storage file", stopped["error"])
        self.assertEqual(
            "failed",
            manager.get_recording("missing_storage")["status"],
        )

    def test_confirmed_recorder_exit_survives_log_close_failure(self) -> None:
        manager = self._manager()
        manager.start_recording("recorder_close_failure")
        self._write_metadata("recorder_close_failure")
        failing_log = _CloseFailure(manager._recording_log)
        manager._recording_log = failing_log

        stopped = manager.stop_recording()
        self.assertEqual("completed", stopped["state"])
        self.assertIn("injected close failure", stopped["error"])
        self.assertTrue(failing_log.attempted)
        self.assertIsNone(manager._recording_process)
        self.assertIsNone(manager._recording_log)
        self.assertEqual("idle", manager.active_mode)

        manager.start_recording("after_recorder_close_failure")
        self._write_metadata("after_recorder_close_failure")
        self.assertEqual("completed", manager.stop_recording()["state"])

    def test_confirmed_player_exit_survives_log_close_failures(self) -> None:
        manager = self._manager()
        self._completed_recording(manager)

        manager.start_playback("run_001")
        natural_log = _CloseFailure(manager._playback_log)
        manager._playback_log = natural_log
        self.factory.processes[-1].returncode = 0
        natural = manager.playback_status()
        self.assertEqual("failed", natural["state"])
        self.assertIn("could not be closed", natural["error"])
        self.assertTrue(natural_log.attempted)
        self.assertIsNone(manager._playback_process)
        self.assertEqual("idle", manager.active_mode)

        manager.start_playback("run_001")
        cancel_log = _CloseFailure(manager._playback_log)
        manager._playback_log = cancel_log
        with self.assertRaises(RecordingBackendUnavailableError) as canceled:
            manager.cancel_playback()
        self.assertEqual("playback_log_close_failed", canceled.exception.code)
        self.assertEqual("failed", manager.playback_status()["state"])
        self.assertIsNone(manager._playback_process)
        self.assertEqual("idle", manager.active_mode)

        manager.start_playback("run_001")
        rate_log = _CloseFailure(manager._playback_log)
        manager._playback_log = rate_log
        process_count = len(self.factory.processes)
        with self.assertRaises(RecordingBackendUnavailableError) as changed:
            manager.set_playback_rate(2.0)
        self.assertEqual("playback_log_close_failed", changed.exception.code)
        self.assertEqual(process_count, len(self.factory.processes))
        self.assertEqual("failed", manager.playback_status()["state"])
        self.assertIsNone(manager._playback_process)
        self.assertEqual("idle", manager.active_mode)

    def test_process_group_children_are_killed_before_owner_release(self) -> None:
        def signal_group(process: _FakeProcess, signal_number: int) -> None:
            process.signals.append(signal_number)
            if signal_number == signal.SIGINT:
                process.returncode = 0
            elif signal_number == signal.SIGKILL:
                process.group_alive = False

        manager = self._manager(
            signal_sender=signal_group,
            process_group_probe=lambda process: process.group_alive,
        )
        manager.start_recording("group_child")
        self._write_metadata("group_child")
        process = self.factory.processes[-1]
        process.group_alive = True

        stopped = manager.stop_recording()
        self.assertEqual("failed", stopped["state"])
        self.assertIn("required SIGKILL cleanup", stopped["error"])
        self.assertEqual([signal.SIGINT, signal.SIGKILL], process.signals)
        self.assertFalse(process.group_alive)
        self.assertEqual("idle", manager.active_mode)

    def test_uncleared_process_group_retains_recorder_ownership(self) -> None:
        def ignore_group_cleanup(
            process: _FakeProcess,
            signal_number: int,
        ) -> None:
            process.signals.append(signal_number)
            if signal_number == signal.SIGINT:
                process.returncode = 0

        manager = self._manager(
            signal_sender=ignore_group_cleanup,
            process_group_probe=lambda process: process.group_alive,
        )
        manager.start_recording("stubborn_group")
        self._write_metadata("stubborn_group")
        process = self.factory.processes[-1]
        process.group_alive = True

        with self.assertRaises(RecordingBackendUnavailableError) as raised:
            manager.stop_recording()
        self.assertEqual("recorder_stop_failed", raised.exception.code)
        self.assertIn("process group remains", str(raised.exception))
        self.assertIs(process, manager._recording_process)
        self.assertEqual("recording", manager.active_mode)

        process.group_alive = False
        stopped = manager.stop_recording()
        self.assertEqual("completed", stopped["state"])
        self.assertEqual("idle", manager.active_mode)

    def test_missing_recording_directory_does_not_leak_recorder(self) -> None:
        manager = self._manager()
        manager.start_recording("deleted_directory")
        process = self.factory.processes[-1]
        shutil.rmtree(self.root / "deleted_directory")

        with self.assertRaises(RecordingBackendUnavailableError) as raised:
            manager.stop_recording()
        self.assertEqual("recording_finalize_failed", raised.exception.code)
        self.assertIn("recording directory", str(raised.exception))
        self.assertEqual(0, process.returncode)
        self.assertIn(signal.SIGINT, process.signals)
        self.assertTrue(process.kwargs["stdout"].closed)
        self.assertEqual("idle", manager.active_mode)

        manager.start_recording("after_deleted_directory")
        self._write_metadata("after_deleted_directory")
        self.assertEqual("completed", manager.stop_recording()["state"])

    def test_symlink_replacement_does_not_leak_or_escape_recorder(self) -> None:
        manager = self._manager()
        manager.start_recording("replaced_directory")
        process = self.factory.processes[-1]
        recording_path = self.root / "replaced_directory"
        displaced_path = self.workspace / "displaced_recording"
        outside_path = self.workspace / "outside_recording"
        outside_path.mkdir()
        sentinel = outside_path / "manifest.json"
        sentinel.write_text("outside\n", encoding="utf-8")
        recording_path.rename(displaced_path)
        recording_path.symlink_to(outside_path, target_is_directory=True)

        with self.assertRaises(RecordingBackendUnavailableError) as raised:
            manager.stop_recording()
        self.assertEqual("recording_finalize_failed", raised.exception.code)
        self.assertIn("symbolic link", str(raised.exception))
        self.assertEqual(0, process.returncode)
        self.assertIn(signal.SIGINT, process.signals)
        self.assertTrue(process.kwargs["stdout"].closed)
        self.assertEqual("idle", manager.active_mode)
        self.assertEqual("outside\n", sentinel.read_text(encoding="utf-8"))

        manager.start_recording("after_symlink_replacement")
        self._write_metadata("after_symlink_replacement")
        self.assertEqual("completed", manager.stop_recording()["state"])

    def test_unsafe_or_corrupt_bag_is_rejected_before_process_start(self) -> None:
        manager = self._manager()
        self._completed_recording(manager)
        process_count = len(self.factory.processes)
        self._write_metadata("run_001", ["/tf", "/cmd_vel"])
        with self.assertRaises(PlaybackSafetyError) as unsafe:
            manager.start_playback("run_001")
        self.assertEqual("/cmd_vel", unsafe.exception.topic)
        self.assertEqual(process_count, len(self.factory.processes))

        metadata = self.root / "run_001" / "bag" / "metadata.yaml"
        metadata.write_text("not_rosbag_metadata: true\n", encoding="utf-8")
        with self.assertRaises(RecordingValidationError) as corrupt:
            manager.start_playback("run_001")
        self.assertEqual("metadata_corrupted", corrupt.exception.code)
        self.assertEqual(process_count, len(self.factory.processes))

    def test_list_marks_corrupt_entries_and_shutdown_stops_owned_process(self) -> None:
        manager = self._manager()
        manager.start_recording("active_1")
        corrupt = self.root / "corrupt_1"
        corrupt.mkdir()
        (corrupt / "manifest.json").write_text("{broken", encoding="utf-8")
        listed = {item["recording_id"]: item for item in manager.list_recordings()}
        self.assertEqual("recording", listed["active_1"]["status"])
        self.assertEqual("corrupted", listed["corrupt_1"]["status"])

        self._write_metadata("active_1")
        result = manager.shutdown()
        self.assertEqual("failed", result["recording"]["state"])
        self.assertEqual("idle", result["active_mode"])
        self.assertIn(signal.SIGINT, self.factory.processes[0].signals)


if __name__ == "__main__":
    unittest.main()
