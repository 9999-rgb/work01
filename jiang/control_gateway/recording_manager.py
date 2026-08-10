"""Safe rosbag2 recording, isolated playback, and task-scenario extraction.

The manager deliberately has no ROS Python imports.  It delegates bag I/O to
the official ``ros2 bag`` command and keeps process creation injectable so the
safety contract can be tested without a running ROS graph.

Playback never republishes a recorded topic under its original name.  Every
topic found in ``metadata.yaml`` is validated and explicitly remapped below
``/xczs/replay/<recording_id>``.  Bags containing command or Action-goal topics
are rejected before a player process is started.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import secrets
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Optional, Sequence

import yaml


_RECORDING_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")
_ROS_TOPIC_PATTERN = re.compile(r"^/[A-Za-z0-9_/]+$")
_TASK_TOPIC_PATTERN = r"/xczs/task/[^/]+/(?:progress|result)"
_CABINET_TOPIC_PATTERNS = (
    r"/xczs/cabinet/control_catalog",
    r"/xczs/cabinet/(?:state|control_state)",
    r"/xczs/cabinet/[^/]+/control_catalog",
    r"/xczs/cabinet/[^/]+/(?:state|control_state)",
    r"/xczs/cabinet/[^/]+/[^/]+/(?:joint_states|pressed|state)",
)

DEFAULT_TOPICS = (
    "/tf",
    "/tf_static",
    "/clock",
    "/odom",
    "/xczs/odom",
    "/amcl_pose",
    "/xczs/localization_pose",
    "/joint_states",
    "/xczs/joint_states",
)

SENSOR_TOPIC_GROUPS = {
    "lidar": (
        "/scan",
        "/xczs/scan",
        "/xczs/lidar/scan",
    ),
    "camera": (
        "/camera/image_raw",
        "/camera/camera_info",
        "/camera/depth/image_raw",
        "/xczs/camera/image_raw",
        "/xczs/camera/camera_info",
        "/xczs/camera/depth/image_raw",
        "/xczs/camera/arm_camera/image_raw",
        "/xczs/camera/arm_camera/camera_info",
    ),
}

_DANGEROUS_TOPIC_COMPONENTS = frozenset(
    {
        "cmd_vel",
        "manual_cmd_vel",
        "joint_trajectory",
        "joint_trajectories",
        "trajectory_command",
    }
)
_MAX_METADATA_BYTES = 16 * 1024 * 1024
_MAX_DURATION_NANOSECONDS = (1 << 63) - 1
_PROCESS_GROUP_CONFIRM_SECONDS = 1.0
_PROCESS_GROUP_POLL_SECONDS = 0.05


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects ambiguous duplicate mapping keys."""

    def construct_mapping(
        self,
        node: Any,
        deep: bool = False,
    ) -> dict[Any, Any]:
        self.flatten_mapping(node)
        mapping: dict[Any, Any] = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                duplicate = key in mapping
            except TypeError as error:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    "found an unhashable key",
                    key_node.start_mark,
                ) from error
            if duplicate:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    f"found duplicate key {key!r}",
                    key_node.start_mark,
                )
            mapping[key] = self.construct_object(value_node, deep=deep)
        return mapping


class RecordingError(RuntimeError):
    """Base error carrying an HTTP-friendly status and stable error code."""

    def __init__(self, message: str, *, code: str, status: int = 400) -> None:
        super().__init__(message)
        self.code = code
        self.status = status


class RecordingConflictError(RecordingError):
    """Raised when recording and playback mutual exclusion would be broken."""

    def __init__(self, message: str) -> None:
        super().__init__(message, code="recording_conflict", status=409)


class RecordingNotFoundError(RecordingError):
    """Raised when a safe recording ID has no retained recording."""

    def __init__(self, recording_id: str) -> None:
        super().__init__(
            f"Unknown recording: {recording_id}",
            code="recording_not_found",
            status=404,
        )


class RecordingValidationError(RecordingError):
    """Raised for malformed IDs, manifests, metadata, or API values."""

    def __init__(self, message: str, *, code: str = "invalid_recording") -> None:
        super().__init__(message, code=code, status=400)


class PlaybackSafetyError(RecordingValidationError):
    """Raised before playback when a bag contains a command interface."""

    def __init__(self, topic: str) -> None:
        super().__init__(
            f"Playback rejected unsafe command topic: {topic}",
            code="unsafe_playback_topic",
        )
        self.topic = topic


class RecordingBackendUnavailableError(RecordingError):
    """Raised when the official rosbag2 process cannot be started."""

    def __init__(self, message: str, *, code: str) -> None:
        super().__init__(message, code=code, status=503)


class RecordingManager:
    """Own one mutually exclusive rosbag2 recorder or isolated player.

    Args:
        recordings_root: Data directory.  Every recording path is resolved and
            checked beneath this root before it is accessed.
        workspace_root: Git/config label base.  Defaults to the parent of the
            recordings directory.
        config_paths: Files hashed into each manifest.
        inventory_snapshot: JSON-compatible cabinet inventory stored in each
            manifest.
        record_topics: Adapter-provided read-only topics merged with the safe
            defaults.  Command and Action-goal topics are always rejected.
        popen_factory: Injectable ``subprocess.Popen`` compatible callable.
        clock: Injectable Unix timestamp clock.
        monotonic_clock: Injectable elapsed-time clock.
        signal_sender: Optional ``(process, signal)`` callback for fake tests.
        process_group_probe: Optional callback reporting whether any process
            remains in the player's/recorder's dedicated process group.
        git_commit: Optional fixed commit; otherwise ``git rev-parse HEAD`` is
            queried from ``workspace_root``.
    """

    def __init__(
        self,
        recordings_root: str | Path,
        workspace_root: str | Path | None = None,
        config_paths: Iterable[str | Path] = (),
        inventory_snapshot: Any = None,
        record_topics: Iterable[str] = (),
        *,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
        signal_sender: Optional[Callable[[Any, int], None]] = None,
        process_group_probe: Optional[Callable[[Any], bool]] = None,
        git_commit: Optional[str] = None,
    ) -> None:
        root = Path(recordings_root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self._root = root.resolve(strict=True)
        if not self._root.is_dir():
            raise RecordingValidationError("recordings_root must be a directory.")

        workspace = (
            Path(workspace_root).expanduser()
            if workspace_root is not None
            else self._root.parent
        )
        self._workspace_root = workspace.resolve(strict=False)
        self._config_paths = tuple(
            Path(path).expanduser().resolve(strict=False) for path in config_paths
        )
        self._inventory_snapshot = _json_copy(
            {} if inventory_snapshot is None else inventory_snapshot,
            "inventory_snapshot",
        )
        self._record_topics = _validated_record_topics(record_topics)
        self._popen = popen_factory
        self._clock = clock
        self._monotonic = monotonic_clock
        self._signal_sender = signal_sender
        self._process_group_probe = process_group_probe
        self._git_commit_override = git_commit
        self._lock = threading.RLock()

        self._recording_process: Any = None
        self._recording_id: Optional[str] = None
        self._recording_started_monotonic: Optional[float] = None
        self._recording_log: Any = None
        self._timeline_sequence = 0

        self._playback_process: Any = None
        self._playback_log: Any = None
        self._playback: dict[str, Any] = self._idle_playback_status()

    @property
    def recordings_root(self) -> Path:
        """Resolved data root used for every filesystem operation."""
        return self._root

    @property
    def active_mode(self) -> str:
        """Return ``idle``, ``recording``, or ``playback``."""
        with self._lock:
            self._refresh_playback_locked()
            if self._recording_process is not None:
                return "recording"
            if self._playback.get("state") in {"playing", "paused"}:
                return "playback"
            return "idle"

    def start_recording(
        self,
        recording_id: Optional[str] = None,
        *,
        include_sensors: bool | Iterable[str] = False,
    ) -> dict[str, Any]:
        """Start a safe rosbag2 recorder and create its initial manifest."""
        with self._lock:
            self._refresh_playback_locked()
            if self.active_mode != "idle":
                raise RecordingConflictError(
                    f"Cannot start recording while {self.active_mode} is active."
                )
            identifier = (
                self._new_recording_id()
                if recording_id is None
                else _validate_recording_id(recording_id)
            )
            groups, sensor_topics = _sensor_topics(include_sensors)
            exact_topics = tuple(
                dict.fromkeys(
                    (*DEFAULT_TOPICS, *self._record_topics, *sensor_topics)
                )
            )
            topic_regex = _record_topic_regex(exact_topics)
            created_at = _finite_time(self._clock(), "clock")
            git_commit = self._git_commit()
            config_hashes = self._config_hashes()
            path = self._recording_path(identifier, must_exist=False)
            if path.exists():
                raise RecordingConflictError(
                    f"Recording already exists: {identifier}"
                )
            path.mkdir(mode=0o750)
            bag_path = path / "bag"
            manifest = {
                "schema_version": 1,
                "recording_id": identifier,
                "status": "recording",
                "created_at": created_at,
                "stopped_at": None,
                "duration_seconds": None,
                "git_commit": git_commit,
                "config_sha256": config_hashes,
                "inventory": _json_copy(
                    self._inventory_snapshot,
                    "inventory_snapshot",
                ),
                "topics": list(exact_topics),
                "dynamic_topic_patterns": [
                    *_CABINET_TOPIC_PATTERNS,
                    _TASK_TOPIC_PATTERN,
                ],
                "recorded_topics": [],
                "storage_files": [],
                "sensors_included": bool(groups),
                "sensor_groups": list(groups),
                "tasks": [],
                "result": None,
            }
            _write_json(path / "manifest.json", manifest)
            (path / "timeline.jsonl").touch(mode=0o640, exist_ok=False)
            _write_json(path / "scenario.yaml", self._empty_scenario(identifier))
            log_handle = (path / "rosbag-record.log").open("ab")
            command = [
                "ros2",
                "bag",
                "record",
                "--output",
                str(bag_path),
                "--regex",
                topic_regex,
            ]
            try:
                process = self._popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception as error:
                log_handle.close()
                manifest.update(
                    status="failed",
                    stopped_at=_finite_time(self._clock(), "clock"),
                    duration_seconds=0.0,
                    result={
                        "exit_code": None,
                        "reason": str(error) or type(error).__name__,
                    },
                )
                _write_json(path / "manifest.json", manifest)
                raise RecordingBackendUnavailableError(
                    f"Failed to start rosbag2 recorder: {error}",
                    code="recorder_start_failed",
                ) from error

            self._recording_process = process
            self._recording_id = identifier
            self._recording_started_monotonic = _finite_time(
                self._monotonic(),
                "monotonic_clock",
            )
            self._recording_log = log_handle
            self._timeline_sequence = 0
            return self.recording_status()

    def stop_recording(
        self,
        result: Optional[Mapping[str, Any]] = None,
        *,
        reason: Optional[str] = None,
    ) -> dict[str, Any]:
        """Stop the active recorder, finalize metadata, manifest, and scenario."""
        with self._lock:
            if self._recording_process is None or self._recording_id is None:
                raise RecordingConflictError("No recording is active.")
            process = self._recording_process
            identifier = self._recording_id
            exit_code, termination_reason = self._stop_process(process)
            if exit_code is None:
                detail = termination_reason or "Process termination was not confirmed."
                raise RecordingBackendUnavailableError(
                    "Failed to stop rosbag2 recorder; ownership was retained: "
                    f"{detail}",
                    code="recorder_stop_failed",
                )

            # Once wait()/poll() has confirmed a terminal return code, this
            # manager must release process ownership even when a damaged
            # timeline or manifest prevents normal finalization.
            try:
                try:
                    path = self._recording_path(identifier, must_exist=True)
                except Exception as error:
                    raise RecordingBackendUnavailableError(
                        "Rosbag2 recorder stopped, but its recording directory "
                        "cannot be safely finalized: "
                        f"{str(error) or type(error).__name__}",
                        code="recording_finalize_failed",
                    ) from error
                return self._finalize_recording_locked(
                    identifier,
                    path,
                    exit_code=exit_code,
                    termination_reason=termination_reason,
                    result=result,
                    reason=reason,
                )
            finally:
                close_error: Optional[str] = None
                try:
                    close_error = self._close_recording_log_locked()
                finally:
                    self._recording_process = None
                    self._recording_id = None
                    self._recording_started_monotonic = None
                if close_error:
                    raise RecordingBackendUnavailableError(
                        "Rosbag2 recorder stopped, but its log could not be "
                        f"closed: {close_error}",
                        code="recording_log_close_failed",
                    )

    def recording_status(self) -> dict[str, Any]:
        """Return a stable status object for the active/latest recorder."""
        with self._lock:
            if self._recording_id is None:
                return {
                    "state": "idle",
                    "recording_id": None,
                    "created_at": None,
                    "stopped_at": None,
                    "duration_seconds": 0.0,
                    "topics": [],
                    "error": None,
                }
            process_code = self._process_poll(self._recording_process)
            if process_code is not None:
                return self.stop_recording(
                    reason=(
                        "ros2 bag record exited unexpectedly with code "
                        f"{process_code}."
                    )
                )
            manifest = self._read_manifest(self._recording_id)
            status = self._recording_status_from_manifest(manifest)
            if self._recording_started_monotonic is not None:
                status["duration_seconds"] = max(
                    0.0,
                    _finite_time(self._monotonic(), "monotonic_clock")
                    - self._recording_started_monotonic,
                )
            return status

    def _finalize_recording_locked(
        self,
        recording_id: str,
        path: Path,
        *,
        exit_code: int,
        termination_reason: Optional[str],
        result: Optional[Mapping[str, Any]],
        reason: Optional[str],
    ) -> dict[str, Any]:
        """Finalize a recorder whose process is confirmed to have exited."""
        stopped_at = _finite_time(self._clock(), "clock")
        started = self._recording_started_monotonic
        elapsed = (
            0.0
            if started is None
            else max(
                0.0,
                _finite_time(self._monotonic(), "monotonic_clock") - started,
            )
        )
        finalization_errors: list[str] = []
        try:
            manifest = self._read_manifest(recording_id)
        except RecordingValidationError as error:
            finalization_errors.append(str(error))
            manifest = {
                "schema_version": 1,
                "recording_id": recording_id,
                "status": "failed",
                "created_at": max(0.0, stopped_at - elapsed),
                "stopped_at": None,
                "duration_seconds": None,
                "git_commit": self._git_commit_override,
                "config_sha256": {},
                "inventory": _json_copy(
                    self._inventory_snapshot,
                    "inventory_snapshot",
                ),
                "topics": list(
                    dict.fromkeys((*DEFAULT_TOPICS, *self._record_topics))
                ),
                "dynamic_topic_patterns": [
                    *_CABINET_TOPIC_PATTERNS,
                    _TASK_TOPIC_PATTERN,
                ],
                "recorded_topics": [],
                "storage_files": [],
                "sensors_included": False,
                "sensor_groups": [],
                "tasks": [],
                "result": None,
            }

        recorded_topics: list[str] = []
        storage_files: list[str] = []
        metadata_duration: Optional[float] = None
        metadata_path = path / "bag" / "metadata.yaml"
        if metadata_path.is_file():
            try:
                metadata_path = self._safe_artifact(
                    path,
                    Path("bag") / "metadata.yaml",
                    must_exist=True,
                )
                metadata = _read_rosbag_metadata(metadata_path)
                recorded_topics = metadata["topics"]
                storage_files = metadata["storage_files"]
                metadata_duration = metadata["duration_seconds"]
            except RecordingValidationError as error:
                finalization_errors.append(str(error))
        else:
            finalization_errors.append("Rosbag metadata was not created.")

        try:
            result_document = _json_copy(dict(result or {}), "result")
        except (TypeError, ValueError, RecordingValidationError) as error:
            finalization_errors.append(str(error))
            result_document = {}

        try:
            scenario = self._write_scenario_locked(recording_id)
        except Exception as error:
            finalization_errors.append(
                "Failed to finalize task scenario: "
                f"{str(error) or type(error).__name__}"
            )
            try:
                scenario = self.load_scenario(recording_id)
            except Exception:
                scenario = self._empty_scenario(recording_id)

        reasons: list[str] = []
        for value in (reason, termination_reason, *finalization_errors):
            if value and value not in reasons:
                reasons.append(value)
        final_reason = "; ".join(reasons) if reasons else None
        successful = exit_code == 0 and final_reason is None
        result_document.update(
            exit_code=exit_code,
            reason=final_reason,
        )
        manifest.update(
            status="completed" if successful else "failed",
            stopped_at=stopped_at,
            duration_seconds=(
                metadata_duration if metadata_duration is not None else elapsed
            ),
            recorded_topics=recorded_topics,
            storage_files=storage_files,
            tasks=_scenario_task_summaries(scenario),
            result=result_document,
        )
        try:
            _write_json(path / "manifest.json", manifest)
        except (OSError, TypeError, ValueError) as error:
            raise RecordingBackendUnavailableError(
                "Recorder stopped, but its failed terminal manifest could not "
                f"be written: {error}",
                code="recording_finalize_failed",
            ) from error
        return self._recording_status_from_manifest(manifest)

    def record_task_event(
        self,
        event: str | Mapping[str, Any],
        data: Optional[Mapping[str, Any]] = None,
        *,
        timestamp: Optional[float] = None,
    ) -> Optional[dict[str, Any]]:
        """Append a task event while recording; return ``None`` when idle.

        ``event`` may be an EventHub envelope or an event type string plus
        ``data``.  This makes the method suitable as a direct event-bridge sink.
        """
        with self._lock:
            if self._recording_id is None:
                return None
            if isinstance(event, Mapping):
                if data is not None or timestamp is not None:
                    raise RecordingValidationError(
                        "Envelope events cannot also provide data or timestamp."
                    )
                event_type = event.get("event")
                event_data = event.get("data", {})
                event_timestamp = event.get("timestamp")
                if event_timestamp is None:
                    event_timestamp = self._clock()
            else:
                event_type = event
                event_data = {} if data is None else data
                event_timestamp = self._clock() if timestamp is None else timestamp
            if not isinstance(event_type, str) or not event_type.strip():
                raise RecordingValidationError("Task event type must not be empty.")
            if not isinstance(event_data, Mapping):
                raise RecordingValidationError("Task event data must be a mapping.")
            event_timestamp = _finite_time(event_timestamp, "event timestamp")
            manifest = self._read_manifest(self._recording_id)
            self._timeline_sequence += 1
            envelope = {
                "sequence": self._timeline_sequence,
                "event": event_type.strip(),
                "timestamp": event_timestamp,
                "elapsed_seconds": max(
                    0.0,
                    event_timestamp - float(manifest["created_at"]),
                ),
                "data": _json_copy(dict(event_data), "task event data"),
            }
            timeline_path = (
                self._recording_path(self._recording_id, must_exist=True)
                / "timeline.jsonl"
            )
            with timeline_path.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(
                        envelope,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        sort_keys=True,
                    )
                    + "\n"
                )
                stream.flush()
            # Keep scenario.yaml useful even if the process crashes before an
            # explicit stop.  The file is tiny compared with rosbag I/O.
            scenario = self._write_scenario_locked(self._recording_id)
            manifest["tasks"] = _scenario_task_summaries(scenario)
            _write_json(
                self._recording_path(self._recording_id, must_exist=True)
                / "manifest.json",
                manifest,
            )
            return copy.deepcopy(envelope)

    def list_recordings(self) -> list[dict[str, Any]]:
        """Return safe manifest summaries, newest first."""
        with self._lock:
            recordings: list[dict[str, Any]] = []
            for candidate in self._root.iterdir():
                if candidate.is_symlink() or not candidate.is_dir():
                    continue
                if not _RECORDING_ID_PATTERN.fullmatch(candidate.name):
                    continue
                try:
                    manifest = self._read_manifest(candidate.name)
                    recordings.append(self._recording_summary(manifest))
                except RecordingError as error:
                    recordings.append(
                        {
                            "recording_id": candidate.name,
                            "status": "corrupted",
                            "created_at": None,
                            "stopped_at": None,
                            "duration_seconds": None,
                            "topics": [],
                            "sensors_included": False,
                            "error": str(error),
                        }
                    )
            recordings.sort(
                key=lambda value: (
                    value.get("created_at") is not None,
                    value.get("created_at") or 0.0,
                    value["recording_id"],
                ),
                reverse=True,
            )
            return recordings

    def get_recording(self, recording_id: str) -> dict[str, Any]:
        """Return a validated manifest plus derived artifact availability."""
        with self._lock:
            identifier = _validate_recording_id(recording_id)
            path = self._recording_path(identifier, must_exist=True)
            manifest = self._read_manifest(identifier)
            result = copy.deepcopy(manifest)
            result["artifacts"] = {
                "bag": (path / "bag" / "metadata.yaml").is_file(),
                "timeline": (path / "timeline.jsonl").is_file(),
                "scenario": (path / "scenario.yaml").is_file(),
            }
            return result

    def timeline(
        self,
        recording_id: str,
        *,
        offset: int = 0,
        limit: int = 500,
    ) -> dict[str, Any]:
        """Read a bounded page from a recording's JSON-lines timeline."""
        if isinstance(offset, bool) or not isinstance(offset, int) or offset < 0:
            raise RecordingValidationError("Timeline offset must be nonnegative.")
        if (
            isinstance(limit, bool)
            or not isinstance(limit, int)
            or limit <= 0
            or limit > 5000
        ):
            raise RecordingValidationError("Timeline limit must be 1..5000.")
        with self._lock:
            path = self._recording_path(recording_id, must_exist=True)
            timeline_path = path / "timeline.jsonl"
            if timeline_path.is_symlink() or not timeline_path.is_file():
                raise RecordingValidationError(
                    f"Recording {recording_id} has no valid timeline.",
                    code="timeline_missing",
                )
            events: list[dict[str, Any]] = []
            total = 0
            try:
                with timeline_path.open(encoding="utf-8") as stream:
                    for index, line in enumerate(stream):
                        total = index + 1
                        if index < offset or len(events) >= limit:
                            continue
                        value = json.loads(line)
                        if not isinstance(value, dict):
                            raise ValueError("event must be an object")
                        events.append(value)
            except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
                raise RecordingValidationError(
                    f"Timeline for {recording_id} is corrupted: {error}",
                    code="timeline_corrupted",
                ) from error
            next_offset = offset + len(events)
            return {
                "recording_id": _validate_recording_id(recording_id),
                "events": events,
                "offset": offset,
                "next_offset": next_offset,
                "has_more": next_offset < total,
                "total": total,
            }

    def load_scenario(self, recording_id: str) -> dict[str, Any]:
        """Load and validate the JSON-compatible ``scenario.yaml`` document."""
        with self._lock:
            path = self._recording_path(recording_id, must_exist=True)
            scenario_path = path / "scenario.yaml"
            if scenario_path.is_symlink() or not scenario_path.is_file():
                raise RecordingValidationError(
                    f"Recording {recording_id} has no valid scenario.",
                    code="scenario_missing",
                )
            try:
                document = json.loads(scenario_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as error:
                raise RecordingValidationError(
                    f"Scenario for {recording_id} is corrupted: {error}",
                    code="scenario_corrupted",
                ) from error
            return _validate_scenario(document, _validate_recording_id(recording_id))

    def start_playback(
        self,
        recording_id: str,
        *,
        rate: float = 1.0,
        start_offset: float = 0.0,
    ) -> dict[str, Any]:
        """Play a bag with every topic remapped into an isolated namespace."""
        with self._lock:
            self._refresh_playback_locked()
            if self.active_mode != "idle":
                raise RecordingConflictError(
                    f"Cannot start playback while {self.active_mode} is active."
                )
            identifier = _validate_recording_id(recording_id)
            rate_value = _positive_rate(rate)
            offset_value = _nonnegative_number(start_offset, "start_offset")
            metadata = self._playback_metadata(identifier)
            duration = metadata["duration_seconds"]
            if offset_value > duration:
                raise RecordingValidationError(
                    "Playback start_offset exceeds recording duration."
                )
            self._launch_playback_locked(
                identifier,
                metadata,
                rate=rate_value,
                start_offset=offset_value,
            )
            return self.playback_status()

    def pause_playback(self) -> dict[str, Any]:
        """Pause the active player with ``SIGSTOP``."""
        with self._lock:
            self._refresh_playback_locked()
            if self._playback.get("state") != "playing":
                raise RecordingConflictError("Playback is not running.")
            position = self._playback_position_locked()
            self._send_signal(self._playback_process, signal.SIGSTOP)
            self._playback["position_seconds"] = position
            self._playback["state"] = "paused"
            self._playback["started_monotonic"] = None
            return self._playback_snapshot_locked()

    def resume_playback(self) -> dict[str, Any]:
        """Resume a paused player with ``SIGCONT``."""
        with self._lock:
            self._refresh_playback_locked()
            if self._playback.get("state") != "paused":
                raise RecordingConflictError("Playback is not paused.")
            self._send_signal(self._playback_process, signal.SIGCONT)
            self._playback["state"] = "playing"
            self._playback["started_monotonic"] = _finite_time(
                self._monotonic(),
                "monotonic_clock",
            )
            return self._playback_snapshot_locked()

    def set_playback_rate(self, rate: float) -> dict[str, Any]:
        """Restart the isolated player at its current offset and a new rate."""
        with self._lock:
            self._refresh_playback_locked()
            if self._playback.get("state") not in {"playing", "paused"}:
                raise RecordingConflictError("Playback is not active.")
            rate_value = _positive_rate(rate)
            was_paused = self._playback["state"] == "paused"
            position = self._playback_position_locked()
            identifier = self._playback["recording_id"]
            metadata = self._playback_metadata(identifier)
            self._playback["position_seconds"] = position
            _, termination_diagnostic = self._terminate_playback_locked(
                "change playback rate"
            )
            self._playback_process = None
            self._playback.update(
                state="failed",
                position_seconds=position,
                started_monotonic=None,
                error=(
                    termination_diagnostic
                    or "Playback stopped before rate restart."
                ),
            )
            close_error = self._close_playback_log_locked()
            if termination_diagnostic or close_error:
                details = []
                if termination_diagnostic:
                    details.append(termination_diagnostic)
                if close_error:
                    details.append(
                        "Playback log could not be closed: " + close_error
                    )
                message = "Playback rate change was aborted: " + "; ".join(
                    details
                )
                self._playback.update(
                    state="failed",
                    position_seconds=position,
                    started_monotonic=None,
                    error=message,
                )
                raise RecordingBackendUnavailableError(
                    message,
                    code=(
                        "playback_log_close_failed"
                        if close_error
                        else "playback_group_cleanup_required"
                    ),
                )
            self._launch_playback_locked(
                identifier,
                metadata,
                rate=rate_value,
                start_offset=position,
            )
            if was_paused:
                self._send_signal(self._playback_process, signal.SIGSTOP)
                self._playback["state"] = "paused"
                self._playback["started_monotonic"] = None
            return self._playback_snapshot_locked()

    def cancel_playback(self) -> dict[str, Any]:
        """Cancel an active player and retain a terminal status snapshot."""
        with self._lock:
            self._refresh_playback_locked()
            if self._playback.get("state") not in {"playing", "paused"}:
                return self._playback_snapshot_locked()
            position = self._playback_position_locked()
            self._playback["position_seconds"] = position
            _, termination_diagnostic = self._terminate_playback_locked(
                "cancel playback"
            )
            self._playback_process = None
            self._playback.update(
                state="failed" if termination_diagnostic else "canceled",
                position_seconds=position,
                started_monotonic=None,
                error=termination_diagnostic,
            )
            close_error = self._close_playback_log_locked()
            terminal_error = termination_diagnostic
            if close_error:
                close_diagnostic = (
                    "Playback was canceled, but its log could not be closed: "
                    f"{close_error}"
                )
                terminal_error = (
                    f"{terminal_error}; {close_diagnostic}"
                    if terminal_error
                    else close_diagnostic
                )
            self._playback.update(
                state="failed" if terminal_error else "canceled",
                position_seconds=position,
                started_monotonic=None,
                error=terminal_error,
            )
            if terminal_error:
                raise RecordingBackendUnavailableError(
                    terminal_error,
                    code=(
                        "playback_log_close_failed"
                        if close_error
                        else "playback_group_cleanup_required"
                    ),
                )
            return self._playback_snapshot_locked()

    def playback_status(self) -> dict[str, Any]:
        """Return state, rate, estimated position/progress, and any error."""
        with self._lock:
            self._refresh_playback_locked()
            return self._playback_snapshot_locked()

    def shutdown(self) -> dict[str, Any]:
        """Best-effort bounded termination of the owned recorder/player."""
        with self._lock:
            recording = None
            playback = None
            if self._recording_process is not None:
                recording = self.stop_recording(reason="Gateway shutdown.")
            if self._playback.get("state") in {"playing", "paused"}:
                playback = self.cancel_playback()
            return {
                "recording": recording,
                "playback": playback,
                "active_mode": self.active_mode,
            }

    def _new_recording_id(self) -> str:
        timestamp_ms = int(_finite_time(self._clock(), "clock") * 1000.0)
        return f"recording_{timestamp_ms}_{secrets.token_hex(3)}"

    def _recording_path(
        self,
        recording_id: str,
        *,
        must_exist: bool,
    ) -> Path:
        identifier = _validate_recording_id(recording_id)
        lexical = self._root / identifier
        if lexical.is_symlink():
            raise RecordingValidationError(
                f"Recording path must not be a symbolic link: {identifier}"
            )
        try:
            resolved = lexical.resolve(strict=must_exist)
        except FileNotFoundError as error:
            raise RecordingNotFoundError(identifier) from error
        if resolved.parent != self._root:
            raise RecordingValidationError("Recording path escapes recordings_root.")
        if must_exist and not resolved.is_dir():
            raise RecordingNotFoundError(identifier)
        return resolved

    def _safe_artifact(
        self,
        recording_path: Path,
        relative_path: Path,
        *,
        must_exist: bool,
    ) -> Path:
        """Resolve an artifact without following links out of its recording."""
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise RecordingValidationError("Recording artifact path is unsafe.")
        current = recording_path
        for part in relative_path.parts:
            current = current / part
            if current.is_symlink():
                raise RecordingValidationError(
                    "Recording artifact must not be a symbolic link: "
                    f"{relative_path}"
                )
        try:
            resolved = (recording_path / relative_path).resolve(
                strict=must_exist
            )
        except FileNotFoundError as error:
            raise RecordingValidationError(
                f"Recording artifact is missing: {relative_path}"
            ) from error
        try:
            resolved.relative_to(recording_path)
        except ValueError as error:
            raise RecordingValidationError(
                "Recording artifact escapes its data directory: "
                f"{relative_path}"
            ) from error
        return resolved

    def _read_manifest(self, recording_id: str) -> dict[str, Any]:
        identifier = _validate_recording_id(recording_id)
        path = self._recording_path(identifier, must_exist=True) / "manifest.json"
        if path.is_symlink() or not path.is_file():
            raise RecordingValidationError(
                f"Recording {identifier} has no valid manifest.",
                code="manifest_missing",
            )
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise RecordingValidationError(
                f"Manifest for {identifier} is corrupted: {error}",
                code="manifest_corrupted",
            ) from error
        if not isinstance(value, dict) or value.get("recording_id") != identifier:
            raise RecordingValidationError(
                f"Manifest identity does not match {identifier}.",
                code="manifest_corrupted",
            )
        if value.get("schema_version") != 1:
            raise RecordingValidationError(
                f"Manifest for {identifier} has an unsupported schema.",
                code="manifest_corrupted",
            )
        return value

    def _recording_summary(self, manifest: Mapping[str, Any]) -> dict[str, Any]:
        result = manifest.get("result")
        error = result.get("reason") if isinstance(result, Mapping) else None
        return {
            "recording_id": manifest["recording_id"],
            "status": manifest.get("status"),
            "created_at": manifest.get("created_at"),
            "stopped_at": manifest.get("stopped_at"),
            "duration_seconds": manifest.get("duration_seconds"),
            "topics": list(
                manifest.get("recorded_topics") or manifest.get("topics") or []
            ),
            "sensors_included": bool(manifest.get("sensors_included")),
            "error": error,
        }

    def _recording_status_from_manifest(
        self,
        manifest: Mapping[str, Any],
    ) -> dict[str, Any]:
        result = manifest.get("result")
        error = result.get("reason") if isinstance(result, Mapping) else None
        return {
            "state": manifest.get("status", "failed"),
            "recording_id": manifest.get("recording_id"),
            "created_at": manifest.get("created_at"),
            "stopped_at": manifest.get("stopped_at"),
            "duration_seconds": manifest.get("duration_seconds") or 0.0,
            "topics": list(
                manifest.get("recorded_topics") or manifest.get("topics") or []
            ),
            "error": error,
        }

    def _config_hashes(self) -> dict[str, str]:
        hashes: dict[str, str] = {}
        for path in self._config_paths:
            if path.is_symlink() or not path.is_file():
                raise RecordingValidationError(
                    f"Configured hash input is not a regular file: {path}",
                    code="config_hash_failed",
                )
            try:
                label = str(path.relative_to(self._workspace_root))
            except ValueError:
                label = path.name
            if label in hashes:
                raise RecordingValidationError(
                    f"Duplicate config hash label: {label}",
                    code="config_hash_failed",
                )
            digest = hashlib.sha256()
            try:
                with path.open("rb") as stream:
                    for block in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(block)
            except OSError as error:
                raise RecordingValidationError(
                    f"Cannot hash config file {path}: {error}",
                    code="config_hash_failed",
                ) from error
            hashes[label] = digest.hexdigest()
        return hashes

    def _git_commit(self) -> Optional[str]:
        if self._git_commit_override is not None:
            value = self._git_commit_override.strip()
            return value or None
        try:
            completed = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=self._workspace_root,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                check=False,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        value = completed.stdout.strip()
        return value if completed.returncode == 0 and value else None

    def _empty_scenario(self, recording_id: str) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "recording_id": recording_id,
            "steps": [],
        }

    def _write_scenario_locked(self, recording_id: str) -> dict[str, Any]:
        events: list[dict[str, Any]] = []
        offset = 0
        while True:
            page = self.timeline(recording_id, offset=offset, limit=5000)
            events.extend(page["events"])
            if not page["has_more"]:
                break
            offset = page["next_offset"]
        accepted: dict[str, dict[str, Any]] = {}
        steps: list[dict[str, Any]] = []
        for event in events:
            data = event.get("data")
            if not isinstance(data, Mapping):
                continue
            task = data.get("task")
            if event.get("event") == "task_accepted" and isinstance(task, Mapping):
                task_type = task.get("type")
                request = task.get("request")
                task_id = task.get("task_id") or data.get("task_id")
                if (
                    task_type not in {"navigate", "operate", "reset"}
                    or not isinstance(request, Mapping)
                    or not isinstance(task_id, str)
                    or not task_id
                ):
                    continue
                step = {
                    "type": task_type,
                    "request": _json_copy(dict(request), "scenario request"),
                    "recorded_task_id": task_id,
                }
                accepted[task_id] = step
                steps.append(step)
            elif event.get("event") == "task_completed":
                task_id = data.get("task_id")
                step = accepted.get(task_id) if isinstance(task_id, str) else None
                if step is None:
                    continue
                step["recorded_outcome"] = data.get("outcome")
                if isinstance(task, Mapping):
                    step["recorded_result"] = _json_copy(
                        task.get("result") or {},
                        "scenario result",
                    )
                    step["recorded_failure_code"] = task.get("failure_code")
                    step["recorded_failure_reason"] = task.get("failure_reason")
        scenario = {
            "schema_version": (
                2 if any(step["type"] == "reset" for step in steps) else 1
            ),
            "recording_id": recording_id,
            "steps": steps,
        }
        path = self._recording_path(recording_id, must_exist=True)
        _write_json(path / "scenario.yaml", scenario)
        return scenario

    def _playback_metadata(self, recording_id: str) -> dict[str, Any]:
        path = self._recording_path(recording_id, must_exist=True)
        metadata_path = path / "bag" / "metadata.yaml"
        if not metadata_path.is_file():
            raise RecordingValidationError(
                f"Recording {recording_id} has no readable rosbag metadata.",
                code="metadata_missing",
            )
        metadata_path = self._safe_artifact(
            path,
            Path("bag") / "metadata.yaml",
            must_exist=True,
        )
        metadata = _read_rosbag_metadata(metadata_path)
        if not metadata["topics"]:
            raise RecordingValidationError(
                f"Recording {recording_id} metadata contains no topics.",
                code="metadata_corrupted",
            )
        for topic in metadata["topics"]:
            if _is_dangerous_topic(topic):
                raise PlaybackSafetyError(topic)
        return metadata

    def _launch_playback_locked(
        self,
        recording_id: str,
        metadata: Mapping[str, Any],
        *,
        rate: float,
        start_offset: float,
    ) -> None:
        path = self._recording_path(recording_id, must_exist=True)
        bag_path = self._safe_artifact(path, Path("bag"), must_exist=True)
        topics = list(metadata["topics"])
        prefix = f"/xczs/replay/{recording_id}"
        remaps = [f"{topic}:={prefix}{topic}" for topic in topics]
        command = [
            "ros2",
            "bag",
            "play",
            str(bag_path),
            "--rate",
            _format_number(rate),
            "--start-offset",
            _format_number(start_offset),
            "--topics",
            *topics,
            "--remap",
            *remaps,
        ]
        log_path = self._safe_artifact(
            path,
            Path("rosbag-play.log"),
            must_exist=False,
        )
        log_handle = log_path.open("ab")
        try:
            process = self._popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception as error:
            log_handle.close()
            self._playback_process = None
            self._playback = {
                **self._idle_playback_status(),
                "state": "failed",
                "recording_id": recording_id,
                "rate": rate,
                "position_seconds": start_offset,
                "duration_seconds": metadata["duration_seconds"],
                "error": str(error) or type(error).__name__,
            }
            raise RecordingBackendUnavailableError(
                f"Failed to start isolated rosbag2 playback: {error}",
                code="playback_start_failed",
            ) from error
        self._playback_process = process
        self._playback_log = log_handle
        self._playback = {
            "state": "playing",
            "recording_id": recording_id,
            "rate": rate,
            "position_seconds": start_offset,
            "duration_seconds": float(metadata["duration_seconds"]),
            "progress": 0.0,
            "error": None,
            "topics": topics,
            "started_monotonic": _finite_time(
                self._monotonic(),
                "monotonic_clock",
            ),
        }

    def _terminate_playback_locked(
        self,
        operation: str,
    ) -> tuple[int, Optional[str]]:
        """Stop the owned player or retain ownership on an unconfirmed stop."""
        was_paused = self._playback.get("state") == "paused"
        resume_error: Optional[str] = None
        if was_paused:
            try:
                self._send_signal(self._playback_process, signal.SIGCONT)
            except Exception as error:
                resume_error = str(error) or type(error).__name__

        exit_code, termination_reason = self._stop_process(
            self._playback_process
        )
        if exit_code is not None:
            return exit_code, termination_reason

        restore_error: Optional[str] = None
        if was_paused:
            try:
                self._send_signal(self._playback_process, signal.SIGSTOP)
                self._playback["state"] = "paused"
                self._playback["started_monotonic"] = None
            except Exception as error:
                restore_error = str(error) or type(error).__name__
                self._playback["state"] = "playing"
                self._playback["started_monotonic"] = _finite_time(
                    self._monotonic(),
                    "monotonic_clock",
                )

        details = [
            termination_reason or "Process termination was not confirmed."
        ]
        if resume_error:
            details.append(f"resume-before-stop failed: {resume_error}")
        if restore_error:
            details.append(f"pause restoration failed: {restore_error}")
        message = (
            f"Failed to {operation}; playback ownership was retained: "
            + "; ".join(details)
        )
        self._playback["error"] = message
        raise RecordingBackendUnavailableError(
            message,
            code="playback_stop_failed",
        )

    def _refresh_playback_locked(self) -> None:
        if self._playback.get("state") not in {"playing", "paused"}:
            return
        observed_code = self._process_poll(self._playback_process)
        if observed_code is None:
            return
        code, termination_diagnostic = self._stop_process(
            self._playback_process
        )
        if code is None:
            self._playback["error"] = termination_diagnostic
            return
        position = self._playback_position_locked()
        self._playback_process = None
        base_errors: list[str] = []
        if code != 0:
            base_errors.append(f"ros2 bag play exited with code {code}.")
        if termination_diagnostic:
            base_errors.append(termination_diagnostic)
        self._playback.update(
            state="completed" if not base_errors else "failed",
            position_seconds=(
                self._playback["duration_seconds"] if code == 0 else position
            ),
            started_monotonic=None,
            error="; ".join(base_errors) if base_errors else None,
        )
        close_error = self._close_playback_log_locked()
        errors = list(base_errors)
        if close_error:
            errors.append(
                "Playback log could not be closed: " + close_error
            )
        self._playback.update(
            state="completed" if not errors else "failed",
            position_seconds=(
                self._playback["duration_seconds"] if code == 0 else position
            ),
            started_monotonic=None,
            error="; ".join(errors) if errors else None,
        )

    def _playback_position_locked(self) -> float:
        position = float(self._playback.get("position_seconds") or 0.0)
        started = self._playback.get("started_monotonic")
        if self._playback.get("state") == "playing" and started is not None:
            elapsed = max(
                0.0,
                _finite_time(self._monotonic(), "monotonic_clock")
                - float(started),
            )
            position += elapsed * float(self._playback.get("rate") or 1.0)
        duration = float(self._playback.get("duration_seconds") or 0.0)
        return min(duration, max(0.0, position))

    def _playback_snapshot_locked(self) -> dict[str, Any]:
        snapshot = copy.deepcopy(self._playback)
        snapshot["position_seconds"] = self._playback_position_locked()
        duration = float(snapshot.get("duration_seconds") or 0.0)
        if snapshot.get("state") == "completed":
            progress = 1.0
        elif duration <= 0.0:
            progress = 0.0
        else:
            progress = min(1.0, snapshot["position_seconds"] / duration)
        snapshot["progress"] = progress
        snapshot.pop("started_monotonic", None)
        snapshot.pop("topics", None)
        return snapshot

    @staticmethod
    def _idle_playback_status() -> dict[str, Any]:
        return {
            "state": "idle",
            "recording_id": None,
            "rate": 1.0,
            "position_seconds": 0.0,
            "duration_seconds": 0.0,
            "progress": 0.0,
            "error": None,
            "topics": [],
            "started_monotonic": None,
        }

    @staticmethod
    def _process_poll(process: Any) -> Optional[int]:
        if process is None:
            return None
        poll = getattr(process, "poll", None)
        if not callable(poll):
            return None
        value = poll()
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, int):
            raise RecordingBackendUnavailableError(
                "Process poll returned an invalid return code; ownership was "
                "retained.",
                code="process_status_failed",
            )
        return value

    def _send_signal(self, process: Any, signal_number: int) -> None:
        if process is None:
            return
        if self._signal_sender is not None:
            self._signal_sender(process, signal_number)
            return
        process_id = getattr(process, "pid", None)
        if (
            isinstance(process_id, bool)
            or not isinstance(process_id, int)
            or process_id <= 0
        ):
            raise RecordingBackendUnavailableError(
                "Process handle has no valid process-group identifier.",
                code="process_status_failed",
            )
        os.killpg(process_id, signal_number)

    def _process_group_exists(self, process: Any) -> bool:
        if self._process_group_probe is not None:
            value = self._process_group_probe(process)
            if not isinstance(value, bool):
                raise RecordingBackendUnavailableError(
                    "Process-group probe returned a non-boolean result.",
                    code="process_status_failed",
                )
            return value
        process_id = getattr(process, "pid", None)
        if (
            isinstance(process_id, bool)
            or not isinstance(process_id, int)
            or process_id <= 0
        ):
            raise RecordingBackendUnavailableError(
                "Process handle has no valid process-group identifier.",
                code="process_status_failed",
            )
        try:
            os.killpg(process_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _confirm_process_group_stopped(
        self,
        process: Any,
        leader_code: int,
        errors: Sequence[str],
    ) -> tuple[Optional[int], Optional[str]]:
        """Require the dedicated process group to be empty before release."""
        try:
            group_exists = self._process_group_exists(process)
        except Exception as error:
            details = [*errors, str(error) or type(error).__name__]
            return None, (
                f"Process leader exited with code {leader_code}, but process-"
                "group termination could not be confirmed. "
                + "; ".join(details)
            )
        if not group_exists:
            return leader_code, (
                None
                if leader_code == 0
                else f"Process exited with code {leader_code}."
            )

        cleanup_errors = list(errors)
        try:
            self._send_signal(process, signal.SIGKILL)
        except Exception as error:
            cleanup_errors.append(
                "Could not send process-group SIGKILL: "
                f"{str(error) or type(error).__name__}"
            )
        deadline = time.monotonic() + _PROCESS_GROUP_CONFIRM_SECONDS
        while True:
            try:
                if not self._process_group_exists(process):
                    message = (
                        "Process group remained after its leader exited and "
                        "required SIGKILL cleanup."
                    )
                    if leader_code != 0:
                        message = (
                            f"Process exited with code {leader_code}; " + message
                        )
                    return leader_code, message
            except Exception as error:
                cleanup_errors.append(str(error) or type(error).__name__)
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(_PROCESS_GROUP_POLL_SECONDS)
        message = (
            f"Process leader exited with code {leader_code}, but its process "
            "group remains after SIGKILL; ownership was retained."
        )
        if cleanup_errors:
            message += " " + "; ".join(cleanup_errors)
        return None, message

    def _stop_process(self, process: Any) -> tuple[Optional[int], Optional[str]]:
        if process is None:
            return None, "Process handle is unavailable."
        errors: list[str] = []
        try:
            existing = self._process_poll(process)
        except Exception as error:
            existing = None
            errors.append(str(error) or type(error).__name__)
        if existing is not None:
            return self._confirm_process_group_stopped(
                process,
                existing,
                errors,
            )
        stages = (
            (signal.SIGINT, 5.0, "SIGINT"),
            (signal.SIGTERM, 2.0, "SIGTERM"),
            (signal.SIGKILL, 1.0, "SIGKILL"),
        )
        for signal_number, timeout, label in stages:
            try:
                self._send_signal(process, signal_number)
            except Exception as error:
                errors.append(
                    f"Could not send {label}: "
                    f"{str(error) or type(error).__name__}"
                )
            waiter = getattr(process, "wait", None)
            if not callable(waiter):
                errors.append("Process handle has no callable wait().")
                break
            try:
                waited_code = waiter(timeout=timeout)
                if (
                    isinstance(waited_code, int)
                    and not isinstance(waited_code, bool)
                ):
                    code = waited_code
                else:
                    code = self._process_poll(process)
                if code is None:
                    errors.append(
                        f"wait() after {label} returned without a terminal "
                        "return code."
                    )
                    continue
                return self._confirm_process_group_stopped(
                    process,
                    code,
                    errors,
                )
            except subprocess.TimeoutExpired:
                continue
            except Exception as error:
                errors.append(
                    f"wait() after {label} failed: "
                    f"{str(error) or type(error).__name__}"
                )
                try:
                    code = self._process_poll(process)
                except Exception as poll_error:
                    errors.append(str(poll_error) or type(poll_error).__name__)
                    code = None
                if code is not None:
                    return self._confirm_process_group_stopped(
                        process,
                        code,
                        errors,
                    )
        try:
            final_code = self._process_poll(process)
        except Exception as error:
            errors.append(str(error) or type(error).__name__)
            final_code = None
        if final_code is not None:
            return self._confirm_process_group_stopped(
                process,
                final_code,
                errors,
            )
        message = "Process did not terminate after SIGKILL."
        if errors:
            message += " " + "; ".join(errors)
        return None, message

    def _close_recording_log_locked(self) -> Optional[str]:
        handle = self._recording_log
        self._recording_log = None
        if handle is None:
            return None
        try:
            handle.close()
        except Exception as error:
            return str(error) or type(error).__name__
        return None

    def _close_playback_log_locked(self) -> Optional[str]:
        handle = self._playback_log
        self._playback_log = None
        if handle is None:
            return None
        try:
            handle.close()
        except Exception as error:
            return str(error) or type(error).__name__
        return None


def _validate_recording_id(value: Any) -> str:
    if not isinstance(value, str) or not _RECORDING_ID_PATTERN.fullmatch(value):
        raise RecordingValidationError(
            "recording_id must match [a-z0-9][a-z0-9_-]{0,63}.",
            code="invalid_recording_id",
        )
    return value


def _json_copy(value: Any, label: str) -> Any:
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))
    except (TypeError, ValueError) as error:
        raise RecordingValidationError(
            f"{label} must be JSON-compatible: {error}"
        ) from error


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    )
    try:
        temporary.write_text(encoded + "\n", encoding="utf-8")
        os.replace(temporary, path)
    except OSError:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def _finite_time(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RecordingValidationError(f"{label} must be a finite number.")
    result = float(value)
    if not math.isfinite(result):
        raise RecordingValidationError(f"{label} must be a finite number.")
    return result


def _nonnegative_number(value: Any, label: str) -> float:
    result = _finite_time(value, label)
    if result < 0.0:
        raise RecordingValidationError(f"{label} must not be negative.")
    return result


def _positive_rate(value: Any) -> float:
    rate = _finite_time(value, "playback rate")
    if rate < 0.1 or rate > 10.0:
        raise RecordingValidationError("Playback rate must be between 0.1 and 10.0.")
    return rate


def _sensor_topics(
    value: bool | Iterable[str],
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    if isinstance(value, bool):
        groups = tuple(SENSOR_TOPIC_GROUPS) if value else ()
    elif isinstance(value, str):
        groups = (value,)
    else:
        try:
            groups = tuple(value)
        except TypeError as error:
            raise RecordingValidationError(
                "include_sensors must be a boolean or sensor-group list."
            ) from error
    if any(not isinstance(group, str) for group in groups):
        raise RecordingValidationError("Sensor group names must be strings.")
    unknown = sorted(set(groups) - set(SENSOR_TOPIC_GROUPS))
    if unknown:
        raise RecordingValidationError(
            "Unknown sensor groups: " + ", ".join(unknown)
        )
    ordered_groups = tuple(group for group in SENSOR_TOPIC_GROUPS if group in groups)
    topics = tuple(
        topic
        for group in ordered_groups
        for topic in SENSOR_TOPIC_GROUPS[group]
    )
    return ordered_groups, topics


def _validated_record_topics(value: Iterable[str]) -> tuple[str, ...]:
    if isinstance(value, str):
        topics = (value,)
    else:
        try:
            topics = tuple(value)
        except TypeError as error:
            raise RecordingValidationError(
                "record_topics must be an iterable of absolute ROS topics."
            ) from error
    validated: list[str] = []
    for topic in topics:
        if (
            not isinstance(topic, str)
            or not _ROS_TOPIC_PATTERN.fullmatch(topic)
            or "//" in topic
        ):
            raise RecordingValidationError(
                f"Invalid read-only recording topic: {topic!r}"
            )
        if _is_dangerous_topic(topic):
            raise PlaybackSafetyError(topic)
        if topic not in validated:
            validated.append(topic)
    return tuple(validated)


def _record_topic_regex(exact_topics: Sequence[str]) -> str:
    alternatives = [re.escape(topic) for topic in exact_topics]
    alternatives.extend(_CABINET_TOPIC_PATTERNS)
    alternatives.append(_TASK_TOPIC_PATTERN)
    return "^(?:" + "|".join(alternatives) + ")$"


def _scenario_task_summaries(
    scenario: Mapping[str, Any],
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for step in scenario.get("steps", []):
        summaries.append(
            {
                "task_id": step.get("recorded_task_id"),
                "type": step.get("type"),
                "request": copy.deepcopy(step.get("request", {})),
                "outcome": step.get("recorded_outcome"),
                "result": copy.deepcopy(step.get("recorded_result", {})),
                "failure_code": step.get("recorded_failure_code"),
                "failure_reason": step.get("recorded_failure_reason"),
            }
        )
    return summaries


def _is_dangerous_topic(topic: str) -> bool:
    components = {part.lower() for part in topic.split("/") if part}
    if components & _DANGEROUS_TOPIC_COMPONENTS:
        return True
    if any(
        "cmd_vel" in component or "joint_trajectory" in component
        for component in components
    ):
        return True
    lowered = topic.lower()
    return (
        "/_action/goal" in lowered
        or lowered.endswith("/goal")
        or "follow_joint_trajectory" in lowered
        or lowered.endswith("/commands")
    )


def _read_rosbag_metadata(path: Path) -> dict[str, Any]:
    """Parse the rosbag2 metadata schema and return every declared topic."""
    try:
        if path.stat().st_size > _MAX_METADATA_BYTES:
            raise RecordingValidationError(
                "Rosbag metadata exceeds the size limit.",
                code="metadata_corrupted",
            )
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise RecordingValidationError(
            f"Cannot read rosbag metadata: {error}",
            code="metadata_corrupted",
        ) from error
    try:
        document = yaml.load(text, Loader=_UniqueKeySafeLoader)
    except (yaml.YAMLError, RecursionError) as error:
        raise RecordingValidationError(
            f"Rosbag metadata YAML is invalid: {error}",
            code="metadata_corrupted",
        ) from error
    if not isinstance(document, dict):
        raise RecordingValidationError(
            "Rosbag metadata root must be a mapping.",
            code="metadata_corrupted",
        )
    information = document.get("rosbag2_bagfile_information")
    if not isinstance(information, dict):
        raise RecordingValidationError(
            "Rosbag metadata has no rosbag2_bagfile_information mapping.",
            code="metadata_corrupted",
        )
    duration = information.get("duration")
    if not isinstance(duration, dict):
        raise RecordingValidationError(
            "Rosbag metadata has no valid duration mapping.",
            code="metadata_corrupted",
        )
    duration_ns = duration.get("nanoseconds")
    if (
        isinstance(duration_ns, bool)
        or not isinstance(duration_ns, int)
        or duration_ns < 0
        or duration_ns > _MAX_DURATION_NANOSECONDS
    ):
        raise RecordingValidationError(
            "Rosbag metadata duration.nanoseconds is invalid.",
            code="metadata_corrupted",
        )

    topic_entries = information.get("topics_with_message_count")
    if not isinstance(topic_entries, list):
        raise RecordingValidationError(
            "Rosbag metadata topics_with_message_count must be a list.",
            code="metadata_corrupted",
        )
    topics: list[str] = []
    for index, entry in enumerate(topic_entries):
        if not isinstance(entry, dict):
            raise RecordingValidationError(
                f"Rosbag metadata topic entry {index} must be a mapping.",
                code="metadata_corrupted",
            )
        message_count = entry.get("message_count")
        if (
            isinstance(message_count, bool)
            or not isinstance(message_count, int)
            or message_count < 0
        ):
            raise RecordingValidationError(
                f"Rosbag metadata topic entry {index} has an invalid "
                "message_count.",
                code="metadata_corrupted",
            )
        topic_metadata = entry.get("topic_metadata")
        if not isinstance(topic_metadata, dict):
            raise RecordingValidationError(
                f"Rosbag metadata topic entry {index} has no valid "
                "topic_metadata mapping.",
                code="metadata_corrupted",
            )
        topic = topic_metadata.get("name")
        if (
            not isinstance(topic, str)
            or not _ROS_TOPIC_PATTERN.fullmatch(topic)
            or "//" in topic
        ):
            raise RecordingValidationError(
                f"Rosbag metadata contains an invalid topic name: {topic!r}",
                code="metadata_corrupted",
            )
        for field in ("type", "serialization_format"):
            value = topic_metadata.get(field)
            if not isinstance(value, str) or not value.strip():
                raise RecordingValidationError(
                    f"Rosbag metadata topic {topic!r} has an invalid {field}.",
                    code="metadata_corrupted",
                )
        topics.append(topic)

    if len(topics) != len(set(topics)):
        raise RecordingValidationError(
            "Rosbag metadata contains duplicate topic names.",
            code="metadata_corrupted",
        )

    relative_files = information.get("relative_file_paths")
    if not isinstance(relative_files, list) or not relative_files:
        raise RecordingValidationError(
            "Rosbag metadata relative_file_paths must be a nonempty list.",
            code="metadata_corrupted",
        )
    bag_directory = path.parent.resolve(strict=True)
    storage_files: list[str] = []
    for index, value in enumerate(relative_files):
        if not isinstance(value, str) or not value.strip():
            raise RecordingValidationError(
                f"Rosbag storage path {index} must be a nonempty string.",
                code="metadata_corrupted",
            )
        relative_path = Path(value)
        if (
            relative_path.is_absolute()
            or not relative_path.parts
            or relative_path == Path(".")
            or ".." in relative_path.parts
        ):
            raise RecordingValidationError(
                f"Rosbag storage path is unsafe: {value!r}",
                code="metadata_corrupted",
            )
        normalized = relative_path.as_posix()
        if normalized in storage_files:
            raise RecordingValidationError(
                f"Rosbag metadata contains duplicate storage path: {value!r}",
                code="metadata_corrupted",
            )
        candidate = bag_directory
        for component in relative_path.parts:
            candidate = candidate / component
            if candidate.is_symlink():
                raise RecordingValidationError(
                    f"Rosbag storage path must not contain symlinks: {value!r}",
                    code="metadata_corrupted",
                )
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(bag_directory)
        except (FileNotFoundError, OSError, ValueError) as error:
            raise RecordingValidationError(
                f"Rosbag storage file is missing or unsafe: {value!r}",
                code="metadata_corrupted",
            ) from error
        if not resolved.is_file():
            raise RecordingValidationError(
                f"Rosbag storage path is not a regular file: {value!r}",
                code="metadata_corrupted",
            )
        storage_files.append(normalized)
    return {
        "duration_seconds": duration_ns / 1_000_000_000.0,
        "topics": topics,
        "storage_files": storage_files,
    }


def _validate_scenario(value: Any, recording_id: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RecordingValidationError(
            "Scenario must be an object.",
            code="scenario_corrupted",
        )
    schema_version = value.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version not in {1, 2}
        or value.get("recording_id") != recording_id
    ):
        raise RecordingValidationError(
            "Scenario schema or recording identity is invalid.",
            code="scenario_corrupted",
        )
    steps = value.get("steps")
    if not isinstance(steps, list):
        raise RecordingValidationError(
            "Scenario steps must be a list.",
            code="scenario_corrupted",
        )
    allowed_types = (
        {"navigate", "operate"}
        if schema_version == 1
        else {"navigate", "operate", "reset"}
    )
    for index, step in enumerate(steps):
        if (
            not isinstance(step, dict)
            or step.get("type") not in allowed_types
            or not isinstance(step.get("request"), dict)
        ):
            raise RecordingValidationError(
                f"Scenario step {index} is invalid.",
                code="scenario_corrupted",
            )
        if step.get("type") == "reset":
            request = step["request"]
            if (
                set(request) != {"cabinet"}
                or not isinstance(request.get("cabinet"), str)
                or not request["cabinet"].strip()
            ):
                raise RecordingValidationError(
                    f"Scenario reset step {index} is invalid.",
                    code="scenario_corrupted",
                )
    return _json_copy(value, "scenario")


def _format_number(value: float) -> str:
    return format(float(value), ".12g")
