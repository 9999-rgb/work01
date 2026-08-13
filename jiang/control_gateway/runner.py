"""Lifecycle and task orchestration for the ROS 2 Web control gateway."""

from __future__ import annotations

import ipaddress
import math
import queue
import threading
import time
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Tuple
from urllib.parse import urlsplit

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .cabinet_client import CabinetClient
from .cabinet_client import CabinetClientError
from .inventory import CabinetInventory
from .inventory import CabinetNotFoundError
from .inventory import InventoryError
from .inventory import NavigationStationOutOfBoundsError
from .inventory import NavigationStationSpec
from .inventory import OccupancyGridBoundary
from .profile_contract import ProfileContractError
from .profile_contract import validate_profile
from .recording_manager import RecordingError
from .recording_manager import RecordingManager
from .ros_node import ControlRequestError
from .ros_node import RosControlNode
from .robot_adapter import RobotAdapterError
from .robot_adapter import load as load_robot_adapter
from .task_manager import EventHubClosed
from .task_manager import TaskCanceledError
from .task_manager import TaskConflictError
from .task_manager import TaskExecutionError
from .task_manager import TaskManager
from .task_manager import TaskManagerClosedError
from .task_manager import TaskManagerError
from .task_manager import TaskNotFoundError
from .task_replay import TaskReplayConflictError
from .task_replay import TaskReplayError
from .task_replay import TaskReplayOrchestrator
from .task_replay import TaskReplayValidationError
from .web_server import ControlHandler


CABINET_SHUTDOWN_TIMEOUT_SEC = 15.0
NAVIGATION_SHUTDOWN_TIMEOUT_SEC = 15.0
TASK_MANAGER_SHUTDOWN_TIMEOUT_SEC = 8.0
SHUTDOWN_RECHECK_TIMEOUT_SEC = 1.0
BACKEND_SHUTDOWN_GRACE_SEC = 3.0
REQUEST_DRAIN_TIMEOUT_SEC = 8.0
NAVIGATION_TIMEOUT_SEC = 180.0
NAVIGATION_CANCEL_GRACE_SEC = 5.0
NAVIGATION_CLOCK_STALL_TIMEOUT_SEC = 30.0
NAVIGATION_FINAL_POSE_WAIT_SEC = 2.0
NAVIGATION_FINAL_POSE_MAX_AGE_SEC = 1.0
# TF and /clock travel through independent DDS subscriptions.  A freshly
# queried transform can therefore be stamped one publication ahead of this
# node's latest clock sample without indicating a simulation-time reset.
NAVIGATION_FINAL_POSE_MAX_FUTURE_SKEW_SEC = 0.10
NAVIGATION_REJECT_RETRY_LIMIT = 10
NAVIGATION_REJECT_RETRY_DELAY_SEC = 0.50
# A cabinet's map pose can move slightly while Nav2 is driving because AMCL
# keeps refining map->odom. Refresh the cabinet-local station after arrival,
# record meaningful drift, and correct only a real base-to-station handoff
# error so localization noise cannot manufacture extra motion.
NAVIGATION_STATION_DRIFT_POSITION_M = 0.02
NAVIGATION_STATION_DRIFT_YAW_RAD = 0.02
NAVIGATION_LOCALIZATION_JUMP_POSITION_M = 0.50
NAVIGATION_LOCALIZATION_JUMP_YAW_RAD = 0.35
NAVIGATION_STATION_CORRECTION_LIMIT = 2
# The cabinet operator's docking controller takes over after handoff and
# refines the pose to its own tighter tolerances (0.015 m / 0.10 rad).
# The handoff tolerances must accept Nav2's actual stopping accuracy so
# that navigation can succeed; the docking phase then narrows the gap.
NAVIGATION_STATION_HANDOFF_POSITION_TOLERANCE_M = 0.15
NAVIGATION_STATION_HANDOFF_YAW_TOLERANCE_RAD = 0.25
OPERATION_TIMEOUT_SEC = 180.0
OPERATION_CANCEL_GRACE_SEC = 5.0
TASK_MONITOR_PERIOD_SEC = 0.10
EXECUTOR_SPIN_PERIOD_SEC = 0.10
NAVIGATION_POSITION_TOLERANCE_M = 0.15
# Nav2's 0.25 rad goal checker and the following TF sample can differ by a
# fraction of a degree while the base settles.  Keep a 5 mrad verification
# margin without relaxing the controller's own stopping criterion.
NAVIGATION_YAW_TOLERANCE_RAD = 0.255
# One map cell and the Nav2 progress checker's required movement are both
# 0.05 m.  Treat smaller cross-axis differences as localization/grid noise;
# asking Nav2 to settle them as a separate leg forces a pointless 90-degree
# corner rotation while the final goal already verifies the exact station.
NAVIGATION_AXIS_EPSILON_M = 0.05
RESET_JOINT_STATE_POLL_SEC = 0.05
MAP_STATION_MARGIN_M = 0.05


class ControlServer:
    """Run manual control and multi-cabinet task clients behind HTTP."""

    def __init__(
        self,
        port: int = 8090,
        cmd_vel_topic: Optional[str] = None,
        joint_trajectory_topic: Optional[str] = None,
        host: str = "127.0.0.1",
        max_linear_speed: float = 0.25,
        max_angular_speed: float = 0.60,
        command_timeout: float = 0.30,
        cabinet_instances_path: Optional[str] = None,
        cabinet_scene_path: Optional[str] = None,
        cabinet_robot_adapter_path: Optional[str] = None,
        allowed_origins: Optional[Iterable[str]] = None,
        cabinet_controls_path: Optional[str] = None,
        cabinet_pose_path: Optional[str] = None,
        robot_control_path: Optional[str] = None,
        recordings_root: Optional[str] = None,
    ) -> None:
        if (
            isinstance(port, bool)
            or not isinstance(port, int)
            or not 1 <= port <= 65535
        ):
            raise ValueError("port must be an integer in [1, 65535].")
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host must be a non-empty string.")
        for label, value in (
            ("max_linear_speed", max_linear_speed),
            ("max_angular_speed", max_angular_speed),
            ("command_timeout", command_timeout),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ValueError(f"{label} must be a positive finite number.")
        if not self._is_loopback_host(host):
            # FastAPI 已启用 JWT 鉴权中间件，允许非 loopback 绑定。
            # 生产部署建议配合反向代理（nginx）做 TLS 终止与额外访问控制。
            pass
        if allowed_origins is None:
            allowed_origins = (
                f"http://localhost:{port}",
                f"http://127.0.0.1:{port}",
            )
        normalized_origin_values = set()
        for origin in allowed_origins:
            if not isinstance(origin, str) or not origin.strip():
                continue
            normalized_origin = origin.strip().rstrip("/")
            parsed_origin = urlsplit(normalized_origin)
            if (
                parsed_origin.scheme not in {"http", "https"}
                or not parsed_origin.netloc
                or parsed_origin.path
                or parsed_origin.query
                or parsed_origin.fragment
            ):
                raise ValueError(
                    f"Invalid Web control origin: {origin!r}."
                )
            normalized_origin_values.add(normalized_origin)
        normalized_origins = frozenset(normalized_origin_values)
        if not normalized_origins:
            raise ValueError("At least one Web control origin is required.")
        workspace = Path(__file__).resolve().parents[2]
        control_config = workspace / "xczs_inspection_robot_control" / "config"
        instances_path = cabinet_instances_path or str(
            control_config / "cabinet_instances.yaml"
        )
        controls_path = cabinet_controls_path or str(
            control_config / "cabinet_controls.yaml"
        )
        scene_path = cabinet_scene_path or str(
            control_config / "cabinet_scene.yaml"
        )
        pose_path = cabinet_pose_path or str(
            control_config / "cabinet_pose.yaml"
        )
        robot_adapter_path = cabinet_robot_adapter_path or str(
            control_config / "cabinet_robot_adapter.yaml"
        )
        control_path = robot_control_path or str(
            control_config / "robot_control.yaml"
        )
        recordings_path = recordings_root or str(workspace / "recordings")
        try:
            validate_profile(
                robot_adapter_path=robot_adapter_path,
                instances_path=instances_path,
                controls_path=controls_path,
                scene_path=scene_path,
                pose_path=pose_path,
            )
        except ProfileContractError as error:
            raise RuntimeError(
                f"Invalid robot/cabinet profile contract: {error}"
            ) from error
        try:
            self._inventory = CabinetInventory.load(instances_path, scene_path)
        except InventoryError as error:
            raise RuntimeError(
                f"Invalid cabinet inventory: {error}"
            ) from error
        try:
            self._robot_adapter = load_robot_adapter(robot_adapter_path)
        except RobotAdapterError as error:
            raise RuntimeError(f"Invalid robot adapter: {error}") from error
        resolved_cmd_vel_topic = (
            cmd_vel_topic
            if cmd_vel_topic is not None
            else self._robot_adapter.manual_cmd_vel_topic
        )
        resolved_joint_trajectory_topic = (
            joint_trajectory_topic
            if joint_trajectory_topic is not None
            else self._robot_adapter.joint_trajectory_topic
        )
        self._manual_cmd_vel_topic = resolved_cmd_vel_topic
        self._joint_trajectory_topic = resolved_joint_trajectory_topic

        self._port = port
        self._host = host.strip()
        self._allowed_origins = normalized_origins
        self._context = Context()
        # The process entry point owns SIGINT/SIGTERM and requests an ordered
        # shutdown through ``stop()``.  Explicitly keep rclpy from shutting
        # this private context down underneath the executor thread.
        rclpy.init(
            context=self._context,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        self._node = RosControlNode(
            resolved_cmd_vel_topic,
            resolved_joint_trajectory_topic,
            max_linear_speed,
            max_angular_speed,
            command_timeout,
            self._context,
            navigation_frame=self._robot_adapter.navigation_frame,
            navigation_base_frame=self._robot_adapter.navigation_base_frame,
            navigation_action=self._robot_adapter.navigation_action,
            navigation_mode_service=(
                self._robot_adapter.navigation_mode_service
            ),
            navigation_mode_topic=self._robot_adapter.navigation_mode_topic,
            map_topic=self._robot_adapter.map_topic,
            localization_pose_topic=(
                self._robot_adapter.localization_pose_topic
            ),
            manual_linear_axis=self._robot_adapter.manual_linear_axis,
            manual_joints=self._robot_adapter.manual_joints,
            joint_state_topic=self._robot_adapter.joint_state_topic,
            cabinet_pose_valid_topics={
                cabinet.name: self._inventory.pose_valid_topic_for(cabinet.name)
                for cabinet in self._inventory
            },
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        # Serialize task admission/cancellation with legacy mutating routes.
        # TaskManager's own lock protects its records, but it cannot make the
        # following ROS side effect atomic with an active-task check.
        self._task_interlock_lock = threading.RLock()

        self._operation_bindings_lock = threading.RLock()
        self._operation_event_queues: Dict[
            str,
            "queue.Queue[Dict[str, Any]]",
        ] = {}
        self._cabinet_clients: Dict[str, CabinetClient] = {}
        for cabinet in self._inventory:
            client = CabinetClient(
                cabinet.name,
                self._cabinet_event_listener,
                context=self._context,
            )
            self._cabinet_clients[cabinet.name] = client
            self._executor.add_node(client)

        self._task_manager = TaskManager()
        self._replay_internal = threading.local()
        self._recording_manager = RecordingManager(
            recordings_path,
            workspace_root=workspace,
            config_paths=(
                instances_path,
                controls_path,
                scene_path,
                pose_path,
                robot_adapter_path,
                control_path,
            ),
            inventory_snapshot=self._inventory.list_cabinets(
                include_station=True
            ),
            record_topics=(
                self._robot_adapter.map_topic,
                self._robot_adapter.localization_pose_topic,
                self._robot_adapter.joint_state_topic,
            ),
        )
        self._task_replay = TaskReplayOrchestrator(
            load_scenario=self._recording_manager.load_scenario,
            submit_navigation=self._replay_submit_navigation,
            submit_operation=self._replay_submit_operation,
            submit_reset=self._replay_submit_reset,
            task_status=self._replay_task_status,
            cancel_task=self._replay_cancel_task,
        )
        self._event_bridge_subscription = self._task_manager.events.subscribe(
            last_event_id=0
        )
        self._event_bridge_thread = threading.Thread(
            target=self._bridge_task_events,
            name="web-control-task-event-bridge",
            daemon=True,
        )
        self._event_bridge_thread.start()

        self._executor_stop_event = threading.Event()
        self._executor_thread = threading.Thread(
            target=self._spin_executor,
            name="web-control-ros-executor",
            daemon=True,
        )
        self._http_server: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None
        self._request_condition = threading.Condition()
        self._active_requests = 0
        self._stopping = False
        self._shutdown_lock = threading.Lock()
        self._ros_teardown_completed = False
        self._shutdown_report: Optional[Dict[str, Any]] = None

    def start(self, *, start_http: bool = True) -> "ControlServer":
        """Start the ROS executor (and optionally the threaded HTTP listener).

        ``start_http=False`` 时由外部 HTTP 框架（FastAPI/uvicorn）接管路由，
        本方法只启动 ROS executor 线程与任务事件桥；``stop()`` 已对未启动的
        HTTP server 做空操作，因此两种模式共用同一套关闭流程。
        """
        if start_http:
            handler = type(
                "_BoundHandler",
                (ControlHandler,),
                {
                    "control_server": self,
                    "allowed_origins": self._allowed_origins,
                },
            )
            self._http_server = ThreadingHTTPServer(
                (self._host, self._port),
                handler,
            )
            self._http_server.daemon_threads = True
            self._http_thread = threading.Thread(
                target=self._http_server.serve_forever,
                name="web-control-http-server",
                daemon=True,
            )
        self._executor_thread.start()
        if start_http:
            self._http_thread.start()
        return self

    def _spin_executor(self) -> None:
        """Spin until the server requests a clean executor-thread exit."""
        while self._context.ok() and not self._executor_stop_event.is_set():
            self._executor.spin_once(timeout_sec=EXECUTOR_SPIN_PERIOD_SEC)

    @staticmethod
    def _is_loopback_host(host: str) -> bool:
        if not isinstance(host, str) or not host.strip():
            return False
        normalized = host.strip().lower()
        if normalized == "localhost":
            return True
        try:
            return ipaddress.ip_address(normalized).is_loopback
        except ValueError:
            return False

    def stop(self) -> None:
        """Stop streams, HTTP, action clients, and finally the ROS context."""
        # A first bounded attempt may deliberately leave ROS alive while an
        # action worker or request is still retiring.  Serialize attempts, but
        # allow a later call to finish teardown once that owner has stopped.
        shutdown_lock = getattr(self, "_shutdown_lock", None)
        if shutdown_lock is None:
            with self._request_condition:
                shutdown_lock = getattr(self, "_shutdown_lock", None)
                if shutdown_lock is None:
                    shutdown_lock = threading.Lock()
                    self._shutdown_lock = shutdown_lock
        with shutdown_lock:
            if getattr(self, "_ros_teardown_completed", False):
                return
            with self._request_condition:
                self._stopping = True
                self._request_condition.notify_all()

            task_replay = getattr(self, "_task_replay", None)
            task_replay_stopped = True
            if task_replay is not None:
                try:
                    task_replay_stopped = task_replay.shutdown(timeout=5.0)
                except Exception as error:  # noqa: BLE001
                    task_replay_stopped = False
                    self._log_shutdown_warning(
                        f"Failed to stop task replay cleanly: {error}"
                    )
                if not task_replay_stopped:
                    self._log_shutdown_warning(
                        "The task replay worker did not stop before teardown."
                    )

            recording_manager = getattr(self, "_recording_manager", None)
            if recording_manager is not None:
                try:
                    if recording_manager.playback_status().get("state") in {
                        "playing",
                        "paused",
                    }:
                        recording_manager.cancel_playback()
                except Exception as error:  # noqa: BLE001
                    self._log_shutdown_warning(
                        f"Failed to stop isolated data playback: {error}"
                    )

            task_manager = getattr(self, "_task_manager", None)
            task_shutdown: Dict[str, Any] = {
                "pending_worker_task_ids": [],
                "pending_cancellation_task_ids": [],
                "workers_stopped": True,
            }
            if task_manager is not None:
                # Keep SSE and the ROS task-event bridge alive until workers
                # have published their final honest state. Runner loops
                # observe shutdown and bound an unresponsive backend wait.
                task_shutdown = task_manager.shutdown(
                    timeout=TASK_MANAGER_SHUTDOWN_TIMEOUT_SEC,
                    cancel_active=True,
                )
                if task_shutdown["pending_worker_task_ids"]:
                    self._log_shutdown_warning(
                        "Task workers did not stop before the bounded "
                        "shutdown deadline: "
                        + ", ".join(
                            task_shutdown["pending_worker_task_ids"]
                        )
                    )
                if task_shutdown["pending_cancellation_task_ids"]:
                    self._log_shutdown_warning(
                        "Backend cancellation callbacks did not stop before "
                        "the bounded shutdown deadline: "
                        + ", ".join(
                            task_shutdown["pending_cancellation_task_ids"]
                        )
                    )
                task_manager.events.close()
            event_bridge_stopped = self._join_thread_for_shutdown(
                getattr(self, "_event_bridge_thread", None),
                timeout=3.0,
            )
            if not event_bridge_stopped:
                self._log_shutdown_warning(
                    "The task event bridge did not stop before teardown."
                )

            recording_manager_stopped = True
            if recording_manager is not None:
                try:
                    recording_manager.shutdown()
                    recording_manager_stopped = (
                        recording_manager.active_mode == "idle"
                    )
                except Exception as error:  # noqa: BLE001
                    recording_manager_stopped = False
                    self._log_shutdown_warning(
                        f"Failed to finalize recording/replay data: {error}"
                    )

            if self._http_server is not None:
                self._http_server.shutdown()
                self._http_server.server_close()
                self._http_server = None
            http_thread_stopped = self._join_thread_for_shutdown(
                getattr(self, "_http_thread", None),
                timeout=3.0,
            )
            if http_thread_stopped:
                self._http_thread = None
            else:
                self._log_shutdown_warning(
                    "The HTTP listener did not stop before teardown."
                )

            # Request handlers are daemon threads, so server_close() does not
            # join them. Drain handlers that entered before the shutdown gate.
            request_deadline = time.monotonic() + REQUEST_DRAIN_TIMEOUT_SEC
            with self._request_condition:
                while self._active_requests > 0:
                    remaining = request_deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self._request_condition.wait(timeout=remaining)
                pending_requests = self._active_requests
            if pending_requests:
                self._log_shutdown_warning(
                    f"{pending_requests} Web request(s) did not drain before "
                    "the bounded shutdown deadline."
                )
            else:
                # A request that entered before ``_stopping`` was set may
                # finish starting a replay after the first shutdown pass.
                # Once all admitted handlers have drained, no new request can
                # cross the gate, so repeat shutdown for any newly acquired
                # owner before ROS resources are destroyed.
                if task_replay is not None and task_replay.is_active:
                    try:
                        task_replay_stopped = task_replay.shutdown(timeout=5.0)
                    except Exception as error:  # noqa: BLE001
                        task_replay_stopped = False
                        self._log_shutdown_warning(
                            "Failed to stop a task replay admitted during "
                            f"gateway shutdown: {error}"
                        )
                if (
                    recording_manager is not None
                    and recording_manager.active_mode != "idle"
                ):
                    try:
                        recording_manager.shutdown()
                        recording_manager_stopped = (
                            recording_manager.active_mode == "idle"
                        )
                    except Exception as error:  # noqa: BLE001
                        recording_manager_stopped = False
                        self._log_shutdown_warning(
                            "Failed to stop recording/replay work admitted "
                            f"during gateway shutdown: {error}"
                        )

            cabinet_clients = getattr(self, "_cabinet_clients", None)
            pending = set()
            legacy_cabinet_pending = False
            if cabinet_clients:
                for client in cabinet_clients.values():
                    try:
                        client.cancel()
                    except Exception:  # noqa: BLE001 - best-effort shutdown
                        continue
                deadline = time.monotonic() + CABINET_SHUTDOWN_TIMEOUT_SEC
                pending = set(cabinet_clients)
                while pending and time.monotonic() < deadline:
                    still_active = set()
                    for name in pending:
                        try:
                            active = cabinet_clients[name].snapshot_status().get(
                                "active"
                            )
                        except Exception:  # noqa: BLE001
                            active = True
                        if active:
                            still_active.add(name)
                    pending = still_active
                    if pending:
                        time.sleep(0.05)
                if pending:
                    self._log_shutdown_warning(
                        "Timed out waiting for cabinet operation termination "
                        "during Web gateway shutdown: "
                        + ", ".join(sorted(pending))
                    )
            else:
                # Compatibility for lifecycle tests and old single-node setups.
                cancel_cabinet = getattr(
                    self._node,
                    "cancel_cabinet_operation",
                    self._node.cancel_cabinet_button,
                )
                cabinet_cancel = cancel_cabinet(allow_idle=True)
                if (
                    cabinet_cancel["status"] == "canceling"
                    and not self._node.wait_for_cabinet_idle(
                        timeout_sec=CABINET_SHUTDOWN_TIMEOUT_SEC
                    )
                ):
                    legacy_cabinet_pending = True
                    self._log_shutdown_warning(
                        "Timed out waiting for the cabinet operation to "
                        "cancel during Web gateway shutdown."
                    )

            self._node.cancel_navigation(allow_idle=True)
            navigation_pending = False
            navigation_snapshot = getattr(
                self._node,
                "navigation_snapshot",
                None,
            )
            if callable(navigation_snapshot):
                navigation_deadline = (
                    time.monotonic() + NAVIGATION_SHUTDOWN_TIMEOUT_SEC
                )
                while True:
                    try:
                        state = navigation_snapshot()
                        navigation_pending = (
                            str(state.get("state", ""))
                            in RosControlNode.ACTIVE_NAVIGATION_STATES
                            or int(state.get("retiring_goals", 0)) > 0
                        )
                    except Exception:  # noqa: BLE001
                        navigation_pending = True
                    if not navigation_pending:
                        break
                    if time.monotonic() >= navigation_deadline:
                        self._log_shutdown_warning(
                            "Timed out waiting for Nav2 to confirm that all "
                            "goals stopped during Web gateway shutdown."
                        )
                        break
                    time.sleep(0.05)
            self._node.emergency_stop()

            # Earlier bounded waits may have timed out even though their
            # workers reached terminal state while HTTP, cabinets, or Nav2
            # were being drained.  Recompute every owner immediately before
            # the teardown decision so a stale timeout snapshot cannot keep
            # an otherwise clean gateway half-shut down.
            if task_replay is not None:
                try:
                    task_replay_stopped = task_replay.shutdown(
                        timeout=SHUTDOWN_RECHECK_TIMEOUT_SEC
                    )
                except Exception as error:  # noqa: BLE001
                    task_replay_stopped = False
                    self._log_shutdown_warning(
                        f"Failed to recheck task replay shutdown: {error}"
                    )
            if task_manager is not None:
                task_shutdown = task_manager.shutdown(
                    timeout=SHUTDOWN_RECHECK_TIMEOUT_SEC,
                    cancel_active=True,
                )
            if recording_manager is not None:
                try:
                    if recording_manager.active_mode != "idle":
                        recording_manager.shutdown()
                    recording_manager_stopped = (
                        recording_manager.active_mode == "idle"
                    )
                except Exception as error:  # noqa: BLE001
                    recording_manager_stopped = False
                    self._log_shutdown_warning(
                        "Failed to recheck recording/replay shutdown: "
                        f"{error}"
                    )

            safe_to_destroy_ros = (
                task_shutdown["workers_stopped"]
                and task_shutdown.get("active_task_id") is None
                and pending_requests == 0
                and not pending
                and not legacy_cabinet_pending
                and not navigation_pending
                and event_bridge_stopped
                and http_thread_stopped
                and task_replay_stopped
                and recording_manager_stopped
            )
            self._shutdown_report = {
                "task_manager": task_shutdown,
                "pending_requests": pending_requests,
                "pending_cabinets": sorted(pending),
                "legacy_cabinet_pending": legacy_cabinet_pending,
                "navigation_pending": navigation_pending,
                "event_bridge_stopped": event_bridge_stopped,
                "http_thread_stopped": http_thread_stopped,
                "task_replay_stopped": task_replay_stopped,
                "recording_manager_stopped": recording_manager_stopped,
                "executor_stopped": False,
                "ros_teardown_completed": False,
            }
            if not safe_to_destroy_ros:
                # Python cannot kill a thread that may be inside rclpy. Keep
                # ROS alive so a later stop() attempt can finish safely.
                return

            # Humble's Executor.shutdown() destroys its internal guard
            # conditions but does not join a concurrent spin() call.  Stop
            # and join our bounded spin loop first so it cannot dereference a
            # guard after shutdown has cleared it.
            executor_stop_event = getattr(
                self,
                "_executor_stop_event",
                None,
            )
            if executor_stop_event is not None:
                executor_stop_event.set()
                self._executor.wake()
            executor_thread_stopped = self._join_thread_for_shutdown(
                getattr(self, "_executor_thread", None),
                timeout=3.0,
            )
            if not executor_thread_stopped:
                self._shutdown_report["executor_stopped"] = False
                self._log_shutdown_warning(
                    "The ROS executor thread did not stop before teardown; ROS "
                    "objects remain intact for a later stop attempt."
                )
                return
            shutdown_result = self._executor.shutdown(timeout_sec=3.0)
            executor_stopped = shutdown_result is not False
            self._shutdown_report["executor_stopped"] = executor_stopped
            if not executor_stopped:
                self._log_shutdown_warning(
                    "The ROS executor did not finish cleanup before teardown; "
                    "ROS objects remain intact for a later stop attempt."
                )
                return
            if cabinet_clients:
                for client in cabinet_clients.values():
                    client.destroy_node()
            self._node.destroy_node()
            self._context.shutdown()
            self._ros_teardown_completed = True
            self._shutdown_report["ros_teardown_completed"] = True

    @staticmethod
    def _join_thread_for_shutdown(
        thread: Optional[threading.Thread],
        *,
        timeout: float,
    ) -> bool:
        """Join a possibly unstarted/fake thread and report whether it stopped."""
        if thread is None:
            return True
        try:
            thread.join(timeout=timeout)
        except RuntimeError:
            # ``threading.Thread.join`` raises before ``start``.  Such a
            # thread cannot be accessing ROS and is already safe.
            ident = getattr(thread, "ident", None)
            if ident is None:
                return True
            raise
        is_alive = getattr(thread, "is_alive", None)
        return not bool(is_alive()) if callable(is_alive) else True

    def health(self) -> Dict[str, Any]:
        """Return gateway, Nav2, cabinet, and global task availability."""
        with self._request_scope():
            navigation = self._node.navigation_snapshot()
            try:
                self._live_map_bounds()
                map_available = True
                map_error = None
            except ControlRequestError as error:
                map_available = False
                details = dict(getattr(error, "details", {}) or {})
                map_error = str(details.get("map_error") or error)
            clients = getattr(self, "_cabinet_clients", {})
            if clients:
                cabinet_states = {
                    name: client.snapshot_status()
                    for name, client in clients.items()
                }
                cabinet_available = all(
                    bool(
                        state.get(
                            "available",
                            state.get("catalog_received")
                            and state.get("operation_available"),
                        )
                    )
                    for state in cabinet_states.values()
                )
                cabinet_active = any(
                    bool(state.get("active"))
                    for state in cabinet_states.values()
                )
            else:
                cabinet = self._node.cabinet_snapshot()
                cabinet_states = {}
                cabinet_available = bool(cabinet["available"])
                cabinet_active = bool(cabinet["active"])
            task_manager = getattr(self, "_task_manager", None)
            replay = (
                self._replay_status_snapshot()
                if getattr(self, "_recording_manager", None) is not None
                else {
                    "mode": "idle",
                    "read_only": False,
                }
            )
            return {
                "status": "ok",
                "navigation_available": navigation["available"],
                "map_available": map_available,
                "map_error": map_error,
                "cabinet_available": cabinet_available,
                "cabinet_active": cabinet_active,
                "cabinet_count": len(clients) if clients else 1,
                "active_task_id": (
                    task_manager.active_task_id
                    if task_manager is not None
                    else None
                ),
                "cabinets": cabinet_states,
                "replay_mode": replay["mode"],
                "replay_read_only": replay["read_only"],
            }

    def cabinets(self) -> Dict[str, Any]:
        """Return all validated cabinet instances and computed stations."""
        with self._request_scope():
            values = self._inventory.list_cabinets(include_station=True)
            return {"count": len(values), "cabinets": values}

    def robot_capabilities(self) -> Dict[str, Any]:
        """Return the ordered manual-control contract used by the Web UI."""
        with self._request_scope():
            adapter = self._robot_adapter
            joints = [
                {
                    "name": joint.name,
                    "group": joint.group,
                    "min_position": joint.min_position,
                    "max_position": joint.max_position,
                    "default_position": joint.default_position,
                    "open_position": joint.open_position,
                }
                for joint in adapter.manual_joints
            ]
            return {
                "schema_version": 1,
                "manual_linear_axis": adapter.manual_linear_axis,
                "frames": {
                    "planning": adapter.planning_frame,
                    "navigation": adapter.navigation_frame,
                    "navigation_base": adapter.navigation_base_frame,
                },
                "topics": {
                    "manual_cmd_vel": self._manual_cmd_vel_topic,
                    "joint_state": adapter.joint_state_topic,
                    "joint_trajectory": self._joint_trajectory_topic,
                },
                "joint_count": len(joints),
                "arm_joint_count": len(adapter.arm_joint_names),
                "gripper_joint_count": len(adapter.gripper_joint_names),
                "manual_joints": joints,
            }

    def recordings(self) -> Dict[str, Any]:
        """List retained recording manifests without exposing data paths."""
        with self._request_scope():
            values = self._call_recording_manager(
                self._recording_manager.list_recordings
            )
            return {"count": len(values), "recordings": values}

    def recording_detail(self, recording_id: str) -> Dict[str, Any]:
        """Return one validated recording manifest."""
        with self._request_scope():
            return self._call_recording_manager(
                self._recording_manager.get_recording,
                recording_id,
            )

    def recording_timeline(self, recording_id: str) -> Dict[str, Any]:
        """Return the bounded task-event timeline for one recording."""
        with self._request_scope():
            return self._call_recording_manager(
                self._recording_manager.timeline,
                recording_id,
            )

    def start_recording(
        self,
        name: Optional[str],
        include_sensors: bool,
    ) -> Dict[str, Any]:
        """Start passive rosbag2 capture; live controls remain available."""
        with self._request_scope():
            recording = self._call_recording_manager(
                self._recording_manager.start_recording,
                name,
                include_sensors=include_sensors,
            )
            snapshot = self._replay_status_snapshot()
            snapshot["recording"] = self._status_field(recording)
            return snapshot

    def stop_recording(self) -> Dict[str, Any]:
        """Stop rosbag2 capture and atomically finalize retained artifacts."""
        with self._request_scope():
            recording = self._call_recording_manager(
                self._recording_manager.stop_recording
            )
            # ``recording_status()`` intentionally reports the currently
            # active recorder.  Once finalization has released that owner it
            # returns idle, so preserve this request's terminal result (and
            # any finalization error) in the HTTP response.
            snapshot = self._replay_status_snapshot()
            snapshot["recording"] = self._status_field(recording)
            return snapshot

    def replay_status(self) -> Dict[str, Any]:
        """Return recording, isolated playback, and task-replay state."""
        with self._request_scope():
            return self._replay_status_snapshot()

    def start_data_playback(
        self,
        recording_id: str,
        rate: float,
    ) -> Dict[str, Any]:
        """Start namespace-isolated rosbag2 playback in read-only mode."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._ensure_backend_quiescent("Data playback")
                if self._task_replay.is_active:
                    raise ControlRequestError(
                        "A task replay is already active.",
                        409,
                    )
                self._quiesce_manual_outputs_for_replay()
                self._call_recording_manager(
                    self._recording_manager.start_playback,
                    recording_id,
                    rate=rate,
                )
            return self._replay_status_snapshot()

    def pause_data_playback(self) -> Dict[str, Any]:
        """Pause the isolated rosbag2 player."""
        with self._request_scope():
            self._call_recording_manager(
                self._recording_manager.pause_playback
            )
            return self._replay_status_snapshot()

    def resume_data_playback(self) -> Dict[str, Any]:
        """Resume the isolated rosbag2 player."""
        with self._request_scope():
            self._call_recording_manager(
                self._recording_manager.resume_playback
            )
            return self._replay_status_snapshot()

    def set_data_playback_rate(self, rate: float) -> Dict[str, Any]:
        """Restart isolated playback at the current position and new rate."""
        with self._request_scope():
            self._call_recording_manager(
                self._recording_manager.set_playback_rate,
                rate,
            )
            return self._replay_status_snapshot()

    def start_task_replay(self, recording_id: str) -> Dict[str, Any]:
        """Re-enact safe semantic tasks through the current control stack."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._ensure_backend_quiescent("Task replay")
                playback = self._call_recording_manager(
                    self._recording_manager.playback_status
                )
                if playback.get("state") in {"playing", "paused"}:
                    raise ControlRequestError(
                        "Isolated data playback is already active.",
                        409,
                    )
                self._quiesce_manual_outputs_for_replay()
                try:
                    self._task_replay.start(recording_id)
                except RecordingError as error:
                    raise self._recording_control_error(error) from error
                except TaskReplayConflictError as error:
                    raise ControlRequestError(str(error), 409) from error
                except TaskReplayValidationError as error:
                    raise ControlRequestError(str(error), 400) from error
                except TaskReplayError as error:
                    raise ControlRequestError(str(error), 400) from error
            return self._replay_status_snapshot()

    def cancel_replay(self) -> Dict[str, Any]:
        """Cancel whichever read-only playback or task replay owns control."""
        with self._request_scope():
            with self._task_interlock_lock:
                if self._task_replay.is_active:
                    self._task_replay.cancel()
                else:
                    self._call_recording_manager(
                        self._recording_manager.cancel_playback
                    )
            return self._replay_status_snapshot()

    def cabinet_controls(self, name: Optional[str] = None) -> Dict[str, Any]:
        """Return one instance's catalog and live physical state."""
        with self._request_scope():
            clients = getattr(self, "_cabinet_clients", None)
            if clients:
                if name is None:
                    name, client = self._legacy_cabinet_client()
                else:
                    client = self._client_for(name)
                snapshot = client.snapshot_controls()
                return {"cabinet": name, **snapshot}
            # Old lifecycle/API compatibility; new servers always use clients.
            return self._node.cabinet_controls_snapshot()

    def submit_navigation_task(
        self,
        cabinet: str,
        control_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Drive to the cabinet or a robot-calibrated control station."""
        with self._request_scope():
            try:
                cabinet_instance = self._inventory.get(cabinet)
                control_station = None
                if control_id is not None:
                    client = self._client_for(cabinet)
                    catalog = client.snapshot_controls()
                    if catalog.get("catalog_received") is not True:
                        raise ControlRequestError(
                            f"Control catalog for {cabinet} is not ready.",
                            503,
                        )
                    known_ids = {
                        str(control.get("control_id", ""))
                        for control in catalog.get("controls", ())
                        if isinstance(control, Mapping)
                    }
                    if control_id not in known_ids:
                        raise ControlRequestError(
                            f"Unknown cabinet control: {control_id}",
                            404,
                        )
                    control_station = (
                        self._robot_adapter.control_navigation_station(
                            control_id
                        )
                    )
                live_map_bounds = self._live_map_bounds()
                live_station_transform = getattr(
                    self._node,
                    "navigation_station_from_tf",
                    None,
                )
                station_refresh = None
                if callable(live_station_transform):
                    station_spec = self._inventory.station_spec_for(
                        cabinet,
                        control_station=control_station,
                    )
                    station = live_station_transform(
                        cabinet,
                        cabinet_instance.frame_id,
                        station_spec,
                    )
                    station = self._inventory.validate_station_bounds(
                        station,
                        boundary=live_map_bounds,
                        margin=MAP_STATION_MARGIN_M,
                    )
                    station_refresh = (
                        cabinet_instance.frame_id,
                        station_spec,
                    )
                else:
                    # Compatibility for isolated lifecycle/test fakes that do
                    # not own TF. Production RosControlNode always resolves
                    # the station from the latest localization transform.
                    station = self._inventory.station_for(
                        cabinet,
                        control_station=control_station,
                        boundary=live_map_bounds,
                        margin=MAP_STATION_MARGIN_M,
                    )
            except CabinetNotFoundError as error:
                raise ControlRequestError(str(error), 404) from error
            except NavigationStationOutOfBoundsError as error:
                raise ControlRequestError(str(error), 400) from error
            except InventoryError as error:
                raise ControlRequestError(str(error), 400) from error
            if station.frame_id != self._robot_adapter.navigation_frame:
                raise ControlRequestError(
                    "Navigation station frame must match the robot adapter; "
                    f"expected {self._robot_adapter.navigation_frame}, "
                    f"but {cabinet} uses {station.frame_id}.",
                    400,
                )
            request = {
                "cabinet": cabinet,
                "station": station.to_dict(),
            }
            if control_id is not None:
                request["control_id"] = control_id
            with self._task_interlock_scope():
                replay_owned = self._replay_internal_authorized()
                try:
                    return self._task_manager.submit(
                        "navigate",
                        request,
                        lambda context: self._execute_navigation_task_owned(
                            context,
                            station.to_dict(),
                            replay_owned,
                            station_refresh=station_refresh,
                        ),
                        cancel_callback=lambda _task_id: (
                            self._node.cancel_navigation()
                        ),
                        thread_name=f"xczs-navigate-{cabinet}",
                    )
                except TaskConflictError as error:
                    raise ControlRequestError(
                        str(error),
                        409,
                        details=error.details,
                    ) from error
                except TaskManagerClosedError as error:
                    raise ControlRequestError(str(error), 503) from error

    def submit_operation_task(
        self,
        cabinet: str,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Dict[str, Any]:
        """Accept a non-navigation cabinet operation task."""
        with self._request_scope():
            client = self._client_for(cabinet)
            if isinstance(force, bool):
                raise ControlRequestError("force must be a positive number.")
            try:
                force_value = None if force is None else float(force)
            except (TypeError, ValueError) as error:
                raise ControlRequestError(
                    "force must be a positive number."
                ) from error
            if force_value is not None and (
                not math.isfinite(force_value) or force_value <= 0.0
            ):
                raise ControlRequestError("force must be a positive number.")
            request = {
                "cabinet": cabinet,
                "control_id": control_id,
                "command": command,
                "target_state": target_state,
                "target_position": target_position,
                "force": force_value,
            }
            with self._task_interlock_scope():
                replay_owned = self._replay_internal_authorized()
                try:
                    return self._task_manager.submit(
                        "operate",
                        request,
                        lambda context: self._execute_operation_task_owned(
                            context,
                            cabinet,
                            client,
                            control_id,
                            command,
                            target_state,
                            target_position,
                            force_value,
                            replay_owned,
                        ),
                        cancel_callback=lambda _task_id: client.cancel(),
                        thread_name=f"xczs-operate-{cabinet}",
                    )
                except TaskConflictError as error:
                    raise ControlRequestError(
                        str(error),
                        409,
                        details=error.details,
                    ) from error
                except TaskManagerClosedError as error:
                    raise ControlRequestError(str(error), 503) from error

    def submit_reset_task(self, cabinet: str) -> Dict[str, Any]:
        """Reset one cabinet through the globally serialized task API."""
        with self._request_scope():
            client = self._client_for(cabinet)
            request = {"cabinet": cabinet}
            with self._task_interlock_scope():
                self._ensure_backend_quiescent("Scene reset")
                replay_owned = self._replay_internal_authorized()
                try:
                    return self._task_manager.submit(
                        "reset",
                        request,
                        lambda context: self._execute_reset_task_owned(
                            context,
                            cabinet,
                            client,
                            replay_owned,
                        ),
                        # Trigger services cannot be canceled after dispatch.
                        # Reject cancellation so TaskManager retains the global
                        # slot until the service call reaches a real terminal.
                        cancel_callback=lambda _task_id: False,
                        thread_name=f"xczs-reset-{cabinet}",
                    )
                except TaskConflictError as error:
                    raise ControlRequestError(
                        str(error),
                        409,
                        details=error.details,
                    ) from error
                except TaskManagerClosedError as error:
                    raise ControlRequestError(str(error), 503) from error

    def task_status(self, task_id: str) -> Dict[str, Any]:
        """Return a retained task snapshot for polling clients."""
        with self._request_scope():
            try:
                return self._task_manager.get_task(task_id)
            except TaskNotFoundError as error:
                raise ControlRequestError(str(error), 404) from error

    def cancel_task(self, task_id: str) -> Dict[str, Any]:
        """Request cancellation without claiming a terminal state early."""
        with self._request_scope():
            with self._task_interlock_scope():
                return self._cancel_managed_task(task_id)

    def subscribe_task_events(  # type: ignore[no-untyped-def]
        self,
        last_event_id: Any = None,
    ):
        """Subscribe to task events without holding the request drain gate."""
        with self._request_condition:
            if self._stopping:
                raise ControlRequestError(
                    "Web control server is stopping.",
                    503,
                )
        try:
            return self._task_manager.events.subscribe(
                last_event_id=last_event_id
            )
        except (ValueError, EventHubClosed) as error:
            status = 503 if isinstance(error, EventHubClosed) else 400
            raise ControlRequestError(str(error), status) from error

    def publish_cmd_vel(
        self,
        linear_y: float,
        angular_z: float,
    ) -> Tuple[float, float]:
        """Forward one manual base target."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"navigate", "operate", "reset"},
                    "Manual base control",
                    canceling_allowed_types={"navigate"},
                )
                return self._node.set_base_target(linear_y, angular_z)

    def publish_joint_trajectory(
        self,
        positions: List[float],
    ) -> List[float]:
        """Forward one manual joint target."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"operate", "reset"},
                    "Manual joint control",
                )
                return self._node.set_joint_target(positions)

    def navigation_status(self) -> Dict[str, Any]:
        """Return Nav2 state and overlays."""
        with self._request_scope():
            return self._node.navigation_snapshot()

    def set_navigation_mode(self, enabled: bool) -> Dict[str, Any]:
        """Switch manual/Nav2 base routing."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"navigate", "operate", "reset"},
                    "The legacy navigation-mode route",
                )
                return self._node.set_navigation_mode(enabled)

    def cancel_navigation(self) -> Dict[str, Any]:
        """Cancel active Nav2 navigation."""
        with self._request_scope():
            with self._task_interlock_scope():
                active_task = self._active_task_snapshot()
                if active_task is not None and active_task.get("type") == (
                    "navigate"
                ):
                    # Cancel the owning task, rather than only its current ROS
                    # goal.  This also closes the accepted-before-send race.
                    return self._cancel_managed_task(
                        str(active_task["task_id"]),
                    )
                self._reject_active_task_types(
                    {"reset"},
                    "The legacy navigation-cancel route",
                )
                return self._node.cancel_navigation()

    def takeover_navigation(self) -> Dict[str, Any]:
        """Cancel Nav2 and switch the base router to zero-speed manual mode."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"operate", "reset"},
                    "Navigation takeover",
                )
                active_task = self._active_task_snapshot()
                if active_task is not None and active_task.get("type") == (
                    "navigate"
                ):
                    # Keep TaskManager and Nav2 ownership in sync.  The task's
                    # cancel callback requests ordinary Nav2 cancellation;
                    # takeover below additionally changes routing to manual.
                    self._cancel_managed_task(
                        str(active_task["task_id"]),
                    )
                return self._node.takeover_navigation()

    def cabinet_status(self) -> Dict[str, Any]:
        """Compatibility status route, available only with one cabinet."""
        with self._request_scope():
            clients = getattr(self, "_cabinet_clients", None)
            if clients:
                name, client = self._legacy_cabinet_client()
                return {"cabinet": name, **client.snapshot_status()}
            return self._node.cabinet_snapshot()

    def press_cabinet_button(
        self,
        button_id: str,
        navigate_to_staging_pose: bool,
    ) -> Dict[str, Any]:
        """Compatibility direct operation, available only with one cabinet."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"navigate", "operate", "reset"},
                    "The legacy cabinet-press route",
                )
                clients = getattr(self, "_cabinet_clients", None)
                if clients:
                    _name, client = self._legacy_cabinet_client()
                    return self._call_cabinet_client(
                        client.submit_operation,
                        button_id,
                        "press",
                        force=5.0,
                        navigate=navigate_to_staging_pose,
                    )
                return self._node.press_cabinet_button(
                    button_id,
                    navigate_to_staging_pose,
                )

    def operate_cabinet_control(
        self,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        navigate_to_staging_pose: bool,
        force: float = 5.0,
    ) -> Dict[str, Any]:
        """Compatibility direct operation, available only with one cabinet."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"navigate", "operate", "reset"},
                    "The legacy cabinet-operation route",
                )
                clients = getattr(self, "_cabinet_clients", None)
                if clients:
                    _name, client = self._legacy_cabinet_client()
                    return self._call_cabinet_client(
                        client.submit_operation,
                        control_id,
                        command,
                        target_state=target_state,
                        target_position=target_position,
                        force=force,
                        navigate=navigate_to_staging_pose,
                    )
                operation = self._node.operate_cabinet_control
                try:
                    return operation(
                        control_id,
                        command,
                        target_state,
                        target_position,
                        navigate_to_staging_pose,
                        force,
                    )
                except TypeError:
                    # Older lifecycle fakes predate the optional force
                    # argument.
                    return operation(
                        control_id,
                        command,
                        target_state,
                        target_position,
                        navigate_to_staging_pose,
                    )

    def reset_cabinet_controls(self) -> Dict[str, Any]:
        """Compatibility reset route, available only with one cabinet."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"navigate", "operate", "reset"},
                    "The legacy cabinet-reset route",
                )
                clients = getattr(self, "_cabinet_clients", None)
                if clients:
                    _name, client = self._legacy_cabinet_client()
                    return self._call_cabinet_client(client.reset)
                return self._node.reset_cabinet_controls()

    def cancel_cabinet_operation(self) -> Dict[str, Any]:
        """Compatibility cancel route, available only with one cabinet."""
        with self._request_scope():
            with self._task_interlock_scope():
                active_task = self._active_task_snapshot()
                if active_task is not None and active_task.get("type") == (
                    "operate"
                ):
                    return self._cancel_managed_task(
                        str(active_task["task_id"]),
                    )
                self._reject_active_task_types(
                    {"reset"},
                    "The legacy cabinet-cancel route",
                )
                clients = getattr(self, "_cabinet_clients", None)
                if clients:
                    _name, client = self._legacy_cabinet_client()
                    return self._call_cabinet_client(client.cancel)
                cancel = getattr(
                    self._node,
                    "cancel_cabinet_operation",
                    self._node.cancel_cabinet_button,
                )
                return cancel()

    def cancel_cabinet_button(self) -> Dict[str, Any]:
        """Compatibility alias for cancel_cabinet_operation."""
        return self.cancel_cabinet_operation()

    def _client_for(self, cabinet: str) -> CabinetClient:
        try:
            self._inventory.get(cabinet)
            return self._cabinet_clients[cabinet]
        except (CabinetNotFoundError, KeyError) as error:
            raise ControlRequestError(
                f"Unknown cabinet: {cabinet}",
                404,
            ) from error

    def _legacy_cabinet_client(self) -> Tuple[str, CabinetClient]:
        if len(self._cabinet_clients) != 1:
            raise ControlRequestError(
                "The unscoped single-cabinet API is unavailable when multiple "
                "cabinet instances are configured; use "
                "/cabinets/{name}/controls "
                "or /task/operate.",
                410,
                details={"cabinets": list(self._inventory.names)},
            )
        return next(iter(self._cabinet_clients.items()))

    @staticmethod
    def _call_cabinet_client(callback: Any, *args: Any, **kwargs: Any) -> Any:
        try:
            return callback(*args, **kwargs)
        except CabinetClientError as error:
            raise ControlRequestError(
                str(error),
                error.status,
                details=error.details,
            ) from error

    def _active_task_snapshot(self) -> Optional[Dict[str, Any]]:
        """Return the active task for interlocks, if this server owns one.

        Some lifecycle regression tests deliberately construct a legacy
        ``ControlServer`` with ``object.__new__`` and no TaskManager.  Those
        instances retain the historical direct-route behavior.
        """
        task_manager = getattr(self, "_task_manager", None)
        if task_manager is None:
            return None
        active_task_id = task_manager.active_task_id
        if active_task_id is None:
            return None
        try:
            return task_manager.get_task(active_task_id)
        except TaskNotFoundError:
            # A real TaskManager updates the record and active id atomically.
            # Preserve the conservative interlock if a compatible manager
            # implementation momentarily exposes only the identifier.
            return {
                "task_id": active_task_id,
                "type": None,
                "status": "active",
            }

    def _reject_active_task_types(
        self,
        blocked_types: set[str],
        operation: str,
        *,
        canceling_allowed_types: Optional[set[str]] = None,
    ) -> None:
        """Reject a legacy mutation that conflicts with the global task."""
        active_task = self._active_task_snapshot()
        if active_task is None:
            return
        task_type = active_task.get("type")
        if task_type is not None and task_type not in blocked_types:
            return
        if (
            active_task.get("status") == "canceling"
            and task_type in (canceling_allowed_types or set())
        ):
            return
        task_id = str(active_task["task_id"])
        raise ControlRequestError(
            f"{operation} is unavailable while task {task_id} is active.",
            409,
            details={
                "active_task_id": task_id,
                "active_task_type": task_type,
                "active_task_status": active_task.get("status"),
            },
        )

    def _cancel_managed_task(self, task_id: str) -> Dict[str, Any]:
        """Cancel through TaskManager and normalize errors for HTTP callers."""
        try:
            return self._task_manager.cancel_task(task_id)
        except TaskNotFoundError as error:
            raise ControlRequestError(str(error), 404) from error
        except TaskManagerError as error:
            raise ControlRequestError(str(error), 409) from error

    def _log_shutdown_warning(self, message: str) -> None:
        try:
            self._node.get_logger().warning(message)
        except Exception:  # noqa: BLE001 - shutdown diagnostics are best effort
            return

    def _live_map_bounds(self) -> OccupancyGridBoundary:
        try:
            map_state = self._node.map_snapshot()
        except ControlRequestError as error:
            raise ControlRequestError(
                "Navigation cannot start until the occupancy map is available.",
                503,
                details={"map_error": str(error)},
            ) from error
        map_frame = map_state.get("frame_id")
        if map_frame != self._robot_adapter.navigation_frame:
            raise ControlRequestError(
                "The occupancy map frame does not match the robot navigation "
                f"frame; expected {self._robot_adapter.navigation_frame}, "
                f"received {map_frame}.",
                503,
            )
        origin = map_state.get("origin", {})
        try:
            return OccupancyGridBoundary(
                width=int(map_state["width"]),
                height=int(map_state["height"]),
                resolution=float(map_state["resolution"]),
                origin_x=float(origin["x"]),
                origin_y=float(origin["y"]),
                origin_yaw=float(origin.get("yaw", 0.0)),
            )
        except (KeyError, TypeError, ValueError, InventoryError) as error:
            raise ControlRequestError(
                f"The occupancy map metadata is invalid: {error}",
                503,
            ) from error

    def _execute_navigation_task_owned(
        self,
        context: Any,
        station: Mapping[str, Any],
        replay_owned: bool,
        *,
        station_refresh: Optional[
            Tuple[str, NavigationStationSpec]
        ] = None,
    ) -> Mapping[str, Any]:
        navigation_kwargs: Dict[str, Any] = {}
        if station_refresh is not None:
            navigation_kwargs["station_refresh"] = station_refresh
        if not replay_owned:
            return self._execute_navigation_task(
                context,
                station,
                **navigation_kwargs,
            )
        with self._replay_internal_scope():
            return self._execute_navigation_task(
                context,
                station,
                **navigation_kwargs,
            )

    def _execute_reset_task_owned(
        self,
        context: Any,
        cabinet: str,
        client: CabinetClient,
        replay_owned: bool,
    ) -> Mapping[str, Any]:
        if not replay_owned:
            return self._execute_reset_task(context, cabinet, client)
        with self._replay_internal_scope():
            return self._execute_reset_task(context, cabinet, client)

    def _execute_reset_task(
        self,
        context: Any,
        cabinet: str,
        client: CabinetClient,
    ) -> Mapping[str, Any]:
        context.raise_if_canceled()
        context.progress(
            "resetting_robot_quiescence",
            0.01,
            message="Stopping pending manual robot commands before reset.",
            data={"cabinet": cabinet, "component": "robot"},
        )
        try:
            self._quiesce_manual_outputs()
        except ControlRequestError as error:
            raise TaskExecutionError(
                str(error),
                code="robot_quiescence_failed",
                details=getattr(error, "details", {}),
                result={"cabinet": cabinet},
            ) from error
        context.progress(
            "resetting_cabinet",
            0.05,
            message=f"Resetting and verifying controls for {cabinet}.",
            data={"cabinet": cabinet, "component": "cabinet"},
        )
        try:
            cabinet_result = client.reset()
        except CabinetClientError as error:
            raise TaskExecutionError(
                str(error),
                code=error.code,
                details=error.details,
                result={"cabinet": cabinet},
            ) from error

        context.progress(
            "resetting_robot_joints",
            0.20,
            message="Returning the arm and gripper to adapter defaults.",
            data={"cabinet": cabinet, "component": "robot_joints"},
        )
        joint_result = self._reset_robot_joints(context, cabinet)
        if context.shutdown_requested:
            raise TaskExecutionError(
                "Gateway shutdown interrupted the reset before base homing.",
                code="shutdown_requested",
                result={
                    "cabinet": cabinet,
                    "components": {
                        "cabinet": dict(cabinet_result),
                        "robot_joints": joint_result,
                    },
                },
            )

        context.progress(
            "resetting_robot_base",
            0.55,
            message="Returning the robot base to its configured initial pose.",
            data={"cabinet": cabinet, "component": "robot_base"},
        )
        # Cabinet reset and legacy joint trajectories cannot be recalled once
        # dispatched, so cancellation is rejected during those phases.  Nav2
        # does support cancellation; enable it immediately before base homing.
        self._task_manager.set_cancel_callback(
            context.task_id,
            lambda _task_id: self._node.cancel_navigation(allow_idle=True),
        )
        reset_pose = self._robot_adapter.reset_base_pose
        reset_station = {
            "cabinet": cabinet,
            "frame_id": reset_pose.frame_id,
            "x": reset_pose.x,
            "y": reset_pose.y,
            "yaw": reset_pose.yaw,
            "purpose": "robot_reset",
        }
        base_result = self._execute_navigation_task(
            context,
            reset_station,
            progress_start=0.55,
            progress_span=0.44,
        )
        return {
            "cabinet": cabinet,
            "status": "reset",
            "message": "Robot and selected cabinet returned to defaults.",
            "components": {
                "cabinet": {
                    "status": str(cabinet_result.get("status") or "reset"),
                    "message": str(
                        cabinet_result.get("message")
                        or "Cabinet controls reset."
                    ),
                },
                "robot_joints": joint_result,
                "robot_base": base_result,
            },
        }

    def _reset_robot_joints(
        self,
        context: Any,
        cabinet: str,
    ) -> Dict[str, Any]:
        """Command adapter defaults and wait for fresh joint-state proof."""
        adapter = self._robot_adapter
        names = tuple(joint.name for joint in adapter.manual_joints)
        targets = tuple(
            float(joint.default_position) for joint in adapter.manual_joints
        )
        command_started = time.monotonic()
        try:
            commanded = self._node.set_joint_target(
                list(targets),
                duration_sec=float(adapter.reset_joint_duration_sec),
            )
        except ControlRequestError as error:
            raise TaskExecutionError(
                str(error),
                code=(
                    "backend_unavailable"
                    if getattr(error, "status", 500) == 503
                    else "robot_joint_reset_rejected"
                ),
                details=getattr(error, "details", {}),
                result={"cabinet": cabinet},
            ) from error

        deadline = command_started + float(adapter.reset_joint_timeout_sec)
        tolerance = float(adapter.reset_joint_tolerance)
        last_snapshot: Mapping[str, Any] = {}
        last_report_at = 0.0
        while True:
            snapshot = self._node.robot_joint_state_snapshot()
            last_snapshot = snapshot
            received_at = snapshot.get("received_monotonic")
            positions = snapshot.get("positions")
            if (
                snapshot.get("available") is True
                and isinstance(received_at, (int, float))
                and float(received_at) >= command_started
                and isinstance(positions, Mapping)
            ):
                try:
                    errors = {
                        name: abs(float(positions[name]) - target)
                        for name, target in zip(names, commanded)
                    }
                except (KeyError, TypeError, ValueError):
                    errors = {}
                if errors and max(errors.values()) <= tolerance:
                    return {
                        "status": "verified",
                        "joint_count": len(names),
                        "max_error_rad": max(errors.values()),
                        "tolerance_rad": tolerance,
                    }

            now = time.monotonic()
            if now >= deadline:
                details: Dict[str, Any] = {
                    "joint_names": list(names),
                    "target_positions": list(commanded),
                    "tolerance_rad": tolerance,
                    "timeout_seconds": float(adapter.reset_joint_timeout_sec),
                    "joint_state_available": bool(
                        last_snapshot.get("available")
                    ),
                }
                if isinstance(last_snapshot.get("positions"), Mapping):
                    details["observed_positions"] = {
                        name: last_snapshot["positions"].get(name)
                        for name in names
                    }
                raise TaskExecutionError(
                    "Robot joints did not reach their configured defaults "
                    "before the reset timeout.",
                    code="robot_joint_reset_timeout",
                    details=details,
                    result={"cabinet": cabinet},
                )
            if now - last_report_at >= 1.0:
                elapsed = max(0.0, now - command_started)
                timeout = float(adapter.reset_joint_timeout_sec)
                context.progress(
                    "resetting_robot_joints",
                    min(0.49, 0.20 + 0.29 * elapsed / timeout),
                    message="Waiting for fresh robot joint-state confirmation.",
                    data={
                        "cabinet": cabinet,
                        "component": "robot_joints",
                    },
                )
                last_report_at = now
            time.sleep(RESET_JOINT_STATE_POLL_SEC)

    def _execute_navigation_task(
        self,
        context: Any,
        station: Mapping[str, Any],
        *,
        progress_start: float = 0.0,
        progress_span: float = 1.0,
        station_refresh: Optional[
            Tuple[str, NavigationStationSpec]
        ] = None,
    ) -> Mapping[str, Any]:
        task_started = time.monotonic()
        initial_snapshot = self._node.navigation_snapshot()
        navigation_clock = self._navigation_clock_state(
            initial_snapshot,
            task_started,
        )
        current_station = dict(station)
        route_legs: List[Dict[str, Any]] = []
        final_result: Mapping[str, Any] = {}
        correction_count = 0
        station_refresh_count = 0
        last_station_drift = {"position_m": 0.0, "yaw_rad": 0.0}
        significant_station_drift = False
        while True:
            context.raise_if_canceled()
            initial_pose = self._node.navigation_snapshot().get(
                "current_pose"
            )
            legs = self._axis_navigation_legs(
                initial_pose,
                current_station,
            )
            total_distance = sum(
                float(leg["distance_m"]) for leg in legs
            )
            if total_distance <= NAVIGATION_AXIS_EPSILON_M:
                weights = [1.0 / len(legs)] * len(legs)
            else:
                weights = [
                    float(leg["distance_m"]) / total_distance
                    for leg in legs
                ]

            if station_refresh is None:
                route_progress_start = progress_start
                route_progress_span = progress_span
            elif correction_count == 0:
                route_progress_start = progress_start
                route_progress_span = progress_span * 0.80
            else:
                correction_span = (
                    progress_span
                    * 0.20
                    / NAVIGATION_STATION_CORRECTION_LIMIT
                )
                route_progress_start = (
                    progress_start
                    + progress_span * 0.80
                    + correction_span * (correction_count - 1)
                )
                route_progress_span = correction_span

            route_progress = 0.0
            for index, (leg, weight) in enumerate(zip(legs, weights)):
                final_result = self._execute_navigation_leg(
                    context,
                    leg["target"],
                    task_started=task_started,
                    progress_start=(
                        route_progress_start
                        + route_progress_span * route_progress
                    ),
                    progress_span=route_progress_span * weight,
                    route_axis=str(leg["axis"]),
                    route_leg_index=index,
                    route_leg_count=len(legs),
                    navigation_clock=navigation_clock,
                )
                # A retained timeout/shutdown terminal releases its
                # reservation only after Nav2 later confirms termination and
                # returns an empty sentinel. Never submit another axis (or a
                # refreshed correction) after that terminal path.
                if not final_result:
                    return {}
                route_record: Dict[str, Any] = {
                    "axis": leg["axis"],
                    "target": {
                        "x": leg["target"]["x"],
                        "y": leg["target"]["y"],
                        "yaw": leg["target"]["yaw"],
                    },
                }
                if correction_count > 0:
                    route_record.update(
                        {
                            "correction": True,
                            "correction_index": correction_count,
                        }
                    )
                route_legs.append(route_record)
                route_progress += weight
                if index + 1 < len(legs):
                    context.progress(
                        f"navigation_{leg['axis']}_complete",
                        min(
                            0.99,
                            route_progress_start
                            + route_progress_span * route_progress,
                        ),
                        message=(
                            f"Navigation {leg['axis'].upper()}-axis leg "
                            "completed; preparing the next axis."
                        ),
                        data={
                            "cabinet": current_station["cabinet"],
                            "route_axis": leg["axis"],
                            "route_leg_index": index + 1,
                            "route_leg_count": len(legs),
                            "correction_index": correction_count,
                        },
                    )

            if station_refresh is None:
                break

            context.raise_if_canceled()
            latest_station = self._refresh_navigation_station_for_task(
                current_station,
                station_refresh,
                task_started=task_started,
            )
            station_refresh_count += 1
            station_drift = self._planar_navigation_error(
                current_station,
                latest_station,
            )
            last_station_drift = station_drift
            drift_is_significant = (
                station_drift["position_m"]
                > NAVIGATION_STATION_DRIFT_POSITION_M
                or station_drift["yaw_rad"]
                > NAVIGATION_STATION_DRIFT_YAW_RAD
            )
            significant_station_drift = (
                significant_station_drift or drift_is_significant
            )
            if (
                station_drift["position_m"]
                > NAVIGATION_LOCALIZATION_JUMP_POSITION_M
                or station_drift["yaw_rad"]
                > NAVIGATION_LOCALIZATION_JUMP_YAW_RAD
            ):
                jump_result = dict(final_result)
                jump_result["station"] = dict(latest_station)
                jump_result["error"] = self._planar_navigation_error(
                    final_result["final_pose"],
                    latest_station,
                )
                raise TaskExecutionError(
                    "The live cabinet station jumped beyond the safe "
                    "localization-correction limit after Nav2 arrived.",
                    code="localization_jump",
                    details={
                        "station_drift": station_drift,
                        "maximum_position_jump_m": (
                            NAVIGATION_LOCALIZATION_JUMP_POSITION_M
                        ),
                        "maximum_yaw_jump_rad": (
                            NAVIGATION_LOCALIZATION_JUMP_YAW_RAD
                        ),
                    },
                    result=jump_result,
                )

            # Query again immediately after resolving the live station.  On
            # RosControlNode this refreshes map->base from TF, so the handoff
            # decision compares a current base pose with the same latest
            # cabinet-local station instead of reusing a cached Nav2 result.
            pose_refresh_started = time.monotonic()
            latest_snapshot = self._node.navigation_snapshot()
            pose_observed_at = time.monotonic()
            final_result = self._navigation_result(
                latest_station,
                latest_snapshot,
                pose_observed_at - task_started,
                minimum_pose_stamp_ros_nanoseconds=latest_snapshot.get(
                    "goal_sent_ros_nanoseconds"
                ),
                minimum_pose_received_monotonic=pose_refresh_started,
                observation_monotonic=pose_observed_at,
                expected_frame=self._robot_adapter.navigation_frame,
            )
            active_elapsed, clock_issue = self._navigation_elapsed(
                latest_snapshot,
                navigation_clock,
                pose_observed_at,
            )
            if (
                active_elapsed >= NAVIGATION_TIMEOUT_SEC
                or clock_issue is not None
            ):
                raise TaskExecutionError(
                    (
                        "The active ROS clock stopped while validating the "
                        "latest cabinet station."
                        if clock_issue is not None
                        else "Navigation timed out while validating the "
                        "latest cabinet station."
                    ),
                    code="navigation_timeout",
                    details={
                        "timeout_clock": navigation_clock["source"],
                        "timeout_cause": (
                            clock_issue or "active_time_limit"
                        ),
                        "active_duration_seconds": active_elapsed,
                        "wall_duration_seconds": (
                            pose_observed_at - task_started
                        ),
                    },
                    result=final_result,
                )
            pose_error = final_result["error"]
            current_station = latest_station

            correction_required = (
                pose_error["position_m"]
                > NAVIGATION_STATION_HANDOFF_POSITION_TOLERANCE_M
                or pose_error["yaw_rad"]
                > NAVIGATION_STATION_HANDOFF_YAW_TOLERANCE_RAD
            )
            if not correction_required:
                break
            if correction_count >= NAVIGATION_STATION_CORRECTION_LIMIT:
                raise TaskExecutionError(
                    "The live cabinet station kept moving or remained "
                    "outside the operation handoff tolerance after the "
                    "bounded navigation corrections.",
                    code="navigation_station_unstable",
                    details={
                        "correction_limit": (
                            NAVIGATION_STATION_CORRECTION_LIMIT
                        ),
                        "station_drift": station_drift,
                        "latest_pose_error": pose_error,
                        "position_tolerance_m": (
                            NAVIGATION_STATION_HANDOFF_POSITION_TOLERANCE_M
                        ),
                        "yaw_tolerance_rad": (
                            NAVIGATION_STATION_HANDOFF_YAW_TOLERANCE_RAD
                        ),
                    },
                    result=final_result,
                )

            correction_count += 1
            context.progress(
                "navigation_station_correction",
                min(
                    0.99,
                    progress_start
                    + progress_span
                    * (
                        0.80
                        + 0.20
                        * (correction_count - 1)
                        / NAVIGATION_STATION_CORRECTION_LIMIT
                    ),
                ),
                message=(
                    "The base is outside the operation handoff margin for "
                    "the latest live station; correcting its pose."
                ),
                data={
                    "cabinet": current_station["cabinet"],
                    "correction_index": correction_count,
                    "correction_limit": (
                        NAVIGATION_STATION_CORRECTION_LIMIT
                    ),
                    "station_drift": station_drift,
                    "latest_pose_error": pose_error,
                },
            )

        result = dict(final_result)
        result["station"] = dict(current_station)
        result["route"] = {
            "policy": "map_x_then_y",
            "legs": route_legs,
            "station_refresh_count": station_refresh_count,
            "correction_count": correction_count,
            "last_station_drift": last_station_drift,
            "significant_station_drift": significant_station_drift,
        }
        return result

    def _refresh_navigation_station_for_task(
        self,
        previous_station: Mapping[str, Any],
        station_refresh: Tuple[str, NavigationStationSpec],
        *,
        task_started: float,
    ) -> Dict[str, Any]:
        """Resolve and validate the latest live station after Nav2 arrival."""
        cabinet = str(previous_station.get("cabinet") or "")
        cabinet_frame, station_spec = station_refresh
        live_station_transform = getattr(
            self._node,
            "navigation_station_from_tf",
            None,
        )
        if not callable(live_station_transform):
            raise TaskExecutionError(
                "Live cabinet-station refresh became unavailable after "
                "navigation started.",
                code="navigation_station_refresh_failed",
                result={
                    "cabinet": cabinet,
                    "station": dict(previous_station),
                    "duration_seconds": time.monotonic() - task_started,
                },
            )
        try:
            station = live_station_transform(
                cabinet,
                cabinet_frame,
                station_spec,
            )
            station = self._inventory.validate_station_bounds(
                station,
                boundary=self._live_map_bounds(),
                margin=MAP_STATION_MARGIN_M,
            )
        except (ControlRequestError, InventoryError) as error:
            details = dict(getattr(error, "details", {}) or {})
            details["refresh_error"] = str(error)
            raise TaskExecutionError(
                "Navigation reached the previous station, but the latest "
                f"live cabinet station could not be resolved: {error}",
                code="navigation_station_refresh_failed",
                details=details,
                result={
                    "cabinet": cabinet,
                    "station": dict(previous_station),
                    "duration_seconds": time.monotonic() - task_started,
                },
            ) from error
        if (
            station.cabinet != cabinet
            or station.frame_id != self._robot_adapter.navigation_frame
        ):
            raise TaskExecutionError(
                "The refreshed cabinet station uses an unexpected cabinet "
                "or navigation frame.",
                code="navigation_station_refresh_failed",
                details={
                    "expected_cabinet": cabinet,
                    "actual_cabinet": station.cabinet,
                    "expected_frame": self._robot_adapter.navigation_frame,
                    "actual_frame": station.frame_id,
                },
                result={
                    "cabinet": cabinet,
                    "station": dict(previous_station),
                    "duration_seconds": time.monotonic() - task_started,
                },
            )
        return station.to_dict()

    @staticmethod
    def _planar_navigation_error(
        source: Mapping[str, Any],
        target: Mapping[str, Any],
    ) -> Dict[str, float]:
        """Return normalized planar position/yaw error between two poses."""
        delta_x = float(source["x"]) - float(target["x"])
        delta_y = float(source["y"]) - float(target["y"])
        yaw_delta = float(source["yaw"]) - float(target["yaw"])
        return {
            "position_m": math.hypot(delta_x, delta_y),
            "yaw_rad": abs(
                math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
            ),
        }

    def _execute_navigation_leg(
        self,
        context: Any,
        station: Mapping[str, Any],
        *,
        task_started: float,
        progress_start: float,
        progress_span: float,
        route_axis: str,
        route_leg_index: int,
        route_leg_count: int,
        navigation_clock: Dict[str, Any],
    ) -> Mapping[str, Any]:
        initial_distance: Optional[float] = None
        initial_pose = self._node.navigation_snapshot().get("current_pose")
        if isinstance(initial_pose, Mapping):
            try:
                initial_distance = self._planar_navigation_error(
                    initial_pose,
                    station,
                )["position_m"]
            except (KeyError, TypeError, ValueError):
                initial_distance = None
        try:
            # Cancellation and legacy control admission use the same lock so
            # an immediately canceled task cannot submit a goal afterward.
            with self._task_interlock_scope():
                context.raise_if_canceled()
                now_before_send = time.monotonic()
                elapsed_before_send = now_before_send - task_started
                active_elapsed, clock_issue = self._navigation_elapsed(
                    self._node.navigation_snapshot(),
                    navigation_clock,
                    now_before_send,
                )
                if (
                    active_elapsed >= NAVIGATION_TIMEOUT_SEC
                    or clock_issue is not None
                ):
                    raise TaskExecutionError(
                        (
                            "The active ROS clock stopped before the next "
                            "axis could start."
                            if clock_issue is not None
                            else "Navigation timed out before the next axis "
                            "could start."
                        ),
                        code="navigation_timeout",
                        details={
                            "timeout_clock": navigation_clock["source"],
                            "active_duration_seconds": active_elapsed,
                            "clock_issue": clock_issue,
                        },
                        result={
                            "cabinet": station["cabinet"],
                            "station": dict(station),
                            "duration_seconds": elapsed_before_send,
                        },
                    )
                self._node.send_navigation_goal(
                    float(station["x"]),
                    float(station["y"]),
                    float(station["yaw"]),
                )
                goal_sent_at = time.monotonic()
        except ControlRequestError as error:
            raise TaskExecutionError(
                str(error),
                code=(
                    "backend_unavailable"
                    if getattr(error, "status", 500) == 503
                    else "navigation_rejected"
                ),
                details=getattr(error, "details", {}),
            ) from error

        last_phase: Optional[str] = None
        last_progress = 0.0
        last_report_at = 0.0
        timeout_started_at: Optional[float] = None
        timeout_cause: Optional[str] = None
        timeout_reported = False
        last_timeout_cancel_at = 0.0
        final_pose_started_active: Optional[float] = None
        last_settling_report_at = 0.0
        rejection_retry_count = 0
        while True:
            snapshot = self._node.navigation_snapshot()
            state = str(snapshot.get("state", "unknown"))
            now = time.monotonic()
            elapsed = now - task_started
            active_elapsed, clock_issue = self._navigation_elapsed(
                snapshot,
                navigation_clock,
                now,
            )
            if timeout_started_at is None:
                if active_elapsed >= NAVIGATION_TIMEOUT_SEC:
                    timeout_started_at = now
                    timeout_cause = "active_time_limit"
                elif clock_issue is not None:
                    timeout_started_at = now
                    timeout_cause = clock_issue
            timed_out = timeout_started_at is not None

            if state == "succeeded":
                if final_pose_started_active is None:
                    final_pose_started_active = active_elapsed
                settling_elapsed = max(
                    0.0,
                    active_elapsed - final_pose_started_active,
                )
                settling_expired = (
                    settling_elapsed >= NAVIGATION_FINAL_POSE_WAIT_SEC
                )
                if timed_out:
                    if timeout_reported:
                        context.release_reservation(
                            backend_termination_confirmed=True,
                            details={"backend_terminal_state": state},
                        )
                        return {}
                    raise TaskExecutionError(
                        "Navigation exceeded its timeout before termination.",
                        code="navigation_timeout",
                        details={
                            "timeout_clock": navigation_clock["source"],
                            "timeout_cause": timeout_cause,
                            "active_duration_seconds": active_elapsed,
                            "wall_duration_seconds": elapsed,
                        },
                        result={
                            "cabinet": station["cabinet"],
                            "station": dict(station),
                            "duration_seconds": elapsed,
                        },
                    )
                try:
                    result = self._navigation_result(
                        station,
                        snapshot,
                        elapsed,
                        minimum_pose_stamp_ros_nanoseconds=snapshot.get(
                            "goal_sent_ros_nanoseconds"
                        ),
                        minimum_pose_received_monotonic=goal_sent_at,
                        observation_monotonic=now,
                        expected_frame=self._robot_adapter.navigation_frame,
                    )
                except TaskExecutionError as error:
                    if (
                        error.code
                        in {"final_pose_stale", "final_pose_frame_mismatch"}
                        and not timed_out
                        and not settling_expired
                    ):
                        if now - last_settling_report_at >= 1.0:
                            context.progress(
                                "navigation_settling",
                                min(
                                    0.99,
                                    progress_start + progress_span * 0.99,
                                ),
                                message=(
                                    "Nav2 reached the goal; waiting for a "
                                    "fresh localized pose."
                                ),
                                data={
                                    "cabinet": station["cabinet"],
                                    "route_axis": route_axis,
                                    "route_leg_index": route_leg_index + 1,
                                    "route_leg_count": route_leg_count,
                                    "verification_error": error.code,
                                    "settling_elapsed_seconds": (
                                        settling_elapsed
                                    ),
                                },
                            )
                            last_settling_report_at = now
                        time.sleep(TASK_MONITOR_PERIOD_SEC)
                        continue
                    raise
                errors = result["error"]
                if (
                    errors["position_m"] > NAVIGATION_POSITION_TOLERANCE_M
                    or errors["yaw_rad"] > NAVIGATION_YAW_TOLERANCE_RAD
                ):
                    if not settling_expired:
                        if now - last_settling_report_at >= 1.0:
                            context.progress(
                                "navigation_settling",
                                min(
                                    0.99,
                                    progress_start + progress_span * 0.99,
                                ),
                                message=(
                                    "Nav2 reached the goal; waiting for the "
                                    "localized pose to settle."
                                ),
                                data={
                                    "cabinet": station["cabinet"],
                                    "route_axis": route_axis,
                                    "route_leg_index": route_leg_index + 1,
                                    "route_leg_count": route_leg_count,
                                    "position_error_m": errors["position_m"],
                                    "yaw_error_rad": errors["yaw_rad"],
                                    "settling_elapsed_seconds": (
                                        settling_elapsed
                                    ),
                                },
                            )
                            last_settling_report_at = now
                        time.sleep(TASK_MONITOR_PERIOD_SEC)
                        continue
                    raise TaskExecutionError(
                        "Navigation ended outside the operation-station "
                        "position/yaw tolerance.",
                        code="pose_deviation_exceeded",
                        details={
                            "position_tolerance_m": (
                                NAVIGATION_POSITION_TOLERANCE_M
                            ),
                            "yaw_tolerance_rad": NAVIGATION_YAW_TOLERANCE_RAD,
                            "settling_window_seconds": (
                                NAVIGATION_FINAL_POSE_WAIT_SEC
                            ),
                            "settling_elapsed_seconds": settling_elapsed,
                        },
                        result=result,
                    )
                return result
            if state == "canceled":
                if timed_out:
                    timeout_result = {
                        "cabinet": station["cabinet"],
                        "station": dict(station),
                        "duration_seconds": elapsed,
                    }
                    if timeout_reported:
                        context.release_reservation(
                            backend_termination_confirmed=True,
                            details={"backend_terminal_state": state},
                        )
                        return {}
                    raise TaskExecutionError(
                        "Navigation timed out and was canceled.",
                        code="navigation_timeout",
                        result=timeout_result,
                    )
                raise TaskCanceledError("Navigation task was canceled.")
            if (
                state == "rejected"
                and not timed_out
                and rejection_retry_count < NAVIGATION_REJECT_RETRY_LIMIT
            ):
                rejection_retry_count += 1
                context.progress(
                    "navigation_waiting_for_system",
                    min(0.99, progress_start + progress_span * last_progress),
                    message=(
                        "Nav2 is still activating; retrying the rejected "
                        "goal without releasing the task reservation."
                    ),
                    data={
                        "cabinet": station["cabinet"],
                        "route_axis": route_axis,
                        "route_leg_index": route_leg_index + 1,
                        "route_leg_count": route_leg_count,
                        "retry": rejection_retry_count,
                        "retry_limit": NAVIGATION_REJECT_RETRY_LIMIT,
                    },
                )
                time.sleep(NAVIGATION_REJECT_RETRY_DELAY_SEC)
                with self._task_interlock_scope():
                    context.raise_if_canceled()
                    if context.shutdown_requested:
                        raise TaskCanceledError(
                            "Navigation task stopped during shutdown."
                        )
                    self._node.send_navigation_goal(
                        float(station["x"]),
                        float(station["y"]),
                        float(station["yaw"]),
                    )
                    goal_sent_at = time.monotonic()
                final_pose_started_active = None
                last_phase = None
                continue
            if state in {"failed", "rejected"}:
                code = (
                    "navigation_timeout"
                    if timed_out
                    else "target_unreachable"
                )
                failure_details = {
                    "nav2_state": state,
                    "goal_retries": rejection_retry_count,
                    "classification": (
                        "Nav2 does not expose enough information here to "
                        "distinguish an obstructed path from another "
                        "unreachable target."
                    ),
                }
                if timed_out:
                    failure_details.update(
                        {
                            "timeout_clock": navigation_clock["source"],
                            "timeout_cause": timeout_cause,
                            "active_duration_seconds": active_elapsed,
                            "wall_duration_seconds": elapsed,
                        }
                    )
                failure_result = {
                    "cabinet": station["cabinet"],
                    "station": dict(station),
                    "duration_seconds": elapsed,
                }
                if timed_out and timeout_reported:
                    context.release_reservation(
                        backend_termination_confirmed=True,
                        details={"backend_terminal_state": state},
                    )
                    return {}
                raise TaskExecutionError(
                    str(snapshot.get("message") or "Navigation failed."),
                    code=code,
                    details=failure_details,
                    result=failure_result,
                )

            remaining = snapshot.get("distance_remaining")
            if (
                initial_distance is None
                and isinstance(remaining, (int, float))
                and math.isfinite(float(remaining))
            ):
                initial_distance = max(0.0, float(remaining))
            progress = self._navigation_progress(
                snapshot,
                last_progress,
                initial_distance,
            )
            should_report = (
                not timeout_reported
                and (
                    state != last_phase
                    or progress > last_progress + 0.005
                    or now - last_report_at >= 1.0
                )
            )
            if should_report:
                last_progress = max(last_progress, progress)
                task_progress = min(
                    0.99,
                    progress_start + progress_span * last_progress,
                )
                context.progress(
                    f"navigation_{state}",
                    task_progress,
                    message=str(
                        snapshot.get("message") or "Navigation running."
                    ),
                    data={
                        "cabinet": station["cabinet"],
                        "distance_remaining": snapshot.get(
                            "distance_remaining"
                        ),
                        "eta_seconds": snapshot.get("eta_seconds"),
                        "recoveries": snapshot.get("recoveries", 0),
                        "route_axis": route_axis,
                        "route_leg_index": route_leg_index + 1,
                        "route_leg_count": route_leg_count,
                    },
                )
                last_phase = state
                last_report_at = now

            timed_out = timeout_started_at is not None
            cancel_for_shutdown = context.shutdown_requested
            if (
                (timed_out or cancel_for_shutdown)
                and now - last_timeout_cancel_at >= 1.0
            ):
                try:
                    self._node.cancel_navigation()
                except Exception:  # noqa: BLE001
                    # Keep the task active: a failed cancel request is not a
                    # confirmed Nav2 terminal state and must not free the lock.
                    pass
                last_timeout_cancel_at = now
            if (
                timed_out
                and not timeout_reported
                and now - timeout_started_at >= NAVIGATION_CANCEL_GRACE_SEC
            ):
                context.fail_retaining_reservation(
                    "Navigation timed out; backend termination is still "
                    "unconfirmed after the cancellation grace period.",
                    code="navigation_timeout",
                    details={
                        "backend_terminal_state": state,
                        "cancel_grace_seconds": NAVIGATION_CANCEL_GRACE_SEC,
                        "timeout_clock": navigation_clock["source"],
                        "timeout_cause": timeout_cause,
                        "active_duration_seconds": active_elapsed,
                        "wall_duration_seconds": elapsed,
                    },
                    result={
                        "cabinet": station["cabinet"],
                        "station": dict(station),
                        "duration_seconds": elapsed,
                    },
                )
                timeout_reported = True
            if (
                cancel_for_shutdown
                and context.shutdown_elapsed >= BACKEND_SHUTDOWN_GRACE_SEC
            ):
                if not timeout_reported:
                    context.fail_retaining_reservation(
                        "Gateway shutdown ended navigation monitoring before "
                        "the backend confirmed termination.",
                        code="shutdown_backend_unconfirmed",
                        details={"backend_terminal_state": state},
                        result={
                            "cabinet": station["cabinet"],
                            "station": dict(station),
                            "duration_seconds": elapsed,
                        },
                    )
                context.release_reservation(
                    backend_termination_confirmed=False,
                    details={
                        "backend_terminal_state": state,
                        "shutdown_monitoring_ended": True,
                    },
                )
                return {}
            # A user cancellation callback already sent the cancel request.
            # Keep polling until Nav2 reports a real terminal state so a new
            # globally-exclusive task cannot overlap a retiring goal.
            time.sleep(TASK_MONITOR_PERIOD_SEC)

    @staticmethod
    def _navigation_ros_time_seconds(
        snapshot: Mapping[str, Any],
    ) -> Optional[float]:
        """Return the active ROS clock sampled with the localized pose."""
        pose = snapshot.get("current_pose")
        if not isinstance(pose, Mapping):
            return None
        value = pose.get("observed_ros_nanoseconds")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        nanoseconds = float(value)
        if not math.isfinite(nanoseconds) or nanoseconds <= 0.0:
            return None
        return nanoseconds / 1_000_000_000.0

    @classmethod
    def _navigation_clock_state(
        cls,
        snapshot: Mapping[str, Any],
        wall_now: float,
    ) -> Dict[str, Any]:
        """Create one task-level timeout clock shared by every axis leg.

        Nav2 durations and TF stamps use the node's ROS clock.  In simulation
        that clock advances more slowly than wall time when Gazebo's real-time
        factor is below one, so a wall deadline would cancel healthy motion.
        Adapters without timestamped localization retain the legacy wall-clock
        behavior instead of silently losing timeout protection.
        """
        ros_now = cls._navigation_ros_time_seconds(snapshot)
        if ros_now is None:
            return {
                "source": "wall",
                "started": wall_now,
                "last": wall_now,
                "last_advanced_wall": wall_now,
            }
        return {
            "source": "ros",
            "started": ros_now,
            "last": ros_now,
            "last_advanced_wall": wall_now,
        }

    @classmethod
    def _navigation_elapsed(
        cls,
        snapshot: Mapping[str, Any],
        clock_state: Dict[str, Any],
        wall_now: float,
    ) -> Tuple[float, Optional[str]]:
        """Return active elapsed time and any ROS-clock watchdog failure."""
        if clock_state["source"] == "wall":
            return max(0.0, wall_now - float(clock_state["started"])), None

        started = float(clock_state["started"])
        previous = float(clock_state["last"])
        ros_now = cls._navigation_ros_time_seconds(snapshot)
        if ros_now is not None:
            if ros_now < previous - 1.0e-6:
                return max(0.0, previous - started), "ros_clock_regressed"
            if ros_now > previous + 1.0e-6:
                clock_state["last"] = ros_now
                clock_state["last_advanced_wall"] = wall_now
                previous = ros_now

        stalled_for = wall_now - float(clock_state["last_advanced_wall"])
        if stalled_for >= NAVIGATION_CLOCK_STALL_TIMEOUT_SEC:
            return max(0.0, previous - started), "ros_clock_stalled"
        return max(0.0, previous - started), None

    @staticmethod
    def _axis_navigation_legs(
        initial_pose: Any,
        station: Mapping[str, Any],
    ) -> Tuple[Dict[str, Any], ...]:
        """Build stoppable map-axis legs, always preferring X before Y."""
        if not isinstance(initial_pose, Mapping):
            raise TaskExecutionError(
                "A localized start pose is required for axis-by-axis navigation.",
                code="initial_pose_unavailable",
            )
        try:
            start_x = float(initial_pose["x"])
            start_y = float(initial_pose["y"])
            start_yaw = float(initial_pose["yaw"])
            target_x = float(station["x"])
            target_y = float(station["y"])
        except (KeyError, TypeError, ValueError) as error:
            raise TaskExecutionError(
                "The localized start pose or navigation station is invalid.",
                code="initial_pose_unavailable",
            ) from error
        if not all(
            math.isfinite(value)
            for value in (start_x, start_y, start_yaw, target_x, target_y)
        ):
            raise TaskExecutionError(
                "The localized start pose or navigation station is invalid.",
                code="initial_pose_unavailable",
            )
        expected_frame = str(station.get("frame_id") or "map")
        actual_frame = initial_pose.get("frame_id")
        if actual_frame not in {None, "", expected_frame}:
            raise TaskExecutionError(
                "The localized start pose uses an unexpected coordinate frame.",
                code="initial_pose_frame_mismatch",
                details={
                    "expected_frame": expected_frame,
                    "actual_frame": actual_frame,
                },
            )

        delta_x = target_x - start_x
        delta_y = target_y - start_y
        legs: List[Dict[str, Any]] = []
        if (
            abs(delta_x) > NAVIGATION_AXIS_EPSILON_M
            and abs(delta_y) > NAVIGATION_AXIS_EPSILON_M
        ):
            corner = dict(station)
            corner.update(
                {
                    "x": target_x,
                    "y": start_y,
                    # This robot is holonomic: preserve its current heading
                    # while completing the X leg instead of adding a turn at
                    # the corner.  The final Y leg remains responsible for
                    # the station heading, so the route is X translation,
                    # then Y translation, then at most one required turn.
                    "yaw": start_yaw,
                }
            )
            legs.append(
                {
                    "axis": "x",
                    "distance_m": abs(delta_x),
                    "target": corner,
                }
            )

        if abs(delta_y) > NAVIGATION_AXIS_EPSILON_M:
            final_axis = "y"
            final_distance = abs(delta_y)
        elif abs(delta_x) > NAVIGATION_AXIS_EPSILON_M:
            final_axis = "x"
            final_distance = abs(delta_x)
        else:
            final_axis = "alignment"
            final_distance = 0.0
        legs.append(
            {
                "axis": final_axis,
                "distance_m": final_distance,
                "target": dict(station),
            }
        )
        return tuple(legs)

    @staticmethod
    def _navigation_progress(
        snapshot: Mapping[str, Any],
        previous: float,
        initial_distance: Optional[float],
    ) -> float:
        state = str(snapshot.get("state", ""))
        if state == "enabling":
            return max(previous, 0.03)
        if state == "sending":
            return max(previous, 0.08)
        if state in {"canceling", "taking_over"}:
            return max(previous, 0.90)
        if state != "navigating":
            return previous
        distance = snapshot.get("distance_remaining")
        if (
            isinstance(distance, (int, float))
            and math.isfinite(float(distance))
        ):
            if initial_distance is not None and initial_distance > 1.0e-3:
                fraction = (
                    1.0
                    - max(0.0, float(distance)) / initial_distance
                )
                return max(previous, min(0.90, 0.10 + 0.80 * fraction))
        return max(previous, 0.10)

    @staticmethod
    def _navigation_result(
        station: Mapping[str, Any],
        snapshot: Mapping[str, Any],
        elapsed: float,
        *,
        minimum_pose_stamp_ros_nanoseconds: Optional[int],
        minimum_pose_received_monotonic: Optional[float] = None,
        observation_monotonic: Optional[float] = None,
        expected_frame: Optional[str] = None,
    ) -> Dict[str, Any]:
        pose = snapshot.get("current_pose")
        if not isinstance(pose, Mapping):
            raise TaskExecutionError(
                "Nav2 succeeded but no final localized pose is available.",
                code="final_pose_unavailable",
            )
        try:
            final_pose = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
            }
        except (KeyError, TypeError, ValueError) as error:
            raise TaskExecutionError(
                "Nav2 succeeded but the final localized pose is invalid.",
                code="final_pose_unavailable",
            ) from error
        if not all(math.isfinite(value) for value in final_pose.values()):
            raise TaskExecutionError(
                "Nav2 succeeded but the final localized pose is invalid.",
                code="final_pose_unavailable",
            )
        expected_frame = expected_frame or str(
            station.get("frame_id") or "map"
        )
        if pose.get("frame_id") != expected_frame:
            raise TaskExecutionError(
                "Nav2 succeeded but the localized pose uses an unexpected "
                "coordinate frame.",
                code="final_pose_frame_mismatch",
                details={
                    "expected_frame": expected_frame,
                    "actual_frame": pose.get("frame_id"),
                },
            )
        try:
            goal_sent_ros_nanoseconds = int(
                minimum_pose_stamp_ros_nanoseconds
            )
            stamp_ros_nanoseconds = int(pose["stamp_ros_nanoseconds"])
            observed_ros_nanoseconds = int(
                pose["observed_ros_nanoseconds"]
            )
        except (KeyError, TypeError, ValueError) as error:
            raise TaskExecutionError(
                "Nav2 succeeded but ROS-time pose freshness cannot be "
                "verified.",
                code="final_pose_stale",
            ) from error
        if goal_sent_ros_nanoseconds <= 0:
            raise TaskExecutionError(
                "Nav2 succeeded but its goal timestamp is invalid.",
                code="final_pose_stale",
                details={
                    "goal_sent_ros_nanoseconds": goal_sent_ros_nanoseconds,
                },
            )
        if stamp_ros_nanoseconds <= 0:
            raise TaskExecutionError(
                "Nav2 succeeded but the localized pose has a zero or "
                "invalid dynamic timestamp.",
                code="final_pose_stale",
                details={
                    "pose_stamp_ros_nanoseconds": stamp_ros_nanoseconds,
                    "pose_source": pose.get("source"),
                },
            )
        if observed_ros_nanoseconds <= 0:
            raise TaskExecutionError(
                "Nav2 succeeded but the active ROS clock is invalid.",
                code="final_pose_stale",
                details={
                    "pose_observed_ros_nanoseconds": (
                        observed_ros_nanoseconds
                    ),
                },
            )
        if stamp_ros_nanoseconds < goal_sent_ros_nanoseconds:
            raise TaskExecutionError(
                "Nav2 succeeded but the latest localized pose predates "
                "the Nav2 goal in ROS time.",
                code="final_pose_stale",
                details={
                    "pose_stamp_ros_nanoseconds": stamp_ros_nanoseconds,
                    "goal_sent_ros_nanoseconds": goal_sent_ros_nanoseconds,
                    "pose_source": pose.get("source"),
                },
            )
        pose_ros_age = (
            observed_ros_nanoseconds - stamp_ros_nanoseconds
        ) / 1_000_000_000.0
        if (
            pose_ros_age < -NAVIGATION_FINAL_POSE_MAX_FUTURE_SKEW_SEC
            or pose_ros_age > NAVIGATION_FINAL_POSE_MAX_AGE_SEC
        ):
            raise TaskExecutionError(
                "Nav2 succeeded but the latest localized pose timestamp is "
                "stale or inconsistent with the active ROS clock.",
                code="final_pose_stale",
                details={
                    "pose_ros_age_seconds": pose_ros_age,
                    "maximum_pose_age_seconds": (
                        NAVIGATION_FINAL_POSE_MAX_AGE_SEC
                    ),
                    "maximum_future_skew_seconds": (
                        NAVIGATION_FINAL_POSE_MAX_FUTURE_SKEW_SEC
                    ),
                    "pose_stamp_ros_nanoseconds": stamp_ros_nanoseconds,
                    "pose_observed_ros_nanoseconds": (
                        observed_ros_nanoseconds
                    ),
                    "pose_source": pose.get("source"),
                },
            )
        if minimum_pose_received_monotonic is not None:
            try:
                received_monotonic = float(pose["received_monotonic"])
            except (KeyError, TypeError, ValueError) as error:
                raise TaskExecutionError(
                    "Nav2 succeeded but pose freshness cannot be verified.",
                    code="final_pose_stale",
                ) from error
            if (
                not math.isfinite(received_monotonic)
                or received_monotonic < minimum_pose_received_monotonic
            ):
                raise TaskExecutionError(
                    "Nav2 succeeded but the latest localized pose predates "
                    "this navigation goal.",
                    code="final_pose_stale",
                    details={
                        "pose_received_monotonic": received_monotonic,
                        "goal_sent_monotonic": (
                            minimum_pose_received_monotonic
                        ),
                    },
                )
            observed_at = (
                time.monotonic()
                if observation_monotonic is None
                else float(observation_monotonic)
            )
            pose_age = observed_at - received_monotonic
            if (
                not math.isfinite(observed_at)
                or not math.isfinite(pose_age)
                or pose_age < 0.0
                or pose_age > NAVIGATION_FINAL_POSE_MAX_AGE_SEC
            ):
                raise TaskExecutionError(
                    "Nav2 succeeded but the latest localized pose is stale.",
                    code="final_pose_stale",
                    details={
                        "pose_age_seconds": pose_age,
                        "maximum_pose_age_seconds": (
                            NAVIGATION_FINAL_POSE_MAX_AGE_SEC
                        ),
                    },
                )
        position_error = math.hypot(
            final_pose["x"] - float(station["x"]),
            final_pose["y"] - float(station["y"]),
        )
        yaw_delta = final_pose["yaw"] - float(station["yaw"])
        yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
        return {
            "cabinet": station["cabinet"],
            "station": dict(station),
            "final_pose": final_pose,
            "error": {
                "position_m": position_error,
                "yaw_rad": yaw_error,
            },
            "duration_seconds": elapsed,
        }

    def _execute_operation_task_owned(
        self,
        context: Any,
        cabinet: str,
        client: CabinetClient,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
        replay_owned: bool,
    ) -> Mapping[str, Any]:
        if not replay_owned:
            return self._execute_operation_task(
                context,
                cabinet,
                client,
                control_id,
                command,
                target_state,
                target_position,
                force,
            )
        with self._replay_internal_scope():
            return self._execute_operation_task(
                context,
                cabinet,
                client,
                control_id,
                command,
                target_state,
                target_position,
                force,
            )

    def _execute_operation_task(
        self,
        context: Any,
        cabinet: str,
        client: CabinetClient,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Mapping[str, Any]:
        event_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        with self._operation_bindings_lock:
            self._operation_event_queues[cabinet] = event_queue
        started = time.monotonic()
        timeout_started_at: Optional[float] = None
        timeout_reported = False
        last_timeout_cancel_at = 0.0
        last_progress = 0.0
        try:
            try:
                # See the navigation equivalent above.  This makes the
                # cancellation check and action submission one admission step.
                with self._task_interlock_scope():
                    context.raise_if_canceled()
                    submission = client.submit_operation(
                        control_id,
                        command,
                        target_state=target_state,
                        target_position=target_position,
                        force=force,
                        navigate=False,
                    )
            except (CabinetClientError, ControlRequestError) as error:
                raise TaskExecutionError(
                    str(error),
                    code=(
                        "backend_unavailable"
                        if getattr(error, "status", 500) == 503
                        else "operation_rejected"
                    ),
                    details=getattr(error, "details", {}),
                ) from error

            submission_status = str(submission.get("status", "accepted"))
            if submission_status == "canceled":
                raise TaskCanceledError(
                    str(submission.get("message") or "Operation was canceled.")
                )
            if submission_status == "failed":
                result = dict(submission.get("result") or {})
                result.update(
                    {
                        "cabinet": cabinet,
                        "control_id": control_id,
                        "command": command,
                        "duration_seconds": time.monotonic() - started,
                    }
                )
                raise TaskExecutionError(
                    str(
                        submission.get("message")
                        or "Cabinet operation failed."
                    ),
                    code=str(
                        submission.get("failure_code")
                        or "operation_failed"
                    ),
                    result=result,
                )

            expected_generation = submission.get("generation")

            while True:
                now = time.monotonic()
                elapsed = now - started
                if (
                    timeout_started_at is None
                    and elapsed >= OPERATION_TIMEOUT_SEC
                ):
                    timeout_started_at = now
                timed_out = timeout_started_at is not None
                cancel_for_shutdown = context.shutdown_requested
                if (
                    (timed_out or cancel_for_shutdown)
                    and now - last_timeout_cancel_at >= 1.0
                ):
                    try:
                        client.cancel()
                    except Exception:  # noqa: BLE001
                        # As with navigation, retain global exclusivity until
                        # the cabinet action itself reports a terminal event.
                        pass
                    last_timeout_cancel_at = now
                try:
                    event = event_queue.get(timeout=TASK_MONITOR_PERIOD_SEC)
                except queue.Empty:
                    event = None

                if (
                    event is not None
                    and expected_generation is not None
                    and event.get("generation") != expected_generation
                ):
                    event = None

                event_type = (
                    str(event.get("event", ""))
                    if event is not None
                    else ""
                )
                if event_type == "feedback":
                    if not timeout_reported:
                        progress_value = event.get("progress", last_progress)
                        try:
                            progress = float(progress_value)
                        except (TypeError, ValueError):
                            progress = last_progress
                        if not math.isfinite(progress):
                            progress = last_progress
                        progress = max(
                            last_progress,
                            min(0.99, max(0.0, progress)),
                        )
                        last_progress = progress
                        context.progress(
                            f"operation_{event.get('phase', 'running')}",
                            progress,
                            message=str(
                                event.get("message") or "Operation running."
                            ),
                            data={
                                "cabinet": cabinet,
                                "control_id": control_id,
                                "phase_code": event.get("phase_code"),
                                "current_position": event.get(
                                    "current_position"
                                ),
                                "target_position": event.get(
                                    "target_position"
                                ),
                                "current_state": event.get("current_state"),
                            },
                        )
                elif event_type == "terminal":
                    result = dict(event.get("result") or {})
                    result.update(
                        {
                            "cabinet": cabinet,
                            "control_id": control_id,
                            "command": command,
                            "duration_seconds": time.monotonic() - started,
                        }
                    )
                    initial = result.get("initial_position")
                    peak = result.get("peak_position")
                    if isinstance(initial, (int, float)) and isinstance(
                        peak, (int, float)
                    ):
                        result["actual_displacement"] = max(
                            0.0,
                            float(peak) - float(initial),
                        )
                    outcome = str(event.get("outcome", "failed"))
                    if timed_out:
                        if timeout_reported:
                            context.release_reservation(
                                backend_termination_confirmed=True,
                                details={"backend_terminal_state": outcome},
                            )
                            return {}
                        raise TaskExecutionError(
                            "Cabinet operation timed out and reached a "
                            "terminal state.",
                            code="operation_timeout",
                            result=result,
                        )
                    if outcome == "success" and bool(
                        event.get("success", True)
                    ):
                        return result
                    if outcome == "canceled":
                        raise TaskCanceledError(
                            str(
                                event.get("message")
                                or "Operation was canceled."
                            )
                        )
                    failure_code = str(
                        event.get("failure_code") or "operation_failed"
                    )
                    raise TaskExecutionError(
                        str(
                            event.get("message")
                            or "Cabinet operation failed."
                        ),
                        code=failure_code,
                        details={"error_code": event.get("error_code")},
                        result=result,
                    )

                now = time.monotonic()
                elapsed = now - started
                if (
                    timed_out
                    and not timeout_reported
                    and now - timeout_started_at >= OPERATION_CANCEL_GRACE_SEC
                ):
                    context.fail_retaining_reservation(
                        "Cabinet operation timed out; backend termination is "
                        "still unconfirmed after the cancellation grace "
                        "period.",
                        code="operation_timeout",
                        details={
                            "backend_terminal_state": "active",
                            "cancel_grace_seconds": OPERATION_CANCEL_GRACE_SEC,
                        },
                        result={
                            "cabinet": cabinet,
                            "control_id": control_id,
                            "command": command,
                            "duration_seconds": elapsed,
                        },
                    )
                    timeout_reported = True
                if (
                    cancel_for_shutdown
                    and context.shutdown_elapsed >= BACKEND_SHUTDOWN_GRACE_SEC
                ):
                    if not timeout_reported:
                        context.fail_retaining_reservation(
                            "Gateway shutdown ended cabinet-operation "
                            "monitoring before the backend confirmed "
                            "termination.",
                            code="shutdown_backend_unconfirmed",
                            details={"backend_terminal_state": "active"},
                            result={
                                "cabinet": cabinet,
                                "control_id": control_id,
                                "command": command,
                                "duration_seconds": elapsed,
                            },
                        )
                    context.release_reservation(
                        backend_termination_confirmed=False,
                        details={
                            "backend_terminal_state": "active",
                            "shutdown_monitoring_ended": True,
                        },
                    )
                    return {}
        finally:
            with self._operation_bindings_lock:
                if self._operation_event_queues.get(cabinet) is event_queue:
                    self._operation_event_queues.pop(cabinet, None)

    def _cabinet_event_listener(self, event: Mapping[str, Any]) -> None:
        cabinet = event.get("cabinet")
        if not isinstance(cabinet, str):
            return
        with self._operation_bindings_lock:
            event_queue = self._operation_event_queues.get(cabinet)
        if event_queue is not None:
            event_queue.put(dict(event))

    @staticmethod
    def _recording_control_error(error: RecordingError) -> ControlRequestError:
        return ControlRequestError(
            str(error),
            error.status,
            details={"code": error.code},
        )

    def _call_recording_manager(
        self,
        callback: Any,
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        try:
            return callback(*args, **kwargs)
        except RecordingError as error:
            raise self._recording_control_error(error) from error

    @staticmethod
    def _status_field(snapshot: Mapping[str, Any]) -> Dict[str, Any]:
        value = dict(snapshot)
        state = value.pop("state", value.get("status", "idle"))
        value["status"] = state
        return value

    def _replay_status_snapshot(self) -> Dict[str, Any]:
        try:
            recording = self._status_field(
                self._recording_manager.recording_status()
            )
            playback = self._status_field(
                self._recording_manager.playback_status()
            )
        except RecordingError as error:
            raise self._recording_control_error(error) from error
        task_replay = self._task_replay.status()
        playback_active = playback["status"] in {"playing", "paused"}
        task_active = task_replay.get("status") in {"running", "canceling"}
        return {
            "mode": (
                "task_replay"
                if task_active
                else "data_playback" if playback_active else "idle"
            ),
            "read_only": playback_active or task_active,
            "recording": recording,
            "playback": playback,
            "task_replay": task_replay,
        }

    def _ensure_backend_quiescent(self, operation: str) -> None:
        active_task = self._active_task_snapshot()
        if active_task is not None:
            task_id = str(active_task.get("task_id", "unknown"))
            raise ControlRequestError(
                f"{operation} cannot start while task {task_id} is active.",
                409,
                details={"active_task_id": task_id},
            )
        navigation = self._node.navigation_snapshot()
        if (
            str(navigation.get("state", ""))
            in RosControlNode.ACTIVE_NAVIGATION_STATES
            or int(navigation.get("retiring_goals", 0) or 0) > 0
        ):
            raise ControlRequestError(
                f"{operation} cannot start while Nav2 is active.",
                409,
            )
        active_cabinets = sorted(
            name
            for name, client in self._cabinet_clients.items()
            if bool(client.snapshot_status().get("active"))
        )
        if active_cabinets:
            raise ControlRequestError(
                f"{operation} cannot start while a cabinet action is active.",
                409,
                details={"active_cabinets": active_cabinets},
            )

    def _quiesce_manual_outputs(self) -> None:
        """Stop legacy writes and wait out an accepted manual trajectory."""
        quiesce = getattr(self._node, "quiesce_manual_outputs", None)
        if not callable(quiesce):
            # Compatibility for lifecycle fakes and older injected nodes.
            self._node.emergency_stop()
            return
        settle_seconds = quiesce()
        if (
            isinstance(settle_seconds, bool)
            or not isinstance(settle_seconds, (int, float))
            or not math.isfinite(float(settle_seconds))
            or float(settle_seconds) < 0.0
        ):
            raise ControlRequestError(
                "Manual output quiescence returned an invalid settle time.",
                503,
            )
        if settle_seconds > 0.0:
            time.sleep(float(settle_seconds))

    def _quiesce_manual_outputs_for_replay(self) -> None:
        """Compatibility wrapper for existing replay admission call sites."""
        self._quiesce_manual_outputs()

    def _replay_internal_authorized(self) -> bool:
        local = getattr(self, "_replay_internal", None)
        return bool(local is not None and getattr(local, "depth", 0) > 0)

    @contextmanager
    def _replay_internal_scope(self) -> Iterator[None]:
        local = getattr(self, "_replay_internal", None)
        if local is None:
            local = threading.local()
            self._replay_internal = local
        previous = int(getattr(local, "depth", 0))
        local.depth = previous + 1
        try:
            yield
        finally:
            local.depth = previous

    def _replay_submit_navigation(
        self,
        cabinet: str,
        control_id: Optional[str],
    ) -> Mapping[str, Any]:
        with self._replay_internal_scope():
            return self.submit_navigation_task(cabinet, control_id)

    def _replay_submit_operation(
        self,
        cabinet: str,
        control_id: str,
        command: Any,
        target_state: Optional[str],
        target_position: Optional[float],
        force: Optional[float],
    ) -> Mapping[str, Any]:
        with self._replay_internal_scope():
            return self.submit_operation_task(
                cabinet,
                control_id,
                command,
                target_state,
                target_position,
                force,
            )

    def _replay_submit_reset(self, cabinet: str) -> Mapping[str, Any]:
        with self._replay_internal_scope():
            return self.submit_reset_task(cabinet)

    def _replay_task_status(self, task_id: str) -> Mapping[str, Any]:
        return self._task_manager.get_task(task_id)

    def _replay_cancel_task(self, task_id: str) -> Mapping[str, Any]:
        with self._replay_internal_scope():
            with self._task_interlock_scope():
                return self._cancel_managed_task(task_id)

    def _bridge_task_events(self) -> None:
        while True:
            try:
                event = self._event_bridge_subscription.get(timeout=0.5)
            except queue.Empty:
                continue
            except EventHubClosed:
                return
            recording_manager = getattr(self, "_recording_manager", None)
            if recording_manager is not None:
                try:
                    recording_manager.record_task_event(event)
                except Exception as error:  # noqa: BLE001
                    try:
                        self._node.get_logger().warning(
                            "Failed to append the recording task timeline: "
                            f"{error}"
                        )
                    except Exception:  # noqa: BLE001
                        pass
            try:
                self._node.publish_task_event(event)
            except Exception as error:  # noqa: BLE001
                try:
                    self._node.get_logger().warning(
                        f"Failed to publish task event to ROS 2: {error}"
                    )
                except Exception:  # noqa: BLE001
                    continue

    @contextmanager
    def _task_interlock_scope(self) -> Iterator[None]:
        """Serialize global-task admission with conflicting ROS mutations."""
        if getattr(self, "_task_manager", None) is None:
            # Compatibility with lifecycle tests and old embedded users that
            # construct a direct-only server without the task subsystem.
            yield
            return
        lock = getattr(self, "_task_interlock_lock", None)
        if lock is None:
            # ``object.__new__`` task-runner tests skip __init__; production
            # servers always create the lock eagerly.
            lock = threading.RLock()
            self._task_interlock_lock = lock
        with lock:
            if (
                self._replay_read_only()
                and not self._replay_internal_authorized()
            ):
                status = self._replay_status_snapshot()
                raise ControlRequestError(
                    "Live control is disabled while replay is active.",
                    409,
                    details={
                        "replay_mode": status["mode"],
                        "read_only": True,
                    },
                )
            yield

    def _replay_read_only(self) -> bool:
        recording_manager = getattr(self, "_recording_manager", None)
        task_replay = getattr(self, "_task_replay", None)
        playback_active = False
        if recording_manager is not None:
            try:
                playback_active = recording_manager.playback_status().get(
                    "state"
                ) in {"playing", "paused"}
            except RecordingError:
                # A write interlock must fail closed when player state cannot
                # be established; status/management routes remain available
                # so the operator can diagnose or cancel the session.
                playback_active = True
        return playback_active or bool(
            task_replay is not None and task_replay.is_active
        )

    @contextmanager
    def _request_scope(self) -> Iterator[None]:
        """Reject new API work during shutdown and drain entered calls."""
        with self._request_condition:
            if self._stopping:
                raise ControlRequestError(
                    "Web control server is stopping.",
                    503,
                )
            self._active_requests += 1
        try:
            yield
        finally:
            with self._request_condition:
                self._active_requests -= 1
                if self._active_requests == 0:
                    self._request_condition.notify_all()
