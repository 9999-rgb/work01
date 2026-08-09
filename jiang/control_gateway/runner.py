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
from .inventory import OccupancyGridBoundary
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
from .web_server import ControlHandler


CABINET_SHUTDOWN_TIMEOUT_SEC = 15.0
NAVIGATION_SHUTDOWN_TIMEOUT_SEC = 15.0
TASK_MANAGER_SHUTDOWN_TIMEOUT_SEC = 8.0
BACKEND_SHUTDOWN_GRACE_SEC = 3.0
REQUEST_DRAIN_TIMEOUT_SEC = 8.0
NAVIGATION_TIMEOUT_SEC = 180.0
NAVIGATION_CANCEL_GRACE_SEC = 5.0
NAVIGATION_FINAL_POSE_WAIT_SEC = 2.0
NAVIGATION_FINAL_POSE_MAX_AGE_SEC = 1.0
OPERATION_TIMEOUT_SEC = 180.0
OPERATION_CANCEL_GRACE_SEC = 5.0
TASK_MONITOR_PERIOD_SEC = 0.10
EXECUTOR_SPIN_PERIOD_SEC = 0.10
NAVIGATION_POSITION_TOLERANCE_M = 0.20
# Nav2's 0.20 rad goal checker and the following TF sample can differ by a
# fraction of a degree while the base settles.  Keep a 5 mrad verification
# margin without relaxing the controller's own stopping criterion.
NAVIGATION_YAW_TOLERANCE_RAD = 0.205
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
    ) -> None:
        if not self._is_loopback_host(host):
            raise ValueError(
                "The unauthenticated control gateway may only bind to a "
                "loopback address."
            )
        if allowed_origins is None:
            allowed_origins = (
                "http://localhost:8080",
                "http://127.0.0.1:8080",
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
        scene_path = cabinet_scene_path or str(
            control_config / "cabinet_scene.yaml"
        )
        robot_adapter_path = cabinet_robot_adapter_path or str(
            control_config / "cabinet_robot_adapter.yaml"
        )
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
        self._host = host
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

    def start(self) -> "ControlServer":
        """Start the ROS executor and threaded HTTP listener."""
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
        self._executor_thread.start()
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="web-control-http-server",
            daemon=True,
        )
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

            safe_to_destroy_ros = (
                task_shutdown["workers_stopped"]
                and task_shutdown.get("active_task_id") is None
                and pending_requests == 0
                and not pending
                and not legacy_cabinet_pending
                and not navigation_pending
                and event_bridge_stopped
                and http_thread_stopped
            )
            self._shutdown_report = {
                "task_manager": task_shutdown,
                "pending_requests": pending_requests,
                "pending_cabinets": sorted(pending),
                "legacy_cabinet_pending": legacy_cabinet_pending,
                "navigation_pending": navigation_pending,
                "event_bridge_stopped": event_bridge_stopped,
                "http_thread_stopped": http_thread_stopped,
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
            return {
                "status": "ok",
                "navigation_available": navigation["available"],
                "cabinet_available": cabinet_available,
                "cabinet_active": cabinet_active,
                "cabinet_count": len(clients) if clients else 1,
                "active_task_id": (
                    task_manager.active_task_id
                    if task_manager is not None
                    else None
                ),
                "cabinets": cabinet_states,
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
                self._inventory.get(cabinet)
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
                station = self._inventory.station_for(
                    cabinet,
                    control_station=control_station,
                    boundary=self._live_map_bounds(),
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
                try:
                    return self._task_manager.submit(
                        "navigate",
                        request,
                        lambda context: self._execute_navigation_task(
                            context,
                            station.to_dict(),
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
                try:
                    return self._task_manager.submit(
                        "operate",
                        request,
                        lambda context: self._execute_operation_task(
                            context,
                            cabinet,
                            client,
                            control_id,
                            command,
                            target_state,
                            target_position,
                            force_value,
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
                    {"navigate", "operate"},
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
                    {"operate"},
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
                    {"navigate", "operate"},
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
                return self._node.cancel_navigation()

    def takeover_navigation(self) -> Dict[str, Any]:
        """Cancel Nav2 and switch the base router to zero-speed manual mode."""
        with self._request_scope():
            with self._task_interlock_scope():
                self._reject_active_task_types(
                    {"operate"},
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
                    {"navigate", "operate"},
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
                    {"navigate", "operate"},
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
                    {"navigate", "operate"},
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

    def _live_map_bounds(self) -> Optional[OccupancyGridBoundary]:
        try:
            map_state = self._node.map_snapshot()
        except ControlRequestError:
            return None
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
        except (KeyError, TypeError, ValueError, InventoryError):
            return None

    def _execute_navigation_task(
        self,
        context: Any,
        station: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        started = time.monotonic()
        initial_distance: Optional[float] = None
        initial_pose = self._node.navigation_snapshot().get("current_pose")
        if isinstance(initial_pose, Mapping):
            try:
                initial_distance = math.hypot(
                    float(station["x"]) - float(initial_pose["x"]),
                    float(station["y"]) - float(initial_pose["y"]),
                )
            except (KeyError, TypeError, ValueError):
                initial_distance = None
        try:
            # Cancellation and legacy control admission use the same lock so
            # an immediately canceled task cannot submit a goal afterward.
            with self._task_interlock_scope():
                context.raise_if_canceled()
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
        timeout_reported = False
        last_timeout_cancel_at = 0.0
        final_pose_deadline: Optional[float] = None
        while True:
            snapshot = self._node.navigation_snapshot()
            state = str(snapshot.get("state", "unknown"))
            now = time.monotonic()
            elapsed = now - started
            if timeout_started_at is None and elapsed >= NAVIGATION_TIMEOUT_SEC:
                timeout_started_at = now
            timed_out = timeout_started_at is not None

            if state == "succeeded":
                if final_pose_deadline is None:
                    final_pose_deadline = now + NAVIGATION_FINAL_POSE_WAIT_SEC
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
                        and now < final_pose_deadline
                    ):
                        time.sleep(TASK_MONITOR_PERIOD_SEC)
                        continue
                    raise
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
                        result=result,
                    )
                errors = result["error"]
                if (
                    errors["position_m"] > NAVIGATION_POSITION_TOLERANCE_M
                    or errors["yaw_rad"] > NAVIGATION_YAW_TOLERANCE_RAD
                ):
                    raise TaskExecutionError(
                        "Navigation ended outside the operation-station "
                        "position/yaw tolerance.",
                        code="pose_deviation_exceeded",
                        details={
                            "position_tolerance_m": (
                                NAVIGATION_POSITION_TOLERANCE_M
                            ),
                            "yaw_tolerance_rad": NAVIGATION_YAW_TOLERANCE_RAD,
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
            if state in {"failed", "rejected"}:
                code = (
                    "navigation_timeout"
                    if timed_out
                    else "target_unreachable"
                )
                failure_details = {
                    "nav2_state": state,
                    "classification": (
                        "Nav2 does not expose enough information here to "
                        "distinguish an obstructed path from another "
                        "unreachable target."
                    ),
                }
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
                context.progress(
                    f"navigation_{state}",
                    last_progress,
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
            pose_ros_age < 0.0
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

    def _bridge_task_events(self) -> None:
        while True:
            try:
                event = self._event_bridge_subscription.get(timeout=0.5)
            except queue.Empty:
                continue
            except EventHubClosed:
                return
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
            yield

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
