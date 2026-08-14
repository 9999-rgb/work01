"""ROS 2 client for the Gazebo entity factory services.

The Web gateway never owns Gazebo; it reaches the running simulator through
the standard factory services ``/spawn_entity``, ``/delete_entity`` and
``/set_entity_state`` (provided by ``gazebo_ros``).  Each method blocks on the
``SingleThreadedExecutor`` that spins this node, mirroring the service-call
pattern used by :class:`~control_gateway.cabinet_client.CabinetClient`.
"""

from __future__ import annotations

import threading
from typing import Any, Optional, Tuple

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from rclpy.context import Context
from rclpy.node import Node

from .ros_node import ControlRequestError

# Fixed entity name for the active static scene floor.  A stable name keeps the
# switch logic trivial: delete this entity before spawning a new scene model.
SCENE_FLOOR_ENTITY = "xczs_scene_floor"

_SERVICE_TIMEOUT_SEC = 10.0


class GazeboClient(Node):
    """Own the Gazebo factory service clients used for scene switching."""

    def __init__(self, *, context: Optional[Context] = None) -> None:
        super().__init__("xczs_gazebo_client", context=context)
        self._spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self._set_state_client = self.create_client(
            SetEntityState, "/set_entity_state"
        )

    def spawn_entity(
        self,
        name: str,
        xml: str,
        *,
        pose: Optional[Tuple[float, float, float, float, float, float]] = None,
        reference_frame: str = "world",
        timeout_sec: float = _SERVICE_TIMEOUT_SEC,
    ) -> None:
        """Spawn one model from a URDF/SDF ``xml`` string."""
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.reference_frame = reference_frame
        if pose is not None:
            request.initial_pose = _pose_from_xyzrpy(*pose)
        response = self._call(
            self._spawn_client, request, timeout_sec, "spawn entity"
        )
        if not bool(response.success):
            raise ControlRequestError(
                str(response.status_message) or f"Failed to spawn '{name}'.",
                503,
            )

    def delete_entity(
        self,
        name: str,
        *,
        ignore_missing: bool = False,
        timeout_sec: float = _SERVICE_TIMEOUT_SEC,
    ) -> None:
        """Delete one model by name.

        ``ignore_missing`` tolerates "already gone" responses so scene
        reconciliation can delete-then-spawn idempotently after a partial
        failure.
        """
        request = DeleteEntity.Request()
        request.name = name
        response = self._call(
            self._delete_client, request, timeout_sec, "delete entity"
        )
        if not bool(response.success):
            message = str(response.status_message) or ""
            if ignore_missing and _looks_absent(message):
                return
            raise ControlRequestError(
                message or f"Failed to delete '{name}'.",
                503,
            )

    def set_entity_state(
        self,
        name: str,
        *,
        x: float,
        y: float,
        z: float,
        yaw: float,
        reference_frame: str = "world",
        timeout_sec: float = _SERVICE_TIMEOUT_SEC,
    ) -> None:
        """Teleport a model (its root link) to a planar world pose."""
        request = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.reference_frame = reference_frame
        state.pose = _pose_from_xyzrpy(x, y, z, 0.0, 0.0, yaw)
        request.state = state
        response = self._call(
            self._set_state_client, request, timeout_sec, "set entity state"
        )
        if not bool(response.success):
            raise ControlRequestError(
                str(response.status_message) or f"Failed to move '{name}'.",
                503,
            )

    @staticmethod
    def _service_ready(client: Any) -> bool:
        try:
            return bool(client.service_is_ready())
        except Exception:  # noqa: BLE001
            return False

    def _call(self, client: Any, request: Any, timeout_sec: float, label: str) -> Any:
        if not self._service_ready(client):
            raise ControlRequestError(
                f"Gazebo {label} service is unavailable.", 503
            )
        try:
            future = client.call_async(request)
        except Exception as error:  # noqa: BLE001
            raise ControlRequestError(
                f"Failed to request Gazebo {label}: {error}", 503
            ) from error
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=max(0.1, float(timeout_sec))):
            raise ControlRequestError(f"Gazebo {label} request timed out.", 503)
        try:
            return future.result()
        except Exception as error:  # noqa: BLE001
            raise ControlRequestError(
                f"Gazebo {label} failed: {error}", 503
            ) from error


_ABSENT_SUBSTRINGS = (
    "does not exist",
    "doesn't exist",
    "not exist",
    "not found",
    "no such",
)


def _looks_absent(message: str) -> bool:
    """Best-effort detection of a Gazebo "entity absent" response."""
    lowered = message.lower()
    return any(fragment in lowered for fragment in _ABSENT_SUBSTRINGS)


def _pose_from_xyzrpy(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float
) -> Pose:
    """Build a geometry_msgs/Pose from fixed-axis xyz + rpy (no math import)."""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    orientation = _quaternion_from_rpy(roll, pitch, yaw)
    pose.orientation = orientation
    return pose


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    import math

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    quaternion = Quaternion()
    quaternion.w = cr * cp * cy + sr * sp * sy
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    return quaternion
