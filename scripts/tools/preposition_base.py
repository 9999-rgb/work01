#!/usr/bin/env python3
"""Teleport the robot base onto a control's docking station in simulation.

``validate_cabinet_simulation --control <id>`` does not run Nav2; the robot
must already be near the selected control's station.  This helper computes the
same station pose the operator uses
(``cabinet_button_operator::calculate_control_staging_poses``: anchor +
outward_axis * standoff, transformed by the cabinet frame, yaw =
atan2(-outward_y, -outward_x) + base_yaw_offset) and teleports the robot
entity there via ``gazebo_ros`` ``/set_entity_state``, then publishes an AMCL
initial-pose hypothesis so localization-aware consumers agree.

Usage:
    scripts/tools/preposition_base.py --control box_5_knob [--cabinet cabinet_a]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, TransformException
from yaml import safe_load

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "jiang"))

import control_gateway.robot_adapter as robot_adapter  # noqa: E402

DEFAULT_ADAPTER = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "cabinet_robot_adapter.yaml"
)
ROBOT_ENTITY = "xczs_inspection_robot"


def _quat_rotate(q, v):
    """Rotate 3-vector ``v`` by quaternion ``q = (x, y, z, w)``.

    Standard q * v quaternion-vector product (``t = 2 * (q_xyz x v)``;
    ``v' = v + w*t + q_xyz x t``).  The cabinet frame is arbitrarily oriented
    (cabinet A rotates the panel 90° about its local x axis), so the local
    offset and outward axis must be rotated by the full quaternion rather than
    a yaw-only approximation.
    """
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


class PrepositionError(RuntimeError):
    """Failed to place the robot base on a control's station."""


def teleport_to_station(
    node: Node,
    *,
    station,
    cabinet_frame: str,
    entity_name: str = ROBOT_ENTITY,
    control_id: str = "",
    standoff_override: float | None = None,
) -> tuple[float, float, float, float, float]:
    """Teleport ``entity_name`` onto ``station`` and seed AMCL.

    Reuses the operator's station math
    (``cabinet_button_operator::calculate_control_staging_poses``): anchor +
    outward_axis * standoff rotated by the cabinet frame yaw, base yaw =
    atan2(-outward_y, -outward_x) + base_yaw_offset, body = base + pi/2.
    ``standoff_override`` replaces the adapter's standoff so reachability can
    be swept without touching the adapter or restarting the operator.

    Returns ``(world_x, world_y, world_z, body_yaw, base_yaw)``.  Callers may
    spin ``node`` while waiting for the cabinet frame TF.
    """
    ax, ay, az = station.outward_axis
    standoff = (
        float(station.standoff)
        if standoff_override is None
        else float(standoff_override)
    )
    local_x = station.local_anchor[0] + ax * standoff
    local_y = station.local_anchor[1] + ay * standoff
    local_z = station.local_anchor[2] + az * standoff

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=10.0)
    last_error: TransformException | None = None
    cx = cy = cz = cyaw = None
    while node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        try:
            tf = tf_buffer.lookup_transform("odom", cabinet_frame, rclpy.time.Time())
            q = tf.transform.rotation
            t = tf.transform.translation
            cx, cy, cz = t.x, t.y, t.z
            break
        except TransformException as error:
            last_error = error
            node.get_logger().debug(
                f"cabinet TF not ready: {error}", throttle_duration_sec=1.0
            )
    if cx is None:
        raise PrepositionError(
            f"Cabinet frame '{cabinet_frame}' unavailable in odom: {last_error}"
        )

    # Full 3-D cabinet-frame rotation: the local offset and outward axis are
    # rotated by the complete quaternion so the station matches the operator's
    # calculate_control_staging_poses exactly (a yaw-only approximation puts
    # the robot ~1.9 m off for cabinet A's 90°-rotated panel).
    qv = (q.x, q.y, q.z, q.w)
    lx, ly, lz = _quat_rotate(qv, (local_x, local_y, local_z))
    world_x = cx + lx
    world_y = cy + ly
    world_z = cz + lz

    # outward direction in world, for the base yaw (same formula as the
    # operator: atan2(-outward_y, -outward_x) + base_yaw_offset).
    ox, oy, _oz = _quat_rotate(qv, (ax, ay, az))
    base_yaw = math.atan2(-oy, -ox) + station.base_yaw_offset

    # base_link is a fixed -pi/2 child of body; body must be +pi/2 ahead.
    body_yaw = base_yaw + math.pi / 2.0

    # Direct service call so the node can be spun while waiting (GazeboClient
    # blocks on a threading.Event and needs an external executor).
    state_client = node.create_client(SetEntityState, "/set_entity_state")
    if not state_client.wait_for_service(timeout_sec=10.0):
        raise PrepositionError("/set_entity_state service is unavailable.")
    request = SetEntityState.Request()
    request.state.name = entity_name
    request.state.reference_frame = "world"
    request.state.pose.position.x = world_x
    request.state.pose.position.y = world_y
    request.state.pose.position.z = world_z
    request.state.pose.orientation.z = math.sin(body_yaw / 2.0)
    request.state.pose.orientation.w = math.cos(body_yaw / 2.0)
    future = state_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done():
        raise PrepositionError("set_entity_state request timed out.")
    response = future.result()
    if response is None or not bool(response.success):
        raise PrepositionError(
            f"Failed to teleport '{entity_name}': "
            f"{getattr(response, 'status_message', 'no response')}"
        )

    # AMCL initial pose (navigation frame yaw = base yaw).
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.pose.position.x = world_x
    pose.pose.pose.position.y = world_y
    pose.pose.pose.orientation.z = math.sin(base_yaw / 2.0)
    pose.pose.pose.orientation.w = math.cos(base_yaw / 2.0)
    pose.pose.covariance[0] = 0.25
    pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = 0.1
    initial_pose_pub = node.create_publisher(
        PoseWithCovarianceStamped, "/initialpose", 10
    )
    for _ in range(5):
        initial_pose_pub.publish(pose)
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    node.get_logger().info(
        f"Teleported '{entity_name}' to '{control_id}' station: "
        f"({world_x:.3f}, {world_y:.3f}, {world_z:.3f}) "
        f"body_yaw={body_yaw:.3f} base_yaw={base_yaw:.3f} "
        f"(cabinet {cabinet_frame})"
    )
    return world_x, world_y, world_z, body_yaw, base_yaw


class Prepositioner(Node):
    def __init__(
        self,
        *,
        control_id: str,
        cabinet: str,
        toolset: str,
        adapter_path: str,
    ) -> None:
        super().__init__("preposition_base")
        self._control_id = control_id
        self._cabinet_frame = f"{cabinet}_frame"
        adapter = robot_adapter.load(adapter_path, toolset=toolset)
        station = adapter.control_navigation_station(control_id)
        if station is None:
            raise PrepositionError(
                f"No navigation station configured for '{control_id}'."
            )
        self._station = station

    def run(self) -> None:
        teleport_to_station(
            self,
            station=self._station,
            cabinet_frame=self._cabinet_frame,
            control_id=self._control_id,
        )
        print(
            f"Robot teleported to '{self._control_id}' station "
            f"(cabinet {self._cabinet_frame})."
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Teleport the robot base onto a control's docking station."
    )
    parser.add_argument("--control", required=True, help="control id, e.g. box_5_knob")
    parser.add_argument(
        "--cabinet", default="cabinet_a", help="cabinet instance name (default: cabinet_a)"
    )
    parser.add_argument("--toolset", default="B", choices=("A", "B"), help="active toolset")
    parser.add_argument(
        "--adapter",
        default=str(DEFAULT_ADAPTER),
        help="cabinet_robot_adapter.yaml path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    rclpy.init()
    try:
        Prepositioner(
            control_id=args.control,
            cabinet=args.cabinet,
            toolset=args.toolset,
            adapter_path=args.adapter,
        ).run()
    except PrepositionError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
