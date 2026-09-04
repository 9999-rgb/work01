#!/usr/bin/env python3
"""Recover a broken AMCL localization after a failed teleport/dock.

Symptoms this repairs (all seen together after preposition_base seeded AMCL
with a wall-clock-stamped /initialpose into a sim-time tf tree):

  * map->odom absent or erratic -> 'not part of the same tree' lookups
  * local_costmap 'sensor origin out of map bounds' wandering windows
  * precision docking timeout: the operator chases a phantom map-frame error
    and physically drives the robot away (observed -6.7, -13.5 in db1 world)

Procedure (mirrors the proven runner boot pattern in
jiang/control_gateway/ros_node.py: wait_for_planar_odom + publish_initial_pose):

  1. Teleport the robot entity to the map-frame station pose via
     /set_entity_state (world origin == map origin for fixture scenes).
  2. Wait until odom->base_link reports the teleported pose (the planar
     mover publishes ~20 Hz; seeding before it lands bakes the pre-teleport
     odom pose into map->odom).
  3. Publish a ZERO-stamped /initialpose hypothesis: tf2 treats stamp 0 as
     'use the latest transform', which survives AMCL's exact-time lookup
     extrapolation that wall/sim 'now' stamps trip over.

Usage:
    python3 scripts/tools/recover_amcl.py --x 1.199 --y 4.400 \
        --body-yaw -1.571 --base-yaw -3.142
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException

ROBOT_ENTITY = "xczs_inspection_robot"
BASE_FRAME = "base_link"


class AmclRecoverer(Node):
    def __init__(self, x: float, y: float, body_yaw: float, base_yaw: float) -> None:
        super().__init__("amcl_recoverer")
        self._target = (x, y, body_yaw)
        self._base_yaw = base_yaw
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer, self)

    def _entity_pose(self) -> tuple[float, float, float] | None:
        client = self.create_client(GetEntityState, "/get_entity_state")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/get_entity_state unavailable")
            return None
        req = GetEntityState.Request()
        req.name = ROBOT_ENTITY
        req.reference_frame = "world"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return None
        state = future.result().state
        q = state.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (state.pose.position.x, state.pose.position.y,
                state.pose.position.z, yaw)

    def teleport(self) -> bool:
        current = self._entity_pose()
        if current is None:
            self.get_logger().error("cannot read current entity pose")
            return False
        self.get_logger().info(
            f"current entity pose: x={current[0]:.4f} y={current[1]:.4f} "
            f"z={current[2]:.4f} yaw={current[3]:.4f}")
        client = self.create_client(SetEntityState, "/set_entity_state")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/set_entity_state unavailable")
            return False
        request = SetEntityState.Request()
        request.state.name = ROBOT_ENTITY
        request.state.reference_frame = "world"
        request.state.pose.position.x = self._target[0]
        request.state.pose.position.y = self._target[1]
        # Keep the current z so a partial-drop fixture pose does not change
        # the robot's running height.
        request.state.pose.position.z = current[2]
        request.state.pose.orientation.z = math.sin(self._target[2] / 2.0)
        request.state.pose.orientation.w = math.cos(self._target[2] / 2.0)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().error("set_entity_state timed out")
            return False
        response = future.result()
        if response is None or not bool(response.success):
            self.get_logger().error(
                f"teleport failed: {getattr(response, 'status_message', '?')}")
            return False
        self.get_logger().info("teleport sent")
        return True

    def _body_pose_in_odom(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                "odom", BASE_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = tf.transform.translation
        return (p.x, p.y, yaw)

    def wait_for_odom(self, timeout_sec: float = 8.0) -> bool:
        """Wait until odom->base_link matches the teleported pose.

        base_link is a fixed -pi/2 child of body, so odom->base_link reports
        body_yaw - pi/2: compare against ``self._base_yaw`` (base-frame yaw).
        The planar mover publishes ~20 Hz; seeding AMCL before a fresh frame
        lands makes it solve map->odom against the pre-teleport odom pose.
        """
        tx, ty = self._target[0], self._target[1]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            pose = self._body_pose_in_odom()
            if pose is not None:
                dx, dy = pose[0] - tx, pose[1] - ty
                dyaw = (pose[2] - self._base_yaw + math.pi) % (2.0 * math.pi) - math.pi
                if dx * dx + dy * dy <= 0.06 ** 2 and abs(dyaw) <= 0.15:
                    self.get_logger().info(
                        f"odom settled at ({pose[0]:.4f}, {pose[1]:.4f}, "
                        f"yaw {pose[2]:.4f})")
                    return True
        self.get_logger().error(
            f"odom did not settle on teleported pose within {timeout_sec}s "
            f"(last: {pose})")
        return False

    def seed_amcl(self, pin: bool = True) -> None:
        """Publish a zero-stamped initial pose (tf2 'use latest' semantics).

        QoS must be RELIABLE: nav2_amcl subscribes /initialpose with
        SystemDefaultsQoS and silently drops best-effort publishers
        ("offering incompatible QoS ... RELIABILITY_QOS_POLICY").
        ``pin=True`` tightens the covariance so scan matching cannot drag the
        hypothesis away from the known teleport pose.
        """
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = self._target[0]
        pose.pose.pose.position.y = self._target[1]
        pose.pose.pose.orientation.z = math.sin(self._base_yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(self._base_yaw / 2.0)
        if pin:
            pose.pose.covariance[0] = 0.0001   # 1 cm std
            pose.pose.covariance[7] = 0.0001
            pose.pose.covariance[35] = 0.0001
        else:
            pose.pose.covariance[0] = 0.25
            pose.pose.covariance[7] = 0.25
            pose.pose.covariance[35] = 0.06853891945200942
        publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(depth=10,
                       reliability=QoSReliabilityPolicy.RELIABLE))
        # Give the discovery handshake time before the first publish; AMCL
        # logs a compatibility warning on first contact otherwise.
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        for _ in range(5):
            publisher.publish(pose)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info("zero-stamped /initialpose published x5 "
                               f"(pin={pin})")

    def verify(self, timeout_sec: float = 6.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self._tf_buffer.lookup_transform(
                    "map", BASE_FRAME, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.4))
            except TransformException:
                continue
            p = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            error = math.hypot(p.x - self._target[0], p.y - self._target[1])
            print(f"map->base_link: x={p.x:.4f} y={p.y:.4f} yaw={yaw:.4f} "
                  f"(target error {error:.4f} m)")
            if error < 0.05:
                return True
        return False

    def run(self) -> int:
        if not self.teleport():
            return 1
        if not self.wait_for_odom():
            return 1
        self.seed_amcl()
        ok = self.verify()
        # Report map->odom as the authority check
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            p = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            print(f"map->odom: x={p.x:.5f} y={p.y:.5f} yaw={yaw:.5f}")
        except TransformException as error:
            print(f"map->odom still unavailable: {str(error)[:80]}")
            return 1
        return 0 if ok else 1


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recover AMCL after a bad teleport seed (see module doc).")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--body-yaw", type=float, required=True,
                        help="robot body yaw in world/map frame")
    parser.add_argument("--base-yaw", type=float, required=True,
                        help="base_link yaw (body - pi/2), used in the seed")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    rclpy.init()
    try:
        recoverer = AmclRecoverer(args.x, args.y, args.body_yaw, args.base_yaw)
        return recoverer.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
