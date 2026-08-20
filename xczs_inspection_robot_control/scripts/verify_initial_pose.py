#!/usr/bin/env python3
"""Verify the spawn-time arm pose without commanding any robot motion.

GazeboSystem applies ``ros2_control`` state-interface ``initial_value`` entries
while loading the model.  This short-lived startup guard waits for both arm
controllers and the joint-state broadcaster, then checks that the measured
arm joints match the shared initial-position file.  It never sends a
trajectory; a mismatch exits non-zero so the parent launch shuts the stack
down instead of hiding a broken initialisation behind corrective motion.
"""

import argparse
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


DESCRIPTION_PACKAGE = "xczs_inspection_robot_description"
DEFAULT_JOINT_STATE_TOPIC = "/xczs/joint_states"
DEFAULT_TIMEOUT_SEC = 15.0
DEFAULT_TOLERANCE = 0.02
POLL_INTERVAL_SEC = 0.05

ARM_CONTROLLERS = (
    (
        "left arm",
        "/xczs/left_arm_controller/follow_joint_trajectory",
    ),
    (
        "right arm",
        "/xczs/right_arm_controller/follow_joint_trajectory",
    ),
)
ARM_JOINTS = tuple(
    f"{side}_arm_{index}_joint"
    for side in ("l", "r")
    for index in range(7)
)


class InitialPoseError(RuntimeError):
    """Raised when spawn-time state is absent, malformed, or out of tolerance."""


def default_positions_file():
    return (
        Path(get_package_share_directory(DESCRIPTION_PACKAGE))
        / "config"
        / "initial_positions.yaml"
    )


def load_expected_positions(path):
    path = Path(path).expanduser()
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        positions = document["initial_positions"]
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        raise InitialPoseError(
            f"could not read initial positions '{path}': {error}"
        ) from error
    if not isinstance(positions, dict):
        raise InitialPoseError("initial_positions must be a mapping")

    missing = [joint for joint in ARM_JOINTS if joint not in positions]
    extra = sorted(set(positions) - set(ARM_JOINTS))
    if missing or extra:
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extra:
            details.append("unexpected " + ", ".join(extra))
        raise InitialPoseError("invalid arm joint set: " + "; ".join(details))

    result = {}
    for joint in ARM_JOINTS:
        value = positions[joint]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise InitialPoseError(f"{joint} must be a finite number")
        value = float(value)
        if not math.isfinite(value):
            raise InitialPoseError(f"{joint} must be a finite number")
        result[joint] = value
    return result


def _joint_state_positions(message):
    if len(message.name) != len(message.position):
        return None
    positions = {}
    for name, value in zip(message.name, message.position):
        if name in positions or not math.isfinite(value):
            return None
        positions[name] = float(value)
    return positions


def verify_initial_pose(
    node,
    expected_positions,
    *,
    joint_state_topic=DEFAULT_JOINT_STATE_TOPIC,
    timeout_sec=DEFAULT_TIMEOUT_SEC,
    tolerance=DEFAULT_TOLERANCE,
):
    """Wait for controllers/current state and verify without sending goals."""
    if not joint_state_topic.startswith("/"):
        raise InitialPoseError("joint state topic must be absolute")
    if not math.isfinite(timeout_sec) or timeout_sec < 0.0:
        raise InitialPoseError("timeout must be finite and non-negative")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise InitialPoseError("tolerance must be finite and positive")

    clients = {
        label: ActionClient(node, FollowJointTrajectory, action_name)
        for label, action_name in ARM_CONTROLLERS
    }
    latest_positions = {}
    malformed_state_seen = False

    def receive_joint_state(message):
        nonlocal latest_positions, malformed_state_seen
        positions = _joint_state_positions(message)
        if positions is None:
            malformed_state_seen = True
            return
        latest_positions.update(positions)

    subscription = node.create_subscription(
        JointState,
        joint_state_topic,
        receive_joint_state,
        10,
    )
    # Keep the subscription alive until the function returns.  Explicitly
    # reference it because some rclpy test doubles do not retain subscriptions.
    _ = subscription
    deadline = time.monotonic() + timeout_sec
    last_errors = {}

    while True:
        unavailable = [
            label for label, client in clients.items()
            if not client.server_is_ready()
        ]
        missing = [
            joint for joint in expected_positions
            if joint not in latest_positions
        ]
        if not unavailable and not missing:
            last_errors = {
                joint: latest_positions[joint] - target
                for joint, target in expected_positions.items()
                if abs(latest_positions[joint] - target) > tolerance
            }
            if not last_errors:
                return

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            details = []
            if unavailable:
                details.append(
                    "controller action unavailable: " + ", ".join(unavailable)
                )
            if missing:
                details.append("joint state missing: " + ", ".join(missing))
            if last_errors:
                errors = ", ".join(
                    f"{joint}={error:+.4f} rad"
                    for joint, error in sorted(last_errors.items())
                )
                details.append(
                    f"spawn pose exceeds {tolerance:.4f} rad tolerance: "
                    + errors
                )
            if malformed_state_seen:
                details.append("malformed joint-state sample received")
            if not details:
                details.append("no valid initial state received")
            raise InitialPoseError("; ".join(details))

        rclpy.spin_once(
            node,
            timeout_sec=min(POLL_INTERVAL_SEC, remaining),
        )


def _parser():
    parser = argparse.ArgumentParser(
        description="Verify the Gazebo spawn-time fortune-cat arm pose."
    )
    parser.add_argument("--positions-file", type=Path)
    parser.add_argument(
        "--joint-state-topic",
        default=DEFAULT_JOINT_STATE_TOPIC,
    )
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_SEC)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    return parser


def _log_startup_error(node, message):
    if node is None:
        print(f"Initial pose verification failed: {message}", file=sys.stderr)
    else:
        node.get_logger().error(f"Initial pose verification failed: {message}")


def main(args=None):
    options = _parser().parse_args(args)
    node = None
    initialized = False
    try:
        positions_file = options.positions_file or default_positions_file()
        expected_positions = load_expected_positions(positions_file)
        rclpy.init()
        initialized = True
        node = Node("xczs_initial_pose_verifier")
        node.get_logger().info(
            "Verifying spawn-time fortune-cat pose (no trajectory is sent)..."
        )
        verify_initial_pose(
            node,
            expected_positions,
            joint_state_topic=options.joint_state_topic,
            timeout_sec=options.timeout,
            tolerance=options.tolerance,
        )
        node.get_logger().info("Spawn-time fortune-cat pose verified.")
        return 0
    except KeyboardInterrupt:
        _log_startup_error(node, "interrupted")
        return 130
    except Exception as error:
        _log_startup_error(node, str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
