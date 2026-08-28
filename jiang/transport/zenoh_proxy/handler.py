
"""
Built-in handlers and standard type registration.

Usage::

    from transport.zenoh_proxy.handler import register_standard_types
    register_standard_types()
"""

from __future__ import annotations

import time
from typing import Any, Dict

from .registry import get_registry
from .loader import load_message_type


# ============================================================================
# Example built-in handlers
# ============================================================================

def add_timestamp(topic: str, data: Dict[str, Any]) -> Dict[str, Any]:
    """Add a server-side reception timestamp to every message."""
    data["_proxy_ts"] = time.time()
    return data


def flatten_twist(topic: str, data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Flatten nested Vector3 fields in Twist messages.

    Flatten the ``linear`` and ``angular`` fields
    for cleaner JSON output.

    Before: ``{"linear": {"x": 1, "y": 0, "z": 0}, "angular": {...}}``
    After:  ``{"linear_x": 1, "linear_y": 0, "linear_z": 0, "angular_x": ...}``
    """
    for prefix in ("linear", "angular"):
        if prefix in data and isinstance(data[prefix], dict):
            for axis in ("x", "y", "z"):
                data[f"{prefix}_{axis}"] = data[prefix].get(axis, 0.0)
            del data[prefix]
    return data


def flatten_joint_state(topic: str, data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert parallel arrays to key-value dicts for browser charting.

    Before: ``{"name": ["j1","j2"], "position": [1.0, 2.0], "velocity": [...]}``
    After:  ``{"positions": {"j1": 1.0, "j2": 2.0}, "velocities": {...}}``

    The original ``name``, ``position``, ``velocity``, ``effort`` arrays are
    preserved alongside the new key-value maps so nothing is lost.
    """
    names = data.get("name", [])
    for field, out_key in (("position", "positions"),
                           ("velocity", "velocities"),
                           ("effort", "efforts")):
        arr = data.get(field, [])
        if isinstance(arr, list) and isinstance(names, list):
            data[out_key] = {n: v for n, v in zip(names, arr)}
    return data


# ============================================================================
# Standard type registration
# ============================================================================

def register_standard_types() -> None:
    """
    Register all standard ROS2 message types used by the project.

    .. warning::

       Topic names alone cannot reliably determine the ROS2 message type.
       Register exact topic patterns when multiple message types could use
       the same final topic name.

       Use **exact** topic patterns where possible, and consult the bridge's
       REST admin API (``http://localhost:8000/@/**``) to verify the actual
       ROS2 type for each topic.
    """
    registry = get_registry()

    # Load standard message types dynamically
    Twist = load_message_type("geometry_msgs.msg.Twist")
    Odometry = load_message_type("nav_msgs.msg.Odometry")

    # -- cmd_vel topics → Twist -------------------------------------------
    if Twist is not None:
        registry.register(
            "*/cmd_vel", Twist,
            description="Any namespace cmd_vel",
        )

    # -- odom topics → Odometry -------------------------------------------
    if Odometry is not None:
        registry.register(
            "*/odom", Odometry,
            handler=add_timestamp,
            description="Odometry (any namespace)",
        )

    # -- joint_states → JointState (any namespace) -----------------------
    JointState = load_message_type("sensor_msgs.msg.JointState")
    if JointState is not None:
        registry.register(
            "*/joint_states", JointState,
            handler=add_timestamp,
            description="JointState (any namespace)",
        )

    # -- joint_trajectory → JointTrajectory (any namespace) --------------
    JointTrajectory = load_message_type("trajectory_msgs.msg.JointTrajectory")
    if JointTrajectory is not None:
        registry.register(
            "*/joint_trajectory", JointTrajectory,
            handler=add_timestamp,
            description="JointTrajectory (any namespace)",
        )

    # -- geometry_msgs/Pose — NOT registered via wildcard! ----------------
    # Register explicitly with the EXACT topic name when needed:
    #   Pose = load_message_type("geometry_msgs.msg.Pose")
    #   registry.register("your_robot/pose", Pose)

    # Print summary
    all_none = (
        Twist is None and Odometry is None
        and JointState is None and JointTrajectory is None
    )
    if all_none:
        print(
            "NOTE: Could not load any standard ROS2 message types — "
            "is rclpy installed and the ROS2 environment sourced?\n"
            "  Standard types will be available once rclpy is importable.\n"
            "  Custom types registered via load_message_type() are unaffected."
        )
        return

    print("Standard types registered:")
    for r in registry.registrations:
        print(f"  {r}")
