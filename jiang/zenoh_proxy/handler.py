
"""
Built-in handlers and standard type registration.

The ``register_standard_types()`` function replaces the old hardcoded if-elif
chain (lines 69-76 of the original ``proxy.py``).

Usage::

    from zenoh_proxy.handler import register_standard_types
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
    """Flatten nested ``linear`` / ``angular`` Vector3 fields in Twist messages
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


# ============================================================================
# Standard type registration (replaces old if-elif chain)
# ============================================================================

def register_standard_types() -> None:
    """Register all standard ROS2 message types used by the project.

    .. warning::

       Topic names alone cannot reliably determine the ROS2 message type.
       For example, ``turtle1/pose`` is ``turtlesim/msg/Pose`` (5×float32),
       while ``robot/pose`` could be ``geometry_msgs/msg/Pose`` (7×float64).

       Use **exact** topic patterns where possible, and consult the bridge's
       REST admin API (``http://localhost:8000/@/**``) to verify the actual
       ROS2 type for each topic.
    """
    registry = get_registry()

    # Load standard message types dynamically
    Twist = load_message_type("geometry_msgs.msg.Twist")
    Odometry = load_message_type("nav_msgs.msg.Odometry")

    # -- TurtleSim Pose (turtlesim/msg/Pose: 5 float32 fields) -------------
    TurtlePose = load_message_type("turtlesim.msg.Pose")

    # -- TurtleSim Color (turtlesim/msg/Color: 3 uint8 fields) -------------
    Color = load_message_type("turtlesim.msg.Color")

    # -- cmd_vel topics → Twist -------------------------------------------
    if Twist is not None:
        registry.register(
            "turtle1/cmd_vel", Twist,
            description="TurtleSim velocity commands",
        )
        registry.register(
            "*/cmd_vel", Twist,
            description="Any namespace cmd_vel",
        )

    # -- TurtleSim pose → turtlesim/msg/Pose -------------------------------
    if TurtlePose is not None:
        registry.register(
            "turtle1/pose", TurtlePose,
            handler=add_timestamp,
            description="TurtleSim pose (turtlesim/msg/Pose)",
        )

    # -- TurtleSim color_sensor → turtlesim/msg/Color ----------------------
    if Color is not None:
        registry.register(
            "turtle1/color_sensor", Color,
            handler=add_timestamp,
            description="TurtleSim color sensor (turtlesim/msg/Color)",
        )

    # -- odom topics → Odometry -------------------------------------------
    if Odometry is not None:
        registry.register(
            "*/odom", Odometry,
            handler=add_timestamp,
            description="Odometry (any namespace)",
        )

    # -- geometry_msgs/Pose — NOT registered via wildcard! ----------------
    # Register explicitly with the EXACT topic name when needed:
    #   Pose = load_message_type("geometry_msgs.msg.Pose")
    #   registry.register("your_robot/pose", Pose)

    # Print summary
    all_none = Twist is None and Odometry is None and TurtlePose is None and Color is None
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
