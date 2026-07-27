
"""
CDR deserialization and ROS2-message-to-dict conversion.

Provides reusable conversion utilities for the XCZS Zenoh bridge.
"""

from __future__ import annotations

import json
from typing import Any, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .registry import TopicRegistration

# Lazy — rclpy is only available on the Linux/ROS2 host, not during
# registration/config on other machines.
_deserialize_message = None


def _get_deserializer():
    """Lazy-load rclpy.serialization.deserialize_message."""
    global _deserialize_message
    if _deserialize_message is None:
        from rclpy.serialization import deserialize_message as dm
        _deserialize_message = dm
    return _deserialize_message


# ============================================================================
# ROS 2 message conversion
# ============================================================================

def msg_to_dict(msg: Any) -> Any:
    """Recursively convert a ROS2 message object to plain Python dicts/lists/scalars.

    Supports nested messages, numpy ndarray, array.array, tuple, and list.
    """
    # Primitive types — return as-is
    if isinstance(msg, (int, float, str, bool, type(None))):
        return msg

    # numpy ndarray (e.g. float64 arrays from JointState/Odometry)
    try:
        import numpy as np
        if isinstance(msg, np.ndarray):
            return msg.tolist()
    except ImportError:
        pass

    # Python built-in array.array (e.g. array('d', [...]))
    try:
        import array
        if isinstance(msg, array.array):
            return msg.tolist()
    except ImportError:
        pass

    # Tuples — recurse into each element
    if isinstance(msg, tuple):
        return [msg_to_dict(item) for item in msg]

    # Lists — recurse into each element
    if isinstance(msg, list):
        return [msg_to_dict(item) for item in msg]

    # ROS2 message objects (have get_fields_and_field_types)
    if hasattr(msg, "get_fields_and_field_types"):
        result = {}
        for field_name in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field_name)
            result[field_name] = msg_to_dict(value)
        return result

    # Fallback — try to convert to string, otherwise return as-is
    try:
        return str(msg)
    except Exception:
        return msg


# ============================================================================
# Full pipeline: CDR bytes → ROS2 msg → dict → handler → JSON string
# ============================================================================

def deserialize_and_convert(
    payload_bytes: bytes,
    registration: TopicRegistration,
    topic: str,
) -> str:
    """Deserialize CDR bytes, convert to dict, apply handler, serialize to JSON.

    Args:
        payload_bytes: Raw CDR bytes from Zenoh.
        registration: The matched ``TopicRegistration`` (carries msg_type and
                      handler).
        topic: The actual topic string (passed to handler).

    Returns:
        JSON-encoded string of the processed data.

    Raises:
        CDRTypeMismatchError: If the CDR payload doesn't match the expected
                              message type (likely a wrong type registration).
    """
    # Step 1: Deserialize CDR → ROS2 message object
    try:
        msg = _get_deserializer()(payload_bytes, registration.msg_type)
    except Exception as e:
        raise CDRTypeMismatchError(
            f"CDR deserialization failed for topic '{topic}':\n"
            f"  Expected type: {registration.msg_type.__module__}."
            f"{registration.msg_type.__name__}\n"
            f"  Payload size:  {len(payload_bytes)} bytes\n"
            f"  Pattern used:  {registration.pattern}\n"
            f"  Original error: {e}\n"
            f"Hint: Run 'curl -s http://localhost:8000/@/** | grep ros2_type' "
            f"to find the actual ROS2 type for this topic."
        ) from e

    # Step 2: ROS2 message → plain Python dict
    data = msg_to_dict(msg)

    # Step 3: Apply optional user-defined handler
    if registration.handler is not None:
        data = registration.handler(topic, data)

    # Step 4: Dict → JSON string
    return json.dumps(data)


class CDRTypeMismatchError(RuntimeError):
    """Raised when CDR bytes don't match the registered message type."""
    pass
