
"""
Zenoh CDR-to-JSON Proxy with plugin-style topic registration.

Provides:
  - TopicRegistry with @register_topic decorator for easy extension
  - Dynamic loading of standard and custom ROS2 message types
  - Multi-topic subscription via Zenoh
  - Extensible handler pipeline for post-deserialization processing

Note:
  ``rclpy`` and ``zenoh`` are lazy-imported — you can use the registry and
  loader on any machine, and only need those dependencies at runtime.
"""

from .registry import TopicRegistry, TopicRegistration, get_registry
from .loader import load_message_type, add_message_path, load_bulk_mapping, list_cached_types

# Lazy imports for modules that require rclpy / zenoh at runtime
# Use the module-level getters below instead of direct imports.


def _get_converter():
    """Lazy-load the converter module (requires rclpy)."""
    from . import converter as _m
    return _m


def _get_handler():
    """Lazy-load the handler module (requires rclpy for register_standard_types)."""
    from . import handler as _m
    return _m


def _get_subscriber():
    """Lazy-load the subscriber module (requires zenoh)."""
    from . import subscriber as _m
    return _m


def _get_runner():
    """Lazy-load the runner module (requires zenoh)."""
    from . import runner as _m
    return _m


# Convenience accessors — callable just like the original classes/functions
def msg_to_dict(msg):
    """Recursively convert a ROS2 message to plain dict.  (Lazy — requires rclpy at call time.)"""
    return _get_converter().msg_to_dict(msg)


def deserialize_and_convert(payload_bytes, registration, topic):
    """CDR bytes → ROS2 msg → dict → handler → JSON.  (Lazy — requires rclpy at call time.)"""
    return _get_converter().deserialize_and_convert(payload_bytes, registration, topic)


def register_standard_types():
    """Register standard ROS2 message types.  (Lazy — requires rclpy at call time.)"""
    return _get_handler().register_standard_types()


# Classes/functions that require zenoh are NOT imported at module level —
# import them explicitly when needed:
#
#     from zenoh_proxy.runner import ProxyRunner
#     from zenoh_proxy.subscriber import SubscriptionManager

__all__ = [
    # Always available (no external deps)
    "TopicRegistry",
    "TopicRegistration",
    "get_registry",
    "load_message_type",
    "add_message_path",
    "load_bulk_mapping",
    "list_cached_types",
    # Lazy accessors
    "msg_to_dict",
    "deserialize_and_convert",
    "register_standard_types",
]
