
"""
Dynamic importer for ROS2 message types.

Supports both standard ROS2 message packages (when the workspace is sourced)
and custom message packages (via ``add_message_path()``).

Usage::

    from transport.zenoh_proxy.loader import load_message_type, add_message_path

    # Standard type (works when ROS2 is sourced)
    Twist = load_message_type("geometry_msgs.msg.Twist")

    # Custom type from an unsourced workspace
    add_message_path("./ros2_ws/install/lib/python3.10/site-packages")
    BatteryStatus = load_message_type("my_msgs.msg.BatteryStatus")
"""

from __future__ import annotations

import importlib
import os
import sys
from typing import Dict, List, Optional, Type

# ---------------------------------------------------------------------------
# Module-level cache and search paths
# ---------------------------------------------------------------------------

#: Cache of loaded message types: ``{"package.MsgName": <class>}``
_msg_type_cache: Dict[str, Type] = {}

#: Extra directories to search for custom message packages
_custom_search_paths: List[str] = []


# ============================================================================
# Public API
# ============================================================================

def add_message_path(path: str) -> None:
    """Add a directory to the search path for custom ROS2 message packages.

    This enables loading messages from non-standard locations, e.g. a ROS2
    workspace that hasn't been sourced.

    Args:
        path: Directory containing message Python packages (e.g.
              ``./ros2_ws/install/lib/python3.10/site-packages``).
    """
    abs_path = os.path.abspath(path)
    if abs_path not in _custom_search_paths:
        _custom_search_paths.append(abs_path)
        if abs_path not in sys.path:
            sys.path.insert(0, abs_path)


def load_message_type(
    type_str: str,
    search_custom: bool = True,
) -> Optional[Type]:
    """Dynamically import a ROS2 message type by its fully-qualified name.

    Args:
        type_str: e.g. ``"geometry_msgs.msg.Twist"`` or
                  ``"my_msgs.msg.BatteryStatus"``.
        search_custom: Whether to also search paths registered via
                       ``add_message_path()``.

    Returns:
        The message class, or ``None`` if not found.
    """
    # Return cached result if available
    if type_str in _msg_type_cache:
        return _msg_type_cache[type_str]

    # Strategy 1: standard import (works when ROS2 env is sourced)
    cls = _try_standard_import(type_str)
    if cls is not None:
        _msg_type_cache[type_str] = cls
        return cls

    # Strategy 2: search custom paths
    if search_custom and _custom_search_paths:
        cls = _try_custom_import(type_str)
        if cls is not None:
            _msg_type_cache[type_str] = cls
            return cls

    return None


def load_bulk_mapping(config: Dict[str, str]) -> Dict[str, Type]:
    """Load a ``{topic_pattern: type_string}`` mapping into ``{pattern: class}``.

    Example::

        mapping = load_bulk_mapping({
            "/robot/cmd_vel": "geometry_msgs.msg.Twist",
            "/robot/battery": "my_msgs.msg.BatteryStatus",
        })

    Types that fail to load are reported as warnings but do **not** halt
    the process — they can be retried later after fixing the path.

    Returns:
        Dict of ``{pattern: msg_class}`` for successfully loaded types.
    """
    result: Dict[str, Type] = {}
    errors: List[str] = []

    for pattern, type_str in config.items():
        cls = load_message_type(type_str)
        if cls is None:
            errors.append(f"  {pattern}: {type_str} (not found)")
        else:
            result[pattern] = cls

    if errors:
        print("WARNING: Could not load the following message types:")
        for err in errors:
            print(err)

    return result


def list_cached_types() -> List[str]:
    """Return a list of cached message type strings."""
    return sorted(_msg_type_cache.keys())


def clear_cache() -> int:
    """Clear the message type cache.  Returns the number of entries cleared."""
    count = len(_msg_type_cache)
    _msg_type_cache.clear()
    return count


# ============================================================================
# Internal helpers
# ============================================================================

def _try_standard_import(type_str: str) -> Optional[Type]:
    """Attempt a standard ``importlib.import_module()`` import."""
    try:
        module_path, class_name = type_str.rsplit(".", 1)
        module = importlib.import_module(module_path)
        cls = getattr(module, class_name)
        return cls
    except (ImportError, AttributeError, ValueError):
        return None


def _try_custom_import(type_str: str) -> Optional[Type]:
    """Attempt to find and load the type from custom search paths."""
    original_path = list(sys.path)

    for search_path in _custom_search_paths:
        try:
            # Ensure the custom path is at the front of sys.path for this attempt
            if search_path not in sys.path:
                sys.path.insert(0, search_path)

            module_path, class_name = type_str.rsplit(".", 1)
            module = importlib.import_module(module_path)
            cls = getattr(module, class_name, None)
            if cls is not None:
                return cls
        except (ImportError, AttributeError, ValueError):
            continue
        finally:
            # Restore original sys.path to avoid pollution
            sys.path[:] = original_path

    return None
