"""Shared validation for browser-visible Zenoh/ROS topic keys."""

from __future__ import annotations

import re


_ROS_KEY_SEGMENT_RE = re.compile(r"^[A-Za-z0-9_]+$")


def normalize_ros_key(key: str) -> str:
    """Normalize one exact ROS topic key and reject Zenoh expressions.

    The Web bridges intentionally expose exact topic subscriptions only.
    Wildcards, empty/path-traversal segments and arbitrary Zenoh key
    expressions would let an unauthenticated or mistakenly exposed helper
    subscribe beyond the monitor's intended topic boundary.
    """
    if not isinstance(key, str):
        raise ValueError("key 必须是字符串。")
    value = key[:-5] if key.endswith("/json") else key
    segments = value.split("/")
    if (
        not value
        or len(value) > 255
        or any(
            not segment
            or len(segment) > 64
            or _ROS_KEY_SEGMENT_RE.fullmatch(segment) is None
            for segment in segments
        )
    ):
        raise ValueError("key 必须是不含通配符的规范 ROS 话题。")
    return value
