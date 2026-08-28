"""Shared construction of explicitly connected Zenoh client sessions."""

from __future__ import annotations

import json
from typing import Any


def configure_client_session(config: Any, connect_endpoint: str) -> Any:
    """Disable multicast scouting and connect only to the named endpoint."""
    if not isinstance(connect_endpoint, str) or not connect_endpoint.strip():
        raise ValueError("Zenoh connect endpoint must be a non-empty string.")
    config.insert_json5("mode", '"client"')
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5(
        "connect/endpoints",
        json.dumps([connect_endpoint.strip()]),
    )
    return config
