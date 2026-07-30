"""Thread-safe latest-value storage shared by ROS 2 and the web server."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional, Tuple


CameraSnapshot = Tuple[int, Optional[bytes], Optional[Dict[str, Any]]]
LidarSnapshot = Tuple[int, Optional[Dict[str, Any]], Optional[str]]


class SensorStreamState:
    """Keep only the latest camera frame and lidar scan."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._camera_sequence = 0
        self._camera_jpeg: Optional[bytes] = None
        self._camera_metadata: Optional[Dict[str, Any]] = None
        self._camera_received_at = 0.0
        self._lidar_sequence = 0
        self._lidar_payload: Optional[Dict[str, Any]] = None
        self._lidar_json: Optional[str] = None
        self._lidar_received_at = 0.0

    def update_camera(
        self,
        jpeg: bytes,
        metadata: Dict[str, Any],
    ) -> None:
        """Replace the current browser-ready JPEG frame."""
        with self._lock:
            self._camera_sequence += 1
            self._camera_jpeg = jpeg
            self._camera_metadata = metadata
            self._camera_received_at = time.monotonic()

    def update_lidar(self, payload: Dict[str, Any]) -> None:
        """Replace the current JSON lidar scan."""
        json_payload = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        with self._lock:
            self._lidar_sequence += 1
            self._lidar_payload = payload
            self._lidar_json = json_payload
            self._lidar_received_at = time.monotonic()

    def camera_snapshot(self) -> CameraSnapshot:
        """Return the latest immutable camera frame."""
        with self._lock:
            return (
                self._camera_sequence,
                self._camera_jpeg,
                self._camera_metadata,
            )

    def lidar_snapshot(self) -> LidarSnapshot:
        """Return the latest immutable lidar scan."""
        with self._lock:
            return (
                self._lidar_sequence,
                self._lidar_payload,
                self._lidar_json,
            )

    def health(self) -> Dict[str, Any]:
        """Return readiness and data-age information."""
        now = time.monotonic()
        with self._lock:
            camera_age = (
                now - self._camera_received_at
                if self._camera_received_at > 0.0
                else None
            )
            lidar_age = (
                now - self._lidar_received_at
                if self._lidar_received_at > 0.0
                else None
            )
            return {
                "status": "ok",
                "camera": {
                    "ready": self._camera_jpeg is not None,
                    "sequence": self._camera_sequence,
                    "age_seconds": camera_age,
                },
                "lidar": {
                    "ready": self._lidar_payload is not None,
                    "sequence": self._lidar_sequence,
                    "age_seconds": lidar_age,
                },
            }
