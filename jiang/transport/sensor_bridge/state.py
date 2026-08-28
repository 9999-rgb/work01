"""Thread-safe latest-value storage shared by ROS 2 and the web server."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Dict, Optional, Tuple


CameraSnapshot = Tuple[int, Optional[bytes], Optional[Dict[str, Any]]]
LidarSnapshot = Tuple[int, Optional[Dict[str, Any]], Optional[str]]
DEFAULT_SENSOR_STALE_AFTER_SECONDS = 5.0


class SensorStreamState:
    """Keep only the latest camera frame and lidar scan."""

    def __init__(
        self,
        *,
        stale_after_seconds: float = DEFAULT_SENSOR_STALE_AFTER_SECONDS,
    ) -> None:
        if (
            isinstance(stale_after_seconds, bool)
            or not isinstance(stale_after_seconds, (int, float))
            or not math.isfinite(float(stale_after_seconds))
            or float(stale_after_seconds) <= 0.0
        ):
            raise ValueError(
                "stale_after_seconds must be a positive finite number."
            )
        self._stale_after_seconds = float(stale_after_seconds)
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

    def fresh_camera_snapshot(self) -> CameraSnapshot:
        """Return the latest frame, hiding a frozen last value fail-closed."""
        now = time.monotonic()
        with self._lock:
            jpeg = self._camera_jpeg
            if not self._is_fresh(jpeg is not None, self._camera_received_at, now):
                jpeg = None
            return self._camera_sequence, jpeg, self._camera_metadata

    def fresh_lidar_snapshot(self) -> LidarSnapshot:
        """Return the latest scan, hiding a frozen last value fail-closed."""
        now = time.monotonic()
        with self._lock:
            payload = self._lidar_payload
            json_payload = self._lidar_json
            if not self._is_fresh(
                payload is not None,
                self._lidar_received_at,
                now,
            ):
                payload = None
                json_payload = None
            return self._lidar_sequence, payload, json_payload

    def health(self) -> Dict[str, Any]:
        """Return readiness and data-age information."""
        now = time.monotonic()
        with self._lock:
            camera_received = self._camera_jpeg is not None
            lidar_received = self._lidar_payload is not None
            camera_age = (
                max(0.0, now - self._camera_received_at)
                if camera_received
                else None
            )
            lidar_age = (
                max(0.0, now - self._lidar_received_at)
                if lidar_received
                else None
            )
            camera_ready = self._is_fresh(
                camera_received,
                self._camera_received_at,
                now,
            )
            lidar_ready = self._is_fresh(
                lidar_received,
                self._lidar_received_at,
                now,
            )
            return {
                "status": (
                    "ok" if camera_ready and lidar_ready else "degraded"
                ),
                "camera": {
                    "ready": camera_ready,
                    "received": camera_received,
                    "stale": camera_received and not camera_ready,
                    "sequence": self._camera_sequence,
                    "age_seconds": camera_age,
                },
                "lidar": {
                    "ready": lidar_ready,
                    "received": lidar_received,
                    "stale": lidar_received and not lidar_ready,
                    "sequence": self._lidar_sequence,
                    "age_seconds": lidar_age,
                },
                "stale_after_seconds": self._stale_after_seconds,
            }

    def _is_fresh(self, present: bool, received_at: float, now: float) -> bool:
        return (
            present
            and max(0.0, now - received_at) <= self._stale_after_seconds
        )
