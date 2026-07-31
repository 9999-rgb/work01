"""ROS 2 subscriptions and browser-format conversion for web sensors."""

from __future__ import annotations

import io
import math
import threading
import time
import traceback
from typing import Any, Dict, List, Optional

import rclpy
from PIL import Image as PillowImage
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan

from .state import SensorStreamState


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def _alternate_camera_topic(topic: str) -> str:
    """Return the other common camera topic variant.

    Some gazebo_ros_camera plugin versions include the <camera_name> as a
    path segment (``/ns/cam_name/image_raw``) while others publish directly
    under the namespace (``/ns/image_raw``).  Flip between the two so the
    bridge can subscribe to both.
    """
    parts = topic.rstrip("/").split("/")
    # Topic must end with …/image_raw and have at least a namespace above it.
    if len(parts) < 3 or parts[-1] != "image_raw":
        return topic
    # If there is a camera_name segment, drop it; otherwise insert one.
    # Heuristic: the camera_name is whatever sits between the namespace tail
    # and "image_raw".  We flip by either removing or inserting a guess.
    camera_name_guess = parts[-2]
    if camera_name_guess in ("image_raw", "camera"):
        # Looks like /ns/image_raw — try /ns/arm_camera/image_raw
        parts.insert(-1, "arm_camera")
    else:
        # Looks like /ns/<name>/image_raw — try /ns/image_raw
        del parts[-2]
    return "/".join(parts)


class SensorStreamNode(Node):
    """Convert ROS 2 Image and LaserScan messages for web clients."""

    # Camera data is continuous and bandwidth-heavy; use RELIABLE QoS so
    # that the subscriber always matches the Gazebo publisher regardless
    # of which reliability level the plugin declares.  (ros2 topic echo
    # auto-detects the publisher's QoS and upgrades accordingly.)
    CAMERA_QOS = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    def __init__(
        self,
        state: SensorStreamState,
        camera_topic: str,
        lidar_topic: str,
        jpeg_quality: int,
        camera_fps: float,
    ) -> None:
        super().__init__("xczs_web_sensor_stream")
        self._state = state
        self._jpeg_quality = max(1, min(95, jpeg_quality))
        self._camera_period = 1.0 / max(1.0, camera_fps)
        self._next_camera_encode_time = 0.0
        self._unsupported_encoding: Optional[str] = None
        self._camera_frame_count = 0
        self._camera_error_count = 0
        self._lidar_frame_count = 0
        self._active_camera_topic: Optional[str] = None

        # Subscribe to the primary camera topic *and* a fallback that
        # drops the camera_name path segment.  The exact topic structure
        # depends on the gazebo_ros_camera plugin version.
        self._camera_subscription = self.create_subscription(
            Image,
            camera_topic,
            self._make_camera_callback(camera_topic),
            self.CAMERA_QOS,
        )
        self._camera_fallback_sub: Optional[rclpy.subscription.Subscription] = None
        camera_fallback_topic = _alternate_camera_topic(camera_topic)
        if camera_fallback_topic != camera_topic:
            self._camera_fallback_sub = self.create_subscription(
                Image,
                camera_fallback_topic,
                self._make_camera_callback(camera_fallback_topic),
                self.CAMERA_QOS,
            )
            self.get_logger().info(
                f"Camera primary: {camera_topic}; "
                f"fallback: {camera_fallback_topic}"
            )
        else:
            self.get_logger().info(f"Camera stream source: {camera_topic}")

        self._lidar_subscription = self.create_subscription(
            LaserScan,
            lidar_topic,
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Lidar stream source: {lidar_topic}")

        # If no frame arrives within 8 seconds, log available image topics
        # so the operator can diagnose a topic-name mismatch.
        self._camera_diag_timer = self.create_timer(
            8.0,
            self._camera_diagnostic_callback,
        )
        self._camera_diag_fired = False

        # Periodically reset the active-topic lock so that a stale fallback
        # subscription doesn't permanently block the primary.
        self._topic_watchdog_timer = self.create_timer(
            15.0,
            self._topic_watchdog_callback,
        )

    def _make_camera_callback(self, topic: str):
        """Return a callback that only processes frames from the active topic."""
        def _callback(message: Image) -> None:
            # If we already locked onto a different topic, ignore this one.
            if (
                self._active_camera_topic is not None
                and self._active_camera_topic != topic
            ):
                return

            now = time.monotonic()
            if now < self._next_camera_encode_time:
                return

            try:
                image = self._decode_image(message)
            except ValueError as error:
                encoding = message.encoding.lower()
                self._camera_error_count += 1
                if encoding != self._unsupported_encoding:
                    self._unsupported_encoding = encoding
                    self.get_logger().warning(str(error))
                elif self._camera_error_count <= 3:
                    self.get_logger().warning(
                        f"Camera decode error (x{self._camera_error_count}): {error}"
                    )
                return
            except Exception as error:  # noqa: BLE001
                self._camera_error_count += 1
                self.get_logger().error(
                    f"Unexpected camera decode error: {error}"
                )
                return

            # Advance the rate-limit window ONLY after a successful decode
            # so that a failing subscription cannot starve a working one.
            self._next_camera_encode_time = now + self._camera_period

            try:
                output = io.BytesIO()
                image.save(
                    output,
                    format="JPEG",
                    quality=self._jpeg_quality,
                    optimize=False,
                )
                metadata = {
                    "frame_id": message.header.frame_id,
                    "stamp": _stamp_to_seconds(message.header.stamp),
                    "width": message.width,
                    "height": message.height,
                    "source_encoding": message.encoding,
                    "jpeg_quality": self._jpeg_quality,
                }
            except Exception as error:  # noqa: BLE001
                self._camera_error_count += 1
                self.get_logger().error(
                    f"Failed to encode camera frame as JPEG: {error}"
                )
                return

            self._state.update_camera(output.getvalue(), metadata)

            self._camera_frame_count += 1
            self._frame_advanced_since_watchdog = True
            if self._active_camera_topic is None:
                self._active_camera_topic = topic
                self.get_logger().info(
                    f"First camera frame on '{topic}': "
                    f"{message.width}x{message.height} {message.encoding} "
                    f"({len(output.getvalue())} bytes JPEG)"
                )

        return _callback

    def _camera_diagnostic_callback(self) -> None:
        """Log available image topics and subscription match status."""
        if self._camera_diag_fired:
            return
        self._camera_diag_fired = True
        if self._active_camera_topic is not None:
            return  # already receiving frames

        topic_names = self.get_topic_names_and_types()
        image_topics = [
            name for name, types in topic_names
            if "sensor_msgs/msg/Image" in types
        ]
        if image_topics:
            # Report publisher match count for each subscription
            primary_count = self.count_publishers(
                self._camera_subscription.topic_name
            )
            fallback_count = 0
            fallback_topic = ""
            if self._camera_fallback_sub is not None:
                fallback_topic = self._camera_fallback_sub.topic_name
                fallback_count = self.count_publishers(fallback_topic)
            self.get_logger().warning(
                "No camera frame received after 8 s. "
                f"Image topics on DDS: {', '.join(sorted(image_topics))}; "
                f"pub matches — primary({primary_count}), "
                f"fallback({fallback_topic}={fallback_count})"
            )
        else:
            self.get_logger().warning(
                "No camera frame received after 8 s, and no "
                "sensor_msgs/msg/Image publishers were discovered. "
                "Is the Gazebo camera sensor loaded and publishing?"
            )

    def _topic_watchdog_callback(self) -> None:
        """Reset active-camera-topic lock if frames stop arriving."""
        if self._active_camera_topic is None:
            return
        # If we have an active topic but haven't received a frame for
        # more than 30 seconds, reset the lock so the other subscription
        # gets a chance.
        if self._camera_frame_count > 0:
            self.get_logger().debug(
                f"Camera watchdog: {self._camera_frame_count} frames "
                f"on '{self._active_camera_topic}', "
                f"{self._camera_error_count} errors"
            )
        # The lock-in is a common cause of silent stalls when the
        # winning topic's publisher disappears.  Reset periodically
        # when the frame counter hasn't advanced (detected via a
        # companion variable set in the callback).
        if (
            self._active_camera_topic is not None
            and not getattr(self, "_frame_advanced_since_watchdog", True)
        ):
            self.get_logger().warning(
                f"Camera topic '{self._active_camera_topic}' may be "
                f"stale — resetting active-topic lock"
            )
            self._active_camera_topic = None
        self._frame_advanced_since_watchdog = False

    @staticmethod
    def _decode_image(message: Image) -> PillowImage.Image:
        encoding = message.encoding.lower()
        encoding_map = {
            "rgb8": ("RGB", "RGB"),
            "bgr8": ("RGB", "BGR"),
            "rgba8": ("RGBA", "RGBA"),
            "bgra8": ("RGBA", "BGRA"),
            "mono8": ("L", "L"),
        }
        image_modes = encoding_map.get(encoding)
        if image_modes is None:
            raise ValueError(
                f"Unsupported camera encoding: {message.encoding}"
            )

        mode, raw_mode = image_modes
        expected_size = int(message.step) * int(message.height)
        raw_data = bytes(message.data)
        if len(raw_data) < expected_size:
            raise ValueError(
                "Camera message is shorter than height * step"
            )
        image = PillowImage.frombytes(
            mode,
            (int(message.width), int(message.height)),
            raw_data,
            "raw",
            raw_mode,
            int(message.step),
            1,
        )
        if image.mode not in ("RGB", "L"):
            image = image.convert("RGB")
        return image

    def _lidar_callback(self, message: LaserScan) -> None:
        try:
            ranges: List[Optional[float]] = []
            valid_ranges: List[float] = []
            for raw_range in message.ranges:
                value = float(raw_range)
                if (
                    math.isfinite(value)
                    and message.range_min <= value <= message.range_max
                ):
                    ranges.append(value)
                    valid_ranges.append(value)
                else:
                    ranges.append(None)

            payload: Dict[str, Any] = {
                "type": "sensor_msgs/msg/LaserScan",
                "frame_id": message.header.frame_id,
                "stamp": _stamp_to_seconds(message.header.stamp),
                "angle_min": float(message.angle_min),
                "angle_max": float(message.angle_max),
                "angle_increment": float(message.angle_increment),
                "time_increment": float(message.time_increment),
                "scan_time": float(message.scan_time),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "sample_count": len(ranges),
                "valid_count": len(valid_ranges),
                "closest_range": min(valid_ranges) if valid_ranges else None,
                "ranges": ranges,
            }
            if message.intensities:
                payload["intensities"] = [
                    float(value) if math.isfinite(value) else None
                    for value in message.intensities
                ]
            self._state.update_lidar(payload)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(
                f"Lidar scan processing failed: {error}"
            )


class SensorRosRuntime:
    """Own a ROS 2 context and executor running in a background thread."""

    def __init__(
        self,
        state: SensorStreamState,
        camera_topic: str,
        lidar_topic: str,
        jpeg_quality: int,
        camera_fps: float,
    ) -> None:
        rclpy.init()
        self._node = SensorStreamNode(
            state=state,
            camera_topic=camera_topic,
            lidar_topic=lidar_topic,
            jpeg_quality=jpeg_quality,
            camera_fps=camera_fps,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._spin_safe,
            name="web-sensor-ros-executor",
            daemon=True,
        )
        self._started = False
        self._stopped = False

    def _spin_safe(self) -> None:
        """Spin the executor, logging any unhandled exception."""
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001
            traceback.print_exc()
            # Re-raise so the daemon thread's death is visible in the
            # process exit code; the parent process watchdog will notice.
            raise

    def start(self) -> "SensorRosRuntime":
        if not self._started:
            self._thread.start()
            self._started = True
        return self

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._executor.shutdown(timeout_sec=3.0)
        if self._started:
            self._thread.join(timeout=3.0)
        self._node.destroy_node()
        rclpy.shutdown()
