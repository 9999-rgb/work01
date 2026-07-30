"""ROS 2 subscriptions and browser-format conversion for web sensors."""

from __future__ import annotations

import io
import math
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from PIL import Image as PillowImage
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from .state import SensorStreamState


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


class SensorStreamNode(Node):
    """Convert ROS 2 Image and LaserScan messages for web clients."""

    def __init__(
        self,
        state: SensorStreamState,
        camera_topic: str,
        lidar_topic: str,
        jpeg_quality: int,
        camera_fps: float,
        context: Context,
    ) -> None:
        super().__init__("xczs_web_sensor_stream", context=context)
        self._state = state
        self._jpeg_quality = max(1, min(95, jpeg_quality))
        self._camera_period = 1.0 / max(1.0, camera_fps)
        self._next_camera_encode_time = 0.0
        self._unsupported_encoding: Optional[str] = None
        self._camera_subscription = self.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            lidar_topic,
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Camera stream source: {camera_topic}; "
            f"lidar stream source: {lidar_topic}"
        )

    def _camera_callback(self, message: Image) -> None:
        now = time.monotonic()
        if now < self._next_camera_encode_time:
            return
        self._next_camera_encode_time = now + self._camera_period

        try:
            image = self._decode_image(message)
        except ValueError as error:
            encoding = message.encoding.lower()
            if encoding != self._unsupported_encoding:
                self._unsupported_encoding = encoding
                self.get_logger().warning(str(error))
            return

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
        self._state.update_camera(output.getvalue(), metadata)

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
        self._context = Context()
        rclpy.init(context=self._context)
        self._node = SensorStreamNode(
            state=state,
            camera_topic=camera_topic,
            lidar_topic=lidar_topic,
            jpeg_quality=jpeg_quality,
            camera_fps=camera_fps,
            context=self._context,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="web-sensor-ros-executor",
            daemon=True,
        )
        self._started = False
        self._stopped = False

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
        self._context.shutdown()
