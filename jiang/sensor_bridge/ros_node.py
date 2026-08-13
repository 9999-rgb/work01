"""ROS 2 subscriptions and browser-format conversion for web sensors."""

from __future__ import annotations

import io
import logging
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import rclpy
from PIL import Image as PillowImage
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
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


logger = logging.getLogger(__name__)

ExecutorFatalCallback = Callable[[str, BaseException], None]


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

    # A BEST_EFFORT reader is compatible with both BEST_EFFORT and RELIABLE
    # writers.  Requesting RELIABLE here would not match the common
    # BEST_EFFORT sensor publishers used by Gazebo and physical cameras.
    CAMERA_QOS = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
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
        context: Optional[Context] = None,
    ) -> None:
        super().__init__("xczs_web_sensor_stream", context=context)
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
        fatal_callback: Optional[ExecutorFatalCallback] = None,
    ) -> None:
        self._fatal_callback = fatal_callback
        self._fatal_lock = threading.Lock()
        self._fatal_error: Optional[Dict[str, str]] = None
        self._context = Context()
        # 与控制网关一样使用私有 context，避免一个子系统 shutdown 默认
        # context 时使同进程内其他 ROS 节点失效。进程信号由 uvicorn 处理。
        rclpy.init(
            context=self._context,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        self._node: Optional[SensorStreamNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        try:
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
        except Exception:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=0.0)
            if self._node is not None:
                self._node.destroy_node()
            self._context.shutdown()
            raise
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._spin_safe,
            name="web-sensor-ros-executor",
            daemon=True,
        )
        self._started = False
        self._stopped = False

    def _spin_safe(self) -> None:
        """Spin the executor and promote an unexpected exit to fatal state."""
        try:
            assert self._executor is not None
            while self._context.ok() and not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.1)
            if not self._stop_event.is_set():
                raise RuntimeError(
                    "Sensor ROS executor stopped before runtime shutdown."
                )
        except BaseException as error:  # noqa: BLE001 - daemon fault boundary
            self._record_fatal(error)

    def _record_fatal(self, error: BaseException) -> bool:
        """Record and report the first non-shutdown executor failure."""
        if self._stop_event.is_set():
            return False
        fatal_error = {
            "component": "sensor_ros_executor",
            "type": type(error).__name__,
            "message": str(error) or repr(error),
        }
        with self._fatal_lock:
            if self._fatal_error is not None:
                return False
            self._fatal_error = fatal_error

        logger.critical(
            "Sensor ROS executor terminated unexpectedly: %s",
            fatal_error["message"],
            exc_info=(type(error), error, error.__traceback__),
        )
        if self._fatal_callback is not None:
            try:
                self._fatal_callback("sensor_ros_executor", error)
            except BaseException:  # noqa: BLE001 - preserve recorded fault
                logger.exception("Sensor executor fatal callback failed")
        return True

    def health(self) -> Dict[str, Any]:
        """Return the runtime thread's independently tracked health state."""
        with self._fatal_lock:
            fatal_error = (
                dict(self._fatal_error)
                if self._fatal_error is not None
                else None
            )
        healthy = fatal_error is None
        return {
            "status": "ok" if healthy else "error",
            "healthy": healthy,
            "thread_alive": self._thread.is_alive(),
            "error": fatal_error,
        }

    def start(self) -> "SensorRosRuntime":
        if not self._started:
            self._thread.start()
            self._started = True
        return self

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._stop_event.set()
        if self._executor is not None:
            self._executor.wake()
        if self._started:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                # 线程仍可能在 rclpy 中访问 guard condition；保留 ROS 对象，
                # 避免 teardown use-after-free。稍后再次 stop 仍可继续回收。
                self._stopped = False
                raise RuntimeError("传感器 ROS executor 线程未在 3 秒内停止。")
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=3.0)
        if self._node is not None:
            self._node.destroy_node()
        self._context.shutdown()
