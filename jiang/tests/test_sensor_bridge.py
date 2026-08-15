"""Sensor bridge topic and QoS contract tests."""

from __future__ import annotations

import unittest
import threading
from unittest.mock import MagicMock, call, patch

from rclpy.qos import ReliabilityPolicy

from sensor_bridge.ros_node import SensorStreamNode
from sensor_bridge.ros_node import SensorRosRuntime
from sensor_bridge.ros_node import _alternate_camera_topic


class SensorBridgeContractTest(unittest.TestCase):
    def test_camera_reader_matches_reliable_sensor_publishers(self) -> None:
        self.assertEqual(
            ReliabilityPolicy.RELIABLE,
            SensorStreamNode.CAMERA_QOS.reliability,
        )

    def test_camera_topic_fallback_switches_supported_layouts(self) -> None:
        self.assertEqual(
            "/xczs/camera/image_raw",
            _alternate_camera_topic(
                "/xczs/camera/arm_camera/image_raw"
            ),
        )
        self.assertEqual(
            "/xczs/camera/arm_camera/image_raw",
            _alternate_camera_topic("/xczs/camera/image_raw"),
        )

    def test_camera_watchdog_rebuilds_stalled_dds_readers(self) -> None:
        node = object.__new__(SensorStreamNode)
        old_primary = MagicMock()
        old_fallback = MagicMock()
        new_primary = MagicMock()
        new_fallback = MagicMock()
        node._camera_subscription = old_primary
        node._camera_fallback_sub = old_fallback
        node._camera_topic = "/camera/arm_camera/image_raw"
        node._camera_fallback_topic = "/camera/image_raw"
        node._camera_frame_count = 4
        node._camera_watchdog_frame_count = 4
        node._camera_error_count = 0
        node._active_camera_topic = node._camera_topic
        node._next_camera_encode_time = 123.0
        node.destroy_subscription = MagicMock()
        node.create_subscription = MagicMock(
            side_effect=[new_primary, new_fallback]
        )
        node.get_logger = MagicMock(return_value=MagicMock())

        node._topic_watchdog_callback()

        self.assertEqual(
            node.destroy_subscription.call_args_list,
            [call(old_primary), call(old_fallback)],
        )
        self.assertIs(node._camera_subscription, new_primary)
        self.assertIs(node._camera_fallback_sub, new_fallback)
        self.assertIsNone(node._active_camera_topic)
        self.assertEqual(node._next_camera_encode_time, 0.0)

    def test_camera_watchdog_keeps_readers_while_frames_advance(self) -> None:
        node = object.__new__(SensorStreamNode)
        node._camera_frame_count = 5
        node._camera_watchdog_frame_count = 4
        node._camera_error_count = 0
        node._active_camera_topic = "/camera/image_raw"
        node._reset_camera_subscriptions = MagicMock()
        node.get_logger = MagicMock(return_value=MagicMock())

        node._topic_watchdog_callback()

        node._reset_camera_subscriptions.assert_not_called()
        self.assertEqual(node._camera_watchdog_frame_count, 5)

    def test_runtime_uses_one_private_context_for_node_and_executor(self) -> None:
        context = MagicMock()
        node = MagicMock()
        executor = MagicMock()
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init") as init,
            patch("sensor_bridge.ros_node.SensorStreamNode", return_value=node) as cls,
            patch(
                "sensor_bridge.ros_node.SingleThreadedExecutor",
                return_value=executor,
            ) as executor_cls,
        ):
            runtime = SensorRosRuntime(
                state=MagicMock(),
                camera_topic="/camera/image_raw",
                lidar_topic="/scan",
                jpeg_quality=80,
                camera_fps=10.0,
            )
            runtime.stop()

        self.assertIs(init.call_args.kwargs["context"], context)
        self.assertIs(cls.call_args.kwargs["context"], context)
        self.assertIs(executor_cls.call_args.kwargs["context"], context)
        context.shutdown.assert_called_once_with()

    def test_runtime_rolls_back_private_context_when_node_init_fails(self) -> None:
        context = MagicMock()
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init"),
            patch(
                "sensor_bridge.ros_node.SensorStreamNode",
                side_effect=RuntimeError("node init failed"),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "node init failed"):
                SensorRosRuntime(
                    state=MagicMock(),
                    camera_topic="/camera/image_raw",
                    lidar_topic="/scan",
                    jpeg_quality=80,
                    camera_fps=10.0,
                )
        context.shutdown.assert_called_once_with()

    def test_runtime_constructor_cleanup_does_not_mask_add_node_error(
        self,
    ) -> None:
        context = MagicMock()
        node = MagicMock()
        executor = MagicMock()
        executor.add_node.side_effect = ValueError("add node failed")
        executor.shutdown.side_effect = RuntimeError("shutdown failed")
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init"),
            patch("sensor_bridge.ros_node.SensorStreamNode", return_value=node),
            patch(
                "sensor_bridge.ros_node.SingleThreadedExecutor",
                return_value=executor,
            ),
        ):
            with self.assertRaisesRegex(ValueError, "add node failed"):
                SensorRosRuntime(
                    state=MagicMock(),
                    camera_topic="/camera/image_raw",
                    lidar_topic="/scan",
                    jpeg_quality=80,
                    camera_fps=10.0,
                )

        executor.shutdown.assert_called_once_with(timeout_sec=0.0)
        node.destroy_node.assert_called_once_with()
        context.shutdown.assert_called_once_with()

    def test_runtime_stop_cleans_other_resources_and_retries_failure(
        self,
    ) -> None:
        context = MagicMock()
        node = MagicMock()
        executor = MagicMock()
        executor.shutdown.side_effect = [
            RuntimeError("executor shutdown failed"),
            True,
        ]
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init"),
            patch("sensor_bridge.ros_node.SensorStreamNode", return_value=node),
            patch(
                "sensor_bridge.ros_node.SingleThreadedExecutor",
                return_value=executor,
            ),
        ):
            runtime = SensorRosRuntime(
                state=MagicMock(),
                camera_topic="/camera/image_raw",
                lidar_topic="/scan",
                jpeg_quality=80,
                camera_fps=10.0,
            )
            with self.assertRaisesRegex(
                RuntimeError,
                "executor shutdown failed",
            ):
                runtime.stop()

            self.assertFalse(runtime._stopped)
            node.destroy_node.assert_called_once_with()
            context.shutdown.assert_called_once_with()
            runtime.stop()

        self.assertTrue(runtime._stopped)
        self.assertEqual(executor.shutdown.call_count, 2)
        node.destroy_node.assert_called_once_with()
        context.shutdown.assert_called_once_with()

    def test_concurrent_runtime_stop_only_tears_resources_down_once(
        self,
    ) -> None:
        context = MagicMock()
        node = MagicMock()
        executor = MagicMock()
        shutdown_entered = threading.Event()
        release_shutdown = threading.Event()
        stop_errors: list[BaseException] = []

        def shutdown(*, timeout_sec: float) -> bool:
            self.assertEqual(timeout_sec, 3.0)
            shutdown_entered.set()
            release_shutdown.wait(timeout=1.0)
            return True

        executor.shutdown.side_effect = shutdown
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init"),
            patch("sensor_bridge.ros_node.SensorStreamNode", return_value=node),
            patch(
                "sensor_bridge.ros_node.SingleThreadedExecutor",
                return_value=executor,
            ),
        ):
            runtime = SensorRosRuntime(
                state=MagicMock(),
                camera_topic="/camera/image_raw",
                lidar_topic="/scan",
                jpeg_quality=80,
                camera_fps=10.0,
            )

            def stop_runtime() -> None:
                try:
                    runtime.stop()
                except BaseException as error:  # noqa: BLE001
                    stop_errors.append(error)

            first = threading.Thread(target=stop_runtime)
            second = threading.Thread(target=stop_runtime)
            first.start()
            self.assertTrue(shutdown_entered.wait(timeout=1.0))
            second.start()
            release_shutdown.set()
            first.join(timeout=1.0)
            second.join(timeout=1.0)

        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(stop_errors, [])
        executor.shutdown.assert_called_once_with(timeout_sec=3.0)
        node.destroy_node.assert_called_once_with()
        context.shutdown.assert_called_once_with()

    def test_runtime_stops_spin_thread_before_executor_shutdown(self) -> None:
        context = MagicMock()
        context.ok.return_value = True
        node = MagicMock()
        executor = MagicMock()
        spin_entered = threading.Event()
        release_spin = threading.Event()

        def spin_once(*, timeout_sec: float) -> None:
            self.assertEqual(timeout_sec, 0.1)
            spin_entered.set()
            release_spin.wait(timeout=1.0)
            raise RuntimeError("executor wake during intentional stop")

        executor.spin_once.side_effect = spin_once
        executor.wake.side_effect = release_spin.set
        fatal_callback = MagicMock()
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init"),
            patch("sensor_bridge.ros_node.SensorStreamNode", return_value=node),
            patch(
                "sensor_bridge.ros_node.SingleThreadedExecutor",
                return_value=executor,
            ),
        ):
            runtime = SensorRosRuntime(
                state=MagicMock(),
                camera_topic="/camera/image_raw",
                lidar_topic="/scan",
                jpeg_quality=80,
                camera_fps=10.0,
                fatal_callback=fatal_callback,
            ).start()
            self.assertTrue(spin_entered.wait(timeout=1.0))
            runtime.stop()

        self.assertFalse(runtime._thread.is_alive())
        executor.shutdown.assert_called_once_with(timeout_sec=3.0)
        node.destroy_node.assert_called_once_with()
        fatal_callback.assert_not_called()
        self.assertTrue(runtime.health()["healthy"])

    def test_runtime_records_executor_fatal_and_notifies_once(self) -> None:
        context = MagicMock()
        context.ok.return_value = True
        node = MagicMock()
        executor = MagicMock()
        executor.spin_once.side_effect = RuntimeError("DDS reader failed")
        fatal_callback = MagicMock()
        with (
            patch("sensor_bridge.ros_node.Context", return_value=context),
            patch("sensor_bridge.ros_node.rclpy.init"),
            patch("sensor_bridge.ros_node.SensorStreamNode", return_value=node),
            patch(
                "sensor_bridge.ros_node.SingleThreadedExecutor",
                return_value=executor,
            ),
        ):
            runtime = SensorRosRuntime(
                state=MagicMock(),
                camera_topic="/camera/image_raw",
                lidar_topic="/scan",
                jpeg_quality=80,
                camera_fps=10.0,
                fatal_callback=fatal_callback,
            ).start()
            runtime._thread.join(timeout=1.0)
            self.assertFalse(runtime._thread.is_alive())

            failed = runtime.health()
            self.assertEqual(failed["status"], "error")
            self.assertFalse(failed["healthy"])
            self.assertEqual(failed["error"]["type"], "RuntimeError")
            self.assertIn("DDS reader failed", failed["error"]["message"])
            fatal_callback.assert_called_once()
            self.assertEqual(
                fatal_callback.call_args.args[0],
                "sensor_ros_executor",
            )
            runtime.stop()

        executor.shutdown.assert_called_once_with(timeout_sec=3.0)
        node.destroy_node.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
