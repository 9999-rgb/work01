"""Sensor bridge topic and QoS contract tests."""

from __future__ import annotations

import unittest
import threading
from unittest.mock import MagicMock, patch

from rclpy.qos import ReliabilityPolicy

from sensor_bridge.ros_node import SensorStreamNode
from sensor_bridge.ros_node import SensorRosRuntime
from sensor_bridge.ros_node import _alternate_camera_topic


class SensorBridgeContractTest(unittest.TestCase):
    def test_camera_reader_matches_best_effort_sensor_publishers(self) -> None:
        self.assertEqual(
            ReliabilityPolicy.BEST_EFFORT,
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
