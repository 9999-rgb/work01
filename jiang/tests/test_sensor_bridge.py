"""Sensor bridge topic and QoS contract tests."""

from __future__ import annotations

import unittest

from rclpy.qos import ReliabilityPolicy

from sensor_bridge.ros_node import SensorStreamNode
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


if __name__ == "__main__":
    unittest.main()
