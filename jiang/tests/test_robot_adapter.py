"""Tests for the typed Web robot-interface adapter."""

from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from dataclasses import FrozenInstanceError
from math import pi
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import yaml


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.robot_adapter import RobotAdapterConfig  # noqa: E402
from control_gateway.robot_adapter import RobotAdapterError  # noqa: E402
from control_gateway.robot_adapter import load  # noqa: E402
from control_gateway.runner import ControlServer  # noqa: E402


def _parameters() -> dict[str, object]:
    return {
        "planning_frame": "robot_odom",
        "manual_linear_axis": "x",
        "navigation_velocity_yaw_offset": 0.25,
        "navigation_frame": "world_map",
        "navigation_base_frame": "mobile/base",
        "navigation_action": "/robot/navigate_to_pose",
        "navigation_readiness_service": "/robot/nav_manager/is_active",
        "navigation_mode_service": "/robot/set_navigation_mode",
        "navigation_mode_topic": "/robot/navigation_mode",
        "map_topic": "/robot/map",
        "localization_pose_topic": "/robot/localization_pose",
        "manual_cmd_vel_topic": "/robot/manual_cmd_vel",
        "navigation_cmd_vel_topic": "/robot/navigation_cmd_vel",
        "base_output_topic": "/robot/base_output",
        "joint_trajectory_topic": "/robot/joint_trajectory",
        "joint_state_topic": "/robot/joint_states",
        "reset_base_pose": {
            "frame_id": "world_map",
            "x": 1.25,
            "y": -0.5,
            "yaw": 0.75,
        },
        "reset_joint_tolerance": 0.03,
        "reset_joint_timeout_sec": 12.0,
        "reset_joint_duration_sec": 4.0,
        "arm_controller_topic": "/robot/arm/joint_trajectory",
        "arm_controller_status_topic": "/robot/arm/status",
        "gripper_controller_topic": "/robot/gripper/joint_trajectory",
        "gripper_controller_status_topic": "/robot/gripper/status",
        "arm_joint_names": ["joint_a"],
        "gripper_joint_names": ["joint_b"],
        "manual_joint_limits": {
            "joint_a": {
                "min_position": -1.0,
                "max_position": 1.0,
                "default_position": 0.2,
            },
            "joint_b": {
                "min_position": 0.0,
                "max_position": 0.5,
                "default_position": 0.0,
                "open_position": 0.5,
            },
        },
    }


class RobotAdapterTest(unittest.TestCase):
    def _load_document(self, document: object) -> RobotAdapterConfig:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_adapter.yaml"
            path.write_text(
                yaml.safe_dump(document, allow_unicode=True),
                encoding="utf-8",
            )
            return load(path)

    def test_loads_authoritative_global_interface(self) -> None:
        adapter = self._load_document(
            {
                "/**": {"ros__parameters": _parameters()},
                "/**/xczs_cabinet_button_operator": {
                    "ros__parameters": {
                        "navigation_base_frame": "ignored_base",
                        "controls": {
                            "button_1": {
                                "navigation_station": {
                                    "local_anchor": [0.25, 1.0, 0.0],
                                    "outward_axis": [0.0, 0.0, 2.0],
                                    "standoff": 0.8,
                                    "base_yaw_offset": 0.1,
                                    "frame_id": "world_map",
                                }
                            }
                        },
                    }
                },
            }
        )

        self.assertEqual("world_map", adapter.navigation_frame)
        self.assertEqual("mobile/base", adapter.navigation_base_frame)
        self.assertEqual("/robot/navigate_to_pose", adapter.navigation_action)
        self.assertEqual(
            "/robot/nav_manager/is_active",
            adapter.navigation_readiness_service,
        )
        self.assertEqual(("joint_a", "joint_b"), adapter.manual_joint_names)
        self.assertEqual(-1.0, adapter.manual_joints[0].min_position)
        self.assertEqual(0.2, adapter.manual_joints[0].default_position)
        self.assertEqual("arm", adapter.manual_joints[0].group)
        self.assertEqual("gripper", adapter.manual_joints[1].group)
        self.assertEqual(0.5, adapter.manual_joints[1].open_position)
        self.assertEqual("world_map", adapter.reset_base_pose.frame_id)
        self.assertEqual(1.25, adapter.reset_base_pose.x)
        self.assertEqual(-0.5, adapter.reset_base_pose.y)
        self.assertEqual(0.75, adapter.reset_base_pose.yaw)
        self.assertEqual(0.03, adapter.reset_joint_tolerance)
        self.assertEqual(12.0, adapter.reset_joint_timeout_sec)
        self.assertEqual(4.0, adapter.reset_joint_duration_sec)
        station = adapter.control_navigation_station("button_1")
        self.assertIsNotNone(station)
        assert station is not None
        self.assertEqual((0.25, 1.0, 0.0), station.local_anchor)
        self.assertEqual((0.0, 0.0, 1.0), station.outward_axis)
        self.assertEqual("world_map", station.frame_id)
        self.assertIsNone(adapter.control_navigation_station("missing"))
        with self.assertRaisesRegex(FrozenInstanceError, "cannot assign"):
            adapter.navigation_frame = "changed"  # type: ignore[misc]
        with self.assertRaisesRegex(FrozenInstanceError, "cannot assign"):
            adapter.reset_base_pose.x = 2.0  # type: ignore[misc]

    def test_rejects_invalid_reset_contract(self) -> None:
        parameters = _parameters()
        parameters["reset_base_pose"] = {
            "frame_id": "world_map",
            "x": 0.0,
            "y": 0.0,
        }
        with self.assertRaisesRegex(RobotAdapterError, "missing.*yaw"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["reset_base_pose"] = {
            "frame_id": "world_map",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "z": 0.0,
        }
        with self.assertRaisesRegex(RobotAdapterError, "unknown fields: z"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["reset_base_pose"]["frame_id"] = "other_map"
        with self.assertRaisesRegex(RobotAdapterError, "match navigation_frame"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["reset_base_pose"]["yaw"] = float("nan")
        with self.assertRaisesRegex(RobotAdapterError, "reset_base_pose.yaw"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        for field, value in (
            ("reset_joint_tolerance", 0.0),
            ("reset_joint_timeout_sec", -1.0),
            ("reset_joint_duration_sec", float("inf")),
        ):
            with self.subTest(field=field):
                parameters = _parameters()
                parameters[field] = value
                with self.assertRaisesRegex(RobotAdapterError, field):
                    self._load_document(
                        {"/**": {"ros__parameters": parameters}}
                    )

        parameters = _parameters()
        parameters["reset_joint_duration_sec"] = 13.0
        with self.assertRaisesRegex(
            RobotAdapterError,
            "must not exceed reset_joint_timeout_sec",
        ):
            self._load_document({"/**": {"ros__parameters": parameters}})

    def test_rejects_invalid_control_navigation_station(self) -> None:
        with self.assertRaisesRegex(
            RobotAdapterError,
            "controls.button_1.navigation_station.standoff",
        ):
            self._load_document(
                {
                    "/**": {"ros__parameters": _parameters()},
                    "/**/xczs_cabinet_button_operator": {
                        "ros__parameters": {
                            "controls": {
                                "button_1": {
                                    "navigation_station": {
                                        "local_anchor": [0.0, 0.0, 0.0],
                                        "outward_axis": [1.0, 0.0, 0.0],
                                        "standoff": 0.0,
                                    }
                                }
                            }
                        }
                    },
                }
            )

        with self.assertRaisesRegex(RobotAdapterError, "unknown fields: frame"):
            self._load_document(
                {
                    "/**": {"ros__parameters": _parameters()},
                    "/**/xczs_cabinet_button_operator": {
                        "ros__parameters": {
                            "controls": {
                                "button_1": {
                                    "navigation_station": {
                                        "local_anchor": [0.0, 0.0, 0.0],
                                        "outward_axis": [1.0, 0.0, 0.0],
                                        "standoff": 0.8,
                                        "frame": "world_map",
                                    }
                                }
                            }
                        }
                    },
                }
            )

    def test_rejects_control_ids_duplicated_after_trimming(self) -> None:
        with self.assertRaisesRegex(
            RobotAdapterError,
            "duplicate control ID after trimming whitespace: button_1",
        ):
            self._load_document(
                {
                    "/**": {"ros__parameters": _parameters()},
                    "/**/xczs_cabinet_button_operator": {
                        "ros__parameters": {
                            "controls": {
                                "button_1": {},
                                " button_1 ": {},
                            }
                        }
                    },
                }
            )

    def test_project_adapter_matches_typed_contract(self) -> None:
        path = (
            JIANG_DIR.parent
            / "xczs_inspection_robot_control"
            / "config"
            / "cabinet_robot_adapter.yaml"
        )

        adapter = load(path)

        self.assertEqual("odom", adapter.planning_frame)
        self.assertEqual("map", adapter.navigation_frame)
        self.assertEqual("y", adapter.manual_linear_axis)
        self.assertEqual(
            (
                "left_arm",
                "right_arm",
                "three_cylinder",
                "two_cylinder",
                "rocker",
                "rotate_button",
            ),
            adapter.joint_group_names,
        )
        self.assertEqual(6, len(adapter.controller_groups))
        self.assertEqual(22, len(adapter.manual_joints))
        self.assertEqual(
            "/xczs/left_arm_controller/joint_trajectory",
            adapter.controller_topic_for_group("left_arm"),
        )
        self.assertEqual(
            "/xczs/rotate_button_controller/follow_joint_trajectory/"
            "_action/status",
            adapter.status_topic_for_group("rotate_button"),
        )
        self.assertEqual(7, len(adapter.joint_names_for_group("left_arm")))
        self.assertEqual(7, len(adapter.joint_names_for_group("right_arm")))
        self.assertEqual(3, len(adapter.joint_names_for_group("three_cylinder")))
        self.assertEqual(2, len(adapter.joint_names_for_group("two_cylinder")))
        self.assertEqual(1, len(adapter.joint_names_for_group("rocker")))
        self.assertEqual(2, len(adapter.joint_names_for_group("rotate_button")))
        self.assertEqual("map", adapter.reset_base_pose.frame_id)
        self.assertEqual(0.0, adapter.reset_base_pose.x)
        self.assertEqual(0.0, adapter.reset_base_pose.y)
        self.assertAlmostEqual(pi / 2.0, adapter.reset_base_pose.yaw)
        self.assertEqual(0.02, adapter.reset_joint_tolerance)
        self.assertEqual(15.0, adapter.reset_joint_timeout_sec)
        self.assertEqual(5.0, adapter.reset_joint_duration_sec)
        defaults = {
            joint.name: joint.default_position for joint in adapter.manual_joints
        }
        self.assertEqual(0.0, defaults["r_three_cyl_finger3_joint"])
        self.assertEqual(0.0, defaults["l_rocker_rotor_joint"])
        stations = dict(adapter.control_navigation_stations)
        self.assertEqual(33, len(stations))
        self.assertEqual(
            stations["cabinet_rear_door"],
            stations["cabinet_main_switch"],
        )

    def test_navigation_frame_defaults_to_map(self) -> None:
        parameters = _parameters()
        parameters.pop("navigation_frame")
        parameters["reset_base_pose"]["frame_id"] = "map"

        adapter = self._load_document(
            {"/**": {"ros__parameters": parameters}}
        )

        self.assertEqual("map", adapter.navigation_frame)

    def test_rejects_missing_required_interface(self) -> None:
        parameters = _parameters()
        parameters.pop("navigation_action")

        with self.assertRaisesRegex(RobotAdapterError, "navigation_action"):
            self._load_document({"/**": {"ros__parameters": parameters}})

    def test_rejects_duplicate_and_invalid_joint_names(self) -> None:
        parameters = _parameters()
        parameters["arm_joint_names"] = ["joint_a", "joint_a"]
        with self.assertRaisesRegex(RobotAdapterError, "unique"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["arm_joint_names"] = ["joint_a", "/joint_b"]
        with self.assertRaisesRegex(RobotAdapterError, "relative"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["gripper_joint_names"] = ["joint_a"]
        with self.assertRaisesRegex(RobotAdapterError, "must not overlap"):
            self._load_document({"/**": {"ros__parameters": parameters}})

    def test_rejects_incomplete_or_invalid_joint_limits(self) -> None:
        parameters = _parameters()
        limits = parameters["manual_joint_limits"]
        assert isinstance(limits, dict)
        limits.pop("joint_b")
        with self.assertRaisesRegex(RobotAdapterError, "missing joint_b"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        limits = parameters["manual_joint_limits"]
        assert isinstance(limits, dict)
        limits["joint_a"] = {
            "min_position": 2.0,
            "max_position": 1.0,
        }
        with self.assertRaisesRegex(RobotAdapterError, "exceeds maximum"):
            self._load_document({"/**": {"ros__parameters": parameters}})

    def test_gripper_interfaces_are_required_only_when_gripper_exists(
        self,
    ) -> None:
        parameters = _parameters()
        parameters["gripper_joint_names"] = []
        parameters["manual_joint_limits"].pop("joint_b")
        parameters.pop("gripper_controller_topic")
        parameters.pop("gripper_controller_status_topic")

        adapter = self._load_document(
            {"/**": {"ros__parameters": parameters}}
        )

        self.assertIsNone(adapter.controller_topic_for_group("gripper"))
        self.assertIsNone(adapter.status_topic_for_group("gripper"))

        parameters = _parameters()
        parameters.pop("gripper_controller_topic")
        with self.assertRaisesRegex(
            RobotAdapterError, "gripper_controller_topic"
        ):
            self._load_document({"/**": {"ros__parameters": parameters}})

    def test_rejects_relative_ros_interface_name(self) -> None:
        parameters = _parameters()
        parameters["navigation_mode_service"] = "set_navigation_mode"

        with self.assertRaisesRegex(RobotAdapterError, "absolute ROS name"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["navigation_base_frame"] = "/absolute_base"
        with self.assertRaisesRegex(RobotAdapterError, "relative name"):
            self._load_document({"/**": {"ros__parameters": parameters}})

        parameters = _parameters()
        parameters["manual_linear_axis"] = "z"
        with self.assertRaisesRegex(RobotAdapterError, "either x or y"):
            self._load_document({"/**": {"ros__parameters": parameters}})

    def test_rejects_ambiguous_legacy_adapter_blocks(self) -> None:
        document = {
            "/operator_a": {
                "ros__parameters": {"navigation_base_frame": "base_a"}
            },
            "/operator_b": {
                "ros__parameters": {"navigation_base_frame": "base_b"}
            },
        }

        with self.assertRaisesRegex(RobotAdapterError, "exactly one legacy"):
            self._load_document(document)

    def test_legacy_single_node_uses_compatible_defaults(self) -> None:
        adapter = self._load_document(
            {
                "/**/operator": {
                    "ros__parameters": {
                        "navigation_base_frame": "base_link"
                    }
                }
            }
        )

        self.assertEqual("map", adapter.navigation_frame)
        self.assertEqual("/navigate_to_pose", adapter.navigation_action)
        self.assertEqual(
            "/lifecycle_manager_navigation/is_active",
            adapter.navigation_readiness_service,
        )
        self.assertEqual("/xczs/manual_cmd_vel", adapter.manual_cmd_vel_topic)
        self.assertEqual(8, len(adapter.manual_joint_names))
        self.assertEqual("y", adapter.manual_linear_axis)
        self.assertEqual("map", adapter.reset_base_pose.frame_id)
        self.assertAlmostEqual(pi / 2.0, adapter.reset_base_pose.yaw)
        self.assertEqual(0.02, adapter.reset_joint_tolerance)
        self.assertEqual(15.0, adapter.reset_joint_timeout_sec)
        self.assertEqual(5.0, adapter.reset_joint_duration_sec)

    def test_runner_passes_adapter_interfaces_and_explicit_overrides(self) -> None:
        parameters = _parameters()
        parameters["manual_cmd_vel_topic"] = "/adapter/cmd_vel"
        parameters["joint_trajectory_topic"] = "/adapter/joints"
        adapter = self._load_document(
            {"/**": {"ros__parameters": parameters}}
        )

        class StopConstruction(RuntimeError):
            pass

        captured: list[tuple[object, ...]] = []
        captured_keywords: list[dict[str, object]] = []

        def capture_node(*args: object, **kwargs: object) -> object:
            captured.append(args)
            captured_keywords.append(kwargs)
            raise StopConstruction

        with (
            patch(
                "control_gateway.runner.validate_profile"
            ) as validate_profile_mock,
            patch(
                "control_gateway.runner.CabinetInventory.load",
                return_value=[],
            ),
            patch(
                "control_gateway.runner.load_robot_adapter",
                return_value=adapter,
            ),
            patch(
                "control_gateway.runner.Context",
                return_value=SimpleNamespace(),
            ),
            patch("control_gateway.runner.rclpy.init"),
            patch(
                "control_gateway.runner.RosControlNode",
                side_effect=capture_node,
            ),
        ):
            with self.assertRaises(StopConstruction):
                ControlServer()
            with self.assertRaises(StopConstruction):
                ControlServer(
                    cmd_vel_topic="/legacy/cmd_vel",
                    joint_trajectory_topic="/legacy/joints",
                )

        self.assertEqual("/adapter/cmd_vel", captured[0][0])
        self.assertEqual("/adapter/joints", captured[0][1])
        self.assertEqual("/legacy/cmd_vel", captured[1][0])
        self.assertEqual("/legacy/joints", captured[1][1])
        self.assertEqual(2, validate_profile_mock.call_count)
        self.assertEqual(
            "world_map",
            captured_keywords[0]["navigation_frame"],
        )
        self.assertEqual(
            "mobile/base",
            captured_keywords[0]["navigation_base_frame"],
        )
        self.assertEqual(
            "/robot/navigate_to_pose",
            captured_keywords[0]["navigation_action"],
        )
        self.assertEqual(
            "/robot/nav_manager/is_active",
            captured_keywords[0]["navigation_readiness_service"],
        )
        self.assertEqual(
            "/robot/set_navigation_mode",
            captured_keywords[0]["navigation_mode_service"],
        )
        self.assertEqual(
            "/robot/navigation_mode",
            captured_keywords[0]["navigation_mode_topic"],
        )
        self.assertEqual(
            ("joint_a", "joint_b"),
            tuple(
                joint.name
                for joint in captured_keywords[0]["manual_joints"]
            ),
        )
        self.assertEqual("/robot/map", captured_keywords[0]["map_topic"])
        self.assertEqual(
            "/robot/localization_pose",
            captured_keywords[0]["localization_pose_topic"],
        )
        self.assertEqual(
            "/robot/joint_states",
            captured_keywords[0]["joint_state_topic"],
        )
        self.assertEqual("x", captured_keywords[0]["manual_linear_axis"])

    def test_runner_serializes_capabilities_without_losing_joint_order(
        self,
    ) -> None:
        adapter = self._load_document(
            {"/**": {"ros__parameters": _parameters()}}
        )
        server = object.__new__(ControlServer)
        server._robot_adapter = adapter
        server._manual_cmd_vel_topic = "/override/manual_cmd_vel"
        server._joint_trajectory_topic = "/override/joint_trajectory"
        server._request_condition = threading.Condition()
        server._active_requests = 0
        server._stopping = False

        capabilities = server.robot_capabilities()

        self.assertEqual("x", capabilities["manual_linear_axis"])
        self.assertEqual("robot_odom", capabilities["frames"]["planning"])
        self.assertEqual(2, capabilities["joint_count"])
        self.assertEqual(
            "/override/joint_trajectory",
            capabilities["topics"]["joint_trajectory"],
        )
        self.assertEqual(
            ["joint_a", "joint_b"],
            [joint["name"] for joint in capabilities["manual_joints"]],
        )
        self.assertIsNone(capabilities["manual_joints"][0]["open_position"])
        self.assertEqual(0.5, capabilities["manual_joints"][1]["open_position"])


if __name__ == "__main__":
    unittest.main()
