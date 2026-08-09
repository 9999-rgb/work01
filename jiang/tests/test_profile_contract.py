"""Tests for the portable robot and cabinet profile contract."""

from __future__ import annotations

import copy
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


JIANG_DIR = Path(__file__).resolve().parents[1]
WORKSPACE = JIANG_DIR.parent
sys.path.insert(0, str(JIANG_DIR))

from control_gateway.profile_contract import ProfileContractError  # noqa: E402
from control_gateway.profile_contract import validate_profile  # noqa: E402


def _robot_parameters() -> dict[str, object]:
    return {
        "planning_frame": "robot_odom",
        "manual_linear_axis": "x",
        "navigation_velocity_yaw_offset": 0.0,
        "allow_embedded_navigation": False,
        "navigation_frame": "world_map",
        "navigation_base_frame": "base_link",
        "navigation_action": "/navigate_to_pose",
        "navigation_mode_service": "/robot/set_navigation_mode",
        "navigation_mode_topic": "/robot/navigation_mode",
        "map_topic": "/map",
        "localization_pose_topic": "/localization_pose",
        "manual_cmd_vel_topic": "/robot/manual_cmd_vel",
        "navigation_cmd_vel_topic": "/cmd_vel",
        "base_output_topic": "/robot/cmd_vel",
        "joint_trajectory_topic": "/robot/joint_trajectory",
        "joint_state_topic": "/joint_states",
        "arm_controller_topic": "/arm/joint_trajectory",
        "arm_controller_status_topic": "/arm/status",
        "arm_joint_names": ["arm_joint"],
        "gripper_joint_names": [],
        "manual_joint_limits": {
            "arm_joint": {
                "min_position": -1.0,
                "max_position": 1.0,
                "default_position": 0.0,
            }
        },
    }


def _documents() -> dict[str, object]:
    return {
        "adapter": {
            "/**": {"ros__parameters": _robot_parameters()},
            "/**/xczs_cabinet_button_operator": {
                "ros__parameters": {
                    "controls": {},
                    "unreachable_control_ids": [],
                }
            },
        },
        "instances": {
            "instances": [
                {
                    "name": "device_a",
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                }
            ]
        },
        "controls": {
            "/**": {
                "ros__parameters": {
                    "control_ids": ["start_button"],
                    "controls": {
                        "start_button": {
                            "type": "button",
                            "display_name": "Start",
                            "joint_name": "start_button_joint",
                            "local_position": [0.0, 0.0, 0.0],
                        }
                    },
                }
            }
        },
        "scene": {
            "/**/xczs_cabinet_planning_scene": {
                "ros__parameters": {
                    "navigation_station": {
                        "frame_id": "world_map",
                        "local_anchor": [0.0, 0.0, 0.0],
                        "outward_axis": [1.0, 0.0, 0.0],
                        "standoff": 0.5,
                    },
                    "frame_part_ids": ["panel"],
                    "frame_parts": {
                        "panel": {
                            "size": [1.0, 1.0, 0.1],
                            "position": [0.0, 0.0, 0.0],
                        }
                    },
                    "control_collision": {
                        "button_size": [0.01, 0.01],
                        "button_center_offset": 0.005,
                    },
                }
            }
        },
        "pose": {
            "/**/xczs_cabinet_pose_authority": {
                "ros__parameters": {"parent_frame": "robot_odom"}
            }
        },
    }


class ProfileContractTest(unittest.TestCase):
    def _validate(self, documents: dict[str, object]):
        with tempfile.TemporaryDirectory() as directory_value:
            directory = Path(directory_value)
            paths = {}
            for label, document in documents.items():
                path = directory / f"{label}.yaml"
                path.write_text(
                    yaml.safe_dump(document, allow_unicode=True),
                    encoding="utf-8",
                )
                paths[label] = path
            return validate_profile(
                robot_adapter_path=paths["adapter"],
                instances_path=paths["instances"],
                controls_path=paths["controls"],
                scene_path=paths["scene"],
                pose_path=paths["pose"],
            )

    def test_accepts_pure_button_device_without_gripper(self) -> None:
        report = self._validate(_documents())

        self.assertEqual(1, report.cabinet_count)
        self.assertEqual(1, report.button_count)
        self.assertEqual(0, report.door_count)
        self.assertEqual(1, report.arm_joint_count)
        self.assertEqual(0, report.gripper_joint_count)

    def test_project_profile_passes_generic_contract(self) -> None:
        config = WORKSPACE / "xczs_inspection_robot_control" / "config"

        report = validate_profile(
            robot_adapter_path=config / "cabinet_robot_adapter.yaml",
            instances_path=config / "cabinet_instances.yaml",
            controls_path=config / "cabinet_controls.yaml",
            scene_path=config / "cabinet_scene.yaml",
            pose_path=config / "cabinet_pose.yaml",
        )

        self.assertEqual(3, report.cabinet_count)
        self.assertEqual(33, report.control_count)
        self.assertEqual("odom", report.planning_frame)
        self.assertEqual("map", report.navigation_frame)

    def test_rejects_planning_and_navigation_frame_mismatches(self) -> None:
        documents = _documents()
        pose = documents["pose"]
        assert isinstance(pose, dict)
        pose["/**/xczs_cabinet_pose_authority"]["ros__parameters"][
            "parent_frame"
        ] = "wrong_odom"
        with self.assertRaisesRegex(ProfileContractError, "planning_frame"):
            self._validate(documents)

        documents = _documents()
        scene = documents["scene"]
        assert isinstance(scene, dict)
        scene["/**/xczs_cabinet_planning_scene"]["ros__parameters"][
            "navigation_station"
        ]["frame_id"] = "wrong_map"
        with self.assertRaisesRegex(ProfileContractError, "expected"):
            self._validate(documents)

    def test_rejects_unknown_or_overlapping_robot_overrides(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        operator = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]
        operator["controls"] = {
            "missing_button": {
                "operable": False,
                "unavailable_reason": "not calibrated",
            }
        }
        with self.assertRaisesRegex(ProfileContractError, "unknown IDs"):
            self._validate(documents)

        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        operator = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]
        operator["controls"] = {
            "start_button": {
                "operable": False,
                "unavailable_reason": "not calibrated",
            }
        }
        operator["unreachable_control_ids"] = ["start_button"]
        operator["unreachable_control_reason"] = "outside workspace"
        with self.assertRaisesRegex(ProfileContractError, "both"):
            self._validate(documents)

    def test_rejects_switch_parent_or_scene_id_mismatch(self) -> None:
        documents = copy.deepcopy(_documents())
        controls = documents["controls"]["/**"]["ros__parameters"]
        controls["control_ids"].extend(["door", "mode_switch"])
        controls["controls"].update(
            {
                "door": {"type": "door"},
                "mode_switch": {
                    "type": "switch",
                    "parent_control_id": "start_button",
                },
            }
        )
        with self.assertRaisesRegex(ProfileContractError, "must reference"):
            self._validate(documents)

        documents = copy.deepcopy(_documents())
        controls = documents["controls"]["/**"]["ros__parameters"]
        controls["control_ids"].append("door")
        controls["controls"]["door"] = {"type": "door"}
        scene = documents["scene"][
            "/**/xczs_cabinet_planning_scene"
        ]["ros__parameters"]
        scene["door"] = {
            "control_id": "wrong_door",
            "hinge_position": [0.0, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
            "panel_size": [1.0, 1.0, 0.1],
            "panel_position": [0.0, 0.0, 0.0],
            "handle_part_ids": ["handle"],
            "handle_parts": {
                "handle": {
                    "size": [0.1, 0.1, 0.1],
                    "position": [0.0, 0.0, 0.0],
                }
            },
        }
        with self.assertRaisesRegex(ProfileContractError, "does not match"):
            self._validate(documents)


if __name__ == "__main__":
    unittest.main()
