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
        "reset_base_pose": {
            "frame_id": "world_map",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
        },
        "reset_joint_tolerance": 0.12,
        "reset_joint_timeout_sec": 15.0,
        "reset_joint_duration_sec": 5.0,
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
                    "inoperable_control_reason": "not physically validated",
                    "docking_base_footprint": [
                        0.2,
                        0.2,
                        0.2,
                        -0.2,
                        -0.2,
                        -0.2,
                        -0.2,
                        0.2,
                    ],
                    "docking_base_footprint_padding": 0.03,
                    "docking_position_tolerance": 0.01,
                    "docking_yaw_tolerance": 0.1,
                    "controls": {
                        "start_button": {
                            "navigation_station": {
                                "frame_id": "world_map",
                                "local_anchor": [0.0, 0.0, 0.0],
                                "outward_axis": [1.0, 0.0, 0.0],
                                "standoff": 0.5,
                            }
                        }
                    },
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
                    "require_pose_valid": True,
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
        with self.assertRaisesRegex(ProfileContractError, "pose_parent_frame"):
            self._validate(documents)

        documents = _documents()
        scene = documents["scene"]
        assert isinstance(scene, dict)
        scene["/**/xczs_cabinet_planning_scene"]["ros__parameters"][
            "navigation_station"
        ]["frame_id"] = "wrong_map"
        with self.assertRaisesRegex(ProfileContractError, "expected"):
            self._validate(documents)

        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "controls"
        ] = {
            "start_button": {
                "navigation_station": {
                    "frame_id": "wrong_map",
                    "local_anchor": [0.0, 0.0, 0.0],
                    "outward_axis": [1.0, 0.0, 0.0],
                    "standoff": 0.5,
                }
            }
        }
        with self.assertRaisesRegex(
            ProfileContractError,
            "device_a/start_button.*expected 'world_map'",
        ):
            self._validate(documents)

    def test_rejects_pose_validity_contract_mismatches(self) -> None:
        documents = _documents()
        pose = documents["pose"]
        assert isinstance(pose, dict)
        pose["/**/xczs_cabinet_pose_authority"]["ros__parameters"][
            "validity_topic"
        ] = "authority_pose_valid"
        with self.assertRaisesRegex(
            ProfileContractError,
            "pose-validity topics must match",
        ):
            self._validate(documents)

        documents = _documents()
        scene = documents["scene"]
        assert isinstance(scene, dict)
        scene["/**/xczs_cabinet_planning_scene"]["ros__parameters"][
            "pose_valid_topic"
        ] = "/absolute/pose_valid"
        with self.assertRaisesRegex(ProfileContractError, "relative ROS topic"):
            self._validate(documents)

    def test_rejects_disabling_pose_validity_guards(self) -> None:
        documents = _documents()
        scene = documents["scene"]
        assert isinstance(scene, dict)
        scene["/**/xczs_cabinet_planning_scene"]["ros__parameters"][
            "require_pose_valid"
        ] = False
        with self.assertRaisesRegex(ProfileContractError, "must be true"):
            self._validate(documents)

        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "require_cabinet_pose_valid"
        ] = False
        with self.assertRaisesRegex(ProfileContractError, "must be true"):
            self._validate(documents)

    def test_rejects_unknown_or_duplicate_availability_policies(self) -> None:
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
        with self.assertRaisesRegex(ProfileContractError, "one authoritative"):
            self._validate(documents)

    def test_allows_station_override_with_safe_default_policy(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        operator = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]
        operator["controls"] = {
            "start_button": {
                "navigation_station": {
                    "frame_id": "world_map",
                    "local_anchor": [0.0, 0.0, 0.0],
                    "outward_axis": [1.0, 0.0, 0.0],
                    "standoff": 0.5,
                }
            }
        }
        report = self._validate(documents)

        self.assertEqual(1, report.control_count)
        self.assertEqual((), report.operable_control_ids)

    def test_allows_explicit_positive_capability(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "operable_control_ids"
        ] = ["start_button"]

        report = self._validate(documents)

        self.assertEqual(1, report.control_count)
        self.assertEqual(("start_button",), report.operable_control_ids)

        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "inoperable_control_reason"
        ] = ""
        with self.assertRaisesRegex(
            ProfileContractError, "inoperable_control_reason"
        ):
            self._validate(documents)

    def test_rejects_operable_station_inside_full_footprint_envelope(
        self,
    ) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        operator = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]
        operator["operable_control_ids"] = ["start_button"]
        operator["controls"]["start_button"]["navigation_station"][
            "standoff"
        ] = 0.22

        with self.assertRaisesRegex(ProfileContractError, "safety envelope"):
            self._validate(documents)

        operator["controls"]["start_button"]["navigation_station"][
            "standoff"
        ] = 0.30
        report = self._validate(documents)
        self.assertEqual(("start_button",), report.operable_control_ids)

    def test_validates_button_tool_roll_offset(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        override = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]["controls"]["start_button"]
        override["tool_roll_offset"] = -0.75

        report = self._validate(documents)
        self.assertEqual(1, report.button_count)

        override["tool_roll_offset"] = 4.0
        with self.assertRaisesRegex(ProfileContractError, r"\[-pi, pi\]"):
            self._validate(documents)

        controls = documents["controls"]
        assert isinstance(controls, dict)
        controls["/**"]["ros__parameters"]["controls"]["start_button"][
            "type"
        ] = "knob"
        controls["/**"]["ros__parameters"]["knob_defaults"] = {
            "state_ids": ["left", "center", "right"]
        }
        scene = documents["scene"]
        assert isinstance(scene, dict)
        collision = scene["/**/xczs_cabinet_planning_scene"][
            "ros__parameters"
        ]["control_collision"]
        collision["knob_size"] = [0.02, 0.01]
        collision["knob_center_offset"] = 0.005
        override["tool_roll_offset"] = 0.0
        with self.assertRaisesRegex(ProfileContractError, "only valid for button"):
            self._validate(documents)

    def test_validates_per_transition_knob_tool_calibration(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        controls_document = documents["controls"]
        scene_document = documents["scene"]
        assert isinstance(adapter, dict)
        assert isinstance(controls_document, dict)
        assert isinstance(scene_document, dict)
        controls_parameters = controls_document["/**"]["ros__parameters"]
        controls_parameters["controls"]["start_button"]["type"] = "knob"
        controls_parameters["knob_defaults"] = {
            "state_ids": ["left", "center", "right"]
        }
        scene = scene_document["/**/xczs_cabinet_planning_scene"][
            "ros__parameters"
        ]
        scene["control_collision"]["knob_size"] = [0.02, 0.01]
        scene["control_collision"]["knob_center_offset"] = 0.005
        override = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]["controls"]["start_button"]
        override["tool_roll_offsets"] = [0.0] * 9
        override["detent_release_fraction"] = 0.60

        report = self._validate(documents)

        self.assertEqual(1, report.knob_count)

        override["tool_roll_offsets"] = [0.0] * 8
        with self.assertRaisesRegex(ProfileContractError, "exactly 9"):
            self._validate(documents)

    def test_rejects_rotary_calibration_on_non_knob_or_invalid_fraction(
        self,
    ) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        override = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]["controls"]["start_button"]
        override["detent_release_fraction"] = 0.60
        with self.assertRaisesRegex(ProfileContractError, "only valid for knob"):
            self._validate(documents)

        documents = _documents()
        adapter = documents["adapter"]
        controls_document = documents["controls"]
        scene_document = documents["scene"]
        assert isinstance(adapter, dict)
        assert isinstance(controls_document, dict)
        assert isinstance(scene_document, dict)
        controls_parameters = controls_document["/**"]["ros__parameters"]
        controls_parameters["controls"]["start_button"]["type"] = "knob"
        controls_parameters["knob_defaults"] = {
            "state_ids": ["left", "center", "right"]
        }
        scene = scene_document["/**/xczs_cabinet_planning_scene"][
            "ros__parameters"
        ]
        scene["control_collision"]["knob_size"] = [0.02, 0.01]
        scene["control_collision"]["knob_center_offset"] = 0.005
        override = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]["controls"]["start_button"]
        override["detent_release_fraction"] = 0.5
        with self.assertRaisesRegex(ProfileContractError, r"\(0.5, 1.0\]"):
            self._validate(documents)

    def test_validates_named_knob_ready_joint_seed(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        controls_document = documents["controls"]
        scene_document = documents["scene"]
        assert isinstance(adapter, dict)
        assert isinstance(controls_document, dict)
        assert isinstance(scene_document, dict)
        robot_parameters = adapter["/**"]["ros__parameters"]
        robot_parameters["manual_joint_group_names"] = ["arm"]
        robot_parameters["manual_joint_groups"] = {
            "arm": ["arm_joint"]
        }
        robot_parameters["controller_namespace"] = "/robot"
        operator = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]
        operator["tool_profiles"] = {"knob": {"move_group": "arm"}}
        override = operator["controls"]["start_button"]
        override["ready_joint_seed"] = {
            "joint_names": ["arm_joint"],
            "positions": [0.25],
        }
        controls_parameters = controls_document["/**"]["ros__parameters"]
        controls_parameters["controls"]["start_button"]["type"] = "knob"
        controls_parameters["knob_defaults"] = {
            "state_ids": ["left", "center", "right"]
        }
        scene = scene_document["/**/xczs_cabinet_planning_scene"][
            "ros__parameters"
        ]
        scene["control_collision"]["knob_size"] = [0.02, 0.01]
        scene["control_collision"]["knob_center_offset"] = 0.005

        report = self._validate(documents)
        self.assertEqual(1, report.knob_count)

        override["ready_joint_seed"]["joint_names"] = ["wrong_joint"]
        with self.assertRaisesRegex(ProfileContractError, "every joint"):
            self._validate(documents)

        override["ready_joint_seed"] = {
            "joint_names": ["arm_joint"],
            "positions": [],
        }
        with self.assertRaisesRegex(ProfileContractError, "exactly 1"):
            self._validate(documents)

    def test_rejects_unknown_or_explicit_empty_operable_sequence(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "operable_control_ids"
        ] = []

        with self.assertRaisesRegex(ProfileContractError, "must be omitted"):
            self._validate(documents)

        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "operable_control_ids"
        ] = ["missing_button"]
        with self.assertRaisesRegex(ProfileContractError, "unknown IDs"):
            self._validate(documents)

    def test_rejects_legacy_fail_open_denylist(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "unreachable_control_ids"
        ] = ["start_button"]

        with self.assertRaisesRegex(ProfileContractError, "fail-open"):
            self._validate(documents)

    def test_rejects_control_without_explicit_navigation_station(self) -> None:
        documents = _documents()
        adapter = documents["adapter"]
        assert isinstance(adapter, dict)
        adapter["/**/xczs_cabinet_button_operator"]["ros__parameters"][
            "controls"
        ] = {}

        with self.assertRaisesRegex(
            ProfileContractError,
            "Every submitted control.*start_button",
        ):
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
