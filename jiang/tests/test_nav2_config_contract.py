"""Regression tests for safety-critical Nav2 controller configuration."""

from __future__ import annotations

import ast
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
NAV2_PARAMS = (
    WORKSPACE
    / "xczs_inspection_robot_nav2"
    / "config"
    / "nav2_params.yaml"
)
NAV2_BT = (
    WORKSPACE
    / "xczs_inspection_robot_nav2"
    / "config"
    / "behavior_trees"
    / "stable_axis_navigation.xml"
)
ROBOT_ADAPTER = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "cabinet_robot_adapter.yaml"
)
RUNNER_SOURCE = WORKSPACE / "jiang" / "control_gateway" / "runner.py"
WEB_VALIDATOR = WORKSPACE / "scripts" / "validate" / "validate_cabinet_web"


def _numeric_constant(path: Path, name: str) -> float:
    module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for statement in module.body:
        if not isinstance(statement, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == name
            for target in statement.targets
        ):
            value = ast.literal_eval(statement.value)
            if not isinstance(value, (int, float)):
                break
            return float(value)
    raise AssertionError(f"Missing numeric constant {name} in {path}")


def _shell_numeric_assignment(path: Path, name: str) -> float:
    source = path.read_text(encoding="utf-8")
    match = re.search(
        rf"(?m)^{re.escape(name)}=([0-9]+(?:\.[0-9]+)?)$",
        source,
    )
    if match is None:
        raise AssertionError(f"Missing numeric assignment {name} in {path}")
    return float(match.group(1))


class Nav2ConfigContractTest(unittest.TestCase):
    def test_bt_navigator_allows_discovery_after_behavior_activation(
        self,
    ) -> None:
        document = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        navigator = document["bt_navigator"]["ros__parameters"]

        self.assertGreaterEqual(navigator["wait_for_service_timeout"], 5000)

    def test_navigation_replans_only_for_new_or_invalid_path(self) -> None:
        document = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        navigator = document["bt_navigator"]["ros__parameters"]
        self.assertEqual(
            "$(find-pkg-share xczs_inspection_robot_nav2)"
            "/config/behavior_trees/stable_axis_navigation.xml",
            navigator["default_nav_to_pose_bt_xml"],
        )

        tree = ET.parse(NAV2_BT)
        root = tree.getroot()
        rate_controller = root.find(".//RateController")
        self.assertIsNotNone(rate_controller)
        self.assertEqual("1.0", rate_controller.attrib["hz"])

        planning_fallback = root.find(
            ".//Fallback[@name='FallbackComputePathToPose']"
        )
        self.assertIsNotNone(planning_fallback)
        self.assertEqual(
            ["ReactiveSequence", "ComputePathToPose"],
            [child.tag for child in planning_fallback],
        )
        path_check = planning_fallback[0]
        self.assertEqual(
            ["Inverter", "IsPathValid"],
            [child.tag for child in path_check],
        )
        self.assertEqual("GlobalUpdatedGoal", path_check[0][0].tag)

        self.assertEqual(1, len(root.findall(".//ComputePathToPose")))
        self.assertEqual(1, len(root.findall(".//IsPathValid")))
        self.assertIsNotNone(root.find(".//FollowPath"))
        self.assertIsNotNone(root.find(".//ClearEntireCostmap"))
        self.assertIsNotNone(root.find(".//Spin"))
        self.assertIsNotNone(root.find(".//Wait"))
        self.assertIsNotNone(root.find(".//BackUp"))

    def test_omnidirectional_controller_tracks_path_without_heading_alignment(
        self,
    ) -> None:
        document = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"]
        follow_path = controller["FollowPath"]
        critics = follow_path["critics"]

        self.assertNotIn("PathAlign", critics)
        self.assertNotIn("GoalAlign", critics)
        for critic in (
            "BaseObstacle",
            "Oscillation",
            "Twirling",
            "PathDist",
            "GoalDist",
            "RotateToGoal",
        ):
            self.assertIn(critic, critics)

        self.assertGreater(follow_path["Twirling.scale"], 0.0)
        self.assertLess(
            follow_path["Twirling.scale"],
            follow_path["RotateToGoal.scale"],
        )
        self.assertLess(follow_path["min_vel_x"], 0.0)
        self.assertGreater(follow_path["max_vel_x"], 0.0)
        self.assertLess(follow_path["min_vel_y"], 0.0)
        self.assertGreater(follow_path["max_vel_y"], 0.0)

    def test_in_place_rotation_counts_as_navigation_progress(self) -> None:
        document = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"]
        progress = controller["progress_checker"]
        goal = controller["general_goal_checker"]

        self.assertFalse(
            goal["stateful"],
            "Nav2 must keep checking XY while the final yaw settles",
        )
        self.assertEqual(
            "nav2_controller::PoseProgressChecker",
            progress["plugin"],
        )
        self.assertGreater(progress["required_movement_angle"], 0.0)
        self.assertLess(
            progress["required_movement_angle"],
            goal["yaw_goal_tolerance"],
        )
        self.assertGreater(progress["movement_time_allowance"], 0.0)

    def test_yaw_tolerances_preserve_controller_and_takeover_layers(
        self,
    ) -> None:
        nav2 = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        controller = nav2["controller_server"]["ros__parameters"]
        nav2_yaw_tolerance = controller["general_goal_checker"][
            "yaw_goal_tolerance"
        ]
        adapter = yaml.safe_load(ROBOT_ADAPTER.read_text(encoding="utf-8"))
        takeover_yaw_tolerance = adapter[
            "/**/xczs_cabinet_button_operator"
        ]["ros__parameters"]["navigation_takeover_yaw_tolerance"]
        task_yaw_tolerance = _numeric_constant(
            RUNNER_SOURCE,
            "NAVIGATION_YAW_TOLERANCE_RAD",
        )

        self.assertLess(nav2_yaw_tolerance, task_yaw_tolerance)
        self.assertLessEqual(task_yaw_tolerance - nav2_yaw_tolerance, 0.01)
        self.assertLess(task_yaw_tolerance, takeover_yaw_tolerance)

    def test_position_tolerances_preserve_controller_and_takeover_layers(
        self,
    ) -> None:
        nav2 = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        controller = nav2["controller_server"]["ros__parameters"]
        nav2_position_tolerance = controller["general_goal_checker"][
            "xy_goal_tolerance"
        ]
        adapter = yaml.safe_load(ROBOT_ADAPTER.read_text(encoding="utf-8"))
        operator = adapter["/**/xczs_cabinet_button_operator"][
            "ros__parameters"
        ]
        takeover_distance = operator["navigation_takeover_distance"]
        task_position_tolerance = _numeric_constant(
            RUNNER_SOURCE,
            "NAVIGATION_POSITION_TOLERANCE_M",
        )
        validator_position_tolerance = _shell_numeric_assignment(
            WEB_VALIDATOR,
            "NAVIGATION_POSITION_TOLERANCE_M",
        )

        self.assertLess(nav2_position_tolerance, task_position_tolerance)
        self.assertLessEqual(task_position_tolerance, takeover_distance)
        self.assertEqual(
            task_position_tolerance,
            validator_position_tolerance,
        )


if __name__ == "__main__":
    unittest.main()
