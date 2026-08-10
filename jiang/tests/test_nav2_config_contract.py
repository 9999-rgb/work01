"""Regression tests for safety-critical Nav2 controller configuration."""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
NAV2_PARAMS = (
    WORKSPACE
    / "xczs_inspection_robot_nav2"
    / "config"
    / "nav2_params.yaml"
)
ROBOT_ADAPTER = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "cabinet_robot_adapter.yaml"
)
RUNNER_SOURCE = WORKSPACE / "jiang" / "control_gateway" / "runner.py"


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


class Nav2ConfigContractTest(unittest.TestCase):
    def test_in_place_rotation_counts_as_navigation_progress(self) -> None:
        document = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"]
        progress = controller["progress_checker"]
        goal = controller["general_goal_checker"]

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


if __name__ == "__main__":
    unittest.main()
