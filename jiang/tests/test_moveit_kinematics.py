from pathlib import Path

import pytest

from control_gateway.moveit_kinematics import (
    KinematicsConfigError,
    configured_solver_classes,
    discover_moveit_kinematics_plugins,
    validate_kinematics_plugins,
)


def _write_plugin_index(prefix: Path, class_name: str) -> None:
    resource = (
        prefix
        / "share"
        / "ament_index"
        / "resource_index"
        / "moveit_core__pluginlib__plugin"
        / "test_kinematics"
    )
    resource.parent.mkdir(parents=True)
    description = prefix / "share" / "test_kinematics" / "plugins.xml"
    description.parent.mkdir(parents=True)
    description.write_text(
        '<library path="test_solver">\n'
        f'  <class name="{class_name}" type="test::Solver" '
        'base_class_type="kinematics::KinematicsBase"/>\n'
        "</library>\n",
        encoding="utf-8",
    )
    resource.write_text(
        "share/test_kinematics/plugins.xml\n",
        encoding="utf-8",
    )


def test_configured_solver_classes_requires_each_group_solver(tmp_path):
    config = tmp_path / "kinematics.yaml"
    config.write_text("left_arm: {}\n", encoding="utf-8")

    with pytest.raises(KinematicsConfigError, match="left_arm.kinematics_solver"):
        configured_solver_classes(config)


def test_discover_and_validate_moveit_plugin_classes(tmp_path):
    class_name = "test_kinematics/TestSolver"
    _write_plugin_index(tmp_path, class_name)
    config = tmp_path / "kinematics.yaml"
    config.write_text(
        "left_arm:\n"
        f"  kinematics_solver: {class_name}\n"
        "right_arm:\n"
        f"  kinematics_solver: {class_name}\n",
        encoding="utf-8",
    )

    assert discover_moveit_kinematics_plugins([tmp_path]) == (class_name,)
    report = validate_kinematics_plugins(config, prefixes=[tmp_path])
    assert report.group_solvers == (
        ("left_arm", class_name),
        ("right_arm", class_name),
    )


def test_validate_moveit_plugin_classes_rejects_missing_solver(tmp_path):
    config = tmp_path / "kinematics.yaml"
    config.write_text(
        "right_arm:\n"
        "  kinematics_solver: missing/Solver\n",
        encoding="utf-8",
    )

    with pytest.raises(KinematicsConfigError, match="missing/Solver"):
        validate_kinematics_plugins(config, prefixes=[tmp_path])
