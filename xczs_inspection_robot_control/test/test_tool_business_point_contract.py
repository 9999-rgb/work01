"""Contract tests for measured, off-axis cabinet tool business points."""

from __future__ import annotations

import math
import struct
from pathlib import Path
from xml.etree import ElementTree

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent
ADAPTERS = (
    PACKAGE / "config" / "cabinet_robot_adapter.yaml",
    ROOT / "jiang" / "samples" / "demo_cabinet" / "cabinet_robot_adapter.yaml",
)
TOOLS_XACRO = (
    ROOT
    / "xczs_inspection_robot_description"
    / "urdf"
    / "components"
    / "tools.xacro"
)
MANIPULATOR_XACRO = TOOLS_XACRO.with_name("dual_arm_manipulator.xacro")
ROBOT_JOINT_XACROS = (MANIPULATOR_XACRO, TOOLS_XACRO)
TOOLSET_SRDFS = (
    ROOT
    / "xczs_inspection_robot_moveit_config"
    / "config"
    / "xczs_inspection_robot_toolset_A.srdf",
    ROOT
    / "xczs_inspection_robot_moveit_config"
    / "config"
    / "xczs_inspection_robot_toolset_B.srdf",
)
FINGER_MESH = (
    ROOT
    / "xczs_inspection_robot_description"
    / "meshes"
    / "tools"
    / "three_cylinder"
    / "endllink1.STL"
)
OPERATOR = PACKAGE / "src" / "cabinet_button_operator.cpp"
PLUGIN = PACKAGE / "src" / "gazebo" / "cabinet_state_plugin.cpp"
GRASP_SERVICE = PACKAGE / "srv" / "SetCabinetGrasp.srv"


def _operator_parameters(path: Path) -> dict[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["/**/xczs_cabinet_button_operator"]["ros__parameters"]


def _binary_stl_vertices(path: Path) -> list[tuple[float, float, float]]:
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + triangle_count * 50
    vertices: list[tuple[float, float, float]] = []
    for triangle in range(triangle_count):
        values = struct.unpack_from("<9f", data, 84 + triangle * 50 + 12)
        vertices.extend(
            (values[index], values[index + 1], values[index + 2])
            for index in (0, 3, 6)
        )
    return vertices


def test_three_cylinder_business_point_matches_zero_pose_mesh() -> None:
    root = ElementTree.parse(TOOLS_XACRO).getroot()
    joint = root.find(
        ".//joint[@name='${prefix}_three_cyl_finger1_joint']/origin"
    )
    assert joint is not None
    joint_origin = [float(value) for value in joint.attrib["xyz"].split()]

    vertices = _binary_stl_vertices(FINGER_MESH)
    distal_z = min(vertex[2] for vertex in vertices)
    distal = [
        vertex
        for vertex in vertices
        if math.isclose(vertex[2], distal_z, abs_tol=1e-6)
    ]
    contact_center = [
        0.5
        * (
            min(vertex[axis] for vertex in distal)
            + max(vertex[axis] for vertex in distal)
        )
        for axis in range(3)
    ]
    measured_tip = [
        joint_origin[axis] + contact_center[axis] for axis in range(3)
    ]

    assert measured_tip == pytest.approx(
        [0.0265, -0.061735, -0.3885], abs=1e-6
    )


def test_robot_joint_soft_limits_stay_within_hard_limits() -> None:
    """Every declared safety interval must be contained by its hard limit."""

    checked_joints = set()
    for path in ROBOT_JOINT_XACROS:
        root = ElementTree.parse(path).getroot()
        for joint in root.findall(".//joint"):
            hard_limit = joint.find("limit")
            soft_limit = joint.find("safety_controller")
            if hard_limit is None or soft_limit is None:
                continue

            joint_name = joint.attrib["name"]
            hard_lower = float(hard_limit.attrib["lower"])
            hard_upper = float(hard_limit.attrib["upper"])
            soft_lower = float(soft_limit.attrib["soft_lower_limit"])
            soft_upper = float(soft_limit.attrib["soft_upper_limit"])
            assert hard_lower <= soft_lower <= soft_upper <= hard_upper, (
                f"{path.name}:{joint_name} safety interval "
                f"[{soft_lower}, {soft_upper}] is outside hard interval "
                f"[{hard_lower}, {hard_upper}]"
            )
            checked_joints.add(joint_name)

    assert {
        "${prefix}_rotbtn_jaw1_joint",
        "${prefix}_rotbtn_jaw2_joint",
    }.issubset(checked_joints)


@pytest.mark.parametrize("srdf_path", TOOLSET_SRDFS, ids=lambda path: path.stem)
def test_toolset_srdf_collision_pairs_are_unique(srdf_path: Path) -> None:
    """A collision pair is unordered and must be declared only once."""

    root = ElementTree.parse(srdf_path).getroot()
    seen = set()
    for entry in root.findall("disable_collisions"):
        pair = frozenset((entry.attrib["link1"], entry.attrib["link2"]))
        assert len(pair) == 2
        assert pair not in seen, (
            f"{srdf_path.name} declares duplicate collision pair "
            f"{sorted(pair)}"
        )
        seen.add(pair)


def test_adapter_uses_full_business_point_and_zero_joint_guard() -> None:
    builtin = _operator_parameters(ADAPTERS[0])
    sample = _operator_parameters(ADAPTERS[1])
    assert builtin == sample
    # 0.001 rad 低于 Gazebo 位置控制可稳定达到的三电缸手指稳态误差
    # （约 0.0015 rad，对应指尖约 0.6 mm 偏移），归位/标定永远无法通过。
    # 0.003 为稳态误差留 2 倍裕量，同时仍比通用归位容差 0.02 严格约 7 倍。
    assert builtin["tool_tip_calibration_joint_tolerance"] == 0.003

    expected_joints = [
        "r_three_cyl_finger1_joint",
        "r_three_cyl_finger2_joint",
        "r_three_cyl_finger3_joint",
    ]
    for control_type in ("button", "door"):
        profile = builtin["tool_profiles"][control_type]
        assert profile["tool_tip_position"] == pytest.approx(
            [0.0265, -0.061735, -0.3885]
        )
        assert profile["calibration_joint_names"] == expected_joints
        assert profile["calibration_joint_positions"] == [0.0, 0.0, 0.0]
        assert "tool_tip_offset" not in profile

    # Door grasp distance is measured at the same calibrated physical finger
    # tip.  Measuring the three-cylinder base-link origin would be about
    # 0.394 m away and can never pass the plugin's 0.12 m attach threshold.
    door = builtin["tool_profiles"]["door"]
    assert door["grasp_point_position"] == pytest.approx(
        door["tool_tip_position"]
    )
    assert math.dist(door["grasp_point_position"], [0.0, 0.0, 0.0]) > 0.12
    for control_type in ("button", "knob", "switch"):
        assert builtin["tool_profiles"][control_type].get(
            "grasp_point_position", [0.0, 0.0, 0.0]
        ) == pytest.approx([0.0, 0.0, 0.0])


def test_toolset_mismatch_guides_the_web_hot_switch_without_world_restart() -> None:
    builtin = _operator_parameters(ADAPTERS[0])
    sample = _operator_parameters(ADAPTERS[1])
    assert builtin == sample

    reason = builtin["toolset_mismatch_reason"]
    assert "Web 一键切换" in reason
    assert "无需重启 Gazebo 世界" in reason
    assert "无法进行有效 MoveIt 规划" in reason
    assert "实时规划验证" not in reason
    assert "并重启后" not in reason


def test_grasp_service_carries_one_link_local_probe_through_attach_and_restore() -> None:
    schema = GRASP_SERVICE.read_text(encoding="utf-8")
    operator = OPERATOR.read_text(encoding="utf-8")
    plugin = PLUGIN.read_text(encoding="utf-8")

    assert "geometry_msgs/Point robot_grasp_point" in schema
    assert "string operation_lease_id" in schema
    assert operator.count(
        "request->robot_grasp_point.x = grasp_point_position_.x();"
    ) == 2
    assert "robot_pose.Rot().RotateVector(request.robot_grasp_point)" in plugin
    assert "control.collision_restore_robot_grasp_point" in plugin
    assert "robot_grasp_point_is_finite(" in plugin


def test_operator_places_and_revalidates_the_physical_business_point() -> None:
    source = OPERATOR.read_text(encoding="utf-8")

    assert source.count(
        "desired_tip_position -\n"
        "      tf2::quatRotate(tool_rotation, tool_tip_position_)"
    ) == 1
    assert source.count(
        "desired_tip_position -\n"
        "          tf2::quatRotate(tool_rotation, tool_tip_position_)"
    ) == 1
    # Definition plus initial/final checks in both the legacy press and the
    # generic cabinet-control actions.
    assert source.count("verify_tool_tip_calibration_state(") == 5
