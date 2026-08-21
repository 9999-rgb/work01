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
FINGER_MESH = (
    ROOT
    / "xczs_inspection_robot_description"
    / "meshes"
    / "tools"
    / "three_cylinder"
    / "endllink1.STL"
)
OPERATOR = PACKAGE / "src" / "cabinet_button_operator.cpp"


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


def test_adapter_uses_full_business_point_and_zero_joint_guard() -> None:
    builtin = _operator_parameters(ADAPTERS[0])
    sample = _operator_parameters(ADAPTERS[1])
    assert builtin == sample
    assert builtin["tool_tip_calibration_joint_tolerance"] == 0.001

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
