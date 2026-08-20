"""Regression contract for isolated physical cabinet-knob grasping."""

from __future__ import annotations

import math
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CABINET_XACROS = (
    WORKSPACE
    / "xczs_inspection_robot_description"
    / "urdf"
    / "control_cabinet.urdf.xacro",
    WORKSPACE
    / "jiang"
    / "samples"
    / "demo_cabinet"
    / "control_cabinet.urdf.xacro",
)
ROBOT_ADAPTERS = (
    WORKSPACE
    / "xczs_inspection_robot_control"
    / "config"
    / "cabinet_robot_adapter.yaml",
    WORKSPACE / "jiang" / "samples" / "demo_cabinet" / "cabinet_robot_adapter.yaml",
)


def _expanded_cabinet(path: Path) -> ET.Element:
    completed = subprocess.run(
        ["xacro", str(path), "cabinet_name:=knob_grasp_contract"],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(completed.stdout)


@pytest.mark.parametrize("cabinet_xacro", CABINET_XACROS)
def test_every_knob_uses_compliant_twist_coupling(
    cabinet_xacro: Path,
) -> None:
    """No knob may fall back to the cross-model rigid-joint path."""
    root = _expanded_cabinet(cabinet_xacro)
    plugin = next(
        element
        for element in root.findall(".//plugin")
        if element.get("filename") == "libxczs_cabinet_state.so"
    )
    knobs = [
        control
        for control in plugin.findall("control")
        if control.findtext("control_type") == "knob"
    ]
    assert len(knobs) == 11

    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for knob in knobs:
        control_id = knob.findtext("control_id")
        values = {
            name: float(knob.findtext(name, "nan"))
            for name in (
                "detent_stiffness",
                "grasp_coupling_stiffness",
                "grasp_coupling_damping",
                "grasp_coupling_max_effort",
            )
        }
        assert all(math.isfinite(value) for value in values.values()), control_id
        assert values["grasp_coupling_stiffness"] > 0.0, control_id
        assert values["grasp_coupling_damping"] > 0.0, control_id
        assert values["grasp_coupling_max_effort"] > 0.0, control_id

        detents = [float(value) for value in knob.findtext("detents", "").split()]
        assert len(detents) == 3 and detents[0] < detents[1] < detents[2]
        midpoint = 0.5 * (detents[1] + detents[2])
        joint = joints[knob.findtext("joint_name")]
        friction = float(joint.find("dynamics").get("friction", "0"))

        # At the center/right boundary, the wrist coupling must still beat
        # the old detent's restoring torque and joint friction.  Otherwise the
        # physical knob can never cross into the requested right detent.
        available_torque = min(
            values["grasp_coupling_max_effort"],
            values["grasp_coupling_stiffness"] * (detents[2] - midpoint),
        )
        resisting_torque = (
            values["detent_stiffness"] * abs(midpoint - detents[1])
            + friction
        )
        assert available_torque > resisting_torque, control_id


def _operator_parameters(path: Path) -> dict[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["/**/xczs_cabinet_button_operator"]["ros__parameters"]


def test_box_5_knob_has_directional_tool_clearance_calibration() -> None:
    builtin = _operator_parameters(ROBOT_ADAPTERS[0])
    sample = _operator_parameters(ROBOT_ADAPTERS[1])
    assert builtin == sample

    controls = builtin["controls"]
    assert isinstance(controls, dict)
    calibration = controls["box_5_knob"]
    assert calibration["navigation_station"]["local_anchor"] == [
        0.717470,
        1.115,
        0.0,
    ]
    assert calibration["navigation_station"]["standoff"] == 0.400
    assert calibration["detent_release_fraction"] == 0.58
    assert calibration["ready_joint_seed"] == {
        "joint_names": [f"r_arm_{index}_joint" for index in range(7)],
        "positions": [
            1.221067,
            1.158552,
            -1.448952,
            -1.806018,
            1.998380,
            0.014786,
            -1.656403,
        ],
    }

    expected_rolls = [
        0.0,
        -0.872664625997,
        0.0,
        -1.1344640138,
        0.0,
        -1.57079632679,
        0.0,
        -1.83259571459,
        0.0,
    ]
    assert calibration["tool_roll_offsets"] == expected_rolls
    # Row-major left/center/right: every physically used adjacent transition
    # has its own measured roll; direct left<->right entries remain unused.
    assert all(
        math.isfinite(calibration["tool_roll_offsets"][index])
        for index in (1, 3, 5, 7)
    )
