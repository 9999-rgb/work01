"""Regression contract for isolated physical cabinet-knob grasping."""

from __future__ import annotations

import math
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest


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
