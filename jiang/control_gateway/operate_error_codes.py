"""Single source of truth for ``OperateCabinetControl`` result-code labels.

:mod:`control_gateway.ros_node` and :mod:`control_gateway.cabinet_client` both
have to turn the numeric ``error_code`` of an ``OperateCabinetControl`` result
into the snake_case string the Web layer reports.  Each carried its own
hand-written copy of the same 19-entry table, so a constant added to
``OperateCabinetControl.action`` could quietly leave one of the two answering
``"unknown_error"`` while the other reported the new name.  The table lives
here once and both call it; ``jiang/tests/test_operate_error_codes.py`` fails if
the action ever grows a constant this table does not cover.

The generated action type is imported at module scope, so this module is only
importable where the ROS workspace is sourced -- which is true of both callers
already, since they import the same action themselves.
"""

from __future__ import annotations

from typing import Optional

from xczs_inspection_robot_interfaces.action import OperateCabinetControl


# ``OperateCabinetControl.Result`` constant -> label reported to the Web layer.
# Every label is the constant name lowercased; that is a convention, not a
# computed mapping, so the pairs stay written out for reviewability.
_RESULT_CODE_NAMES = {
    "SUCCESS": "success",
    "INVALID_CONTROL": "invalid_control",
    "UNSUPPORTED_COMMAND": "unsupported_command",
    "NOT_READY": "not_ready",
    "NAVIGATION_FAILED": "navigation_failed",
    "PLANNING_FAILED": "planning_failed",
    "EXECUTION_FAILED": "execution_failed",
    "GRASP_FAILED": "grasp_failed",
    "TARGET_NOT_REACHED": "target_not_reached",
    "RELEASE_FAILED": "release_failed",
    "CANCELED": "canceled",
    "INTERNAL_ERROR": "internal_error",
    "INVALID_FORCE": "invalid_force",
    "INSUFFICIENT_FORCE": "insufficient_force",
    "UNREACHABLE": "unreachable",
    "CONTACT_DETECTION_TIMEOUT": "contact_detection_timeout",
    "RESOURCE_BUSY": "resource_busy",
    "LEASE_LOST": "lease_lost",
    "TOOLSET_MISMATCH": "toolset_mismatch",
}


def operate_error_code_name(error_code: Optional[int]) -> str:
    """Return the Web-facing label for an ``OperateCabinetControl`` result code.

    ``None`` means the result channel itself never delivered a code and is
    reported as ``"result_channel_failed"``; a code the action does not define
    is reported as ``"unknown_error"``.
    """
    if error_code is None:
        return "result_channel_failed"
    code = int(error_code)
    for constant, name in _RESULT_CODE_NAMES.items():
        if getattr(OperateCabinetControl.Result, constant, None) == code:
            return name
    return "unknown_error"
