"""``OperateCabinetControl`` 结果码 → 标签表的覆盖守卫。

``ros_node`` 与 ``cabinet_client`` 共用 ``control_gateway.operate_error_codes``
里的同一张表；本测试锁住它与 ``OperateCabinetControl.action`` 的一致性——
一旦 action 新增常量而表没跟上，这里会失败，而不是让 Web 层悄悄回落到
``"unknown_error"``。

需要已 source 的 ROS 工作区（要导入生成的 action 类型）。
"""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path


JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from control_gateway.operate_error_codes import (  # noqa: E402
    _RESULT_CODE_NAMES,
    operate_error_code_name,
)
from xczs_inspection_robot_interfaces.action import (  # noqa: E402
    OperateCabinetControl,
)


def _result_constants() -> dict:
    """返回 ``Result`` 上的整型常量名 → 值（跳过 ``SLOT_TYPES`` / property）。"""
    return {
        name: value
        for name, value in vars(OperateCabinetControl.Result).items()
        if not name.startswith("_") and isinstance(value, int)
    }


class OperateErrorCodeNameTest(unittest.TestCase):
    def test_table_covers_every_result_constant(self) -> None:
        constants = _result_constants()
        self.assertEqual(
            set(constants),
            set(_RESULT_CODE_NAMES),
            "operate_error_codes 的表与 OperateCabinetControl.action 的 Result 常量不一致。",
        )

    def test_every_constant_maps_to_its_lowercased_name(self) -> None:
        for name, value in _result_constants().items():
            with self.subTest(constant=name):
                self.assertEqual(operate_error_code_name(value), name.lower())

    def test_none_reports_result_channel_failure(self) -> None:
        # 结果通道没送回 code 时，两个调用方都必须报同一个标签，
        # 而不是抛 TypeError（cabinet_client 以前没有 None 分支）。
        self.assertEqual(
            operate_error_code_name(None), "result_channel_failed"
        )

    def test_undefined_code_reports_unknown_error(self) -> None:
        undefined = max(_result_constants().values()) + 1
        self.assertEqual(operate_error_code_name(undefined), "unknown_error")

    def test_success_is_zero_and_named_success(self) -> None:
        # 0 同时是「默认值」和 SUCCESS，标签必须是 success 而不是 unknown_error。
        self.assertEqual(OperateCabinetControl.Result.SUCCESS, 0)
        self.assertEqual(operate_error_code_name(0), "success")


if __name__ == "__main__":
    unittest.main()
