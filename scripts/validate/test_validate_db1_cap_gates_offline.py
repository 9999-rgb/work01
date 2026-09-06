#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""validate_db1_cap_gates 的离线失败-恢复测试（不启动任何 ROS/Web/Gazebo）。

用进程内假件替换 HTTP（模块级 _request）、ros2 CLI（模块级 subprocess.run）与
时间（模块级 time.monotonic/time.sleep），逐条证明 AGENT §5.2 的收尾分支：
  1. 任务被接受后超时 -> 取消 + 等终态 + 恢复 debug_stage_cap=0；
  2. 409 全局占用 -> 等占用任务结束重试一次后通过；
  3. ros2 param set 失败 -> 不登记参数改动，收尾安全（不误恢复）；
  4. 门中 KeyboardInterrupt -> 取消活动任务 + 恢复参数，返回 130；
  5. 门失败（本次结果缺封顶标记）-> 已登记参数改动，收尾恢复为 0，
     摘要仍输出已完成 gates 与首个失败门。

运行方式（任选其一，需在仓库根目录）：
  python3 scripts/validate/test_validate_db1_cap_gates_offline.py
  python3 -m unittest discover -s scripts/validate \
      -p 'test_validate_db1_cap_gates_offline.py'
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import io
import json
import os
import sys
import types
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace

_VALIDATE_DIR = os.path.dirname(os.path.abspath(__file__))
if _VALIDATE_DIR not in sys.path:
    sys.path.insert(0, _VALIDATE_DIR)


def _load_module(name: str) -> types.ModuleType:
    # validate_db1_cap_gates 无 .py 扩展名：spec_from_file_location 不认识其
    # 后缀会返回 loader=None，故直接用 SourceFileLoader 显式加载。
    path = os.path.join(_VALIDATE_DIR, name)
    module_name = "_offline_" + name.replace(".", "_")
    loader = importlib.machinery.SourceFileLoader(module_name, path)
    spec = importlib.util.spec_from_loader(module_name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


VALIDATOR = _load_module("validate_db1_cap_gates")

from _db1_shared import TYPE_BUTTON, TYPE_KNOB  # noqa: E402

# 固定的柜/抽屉目录（与电气夹层场景合同一致的最小夹具）。
SCENE = "electrical_mezzanine"
CABINET = "electrical_mezzanine"
DRAWER = "db1"

DRAWER_ITEM = {
    "control_id": DRAWER,
    "control_type": VALIDATOR.TYPE_DRAWER,
    "required_toolset": "A",
    "operable": True,
    "toolset_compatible": True,
    "adapter_validated": True,
    "valid": True,
    "in_motion": False,
    "current_state": "closed",
    "current_position": 0.0,
    "transition_sequence": "closed",
    "state_ids": ["closed", "open"],
    "state_positions": [0.0, 0.3],
}

OTHER_ITEMS = [
    {
        "control_id": "clamp_a",
        "control_type": TYPE_BUTTON,
        "required_toolset": "A",
        "operable": True,
        "valid": True,
        "in_motion": False,
        "current_state": "idle",
        "current_position": 0.0,
        "transition_sequence": "idle",
        "state_ids": ["idle", "active"],
        "state_positions": [0.0, 1.0],
    },
    {
        "control_id": "fan_knob",
        "control_type": TYPE_KNOB,
        "required_toolset": "A",
        "operable": True,
        "valid": True,
        "in_motion": False,
        "current_state": "off",
        "current_position": 0.0,
        "transition_sequence": "off",
        "state_ids": ["off", "low", "high"],
        "state_positions": [0.0, 0.2, 0.5],
    },
]


class FakeTime:
    """单调钟每访问一次就大步前进，sleep 为 no-op：避免测试真实等待。"""

    def __init__(self, step: float = 5.0) -> None:
        self._now = 0.0
        self.step = step

    def monotonic(self) -> float:
        value = self._now
        self._now += self.step
        return value

    def sleep(self, seconds: float) -> None:  # noqa: U100 - 故意不等待
        return None


class FakeRos:
    """ros2 param get/set CLI 假件；跟踪 operator 参数实际值。"""

    def __init__(self) -> None:
        self.cap_value = 0
        self.fail_on_set = False
        self.calls: list = []

    def run(self, argv, env=None, capture_output=None, text=None, timeout=None):  # noqa: U100
        argv = list(argv)
        self.calls.append(argv)
        is_set = "set" in argv
        if is_set and self.fail_on_set:
            return SimpleNamespace(returncode=1, stdout="", stderr="boom")
        if is_set:
            index = argv.index("debug_stage_cap")
            self.cap_value = int(argv[index + 1])
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        return SimpleNamespace(
            returncode=0,
            stdout="Integer value is: %d" % self.cap_value,
            stderr="",
        )


def _success_terminal(task_id: str, cap: int, message: str) -> dict:
    return {
        "task_id": task_id,
        "status": "success",
        "duration_seconds": 0.4,
        "request": {
            "cabinet": CABINET,
            "control_id": DRAWER,
            "command": "set_state",
            "target_state": "open",
        },
        "result": {
            "message": message,
            "catalog_generation": 7,
            "physical_outcome_confirmed": True,
            "final_state_verified": True,
            "transport_succeeded": True,
            "recovery_succeeded": True,
            "grasp_released": True,
            "final_state": "closed",
            "final_position": 0.0,
            "peak_position": 0.012,
            "diagnostic_stage": "cap%d_recovery" % cap,
            "operation_executed": True,
        },
    }


class FakeWeb:
    """HTTP 层假件：/scenes、/scene/switch、/cabinets/*/controls、/task/*。

    mode：
      "success"      operate 任务直接 success（含封顶标记）
      "badmarker"    operate success 但 result.message 缺封顶标记
      "pending"      接受任务后一直 pending -> 触发任务超时
      "ki"           接受任务后首次查询抛 KeyboardInterrupt
    busy：首次 operate POST 返回 409（被他人任务占用）。
    """

    def __init__(self, mode: str = "success", busy: bool = False) -> None:
        self.mode = mode
        self.busy = busy
        self.canceled = False
        self.ki_raised = False
        self.operate_posts = 0
        self.records: list = []

    def _controls(self) -> list:
        items = [dict(DRAWER_ITEM)]
        for other in OTHER_ITEMS:
            items.append(dict(other))
        return items

    def handle(self, method: str, path: str,
               body: dict = None) -> dict:  # noqa: U100
        self.records.append((method, path))
        if path == "/scenes":
            return {"active": SCENE}
        if path == "/scene/switch":
            return {"status": "unchanged"}
        if path == "/cabinets/" + CABINET + "/controls":
            return {
                "available": True,
                "catalog_coherent": True,
                "catalog_active_toolset": "A",
                "controls": self._controls(),
            }
        if path == "/task/navigate":
            return {"task_id": "nav-1"}
        if path == "/task/nav-1/status":
            return {"status": "success", "duration_seconds": 0.1,
                    "result": {"error": {"position_m": 0.005,
                                         "yaw_rad": 0.01}}}
        if path == "/task/operate" and method == "POST":
            self.operate_posts += 1
            if self.busy and self.operate_posts == 1:
                raise VALIDATOR.WebError(
                    409,
                    {"error": "global task busy",
                     "details": {"active_task_id": "busy-1"}},
                )
            return {"task_id": "op-1"}
        if path == "/task/busy-1/status":
            return {"status": "success", "duration_seconds": 0.1,
                    "result": {"error": {"position_m": 0.001,
                                         "yaw_rad": 0.0}}}
        if path == "/task/op-1/cancel" and method == "POST":
            self.canceled = True
            return {"status": "canceling"}
        if path == "/task/op-1/status":
            if self.canceled:
                return {"status": "canceled", "task_id": "op-1",
                        "request": {"cabinet": CABINET,
                                    "control_id": DRAWER},
                        "result": {}}
            if self.mode == "ki":
                if not self.ki_raised:
                    self.ki_raised = True
                    raise KeyboardInterrupt()
                return {"status": "pending", "task_id": "op-1"}
            if self.mode == "pending":
                return {"status": "pending", "task_id": "op-1"}
            if self.mode == "badmarker":
                return _success_terminal(
                    "op-1", 1, "operation finished but no cap marker present")
            return _success_terminal(
                "op-1", 1,
                "cap1 reached: debug_stage_cap=1 reached: "
                "rail latch relocked; actuators home")
        raise AssertionError("假件未处理: %s %s" % (method, path))


def _load_summary(stdout: str) -> dict:
    return json.loads(stdout)


class CapGatesOfflineTest(unittest.TestCase):
    """在进程内运行 main()，用假件替换 _request/subprocess/time。"""

    def _run(self, mode: str = "success", busy: bool = False,
             caps: str = "1", timeout: int = 1,
             fail_param_set: bool = False) -> tuple:
        fake_web = FakeWeb(mode=mode, busy=busy)
        fake_ros = FakeRos()
        fake_ros.fail_on_set = fail_param_set
        fake_time = FakeTime()

        def fake_request(method, api, path, body=None, headers=None):  # noqa: U100
            return fake_web.handle(method, path, body)

        original = (
            VALIDATOR._request,
            VALIDATOR.subprocess,
            VALIDATOR.time,
            list(sys.argv),
        )
        VALIDATOR._request = fake_request
        VALIDATOR.subprocess = SimpleNamespace(run=fake_ros.run)
        VALIDATOR.time = fake_time
        argv = ["validate_db1_cap_gates", "--scene", SCENE,
                "--cabinet", CABINET, "--drawer", DRAWER,
                "--caps", caps, "--no-navigate",
                "--timeout", str(timeout)]
        sys.argv = argv
        buffer = io.StringIO()
        try:
            with redirect_stdout(buffer):
                code = VALIDATOR.main()
        finally:
            (VALIDATOR._request, VALIDATOR.subprocess, VALIDATOR.time,
             sys.argv) = original
        return code, _load_summary(buffer.getvalue()), fake_web, fake_ros

    # -- §5.2 #2 任务被接受后超时：取消 + 等终态 + 恢复参数 ----------------
    def test_task_accepted_then_timeout_cancels_and_restores(self) -> None:
        code, summary, fake_web, fake_ros = self._run(mode="pending")
        self.assertEqual(code, 1)
        self.assertFalse(summary["ok"])
        self.assertEqual(summary["gates"][0]["status"], "FAIL")
        self.assertIsNotNone(summary["gates"][0]["task_id"])
        # cleanup：活动任务已取消并确认终态；参数恢复为 0。
        cleanup = summary["cleanup"]
        self.assertIs(cleanup["active_task_canceled"], True)
        self.assertEqual(cleanup["active_task_terminal"], "canceled")
        self.assertIs(cleanup["param_restored"], True)
        self.assertTrue(fake_web.canceled)
        # ros2 CLI 曾 set 1，收尾再 set 0。
        set_calls = [call for call in fake_ros.calls if "set" in call]
        self.assertTrue(any(int(call[call.index("debug_stage_cap") + 1]) == 1
                            for call in set_calls))
        self.assertTrue(any(int(call[call.index("debug_stage_cap") + 1]) == 0
                            for call in set_calls))

    # -- §5.2 #2 409 占用：等占用任务结束重试一次后通过 -------------------
    def test_409_busy_retry_then_pass(self) -> None:
        code, summary, fake_web, _fake_ros = self._run(
            mode="success", busy=True)
        self.assertEqual(code, 0)
        self.assertTrue(summary["ok"])
        # 首次 operate POST 被 409 拒绝，第二次才接受。
        operate_posts = [
            item for item in fake_web.records
            if item[0] == "POST" and item[1] == "/task/operate"
        ]
        self.assertEqual(len(operate_posts), 2)
        # 本门通过：任务 id/代次/开始时间齐全，证据与封顶标记齐全。
        gate = summary["gates"][0]
        self.assertEqual(gate["status"], "PASS")
        self.assertEqual(gate["task_id"], "op-1")
        self.assertEqual(gate["catalog_generation"], 7)
        self.assertIsNotNone(gate["started_at"])
        self.assertIsNotNone(gate["elapsed_s"])
        self.assertTrue(gate["evidence"]["physical_outcome_confirmed"])
        self.assertTrue(gate["evidence"]["recovery_succeeded"])
        self.assertTrue(gate["evidence"]["grasp_released"])
        self.assertNotEqual(gate["evidence"]["latch_or_home_evidence"],
                            "not_verified")
        self.assertIn("closed", str(gate["drawer_after"]))
        # 参数恢复为 0（任务已终态，cleanup 无需取消）。
        self.assertIs(summary["cleanup"]["active_task_canceled"], False)
        self.assertIs(summary["cleanup"]["param_restored"], True)

    # -- §5.2 #1 参数设置失败：不登记改动，收尾安全（不误恢复） ------------
    def test_param_set_failure_is_clean(self) -> None:
        code, summary, fake_web, fake_ros = self._run(
            mode="success", fail_param_set=True)
        self.assertEqual(code, 1)
        gate = summary["gates"][0]
        self.assertEqual(gate["status"], "FAIL")
        self.assertIn("param set", str(summary["error"]))
        # 从未 set 0（因为从未成功改过参数），收尾仍标记已恢复/未改动。
        set_calls = [call for call in fake_ros.calls if "set" in call]
        self.assertEqual(len(set_calls), 1)  # 仅失败的 set 1
        self.assertIs(summary["cleanup"]["param_restored"], True)
        self.assertIn("未改动", summary["cleanup"]["note"])

    # -- §5.2 #2 门中 KeyboardInterrupt：取消活动任务 + 恢复参数，rc=130 -----
    def test_keyboard_interrupt_midtask_cancels_and_restores(self) -> None:
        code, summary, fake_web, fake_ros = self._run(mode="ki")
        self.assertEqual(code, 130)
        self.assertTrue(summary.get("interrupted"))
        self.assertEqual(summary["gates"][0]["status"], "FAIL")
        self.assertEqual(summary["gates"][0]["first_fail"],
                         "KeyboardInterrupt")
        self.assertIs(summary["cleanup"]["active_task_canceled"], True)
        self.assertEqual(summary["cleanup"]["active_task_terminal"],
                         "canceled")
        self.assertIs(summary["cleanup"]["param_restored"], True)
        set_calls = [call for call in fake_ros.calls if "set" in call]
        self.assertTrue(any(int(call[call.index("debug_stage_cap") + 1]) == 1
                            for call in set_calls))
        self.assertTrue(any(int(call[call.index("debug_stage_cap") + 1]) == 0
                            for call in set_calls))

    # -- §5.2 #1/#7 门失败（缺封顶标记）：已登记改动则恢复 0；摘要含失败门 -
    def test_gate_failure_restores_to_zero_and_reports_first_fail(self) -> None:
        code, summary, fake_web, fake_ros = self._run(mode="badmarker")
        self.assertEqual(code, 1)
        gate = summary["gates"][0]
        self.assertEqual(gate["status"], "FAIL")
        self.assertIn("封顶标记", str(summary["error"]))
        self.assertEqual(summary["first_fail_gate"]["cap"], 1)
        # 门未通过但完成了的 gates 与失败原因都已输出。
        self.assertEqual(len(summary["gates"]), 1)
        self.assertEqual(summary["gates"][0]["first_fail"], gate["first_fail"])
        # 参数改动已登记 -> 收尾恢复为 0。
        self.assertIs(summary["cleanup"]["param_restored"], True)
        set_calls = [call for call in fake_ros.calls if "set" in call]
        self.assertTrue(any(int(call[call.index("debug_stage_cap") + 1]) == 1
                            for call in set_calls))
        self.assertTrue(any(int(call[call.index("debug_stage_cap") + 1]) == 0
                            for call in set_calls))


if __name__ == "__main__":
    unittest.main(verbosity=2)
