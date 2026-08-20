"""Task-record store 纯逻辑单元测试（不依赖 ROS）。

覆盖 ``app.tasks.store.TaskRecordStore``（实现 ``TaskRecordSink``）：
事件过滤、主表 upsert（accepted→progress→completed 生命周期）、~1Hz 明细
全量落盘、控件中文名映射、分页列表 / 详情 / 进度明细查询、聚合摘要、
终态保留策略剪枝，以及 ``app.tasks.labels`` 的码→中文映射与控件目录解析。

与 ``test_asset_store.py`` 同构：注入假 ``control_gateway`` 包、每个用例独立
临时 SQLite 库，以驱动 store 的 URL-keyed engine 缓存。
"""

from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from app.config import settings  # noqa: E402
from app.tasks import labels  # noqa: E402
from app.tasks.store import TaskRecordStore  # noqa: E402


def _task(**overrides: object) -> dict:
    """构造一个 TaskManager 完整任务快照 dict（navigate 为例）。"""
    values = {
        "task_id": "navigate_1750000000_000000",
        "type": "navigate",
        "status": "accepted",
        "phase": "accepted",
        "progress": 0.0,
        "message": "",
        "request": {"cabinet": "demo_cabinet"},
        "created_at": 1750000000.0,
        "updated_at": 1750000000.0,
    }
    values.update(overrides)  # type: ignore[arg-type]
    return values  # type: ignore[return-value]


def _event(event: str, task: dict, *, sequence: int = 1, timestamp: float | None = None):
    """构造 EventHub 事件信封（data.task 携带完整快照）。"""
    return {
        "sequence": sequence,
        "event": event,
        "timestamp": timestamp if timestamp is not None else task.get("updated_at", 0.0),
        "data": {"task": task},
    }


def _operate_task(**overrides: object) -> dict:
    values = {
        "task_id": "operate_1750000100_000001",
        "type": "operate",
        "status": "accepted",
        "phase": "accepted",
        "progress": 0.0,
        "message": "",
        "request": {
            "cabinet": "demo_cabinet",
            "control_id": "box_3_switch_1",
            "command": "set_state",
            "target_state": "on",
        },
        "created_at": 1750000100.0,
        "updated_at": 1750000100.0,
    }
    values.update(overrides)  # type: ignore[arg-type]
    return values  # type: ignore[return-value]


class TaskRecordStoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.db_path = Path(self._temporary_directory.name) / "tasks.db"
        self.store = TaskRecordStore(
            f"sqlite+aiosqlite:///{self.db_path}",
            control_names={"box_3_switch_1": "3号模块开关1"},
        )

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    # ── 事件过滤 ─────────────────────────────────────────────────────

    def test_ignores_untracked_events(self) -> None:
        self.store.record_event(_event("task_reset", _task()))
        self.store.record_event(_event("task_canceled", _task()))
        total, items = self.store.list_records()
        self.assertEqual(0, total)
        self.assertEqual([], items)

    def test_ignores_reset_type(self) -> None:
        task = _task(type="reset", request={"cabinet": "demo_cabinet"})
        self.store.record_event(_event("task_accepted", task))
        total, _ = self.store.list_records()
        self.assertEqual(0, total)

    def test_ignores_malformed_events(self) -> None:
        self.store.record_event({})  # 无 event
        self.store.record_event({"event": "task_progress"})  # 无 data
        self.store.record_event(
            {"event": "task_progress", "data": {}}  # 无 task
        )
        self.store.record_event(
            _event("task_progress", _task(task_id=""))  # 空 task_id
        )
        total, _ = self.store.list_records()
        self.assertEqual(0, total)

    # ── 生命周期 upsert ──────────────────────────────────────────────

    def test_lifecycle_upserts_single_row_with_terminal_fields(self) -> None:
        self.store.record_event(
            _event("task_accepted", _task(status="accepted"))
        )
        self.store.record_event(
            _event(
                "task_progress",
                _task(status="running", phase="navigation_travel",
                      progress=0.5, message="行进中", updated_at=1750000005.0),
                sequence=5,
                timestamp=1750000005.0,
            )
        )
        completed = _task(
            status="failed",
            phase="completed",
            progress=1.0,
            updated_at=1750000010.0,
            completed_at=1750000010.0,
            duration_seconds=10.0,
            failure_code="target_unreachable",
            failure_reason="终点被占用",
            failure_details={"code": "target_unreachable"},
        )
        self.store.record_event(
            _event("task_completed", completed, sequence=8, timestamp=1750000010.0)
        )

        total, items = self.store.list_records()
        self.assertEqual(1, total)
        record = items[0]
        self.assertEqual("navigate_1750000000_000000", record["task_id"])
        self.assertEqual("failed", record["status"])
        self.assertEqual("completed", record["phase"])
        self.assertEqual(1.0, record["progress"])
        self.assertEqual("demo_cabinet", record["cabinet"])
        self.assertEqual("target_unreachable", record["failure_code"])
        self.assertEqual(10.0, record["duration_seconds"])
        self.assertEqual(1750000010.0, record["completed_at"])

        detail = self.store.get_record("navigate_1750000000_000000")
        self.assertIsNotNone(detail)
        self.assertEqual(
            {"code": "target_unreachable"}, detail["failure_details"]
        )
        self.assertEqual("终点被占用", detail["failure_reason"])

    # ── 明细全量落盘（~1Hz，不去重） ────────────────────────────────

    def test_progress_events_fully_persisted_and_ordered(self) -> None:
        self.store.record_event(_event("task_accepted", _task()))
        for index in range(1, 4):
            self.store.record_event(
                _event(
                    "task_progress",
                    _task(status="running", progress=index / 4.0,
                          updated_at=1750000000.0 + index),
                    sequence=index + 1,
                    timestamp=1750000000.0 + index,
                )
            )
        self.store.record_event(
            _event(
                "task_completed",
                _task(status="success", progress=1.0,
                      updated_at=1750000005.0),
                sequence=5,
                timestamp=1750000005.0,
            )
        )

        events = self.store.list_progress_events("navigate_1750000000_000000")
        self.assertEqual(5, len(events))  # accepted + 3 心跳 + completed
        # 按 id 自增排序还原时间线
        self.assertEqual(
            ["task_accepted", "task_progress", "task_progress",
             "task_progress", "task_completed"],
            [event["event"] for event in events],
        )
        self.assertEqual(0.25, events[1]["progress"])
        self.assertEqual(1, events[0]["sequence"])
        self.assertEqual(1.0, events[1]["elapsed_seconds"])

    # ── 控件中文名 ───────────────────────────────────────────────────

    def test_control_name_from_mapping_with_fallback(self) -> None:
        self.store.record_event(_event("task_accepted", _operate_task()))
        record = self.store.get_record("operate_1750000100_000001")
        self.assertEqual("3号模块开关1", record["control_name"])
        self.assertEqual("box_3_switch_1", record["control_id"])
        self.assertEqual("set_state", record["command"])
        self.assertEqual("on", record["target_state"])
        self.assertEqual("demo_cabinet", record["cabinet"])

    def test_control_name_unknown_falls_back_to_id(self) -> None:
        task = _operate_task(control_id="box_9_unknown")
        task["request"]["control_id"] = "box_9_unknown"
        self.store.record_event(_event("task_accepted", task))
        record = self.store.get_record("operate_1750000100_000001")
        self.assertEqual("box_9_unknown", record["control_name"])

    # ── 查询：过滤与分页 ─────────────────────────────────────────────

    def test_list_records_filters_and_paginates(self) -> None:
        for index, (snapshot, status) in enumerate(
            [
                (_task(task_id="nav_success"), "success"),
                (_task(task_id="nav_failed"), "failed"),
                (_operate_task(), "success"),
            ]
        ):
            snapshot = dict(snapshot)
            snapshot["status"] = status
            snapshot["created_at"] = 1750001000.0 + index
            snapshot["updated_at"] = 1750001000.0 + index
            self.store.record_event(
                _event("task_completed", snapshot, timestamp=1750001000.0 + index)
            )

        total, items = self.store.list_records(task_type="operate")
        self.assertEqual(1, total)
        self.assertEqual("operate", items[0]["type"])

        total, items = self.store.list_records(status="success")
        self.assertEqual(2, total)

        total, _ = self.store.list_records(
            control_id="box_3_switch_1", failure_code="target_unreachable"
        )
        self.assertEqual(0, total)

        total, _ = self.store.list_records(
            since=1750001001.0, until=1750001002.0
        )
        self.assertEqual(2, total)

        total, items = self.store.list_records(page=1, page_size=1)
        self.assertEqual(3, total)
        self.assertEqual(1, len(items))

    def test_get_record_missing_returns_none(self) -> None:
        self.assertIsNone(self.store.get_record("nope"))

    # ── 聚合摘要 ─────────────────────────────────────────────────────

    def test_summary_aggregates_dimensions(self) -> None:
        for index, snapshot in enumerate(
            [
                _task(task_id="nav_a", status="success", duration_seconds=5.0),
                _task(task_id="nav_b", status="success", duration_seconds=7.0),
                _operate_task(status="failed", duration_seconds=3.0,
                              failure_code="operation_timeout"),
            ]
        ):
            snapshot = dict(snapshot)
            snapshot["updated_at"] = 1750002000.0 + index
            self.store.record_event(
                _event("task_completed", snapshot, timestamp=1750002000.0 + index)
            )

        summary = self.store.summary()
        self.assertEqual(3, summary["total"])
        self.assertEqual({"navigate": 2, "operate": 1}, summary["by_type"])
        self.assertEqual(
            {"success": 2, "failed": 1}, summary["by_status"]
        )
        self.assertEqual(
            {"operation_timeout": 1}, summary["by_failure_code"]
        )
        self.assertEqual(
            {"demo_cabinet": 3}, summary["by_cabinet"]
        )
        self.assertEqual(
            {"navigate": 6.0, "operate": 3.0},
            summary["avg_duration_seconds"],
        )

    # ── 保留策略 ─────────────────────────────────────────────────────

    def test_retention_prunes_oldest_records_and_cascades_events(self) -> None:
        with mock.patch.object(settings, "task_record_retention", 2):
            for index in range(3):
                task_id = f"navigate_{1750003000 + index}_000000"
                snapshot = _task(
                    task_id=task_id,
                    status="success",
                    updated_at=1750003000.0 + index,
                    completed_at=1750003000.0 + index,
                    duration_seconds=1.0,
                )
                self.store.record_event(_event("task_accepted", snapshot))
                self.store.record_event(
                    _event(
                        "task_progress",
                        _task(task_id=task_id, status="running",
                              updated_at=1750003000.5 + index),
                        timestamp=1750003000.5 + index,
                    )
                )
                self.store.record_event(
                    _event("task_completed", snapshot,
                           timestamp=1750003000.0 + index)
                )

            total, items = self.store.list_records()
            self.assertEqual(2, total)
            # 最旧的 1 条已删除，保留最新的（列表按 id 倒序）
            self.assertEqual(
                ["navigate_1750003002_000000", "navigate_1750003001_000000"],
                [record["task_id"] for record in items],
            )
            # 级联删除最旧任务的进度明细
            self.assertEqual(
                [],
                self.store.list_progress_events("navigate_1750003000_000000"),
            )
            # 保留任务的明细仍在
            self.assertEqual(
                3,
                len(self.store.list_progress_events("navigate_1750003002_000000")),
            )

    def test_retention_disabled_when_zero(self) -> None:
        with mock.patch.object(settings, "task_record_retention", 0):
            for index in range(3):
                snapshot = _task(
                    task_id=f"nav_{index}",
                    status="success",
                    updated_at=1750004000.0 + index,
                    completed_at=1750004000.0 + index,
                )
                self.store.record_event(
                    _event("task_completed", snapshot,
                           timestamp=1750004000.0 + index)
                )
            total, _ = self.store.list_records()
            self.assertEqual(3, total)


class TaskRecordLabelsTest(unittest.TestCase):
    """码→中文标签映射（展示层）。"""

    def test_type_and_status_names(self) -> None:
        self.assertEqual("导航/巡检", labels.type_name("navigate"))
        self.assertEqual("操作", labels.type_name("operate"))
        self.assertEqual("成功", labels.status_name("success"))
        self.assertEqual("失败", labels.status_name("failed"))
        self.assertEqual("取消中", labels.status_name("canceling"))

    def test_command_and_failure_names(self) -> None:
        self.assertEqual("按压", labels.command_name("press"))
        self.assertEqual("拨动", labels.command_name("toggle"))
        self.assertEqual("操作超时", labels.failure_code_name("operation_timeout"))
        self.assertEqual("目标不可达", labels.failure_code_name("target_unreachable"))

    def test_phase_name_prefix_fallback(self) -> None:
        self.assertEqual("导航中", labels.phase_name("navigation_travel"))
        self.assertEqual("操作中", labels.phase_name("operation_approach"))
        self.assertEqual("已接受", labels.phase_name("accepted"))
        self.assertEqual("已完成", labels.phase_name("completed"))
        self.assertEqual("some_custom_phase", labels.phase_name("some_custom_phase"))

    def test_unknown_codes_fall_back_to_original(self) -> None:
        self.assertEqual("mystery", labels.status_name("mystery"))
        self.assertEqual("mystery", labels.failure_code_name("mystery"))
        self.assertIsNone(labels.type_name(None))

    def test_load_control_display_names_from_catalog(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "catalog.yaml"
            path.write_text(
                "xczs/robot:\n"
                "  ros__parameters:\n"
                "    controls:\n"
                "      box_1_button_1:\n"
                "        display_name: '1号模块红色按钮'\n"
                "      box_1_button_2:\n"
                "        display_name: ''\n"
                "      box_1_button_3:\n"
                "        type: button\n",
                encoding="utf-8",
            )
            names = labels.load_control_display_names(path)
            self.assertEqual("1号模块红色按钮", names["box_1_button_1"])
            # 空 display_name / 无 display_name → 回退 control_id
            self.assertEqual("box_1_button_2", names["box_1_button_2"])
            self.assertEqual("box_1_button_3", names["box_1_button_3"])

    def test_load_control_display_names_tolerates_bad_input(self) -> None:
        self.assertEqual({}, labels.load_control_display_names(None))
        self.assertEqual(
            {}, labels.load_control_display_names("/nonexistent/catalog.yaml")
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "catalog.yaml"
            path.write_text(": not: valid: yaml\n", encoding="utf-8")
            self.assertEqual({}, labels.load_control_display_names(path))


if __name__ == "__main__":
    unittest.main()
