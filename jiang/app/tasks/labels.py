"""任务记录展示层：码→中文标签映射 + 控件中文名解析。

仅 FastAPI 展示层使用，不落库；未知码回退原文。``load_control_display_names``
读取活动控件目录 YAML（``/**/ros__parameters/controls/<control_id>.display_name``）
生成 control_id → 中文名映射，供 ``TaskRecordStore`` 写入 ``control_name``。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

import yaml

#: 任务类型中文名。
TYPE_LABELS: dict[str, str] = {
    "navigate": "导航/巡检",
    "operate": "操作",
}

#: 任务状态中文名（TaskManager 状态值全集）。
STATUS_LABELS: dict[str, str] = {
    "accepted": "已接受",
    "running": "运行中",
    "canceling": "取消中",
    "success": "成功",
    "failed": "失败",
    "canceled": "已取消",
}

#: operate 命令中文名。
COMMAND_LABELS: dict[str, str] = {
    "press": "按压",
    "set_state": "设定状态",
    "set_position": "设定位置",
    "toggle": "拨动",
}

#: 已知失败码中文名（未知回退原文）。
FAILURE_CODE_LABELS: dict[str, str] = {
    # 导航
    "navigation_timeout": "导航超时",
    "target_unreachable": "目标不可达",
    "pose_deviation_exceeded": "位姿偏差超限",
    "localization_jump": "定位跳变",
    "navigation_station_unstable": "导航工位不稳定",
    "navigation_station_refresh_failed": "导航工位刷新失败",
    "backend_unavailable": "后端不可用",
    "navigation_rejected": "导航被拒",
    "final_pose_unavailable": "终位姿不可用",
    "final_pose_frame_mismatch": "终位姿坐标系不匹配",
    "final_pose_stale": "终位姿已过期",
    "initial_pose_unavailable": "初始位姿不可用",
    "shutdown_backend_unconfirmed": "关闭后端未确认",
    # 操作
    "operation_timeout": "操作超时",
    "operation_failed": "操作失败",
    "operation_rejected": "操作被拒",
    "not_ready": "未就绪",
    "operation_active": "已有操作进行中",
    "result_channel_failed": "结果通道失败",
}

#: 精确阶段中文名。
_PHASE_EXACT_LABELS: dict[str, str] = {
    "accepted": "已接受",
    "completed": "已完成",
    "canceling": "取消中",
    "cancel_failed": "取消失败",
    "rejected": "被拒绝",
}

#: 阶段前缀回退：navigation_* / operation_* 归并为大类。
_PHASE_PREFIX_LABELS: tuple[tuple[str, str], ...] = (
    ("navigation_", "导航中"),
    ("operation_", "操作中"),
)


def type_name(value: str | None) -> str | None:
    return None if value is None else TYPE_LABELS.get(value, value)


def status_name(value: str | None) -> str | None:
    return None if value is None else STATUS_LABELS.get(value, value)


def command_name(value: str | None) -> str | None:
    return None if value is None else COMMAND_LABELS.get(value, value)


def failure_code_name(value: str | None) -> str | None:
    return None if value is None else FAILURE_CODE_LABELS.get(value, value)


def phase_name(value: str | None) -> str | None:
    """阶段中文名：精确匹配优先，其次前缀大类，最后原文。"""
    if value is None:
        return None
    if value in _PHASE_EXACT_LABELS:
        return _PHASE_EXACT_LABELS[value]
    for prefix, label in _PHASE_PREFIX_LABELS:
        if value.startswith(prefix):
            return label
    return value


def load_control_display_names(path: str | os.PathLike[str] | None) -> dict[str, str]:
    """读控件目录 YAML，返回 ``control_id → display_name`` 映射。

    结构：``/**/ros__parameters/controls/<control_id>.display_name``。路径缺失、
    文件损坏或结构不匹配时返回空映射（store 回退 control_id 原文）。
    """
    if path is None:
        return {}
    catalog_path = Path(path)
    if not catalog_path.is_file():
        return {}
    try:
        with catalog_path.open("r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle)
    except Exception:  # noqa: BLE001 - 展示层尽力而为
        return {}
    if not isinstance(document, Mapping):
        return {}
    controls: Mapping[str, object] = {}
    for node in document.values():
        if not isinstance(node, Mapping):
            continue
        params = node.get("ros__parameters")
        if isinstance(params, Mapping) and isinstance(params.get("controls"), Mapping):
            controls = params["controls"]  # type: ignore[assignment]
            break
    names: dict[str, str] = {}
    for control_id, entry in controls.items():
        if not isinstance(entry, Mapping):
            continue
        name = str(entry.get("display_name", "")).strip()
        names[str(control_id)] = name or str(control_id)
    return names
