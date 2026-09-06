#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""validate_db1_drawer 与 validate_db1_cap_gates 共用的抽屉控件检查（单一实现）。

docx/AGENT-电气夹层抽拉柜后续实现与验收执行方案.md §5 要求两把验收器复用同一套
有效性/运动/位置/状态序列检查，同一逻辑只保留一份并由两处共同调用，避免复制后
各自漂移。本模块只含纯数据判定（读取控制快照 dict），不发起任何 HTTP/ROS 通信；
网络差异、鉴权、超时等留在各验收器内。

本模块位置约定：位于 scripts/validate/，运行 validate_db1_* 脚本时该目录是
sys.path[0]，`from _db1_shared import ...` 可解析。只使用标准库。

规则要点（AGENT §5.2）：
- 全部位置检查必须是有限数，拒绝 NaN/Inf；
- 静止判定必须看当前 in_motion（非历史），valid 必须为 true；
- 非目标控件按 control_id 集合比较，不复用序号 zip；
- 状态位置（closed/open detent）来自目录 state_positions，不硬编码。
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict, List

# msg/CabinetControl.msg 常量（与接口定义保持一致）。
TYPE_BUTTON = 0
TYPE_KNOB = 1
TYPE_SWITCH = 2
TYPE_DOOR = 3
TYPE_SLIDER = 4
TYPE_DRAWER = 5
TYPE_LABELS: Dict[int, str] = {
    TYPE_BUTTON: "button",
    TYPE_KNOB: "knob",
    TYPE_SWITCH: "switch",
    TYPE_DOOR: "door",
    TYPE_SLIDER: "slider",
    TYPE_DRAWER: "drawer",
}

# 其余控件“未变化”的判定误差（历史合同值，两把验收器共用）。
DEFAULT_UNTOUCHED_POSITION_EPS_M = 1e-4

# 快照需记录的字段。current_position 单独做带容差的数值比较（不能进字符串
# 签名，否则 ±1e-4 内的微小浮点差会误判为漂移）。
_MOTION_KEYS = ("valid", "in_motion", "current_state",
                "current_position", "transition_sequence")
# 非目标控件必须保持静止且有效；任何一项未保持即串扰。
_SIGNATURE_KEYS = ("valid", "in_motion", "current_state",
                   "transition_sequence")


class Db1ValidationError(Exception):
    """抽屉控件检查失败（人类可读中文原因）。"""


def is_finite_number(value: Any) -> bool:
    """仅接受有限数值（bool 也是 int 子类，按非数值拒绝）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def require_position_number(value: Any, label: str) -> float:
    """位置必须存在且为有限数：拒绝 NaN/Inf/None/字符串等。"""
    if not is_finite_number(value):
        raise Db1ValidationError(
            "%s 不是有限数值（NaN/Inf/非数值一律拒绝）：%r" % (label, value)
        )
    return float(value)


def control_by_id(
    controls: List[Dict[str, Any]], control_id: str
) -> Dict[str, Any]:
    """按 control_id 取出控件；缺失即抛错。"""
    for item in controls:
        if item.get("control_id") == control_id:
            return item
    raise Db1ValidationError("控件目录中不存在 %s" % control_id)


def expected_position_for_state(
    control: Dict[str, Any], state: str
) -> float:
    """从目录 state_ids/state_positions 求目标状态的 detent 位置（有限数）。"""
    state_ids = control.get("state_ids") or []
    state_positions = control.get("state_positions") or []
    for index, state_id in enumerate(state_ids):
        if state_id == state:
            if index >= len(state_positions):
                raise Db1ValidationError("目录缺少 %s 的状态位置" % state)
            return require_position_number(
                state_positions[index],
                "%s 的状态位置(state=%s)" % (control.get("control_id"), state),
            )
    raise Db1ValidationError(
        "目录缺少 %s 状态（state_ids=%s）" % (state, state_ids)
    )


def valid_and_stationary(control: Dict[str, Any]) -> bool:
    """当前 valid==true 且 in_motion==false（读的是当前字段，不是历史）。"""
    return (
        control.get("valid") is True
        and control.get("in_motion") is False
    )


def require_stationary_and_valid(
    control: Dict[str, Any], drawer: str, label: str
) -> None:
    """控件必须当前静止且有效；用于门后收尾/基线复核。"""
    if not valid_and_stationary(control):
        raise Db1ValidationError(
            "%s 后 %s 未静止（valid=%s in_motion=%s）"
            % (label, drawer, control.get("valid"), control.get("in_motion"))
        )


def untouched_snapshot(
    controls: List[Dict[str, Any]], drawer: str
) -> List[Dict[str, Any]]:
    """记录非目标控件的快照（按 control_id 集合，排除目标 drawer）。

    返回按键 _MOTION_KEYS 的条目，按 control_id 排序，便于集合比较。
    """
    snapshot: List[Dict[str, Any]] = []
    for item in controls:
        if item.get("control_id") == drawer:
            continue
        entry = {"control_id": item.get("control_id")}
        for key in _MOTION_KEYS:
            entry[key] = item.get(key)
        snapshot.append(entry)
    return sorted(snapshot, key=lambda entry: str(entry["control_id"]))


def _motion_signature(entry: Dict[str, Any]) -> str:
    return "|".join(
        str(entry.get(key)) for key in _SIGNATURE_KEYS
    )


def require_others_stable(
    snapshot: List[Dict[str, Any]], cabinet: str
) -> None:
    """非目标控件初始即应静止且有效，否则当场拒绝后续比较。"""
    unstable = [
        entry
        for entry in snapshot
        if entry["valid"] is not True or entry["in_motion"] is not False
    ]
    if unstable:
        raise Db1ValidationError(
            "%s 的非目标控件初始即非静止/无效：%s"
            % (
                cabinet,
                ", ".join(
                    "%s(valid=%s,in_motion=%s)"
                    % (entry["control_id"], entry["valid"], entry["in_motion"])
                    for entry in unstable
                ),
            )
        )


def untouched_drifted(
    before: List[Dict[str, Any]],
    after: List[Dict[str, Any]],
    drawer: str,
    eps: float = DEFAULT_UNTOUCHED_POSITION_EPS_M,
) -> List[str]:
    """按 control_id 集合比较前后快照，返回所有漂移的非目标控件 id。

    判定口径（与 validate_db1_drawer 合同一致但改为按 id 集合比较）：
    - 前后 id 集合必须一致（多出来/消失的都算漂移）；
    - 同一 id 的 valid/in_motion/current_state/transition_sequence 必须不变；
    - current_position 必须是有限数，且差量在 [-eps, +eps] 内（NaN/Inf 即漂移）。
    """
    before_by_id = {entry["control_id"]: entry for entry in before}
    after_by_id = {entry["control_id"]: entry for entry in after}
    drifted: List[str] = []
    for control_id, previous in before_by_id.items():
        current = after_by_id.get(control_id)
        if current is None:
            drifted.append(control_id)
            continue
        if _motion_signature(previous) != _motion_signature(current):
            drifted.append(control_id)
            continue
        if not is_finite_number(previous["current_position"]) or not is_finite_number(
            current["current_position"]
        ):
            drifted.append(control_id)
            continue
        delta = float(current["current_position"]) - float(
            previous["current_position"]
        )
        if delta < -eps or delta > eps:
            drifted.append(control_id)
    for control_id in after_by_id:
        if control_id not in before_by_id:
            drifted.append(control_id)
    return drifted


def require_untouched_unchanged(
    before: List[Dict[str, Any]],
    after: List[Dict[str, Any]],
    drawer: str,
    eps: float = DEFAULT_UNTOUCHED_POSITION_EPS_M,
) -> None:
    """若发现非目标控件漂移则抛错（物理串扰检测）。"""
    drifted = untouched_drifted(before, after, drawer, eps)
    if drifted:
        raise Db1ValidationError(
            "检测到物理串扰：非目标控件状态发生变化：%s（目标=%s）"
            % (", ".join(sorted(drifted)), drawer)
        )


def require_settled(
    fetch: Callable[[], List[Dict[str, Any]]],
    drawer_item: Dict[str, Any],
    drawer: str,
    expected_state: str,
    tolerance: float,
    label: str,
) -> None:
    """复核控件静止于期望状态/位置（读当前数据，拒绝 NaN/Inf）。

    fetch 由各验收器提供：拉取该柜最新 controls 列表的回调（HTTP 细节在外层）。
    """
    controls = fetch()
    item = control_by_id(controls, drawer)
    require_stationary_and_valid(item, drawer, label)
    expected_position = expected_position_for_state(drawer_item, expected_state)
    state = item.get("current_state")
    if state != expected_state:
        raise Db1ValidationError(
            "%s 后 %s 状态为 %s，期望 %s" % (label, drawer, state, expected_state)
        )
    position = require_position_number(
        item.get("current_position"),
        "%s 后 %s current_position" % (label, drawer),
    )
    if abs(position - expected_position) > tolerance:
        raise Db1ValidationError(
            "%s 后 %s 位置 %.4f 超出 %s 容差 %.4f±%.3f"
            % (label, drawer, position, expected_state, expected_position,
               tolerance)
        )
