#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抽屉解锁放行模式配置契约（离线静态检查）。

2026-09-09 起 db1 解锁语义由 AGENT 方案 §4.2 的 simulated_linkage 仿真近似
升级为**真按钮**：右解锁短级电缸把 b1p 帽真实推入，插件读 b1p_joint 位移取证。
本脚本把新契约钉成结构断言，防止未来误退回仿真近似、或让四处配置（xacro /
plugin / adapter YAML / controls YAML）悄悄漂移。

检查项：
  1. 全场景不得再出现 <unlock_simulated_linkage>（仿真近似已废止）。
  2. db1 抽屉控制块恰有 1 处 <unlock_button_id>，且它指向同场景一个真实的
     <control_type>button</control_type> 控制（按钮必须是真控制、真 joint）。
  3. db1 必须显式给出左右保持钩杆 link + point。
  4. 保持钩杆 link 名必须等于适配 YAML drawer_tools.{left,right}.
     gripper_contact_link —— 钩杆角色以适配器为准、从 YAML 现读，不硬编码
     （硬编码副本会在换工具杆后静默过期：017c 换杆即是一例）。
  5. 插件解析 <unlock_button_id> 并读该按钮 joint 的 Position 与 press_
     threshold 比较（真位移物证，非自由位姿声明）。
  6. 插件不得存在任何把 simulation_acceptance 置 true 的路径。
  7. 严格物理触点距离门保留（!right_tool_contact → 拒绝 + 原拒绝消息）。

用法：python3 scripts/tools/check_drawer_unlock_mode_config.py
先决条件：无（离线文本检查，不连 ROS / Gazebo）。
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
XACRO = ROOT / "xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro"
PLUGIN = ROOT / "xczs_inspection_robot_gazebo/src/cabinet_state_plugin.cpp"
ADAPTER = ROOT / "xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_adapter.yaml"

DRAWER_ID = "db1"


def check(name, cond, detail):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}  {detail}")
    return bool(cond)


def adapter_gripper_links(text):
    """适配 YAML 里左右 gripper（把手保持钩杆）link 名。

    适配文件是 ROS 参数 YAML（顶层为 /**/ros__parameters 嵌套），故按结构取
    drawer_tools.<side>.gripper_contact_link，而不是硬编码一份 link 名副本
    —— 副本会在换工具杆后静默过期（017c 换杆即是一例）。
    """
    import yaml

    document = yaml.safe_load(text)

    def find_drawer_tools(node):
        if isinstance(node, dict):
            if isinstance(node.get("drawer_tools"), dict):
                return node["drawer_tools"]
            for value in node.values():
                found = find_drawer_tools(value)
                if found is not None:
                    return found
        return None

    tools = find_drawer_tools(document)
    if not tools:
        return {}
    links = {}
    for side in ("left", "right"):
        entry = tools.get(side)
        if isinstance(entry, dict) and entry.get("gripper_contact_link"):
            links[side] = str(entry["gripper_contact_link"]).strip()
    return links


def main() -> int:
    results = []
    xacro_text = XACRO.read_text(encoding="utf-8")
    plugin_text = PLUGIN.read_text(encoding="utf-8")
    adapter_text = ADAPTER.read_text(encoding="utf-8")

    # 1) 仿真近似已废止：不得再有 sim 启用元素。
    sim = re.findall(r"<unlock_simulated_linkage>", xacro_text)
    results.append(check(
        "无 simulated_linkage（仿真近似已废止）",
        not sim,
        f"{len(sim)} 处 <unlock_simulated_linkage>"))

    # 2) 定位 db1 抽屉控制块，取其 <unlock_button_id> 与其指向的按钮控制。
    db1_match = re.search(
        rf"<control_id>\s*{DRAWER_ID}\s*</control_id>", xacro_text)
    if not db1_match:
        results.append(check("db1 抽屉控制块存在", False, "未找到 control_id=db1"))
        print(f"\n{sum(1 for r in results if r)}/{len(results)} offline config checks passed")
        return 1
    next_id = xacro_text.find("<control_id>", db1_match.end())
    block = xacro_text[db1_match.start():next_id if next_id > 0 else len(xacro_text)]

    button_ids = re.findall(
        r"<unlock_button_id>\s*([^<]+?)\s*</unlock_button_id>", block)
    results.append(check(
        "db1 恰 1 处 unlock_button_id",
        len(button_ids) == 1,
        f"{button_ids}"))

    if len(button_ids) == 1:
        button_id = button_ids[0]
        button_block = re.search(
            rf"<control_id>\s*{re.escape(button_id)}\s*</control_id>(.*?)</control>",
            xacro_text, re.S)
        is_button = bool(button_block) and "<control_type>button</control_type>" in button_block.group(1)
        results.append(check(
            "unlock_button_id 指向同场景真实 button 控制",
            is_button,
            f"id={button_id} button_block={'found' if button_block else 'MISSING'} "
            f"control_type=button:{is_button}"))

    # 3) 保持钩杆 link + point 齐全。
    left = re.search(r"<unlock_hold_left_link>\s*([^<]+?)\s*</unlock_hold_left_link>", block)
    right = re.search(r"<unlock_hold_right_link>\s*([^<]+?)\s*</unlock_hold_right_link>", block)
    left_pt = re.search(r"<unlock_hold_left_point>\s*([^<]+?)\s*</unlock_hold_left_point>", block)
    right_pt = re.search(r"<unlock_hold_right_point>\s*([^<]+?)\s*</unlock_hold_right_point>", block)
    results.append(check(
        "db1 保持钩杆 link/point 齐全",
        bool(left and right and left_pt and right_pt),
        f"left={left.group(1) if left else None} right={right.group(1) if right else None} "
        f"left_pt={left_pt.group(1) if left_pt else None} "
        f"right_pt={right_pt.group(1) if right_pt else None}"))

    # 4) 保持钩杆 == 适配器 gripper（现读 YAML，不硬编码）。
    gripper = adapter_gripper_links(adapter_text)
    results.append(check(
        "适配器 gripper link 可解析",
        set(gripper) == {"left", "right"},
        f"{gripper}"))
    if left and right and set(gripper) == {"left", "right"}:
        results.append(check(
            "保持钩杆 == 适配器 gripper_contact_link",
            left.group(1) == gripper["left"] and right.group(1) == gripper["right"],
            f"xacro {left.group(1)}/{right.group(1)} vs "
            f"adapter {gripper['left']}/{gripper['right']}"))

    # 5) 插件真按钮门：解析 unlock_button_id + 读按钮 joint Position vs
    #    press_threshold。两者缺一即退化为「自由位姿声明」。
    parses_button = re.search(
        r'"unlock_button_id"\s*,\s*""\)', plugin_text)
    button_evidence = re.search(
        r"button_position\s*>=\s*button_press_threshold", plugin_text)
    refusal = re.search(r"is not pressed \(its joint position", plugin_text)
    results.append(check(
        "插件解析 unlock_button_id 并做真位移判据",
        bool(parses_button and button_evidence and refusal),
        f"parse={bool(parses_button)} position>=threshold={bool(button_evidence)} "
        f"refusal_message={bool(refusal)}"))

    # 6) 不得存在把 simulation_acceptance 置 true 的路径（恒 false）。
    sim_true = re.findall(r"simulation_acceptance\s*=\s*true", plugin_text)
    results.append(check(
        "无 simulation_acceptance=true 路径",
        not sim_true,
        f"{len(sim_true)} 处"))

    # 7) 严格物理触点距离门保留。
    strict_guard = re.search(
        r"if\s*\(\s*!right_tool_contact\s*\)", plugin_text)
    strict_message = re.search(
        r"Unlock contact link is not inside the unlock zone", plugin_text)
    results.append(check(
        "严格距离门保留（!right_tool_contact 拒绝）",
        bool(strict_guard and strict_message),
        f"guard={bool(strict_guard)} message={bool(strict_message)}"))

    passed = sum(1 for r in results if r)
    print(f"\n{passed}/{len(results)} offline config checks passed")
    return 0 if passed == len(results) and results else 1


if __name__ == "__main__":
    sys.exit(main())
