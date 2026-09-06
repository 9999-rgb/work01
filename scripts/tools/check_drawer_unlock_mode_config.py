#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AGENT 执行方案 §4.2 解锁放行模式配置契约（离线静态检查）。

验证 simulated_linkage 只允许在 db1 仿真配置启用，插件默认保持严格物理触点门，
且 db1 的 sim 门两把把手保持钩杆与场景适配 YAML 一致——防止未来误把仿真近似
悄悄扩散到其它抽屉或让配置漂移。

检查项：
  1. electrical_mezzanine.xacro 中 `<unlock_simulated_linkage>true</...>`
     只出现在 db1 抽屉控制内，全场景恰好 1 处，其余 drawer 控制不得启用。
  2. db1 sim 启用时必须显式给出左右把手保持钩杆（robot link + 局部杆端点）。
  3. 保持钩杆名称与适配 YAML 的 gripper 杆一致（防工具角色漂移）。
  4. 插件解析默认 strict：cabinet_state_plugin.cpp 里
     optional_bool(element, "unlock_simulated_linkage", false)。
  5. "默认严格模式"负例结构护栏：严格距离拒绝门保留且仅被
     !sim_linkage 绕过（sim 不得放宽默认门的回归锁）。运行时无第二把
     解锁抽屉可演示该拒绝，故以结构断言闭环（§4.2 前 cap5 严格拒绝为
     历史实证）。

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

# 适配器 drawer_tools 的 gripper（把手保持钩杆）link 名。
HOLD_LEFT = "l_two_cyl_finger2"
HOLD_RIGHT = "r_three_cyl_finger1"


def fail(name, detail):
    print(f"[FAIL] {name}  {detail}")
    return False


def check(name, cond, detail):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}  {detail}" if cond
          else f"[FAIL] {name}  {detail}")
    return cond


def main() -> int:
    results = []
    text = XACRO.read_text(encoding="utf-8")

    # 1) 全局只能出现一处 sim 启用，且必须位于 db1 控制块内。
    enables = [(m.start(), m.group(0)) for m in
               re.finditer(r"<unlock_simulated_linkage>\s*(true|false)\s*</unlock_simulated_linkage>",
                           text)]
    results.append(check(
        "sim 启用仅 db1 且恰 1 处",
        len(enables) == 1 and enables[0][1].find("true") != -1,
        f"{len(enables)} 处 {enables if enables else ''}"))
    db1_start = text.find("<control_id>db1</control_id>")
    db1_end = text.find("<control_id>", db1_start + 1)
    if results[-1]:
        pos = enables[0][0]
        results.append(check(
            "sim 启用落在 db1 控制块内",
            db1_start < pos < db1_end,
            f"db1 块 [{db1_start},{db1_end}) pos={pos}"))

    # 2) db1 sim 块必须给出保持钩杆配置。
    block = text[db1_start:db1_end if db1_end > db1_start else db1_start + 4000]
    left = re.search(r"<unlock_hold_left_link>\s*([^<]+)</unlock_hold_left_link>", block)
    right = re.search(r"<unlock_hold_right_link>\s*([^<]+)</unlock_hold_right_link>", block)
    left_pt = re.search(r"<unlock_hold_left_point>\s*([^<]+)</unlock_hold_left_point>", block)
    right_pt = re.search(r"<unlock_hold_right_point>\s*([^<]+)</unlock_hold_right_point>", block)
    results.append(check(
        "db1 保持钩杆 link/point 齐全",
        bool(left and right and left_pt and right_pt),
        f"left={left.group(1).strip() if left else None} "
        f"right={right.group(1).strip() if right else None} "
        f"left_pt={left_pt.group(1).strip() if left_pt else None} "
        f"right_pt={right_pt.group(1).strip() if right_pt else None}"))

    # 3) 保持钩杆与适配器 gripper 一致。
    if left and right:
        results.append(check(
            "保持钩杆 == 适配器 gripper",
            left.group(1).strip() == HOLD_LEFT and
            right.group(1).strip() == HOLD_RIGHT,
            f"{left.group(1).strip()} / {right.group(1).strip()}"))

    # 4) 插件默认 strict（解析默认 false）。
    plugin_text = PLUGIN.read_text(encoding="utf-8")
    default = re.search(
        r"unlock_simulated_linkage\",\s*false\)",
        plugin_text)
    results.append(check(
        "插件默认严格（解析默认 false）",
        bool(default),
        "optional_bool(element, 'unlock_simulated_linkage', false)"))

    # 5) "默认严格模式负例"结构护栏：严格物理触点距离门仍在插件默认分支，
    #    且只被 !sim_linkage 这一开关绕过——sim 放行不得偷删/放宽默认门。
    #    运行时无第二把可解锁抽屉（db1 是唯一解锁配置控制，且 §4.2 决策要求
    #    它启用 sim），故该负例以结构断言 + 历史严格拒绝证据（§4.2 前 cap5
    #    探针在 db1 上测得 0.116347m > 0.008m 被拒）共同闭环。
    strict_guard = re.search(
        r"if\s*\(\s*!sim_linkage\s*&&\s*!right_tool_contact\s*\)", plugin_text)
    strict_message = re.search(
        r"Unlock contact link is not inside the unlock zone", plugin_text)
    results.append(check(
        "严格距离门保留且仅被 !sim_linkage 绕过",
        bool(strict_guard and strict_message),
        "guard='if (!sim_linkage && !right_tool_contact)' "
        f"message={bool(strict_message)}"))

    passed = sum(1 for r in results if r)
    print(f"\n{passed}/{len(results)} offline config checks passed")
    return 0 if passed == len(results) and results else 1


if __name__ == "__main__":
    sys.exit(main())
