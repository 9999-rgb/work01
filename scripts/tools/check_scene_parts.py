"""核对电气夹层场景的实体构成（开发排查工具）。

三条独立判据：
  1. `/get_entity_state` 逐个查绘制抽屉本体、legacy 遗留 part、db1 真按钮
     link —— 与 operator 的物理真值测量走同一个服务，故名实一致；
  2. `/link_states` 全量 link 名交叉核对 —— 不依赖上面的名字解析与作用域
     猜测，作为独立第二来源；
  3. 结果按 ok / not_found / timeout / unavailable 分开报。

2026-09-10 修正（重要）：过去 `get()` 把「服务超时/不可用」与「实体不存在」
一律打成 `MISSING/ERR`。gzserver 忙时（boot 期、导航期，CPU 常在 80%+）5 s
超时几乎必然发生 —— 现场因此把**确实存在**的 b1 / b1p 误读成「抽屉本体和真
解锁按钮没随场景出现」。手动复核同刻 `/get_entity_state` 对
`xczs_scene_floor::b1` 与 `::b1p` 均 success=True（b1p 实测
(0.0997, 4.6850, 0.9990)，与合同 (0.099, 4.685, 1.000) 相符），模型从未缺失。
「查不到」和「查不动」是两件事，绝不能共用一句结论。
"""

import sys
import time

import rclpy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node

SERVICE = "/get_entity_state"
SCENE = "xczs_scene_floor"
DRAWERS = ["s1", "s2", "b1", "s3", "m1"]
# 抽屉实体合并进「一体」模型后不再存在的独立 part。注意 b1p 不在其中：
# 2026-09-09 用户拍板把 b1p 恢复为 db1 的真实可按解锁按钮（b1 下的 prismatic
# 子 link，见 electrical_mezzanine.xacro），它是**应该存在**的现存部件，不是
# 遗留 part；列在这里会被误报成「legacy 未清」。
LEGACY = ["s1r", "s1p", "s2r", "s2p", "b1r", "s3r", "s3p", "m1r", "m1p"]
# db1 的真实按钮 link：与 b1 之间是 prismatic 子关节（帽的压入行程即解锁物证）。
BUTTON_LINKS = {"b1p": "b1p_joint"}

rclpy.init()
node = Node("check_parts")
cli = node.create_client(GetEntityState, SERVICE)
if not cli.wait_for_service(timeout_sec=20.0):
    print(f"FATAL: {SERVICE} 不可用（gazebo_ros_state 插件未起 / gzserver 未就绪）")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(2)


def get(name):
    """查一个实体；返回 ``(state, status)``。

    status ∈ {ok, not_found, timeout, unavailable} —— 与 operator
    ``drawer_physics_link_point`` 同一条判据链（同名服务、同一 ``success``
    语义），只是把失败原因如实分开，不再糊成一句 MISSING。
    """
    if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=10.0):
        return None, "unavailable"
    request = GetEntityState.Request()
    request.name = name
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    response = future.result()
    if response is None:
        return None, "timeout"
    if not response.success:
        return None, "not_found"
    return response.state, "ok"


def report(label, name, expect):
    state, status = get(name)
    if status != "ok":
        print(f"{label}: {status.upper()} ({name}, 期望 {expect})")
        return None
    p = state.pose.position
    print(f"{label}: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} ({name})")
    return state


print("=== drawer bodies (should be PRESENT) ===")
bodies_ok = 0
for drawer in DRAWERS:
    if report(drawer, f"{SCENE}::{drawer}", "存在") is not None:
        bodies_ok += 1

print("=== legacy detached parts (should all be absent after merge) ===")
legacy_present = []
for part in LEGACY:
    state = report(part, f"{SCENE}::{part}", "已并入一体，不存在")
    if state is not None and state.pose.position.z > 0.001:
        legacy_present.append(part)
print(f"legacy parts still present (z>0.001): {len(legacy_present)} {legacy_present}")

print("=== db1 real unlock button link (should be PRESENT, z≈1.0) ===")
button_ok = 0
for link, joint in BUTTON_LINKS.items():
    state = report(f"{link} ({joint})", f"{SCENE}::{link}", "存在且 z≈1.0")
    if state is not None and state.pose.position.z > 0.5:
        button_ok += 1

# 第二来源交叉核对：/link_states 全量 link 名。与上面各自独立 —— 上面走服务
# 解析单个名字，这里直接读世界的 link 名清单，作用域/解析差异不会同时骗过两者。
print("=== cross-check via /link_states (independent name source) ===")
watched = set(DRAWERS) | set(LEGACY) | set(BUTTON_LINKS)
seen = set()
sub = node.create_subscription(
    LinkStates, "/link_states", lambda m: seen.update(n for n in m.name), 10)
deadline = time.time() + 20.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
    if watched & seen:
        break
if not seen:
    print("  /link_states 无数据（世界未发布 link 状态？）")
else:
    for name in sorted(watched & seen):
        print(f"  {name}: 在 /link_states 中（scoped={[n for n in seen if n.endswith('::' + name)][:1]}）")

print(
    f"SUMMARY: drawer bodies {bodies_ok}/{len(DRAWERS)} ok, "
    f"db1 button {button_ok}/{len(BUTTON_LINKS)} ok, "
    f"legacy present {len(legacy_present)} (应为 0)"
)
node.destroy_node()
rclpy.shutdown()
