# 精确停靠断困加力（2026-09-22）

## 变更与结论

- **背景现象**：在发电机层按按钮时报
  `The robot could not reach the odometry staging pose within the precision-docking timeout`
  —— 停靠误差**恒在 0.042 m**、45 s 超时；而机器人**物理上距工位只有 0.041 m**。
- **实测到的失败签名**：
  1. 路由器**确实在发指令**：`/xczs/cmd_vel` 上量到 0.0335 m/s，恰等于
     `docking_linear_gain 0.8 × 残差 0.042`，与设计一致；
  2. 底盘**几乎不动**：40 s 内真值位移 **1.3 mm**（原地抖）；
  3. 而同样量级的手动指令都能推动底盘：0.02 / 0.05 / 0.10 / 0.15 m/s 分别位移
     0.023 / 0.045 / 0.107 / 0.167 m，**斜向 (0.05, 0.05) 也走 0.057 m**；
  4. 停靠期按消息统计的非零占空比只有 **38~42%**（501 条里 189~212 条），非零段按
     ~80 ms 分段断续。
  ⇒ 有效速度 ≈ `0.8 × 残差 × 0.4`。**残差小时（如 4 cm）有效速度约 0.013 m/s，低于
  从静止起动的摩擦门槛 → 车起不来 → 误差不动 → 超时**；残差大时（0.15~0.20 m）指令
  够大、一旦滚起来就能收敛——这解释了该失败的「时好时坏」。
- **改法**：停靠循环加**断困加力**。位置误差在 `docking_breakaway_stall_sec`(0.6 s)
  内改善不足 `docking_breakaway_progress`(2 mm) 时，把指令抬到
  `docking_breakaway_speed`(0.08 m/s)；重新有进展就退回比例项。抬高的速度仍受
  `docking_max_linear_speed`(0.15) 夹钳。反馈里会显示 `(breakaway applied)`，便于
  现场识别它是否触发。
- 三个参数写进**三份**适配器（内置、样例、资产库副本）；契约测试强制内置与样例一致。
  > 后续核对（2026-09-22）：**这三份都不是 operator 停靠循环的参数源**——实例适配器
  > `config/scene_controls/<scene>_adapter.yaml` 才是（经 `cabinet_instances.yaml` 的
  > `adapter_config` 传入；实测两个 operator 进程的 `--params-file` 都是它），而它缺这三个键，
  > 值靠 C++ 兜底默认（0.08 / 0.6 / 0.002，与本节点数值相同）生效。改这三个键对停靠循环
  > 无效且不报错。
  > 另外上句「契约测试强制内置与样例一致」的覆盖面被高估：`test_tool_business_point_contract.py`
  > 确实逐键比对内置与样例，但**显式排除 `controls` 段**，而逐控件退距正落在被排除的范围内——
  > 例如 `box_3_button_1/2` 的 standoff 在内置已重校准为 0.730、样例与资产库副本仍是旧值
  > 0.780，无人拦；受该测试保护的只有 `docking_breakaway_*` 这类顶层参数。资产库那份不在
  > 比对范围内。

## 证据

| 场景 | 停靠起始误差 | 结果 |
| --- | ---: | --- |
| 导航到站后按压（两次） | 0.196 / 0.113 m | success 57.2 / 55.3 s，7 s 内收敛进 8 mm 容差 |
| 切场景 + 切套装之后再按压 | 0.151 m | success 62.3 s |
| **人为造 4 cm 小残差**后按压 | 0.035 m | **success 58.3 s**，6 s 收敛 |
| 以上各次 | — | **均未触发 breakaway**（正常路径未被改动） |

## 校验与复现

- 全量 `jiang/tests` 与 `colcon test`（control 包 **202 项**）通过。
- 复现停靠观测：`POST /task/operate {"cabinet":"generator_plant","control_id":"fbutton1",
  "command":"press"}`，轮询 `/task/{id}/status` 看 `Precision docking error: …`；
  配合采样 `/xczs/cmd_vel` 与 `/model_states` 可同时得到「指令」与「实际位移」。
- **未做到（诚实记录）**：**原失败的触发条件没能复现**。按用户描述的路径（切到发电机层
  场景 + 套装 B → 按 fbutton1）实测成功；人为造同样的 4 cm 小残差也成功；切换之后也没有
  重复的插件实例（`planar_move` 始终 1 个）。因此本修复是**按测得的失败签名对症**
  （「有指令、车却不动」），**不是**「复现 → 修 → 复验」的闭环证明。
- **未查清**：那 60% 的零是谁发的。该话题的发布 / 订阅方较多
  （`base_command_router`、zenoh bridge、`planar_move`、`planar_stabilizer`），本轮未能
  确定归属。建议下一步：在停靠窗口结束时记录**实测底盘位移**（形如
  `docking: commanded X m/s, moved Y m`），让下一次卡死自证是「指令太弱」还是「车被按住」。
