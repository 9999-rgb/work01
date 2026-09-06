# AGENT 执行方案：电气夹层抽拉柜钩取、侧缝支撑、右把手解锁与双臂抽拉

> **2026-09-06 执行入口更新：** 后续工作以[最新进度审查与无人值守执行方案](AGENT-电气夹层抽拉柜后续实现与验收执行方案.md)为准。用户已明确授权“无需提问，长时间离场，Agent 自行决策”；本文中旧的询问/等待确认流程不再作为执行阻塞。下文保留用于需求追溯，其中实体侧缝碰撞、同位姿解锁及旧验收流程须按新方案的几何证据和实现分支执行。

> 项目目录：`/home/live/work01`
>
> 目标场景：`electrical_mezzanine`（电气夹层）
>
> 文档日期：2026-09-03
>
> 文档性质：供编码 Agent 分阶段实施和验收；本文不代表功能已经通过 Gazebo 验收。

## 0. 新需求冻结

### 0.1 机构定义

本项目末端不是普通开合夹爪，而是多根独立伸缩电缸组成的专用工具：

- 左手：一根钩住左把手，一根伸入左侧柜体侧缝并支撑。
- 右手：一根钩住右把手，一根伸入右侧柜体侧缝并支撑，另有一根短行程电缸按下右把手本体上的解锁按钮。
- 夹爪的职责是“钩住把手并随机械臂移动”，不是普通两指闭合夹紧。
- 支撑点是把手外侧附近的柜体左右侧缝，不是把手下方的柜面。
- 解锁按钮就在右把手本体上。
- `s1p/s2p/b1p/s3p/m1p` 小圆点仍是指示灯，不参与解锁。

### 0.2 正确动作顺序

打开抽屉：

```text
预检查
  -> 底盘停靠并保持
  -> 双臂一次到达工作位姿
  -> 左右夹爪分别钩住左右把手
  -> 左右支撑电缸伸入两侧柜体侧缝并抵紧
  -> 右手独立解锁电缸按下右把手本体按钮
  -> 建立双手抽拉耦合
  -> 保持钩取、支撑和解锁，同步向 +X 抽出
  -> 到达开位并稳定
  -> 安全释放和撤离
```

关闭抽屉不重复解锁：

```text
预检查
  -> 底盘停靠并保持
  -> 双臂到达当前开位把手的工作位姿
  -> 左右夹爪钩住把手
  -> 左右支撑电缸伸入侧缝并建立支撑
  -> 建立双手耦合
  -> 同步向 -X 推回
  -> 到达关位并确认重新锁止
  -> 松开钩取、收回支撑、双臂撤离
```

禁止继续采用“右臂先去解锁位，再去支撑位，最后去抓取位”的旧流程。三个机构必须在同一个双臂工作位姿下，通过各自电缸按顺序伸出完成动作。

### 0.3 总体验收标准

- 工作位姿由真实钩爪接触点计算，不使用笼统工具中心点。
- 两个钩爪先挂住把手，并有左右独立接触证据。
- 两个支撑电缸随后伸入左右侧缝，并有独立接触证据。
- 右侧解锁电缸最后按下右把手按钮，并有位置、行程和接触证据。
- 抽拉期间底盘固定，左右臂沿同一时间轴同步运动。
- Gazebo 无持续抖动、穿模、刚性闭环打架或单侧拖拽。
- Web、任务层、ROS 2 Action、控制器和 Gazebo 反馈一致。
- `./run_all.sh --web` 冷启动后能从 Web 完成打开和关闭。
- 三个内置场景完成配色，且不改变几何、碰撞和物理参数。

## 1. Agent 工作边界

### 1.1 开始前

完整阅读 `AGENTS.md`、`CLAUDE.md`、本文件、`docx/P0-电气夹层抽拉柜几何合同.md` 和总方案。随后执行：

```bash
cd /home/live/work01
git status --short
git diff --check
```

仓库存在用户和前序 Agent 的未提交修改。必须先理解差异再局部修改，禁止整文件覆盖、回滚用户改动或修改生成目录。

### 1.2 禁止的捷径

- 不得直接写抽屉关节位置伪造成功。
- 不得用底盘移动代替双臂抽拉。
- 不得把 `*p` 指示灯改造成按钮。
- 不得只移动机械臂到柜面，就宣称电缸已伸出或接触。
- 不得在只有一侧钩取成功时开始抽拉。
- 不得同时创建两个完全刚性的闭环约束而忽略 Gazebo 抖动。
- 不得通过放宽容差掩盖错误的工具点、方向或工作位姿。
- 几何门未通过前不得运行完整动作。
- 未经用户确认，不得修改可见外形、STL 或引入第三方依赖。

## 2. 当前工程事实与旧合同废止项

### 2.1 可复用能力

优先复用项目已有的：

- `CabinetControl.TYPE_DRAWER=5`
- `OperateCabinetControl.action`
- `SetCabinetBimanualGrasp.srv`
- `SetCabinetUnlock.srv`
- Gazebo 抽屉导轨、锁止和双手耦合基础逻辑
- C++ 双臂笛卡尔分段执行基础
- Web/任务层 drawer 基础映射
- `db1` 左右把手坐标和 `+X` 外拉方向
- 工具电缸 controller 和 joint state 反馈

### 2.2 当前工具职责映射

按现有 Xacro/YAML 得到以下映射，仍须用 Gazebo 单关节运动复核：

| 侧别 | 职责 | 当前关节 | 当前接触链接 |
|---|---|---|---|
| 左手 | 侧缝支撑 | `l_two_cyl_finger1_joint` | `l_two_cyl_finger1` |
| 左手 | 把手钩取 | `l_two_cyl_finger2_joint` | `l_two_cyl_finger2` |
| 右手 | 把手钩取 | `r_three_cyl_finger1_joint` | `r_three_cyl_finger1` |
| 右手 | 侧缝支撑 | `r_three_cyl_finger2_joint` | `r_three_cyl_finger2` |
| 右手 | 把手按钮解锁 | `r_three_cyl_finger3_joint` | `r_three_cyl_finger3` |

`r_three_cyl_finger3_joint` 串联在右支撑 `finger2` 下。计算解锁触点世界坐标时，必须计入右支撑电缸的实时伸出量，不能把第三根杆当成直接安装在工具基座上的独立杆。

### 2.3 必须废止的旧内容

- “先解锁、再支撑、最后抓取”的顺序。
- P0 中“支撑点在把手下方约 `0.05 m`”的定义。
- 当前 `db1` 支撑点使用 `z=0.884` 的配置。
- 为解锁、支撑、抓取分别规划三个机械臂位姿。
- 使用通用 `tool_tip_position` 同时代表钩爪、支撑杆和解锁杆。

P0 中抽屉轴、把手坐标、行程和指示灯语义仍有效；支撑点和解锁按钮合同必须按本文重新标定。

## 3. P1：冻结专用末端几何

新建 `docx/P1-抽拉柜专用末端几何与接触点标定.md`，逐个记录五个执行电缸的：

- 关节名、父子链接、关节轴和有效行程。
- 收回位、工作位和安全上限。
- 杆端接触点在移动链接局部系及工具基座零位系中的坐标。
- 钩取、侧缝支撑或解锁的唯一职责。

右侧解锁杆必须记录串联关系：

```text
right_tool_base
  -> r_three_cyl_finger2_joint（侧缝支撑位移）
  -> r_three_cyl_finger3_joint（解锁位移）
  -> unlock_contact_point
```

辨识时双臂置于远离柜体的安全位，每次只动一个电缸。长行程首次只动 `0.01 m`，短解锁电缸首次只动 `0.002 m`；同步记录 `/xczs/joint_states`、TF 和 Gazebo 正/侧视证据。

P1 验收门：

- [ ] 五个职责均有唯一关节和接触链接。
- [ ] 左右钩爪、左右支撑杆和右解锁杆可独立控制。
- [ ] 右解锁杆坐标包含父级支撑杆实时位移。
- [ ] 每个工作位保留机械行程余量。
- [ ] 工具自身无碰撞和持续弹跳。
- [ ] 理论杆端点与 TF 实测误差不超过 `5 mm`。

P1 未通过，不得进入柜前自动动作。

## 4. P2：重建 `db1` 工作位姿合同

### 4.1 一次到位

双臂只规划一次柜前工作位姿：

- 左臂让左钩爪真实钩点对准左把手可钩区域。
- 右臂让右钩爪真实钩点对准右把手可钩区域。
- 到位时其余电缸处于不会提前碰撞的准备位置。
- 后续支撑和解锁只通过电缸伸出完成，不再移动整臂。

不得用工具基座中心或最深杆端点直接对准把手。左右目标位姿分别满足：

```text
T_world_tool = T_world_target_hook * inverse(T_tool_real_hook_contact)
```

### 4.2 已知几何

```yaml
drawer_axis: [1.0, 0.0, 0.0]
left_handle_point:  [0.099, 4.107, 0.952]
right_handle_point: [0.099, 4.693, 0.952]
state_positions: [0.0, 0.30]
```

这些是初始钩取参考中心。必须在 Gazebo 确认钩爪应落在中心、内缘还是背面钩持面，并把最终偏移写入 YAML，不能靠扩大抓取容差处理。

### 4.3 左右侧缝

- 左支撑杆在左把手外侧，伸入左侧柜体缝隙。
- 右支撑杆在右把手外侧，伸入右侧柜体缝隙。
- 工具上的“支撑杆—钩爪”横向间距必须匹配“把手钩点—侧缝支撑点”横向间距。
- 支撑杆沿自身伸缩轴进入侧缝，不允许靠整臂斜压。

基于当前 `db1` 面板边界和约 `68 mm` 的杆列间距，以下只作为碰撞预检候选值：

```yaml
left_side_gap_point_candidate:  [0.099, 4.039, 0.952]
right_side_gap_point_candidate: [0.099, 4.761, 0.952]
```

候选值不能直接视为最终标定。必须通过模型边界、TF 和 Gazebo 低速接触确认侧缝中心、进入深度和接触法向，再写入 `left_support_point/right_support_point`。

### 4.4 右把手解锁按钮

解锁按钮位于右把手本体上，合同至少包含：

```yaml
unlock_button_point: [x, y, z]
unlock_button_normal: [nx, ny, nz]
unlock_motor_joint: r_three_cyl_finger3_joint
unlock_contact_link: r_three_cyl_finger3
unlock_retracted_position: <P1_VALUE>
unlock_pressed_position: <P1_VALUE>
unlock_extension_floor: <P1_VALUE>
unlock_extension_ceiling: <P1_VALUE>
```

按钮目标点必须由“右钩爪已钩住右把手、右支撑已伸入右侧缝”这一最终工具构型反推。禁止让右臂先单独按按钮再返回把手。

若按钮已烘焙在把手 STL 中而没有独立关节，可在真实按钮位置增加不可见接触区或传感器；不得改变可见按钮外形。

### 4.5 一姿态三接触几何门

不执行抽拉，只验证同一工作位姿能同时满足：

1. 左钩爪接触左把手。
2. 右钩爪接触右把手。
3. 左支撑杆伸入左侧缝并稳定。
4. 右支撑杆伸入右侧缝并稳定。
5. 右解锁杆在支撑保持时按到右把手按钮。

任一项需要再次移动整臂，说明工具滚转角、钩点或接触点合同错误，必须返回 P1/P2。

## 5. P3：消除抖动与位姿偏差

当前已观察到机械臂持续剧烈抖动和工作位姿偏差，正式动作前必须处理。

按以下顺序排查：

1. 通用 `tool_tip_position` 是否被错当成真实钩点/支撑点/解锁点。
2. 左右工具滚转角是否使支撑杆位于把手外侧的侧缝方向。
3. 右解锁正向运动学是否遗漏父级 `finger2` 位移。
4. 目标是否落入碰撞体内部，或支撑杆是否伸进实体柜板。
5. 左右轨迹是否顺序发送或没有同时启动。
6. Gazebo 是否建立两个完全刚性的闭环约束。
7. controller PID、速度和加速度；几何正确后才允许调参。

双臂闭环建议：

- 一侧作为主运动耦合，传递抽屉位移。
- 另一侧采用柔顺跟随并持续监测接触和偏载。
- 两侧都必须真实钩住；柔顺不代表允许悬空。
- 支撑杆使用接触支撑，不再额外创建与世界完全刚性的固定关节。
- 任一接触丢失超过宽限时间，立即停止双臂、解除耦合并安全恢复。

首次只做低速、低加速度和 `0.03 m` 短行程。先证明静止工作位姿稳定 `3 s`，再伸电缸；电缸到位后稳定 `1 s` 且无高频振荡，才允许 attach。

稳定性验收：

- 工作位姿位置误差不超过 `8 mm`，姿态误差不超过 `0.02 rad`。
- 静止窗口内末端漂移不超过 `3 mm`。
- 电缸接触后无肉眼可见持续抖动。
- 关节速度回落到稳定阈值，不连续正负翻转。
- Gazebo 和 controller 无持续错误。

## 6. P4：C++ 状态机重排

主要修改 `xczs_inspection_robot_control/src/cabinet_button_operator.cpp`。

### 6.1 打开状态机

| 顺序 | 状态 | 执行器 | 成功证据 | 失败恢复 |
|---:|---|---|---|---|
| 1 | `PRECHECK` | 无 | 场景、工具 A、租约、关位有效 | 不移动，失败 |
| 2 | `DOCKING` | 底盘 | 停靠误差通过并保持 | 停车，失败 |
| 3 | `MOVE_TO_WORK_POSE` | 左右臂 | 两个真实钩点到达预钩位 | 回安全位 |
| 4 | `HOOK_HANDLES` | 左右钩爪 | 两侧把手接触均成立 | 松钩撤离 |
| 5 | `EXTEND_SUPPORTS` | 左右支撑杆 | 两侧缝接触均稳定 | 收支撑、松钩 |
| 6 | `PRESS_UNLOCK` | 右解锁杆 | 右把手按钮接触、伸出量和解锁均成立 | 全部收回 |
| 7 | `BIMANUAL_ATTACH` | Gazebo 服务 | 主约束和从侧监控成立 | detach 清理 |
| 8 | `PULL_BREAKAWAY` | 左右臂 | 保持接触，同步拉出首段 | 同时制动 |
| 9 | `PULL_TO_OPEN` | 左右臂 | 沿 `+X` 到开位 | 制动恢复 |
| 10 | `VERIFY_OPEN` | 状态反馈 | 位置、速度、同步、接触稳定 | 保持安全状态 |
| 11 | `RELEASE` | 服务和电缸 | 按安全顺序释放 | 强制清理 |
| 12 | `RETREAT` | 左右臂 | 回运输位 | 报告恢复失败 |

默认在完整抽拉期间保持解锁杆按压。只有实测证明抽屉离开关位后锁止持续解除，才允许在 `PULL_BREAKAWAY` 后提前收回。

### 6.2 关闭状态机

```text
PRECHECK -> DOCKING -> MOVE_TO_WORK_POSE -> HOOK_HANDLES
  -> EXTEND_SUPPORTS -> BIMANUAL_ATTACH -> PUSH_TO_CLOSED
  -> VERIFY_CLOSED_AND_LATCHED -> RELEASE -> RETREAT
```

### 6.3 电缸阶段保持合同

| 阶段 | 钩爪 | 支撑杆 | 解锁杆 |
|---|---|---|---|
| `home` | 收回 | 收回 | 收回 |
| `hook` | 工作位 | 收回 | 收回 |
| `support` | 保持工作位 | 工作位 | 收回 |
| `unlock` | 保持工作位 | 保持工作位 | 工作位 |
| `pull/push` | 保持工作位 | 保持工作位 | 打开默认保持；关闭收回 |

进入下一阶段时不得把上一阶段电缸隐式归零。

### 6.4 删除旧动作

- 删除 drawer 分支中的独立 `unlock_pose` 整臂移动。
- 删除解锁后退回 `ready` 再规划的动作。
- 删除独立 `support_pose` 整臂移动。
- 抓取后不再计算另一套工作姿态。
- 只保留一次预钩/钩取工作位姿和随抽屉平移的同步轨迹。

### 6.5 阶段反馈

Web/SSE 至少显示：停靠、到达工作位姿、双钩取、双侧缝支撑、右把手解锁、建立耦合、同步拉出/推回、位置验证、释放撤离，以及失败侧别和物理原因。

## 7. P5：Gazebo 证据与安全门

### 7.1 钩取

- 使用实际钩爪链接和实际钩点。
- 钩点距对应把手钩持面小于标定阈值。
- 钩爪关节达到工作位置。
- 只有一侧成立时，`attach=true` 必须被拒绝。

### 7.2 侧缝支撑

- 支撑关节达到工作伸出量。
- 支撑杆端进入对应侧缝接触区域。
- 接触方向与侧缝法向相符。
- 接触持续一个稳定窗口，无穿透和弹跳。

若 `dianqiground` 只有地板碰撞、柜体外观没有真实碰撞体，应在确认侧缝几何后增加不可见、局部、最小范围的碰撞代理。不得改变可见外观，也不得用大盒子封住整个柜面。

### 7.3 解锁

`SetCabinetUnlock` 成功必须同时满足：

- 操作租约、活动控件和场景代次有效。
- 右钩爪仍钩住右把手。
- 两侧支撑仍成立。
- `r_three_cyl_finger3_joint` 达到最小伸出量且未过冲。
- `r_three_cyl_finger3` 真实触点进入右把手按钮接触区。
- 按钮保持时间达到阈值。

仅请求中的 `pressed=true` 不构成物理证据。

### 7.4 持续监控

抽拉/推回期间持续监控：左右钩点距离、左右侧缝支撑距离、打开时的解锁触点与行程、双臂同步误差、抽屉非导轨方向偏移、租约/心跳/取消/场景代次。任一关键证据丢失，必须同时停止两臂并释放耦合。

## 8. `db1` 调试顺序

每项通过后才进入下一项：

1. 静态检查工具职责、坐标和参数范围。
2. 远离柜体逐个移动五个电缸。
3. 双臂只到预钩工作位姿，不伸电缸。
4. 双钩爪低速钩住把手并保持 `3 s`。
5. 保持钩取，双支撑低速伸入侧缝并保持 `3 s`。
6. 保持钩取和支撑，右解锁杆低速按按钮，不解锁导轨。
7. 接入真实解锁服务，验证锁止解除。
8. 建立双手耦合但不移动，保持 `3 s`。
9. 同步抽拉 `0.03 m`。
10. 同步抽拉 `0.10 m`。
11. 同步抽拉到 `0.30 m`。
12. 从 `0.30 m` 同步推回 `0.0 m`。
13. 接入完整 Action。
14. 通过 `./run_all.sh --web` 从 Web 执行。

每阶段记录：左右末端目标/实测位姿，五个电缸目标/实测位置，五个触点世界坐标和距离，双臂同步误差，抽屉位置/速度/非轴向偏移，碰撞/穿透/抖动/controller 状态，以及 Action 物理结果。

出现位姿超差、钩取错误、支撑未进入侧缝、解锁未按到按钮、持续抖动、关节速度翻转或约束发散时，立即停止，不得继续完整抽拉。

## 9. 推广到其余抽屉

`db1` 冷启动稳定后，依次推广 `dm1`、`ds2`、`ds3`、`ds1`。每个抽屉只通过 YAML 提供把手、侧缝、按钮、工位和行程差异，不复制 C++ 状态机。

`ds1` 把手高度约 `2.003 m`，必须单独验证完整双臂路径。若底盘固定且不改变外观时不可达，应提交 IK/碰撞证据并询问用户，不能用底盘拖拽代替。

只有某个抽屉通过自身冷启动开关验收，才可加入 `operable_control_ids`。

## 10. 全场景配色

抽拉动作稳定后，为 `cabinet_operation`、`electrical_mezzanine`、`generator_plant` 配色：

- 环境主体用低饱和蓝灰或暖灰。
- 柜体面板用较浅颜色，与背景形成明度差。
- 把手和机械机构用深灰金属色。
- 指示灯用红、绿、琥珀等高辨识色，但 UI 仍为只读指示灯。
- 安全警示构件用黄或橙，避免全场景高饱和。

只修改 `visual/material/ambient/diffuse` 等视觉字段，不得修改 STL、可见形状、link/joint pose、collision、inertial、controller 或插件物理参数。三个场景分别启动截图验收，并用结构化差异检查证明物理字段未变。

## 11. 预计修改文件

按实际需要局部修改：

- `xczs_inspection_robot_control/src/cabinet_button_operator.cpp`
- `xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_controls.yaml`
- `xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_adapter.yaml`
- `xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro`
- `xczs_inspection_robot_gazebo/src/cabinet_interaction_plugin.cpp`
- `xczs_inspection_robot_interfaces/srv/SetCabinetUnlock.srv`
- `xczs_inspection_robot_interfaces/srv/SetCabinetBimanualGrasp.srv`
- `jiang/` 下 drawer 的反馈和 Web 映射文件
- 三个场景对应的 material/Xacro 文件
- `scripts/validate/` 下相关校验器和回归脚本
- `docx/P0-电气夹层抽拉柜几何合同.md`
- `docx/P1-抽拉柜专用末端几何与接触点标定.md`

不修改 `build/`、`install/`、`log/`、`docs/build/`。

## 12. 检查、构建与运行

### 12.1 静态检查

```bash
git diff --check
scripts/validate/check_adapter_contract
scripts/validate/check_cabinet_model
scripts/validate/check_scene_config
```

必须增加/更新断言：

- 阶段顺序为 hook → support → unlock → pull。
- 支撑点位于左右侧缝，不在把手下方。
- 右解锁触点计算包含父级支撑杆位移。
- `*p` 不在可操作目录且不产出解锁证据。
- 无双钩取、双支撑或真实按钮接触时禁止抽拉。
- 正式抽屉路径中 `base_free=false`。
- 取消、重置、租约失效和场景切换释放全部约束。

### 12.2 构建与测试

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  xczs_inspection_robot_interfaces \
  xczs_inspection_robot_description \
  xczs_inspection_robot_moveit_config \
  xczs_inspection_robot_gazebo \
  xczs_inspection_robot_control \
  xczs_inspection_robot_bringup

colcon test --packages-select \
  xczs_inspection_robot_interfaces \
  xczs_inspection_robot_description \
  xczs_inspection_robot_moveit_config \
  xczs_inspection_robot_gazebo \
  xczs_inspection_robot_control \
  xczs_inspection_robot_bringup

colcon test-result --verbose
pytest -q jiang/tests/
XCZS_PREFLIGHT_ONLY=true ./run_all.sh --web
colcon build --symlink-install
```

静态门、构建和短行程物理门全部通过后，才运行：

```bash
./run_all.sh --web
```

不得在机械臂仍会剧烈抖动或工作位姿未通过 P2/P3 时运行完整开关。

## 13. 最终验收矩阵

`db1` 至少完成 3 次全新 Gazebo 启动，每次连续执行：

```text
closed -> open -> closed -> open -> closed -> open -> closed
```

每轮必须满足：

- [ ] 双臂只到达一次工作位姿。
- [ ] 左右钩爪先钩住把手。
- [ ] 左右支撑杆随后进入侧缝。
- [ ] 右解锁杆最后按下右把手按钮。
- [ ] 三类接触建立后才开始抽拉。
- [ ] 抽拉和推回期间底盘固定。
- [ ] 无持续抖动、穿模或单侧拖拽。
- [ ] 开位稳定，关位稳定并重新锁止。
- [ ] 失败时双臂同时停止且所有约束可清理。
- [ ] 不依赖人工碰抽屉或手工修正姿态。

全链路还必须验证 Gazebo 单机构/短行程、ROS 2 Action、任务反馈与取消、Web 打开关闭、错误工具、租约失效、场景切换，以及 `physical_outcome_confirmed=true` 只在物理证据齐全时出现。

## 14. Agent 汇报模板

```markdown
### 阶段：P<number> <名称>

完成内容：
- ...

修改文件：
- `path/to/file`

验证命令与结果：
- `<command>`：PASS/FAIL，关键输出 ...

物理证据：
- 左/右钩爪位置与接触距离：... / ...
- 左/右支撑位置与侧缝接触距离：... / ...
- 右解锁电缸位置与按钮接触距离：...
- 左/右末端位姿误差：... / ...
- 双臂同步误差：...
- 抽屉关节位置和速度：...

仍未解决：
- ...

下一阶段及进入条件：
- ...
```

## 15. 最终交付清单

- [ ] 更新 P0：支撑点明确为左右侧缝。
- [ ] P1：五个专用接触点和电缸标定记录。
- [ ] 一次到达工作位姿的双臂规划。
- [ ] hook → support → unlock → pull 状态机。
- [ ] 右把手本体按钮的真实解锁证据。
- [ ] 双钩取、双支撑和同步抽拉持续监控。
- [ ] 抖动根因修复和稳定性数据。
- [ ] `db1` 三次冷启动、每次三轮开关证据。
- [ ] `dm1/ds2/ds3/ds1` 分阶段推广结果。
- [ ] Web 到 Gazebo 全链路及异常回收验收。
- [ ] 三个场景配色及物理字段未变证明。
- [ ] 静态检查、构建、测试和 `./run_all.sh --web` 回归结果。

未完成物理和全链路验收前，只能报告“已实现到某阶段”，不得报告“抽拉柜功能已经完成”。
