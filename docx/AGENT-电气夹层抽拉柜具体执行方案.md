# AGENT 执行手册：电气夹层抽拉柜双臂操作与全场景配色

> 用途：供后续编码 Agent 直接按阶段执行。  
> 项目：`/home/live/work01`  
> 目标场景：`electrical_mezzanine`  
> 日期：2026-09-03  
> 几何真值：[`P0-电气夹层抽拉柜几何合同.md`](P0-电气夹层抽拉柜几何合同.md)

## 0. Agent 总指令

你要完成的不是“让抽屉状态变成 open”，而是让机器人在 Gazebo 中按真实机械顺序完成操作，并由 Web 到 Gazebo 全链路证明结果：

1. 机械臂旋转到适合作业的角度。
2. 右臂上的解锁电机伸出，把右把手上的解锁结构推进到解锁位置。
3. 解锁电机退回。
4. 左右机械臂各自的支撑伸缩电机伸出，真实抵住把手附近的柜体表面。
5. 两只机械臂的夹爪分别抓住左、右把手。
6. 保持双侧支撑和双侧抓取，两条机械臂同步沿 `+X` 拉出抽屉。
7. 关闭时重新建立双侧支撑和抓取，然后两条机械臂同步沿 `-X` 推回抽屉。
8. 抽屉关闭并锁止后，松开夹爪、收回支撑电机、机械臂回安全位。

### 0.1 不可变更的事实

- 场景是 `electrical_mezzanine`，不是 `generator_plant`。
- 当前可动抽屉只有 `ds1`、`ds2`、`db1`、`ds3`、`dm1` 五组。
- 外拉方向已经实测为 `+X`；关闭方向为 `-X`。
- 关位是 `0.0 m`，初始目标开位是 `0.30 m`，关节上限是 `0.38 m`。
- 柜面上 `*p` 小圆点是指示灯，不能被改成解锁按钮或普通可操作按钮。
- 解锁按钮/推进区位于右把手处，与 `*p` 指示灯是两个不同语义对象。
- 每条机械臂上用于支撑的伸缩电机必须真实伸出并抵住柜体。
- 夹爪负责抓住把手；支撑电机和夹爪是不同职责，不能用同一个“末端点靠近”事件同时冒充。
- 正式验收时，抽拉动力来自两条机械臂的同步运动。底盘只负责停靠并保持，不得以底盘平移替代双臂抽拉。

### 0.2 禁止的实现捷径

- 禁止直接写抽屉关节位置来伪造成功。
- 禁止把 `b1p` 或其他 `*p` 指示灯改为可压按钮。
- 禁止只移动整条机械臂到柜面，就宣称“支撑电机已伸出”。
- 禁止只建立 Gazebo 隐形抓取约束，就宣称“夹爪已抓住把手”。
- 禁止单臂拉动后让另一条手臂补动作。
- 禁止使用底盘 `base_free`/grab-and-drive 作为正式抽拉路径。
- 禁止先把 5 个抽屉全部加入 `operable_control_ids`，再补物理验收。
- 禁止修改 `build/`、`install/`、`log/`、`docs/build/` 中的生成文件。
- 禁止覆盖当前工作区已有改动或执行 `git reset --hard`、`git checkout --`。
- 禁止在未获用户同意时修改可见模型外形或引入第三方依赖。

## 1. 开始工作前的仓库协议

### 1.1 必读文件

按顺序完整阅读：

1. `AGENTS.md`
2. `CLAUDE.md`
3. `docx/P0-电气夹层抽拉柜几何合同.md`
4. `docx/电气夹层抽拉柜双手操作与全场景配色实施执行方案.md`
5. 本文件

若总方案与 P0 几何合同冲突，以 P0 为准。例如总方案中的旧 `-X` 推断已失效，真实外拉方向是 `+X`。

### 1.2 保护当前工作区

执行：

```bash
cd /home/live/work01
git status --short
git diff --check
git diff -- \
  xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_controls.yaml \
  xczs_inspection_robot_control/src/cabinet_button_operator.cpp \
  xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro
```

2026-09-03 检查时存在 3 个未提交修改：

- `xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_controls.yaml`
- `xczs_inspection_robot_control/src/cabinet_button_operator.cpp`
- `xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro`

这些改动尝试解决 `db1` 解锁帽过压和读回不稳定。Agent 必须先理解差异，随后做局部修正；不能整文件覆盖。

### 1.3 当前实现盘点

以下能力已经存在，应复用，不要重复造接口：

- `CabinetControl.TYPE_DRAWER=5`
- `SetCabinetBimanualGrasp.srv`
- `SetCabinetUnlock.srv`
- Gazebo `ControlKind::kDrawer`
- 双侧抓取距离检查与耦合中断
- C++ 中的双臂 ready/support/grasp/drag 基础流程
- Web/任务层的 `TYPE_DRAWER` 基础映射
- `db1` 的 P0 把手和支撑坐标

当前尚不能视为完成：

- `db1` 把 `b1p` 指示灯当成物理解锁按钮，语义错误。
- 当前 support 主要是把整条机械臂移动到 support pose，没有证明支撑伸缩电机实际伸出。
- 当前抓取点以工具基座业务点为主，没有证明夹爪电机执行了抓把手动作。
- 当前存在 `base_free`/grab-and-drive 路径，不能作为用户要求的双臂抽拉验收。
- 只有 `db1` 是完整 `drawer` 合同，其余 4 组仍保留 `slider` 语义。
- `ds1` 尚未解决高位双臂可达性。
- 三个场景虽然已有部分颜色，但没有完成统一视觉验收。

## 2. 先冻结末端电机职责

### 2.1 为什么必须先做这一步

工具套装 A 的右端是三电缸，左端是两电缸：

- 右端关节：
  - `r_three_cyl_finger1_joint`
  - `r_three_cyl_finger2_joint`
  - `r_three_cyl_finger3_joint`
- 左端关节：
  - `l_two_cyl_finger1_joint`
  - `l_two_cyl_finger2_joint`

其中右侧 `finger3` 是串联在 `finger2` 下的短行程级，行程为 `0.0254 m`；其余长行程电缸上限为 `0.12 m`。

从结构上看，右侧短级很可能适合作为解锁推进电机，但 Agent 不得只凭名称认定。必须在 Gazebo 中逐个单独运动，确认哪个实体是：

- 右侧解锁推进电机
- 右侧柜体支撑电机
- 右侧夹爪
- 左侧柜体支撑电机
- 左侧夹爪

### 2.2 单关节辨识步骤

1. 使用工具套装 A 启动机器人，不执行抽屉任务。
2. 将左右臂移动到远离柜体、不会碰撞的观察姿态。
3. 每次只给一个末端电机关节发送小行程命令。
4. 记录：
   - 关节名称
   - 对应运动链接
   - 运动方向
   - 有效行程
   - 末端接触面的局部坐标
   - 是否适合解锁、支撑或夹取
5. 对每个关节保存正视和侧视证据。
6. 将结论写入 `docx/P1-末端电机职责与接触点标定.md`。

建议首次探测行程：

- 长行程关节先移动 `0.01 m`，确认方向后再逐步增加。
- 短行程关节先移动 `0.002 m`，确认方向后再逐步增加。
- 不得第一次命令就打满行程。

### 2.3 P1 输出表

Agent 必须填写而不是跳过：

| 职责 | 最终关节 | 接触链接 | 局部接触点 | 工作位置 | 收回位置 | 证据 |
|---|---|---|---|---:|---:|---|
| 右侧解锁推进 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 截图/关节状态 |
| 右侧柜体支撑 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 截图/关节状态 |
| 右侧夹爪 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 截图/关节状态 |
| 左侧柜体支撑 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 截图/关节状态 |
| 左侧夹爪 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 截图/关节状态 |

### 2.4 P1 验收门

- [ ] 每个职责都有唯一关节和接触链接。
- [ ] 支撑电机与夹爪职责没有混淆。
- [ ] 右侧解锁电机能够独立运动。
- [ ] 每个工作位置都小于物理上限，并保留余量。
- [ ] 工具收回后不与自身或柜体碰撞。
- [ ] 未修改工具可见外形。

P1 未通过前，不得继续编写抽屉正式动作序列。

## 3. 修正解锁语义

### 3.1 目标合同

解锁必须同时满足：

1. 右臂已旋转到正确作业角度。
2. 已标定的右侧解锁电机实际伸出。
3. 解锁接触链接进入右把手解锁区。
4. 接触持续时间、伸出量和位置均满足阈值。
5. `*p` 指示灯没有发生关节运动，也没有进入可操作目录。

### 3.2 模型处理原则

P0 已确认右把手解锁推进区是逻辑区，并非 `*p` 指示灯。优先采用以下不改变可见外形的方案：

- 在右把手处配置不可见的解锁接触区域。
- 由 Gazebo 插件读取真实解锁电机接触链接的世界位置。
- 结合解锁电机关节实际伸出量判断推进是否完成。
- 解锁成功后释放抽屉轨道锁止。
- 抽屉关闭并稳定后自动重新锁止。

不得继续使用 `b1p_joint` 位移作为解锁物证。

### 3.3 对当前未提交改动的处理

逐项检查并局部修正：

#### `electrical_mezzanine.xacro`

- 删除或停用把 `b1p` 作为 `db1_unlock_button` 的物理按钮控制。
- `b1p` 恢复为固定视觉指示灯。
- 保留真正需要的抽屉阻尼、导轨和双手耦合改进。
- 解锁区只能有 collision/sensor 语义，不能新增可见按钮。

#### `electrical_mezzanine_controls.yaml`

- 删除 `unlock_button_joint_state_topic`、`unlock_button_joint_name` 和基于指示帽的阈值。
- 增加经过 P1 标定的解锁电机关节、接触链接、接触点、伸出位置和收回位置。
- 保留 `unlock_press_point`，但其含义改为右把手解锁区中心。

#### `cabinet_button_operator.cpp`

- 删除或重构 `bounded_drawer_unlock_press()` 中对 `b1p_joint` 的依赖。
- 有界小步推进仍可保留，但读回应来自右侧解锁电机自身关节和接触证据。
- 达不到阈值时必须失败，不能调用服务强制解锁。

#### `SetCabinetUnlock.srv` 与 Gazebo 插件

- 尽量复用现有服务，不新增重复服务。
- `pressed` 字段改为“真实解锁推进成立”，其证据来自解锁电机伸出和接触。
- 插件必须验证当前操作租约、活动控件、右侧接触链接、接触距离和关节伸出量。

### 3.4 解锁故障测试

必须验证以下请求均失败（验证顺序按“park 可证 → 现场可证”分批，见 3.4.1）：

1. 解锁电机未伸出，仅机械臂靠近（现场，§7.2 阶段 2/3 覆测）。
2. 解锁电机伸出，但位置不在解锁区（park 已证，见下）。
3. 右工具进入解锁区，但没有有效操作租约（park 空/外来租约已证；解锁区内无租约
   变体随阶段 3 覆测）。
4. 右工具按压 `*p` 指示灯（`b1p` 为固定灯，无按钮位移，现场覆测）。
5. 解锁电机过冲或达到安全上限（须工具在解锁区先过距离门，现场覆测）。
6. 解锁过程中场景切换或任务取消（任务取消现场覆测；场景切换属 §10 场景级回归）。

#### 3.4.1 验收记录（2026-09-03，park 可证项全部通过）

驱动：`scripts/tools/s34_unlock_fault_test.py`（活栈 DOMAIN_ID=42，目标 db1，
真实协调器租约 ACQUIRE/RENEW/RELEASE + transient_local active_control + 0.4 s
心跳 + 3 关节 FJT 驱动 `r_three_cyl_finger3_joint`）。逐项 fail-loud 判定：
插件必须返回明确拒绝消息、`drawer_unlocked=false`、抽屉滑轨实测纹丝不动。

| 项 | 构造 | 结果 |
|---|---|---|
| 空操作租约 | `operation_lease_id=""`，无会话 | FAIL：`Drawer unlock requires a non-empty operation lease identity.` |
| 外来租约 | `operation_lease_id="lease-not-mine"`，无会话/心跳 | FAIL：`requires a fresh heartbeat from the exact active control and global operation lease session.` |
| 电机伸出但不在解锁区 | 真实租约+会话，FJT 伸 `finger3`→0.008（实测 0.0077 m），工具距解锁区中心 4.549 m | FAIL：`Unlock contact link is not inside the unlock zone (distance 4.549 m > 0.03 m)`；`pressed=true, right_tool_contact=false`；滑轨 0.0008→0.0008 m 不动 |

结论：距离门先于电机门触发，与设计一致；现场项（1/4/5 及解锁区内无租约变体）须
工具先进入解锁区，已并入 §7.2 阶段 2/3 现场按序补测，并复用本驱动骨架。

## 4. 实现真实支撑动作

### 4.1 支撑不是机械臂末端位姿

用户要求的是：机械臂上的可伸长电机向前伸出，抵住把手附近的柜体，从而为后续夹爪抽拉提供支撑力。

因此，正式流程必须包含清晰的两级动作：

1. 机械臂本体先旋转并移动到支撑预备姿态。
2. 机械臂保持姿态，支撑伸缩电机再独立伸出并接触柜体。

现有 `left_support_pose` / `right_support_pose` 只能用于把工具本体带到预备位置，不能作为“支撑已建立”的证据。

### 4.2 支撑配置

在 `electrical_mezzanine_adapter.yaml` 的 `drawer_tools.left/right` 中增加经过 P1 实测的职责字段，示例：

```yaml
drawer_tools:
  left:
    move_group: left_arm
    support_joint: <P1_LEFT_SUPPORT_JOINT>
    support_contact_link: <P1_LEFT_SUPPORT_LINK>
    support_contact_point: [x, y, z]
    support_retracted_position: 0.0
    support_contact_position: <P1_VALUE>
    gripper_joint: <P1_LEFT_GRIPPER_JOINT>
    gripper_open_position: <P1_VALUE>
    gripper_grasp_position: <P1_VALUE>
  right:
    move_group: right_arm
    unlock_joint: <P1_RIGHT_UNLOCK_JOINT>
    unlock_contact_link: <P1_RIGHT_UNLOCK_LINK>
    unlock_retracted_position: 0.0
    unlock_pressed_position: <P1_VALUE>
    support_joint: <P1_RIGHT_SUPPORT_JOINT>
    support_contact_link: <P1_RIGHT_SUPPORT_LINK>
    support_contact_point: [x, y, z]
    support_retracted_position: 0.0
    support_contact_position: <P1_VALUE>
    gripper_joint: <P1_RIGHT_GRIPPER_JOINT>
    gripper_open_position: <P1_VALUE>
    gripper_grasp_position: <P1_VALUE>
```

字段名称可按现有配置风格调整，但职责必须明确，不能只保留一个笼统的 `calibration_joint_positions`。

### 4.3 支撑建立判据

左右两侧各自必须同时满足：

- 支撑关节已经到达工作位置。
- 支撑接触链接位于对应支撑点容差内。
- 接触持续至少一个稳定窗口。
- 关节速度已经稳定。
- 支撑接触没有发生穿透或连续弹跳。

建议先沿用项目已有 `stable_state_duration=0.30 s`，距离阈值从 `0.02 m` 起步，再根据实测收紧。

### 4.4 支撑保持

- 从夹爪抓取开始，直到抽屉到位前，支撑电机必须保持工作位置。
- 任一侧支撑丢失时立即停止双臂轨迹并解除抽屉耦合。
- 关闭完成后先松夹爪，再收回支撑电机。
- 不允许支撑电机在抽拉途中提前收回。

## 5. 实现真实夹爪抓取

### 5.1 抓取顺序

1. 两臂到达左右把手预抓取姿态。
2. 左右支撑电机伸出并确认接触。
3. 两侧夹爪从 open 位置运动到 grasp 位置。
4. 插件分别验证左夹爪接触左把手、右夹爪接触右把手。
5. 两侧都通过后才能调用 `SetCabinetBimanualGrasp(attach=true)`。

### 5.2 服务合同修正

复用 `SetCabinetBimanualGrasp.srv`，但请求中的：

- `left_robot_link`
- `right_robot_link`
- `left_robot_grasp_point`
- `right_robot_grasp_point`

必须来自 P1 标定后的真实夹爪链接和夹取点，不能继续无条件使用工具基座上的抽象业务点。

### 5.3 抓取验收

- [ ] 夹爪电机关节发生了可观测位移。
- [ ] 左夹爪与左把手真实接触。
- [ ] 右夹爪与右把手真实接触。
- [ ] 只有一侧接触时，attach 被拒绝。
- [ ] 两侧抓取后轻微扰动不会立即脱离。
- [ ] 任一侧脱离超过宽限时间，耦合自动释放并上报故障。

## 6. C++ 正式动作状态机

在 `xczs_inspection_robot_control/src/cabinet_button_operator.cpp` 中将抽屉流程拆为明确阶段。每个阶段都必须有 Action feedback、进入条件、成功证据、超时和恢复动作。

### 6.1 打开流程

| 顺序 | 状态 | 主要执行器 | 成功证据 | 失败恢复 |
|---:|---|---|---|---|
| 1 | `PRECHECK` | 无 | 场景、套装 A、租约、状态正常 | 不移动，直接失败 |
| 2 | `DOCKING` | 底盘 | 到达抽屉工位 | 停车并失败 |
| 3 | `ROTATING_TO_WORK_ANGLE` | 左右机械臂 | 两臂到达作业角度 | 回 home |
| 4 | `UNLOCK_READY` | 右机械臂 | 解锁电机对准右把手解锁区 | 右臂撤离 |
| 5 | `UNLOCK_EXTENDING` | 右解锁电机 | 伸出量和接触均达标 | 收回解锁电机 |
| 6 | `UNLOCK_RETRACTING` | 右解锁电机 | 已收回且轨道保持解锁 | 重新锁止并撤离 |
| 7 | `SUPPORT_READY` | 左右机械臂 | 工具到达支撑预备位 | 双臂撤离 |
| 8 | `SUPPORT_EXTENDING` | 左右支撑电机 | 两侧真实接触并稳定 | 两侧支撑收回 |
| 9 | `GRIPPER_ALIGNING` | 左右机械臂 | 两夹爪对准两把手 | 保持支撑后撤离 |
| 10 | `GRIPPER_CLOSING` | 左右夹爪 | 两侧夹爪真实接触 | 松夹爪、收支撑 |
| 11 | `BIMANUAL_ATTACHING` | Gazebo 双手服务 | 两侧 attach 同时成功 | detach、松夹爪 |
| 12 | `PULLING` | 左右机械臂 | 同步沿 `+X` 到 `0.30 m` | 制动并 detach |
| 13 | `OPEN_VERIFYING` | 状态反馈 | 位置、速度、接触均稳定 | 保持安全状态并失败 |
| 14 | `RELEASING` | 夹爪、支撑电机 | detach、松爪、支撑收回 | 强制安全清理 |
| 15 | `RETREATING` | 左右机械臂 | 回到安全位 | 报告恢复失败 |

### 6.2 关闭流程

关闭不执行解锁步骤：

```text
PRECHECK
  -> DOCKING
  -> ROTATING_TO_WORK_ANGLE
  -> SUPPORT_READY
  -> SUPPORT_EXTENDING
  -> GRIPPER_ALIGNING
  -> GRIPPER_CLOSING
  -> BIMANUAL_ATTACHING
  -> PUSHING(-X)
  -> CLOSED_VERIFYING
  -> RELATCH_VERIFYING
  -> RELEASING
  -> RETREATING
```

### 6.3 双臂同步要求

- 左右臂轨迹必须使用同一开始屏障和同一时间轴。
- 不允许先执行左臂轨迹再执行右臂轨迹。
- 两侧笛卡尔位移目标必须对应同一个抽屉轨道位置。
- 当前 `drawer_sync_tolerance=0.05 m` 只可作为初始保护上限；在 `db1` 稳定后逐步收紧。
- 同步误差越限时停止两个控制器，并调用双手 detach。
- 正式路径中 `base_free` 必须为 `false`。
- 底盘在抽拉阶段必须保持停靠位置，不得沿轨道方向带动抽屉。

## 7. `db1` 样板实施顺序

只在 `db1` 上完成全链路后，才能推广到其余抽屉。

### 7.1 `db1` 配置

以 P0 坐标为准：

```yaml
drawer_axis: [1.0, 0.0, 0.0]
left_handle_point:  [0.099, 4.107, 0.952]
right_handle_point: [0.099, 4.693, 0.952]
left_support_point:  [0.099, 4.107, 0.902]
right_support_point: [0.099, 4.693, 0.902]
state_positions: [0.0, 0.30]
```

右把手解锁区的最终中心必须在 P1 中复核。不得直接复用 `b1p` 指示灯中心。

### 7.2 `db1` 单阶段调试顺序

每完成一项才进入下一项：

1. 只测试两臂旋转到作业角度。
2. 只测试右侧解锁电机伸出/收回，不解锁轨道。
3. 测试右侧解锁接触后释放轨道锁止。
4. 只测试左右支撑电机伸出/收回。
5. 只测试左右夹爪打开/抓取/松开。
6. 测试双侧 attach/detach，不移动抽屉。
7. 抽拉 `0.03 m`。
8. 抽拉 `0.10 m`。
9. 抽拉到 `0.30 m`。
10. 从 `0.30 m` 推回 `0.0 m`。
11. 接入完整 Action。
12. 接入 Web 任务。

### 7.3 `db1` 冷启动验收

必须至少完成 3 次全新 Gazebo 启动，每次连续执行：

```text
closed -> open -> closed -> open -> closed -> open -> closed
```

每次记录：

- 右解锁电机伸出量
- 左右支撑电机伸出量
- 左右夹爪位置
- 左右接触距离
- 双臂同步误差
- 抽屉关节峰值和稳定值
- 是否发生穿模、抖动或控制器超时
- Action 结果和 `physical_outcome_confirmed`

任何一次依赖“先手动碰一下抽屉”才能成功，都判定冷启动失败。

## 8. 推广到其余 4 组抽屉

推广顺序：

1. `dm1`
2. `ds2`
3. `ds3`
4. `ds1`

### 8.1 配置推广规则

- 复用同一套状态机和插件逻辑。
- 每个抽屉只允许通过 YAML 提供几何、工位和行程差异。
- 不复制 5 套 C++ 分支。
- 每组都配置左右把手、左右支撑点和独立解锁区。
- 每组单独完成冷启动开关验收后，才加入 `operable_control_ids`。

### 8.2 P0 把手和支撑坐标

| 控件 | 左把手 | 右把手 | 左支撑 | 右支撑 |
|---|---|---|---|---|
| `ds1` | `(0.099,4.496,2.003)` | `(0.099,4.666,2.003)` | `(0.099,4.496,1.953)` | `(0.099,4.666,1.953)` |
| `ds2` | `(0.099,4.134,1.463)` | `(0.099,4.304,1.463)` | `(0.099,4.134,1.413)` | `(0.099,4.304,1.413)` |
| `db1` | `(0.099,4.107,0.952)` | `(0.099,4.693,0.952)` | `(0.099,4.107,0.902)` | `(0.099,4.693,0.902)` |
| `ds3` | `(0.099,5.296,1.458)` | `(0.099,5.518,1.458)` | `(0.099,5.296,1.408)` | `(0.099,5.518,1.408)` |
| `dm1` | `(0.099,4.907,0.955)` | `(0.099,5.493,0.955)` | `(0.099,4.907,0.905)` | `(0.099,5.493,0.905)` |

### 8.3 `ds1` 特殊门

`ds1` 把手高度约 `2.003 m`，高于已有单臂操作窗口。Agent 必须：

1. 单独搜索左右臂作业角度和底盘停靠位。
2. 验证 ready、解锁、支撑、抓取和完整 `0.30 m` 轨迹，不只验证一个 IK 点。
3. 保持底盘在抽拉阶段固定。
4. 若完整轨迹不可达，停止并提供：
   - 失败姿态
   - IK/碰撞证据
   - 可达边界
   - 不改变模型外观的候选方案
5. 未经用户确认，不得通过移动柜体、加长手臂或让底盘替代抽拉解决。

## 9. Web 与任务层

### 9.1 目录语义

- 5 个目标显示为“抽拉柜”。
- 操作只显示“打开”和“关闭”。
- `*p` 显示为指示灯或不进入操作目录。
- 抽屉声明 `required_toolset=A`。
- 未完成实体验收的抽屉 `adapter_validated=false`。

### 9.2 阶段反馈

Web/SSE 至少展示：

- 正在停靠
- 机械臂正在转到作业角度
- 右侧解锁电机正在推进
- 解锁完成
- 左右支撑电机正在伸出
- 双侧支撑已建立
- 左右夹爪正在抓取
- 双侧抓取已建立
- 正在同步拉出/推回
- 抽屉已打开/关闭
- 正在松爪并收回支撑电机
- 失败阶段和具体原因

### 9.3 验证脚本适配

当前 `validate_cabinet_simulation` 和 `validate_cabinet_web` 的默认计数主要面向按钮、旋钮、开关和门。Agent 应以向后兼容方式增加 drawer 计数或显式控制 ID参数，不能破坏默认 4 类柜体测试。

## 10. 三个场景配色

配色在机械动作稳定后执行。

### 10.1 修改范围

- `xczs_inspection_robot_description/urdf/control_cabinet/components/materials.xacro`
- `xczs_inspection_robot_description/urdf/control_cabinet/components/cabinet_body.xacro`
- `xczs_inspection_robot_description/urdf/control_cabinet/components/cabinet_modules.xacro`
- `xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro`
- `xczs_inspection_robot_description/urdf/scenes/generator_plant.xacro`

### 10.2 颜色语义

| 对象 | 建议颜色 | 注意事项 |
|---|---|---|
| 场景环境 | 低饱和蓝灰/暖灰 | 保持背景克制 |
| 抽屉面板 | 浅蓝灰 | 与环境有明度差 |
| 把手和机械机构 | 深灰金属色 | 突出可抓位置 |
| 指示灯 `*p` | 红/绿/琥珀 | 高亮但不可操作 |
| 警示构件 | 黄/橙 | 只用于安全提示 |

### 10.3 配色硬性检查

- 只允许 `visual/material/ambient/diffuse` 等视觉字段变化。
- STL、link pose、joint、collision、inertial 和插件参数不得因配色改变。
- 分别启动 3 个场景并截图。
- 光照下必须能区分环境、面板、把手和指示灯。
- 不得因颜色高亮把指示灯误标为可操作按钮。

## 11. 自动化检查与构建

### 11.1 每个小阶段

执行：

```bash
git diff --check
scripts/validate/check_adapter_contract
scripts/validate/check_cabinet_model
scripts/validate/check_scene_config
```

若某个检查尚不认识 drawer，应先更新检查器并增加反例测试，不得跳过。

### 11.2 构建

先构建受影响包：

```bash
colcon build --symlink-install --packages-select \
  xczs_inspection_robot_interfaces \
  xczs_inspection_robot_description \
  xczs_inspection_robot_moveit_config \
  xczs_inspection_robot_gazebo \
  xczs_inspection_robot_control \
  xczs_inspection_robot_bringup
```

然后执行完整工作区构建：

```bash
colcon build --symlink-install
```

### 11.3 测试

```bash
colcon test --packages-select \
  xczs_inspection_robot_interfaces \
  xczs_inspection_robot_description \
  xczs_inspection_robot_moveit_config \
  xczs_inspection_robot_gazebo \
  xczs_inspection_robot_control \
  xczs_inspection_robot_bringup

colcon test-result --verbose
pytest -q jiang/tests/
XCZS_PREFLIGHT_ONLY=true ./run_all.sh
```

### 11.4 必须增加的自动化用例

- `TYPE_DRAWER` 目录序列化与 Web 映射。
- `*p` 不在可操作控制目录。
- 没有解锁电机物证时 unlock 失败。
- 没有双侧支撑时抓取或抽拉失败。
- 没有双侧夹爪接触时 attach 失败。
- `base_free=true` 不进入正式抽屉任务路径。
- 左右同步误差越限时安全中止。
- 取消、重置、租约失效和场景切换会释放全部约束。
- 5 组抽屉配置字段完整且坐标与 P0 一致。
- 三场景颜色变更不影响物理字段。

## 12. 统一入口运行验收

### 12.1 启动前

- 不打印密码、Token 或其他密钥。
- 不终止不属于本项目的进程。
- 使用独立 ROS domain/端口时，记录实际参数但隐藏敏感值。
- 确认活动场景为 `electrical_mezzanine`，工具套装为 A。

### 12.2 启动

```bash
./run_all.sh
```

如需通过已有资产选择切换场景，使用项目当前正式选择流程；不要修改启动脚本默认值来绕过场景选择。

### 12.3 分层验收

1. **Gazebo 底层**
   - 单独验证解锁、支撑、夹爪、attach、短行程抽拉和推回。
2. **ROS 2 Action**
   - 使用 `OperateCabinetControl` 打开/关闭 `db1`。
3. **任务层**
   - 验证任务创建、反馈、取消和结果。
4. **Web**
   - 从页面点击打开/关闭，观察完整动作。
5. **场景切换**
   - 在操作中切换场景，确认安全中止。
6. **全量**
   - 对已验证的 5 组抽屉逐组执行。

### 12.4 每个抽屉的最终通过条件

- [ ] 机械臂明显转到作业角度。
- [ ] 右侧解锁电机实际伸出并推进右把手解锁结构。
- [ ] 指示灯没有被按压或移动。
- [ ] 左右支撑电机实际伸出并抵住附近柜体。
- [ ] 左右夹爪电机实际动作并抓住两个把手。
- [ ] 两条机械臂同步拉出，底盘没有代替抽拉。
- [ ] 抽屉到达 `0.30 m` 开位并稳定。
- [ ] 两条机械臂同步推回 `0.0 m`。
- [ ] 关闭后锁止恢复。
- [ ] 松爪、收支撑和撤离顺序正确。
- [ ] Action 返回成功且带有物理结果证据。
- [ ] 连续 3 次开关无穿模、无乱跑、无残留约束。

## 13. 失败时的处理规则

### 13.1 可以自行继续的情况

- 配置字段遗漏或静态检查失败。
- 单元测试暴露的类型映射错误。
- 已知坐标的小范围标定偏差。
- 控制器容差或速度需要在安全范围内微调。

每次调整必须有前后证据，不能盲目扩大容差。

### 13.2 必须停止并询问用户的情况

- 无法从现有末端结构中唯一确定支撑电机和夹爪职责。
- 右把手上没有可对应的解锁结构，逻辑区位置也无法确认。
- 必须增加可见按钮、把手或其他机构。
- 必须修改 STL、机器人手臂长度或柜体外形。
- 必须引入新的第三方依赖。
- `ds1` 在底盘固定条件下无法完成完整双臂轨迹。
- 真实双臂抽拉必须改成底盘拖拽才能完成。

### 13.3 禁止隐藏的问题

- 规划成功不等于执行成功。
- Action 成功不等于物理动作正确。
- 抽屉关节到位不等于夹爪和支撑电机真实工作。
- 一次热启动成功不等于冷启动可靠。
- 颜色写入 Xacro 不等于 Gazebo 中视觉效果已验收。

## 14. Agent 每阶段汇报模板

每完成一个阶段，用下面格式更新用户：

```markdown
### 阶段：P<number> <名称>

完成内容：
- ...

修改文件：
- `path/to/file`

验证命令与结果：
- `<command>`：PASS/FAIL，关键输出 ...

物理证据：
- 解锁电机位置：...
- 左/右支撑电机位置：... / ...
- 左/右夹爪位置：... / ...
- 抽屉关节位置：...
- 同步误差：...

仍未解决：
- ...

下一阶段：
- ...
```

## 15. 最终交付清单

- [ ] `docx/P1-末端电机职责与接触点标定.md`
- [ ] 正确的右把手解锁区合同，且与 `*p` 指示灯分离
- [ ] 两侧支撑电机独立控制和接触反馈
- [ ] 两侧夹爪独立控制和把手接触反馈
- [ ] 双臂同步抽拉/推回状态机
- [ ] `db1` 三次冷启动、每次三轮开关证据
- [ ] `dm1`、`ds2`、`ds3` 逐项物理验收
- [ ] `ds1` 完整物理验收或经用户确认的不可达处理
- [ ] 5 组抽屉 Web 全链路验收
- [ ] 取消、重置、错工具套装和场景切换验收
- [ ] 3 个内置场景配色和截图验收
- [ ] 静态检查、构建、测试、预检结果
- [ ] `./run_all.sh` 最终运行回归

只有上述交付全部完成，才能向用户报告“抽拉柜功能与全场景配色已完成”。
