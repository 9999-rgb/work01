# AGENT 后续实现与验收方案：以可实现为第一优先级完成 db1 抽拉柜

> 本文是后续编码 Agent 的直接执行合同。目标不是一次性模拟全部真实接触，而是先用项目现有能力稳定完成 `electrical_mezzanine/db1` 的“就位 → 钩住 → 侧缝支撑 → 右把手解锁 → 双臂抽拉 → 推回复闩”闭环，再决定是否增加更高保真物理接触。

## 0. 范围冻结与完成定义

### 0.1 本轮必须完成

1. 只完成电气夹层场景 `electrical_mezzanine` 的抽拉柜 `db1`。
2. 修复当前 `cap3` 前后的不稳定问题，使机械臂不再因为工具控制器复位、回零、二次追赶而抖动或脱离把手。
3. 保持下面的真实动作语义：
   - 左右机械臂先到同一个抽屉工作位姿；
   - 左右钩爪电缸压贴并保持在两侧把手；
   - 左右支撑电缸伸入柜体两侧侧缝的标定区域；
   - 右手额外解锁电缸按压右把手本体上的解锁位置；
   - 解锁成功后建立双臂柔顺耦合并拉出抽屉；
   - 关闭时双臂推回，解除耦合、收回电缸并复闩。
4. 通过现有分级门 `cap1`～`cap8`，再完成完整打开和关闭。
5. 补一个只面向 `db1` 的 Web 全链路验收脚本。
6. 检查三个场景已有配色；只修复实际缺色或材质引用错误，不做无关的整体重配色。

### 0.2 本轮明确不做

- 不把 `dm1`、`ds1`、`ds2`、`ds3` 一并改造成可操作抽屉。
- 不为侧缝新增刚性碰撞块、力传感器、接触插件或新的 ROS 接口。
- 不把钩爪与把手建立两个固定关节；这会形成闭环约束并重新引入抖动风险。
- 不使用底盘移动代替双臂抽拉，`base_free` 必须保持为 `false`。
- 不修改用户可见的柜体、把手或按钮外观。
- 不新增第三方依赖，不扩展场景资产清单格式。
- 不编辑 `docs/build/`、`build/`、`install/`、`log/` 下的生成文件。

### 0.3 “完成”的唯一判定

同时满足以下条件才可宣布完成：

- 静态合同、构建和相关自动化测试全部通过；
- `cap1`～`cap8` 按顺序通过，`cap2` 和 `cap3` 各连续通过 3 次；
- 在一次干净启动中，`db1` 连续完成 3 个“打开 → 关闭”循环；
- 机械臂没有持续高频抖动、控制器重启、工具自由追赶或明显碰撞；
- Web 发起的任务确实到达 ROS 2 Action 和 Gazebo 插件，不能用直接改关节状态代替；
- 其他控件状态没有被 `db1` 任务意外改变；
- 证据包含命令、日志、关节/抽屉位置和人工画面检查。

## 1. 重新审查后的工程事实

### 1.1 已存在且应直接复用的链路

当前主链路已经存在：

```text
Web/FastAPI
  → TaskManager / ControlServer
  → CabinetClient
  → OperateCabinetControl Action
  → cabinet_button_operator
  → ros2_control / MoveIt
  → cabinet_state_plugin
  → Gazebo 抽屉关节
```

`xczs_inspection_robot_gazebo/src/cabinet_state_plugin.cpp` 已经提供：

- 抽屉闩锁、解锁和自动复闩；
- 右手解锁电缸端点与逻辑解锁区的距离校验；
- 左右工具点到把手区域的双手几何校验；
- 租约和心跳；
- 双臂附着后的柔顺线性耦合；
- 距离失效时断开耦合并发布故障。

因此不得重写第二套抽屉物理系统，也不得再增加裸 socket 或旁路控制。

### 1.2 当前模型能表达什么

- `db1` 把手碰撞是平板近似，不存在真正可穿入并锁住钩子的孔腔。
- 当前可实现的“钩住”是：钩爪电缸沿标定方向压贴把手刚体，保持几何接近；随后由插件柔顺耦合承担抽拉跟随。
- 右把手本体上的解锁按钮目前是逻辑解锁区，不是独立可动按钮关节。
- 场景插件没有左右侧缝的力/接触服务；控制节点当前只能检查支撑杆伸出量、端点位置和任务稳定性。

文档和结果消息必须如实使用以下术语：

| 用户语义 | 本轮可验证实现 |
| --- | --- |
| 钩住把手 | 钩爪压贴、位置保持、工具点未脱离，之后建立柔顺耦合 |
| 抵住侧缝 | 支撑杆伸到侧缝标定区并保持；抽屉仍闩定且不移动 |
| 按下解锁按钮 | 右解锁杆伸到右把手逻辑区，插件验证真实关节伸出量后解闩 |
| 双手拉出 | `base_free=false`，左右臂通过既有柔顺耦合驱动抽屉轨道 |

不得在没有碰撞/力数据时把“端点进入标定区”写成“已测得真实支撑力”。

### 1.3 当前已知阻塞

当前 `cap3` 的主要失败来源不是缺少更多物理模型，而是钩爪封定后的控制流程过度复杂：

1. 钩爪压贴并保持 3 秒；
2. 读取座封位置；
3. 停用、重新配置并激活左右工具控制器；
4. 工具关节先回到 home；
5. 再追赶座封位置并额外推进；
6. 进入长时间 park/freeze。

实测已出现抽屉漂移超过门限和右钩爪自由追赶越过座封点。正常路径不应依赖控制器重启，也不应制造一次“先失去接触再找回接触”的窗口。

## 2. 总体实现原则

1. **优先删复杂状态。** 先删除正常流程中的控制器复位、回零和二次追赶，再观察，不先调大 PID。
2. **沿用现有合同。** 解锁和抽拉复用现有 service/action/plugin，不增加同义接口。
3. **一次只过一扇门。** 每个 cap 通过后才进入下一阶段；失败只修复首个失败门。
4. **参数必须来自配置。** 场景几何留在 `electrical_mezzanine_adapter.yaml`，不得散落为新硬编码常量。
5. **失败必须安全收尾。** 取消、超时和异常均需断开耦合、收回解锁/支撑/钩爪、复闩并退让。
6. **不把观测脚本当控制器。** `scripts/tools/` 只采样和分析，不直接驱动业务状态。

## 3. P0：冻结基线，保护现有改动

### 3.1 开始前记录

```bash
cd /home/live/work01
git status --short
git diff --check
git diff --stat
```

保存所有与本任务已有修改的 diff；不得 reset、checkout 或覆盖用户文件。

### 3.2 静态基线

```bash
cd /home/live/work01
scripts/validate/check_cabinet_model
scripts/validate/check_scene_config
xacro xczs_inspection_robot_description/urdf/scenes/electrical_mezzanine.xacro >/dev/null
```

预期：模型、场景和 Xacro 检查通过。若此处失败，先修静态错误，不启动 Gazebo。

## 4. P1：把钩爪封定改成“原位接管”

### 4.1 修改位置

主文件：

```text
xczs_inspection_robot_control/src/cabinet_button_operator.cpp
```

重点审查 `seal_drawer_hooks_by_probe()` 及其调用方。保留现有低速探测、接触确认和 3 秒稳定保持；删除正常成功路径中的以下行为：

- deactivate/configure/activate 工具控制器；
- 钩爪回 home；
- 从 home 重新追赶座封点；
- 为追赶额外添加固定推进量；
- 90 秒 park/freeze。

控制器复位函数可以保留为异常恢复工具，但正常 `db1` 打开路径不得调用。

### 4.2 最小状态结构

在单次抽屉任务内部保存座封参考，示意结构：

```cpp
struct DrawerHookHoldState
{
  double left_joint_ref{0.0};
  double right_joint_ref{0.0};
  bool valid{false};
};
```

要求：

- 仅存在于一次 Action 执行上下文中，不做全局缓存；
- 只有左右两侧都完成接触与稳定保持后才置 `valid=true`；
- 任一侧计算、发送或读回失败，整阶段失败并进入统一安全收尾。

### 4.3 座封参考的计算

不能使用工具端点到 home 端点的欧氏距离，因为该距离会混入横向抖动。每侧都应：

1. 从现有工具标定取得真实伸缩轴单位向量；
2. 取得 home 时的物理端点 `p_home`；
3. 取得接触稳定时端点 `p_contact`；
4. 计算有符号投影：

```text
extension = dot(p_contact - p_home, extension_axis)
joint_ref = home_joint + extension
```

5. 将 `joint_ref` 限制在该关节现有上下限内；
6. 立即以当前位置参考接管，不先回零。

若项目已有“端点位移转关节位移”函数，必须复用，禁止复制一份近似实现。

### 4.4 后续阶段必须继承保持值

工具控制器使用完整关节目标，所以后续每次发送目标都必须显式携带钩爪保持参考，不能让静态 `gripper_grasp_position` 覆盖实测座封位置：

| 阶段 | 左工具目标 | 右工具目标 |
| --- | --- | --- |
| 钩爪保持 | `hook=left_ref` | `hook=right_ref` |
| 侧缝支撑 | `hook=left_ref, support=0.041` | `hook=right_ref, support=0.042, unlock=0` |
| 解锁按压 | 保持上一目标 | `hook=right_ref, support=0.042, unlock=0.008` |
| attach/抽拉 | 保持座封和支撑 | 保持座封、支撑和解锁 |

表中数值以当前 `electrical_mezzanine_adapter.yaml` 为准；若实际配置不同，应读取配置，不在 C++ 另写常量。

### 4.5 P1 单元测试

至少覆盖：

- 投影在正、负和横向扰动下的结果；
- 关节上下限裁剪；
- 左右任一侧无效时不得进入后续阶段；
- support/unlock 完整关节目标保留 hook ref；
- 取消和异常不残留任务级保持状态；
- 正常路径不出现 controller deactivate/activate。

### 4.6 P1 验收门

- `cap2` 连续 3 次通过；
- 两侧接触保持期间抽屉仍闩定且位置变化不超过现有门限；
- 日志中无控制器重启、回 home、二次自由追赶；
- 两个钩爪在进入支撑阶段前后没有明显位置跳变；
- 若仍抖动，先输出目标值、实测值、控制器状态和 Gazebo 接触证据，再决定是否调参。

## 5. P2：用可验证的几何保持完成侧缝支撑

### 5.1 本轮实现

复用当前左右支撑电缸和适配器几何：

- 左右钩爪继续保持 P1 的座封参考；
- 左右支撑杆低速伸到各自配置目标；
- 校验支撑关节到位；
- 校验支撑杆端点进入各自侧缝标定区域；
- 保持一小段稳定时间；
- 校验抽屉仍为关闭、闩定且位移未超过门限。

这满足“机械臂上的可伸长电机抵到附近柜体侧缝”的任务顺序，同时不引入尚不存在的力传感系统。

### 5.2 禁止项

本轮不得为了让 `cap3` 通过而：

- 给柜体临时增加不可见刚性碰撞墙；
- 添加固定关节或高刚度闭环；
- 仅凭 Gazebo 画面宣告存在支撑力；
- 放宽到位、漂移或距离门限掩盖失败；
- 修改模型可见外观。

如果用户后续明确要求“必须有可测物理支撑力”，应另立增强任务，先设计接触传感、碰撞几何和闭环稳定性，不能混入本轮 MVP。

### 5.3 P2 验收门

- `cap3` 连续 3 次通过；
- 支撑伸出时，hook ref 始终被保留；
- 左右支撑端点分别进入自己的侧缝区域，不能只检查一侧；
- 支撑阶段抽屉没有被提前解锁或拉动；
- 无持续高频抖动、控制器复位或工具端自由越过目标。

## 6. P3：复用现有右把手逻辑解锁

### 6.1 实现顺序

1. 确认左右钩爪保持有效；
2. 确认左右支撑阶段已通过；
3. 右手解锁电缸伸到适配器配置值；
4. 校验真实关节读回；
5. 调用现有解锁服务；
6. 由插件同时校验端点到逻辑解锁区的距离、关节伸出范围和租约；
7. 在 attach 和抽拉期间保持解锁杆，不提前回撤。

### 6.2 不做的事情

- 不把右把手旁的指示灯改成按钮；
- 不增加一个新的可动按钮模型；
- 不仅根据指令目标值宣告解锁成功；
- 不绕过插件直接修改抽屉闩状态。

### 6.3 验收门

- `cap4`：解锁杆已伸出且读回达标，但服务尚未放闩；
- `cap5`：服务成功放闩，解锁杆仍保持；
- 错误位置、错误伸出量、租约失效均应 fail closed。

## 7. P4：复用柔顺双臂耦合完成抽拉

### 7.1 实现要求

- `base_free=false`；
- 使用现有 `SetCabinetBimanualGrasp` 和插件柔顺耦合；
- attach 前再次验证左右工具点距离、解锁状态和租约；
- 拉出时左右机械臂同步沿抽屉轨道方向运动；
- 运行中持续做距离 keepalive；
- 任一侧失去几何保持时立即断开耦合并进入安全收尾；
- 关闭时沿相反方向推回关闭位，确认复闩后才释放工具。

### 7.2 分级门

| 门 | 动作 | 必须证明 |
| --- | --- | --- |
| cap6 | attach 后保持 3 秒，不拉动 | 柔顺耦合稳定、抽屉未漂移 |
| cap7 | 拉出 0.03 m，再推回 | 实测轨道位置进入紧容差带并复位 |
| cap8 | 拉出 0.10 m，再推回 | 距离保持、轨道实际运动、无明显抖动并复闩 |

不得使用通用的宽松滑块容差替代 `debug_capped_pull_tolerance`；不能把“目标已发送”当成“抽屉已移动”。

### 7.3 完整打开与关闭

`cap8` 通过后，将 `debug_stage_cap` 恢复为 `0`，执行：

1. Web 请求 `db1` 打开；
2. 验证 Action 成功、轨道位置达到打开目标；
3. Web 请求 `db1` 关闭；
4. 验证轨道回到关闭位置并复闩；
5. 验证钩爪、支撑杆和解锁杆均收回；
6. 验证机械臂安全退让。

在同一次干净启动中连续执行 3 个循环。任何一轮失败都从首个失败阶段重新定位，不能只重试到偶然成功。

## 8. P5：修正静态合同和脆弱测试

### 8.1 坐标系合同

当前电气夹层 fixture 文件里的 cabinet pose 使用 `odom`，但启动层会用适配器的 `pose_parent_frame=map` 覆盖运行时父坐标系。运行时权威是 `map`，静态检查器没有理解这次覆盖，导致显式检查误报。

最小修复原则：

- 不给 fixture schema 新增 `pose_config`；
- 不复制第二份 pose 文件；
- 给 `scripts/validate/check_adapter_contract` 增加可选的 `--instance-id`；指定 fixture 时，从现有 `cabinet_instances.yaml` 解析该实例的 controls/scene/adapter，并按启动文件相同规则计算 effective parent frame；
- 普通非 fixture 柜体仍保持严格匹配；
- 增加“fixture override 通过”和“无 override 的真实不匹配失败”两类测试。

修复后执行：

```bash
cd /home/live/work01
scripts/validate/check_adapter_contract
scripts/validate/check_adapter_contract --instance-id electrical_mezzanine
```

两者都必须返回 0。

### 8.2 修正测试，不删除测试

当前部分控制测试依赖源码文本出现次数或完整字典相等，已有合法变化后会误报。应改为语义断言：

- 断言关键调用发生在正确阶段，而不是固定出现 N 次；
- 分别断言 builtin/sample 必需字段，不要求完整字典永久相等；
- 断言租约在操作路径被建立、传递和清理，不统计赋值字符串次数；
- 断言业务点经过统一坐标变换，不统计表达式文本次数。

禁止通过删除、跳过或放宽真实安全断言来获得绿色测试。

## 9. P6：增加专用 db1 Web 验收器

### 9.1 原因

现有 `scripts/validate/validate_cabinet_web` 假定经典柜体控件类型和按钮数量，不能准确表达电气夹层的 4 个规划滑块加 1 个可操作抽屉。不要大改通用验证器。

### 9.2 新文件

```text
scripts/validate/validate_db1_drawer
```

只使用标准库和项目已有依赖，并通过 HTTP/Web 业务入口执行：

1. 读取当前活动场景，必要时通过显式参数切换到 `electrical_mezzanine`；
2. GET 电气夹层控件列表；
3. 校验 `db1` 存在、类型为 drawer、可操作且要求 toolset A；
4. 记录其余控件状态；
5. 发起 navigate；
6. 发起 `db1 set_state=open`；
7. 轮询任务直到成功或超时；
8. 发起 `db1 set_state=closed` 并轮询；
9. 按 `--cycles` 重复；
10. 校验其他控件未变化；
11. 输出机器可读摘要并用退出码表示成功/失败。

脚本不得：

- 直接调用 Gazebo 改关节；
- 直接绕过 Web 调 ROS Action；
- 自动放宽超时和容差；
- 静默切换场景；
- 在命令行或日志打印令牌。

需要鉴权时只从环境变量读取，日志中统一显示 `<REDACTED>`。

## 10. P7：三个场景配色只做缺口修复

检查：

- `cabinet_operation`
- `electrical_mezzanine`
- `generator_plant`

执行方式：

1. Xacro/SDF 静态展开，确认 material 引用存在；
2. 从 `./run_all.sh --web` 逐个切换场景；
3. 画面检查柜体、机器人、地面、主要设备和指示区域是否仍为默认灰或材质丢失；
4. 只有发现明确缺口时，修改对应场景自己的 material/Gazebo material；
5. 不改变几何、碰撞、惯量和关节；
6. 不为了“统一风格”覆盖已有合理配色。

配色验收至少保存每个场景一张全景截图。若只是灯光导致发灰，优先修灯光/渲染引用，不改模型颜色数值。

## 11. 构建与自动化验证

### 11.1 构建

源码修改后执行：

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  xczs_inspection_robot_interfaces \
  xczs_inspection_robot_description \
  xczs_inspection_robot_gazebo \
  xczs_inspection_robot_control \
  xczs_inspection_robot_bringup
```

构建失败时只修首个相关错误，不删除现有包，也不编辑生成目录。

### 11.2 静态和控制测试

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source /home/live/work01/install/setup.bash

scripts/validate/check_cabinet_model
scripts/validate/check_scene_config
scripts/validate/check_adapter_contract

colcon test --packages-select xczs_inspection_robot_control
colcon test-result --verbose
```

要求相关测试 0 失败。已知的源码计数式脆弱测试必须按第 8.2 节修复后重新运行。

### 11.3 Gateway/Web 回归

必须从 `jiang/` 运行，避免仓库源码目录遮蔽 `install/` 中生成的 Python interface：

```bash
cd /home/live/work01/jiang
source /opt/ros/humble/setup.bash
source /home/live/work01/install/setup.bash
PYTHONPATH=/home/live/work01/jiang${PYTHONPATH:+:$PYTHONPATH} \
python3 -m pytest -q \
  tests/test_profile_contract.py \
  tests/test_scene_switch.py \
  tests/test_control_gateway_http.py
```

当前审查基线为 65 项通过；修改后不得下降。

## 12. 运行时分级执行顺序

### 12.1 启动

```bash
cd /home/live/work01
./run_all.sh --web
```

等待统一入口健康后再发任务。不得同时保留第二套 Gazebo、旧 operator 或旧 ros2_control 实例。

### 12.2 每一轮前检查

- 活动场景是 `electrical_mezzanine`；
- 只有目标 `db1` 被标记为可执行；
- 工具集为 A；
- 抽屉处于关闭和闩定状态；
- 左右工具电缸均在 home；
- 无上轮残留 lease、attach、fault 或 Action；
- TF 权威父坐标系为 `map`；
- 录制本轮 operator、插件、controller 和关节状态日志。

### 12.3 门序列

严格依次执行：

```text
cap1 工作位姿
  → cap2 双钩压贴和原位保持（连续 3 次）
  → cap3 双侧缝支撑（连续 3 次）
  → cap4 右手解锁杆按压
  → cap5 插件真实放闩
  → cap6 柔顺耦合保持
  → cap7 拉 0.03 m 并推回
  → cap8 拉 0.10 m 并推回
  → debug_stage_cap=0 完整打开/关闭 3 循环
```

每次只通过运行时参数切换 `debug_stage_cap`，不为每个 cap 编译不同代码。一次失败后先安全收尾，再分析首个异常时间点。

### 12.4 每门证据

每次结果至少记录：

```text
场景 / control_id / toolset
debug_stage_cap
任务开始和结束时间
Action result_code / message
左钩爪目标值与实测值
右钩爪目标值与实测值
左右支撑目标值与实测值
右解锁杆目标值与实测值
抽屉轨道初值、峰值、终值
闩锁状态 / attach 状态 / lease 状态
左右工具点到各自目标区域距离
controller 是否重启或报错
operation_fault
截图或录像路径
PASS / FAIL 和首个失败断言
```

禁止只写“画面看起来正常”。

## 13. 故障与收尾合同

正常结束、取消、超时和异常必须走同一套幂等收尾：

1. 停止继续拉动；
2. 断开双臂耦合；
3. 若抽屉可安全推回则推回关闭位；
4. 调用复闩并验证；
5. 收回右解锁杆；
6. 收回左右支撑杆；
7. 收回左右钩爪；
8. 双臂退让；
9. 释放 lease；
10. 清理任务级 `DrawerHookHoldState`；
11. 发布明确结果和首个失败原因。

最低异常验收：在 cap6 后取消一次任务，确认耦合解除、抽屉复闩、全部电缸收回且下一任务可正常开始。

## 14. 提交前最终清单

- [ ] 仅修改与本任务有关的源码、配置、测试、验证脚本和文档。
- [ ] 未修改任何 `docs/build/`、`build/`、`install/` 或 `log/` 文件。
- [ ] 正常路径不再重启工具控制器或让钩爪回 home 后追赶。
- [ ] hook 座封参考贯穿 support、unlock、attach 和 pull。
- [ ] `cap2`、`cap3` 各连续通过 3 次。
- [ ] `cap1`～`cap8` 全部通过。
- [ ] 完整打开/关闭连续 3 循环通过。
- [ ] 取消后的幂等安全收尾通过。
- [ ] adapter contract 在运行时 frame override 语义下通过。
- [ ] control 测试 0 失败。
- [ ] Gateway/Web 三组测试不少于当前 65 项且 0 失败。
- [ ] `scripts/validate/validate_db1_drawer --cycles 3` 通过。
- [ ] 三个场景完成画面配色检查，只修复真实缺口。
- [ ] `git diff --check` 通过。
- [ ] 交付报告包含改动文件、命令、结果、残余风险和证据路径。

## 15. 必须停止并询问用户的情况

遇到以下任一情况，不得擅自扩大实现：

- 只有改变柜体、把手、按钮或侧缝的可见外观才能继续；
- 需要新增第三方库、Gazebo 插件或新的硬件接口；
- 用户要求“真实支撑力”，但现有系统没有可用接触/力传感证据；
- `db1` 之外的抽屉也要转为可操作，涉及批量重构模型；
- 需要改变 Action/Service 对外语义，可能影响 Web 或其他场景；
- 同一目标连续 3 次仍表现为非确定性失败，且日志无法归因；
- 需要删除或覆盖用户已有修改。

此时应报告：当前证据、首个阻塞点、最小可选方案及其代价，等待用户选择。
