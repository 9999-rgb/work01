# AGENT 后续执行方案：电气夹层 db1 进度审查、难点处理与无人值守推进

> 审查日期：2026-09-06（Asia/Shanghai）。仓库：`/home/live/work01`。
> 基线：`b56680c`（008）+ 当前未提交修改。本文替代本文件此前版本的待办顺序；旧“具体执行方案”仅用于追溯用户机构语义。
> 结论：基础代码与静态合同已经落地；运行时有一次 cap3 完整成功证据，但重复稳定性未通过。解锁触点几何存在明确矛盾；最新 fix21 所在启动失败，尚不能宣告其运行有效。完整抽拉、推回和 Web 三循环仍未验收。

## 0. 无人值守授权与执行规则

**用户明确要求：不需要问我问题；我会长时间不在场，Agent 自行决策即可。**

1. 本任务范围内的诊断、实现路径、参数选择、测试、重编译、仿真启动与恢复由 Agent 自行决定；不要调用提问工具，不要因常规方案选择停下来等待回复。
2. 以当前仓库、日志、实际几何和回归结果为依据。选择改动最小、可回退、可以验证的方案，每项重要决定记录“证据 → 选择 → 结果”。
3. 保留现有用户改动。不要 reset 工作树，不删除历史代码或大目录；失败试验只回退自己本轮引入的明确差异。
4. 可自行修改本任务的软件、配置、校验脚本及不改变可见外观的仿真近似。优先复用项目和已安装组件；不把新增依赖、外部服务或硬件改造设为推进前提。
5. 当前实际任务是“审查并更新执行文档”；后续编码 Agent 按本文执行时，不需要重新逐项询问实现细节。
6. 遇到真实几何不可实现时，执行第 4 节的确定性分支：保留严格模式的失败结论，必要时实现明确标识的仿真机构近似。不得通过放宽触点门限、改成功标志或偷偷移动按钮坐标制造成功。
7. 当前分支无法继续时，自动安全停止该分支、保存证据，继续其他不依赖它的工作。最终报告真实未完成项；无需以提问结束任务。
8. 同一配置最多重复验证 3 次。连续失败后必须分析证据或更换有依据的方案，禁止无上限重启、重试或叠加 fix 编号。
9. 保持已有 ROS 2 topic/service/action 通信及工具集门控。涉及物理操作只使用仿真，不将本方案自动应用到真实机器人。

## 1. 当前做到哪里了

### 1.1 代码与验证进度

| 项目 | 本轮核实状态 | 后续处理 |
| --- | --- | --- |
| Web → 任务层 → CabinetClient → Action → operator → Gazebo | 链路已有实现 | 复用，补全终端证据 |
| 原位保持、任务级 DrawerHookHoldState、投影纯函数 | 已实现；后续 fix15 改成保留压贴目标 | 不按旧文档重做 |
| fixture 的 map/odom 合同 | `--instance-id electrical_mezzanine` 本轮通过 | 不再列为开发任务 |
| db1 Web 验收器 | `scripts/validate/validate_db1_drawer` 已提交 | 修补检查模式与失败恢复，复用已有逻辑 |
| cap Web 验收器 | `scripts/validate/validate_db1_cap_gates` 存在、未跟踪 | 修失败参数恢复、证据关联和串扰校验 |
| 动态侧缝目标 | 已有 `support_contact_inset=0.006` 及现场测量计算 | 保留单次成功基线，核对重复稳定性 |
| cap3 | `/tmp/cap3_run2.log` 明确成功，前一轮失败 | 不能标记连续稳定通过 |
| cap5 解锁 | 历史运行已到解锁杆伸出，插件拒绝解锁 | 优先解决触点几何 |
| fix21 新接管判据 | 源码及较新的 operator 二进制存在 | 启动失败，运行效果未验证；先补采样新鲜度缺陷 |
| cap6～8、完整 0.3 m 打开/关闭 | 本轮所查最新证据中没有完整通过记录 | 全部保持未验收 |
| 三场景配色 | 已有材质定义 | 尚需统一入口下的画面验收 |

本轮静态/单元验证：

- `check_adapter_contract`、电气夹层实例合同、`check_cabinet_model`、`check_scene_config`、电气夹层 Xacro 展开：通过。
- Gateway 三组测试：74 passed。
- 控制侧四组 Python 合同测试：29 passed。
- 运行已有 `drawer_hook_seat_projection_test` 二进制：9 passed；二进制时间晚于对应测试源码。
- `git diff --check`：通过。
- `colcon test-result` 显示 183 tests / 0 failures，但 `log/latest_test` 指向 2026-09-05 14:21。这是历史整包测试报告，不能作为当前所有未提交改动的整包验证。
- 本轮审查时进程列表未见 Gazebo/operator 运行；没有新发运动任务。

### 1.2 不应再重复做的事情

- 不再创建第二个 db1 Web 验收器。
- 不再重做 fixture schema 或 frame override。
- 不再要求正常钩取路径重启控制器、回 home 后重新追赶。
- 不再把 `hook_hold.left_joint_ref/right_joint_ref` 理解为“实测座封位置”：当前实际承载的是压贴指令参考，约 0.030 m；实测触点位置约 0.01～0.02 m，含义不同。
- 不再宣称电气夹层只有 db1 在允许列表。当前允许列表是 `db1, dm1, ds2, ds3`；本轮只选择 db1 测试，其余保留现有 slider 行为并检查串扰。

## 2. 当前最重要的阻塞与证据

### 2.1 启动阻塞：最新 fix21 没有进入业务验证

证据：`/tmp/xczs_bridge_boot_fix21.log`。

- behavior_server 日志已有 spin 创建和激活记录。
- bt_navigator 随后报 `"spin" action server not available after waiting for 5.00s`。
- lifecycle manager 启动失败；supervisor 报 Nav2 lifecycle manager 不 active；统一入口 120 秒未就绪。

因此“缺少 Spin 插件”尚不是根因。优先核查 Action 发现、命名空间、ROS domain、RMW/桥接设置和启动时序。现有 `wait_for_service_timeout` 已为 5000 ms，不能再把“设置为 5 秒”列为新修复。

### 2.2 机械保持：仍有实际回缩，不能只改判据

证据：

- `/tmp/cap2_rerun.out`、`/tmp/cap2b_run.out`：降参考接管后右杆由座封约 16 mm 回到 4～6 mm。
- 当前 fix15 保持深压目标，避免上述指令卸载。
- `/tmp/xczs_bridge_boot_fix20.log`：仍有右杆座封约 11.5/14.4 mm、观测最低约 5.2/6.8 mm 的失败。
- `/tmp/rod_trace_fix20.log` 和当前源码注释均提示：指令不变时仍存在瞬态回缩。回缩的根因尚不能仅凭 effort 判断为无害噪声。

fix21 允许短时间回缩后重新在位，只能算“减少瞬态误报的候选判据”。它没有证明用户看到的抖动已消失。关节 effort 是驱动输出/负载线索，也可能来自工具内部卡碰，不能独立当作把手接触力。

### 2.3 解锁几何：当前三个接触点不能同时成立

文件依据：

- `xczs_inspection_robot_description/urdf/components/tools.xacro`
- `xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_adapter.yaml`
- `xczs_inspection_robot_control/config/scene_controls/electrical_mezzanine_controls.yaml`

在右工具 base 坐标中，零位接触点为：

```text
right_hook   = [ 0.034, -0.061735, -0.388500]
right_unlock = [-0.046,  0.020265, -0.358952]
delta_xy     = [-0.080,  0.082]
最小横向间距 = sqrt(0.080² + 0.082²) = 0.114560 m
```

这些伸缩关节轴平行，只能改变 z 分量；整臂刚体转动也不改变两点间距下界。配置却将 `right_handle_point` 和 `unlock_press_point` 都设为 `[0.099, 4.693, 0.952]`。

即使采用插件较宽的 hook 20 mm + unlock 8 mm 两个门限，总误差预算也只有 28 mm，远小于 114.560 mm。因此，在当前触点定义下，“同一工作位姿右钩保持 + 右解锁杆触达同一点”不可实现。

现场吻合证据：`/tmp/xczs_restart.log` 中解锁关节读回 0.0079 m，但插件拒绝：距离 0.118545 m > 0.008 m。继续调伸出量或反复跑 cap3 无法消除这一横向矛盾。

这个结论针对当前模型/配置的点定义，不代表已经证明真实硬件设计错误。必须先核实是否选错了钩面、按钮面或杆端。

### 2.4 新判据与验收脚本可能给出错误结论

源码审查发现：

1. fix21 的左右 `inband` 在物理读数缺失时不清除，且通过条件只要求历史 sample 数量 > 0。一帧在带内之后持续缺数，计时仍可满足确认时长；这不是连续新鲜证据。
2. `validate_db1_cap_gates` 仅在成功出口恢复 `debug_stage_cap=0`；异常、超时和 Ctrl+C 路径没有统一 finally。
3. cap 标记从 `/health` 的长期缓存 message 读取，连续相同 cap 时可能误用上一任务消息。
4. 非目标控件比较使用 `zip`，未校验集合/数量，并检查了 `prev["in_motion"]` 而非 `curr["in_motion"]`。
5. “closed 且位置小于 20 mm”不能证明闩锁、工具 home、租约释放以及没有在运动。
6. `validate_db1_drawer --check-only` 在发现非目标场景时仍会切换场景；帮助文案不能被理解为纯只读预检。

这些应先修，再收集连续通过率。

### 2.5 后续打开/关闭还有坐标和恢复风险

- **关闭路径有确定的判据冲突：** open/close 都调用 `drive_drawer_hook_stage()`，而压贴与保持阶段使用绝对 `drawer_position > 0.0025` 判失败。从 0.30 m 开位执行 close 会触发这条“闭位抽屉被顶开”的检查。必须区分打开前闭位锁止和关闭前开位保持，不能等完整运行才处理。
- `physics_anchor_entity: b1` 使用可移动抽屉作柜体锚点，减去固定闭合局部原点。关闭任务从开位重新 latch 时，可能把抽屉行程误计入柜体原点。应以测试确认；优先用静止实体，或显式扣除已测轨道位移。
- 物理查询未显式指定参考系，代码直接使用返回位置修正 planning 变换。当前假设 `world==odom`，要以运行时变换证据验证，不能扩展成通用保证。
- 支撑点当前随 drawer rail point 计算；这不等于固定柜体支撑。真实固定支撑在抽拉时需要运动补偿；本轮几何近似不得声称持续真实受力。
- 旧文档“先 detach 再主动推回”的统一恢复顺序不可直接执行：解除传力后未必还能推回。
- 正常流程使用 best-effort 电缸回收；仅有警告后继续退臂不是“全部回收已验证”。

## 3. 推进顺序与节省时间的方法

按以下顺序推进，每步保存可独立检查的结果：

```text
A. 修验收工具与新鲜度判据
B. 恢复统一入口可重复启动
C. 确定右手解锁几何的可实现分支
D. 稳定钩爪与侧缝阶段
E. 通过所选模式的 cap4～8
F. 验证打开后的重新接近、关闭、异常收尾
G. Web 连续三循环 + 三场景配色
```

A 与 C 的离线检查不依赖 Gazebo启动，可先完成。每个新修复先跑一次最短相关 cap；单次通过后继续探测下一道门。待几何和整链路已可运行，再做 cap2/cap3 各 3 次稳定性验收。这样无需每改一行就从导航开始重复全序列。

最终连续测试必须用同一源码/配置/二进制，不得把多个 fix 版本的偶然通过拼成连续成功。

## 4. C：先给解锁几何一个确定、可实施的结论

### 4.1 首选：核实触点，解决合同错误

新增一个小型静态检查，放在 `scripts/validate/` 或已有合同测试中：

1. 从展开 URDF 和 YAML 计算右 hook、support、unlock 的相对位置和关节轴。
2. 验证局部触点确实位于对应可见工具的真实作用面；对照已有探针，禁止把工具原点当杆端。
3. 计算可达行程和横向残差下界；输出数值化不可达原因。
4. 将场景 YAML 的夹具局部点与插件的抽屉 link 局部点变换到同一坐标系比较，防止仅改 operator 一侧。
5. 如果只是错误标定，修正真实作用面的坐标，保持 8 mm 解锁门限并复跑 cap4/5。
6. 允许选择右把手本体上真实存在的另一作用面；必须有几何依据，不能把按钮移到空中追随 finger3。

### 4.2 若真实接触模式仍不可达：自行完成仿真机构近似

用户已授权无人值守技术决策，可采用此分支继续软件闭环，不等待询问：

- 保留现有严格物理触点模式及其拒绝行为。
- 在现有 Gazebo cabinet 插件内增加仅 db1 仿真配置可启用的模式；建议名称 `simulated_linkage`。这是待实现选项，不是当前已有参数。
- 模拟“右解锁电缸通过未建模的传动件驱动右把手按钮”的关系；按钮仍位于右把手，工具可见几何保持原样。
- 放行仍需真实解锁关节行程、左右把手几何保持、有效租约和正确阶段；侧缝只宣称已校验伸出/位置。禁止仅凭 Web 目标放闩。
- 通过现有 ROS 通道传递所需阶段证据，优先复用现有消息；只在确实缺少证据时做最小接口扩展并重建所有消费者。
- 仿真配置、Action message、Web 展示和验收摘要明确标识模式。用单独的 `simulation_acceptance` 结论，不能把该分支报告成“finger3 真实端点按到了按钮”。
- 非 Gazebo/未配置模式仍使用严格触点校验。增加越界关节、缺失把手保持、错误租约和默认严格模式的负例。
- 不引入第二套插件，不改全局默认模式，不使用 Gazebo SetEntityState 直接移动抽屉伪造操作。

这条分支的完成含义是“动作时序和抽拉任务链路在明确的机构近似下运行”；真实末端同时接触和真实侧缝受力仍列为未完成。若必须修改对外 physical 字段含义才能实现，保留原字段合同，另加明确模式/证据描述和兼容测试。

## 5. A：先修不会受机械噪声影响的软件问题

### 5.1 采样连续性

在 `cabinet_button_operator.cpp` 的 fix21 接管确认中：

- 每侧保存最近成功样本时间和序号。
- 连续确认必须由多帧新样本覆盖指定时长；缺数超过采样预算即打断连续计时。
- 获取两侧样本后更新判断时钟；同步查询耗时不能被当作有效在位时间。
- 明确区分超界、持续缺数、采样过旧和机械振荡。
- 将时序判断抽成小型纯逻辑，使用合成事件测试：一帧成功后缺数不得通过、单侧持续缺数不得通过、持续回缩拒绝、正常双侧连续样本通过。
- 不以新的确认规则代替实际抖动指标。

### 5.2 cap 验收器

修改已有 `scripts/validate/validate_db1_cap_gates`：

1. 保存原参数；无人值守独占验收要求起始 cap 为 0，结束恢复为 0。
2. finally 覆盖成功、失败、超时、KeyboardInterrupt。若任务仍活动，先取消本脚本提交的任务并等终态，确认收尾后恢复参数。
3. 记录 task_id、场景代次、cap、开始时间；优先从本次任务结果读取 cap 证据。必要时最小补齐 gateway 的结果透传，禁止复用无任务关联的缓存消息。
4. 非目标控件按 control_id 集合比较，并复用 `validate_db1_drawer` 的有效性、运动、位置、状态序列检查。
5. 对所有位置检查有限数，拒绝 NaN/Inf；验证当前而非历史 `in_motion`。
6. 每门输出当前任务的物理/恢复/释放证据；取不到闩锁或 home 证据就标记未验证，不能用 closed 推导。
7. 失败也输出已完成 gates、首个失败和 cleanup 状态，禁止只留一个 error 丢失上下文。
8. 提交前补模拟 HTTP/参数调用的失败恢复测试，不用真实机械运动测试这些分支。

`validate_db1_drawer` 继续使用；将 check-only 改为真正只读，场景不符时明确报告。运动模式可按本无人值守授权切换目标场景，但须记录切换结果。

## 6. B：恢复统一启动

检查文件：

- `run_all.sh`
- `xczs_inspection_robot_nav2/launch/navigation.launch.py`
- `xczs_inspection_robot_nav2/config/nav2_params.yaml`
- `xczs_inspection_robot_bringup/launch/inspection_robot.launch.py`

执行：

1. 保存最近一次启动日志；复用统一入口的进程管理，不用宽泛 pkill 杀所有 ROS/Python 进程。
2. 在同一 ROS domain/RMW 配置下核对 behavior_server 状态与 spin Action server 可发现性；先区分发现延迟、错误命名空间和服务真正缺失。
3. 若确认是发现延迟，可小幅提高现有等待预算，或使用已有 lifecycle manager 做一次有界重试。不要移除导航健康门或禁用失败检测。
4. 若桥接配置影响本地服务发现，修对应配置；不要为了启动切换整个项目通信架构。
5. 达到统一入口健康后先做无运动检查，再进入 cap。
6. 初次可工作后推进下游；发布验收时再验证两次干净启动，记录各阶段耗时。连续失败即进入诊断，不无限重启。

## 7. D：保留压贴目标，降低实际振荡

### 7.1 不回退到已被证伪的接管法

当前 fix15 保持约 0.030 m 压贴参考。投影得出的实际座封值用于几何观测，不直接作为新指令，除非新控制方法已证明不会卸载。

后续 support/unlock 的全关节目标继续传递有效的 hook 保持值。整理变量名或注释，使 `command_ref` 和 `measured_seat` 明确分开；不要再打印“measured 0.030”而实际上只是命令值。

### 7.2 最小诊断实验

复用 `rod_trace_recorder.py`、`rod_seat_geom_probe.py`、`rod_sweep_probe.py`，先读脚本确认是否发运动：其中 sweep 会驱动电缸，不能与业务任务同时运行。

一次只比较一个变量：

1. 工具离柜时，右钩爪低速小行程伸缩是否仍卡滞/跳变？
2. 到把手时，左右 hook 指令、关节实测、物理端点、effort、臂关节和实际接触对象分别怎样变化？
3. 若离柜也有问题，先检查工具内部碰撞或电缸控制稳定性；不要继续偏移柜体目标。
4. 若仅贴把手时振荡，优先检查板缘接触、实际几何和压贴过冲；在几何有效范围内降低接近速度/过冲再比对。
5. 只有证据指向控制环时才调局部阻尼或增益。当前 YAML update_rate=100，旧注释写“有效约 10 Hz”，必须实测仿真时间下控制频率，不能直接沿用注释推导参数。
6. 不因一轮失败提高距离或位移容差。每次改参保留原值、变化原因、波形和对应构建。

速度/振幅指标必须基于实际数据制定：记录峰峰值、连续超差时长和末端距离。持续肉眼抖动或实际杆行程反复回缩不能仅因 cap 返回 success 而验收。

### 7.3 侧缝阶段

保留动态目标与 inset=0.006 的单次成功基线，避免重回固定 0.041/0.042 的硬编码方案。

- 明确所有点同帧；优先沿真实伸缩轴投影求目标，当前按世界 x 计算的实现只适用于已验证方向。
- 指令到位、两侧缝几何、hook 保持、抽屉不被提前拉动都要通过。
- 支撑是当前几何近似，不证明真实支撑力。
- 成功后先探测解锁/抽拉；最终同版 cap2、cap3 各连续 3 次通过。

## 8. E/F：按行程验证耦合、完整关闭与恢复

### 8.1 先修固定参考

先修关闭前钩取合同：显式传入 opening/closing 和阶段起始轨位。打开前检查闭位与闩锁；关闭前检查合法开位及相对起始位置的非预期漂移，不能要求绝对轨位接近 0。关闭前若没有闭位闩提供反力，不能直接复用“顶住锁止面建立压贴力”的前提；应结合现有开位 detent/保持能力选择受控几何接近及有效耦合路径，并验证没有提前推动抽屉。至少增加 closed→open、open→closed、mid→closed 三种阶段策略测试，以及真正超差仍失败的负例。

在进入完整打开/关闭前修正 b1 动态锚点风险：

- 优先选择已有静止柜体/场景 link 作为锚点；没有合适实体时，用实际轨道位置扣除 b1 运动分量。
- 以明确坐标变换处理 Gazebo world 与 planning frame，不只覆盖平移保留不一致旋转。
- 增加闭位、0.10 m、0.30 m 的纯变换测试：同一静止柜体原点不随抽屉位置漂移。
- 关闭任务重新 latch 后，目标不得重复叠加抽屉行程。

### 8.2 运行顺序

| 阶段 | 验收内容 |
| --- | --- |
| cap4 | 解锁电缸伸出读回；严格模式需同帧测触点 |
| cap5 | 所选模式下合法放闩，摘要记录模式 |
| cap6 | 底盘固定，既有柔顺双手耦合保持 3 秒 |
| cap7 | 实测拉 0.03 m，再推回；使用紧位置容差 |
| cap8 | 实测拉 0.10 m，再推回 |
| 完整 open | 从当前状态到 0.30 m 的左右臂全路径可达和碰撞检查，然后实测 |
| 完整 close | 从开位重新接近、抓取、推回、复闩和释放 |

`base_free=false` 保持不变。0.10 m 成功不能代替 0.30 m 成功；源码中仍有约 0.15 m 自碰撞历史注释，应先实算整段路径，避免每次到中途才发现不可达。

若 0.30 m 不可达，自行尝试允许范围内的停靠点/IK 分支/双臂路径优化。仍不可达时保留明确的部分行程演示能力，报告实际最大可验收行程；不得改 open=0.10 然后声称原 0.30 m 需求完成。

### 8.3 按实际状态收尾

不要对所有故障强制套用“detach → 推回”：

- 正常关闭：保持有效耦合推回 → 验证闭位/复闩 → 解除耦合 → 电缸回收 → 安全退臂。
- 打开成功：验证开位稳定 → 正常释放/回收/退臂；保留打开状态供下一次 close。
- 耦合有效且允许恢复：使用当前有效传力关系受控推回，然后释放。
- 接触/租约失效：停止运动并解除危险约束；不得盲目继续推回。记录未回收状态，由已有插件/恢复流程有界处理。
- 电缸未回收：不能继续擦过柜面的退臂轨迹，也不能把 recovery/transport 写成成功。

分别验收：cap6 保持期间取消一次、非零行程取消一次、完整 close 后能重新 open。每次都验证本任务 lease、耦合和参数没有残留。

## 9. G：Web 闭环与配色收尾

现有命令（先 `--help`，使用环境变量提供鉴权，不打印令牌）：

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source install/setup.bash

# 先按第 5 节修补脚本，再执行；一个终端保持统一入口运行。
./run_all.sh --web

# 以下在共享相同 ROS domain/RMW 的另一个终端执行：
scripts/validate/validate_db1_cap_gates --caps 1,2,3,4,5,6,7,8
scripts/validate/validate_db1_cap_gates --caps 2,2,2,3,3,3
scripts/validate/validate_db1_drawer --cycles 3
```

测试命令只是示例顺序；修复阶段按最短相关 cap 执行。标准 cap 编号保持 1～8，不新增一套同义阶段。

- 默认导航走 Web 正式入口。已停靠时可用现有 `--no-navigate` 加快局部定位；最终至少一轮必须包含正式导航。
- 仿真机构近似模式必须传递并输出模式；验证器尚无该参数时，先扩展最小配置/证据读取能力再运行，不能只改文案。
- 对其他控件检查位置、状态、有效性和 transition sequence。它们当前仍可操作，但本次不得自动驱动。
- 完成抽拉后，检查 cabinet_operation、electrical_mezzanine、generator_plant 三场景配色，各保存全景和主要设备截图；只修缺失材质、引用或明确颜色问题。
- 配色工作可在机械分支受阻时独立推进，不得因此把机械任务标记完成。

## 10. 构建、测试与证据管理

### 10.1 构建策略

- 仅改 Python/验证脚本：跑对应测试。
- 改 operator/纯逻辑：构建 control 包并运行相关测试。
- 改插件：构建 gazebo 包并重启本次仿真世界。
- 改接口：重建 interfaces 和其相关消费者；改导航/模型则构建对应包，并验证启动确实加载新配置。
- 最终做一次相关包整体构建、`colcon test` 和统一入口回归。不要用旧 test-result 充当新结果。

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source install/setup.bash
scripts/validate/check_adapter_contract
scripts/validate/check_adapter_contract --instance-id electrical_mezzanine
scripts/validate/check_cabinet_model
scripts/validate/check_scene_config
colcon build --symlink-install --packages-select xczs_inspection_robot_control
colcon test --packages-select xczs_inspection_robot_control
colcon test-result --test-result-base build/xczs_inspection_robot_control --verbose

cd /home/live/work01/jiang
PYTHONPATH=/home/live/work01/jiang${PYTHONPATH:+:$PYTHONPATH} python3 -m pytest -q \
  tests/test_profile_contract.py tests/test_scene_switch.py tests/test_control_gateway_http.py
```

若修改 Nav2，补跑 `xczs_inspection_robot_nav2/test/test_navigation_launch_policy.py`。对本轮新增逻辑写行为测试；不要靠源码字符串出现次数判断实现正确。

### 10.2 无人值守证据

每批实验用独立短名称目录，放在仓库忽略的 `log/` 子目录；简短进度记录放 `docx/`，不要提交巨量采样日志。

每轮记录：

```text
run_id / task_id / cap / acceptance_mode
git HEAD / 当前 diff 摘要或哈希 / binary 时间或哈希
活动场景 / toolset / ROS_DOMAIN_ID / RMW / sim time
startup 是否 ready
hook command_ref / measured_seat / fresh sample count / age / dropout
support target / actual / geometry distance
unlock joint actual / tip-zone distance / selected mode
rail start / peak / final / latch / coupling / lease
Action terminal / recovery / transport / parameter restoration
首个失败门 / cleanup 结果 / 下一项决定
日志和截图路径
```

采样脚本设置运行时长或由本次脚本管理 PID，任务结束关闭采样。只记录需要的话题，禁止持续导出无限 TF trace；用户磁盘空间有限。

## 11. 最终交付与无人值守决策出口

交付报告必须分别列出：

- **已实现**：代码、配置和验证器实际改变了什么。
- **本次已验证**：命令、通过数、连续运行次数、完整行程与模式。
- **仍未完成**：真实末端接触、真实侧缝受力、不可达行程或没有通过的恢复项。
- **自动决策**：为何修标定、保留压贴参考、选择仿真机构近似或停止某个运动分支。
- **可继续入口**：最近成功基线和首个未通过门。

完成标准：

- [ ] 本次构建/测试通过，历史报告与本次结果区分清楚。
- [ ] 统一入口可启动；最新代码确实得到运行。
- [ ] 几何矛盾已有实测/数学结论，并实施了明确可行的分支。
- [ ] fix21 不因缺数或旧样本假通过，实际抖动有观测结论。
- [ ] cap2/cap3 同版各连续三次通过；后续 cap 按所选模式验证。
- [ ] 完整 0.30 m Web 打开/关闭三循环通过；否则明确报告部分完成。
- [ ] 取消、恢复、退臂和调试参数恢复通过。
- [ ] 所有仿真近似在配置、界面/结果和报告中可辨识。
- [ ] 三场景配色有画面证据。
- [ ] 现有工作区改动保留，日志受控，无交互提问或等待用户确认。

**Agent 无需提问。自行选择、实现、验证；遇到不可实现的物理条件，保留证据和真实边界，并继续其他可完成的部分。**
