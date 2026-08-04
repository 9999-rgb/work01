# 控制柜自动操作仿真

本项目在 Gazebo、MoveIt 2、Nav2 和 Web 控制端之间提供控制柜自动操作闭环。自动操作必须由机械臂到达目标并触发 Gazebo 中的物理关节状态；仅完成轨迹规划不视为成功。

## 控制件清单

控制目录固定包含 33 个可操作件：

- 20 个按钮：行程 8 mm，超过 6 mm 进入 `pressed`，回到 3 mm 以下进入 `released`。
- 11 个旋钮：仿真三档 `left=-45°`、`center=0°`、`right=45°`，默认 `center`。
- 1 个背门总开关：仿真两档 `off=0°`、`on=90°`，默认 `off`。
- 1 个背门：仿真两档 `closed=0°`、`open=90°`，默认 `closed`。

旋钮档位、总开关角度和背门结构是现有 CAD 缺少业务定义或独立门板资产时采用的明确仿真假设，不代表真实设备的电气参数。柜壳因此使用可碰撞的程序化框体和独立背门；U 形把手保留可视几何及 MoveIt 碰撞几何，原模块网格继续用于前面板外观。

原六轴机械臂的安装高度无法覆盖 2.23 m 柜体，因此仿真模型增加了
`body_arm_lift` 竖直滑台，行程为 0～0.85 m。该轴属于明确的仿真假设，已同时接入 URDF、Gazebo、ros2_control 和 MoveIt；自动动作会由 MoveIt 联合规划，Web 手动调试区也可查看、调节并归零此轴。合成滑台与二号臂在设计包络内允许重叠，SRDF 中对此碰撞对有显式说明。

## 各组件职责

- Gazebo 保存按钮、旋钮、开关和柜门的真实关节状态，并实现按钮弹簧、旋钮档位、开关/柜门锁止、近距工具约束和复位。
- MoveIt 2 负责竖直滑台和六轴机械臂的联合碰撞检查、到位、接近、操作和安全回撤，是自动操作的必需组件。
- Nav2 只负责把移动底盘送到控件所在柜面前的操作工位。Web 中可取消“先导航到操作工位”，此时机器人必须已经停在可达位置。
- Web 端负责选择控件、命令和目标状态，显示物理位置、离散状态、动作阶段，提供取消与复位。

MoveIt 2 是控制柜动作节点的内部执行依赖，不再提供独立的 Web
“MoveIt 2 面板”。用户只需在控制柜面板中提交完整操作；动作节点会自动调用
MoveIt 2，Web 端仍保留通用机械臂手动调试区，供单独调节滑台、机械臂和夹爪使用。

控制柜动作节点在启动时会直接接收本机器人对应的 URDF、SRDF 和运动学参数，不从全局 `robot_description` 话题猜测模型。因此同一 ROS 2 网络中即使还有相机或其他机器人的描述发布者，连续动作也不会在第二次初始化 MoveIt 客户端时误载其他模型。

旋钮、总开关和柜门采用仿真工具锁扣：只有 `button_press_tip` 到达配置的操作点并处于允许距离内，Gazebo 才建立临时跨模型约束。随后关节角度由机械臂执行的旋转或门把手弧线带动；操作接口不会直接写目标关节角。按钮仍由探针真实接触压下。

三个装饰把手部件不参与 Gazebo 接触，门板实体碰撞仍保留，并与固定柜框的四周及深度方向各留出 10 mm 装配间隙。柜门夹持由经过近距校验的固定约束承担。门被抓取、释放并回弹到档位期间，插件会暂时关闭门板、门上开关底座和开关手柄组成的完整运动子树碰撞；确认门已稳定后再统一恢复。这样可避免撤离姿态仍位于开关扫掠体积内时，把大质量门板卡在过中心边界。

自动导航使用距操作点 0.93 m 的工位。Nav2 先完成粗定位；进入 0.15 m
安全接管距离且朝向误差不超过 0.35 rad 后，动作节点取消 Nav2 目标，转由里程计闭环精确停靠。接管距离使用 Nav2 反馈的 `current_pose` 与目标工位在同一坐标系下的二维欧氏直距；`distance_remaining` 仅用于显示，不参与安全判定。坐标系不一致、位置不是有限值或四元数无效时不会累计接管时间；机器人重新远离时会撤销已有许可，取消 Nav2 前后还会通过 TF 复核距离。若底盘已经持续处于安全距离内、但 Nav2 的末端朝向连续
4.0 s 无法满足朝向门限，系统会主动进入同一里程计停靠流程，避免一直卡在
Nav2 旋转阶段。精确停靠最多等待 45.0 s，角速度上限为 0.45 rad/s。该兜底只放宽粗定位的交接条件，最终位置和朝向仍必须通过里程计停靠容差，不能绕过精确停靠或 Gazebo 接触。

AMCL 使用适配全向底盘的 `OmniMotionModel`。DWB 局部控制器允许 X/Y 双向平移，速度范围均为 ±0.25 m/s，加减速度范围均为 ±0.50 m/s²，`vx_samples` 和 `vy_samples` 均为 20；`velocity_smoother` 使用相同的 X/Y 限值。导航超时时动作节点会立即取消 Nav2 并切断导航速度，再进入机械臂恢复流程。

0.93 m 工位为伸直机械臂和柜体碰撞包络保留余量。MoveIt 场景以单条可靠的规划场景差量同步柜框、33 个控件以及门/开关的动态姿态，并且只在动作期间移除当前目标碰撞体。背门总开关随背门一起运动；操作子控件时会解析并等待整条父关节链停稳，再以新的物理状态样本验收，避免把开门过程中的旧状态误判为操作成功。

旋钮、总开关和后门使用 Gazebo 固定约束模拟夹持。探针沿控件表面法向分别保持 0.020 m、0.012 m 和 0.015 m 外偏置。旋钮和总开关由机械臂沿完整关节圆弧带到目标档位，独立停稳 0.30 s 后释放、撤离，再依据新的物理状态样本验收。

后门采用过中心双档机构。机械臂只执行初始位置到目标位置行程的 56%，即开门带到约 50.4°、关门带回约 39.6°后释放。门档仅在近距物理约束已建立时按 0.02 rad 迟滞阈值切换；机械臂引导期间阻尼为 2.0、档位预载刚度为 0.5，释放后锁存目标档位，再切换到 25.0 的阻尼和 60.0 的刚度，由接近门板实际转动惯量临界阻尼的物理回弹完成到 0°或 90°。系统最长等待 90 s，并以释放后的新状态样本和低速度状态验收。

物理抓取期间底盘通过临时世界固定约束模拟驻车制动，机器人平面稳定器暂停位置回写；释放后两者同时恢复。跨模型约束在 ODE 中可能向柜体传播瞬时冲击，因此旋钮、总开关和柜门的档位目标只允许在“该控件本身已被近距抓取”时切换。其他控件即使短时受扰也会被原档位弹簧拉回，不会把冲击锁存成业务状态。

## 启动和 Web 验证

先构建并启动统一入口：

```bash
colcon build --symlink-install
source install/setup.bash
./run_all.sh --web --no-gui
```

浏览器打开 `http://localhost:8080/monitor.html`，进入“控制柜自动操作”：

1. 在按类型分组的目标列表中选择控制件。
2. 选择按下、档位、ON/OFF 或 OPEN/CLOSED。
3. 根据机器人当前位置决定是否勾选“先导航到控制柜操作工位”。
4. 提交后观察动作阶段、当前位置、目标位置和物理状态。
5. 可随时取消；复位仅在没有控制柜动作、Nav2 或普通 MoveIt 动作时允许。

最短验证路径可选择“10 号模块红色按钮”，其控制件 ID 是
`box_10_button_1`，命令选择“按下”。保留“先导航到控制柜操作工位”时会验证
Nav2、里程计精确停靠、MoveIt 2 和 Gazebo 物理按压的完整链路；取消勾选时只验证机器人已在可达工位后的机械臂与物理闭环。成功结果应显示按钮先达到
`pressed`，释放后回到 `released`，动作阶段最终变为成功，活动目标被清空。

取消会停止本次 Nav2/里程计与 MoveIt 2 运动，释放可能已经建立的 Gazebo
工具约束，并尽力安全撤离和收回机械臂。复位会把全部控制件恢复到确定的默认档位；它与控制柜动作互斥，不应用来替代取消正在执行的动作。

## ROS 2 接口

- 通用动作：`/xczs/operate_cabinet_control`，类型 `OperateCabinetControl`。
- 按钮兼容动作：`/xczs/press_cabinet_button`，类型 `PressCabinetButton`。
- 控制目录：`/xczs/cabinet/control_catalog`，可靠、持久化 QoS。
- 单件状态：`/xczs/cabinet/<control_id>/state`，类型 `CabinetControlState`。
- 活动目标：`/xczs/cabinet/active_control`，供 MoveIt 动态碰撞场景临时开放当前目标。
- 安全复位：`/xczs/cabinet/reset_controls`，类型 `std_srvs/Trigger`。
- Gazebo 内部复位：`/xczs/cabinet/reset_physics`。
- Gazebo 工具约束：`/xczs/cabinet/grasp`，类型 `SetCabinetGrasp`。

`SET_POSITION` 仅接受目录中的物理档位位置，不用于任意连续角度控制；Web
界面因此直接展示档位/状态选项。需要按数值调用 API 时，数值应与
`state_positions` 中的某一项一致。

Web API 保留 `/cabinet/press`，并增加：

- `POST /cabinet/operate`
- `POST /cabinet/cancel`
- `POST /cabinet/reset`
- `GET /cabinet/controls`
- `GET /cabinet/status`

例如可直接通过 HTTP 提交并观察上述验证按钮：

```bash
curl -sS -X POST http://localhost:8090/cabinet/operate \
  -H 'Content-Type: application/json' \
  -d '{"control_id":"box_10_button_1","command":"press","navigate_to_staging_pose":true}'
curl -sS http://localhost:8090/cabinet/status
```

需要中止或在终态后恢复全部默认档位时分别调用：

```bash
curl -sS -X POST http://localhost:8090/cabinet/cancel \
  -H 'Content-Type: application/json' -d '{}'
curl -sS -X POST http://localhost:8090/cabinet/reset \
  -H 'Content-Type: application/json' -d '{}'
```

## 自动验收

通过真实 Web HTTP 接口依次验证按钮、旋钮、随门移动的总开关和后门，并核对每次操作后的 Gazebo 物理终态：

```bash
scripts/validate_cabinet_web
```

附加验证非法请求、并发冲突、取消和复位互锁：

```bash
scripts/validate_cabinet_web --contract-tests
```

该脚本要求统一入口以 `--web` 启动，默认保留 Nav2 自动停靠；机器人已在操作工位时可使用 `--no-navigate`。脚本异常或中断时会尝试取消活动动作并恢复全部默认档位。

只检查 33 项目录和所有物理状态话题：

```bash
source install/setup.bash
scripts/validate_cabinet_simulation --inventory-only
```

执行每个控制件的代表动作：

```bash
scripts/validate_cabinet_simulation
```

执行全部按钮，并让旋钮、总开关、柜门遍历后回到默认状态：

```bash
scripts/validate_cabinet_simulation --exhaustive
```

附加验证数值档位、切换、非法数值拒绝、抓取阶段取消、活动目标清理、
复位互锁，以及“开门后操作随门移动的总开关”父子几何链：

```bash
scripts/validate_cabinet_simulation --contract-tests
```

让每次动作包含 Nav2 自动停靠：

```bash
scripts/validate_cabinet_simulation --navigate --exhaustive --contract-tests
```

也可用一个或多个 `--control <control_id>` 做定点回归。

例如只验证 Web 界面默认展示的 10 号模块红色按钮及完整导航链：

```bash
scripts/validate_cabinet_simulation \
  --control box_10_button_1 --navigate --timeout 300
```

`--exhaustive` 一轮共执行 57 次物理动作：20 次按钮按下/释放、11 个旋钮各遍历右/左/中三档、总开关开/关、后门开/关。脚本在每次动作后独立核对 Gazebo 状态、非目标控件未改变以及活动碰撞目标，并在异常或中断时尝试取消动作与复位。

## 仿真边界

控制柜位姿由启动参数和 `odom -> control_cabinet_frame` 静态变换提供，属于仿真真值定位。当前 RGB 相机没有深度、目标识别和手眼标定链路，因此本功能不宣称视觉定位；Gazebo 的近距约束与关节状态闭环也不等同于可直接迁移到真实机械臂的力控。

Ubuntu 22.04 / ROS 2 Humble 当前二进制 MoveIt 2.5.9 在 `move_group` 收到 Ctrl-C 后可能于第三方析构路径退出为 `-11`。统一启动脚本仍会取消动作并清理所有子进程；升级或回溯修补 MoveIt 属于第三方版本变更，需单独评估。
