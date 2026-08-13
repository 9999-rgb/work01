# 控制柜多实例仿真与 Web 任务

本项目在 Gazebo、MoveIt 2、Nav2 和 Web 控制端之间提供控制柜物理操作闭环。轨迹规划本身不算成功：操作结果必须来自 Gazebo 控件关节、按钮触发状态或档位状态的反馈。

## 系统分层

- 通用任务层：Web 页面、HTTP/SSE、全局任务互斥、导航调度、取消和结果记录。
- 机器人适配层：`config/cabinet_robot_adapter.yaml`，声明 MoveIt 组、工具、底盘帧、停靠参数和当前机器人的不可达控件。
- 场景适配层：`config/cabinet_instances.yaml`、`cabinet_scene.yaml`、`cabinet_controls.yaml` 和柜体 Xacro。

更换机器人时主要替换机器人模型、MoveIt/Nav2 配置和机器人适配参数；更换场地或柜体时替换实例、地图、模型与控件几何。任务 API 和 Web 页面不需要按实例复制。

`cabinet_robot_adapter.yaml` 的 `/**.ros__parameters` 是跨节点接口合同，Web 网关、底盘路由、手动轨迹路由和柜体 operator 必须读取同一份文件。当前合同包括：

- MoveIt 规划坐标系、规划组、末端执行器、命名安全位姿和规划约束；
- Nav2 Action、lifecycle readiness Service、模式 Service/Topic、地图、定位和底盘 TF；
- 手动底盘输入、Nav2 速度输入、实际底盘输出及两坐标系的平面旋转；
- 机械臂与夹爪关节分组、逐关节安全范围、默认位置、夹爪打开位置和 controller Topic。

Web 通过 `/robot/capabilities` 取得上述手动关节能力并动态生成控件，不再假定机器人一定是“六轴加双夹爪”。新适配包缺字段、关节重复、限位不完整、Topic 名非法或 frame 不一致时会在启动阶段失败，不会退回源码中的猜测值。

柜体目录中的 33 个控件都会在 Web 中显示，并且都可以选择和提交任务。目录字段 `operable` 表示“当前机器人适配包是否已通过该控件的完整安全闭环验证”，不表示设备是否具有该功能。`operable=false` 的控件仍可提交；Web 先按同一逐控件工位完成独立导航，再向 ROS 2 `OperateCabinetControl` Action 发送 Goal。operator 随后读取实时 Gazebo 控件状态、柜体 TF 和 MoveIt planning scene，串联规划预备、接近、操作、撤回与归位路径。该验证分支不会调用轨迹执行、物理抓取或控件写入，因此不会为了探测失败而留下半完成状态。

## 多柜体实例

`config/cabinet_instances.yaml` 是实例 inventory。每项包含唯一 `name` 和有限的 `x/y/z/roll/pitch/yaw`；`pitch` 可省略。统一 launch 在运行时为每项展开一份 Xacro，并创建：

- Gazebo entity：`<name>`
- ROS namespace：`/xczs/cabinet/<name>`
- TF：`odom -> <name>_frame`
- 操作 Action：`/xczs/cabinet/<name>/operate_cabinet_control`
- 控制目录：`/xczs/cabinet/<name>/control_catalog`
- 控件状态：`/xczs/cabinet/<name>/<control_id>/state`
- 位姿有效状态、复位和抓取服务：均位于同一实例 namespace

MoveIt world 中每个碰撞对象带实例前缀，多个 planning-scene 节点不会互相覆盖。每柜独立的 `grasp_active` 由 `cabinet_grasp_aggregator` 做逻辑 OR，再发布全局机器人稳定器所需的话题。

新增或删除柜体只修改 inventory。示例三柜及其导航工位均位于默认 10 m × 10 m 地图内。

更换设备模型时可用 `CABINET_XACRO_PATH` 指向新的 Xacro，同时替换 `CABINET_CONTROLS_PATH` 和 `CABINET_SCENE_PATH`。设备模型需要实现统一物理合同：目录中的每个控件必须有对应关节和状态发布，实例 namespace 下提供 control catalog、state、reset 和 grasp 接口；按钮的行程、刚度和触发阈值必须与 controls 参数一致。设备内部几何可以不同，通用任务 API 不依赖 `box_*` 命名。

## 无升降轴机器人

机器人已移除 `body_arm_lift`、导轨、托架及所有对应的 ros2_control、MoveIt 和 Web 接口。当前默认 profile 是六轴机械臂加两个夹爪关节，但手动轨迹长度和顺序不再固定在源码中：Web 从 `/robot/capabilities` 读取 `arm_joint_names`、`gripper_joint_names` 及逐关节限位，后端再按同一适配参数验证。

移除升降轴后，不伪造无法实现或尚未通过安全验证的动作。当前机器人的运动学与仿真执行筛查结果写在机器人适配参数中：

- 已通过导航、MoveIt 接近、实际按压反馈和安全撤回完整闭环验证：`box_8_button_1`、`box_8_button_2`、`box_10_button_1`、`box_10_button_2`、`box_11_button_1` 和 `box_11_button_2`。
- `box_5_button_1/2`、`box_6_button_1/2` 和 `box_7_button_1/2` 的逐控件导航工位已验证可到达，但当前六轴机械臂的无碰撞接触路径分别只完成 62.1%、66.7%、66.7%、65.6%、69.7% 和 78.1%，均低于 99% 安全门槛，因此当前为 `operable=false`。
- `box_1_button_1/2` 至 `box_4_button_1/2`、`box_1_knob` 至 `box_6_knob` 以及 `cabinet_main_switch` 在移除升降轴后超出当前六轴机械臂从标准工位可覆盖的运动学工作空间。

路径比例未达标不等同于设备没有按钮功能，也不等同于纯 IK 不可达；它表示当前机器人与当前碰撞场景无法通过完整安全操作验收。更换机器人、重新标定工位或优化无碰撞路径后，可逐项复测并在适配参数中重新启用。

`cabinet_rear_door` 已配置独立的柜后精确工位。实测中机器人能抓住把手并将门打开到 90°，但开门后安全脱离路径只完成 65.8%，关门预抓取也无法完成无碰撞规划。为避免留下半完成的柜门状态，在开门、关门和安全脱离全部通过前，它保持 `operable=false`；每次提交仍会从当前仿真状态重新做完整只规划验证，但不会真正抓取或转动柜门。

planning-scene 适配器不会再强制设备必须同时具有一个门和一个门上开关。纯按钮/旋钮设备、仅门设备、固定开关，以及带门和门上开关的当前柜体都使用同一节点；只有目录中实际出现对应类型时才要求其碰撞几何。若开关声明父控件，该父控件必须是目录中的门。

7～11 号旋钮通过了几何可达与 IK 候选筛查，但当前 Gazebo 位置控制后端的刚性跨模型抓取会带动其他控件，产生可观测的非目标误触。隔离抓取碰撞的柔顺方案已能消除串扰，但在当前后端下无法把目标旋钮送到要求档位。因此适配层暂时将这 5 个旋钮的当前机器人能力设为 `operable=false`；控件本身仍可在 Web 提交并取得失败原因。将来更换通过验证的执行控制器或机器人适配包后，可在不改任务 API 的前提下重新启用。

上述未通过当前机器人验收的控件仍出现在 Web 目录中，并携带 `operable=false` 与历史 `unavailable_reason`。提交后的实时只规划验证若在某段失败，任务返回 `planning_failed`，并通过 `diagnostic_stage`、`path_fraction`、`required_fraction` 和 `moveit_error_code` 给出失败阶段、实际完成比例、安全门槛及 MoveIt 原因；若所有运动学路径都通过，任务仍以 `unreachable` 结束，因为没有执行物理接触，不能据此宣称按钮触发、档位到达、无串扰或安全恢复已经通过。结果中的 `validation_performed=true`、`operation_executed=false` 和 `policy_reason` 明确区分“已实时规划验证”和“已实际操作”。理论可达不等于物理闭环成功，只有 `operable=true` 的正常分支才会发送轨迹并以 Gazebo 物理反馈判定成功。

## 按钮力道

按钮参数由一个 `cabinet_controls.yaml` 同时提供给执行器目录和 Gazebo 模型。launch 会在展开每个柜体 URDF 时注入 `button_defaults`，并对 `controls.<id>` 中的单按钮覆盖逐个同步关节行程、弹簧刚度和触发阈值：

- 弹簧刚度：800 N/m
- 物理行程：0～8 mm
- 触发阈值：6 mm
- 默认请求：5.0 N

执行位移按 `位移 = force / 800` 计算。因此线性模型中的触发下限为 4.8 N，最大有效设置为 6.4 N。

MoveIt 到达名义按压位姿后，执行器会读取 Gazebo 的实际按钮关节位移。只有实际行程比请求力对应的目标行程少 0.05 mm 以上时，才在最多额外 1 mm 的适配器限值内做向内小步补偿，最多尝试 3 次；已达标或超程时不做反向修正。这个闭环只补齐接触求解产生的欠行程；成功判定仍只看物理反馈，不会把规划位姿当作已按下。

按钮向内施力段允许执行至少 95% 的近完整笛卡尔候选路径，再在必要时由上述物理反馈只补欠行程；接近和撤回段仍要求至少 99%，因此该容差不会放宽离柜安全路径。

- `4.8 N ≤ force ≤ 6.4 N`：执行物理按压并验证触发和释放。
- `0 N < force < 4.8 N`：仍实际移动探针并安全回撤，终态返回 `insufficient_force`，同时给出请求力、实测峰值位移、估算力和 `button_triggered`。
- Web `/task/operate` 省略 `force` 时使用控件目录的 `default_force`；显式传 `force=0`、负数或非有限数值会在任务接纳前作为无效 HTTP 请求拒绝。
- 直接调用底层 `OperateCabinetControl` Action 时，恰好 `force=0` 是“使用目录默认力”的协议哨兵值；负数、非有限数值或 `force > 6.4 N` 返回 `invalid_force`，不执行超行程动作。
- 旋钮、开关和门接受统一请求结构，但不应用 force。

## 导航和操作的关系

Nav2 只负责把机器人送到指定柜体的操作工位，MoveIt 2 负责机械臂碰撞检查、接近、操作和撤回。当 `/task/navigate` 携带 `control_id` 时，任务层使用机器人适配参数中该控件的 `controls.<control_id>.navigation_station`，并通过最新 `map -> <cabinet>_frame` TF 将柜体局部几何换算为 Nav2 目标。Nav2 每次到达后还会重新解析同一局部工位，并用最新底盘 TF 做交接校验；若 AMCL 在行驶中修正了 Map/Odom，只有底盘距最新工位超过 0.12 m 或 0.15 rad 时才在同一任务、同一超时预算内追加校正，最多两次。超过 0.50 m 或 0.35 rad 的单次定位跳变会明确失败而不会追逐异常目标。这样 Web 粗导航与底层精停不会使用两套位置，也不会因正常的小幅定位噪声产生多余动作。当前 profile 合同要求全部 33 个控件都有显式工位，缺失项会在启动预检中失败，不再静默回退到不相关工位；门上总开关与后门共用柜后工位。只有省略 `control_id` 的旧柜体级导航才使用 `cabinet_scene.yaml/navigation_station` 公共工位，以保持旧客户端兼容。

旧 `PressCabinetButton` Action 仅作为已验收、`operable=true` 按钮的兼容入口，不提供只规划验证和结构化诊断，已不属于完整能力接口。Web、任务录制与新客户端统一使用 `OperateCabinetControl`；若通用 Action 不可用，受限按钮会明确返回 503，而不会回退到旧 Action 后被静态 Goal 拒绝。

导航任务默认按地图坐标系分轴执行。任务层先以目标工位的 X 坐标和当前 Y 坐标构造中间点，完成 X 方向阶段后，再前往最终工位完成 Y 方向阶段；已在容差内的零长度阶段可直接跳过。全向底盘在中间点保持任务开始时的朝向，不再为了下一轴额外转向；最终工位朝向只在最后一段完成，因此整条无障碍路线最多包含一次必要转向。只有前一阶段成功并释放相应导航目标后才会开始后一阶段；任一阶段失败或取消都不会继续。

“分轴”是无障碍正常情况下的路径顺序，不是绕过 Nav2 的直线速度控制。两个阶段都作为 Nav2 目标执行，并继续服从当前代价地图、footprint、碰撞检查和局部规划器。局部规划器利用全向能力沿路径横移，不会为了路径切向反复摆正车头；全局路径仅在目标更新或当前路径失效时重算。如果轴向直线上出现障碍，Nav2 会检测路径失效并重新规划，也可以暂时偏离该轴安全绕行；安全可达性始终高于轨迹的几何纯度。

`/task/operate` **绝不隐式导航**。需要完整流程时，Web 先提交 `/task/navigate`，等待成功后再提交 `/task/operate`。导航失败或取消时不会继续操作。手动方向输入仍可触发现有 Nav2 接管流程；对应导航任务会在 Nav2 确认取消后才释放全局任务锁。

Web 不展示占用地图或全局路径，也不提供任意坐标导航入口。用户只能从 inventory 选择柜体，任务层使用已校验的 `navigation_station` 发送 Nav2 目标。底层仍订阅地图，仅用于工位边界和占用安全检查。

默认场地将三个柜体沿世界坐标 Y 轴排列，并保留 `cabinet_a` 已验证的标准位姿：

| 柜体 | X | Y | Roll | Yaw |
|---|---:|---:|---:|---:|
| `cabinet_a` | 2.00 | 0.33 | π/2 | -π/2 |
| `cabinet_b` | 2.00 | 2.83 | π/2 | -π/2 |
| `cabinet_c` | 2.00 | -2.17 | π/2 | -π/2 |

相邻柜体参考点间隔 2.50 m。柜体在世界 Y 轴方向宽 0.662 m，因此最小本体净距为 1.838 m；该间距可容纳带 0.03 m padding 的 Nav2 footprint，并为侧面工位保留至少约 0.328 m 的物理余量。现有前工位位于 `x=1.07, y=柜体Y-0.331`。后门任务应选择后侧工位，场地按约 `x=3.80, y=柜体Y-0.331, yaw=π` 预留空间，不能从前工位隔着柜体执行。前、后和最外侧工位的完整机器人 footprint 均位于默认 `[-5, 5]` 地图边界内。

同一原则也下沉到默认 operator 配置：`allow_embedded_navigation=false`。绕过 Web 直接向底层 Action 发送 `navigate_to_staging_pose=true` 时，会在任何 MoveIt、Nav2 或租约动作前返回明确的导航失败原因。旧客户端确实需要兼容算法时可以在专用适配包中显式开启，但该旧路径按控件位置计算工位，不属于通用任务合同。

标准 Nav2 `NavigateToPose` 的 ABORTED 结果无法可靠区分“碰撞”与“目标不可达”。没有独立碰撞监视器证据时，系统诚实返回 `target_unreachable`，不会伪造碰撞原因。

目标栅格检查会应用 OccupancyGrid `origin.yaw` 的逆变换。Nav2 报告成功后，网关主动查询最新 `map -> navigation_base_frame` TF，并只接受本次 goal 发送之后取得的位姿，避免用陈旧 AMCL 消息缓存伪造到达。`navigation_base_frame` 直接读取共用的机器人适配参数包，不在 Web 网关中重复硬编码。

## Web API

Gazebo Classic 的 RGB 相机即使关闭图形客户端也仍依赖渲染上下文。未设置
`DISPLAY` 时统一启动入口会禁用 Gazebo GUI，同时明确提示相机不会发布图像；
LiDAR 等不依赖渲染的传感器仍可工作。需要相机流时应提供可用的 X Display，
或由部署环境自行提供虚拟显示服务（项目当前不引入 Xvfb 依赖）。在收到首帧前，
`/sensors/health` 返回 `degraded`，`/camera.jpg` 和 `/camera.mjpg` 返回 HTTP 503，
避免用成功状态或空的 HTTP 200 流掩盖相机不可用。

迁移到 FastAPI 后，**API 参考由 OpenAPI 自动生成**，始终与代码同步：

- Swagger UI（可交互调用）：`http://localhost:8090/docs`
- ReDoc（只读文档）：`http://localhost:8090/redoc`
- OpenAPI JSON：`http://localhost:8090/openapi.json`

设置了 `XCZS_SWAGGER_TOKEN` 时，`/docs` 与 `/redoc` 需携带 `?token=` 访问。

只有 `/auth/login`、`/health` 与监控静态页面公开；控制接口、相机/LiDAR
传感器流和 Zenoh SSE 均需要 JWT Bearer Token（`Authorization: Bearer <token>`，
登录接口为 `POST /auth/login`）。浏览器原生 EventSource/WebSocket 无法携带自定义
Header，因此任务 SSE、Zenoh SSE、MJPEG 和 LiDAR WebSocket 使用 `?token=` 传参；
普通 REST 接口不接受 URL 中的 token。

主要接口（历史表格，字段以 OpenAPI 为准）：

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/cabinets` | inventory 和每柜计算后的工位 |
| GET | `/robot/capabilities` | 当前机器人 frame、手动关节分组与安全范围 |
| GET | `/cabinets/<name>/controls` | 该实例的目录与实时状态 |
| POST | `/task/navigate` | 按柜体导航；可选 `control_id` 用于选择逐控件工位 |
| POST | `/task/operate` | 操作指定实例的控件，不含导航 |
| POST | `/task/reset` | 将 `cabinet` 指定的任务场景归零：底盘回初始位姿、机械臂/夹爪回默认关节，并复位该柜体控件 |
| GET | `/task/<id>/status` | 状态轮询 fallback |
| POST | `/task/<id>/cancel` | 请求取消 |
| GET | `/task/events` | 可重连 SSE 事件流 |
| POST | `/cmd_vel` | 手动底盘调试 |
| POST | `/joint_trajectory` | 按 adapter 声明的关节顺序进行手动调试 |

导航示例：

```bash
curl -sS -X POST http://localhost:8090/task/navigate \
  -H "Authorization: Bearer $XCZS_CONTROL_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a","control_id":"box_11_button_1"}'
```

其中 `control_id` 可省略；省略时导航到该柜体的公共工位。

按钮示例（5 N）：

```bash
curl -sS -X POST http://localhost:8090/task/operate \
  -H "Authorization: Bearer $XCZS_CONTROL_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a","control_id":"box_10_button_1","command":"press","force":5.0}'
```

力度不足验证（4 N）：

```bash
curl -sS -X POST http://localhost:8090/task/operate \
  -H "Authorization: Bearer $XCZS_CONTROL_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a","control_id":"box_10_button_1","command":"press","force":4.0}'
```

将指定柜体的任务场景归零：

```bash
curl -sS -X POST http://localhost:8090/task/reset \
  -H "Authorization: Bearer $XCZS_CONTROL_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a"}'
```

`/task/reset` 只接受非空 `cabinet` 字段，该字段选择要归零的柜体任务场景。一次成功归零必须同时确认：共享机器人底盘已回到配置的初始位姿，机械臂和夹爪已回到机器人 adapter 声明的默认关节位置，且所选柜体的全部控件已回到配置默认状态。其中任一部分未完成或未通过最终状态验证，整个 reset 任务都不得报告 `success`。该 scoped 请求不会改动其他柜体的控件状态。未知柜体在接纳前返回 404；任一归零后端不可用、超时或状态验证失败则作为已接纳任务的 `failed` 终态记录。无 scope 的 `/cabinet/reset` 仅保留为单柜旧客户端兼容接口；需要可录制的完整场景归零时必须使用 `/task/reset`。

有效请求立即返回 `{type}_{timestamp_ms}_{random6}` 格式的 `task_id`。Web `TaskManager` 是第一层全局互斥：所有柜体共用一个活动任务/保留资源槽，导航、操作和场景归零都不能并发。并发 Web 请求返回 HTTP 409 和 `active_task_id`。后端 Action 或 Service 暂不可用、规划失败、力度不足或归零状态验证失败发生在任务接受之后，因此表现为该任务的 `failed` 终态，而不是丢失任务 ID。

导航或操作超时后，Web 立即收到 `navigation_timeout` 或 `operation_timeout` 终态。如底层 Action 尚未确认退出，任务记录保持 `reservation_active=true`，新任务仍返回 409；只有收到底层终态后才释放全局资源，避免超时动作与新动作重叠。

低层 `/xczs/operation_lease` 是第二层全局互斥，用于防住绕过 Web、直接向不同柜体 namespace Action 发送的并发动作。每次实际运动前，operator 必须获取带 owner/lease ID 的 3 秒短租约，每 0.75 秒续租，coordinator 将单次请求上限限为 5 秒。已有持有者时新操作返回 `resource_busy`；续租超时、服务不可用、token/owner 不匹配或租约失效时返回 `lease_lost`，并立即停止 MoveIt、Nav2 和底盘输出。进程崩溃或无法续租后，最多按 TTL 自动释放，不会永久锁死后续操作。

SSE 业务事件为 `task_accepted`、`task_progress` 和 `task_completed`，支持 `Last-Event-ID` 重放与 15 秒心跳。超时后底层资源退出时会额外发布 `task_reservation_released`。相同 JSON 还发布到：

- `/xczs/task/<id>/progress`
- `/xczs/task/<id>/result`（可靠、transient-local）

## 录制、数据回放与任务重演

录制和回放不是控制柜操作的硬依赖，而是用于复现失败、回归验收、演示和迁移新机器人。实现分为两个边界明确的层次：

- **数据记录/回放**使用 ROS 2 官方 `rosbag2` 命令行，不自行实现 bag 格式。回放只用于观察历史状态，不会重新驱动机器人。
- **任务重演**读取记录中提取的语义步骤，再调用现有 `/task/navigate`、`/task/operate` 和 `/task/reset`。它重新经过当前 Nav2、MoveIt、底盘与关节状态验证、Gazebo 柜体物理验证、任务互斥和安全检查，不回灌历史速度或轨迹。

### 记录内容和文件

运行数据默认写入项目根目录的 `recordings/`，可在统一启动前用 `RECORDINGS_ROOT=/data/xczs-recordings` 指向独立数据盘。默认目录已从 Git 中忽略；bag、运行日志和导出数据不得提交。每次记录使用独立且经过校验的 `recording_id` 目录：

```text
recordings/<recording_id>/
├── bag/                  # rosbag2 数据与 metadata.yaml
├── manifest.json         # 可复现性清单和执行结果
├── timeline.jsonl        # Web 任务事件时间线
├── scenario.yaml         # 可重演的 navigate/operate/reset 语义步骤
├── rosbag-record.log
└── rosbag-play.log
```

`manifest.json` 包含记录时的 Git 提交、关键配置文件 SHA-256、柜体 inventory 快照、请求记录的 Topic、实际写入的 Topic、是否包含传感器、持续时间和停止结果。`timeline.jsonl` 按顺序保存 `task_accepted`、`task_progress`、`task_completed` 和资源释放等事件；通过 `/task/reset` 提交的复位与导航、操作一样进入这条时间线。`scenario.yaml` 是从任务接受与终态事件生成的 JSON 兼容文档，而不是从 `/cmd_vel` 反推任务；不含复位步骤的旧格式使用 schema version 1，包含复位步骤时使用 schema version 2。

默认记录以下状态类 Topic；图中不存在的候选 Topic 不会产生数据：

- `/tf`、`/tf_static`、`/clock`；
- `/odom`、`/xczs/odom`、`/amcl_pose`、`/xczs/localization_pose`；
- `/joint_states`、`/xczs/joint_states`；
- 各柜体实例的 control catalog、物理 state、按钮位移/触发状态；
- `/xczs/task/<task_id>/progress` 和 `/xczs/task/<task_id>/result`。

Web 中勾选“包含传感器”后，还会加入相机图像、相机标定、深度图和激光雷达候选 Topic。高频传感器数据可能显著增加文件体积，日常任务回归默认不记录。

录制是被动观察，可以与正常导航、柜体操作或场景归零并存，不会使 Web 进入只读状态。如需可确定复现任务初始状态，应当先开始录制，再为目标柜体显式提交 `/task/reset`；只有底盘初始位姿、机械臂/夹爪默认关节和柜体默认控件状态全部确认后，再开始导航和操作。任务重演不会为旧记录或未记录的柜体隐式补做场景归零，以免篡改原场景意图。

数据回放和任务重演彼此互斥，并且只能在没有活动任务或运动资源时启动。启动回放前，网关还会立即停止手动底盘输出、清除尚未发布的手动关节轨迹，并等待已经被控制器接受的 0.5 秒轨迹窗口连同 0.1 秒安全余量结束，随后才取得回放只读所有权。

### 隔离数据回放

数据回放启动前会读取 bag 的 `metadata.yaml`，校验每个 Topic，并显式将**所有**记录 Topic 重映射到：

```text
/xczs/replay/<recording_id>/<原 Topic 去掉开头斜杠>
```

例如历史 `/tf` 只发布为 `/xczs/replay/<recording_id>/tf`，历史 `/clock` 只发布为 `/xczs/replay/<recording_id>/clock`，不会覆盖实时仿真的时间和 TF。回放进程不在原 Topic 名称下发布任何数据，也不需要改动当前 ROS Domain。包含 `cmd_vel`、关节轨迹、Action goal、controller command 等危险 Topic 的外部 bag 会在启动进程前被拒绝。暂停、恢复、倍率切换和取消均只管理这个隔离的 `ros2 bag play` 进程。

数据回放和任务重演活动期间，Web 显示明确的只读状态，并同时在前端和后端拒绝导航、底盘、机械臂、柜体任务、复位和旧接口命令。不能只依赖按钮置灰：即使客户端绕过页面直接发送 HTTP 请求，后端仍返回 HTTP 409。回放取消、状态查询和只读记录查询保持可用。

该只读互锁覆盖统一 Web 网关及由它发起的 ROS 任务；直接绕过网关向 ROS 2 Topic、Service 或 Action 写入的外部客户端不受 HTTP 互锁管理，属于受信任的低层运维接口。运行数据回放时不得同时启动这类写入客户端；需要让不受信任的 ROS 参与者接入时，应另外使用独立 `ROS_DOMAIN_ID` 或 DDS 网络访问策略。即使出现误接入，rosbag2 数据自身仍只会发布到上述隔离 namespace，不会回灌原始控制 Topic。

每个 rosbag2 子进程运行在独立进程组中。正常停止、取消、倍率切换以及网关收到 `SIGINT`/`SIGTERM` 时，管理器会逐级发送信号，并在确认整个进程组清空后才释放回放所有权。操作系统直接 `SIGKILL` 网关、断电或宿主机崩溃时，用户态程序无法执行任何清理；生产部署应由 systemd、容器或同类进程监管器以 cgroup 为单位终止整组进程。孤立的数据播放器仍只能发布到隔离 namespace，但被动 recorder 可能继续占用磁盘，恢复服务前应先由监管器确认旧进程组已清除。

### 任务重演

场景读取器同时兼容 schema version 1 和 2：

- version 1 是旧格式，只允许 `navigate` 和 `operate`；已有录制不需要迁移。
- version 2 在保留 `navigate` 和 `operate` 的基础上新增 `reset`。只有录制时真实接纳过的复位任务才会生成该步骤。

version 2 场景示例：

```yaml
schema_version: 2
recording_id: recording_...
steps:
  - type: reset
    request:
      cabinet: cabinet_a
  - type: navigate
    request:
      cabinet: cabinet_a
      control_id: box_10_button_1
  - type: operate
    request:
      cabinet: cabinet_a
      control_id: box_10_button_1
      command: press
      force: 5.0
```

`reset` 步骤的 request 只允许 `cabinet`，并通过 scoped `/task/reset` 重新执行完整场景归零：共享机器人底盘回到配置初始位姿，机械臂和夹爪回到 adapter 默认关节，所选柜体控件回到配置默认状态；它不会被扩大成“复位所有柜体控件”。导航步骤中即使历史记录带有绝对 `station`，重演时也会丢弃它，并根据**当前**柜体位姿与机器人 adapter 重新计算安全工位，再按 X 后 Y 的分轴策略执行。操作步骤保留柜体、控件、命令、目标状态/位置和请求力。未知步骤、版本不允许的步骤、额外字段、非法标识、非有限数值、零力/负力或空场景会在产生任何物理副作用前拒绝。

重演按顺序提交语义任务，并等待每一步进入终态且释放资源后才执行下一步。任一步场景归零失败、导航失败、MoveIt 不可达、力度不足、物理反馈不符或被取消，整个重演立即停止并保留当前失败码和原因；记录中的历史“成功”不会绕过当前环境的安全检查。取消重演会同时请求取消当前活动任务；已被后端接纳且无法立即停止的归零子操作会继续保留全局任务资源，直到后端确认它已结束，且不会执行后续场景步骤。任务重演具有真实运动和物理状态副作用，不能把它当作只读数据播放。

### Web API

| 方法 | 路径 | 请求/说明 |
|---|---|---|
| GET | `/recordings` | 记录列表与数量 |
| GET | `/recordings/<recording_id>` | 清单详情和产物可用性 |
| GET | `/recordings/<recording_id>/timeline` | 有界任务事件时间线 |
| GET | `/replay/status` | 录制、数据回放、任务重演及 `read_only` 状态 |
| POST | `/recording/start` | `{"name":"可选安全ID","include_sensors":false}`；ID 仅含小写字母、数字、`_`、`-`，最长 64 字符 |
| POST | `/recording/stop` | `{}` |
| POST | `/replay/data/start` | `{"recording_id":"...","rate":1.0}`，倍率范围 0.1～10.0 |
| POST | `/replay/data/pause` | `{}` |
| POST | `/replay/data/resume` | `{}` |
| POST | `/replay/data/rate` | `{"rate":2.0}` |
| POST | `/replay/task/start` | `{"recording_id":"..."}`，会真实执行任务 |
| POST | `/replay/cancel` | `{}`，取消当前数据回放或任务重演 |

所有变更接口只接受 `Content-Type: application/json` 和表中声明的字段，接受异步请求时返回 HTTP 202。未知记录返回 404，状态冲突返回 409，清单、时间线、场景或 bag 元数据损坏返回 400，`rosbag2` 后端无法启动返回 503；响应正文包含明确错误原因。

### 安全验收脚本

统一入口运行后，先执行不会改变系统状态的合同检查：

```bash
XCZS_CONTROL_TOKEN=<login-token> scripts/validate_recording_replay
```

也可用 `XCZS_CONTROL_USERNAME` 和 `XCZS_CONTROL_PASSWORD` 让脚本先调用 `/auth/login`；浏览器中的当前 token 可直接通过 `XCZS_CONTROL_TOKEN` 传入。

默认模式只读取列表、详情、时间线和状态，并用必然无效的 JSON 验证 POST 路由，不开始录制或回放。显式加入 `--runtime` 后，脚本会创建短时被动记录，检查 Git/配置哈希/inventory/时间线/scenario，暂停隔离数据回放，确认所有回放 Topic 均位于 `/xczs/replay/<recording_id>/`，并用一次零速度请求验证后端只读互锁；它仍不会导航或操作机械臂：

```bash
scripts/validate_recording_replay --runtime
```

只有同时提供 `--allow-motion` 和一个经过人工确认的已有场景，脚本才允许任务重演：

```bash
scripts/validate_recording_replay --runtime --allow-motion \
  --task-recording recording_1234567890_abcdef
```

要验收“4 N 力度不足后立即停止”的失败场景，再加 `--expect-task-failure`；脚本会要求重演终态为 `failed` 且带明确原因。中断运行时，脚本只停止自己创建的 recorder/player 或任务重演，不会接管启动前已有的会话。

## 启动与参数包接口

统一入口：

```bash
./run_all.sh
```

同机或局域网内有其他 ROS 2 / Gazebo 仿真时，建议为本次测试选择独立域：

```bash
ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 \
BRIDGE_TCP_PORT=17447 BRIDGE_REST_PORT=18000 \
CONTROL_HOST=127.0.0.1 CONTROL_PORT=8090 \
XCZS_CONTROL_ORIGINS=http://localhost:8090,http://127.0.0.1:8090 \
./run_all.sh --web
```

统一入口在未设置时使用 `ROS_DOMAIN_ID=42` 和
`ROS_LOCALHOST_ONLY=0`，以兼容局域网中的外部完整机器人栈；需要严格隔离测试时
应像上例一样显式设置 `ROS_LOCALHOST_ONLY=1`。启动摘要会显示最终 DDS 域、
本机限制、Zenoh 发现策略和 `GAZEBO_MASTER_URI`，便于确认所有调试终端是否
位于同一隔离环境。`ROS_LOCALHOST_ONLY=1` 且用户没有提供 `CYCLONEDDS_URI` 时，脚本会
注入 `MaxAutoParticipantIndex=60`，避免完整仿真节点较多时耗尽 CycloneDDS
默认 participant index；已有非空 `CYCLONEDDS_URI` 始终原样保留。

Zenoh Bridge 默认只监听 `127.0.0.1`，并关闭 multicast scouting，不会因
局域网发现而主动连接其他仿真的 Zenoh router。只有确实需要让局域网 Zenoh
客户端直接连接本桥时，才显式设置 `XCZS_ZENOH_LAN_ENABLED=true`；该开关会
同时改为监听 `0.0.0.0` 并恢复 multicast scouting，不能再视为网络隔离模式。
Web 页面是否对局域网开放仍由独立的 `CONTROL_HOST` 控制，不需要为浏览器访问
而开放 Zenoh。

Gazebo Classic 不识别 `ROS_DOMAIN_ID`。因此未显式设置
`GAZEBO_MASTER_URI` 时，统一入口使用
`http://127.0.0.1:$((11345 + ROS_DOMAIN_ID))`，例如域 142 对应端口
11487；`gzserver`、spawn 节点和 Gazebo ROS 插件继承同一 URI。启动本地
Gazebo 前还会按其实际 IPv4 wildcard 监听范围预检该端口。连接外部 Gazebo
时可显式提供 `GAZEBO_MASTER_URI=http://host:port`，脚本不会覆盖，也不会用
本机同号端口的占用情况拒绝远端 master；启动摘要会明确标记已跳过本机预检。

统一入口可以用环境变量替换全部三层适配参数包：

```bash
CABINET_INSTANCES_PATH=/path/to/instances.yaml \
CABINET_CONTROLS_PATH=/path/to/controls.yaml \
CABINET_SCENE_PATH=/path/to/scene.yaml \
CABINET_POSE_PATH=/path/to/pose.yaml \
CABINET_ROBOT_ADAPTER_PATH=/path/to/robot_adapter.yaml \
ROBOT_CONTROL_PATH=/path/to/robot_control.yaml \
SIMULATION_WORLD_PATH=/path/to/world.sdf \
CABINET_XACRO_PATH=/path/to/device.urdf.xacro \
ROBOT_NAME=my_robot \
ROBOT_XACRO_PATH=/path/to/robot.urdf.xacro \
MOVEIT_CONFIG_PACKAGE=my_robot_moveit_config \
MOVEIT_SRDF_PATH=/path/to/robot.srdf \
MOVEIT_KINEMATICS_PATH=/path/to/kinematics.yaml \
MOVEIT_JOINT_LIMITS_PATH=/path/to/joint_limits.yaml \
MOVEIT_CONTROLLERS_PATH=/path/to/moveit_controllers.yaml \
MOVEIT_LAUNCH_PATH=/path/to/move_group.launch.py \
NAV2_LAUNCH_PATH=/path/to/navigation.launch.py \
NAV2_MAP_PATH=/path/to/map.yaml \
NAV2_PARAMS_FILE=/path/to/nav2_params.yaml \
./run_all.sh
```

当前 XCZS 仿真 bringup 只是默认 profile，不是通用任务层的硬依赖。还可用 `USE_SIM_TIME`、`MOVEIT_ENABLED`、`CABINET_BRINGUP`、`SPAWN_CABINET`、`SPAWN_Z` 和 `CABINET_POSE_SOURCE` 控制启动边界。`XCZS_PREFLIGHT_ONLY=true` 会在不启动进程的前提下检查最终组合需要的文件、参数、ROS 包、Web/ROS Python 导入、跨 YAML profile 合同、Zenoh 可执行文件以及预定端口是否可绑定。被关闭的子系统不会强制要求其专属模型文件或 Web 依赖存在。正式启动会继续等待 Web HTTP 就绪，以及 Nav2 Action、Nav2 lifecycle manager 明确报告全部受管导航节点已 active、occupancy map 和需要的柜体 Action 同时可用；可用 `XCZS_STARTUP_TIMEOUT_SEC`、`XCZS_ROBOT_READY_TIMEOUT_SEC` 和 `XCZS_SHUTDOWN_TIMEOUT_SEC` 调整有界等待时间。就绪检查只调用 adapter 的 `navigation_readiness_service`（标准 Nav2 为 `/lifecycle_manager_navigation/is_active`），不会为探测状态发送移动 Goal。外部 Nav2 使用不同 lifecycle manager 名称或命名空间时必须在 adapter 中显式配置该服务。

如果新机器人已有自己的 Gazebo、controller、MoveIt 和 Nav2 bringup，先启动它，再以 `ROBOT_BRINGUP=false GAZEBO_ENABLED=false` 运行本入口。此时本项目仍可加载设备实例、位姿权威、碰撞场景、操作节点和 Web 任务层；新机器人必须提供适配文件中声明的 ROS 2 接口。内置键盘控制在该模式下会明确拒绝启动，不会静默失效；请使用动态 Web 控制或外部机器人自己的手动界面。

若外部 world 已经包含满足物理接口合同的设备实体，使用 `CABINET_BRINGUP=true SPAWN_CABINET=false`：本项目保留目录、位姿、碰撞场景和 operator，但不会重复生成 Gazebo entity。若连设备节点也由外部系统提供，则同时设置 `CABINET_BRINGUP=false SPAWN_CABINET=false`，Web 会按 inventory 中的 namespace 连接外部 Action/Topic/Service。

单独调试 Python 辅助进程时，`sse_bridge.py` 和
`python3 -m sensor_bridge.runner` 默认都只监听 `127.0.0.1`；确需远程访问时再显式传
`--host`。这两个独立调试服务不接入统一 Web 的 JWT 门禁，开放到非回环地址前应另行
设置网络访问控制。`run_xczs_proxy.py` 的 `--control-port` 默认关闭，因为该兼容控制接口没有
JWT；显式传入非零端口会打印安全警告，并且仍只绑定回环地址。正式 Web 控制应使用
`control_server.py`。这些 Zenoh 客户端都固定使用 client mode、关闭 multicast
scouting，并只连接命令行指定的本地 endpoint，避免发现局域网内其他仿真的 router。

迁移到新机器人/新设备时按以下顺序验收：

1. 先独立验证新机器人的 `robot_description`、controller、MoveIt 规划组和 Nav2 `NavigateToPose`；
2. 复制机器人 adapter，填写 frame、Topic/Service/Action、工具 link、关节分组与逐关节限位；
3. 替换设备 Xacro、controls、scene 和 instances，检查目录控件与物理关节一一对应；
4. 先运行与机型无关的 `scripts/check_adapter_contract`；当前 XCZS profile 还应运行 `scripts/check_cabinet_model` 和单元测试，再用统一入口做 launch 冒烟；
5. 重新标定工位与控件能力，只有完整接近、操作、物理反馈和撤回均通过的控件才设为 `operable=true`。

Web 任务网关与仿真 launch 必须使用同一份 instances/scene 参数包。统一入口默认监听 `0.0.0.0:8090`，便于局域网访问；控制路由默认启用 JWT 鉴权。未配置 `XCZS_SECRET_KEY` 时每次启动生成强随机密钥，服务重启后旧 token 失效；需要持久会话时应显式设置至少 32 字符的密钥，JWT 算法固定为 `HS256`，并按需用 `CONTROL_HOST` 收紧监听地址。浏览器 Origin 默认限制为同端口的 localhost 地址；更换静态页面地址时可设置逗号分隔的 `XCZS_CONTROL_ORIGINS`。启动脚本不会停止已有 Zenoh 或 Web 进程；二者占用预定端口时预检会显示可获取的监听进程信息并退出。

## 安全与已知边界

- 柜体位姿默认来自静态仿真真值；位置不固定时必须接入视觉/标记定位并向各实例 `pose_measurement` 发布可信位姿。
- 位姿无效、状态过期、父控件运动中、MoveIt/Nav2 不可用、接触未触发、释放失败和取消都会返回明确失败。
- 柜门操作始终保留门板碰撞体，只在实际抓取期间开放当前把手；门板、把手和门上开关以物理关节状态同步。
- 机器人适配器中的不可达表来自当前模型与标准工位，换机器人后必须重新标定和验证。
- 当前精停实现通过速度 Topic 输出平面 `x/y/yaw` 修正，要求底盘路由能够执行全向速度。仅提供标准 Nav2 但不能横移的差速机器人不能直接宣称支持柜体操作；应提供兼容的精停适配器，或在重新验证前将控件标记为不可操作并返回原因。
- 仿真中的“估算力”由弹簧刚度和物理位移得到，不等同于真实机械臂的力矩传感器闭环。迁移实机时应由硬件力控或末端力传感器替换该适配实现。
