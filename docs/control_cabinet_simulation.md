# 控制柜多实例仿真与 Web 任务

本项目在 Gazebo、MoveIt 2、Nav2 和 Web 控制端之间提供控制柜物理操作闭环。轨迹规划本身不算成功：操作结果必须来自 Gazebo 控件关节、按钮触发状态或档位状态的反馈。

## 系统分层

- 通用任务层：Web 页面、HTTP/SSE、全局任务互斥、导航调度、取消和结果记录。
- 机器人适配层：`config/cabinet_robot_adapter.yaml`，声明 MoveIt 组、工具、底盘帧、停靠参数和当前机器人的不可达控件。
- 场景适配层：`config/cabinet_instances.yaml`、`cabinet_scene.yaml`、`cabinet_controls.yaml` 和柜体 Xacro。

更换机器人时主要替换机器人模型、MoveIt/Nav2 配置和机器人适配参数；更换场地或柜体时替换实例、地图、模型与控件几何。任务 API 和 Web 页面不需要按实例复制。

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

## 无升降轴机器人

机器人已移除 `body_arm_lift`、导轨、托架及所有对应的 ros2_control、MoveIt 和 Web 接口。当前手动轨迹固定为 8 个数：六轴机械臂加两个夹爪关节。

移除升降轴后，不伪造无法实现或尚未通过安全验证的动作。当前标准工位的运动学与仿真执行筛查结果写在机器人适配参数中：

- 已通过完整运行验证：仅 `box_10_button_1`。
- 不可操作：除 `box_10_button_1` 外的其余 32 个控件；目录仍会返回每项的能力状态和失败原因。

`box_5_button_1/2`、`box_6_button_1/2`、`box_7_button_1/2`、`box_8_button_1/2`、`box_10_button_2` 和 `box_11_button_1/2` 曾是几何候选，但在 `cabinet_a` 标准工位逐项执行完整 MoveIt 接近、按压和撤回流程时，无碰撞路径运行验证均未通过。它们因此按安全能力降级为 `operable=false`；这不等同于“纯运动学不可达”，后续重新标定并通过同一套闭环验收后可在适配参数中逐项重新启用。

`cabinet_rear_door` 的把手位于柜体背面，而当前目录只配置了正面标准工位。从正面无法绕过柜体实体到达把手；当前三柜布局也没有无碰撞、地图内的安全后侧工位，所以适配层将它标记为不可操作并返回具体原因。后续增加按控件选择的后侧 Nav2 工位并重新验证场地后，可以只通过新的适配参数包重新启用，不需要删除柜门模型或改 Web 协议。

7～11 号旋钮通过了几何可达与 IK 候选筛查，但当前 Gazebo 位置控制后端的刚性跨模型抓取会带动其他控件，产生可观测的非目标误触。隔离抓取碰撞的柔顺方案已能消除串扰，但在当前后端下无法把目标旋钮送到要求档位。因此适配层暂时将这 5 个旋钮标记为不可操作；将来更换通过验证的执行控制器或机器人适配包后，可在不改任务 API 的前提下重新启用。

上述不可操作控件仍出现在目录中，并携带 `operable=false` 与具体 `unavailable_reason`。提交后任务统一以适配层能力失败码 `unreachable` 结束，不会启动机械臂；该码既包含运动学不可达，也包含尚未通过安全验证的机器人能力。理论可达不等于必然成功；MoveIt 碰撞、当前姿态或环境变化仍可能使具体一次动作失败。

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

Nav2 只负责把机器人送到由柜体完整 RPY 和 `cabinet_scene.yaml/navigation_station` 计算出的工位。MoveIt 2 负责机械臂碰撞检查、接近、操作和撤回。

`/task/operate` **绝不隐式导航**。需要完整流程时，Web 先提交 `/task/navigate`，等待成功后再提交 `/task/operate`。导航失败或取消时不会继续操作。手动方向输入仍可触发现有 Nav2 接管流程；对应导航任务会在 Nav2 确认取消后才释放全局任务锁。

标准 Nav2 `NavigateToPose` 的 ABORTED 结果无法可靠区分“碰撞”与“目标不可达”。没有独立碰撞监视器证据时，系统诚实返回 `target_unreachable`，不会伪造碰撞原因。

目标栅格检查会应用 OccupancyGrid `origin.yaw` 的逆变换。Nav2 报告成功后，网关主动查询最新 `map -> navigation_base_frame` TF，并只接受本次 goal 发送之后取得的位姿，避免用陈旧 AMCL 消息缓存伪造到达。`navigation_base_frame` 直接读取共用的机器人适配参数包，不在 Web 网关中重复硬编码。

## Web API

主要接口：

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/cabinets` | inventory 和每柜计算后的工位 |
| GET | `/cabinets/<name>/controls` | 该实例的目录与实时状态 |
| POST | `/task/navigate` | 按柜体名导航 |
| POST | `/task/operate` | 操作指定实例的控件，不含导航 |
| GET | `/task/<id>/status` | 状态轮询 fallback |
| POST | `/task/<id>/cancel` | 请求取消 |
| GET | `/task/events` | 可重连 SSE 事件流 |
| POST | `/cmd_vel` | 手动底盘调试 |
| POST | `/joint_trajectory` | 六轴机械臂和夹爪调试 |

导航示例：

```bash
curl -sS -X POST http://localhost:8090/task/navigate \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a"}'
```

按钮示例（5 N）：

```bash
curl -sS -X POST http://localhost:8090/task/operate \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a","control_id":"box_10_button_1","command":"press","force":5.0}'
```

力度不足验证（4 N）：

```bash
curl -sS -X POST http://localhost:8090/task/operate \
  -H 'Content-Type: application/json' \
  -d '{"cabinet":"cabinet_a","control_id":"box_10_button_1","command":"press","force":4.0}'
```

有效请求立即返回 `{type}_{timestamp_ms}_{random6}` 格式的 `task_id`。Web `TaskManager` 是第一层全局互斥：所有柜体共用一个活动任务/保留资源槽，导航和操作都不能并发。并发 Web 请求返回 HTTP 409 和 `active_task_id`。后端 Action 暂不可用、规划失败或力度不足发生在任务接受之后，因此表现为该任务的 `failed` 终态，而不是丢失任务 ID。

导航或操作超时后，Web 立即收到 `navigation_timeout` 或 `operation_timeout` 终态。如底层 Action 尚未确认退出，任务记录保持 `reservation_active=true`，新任务仍返回 409；只有收到底层终态后才释放全局资源，避免超时动作与新动作重叠。

低层 `/xczs/operation_lease` 是第二层全局互斥，用于防住绕过 Web、直接向不同柜体 namespace Action 发送的并发动作。每次实际运动前，operator 必须获取带 owner/lease ID 的 3 秒短租约，每 0.75 秒续租，coordinator 将单次请求上限限为 5 秒。已有持有者时新操作返回 `resource_busy`；续租超时、服务不可用、token/owner 不匹配或租约失效时返回 `lease_lost`，并立即停止 MoveIt、Nav2 和底盘输出。进程崩溃或无法续租后，最多按 TTL 自动释放，不会永久锁死后续操作。

SSE 业务事件为 `task_accepted`、`task_progress` 和 `task_completed`，支持 `Last-Event-ID` 重放与 15 秒心跳。超时后底层资源退出时会额外发布 `task_reservation_released`。相同 JSON 还发布到：

- `/xczs/task/<id>/progress`
- `/xczs/task/<id>/result`（可靠、transient-local）

## 启动与参数包接口

统一入口：

```bash
./run_all.sh --web
```

统一入口可以用环境变量替换全部三层适配参数包：

```bash
CABINET_INSTANCES_PATH=/path/to/instances.yaml \
CABINET_CONTROLS_PATH=/path/to/controls.yaml \
CABINET_SCENE_PATH=/path/to/scene.yaml \
CABINET_POSE_PATH=/path/to/pose.yaml \
CABINET_ROBOT_ADAPTER_PATH=/path/to/robot_adapter.yaml \
./run_all.sh --web
```

Web 任务网关与仿真 launch 必须使用同一份 instances/scene 参数包。控制 API 默认且强制绑定 loopback，不会把未鉴权的机器人控制端口暴露到局域网。浏览器 Origin 默认限制为当前 monitor 端口；更换静态页面地址时可设置逗号分隔的 `XCZS_CONTROL_ORIGINS`。

## 安全与已知边界

- 柜体位姿默认来自静态仿真真值；位置不固定时必须接入视觉/标记定位并向各实例 `pose_measurement` 发布可信位姿。
- 位姿无效、状态过期、父控件运动中、MoveIt/Nav2 不可用、接触未触发、释放失败和取消都会返回明确失败。
- 柜门操作始终保留门板碰撞体，只在实际抓取期间开放当前把手；门板、把手和门上开关以物理关节状态同步。
- 机器人适配器中的不可达表来自当前模型与标准工位，换机器人后必须重新标定和验证。
- 仿真中的“估算力”由弹簧刚度和物理位移得到，不等同于真实机械臂的力矩传感器闭环。迁移实机时应由硬件力控或末端力传感器替换该适配实现。
