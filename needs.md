# XCZS 巡操机器人仿真系统 — 需求文档

## 一、多柜体实例

- 同一种柜体类型，N 个实例，放在世界不同位置
- 一份 `cabinet_instances.yaml` 定义每个实例的 name + 位姿（x/y/z/roll/yaw）
- launch 启动时读 YAML，循环生成每个实例的 spawn + pose_authority + button_operator + planning_scene
- 所有实例共用一份 `cabinet_controls.yaml` 和 `cabinet_scene.yaml`
- 每个实例独立 namespace：`/xczs/cabinet/<name>/`，独立 TF 帧：`<name>_frame`
- C++ 源码零改动

## 二、导航任务

下发一个导航任务，让机器人自主走到指定柜子面前。

### 端点

```
POST /task/navigate
```

### 参数

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `cabinet` | string | 是 | 目标柜体名，必须在 inventory 中存在 |

系统根据柜体名从 inventory 查出位姿，反算柜子正前方的操作工位坐标。走 Nav2 自主导航过去。过程中有碰撞检测。到了之后校验实际位姿和目标的偏差。

### 返回结果

成功返回最终位姿、偏差值、耗时。失败返回失败原因。

失败原因：碰撞到障碍物、导航超时、目标不可达、位姿偏差过大、被取消。

---

## 三、操作任务

下发一个操作任务，让机械臂操作指定柜子上的指定控件。

### 端点

```
POST /task/operate
```

### 参数

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `cabinet` | string | 是 | 目标柜体名 |
| `control_id` | string | 是 | 控件 ID（如 `box_1_button_1`） |
| `command` | string | 是 | `press` / `set_state` / `set_position` / `toggle` |
| `force` | number | 否 | 按钮接触力 (N)，默认 5.0。旋钮/开关/门忽略 |
| `target_state` | string | 条件 | `set_state` 时必填 |
| `target_position` | number | 条件 | `set_position` 时必填 |

**操作任务不包含导航。** 系统假设机器人已经在工位。如果操作前需要先到柜子面前，Web 端自己先发导航任务等完成再发操作任务。

### force 参数

- 只对按钮（button 类型）有效
- 力 / 弹簧刚度(800N/m) = 探针位移
- 位移 >= 6mm（press_threshold）→ 按钮触发成功
- 位移 < 6mm → 失败，返回"力度不足"

### 返回结果

成功返回实际位移、按钮是否触发、耗时。失败返回失败原因。

失败原因：力度不足、机械臂不可达、接触检测超时、操作超时、被取消。

---

## 四、command 详解

| command | 适用控件 | 说明 |
|---------|---------|------|
| `press` | button | 按下一键 |
| `set_state` | knob, switch, door | 移动到指定离散档位，需 `target_state` |
| `set_position` | knob, switch, door | 移动到物理档位位置，需 `target_position`（必须是某个 preset） |
| `toggle` | knob, switch, door | 切到下一个档位（循环） |

---

## 五、任务跟踪

- 每个任务下发后立即返回 `task_id`，格式：`{type}_{timestamp_ms}_{random6}`
- SSE 实时推送三类事件：
  - `task_accepted` — 任务被接受
  - `task_progress` — 执行中持续推送（含 phase、progress、业务数据）
  - `task_completed` — 终态（success / failed / canceled）
- HTTP `GET /task/{id}/status` 作为轮询 fallback
- `POST /task/{id}/cancel` 取消任务
- 同时只允许一个任务执行，并发返回 409，附带当前活跃任务 ID
- ROS 2 话题同步发布：`/xczs/task/{id}/progress` 和 `/xczs/task/{id}/result`

---

## 六、Web API 汇总

| 端点 | 方法 | 说明 |
|------|------|------|
| `GET /cabinets` | GET | 列出所有柜体实例 |
| `GET /cabinets/{name}/controls` | GET | 某柜体的控件目录 + 实时物理状态 |
| `POST /task/navigate` | POST | 下发导航任务 |
| `POST /task/operate` | POST | 下发操作任务 |
| `POST /task/{id}/cancel` | POST | 取消任务 |
| `GET /task/{id}/status` | GET | 轮询任务状态/结果 |
| `GET /task/events` | GET | SSE 实时事件流 |
| `POST /cmd_vel` | POST | 保留：手动底盘调试 |
| `POST /joint_trajectory` | POST | 保留：手动机械臂调试 |

---

## 七、文件变更

| 文件 | 动作 | 说明 |
|------|------|------|
| `config/cabinet_instances.yaml` | 新增 | 柜体实例清单 |
| `action/OperateCabinetControl.action` | 修改 | Goal 新增 `float64 force` |
| `src/cabinet_button_operator.cpp` | 修改 | force→位移 + 判定逻辑 |
| `urdf/control_cabinet/components/gazebo.xacro` | 修改 | namespace 参数化 |
| `launch/inspection_robot.launch.py` | 重写 | OpaqueFunction 多实例 |
| `control_gateway/ros_node.py` | 修改 | 多 cabinet client + task ID + SSE |
| `control_gateway/web_server.py` | 修改 | /task/* 端点 |
| `control_server.py` | 修改 | --cabinet-instances 参数 |
| `start_xczs_bridge.sh` | 修改 | 传参 |
| `config/cabinet_controls.yaml` | 不变 | 共用 |
| `config/cabinet_scene.yaml` | 不变 | 共用 |
| 其余 C++ 源码 | 不变 | 参数化完整 |

以上所需参数我后续都会以参数包形式传入，你需要预定好接口。

同时去掉升降梯，实现不了的功能没必要硬实现，可以返回执行失败的结果并且告诉我原因，要可以设置力道，比如说按下按钮在多少牛的力范围内可实现，力度小了就执行失败返回结果。web端也要用相应的更新。