# jiang — 通用任务层（Web/HTTP/SSE/任务/录制回放/资产）

## 总体介绍

三层适配架构的第一层（Python）：把浏览器/API 指令翻译成 ROS 2 action/service 调用并回传
实时状态。本层不实现控制本体，只做编排与适配——Web 服务、任务调度、录制回放、资产导入、
以及把 Zenoh/传感器数据桥接进任务层的传输封装都集中在这里。

## 模块架构

- `control_server.py` —— 进程入口：解析参数、组装子系统、启动 uvicorn（:8090），并把
  `app/` 目录作为 `monitor.html` / `history.html` 的静态服务目录。
- `control_gateway/` —— 核心网关（无 FastAPI 依赖，可独立测试）：
  - 编排：`runner.py`（ROS executor/任务调度/场景切换/录制回放）、`ros_node.py`（Nav2 action、
    map/amcl 订阅、工具切换）、`cabinet_client.py` / `gazebo_client.py`。
  - 任务与录制：`task_manager.py`（互斥调度 + 可重连 SSE）、`recording_manager.py` /
    `task_replay.py`（rosbag2 录制、隔离回放、任务重演）。
  - 配置与资产：`robot_adapter.py` / `profile_contract.py` / `inventory.py` / `scene_catalog.py`
    （适配 YAML 解析）、`asset_library.py` / `asset_manifest.py` / `asset_validators.py` /
    `asset_scene_provider.py`（资产导入/校验/选择）。
- `transport/` —— 外部系统传输桥接层（只做桥接与格式转换，不含业务逻辑）：
  - Zenoh：`sse_bridge.py`（Zenoh→HTTP SSE）、`zenoh_session.py` / `zenoh_key.py`
    （客户端会话构造与 topic key 校验）、`zenoh_proxy/` + `run_xczs_proxy.py`
    （CDR→JSON 代理，浏览器监控面板数据源；入口 `python3 -m transport.run_xczs_proxy`）。
  - 传感器流：`sensor_bridge/`（相机/LiDAR 的 ROS 订阅与浏览器格式转换）。
- `app/` —— FastAPI 应用工厂：`api/`（REST）、`sse/`（复用 `transport.ZenohSource` 的 SSE 路由）、
  `sensors/`（MJPEG + LiDAR WS）、`auth/`（JWT）、`tasks/`、`assets/`（SQLite store）、
  `database/` + `alembic/`（迁移）；`monitor.html` / `history.html` 前端静态页也在这里。
- 其余：`scripts/toolset_supervisor.py`（A/B 工具切换）、`config/zenoh_bridge.json5`、
  `data/`（资产库）、`samples/`、`tests/`、`alembic.ini`、`start_xczs_bridge.sh`（一键启动）。

## 功能介绍

- 柜体任务：`POST /task/navigate|operate|reset`、`GET /task/{id}/status`、`POST /task/{id}/cancel`、
  `GET /task/events`（SSE）、`GET /task/history`、`GET /health`、`GET /cabinets`、`GET /cabinets/{name}/controls`。
- 手动控制：`POST /cmd_vel`、`POST /joint_trajectory`；导航：`GET /navigation/status`、`POST /navigation/mode`、`POST /navigation/takeover`。
- 录制回放：`POST /recording/start|stop`、`GET /recordings`（+`{id}/timeline`）、`POST /replay/data/start|pause|resume|rate`、
  `POST /replay/task/start`、`GET /replay/status`、`POST /replay/cancel`；回放活跃时写操作一律 409。
- 资产/场景：`GET /assets`、`POST /assets/import`、`GET|POST /assets/selection`、`DELETE /assets/{kind}/{name}`、`GET /scenes`、`POST /scene/switch`。
- 机器人：`GET /robot/capabilities`、`GET /robot/toolset/status`、`POST /robot/toolset/switch`（经 `/xczs/toolset/switch` service 与 status topic）。

## 与项目的关系

- 依赖：`xczs_inspection_robot_interfaces`（operate 两个 action）；`control/config/*.yaml`（硬编码路径跨层合同）；
  `description` 的 `control_cabinet.urdf.xacro`；运行时对接 Gazebo 实体服务、Nav2（`/navigate_to_pose`、`/map_server/load_map`、`/amcl_pose`）与 Zenoh 桥。
- 被消费：`run_all.sh` → `start_xczs_bridge.sh` 先校验配置合同与依赖，随后 `ros2 launch`
  `xczs_inspection_robot_bringup inspection_robot.launch.py` 拉起整套 ROS 栈，再依次
  启动 Zenoh bridge、可选 CDR→JSON 代理，最后才起 FastAPI（`control_server.py`）。
- 数据流：`monitor.html` → FastAPI REST/SSE → `ControlServer` → `RosControlNode` / `CabinetClient`
  → ROS 2 action/service → Gazebo/MoveIt/Nav2；反向经 `/xczs/joint_states`、`/amcl_pose` 订阅回报实时状态。
