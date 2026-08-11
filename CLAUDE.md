# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# 项目开发规范
- 优先在github上找类似的成熟的功能进行复用，且学习他们的架构。
- 代码编写必须规范、清晰并保持可维护性。切记要懂得模块化，结构清晰方便复用，可读性要强。
- 优先使用 ROS 2 的 `rclcpp`、`rclpy` 及社区成熟方案，避免重复造轮子。
- 节点间通信统一使用 ROS 2 的 topic、service 或 action，不使用裸 socket。
- 遇到不明确的需求或技术问题时，先向用户确认再实施。
- 引入第三方依赖、插件或复杂方案前，先向用户确认。
- 功能节点优先使用 C++ 开发，仅在 C++ 不适合时使用 Python。
- 无法做到的事情请不要隐瞒，直接告诉我，我会尝试完成。
- 需要新的技术请告诉我，我会去查找对应资料并且引入。

## 文件放置规范

- 可执行脚本放入 `scripts/`。
- 自定义消息放入 `msg/`。
- 启动文件放入 `launch/`。
- 配置文件放入 `config/`。
- 其他文件按照 ROS 2 功能包规范分类存放；无法确定时先询问用户。
- 机器人模型、网格和仿真环境统一放入
  `xczs_inspection_robot_description`。
- 控制节点和统一启动入口统一放入
  `xczs_inspection_robot_control`。
- 不提交 `build/`、`install/`、`log/`、Python 缓存和导出日志等生成文件。

## 项目约定

- 项目说明文档使用中文，命令、话题名称、文件名和代码标识符使用英文。
- ROS 2 启动文件统一使用 Python launch。
- 项目统一启动入口为 run_all.sh
- 修改源码后必须重新编译，并验证统一启动入口能够正常运行。

## 版本管理

- 全程使用 Git 规范管理代码。
- 提交前完成必要的构建、格式和运行检查。
- 提交信息应准确描述本次变更。

## 常用命令

环境：ROS 2 Humble + colcon 工作区，ROOT 为仓库根目录。所有命令先 source 环境再执行。

```bash
# 构建（symlink-install 使 Python 包与 launch 改动即时生效）
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Python 依赖（aiohttp / eclipse-zenoh / Pillow / PyYAML）
pip install -r jiang/requirements.txt
```

### 运行

```bash
./run_all.sh                     # 默认：Web 控制（Gazebo + Nav2 + 控制柜任务）
./run_all.sh --web               # 与无参数相同
./run_all.sh --with-proxy        # 附加 CDR→JSON 代理
./run_all.sh --keyboard          # 键盘调试控制（不启动 Web 控制服务）

# 只做启动预检、不启动任何进程：
XCZS_PREFLIGHT_ONLY=true ./run_all.sh
```

启动后：监控面板 `http://localhost:8080/monitor.html`，SSE 数据 `http://localhost:8001`，传感器流 `http://localhost:8003`。全部路径与开关均可用环境变量覆盖（见 `jiang/start_xczs_bridge.sh` 顶部）；外部机器人栈模式用 `ROBOT_BRINGUP=false GAZEBO_ENABLED=false`。

### 测试

```bash
# C++ 单元测试（ament_gtest）
colcon test --packages-select xczs_inspection_robot_control --event-handlers console_direct+
colcon test-result --verbose

# Python 测试（需已 source workspace；纯逻辑模块不依赖 ROS，ros_node 相关用例需要）
python3 -m pytest jiang/tests/ -q        # 全部 233 个用例
python3 -m pytest jiang/tests/test_inventory.py -q   # 单个文件
```

### 验收脚本（`scripts/`）

- `scripts/check_adapter_contract` — 校验跨文件机器人/导航/柜体 profile 合同（不依赖运行）。
- `scripts/check_cabinet_model` — 校验柜体 Xacro 与 controls 目录的物理合同一致性。
- `scripts/validate_cabinet_web [--exhaustive]` — 经 Web → 任务层 → ROS 2 → Gazebo 的完整柜体操作验收。
- `scripts/validate_cabinet_simulation` — 确定性底层控件检查（不跑 Nav2，需预先就位）。
- `scripts/validate_recording_replay [--runtime]` — 录制/回放 HTTP 合同检查；默认只读，`--runtime` 才被动记录，`--allow-motion` 才允许任务重演。

## 架构概览

这是一个 Gazebo + MoveIt 2 + Nav2 + Web 控制的巡检机器人「控制柜操作」闭环仿真系统。轨迹规划成功不算成功，操作结果必须以 Gazebo 控件关节、按钮触发或档位状态等物理反馈为准。

### 三层适配结构

1. **通用任务层**（`jiang/`，Python）：Web 页面、HTTP/SSE、全局任务互斥、导航调度、取消与结果记录、录制/回放。单入口 `jiang/control_server.py` → `control_gateway/`（runner、ros_node、web_server、task_manager、cabinet_client、recording_manager、task_replay、inventory、robot_adapter、profile_contract 等模块）。其余 Web 基础设施：`jiang/sse_bridge.py`（Zenoh→SSE）、`jiang/zenoh_proxy/`（CDR→JSON）、`jiang/sensor_bridge/`（MJPEG/WebSocket 传感器流）。
2. **机器人适配层**：`xczs_inspection_robot_control/config/cabinet_robot_adapter.yaml` 的 `/**.ros__parameters` 是跨节点接口合同——MoveIt 规划组/工具/底盘帧、Nav2 Action 与 TF、手动关节分组与安全范围、逐控件可达性（`operable`）。Web 网关、底盘路由、手动轨迹路由和柜体 operator 必须读取同一份文件。
3. **场景适配层**：`cabinet_instances.yaml`（实例 inventory）、`cabinet_scene.yaml`（导航工位）、`cabinet_controls.yaml`（控件目录与按钮物理参数）、`control_cabinet.urdf.xacro`（设备几何）。更换机器人/设备/场地只改适配包，不动任务 API 和 Web 页面。

### 运行时组件

`start_xczs_bridge.sh` 依次拉起 Zenoh bridge、SSE 桥、传感器流、HTTP 文件服务器、Web 控制服务，最后执行 `ros2 launch xczs_inspection_robot_control inspection_robot.launch.py`（913 行）按需 include Gazebo、MoveIt、Nav2、机器人 bringup 和各柜体节点。所有配置路径以 launch 参数下发。

C++ 节点位于 `xczs_inspection_robot_control/src/`（`include/` 存放可复用的纯逻辑头文件），包括底盘路由 `base_command_router`、手动轨迹路由、规划场景适配 `cabinet_planning_scene`、位姿权威 `cabinet_pose_authority`、按钮 operator、`operation_lease_coordinator`（防并发租约）、`cabinet_grasp_aggregator`，以及 Gazebo 插件 `src/gazebo/planar_stabilizer_plugin.cpp` 与 `cabinet_state_plugin.cpp`。自定义接口定义于 `action/`、`msg/`、`srv/`。

机器人模型在 `xczs_inspection_robot_description/`（URDF/Xacro、STL 网格、worlds、`config/ros2_controllers.yaml`）；`xczs_inspection_robot_moveit_config/` 与 `xczs_inspection_robot_nav2/` 为对应配置包。

### 多柜体实例

`cabinet_instances.yaml` 每项展开为：Gazebo entity、ROS namespace `/xczs/cabinet/<name>/`、TF `<name>_frame`、操作 Action、控制目录、控件状态 topic 及 reset/grasp service。MoveIt 碰撞对象带实例前缀避免相互覆盖。`cabinet_robot_adapter.yaml` 中 `operable=false` 表示该机器人尚未通过完整物理闭环验收；控件仍在 Web 显示并可提交。底层 Action 会基于实时 Gazebo 状态、TF 和 MoveIt planning scene 串联验证预备、接近、操作、撤回与归位路径，再返回实际失败阶段和路径比例；该分支绝不执行机械臂轨迹、建立抓取或改变柜体状态。

### 关键设计模式

- **不可变 dataclass 合同**：所有 YAML → dataclass → 跨文件校验（`profile_contract.py`），启动时严格校验，畸形配置立即失败而非回退猜测值。
- **Python 测试无 ROS 假模块注入**：纯模块（task_manager、inventory、recording_manager、velocity_profile、task_replay、profile_contract）用 `sys.modules.setdefault("control_gateway", ModuleType(...))` 在测试内注入假包，不依赖 source；`ros_node.py` 直接 import ROS 生成消息，相关测试需先 source workspace。
- **外部副作用可注入**：`popen_factory`、`clock`、`signal_sender`、`action_client_factory` 等使测试可替换真实进程与 ROS 客户端。
- **录制/回放只读互锁**：回放活跃时拒绝一切写操作（409）；回放 topic 全部重映射到 `/xczs/replay/<id>/`；任务重演通过重提交语义任务而非重放运动命令。
- **导航与操作分离**：`/task/navigate` 与 `/task/operate` 是两个独立任务，operate 绝不隐式导航；Nav2 失败时诚实返回 `target_unreachable`。

详细设计与验收边界见 `docs/control_cabinet_simulation.md`（控制柜多实例仿真与 Web 任务）。
