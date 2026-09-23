# 架构总览

> 状态：已确认 · 版本：v1.1（B 档包结构重组后）· 日期：2026-09-22（v1 基线为 2026-08-28）

本文档描述本仓库的顶层结构、各包职责、三层适配架构与运行时组件。
资产导入 / 校验 / 选择的专项设计见 `asset_import_requirements.md`。

## 1. 顶层结构

```
run_all.sh                       # 统一启动入口（--web / --with-proxy / 预检；--keyboard 已移除）
CLAUDE.md                        # 项目开发规范与文件放置规范
docs/architecture.md             # 本文档
docs/asset_import_requirements.md# 场景/柜体资产导入需求
docs/build/                      # 遗留 colcon 构建树（约 300 MB，已 gitignore，可删）
jiang/                           # 通用任务层（Web/HTTP/SSE/任务管理/录制回放，Python）
scripts/                         # 可执行脚本（见 §4 分桶）
xczs_inspection_robot_interfaces/# NEW  msg/srv/action 自定义接口（纯接口包）
xczs_inspection_robot_gazebo/    # NEW  Gazebo 插件 + worlds（仿真底座包）
xczs_inspection_robot_bringup/   # NEW  统一启动入口（launch + 配套脚本，无代码）
xczs_inspection_robot_control/   #      控制节点 + 纯逻辑头 + 场景适配 YAML
xczs_inspection_robot_description#      机器人模型（URDF/Xacro、meshes）
xczs_inspection_robot_moveit_config / _nav2   # MoveIt / Nav2 配置（标准，未动）
model/、仿真场景20260831/、recordings/  # 数据目录（非 ROS 包）
                                 #   model/ 与 仿真场景20260831/ 带 COLCON_IGNORE；
                                 #   recordings/ 无 COLCON_IGNORE，已在 .gitignore
```

工作区根即 colcon 包根（无 `src/`）。新包平放在根目录，colcon 自动发现。

## 2. 七个 ROS 包（一包一职责）

B 档重组把原先「接口 + 节点 + 插件 + 巨型 launch + 配置全塞 control」的混乱结构，
拆成职责互不重叠的七个包。依赖方向单向向下：`interfaces ← control ← gazebo`，
`bringup` 只组装、exec_depend 全部。

| 包 | 职责 | 内容 | 不该放什么 |
|---|---|---|---|
| `xczs_inspection_robot_interfaces` | 自定义接口 | `action/` `msg/` `srv/`（11 个，含 `OperateCabinetControl`、`ManageOperationLease`、`CabinetControl*`） | 任何实现 |
| `xczs_inspection_robot_gazebo` | 仿真底座 | 3 个 Gazebo 插件（`planar_stabilizer` / `cabinet_state` / `ros_global_args_guard`，lib 名不变）+ `worlds/` | 节点、launch、接口 |
| `xczs_inspection_robot_bringup` | 统一启动入口 | `launch/inspection_robot.launch.py` + `scripts/verify_initial_pose.py` | 节点实现、配置合同 |
| `xczs_inspection_robot_control` | 控制节点 | `src/` 节点（base_command_router / 手动轨迹路由 / cabinet_planning_scene / cabinet_pose_authority / 按钮 operator / operation_lease_coordinator / cabinet_grasp_aggregator）+ `include/` 纯逻辑头 + `config/` 场景适配 YAML + `test/` | 接口、插件、launch、Python 运维脚本 |
| `xczs_inspection_robot_description` | 机器人模型 | `urdf/` `meshes/` `config/`（ros2_controllers_toolset_{A,B}.yaml、initial_positions.yaml） | worlds（已移 gazebo） |
| `xczs_inspection_robot_moveit_config` | MoveIt 配置 | 标准布局 | — |
| `xczs_inspection_robot_nav2` | Nav2 配置 | 标准布局 | — |

### 关键约束（物理不可改，只允许在配置/参数/operator 代码层修正）

- **插件 lib 名必须保持** `libxczs_planar_stabilizer.so` / `libxczs_cabinet_state.so`
  / `libxczs_ros_global_args_guard.so`——world/xacro/样例/`model/` 副本按文件名引用。
  新包 `add_library` 目标名不变、`LIBRARY DESTINATION lib`。
- **`cabinet_state_plugin.cpp` 依赖 control 的本地策略头**
  `xczs_inspection_robot_control/cabinet_grasp_safety_policy.hpp` → gazebo 包
  `<depend>xczs_inspection_robot_control</depend>`，control 保留
  `ament_export_include_directories(include)`。
- **插件加载机制**：不设 `GAZEBO_PLUGIN_PATH`、不写 `plugin_path`。插件靠 colcon
  `install/setup.bash` 把各包 `lib/` 注入 `LD_LIBRARY_PATH`，gzserver 对裸插件名
  dlopen 时按 `LD_LIBRARY_PATH` 搜索。gazebo/lib 构建后同样进路径，移动安全。
- **场景适配 YAML 留在 control/config**：7 个 YAML（instances/scene/controls/
  adapter/pose 等）被 Python 层与验收脚本按硬编码路径读取，是跨层合同。

## 3. 三层适配架构

1. **通用任务层**（`jiang/`，Python）：Web 页面、HTTP/SSE、全局任务互斥、导航调度、
   取消与结果记录、录制/回放、资产导入与选择。单入口
   `jiang/control_server.py` → `control_gateway/`（runner、ros_node、web_server、
   task_manager、cabinet_client、recording_manager、task_replay、inventory、
   robot_adapter、profile_contract、asset_manifest/asset_library/asset_validators）。
   其余 Web 基础设施：`jiang/transport/`（Zenoh→SSE `sse_bridge.py`、CDR→JSON
   `zenoh_proxy/` + `run_xczs_proxy.py`、MJPEG/WebSocket 传感器流 `sensor_bridge/`）、
   `jiang/scripts/toolset_supervisor.py`、`jiang/config/zenoh_bridge.json5`。
   监控面板的数据源实际是 `app/sse/router.py` 的 `/sse`（ZenohSource + 本地解码），
   不是 CDR→JSON 代理：代理回写的 `{topic}/json` 在仓内无消费者，且默认启动不拉起它
   （仅 `run_all.sh --with-proxy` 时起）。
2. **机器人适配层**：`xczs_inspection_robot_control/config/cabinet_robot_adapter.yaml`
   的 `/**.ros__parameters` 是跨节点接口合同——MoveIt 规划组/工具/底盘帧、Nav2
   Action 与 TF、手动关节分组与安全范围、逐控件可达性（`operable`）。Web 网关、
   底盘路由、手动轨迹路由和柜体 operator 读取同一份文件。
3. **场景适配层**：`cabinet_instances.yaml`（实例 inventory）、`cabinet_scene.yaml`
   （导航工位）、`cabinet_controls.yaml`（控件目录与按钮物理参数）、
   `control_cabinet.urdf.xacro`（设备几何）。更换机器人/设备/场地只改适配包，
   不动任务 API 和 Web 页面。

**运行时的实际来源**（同一份 YAML 常有两处副本，启动脚本按资产选择注入其一，实测取
活栈进程环境）：柜体五件套（controls / scene / pose / robot_adapter / xacro）与场景
catalog 取自资产库 `jiang/data/assets/{cabinet/demo_cabinet,scene/<name>}/`；`cabinet_instances.yaml`
仍取 `control/config/`；而 operator 停靠与操作循环真正加载的**实例适配器**是
`config/scene_controls/<scene>_adapter.yaml`（经 `cabinet_instances.yaml` 的
`adapter_config` 传入）——含 `docking_*` 等运行参数，不在资产库副本里。
同步只有一条自动通道：`generate_scene_maps.py` 把地图镜像进资产库同场景副本；柜体五件套与
`scene_controls/` 无镜像、无比对，改源本不会自动流到运行时那份。

## 4. scripts/ 分桶

`scripts/` 只放可执行脚本，按用途分两个桶：

- **`scripts/validate/`** —— 验收/校验脚本（面向用户与 CI 的「门」）：
  `check_adapter_contract`（跨文件 profile 合同）、`check_cabinet_model`（柜体
  物理合同，`--asset` 资产模式）、`check_scene_config`（场景地图/Nav2 参数）、
  `validate_cabinet_simulation`（确定性底层控件检查）、`validate_cabinet_web`
  （Web→任务层→ROS→Gazebo 完整闭环）、`validate_recording_replay`（录制/回放
  合同）。
- **`scripts/tools/`** —— 开发工具脚本：
  `xczs_import_asset`（资产 CLI 导入，bridge 启动时也会调用）、
  `cabinet_validation_targets.py`（可测试性目标选择，被 validate 脚本与测试复用）、
  `classify_controls.py`（控件分类）、`generate_scene_maps.py`（场景地图生成，
  写完镜像进资产库同场景副本——运行时读的是资产库那份，不镜像会留下静默过期快照）、
  `package_asset_samples.sh`（样例资产打包）、`preposition_base.py`（预置位）、
  `taught_sequence_ros.py` 与 `xczs_controllers.py`（教学序列与控制器直驱，均被
  目录外调用，属生产链路而非一次性探针）。

两个桶内的脚本靠 `parents[N]` 相对自身定位工作区根，再把 `jiang/` 加入
`sys.path`；移动脚本时 `parents` 深度必须同步。约定上按 `python3 <path>` 调用，
因此 `validate/` 下有 7 个脚本（`check_drawer_motion.py`、`record_drawer_motion.py`、
`generator_contact_audit.py` 等）没有 +x 位，直接 `./` 执行会失败。

## 5. 运行时组件与启动链

`jiang/start_xczs_bridge.sh` 先校验配置合同、Python/ROS 依赖、Zenoh 二进制和端口，
再执行 `ros2 launch xczs_inspection_robot_bringup inspection_robot.launch.py` 拉起 ROS 栈
（世界 owner + 工具集监督器 + 机器人子栈）按需 include Gazebo、MoveIt、Nav2、机器人
bringup 和各柜体节点；随后依次拉起 Zenoh bridge、可选 CDR→JSON 代理，最后起统一的
FastAPI Web 服务。SSE、相机、雷达和静态页已合并
到同一 FastAPI 进程。脚本只终止本次注册的进程组；预检模式（`XCZS_PREFLIGHT_ONLY=true`）
只验证配置，端口占用以 ⚠ 摘要提示而非硬性失败。

launch 的承载性启动链（spawn_robot → controllers → tool_controllers → 位姿校验 →
放行 router/moveit/nav2）跨子 launch 靠 `OnProcessExit` 的**事件对象身份**串联
（`target_action=<对象>`，不是按名字匹配）。启动期的时序门只有两处，都不在租约协调器里：
launch 把 base/trajectory router、move_group、Nav2 挡在位姿校验之后，operator 侧再
`wait_for_service` 等租约服务。租约协调器自身没有就绪输入——构造即建服务并授租
（`operation_lease_coordinator.cpp` 只校验 `service_name` 与 `maximum_lease_duration`），
因此其它客户端（如 `scripts/tools/*.py`）可以在栈起来前先取走全局租约，真正的 operator
上来时会先吃一次 `RESOURCE_BUSY`，租约到期（≤5 s）后自愈。

## 6. 关键设计模式

- **不可变 dataclass 合同**：所有 YAML → dataclass → 跨文件校验
  （`profile_contract.py`），启动时严格校验，畸形配置立即失败而非回退猜测值。
- **Python 测试无 ROS 假模块注入**：纯模块用 `sys.modules.setdefault("control_gateway",
  ModuleType(...))` 在测试内注入假包，不依赖 source；`ros_node.py` 直接 import ROS
  生成消息，相关测试需先 source workspace。注意 `control_gateway/__init__.py` 自
  3cfc713（2026-08-09）起已是 PEP 562 惰性 `__getattr__`，该注入不再是必需手段；
  代价是全会话里没走过真包的惰性导出，`from control_gateway import ControlServer`
  这条生产路径（`control_server.py`、`run_xczs_proxy.py`）在测试中没有覆盖。
- **外部副作用可注入**：`popen_factory`、`clock`、`signal_sender`、
  `action_client_factory` 等使测试可替换真实进程与 ROS 客户端。
- **录制/回放只读互锁**：回放活跃时拒绝一切写操作（409）；回放 topic 全部重映射
  到 `/xczs/replay/<id>/`；任务重演通过重提交语义任务而非重放运动命令。
- **导航与操作分离**：`/task/navigate` 与 `/task/operate` 是两个独立任务，operate
  绝不隐式导航；Nav2 失败时诚实返回 `target_unreachable`。
- **轨迹规划成功不算成功**：操作结果必须以 Gazebo 控件关节、按钮触发或档位状态等
  物理反馈为准。
- **拓扑切换必须显式携带它依赖的上下文**：换末端套装会重启整棵机器人子栈，而子栈的启动
  参数来自启动那一刻——先切场景、再切套装，会把场景换回启动时那个（Nav2 因此载入错图、
  卡在配置阶段、被就绪门回滚）。现由任务层把**活动场景**及其 scenes.yaml 随切换请求一并
  下发，并覆盖启动期烘死的值。详见 `toolset_scene_switch_verification.md`。
- **物理真值定位重播种（仿真专属）**：长距离行驶会积累里程计漂移（实测一次 17 m 行程
  的末端信念偏离真值 7.9 m），Nav2 据此算不出计划而中止导航。导航任务开始、导航途中
  每 2 s、操作任务开始前三处按 Gazebo 真值校准 AMCL 信念，只在校准量超阈值时才发
  `/initialpose`。**播种值取机器人此刻的真实位姿而非工位目标点**——机器人真的不在工位
  时，工位门仍如实失败，不会把错位掩盖成「已到站」。与既有的「物理锚定」同源。
  详见 `localization_realign_verification.md`。
