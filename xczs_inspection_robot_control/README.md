# xczs_inspection_robot_control

## 总体介绍

巡检机器人控制核心：9 个 C++ 节点 + 无 ROS 依赖的纯逻辑头 + 7 份跨层合同 YAML。
把「点一下按钮」翻译成「MoveIt 规划 + 操作租约互斥 + Gazebo 物理确认」的控制闭环，
覆盖手动控制、指令路由、柜体自动操作、操作互斥与场景碰撞管理。

## 模块架构

- `src/` 9 个可执行节点（CMakeLists.txt 定义，install 到 `lib/xczs_inspection_robot_control`）：
  - 手动控制：`keyboard_teleop`（节点 `xczs_keyboard_teleop`）、`inspection_robot_gui`（Qt5 GUI）。
  - 指令路由：`base_command_router`、`legacy_trajectory_router` 负责手动/导航指令与底层控制器的转发。
  - 柜体操作：`cabinet_button_operator`（MoveIt 驱动的按钮/旋钮操作）、`operation_lease_coordinator`（操作租约互斥）、`cabinet_grasp_aggregator`（grasp 信号汇聚）。
  - 场景支撑：`cabinet_planning_scene`（MoveIt 碰撞对象）、`cabinet_pose_authority`（柜体位姿权威）。
- `include/xczs_inspection_robot_control/`：无 ROS 依赖的纯逻辑头，可独立测试并被多节点复用——如 `operation_lease_state.hpp`（租约状态机）、`operation_validation_policy.hpp`（操作前基座运动准备边界）、`jerk_limited_velocity_profile.hpp`、`cabinet_grasp_safety_policy.hpp`（被 gazebo 包插件引用）。
- `config/`：7 个场景适配 YAML（`cabinet_robot_adapter.yaml`、`cabinet_controls.yaml`、`cabinet_instances.yaml`、`cabinet_scene.yaml`、`cabinet_pose.yaml`、`robot_control.yaml`、`scenes.yaml`），是跨层合同。
- `test/`：7 个 GTest（策略/状态/运动学/位姿帧合同）+ 6 个 pytest（launch 启动策略、抓手合同、工具集验证）。

## 功能介绍

- 底盘控制：`base_command_router` 合并手动 `/xczs/manual_cmd_vel` 与导航 `/cmd_vel`，输出 `/xczs/cmd_vel`；用 `/xczs/set_navigation_mode` 服务切换模式（`cabinet_robot_adapter.yaml`）。
- 关节控制：`keyboard_teleop` / `inspection_robot_gui` 发布 `/xczs/joint_trajectory`；`legacy_trajectory_router` 按组拆到 `/xczs/arm_controller/joint_trajectory`、`/xczs/gripper_controller/joint_trajectory`。
- 柜体自动操作：`cabinet_button_operator` 暴露 `OperateCabinetControl` / `PressCabinetButton` action，发布 `control_catalog` / `active_control`，调 `grasp` 服务；结果以 Gazebo 物理反馈为准，不把「规划成功」当成功。
- 操作互斥：`operation_lease_coordinator` 通过 `/xczs/operation_lease` 服务（`ManageOperationLease.srv`）分配租约，启动完成前拒绝一切操作；边界规则在 `operation_validation_policy.hpp`。
- 场景支撑：`cabinet_planning_scene` 按 `cabinet_scene.yaml` 维护碰撞对象；`cabinet_pose_authority` 校验柜体位姿并发布 `pose_valid`；`cabinet_grasp_aggregator` 聚合各末端 grasp 信号到 `/xczs/cabinet/grasp_active`。

## 与项目的关系

- 依赖：`xczs_inspection_robot_interfaces`（msg/srv/action）、MoveIt、`nav2_msgs`、`tf2`；运行期依赖 `xczs_inspection_robot_description` / `_moveit_config` / `_nav2` 及 Gazebo controllers。
- 被消费：`xczs_inspection_robot_bringup` 的 `inspection_robot.launch.py` 组装本包节点（如 `cabinet_button_operator`、`operation_lease_coordinator`）；`xczs_inspection_robot_gazebo` 插件引用 `cabinet_grasp_safety_policy.hpp`，故保留 `ament_export_include_directories`。
- 三层适配架构中的位置：`config/cabinet_robot_adapter.yaml` 是机器人适配层合同（Web 网关、底盘路由、手动轨迹路由与柜体 operator 读同一份）；`cabinet_controls.yaml` / `cabinet_instances.yaml` / `cabinet_scene.yaml` / `scenes.yaml` 是场景适配层，被 `jiang/` 任务层与 `scripts/validate/` 验收脚本按硬编码路径读取。换机器人/设备/场地只改 config，不动任务 API 与 Web 页面。
- 启动链：`start_xczs_bridge.sh` → bringup launch（spawn → controllers → 位姿校验 → 放行 routers/moveit/nav2）；手动操作经租约协调器门控。
