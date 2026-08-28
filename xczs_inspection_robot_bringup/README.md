# xczs_inspection_robot_bringup

## 总体介绍

统一启动入口：一条 `ros2 launch xczs_inspection_robot_bringup` 命令按依赖顺序把
Gazebo、MoveIt、Nav2、ros2_control 与柜体控制节点组装成整套仿真栈，并用
「spawn → 控制器 → 位姿校验 → 放行」启动链看门狗保证任一环失败即整体停机。
本包只组装各职责包提供的节点与 launch，自身不实现节点、不定义接口、不持有配置合同。

## 模块架构

- `launch/inspection_robot.launch.py` —— 组装枢纽，运行时读取场景/适配 YAML 生成启动图：
  - include 子 launch：`gazebo_ros` gzserver/gzclient、`moveit_config` move_group
    （OMPL）、`nav2` navigation（地图按场景选择）。
  - 本地 `Node`：`robot_state_publisher`、`xczs_controller_spawner`、
    `xczs_tool_controller_spawner`、`xczs_base_command_router`、
    `xczs_legacy_trajectory_router`、`xczs_operation_lease_coordinator`，
    可选 `xczs_keyboard_teleop` / `xczs_inspection_robot_gui`。
  - 按 `cabinet_instances.yaml` 逐个布柜体（命名空间 `/xczs/cabinet/<name>`）：
    `xczs_cabinet_pose_authority`、`xczs_cabinet_planning_scene`、
    `xczs_cabinet_button_operator`；另起单实例 `xczs_cabinet_grasp_aggregator`
    汇聚各柜 grasp 信号 → `/xczs/cabinet/grasp_active`。
- `scripts/verify_initial_pose.py` —— 启动哨兵：比对实测关节与 `initial_positions.yaml`，
  超差非零退出使父 launch 停机。
- 启动链（`OnProcessExit` 按名串联）：`spawn_robot → controllers →
  tool_controllers → 位姿校验 → 放行 router/moveit/nav2`；任一环失败整栈关闭。

## 功能

- 统一入口：`ros2 launch xczs_inspection_robot_bringup inspection_robot.launch.py`。
- 单命令组装整栈：Gazebo world、MoveIt `move_group`、Nav2（map/参数随场景切换）。
- spawn：机器人走 `-topic robot_description`，场景地板与柜体走 `-file` 载入模型。
- ros2_control：`xczs_controller_spawner` 拉起 state/双臂控制器（管理器
  `/xczs/controller_manager`），`xczs_tool_controller_spawner` 按套装拉起工具控制器。
- 工具套装 `toolset=A|B` 互斥（A=三电缸+两电缸，B=旋转按钮+摇杆）；位姿校验通过后
  向 `/xczs/joint_hold_enabled` 发布 `False` 释放 spawn 期关节锁。
- 关节状态统一走 `adapter_joint_state_topic`（默认 `/xczs/joint_states`，来自
  `cabinet_robot_adapter.yaml`），`robot_state_publisher` 与 MoveIt 均重映射至此。
- 常用开关：`gazebo/gui/paused/robot_bringup/moveit/nav2/teleop/control_gui/scene/spawn_cabinet`；
  `robot_bringup=false` 时本地不 spawn、不建控制器，只对接外部栈。

## 与项目的关系

- 依赖（exec_depend）：`control`、`description`、`gazebo`、`moveit_config`、`nav2` 及
  `gazebo_ros/controller_manager/robot_state_publisher/xacro/moveit_configs_utils`。
- 被消费：`jiang/start_xczs_bridge.sh`（任务层唯一入口）执行本 launch 拉起整套栈；
  `run_all.sh` 转调 `start_xczs_bridge.sh`。
- 三层适配定位：组装枢纽——读取场景适配层 `control/config/` 的 instances/scene/controls/pose/adapter YAML，实例化机器人适配层节点，向上为任务层提供整栈。
