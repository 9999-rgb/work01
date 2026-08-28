# xczs_inspection_robot_moveit_config

## 总体介绍

MoveIt 2 运动规划配置：为双臂 + 末端工具提供规划与轨迹执行能力。SRDF 语义描述、
LMA 逆解、OMPL 规划管线、关节限位与轨迹执行映射都集中在 `config/`，按 `toolset`
参数联动切换 A/B 两套工具对应的文件组。

## 模块架构

- `launch/move_group.launch.py` —— 本包唯一运行时入口：启动 `moveit_ros_move_group` 的 `move_group` 节点，可选启动 `rviz2`（`moveit_rviz` 节点，加载 `config/moveit.rviz`）。
- `config/` —— 全部由 launch 参数显式指定的语义配置：
  - `xczs_inspection_robot_toolset_{A,B}.srdf` —— 语义机器人描述（SRDF）：浮基 `body` 组（floating，`parent_frame=odom`）、双臂链、工具规划组、end_effector、group_state、碰撞豁免。
  - `kinematics.yaml` —— 双臂逆解：`lma_kinematics_plugin/LMAKinematicsPlugin`。
  - `ompl_planning.yaml` —— OMPL 规划管线（`ompl_interface/OMPLPlanner`）与逐组 planner_configs / request_adapters。
  - `joint_limits_toolset_{A,B}.yaml` —— 关节位置/速度/加速度限位。
  - `moveit_controllers_toolset_{A,B}.yaml` —— 轨迹执行映射（`moveit_simple_controller_manager`）。
- `test/test_move_group_launch_policy.py` —— move_group 必需进程退出看门狗的 launch 策略测试（`ament_cmake_pytest`）。

## 功能

- 双臂规划：`left_arm` / `right_arm`（7 自由度冗余）；双臂用 `RRTConnect` / `RRTstar` / `PRMstar`，工具组用 `RRTConnect`。
- 末端工具双套装 A/B：A = 三电缸（右，`three_cylinder`）+ 两电缸（左，`two_cylinder`）；B = 旋转按钮（右，`rotate_button`）+ 摇入摇出（左，`rocker`）。`toolset` launch 参数联动切换 SRDF / joint_limits / controllers 三组文件。
- 轨迹执行：经 `FollowJointTrajectory` action（`xczs/*_controller`，`action_ns: follow_joint_trajectory`，`moveit_manage_controllers: false`）下发到仿真控制器。
- 规划场景监控：`publish_robot_description` / `publish_robot_description_semantic`。
- 折叠位可规划：`default_robot_padding` 0.001 m（fortune-cat 折叠待机下 r_arm_4/r_arm_6 间隙 <10 mm，默认 10 mm 自填充误判碰撞）。
- 鲁棒性：`move_group` 意外退出时向本子 launch 发 `Shutdown`；joint_states 重映射到 `/xczs/joint_states`。

## 与项目的关系

- 依赖 `xczs_inspection_robot_description`（URDF xacro，`toolset` 映射）与 moveit 生态（`moveit_configs_utils`、`moveit_kinematics`、`moveit_planners_ompl`、`moveit_ros_move_group`、`moveit_ros_visualization` 等）。
- 被 `xczs_inspection_robot_bringup/launch/inspection_robot.launch.py` include：`moveit` / `moveit_rviz` 开关控制加载，并复用 `MoveItConfigsBuilder` 为控制层构建客户端配置（规划场景 / robot_description）。
- 被 `xczs_inspection_robot_control` 消费：`src/cabinet_button_operator.cpp` 用 `MoveGroupInterface` 规划；`config/cabinet_robot_adapter.yaml`（机器人适配层合同）按 `left_arm` / `right_arm` 等规划组与 `move_group_namespace` 引用本包定义。
- 启动链：`run_all.sh` → `jiang/start_xczs_bridge.sh` → bringup launch（spawn_robot → controllers → tool_controllers → 位姿校验 → 放行 router/moveit/nav2）。
