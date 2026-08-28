# xczs_inspection_robot_description

## 总体介绍

机器人/柜体模型与配置源：把 LinkForge 双壁导出与 SolidWorks 导出的四套末端工具
转成 xacro/URDF，连同 STL 网格、ros2_control 与初始位姿配置一起提供给整栈。
场景 world 由 `xczs_inspection_robot_gazebo` 负责，本包专注模型本身——
`control_cabinet.urdf.xacro` 即柜体设备的几何合同。

## 模块架构

- `urdf/xczs_inspection_robot.urdf.xacro` —— 机器人主入口，按序 include
  `urdf/components/`（properties/materials/dual_arm_body/dual_arm_manipulator/
  tools/sensors/ros2_control/gazebo_plugins）。
- `urdf/control_cabinet.urdf.xacro` —— 柜体设备模型（独立入口，CMake 目标
  `generate_control_cabinet_urdf`），include `urdf/control_cabinet/components/`。
- `urdf/scenes/` —— 场景地面 SDF（`electrical_mezzanine.sdf` / `generator_plant.sdf`）。
- `meshes/` —— STL 网格，按目录分：`dual_arm/`（每 link 视觉+碰撞成对）、`tools/`
  （rocker/rotate_button/three_cylinder/two_cylinder）、`control_cabinet/`、`scenes/`。
- `config/` —— `ros2_controllers_toolset_{A,B}.yaml`、`initial_positions.yaml`、
  `convex_hull.mlx`（凸包滤镜）。
- `scripts/` —— 碰撞网格再生成：`generate_collision_meshes.sh` → `decimate_arm.py`
  + meshlabserver 凸包。
- 协作：`ros2_control.xacro` 生成 `GazeboSystem` 硬件插件，按 `$(arg toolset)` 加载
  控制器 YAML，并经 `xacro.load_yaml` 读 `initial_positions.yaml` 作 14 关节出生位姿。

## 功能介绍

- 双 7-DOF 机械臂 + 四轮底盘（wheel 仅 state，平面运动由 `libgazebo_ros_planar_move.so` 驱动）。
- 末端工具套装 A/B 互斥（`toolset`，默认 A）：A=右 `three_cylinder`+左 `two_cylinder`；
  B=右 `rotate_button`+左 `rocker`。臂关节 position、工具关节 effort，控制器名套装无关。
- 传感器：右腕相机（`libgazebo_ros_camera.so`，`/xczs/camera/arm_camera/image_raw|camera_info`）；
  机身平面激光（`libgazebo_ros_ray_sensor.so`，`/xczs/lidar/scan`）。
- 插件：`libxczs_planar_stabilizer.so`（启动期保持位姿）、`libgazebo_ros2_control.so`。
- 柜体模型：11 箱体 + 按钮/旋钮/主开关/后门，由 `libxczs_cabinet_state.so`（gazebo 包）驱动，
  每控件在 `/xczs/cabinet/<name>/...` 发布状态并提供 grasp / `reset_physics` 服务。

## 与项目的关系

- 依赖（exec_depend）：`gazebo_plugins/gazebo_ros/gazebo_ros2_control/joint_state_broadcaster/
  joint_trajectory_controller/sensor_msgs/xacro`。
- 被消费：`bringup`（按 `toolset`+`gazebo_plugin_instance_id` 构建并 spawn 机器人；
  `verify_initial_pose.py` 读 `initial_positions.yaml`）；`control`（`scenes.yaml` 引用
  `urdf/scenes/*.sdf`）；`moveit_config` / `nav2`（构建机器人描述）；`jiang/`
  （`control_server.py` / `runner.py` 按硬编码路径读 `control_cabinet.urdf.xacro`）。
- 位置：`control_cabinet.urdf.xacro` 即设备几何合同（`docs/architecture.md` §3）；机器人本体
  供机器人适配层与 bringup 启动链；场景 world 由 `xczs_inspection_robot_gazebo` 提供。
