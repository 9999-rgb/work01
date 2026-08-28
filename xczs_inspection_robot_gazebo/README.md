# xczs_inspection_robot_gazebo

## 总体介绍

仿真底座包：3 个 Gazebo C++ 插件 + 1 个世界文件。`planar_stabilizer` 稳住底盘与
臂部位姿、`cabinet_state` 让柜体按钮/旋钮可被操作并有物理反馈、`global_args_guard`
治理进程级 ROS 参数。操作成败以本包发布的关节/状态物理量作为最终判据。

## 模块架构

```
src/                                    # 3 个插件（编译为共享库）
  planar_stabilizer_plugin.cpp          #   -> libxczs_planar_stabilizer.so    ModelPlugin（机器人）
  cabinet_state_plugin.cpp              #   -> libxczs_cabinet_state.so        ModelPlugin（柜体）
  xczs_ros_global_args_guard.cpp        #   -> libxczs_ros_global_args_guard.so  WorldPlugin
worlds/inspection_robot.world           # ODE 物理 500 Hz + gazebo_ros_state + 全局参数守卫
```

- 组件按插件分工：机器人底盘的位姿保持 / 柜体可操作控件仿真 / gzserver 进程级 ROS 参数治理，互不耦合。
- 加载方式：不设 `GAZEBO_PLUGIN_PATH`，靠 colcon 把各包 `lib/` 注入 `LD_LIBRARY_PATH`，world / xacro 按裸插件名引用。
- 协作关键点：`planar_stabilizer` 订阅 `grasp_active` 感知抓握，柜体 grasp 期间松开关节保持，避免与 ros2_control 抢写。

## 功能介绍

- `libxczs_planar_stabilizer.so`：消除底盘滚转/俯仰与高度漂移；无 `cmd_vel` 时锁平面位姿、抓握期间解锁；spawn 后保持臂关节位姿（`hold_joint`），`verify_initial_pose.py` 置 `/xczs/joint_hold_enabled` 为 false 后释放；对场景切换的瞬时位移（teleport）宽容。
- `libxczs_cabinet_state.so`：按 SDF `<control>` 配置仿真按钮/旋钮/开关/柜门（弹簧 + 档位 detent + 迟滞）；每控件发布 `/xczs/cabinet/<id>/joint_states|pressed|state`；grasp 服务把机器人 link 与控件建立运行时 ODE fixed joint 并锁底盘（含柔顺耦合抓握）；抓握期掩蔽致动碰撞并在工具撤离后恢复；操作心跳看门狗超时自动故障恢复；操作前位姿扰动检测。
- `libxczs_ros_global_args_guard.so`：清理 `gazebo_ros2_control` 每次 spawn 注入的进程级 `__ns` 重映射，修复工具套装切换后传感器命名空间被截断的问题。
- ROS 接口（均可在包内源码与 `description` 的 xacro 中找到证据；`<name>` 为柜体
  实例名，如 `cabinet_01`）：
  - 服务：`/xczs/cabinet/<name>/reset_physics`、`/xczs/cabinet/<name>/grasp`
    （`SetCabinetGrasp`）、`/xczs/global_args_guard/reset`
  - 话题：`/xczs/cmd_vel`、`/xczs/joint_hold_enabled`、`/xczs/cabinet/<name>/grasp_active`、
    `/xczs/cabinet/<name>/active_control`、`/xczs/cabinet/<name>/operation_heartbeat`、
    `/xczs/cabinet/<name>/operation_fault`

## 与项目的关系

- 依赖：`xczs_inspection_robot_control`（`cabinet_grasp_safety_policy.hpp` 本地策略头）、`xczs_inspection_robot_interfaces`（`CabinetControl` / `CabinetControlState` / `SetCabinetGrasp`）。
- 被消费：`xczs_inspection_robot_bringup/launch/inspection_robot.launch.py`（`world` 参数默认此包 `worlds/inspection_robot.world`）；`description` 的 `gazebo_plugins.xacro` 与 `control_cabinet/components/gazebo.xacro` 按文件名加载插件；`jiang/scripts/toolset_supervisor.py` 每次 spawn 前调用 `/xczs/global_args_guard/reset`；`jiang/start_xczs_bridge.sh` 默认 `SIMULATION_WORLD_PATH` 指向此 world；`model/`、`jiang/samples/` 的柜体副本亦按 lib 文件名引用。
- 位置：三层适配架构中的仿真底座——向上为机器人适配层（`control/config` 的 adapter YAML）与通用任务层（`jiang/`）提供物理反馈，操作成败以本包控件关节/状态等物理量作为最终判据。
