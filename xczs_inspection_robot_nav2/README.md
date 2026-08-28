# xczs_inspection_robot_nav2

## 总体介绍

导航配置：为全向底盘提供定位（AMCL）、全局规划（Navfn）、DWB 局部控制与恢复，
随活动场景切换地图与初始位姿。标准 Nav2 布局——包内只维护 launch / 参数 / 行为树 /
地图，不写自研节点，导航进程由 `nav2_bringup` 提供。

## 模块架构

- `launch/navigation.launch.py` —— 唯一入口：include `nav2_bringup/bringup_launch.py`
  （关闭 slam / 组合 / respawn），可选 rviz2（`nav2_default_view.rviz`），严格校验
  `autostart` / `rviz` / `use_sim_time` 三个布尔参数。
- `config/nav2_params.yaml` —— 全部 Nav2 节点参数（amcl / planner / controller /
  costmap / bt_navigator / behavior / smoother / velocity_smoother / map_server /
  waypoint_follower 等）。
- `config/behavior_trees/stable_axis_navigation.xml` —— 自 vendored 行为树：仅在目标
  变化或全局路径失效时重规划（`IsPathValid` + `ComputePathToPose` + 恢复子树）。
- `maps/` —— 三张场景占用栅格地图（`.pgm` + `.yaml`）：`inspection_map` /
  `inspection_map_electrical_mezzanine` / `inspection_map_generator_plant`。
- `test/test_navigation_launch_policy.py` —— launch 布尔参数校验与归一化测试。

## 功能介绍

- 定位 `amcl`：`OmniMotionModel`，订阅 `/xczs/lidar/scan`，信任 planar mover 真值
  odom（低噪声 alpha），`set_initial_pose` + `always_reset_initial_pose`。
- 全局规划 `planner_server`：`NavfnPlanner`（`GridBased`，`use_astar`，
  `allow_unknown=true` 支持穿越柜体背面未知走廊）。
- 局部控制 `controller_server`：`DWBLocalPlanner`（`FollowPath`），全向 vx/vy/vtheta；
  `PoseProgressChecker` 计入原地转向，`SimpleGoalChecker` 0.20 m / 0.25 rad（stateful=false）。
- 导航行为 `bt_navigator`：`default_nav_to_pose_bt_xml` 指向本包行为树。
- 恢复 `behavior_server`：spin / backup / drive_on_heading / wait / assisted_teleop。
- 平滑与限速 `smoother_server` / `velocity_smoother`（订阅 `/xczs/odom`）。
- 地图服务 `map_server` / `map_saver`；`waypoint_follower`（wait_at_waypoint）。
- 输入话题：`/xczs/lidar/scan`（AMCL 与 costmap 障碍）、`/xczs/odom`（真值里程计）。

## 与项目的关系

- 依赖：`nav2_bringup` / `navigation2` / `rviz2` / `xczs_inspection_robot_description`。
- 启动链：`xczs_inspection_robot_bringup` 的 `inspection_robot.launch.py` include
  `launch/navigation.launch.py`，按活动场景改写 `amcl.initial_pose`（= `robot_spawn`）
  并传入场景地图；`jiang/start_xczs_bridge.sh` 在 `NAV2_ENABLED` 时预检本包文件
  （launch / map / 参数）；`CONTROL_MODE=web` 或 `CABINET_BRINGUP=true` 时运行
  `check_scene_config` 跨文件合同校验。
- 三层适配架构中的位置：
  - 场景适配层（`xczs_inspection_robot_control/config/scenes.yaml`）为每个场景的
    `nav2_map` 指定本包地图；切换场景 = `LoadMap` + teleport 重定位。
  - 任务层（`jiang/control_gateway/runner.py`）经 Nav2 执行导航动作并做容差复核。
- 校验合同：`scripts/validate/check_scene_config` 核对地图/参数存在；
  `jiang/tests/test_nav2_config_contract.py` 强制容差分层
  （checker 0.20 m < 任务层 ≤ 接管距离）与行为树结构。
- 地图生成：`scripts/tools/generate_scene_maps.py`（开发期工具，由场景 STL 切出栅格图）。
