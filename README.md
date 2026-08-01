# XCZS 巡操机器人仿真

基于 ROS 2 Humble 和 Gazebo Classic 的巡操机器人仿真项目，包含移动底盘、
六自由度机械臂、两指夹爪、腕部 RGB 相机、车身 2D 激光雷达、手动控制、
MoveIt 2 机械臂路径规划、Nav2 底盘自主导航、柜体避碰以及 Web 数据监控。

![XCZS 巡操机器人](docs/images/xczs_inspection_robot_preview.png)

## 功能概览

- ROS 2 Humble + Gazebo Classic 11 仿真，不包含 ROS 1 启动或通信逻辑
- 移动底盘、六自由度机械臂、两指夹爪和控制柜碰撞仿真
- 腕部 RGB 相机和车身 180° 2D 激光雷达
- MoveIt 2 逆运动学、OMPL 路径规划、自碰撞与控制柜碰撞检查
- Nav2 + AMCL 定位、全局/局部路径规划、激光雷达动态避障和柜体绕障
- 控制柜实体按钮行程、弹簧回弹及 Nav2 + MoveIt 闭环按压操作
- `ros2_control` + `JointTrajectoryController` 标准轨迹执行链路
- 手动底盘与 Nav2 速度命令路由、模式互斥和失联自动停车
- Qt GUI、键盘和网页三种手动控制模式
- 保留 `/xczs/joint_trajectory` 手动接口，并在 MoveIt 执行期间自动阻止同组手动指令
- 浏览器 MJPEG 相机画面、WebSocket 雷达扫描和 SSE 状态监控
- `run_all.sh` 一键启动、统一退出和子进程清理

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- MoveIt 2、Nav2、`ros2_control` 和 `gazebo_ros2_control`
- `colcon`、Xacro 和 Qt 5

可通过以下命令安装主要 ROS 2 依赖：

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers
```

Web 监控还需要：

- `/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds`
- Python 依赖：`jiang/requirements.txt`（Zenoh、HTTP/WebSocket、JPEG 编码）

## 获取与编译

```bash
git clone https://github.com/9999-rgb/work01.git
cd work01
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```


## 快速启动

推荐使用网页控制模式启动完整系统：

```bash
/usr/bin/python3 -m pip install -r jiang/requirements.txt
./run_all.sh --web
```

启动完成后打开：

```text
http://localhost:8080/monitor.html
```

`--web` 会在同一个进程树中启动 Gazebo、MoveIt 2、Nav2、传感器服务和网页
控制服务，不需要再分别打开 RViz 或命令行控制节点。页面打开后会自动
连接监控、相机、雷达和控制服务，可直接使用手动底盘、地图导航、机械臂
规划和夹爪控制。按 `Ctrl+C` 可统一关闭全部进程。

## 其他启动方式

### 仅启动 ROS 2 仿真

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py
```

该命令启动 Gazebo、巡操机器人、控制柜模型、Qt 控制界面、三个
`ros2_control` 控制器和 MoveIt 2 `move_group`。控制柜默认位于 `x=2.0`、
`y=0.33`，并已从 SolidWorks 的 Y-up 坐标系旋转为 Gazebo 的 Z-up
坐标系；同一柜体包围盒会自动加入 MoveIt Planning Scene。

如需调整控制柜位置或不加载控制柜，可使用统一启动入口的参数：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  cabinet_x:=3.0 cabinet_y:=0.0 cabinet_yaw:=-1.5708

ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  spawn_cabinet:=false
```

MoveIt 2 默认启用。需要 Motion Planning 面板时启动 RViz：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  moveit_rviz:=true
```

仅做传统手动调试、不启动规划层时：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  moveit:=false
```

### 完整系统控制模式

首次使用 Web 功能时安装 Python 依赖：

```bash
/usr/bin/python3 -m pip install -r jiang/requirements.txt
```

按需要选择一种控制模式：

| 模式 | 命令 | 用途 |
| --- | --- | --- |
| Web 统一控制 | `./run_all.sh --web` | 浏览器控制底盘、Nav2、MoveIt 2、夹爪并监控传感器 |
| Qt GUI | `./run_all.sh --manual` | 使用桌面控制界面 |
| 键盘遥控 | `./run_all.sh --keyboard` | 使用键盘控制机器人 |
| Nav2 导航 | `./run_all.sh --nav2` | 使用 AMCL 定位、规划绕障并打开 Nav2 RViz |

无图形桌面或不需要 Gazebo 窗口时，在命令后添加 `--no-gui`，例如：

```bash
./run_all.sh --web --no-gui
```

三种手动控制模式互斥，不要同时启动多个控制节点。Web 模式会自动启用 Nav2
和 MoveIt 2，底盘初始处于 Nav2 模式；点击页面中的“切换手动模式”后可使用
方向键，发送新的导航目标时会自动切回 Nav2 模式。手动控制和 MoveIt 规划可以
同时保持在线，但不能同时向同一控制组执行轨迹；兼容路由器会在 MoveIt
action 执行期间拒绝同组手动轨迹。

## MoveIt 2 规划控制

推荐直接运行 `./run_all.sh --web`，在网页的 “MoveIt 2 Motion” 区域选择
机械臂回零、夹爪开合，或填写末端位置和姿态后执行“仅规划”或“规划并执行”。
页面会显示规划状态、规划耗时、错误码，并可取消正在执行的动作。

下面的命令行方式保留用于调试。启动统一入口并等待控制器激活后，可在另一个
终端执行。默认只规划，添加 `--execute` 才会让 Gazebo 中的机器人运动。

机械臂规划回零：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xczs_inspection_robot_control moveit_planner \
  --group manipulator --named home --execute
```

指定末端 `end` 在 `body` 坐标系中的位置和四元数：

```bash
ros2 run xczs_inspection_robot_control moveit_planner \
  --group manipulator \
  --frame body \
  --pose -0.226 0.787 0.318 0.653 0.153 0.741 0.015 \
  --plan-only
```

夹爪规划开合：

```bash
ros2 run xczs_inspection_robot_control moveit_planner \
  --group gripper --named open --execute
ros2 run xczs_inspection_robot_control moveit_planner \
  --group gripper --named closed --execute
```

规划目标会检查关节限位、机器人自碰撞和 Planning Scene 中的控制柜。MoveIt
只负责机械臂和夹爪，底盘自主导航由 Nav2 独立负责。

## Nav2 底盘自主导航

推荐在网页中统一控制。下面一条命令会启动 Gazebo、机器人、控制柜、AMCL、
Nav2、MoveIt 2 和网页服务，不会额外打开导航 RViz：

```bash
./run_all.sh --web
```

页面地图中可直接单击空闲区域选择目标，调整 `x`、`y` 和朝向后开始导航；
页面会显示机器人实时位置、全局路径、剩余距离、预计时间和恢复次数，也可随时
取消导航或切回手动底盘模式。

需要使用 Nav2 RViz 做桌面调试时：

```bash
./run_all.sh --nav2
```

也可以只通过统一 ROS 2 启动入口启用 Nav2：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  nav2:=true nav2_rviz:=true
```

系统默认加载
`xczs_inspection_robot_nav2/maps/inspection_map.yaml`，并根据机器人默认出生
姿态自动设置 AMCL 初始位姿。RViz 中等待 Nav2 节点激活后，点击
`Nav2 Goal` 并在地图上指定目标位置和朝向，底盘会规划路径、绕开静态柜体，
并使用车身雷达更新局部和全局代价地图。

也可从命令行发送地图坐标系目标，例如移动到 `(0, 1)`，末端朝向 `+Y`：

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {
    position: {x: 0.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.70710678, w: 0.70710678}
  }}}"
```

手动底盘命令与 Nav2 命令不会同时下发。运行中可显式切换模式：

```bash
# 启用 Nav2，阻止 GUI、键盘和 Web 底盘速度
ros2 service call /xczs/set_navigation_mode \
  std_srvs/srv/SetBool "{data: true}"

# 恢复手动底盘控制，阻止 Nav2 速度
ros2 service call /xczs/set_navigation_mode \
  std_srvs/srv/SetBool "{data: false}"
```

默认地图与统一启动入口中的默认柜体位姿匹配。修改 `cabinet_x`、
`cabinet_y` 或 `cabinet_yaw` 后，雷达仍能检测真实障碍，但静态地图中的柜体
位置不会自动改变；用于正式导航前应同步更新地图，或通过 SLAM 重新建图。

## 控制柜按钮操作验证

当前支持 `box_10_button_1`，即控制柜 10 号模块上的红色按钮。从柜体正面看，
它位于同模块绿色按钮左侧，默认世界坐标约为
`(1.989, -0.162, 0.574)`。按钮具有 8 mm 物理行程和弹簧回弹。

启动带 Nav2 的统一入口：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  nav2:=true control_gui:=false
```

等待 Gazebo、MoveIt 2 和 Nav2 就绪后发送操作目标：

```bash
ros2 action send_goal /xczs/press_cabinet_button \
  xczs_inspection_robot_control/action/PressCabinetButton \
  "{button_id: box_10_button_1, navigate_to_staging_pose: true}" \
  --feedback
```

机器人会依次完成底盘停靠、机械臂预接触、直线按压、按下状态确认、撤离和
回弹确认。成功结果中的 `max_travel` 应不小于 `0.006` 米，目标值约为
`0.007` 米。可在其他终端观察按钮行程和按下状态：

```bash
ros2 topic echo /xczs/cabinet/box_10_button_1/joint_states
ros2 topic echo /xczs/cabinet/box_10_button_1/pressed
```

如果底盘已经停在柜体前，可以将目标中的
`navigate_to_staging_pose` 设为 `false`，跳过自动导航。

## Web 监控

启动完整系统后访问：

```text
http://localhost:8080/monitor.html
```

页面会自动连接各项 Web 服务，再选择需要监控的话题即可。监控数据每秒
更新一次；Web 控制功能仅在 `--web` 模式下可用。页面提供以下统一功能：

- 手动底盘速度控制，以及手动/Nav2 模式互斥切换
- Nav2 占用地图、机器人位置和全局路径显示，地图选点、目标导航与取消
- MoveIt 2 机械臂回零、末端位姿规划/执行、夹爪开合与动作取消
- 相机 MJPEG 图像、雷达扫描图以及 ROS 2 话题状态监控

腕部相机和车身雷达无需手动订阅，页面加载后会自动连接 MJPEG 图像流和雷达
WebSocket，流中断后会自动重连。浏览器相机流默认为约 `10 FPS`，雷达 WebSocket 默认为约
`10 Hz`；这不会改变 ROS 2 原始相机 `30 Hz` 和雷达 `40 Hz` 的发布频率。

| 服务 | 地址 |
| --- | --- |
| 监控页面 | `http://localhost:8080/monitor.html` |
| SSE 数据 | `http://localhost:8001` |
| 相机与雷达流 | `http://localhost:8003` |
| Web 控制 | `http://localhost:8090` |
| Zenoh TCP | `tcp/localhost:7447` |

前后端接口格式和调用示例见
[FRONTEND_API.md](FRONTEND_API.md)。

传感器流服务提供以下浏览器接口：

| 接口 | 格式 | 用途 |
| --- | --- | --- |
| `/camera.mjpg` | `multipart/x-mixed-replace` | 浏览器实时相机画面 |
| `/camera.jpg` | `image/jpeg` | 获取最新单帧图像 |
| `/lidar.json` | JSON | 获取最新一帧雷达扫描 |
| `/lidar/ws` | WebSocket JSON | 持续接收雷达扫描 |
| `/health` | JSON | 检查相机和雷达数据状态 |

可直接使用以下命令检查传感器 Web 服务：

```bash
curl http://localhost:8003/health
curl http://localhost:8003/lidar.json
curl http://localhost:8003/camera.jpg --output camera.jpg
```

从其他计算机访问并控制时，应让控制服务监听局域网地址：

```bash
CONTROL_HOST=0.0.0.0 ./run_all.sh --web
```

然后在另一台计算机打开
`http://<simulation-host-ip>:8080/monitor.html`。页面会自动使用当前主机名
连接 `8001`、`8003` 和 `8090` 端口，无需手动把 `localhost` 改成服务器
地址。局域网防火墙至少需要允许 TCP 端口 `8001`、`8003`、`8080` 和
`8090`；浏览器不需要直接访问 Zenoh TCP 端口。

Web 服务端口可通过环境变量调整：

| 环境变量 | 默认值 | 用途 |
| --- | --- | --- |
| `MONITOR_PORT` | `8080` | 监控页面 |
| `SSE_PORT` | `8001` | 常规 ROS 2 数据 SSE |
| `SENSOR_PORT` | `8003` | 相机和雷达 Web 服务 |
| `CONTROL_PORT` | `8090` | 网页控制服务 |
| `CONTROL_HOST` | `127.0.0.1` | 网页控制监听地址 |
| `SENSOR_HOST` | `0.0.0.0` | 传感器流监听地址 |

## 键盘控制

| 按键 | 功能 |
| --- | --- |
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左转 / 右转 |
| `X` / 空格 | 平滑停止 / 紧急停止 |
| `1`～`6` | 选择机械臂关节 |
| `[` / `]` | 减小 / 增大关节角度 |
| `R` | 机械臂回零 |
| `O` / `P` | 打开 / 关闭夹爪 |
| `Q` | 退出 |

## ROS 2 节点

| 节点 | 功能 |
| --- | --- |
| `/gazebo` | 运行物理仿真并发布仿真时间 |
| `/robot_state_publisher` | 发布机器人模型和 TF |
| `/xczs/xczs_planar_move` | 接收底盘速度并发布里程计 |
| `/xczs_base_command_router` | 在手动和 Nav2 底盘命令之间互斥切换、限速和超时停车 |
| `/xczs/controller_manager` | 管理 Gazebo `ros2_control` 硬件和控制器 |
| `/xczs/joint_state_broadcaster` | 发布全部仿真关节状态 |
| `/xczs/arm_controller` | 执行六轴机械臂标准关节轨迹 |
| `/xczs/gripper_controller` | 执行两指夹爪标准关节轨迹 |
| `/move_group` | MoveIt 2 规划、碰撞检查和轨迹执行 |
| `/amcl` | 使用静态地图和激光雷达估计底盘位姿 |
| `/planner_server`、`/controller_server` | Nav2 全局路径规划和局部轨迹控制 |
| `/bt_navigator` | 执行 Nav2 导航行为树和恢复行为 |
| `/xczs_cabinet_planning_scene` | 将控制柜碰撞体同步到 Planning Scene |
| `/xczs_cabinet_button_operator` | 执行底盘停靠、机械臂按压及按钮状态闭环验证 |
| `/xczs_legacy_trajectory_router` | 将旧手动轨迹拆分到两个标准控制器并执行互斥 |
| `/xczs_inspection_robot_gui` | Qt GUI 控制节点 |
| `/xczs_keyboard_teleop` | 键盘控制节点 |
| `/xczs_web_control_server` | Web 手动控制、Nav2 action 和 MoveIt action 网关 |
| `/xczs_web_sensor_stream` | 相机 JPEG/MJPEG 与雷达 JSON/WebSocket 转换 |

控制节点根据启动模式选择，不会全部同时运行。

## ROS 2 话题

| 话题 | 消息类型 | 方向与用途 |
| --- | --- | --- |
| `/xczs/manual_cmd_vel` | `geometry_msgs/msg/Twist` | GUI、键盘或 Web → 手动命令输入 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 → 导航命令输入（REP-105 `base_link`） |
| `/xczs/cmd_vel` | `geometry_msgs/msg/Twist` | 命令路由器 → Gazebo 底盘最终速度 |
| `/xczs/navigation_mode` | `std_msgs/msg/Bool` | 命令路由器 → 当前手动/Nav2 模式 |
| `/xczs/odom` | `nav_msgs/msg/Odometry` | Gazebo → 底盘里程计 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 静态占用地图 |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL 地图定位结果 |
| `/plan` | `nav_msgs/msg/Path` | Nav2 当前全局规划路径 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 雷达更新的局部代价地图 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 静态地图与雷达融合的全局代价地图 |
| `/xczs/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 控制节点 → 机械臂和夹爪目标 |
| `/xczs/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → 关节状态 |
| `/xczs/cabinet/box_10_button_1/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → 红色按钮行程 |
| `/xczs/cabinet/box_10_button_1/pressed` | `std_msgs/msg/Bool` | Gazebo → 红色按钮按下状态 |
| `/xczs/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 机械臂控制器话题入口 |
| `/xczs/gripper_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 夹爪控制器话题入口 |
| `/xczs/camera/arm_camera/image_raw` | `sensor_msgs/msg/Image` | 腕部相机彩色图像 |
| `/xczs/camera/arm_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 腕部相机标定参数 |
| `/xczs/lidar/scan` | `sensor_msgs/msg/LaserScan` | 车身 180° 激光扫描 |
| `/robot_description` | `std_msgs/msg/String` | 机器人模型描述 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 机器人坐标变换 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时间 |

MoveIt 通过以下标准 action 执行轨迹：

```text
/xczs/arm_controller/follow_joint_trajectory
/xczs/gripper_controller/follow_joint_trajectory
```

Nav2 提供以下标准导航 action：

```text
/navigate_to_pose
/navigate_through_poses
/follow_waypoints
```

控制柜按钮操作提供以下自定义 action：

```text
/xczs/press_cabinet_button
```

节点间通信全部使用 ROS 2 topic、service 和 action。按钮 action 定义在控制
功能包内，不需要额外的独立消息包。

## 项目结构

```text
work01/
├── xczs_inspection_robot_description/  # 机器人模型与仿真环境
│   ├── meshes/
│   │   └── control_cabinet/                  # 控制柜英文命名网格
│   └── urdf/
│       ├── xczs_inspection_robot.urdf.xacro  # 机器人模型组装入口
│       ├── control_cabinet.urdf.xacro        # 控制柜模型组装入口
│       ├── components/                       # 机器人模型功能组件
│       └── control_cabinet/components/       # 控制柜模型功能组件
│   └── config/
│       └── ros2_controllers.yaml             # Gazebo 控制器参数
├── xczs_inspection_robot_moveit_config/      # MoveIt 2 规划配置
│   ├── config/
│   │   ├── xczs_inspection_robot.srdf        # 规划组、末端和碰撞矩阵
│   │   ├── kinematics.yaml                   # KDL 逆运动学
│   │   ├── joint_limits.yaml                 # 规划速度和关节限位
│   │   ├── ompl_planning.yaml                # OMPL 规划器
│   │   └── moveit_controllers.yaml           # FollowJointTrajectory 对接
│   └── launch/move_group.launch.py           # MoveIt 规划层入口
├── xczs_inspection_robot_nav2/                # Nav2 底盘自主导航配置
│   ├── config/nav2_params.yaml                # AMCL、规划器与代价地图参数
│   ├── launch/navigation.launch.py            # Nav2 模块启动入口
│   └── maps/inspection_map.{yaml,pgm}         # 默认仿真静态地图
├── xczs_inspection_robot_control/            # 控制节点和统一 launch
│   ├── action/PressCabinetButton.action       # 控制柜按钮操作接口
│   ├── launch/inspection_robot.launch.py     # 项目统一启动入口
│   └── src/
│       ├── moveit_planner.cpp                # 位姿/命名目标规划工具
│       ├── cabinet_planning_scene.cpp        # 柜体碰撞场景
│       ├── cabinet_button_operator.cpp       # 控制柜按钮操作状态机
│       ├── base_command_router.cpp            # 手动/Nav2 底盘命令互斥
│       └── legacy_trajectory_router.cpp      # 旧接口兼容与控制互斥
├── jiang/                              # Web、Zenoh 和前后端接口
│   ├── monitor.html                    # 浏览器监控与控制页面
│   ├── sensor_bridge/                  # ROS 2 传感器 Web 转换模块
│   ├── control_gateway/                # 手动、Nav2 与 MoveIt 2 Web 网关模块
│   ├── scripts/sensor_stream_server    # 传感器流启动入口
│   ├── control_server.py               # Web 控制网关启动入口
│   └── sse_bridge.py                   # Zenoh → SSE 数据服务
├── docs/                               # 图片等项目文档资源
└── run_all.sh                          # 完整系统启动入口
```

## 常见检查

确认三个 `ros2_control` 控制器均为 `active`：

```bash
ros2 control list_controllers -c /xczs/controller_manager
```

确认 MoveIt 服务、执行 action 和控制柜碰撞体：

```bash
ros2 action list | grep -E 'move_action|follow_joint_trajectory'
ros2 service call /get_planning_scene \
  moveit_msgs/srv/GetPlanningScene '{components: {components: 16}}'
```

确认 ROS 2 传感器原始数据：

```bash
ros2 topic hz /xczs/camera/arm_camera/image_raw
ros2 topic hz /xczs/lidar/scan
```

确认 Nav2 生命周期、定位和导航 action：

```bash
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /amcl_pose
ros2 action list | grep -E 'navigate_to_pose|navigate_through_poses|follow_waypoints'
```

如果网页显示“等待 ROS 2 图像”或雷达持续重连，依次检查：

```bash
curl http://localhost:8003/health
ros2 topic info /xczs/camera/arm_camera/image_raw --verbose
ros2 topic info /xczs/lidar/scan --verbose
```

如果端口被占用：

```bash
lsof -nP -iTCP:8003 -sTCP:LISTEN
```

更完整的 SSE、MJPEG、WebSocket 和控制请求格式参见
[FRONTEND_API.md](FRONTEND_API.md)。

当前相机是 RGB 相机、雷达是 2D `LaserScan`，两者都不是深度点云，因此没有
配置 MoveIt 3D OctoMap 更新插件。启动时出现“未定义 3D sensor plugin”的
日志不影响机器人自碰撞、URDF 碰撞体或控制柜 Planning Scene 检查；以后增加
深度相机或 3D 雷达时，再配置 `PointCloudOctomapUpdater`。
