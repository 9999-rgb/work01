# XCZS 巡操机器人仿真

基于 ROS 2 Humble 和 Gazebo Classic 的巡操机器人仿真项目，包含移动底盘、
六自由度机械臂、两指夹爪、腕部 RGB 相机、车身 2D 激光雷达、手动控制、
MoveIt 2 路径规划、柜体避碰以及 Web 数据监控。

![XCZS 巡操机器人](docs/images/xczs_inspection_robot_preview.png)

## 功能概览

- ROS 2 Humble + Gazebo Classic 11 仿真，不包含 ROS 1 启动或通信逻辑
- 移动底盘、六自由度机械臂、两指夹爪和控制柜碰撞仿真
- 腕部 RGB 相机和车身 180° 2D 激光雷达
- MoveIt 2 逆运动学、OMPL 路径规划、自碰撞与控制柜碰撞检查
- `ros2_control` + `JointTrajectoryController` 标准轨迹执行链路
- Qt GUI、键盘和网页三种互斥控制模式
- 保留 `/xczs/joint_trajectory` 手动接口，并在 MoveIt 执行期间自动阻止同组手动指令
- 浏览器 MJPEG 相机画面、WebSocket 雷达扫描和 SSE 状态监控
- `run_all.sh` 一键启动、统一退出和子进程清理

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- MoveIt 2、`ros2_control` 和 `gazebo_ros2_control`
- `colcon`、Xacro 和 Qt 5

可通过以下命令安装主要 ROS 2 依赖：

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
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

点击页面顶部的“连接”，相机和雷达会自动连接；底盘、机械臂和夹爪控制服务
也会自动检测。按 `Ctrl+C` 可统一关闭 Gazebo、Web 服务和桥接进程。

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
| Web 控制 | `./run_all.sh --web` | 浏览器监控并控制底盘、机械臂和夹爪 |
| Qt GUI | `./run_all.sh --manual` | 使用桌面控制界面 |
| 键盘遥控 | `./run_all.sh --keyboard` | 使用键盘控制机器人 |

无图形桌面或不需要 Gazebo 窗口时，在命令后添加 `--no-gui`，例如：

```bash
./run_all.sh --web --no-gui
```

三种手动控制模式互斥，不要同时启动多个控制节点。手动控制和 MoveIt 规划
可以同时保持在线，但不能同时向同一控制组执行轨迹；兼容路由器会在 MoveIt
action 执行期间拒绝同组手动轨迹。

## MoveIt 2 规划控制

启动统一入口并等待控制器激活后，可在另一个终端执行以下命令。默认只规划，
添加 `--execute` 才会让 Gazebo 中的机器人运动。

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

规划目标会检查关节限位、机器人自碰撞和 Planning Scene 中的控制柜。底盘
导航仍由 `/xczs/cmd_vel` 控制；后续需要自主导航时应接入 Nav2，而不是把
底盘加入机械臂 MoveIt 规划组。

## Web 监控

启动完整系统后访问：

```text
http://localhost:8080/monitor.html
```

点击“连接”，再选择需要监控的话题。监控数据每秒更新一次；Web 控制功能仅在
`--web` 模式下可用。腕部相机和车身雷达无需手动订阅，点击“连接”后会自动
连接 MJPEG 图像流和雷达 WebSocket。浏览器相机流默认为约 `10 FPS`，雷达
WebSocket 默认为约 `10 Hz`；这不会改变 ROS 2 原始相机 `30 Hz` 和雷达
`40 Hz` 的发布频率。

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
| `/xczs/controller_manager` | 管理 Gazebo `ros2_control` 硬件和控制器 |
| `/xczs/joint_state_broadcaster` | 发布全部仿真关节状态 |
| `/xczs/arm_controller` | 执行六轴机械臂标准关节轨迹 |
| `/xczs/gripper_controller` | 执行两指夹爪标准关节轨迹 |
| `/move_group` | MoveIt 2 规划、碰撞检查和轨迹执行 |
| `/xczs_cabinet_planning_scene` | 将控制柜碰撞体同步到 Planning Scene |
| `/xczs_legacy_trajectory_router` | 将旧手动轨迹拆分到两个标准控制器并执行互斥 |
| `/xczs_inspection_robot_gui` | Qt GUI 控制节点 |
| `/xczs_keyboard_teleop` | 键盘控制节点 |
| `/xczs_web_control_server` | Web 控制节点 |
| `/xczs_web_sensor_stream` | 相机 JPEG/MJPEG 与雷达 JSON/WebSocket 转换 |

控制节点根据启动模式选择，不会全部同时运行。

## ROS 2 话题

| 话题 | 消息类型 | 方向与用途 |
| --- | --- | --- |
| `/xczs/cmd_vel` | `geometry_msgs/msg/Twist` | 控制节点 → 底盘速度控制 |
| `/xczs/odom` | `nav_msgs/msg/Odometry` | Gazebo → 底盘里程计 |
| `/xczs/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 控制节点 → 机械臂和夹爪目标 |
| `/xczs/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → 关节状态 |
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

项目当前全部使用 ROS 2 标准消息，不需要额外的自定义消息包。

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
├── xczs_inspection_robot_control/            # 控制节点和统一 launch
│   ├── launch/inspection_robot.launch.py     # 项目统一启动入口
│   └── src/
│       ├── moveit_planner.cpp                # 位姿/命名目标规划工具
│       ├── cabinet_planning_scene.cpp        # 柜体碰撞场景
│       └── legacy_trajectory_router.cpp      # 旧接口兼容与控制互斥
├── jiang/                              # Web、Zenoh 和前后端接口
│   ├── monitor.html                    # 浏览器监控与控制页面
│   ├── sensor_bridge/                  # ROS 2 传感器 Web 转换模块
│   ├── scripts/sensor_stream_server    # 传感器流启动入口
│   ├── control_server.py               # HTTP → ROS 2 控制服务
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
