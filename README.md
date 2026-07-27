# XCZS 巡操机器人仿真项目

本项目基于 ROS 2 Humble 和 Gazebo Classic，实现巡操机器人的模型仿真、
底盘移动、六轴机械臂控制和夹爪控制。

## 功能包

- `xczs_inspection_robot_description`：Xacro 模型、网格和 Gazebo 世界。
- `xczs_inspection_robot_control`：GUI、键盘、自主任务调度节点和统一启动文件。

## 编译

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

如果 `python3` 指向 Miniconda，请先执行 `conda deactivate`。

## 启动

默认启动 Gazebo 和 GUI 控制界面：

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py
```

GUI 支持按住按钮控制前进、后退和转向，松开后平滑停止；同时可以设置六个机械臂
关节角度、机械臂回零以及控制夹爪开合。

使用键盘控制器：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  control_gui:=false teleop:=true
```

键盘按键：

- `W` / `S`：前进 / 后退。
- `A` / `D`：左转 / 右转。
- `X`：平滑停止，空格：紧急停止。
- `1`～`6`：选择机械臂关节，`[` / `]`：调整角度，`R`：回零。
- `O` / `P`：打开 / 关闭夹爪。
- `Q`：退出。

不建议同时启动 GUI 和键盘控制器，避免两个节点同时发布控制指令。

### 任务调度器（自主巡检）

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
  control_gui:=false task_scheduler:=true
```

任务调度器按 A→B→C→D 流水线自主执行巡检：
- **A 全局导航**：里程计推算 + 比例控制器驱动底盘至目标巡检点
- **B 精确定位**：到达后短暂停留模拟视觉伺服
- **C 机械臂操作**：按预设关节序列展开-接近-操作-折叠
- **D 收回转移**：机械臂归零，循环至下一巡检点

## 一键启动（仿真 + 桥 + 监控面板）

```bash
./run_all.sh                # Gazebo + Qt GUI + 浏览器监控
./run_all.sh --web          # 浏览器控制模式
./run_all.sh --task         # 自主任务调度器模式
./run_all.sh --keyboard     # 键盘控制模式
./run_all.sh --with-proxy   # 附加 JSON 再发布代理
./run_all.sh --no-gui       # 无显示器时自动使用任务调度器
```

启动后打开浏览器访问 `http://localhost:8080/monitor.html`：

1. 点击"连接"
2. 点击"订阅全部 XCZS 话题"
3. 观察实时数据（底盘速度、里程计、关节状态、任务阶段）

浏览器控制底盘、机械臂和夹爪时必须使用 `./run_all.sh --web`。各控制模式
互斥，避免多个节点同时向控制话题发布指令。

首次使用 Zenoh 功能前安装 Python 依赖：

```bash
/usr/bin/python3 -m pip install -r jiang/requirements.txt
```

一键脚本还需要
`/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds`。

## Zenoh 桥与实时监控

本项目通过 [Zenoh](https://zenoh.io/) 将 ROS2 话题实时转发至浏览器：

```
Gazebo/ROS2 → zenoh-bridge-ros2dds → TCP:7447 → SSE bridge:8001 → monitor.html
                                             └→ JSON proxy（可选）
monitor.html → HTTP control:8090 → rclpy → ROS 2 控制话题
```

| 组件 | 地址 | 说明 |
|------|------|------|
| Zenoh Bridge TCP | `tcp/localhost:7447` | ROS 2 与 Zenoh 数据桥 |
| SSE 数据桥 | `http://localhost:8001` | 浏览器实时数据源 |
| 监控面板 HTTP | `http://localhost:8080/monitor.html` | 零依赖实时看板 |
| Web 控制服务 | `http://localhost:8090` | 仅 `--web` 模式启动 |
| CDR→JSON 代理 | `jiang/run_xczs_proxy.py` | 可选 JSON 再发布 |

监控面板功能：
- 实时数据卡片（底盘速度、里程计、关节状态、关节指令）
- 任务阶段指示灯（A→B→C→D 流水线可视化）
- 机械臂关节角度柱状图
- 巡检点进度计数

## ROS 2 节点

| 节点 | 功能 |
| --- | --- |
| `/gazebo` | Gazebo 仿真接口。 |
| `/robot_state_publisher` | 发布机器人模型和 TF。 |
| `/xczs_inspection_robot_gui` | GUI 控制节点。 |
| `/xczs_keyboard_teleop` | 键盘控制节点。 |
| `/xczs_task_scheduler` | 自主巡检任务调度。 |
| `/xczs_web_control_server` | 浏览器控制，仅 Web 模式启动。 |
| `/xczs/xczs_planar_move` | 底盘运动和里程计。 |
| `/xczs/xczs_joint_pose_trajectory` | 机械臂和夹爪位置控制。 |
| `/xczs/xczs_joint_state_publisher` | 关节状态发布。 |

Qt GUI、键盘、任务调度器和浏览器控制根据运行模式四选一。

## ROS 2 话题

| 话题 | 消息类型 | 功能 |
| --- | --- | --- |
| `/robot_description` | `std_msgs/msg/String` | Xacro 展开后的机器人模型。 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 机器人坐标变换。 |
| `/xczs/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令。 |
| `/xczs/odom` | `nav_msgs/msg/Odometry` | 底盘里程计。 |
| `/xczs/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 机械臂和夹爪目标位置。 |
| `/xczs/joint_states` | `sensor_msgs/msg/JointState` | 机械臂、夹爪和车轮关节状态。 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时间。 |
| `/mission/phase` | `std_msgs/msg/String` | 任务调度器当前阶段（A/B/C/D/IDLE/DONE）。 |
| `/mission/current_waypoint` | `std_msgs/msg/Int32` | 当前巡检点序号。 |

当前控制接口均使用 ROS 2 标准消息，不需要单独维护自定义消息包。
