# XCZS 巡操机器人仿真项目

本项目基于 ROS 2 Humble 和 Gazebo Classic，实现巡操机器人的模型仿真、
底盘移动、六轴机械臂控制和夹爪控制。

## 功能包

- `xczs_inspection_robot_description`：Xacro 模型、网格和 Gazebo 世界。
- `xczs_inspection_robot_control`：GUI 控制节点、键盘控制节点和统一启动文件。

## 编译

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/opt/ros/humble/bin:/bin \
  colcon build \
  --packages-select \
  xczs_inspection_robot_description \
  xczs_inspection_robot_control \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

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

## ROS 2 节点

| 节点 | 功能 |
| --- | --- |
| `/gazebo` | Gazebo 仿真接口。 |
| `/robot_state_publisher` | 发布机器人模型和 TF。 |
| `/xczs_inspection_robot_gui` | GUI 控制节点。 |
| `/xczs_keyboard_teleop` | 键盘控制节点。 |
| `/xczs/xczs_planar_move` | 底盘运动和里程计。 |
| `/xczs/xczs_joint_pose_trajectory` | 机械臂和夹爪位置控制。 |
| `/xczs/xczs_joint_state_publisher` | 关节状态发布。 |

GUI 和键盘节点根据启动参数二选一，其余节点由统一启动文件启动。

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

当前控制接口均使用 ROS 2 标准消息，不需要单独维护自定义消息包。
