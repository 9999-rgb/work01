# XCZS 巡操机器人仿真项目

本项目基于 ROS 2 Humble 和 Gazebo Classic，实现 XCZS 巡操机器人的模型仿真与键盘遥控。
目前支持底盘移动、六轴机械臂控制和夹爪开合。

## 项目结构

- `xczs_inspection_robot_description`：机器人 URDF、外观网格、碰撞网格和 Gazebo 仿真环境。
- `xczs_inspection_robot_control`：C++ 键盘遥控节点和统一启动文件。
- `build/`：`colcon` 生成的构建文件。
- `install/`：编译完成后的安装文件。
- `log/`：`colcon` 生成的构建日志。

`build/`、`install/` 和 `log/` 都是可重新生成的目录，不属于项目源码。

## 编译项目

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

## 启动项目

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py
```

该命令会自动完成以下操作：

- 启动 Gazebo Classic；
- 加载巡操机器人模型；
- 加载底盘、机械臂和夹爪控制接口；
- 打开独立的键盘遥控终端。

在启动命令所在的主终端按 `Ctrl+C`，可以统一关闭 Gazebo 和键盘遥控节点。

## 启动参数

无 Gazebo 图形界面启动：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py gui:=false
```

不启动键盘遥控终端：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py teleop:=false
```

以物理暂停状态启动：

```bash
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py paused:=true
```

## 键盘遥控

### 底盘

- `W` / `S`：前进 / 后退。
- `A` / `D`：左转 / 右转。
- `X` 或空格：停止底盘。

底盘连续 0.5 秒没有收到运动按键后会自动停止，防止键盘终端失去焦点时机器人继续运动。

### 六轴机械臂

- `1`～`6`：选择对应的机械臂关节。
- `[` / `]` 或 `-` / `=`：以 0.1 rad 为步长减小 / 增大所选关节角度。
- `R`：将六个机械臂关节恢复到零位。

键盘节点启动后会先读取 `/xczs/joint_states`，以机器人当前姿态初始化控制目标，
不会在启动时强制机械臂或夹爪回零。

### 夹爪

- `O`：打开夹爪。
- `P`：关闭夹爪。

### 其他

- `H` 或 `?`：显示完整按键帮助。
- `Q` 或 `Ctrl+C`：停止底盘并关闭键盘遥控节点。

## ROS 2 通信接口

- `/xczs/cmd_vel`：底盘速度指令，消息类型为 `geometry_msgs/msg/Twist`。
- `/xczs/odom`：底盘里程计，消息类型为 `nav_msgs/msg/Odometry`。
- `/xczs/joint_trajectory`：机械臂和夹爪关节指令，消息类型为
  `trajectory_msgs/msg/JointTrajectory`。
- `/xczs/joint_states`：机器人关节状态，消息类型为 `sensor_msgs/msg/JointState`。

## 注意事项

- 当前控制节点主要用于 Gazebo 功能测试。
- 底盘使用平面运动插件，不等同于真实驱动轮的动力学控制。
- 机械臂和夹爪使用 Gazebo 关节位置接口，不包含真实电机的力矩控制。
- 修改 C++ 源码后需要重新执行编译命令，再运行统一启动文件。
