# XCZS 巡操机器人仿真项目

本项目基于 ROS 2 Humble 和 Gazebo Classic，实现 XCZS 巡操机器人的模型仿真与键盘遥控。
目前支持底盘移动、六轴机械臂控制和夹爪开合。

## 项目结构

- `xczs_inspection_robot_description`：机器人 Xacro、外观网格、碰撞网格和 Gazebo 仿真环境。
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
- 展开机器人 Xacro，并通过 `robot_state_publisher` 发布模型和 TF；
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
- `X`：按 S 曲线平滑减速并停止。
- 空格：立即发送零速度，仅用于需要快速停车的情况。

移动时需要长按方向键。底盘连续 0.75 秒没有收到运动按键后，会按 S 曲线自动减速，
防止键盘终端失去焦点时机器人继续运动。0.75 秒覆盖常见键盘首次连发延迟，
可以避免长按开始阶段出现“走—停—再走”。

底盘速度以 50 Hz 发布，五次 S 曲线同时限制加速度和加加速度（jerk）。
默认线加速度上限为 `0.5 m/s²`、线 jerk 上限为 `2.0 m/s³`，
角加速度上限为 `1.2 rad/s²`、角 jerk 上限为 `4.8 rad/s³`。
普通停车建议使用 `X`；空格、退出节点和关闭启动文件会立即发送零速度。

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

### 节点

| 节点 | 功能 |
| --- | --- |
| `/gazebo` | Gazebo Classic ROS 2 接口和仿真服务。 |
| `/robot_state_publisher` | 发布机器人描述及各关节 TF。 |
| `/xczs_keyboard_teleop` | 读取键盘，发布底盘、机械臂和夹爪指令。 |
| `/xczs/xczs_planar_move` | 接收底盘速度并发布里程计和 `odom → body` TF。 |
| `/xczs/xczs_joint_pose_trajectory` | 接收机械臂和夹爪关节目标。 |
| `/xczs/xczs_joint_state_publisher` | 发布 12 个可动关节的状态。 |

`teleop:=false` 时不会启动 `/xczs_keyboard_teleop`，其余节点仍会启动。

### 话题

| 话题 | 消息类型 | 发布方 → 订阅方 | 说明 |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → 使用仿真时间的节点 | Gazebo 仿真时钟。 |
| `/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` → 模型加载工具 | Xacro 展开后的完整 URDF 文本。 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo、`robot_state_publisher` → TF 使用方 | 机器人动态坐标变换。 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` → TF 使用方 | 固定坐标变换；当前模型没有固定关节，但接口会保留。 |
| `/xczs/cmd_vel` | `geometry_msgs/msg/Twist` | 键盘节点 → 平面运动插件 | `linear.y` 为前后速度，`angular.z` 为转向角速度，其他分量保持为 0。 |
| `/xczs/odom` | `nav_msgs/msg/Odometry` | 平面运动插件 → 用户节点 | 底盘里程计，`frame_id` 为 `odom`，`child_frame_id` 为 `body`。 |
| `/xczs/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 键盘节点 → 关节位置插件 | 六轴机械臂和两个夹爪关节的位置目标。 |
| `/xczs/joint_states` | `sensor_msgs/msg/JointState` | 关节状态插件 → 键盘节点、`robot_state_publisher` | 六轴机械臂、两个夹爪和四个车轮的角度与速度。 |

`/rosout` 和 `/parameter_events` 是 ROS 2 自动创建的日志、参数事件话题，
不属于机器人控制接口。

机械臂与夹爪轨迹使用以下八个关节名，顺序固定：

1. `body_arm1`
2. `arm1_arm2`
3. `arm2_arm3`
4. `arm3_arm4`
5. `arm4_arm5`
6. `arm5_end`
7. `end_worklink1`
8. `end_worklink2`

`/xczs/joint_states` 还会在上述八个关节之后发布四个连续车轮关节：
`body_frw`、`body_flw`、`body_brw` 和 `body_blw`。

### TF 结构

```text
odom
└── body
    ├── arm1
    │   └── arm2
    │       └── arm3
    │           └── arm4
    │               └── arm5
    │                   └── end
    │                       ├── worklink1
    │                       └── worklink2
    ├── front_right_wheel
    ├── front_left_wheel
    ├── back_right_wheel
    └── back_left_wheel
```

### Gazebo 服务

| 服务 | 服务类型 | 功能 |
| --- | --- | --- |
| `/spawn_entity` | `gazebo_msgs/srv/SpawnEntity` | 向仿真中加载实体。 |
| `/delete_entity` | `gazebo_msgs/srv/DeleteEntity` | 从仿真中删除实体。 |
| `/get_entity_state` | `gazebo_msgs/srv/GetEntityState` | 查询实体位姿和速度。 |
| `/get_model_list` | `gazebo_msgs/srv/GetModelList` | 查询当前模型列表。 |
| `/pause_physics` | `std_srvs/srv/Empty` | 暂停物理仿真。 |
| `/unpause_physics` | `std_srvs/srv/Empty` | 恢复物理仿真。 |
| `/reset_world` | `std_srvs/srv/Empty` | 重置模型状态。 |
| `/reset_simulation` | `std_srvs/srv/Empty` | 重置模型状态和仿真时间。 |
| `/apply_joint_effort` | `gazebo_msgs/srv/ApplyJointEffort` | 对关节施加测试力矩。 |
| `/clear_joint_efforts` | `gazebo_msgs/srv/JointRequest` | 清除关节测试力矩。 |
| `/apply_link_wrench` | `gazebo_msgs/srv/ApplyLinkWrench` | 对部件施加测试力或力矩。 |
| `/clear_link_wrenches` | `gazebo_msgs/srv/LinkRequest` | 清除部件测试力和力矩。 |

### 键盘节点参数

默认参数集中在
`xczs_inspection_robot_control/config/keyboard_teleop.yaml`：

| 参数 | 默认值 | 单位或说明 |
| --- | ---: | --- |
| `linear_speed` | `0.25` | 最大前后速度，m/s。 |
| `angular_speed` | `0.60` | 最大转向角速度，rad/s。 |
| `linear_acceleration` | `0.50` | 线加速度上限，m/s²。 |
| `angular_acceleration` | `1.20` | 角加速度上限，rad/s²。 |
| `linear_jerk` | `2.00` | 线 jerk 上限，m/s³。 |
| `angular_jerk` | `4.80` | 角 jerk 上限，rad/s³。 |
| `base_command_rate` | `50.0` | 底盘速度发布频率，Hz。 |
| `command_timeout` | `0.75` | 最后一次移动按键后的安全超时，s。 |
| `joint_step` | `0.10` | 单次机械臂调节步长，rad。 |
| `joint_limit` | `2.80` | 键盘控制使用的软件角度限位，rad。 |
| `gripper_open_angle` | `0.35` | 单侧夹爪打开角度，rad。 |
| `joint_command_rate` | `20.0` | 关节指令重复发布频率，Hz。 |
| `joint_command_repeats` | `5` | 每次操作重复发布关节指令的次数。 |
| `cmd_vel_topic` | `/xczs/cmd_vel` | 底盘速度话题。 |
| `joint_trajectory_topic` | `/xczs/joint_trajectory` | 关节目标话题。 |
| `joint_state_topic` | `/xczs/joint_states` | 关节状态话题。 |

统一启动文件会在键盘节点启动时读取该 YAML。当前参数在节点启动时生效；
修改配置后需要重新启动键盘节点，运行中执行 `ros2 param set` 不会改变已经加载的控制曲线。

## 注意事项

- 当前控制节点主要用于 Gazebo 功能测试。
- 机器人描述源文件位于
  `xczs_inspection_robot_description/urdf/xczs_inspection_robot.urdf.xacro`。
- 本次平滑控制只调整速度指令曲线，没有修改质量、惯量、摩擦、接触刚度或
  Gazebo ODE 求解参数。
- 底盘使用平面运动插件，不等同于真实驱动轮的动力学控制。
- 四个车轮使用无角度限位的连续关节，可以持续滚动。
- 机械臂和夹爪使用 Gazebo 关节位置接口，不包含真实电机的力矩控制。
- 修改 C++ 源码后需要重新执行编译命令，再运行统一启动文件。
