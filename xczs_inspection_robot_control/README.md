# XCZS Inspection Robot Keyboard Control

This ROS 2 package provides interactive keyboard control for the XCZS
inspection robot in Gazebo Classic.

## Interfaces

- Base command: `/xczs/cmd_vel` (`geometry_msgs/msg/Twist`)
- Joint command: `/xczs/joint_trajectory`
  (`trajectory_msgs/msg/JointTrajectory`)
- Joint feedback: `/xczs/joint_states` (`sensor_msgs/msg/JointState`)
- Base odometry: `/xczs/odom` (`nav_msgs/msg/Odometry`)

The Gazebo planar-motion plugin is used for initial simulation testing. It
provides reliable base motion without assuming a standard differential-drive
axis layout. A physical wheel controller can replace it after the chassis
coordinate system and wheel kinematics are finalized.

## Run

Build both packages:

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

Start Gazebo in the first terminal:

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xczs_inspection_robot_description gazebo.launch.py paused:=false
```

Start keyboard control in a second terminal:

```bash
cd /home/live/work01
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xczs_inspection_robot_control keyboard_teleop
```

## Key Map

- `W` / `S`: move the base forward / backward
- `A` / `D`: turn the base left / right
- `X` or `Space`: stop the base
- `1` ... `6`: select an arm joint
- `[` / `]`: decrease / increase the selected joint angle
- `R`: reset all arm joints to zero
- `O` / `P`: open / close the gripper
- `H` or `?`: display help
- `Q` or `Ctrl-C`: stop the base and quit

The base command has a 0.5-second dead-man timeout. Hold or repeatedly press a
motion key to keep moving. All speeds, joint increments and timeouts are
available as ROS 2 node parameters.
