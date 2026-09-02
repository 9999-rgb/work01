#!/bin/bash
# P2 headless acceptance: gzserver + test world + plugin.
source /opt/ros/humble/setup.bash
cd /home/live/work01
source install/setup.bash
export ROS_DOMAIN_ID=42
# ModelDB 挂起修复（与 restart_xczs_drawer_test.sh / run_all.sh 一致）：
# world 的 model://sun / model://ground_plane 走远程模型库会被本机代理卡死。
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH=/home/live/work01/install/xczs_inspection_robot_gazebo/lib
export LD_LIBRARY_PATH=/home/live/work01/install/xczs_inspection_robot_interfaces/lib:$LD_LIBRARY_PATH
exec gzserver --verbose --seed 1 -slibgazebo_ros_init.so /home/live/work01/scratch_drawer_p2/world_drawer_test.sdf
