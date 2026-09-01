#!/bin/bash
# P2 headless acceptance: gzserver + test world + plugin.
source /opt/ros/humble/setup.bash
cd /home/live/work01
source install/setup.bash
export ROS_DOMAIN_ID=42
export GAZEBO_PLUGIN_PATH=/home/live/work01/install/xczs_inspection_robot_gazebo/lib
export LD_LIBRARY_PATH=/home/live/work01/install/xczs_inspection_robot_interfaces/lib:$LD_LIBRARY_PATH
exec gzserver --verbose --seed 1 -slibgazebo_ros_init.so /home/live/work01/scratch_drawer_p2/world_drawer_test.sdf
