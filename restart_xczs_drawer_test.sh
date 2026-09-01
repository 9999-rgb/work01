#!/bin/bash
# XCZS electrical_mezzanine drawer test stack — two-part restart
# Part 1: Gazebo world (gzserver + scene).  Part 2: toolset supervisor (robot).
set -e
cd /home/live/work01
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /home/live/work01/install/setup.bash

# --- Gazebo 模型解析必须本地化（否则 gzserver 启动会挂起） -----------------
# world 里的 model://sun 与 model://ground_plane 只在 GAZEBO_MODEL_PATH 找不到
# 时会去远程模型库下载；本机走代理（198.18.0.225:80 连接失败重试循环）会卡死
# gzserver，ogre.log 恰好停在 "Creating resource group ShaderGeneratorResourceGroup"
# （曾误判为 OGRE 着色器缓存损坏）。与 run_all.sh 保持一致：
#   1) 禁用远程模型库（GAZEBO_MODEL_DATABASE_URI=""）；
#   2) 把系统模型路径 /usr/share/gazebo-11/models 前置进 GAZEBO_MODEL_PATH，
#      让 sun/ground_plane 等从本地解析。
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

# --- 幂等清场：杀掉上一轮残留的 gzserver / supervisor / launch / 全部节点 ---
# 关键：不能只杀 launch/supervisor 父进程 —— SIGKILL 父进程后其子节点会 reparent
# 到 systemd --user 永久存活。本轮 db1 首个续租即 LEASE_LOST 的根因就是上一轮
# 残留的 operation_lease_coordinator 与当轮协调器同时应答同一 ACQUIRE、双授租约。
# pose_authority / grasp_aggregator / trajectory_router 同理，多份残留会双应答。
# 因此除父进程外，还必须按二进制路径直接杀掉所有孤儿节点（含 nav2 / moveit）。
# gzserver 里可能残留上一轮已 spawn 的机器人模型，不杀掉会导致本轮重复 spawn。
# （gzserver 若被 SIGKILL 打断在 OGRE 着色器缓存写入期间会损坏缓存，见下方
#   wait_spawn 的兜底分支。）
for p in $(pgrep -x gzserver 2>/dev/null || true); do kill -9 "$p" 2>/dev/null || true; done
pkill -9 -f "toolset_supervisor[.]py" 2>/dev/null || true
pkill -9 -f "ros2 launch xczs_inspection_robot_[b]ringup" 2>/dev/null || true
pkill -9 -f "xczs_inspection_robot_control" 2>/dev/null || true
pkill -9 -f "xczs_inspection_robot_nav2" 2>/dev/null || true
pkill -9 -f "moveit_ros_move_group/move_group" 2>/dev/null || true
pkill -9 -f "/opt/ros/humble/lib/nav2_" 2>/dev/null || true
pkill -9 -f "/controller_manager/spawner" 2>/dev/null || true
pkill -9 -f "/robot_state_publisher" 2>/dev/null || true
sleep 2

WORLD_LAUNCH=(
  ros2 launch xczs_inspection_robot_bringup inspection_robot.launch.py
  gui:=false gazebo:=true robot_bringup:=false use_sim_time:=true
  control_gui:=false teleop:=false moveit:=false moveit_rviz:=false
  nav2:=false nav2_rviz:=false
  world:=/home/live/work01/xczs_inspection_robot_gazebo/worlds/inspection_robot.world
  robot_name:=xczs_inspection_robot
  cabinet_robot_adapter:=/home/live/work01/jiang/data/assets/cabinet/demo_cabinet/cabinet_robot_adapter.yaml
  cabinet_bringup:=false spawn_cabinet:=false
  scene:=electrical_mezzanine
  scenes_config:=/home/live/work01/jiang/data/assets/scene/electrical_mezzanine/scenes.yaml
  spawn_z:=0.515 cabinet_pose_source:=static
)

# 等待 gzserver 起来并真正响应 /spawn_entity。service list 可能显示刚死节点
# 的陈旧 DDS 公告，所以用真实（空 xml -> 报错响应）service call 探测。
start_world() {
  "${WORLD_LAUNCH[@]}" > /tmp/xczs_world.log 2>&1 &
  WORLD_PID=$!
}
wait_spawn() {
  for i in $(seq 1 25); do
    if timeout 5 ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{}" >/dev/null 2>&1; then
      echo "gzserver + /spawn_entity responding after $((i*2))s"
      return 0
    fi
    if ! kill -0 $WORLD_PID 2>/dev/null; then
      echo "world launch exited early ($i iterations)"
      return 1
    fi
    sleep 2
  done
  return 1
}

start_world
if ! wait_spawn; then
  # 兜底分支：上述环境变量修复后基本不会走到这里。50s 内不响应且进程仍存活
  # 时，清空 OGRE 着色器缓存（/tmp/gazebo-live-rtshaderlibcache）并重启一次
  # world；进程曾被 SIGKILL 打断在缓存写入期间损坏时这一补救有效。
  echo "gzserver not responding in 50s -> clear OGRE shader cache and retry"
  kill -9 $WORLD_PID 2>/dev/null || true
  sleep 1
  for p in $(pgrep -x gzserver 2>/dev/null || true); do kill -9 "$p" 2>/dev/null || true; done
  sleep 1
  rm -rf /tmp/gazebo-live-rtshaderlibcache
  start_world
  wait_spawn
fi

# Part 2: toolset supervisor
# --ready-timeout 420：机器人模型首轮 spawn 因 OGRE 着色器缓存重建可能耗时
# ~200s，随后才链式启动控制器 spawner -> 位姿校验 -> move_group/nav2。120s 会
# 误判超时并把 robot 子进程杀掉（本轮已踩坑）。缓存命中时启动很快，无需改动。
exec /usr/bin/python3 /home/live/work01/jiang/scripts/toolset_supervisor.py \
--toolset A --robot-name xczs_inspection_robot --initial-spawn-cabinet true --cabinet-bringup true --ready-timeout 420 --require-nav2 --require-cabinet-action --cabinet-action /xczs/cabinet/cabinet_a/operate_cabinet_control --cabinet-action /xczs/cabinet/cabinet_b/operate_cabinet_control --cabinet-action /xczs/cabinet/cabinet_c/operate_cabinet_control --cabinet-action /xczs/cabinet/generator_plant/operate_cabinet_control --cabinet-action /xczs/cabinet/electrical_mezzanine/operate_cabinet_control --launch-arg use_sim_time:=true --launch-arg moveit:=true --launch-arg nav2:=true --launch-arg robot_name:=xczs_inspection_robot --launch-arg robot_xacro:=/home/live/work01/xczs_inspection_robot_description/urdf/xczs_inspection_robot.urdf.xacro --launch-arg nav2_launch:=/home/live/work01/xczs_inspection_robot_nav2/launch/navigation.launch.py --launch-arg nav2_params_file:=/home/live/work01/xczs_inspection_robot_nav2/config/nav2_params.yaml --launch-arg robot_control:=/home/live/work01/xczs_inspection_robot_control/config/robot_control.yaml --launch-arg cabinet_instances:=/home/live/work01/xczs_inspection_robot_control/config/cabinet_instances.yaml --launch-arg cabinet_controls:=/home/live/work01/jiang/data/assets/cabinet/demo_cabinet/cabinet_controls.yaml --launch-arg cabinet_scene:=/home/live/work01/jiang/data/assets/cabinet/demo_cabinet/cabinet_scene.yaml --launch-arg cabinet_pose:=/home/live/work01/jiang/data/assets/cabinet/demo_cabinet/cabinet_pose.yaml --launch-arg cabinet_robot_adapter:=/home/live/work01/jiang/data/assets/cabinet/demo_cabinet/cabinet_robot_adapter.yaml --launch-arg cabinet_xacro:=/home/live/work01/jiang/data/assets/cabinet/demo_cabinet/control_cabinet.urdf.xacro --launch-arg scene:=electrical_mezzanine --launch-arg scenes_config:=/home/live/work01/jiang/data/assets/scene/electrical_mezzanine/scenes.yaml --launch-arg spawn_z:=0.515 --launch-arg cabinet_pose_source:=static --launch-arg moveit_config_package:=xczs_inspection_robot_moveit_config --launch-arg moveit_srdf:=/home/live/work01/xczs_inspection_robot_moveit_config/config/xczs_inspection_robot_toolset_{toolset}.srdf --launch-arg moveit_kinematics:=/home/live/work01/xczs_inspection_robot_moveit_config/config/kinematics.yaml --launch-arg moveit_joint_limits:=/home/live/work01/xczs_inspection_robot_moveit_config/config/joint_limits_toolset_{toolset}.yaml --launch-arg moveit_controllers:=/home/live/work01/xczs_inspection_robot_moveit_config/config/moveit_controllers_toolset_{toolset}.yaml --launch-arg moveit_rviz_config:=/home/live/work01/xczs_inspection_robot_moveit_config/config/moveit.rviz --launch-arg moveit_launch:=/home/live/work01/xczs_inspection_robot_moveit_config/launch/move_group.launch.py
