#!/usr/bin/env python3
"""
水电站巡检机器人 — Zenoh 代理启动脚本

在这里集中管理所有需要透传的 ROS2 话题。
运行方式：
    python run_proxy.py

前置条件：
    # 终端 1
    zenohd

    # 终端 2
    zenoh-bridge-ros2dds
"""

from zenoh_proxy import (
    get_registry,
    load_message_type,
    add_message_path,
    register_standard_types,
)
from zenoh_proxy.runner import ProxyRunner

# ============================================================================
# 1. 如果你的自定义消息包不在标准路径，添加搜索路径
# ============================================================================
# add_message_path("./ros2_ws/install/lib/python3.10/site-packages")

# ============================================================================
# 2. 注册标准 ROS2 消息类型（替代旧 if-elif 链）
# ============================================================================
register_standard_types()

# ============================================================================
# 3. 注册你的自定义话题 — 下面逐条添加
# ============================================================================
registry = get_registry()

# ---- 方式 A：纯透传（只需反序列化 → JSON，无需处理）----
# 适用于：/odom, /scan, /joint_states 等标准类型

# Twist = load_message_type("geometry_msgs.msg.Twist")
# Pose = load_message_type("geometry_msgs.msg.Pose")
# Odometry = load_message_type("nav_msgs.msg.Odometry")
# LaserScan = load_message_type("sensor_msgs.msg.LaserScan")
# JointState = load_message_type("sensor_msgs.msg.JointState")

# registry.register("/odom", Odometry)
# registry.register("/scan", LaserScan)
# registry.register("/joint_states", JointState)

# ---- 方式 B：带自定义处理（在 JSON 序列化前做转换/增强）----
# 适用于：需要过滤字段、添加时间戳、合并数据等场景

# @registry.register_topic("/cmd_vel", Twist, description="底盘速度指令")
# def handle_cmd_vel(topic, data):
#     """在透传的 cmd_vel 数据上添加时间戳"""
#     import time
#     data["_proxy_ts"] = time.time()
#     return data

# ---- 方式 C：自定义消息类型（项目中后期定义）----
# 适用于：my_msgs/msg/MissionStatus, my_msgs/msg/Metrics 等
# 前提：已通过 colcon build 编译了自定义消息包

# MissionStatus = load_message_type("my_msgs.msg.MissionStatus")
# Metrics = load_message_type("my_msgs.msg.Metrics")
#
# registry.register("/mission/phase/*", MissionStatus)
# registry.register("/metrics", Metrics)

# ============================================================================
# 4. 启动代理
# ============================================================================
if __name__ == "__main__":
    runner = ProxyRunner(
        host="127.0.0.1",   # Zenoh 路由器地址
        port=7447,           # Zenoh 路由器端口
    )
    runner.connect()
    runner.subscribe_all_registered()  # 自动订阅上面注册的所有模式
    runner.spin()                      # 阻塞运行，Ctrl+C 退出
