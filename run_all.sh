#!/bin/bash
# ============================================================================
# XCZS 巡操机器人仿真系统 — 一键启动
#
# 用法:
#   ./run_all.sh                  # 统一 Web 控制（Gazebo + Nav2 + 控制柜任务）
#   ./run_all.sh --web            # 兼容写法，与无参数启动相同
#   ./run_all.sh --with-proxy     # 附加 CDR→JSON 代理（精细化数据处理）
#   同机隔离启动（端口应避开其他实例）:
#     ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 BRIDGE_TCP_PORT=17447 \
#       BRIDGE_REST_PORT=18000 CONTROL_HOST=127.0.0.1 CONTROL_PORT=18090 \
#       XCZS_CONTROL_ORIGINS=http://localhost:18090,http://127.0.0.1:18090 \
#       ./run_all.sh --web
#
#
# 启动后（默认监听所有网卡 :8090，其他电脑可访问）:
#   监控面板: http://<服务器IP>:8090/monitor.html
#   API 文档:  http://<服务器IP>:8090/docs
#   SSE 数据:  http://<服务器IP>:8090/sse/<key>
#   传感器流:  http://<服务器IP>:8090/camera.mjpg
# ============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 末端工具套装属于互斥的 URDF / ros2_control / MoveIt 拓扑。本地 Web+Gazebo
# 模式由启动入口保持 Gazebo 世界并替换机器人运行栈；不会重启整套仿真。
exec "$SCRIPT_DIR/jiang/start_xczs_bridge.sh" "$@"
