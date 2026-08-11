#!/bin/bash
# ============================================================================
# XCZS 巡操机器人仿真系统 — 一键启动
#
# 用法:
#   ./run_all.sh                  # 统一 Web 控制（Gazebo + Nav2 + 控制柜任务）
#   ./run_all.sh --web            # 兼容写法，与无参数启动相同
#   ./run_all.sh --with-proxy     # 附加 CDR→JSON 代理（精细化数据处理）
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

# 转发所有参数给 jiang/start_xczs_bridge.sh
exec "$SCRIPT_DIR/jiang/start_xczs_bridge.sh" "$@"
