#!/bin/bash
# ============================================================================
# XCZS 巡操机器人仿真系统 — 一键启动
#
# 用法:
#   ./run_all.sh                  # 全部启动（Gazebo + 桥 + 监控 + Qt GUI）
#   ./run_all.sh --manual         # GUI 手动控制模式
#   ./run_all.sh --web            # 浏览器控制模式（最常用）
#   ./run_all.sh --task           # 自主任务调度器模式
#   ./run_all.sh --with-proxy     # 附加 CDR→JSON 代理（精细化数据处理）
#   ./run_all.sh --no-gui         # headless 模式
#   CONTROL_HOST=0.0.0.0 ./run_all.sh --web   # 供其他计算机访问 Web 控制
#
# 启动后:
#   监控面板: http://localhost:8080/monitor.html
#   SSE 数据:  http://localhost:8001
#   传感器流:  http://localhost:8003
# ============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 转发所有参数给 jiang/start_xczs_bridge.sh
exec "$SCRIPT_DIR/jiang/start_xczs_bridge.sh" "$@"
