#!/bin/bash
# ============================================================================
# XCZS 巡操机器人仿真系统 — 一键启动
#
# 用法:
#   ./run_all.sh                  # 全部启动（Gazebo + 桥 + 监控 + 任务调度器）
#   ./run_all.sh --manual         # GUI 手动控制模式
#   ./run_all.sh --with-proxy     # 附加 CDR→JSON 代理（精细化数据处理）
#   ./run_all.sh --no-gui         # headless 模式
#
# 启动后:
#   监控面板: http://localhost:8080/monitor.html
#   REST API:  http://localhost:8000
# ============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 转发所有参数给 jiang/start_xczs_bridge.sh
exec "$SCRIPT_DIR/jiang/start_xczs_bridge.sh" "$@"
