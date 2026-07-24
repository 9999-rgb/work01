#!/usr/bin/env bash
# =============================================================================
# Zenoh 数据桥 + Proxy 启动脚本
# =============================================================================
# 启动 zenoh-bridge-ros2dds（ROS2 ↔ Zenoh 桥）和 run_proxy.py（CDR → JSON），
# 将 ROS2 仿真数据实时转发到浏览器监控面板。
#
# 数据流：
#   ROS2 节点 → zenoh-bridge-ros2dds → Zenoh 网络
#                   ↓
#         tcp/0.0.0.0:7447  (Zenoh 原生协议)
#         ws/0.0.0.0:10000  (WebSocket)
#         http://:8000      (REST SSE，浏览器直连)
#                   ↓
#         run_proxy.py (CDR 反序列化 → JSON)
#                   ↓
#         monitor.html (SSE 实时显示)
#
# Usage:
#   ./start_zenoh_proxy.sh          # 启动桥 + 代理
#   ./start_zenoh_proxy.sh stop     # 停止所有相关进程
# =============================================================================

set -eo pipefail

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
ZENOH_BRIDGE="/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds"
PROXY_SCRIPT="$SCRIPT_DIR/run_proxy.py"

# Zenoh bridge 监听端点
ZENOH_TCP="tcp/0.0.0.0:7447"
ZENOH_WS="ws/0.0.0.0:10000"
ZENOH_REST_PORT="8000"

# HTTP 控制服务器端口
CONTROL_PORT="${CONTROL_PORT:-8090}"

# 进程名标识
BRIDGE_NAME="zenoh-bridge-ros2dds"
PROXY_NAME="run_proxy.py"

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
info()  { echo -e "\033[1;32m[INFO]\033[0m  $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m  $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; }

# -----------------------------------------------------------------------------
# Stop mode
# -----------------------------------------------------------------------------
do_stop() {
    info "Stopping $BRIDGE_NAME and $PROXY_NAME …"
    pkill -f "$BRIDGE_NAME" 2>/dev/null || true
    pkill -f "$PROXY_NAME"  2>/dev/null || true
    sleep 1
    local remaining
    remaining=$(ps aux | grep -E "$BRIDGE_NAME|$PROXY_NAME" | grep -v grep | grep -v defunct | wc -l)
    if [[ "$remaining" -eq 0 ]]; then
        info "All stopped."
    else
        warn "$remaining process(es) still running, trying SIGKILL …"
        pkill -9 -f "$BRIDGE_NAME" 2>/dev/null || true
        pkill -9 -f "$PROXY_NAME"  2>/dev/null || true
    fi
    exit 0
}

if [[ "${1:-}" == "stop" ]]; then
    do_stop
fi

# -----------------------------------------------------------------------------
# Prerequisites
# -----------------------------------------------------------------------------
if [[ ! -f "$ROS_SETUP" ]]; then
    error "ROS 2 Humble setup not found: $ROS_SETUP"
    exit 1
fi

if [[ ! -x "$ZENOH_BRIDGE" ]]; then
    error "zenoh-bridge-ros2dds not found: $ZENOH_BRIDGE"
    exit 1
fi

if [[ ! -f "$PROXY_SCRIPT" ]]; then
    error "run_proxy.py not found: $PROXY_SCRIPT"
    exit 1
fi

# -----------------------------------------------------------------------------
# Kill old instances
# -----------------------------------------------------------------------------
OLD_BRIDGE=$(ps aux | grep "$BRIDGE_NAME" | grep -v grep | grep -v defunct | awk '{print $2}' || true)
OLD_PROXY=$(ps aux | grep "$PROXY_NAME" | grep -v grep | grep -v defunct | awk '{print $2}' || true)

if [[ -n "$OLD_BRIDGE" ]] || [[ -n "$OLD_PROXY" ]]; then
    warn "Found existing instances, stopping them first …"
    echo "$OLD_BRIDGE" | xargs -r kill 2>/dev/null || true
    echo "$OLD_PROXY"  | xargs -r kill 2>/dev/null || true
    sleep 2
    # Force kill if still alive
    echo "$OLD_BRIDGE" | xargs -r kill -9 2>/dev/null || true
    echo "$OLD_PROXY"  | xargs -r kill -9 2>/dev/null || true
    sleep 1
fi

# -----------------------------------------------------------------------------
# Source ROS 2
# -----------------------------------------------------------------------------
info "Sourcing ROS 2 Humble …"
# shellcheck disable=SC1090
source "$ROS_SETUP"

# -----------------------------------------------------------------------------
# Start zenoh-bridge-ros2dds
# -----------------------------------------------------------------------------
info "Starting $BRIDGE_NAME …"
info "  TCP:       $ZENOH_TCP"
info "  WebSocket: $ZENOH_WS"
info "  REST SSE:  http://localhost:$ZENOH_REST_PORT"

"$ZENOH_BRIDGE" \
    --listen "$ZENOH_TCP" \
    --listen "$ZENOH_WS" \
    --rest-http-port "$ZENOH_REST_PORT" &
BRIDGE_PID=$!
echo "  PID: $BRIDGE_PID"

# 等待 bridge 就绪
info "Waiting for bridge to be ready …"
for i in $(seq 1 15); do
    if curl -s "http://localhost:$ZENOH_REST_PORT/" >/dev/null 2>&1; then
        info "Bridge REST endpoint is up."
        break
    fi
    if [[ "$i" -eq 15 ]]; then
        error "Bridge failed to start within 15 seconds."
        kill "$BRIDGE_PID" 2>/dev/null || true
        exit 1
    fi
    sleep 1
done

# -----------------------------------------------------------------------------
# Start run_proxy.py
# -----------------------------------------------------------------------------
info "Starting $PROXY_SCRIPT …"
python3 "$PROXY_SCRIPT" --control-port "$CONTROL_PORT" &
PROXY_PID=$!
echo "  PID: $PROXY_PID"

# -----------------------------------------------------------------------------
# Status
# -----------------------------------------------------------------------------
info "=============================================="
info "  Zenoh 数据桥已启动"
info "  Bridge PID:  $BRIDGE_PID  (REST: http://localhost:$ZENOH_REST_PORT)"
info "  Proxy PID:   $PROXY_PID  (Control: http://localhost:$CONTROL_PORT)"
info "=============================================="
info "在浏览器打开 monitor.html 可查看实时数据并控制机器人。"
info "  POST /cmd_vel          底盘速度控制"
info "  POST /joint_trajectory 机械臂关节控制"
info "  GET  /health           健康检查"
info "停止命令: $0 stop"

# Keep the script running so the user sees the status and can Ctrl+C
wait
