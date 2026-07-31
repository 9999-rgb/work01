#!/bin/bash
# ============================================================================
# XCZS 巡操机器人 — 仿真 + Zenoh 桥 + 监控面板 一键启动脚本
#
# 用法:
#   ./start_xczs_bridge.sh                   # 启动全部（含 Gazebo GUI）
#   ./start_xczs_bridge.sh --no-gui          # 无 Gazebo 界面（headless）
#   ./start_xczs_bridge.sh --with-proxy      # 同时启动 CDR→JSON 代理
#   ./start_xczs_bridge.sh --manual          # 使用 GUI 手动控制
#   ./start_xczs_bridge.sh --web             # 使用浏览器控制
#   ./start_xczs_bridge.sh --nav2            # 使用 Nav2 自主导航
#
# 启动后:
#   - 监控面板: http://localhost:8080/monitor.html
#   - SSE 数据:  http://localhost:8001
#   - 传感器流:  http://localhost:8003
#   - Zenoh TCP: tcp/localhost:7447
# ============================================================================
set -eo pipefail

# ── 初始化 PID 变量（避免 cleanup 时未绑定） ─────────────────────
BRIDGE_PID=""
PROXY_PID=""
SSE_PID=""
SENSOR_PID=""
HTTP_PID=""
CONTROL_PID=""
CLEANUP_DONE="false"

# ── 路径配置 ──────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORK_DIR="$(dirname "$SCRIPT_DIR")"          # work01 根目录
JIANG_DIR="$WORK_DIR/jiang"
ZENOH_BRIDGE="/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds"
MONITOR_PORT="${MONITOR_PORT:-8080}"
BRIDGE_REST_PORT="${BRIDGE_REST_PORT:-8000}"
BRIDGE_TCP_PORT="${BRIDGE_TCP_PORT:-7447}"
SSE_PORT="${SSE_PORT:-8001}"
SENSOR_PORT="${SENSOR_PORT:-8003}"
SENSOR_HOST="${SENSOR_HOST:-0.0.0.0}"
CONTROL_PORT="${CONTROL_PORT:-8090}"
CONTROL_HOST="${CONTROL_HOST:-127.0.0.1}"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORK_DIR/install/setup.bash"
PYTHON_BIN="/usr/bin/python3"

# ── 选项 ──────────────────────────────────────────────────────────
GAZEBO_GUI="true"
WITH_PROXY="false"
CONTROL_MODE="manual"
NAV2_ENABLED="false"
NAV2_RVIZ="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-gui)    GAZEBO_GUI="false"; shift ;;
        --with-proxy) WITH_PROXY="true"; shift ;;
        --manual)    CONTROL_MODE="manual"; shift ;;
        --web)       CONTROL_MODE="web"; shift ;;
        --keyboard)  CONTROL_MODE="keyboard"; shift ;;
        --nav2)
            NAV2_ENABLED="true"
            NAV2_RVIZ="true"
            shift
            ;;
        -h|--help)
            sed -n '3,15p' "$0" | sed 's/^# *//'
            exit 0
            ;;
        *) echo "未知选项: $1"; exit 1 ;;
    esac
done

# ── 清理函数 ──────────────────────────────────────────────────────
cleanup() {
    if [ "$CLEANUP_DONE" = "true" ]; then
        return
    fi
    CLEANUP_DONE="true"
    echo ""
    echo "═══════════════════════════════════════════"
    echo "  正在关闭..."
    echo "═══════════════════════════════════════════"
    for pid in \
        ${BRIDGE_PID:-} \
        ${PROXY_PID:-} \
        ${SSE_PID:-} \
        ${SENSOR_PID:-} \
        ${HTTP_PID:-} \
        ${CONTROL_PID:-}; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    # 等待进程退出
    for pid in \
        ${BRIDGE_PID:-} \
        ${PROXY_PID:-} \
        ${SSE_PID:-} \
        ${SENSOR_PID:-} \
        ${HTTP_PID:-} \
        ${CONTROL_PID:-}; do
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
    echo "  已全部关闭"
}
trap cleanup EXIT INT TERM

# ── 前置检查 ──────────────────────────────────────────────────────
if [ ! -f "$ROS2_SETUP" ]; then
    echo "ERROR: ROS2 setup not found at $ROS2_SETUP"
    exit 1
fi
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "ERROR: Workspace not built. Run: cd $WORK_DIR && colcon build --symlink-install"
    exit 1
fi
if [ ! -x "$PYTHON_BIN" ]; then
    echo "ERROR: System Python not found at $PYTHON_BIN"
    exit 1
fi

# ── 加载 ROS2 环境 ────────────────────────────────────────────────
source "$ROS2_SETUP"
source "$WORKSPACE_SETUP"
if [ "$NAV2_ENABLED" = "true" ] &&
    ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    echo "ERROR: Nav2 binary packages are not installed."
    echo "Run: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
    exit 1
fi
if ! "$PYTHON_BIN" -c \
    "import aiohttp, rclpy, zenoh; from PIL import Image" \
    2>/dev/null; then
    echo "ERROR: Python dependencies are incomplete."
    echo "Run: $PYTHON_BIN -m pip install -r $JIANG_DIR/requirements.txt"
    exit 1
fi

# ── 自动检测 headless 环境 ─────────────────────────────────────────
if [ -z "${DISPLAY:-}" ]; then
    echo "⚠ 未检测到显示器 (DISPLAY 未设置)，自动切换为 headless 模式"
    GAZEBO_GUI="false"
    NAV2_RVIZ="false"
fi
if [ "$GAZEBO_GUI" = "false" ] && [ "$CONTROL_MODE" = "manual" ]; then
    echo "  GUI 不可用，自动切换为浏览器控制"
    CONTROL_MODE="web"
fi

echo "═══════════════════════════════════════════"
echo "  XCZS 巡操机器人仿真系统"
echo "═══════════════════════════════════════════"
echo ""
case "$CONTROL_MODE" in
    keyboard) MODE_LABEL="键盘遥控" ;;
    web) MODE_LABEL="浏览器控制" ;;
    *) MODE_LABEL="GUI 手动控制" ;;
esac
echo "  模式:       $MODE_LABEL"
echo "  Gazebo GUI: $GAZEBO_GUI"
echo "  Zenoh 代理: $(if [ "$WITH_PROXY" = "true" ]; then echo '启用'; else echo '禁用（桥自带 JSON）'; fi)"
echo "  Nav2 导航: $(if [ "$NAV2_ENABLED" = "true" ]; then echo '启用'; else echo '禁用'; fi)"
echo ""

# ── 函数：启动 Zenoh Bridge ────────────────────────────────────────
_start_bridge() {
    if [ ! -x "$ZENOH_BRIDGE" ]; then
        echo "ERROR: zenoh-bridge-ros2dds 未找到: $ZENOH_BRIDGE"
        exit 1
    fi
    "$ZENOH_BRIDGE" \
        --listen "tcp/0.0.0.0:$BRIDGE_TCP_PORT" \
        --rest-http-port "$BRIDGE_REST_PORT" \
        &
    BRIDGE_PID=$!
    sleep 2
    if kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "       Zenoh Bridge 已启动 (PID $BRIDGE_PID)"
        echo "       TCP: tcp/localhost:$BRIDGE_TCP_PORT"
        echo "       REST: http://localhost:$BRIDGE_REST_PORT"
    else
        echo "ERROR: Zenoh Bridge 启动失败"
        exit 1
    fi
}

_check_process() {
    local pid="$1"
    local label="$2"
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: $label 启动失败"
        exit 1
    fi
}

# ── 1. 启动 Zenoh Bridge ──────────────────────────────────────────
echo "[1/6] Zenoh Bridge..."

# 如果系统桥已存在且有 ros2dds（/zenoh_bridge_ros2dds 节点存在），直接复用
if lsof -ti:"$BRIDGE_TCP_PORT" >/dev/null 2>&1; then
    source "$ROS2_SETUP" 2>/dev/null
    if ros2 node list 2>/dev/null | grep -q zenoh_bridge_ros2dds; then
        echo "       系统 Zenoh 桥已运行且 ros2dds 正常，直接复用"
        echo "       TCP: tcp/localhost:$BRIDGE_TCP_PORT"
        BRIDGE_PID=""
    else
        # 端口被非 ROS 2 桥占用，使用不会与 SSE 冲突的备用端口。
        echo "       系统桥无 ros2dds，使用备用端口 7448"
        BRIDGE_TCP_PORT=7448
        BRIDGE_REST_PORT=8002
        _start_bridge
    fi
else
    _start_bridge
fi

# ── 2. 启动 CDR→JSON 代理（可选） ─────────────────────────────────
if [ "$WITH_PROXY" = "true" ]; then
    echo "[2/6] 启动 XCZS Zenoh 代理..."
    cd "$JIANG_DIR"
    "$PYTHON_BIN" run_xczs_proxy.py \
        --port "$BRIDGE_TCP_PORT" \
        --control-port 0 &
    PROXY_PID=$!
    sleep 1
    _check_process "$PROXY_PID" "XCZS Zenoh 代理"
    echo "       代理已启动 (PID $PROXY_PID)"
else
    echo "[2/6] 跳过 Zenoh 代理（使用桥自带 JSON 转发）"
    PROXY_PID=""
fi

# ── 3a. 启动 SSE 桥（Zenoh TCP → HTTP SSE） ─────────────────────
echo "[3a/6] 启动 SSE 数据桥 (port $SSE_PORT)..."
cd "$JIANG_DIR"
"$PYTHON_BIN" sse_bridge.py \
    --port "$SSE_PORT" \
    --zenoh "tcp/localhost:$BRIDGE_TCP_PORT" &
SSE_PID=$!
sleep 1
_check_process "$SSE_PID" "SSE 数据桥"
echo "       SSE 数据桥: http://localhost:$SSE_PORT/<key>"

# ── 3b. 启动 ROS 2 传感器 Web 流 ────────────────────────────────
echo "[3b/6] 启动传感器流服务 (port $SENSOR_PORT)..."
"$PYTHON_BIN" scripts/sensor_stream_server \
    --host "$SENSOR_HOST" \
    --port "$SENSOR_PORT" &
SENSOR_PID=$!
sleep 1
_check_process "$SENSOR_PID" "传感器流服务"
echo "       相机 MJPEG: http://localhost:$SENSOR_PORT/camera.mjpg"
echo "       雷达 WebSocket: ws://localhost:$SENSOR_PORT/lidar/ws"

# ── 3c. 启动 HTTP 文件服务器（monitor.html） ────────────────────────
echo "[3c/6] 启动 HTTP 服务器 (port $MONITOR_PORT)..."
"$PYTHON_BIN" -m http.server "$MONITOR_PORT" --bind 0.0.0.0 &
HTTP_PID=$!
sleep 1
_check_process "$HTTP_PID" "HTTP 文件服务器"
echo "       监控面板: http://localhost:$MONITOR_PORT/monitor.html"

# ── 4. 按需启动浏览器控制服务 ───────────────────────────────────
if [ "$CONTROL_MODE" = "web" ]; then
    echo "[4/6] 启动 Web 控制服务 (port $CONTROL_PORT)..."
    "$PYTHON_BIN" control_server.py \
        --host "$CONTROL_HOST" \
        --port "$CONTROL_PORT" &
    CONTROL_PID=$!
    sleep 1
    _check_process "$CONTROL_PID" "Web 控制服务"
    echo "       Web 控制: http://localhost:$CONTROL_PORT"
else
    echo "[4/6] 跳过 Web 控制服务（当前模式由专用 ROS 2 节点控制）"
fi

# ── 5. 显示访问地址 ───────────────────────────────────────────────
echo ""
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║  🌐 监控面板: http://localhost:$MONITOR_PORT/monitor.html"
echo "  ║  📡 SSE 数据:  http://localhost:$SSE_PORT"
echo "  ║  📷 传感器流:  http://localhost:$SENSOR_PORT"
echo "  ╚══════════════════════════════════════════════════════╝"
echo ""

# ── 6. 启动 Gazebo + 机器人 ───────────────────────────────────────
echo "[6/6] 启动 Gazebo + XCZS 机器人..."

if [ "$CONTROL_MODE" = "keyboard" ]; then
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=false" \
        "teleop:=true" \
        "nav2:=$NAV2_ENABLED" \
        "nav2_rviz:=$NAV2_RVIZ"
elif [ "$CONTROL_MODE" = "web" ]; then
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=false" \
        "teleop:=false" \
        "nav2:=$NAV2_ENABLED" \
        "nav2_rviz:=$NAV2_RVIZ"
else
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=true" \
        "nav2:=$NAV2_ENABLED" \
        "nav2_rviz:=$NAV2_RVIZ"
fi
