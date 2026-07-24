#!/bin/bash
# ============================================================================
# XCZS 巡操机器人 — 仿真 + Zenoh 桥 + 监控面板 一键启动脚本
#
# 用法:
#   ./start_xczs_bridge.sh                   # 启动全部（含 Gazebo GUI）
#   ./start_xczs_bridge.sh --no-gui          # 无 Gazebo 界面（headless）
#   ./start_xczs_bridge.sh --with-proxy      # 同时启动 CDR→JSON 代理
#   ./start_xczs_bridge.sh --manual          # 使用 GUI 手动控制（非任务调度器）
#
# 启动后:
#   - 监控面板: http://localhost:8080/monitor.html
#   - REST API:  http://localhost:8000
#   - Zenoh TCP: tcp/localhost:7447
# ============================================================================
set -eo pipefail

# ── 初始化 PID 变量（避免 cleanup 时未绑定） ─────────────────────
BRIDGE_PID=""
PROXY_PID=""
HTTP_PID=""

# ── 路径配置 ──────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORK_DIR="$(dirname "$SCRIPT_DIR")"          # work01 根目录
JIANG_DIR="$WORK_DIR/jiang"
ZENOH_BRIDGE="/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds"
MONITOR_PORT="${MONITOR_PORT:-8080}"
BRIDGE_REST_PORT="${BRIDGE_REST_PORT:-8000}"
BRIDGE_TCP_PORT="${BRIDGE_TCP_PORT:-7447}"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORK_DIR/install/setup.bash"

# ── 选项 ──────────────────────────────────────────────────────────
GAZEBO_GUI="true"
WITH_PROXY="false"
TASK_MODE="manual"         # 默认: GUI 手动控制, "task" = 任务调度器

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-gui)    GAZEBO_GUI="false"; shift ;;
        --with-proxy) WITH_PROXY="true"; shift ;;
        --manual)    TASK_MODE="manual"; shift ;;
        --task)      TASK_MODE="task"; shift ;;
        -h|--help)
            sed -n '2,14p' "$0" | sed 's/^# //'
            exit 0
            ;;
        *) echo "未知选项: $1"; exit 1 ;;
    esac
done

# ── 清理函数 ──────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "═══════════════════════════════════════════"
    echo "  正在关闭..."
    echo "═══════════════════════════════════════════"
    for pid in ${BRIDGE_PID:-} ${PROXY_PID:-} ${HTTP_PID:-}; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    # 等待进程退出
    for pid in ${BRIDGE_PID:-} ${PROXY_PID:-} ${HTTP_PID:-}; do
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

# ── 加载 ROS2 环境 ────────────────────────────────────────────────
source "$ROS2_SETUP"
source "$WORKSPACE_SETUP"

echo "═══════════════════════════════════════════"
echo "  XCZS 巡操机器人仿真系统"
echo "═══════════════════════════════════════════"
echo ""
echo "  模式:       $(if [ "$TASK_MODE" = "task" ]; then echo '任务调度器'; else echo 'GUI 手动控制'; fi)"
echo "  Gazebo GUI: $GAZEBO_GUI"
echo "  Zenoh 代理: $(if [ "$WITH_PROXY" = "true" ]; then echo '启用'; else echo '禁用（桥自带 JSON）'; fi)"
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

# ── 1. 启动 Zenoh Bridge ──────────────────────────────────────────
echo "[1/4] Zenoh Bridge..."

BRIDGE_ALREADY_RUNNING=false
if lsof -ti:"$BRIDGE_TCP_PORT" >/dev/null 2>&1; then
    # 检查 REST API 是否可达
    if curl -s --max-time 2 "http://localhost:$BRIDGE_REST_PORT/@/router/local" >/dev/null 2>&1; then
        echo "       复用已有的 Zenoh Bridge（端口 $BRIDGE_TCP_PORT, REST $BRIDGE_REST_PORT）"
        BRIDGE_PID=""
        BRIDGE_ALREADY_RUNNING=true
    else
        # 桥在运行但 REST 不可用 → 需要替换
        echo "       已有的桥缺少 REST 插件，正在替换..."
        pkill -9 -f "zenoh-bridge-ros2dds" 2>/dev/null || true
        sleep 2
    fi
fi

if [ "$BRIDGE_ALREADY_RUNNING" = false ]; then
    _start_bridge
fi

# ── 2. 启动 CDR→JSON 代理（可选） ─────────────────────────────────
if [ "$WITH_PROXY" = "true" ]; then
    echo "[2/4] 启动 XCZS Zenoh 代理..."
    cd "$JIANG_DIR"
    python3 run_xczs_proxy.py &
    PROXY_PID=$!
    sleep 1
    echo "       代理已启动 (PID $PROXY_PID)"
else
    echo "[2/4] 跳过 Zenoh 代理（使用桥自带 JSON 转发）"
    PROXY_PID=""
fi

# ── 3. 启动 HTTP 文件服务器（monitor.html） ────────────────────────
echo "[3/4] 启动 HTTP 服务器 (port $MONITOR_PORT)..."
cd "$JIANG_DIR"
python3 -m http.server "$MONITOR_PORT" --bind 0.0.0.0 &
HTTP_PID=$!
sleep 1
echo "       监控面板: http://localhost:$MONITOR_PORT/monitor.html"

# ── 4. 启动 Gazebo + 机器人 ───────────────────────────────────────
echo "[4/4] 启动 Gazebo + XCZS 机器人..."
echo ""
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║  🌐 监控面板: http://localhost:$MONITOR_PORT/monitor.html"
echo "  ║  📡 REST API:  http://localhost:$BRIDGE_REST_PORT"
echo "  ╚══════════════════════════════════════════════════════╝"
echo ""

if [ "$TASK_MODE" = "task" ]; then
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=false" \
        "task_scheduler:=true"
else
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=true"
fi
