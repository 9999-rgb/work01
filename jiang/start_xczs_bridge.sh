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
SSE_PID=""
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
TASK_MODE="manual"    # 默认: GUI 手动控制, "task" = 任务调度器, "keyboard" = 键盘

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-gui)    GAZEBO_GUI="false"; shift ;;
        --with-proxy) WITH_PROXY="true"; shift ;;
        --manual)    TASK_MODE="manual"; shift ;;
        --task)      TASK_MODE="task"; shift ;;
        --keyboard)  TASK_MODE="keyboard"; shift ;;
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
    for pid in ${BRIDGE_PID:-} ${PROXY_PID:-} ${SSE_PID:-} ${HTTP_PID:-}; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    # 等待进程退出
    for pid in ${BRIDGE_PID:-} ${PROXY_PID:-} ${SSE_PID:-} ${HTTP_PID:-}; do
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
    echo "  已全部关闭"
    # 恢复系统 zenoh 桥（如果之前被我们停掉了）
    systemctl start zenoh-ros2dds 2>/dev/null || true
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

# ── 自动检测 headless 环境 ─────────────────────────────────────────
if [ -z "${DISPLAY:-}" ]; then
    echo "⚠ 未检测到显示器 (DISPLAY 未设置)，自动切换为 headless 模式"
    GAZEBO_GUI="false"
    if [ "$TASK_MODE" = "manual" ]; then
        echo "  GUI 不可用，改用任务调度器（自主巡检）"
        TASK_MODE="task"
    fi
fi

echo "═══════════════════════════════════════════"
echo "  XCZS 巡操机器人仿真系统"
echo "═══════════════════════════════════════════"
echo ""
echo "  模式:       $(if [ "$TASK_MODE" = "task" ]; then echo '任务调度器'; elif [ "$TASK_MODE" = "keyboard" ]; then echo '键盘遥控'; else echo 'GUI 手动控制'; fi)"
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
echo "[1/5] Zenoh Bridge..."

# 先停掉可能存在的 systemd 服务（它以 peer 模式运行，缺少 ros2dds）
systemctl stop zenoh-ros2dds 2>/dev/null && echo "       已停止系统 zenoh 服务" || true

# 再杀掉任何残留的桥进程
pkill -9 -f "zenoh-bridge-ros2dds" 2>/dev/null || true
sleep 1

# 确保端口已释放
for i in $(seq 1 10); do
    if lsof -ti:"$BRIDGE_TCP_PORT" >/dev/null 2>&1; then
        echo "       等待端口 $BRIDGE_TCP_PORT 释放... ($i/10)"
        sleep 1
    else
        break
    fi
done

# 最后保险：如果端口还是被占，切到备用端口
if lsof -ti:"$BRIDGE_TCP_PORT" >/dev/null 2>&1; then
    echo "       端口 $BRIDGE_TCP_PORT 仍被占用，切换到备用端口"
    BRIDGE_TCP_PORT=$((BRIDGE_TCP_PORT + 1))
    BRIDGE_REST_PORT=$((BRIDGE_REST_PORT + 1))
fi

_start_bridge

# ── 2. 启动 CDR→JSON 代理（可选） ─────────────────────────────────
if [ "$WITH_PROXY" = "true" ]; then
    echo "[2/5] 启动 XCZS Zenoh 代理..."
    cd "$JIANG_DIR"
    python3 run_xczs_proxy.py &
    PROXY_PID=$!
    sleep 1
    echo "       代理已启动 (PID $PROXY_PID)"
else
    echo "[2/5] 跳过 Zenoh 代理（使用桥自带 JSON 转发）"
    PROXY_PID=""
fi

# ── 3a. 启动 SSE 桥（Zenoh TCP → HTTP SSE） ─────────────────────
SSE_PORT="${SSE_PORT:-8001}"
echo "[3a/5] 启动 SSE 数据桥 (port $SSE_PORT)..."
cd "$JIANG_DIR"
python3 sse_bridge.py --port "$SSE_PORT" --zenoh "tcp/localhost:$BRIDGE_TCP_PORT" &
SSE_PID=$!
sleep 1
echo "       SSE 数据桥: http://localhost:$SSE_PORT/<key>"

# ── 3b. 启动 HTTP 文件服务器（monitor.html） ────────────────────────
echo "[3b/5] 启动 HTTP 服务器 (port $MONITOR_PORT)..."
python3 -m http.server "$MONITOR_PORT" --bind 0.0.0.0 &
HTTP_PID=$!
sleep 1
echo "       监控面板: http://localhost:$MONITOR_PORT/monitor.html"

# ── 4. 启动 Gazebo + 机器人 ───────────────────────────────────────
echo "[5/5] 启动 Gazebo + XCZS 机器人..."
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
elif [ "$TASK_MODE" = "keyboard" ]; then
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=false" \
        "teleop:=true"
else
    ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "gui:=$GAZEBO_GUI" \
        "control_gui:=true"
fi
