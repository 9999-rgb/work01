#!/bin/bash
# ============================================================================
# XCZS 巡操机器人 — 仿真 + Zenoh 桥 + 监控面板 一键启动脚本
#
# 用法:
#   ./run_all.sh                  # 统一 Web 控制（含 Nav2 和控制柜任务）
#   ./run_all.sh --web            # 兼容写法，与无参数启动相同
#   ./run_all.sh --with-proxy     # 同时启动 CDR→JSON 代理
#   ./run_all.sh --keyboard       # 键盘调试控制（不启动 Web 控制服务）
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
CONTROL_ALLOWED_ORIGINS="${XCZS_CONTROL_ORIGINS:-http://localhost:$MONITOR_PORT,http://127.0.0.1:$MONITOR_PORT}"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORK_DIR/install/setup.bash"
PYTHON_BIN="/usr/bin/python3"
CABINET_INSTANCES_PATH="${CABINET_INSTANCES_PATH:-$WORK_DIR/xczs_inspection_robot_control/config/cabinet_instances.yaml}"
CABINET_CONTROLS_PATH="${CABINET_CONTROLS_PATH:-$WORK_DIR/xczs_inspection_robot_control/config/cabinet_controls.yaml}"
CABINET_SCENE_PATH="${CABINET_SCENE_PATH:-$WORK_DIR/xczs_inspection_robot_control/config/cabinet_scene.yaml}"
CABINET_POSE_PATH="${CABINET_POSE_PATH:-$WORK_DIR/xczs_inspection_robot_control/config/cabinet_pose.yaml}"
CABINET_ROBOT_ADAPTER_PATH="${CABINET_ROBOT_ADAPTER_PATH:-$WORK_DIR/xczs_inspection_robot_control/config/cabinet_robot_adapter.yaml}"
ROBOT_CONTROL_PATH="${ROBOT_CONTROL_PATH:-$WORK_DIR/xczs_inspection_robot_control/config/robot_control.yaml}"
RECORDINGS_ROOT="${RECORDINGS_ROOT:-$WORK_DIR/recordings}"
SIMULATION_WORLD_PATH="${SIMULATION_WORLD_PATH:-$WORK_DIR/xczs_inspection_robot_description/worlds/inspection_robot.world}"
CABINET_XACRO_PATH="${CABINET_XACRO_PATH:-$WORK_DIR/xczs_inspection_robot_description/urdf/control_cabinet.urdf.xacro}"
ROBOT_NAME="${ROBOT_NAME:-xczs_inspection_robot}"
ROBOT_XACRO_PATH="${ROBOT_XACRO_PATH:-$WORK_DIR/xczs_inspection_robot_description/urdf/xczs_inspection_robot.urdf.xacro}"
MOVEIT_CONFIG_PACKAGE="${MOVEIT_CONFIG_PACKAGE:-xczs_inspection_robot_moveit_config}"
MOVEIT_SRDF_PATH="${MOVEIT_SRDF_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/xczs_inspection_robot.srdf}"
MOVEIT_KINEMATICS_PATH="${MOVEIT_KINEMATICS_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/kinematics.yaml}"
MOVEIT_JOINT_LIMITS_PATH="${MOVEIT_JOINT_LIMITS_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/joint_limits.yaml}"
MOVEIT_CONTROLLERS_PATH="${MOVEIT_CONTROLLERS_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/moveit_controllers.yaml}"
MOVEIT_RVIZ_CONFIG_PATH="${MOVEIT_RVIZ_CONFIG_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/moveit.rviz}"
MOVEIT_LAUNCH_PATH="${MOVEIT_LAUNCH_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/launch/move_group.launch.py}"
NAV2_LAUNCH_PATH="${NAV2_LAUNCH_PATH:-$WORK_DIR/xczs_inspection_robot_nav2/launch/navigation.launch.py}"
NAV2_MAP_PATH="${NAV2_MAP_PATH:-$WORK_DIR/xczs_inspection_robot_nav2/maps/inspection_map.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$WORK_DIR/xczs_inspection_robot_nav2/config/nav2_params.yaml}"
ROBOT_BRINGUP="${ROBOT_BRINGUP:-true}"
GAZEBO_ENABLED="${GAZEBO_ENABLED:-true}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
MOVEIT_ENABLED="${MOVEIT_ENABLED:-true}"
CABINET_BRINGUP="${CABINET_BRINGUP:-true}"
SPAWN_CABINET="${SPAWN_CABINET:-true}"
SPAWN_Z="${SPAWN_Z:-0.515}"
CABINET_POSE_SOURCE="${CABINET_POSE_SOURCE:-static}"
PREFLIGHT_ONLY="${XCZS_PREFLIGHT_ONLY:-false}"

IFS=',' read -r -a CONTROL_ORIGINS <<< "$CONTROL_ALLOWED_ORIGINS"
CONTROL_ORIGIN_ARGS=()
for origin in "${CONTROL_ORIGINS[@]}"; do
    origin="${origin#"${origin%%[![:space:]]*}"}"
    origin="${origin%"${origin##*[![:space:]]}"}"
    if [ -n "$origin" ]; then
        CONTROL_ORIGIN_ARGS+=(--allowed-origin "$origin")
    fi
done

if [ "${#CONTROL_ORIGIN_ARGS[@]}" -eq 0 ]; then
    echo "ERROR: XCZS_CONTROL_ORIGINS 至少需要一个 Web Origin。"
    exit 1
fi

# ── 选项 ──────────────────────────────────────────────────────────
GAZEBO_GUI="true"
WITH_PROXY="false"
CONTROL_MODE="web"
NAV2_ENABLED="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-proxy) WITH_PROXY="true"; shift ;;
        --web)       CONTROL_MODE="web"; shift ;;
        --keyboard)  CONTROL_MODE="keyboard"; shift ;;
        -h|--help)
            sed -n '3,15p' "$0" | sed 's/^# *//'
            exit 0
            ;;
        *) echo "未知选项: $1"; exit 1 ;;
    esac
done

# ── 解析最终运行模式 ──────────────────────────────────────────────
if [ "$GAZEBO_ENABLED" = "false" ]; then
    GAZEBO_GUI="false"
fi
if [ -z "${DISPLAY:-}" ]; then
    echo "⚠ 未检测到显示器 (DISPLAY 未设置)，已禁用 Gazebo 图形界面"
    GAZEBO_GUI="false"
fi
if [ "$CONTROL_MODE" = "keyboard" ] && [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: 键盘调试控制需要 DISPLAY 和终端界面。"
    exit 1
fi
# Web 是统一操作界面，任务导航需要 Nav2；外接完整机器人栈模式由外部
# 提供相同 Action/Service 合同，本 launch 不重复启动 Nav2。
if [ "$CONTROL_MODE" = "web" ] && [ "$ROBOT_BRINGUP" = "true" ]; then
    NAV2_ENABLED="true"
fi

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
for boolean_value in \
    "$ROBOT_BRINGUP" \
    "$GAZEBO_ENABLED" \
    "$USE_SIM_TIME" \
    "$MOVEIT_ENABLED" \
    "$CABINET_BRINGUP" \
    "$SPAWN_CABINET" \
    "$PREFLIGHT_ONLY"; do
    if [ "$boolean_value" != "true" ] && [ "$boolean_value" != "false" ]; then
        echo "ERROR: ROBOT_BRINGUP、GAZEBO_ENABLED、USE_SIM_TIME、MOVEIT_ENABLED、CABINET_BRINGUP、SPAWN_CABINET 和 XCZS_PREFLIGHT_ONLY 必须为 true 或 false。"
        exit 1
    fi
done
if [ "$CABINET_POSE_SOURCE" != "static" ] && [ "$CABINET_POSE_SOURCE" != "topic" ]; then
    echo "ERROR: CABINET_POSE_SOURCE 必须为 static 或 topic。"
    exit 1
fi
if [ "$SPAWN_CABINET" = "true" ] && [ "$CABINET_BRINGUP" = "false" ]; then
    echo "ERROR: SPAWN_CABINET=true 需要 CABINET_BRINGUP=true。"
    exit 1
fi
if ! "$PYTHON_BIN" -c \
    "import math,sys; value=float(sys.argv[1]); assert math.isfinite(value)" \
    "$SPAWN_Z" 2>/dev/null; then
    echo "ERROR: SPAWN_Z 必须是有限数字。"
    exit 1
fi
if [ "$ROBOT_BRINGUP" = "false" ] && [ "$CONTROL_MODE" = "keyboard" ]; then
    echo "ERROR: ROBOT_BRINGUP=false 表示外部提供完整机器人栈，内置键盘控制不会启动。"
    echo "请使用默认 Web 入口，或由外部机器人栈提供自己的手动控制界面。"
    exit 1
fi

_require_file() {
    local path="$1"
    if [ ! -f "$path" ]; then
        echo "ERROR: 配置或模型文件不存在: $path"
        exit 1
    fi
}

_require_file "$CABINET_ROBOT_ADAPTER_PATH"
if [ "$CONTROL_MODE" = "web" ]; then
    _require_file "$CABINET_INSTANCES_PATH"
    _require_file "$CABINET_SCENE_PATH"
fi
if [ "$CABINET_BRINGUP" = "true" ]; then
    _require_file "$CABINET_INSTANCES_PATH"
    _require_file "$CABINET_CONTROLS_PATH"
    _require_file "$CABINET_SCENE_PATH"
    _require_file "$CABINET_POSE_PATH"
    if [ "$MOVEIT_ENABLED" = "true" ]; then
        _require_file "$ROBOT_XACRO_PATH"
        _require_file "$MOVEIT_SRDF_PATH"
        _require_file "$MOVEIT_KINEMATICS_PATH"
    fi
fi
if [ "$SPAWN_CABINET" = "true" ]; then
    _require_file "$CABINET_XACRO_PATH"
fi
if [ "$ROBOT_BRINGUP" = "true" ]; then
    _require_file "$ROBOT_CONTROL_PATH"
    _require_file "$ROBOT_XACRO_PATH"
    if [ "$MOVEIT_ENABLED" = "true" ]; then
        _require_file "$MOVEIT_SRDF_PATH"
        _require_file "$MOVEIT_KINEMATICS_PATH"
        _require_file "$MOVEIT_JOINT_LIMITS_PATH"
        _require_file "$MOVEIT_CONTROLLERS_PATH"
        _require_file "$MOVEIT_RVIZ_CONFIG_PATH"
        _require_file "$MOVEIT_LAUNCH_PATH"
    fi
    if [ "$NAV2_ENABLED" = "true" ]; then
        _require_file "$NAV2_LAUNCH_PATH"
        _require_file "$NAV2_MAP_PATH"
        _require_file "$NAV2_PARAMS_FILE"
    fi
fi
if [ "$GAZEBO_ENABLED" = "true" ]; then
    _require_file "$SIMULATION_WORLD_PATH"
fi

# ── 加载 ROS2 环境 ────────────────────────────────────────────────
source "$ROS2_SETUP"
source "$WORKSPACE_SETUP"
if [ "$MOVEIT_ENABLED" = "true" ] && [ "$CABINET_BRINGUP" = "true" ] && \
    ! ros2 pkg prefix "$MOVEIT_CONFIG_PACKAGE" >/dev/null 2>&1; then
    echo "ERROR: MoveIt 配置包不存在: $MOVEIT_CONFIG_PACKAGE"
    exit 1
fi
if [ "$NAV2_ENABLED" = "true" ] &&
    [ "$ROBOT_BRINGUP" = "true" ] &&
    ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    echo "ERROR: Nav2 binary packages are not installed."
    echo "Run: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup"
    exit 1
fi
if ! "$PYTHON_BIN" -c \
    "import aiohttp, rclpy, zenoh; from PIL import Image; from xczs_inspection_robot_control.action import OperateCabinetControl, PressCabinetButton; from xczs_inspection_robot_control.msg import CabinetControlCatalog, CabinetControlState" \
    2>/dev/null; then
    echo "ERROR: Python dependencies are incomplete."
    echo "Run colcon build, source install/setup.bash, then install:"
    echo "  $PYTHON_BIN -m pip install -r $JIANG_DIR/requirements.txt"
    exit 1
fi

echo "═══════════════════════════════════════════"
echo "  XCZS 巡操机器人仿真系统"
echo "═══════════════════════════════════════════"
echo ""
case "$CONTROL_MODE" in
    keyboard) MODE_LABEL="键盘遥控" ;;
    *) MODE_LABEL="浏览器统一控制" ;;
esac
echo "  模式:       $MODE_LABEL"
echo "  Gazebo GUI: $GAZEBO_GUI"
echo "  Zenoh 代理: $(if [ "$WITH_PROXY" = "true" ]; then echo '启用'; else echo '禁用（桥自带 JSON）'; fi)"
if [ "$CONTROL_MODE" = "web" ] && [ "$ROBOT_BRINGUP" = "false" ]; then
    NAV2_LABEL="由外部机器人栈提供"
elif [ "$NAV2_ENABLED" = "true" ]; then
    NAV2_LABEL="启用"
else
    NAV2_LABEL="禁用"
fi
echo "  Nav2 导航: $NAV2_LABEL"
echo ""

if [ "$PREFLIGHT_ONLY" = "true" ]; then
    echo "  启动配置预检通过（XCZS_PREFLIGHT_ONLY=true，未启动进程）。"
    CLEANUP_DONE="true"
    exit 0
fi

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
        --port "$CONTROL_PORT" \
        --cabinet-instances "$CABINET_INSTANCES_PATH" \
        --cabinet-controls "$CABINET_CONTROLS_PATH" \
        --cabinet-scene "$CABINET_SCENE_PATH" \
        --cabinet-pose "$CABINET_POSE_PATH" \
        --cabinet-robot-adapter "$CABINET_ROBOT_ADAPTER_PATH" \
        --robot-control "$ROBOT_CONTROL_PATH" \
        --recordings-root "$RECORDINGS_ROOT" \
        "${CONTROL_ORIGIN_ARGS[@]}" &
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

TELEOP_ENABLED="false"
if [ "$CONTROL_MODE" = "keyboard" ]; then
    TELEOP_ENABLED="true"
fi

LAUNCH_ARGS=(
    "gui:=$GAZEBO_GUI"
    "gazebo:=$GAZEBO_ENABLED"
    "robot_bringup:=$ROBOT_BRINGUP"
    "use_sim_time:=$USE_SIM_TIME"
    "control_gui:=false"
    "teleop:=$TELEOP_ENABLED"
    "moveit:=$MOVEIT_ENABLED"
    "nav2:=$NAV2_ENABLED"
    "nav2_rviz:=false"
    "robot_name:=$ROBOT_NAME"
    "robot_xacro:=$ROBOT_XACRO_PATH"
    "moveit_config_package:=$MOVEIT_CONFIG_PACKAGE"
    "moveit_srdf:=$MOVEIT_SRDF_PATH"
    "moveit_kinematics:=$MOVEIT_KINEMATICS_PATH"
    "moveit_joint_limits:=$MOVEIT_JOINT_LIMITS_PATH"
    "moveit_controllers:=$MOVEIT_CONTROLLERS_PATH"
    "moveit_rviz_config:=$MOVEIT_RVIZ_CONFIG_PATH"
    "moveit_launch:=$MOVEIT_LAUNCH_PATH"
    "nav2_launch:=$NAV2_LAUNCH_PATH"
    "nav2_map:=$NAV2_MAP_PATH"
    "nav2_params_file:=$NAV2_PARAMS_FILE"
    "world:=$SIMULATION_WORLD_PATH"
    "robot_control:=$ROBOT_CONTROL_PATH"
    "cabinet_instances:=$CABINET_INSTANCES_PATH"
    "cabinet_controls:=$CABINET_CONTROLS_PATH"
    "cabinet_scene:=$CABINET_SCENE_PATH"
    "cabinet_pose:=$CABINET_POSE_PATH"
    "cabinet_robot_adapter:=$CABINET_ROBOT_ADAPTER_PATH"
    "cabinet_xacro:=$CABINET_XACRO_PATH"
    "cabinet_bringup:=$CABINET_BRINGUP"
    "spawn_cabinet:=$SPAWN_CABINET"
    "spawn_z:=$SPAWN_Z"
    "cabinet_pose_source:=$CABINET_POSE_SOURCE"
)
ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
    "${LAUNCH_ARGS[@]}"
