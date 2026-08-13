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
# 启动后（FastAPI 三合一，默认监听所有网卡 :8090）:
#   - 监控面板: http://<服务器IP>:8090/monitor.html
#   - API 文档:  http://<服务器IP>:8090/docs
#   - SSE 数据:  http://<服务器IP>:8090/sse/<key>
#   - 传感器流:  http://<服务器IP>:8090/camera.mjpg
#   - Zenoh TCP: tcp/localhost:7447
# ============================================================================
set -Eeo pipefail

# ── 初始化 PID 变量（避免 cleanup 时未绑定） ─────────────────────
BRIDGE_PID=""
PROXY_PID=""
CONTROL_PID=""
LAUNCH_PID=""
CLEANUP_DONE="false"
STARTUP_TIMEOUT_SEC="${XCZS_STARTUP_TIMEOUT_SEC:-30}"
ROBOT_READY_TIMEOUT_SEC="${XCZS_ROBOT_READY_TIMEOUT_SEC:-120}"
SHUTDOWN_TIMEOUT_SEC="${XCZS_SHUTDOWN_TIMEOUT_SEC:-60}"
MANAGED_PIDS=()
MANAGED_LABELS=()

# ── 路径配置 ──────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORK_DIR="$(dirname "$SCRIPT_DIR")"          # work01 根目录
JIANG_DIR="$WORK_DIR/jiang"
ZENOH_BRIDGE="${ZENOH_BRIDGE:-/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds}"
BRIDGE_REST_PORT="${BRIDGE_REST_PORT:-8000}"
BRIDGE_TCP_PORT="${BRIDGE_TCP_PORT:-7447}"
CONTROL_PORT="${CONTROL_PORT:-8090}"
CONTROL_HOST="${CONTROL_HOST:-0.0.0.0}"
CONTROL_ALLOWED_ORIGINS="${XCZS_CONTROL_ORIGINS:-http://localhost:$CONTROL_PORT,http://127.0.0.1:$CONTROL_PORT}"
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

if [ "$CONTROL_MODE" = "web" ] && [ "${#CONTROL_ORIGIN_ARGS[@]}" -eq 0 ]; then
    echo "ERROR: XCZS_CONTROL_ORIGINS 至少需要一个 Web Origin。"
    exit 1
fi

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
LOCAL_LAUNCH_PERSISTENT="false"
if [ "$GAZEBO_ENABLED" = "true" ] || [ "$ROBOT_BRINGUP" = "true" ] ||
    [ "$CABINET_BRINGUP" = "true" ]; then
    LOCAL_LAUNCH_PERSISTENT="true"
fi

# ── 清理函数 ──────────────────────────────────────────────────────
_register_process() {
    local pid="$1"
    local label="$2"
    local attempt=0
    local process_group=""
    if ! [[ "$pid" =~ ^[1-9][0-9]*$ ]]; then
        echo "ERROR: $label 返回了无效 PID: $pid"
        exit 1
    fi
    MANAGED_PIDS+=("$pid")
    MANAGED_LABELS+=("$label")
    # setsid 应使该 PID 同时成为新进程组的 ID。确认后才继续，
    # 避免清理时只停主进程却遗留 launch/uvicorn 子进程。
    for ((attempt=0; attempt < 20; attempt++)); do
        process_group="$(
            ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]'
        )"
        if [ "$process_group" = "$pid" ]; then
            return 0
        fi
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.01
    done
    echo "ERROR: $label 未进入独立进程组（PID=$pid, PGID=${process_group:-unknown}）。" >&2
    kill -TERM "$pid" 2>/dev/null || true
    exit 1
}

_unregister_last_process() {
    local pid="$1"
    local last_index=$((${#MANAGED_PIDS[@]} - 1))
    if (( last_index < 0 )) || [ "${MANAGED_PIDS[$last_index]}" != "$pid" ]; then
        echo "ERROR: 只能逆序注销最后一个受管进程: $pid" >&2
        exit 1
    fi
    unset "MANAGED_PIDS[$last_index]"
    unset "MANAGED_LABELS[$last_index]"
}

_process_is_running() {
    local pid="$1"
    local state=""
    kill -0 "$pid" 2>/dev/null || return 1
    state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
    [ -n "$state" ] && [[ "$state" != Z* ]]
}

_signal_process_group() {
    local pid="$1"
    local signal="$2"
    # 后台命令由 setsid 启动，PID 同时是进程组 ID。只终止本次
    # 注册的进程组，不扫描或终止全机同名进程。
    kill "-$signal" -- "-$pid" 2>/dev/null ||
        kill "-$signal" "$pid" 2>/dev/null || true
}

cleanup() {
    local exit_code=$?
    local deadline=0
    local index=0
    local pid=""
    local still_running="false"
    if [ "$CLEANUP_DONE" = "true" ]; then
        return "$exit_code"
    fi
    CLEANUP_DONE="true"
    if [ "${#MANAGED_PIDS[@]}" -gt 0 ]; then
        echo ""
        echo "═══════════════════════════════════════════"
        echo "  正在关闭本次启动的进程..."
        echo "═══════════════════════════════════════════"
    fi

    # 按逆启动顺序停止：先 launch，再 Web/代理，最后 bridge。
    for ((index=${#MANAGED_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${MANAGED_PIDS[$index]}"
        if _process_is_running "$pid"; then
            _signal_process_group "$pid" TERM
        fi
    done

    deadline=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        still_running="false"
        for pid in "${MANAGED_PIDS[@]}"; do
            if _process_is_running "$pid"; then
                still_running="true"
                break
            fi
        done
        [ "$still_running" = "false" ] && break
        sleep 0.2
    done

    for ((index=${#MANAGED_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${MANAGED_PIDS[$index]}"
        if _process_is_running "$pid"; then
            echo "WARNING: ${MANAGED_LABELS[$index]} 未在 ${SHUTDOWN_TIMEOUT_SEC}s 内退出，强制终止。" >&2
            _signal_process_group "$pid" KILL
        fi
    done
    for pid in "${MANAGED_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    if [ "${#MANAGED_PIDS[@]}" -gt 0 ]; then
        echo "  本次启动的进程已全部关闭"
    fi
    return "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
if [ ! -x "$ZENOH_BRIDGE" ]; then
    echo "ERROR: zenoh-bridge-ros2dds 不存在或不可执行: $ZENOH_BRIDGE"
    exit 1
fi
for required_command in setsid ps tr; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        echo "ERROR: 启动脚本缺少系统命令: $required_command"
        exit 1
    fi
done
if [ "$CONTROL_MODE" = "web" ] && ! command -v curl >/dev/null 2>&1; then
    echo "ERROR: Web 服务 readiness 检查需要 curl。"
    exit 1
fi

_require_integer_range() {
    local value="$1"
    local minimum="$2"
    local maximum="$3"
    local name="$4"
    if ! [[ "$value" =~ ^[0-9]+$ ]] ||
        (( value < minimum || value > maximum )); then
        echo "ERROR: $name 必须是 ${minimum}..${maximum} 的整数。"
        exit 1
    fi
}

_describe_port_owner() {
    local port="$1"
    local details=""
    if command -v ss >/dev/null 2>&1; then
        details="$(ss -H -ltnp "sport = :$port" 2>/dev/null || true)"
        if [ -n "$details" ]; then
            echo "       当前监听: $details" >&2
        fi
    fi
}

_require_port_available() {
    local host="$1"
    local port="$2"
    local label="$3"
    if ! "$PYTHON_BIN" - "$host" "$port" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
errors = []
seen = set()
try:
    addresses = socket.getaddrinfo(
        host, port, type=socket.SOCK_STREAM, flags=socket.AI_PASSIVE
    )
except OSError as error:
    print(error, file=sys.stderr)
    raise SystemExit(1)
for family, socktype, protocol, _canonname, address in addresses:
    key = (family, address)
    if key in seen:
        continue
    seen.add(key)
    sock = socket.socket(family, socktype, protocol)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(address)
    except OSError as error:
        errors.append(f"{address}: {error}")
    finally:
        sock.close()
if not errors and seen:
    raise SystemExit(0)
print("; ".join(dict.fromkeys(errors)), file=sys.stderr)
raise SystemExit(1)
PY
    then
        echo "ERROR: $label 无法绑定 $host:$port（端口被占用或地址不可用）。" >&2
        _describe_port_owner "$port"
        exit 1
    fi
}

_require_integer_range "$BRIDGE_TCP_PORT" 1 65535 BRIDGE_TCP_PORT
_require_integer_range "$BRIDGE_REST_PORT" 1 65535 BRIDGE_REST_PORT
_require_integer_range "$STARTUP_TIMEOUT_SEC" 1 60 XCZS_STARTUP_TIMEOUT_SEC
_require_integer_range "$ROBOT_READY_TIMEOUT_SEC" 1 300 XCZS_ROBOT_READY_TIMEOUT_SEC
_require_integer_range "$SHUTDOWN_TIMEOUT_SEC" 1 60 XCZS_SHUTDOWN_TIMEOUT_SEC
if [ "$BRIDGE_TCP_PORT" = "$BRIDGE_REST_PORT" ]; then
    echo "ERROR: BRIDGE_TCP_PORT 与 BRIDGE_REST_PORT 不能相同。"
    exit 1
fi
if [ "$CONTROL_MODE" = "web" ]; then
    _require_integer_range "$CONTROL_PORT" 1 65535 CONTROL_PORT
    if [ "$CONTROL_PORT" = "$BRIDGE_TCP_PORT" ] ||
        [ "$CONTROL_PORT" = "$BRIDGE_REST_PORT" ]; then
        echo "ERROR: CONTROL_PORT 不能与 Zenoh 端口相同。"
        exit 1
    fi
    if [ -z "$CONTROL_HOST" ]; then
        echo "ERROR: CONTROL_HOST 不能为空。"
        exit 1
    fi
    if [[ "$CONTROL_HOST" == \[*\] ]]; then
        echo "ERROR: CONTROL_HOST 中的 IPv6 地址不要带方括号（例如使用 ::1）。"
        exit 1
    fi
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
    _require_file "$CABINET_CONTROLS_PATH"
    _require_file "$CABINET_SCENE_PATH"
    _require_file "$CABINET_POSE_PATH"
    _require_file "$ROBOT_CONTROL_PATH"
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

# ── ROS 域隔离 ─────────────────────────────────────────────────────
# 局域网内可能有其他 ROS 2 / Gazebo 仿真，使用独立的 ROS_DOMAIN_ID 避免
# DDS 流量互相干扰。可通过环境变量覆盖。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
_require_integer_range "$ROS_DOMAIN_ID" 0 232 ROS_DOMAIN_ID

# ── 加载 ROS2 环境 ────────────────────────────────────────────────
source "$ROS2_SETUP"
source "$WORKSPACE_SETUP"
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: source 环境后仍找不到 ros2 命令。"
    exit 1
fi

REQUIRED_ROS_PACKAGES=(
    xczs_inspection_robot_control
)
if [ "$ROBOT_BRINGUP" = "true" ] || [ "$SPAWN_CABINET" = "true" ]; then
    REQUIRED_ROS_PACKAGES+=(xczs_inspection_robot_description xacro)
fi
if [ "$ROBOT_BRINGUP" = "true" ]; then
    REQUIRED_ROS_PACKAGES+=(
        robot_state_publisher
        controller_manager
        joint_state_broadcaster
        joint_trajectory_controller
    )
fi
if [ "$GAZEBO_ENABLED" = "true" ] || [ "$ROBOT_BRINGUP" = "true" ] ||
    [ "$SPAWN_CABINET" = "true" ]; then
    REQUIRED_ROS_PACKAGES+=(gazebo_ros)
fi
if [ "$ROBOT_BRINGUP" = "true" ]; then
    REQUIRED_ROS_PACKAGES+=(gazebo_plugins gazebo_ros2_control)
fi
if [ "$MOVEIT_ENABLED" = "true" ] &&
    { [ "$ROBOT_BRINGUP" = "true" ] ||
      [ "$CABINET_BRINGUP" = "true" ]; }; then
    REQUIRED_ROS_PACKAGES+=(
        "$MOVEIT_CONFIG_PACKAGE"
        moveit_configs_utils
        moveit_kinematics
        moveit_planners_ompl
        moveit_ros_move_group
        moveit_simple_controller_manager
    )
fi
if [ "$NAV2_ENABLED" = "true" ]; then
    REQUIRED_ROS_PACKAGES+=(
        xczs_inspection_robot_nav2
        nav2_bringup
        navigation2
    )
fi
if [ "$CONTROL_MODE" = "web" ]; then
    REQUIRED_ROS_PACKAGES+=(ros2bag)
fi
for required_package in "${REQUIRED_ROS_PACKAGES[@]}"; do
    if ! ros2 pkg prefix "$required_package" >/dev/null 2>&1; then
        echo "ERROR: 启动组合所需 ROS 2 包不存在: $required_package"
        exit 1
    fi
done
if [ "$CONTROL_MODE" = "web" ]; then
    echo "  检查 Web/ROS Python 运行时依赖..."
    if ! env PYTHONPATH="$JIANG_DIR${PYTHONPATH:+:$PYTHONPATH}" \
        "$PYTHON_BIN" - "${CONTROL_ORIGINS[@]}" <<'PY'
import sys

try:
    import aiohttp, alembic, aiosqlite, bcrypt, cryptography, fastapi
    import jose, multipart, passlib, pydantic, pydantic_settings
    import sqlalchemy, sse_starlette, uvicorn, yaml, zenoh
    from PIL import Image
    from app.main import _normalize_allowed_origins, create_app
    from control_gateway import ControlServer
    from sensor_bridge.ros_node import SensorRosRuntime
    from sse_bridge import ZenohSource
    from xczs_inspection_robot_control.action import (
        OperateCabinetControl,
        PressCabinetButton,
    )
    from xczs_inspection_robot_control.msg import (
        CabinetControl,
        CabinetControlCatalog,
        CabinetControlState,
    )
except Exception as error:
    print(
        "Web/ROS Python 运行时依赖导入失败: "
        f"{type(error).__name__}: {error}",
        file=sys.stderr,
    )
    raise SystemExit(1)

try:
    _normalize_allowed_origins(sys.argv[1:])
except ValueError as error:
    print(f"XCZS_CONTROL_ORIGINS 无效: {error}", file=sys.stderr)
    raise SystemExit(1)
PY
    then
        echo "ERROR: Web 启动预检失败。若上方报告依赖缺失，可运行：" >&2
        echo "  $PYTHON_BIN -m pip install -r $JIANG_DIR/requirements.txt" >&2
        exit 1
    fi
fi
if [ "$WITH_PROXY" = "true" ]; then
    if ! env PYTHONPATH="$JIANG_DIR${PYTHONPATH:+:$PYTHONPATH}" \
        "$PYTHON_BIN" -c 'import aiohttp, run_xczs_proxy, zenoh'; then
        echo "ERROR: CDR→JSON 代理的 Python 依赖导入失败。"
        exit 1
    fi
fi

if [ "$CONTROL_MODE" = "web" ] || [ "$CABINET_BRINGUP" = "true" ]; then
    echo "  检查跨文件机器人/导航/柜体配置合同..."
    if ! "$PYTHON_BIN" "$WORK_DIR/scripts/check_adapter_contract" \
        --robot-adapter "$CABINET_ROBOT_ADAPTER_PATH" \
        --instances "$CABINET_INSTANCES_PATH" \
        --controls "$CABINET_CONTROLS_PATH" \
        --scene "$CABINET_SCENE_PATH" \
        --pose "$CABINET_POSE_PATH"; then
        echo "ERROR: 启动配置合同校验失败。"
        exit 1
    fi
fi

# 预检与正式启动采用同一端口检查。端口被外部服务占用时只报错，
# 不复用、不终止不属于本次启动的进程。
_require_port_available 0.0.0.0 "$BRIDGE_TCP_PORT" "Zenoh TCP"
_require_port_available 0.0.0.0 "$BRIDGE_REST_PORT" "Zenoh REST"
if [ "$CONTROL_MODE" = "web" ]; then
    _require_port_available "$CONTROL_HOST" "$CONTROL_PORT" "Web 控制服务"
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
_assert_managed_processes_running() {
    local index=0
    local status=0
    for ((index=0; index < ${#MANAGED_PIDS[@]}; index++)); do
        if ! _process_is_running "${MANAGED_PIDS[$index]}"; then
            if wait "${MANAGED_PIDS[$index]}" 2>/dev/null; then
                status=0
            else
                status=$?
            fi
            echo "ERROR: ${MANAGED_LABELS[$index]} 意外退出（status=$status）。" >&2
            exit 1
        fi
    done
}

_wait_for_tcp() {
    local host="$1"
    local port="$2"
    local label="$3"
    local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        _assert_managed_processes_running
        if "$PYTHON_BIN" - "$host" "$port" <<'PY' 2>/dev/null
import socket
import sys

try:
    with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.5):
        pass
except OSError:
    raise SystemExit(1)
PY
        then
            return 0
        fi
        sleep 0.2
    done
    echo "ERROR: $label 未在 ${STARTUP_TIMEOUT_SEC}s 内监听 $host:$port。" >&2
    exit 1
}

_wait_for_http() {
    local url="$1"
    local label="$2"
    local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        _assert_managed_processes_running
        if curl -fsS --connect-timeout 1 --max-time 2 "$url" \
            >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.2
    done
    echo "ERROR: $label 未在 ${STARTUP_TIMEOUT_SEC}s 内通过 readiness: $url" >&2
    exit 1
}

_wait_for_task_stack() {
    local url="$1"
    local require_cabinet="$2"
    local deadline=$((SECONDS + ROBOT_READY_TIMEOUT_SEC))
    local health_body=""
    local health_summary=""
    while (( SECONDS < deadline )); do
        _assert_managed_processes_running
        health_body="$(
            curl -fsS --connect-timeout 1 --max-time 2 "$url" 2>/dev/null ||
                true
        )"
        if printf '%s' "$health_body" | "$PYTHON_BIN" -c '
import json
import sys

require_cabinet = sys.argv[1] == "true"
try:
    health = json.load(sys.stdin)
except (json.JSONDecodeError, TypeError):
    raise SystemExit(1)
if not (
    health.get("status") == "ok"
    and health.get("navigation_available") is True
    and health.get("map_available") is True
    and isinstance(health.get("cabinet_count"), int)
    and health["cabinet_count"] >= 1
    and (not require_cabinet or health.get("cabinet_available") is True)
):
    raise SystemExit(1)
' "$require_cabinet"; then
            return 0
        fi
        sleep 0.5
    done
    echo "ERROR: ROS 任务栈未在 ${ROBOT_READY_TIMEOUT_SEC}s 内就绪。" >&2
    if [ -n "$health_body" ]; then
        health_summary="$(
            printf '%s' "$health_body" | "$PYTHON_BIN" -c '
import json
import sys

health = json.load(sys.stdin)
summary = {
    key: health.get(key)
    for key in (
        "status",
        "navigation_available",
        "map_available",
        "map_error",
        "cabinet_available",
        "cabinet_count",
    )
}
print(json.dumps(summary, ensure_ascii=False, separators=(",", ":")))
' 2>/dev/null || true
        )"
        if [ -n "$health_summary" ]; then
            echo "       最后的 /health: $health_summary" >&2
        else
            echo "       最后的 /health 不是有效 JSON。" >&2
        fi
    fi
    if [ "$require_cabinet" = "true" ]; then
        echo "       需要 Nav2 Action、有效 occupancy map、所有柜体目录和操作 Action 同时可用。" >&2
    else
        echo "       需要 Nav2 Action 和有效 occupancy map 可用。" >&2
    fi
    exit 1
}

_wait_for_stable_processes() {
    local checks="${1:-5}"
    local count=0
    for ((count=0; count < checks; count++)); do
        _assert_managed_processes_running
        sleep 0.2
    done
}

_monitor_managed_processes() {
    local index=0
    local pid=""
    local status=0
    while true; do
        for ((index=0; index < ${#MANAGED_PIDS[@]}; index++)); do
            pid="${MANAGED_PIDS[$index]}"
            if ! _process_is_running "$pid"; then
                if wait "$pid" 2>/dev/null; then
                    status=0
                else
                    status=$?
                fi
                if [ "$pid" = "$LAUNCH_PID" ] && [ "$status" -eq 0 ]; then
                    echo "ROS 2 launch 已正常结束。"
                    return 0
                fi
                echo "ERROR: ${MANAGED_LABELS[$index]} 意外退出（status=$status）。" >&2
                if [ "$status" -eq 0 ]; then
                    return 1
                fi
                return "$status"
            fi
        done
        sleep 0.5
    done
}

_start_bridge() {
    setsid "$ZENOH_BRIDGE" \
        --listen "tcp/0.0.0.0:$BRIDGE_TCP_PORT" \
        --rest-http-port "$BRIDGE_REST_PORT" &
    BRIDGE_PID=$!
    _register_process "$BRIDGE_PID" "Zenoh Bridge"
    _wait_for_tcp 127.0.0.1 "$BRIDGE_TCP_PORT" "Zenoh TCP"
    _wait_for_tcp 127.0.0.1 "$BRIDGE_REST_PORT" "Zenoh REST"
    echo "       Zenoh Bridge 已就绪 (PID $BRIDGE_PID)"
    echo "       TCP: tcp/localhost:$BRIDGE_TCP_PORT"
    echo "       REST: http://localhost:$BRIDGE_REST_PORT"
}

# ── 1. 启动 Zenoh Bridge ──────────────────────────────────────────
echo "[1/6] Zenoh Bridge..."

# 端口冲突已在预检阶段 fail-fast，不复用或终止全机上的其他 Zenoh 桥。
_start_bridge

# ── 2. 启动 CDR→JSON 代理（可选） ─────────────────────────────────
if [ "$WITH_PROXY" = "true" ]; then
    echo "[2/6] 启动 XCZS Zenoh 代理..."
    cd "$JIANG_DIR"
    setsid "$PYTHON_BIN" run_xczs_proxy.py \
        --port "$BRIDGE_TCP_PORT" \
        --control-port 0 &
    PROXY_PID=$!
    _register_process "$PROXY_PID" "XCZS Zenoh 代理"
    _wait_for_stable_processes 5
    echo "       代理已启动 (PID $PROXY_PID)"
else
    echo "[2/6] 跳过 Zenoh 代理（使用桥自带 JSON 转发）"
    PROXY_PID=""
fi

# ── 3. 启动统一 Web 服务（三合一：控制 API + SSE + 传感器 + 静态页） ──
# 迁移到 FastAPI 后，原先独立的 SSE 桥、传感器流、HTTP 文件服务器与
# Web 控制服务合并为单个 uvicorn 进程（默认 CONTROL_PORT 8090）。
if [ "$CONTROL_MODE" = "web" ]; then
    echo "[3/6] 启动统一 Web 控制服务 (port $CONTROL_PORT)..."
    cd "$JIANG_DIR"
    setsid "$PYTHON_BIN" control_server.py \
        --host "$CONTROL_HOST" \
        --port "$CONTROL_PORT" \
        --cabinet-instances "$CABINET_INSTANCES_PATH" \
        --cabinet-controls "$CABINET_CONTROLS_PATH" \
        --cabinet-scene "$CABINET_SCENE_PATH" \
        --cabinet-pose "$CABINET_POSE_PATH" \
        --cabinet-robot-adapter "$CABINET_ROBOT_ADAPTER_PATH" \
        --robot-control "$ROBOT_CONTROL_PATH" \
        --recordings-root "$RECORDINGS_ROOT" \
        --zenoh "tcp/localhost:$BRIDGE_TCP_PORT" \
        "${CONTROL_ORIGIN_ARGS[@]}" &
    CONTROL_PID=$!
    _register_process "$CONTROL_PID" "Web 控制服务"
    case "$CONTROL_HOST" in
        0.0.0.0|::|'[::]') _READY_HOST="127.0.0.1" ;;
        *) _READY_HOST="$CONTROL_HOST" ;;
    esac
    case "$_READY_HOST" in
        *:*) _READY_URL_HOST="[$_READY_HOST]" ;;
        *) _READY_URL_HOST="$_READY_HOST" ;;
    esac
    _wait_for_http "http://${_READY_URL_HOST}:$CONTROL_PORT/health" "Web 控制服务"
    # 只有监听 wildcard 时才展示局域网 IP；绑定具体地址时原样展示。
    case "$CONTROL_HOST" in
        0.0.0.0|::|'[::]')
            _LAN_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
            _SHOW_HOST="${_LAN_IP:-127.0.0.1}"
            ;;
        *) _SHOW_HOST="$CONTROL_HOST" ;;
    esac
    case "$_SHOW_HOST" in
        *:*) _SHOW_HOST="[$_SHOW_HOST]" ;;
    esac
    echo "       Web 控制: http://${_SHOW_HOST}:$CONTROL_PORT"
    echo "       API 文档:  http://${_SHOW_HOST}:$CONTROL_PORT/docs"
    echo "       监控面板: http://${_SHOW_HOST}:$CONTROL_PORT/monitor.html"
    echo "       SSE 数据:  http://${_SHOW_HOST}:$CONTROL_PORT/sse/<key>"
    echo "       相机 MJPEG: http://${_SHOW_HOST}:$CONTROL_PORT/camera.mjpg"
    echo "       雷达 WebSocket: ws://${_SHOW_HOST}:$CONTROL_PORT/lidar/ws"
else
    echo "[3/6] 跳过 Web 控制服务（键盘模式由专用 ROS 2 节点控制）"
fi

# ── 5. 显示访问地址 ───────────────────────────────────────────────
echo ""
if [ "$CONTROL_MODE" = "web" ]; then
    echo "  ╔══════════════════════════════════════════════════════╗"
    echo "  ║  🌐 监控面板: http://${_SHOW_HOST}:$CONTROL_PORT/monitor.html"
    echo "  ║  📚 API 文档:  http://${_SHOW_HOST}:$CONTROL_PORT/docs"
    echo "  ║  📡 SSE 数据:  http://${_SHOW_HOST}:$CONTROL_PORT/sse/<key>"
    echo "  ║  📷 传感器流:  http://${_SHOW_HOST}:$CONTROL_PORT/camera.mjpg"
    echo "  ╚══════════════════════════════════════════════════════╝"
fi
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
setsid ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
    "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
_register_process "$LAUNCH_PID" "ROS 2 launch"
if [ "$LOCAL_LAUNCH_PERSISTENT" = "true" ]; then
    _wait_for_stable_processes 10
else
    echo "       本入口未启用本地 ROS 节点，校验 launch 配置后连接外部完整栈..."
    if wait "$LAUNCH_PID"; then
        _unregister_last_process "$LAUNCH_PID"
        LAUNCH_PID=""
    else
        launch_status=$?
        echo "ERROR: ROS 2 launch 配置校验失败（status=$launch_status）。" >&2
        exit "$launch_status"
    fi
fi
if [ "$CONTROL_MODE" = "web" ]; then
    _wait_for_task_stack \
        "http://${_READY_URL_HOST}:$CONTROL_PORT/health" "$MOVEIT_ENABLED"
    if [ "$MOVEIT_ENABLED" = "true" ]; then
        echo "       ROS 任务栈已就绪（Nav2 和柜体 Action 可用）"
    else
        echo "       ROS 任务栈已就绪（Nav2 可用）"
    fi
fi
if [ -n "$LAUNCH_PID" ]; then
    echo "       ROS 2 launch 已启动 (PID $LAUNCH_PID)"
else
    echo "       本地 ROS launch 配置已通过，正在使用外部完整栈"
fi
echo "       持续监控 Zenoh、Web 和 ROS 2 launch；任一进程异常退出将关闭本次启动。"

if _monitor_managed_processes; then
    exit 0
else
    runtime_status=$?
    exit "$runtime_status"
fi
