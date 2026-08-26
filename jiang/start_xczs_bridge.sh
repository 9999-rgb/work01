#!/bin/bash
# ============================================================================
# XCZS 巡操机器人 — 仿真 + Zenoh 桥 + 监控面板 一键启动脚本
#
# 用法:
#   ./run_all.sh                  # 统一 Web 控制（含 Nav2 和控制柜任务）
#   ./run_all.sh --web            # 兼容写法，与无参数启动相同
#   ./run_all.sh --with-proxy     # 同时启动 CDR→JSON 代理
#   ./run_all.sh --keyboard       # 键盘调试控制（不启动 Web 控制服务）
#   同机隔离启动（端口应避开其他实例）:
#     ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 BRIDGE_TCP_PORT=17447 \
#       BRIDGE_REST_PORT=18000 CONTROL_HOST=127.0.0.1 CONTROL_PORT=18090 \
#       XCZS_CONTROL_ORIGINS=http://localhost:18090,http://127.0.0.1:18090 \
#       ./run_all.sh --web
#   端口被本项目旧实例进程占用时，启动会自动终止占用进程后继续；
#   非本项目进程占用会明确报错，不会误杀。
#
# 启动后（FastAPI 三合一，默认监听所有网卡 :8090）:
#   - 监控面板: http://<服务器IP>:8090/monitor.html
#   - API 文档:  http://<服务器IP>:8090/docs
#   - SSE 数据:  http://<服务器IP>:8090/sse/<key>
#   - 传感器流:  http://<服务器IP>:8090/camera.mjpg
#   - Zenoh TCP: tcp/127.0.0.1:7447（默认本机隔离）
# ============================================================================
set -Eeo pipefail

# ── 初始化 PID 变量（避免 cleanup 时未绑定） ─────────────────────
BRIDGE_PID=""
PROXY_PID=""
CONTROL_PID=""
LAUNCH_PID=""
WORLD_LAUNCH_PID=""
TOOLSET_SUPERVISOR_PID=""
CLEANUP_DONE="false"
STARTUP_TIMEOUT_SEC="${XCZS_STARTUP_TIMEOUT_SEC:-30}"
ROBOT_READY_TIMEOUT_SEC="${XCZS_ROBOT_READY_TIMEOUT_SEC:-120}"
SHUTDOWN_TIMEOUT_SEC="${XCZS_SHUTDOWN_TIMEOUT_SEC:-60}"
MANAGED_GUARD_TIMEOUT_SEC="${XCZS_MANAGED_GUARD_TIMEOUT_SEC:-5}"
MANAGED_PIDS=()
MANAGED_PGIDS=()
MANAGED_LABELS=()
LAUNCH_RUNTIME_DIRECTORY=""
GAZEBO_LOG_DIRECTORY=""
# Capture the caller's intent before any ROS/Gazebo environment hook runs.
# An explicitly supplied URI remains supported; otherwise this project uses
# only its installed/local models and never waits on the retired remote model
# database during cold start.
GAZEBO_MODEL_DATABASE_URI_EXPLICIT="${GAZEBO_MODEL_DATABASE_URI+x}"
GAZEBO_MODEL_DATABASE_URI_REQUESTED="${GAZEBO_MODEL_DATABASE_URI-}"

# ── 路径配置 ──────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORK_DIR="$(dirname "$SCRIPT_DIR")"          # work01 根目录
JIANG_DIR="$WORK_DIR/jiang"
ZENOH_BRIDGE="${ZENOH_BRIDGE:-/opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds}"
ZENOH_BRIDGE_CONFIG="${ZENOH_BRIDGE_CONFIG:-$WORK_DIR/xczs_inspection_robot_control/config/zenoh_bridge.json5}"
BRIDGE_REST_PORT="${BRIDGE_REST_PORT:-8000}"
BRIDGE_TCP_PORT="${BRIDGE_TCP_PORT:-7447}"
ZENOH_LAN_ENABLED="${XCZS_ZENOH_LAN_ENABLED:-false}"
CONTROL_PORT="${CONTROL_PORT:-8090}"
CONTROL_HOST="${CONTROL_HOST:-0.0.0.0}"
CONTROL_ALLOWED_ORIGINS="${XCZS_CONTROL_ORIGINS:-}"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORK_DIR/install/setup.bash"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

# 帮助与参数错误不应依赖资产数据库可用；在任何迁移或文件访问之前先完成纯
# 参数预检。后面的正式解析仍负责设置运行模式。
for _startup_argument in "$@"; do
    case "$_startup_argument" in
        --with-proxy|--web|--keyboard) ;;
        -h|--help)
            sed -n '3,15p' "$0" | sed 's/^# *//'
            exit 0
            ;;
        *) echo "未知选项: $_startup_argument" >&2; exit 1 ;;
    esac
done

# ── 资产库选择 → 现有环境指针 ──────────────────────────────────────
# 选择持久化在 SQLite（``selection`` 表，与 auth 用户同库）。这里通过 CLI 的
# ``--print-env`` 把选中的场景/柜体/工具套装映射到现有的 CABINET_*_PATH /
# SCENES_CONFIG / SCENE / TOOLSET 环境变量，由下方默认值块与 launch/Web 消费。仅当用户
# 没有显式设置对应变量时生效——显式环境变量优先于资产库选择。无选择（空表 /
# 首次启动）正常返回空结果；数据库迁移或 schema 错误必须阻止带病启动。
XCZS_ASSETS_DIR="${XCZS_ASSETS_DIR:-$WORK_DIR/jiang/data/assets}"
if ! ASSET_SELECTION_LINES="$(
    "$PYTHON_BIN" "$WORK_DIR/scripts/xczs_import_asset" \
        --assets-dir "$XCZS_ASSETS_DIR" --print-env
)"; then
    echo "ERROR: 资产选择或数据库迁移失败，已中止启动。" >&2
    exit 1
fi
while IFS='=' read -r _asset_key _asset_value; do
    [ -z "$_asset_key" ] && continue
    if [ -z "${!_asset_key:-}" ]; then
        export "$_asset_key=$_asset_value"
    fi
done <<< "$ASSET_SELECTION_LINES"

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
# 末端工具套装 A/B（Web 选择持久化在资产库，见 selection 表的 toolset 列）。
# 归一为大写；非 A/B 一律回退 A。所有按套装的模型/控制器/MoveIt 路径由此派生。
# 在本地 Web+Gazebo 模式，toolset_supervisor 会保持 Gazebo 世界并替换机器人
# 实体/控制栈；本变量仅表示首次加载的套装。其他启动组合沿用单体 launch。
TOOLSET="$(printf '%s' "${TOOLSET:-A}" | tr '[:lower:]' '[:upper:]')"
case "$TOOLSET" in
    A|B) ;;
    *) TOOLSET="A" ;;
esac
export XCZS_ACTIVE_TOOLSET="$TOOLSET"
MOVEIT_CONFIG_PACKAGE="${MOVEIT_CONFIG_PACKAGE:-xczs_inspection_robot_moveit_config}"
MOVEIT_SRDF_PATH="${MOVEIT_SRDF_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/xczs_inspection_robot_toolset_${TOOLSET}.srdf}"
MOVEIT_KINEMATICS_PATH="${MOVEIT_KINEMATICS_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/kinematics.yaml}"
MOVEIT_JOINT_LIMITS_PATH="${MOVEIT_JOINT_LIMITS_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/joint_limits_toolset_${TOOLSET}.yaml}"
MOVEIT_CONTROLLERS_PATH="${MOVEIT_CONTROLLERS_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/moveit_controllers_toolset_${TOOLSET}.yaml}"
MOVEIT_RVIZ_CONFIG_PATH="${MOVEIT_RVIZ_CONFIG_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/config/moveit.rviz}"
MOVEIT_LAUNCH_PATH="${MOVEIT_LAUNCH_PATH:-$WORK_DIR/xczs_inspection_robot_moveit_config/launch/move_group.launch.py}"
NAV2_LAUNCH_PATH="${NAV2_LAUNCH_PATH:-$WORK_DIR/xczs_inspection_robot_nav2/launch/navigation.launch.py}"
# 可选覆盖：默认留空，由 scenes.yaml 的 nav2_map 决定实际地图。
NAV2_MAP_PATH="${NAV2_MAP_PATH:-}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$WORK_DIR/xczs_inspection_robot_nav2/config/nav2_params.yaml}"
SCENE="${SCENE:-cabinet_operation}"
SCENES_CONFIG="${SCENES_CONFIG:-$WORK_DIR/xczs_inspection_robot_control/config/scenes.yaml}"
ROBOT_BRINGUP="${ROBOT_BRINGUP:-true}"
GAZEBO_ENABLED="${GAZEBO_ENABLED:-true}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
MOVEIT_ENABLED="${MOVEIT_ENABLED:-true}"
CABINET_BRINGUP="${CABINET_BRINGUP:-true}"
SPAWN_CABINET="${SPAWN_CABINET:-true}"
SPAWN_Z="${SPAWN_Z:-0.515}"
CABINET_POSE_SOURCE="${CABINET_POSE_SOURCE:-static}"
PREFLIGHT_ONLY="${XCZS_PREFLIGHT_ONLY:-false}"
# 仅本地 Web + Gazebo + 内置机器人栈支持运行中切换；保留显式开关，方便
# 诊断外部栈或旧部署。关闭时严格走历史单体 launch 路径。
TOOLSET_HOTSWAP_REQUESTED="${XCZS_TOOLSET_HOTSWAP:-true}"
TOOLSET_HOTSWAP_ACTIVE="false"
TOOLSET_HOTSWAP_LABEL="未启用（等待运行模式判定）"
TOOLSET_SUPERVISOR_SCRIPT="$WORK_DIR/xczs_inspection_robot_control/scripts/toolset_supervisor.py"

CONTROL_ORIGINS=()
NORMALIZED_CONTROL_ORIGINS=()
CONTROL_ORIGIN_ARGS=()

# ── 选项 ──────────────────────────────────────────────────────────
GAZEBO_GUI="true"
WITH_PROXY="false"
CONTROL_MODE="web"
NAV2_ENABLED="false"
ZENOH_REQUIRED="false"

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

if [ "$CONTROL_MODE" = "web" ] || [ "$WITH_PROXY" = "true" ]; then
    ZENOH_REQUIRED="true"
fi

# ── 解析最终运行模式 ──────────────────────────────────────────────
if [ "$GAZEBO_ENABLED" = "false" ]; then
    GAZEBO_GUI="false"
fi
DISPLAY_AVAILABLE="true"
DISPLAY_ERROR=""
DISPLAY_STATUS="未验证（系统缺少 xdpyinfo 或 timeout）"
if [ -z "${DISPLAY:-}" ]; then
    DISPLAY_AVAILABLE="false"
    DISPLAY_ERROR="DISPLAY 未设置"
    DISPLAY_STATUS="不可用（$DISPLAY_ERROR）"
elif command -v xdpyinfo >/dev/null 2>&1 &&
    command -v timeout >/dev/null 2>&1; then
    if timeout --kill-after=1 3 xdpyinfo -display "$DISPLAY" \
        >/dev/null 2>&1; then
        DISPLAY_STATUS="已验证可连接（DISPLAY=$DISPLAY）"
    else
        DISPLAY_AVAILABLE="false"
        DISPLAY_ERROR="无法连接 DISPLAY=$DISPLAY"
        DISPLAY_STATUS="不可用（$DISPLAY_ERROR）"
    fi
fi
if [ "$DISPLAY_AVAILABLE" = "false" ]; then
    echo "⚠ 未检测到可用显示器 ($DISPLAY_ERROR)，已禁用 Gazebo 图形界面；基于渲染的 RGB 相机也不会发布图像"
    GAZEBO_GUI="false"
fi

# ── 显示/图形栈就绪等待 ─────────────────────────────────────────
# 刚开机/刚登录后 X server 能立即应答，但 NVIDIA/Mutter 的 GLX 上下文栈
# 仍在初始化；此时启动 gzserver，其 OGRE 渲染窗口创建会在 GLX FSAA 阶段
# 段错误（实测：登录后约 60-90 秒内启动 gzserver 崩溃，约 4 分钟后成功）。
# 通过 X socket 创建时间判断显示是否“新鲜”：过新则有界等待其成熟，
# 避免“开机即启动”撞上该竞态。正常启动（显示早已就绪）零延迟。
# 阈值可用 XCZS_DISPLAY_SETTLE_SECONDS / XCZS_DISPLAY_SETTLE_MAX_WAIT 覆盖。
if [ "$DISPLAY_AVAILABLE" = "true" ] &&
    [ "$GAZEBO_ENABLED" = "true" ] &&
    [ "$PREFLIGHT_ONLY" != "true" ]; then
    _display_num="${DISPLAY##*:}"
    _display_num="${_display_num%%.*}"
    _display_socket="/tmp/.X11-unix/X${_display_num}"
    _display_settle_seconds="${XCZS_DISPLAY_SETTLE_SECONDS:-180}"
    _display_settle_max_wait="${XCZS_DISPLAY_SETTLE_MAX_WAIT:-180}"
    if [ -S "$_display_socket" ]; then
        _x_age=$(( $(date +%s) - $(stat -c %Y "$_display_socket" 2>/dev/null || echo 0) ))
        if [ "$_x_age" -lt 0 ]; then _x_age=0; fi
        _waited=0
        while [ "$_x_age" -lt "$_display_settle_seconds" ] &&
            [ "$_waited" -lt "$_display_settle_max_wait" ]; do
            if [ "$_waited" -eq 0 ]; then
                echo "⚠ 显示刚启动 ${_x_age}s（低于 ${_display_settle_seconds}s 就绪阈值），等待图形栈稳定，避免 gzserver GLX 启动崩溃..."
            fi
            sleep 5
            _waited=$(( _waited + 5 ))
            _x_age=$(( $(date +%s) - $(stat -c %Y "$_display_socket" 2>/dev/null || echo 0) ))
        done
        if [ "$_waited" -gt 0 ]; then
            echo "  图形栈就绪等待完成（已等 ${_waited}s，显示年龄 ${_x_age}s）。"
            DISPLAY_STATUS="$DISPLAY_STATUS（已等待图形栈稳定）"
        fi
    fi
fi

if [ "$CONTROL_MODE" = "keyboard" ] &&
    [ "$DISPLAY_AVAILABLE" = "false" ]; then
    echo "ERROR: 键盘调试控制需要可连接的 DISPLAY 和终端界面。"
    exit 1
fi
if [ "$CONTROL_MODE" = "keyboard" ] &&
    ! command -v xfce4-terminal >/dev/null 2>&1; then
    echo "ERROR: 键盘调试控制的启动前缀依赖 xfce4-terminal，请安装后重试。"
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
    # 每个后台命令都由 setsid 启动，预期 PGID 与 leader PID 相同。
    # PGID 独立保存；leader 退出后仍可清理同组的 launch 子进程。
    MANAGED_PGIDS+=("$pid")
    MANAGED_LABELS+=("$label")
    # setsid 应使该 PID 同时成为新进程组的 ID。确认后才继续，
    # 避免清理时只停主进程却遗留 launch/uvicorn 子进程。
    for ((attempt=0; attempt < 20; attempt++)); do
        process_group="$(
            ps -o pgid= -p "$pid" 2>/dev/null |
                tr -d '[:space:]' || true
        )"
        if [ "$process_group" = "$pid" ]; then
            return 0
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            # leader 可能在登记期间退出；此时精确的进程组仍可能包含
            # 它启动的子进程，保留已知 PGID 供监控和 cleanup 使用。
            return 0
        fi
        sleep 0.01
    done
    echo "ERROR: $label 未进入独立进程组（PID=$pid, PGID=${process_group:-unknown}）。" >&2
    kill -TERM "$pid" 2>/dev/null || true
    exit 1
}

_start_managed_process() {
    local pid_variable="$1"
    local label="$2"
    local supervisor_pid="$BASHPID"
    local supervisor_identity=""
    local managed_pid=""
    shift 2
    if [ "$#" -eq 0 ]; then
        echo "ERROR: $label 缺少启动命令。" >&2
        exit 1
    fi
    supervisor_identity="$(
        ps -o lstart=,sid= -p "$supervisor_pid" 2>/dev/null || true
    )"
    if [ -z "$supervisor_identity" ]; then
        echo "ERROR: 无法读取启动监督进程 PID=$supervisor_pid 的身份。" >&2
        exit 1
    fi

    # guardian 与目标命令共同位于 setsid 创建的独立会话/进程组内。
    # 正常退出由 cleanup 精确终止这个 PGID；若外层监督 shell 遭到
    # SIGKILL、终端强制回收等无法运行 EXIT trap 的情况，guardian 会
    # 发现监督进程身份消失，并只清理自己的 PGID。目标命令启动后才
    # 忽略终止信号，避免把忽略属性继承给 ros2 launch/gzserver。
    setsid bash -c '
set +e
supervisor_pid="$1"
supervisor_identity="$2"
guard_timeout="$3"
shift 3

# Never start the target unless setsid actually gave this guardian a private
# process group.  Otherwise a later group signal could touch the parent
# terminal session, while killing only the guardian would orphan the target.
own_group="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d "[:space:]")"
if [ "$own_group" != "$$" ]; then
    echo "xczs guardian did not enter an independent process group" >&2
    exit 125
fi

"$@" &
target_pid=$!
trap "" HUP INT TERM

supervisor_is_running() {
    local current_identity=""
    current_identity="$(
        ps -o lstart=,sid= -p "$supervisor_pid" 2>/dev/null || true
    )"
    [ -n "$current_identity" ] &&
        [ "$current_identity" = "$supervisor_identity" ]
}

target_is_running() {
    local process_details=""
    local state=""
    process_details="$(ps -o stat= -p "$target_pid" 2>/dev/null)" || return 1
    read -r state <<< "$process_details"
    [ -n "$state" ] && [[ "$state" != Z* ]]
}

group_has_other_processes() {
    local process_group=""
    local process_pid=""
    local state=""
    local process_snapshot=""
    # 先等 ps 快照进程退出再解析，否则检查动作自身也属于当前 PGID，
    # 会被误判成目标命令留下的子孙。
    process_snapshot="$(ps -eo pgid=,pid=,stat= 2>/dev/null || true)"
    while read -r process_group process_pid state; do
        if [ "$process_group" = "$$" ] && [ "$process_pid" != "$$" ] &&
            [ -n "$state" ] && [[ "$state" != Z* ]] &&
            kill -0 "$process_pid" 2>/dev/null; then
            return 0
        fi
    done <<< "$process_snapshot"
    return 1
}

abandoned="false"
while target_is_running; do
    if ! supervisor_is_running; then
        abandoned="true"
        break
    fi
    # 一秒级监督足以处理宿主 shell 消失，同时避免长期仿真中频繁启动
    # ps 给 CPU 和进程调度带来额外抖动。
    sleep 1
done

target_status=0
if [ "$abandoned" = "false" ]; then
    wait "$target_pid"
    target_status=$?
else
    target_status=143
fi

# 主命令退出并不代表同组的 Gazebo/launch 子孙已退出。无论是主命令
# 先结束还是监督进程消失，都给全组一次 TERM 宽限，再强制收尾。
if group_has_other_processes; then
    kill -TERM -- "-$$" 2>/dev/null || true
    deadline=$((SECONDS + guard_timeout))
    while group_has_other_processes && (( SECONDS < deadline )); do
        sleep 0.2
    done
fi
if group_has_other_processes; then
    kill -KILL -- "-$$" 2>/dev/null || true
fi
exit "$target_status"
' xczs-managed-guardian \
        "$supervisor_pid" "$supervisor_identity" \
        "$MANAGED_GUARD_TIMEOUT_SEC" "$@" &
    managed_pid=$!
    printf -v "$pid_variable" '%d' "$managed_pid"
    _register_process "$managed_pid" "$label"
}

_unregister_last_process() {
    local pid="$1"
    local last_index=$((${#MANAGED_PIDS[@]} - 1))
    local process_group=""
    if (( last_index < 0 )) || [ "${MANAGED_PIDS[$last_index]}" != "$pid" ]; then
        echo "ERROR: 只能逆序注销最后一个受管进程: $pid" >&2
        exit 1
    fi
    process_group="${MANAGED_PGIDS[$last_index]}"
    if _process_group_is_running "$process_group"; then
        echo "ERROR: 受管进程组 PGID=$process_group 仍有子进程，不能注销。" >&2
        exit 1
    fi
    unset "MANAGED_PIDS[$last_index]"
    unset "MANAGED_PGIDS[$last_index]"
    unset "MANAGED_LABELS[$last_index]"
}

_managed_leader_is_running() {
    local pid="$1"
    local expected_group="$2"
    local process_details=""
    local process_group=""
    local state=""
    process_details="$(ps -o pgid=,stat= -p "$pid" 2>/dev/null)" || return 1
    read -r process_group state <<< "$process_details"
    # 同时核对 PGID，不能把后来复用相同数值 PID 的外部进程误认为
    # 本次启动的 leader。
    [ "$process_group" = "$expected_group" ] &&
        [ -n "$state" ] && [[ "$state" != Z* ]]
}

_process_group_is_running() {
    local expected_group="$1"
    local process_group=""
    local state=""
    while read -r process_group state; do
        if [ "$process_group" = "$expected_group" ] &&
            [ -n "$state" ] && [[ "$state" != Z* ]]; then
            return 0
        fi
    done < <(ps -eo pgid=,stat= 2>/dev/null)
    return 1
}

_signal_process_group() {
    local process_group="$1"
    local signal="$2"
    # 只向登记时确认的精确进程组发信号。绝不退化为按 leader PID
    # 发信号，以免 leader 退出后误伤复用相同 PID 的外部进程。
    _process_group_is_running "$process_group" || return 0
    kill "-$signal" -- "-$process_group" 2>/dev/null || true
}

_wait_for_managed_groups() {
    local timeout="$1"
    local deadline=$((SECONDS + timeout))
    local process_group=""
    local still_running="false"
    while true; do
        still_running="false"
        for process_group in "${MANAGED_PGIDS[@]}"; do
            if _process_group_is_running "$process_group"; then
                still_running="true"
                break
            fi
        done
        [ "$still_running" = "false" ] && return 0
        (( SECONDS >= deadline )) && return 1
        sleep 0.2
    done
}

_create_launch_runtime_directory() {
    if [ -n "${LAUNCH_RUNTIME_DIRECTORY:-}" ]; then
        echo "ERROR: 本次启动的临时目录已经创建。" >&2
        return 1
    fi
    LAUNCH_RUNTIME_DIRECTORY="$(
        mktemp -d /tmp/xczs_runtime_XXXXXXXX
    )" || {
        echo "ERROR: 无法创建本次启动的临时目录。" >&2
        return 1
    }
    export XCZS_LAUNCH_RUNTIME_DIRECTORY="$LAUNCH_RUNTIME_DIRECTORY"
}

_configure_gazebo_log_directory() {
    # Gazebo Classic 默认会把运行日志放到用户级目录，长时间仿真会无界增长。
    # 本入口已经拥有一个每次运行独立的临时目录，因此仅在用户没有显式指定
    # GAZEBO_LOG_PATH 时把本次日志重定向到其中；模型资源缓存绝不触碰。
    if [ "$GAZEBO_ENABLED" != "true" ]; then
        return 0
    fi
    if [ -n "${GAZEBO_LOG_PATH:-}" ]; then
        GAZEBO_LOG_DIRECTORY="$GAZEBO_LOG_PATH"
        return 0
    fi
    if [ -z "${LAUNCH_RUNTIME_DIRECTORY:-}" ]; then
        echo "ERROR: 创建 Gazebo 日志目录前必须先创建本次运行目录。" >&2
        return 1
    fi
    GAZEBO_LOG_DIRECTORY="$LAUNCH_RUNTIME_DIRECTORY/gazebo_logs"
    mkdir -p -- "$GAZEBO_LOG_DIRECTORY" || {
        echo "ERROR: 无法创建本次 Gazebo 日志目录: $GAZEBO_LOG_DIRECTORY" >&2
        return 1
    }
    export GAZEBO_LOG_PATH="$GAZEBO_LOG_DIRECTORY"
}

_cleanup_launch_runtime_directory() {
    local runtime_directory="${LAUNCH_RUNTIME_DIRECTORY:-}"
    [ -z "$runtime_directory" ] && return 0
    # 该值只由上面的 mktemp 生成；再次约束绝对前缀，避免任何宽范围删除。
    case "$runtime_directory" in
        /tmp/xczs_runtime_*) ;;
        *)
            echo "ERROR: 拒绝清理异常临时目录: $runtime_directory" >&2
            return 1
            ;;
    esac
    if [ -e "$runtime_directory" ]; then
        rm -rf -- "$runtime_directory" || {
            echo "ERROR: 无法清理本次启动的临时目录: $runtime_directory" >&2
            return 1
        }
    fi
    if [ -e "$runtime_directory" ]; then
        echo "ERROR: 本次启动的临时目录仍然存在: $runtime_directory" >&2
        return 1
    fi
    LAUNCH_RUNTIME_DIRECTORY=""
    unset XCZS_LAUNCH_RUNTIME_DIRECTORY
}

cleanup() {
    local exit_code="${1:-$?}"
    local all_closed="true"
    local forced="false"
    local index=0
    local pid=""
    local process_group=""
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

    # 按逆启动顺序发出停止信号：Web/代理与可替换机器人监督器先于常驻
    # Gazebo 世界登记，因此所有受管进程都能在统一宽限期内自行收尾。
    for ((index=${#MANAGED_PIDS[@]} - 1; index >= 0; index--)); do
        process_group="${MANAGED_PGIDS[$index]}"
        if _process_group_is_running "$process_group"; then
            _signal_process_group "$process_group" TERM
        fi
    done

    _wait_for_managed_groups "$SHUTDOWN_TIMEOUT_SEC" || true

    for ((index=${#MANAGED_PIDS[@]} - 1; index >= 0; index--)); do
        process_group="${MANAGED_PGIDS[$index]}"
        if _process_group_is_running "$process_group"; then
            echo "WARNING: ${MANAGED_LABELS[$index]} 未在 ${SHUTDOWN_TIMEOUT_SEC}s 内退出，强制终止。" >&2
            _signal_process_group "$process_group" KILL
            forced="true"
        fi
    done
    if [ "$forced" = "true" ]; then
        _wait_for_managed_groups "$SHUTDOWN_TIMEOUT_SEC" || true
    fi
    for pid in "${MANAGED_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    if ! _cleanup_launch_runtime_directory; then
        all_closed="false"
        if [ "$exit_code" -eq 0 ]; then
            exit_code=1
        fi
    fi
    if [ "${#MANAGED_PIDS[@]}" -gt 0 ]; then
        for process_group in "${MANAGED_PGIDS[@]}"; do
            if _process_group_is_running "$process_group"; then
                all_closed="false"
                echo "ERROR: 受管进程组 PGID=$process_group 仍未退出。" >&2
            fi
        done
        if [ "$all_closed" = "true" ]; then
            echo "  本次启动的进程已全部关闭"
        elif [ "$exit_code" -eq 0 ]; then
            exit_code=1
        fi
    fi
    return "$exit_code"
}

_cleanup_on_exit() {
    local exit_code=$?
    # Bash preserves the status that triggered EXIT and ignores the trap
    # function's return value.  Remove the trap before the final explicit
    # exit so a cleanup failure (for example, a surviving managed PGID) is
    # observable by callers without recursively invoking cleanup.
    trap - EXIT
    set +e
    cleanup "$exit_code"
    exit_code=$?
    exit "$exit_code"
}

trap _cleanup_on_exit EXIT
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
if [ "$ZENOH_REQUIRED" = "true" ] && [ ! -x "$ZENOH_BRIDGE" ]; then
    echo "ERROR: zenoh-bridge-ros2dds 不存在或不可执行: $ZENOH_BRIDGE"
    exit 1
fi
for required_command in setsid ps tr mktemp mkdir rm; do
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
    local target_name="$4"
    local display_name="${5:-$target_name}"
    local decimal=""
    # Bash arithmetic treats a leading zero as octal.  Normalize validated
    # decimal text before it participates in ranges or derived ports, so
    # values such as ROS_DOMAIN_ID=08 behave like ordinary decimal input.
    if ! [[ "$value" =~ ^[0-9]+$ ]] || [ "${#value}" -gt 10 ]; then
        echo "ERROR: $display_name 必须是 ${minimum}..${maximum} 的整数。"
        exit 1
    fi
    decimal=$((10#$value))
    if (( decimal < minimum || decimal > maximum )); then
        echo "ERROR: $display_name 必须是 ${minimum}..${maximum} 的整数。"
        exit 1
    fi
    printf -v "$target_name" '%d' "$decimal"
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

# 绑定测试：0 = 可绑定，1 = 不可绑定。预检模式静默返回结果，正式启动才
# 回显原始绑定错误。只能通过 if / 条件上下文调用——在 set -e 下用命令
# 替换捕获失败会直接中止脚本。
_port_bindable() {
    local host="$1"
    local port="$2"
    PREFLIGHT_ONLY="$PREFLIGHT_ONLY" "$PYTHON_BIN" - "$host" "$port" <<'PY'
import os
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
quiet = os.environ.get("PREFLIGHT_ONLY") == "true"
errors = []
seen = set()
try:
    addresses = socket.getaddrinfo(
        host, port, type=socket.SOCK_STREAM, flags=socket.AI_PASSIVE
    )
except OSError as error:
    if not quiet:
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
if not quiet:
    print("; ".join(dict.fromkeys(errors)), file=sys.stderr)
raise SystemExit(1)
PY
}

# 终止占用给定端口的旧实例进程并等待端口释放。
# 只自动终止本项目运行实例的进程（zenoh-bridge / gzserver / 控制服务 /
# 启动脚本等）；非本项目进程占用时拒绝终止并返回 1，避免误杀无关服务。
# 返回 0 = 端口已释放；1 = 无法释放（调用方将报错退出）。
_kill_port_holder() {
    local port="$1"
    local label="$2"
    local pids pid pcomm pcmd i
    # set -e 下命令替换失败会中止脚本，ss 查询失败时降级为“无占用者”。
    pids="$(ss -H -ltnp "sport = :$port" 2>/dev/null \
        | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' | sort -u || true)"
    if [ -z "$pids" ]; then
        # 没有监听进程但仍无法绑定（如刚终止的进程留下的监听 socket /
        # TIME_WAIT 尚未消退），无从终止；返回 2 让调用方有界等待后复检绑定。
        return 2
    fi
    for pid in $pids; do
        pcomm="$(ps -p "$pid" -o comm= 2>/dev/null || true)"
        pcmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
        if [ -z "$pcomm" ]; then
            # 进程在 ss 与 ps 之间已退出，占用视为消失，交给绑定复检。
            continue
        fi
        case "${pcomm} ${pcmd}" in
            *"zenoh-bridge"*|*"gzserver"*|*"gzclient"*|*"control_server"*|*"toolset_supervisor"*|*"start_xczs_bridge"*|*"run_all.sh"*|*"xczs"*)
                echo "  终止旧实例进程 pid=$pid（$pcomm）。"
                kill -TERM "$pid" 2>/dev/null || true
                ;;
            *)
                echo "ERROR: $label 端口被非本项目进程 pid=$pid（$pcomm）占用，拒绝自动终止。" >&2
                echo "      请先手动停止该服务后再启动。" >&2
                return 1
                ;;
        esac
    done
    # 优雅退出宽限：gzserver 关闭复杂世界较慢，默认等 10 秒（可用
    # XCZS_PORT_RELEASE_GRACE 覆盖）等待监听 socket 释放；仍占用则强制终止。
    local _grace=$(( ${XCZS_PORT_RELEASE_GRACE:-10} < 1 ? 1 : ${XCZS_PORT_RELEASE_GRACE:-10} ))
    local _i
    for ((_i = 1; _i <= _grace; _i++)); do
        sleep 1
        if ! ss -H -ltnp "sport = :$port" 2>/dev/null | grep -q 'pid='; then
            return 0
        fi
    done
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "  进程 pid=$pid 未在 ${_grace} 秒内退出，发送 SIGKILL。"
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    # SIGKILL 后进程消失需要一点时间（僵尸回收、内核拆除 socket），再等最多 5 秒。
    for ((_i = 1; _i <= 5; _i++)); do
        sleep 1
        if ! ss -H -ltnp "sport = :$port" 2>/dev/null | grep -q 'pid='; then
            return 0
        fi
    done
    # 端口仍未释放：可能是已死进程的 TIME_WAIT/半关闭状态短暂占据。
    # 返回 2，调用方将做有界绑定复检（SO_REUSEADDR 通常可越过 TIME_WAIT）。
    return 2
}

_require_port_available() {
    local host="$1"
    local port="$2"
    local label="$3"
    if _port_bindable "$host" "$port"; then
        return 0
    fi
    if [ "$PREFLIGHT_ONLY" = "true" ]; then
        # 预检只验证启动配置；端口被已运行的 XCZS 实例占用不是配置
        # 错误，收集到预检摘要里提示，不中止。正式启动才会处理占用。
        PREFLIGHT_PORT_CONFLICTS+=("$label|$port")
        return 0
    fi
    # 端口被占用：终止旧实例进程后继续启动；无法自动释放才硬性失败。
    echo "WARN: $label 端口 $host:$port 被占用，尝试终止旧实例进程。"
    local _kill_rc _tries
    _kill_port_holder "$port" "$label"
    _kill_rc=$?
    if [ "$_kill_rc" -eq 1 ]; then
        # 非本项目进程占用：_kill_port_holder 已输出明确拒绝原因，硬性失败。
        echo "ERROR: $label 无法绑定 $host:$port（被非本项目进程占用）。" >&2
        _describe_port_owner "$port"
        exit 1
    fi
    # 占用进程已处理（0=监听 socket 已释放；2=无占用者可终止，或 SIGKILL 后
    # TIME_WAIT 短暂收尾），统一有界等待后复检绑定，避免误报“未能自动释放”。
    for _tries in 1 2 3 4 5 6 7 8; do
        if _port_bindable "$host" "$port"; then
            echo "  → 端口 $host:$port 已释放，继续启动。"
            return 0
        fi
        sleep 2
    done
    echo "ERROR: $label 无法绑定 $host:$port（端口被占用且未能自动释放）。" >&2
    _describe_port_owner "$port"
    exit 1
}

if [ "$ZENOH_REQUIRED" = "true" ]; then
    _require_integer_range "$BRIDGE_TCP_PORT" 1 65535 BRIDGE_TCP_PORT
    _require_integer_range "$BRIDGE_REST_PORT" 1 65535 BRIDGE_REST_PORT
fi
_require_integer_range \
    "$STARTUP_TIMEOUT_SEC" 1 60 STARTUP_TIMEOUT_SEC XCZS_STARTUP_TIMEOUT_SEC
_require_integer_range \
    "$ROBOT_READY_TIMEOUT_SEC" 1 300 \
    ROBOT_READY_TIMEOUT_SEC XCZS_ROBOT_READY_TIMEOUT_SEC
_require_integer_range \
    "$SHUTDOWN_TIMEOUT_SEC" 1 60 \
    SHUTDOWN_TIMEOUT_SEC XCZS_SHUTDOWN_TIMEOUT_SEC
_require_integer_range \
    "$MANAGED_GUARD_TIMEOUT_SEC" 1 30 \
    MANAGED_GUARD_TIMEOUT_SEC XCZS_MANAGED_GUARD_TIMEOUT_SEC
if [ "$ZENOH_REQUIRED" = "true" ] &&
    [ "$BRIDGE_TCP_PORT" = "$BRIDGE_REST_PORT" ]; then
    echo "ERROR: BRIDGE_TCP_PORT 与 BRIDGE_REST_PORT 不能相同。"
    exit 1
fi
if [ "$CONTROL_MODE" = "web" ]; then
    _require_integer_range "$CONTROL_PORT" 1 65535 CONTROL_PORT
    if [ -z "$CONTROL_ALLOWED_ORIGINS" ]; then
        CONTROL_ALLOWED_ORIGINS="http://localhost:$CONTROL_PORT,http://127.0.0.1:$CONTROL_PORT"
    fi
    IFS=',' read -r -a CONTROL_ORIGINS <<< "$CONTROL_ALLOWED_ORIGINS"
    for origin in "${CONTROL_ORIGINS[@]}"; do
        origin="${origin#"${origin%%[![:space:]]*}"}"
        origin="${origin%"${origin##*[![:space:]]}"}"
        if [ -n "$origin" ]; then
            NORMALIZED_CONTROL_ORIGINS+=("$origin")
            CONTROL_ORIGIN_ARGS+=(--allowed-origin "$origin")
        fi
    done
    if [ "${#CONTROL_ORIGIN_ARGS[@]}" -eq 0 ]; then
        echo "ERROR: XCZS_CONTROL_ORIGINS 至少需要一个 Web Origin。"
        exit 1
    fi
    CONTROL_ORIGINS=("${NORMALIZED_CONTROL_ORIGINS[@]}")
    CONTROL_ALLOWED_ORIGINS="$(IFS=,; echo "${CONTROL_ORIGINS[*]}")"
    if [ "$ZENOH_REQUIRED" = "true" ] &&
        { [ "$CONTROL_PORT" = "$BRIDGE_TCP_PORT" ] ||
          [ "$CONTROL_PORT" = "$BRIDGE_REST_PORT" ]; }; then
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
    "$TOOLSET_HOTSWAP_REQUESTED" \
    "$PREFLIGHT_ONLY"; do
    if [ "$boolean_value" != "true" ] && [ "$boolean_value" != "false" ]; then
        echo "ERROR: ROBOT_BRINGUP、GAZEBO_ENABLED、USE_SIM_TIME、MOVEIT_ENABLED、CABINET_BRINGUP、SPAWN_CABINET、XCZS_TOOLSET_HOTSWAP 和 XCZS_PREFLIGHT_ONLY 必须为 true 或 false。"
        exit 1
    fi
done
if [ "$ZENOH_REQUIRED" = "true" ] &&
    [ "$ZENOH_LAN_ENABLED" != "true" ] &&
    [ "$ZENOH_LAN_ENABLED" != "false" ]; then
    echo "ERROR: XCZS_ZENOH_LAN_ENABLED 必须为 true 或 false。"
    exit 1
fi
if [ "$ZENOH_LAN_ENABLED" = "true" ]; then
    ZENOH_BIND_HOST="0.0.0.0"
    ZENOH_SCOUTING_ARGS=()
    ZENOH_NETWORK_LABEL="局域网开放（已启用 multicast scouting）"
else
    ZENOH_BIND_HOST="127.0.0.1"
    ZENOH_SCOUTING_ARGS=(--no-multicast-scouting)
    ZENOH_NETWORK_LABEL="本机隔离（multicast scouting 已禁用）"
fi
if [ "$ZENOH_REQUIRED" = "false" ]; then
    ZENOH_NETWORK_LABEL="未启用（键盘模式无需 Zenoh）"
fi
CABINET_ACTION_REQUIRED="false"
if [ "$CABINET_BRINGUP" = "true" ] && [ "$MOVEIT_ENABLED" = "true" ]; then
    # cabinet_button_operator 只在这两个子系统同时启用时启动；其他组合
    # 仍可提供导航能力，不应等待一个按配置不会存在的 Action。
    CABINET_ACTION_REQUIRED="true"
fi
# A/B 的关节和 ros2_control 接口互斥，Gazebo Classic 无法把它们热插拔到
# 同一个实体。受管模式让世界进程常驻，toolset_supervisor 仅替换机器人及其
# 控制/柜体运行栈。因此只在入口实际拥有 Web、Gazebo 和机器人时开启；外部
# 栈、无 Gazebo 或键盘模式都保留既有单体 launch 行为。
if [ "$TOOLSET_HOTSWAP_REQUESTED" = "true" ] && \
    [ "$CONTROL_MODE" = "web" ] && \
    [ "$GAZEBO_ENABLED" = "true" ] && \
    [ "$ROBOT_BRINGUP" = "true" ]; then
    TOOLSET_HOTSWAP_ACTIVE="true"
    TOOLSET_HOTSWAP_LABEL="启用（Gazebo 世界常驻，机器人末端可一键替换）"
elif [ "$TOOLSET_HOTSWAP_REQUESTED" = "false" ]; then
    TOOLSET_HOTSWAP_LABEL="已由 XCZS_TOOLSET_HOTSWAP=false 关闭（单体 launch）"
else
    TOOLSET_HOTSWAP_LABEL="当前模式不支持（需要 Web + 本地 Gazebo + 内置机器人栈）"
fi
if [ "$CABINET_POSE_SOURCE" != "static" ] && [ "$CABINET_POSE_SOURCE" != "topic" ]; then
    echo "ERROR: CABINET_POSE_SOURCE 必须为 static 或 topic。"
    exit 1
fi
if [ "$SPAWN_CABINET" = "true" ] && [ "$CABINET_BRINGUP" = "false" ]; then
    echo "ERROR: SPAWN_CABINET=true 需要 CABINET_BRINGUP=true。"
    exit 1
fi
if ! "$PYTHON_BIN" -c '
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
if not math.isfinite(value):
    raise SystemExit(1)
' "$SPAWN_Z" 2>/dev/null; then
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

_read_cabinet_operation_actions() {
    # The cabinet operators are namespaced per physical instance.  Do not use
    # the legacy unscoped action name here: Zenoh may advertise a proxy for it
    # even when no real rclcpp action server is ready, which would leave the
    # hot-switch supervisor permanently in "starting".
    "$PYTHON_BIN" - "$CABINET_INSTANCES_PATH" <<'PY'
import re
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1]).expanduser()
try:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    instances = document["instances"]
except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
    raise SystemExit(f"cannot read cabinet instances: {error}")

if not isinstance(instances, list) or not instances:
    raise SystemExit("cabinet instances must contain at least one item")

pattern = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
names = set()
for instance in instances:
    name = instance.get("name") if isinstance(instance, dict) else None
    if not isinstance(name, str) or pattern.fullmatch(name) is None:
        raise SystemExit("cabinet instance name is invalid")
    if name in names:
        raise SystemExit(f"duplicate cabinet instance name: {name}")
    names.add(name)
    print(f"/xczs/cabinet/{name}/operate_cabinet_control")
PY
}

_toolset_path_template() {
    local path="$1"
    local label="$2"
    local active_marker="toolset_${TOOLSET}"
    local replacement='toolset_{toolset}'
    # A runtime switch needs an unambiguous path for *both* A and B.  Accept a
    # caller-provided ``{toolset}`` placeholder, or derive it from the active
    # conventional filename.  A fixed A/B file would silently launch a robot
    # whose URDF and MoveIt semantic/controller contracts disagree.
    if [[ "$path" == *"{toolset}"* ]]; then
        printf '%s' "$path"
        return 0
    fi
    if [[ "$path" == *"$active_marker"* ]]; then
        printf '%s' "${path//"$active_marker"/$replacement}"
        return 0
    fi
    echo "ERROR: 启用末端热切换时，$label 必须包含 {toolset} 或当前套装标记 $active_marker: $path" >&2
    exit 1
}

_toolset_path_for() {
    local template="$1"
    local target_toolset="$2"
    printf '%s' "${template//\{toolset\}/$target_toolset}"
}

_require_file "$CABINET_ROBOT_ADAPTER_PATH"
_require_file "$SCENES_CONFIG"
if [ "$ZENOH_REQUIRED" = "true" ]; then
    _require_file "$ZENOH_BRIDGE_CONFIG"
fi
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
        _require_file "$MOVEIT_JOINT_LIMITS_PATH"
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
        if [ -n "$NAV2_MAP_PATH" ]; then
            _require_file "$NAV2_MAP_PATH"
        fi
        _require_file "$NAV2_PARAMS_FILE"
    fi
fi
if [ "$GAZEBO_ENABLED" = "true" ]; then
    _require_file "$SIMULATION_WORLD_PATH"
fi
if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ]; then
    _require_file "$TOOLSET_SUPERVISOR_SCRIPT"
fi

CABINET_OPERATION_ACTIONS=()
if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ] && \
    [ "$CABINET_ACTION_REQUIRED" = "true" ]; then
    if ! _cabinet_operation_actions="$(_read_cabinet_operation_actions)"; then
        echo "ERROR: 无法解析柜体操作 Action 名称，已中止末端热切换启动。" >&2
        exit 1
    fi
    while IFS= read -r _cabinet_operation_action; do
        [ -z "$_cabinet_operation_action" ] && continue
        CABINET_OPERATION_ACTIONS+=("$_cabinet_operation_action")
    done <<< "$_cabinet_operation_actions"
    if [ "${#CABINET_OPERATION_ACTIONS[@]}" -eq 0 ]; then
        echo "ERROR: 末端热切换需要至少一个已配置柜体操作 Action。" >&2
        exit 1
    fi
fi

# The supervisor expands {toolset} immediately before starting each child
# launch.  Validate both resolved configurations before Gazebo is started so a
# bad B profile is reported at startup rather than after the robot was removed.
MOVEIT_SRDF_TEMPLATE="$MOVEIT_SRDF_PATH"
MOVEIT_JOINT_LIMITS_TEMPLATE="$MOVEIT_JOINT_LIMITS_PATH"
MOVEIT_CONTROLLERS_TEMPLATE="$MOVEIT_CONTROLLERS_PATH"
if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ] && [ "$MOVEIT_ENABLED" = "true" ]; then
    MOVEIT_SRDF_TEMPLATE="$(_toolset_path_template "$MOVEIT_SRDF_PATH" "MOVEIT_SRDF_PATH")"
    MOVEIT_JOINT_LIMITS_TEMPLATE="$(_toolset_path_template "$MOVEIT_JOINT_LIMITS_PATH" "MOVEIT_JOINT_LIMITS_PATH")"
    MOVEIT_CONTROLLERS_TEMPLATE="$(_toolset_path_template "$MOVEIT_CONTROLLERS_PATH" "MOVEIT_CONTROLLERS_PATH")"
    for _toolset_preflight in A B; do
        _require_file "$(_toolset_path_for "$MOVEIT_SRDF_TEMPLATE" "$_toolset_preflight")"
        _require_file "$(_toolset_path_for "$MOVEIT_JOINT_LIMITS_TEMPLATE" "$_toolset_preflight")"
        _require_file "$(_toolset_path_for "$MOVEIT_CONTROLLERS_TEMPLATE" "$_toolset_preflight")"
    done
fi

# ── ROS 域隔离 ─────────────────────────────────────────────────────
# 局域网内可能有其他 ROS 2 / Gazebo 仿真，使用独立的 ROS_DOMAIN_ID 避免
# DDS 流量互相干扰。可通过环境变量覆盖。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
_require_integer_range "$ROS_DOMAIN_ID" 0 232 ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
if [ "$ROS_LOCALHOST_ONLY" != "0" ] && [ "$ROS_LOCALHOST_ONLY" != "1" ]; then
    echo "ERROR: ROS_LOCALHOST_ONLY 必须为 0 或 1。"
    exit 1
fi

# CycloneDDS 的 localhost-only 自动 participant index 默认只尝试 0..9；
# 节点较多的完整仿真可能因此随机启动失败。只在用户完全没有设置
# CYCLONEDDS_URI 时扩大范围，绝不覆盖非空的自定义 DDS 网络配置。
CYCLONEDDS_WORKAROUND="未注入（ROS_LOCALHOST_ONLY=0）"
if [ "$ROS_LOCALHOST_ONLY" = "1" ]; then
    if [ -z "${CYCLONEDDS_URI:-}" ]; then
        export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
        CYCLONEDDS_WORKAROUND="已注入 MaxAutoParticipantIndex=60"
    else
        CYCLONEDDS_WORKAROUND="保留用户 CYCLONEDDS_URI"
    fi
fi

# Gazebo Classic 不识别 ROS_DOMAIN_ID；所有 gzserver、spawn_entity 和 ROS
# 插件通过同一个 GAZEBO_MASTER_URI 协同，因此可安全按 DDS 域派生端口。
# 用户显式提供 URI 时原样保留，方便连接外部 Gazebo master。
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:$((11345 + ROS_DOMAIN_ID))}"
GAZEBO_MASTER_ENDPOINT="$($PYTHON_BIN - "$GAZEBO_MASTER_URI" <<'PY'
import ipaddress
import sys
import unicodedata
from urllib.parse import urlsplit

uri = sys.argv[1]
if any(
    character.isspace()
    or unicodedata.category(character).startswith("C")
    for character in uri
):
    raise SystemExit(1)
try:
    parsed = urlsplit(uri)
    port = parsed.port
except ValueError as error:
    print(error, file=sys.stderr)
    raise SystemExit(1)
if (
    parsed.scheme != "http"
    or not parsed.hostname
    or port is None
    or parsed.username is not None
    or parsed.password is not None
    or parsed.query
    or parsed.fragment
    or parsed.path not in ("", "/")
):
    raise SystemExit(1)
if not 1 <= port <= 65535:
    raise SystemExit(1)
try:
    is_loopback = ipaddress.ip_address(parsed.hostname).is_loopback
except ValueError:
    is_loopback = parsed.hostname.rstrip(".").lower() == "localhost"
print(parsed.hostname, port, "true" if is_loopback else "false")
PY
)" || {
    echo "ERROR: GAZEBO_MASTER_URI 必须为 http://host:port（不能包含认证、查询或额外路径）。" >&2
    exit 1
}
read -r GAZEBO_MASTER_HOST GAZEBO_MASTER_PORT GAZEBO_MASTER_IS_LOCAL \
    <<< "$GAZEBO_MASTER_ENDPOINT"
if [ "$GAZEBO_MASTER_IS_LOCAL" = "true" ]; then
    GAZEBO_PREFLIGHT_LABEL="本机端口将执行绑定预检"
else
    GAZEBO_PREFLIGHT_LABEL="远端 master，跳过本机端口预检"
fi
if [ "$GAZEBO_ENABLED" = "false" ]; then
    GAZEBO_PREFLIGHT_LABEL="本地 Gazebo 未启用，跳过端口预检"
fi
if [ "$GAZEBO_ENABLED" = "true" ] &&
    [ "$GAZEBO_MASTER_IS_LOCAL" = "true" ]; then
    if { [ "$ZENOH_REQUIRED" = "true" ] &&
         { [ "$GAZEBO_MASTER_PORT" = "$BRIDGE_TCP_PORT" ] ||
           [ "$GAZEBO_MASTER_PORT" = "$BRIDGE_REST_PORT" ]; }; } ||
        { [ "$CONTROL_MODE" = "web" ] &&
          [ "$GAZEBO_MASTER_PORT" = "$CONTROL_PORT" ]; }; then
        echo "ERROR: GAZEBO_MASTER_URI 端口不能与 Zenoh 或 Web 端口相同。" >&2
        exit 1
    fi
fi

# ── 加载 ROS2 环境 ────────────────────────────────────────────────
source "$ROS2_SETUP"
source "$WORKSPACE_SETUP"
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: source 环境后仍找不到 ros2 命令。"
    exit 1
fi
if [ "$GAZEBO_MODEL_DATABASE_URI_EXPLICIT" = "x" ]; then
    export GAZEBO_MODEL_DATABASE_URI="$GAZEBO_MODEL_DATABASE_URI_REQUESTED"
    GAZEBO_MODEL_DATABASE_LABEL="保留用户配置"
else
    export GAZEBO_MODEL_DATABASE_URI=""
    GAZEBO_MODEL_DATABASE_LABEL="禁用远程库（仅使用本地模型）"
fi
GAZEBO_SYSTEM_MODEL_PATH="/usr/share/gazebo-11/models"
if [ -d "$GAZEBO_SYSTEM_MODEL_PATH" ]; then
    case ":${GAZEBO_MODEL_PATH:-}:" in
        *":$GAZEBO_SYSTEM_MODEL_PATH:"*) ;;
        *)
            export GAZEBO_MODEL_PATH="$GAZEBO_SYSTEM_MODEL_PATH${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
            ;;
    esac
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
    import alembic, aiosqlite, bcrypt, cryptography, fastapi
    import jose, multipart, passlib, pydantic, pydantic_settings
    import sqlalchemy, uvicorn, yaml, zenoh
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
        "$PYTHON_BIN" -c 'import run_xczs_proxy, zenoh'; then
        echo "ERROR: CDR→JSON 代理的 Python 依赖导入失败。"
        exit 1
    fi
fi

if [ "$CONTROL_MODE" = "web" ] || [ "$CABINET_BRINGUP" = "true" ]; then
    echo "  检查跨文件机器人/导航/柜体配置合同..."
    TOOLSET_CONTRACTS=("$TOOLSET")
    if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ]; then
        # Web 会在同一 Gazebo 世界内请求 A/B，因此两个配置都必须在启动
        # 前通过模型和业务点合同，不能只验证本次初始套装。
        TOOLSET_CONTRACTS=(A B)
    fi
    for _toolset_contract in "${TOOLSET_CONTRACTS[@]}"; do
        if ! "$PYTHON_BIN" "$WORK_DIR/scripts/check_adapter_contract" \
            --robot-adapter "$CABINET_ROBOT_ADAPTER_PATH" \
            --instances "$CABINET_INSTANCES_PATH" \
            --controls "$CABINET_CONTROLS_PATH" \
            --scene "$CABINET_SCENE_PATH" \
            --pose "$CABINET_POSE_PATH" \
            --toolset "$_toolset_contract" \
            --kinematics "$MOVEIT_KINEMATICS_PATH"; then
            echo "ERROR: 工具套装 $_toolset_contract 的启动配置合同校验失败。"
            exit 1
        fi
        if ! "$PYTHON_BIN" "$WORK_DIR/scripts/check_cabinet_model" \
            --toolset "$_toolset_contract"; then
            echo "ERROR: 工具套装 $_toolset_contract 的控制柜模型合同校验失败。"
            exit 1
        fi
    done
    if ! "$PYTHON_BIN" "$WORK_DIR/scripts/check_scene_config" \
        --scenes "$SCENES_CONFIG" \
        --nav2-params "$NAV2_PARAMS_FILE"; then
        echo "ERROR: 场景目录合同校验失败。"
        exit 1
    fi
fi

# 预检与正式启动采用同一端口检查。正式启动时端口被本项目旧实例进程
# （zenoh-bridge / gzserver / 控制服务 / 启动脚本）占用会自动终止占用
# 进程后继续；被非本项目进程占用仍只报错、不误杀。预检模式只验证配置，
# 端口占用收集到 PREFLIGHT_PORT_CONFLICTS 在摘要里提示，不当作配置错误。
PREFLIGHT_PORT_CONFLICTS=()
if [ "$ZENOH_REQUIRED" = "true" ]; then
    _require_port_available "$ZENOH_BIND_HOST" "$BRIDGE_TCP_PORT" "Zenoh TCP"
    _require_port_available "$ZENOH_BIND_HOST" "$BRIDGE_REST_PORT" "Zenoh REST"
fi
if [ "$CONTROL_MODE" = "web" ]; then
    _require_port_available "$CONTROL_HOST" "$CONTROL_PORT" "Web 控制服务"
fi
if [ "$GAZEBO_ENABLED" = "true" ] &&
    [ "$GAZEBO_MASTER_IS_LOCAL" = "true" ]; then
    # Gazebo Classic master 即使 URI 使用 127.0.0.1 也实际监听 IPv4
    # wildcard；按真实绑定范围预检，避免遗漏其他本机网卡上的占用者。
    _require_port_available \
        0.0.0.0 "$GAZEBO_MASTER_PORT" "Gazebo Master"
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
echo "  X Display:  $DISPLAY_STATUS"
if [ "$WITH_PROXY" = "true" ]; then
    ZENOH_PROXY_LABEL="启用"
elif [ "$ZENOH_REQUIRED" = "true" ]; then
    ZENOH_PROXY_LABEL="禁用（桥自带 JSON）"
else
    ZENOH_PROXY_LABEL="完全跳过（键盘模式无需 Zenoh）"
fi
echo "  Zenoh 代理: $ZENOH_PROXY_LABEL"
echo "  ROS 2 域:   $ROS_DOMAIN_ID"
echo "  DDS 本机:   ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY；$CYCLONEDDS_WORKAROUND"
echo "  Zenoh 网络: $ZENOH_NETWORK_LABEL"
if [ "$CONTROL_MODE" = "web" ]; then
    echo "  Web Origins: $CONTROL_ALLOWED_ORIGINS"
fi
echo "  Gazebo URI: $GAZEBO_MASTER_URI"
echo "  Gazebo 检查: $GAZEBO_PREFLIGHT_LABEL"
echo "  Gazebo 模型库: $GAZEBO_MODEL_DATABASE_LABEL"
echo "  末端热切换: $TOOLSET_HOTSWAP_LABEL"
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
    if [ "${#PREFLIGHT_PORT_CONFLICTS[@]}" -gt 0 ]; then
        echo ""
        echo "  ⚠ 配置校验通过，但以下端口已被占用（通常是本项目的运行实例）:"
        for _preflight_conflict in "${PREFLIGHT_PORT_CONFLICTS[@]}"; do
            _preflight_conflict_port="${_preflight_conflict##*|}"
            echo "      - ${_preflight_conflict%%|*}"
            _describe_port_owner "$_preflight_conflict_port"
        done
        echo "    正式启动会自动终止这些旧实例进程并继续；如需同时运行两个实例，"
        echo "    按启动脚本头部的同机隔离示例用环境变量覆盖端口与 ROS_DOMAIN_ID。"
    fi
    echo "  启动配置预检通过（XCZS_PREFLIGHT_ONLY=true，未启动进程）。"
    CLEANUP_DONE="true"
    exit 0
fi

# ── 函数：启动 Zenoh Bridge ────────────────────────────────────────
_assert_managed_processes_running() {
    local index=0
    local status=0
    for ((index=0; index < ${#MANAGED_PIDS[@]}; index++)); do
        if ! _managed_leader_is_running \
            "${MANAGED_PIDS[$index]}" "${MANAGED_PGIDS[$index]}"; then
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

_task_stack_is_ready() {
    local require_cabinet="$1"
    "$PYTHON_BIN" -c '
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
' "$require_cabinet"
}

_toolset_transition_is_active() {
    # During an accepted A/B replacement the supervisor deliberately stops
    # controllers, MoveIt, Nav2 and cabinet runtime nodes for a bounded time.
    # This accepts either the lightweight /robot/toolset/status document or
    # the legacy nested "toolset" object from /health.  The latter can block
    # while Nav2's map service is being replaced, so the runtime monitor uses
    # the former first and must not turn that normal maintenance window into a
    # full-system teardown.
    #
    # A "failed" state is also a recoverable hold rather than a dead stack:
    # the supervisor keeps the Gazebo world alive after an aborted switch or
    # rollback, so the runtime monitor must not tear the world down over it.
    # The process-leader PID check still catches a genuinely dead supervisor.
    "$PYTHON_BIN" -c '
import json
import sys

try:
    document = json.load(sys.stdin)
except (json.JSONDecodeError, AttributeError, TypeError):
    raise SystemExit(1)

if not isinstance(document, dict):
    raise SystemExit(1)
toolset = document.get("toolset")
if not isinstance(toolset, dict):
    toolset = document

raise SystemExit(
    0
    if isinstance(toolset, dict)
    and toolset.get("managed") is True
    and toolset.get("state") in {"starting", "switching", "failed"}
    else 1
)
'
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
        if printf '%s' "$health_body" |
            _task_stack_is_ready "$require_cabinet"; then
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
        "navigation_action_available",
        "navigation_lifecycle_active",
        "navigation_lifecycle_state",
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

_wait_for_gazebo_factory_service() {
    # A live gzserver process is not enough: during first start Gazebo may
    # still fetch model metadata before gazebo_ros_factory advertises
    # /spawn_entity.  Starting the toolset supervisor earlier causes every
    # robot/cabinet spawn to fail, even though Gazebo itself remains alive.
    local deadline=$((SECONDS + ROBOT_READY_TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        _assert_managed_processes_running
        if "$PYTHON_BIN" - <<'PY' 2>/dev/null
import rclpy
from gazebo_msgs.srv import SpawnEntity

rclpy.init(args=None)
node = None
try:
    node = rclpy.create_node("xczs_gazebo_factory_readiness_probe")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    for _ in range(3):
        rclpy.spin_once(node, timeout_sec=0.1)
        if client.service_is_ready():
            raise SystemExit(0)
    raise SystemExit(1)
finally:
    if node is not None:
        node.destroy_node()
    rclpy.shutdown()
PY
        then
            return 0
        fi
        sleep 0.5
    done
    echo "ERROR: Gazebo 工厂服务 /spawn_entity 未在 ${ROBOT_READY_TIMEOUT_SEC}s 内就绪。" >&2
    echo "       gzserver 仍在运行，但尚不能安全生成机器人或控制柜实体。" >&2
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
    local consecutive_health_failures=0
    local health_body=""
    local toolset_status_body=""
    local index=0
    local next_health_check=$SECONDS
    local pid=""
    local status=0
    while true; do
        for ((index=0; index < ${#MANAGED_PIDS[@]}; index++)); do
            pid="${MANAGED_PIDS[$index]}"
            if ! _managed_leader_is_running \
                "$pid" "${MANAGED_PGIDS[$index]}"; then
                if wait "$pid" 2>/dev/null; then
                    status=0
                else
                    status=$?
                fi
                echo "ERROR: ${MANAGED_LABELS[$index]} 意外退出（status=$status）。" >&2
                if [ "$status" -eq 0 ]; then
                    return 1
                fi
                return "$status"
            fi
        done
        # 进程 leader 存活不代表 ROS task stack 仍可执行。持续复用启动
        # readiness 合同，覆盖外部完整栈掉线、Nav2 lifecycle 降级以及
        # 柜体 Action 运行期消失。允许短暂的 3 次失败，避免瞬时抖动误退。
        if [ "$CONTROL_MODE" = "web" ] &&
            (( SECONDS >= next_health_check )); then
            next_health_check=$((SECONDS + 2))
            # /health performs synchronous map, Nav2 and cabinet probes.  A
            # hot switch intentionally drains those services, so consult the
            # dedicated read-only supervisor status before performing that
            # expensive probe.  This endpoint is public like /health because
            # it exposes no control capability and is also used by this local
            # process liveness monitor before any user has authenticated.
            if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ]; then
                toolset_status_body="$(
                    curl -fsS --connect-timeout 1 --max-time 2 \
                        "http://${_READY_URL_HOST}:$CONTROL_PORT/robot/toolset/status" \
                        2>/dev/null || true
                )"
                if printf '%s' "$toolset_status_body" |
                    _toolset_transition_is_active; then
                    consecutive_health_failures=0
                    sleep 0.5
                    continue
                fi
            fi
            health_body="$(
                curl -fsS --connect-timeout 1 --max-time 2 \
                    "http://${_READY_URL_HOST}:$CONTROL_PORT/health" \
                    2>/dev/null || true
            )"
            if printf '%s' "$health_body" |
                _toolset_transition_is_active; then
                consecutive_health_failures=0
            elif printf '%s' "$health_body" |
                _task_stack_is_ready "$CABINET_ACTION_REQUIRED"; then
                consecutive_health_failures=0
            else
                consecutive_health_failures=$((
                    consecutive_health_failures + 1
                ))
                if (( consecutive_health_failures >= 3 )); then
                    echo "ERROR: ROS 任务栈连续 3 次未通过运行期健康检查。" >&2
                    return 1
                fi
            fi
        fi
        sleep 0.5
    done
}

_start_bridge() {
    _start_managed_process BRIDGE_PID "Zenoh Bridge" "$ZENOH_BRIDGE" \
        --config "$ZENOH_BRIDGE_CONFIG" \
        --listen "tcp/$ZENOH_BIND_HOST:$BRIDGE_TCP_PORT" \
        --rest-http-port "$ZENOH_BIND_HOST:$BRIDGE_REST_PORT" \
        "${ZENOH_SCOUTING_ARGS[@]}"
    _wait_for_tcp 127.0.0.1 "$BRIDGE_TCP_PORT" "Zenoh TCP"
    _wait_for_tcp 127.0.0.1 "$BRIDGE_REST_PORT" "Zenoh REST"
    echo "       Zenoh Bridge 已就绪 (PID $BRIDGE_PID)"
    echo "       TCP: tcp/$ZENOH_BIND_HOST:$BRIDGE_TCP_PORT"
    echo "       REST: http://$ZENOH_BIND_HOST:$BRIDGE_REST_PORT"
}

# Zenoh 延后到 ROS/Gazebo publishers 稳定后再启动，以降低启动期端点
# 抖动。zenoh_bridge.json5 还会阻止相机原始大消息被桥回注为第二个 DDS
# writer；Web 传感器进程直接订阅 Gazebo 的 ROS 2 camera writer。

_start_web_control() {
    local toolset_supervisor_args=()
    # 迁移到 FastAPI 后，原先独立的 SSE 桥、传感器流、HTTP 文件服务器与
    # Web 控制服务合并为单个 uvicorn 进程（默认 CONTROL_PORT 8090）。
    # 本地仿真与 Zenoh 路由先稳定，再创建长期运行的传感器 readers。
    if [ "$CONTROL_MODE" != "web" ]; then
        return
    fi
    if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ]; then
        toolset_supervisor_args=(--toolset-supervisor)
    fi
    echo "       启动统一 Web 控制服务 (port $CONTROL_PORT)..."
    cd "$JIANG_DIR"
    _start_managed_process CONTROL_PID "Web 控制服务" \
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
        --scenes-config "$SCENES_CONFIG" \
        --cabinet-xacro "$CABINET_XACRO_PATH" \
        --robot-entity "$ROBOT_NAME" \
        --toolset "$TOOLSET" \
        "${toolset_supervisor_args[@]}" \
        --initial-scene "$SCENE" \
        --zenoh "tcp/127.0.0.1:$BRIDGE_TCP_PORT" \
        "${CONTROL_ORIGIN_ARGS[@]}"
    case "$CONTROL_HOST" in
        0.0.0.0) _READY_HOST="127.0.0.1" ;;
        ::) _READY_HOST="::1" ;;
        *) _READY_HOST="$CONTROL_HOST" ;;
    esac
    case "$_READY_HOST" in
        *:*) _READY_URL_HOST="[$_READY_HOST]" ;;
        *) _READY_URL_HOST="$_READY_HOST" ;;
    esac
    _wait_for_http "http://${_READY_URL_HOST}:$CONTROL_PORT/health" "Web 控制服务"
    # 只有监听 wildcard 时才展示局域网 IP；绑定具体地址时原样展示。
    case "$CONTROL_HOST" in
        0.0.0.0)
            _LAN_IP="$(hostname -I 2>/dev/null | awk '
                { for (i = 1; i <= NF; i++)
                    if ($i !~ /:/) { print $i; exit } }
            ' || true)"
            _SHOW_HOST="${_LAN_IP:-127.0.0.1}"
            ;;
        ::)
            _LAN_IP="$(hostname -I 2>/dev/null | awk '
                { for (i = 1; i <= NF; i++)
                    if ($i ~ /:/) { print $i; exit } }
            ' || true)"
            _SHOW_HOST="${_LAN_IP:-::1}"
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
}

if [ "$CONTROL_MODE" = "web" ]; then
    echo "[1-3/6] Zenoh 与 Web 将在 ROS/Gazebo 稳定后启动"
else
    echo "[1-3/6] 按键模式按需延后启动 Zenoh 代理；不启动 Web"
fi

# ── 6. 启动 Gazebo + 机器人 ───────────────────────────────────────
echo "[6/6] 启动 Gazebo + XCZS 机器人..."

TELEOP_ENABLED="false"
if [ "$CONTROL_MODE" = "keyboard" ]; then
    TELEOP_ENABLED="true"
fi

# This list is owned by the robot child in managed toolset mode.  It contains
# only settings that stay stable across A/B; the supervisor owns topology,
# process lifecycle and all spawn switches.  ``{toolset}`` is expanded by the
# supervisor immediately before every child launch.
ROBOT_CHILD_STATIC_LAUNCH_ARGS=(
    "use_sim_time:=$USE_SIM_TIME"
    "moveit:=$MOVEIT_ENABLED"
    "nav2:=$NAV2_ENABLED"
    "robot_name:=$ROBOT_NAME"
    "robot_xacro:=$ROBOT_XACRO_PATH"
    "nav2_launch:=$NAV2_LAUNCH_PATH"
    "nav2_params_file:=$NAV2_PARAMS_FILE"
    "robot_control:=$ROBOT_CONTROL_PATH"
    "cabinet_instances:=$CABINET_INSTANCES_PATH"
    "cabinet_controls:=$CABINET_CONTROLS_PATH"
    "cabinet_scene:=$CABINET_SCENE_PATH"
    "cabinet_pose:=$CABINET_POSE_PATH"
    "cabinet_robot_adapter:=$CABINET_ROBOT_ADAPTER_PATH"
    "cabinet_xacro:=$CABINET_XACRO_PATH"
    "scene:=$SCENE"
    "scenes_config:=$SCENES_CONFIG"
    "spawn_z:=$SPAWN_Z"
    "cabinet_pose_source:=$CABINET_POSE_SOURCE"
)
if [ "$MOVEIT_ENABLED" = "true" ]; then
    ROBOT_CHILD_STATIC_LAUNCH_ARGS+=(
        "moveit_config_package:=$MOVEIT_CONFIG_PACKAGE"
        "moveit_srdf:=$MOVEIT_SRDF_TEMPLATE"
        "moveit_kinematics:=$MOVEIT_KINEMATICS_PATH"
        "moveit_joint_limits:=$MOVEIT_JOINT_LIMITS_TEMPLATE"
        "moveit_controllers:=$MOVEIT_CONTROLLERS_TEMPLATE"
        "moveit_rviz_config:=$MOVEIT_RVIZ_CONFIG_PATH"
        "moveit_launch:=$MOVEIT_LAUNCH_PATH"
    )
fi
# nav2_map 是可选覆盖：默认留空由 scenes.yaml 决定实际地图。空值不能作为
# launch 参数传递（会生成非法的 ``nav2_map:=``），仅在显式设置时追加。
if [ -n "$NAV2_MAP_PATH" ]; then
    ROBOT_CHILD_STATIC_LAUNCH_ARGS+=("nav2_map:=$NAV2_MAP_PATH")
fi

_create_launch_runtime_directory
_configure_gazebo_log_directory
if [ "$GAZEBO_ENABLED" = "true" ]; then
    if [ "$GAZEBO_LOG_DIRECTORY" = "$LAUNCH_RUNTIME_DIRECTORY/gazebo_logs" ]; then
        echo "       Gazebo 日志已重定向到本次临时目录（退出后自动清理）"
    else
        echo "       保留用户指定的 Gazebo 日志目录: $GAZEBO_LOG_DIRECTORY"
    fi
fi

if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ]; then
    # The world owner deliberately has no robot/cabinet runtime nodes.  It
    # starts gzserver, gzclient and the static scene floor once; the supervisor
    # below owns the replaceable robot entity plus toolset-specific control,
    # MoveIt, Nav2 and cabinet runtime processes.
    WORLD_LAUNCH_ARGS=(
        "gui:=$GAZEBO_GUI"
        "gazebo:=true"
        "robot_bringup:=false"
        "use_sim_time:=$USE_SIM_TIME"
        "control_gui:=false"
        "teleop:=false"
        "moveit:=false"
        "moveit_rviz:=false"
        "nav2:=false"
        "nav2_rviz:=false"
        "world:=$SIMULATION_WORLD_PATH"
        "robot_name:=$ROBOT_NAME"
        "cabinet_robot_adapter:=$CABINET_ROBOT_ADAPTER_PATH"
        "cabinet_bringup:=false"
        "spawn_cabinet:=false"
        "scene:=$SCENE"
        "scenes_config:=$SCENES_CONFIG"
        "spawn_z:=$SPAWN_Z"
        "cabinet_pose_source:=$CABINET_POSE_SOURCE"
    )
    echo "       启动常驻 Gazebo 世界（不含机器人与柜体运行节点）..."
    _start_managed_process WORLD_LAUNCH_PID "Gazebo 世界 launch" \
        ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "${WORLD_LAUNCH_ARGS[@]}"
    # Fixed waiting only catches immediate child-process failures.  The
    # service gate below owns actual Gazebo factory readiness.
    _wait_for_stable_processes 50
    _wait_for_gazebo_factory_service

    TOOLSET_SUPERVISOR_ARGS=(
        --toolset "$TOOLSET"
        --robot-name "$ROBOT_NAME"
        --initial-spawn-cabinet "$SPAWN_CABINET"
        --cabinet-bringup "$CABINET_BRINGUP"
        --ready-timeout "$ROBOT_READY_TIMEOUT_SEC"
    )
    if [ "$NAV2_ENABLED" = "true" ]; then
        TOOLSET_SUPERVISOR_ARGS+=(--require-nav2)
    fi
    if [ "$CABINET_ACTION_REQUIRED" = "true" ]; then
        TOOLSET_SUPERVISOR_ARGS+=(--require-cabinet-action)
        for _cabinet_operation_action in "${CABINET_OPERATION_ACTIONS[@]}"; do
            TOOLSET_SUPERVISOR_ARGS+=(
                --cabinet-action "$_cabinet_operation_action"
            )
        done
    fi
    for _robot_child_launch_arg in "${ROBOT_CHILD_STATIC_LAUNCH_ARGS[@]}"; do
        TOOLSET_SUPERVISOR_ARGS+=(
            --launch-arg "$_robot_child_launch_arg"
        )
    done
    echo "       启动末端工具集监督器（首次套装 $TOOLSET）..."
    _start_managed_process TOOLSET_SUPERVISOR_PID "末端工具集监督器" \
        "$PYTHON_BIN" "$TOOLSET_SUPERVISOR_SCRIPT" \
        "${TOOLSET_SUPERVISOR_ARGS[@]}"
    _wait_for_stable_processes 50
else
    # Legacy/special-mode path: keep the previous single launch exactly as a
    # unit.  It covers keyboard use, external robot stacks, no-Gazebo runs and
    # deployments that explicitly disable XCZS_TOOLSET_HOTSWAP.
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
        "toolset:=$TOOLSET"
        "moveit_config_package:=$MOVEIT_CONFIG_PACKAGE"
        "moveit_srdf:=$MOVEIT_SRDF_PATH"
        "moveit_kinematics:=$MOVEIT_KINEMATICS_PATH"
        "moveit_joint_limits:=$MOVEIT_JOINT_LIMITS_PATH"
        "moveit_controllers:=$MOVEIT_CONTROLLERS_PATH"
        "moveit_rviz_config:=$MOVEIT_RVIZ_CONFIG_PATH"
        "moveit_launch:=$MOVEIT_LAUNCH_PATH"
        "nav2_launch:=$NAV2_LAUNCH_PATH"
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
        "scene:=$SCENE"
        "scenes_config:=$SCENES_CONFIG"
        "spawn_z:=$SPAWN_Z"
        "cabinet_pose_source:=$CABINET_POSE_SOURCE"
    )
    if [ -n "$NAV2_MAP_PATH" ]; then
        LAUNCH_ARGS+=("nav2_map:=$NAV2_MAP_PATH")
    fi
    _start_managed_process LAUNCH_PID "ROS 2 launch" \
        ros2 launch xczs_inspection_robot_control inspection_robot.launch.py \
        "${LAUNCH_ARGS[@]}"
    if [ "$LOCAL_LAUNCH_PERSISTENT" = "true" ]; then
        # Each check is 200 ms.  Ten real seconds gives Gazebo sensor plugins
        # and the Zenoh DDS routes time to finish their initial endpoint churn
        # before the Web process creates long-lived image readers.
        _wait_for_stable_processes 50
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
fi

# ROS/Gazebo publishers 已完成首轮发现后再创建 Zenoh 路由；等桥的 DDS
# writers 稳定后，最后创建 Web 传感器 readers。外部栈模式的本地 launch
# 已完成配置校验，此时同样可安全连接既有端点。
echo "[1/6] Zenoh Bridge..."
if [ "$ZENOH_REQUIRED" = "true" ]; then
    _start_bridge
    _wait_for_stable_processes 25
else
    echo "       跳过（键盘模式未启用 CDR→JSON 代理）"
fi

if [ "$WITH_PROXY" = "true" ]; then
    echo "[2/6] 启动 XCZS Zenoh 代理..."
    cd "$JIANG_DIR"
    _start_managed_process PROXY_PID "XCZS Zenoh 代理" \
        "$PYTHON_BIN" run_xczs_proxy.py \
        --port "$BRIDGE_TCP_PORT" \
        --control-port 0
    _wait_for_stable_processes 5
    echo "       代理已启动 (PID $PROXY_PID)"
else
    if [ "$ZENOH_REQUIRED" = "true" ]; then
        echo "[2/6] 跳过 Zenoh 代理（使用桥自带 JSON 转发）"
    else
        echo "[2/6] 跳过 Zenoh 代理（键盘模式无需 Zenoh）"
    fi
    PROXY_PID=""
fi

_start_web_control

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

if [ "$CONTROL_MODE" = "web" ]; then
    _wait_for_task_stack \
        "http://${_READY_URL_HOST}:$CONTROL_PORT/health" "$CABINET_ACTION_REQUIRED"
    if [ "$CABINET_ACTION_REQUIRED" = "true" ]; then
        echo "       ROS 任务栈已就绪（Nav2 和柜体 Action 可用）"
    else
        echo "       ROS 任务栈已就绪（Nav2 可用）"
    fi
fi
if [ "$TOOLSET_HOTSWAP_ACTIVE" = "true" ]; then
    echo "       Gazebo 世界 launch 已启动 (PID $WORLD_LAUNCH_PID)"
    echo "       末端工具集监督器已启动 (PID $TOOLSET_SUPERVISOR_PID)"
elif [ -n "$LAUNCH_PID" ]; then
    echo "       ROS 2 launch 已启动 (PID $LAUNCH_PID)"
else
    echo "       本地 ROS launch 配置已通过，正在使用外部完整栈"
fi
echo "       持续监控 Zenoh、Web、Gazebo 世界和机器人运行栈；任一受管进程异常退出将关闭本次启动。"

if _monitor_managed_processes; then
    exit 0
else
    runtime_status=$?
    exit "$runtime_status"
fi
