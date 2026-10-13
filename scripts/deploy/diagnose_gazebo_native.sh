#!/usr/bin/env bash
# 独立临时容器检查 Gazebo 原生 world_stats 与 ray；不操作现有仿真。
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NAME=xczs-simulation
PROBE_NAME="xczs-native-probe-$$"
STACK=0
case "${1:-}" in
    '') ;;
    --stack) STACK=1 ;;
    *) echo '用法：bash scripts/deploy/diagnose_gazebo_native.sh [--stack]' >&2; exit 2 ;;
esac
(( $# <= 1 )) || { echo '参数过多。' >&2; exit 2; }
[[ -f "$ROOT/scripts/deploy/gazebo_native_probe.py" ]] || { echo '缺少原生数据探针，请拉取完整代码。' >&2; exit 1; }
GDB_BIN=''
if (( STACK )); then
    GDB_BIN="$(command -v gdb || true)"
    [[ -n "$GDB_BIN" ]] || { echo '宿主机缺少 gdb，无法读取线程栈。' >&2; exit 1; }
fi
docker_local() { docker --host unix:///var/run/docker.sock "$@"; }
OWNER="$(docker_local inspect -f '{{index .Config.Labels "org.xczs.project"}}' "$NAME")"
[[ "$OWNER" == "$ROOT" ]] || { echo '容器不属于当前项目目录，停止诊断。' >&2; exit 1; }
IMAGE="$(docker_local inspect -f '{{.Image}}' "$NAME")"
[[ "$IMAGE" =~ ^sha256:[0-9a-f]{64}$ ]] || { echo '无法取得当前容器的镜像 ID。' >&2; exit 1; }
CID_DIR="$(mktemp -d -t xczs-native-probe.XXXXXX)"
CID_FILE="$CID_DIR/container.id"
cleanup() {
    local probe_id
    if [[ -s "$CID_FILE" ]]; then
        probe_id="$(<"$CID_FILE")"
        if [[ "$probe_id" =~ ^[0-9a-f]{64}$ ]]; then
            docker_local rm -f "$probe_id" >/dev/null 2>&1 || true
        fi
    fi
    rm -f -- "$CID_FILE"
    rmdir -- "$CID_DIR" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
read_stack() {
    local done_file="$1" probe_id processes host_pid thread_dirs candidate
    [[ "$done_file" =~ ^/tmp/xczs-native-probe-[a-zA-Z0-9_-]+/stack\.done$ ]] || return 1
    [[ -s "$CID_FILE" ]] || return 1
    probe_id="$(<"$CID_FILE")"
    [[ "$probe_id" =~ ^[0-9a-f]{64}$ ]] || return 1
    if processes="$(docker_local top "$probe_id" -eo pid,comm 2>/dev/null)"; then
        host_pid="$(awk '$2 == "gzserver" {print $1; exit}' <<< "$processes")"
        if [[ "$host_pid" =~ ^[0-9]+$ ]]; then
            thread_dirs="/proc/$host_pid/root/lib:/proc/$host_pid/root/usr/lib"
            for candidate in /proc/"$host_pid"/root/{lib,usr/lib}/*/libthread_db.so.1; do
                [[ ! -f "$candidate" ]] || thread_dirs+=":${candidate%/*}"
            done
            echo '=== 临时 gzserver 的主线程及各线程调用栈 ==='
            if ! timeout --kill-after=2 20 "$GDB_BIN" -nx -batch \
                -iex "set sysroot /proc/$host_pid/root" \
                -iex "set libthread-db-search-path $thread_dirs" \
                -iex "set auto-load safe-path /proc/$host_pid/root/lib:/proc/$host_pid/root/usr/lib" \
                -iex 'set pagination off' -iex 'set print frame-arguments none' \
                -p "$host_pid" -ex 'thread 1' -ex 'bt 32' \
                -ex 'thread apply all bt 6' -ex 'detach' -ex 'quit' 2>&1; then
                echo '线程栈读取失败或超时；保留下面的原生诊断信息。'
            fi
        else
            echo '临时容器中没有找到 gzserver，无法读取线程栈。'
        fi
    else
        echo '无法读取临时容器进程列表。'
    fi
    docker_local exec "$probe_id" touch "$done_file" >/dev/null 2>&1 || true
}
relay() {
    local line
    while IFS= read -r line; do
        if (( STACK )) && [[ "$line" == '__XCZS_NATIVE_STACK__ '* ]]; then
            read_stack "${line#__XCZS_NATIVE_STACK__ }" || echo '线程栈请求无效。'
        else
            printf '%s\n' "$line"
        fi
    done
}
echo '运行独立原生雷达对照，预计约 30–60 秒；当前仿真保持运行。'
(( ! STACK )) || echo '失败时自动读取线程栈，最多额外等待约 30 秒。'
docker_local run --rm -i --name "$PROBE_NAME" --cidfile "$CID_FILE" \
    --env XCZS_NATIVE_STACK="$STACK" --entrypoint bash "$IMAGE" -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/work01/install/setup.bash
    exec python3 -u -
' < "$ROOT/scripts/deploy/gazebo_native_probe.py" | relay
