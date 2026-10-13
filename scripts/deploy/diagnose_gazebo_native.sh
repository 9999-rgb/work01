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
' <<'PY' | relay
from collections import Counter
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

WORLD = """<sdf version="1.7"><world name="native_probe">
  <physics name="probe_ode" type="ode"><max_step_size>0.01</max_step_size>
    <real_time_update_rate>100</real_time_update_rate></physics>
  <model name="ray_probe"><static>true</static><link name="sensor_link">
    <pose>0 0 1 0 0 0</pose><sensor name="ray" type="ray">
      <always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>
      <ray><scan><horizontal><samples>8</samples><resolution>1</resolution>
        <min_angle>-1</min_angle><max_angle>1</max_angle></horizontal></scan>
        <range><min>0.1</min><max>5</max><resolution>0.01</resolution></range>
      </ray></sensor></link></model>
</world></sdf>"""


def probe(label, args, env, timeout=8):
    try:
        result = subprocess.run(["stdbuf", "-oL", "-eL", *args], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=timeout)
        code, output = result.returncode, result.stdout
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        code = 124
    except OSError as error:
        code, output = 127, str(error)
    print(f"=== {label}：退出码 {code} ===", flush=True)
    print("\n".join(output.splitlines()[:8]) or "无输出", flush=True)
    return code, output


passed = False
server = None
with tempfile.TemporaryDirectory(prefix="xczs-native-probe-") as root:
    directory = Path(root)
    world = directory / "probe.world"
    world.write_text(WORLD)
    log = directory / "gzserver.log"
    env = dict(os.environ)
    env.pop("DISPLAY", None)
    env.pop("XAUTHORITY", None)
    env.update(GAZEBO_MASTER_URI="http://127.0.0.1:11492",
               GAZEBO_MODEL_DATABASE_URI="", GAZEBO_LOG_PATH=str(directory / "logs"))
    try:
        with log.open("w") as stream:
            server = subprocess.Popen(["gzserver", "--verbose", str(world)], env=env,
                                      stdout=stream, stderr=subprocess.STDOUT,
                                      start_new_session=True)
        deadline = time.monotonic() + 15
        while True:
            if server.poll() is not None:
                raise RuntimeError(f"gzserver 提前退出：{server.returncode}")
            sockets = subprocess.run(["ss", "-lntH"], stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, timeout=2)
            if sockets.returncode == 0 and re.search(r":11492(?:\s|$)", sockets.stdout):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("Gazebo master 未在 15 秒内监听")
            time.sleep(0.25)
        time.sleep(2)
        stats_code, stats = probe("原生世界统计", ["gz", "stats", "--world-name",
                                 "native_probe", "--duration", "3"], env)
        ray_code, ray = probe("原生雷达频率", ["gz", "topic", "--hz",
                              "/gazebo/native_probe/ray_probe/sensor_link/ray/scan",
                              "--duration", "3"], env)
        passed = (stats_code == ray_code == 0 and server.poll() is None
                  and re.search(r"Factor\[[\d.]+\].*SimTime\[[\d.]+\]", stats) is not None
                  and any(float(value) > 0 for value in re.findall(r"Hz:\s*([\d.]+)", ray)))
        print("结果：原生世界统计与雷达均有实际数据。" if passed
              else "结果：原生对照失败，不能认定传感器工作正常。", flush=True)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"原生对照失败：{error}", flush=True)
    finally:
        if not passed and server is not None and server.poll() is None:
            print("=== 失败时的主线程、线程等待状态及核心库 ===", flush=True)
            proc = Path("/proc") / str(server.pid)
            try:
                print("主线程等待位置：" + (proc / "wchan").read_text().strip(), flush=True)
                waiting = Counter()
                for task in (proc / "task").iterdir():
                    try:
                        waiting[(task / "wchan").read_text().strip()] += 1
                    except OSError:
                        continue
                for value, count in sorted(waiting.items()):
                    print(f"线程等待 {value}: {count}", flush=True)
                libraries = sorted({line.split()[-1] for line in (proc / "maps").read_text().splitlines()
                                    if re.search(r"/(?:libgazebo[^/]*|libtbb[^/]*|libboost_system[^/]*|libprotobuf[^/]*)\.so", line)})
                for library in libraries:
                    print(library, flush=True)
            except OSError as error:
                print(f"进程状态读取失败：{error}", flush=True)
            probe("Gazebo 控制连接", ["ss", "-ntp"], env, timeout=4)
            for topic in ("/gazebo/native_probe/world_stats", "/gazebo/native_probe/pose/info"):
                probe("原生发布注册 " + topic, ["gz", "topic", "--info", topic], env, timeout=4)
            if os.environ.get("XCZS_NATIVE_STACK") == "1":
                done_file = directory / "stack.done"
                print("__XCZS_NATIVE_STACK__ " + str(done_file), flush=True)
                deadline = time.monotonic() + 30
                while not done_file.exists() and time.monotonic() < deadline:
                    time.sleep(0.2)
        if not passed and log.exists():
            print("=== gzserver 最后 20 行 ===", flush=True)
            print("\n".join(log.read_text(errors="replace").splitlines()[-20:]), flush=True)
        if server is not None and server.poll() is None:
            try:
                os.killpg(server.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(server.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                server.wait(timeout=3)
raise SystemExit(0 if passed else 1)
PY
