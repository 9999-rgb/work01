#!/usr/bin/env bash
# 独立临时容器检查 Gazebo 原生 world_stats 与 ray；不操作现有仿真。
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NAME=xczs-simulation
PROBE_NAME="xczs-native-probe-$$"
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
echo '运行独立原生雷达对照，预计约 30–45 秒；当前仿真保持运行。'
docker_local run --rm -i --name "$PROBE_NAME" --cidfile "$CID_FILE" --entrypoint bash "$IMAGE" -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /opt/work01/install/setup.bash
    exec python3 -u -
' <<'PY'
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


def probe(label, args, env):
    try:
        result = subprocess.run(["stdbuf", "-oL", "-eL", *args], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=8)
        code, output = result.returncode, result.stdout
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        code = 124
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
