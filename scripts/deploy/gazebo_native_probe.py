#!/usr/bin/env python3
"""检查最小 Gazebo 世界的实际原生统计和 CPU 雷达样本。"""
from collections import Counter
import os
from pathlib import Path
import re
import signal
import socket
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
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    env.update(GAZEBO_MASTER_URI=f"http://127.0.0.1:{port}",
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
            if sockets.returncode == 0 and re.search(rf":{port}(?:\s|$)", sockets.stdout):
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
