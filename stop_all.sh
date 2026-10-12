#!/usr/bin/env bash
# 一键停止本项目的 Docker 仿真容器及本机仿真；--dry-run 仅列出目标。
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
command -v python3 >/dev/null || { echo '需要 python3。' >&2; exit 1; }
exec python3 - "$ROOT" "$@" <<'PY'
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

root = Path(sys.argv[1])
args = sys.argv[2:]
if args not in ([], ['--dry-run']):
    raise SystemExit('用法：bash stop_all.sh [--dry-run]')
dry = bool(args)
wait_seconds = int(os.environ.get('XCZS_STOP_TIMEOUT', '20'))
if not 1 <= wait_seconds <= 120:
    raise SystemExit('XCZS_STOP_TIMEOUT 必须为 1～120 秒')
errors = []


def run_docker(*args):
    return subprocess.run(['docker', '--host', 'unix:///var/run/docker.sock', *args],
                          capture_output=True, text=True, timeout=60)


# 手动 docker stop 会抑制 unless-stopped 重启，不删除容器、镜像或数据卷。
if Path('/var/run/docker.sock').exists():
    try:
        result = run_docker('ps', '-aq', '--filter', f'label=org.xczs.project={root}')
        if result.returncode:
            errors.append('无法查询 Docker；请检查权限（必要时 sudo bash stop_all.sh）。')
        else:
            for cid in result.stdout.split():
                print(f'{"将停止" if dry else "停止"}项目容器 {cid}', flush=True)
                if not dry:
                    result = run_docker('stop', '--time', '30', cid)
                    if result.returncode:
                        errors.append(f'停止容器 {cid} 失败。')
                    else:
                        result = run_docker('inspect', '-f', '{{.State.Running}}', cid)
                        if result.returncode or result.stdout.strip() != 'false':
                            errors.append(f'容器 {cid} 未确认停止。')
    except (OSError, subprocess.TimeoutExpired):
        errors.append('Docker 命令不可用或超时；未确认容器停止。')


def snapshot():
    items = {}
    for folder in Path('/proc').iterdir():
        if not folder.name.isdigit():
            continue
        try:
            stat = (folder / 'stat').read_text().rsplit(')', 1)[1].split()
            argv = (folder / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            items[int(folder.name)] = (int(stat[1]), stat[19], stat[0], argv,
                                       Path(os.readlink(folder / 'cwd')))
        except (OSError, ValueError, IndexError):
            continue
    return items


initial = snapshot()
excluded = {os.getpid(), 1}
pid = os.getpid()
while pid in initial and initial[pid][0] not in excluded:
    pid = initial[pid][0]
    excluded.add(pid)


def belongs(info):
    _, _, _, argv, cwd = info
    # 按独立 argv 项识别入口/已编译节点，绝不匹配 shell -c 的整段文本。
    executable = Path(argv[0]).name if argv else ''
    command_args = argv[:2] if executable in {'bash', 'sh', 'python', 'python3'} else argv[:1]
    for arg in command_args:
        if not arg or arg.startswith('-') or '\n' in arg:
            continue
        candidate = Path(os.path.normpath(str(cwd / arg)))
        if candidate in (root / 'run_all.sh', root / 'jiang/start_xczs_bridge.sh'):
            return True
        try:
            relative = candidate.relative_to(root / 'install')
        except ValueError:
            continue
        if len(relative.parts) >= 3 and relative.parts[0].startswith('xczs_inspection_robot_') and relative.parts[1] == 'lib':
            return True
    # 识别已脱离启动进程、但仍保留工作空间环境的 Gazebo/ROS 服务。
    names = {Path(arg).name for arg in argv[:3] if arg}
    known = {'gzserver', 'gzclient', 'rviz2', 'ros2', 'move_group',
             'robot_state_publisher', 'controller_manager', 'zenoh-bridge-ros2dds', 'uvicorn'}
    if names & known:
        return cwd == root or cwd == root / 'jiang'
    return False


targets = {}

def collect(items):
    for pid, info in items.items():
        if pid not in excluded and info[2] != 'Z' and belongs(info):
            targets[pid] = info[1]
    # 捕获启动器的所有子孙，包括 guardian、Gazebo、相机与 Web 服务。
    changed = True
    while changed:
        changed = False
        for pid, info in items.items():
            if pid not in targets and pid not in excluded and info[0] in targets:
                parent = items.get(info[0])
                if parent and parent[1] == targets[info[0]]:
                    targets[pid] = info[1]
                    changed = True


def alive(pid, identity):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return stat[19] == identity and stat[0] != 'Z'
    except (OSError, IndexError):
        return False


def send(pid, identity, sig):
    if alive(pid, identity):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError:
            errors.append(f'无权停止 PID {pid}；请使用 sudo bash stop_all.sh。')


collect(initial)
for pid in sorted(targets):
    print(f'{"将停止" if dry else "停止"}本机项目进程 PID={pid}', flush=True)
if not dry:
    signaled = set()
    deadline = time.monotonic() + wait_seconds
    while True:
        collect(snapshot())
        for pid, identity in targets.items():
            if (pid, identity) not in signaled:
                send(pid, identity, signal.SIGTERM)
                signaled.add((pid, identity))
        if not any(alive(pid, identity) for pid, identity in targets.items()):
            break
        if time.monotonic() >= deadline:
            for pid, identity in targets.items():
                send(pid, identity, signal.SIGKILL)
            time.sleep(1)
            break
        time.sleep(0.2)
    collect(snapshot())
    remaining = [str(pid) for pid, identity in targets.items() if alive(pid, identity)]
    if remaining:
        errors.append('仍有项目进程：' + ', '.join(remaining))
if errors:
    for error in sorted(set(errors)):
        print(error, file=sys.stderr)
    raise SystemExit(1)
print('预览完成，未停止任何进程。' if dry else '停止检查完成：未发现仍运行的本项目仿真进程。')
PY
