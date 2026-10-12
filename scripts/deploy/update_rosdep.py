"""Build-only rosdep update: IPv4, longer requests, bounded retries.

Run each attempt in a child so rosdep module/socket changes never affect the
runtime services. A partially refreshed cache never counts as success.
"""
import runpy
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path


def update_once():
    original_getaddrinfo = socket.getaddrinfo
    original_urlopen = urllib.request.urlopen

    def ipv4_addresses(host, port, family=0, type=0, proto=0, flags=0):
        return original_getaddrinfo(host, port, socket.AF_INET, type, proto, flags)

    def open_with_timeout(url, data=None, timeout=60, **kwargs):
        return original_urlopen(url, data=data, timeout=60, **kwargs)

    socket.getaddrinfo = ipv4_addresses
    urllib.request.urlopen = open_with_timeout
    # Import rosdep only after patching urllib; its modules import urlopen.
    sys.argv = ['rosdep', 'update', '--rosdistro', 'humble']
    runpy.run_path('/usr/bin/rosdep', run_name='__main__')


def main():
    for attempt in range(1, 4):
        print(f'rosdep 更新：第 {attempt}/3 次（IPv4，单请求 60 秒，单次最长 300 秒）', flush=True)
        try:
            result = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), '--once'],
                timeout=300, check=False,
            )
            if result.returncode == 0:
                return 0
        except subprocess.TimeoutExpired:
            print('本次 rosdep 更新超时。', flush=True)
        if attempt < 3:
            time.sleep(5 * attempt)
    print('rosdep 更新连续失败，停止构建；未跳过依赖检查。', file=sys.stderr)
    return 1


if __name__ == '__main__':
    if sys.argv[1:] == ['--once']:
        update_once()
    else:
        raise SystemExit(main())
