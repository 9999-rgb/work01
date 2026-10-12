#!/usr/bin/env bash
set -Eeuo pipefail
cd /opt/work01
nginx -t
nginx -g 'daemon off;' &
NGINX_PID=$!
SIM_PID=''
cleanup() {
    trap - EXIT TERM INT
    [[ -z "$SIM_PID" ]] || kill -TERM "$SIM_PID" 2>/dev/null || true
    kill -QUIT "$NGINX_PID" 2>/dev/null || true
    wait || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
if [[ ${XCZS_GUI:-0} == 1 ]]; then
    xdpyinfo >/dev/null 2>&1 || { echo '无法访问桌面显示，请在桌面终端重新部署，或 GUI=0 ./start。' >&2; exit 1; }
    ./run_all.sh --web &
else
    xvfb-run -a -s '-screen 0 1280x720x24 -nolisten tcp' bash -c '
        echo "正在检查虚拟显示 OpenGL（最长 20 秒）……"
        if ! timeout 20 glxinfo -B; then
            echo "ERROR: 虚拟显示无法建立 OpenGL 上下文，仿真尚未启动。" >&2
            exit 1
        fi
        # Xvfb 已能实际创建 GLX 上下文，无需实体桌面的冷启动保护等待。
        export XCZS_DISPLAY_SETTLE_SECONDS="${XCZS_DISPLAY_SETTLE_SECONDS:-0}"
        export XCZS_DISPLAY_SETTLE_MAX_WAIT="${XCZS_DISPLAY_SETTLE_MAX_WAIT:-0}"
        exec ./run_all.sh --web
    ' &
fi
SIM_PID=$!
# 任一主服务退出时整个容器退出，由 Docker 重启，避免留下半运行状态。
set +e
wait -n "$NGINX_PID" "$SIM_PID"
STATUS=$?
set -e
((STATUS != 0)) || STATUS=1
exit "$STATUS"
