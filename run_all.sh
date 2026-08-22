#!/bin/bash
# ============================================================================
# XCZS 巡操机器人仿真系统 — 一键启动
#
# 用法:
#   ./run_all.sh                  # 统一 Web 控制（Gazebo + Nav2 + 控制柜任务）
#   ./run_all.sh --web            # 兼容写法，与无参数启动相同
#   ./run_all.sh --with-proxy     # 附加 CDR→JSON 代理（精细化数据处理）
#   同机隔离启动（端口应避开其他实例）:
#     ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 BRIDGE_TCP_PORT=17447 \
#       BRIDGE_REST_PORT=18000 CONTROL_HOST=127.0.0.1 CONTROL_PORT=8090 \
#       XCZS_CONTROL_ORIGINS=http://localhost:8090,http://127.0.0.1:8090 \
#       ./run_all.sh --web
#
#
# 启动后（默认监听所有网卡 :8090，其他电脑可访问）:
#   监控面板: http://<服务器IP>:8090/monitor.html
#   API 文档:  http://<服务器IP>:8090/docs
#   SSE 数据:  http://<服务器IP>:8090/sse/<key>
#   传感器流:  http://<服务器IP>:8090/camera.mjpg
# ============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 转发所有参数给 jiang/start_xczs_bridge.sh。退出码 42 = 末端工具套装
# 切换标记触发，需要重读资产库选择并以新套装重新拉起整个栈。
# 注意：不能用 `if cmd; then ...; fi` 捕获退出码——条件为假且无 else 时
# if 语句自身返回 0，会吞掉 42。必须用 `cmd || status=$?` 直接取码。
while true; do
    status=0
    "$SCRIPT_DIR/jiang/start_xczs_bridge.sh" "$@" || status=$?
    if [ "$status" -eq 0 ]; then
        exit 0
    fi
    if [ "$status" -eq 42 ]; then
        echo "工具套装已切换，自动重启机器人栈。"
        # Web 切换以持久化选择为唯一事实来源：清除外部导出的 TOOLSET，
        # 否则 env 优先级会吞掉本次切换（重启后栈仍停在旧套装）。
        unset TOOLSET 2>/dev/null || true
        continue
    fi
    exit "$status"
done
