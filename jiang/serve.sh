#!/bin/bash
# 启动 HTTP 静态文件服务器 (用于 monitor.html)
# 访问地址: http://localhost:8080/monitor.html

PORT=${1:-8080}
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Serving $DIR on http://localhost:$PORT"
echo "打开 http://localhost:$PORT/monitor.html"
echo "Ctrl+C 停止"
echo ""

python3 -m http.server "$PORT" --directory "$DIR"
