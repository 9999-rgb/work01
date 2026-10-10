"""同时检查反向代理、机器人就绪状态与页面服务地址。"""
import json
from urllib.request import urlopen

with urlopen('http://127.0.0.1/api/robot/toolset/status', timeout=2) as response:
    status = json.load(response)
if not (status.get('ready') and status.get('gateway_synced')):
    raise SystemExit('机器人尚未就绪')
with urlopen('http://127.0.0.1/api/health', timeout=2) as response:
    if response.status != 200:
        raise SystemExit('API 未就绪')
with urlopen('http://127.0.0.1/monitor.html', timeout=2) as response:
    html = response.read().decode()
if html.count('window.location.origin + "/api"') < 3:
    raise SystemExit('页面代理地址未完成替换')
