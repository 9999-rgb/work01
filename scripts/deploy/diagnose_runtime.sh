#!/usr/bin/env bash
# 麒麟运行诊断：重启本项目容器一次，从同一轮启动采集 80 秒 ROS 数据。
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NAME=xczs-simulation
docker_local() { docker --host unix:///var/run/docker.sock "$@"; }
OWNER="$(docker_local inspect -f '{{index .Config.Labels "org.xczs.project"}}' "$NAME")"
[[ "$OWNER" == "$ROOT" ]] || { echo '容器不属于当前项目目录，停止诊断。' >&2; exit 1; }
echo '重启项目容器一次；接下来自动采样 80 秒，期间会打印进度。'
docker_local restart --time 30 "$NAME" >/dev/null
STARTED="$(docker_local inspect -f '{{.State.StartedAt}}' "$NAME")"
PROBE_FAILED=0
if ! docker_local exec -i "$NAME" bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/work01/install/setup.bash
    export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"
    exec /opt/venv/bin/python -u -
' <<'PY'
import json
import time
from collections import deque
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

TYPES = {
    'sensor_msgs/msg/LaserScan': LaserScan,
    'sensor_msgs/msg/Image': Image,
    'nav_msgs/msg/Odometry': Odometry,
    'rosgraph_msgs/msg/Clock': Clock,
    'tf2_msgs/msg/TFMessage': TFMessage,
}
EXPECTED = {
    '/clock': Clock,
    '/xczs/odom': Odometry,
    '/xczs/lidar/scan': LaserScan,
    '/xczs/camera/arm_camera/image_raw': Image,
    '/xczs/camera/image_raw': Image,
    '/tf': TFMessage,
    '/tf_static': TFMessage,
}
rclpy.init()
node = rclpy.create_node('xczs_runtime_diagnose')
subscriptions = {}
data = {}
transforms = {}


def stamp(msg):
    value = msg.clock if isinstance(msg, Clock) else msg.header.stamp
    return round(value.sec + value.nanosec / 1e9, 6)


def receive(topic, msg):
    item = data.setdefault(topic, {'count': 0})
    item['count'] += 1
    if isinstance(msg, TFMessage):
        for transform in msg.transforms:
            key = (topic, transform.header.frame_id, transform.child_frame_id)
            transforms[key] = round(transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9, 6)
        return
    value = stamp(msg)
    item.setdefault('first_stamp', value)
    item['last_stamp'] = value
    if hasattr(msg, 'header'):
        item['frame'] = msg.header.frame_id
    if isinstance(msg, Odometry):
        item['child_frame'] = msg.child_frame_id
    elif isinstance(msg, LaserScan):
        item['ranges_count'] = len(msg.ranges)
    elif isinstance(msg, Image):
        item['size'] = [msg.width, msg.height]


def subscribe(topic, msg_type):
    if topic in subscriptions:
        return
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE
                     if msg_type is TFMessage else ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL
                     if topic.endswith('tf_static') else DurabilityPolicy.VOLATILE)
    subscriptions[topic] = node.create_subscription(
        msg_type, topic, lambda msg: receive(topic, msg), qos)


def connected(start, goal):
    graph = {}
    for topic, parent, child in transforms:
        if topic not in ('/tf', '/tf_static'):
            continue
        graph.setdefault(parent, set()).add(child)
        graph.setdefault(child, set()).add(parent)
    queue = deque([start])
    seen = {start}
    while queue:
        current = queue.popleft()
        if current == goal:
            return True
        for neighbour in graph.get(current, set()) - seen:
            seen.add(neighbour)
            queue.append(neighbour)
    return False


try:
    for topic, msg_type in EXPECTED.items():
        subscribe(topic, msg_type)
    started = time.monotonic()
    next_discovery = started
    next_progress = started
    while time.monotonic() - started < 80:
        now = time.monotonic()
        if now >= next_discovery:
            for topic, types in node.get_topic_names_and_types():
                for msg_type in types:
                    if msg_type in TYPES and node.count_publishers(topic):
                        subscribe(topic, TYPES[msg_type])
                        break
            next_discovery = now + 2
        if now >= next_progress:
            print('采样 %ds/80s：clock=%d odom=%d scan=%d map→odom=%s' % (
                now - started, data.get('/clock', {}).get('count', 0),
                data.get('/xczs/odom', {}).get('count', 0),
                data.get('/xczs/lidar/scan', {}).get('count', 0),
                connected('map', 'odom')), flush=True)
            next_progress = now + 10
        rclpy.spin_once(node, timeout_sec=0.1)
    print('=== 传感器和里程计：发布者数 + 实际收到的数据 ===')
    for topic in sorted(subscriptions):
        if topic in ('/tf', '/tf_static'):
            continue
        print(json.dumps({'topic': topic, 'publishers': node.count_publishers(topic),
                          **data.get(topic, {'count': 0})}, ensure_ascii=False))
    for topic in ('/tf', '/tf_static'):
        print(json.dumps({'topic': topic, 'publishers': node.count_publishers(topic),
                          **data.get(topic, {'count': 0})}, ensure_ascii=False))
    print('=== 关键坐标连接（只计算标准 /tf 与 /tf_static） ===')
    for parent, child in [('odom', 'body'), ('body', 'base_link'), ('map', 'odom')]:
        print(f'{parent} → {child}: {connected(parent, child)}')
    print('=== 与 map、odom、body、雷达相关的实际 TF 边 ===')
    for (topic, parent, child), value in sorted(transforms.items()):
        if parent in ('map', 'odom', 'body') or child in ('map', 'odom', 'body', 'base_link') or 'lidar' in child:
            print(f'{topic}: {parent} → {child}, stamp={value}')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
then
    PROBE_FAILED=1
    echo '采样进程未完成（可能容器退出或重启）；下面打印最近故障信息。' >&2
fi
echo '=== 最近启动错误 ==='
docker_local logs --since "$STARTED" "$NAME" 2>&1 |
    sed -E 's/(密码[：:]|XCZS_(ADMIN_PASSWORD|SECRET_KEY)[=:]).*/\1<REDACTED>/' |
    grep -E 'ERROR:|Application startup failed|Failed to load|cannot open shared|undefined symbol|No camera|最后的 /health' |
    tail -n 15 || true
docker_local inspect -f '状态={{.State.Status}} 健康={{.State.Health.Status}} 重启次数={{.RestartCount}}' "$NAME"
exit "$PROBE_FAILED"
