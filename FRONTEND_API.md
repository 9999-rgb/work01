# XCZS 巡操机器人 — 前端对接 API 文档

## 概述

前端通过 **SSE（Server-Sent Events）** 读取常规 ROS 2 数据，通过
**MJPEG** 显示相机，通过 **WebSocket JSON** 显示雷达，并通过
**HTTP POST** 发送控制指令。

```
monitor.html ──SSE GET──▶ SSE bridge :8001/{topic}/json   ← 数据读取
monitor.html ──MJPEG────▶ Sensor API :8003/camera.mjpg    ← 相机图像
monitor.html ──WebSocket▶ Sensor API :8003/lidar/ws       ← 雷达扫描
monitor.html ──HTTP─────▶ Control API :8090              ← 手动控制
                                      ├── Nav2 action    ← 导航目标
                                      └── MoveIt action  ← 机械臂规划
```

---

## 一、数据读取（SSE 订阅）

### 连接地址

```
http://localhost:8001
```

### 订阅方式

浏览器端使用 `EventSource` 建立 SSE 长连接：

```javascript
const es = new EventSource('http://localhost:8001/xczs/cmd_vel/json');
es.addEventListener('PUT', (event) => {
  const data = JSON.parse(event.data);
  // data.key   — 话题名
  // data.value — 实际消息内容（JSON）
  // data.timestamp — 时间戳
});
```

### 可用话题

所有话题后追加 `/json` 后缀即可获取 JSON 格式数据，数据体为数据来源：用户提供 robot_control.yaml 中所有话题的完整消息类型定义）：

| SSE URL | ROS2 话题 | 消息类型 | 说明 |
|---|---|---|---|
| `http://localhost:8001/xczs/cmd_vel/json` | `/xczs/cmd_vel` | `geometry_msgs/Twist` | 路由后的底盘最终速度指令 |
| `http://localhost:8001/xczs/manual_cmd_vel/json` | `/xczs/manual_cmd_vel` | `geometry_msgs/Twist` | Web、GUI 或键盘手动速度输入 |
| `http://localhost:8001/xczs/odom/json` | `/xczs/odom` | `nav_msgs/Odometry` | 底盘里程计 |
| `http://localhost:8001/amcl_pose/json` | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Nav2 地图定位结果 |
| `http://localhost:8001/plan/json` | `/plan` | `nav_msgs/Path` | Nav2 当前全局规划路径 |
| `http://localhost:8001/xczs/joint_states/json` | `/xczs/joint_states` | `sensor_msgs/JointState` | 关节状态（位置/速度/力矩） |
| `http://localhost:8001/xczs/joint_trajectory/json` | `/xczs/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 机械臂/夹爪目标位置 |
| `http://localhost:8001/robot_description/json` | `/robot_description` | `std_msgs/String` | 机器人 URDF/Xacro 模型 |
| `http://localhost:8001/tf/json` | `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |
| `http://localhost:8001/tf_static/json` | `/tf_static` | `tf2_msgs/TFMessage` | 静态坐标变换 |
| `http://localhost:8001/clock/json` | `/clock` | `rosgraph_msgs/Clock` | 仿真时钟 |

### SSE 响应格式

每条消息以 `event:PUT` 命名事件下发：

```json
{
  "key": "xczs/cmd_vel",
  "value": {
    "linear_x": 0.0,
    "linear_y": 0.25,
    "linear_z": 0.0,
    "angular_x": 0.0,
    "angular_y": 0.0,
    "angular_z": 0.6
  },
  "encoding": "application/json",
  "timestamp": "1784866351000"
}
```

前端取 `data.value` 即为实际消息内容。

### 关键消息体结构

**Twist**（cmd_vel）：
```json
{
  "linear_x": 0.0,
  "linear_y": 0.25,
  "linear_z": 0.0,
  "angular_x": 0.0,
  "angular_y": 0.0,
  "angular_z": 0.6
}
```
> 机器人前方为 +Y 轴：前进 = `linear_y > 0`，左转 = `angular_z > 0`

**Odometry**（odom）：
```json
{
  "pose": {
    "pose": {
      "position":    { "x": 1.5, "y": 0.3, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.1, "w": 0.99 }
    }
  },
  "twist": {
    "twist": {
      "linear":  { "x": 0.0, "y": 0.2, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
    }
  }
}
```

**JointState**（joint_states）：
```json
{
  "name": ["body_arm1","arm1_arm2","arm2_arm3","arm3_arm4","arm4_arm5","arm5_end",
           "end_worklink1","end_worklink2",
           "body_frw","body_flw","body_brw","body_blw"],
  "position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "velocity": [0.0, ...],
  "effort":   [0.0, ...]
}
```
> 前 8 个是机械臂+夹爪关节，后 4 个是车轮关节

---

## 二、相机与雷达接口

### 服务地址

```text
http://localhost:8003
```

| 方法 | 端点 | 响应格式 | 说明 |
|---|---|---|---|
| `GET` | `/health` | JSON | 相机、雷达数据就绪状态和数据年龄 |
| `GET` | `/camera.mjpg` | MJPEG | 浏览器实时相机流 |
| `GET` | `/camera.jpg` | JPEG | 最新相机单帧 |
| `GET` | `/lidar.json` | JSON | 最新雷达扫描快照 |
| `WS` | `/lidar/ws` | JSON | 持续推送最新雷达扫描 |

### 相机 MJPEG

浏览器可直接通过 `<img>` 标签显示，不需要额外 JavaScript 解码：

```html
<img src="http://localhost:8003/camera.mjpg" alt="XCZS camera">
```

单帧图像可直接下载：

```bash
curl http://localhost:8003/camera.jpg --output camera.jpg
```

服务端默认把 ROS 2 的 `800×800 rgb8` 图像转换为质量 `80` 的 JPEG，并以
约 `10 FPS` 向浏览器传输；这不会改变 ROS 2 原始图像话题的 `30 Hz` 频率。

### 雷达 JSON 快照

```bash
curl http://localhost:8003/lidar.json
```

主要字段：

```json
{
  "type": "sensor_msgs/msg/LaserScan",
  "frame_id": "body_lidar_link",
  "angle_min": -1.570796,
  "angle_max": 1.570796,
  "angle_increment": 0.004369,
  "range_min": 0.1,
  "range_max": 30.0,
  "sample_count": 720,
  "valid_count": 141,
  "closest_range": 0.142,
  "ranges": [null, null, 1.82]
}
```

无有效回波的 `NaN` 或无穷大数据会转换为 JSON `null`，因此浏览器可直接
调用 `JSON.parse()`，不会收到非法 JSON 数值。

### 雷达 WebSocket

```javascript
const socket = new WebSocket('ws://localhost:8003/lidar/ws');
socket.onmessage = (event) => {
  const scan = JSON.parse(event.data);
  console.log(scan.sample_count, scan.closest_range, scan.ranges);
};
```

WebSocket 默认最多以约 `10 Hz` 推送最新雷达帧。服务只保留最新一帧，慢速
浏览器不会导致 ROS 2 的 `40 Hz` 雷达消息不断积压。

### 健康检查

```bash
curl http://localhost:8003/health
```

相机和雷达都已收到数据时，响应示例：

```json
{
  "status": "ok",
  "camera": {"ready": true, "sequence": 12, "age_seconds": 0.03},
  "lidar": {"ready": true, "sequence": 48, "age_seconds": 0.01}
}
```

---

## 三、控制指令（HTTP POST）

### 控制服务器地址

```
http://localhost:8090
```

### 接口列表

| 方法 | 端点 | 说明 |
|---|---|---|
| `GET` | `/health` | 服务、Nav2 和 MoveIt 2 可用状态 |
| `GET` | `/navigation/status` | Nav2 状态、定位、路径和反馈 |
| `GET` | `/navigation/map` | Nav2 占用地图 |
| `GET` | `/motion/status` | MoveIt 2 规划和执行状态 |
| `POST` | `/cmd_vel` | 底盘速度控制 |
| `POST` | `/joint_trajectory` | 机械臂 + 夹爪控制 |
| `POST` | `/navigation/mode` | 切换手动/Nav2 底盘模式 |
| `POST` | `/navigation/goal` | 发送 Nav2 地图坐标目标 |
| `POST` | `/navigation/cancel` | 取消 Nav2 导航 |
| `POST` | `/motion/named` | MoveIt 2 命名目标规划/执行 |
| `POST` | `/motion/pose` | MoveIt 2 末端位姿规划/执行 |
| `POST` | `/motion/cancel` | 取消 MoveIt 2 动作 |

所有接口均支持 CORS，可从任意域名访问。

---

### GET /health

健康检查。

**请求：**
```bash
curl http://localhost:8090/health
```

**响应：**
```json
{
  "status": "ok",
  "navigation_available": true,
  "moveit_available": true
}
```

---

### POST /cmd_vel

发送手动底盘速度。控制服务实际发布到 `/xczs/manual_cmd_vel`，再由底盘命令
路由器转发到 Gazebo。Nav2 模式启用时该接口仍会正常响应 HTTP 请求，但手动
运动命令会被路由器阻止，避免绕过导航规划和碰撞保护。

**请求：**

```bash
curl -X POST http://localhost:8090/cmd_vel \
  -H 'Content-Type: application/json' \
  -d '{"linear_y": 0.25, "angular_z": 0.0}'
```

```javascript
// 前端 fetch 示例
await fetch('http://localhost:8090/cmd_vel', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ linear_y: 0.25, angular_z: 0.0 })
});
```

**请求体：**

| 字段 | 类型 | 说明 |
|---|---|---|
| `linear_y` | float | 前进/后退速度 (m/s)，机器人前方为 +Y 轴 |
| `angular_z` | float | 旋转速度 (rad/s)，正 = 左转 |

> 底盘持续运动建议以 10 Hz 重复发送。服务端超过 0.3 秒未收到新指令时会自动
> 平滑停车；停止时应主动发送 `{"linear_y":0,"angular_z":0}`。
> 恢复 Web 手动底盘控制前，执行
> `ros2 service call /xczs/set_navigation_mode std_srvs/srv/SetBool
> "{data: false}"`；需要重新交给 Nav2 时将 `data` 设置为 `true`。

**响应：**
```json
{"status": "ok", "linear_y": 0.25, "angular_z": 0.0}
```

**操作对照表：**

| 操作 | linear_y | angular_z |
|---|---|---|
| 前进 | 0.25 | 0 |
| 后退 | -0.25 | 0 |
| 左转 | 0 | 0.6 |
| 右转 | 0 | -0.6 |
| 停止 | 0 | 0 |

---

### POST /joint_trajectory

控制机械臂 6 个关节 + 夹爪 2 个手指。

**请求：**

```bash
curl -X POST http://localhost:8090/joint_trajectory \
  -H 'Content-Type: application/json' \
  -d '{"positions":[0.0,0.0,0.0,0.0,0.0,0.0,0.35,-0.35]}'
```

```javascript
// 前端 fetch 示例
await fetch('http://localhost:8090/joint_trajectory', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ positions: [0, 0, 0, 0, 0, 0, 0.35, -0.35] })
});
```

**请求体：**

| 字段 | 类型 | 说明 |
|---|---|---|
| `positions` | float[8] | 8 个关节目标角度 (rad) |

**8 个关节定义：**

| 索引 | 关节名 | 类型 | 范围 |
|---|---|---|---|
| 0 | `body_arm1` | 机械臂关节 1 | ±2.80 rad |
| 1 | `arm1_arm2` | 机械臂关节 2 | ±2.80 rad |
| 2 | `arm2_arm3` | 机械臂关节 3 | ±2.80 rad |
| 3 | `arm3_arm4` | 机械臂关节 4 | ±2.80 rad |
| 4 | `arm4_arm5` | 机械臂关节 5 | ±2.80 rad |
| 5 | `arm5_end` | 机械臂关节 6 | ±2.80 rad |
| 6 | `end_worklink1` | 夹爪手指 1 | 0 ~ 0.35 rad |
| 7 | `end_worklink2` | 夹爪手指 2 | -0.35 ~ 0 rad |

**常用预设值：**

| 操作 | positions |
|---|---|
| 全部归零 | `[0, 0, 0, 0, 0, 0, 0, 0]` |
| 打开夹爪 | `[*, *, *, *, *, *, 0.35, -0.35]` |
| 关闭夹爪 | `[*, *, *, *, *, *, 0, 0]` |

> 接口不接受 `*`。前端应缓存当前六个机械臂关节角度，并在控制夹爪时将这些数值
> 一并发送。

**响应：**
```json
{"status": "ok", "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35, -0.35]}
```

该接口属于手动关节调试模式，不执行逆运动学和路径规划。MoveIt 正在通过
`FollowJointTrajectory` action 执行机械臂或夹爪时，ROS 2 兼容路由器会拒绝
对应控制组的手动轨迹，避免 Web 指令绕过碰撞保护并中断规划动作。需要指定
末端位姿时，应使用 MoveIt 规划接口；当前命令行入口和参数格式见
[README.md](README.md#moveit-2-规划控制)。

---

## 四、Nav2 导航接口

网页通过标准 ROS 2 `NavigateToPose` action 控制 Nav2。提交目标时，服务会
自动把底盘路由切换为 Nav2 模式；切回手动模式时会先取消活动导航并停车。

### GET /navigation/status

```bash
curl http://localhost:8090/navigation/status
```

响应包含 action 可用性、命令路由模式、当前目标、AMCL 位置、全局路径和导航
反馈。`state` 可能为 `idle`、`enabling`、`sending`、`navigating`、
`canceling`、`succeeded`、`canceled`、`rejected` 或 `failed`。

```json
{
  "available": true,
  "mode": true,
  "state": "navigating",
  "message": "Nav2 is driving to the selected goal.",
  "goal": {"x": 0.0, "y": 1.0, "yaw": 1.57079632679},
  "current_pose": {"x": 0.0, "y": 0.42, "yaw": 1.55},
  "distance_remaining": 0.58,
  "eta_seconds": 2.4,
  "navigation_time_seconds": 3.1,
  "recoveries": 0,
  "plan": [{"x": 0.0, "y": 0.42}, {"x": 0.0, "y": 1.0}]
}
```

### GET /navigation/map

返回 `/map` 的 `nav_msgs/OccupancyGrid` 浏览器快照：

```bash
curl http://localhost:8090/navigation/map
```

```json
{
  "frame_id": "map",
  "width": 200,
  "height": 200,
  "resolution": 0.05,
  "origin": {"x": -5.0, "y": -5.0, "yaw": 0.0},
  "data": [-1, 0, 0, 100]
}
```

`data` 按 ROS 2 OccupancyGrid 规则使用行优先排列：`-1` 表示未知，`0` 表示
空闲，`100` 表示占用。

### POST /navigation/mode

```bash
# 启用 Nav2 底盘命令
curl -X POST http://localhost:8090/navigation/mode \
  -H 'Content-Type: application/json' \
  -d '{"enabled":true}'

# 恢复网页手动底盘控制
curl -X POST http://localhost:8090/navigation/mode \
  -H 'Content-Type: application/json' \
  -d '{"enabled":false}'
```

### POST /navigation/goal

目标使用 `map` 坐标系，`yaw` 单位为弧度：

```bash
curl -X POST http://localhost:8090/navigation/goal \
  -H 'Content-Type: application/json' \
  -d '{"x":0.0,"y":1.0,"yaw":1.57079632679}'
```

服务会拒绝地图外、占用或未知栅格中的目标，也不会覆盖仍在执行的导航目标。
成功接收后返回 HTTP `202`；后续进度通过 `/navigation/status` 获取。

### POST /navigation/cancel

```bash
curl -X POST http://localhost:8090/navigation/cancel \
  -H 'Content-Type: application/json' \
  -d '{}'
```

---

## 五、MoveIt 2 规划接口

网页通过标准 ROS 2 `MoveGroup` action 提交规划。`execute:false` 只进行逆运动
学、路径规划和碰撞检查，`execute:true` 会在规划成功后通过
`JointTrajectoryController` 执行轨迹。

### GET /motion/status

```bash
curl http://localhost:8090/motion/status
```

`state` 可能为 `idle`、`sending`、`planning`、`executing`、
`canceling`、`succeeded`、`canceled`、`rejected` 或 `failed`。

```json
{
  "available": true,
  "state": "succeeded",
  "message": "MoveIt motion succeeded.",
  "execute": false,
  "target": {
    "type": "named",
    "group": "manipulator",
    "name": "home"
  },
  "planning_time": 0.031,
  "error_code": 1
}
```

### POST /motion/named

当前支持 `manipulator/home`、`gripper/open` 和 `gripper/closed`：

```bash
curl -X POST http://localhost:8090/motion/named \
  -H 'Content-Type: application/json' \
  -d '{"group":"manipulator","target":"home","execute":true}'

curl -X POST http://localhost:8090/motion/named \
  -H 'Content-Type: application/json' \
  -d '{"group":"gripper","target":"open","execute":true}'
```

### POST /motion/pose

位置单位为米，四元数顺序为 `[x, y, z, w]`。允许的参考坐标系为 `body`、
`odom` 和 `map`：

```bash
curl -X POST http://localhost:8090/motion/pose \
  -H 'Content-Type: application/json' \
  -d '{
    "frame_id":"body",
    "position":[-0.226,0.787,0.318],
    "orientation":[0.653,0.153,0.741,0.015],
    "execute":false
  }'
```

把 `execute` 改为 `true` 即可规划并执行。服务会归一化非零四元数；MoveIt 2
负责关节限位、自碰撞和 Planning Scene 柜体碰撞检查。

### POST /motion/cancel

```bash
curl -X POST http://localhost:8090/motion/cancel \
  -H 'Content-Type: application/json' \
  -d '{}'
```

同一时间只能执行一个 MoveIt 目标。规划执行期间，旧的手动关节接口会被对应
控制组拒绝，避免绕过碰撞检查。

---

## 六、服务启动方式

```bash
# 方式 1：一键启动网页、Gazebo、Nav2 和 MoveIt 2
./run_all.sh --web

# 方式 2：单独启动控制服务
source /opt/ros/humble/setup.bash
/usr/bin/python3 jiang/control_server.py

# 单独启动相机与雷达流服务
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 jiang/scripts/sensor_stream_server
```

启动后确认端口可访问：
```bash
curl http://localhost:8090/health
curl http://localhost:8090/navigation/status
curl http://localhost:8090/motion/status
curl http://localhost:8003/health
curl http://localhost:8003/lidar.json
curl -N http://localhost:8001/xczs/odom/json
```
