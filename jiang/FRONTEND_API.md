# XCZS 巡操机器人 — 前端对接 API 文档

## 概述

前端通过 **SSE（Server-Sent Events）** 读取机器人实时数据，通过 **HTTP POST** 发送控制指令。

```
monitor.html ──SSE GET──▶ SSE bridge :8001/{topic}/json   ← 数据读取
monitor.html ──POST─────▶ HTTP :8090/cmd_vel             ← 底盘控制
monitor.html ──POST─────▶ HTTP :8090/joint_trajectory    ← 关节控制
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
| `http://localhost:8001/xczs/cmd_vel/json` | `/xczs/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令 |
| `http://localhost:8001/xczs/odom/json` | `/xczs/odom` | `nav_msgs/Odometry` | 底盘里程计 |
| `http://localhost:8001/xczs/joint_states/json` | `/xczs/joint_states` | `sensor_msgs/JointState` | 关节状态（位置/速度/力矩） |
| `http://localhost:8001/xczs/joint_trajectory/json` | `/xczs/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 机械臂/夹爪目标位置 |
| `http://localhost:8001/robot_description/json` | `/robot_description` | `std_msgs/String` | 机器人 URDF/Xacro 模型 |
| `http://localhost:8001/tf/json` | `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |
| `http://localhost:8001/tf_static/json` | `/tf_static` | `tf2_msgs/TFMessage` | 静态坐标变换 |
| `http://localhost:8001/clock/json` | `/clock` | `rosgraph_msgs/Clock` | 仿真时钟 |
| `http://localhost:8001/mission/phase/json` | `/mission/phase` | `std_msgs/String` | 任务阶段（A/B/C/D/IDLE/DONE） |
| `http://localhost:8001/mission/current_waypoint/json` | `/mission/current_waypoint` | `std_msgs/Int32` | 当前巡检点序号 |

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

## 二、控制指令（HTTP POST）

### 控制服务器地址

```
http://localhost:8090
```

### 接口列表

| 方法 | 端点 | 说明 |
|---|---|---|
| `GET` | `/health` | 健康检查 |
| `POST` | `/cmd_vel` | 底盘速度控制 |
| `POST` | `/joint_trajectory` | 机械臂 + 夹爪控制 |

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
{"status": "ok"}
```

---

### POST /cmd_vel

控制底盘速度。

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

---

## 三、控制服务器启动方式

```bash
# 方式 1：一键启动浏览器控制模式
./run_all.sh --web

# 方式 2：单独启动控制服务
source /opt/ros/humble/setup.bash
/usr/bin/python3 jiang/control_server.py
```

启动后确认端口可访问：
```bash
curl http://localhost:8090/health    # 应返回 {"status":"ok"}
curl -N http://localhost:8001/xczs/odom/json
```
