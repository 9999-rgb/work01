# CLAUDE.md

本文件为 Claude Code（claude.ai/code）在本仓库中工作时提供指导。

## 项目概述

水电站配电室巡检机器人仿真系统 — 基于 **ROS2 + Gazebo Harmonic** 搭建的水电站配电室巡检机器人仿真验证平台。

### 核心系统架构（源自需求文档）

- **仿真引擎**：Gazebo Harmonic + DART 物理引擎
- **机器人模型**：自研差速轮底盘 + JAKA 协作机械臂 + 传感器（激光雷达、深度相机、IMU）
- **场景**：水电站配电室（开关柜阵列、运维通道、电缆沟）
- **控制栈**：
  - **Nav2** — 导航（AMCL 定位、路径规划、避障）
  - **MoveIt2** — 机械臂运动学、轨迹规划、碰撞检测
  - **控制节点** — 底盘差速控制器、机械臂关节控制器、力控模块
  - **感知** — 激光雷达点云处理、深度相机图像识别、IMU 数据融合
  - **任务调度** — 状态机管理、任务序列编排、异常处理
- **交互能力**：开关柜支持柜门开合、按钮旋转等动态交互

### 主要运行模式

| 模式 | 用途 |
|------|------|
| 单工况调试 | 算法开发、参数调优，实时可视化，支持单步执行 |
| 标准工况验证 | 加载标准工况库，自动执行，输出报告 |
| 边界搜索 | 自动参数扫描，定位失效临界点 |
| 蒙特卡洛批量 | 随机扰动注入，大规模统计，可靠性评估 |
| 对比评估 | 相同工况集下对比不同控制/规划策略的指标 |

### 任务流水线

`全局导航 (A) → 精确定位 (B) → 机械臂展开与操作 (C) → 收回与转移 (D)`

按钮操作时序：底盘停靠 → 机械臂展开 → 末端接近按钮（视觉伺服） → 接触检测 → 力位混合控制（旋转按钮） → 旋转到位确认 → 机械臂收回 → 底盘转移至下一巡检点。

### 空间约束

- 柜体间距：60-80cm
- 按钮高度范围：0.8-1.8m
- 高安全等级、防误操作

## 当前状态

本项目处于**需求与规划阶段**，尚无仿真实现代码。

**已完成**：
- `monitor.html` — 基于 SSE 的实时数据监控面板（零依赖，浏览器原生）
- `pub_cmd_vel.py` — ROS2 测试发布节点
- `test_turtle_ws.py` — Zenoh WS 连接验证脚本
- `test_ws.py` — 原生 WebSocket 连通性测试
- `zenoh-bridge-ros2dds` 已配置运行，ROS2 ↔ Zenoh 数据桥可用

**文档**：
- `需求.md` — 完整系统需求及运行流程，含详细流程图
- `核心需求概述.md` — 精简需求概述，含待确认事项

### 待确认事项

JAKA 机械臂具体型号、底盘详细尺寸、传感器型号及安装位姿、按钮物理参数、配电室典型布局、是否有实体机对标校准、批量仿真规模、多机协同需求、仿真加速需求。

## 技术栈（规划）

- **ROS2**（Humble/Iron — 待定）
- **Gazebo Harmonic** + DART 物理引擎
- **URDF** 用于机器人描述，**SDF** 用于场景描述
- **Nav2** 导航框架
- **MoveIt2** 机械臂运动规划
- **C++/Python** 编写 ROS2 节点（ROS2 生态惯例）

## 开发工具链（已就绪）

### Zenoh Bridge — ROS2 ↔ 浏览器实时数据桥

**命令**：`/home/zwy/Desktop/zenoh-bridge-ros2dds --listen tcp/0.0.0.0:7447 --listen ws/0.0.0.0:10000 --rest-http-port 8000`

三个端点：

| 端点 | 地址 | 用途 |
|------|------|------|
| TCP | `tcp/0.0.0.0:7447` | Zenoh 原生协议（Peer 模式） |
| WebSocket | `ws/0.0.0.0:10000` | Zenoh 二进制协议（仅 zenoh 客户端可用） |
| REST SSE | `http://localhost:8000` | HTTP 长连接 SSE，浏览器原生支持 |

**关键行为**：
- 自动将 ROS2 话题转发到 Zenoh key expression（如 `/turtle1/cmd_vel` → `turtle1/cmd_vel`）
- 原始 CDR 二进制发布在 `{topic}`，JSON 格式发布在 `{topic}/json`
- SSE 响应格式：`event:PUT` + `data:{"key":"...","value":{...},"encoding":"application/json","timestamp":"..."}`]
- CORS 已开启（`Access-Control-Allow-Origin: *`），可从任意来源访问
- `$$ ros2 node list` 中显示为 `/zenoh_bridge_ros2dds`

**已验证连接方式**：

| 方式 | 可行 | 说明 |
|------|------|------|
| Python `eclipse-zenoh` 库 TCP 模式 | ✅ | `connect.endpoints: ["tcp/localhost:7447"]` |
| Python `eclipse-zenoh` 库 WS 模式 | ✅ | `mode:"client"`, `connect.endpoints: ["ws/localhost:10000"]`（注意：不是 `ws://`） |
| 浏览器 `@eclipse-zenoh/zenoh-ts` | ✅ | 需 import map 补全依赖 |
| 浏览器原生 `EventSource` (SSE) | ✅ | `new EventSource("http://localhost:8000/{topic}")`，推荐方案 |
| 原生 WebSocket（无 zenoh 协议） | ❌ | 连接可建立但无数据传输 — 缺少 Zenoh 会话/订阅握手 |

### monitor.html — 实时数据监控面板

**文件**：`monitor.html`

**架构**：纯 HTML + 原生 `EventSource`，**零 JS 依赖**，直接从 Zenoh REST SSE 获取 JSON 数据。

**使用方式**：
1. 浏览器打开 `monitor.html`
2. REST API 地址默认 `http://localhost:8000`，点击"连接"
3. 点击预设话题或输入自定义话题（如 `turtle1/cmd_vel/json`）订阅
4. 数据实时卡片展示，含 JSON 语法高亮

**设计要点**：
- 不用 `@eclipse-zenoh/zenoh-ts`（该库通过 unpkg 加载时会触发 `typed-duration`、`channel-ts`、`@thi.ng/leb128` 等裸模块解析失败）
- `EventSource` 自动重连，浏览器原生 SSE 支持
- 每个话题一个独立 EventSource 实例，取消订阅即 `close()`

### 测试脚本

| 文件 | 用途 |
|------|------|
| `pub_cmd_vel.py` | ROS2 节点 — 持续发布 `geometry_msgs/Twist` 到 `/turtle1/cmd_vel`（5Hz，带正弦波 angular.z） |
| `test_turtle_ws.py` | Python zenoh 客户端 — WS 连接 + 订阅 `turtle1/cmd_vel/json` 验证数据流 |
| `test_ws.py` | 原生 WebSocket 测试（仅验证连接，无法获取数据） |

## 项目约定

- 仿真模型需与实物 1:1 对应
- SDF 场景文件需支持多布局配置
- 扰动注入（地面打滑、传感器噪声、通信延迟、负载突变）需可配置
- 所有仿真运行必须记录：底盘轨迹、机械臂关节角/力矩、末端位姿、传感器数据
- 输出指标：任务完成率、轨迹稳定性（RMSE）、安全距离合格率、操作成功率、时间效率
