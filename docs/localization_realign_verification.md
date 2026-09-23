# 定位重播种与场景地图同步验收（2026-09-22）

## 变更与结论

- **导航与操作的定位重播种**（`jiang/control_gateway/runner.py`）。长距离行驶会积累里程计
  漂移：实测一次 17 m 行程的末端 AMCL 信念偏离物理真值 **7.9 m**。Nav2 据此算不出计划
  （`Resulting plan has 0 poses in it` → `Controller patience exceeded` → `Navigation was
  aborted`），同时按错误位置打方向，使底盘偏出走廊。现按 Gazebo 物理真值周期性校准信念，
  共三处触发：
  1. 导航任务开始时（`_execute_navigation_task`，路由循环之前）；
  2. 导航途中每 2 s（监视循环内，`NAVIGATION_REALIGN_PERIOD_SEC`）；
  3. 操作任务开始前（工位门等待之前）。
  三处都只在偏差超过 `0.05 m / 0.05 rad` 时才重发 `/initialpose` 并等收敛（复用开机播种
  的 `_confirm_localization`）；偏差小时只做一次读数比较即返回，不发消息。
- **关键不变量**：播种值取机器人**此刻的真实位姿**，不是工位目标点。因此机器人若真的不在
  工位，纠正后工位门照样如实失败 —— 不会把错位掩盖成「已到站」。
- 该机制是**仿真专属**：真值来自 Gazebo `/get_entity_state`
  （`gazebo_client.get_entity_state`）。真机没有对应服务时该方法抛
  `ControlRequestError(503)`，由调用方（`runner`）捕获后返回 None，语义不变。
  它与既有的「物理锚定」同源 —— operator 已用同一套真值修正柜体几何，这里只是把同一套
  真值也用到机器人自身的信念上。
- **场景地图同步**（`scripts/tools/generate_scene_maps.py`）。该脚本原先只写仓库
  `xczs_inspection_robot_nav2/maps/`；而运行时经资产库 `scenes.yaml` 读的是
  `jiang/data/assets/scene/<name>/maps/` 的自包含副本（导入时复制、`nav2_map` 归一化为
  绝对路径）。于是地图一重生成，资产库那份就成了**静默过期快照**。现脚本在写完后自动镜像
  到资产库同场景副本（该资产存在才写）。

## 导航与操作证据

操作侧三步（同一 dock、同一转移，全程只读测量）：

| 步骤 | 条件 | 结果 |
| --- | --- | --- |
| 基线 | 信念误差 3 cm / 0.55° | **未触发**重播种（日志 0 条），操作 success 69.2 s |
| 注入漂移 | 人为发偏 0.706 m / 0.170 rad 的 `/initialpose` | 日志报 `Localization drifted ... re-hypothesized AMCL at the physical pose`，操作 success |
| 负例（验不变量） | 把机器人传送到工位外 3.00 m | 重播种到**真实所在**（−16.186, 6.212），随后工位门如实失败：`position error 2.999 m vs 1.0 m`、`the cabinet action was not started` |

导航侧前后对照（同一条 `fr20422_knob` 工位导航，同一起点、同一张地图）：

| 指标 | 修复前 | 修复后 |
| --- | ---: | ---: |
| 结果 | `Navigation was aborted` | **success，60.2 s** |
| Nav2 recoveries | 11 | **0** |
| 末端信念 vs 真值 | 偏 7.9 m | 约 0.03 m |
| 终点误差 | —（未到达） | **0.163 m** |

途中校准在该次 60 s 行程中触发 15 次，每次修正量 0.064–0.075 m，即**持续小幅钉住**而非
「跳一下」，因此 Nav2 全程都能正确规划。

**横向偏差**（Gazebo 真值，走廊段 x∈[−32, −16.5]，中线 y=5.695）：

| | 最大偏差 | 平均偏差 |
| --- | ---: | ---: |
| 修复前 | 0.285 m | 0.056 m |
| 修复后 | **0.249 m** | 0.085 m |

走廊在工位段的连通可用带宽 **1.99 m**（y∈[4.70, 6.69]）；机器人含末端工具的横向包络
**1.535 m**（`measure_tool_extent.py --frame body` 的网格包围盒并集，本身偏保守），故净空
为 **0.228 m**。修复后最差点（x≈−19.1，偏向柜体侧）仍超出该净空约 2 cm。原因是路径没有压
中线 —— 代价地图的零代价带半宽为 `(1.99 − 2×0.60)/2 = 0.395 m`，允许它偏到这个程度。
**因此不声称臂与柜体完全不接触**，只声称"因定位发散而偏出走廊中线"这一条已被消除。

## 场景地图证据

- 运行时 `/map` 元数据：修复前 821×548 / origin `(−40.012, −1.0)`（资产库旧快照），
  修复后 **841×568 / origin `(−40.512, −1.5)`**，与仓库现行地图逐字节一致（md5 相同）。
- 出生点 0.84 m 见方内在现行地图中占用 **0/289 = 0%**。
- 读图注意：PGM 文件的行序是自上而下（左下角为 origin），直接按 `row=(y−oy)/res` 读会得到
  **上下镜像**的结果；本轮的占用统计按翻转后的行序复核过（健全性自检：柜体实体内部判为占用、
  各控件工位判为空闲）。

## 校验与复现

- `jiang/tests` 全量 **761 项通过**（改动前后各跑一次）；控制包与全部 7 个包
  `colcon build --symlink-install` 通过。
- 三处触发点均经真实 Web API 活验（`POST /task/operate`、`POST /task/navigate`），
  重启入口为 `./run_all.sh --web`。
- **未做**：六个旋钮的逐控件回归（本轮只验收了 fr135 与 fr20422 两条转移）；臂与柜体的接触
  判定（无接触传感器，只有横向偏差这一间接量）；`generate_scene_maps.py` 的镜像分支没有
  端到端跑过（需要 `vtk` 且会重写地图数据），只做了语法检查与相关单测。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 判读要点：跑完看日志里有没有 Localization drifted 行，以及 recoveries 是否为 0
python3 scripts/tools/measure_tool_extent.py --frame body    # 横向包络
```
