# 两场景连续全量验收（2026-10-17）

**本机连续一批通过：15 个工位自主导航、50 次动作检查。** 使用 Web API → ROS 2 → Gazebo 实际执行，包含四个抽拉柜、六个旋钮、四个按钮和摇杆；未预定位机器人或直接设置控件终态。麒麟 ARM64 尚需拉取、重建后复测。

## 本轮发现与修复

初次重跑在 db1 停止：8 次动作请求成功，但第二轮打开后的收臂阶段，左侧支撑杆相对基座瞬时偏转 **8.318°**，超过原 5° 验收线，整批判失败。原始记录保留在 `log/retest_full_20261017_1341/full/`，没有改写为成功。

柜外准备／收臂执行器只复制了 MoveIt 轨迹的位置和时间，丢掉规划的速度与加速度，导致控制器逐段线性插值，在路径拐点突然换速。本轮在这些未改时序的规划路径中保留速度和加速度，并按时间缩放分别除以比例及比例平方。抽拉路径另按实际笛卡尔路程重新定时，仍不复用旧导数；B 夹爪的仅位置轨迹不受影响。

修正后的完整批次中 db1 最大杆偏转为 **0.684°**。没有修改碰撞几何、质量惯量、操作行程或验收阈值。专项测试 `scripts/validate/test_taught_drawer_safety.py` **14 项通过**，包含关节顺序、时间／导数缩放和抽拉路径不复用旧导数的检查。

## 连续批次结果

| 范围 | 动作检查 | 实际结果 |
| --- | ---: | --- |
| db1 | 8 | 两轮开闭、重复目标、释放收臂及几何通过；7,288 帧，最大杆偏转 0.684° |
| dm1 | 8 | 全部通过；7,460 帧，最大杆偏转 1.594° |
| ds2 | 8 | 全部通过；7,100 帧，最大杆偏转 0.731°，支撑杆最大偏移 0.031 mm |
| ds3 | 8 | 全部通过；6,993 帧，最大杆偏转 1.844° |
| 六个旋钮双向 | 12 | 夹紧、拉出、旋转、插回、松爪、收臂和终态通过，无旁路控件串动 |
| 四个按钮 | 4 | 实际按下行程、触发、释放和收臂通过 |
| 摇杆整圈与短转停止 | 2 | 均确认后端终止、任务占用释放、插口停止、退出及收臂 |

四个抽拉柜合计 **28,841 帧**几何记录。B 整段 **3,249 帧**几何检查通过，最小柜体采样净空 **1.246 mm**，目标接触最小有符号距离 **-0.0581 mm**，满足原 1 mm 数值接触容差。采样不能证明完整连续无碰撞，结果同时结合规划碰撞检查、实际行程、接触保护和终态。四张抽拉最不利时刻图及旋钮、按钮、摇杆三张代表接触图均已查看。

六个旋钮全过程轴向峰值为 **7.67–7.93 mm**，释放阶段不超过 **0.501 mm**，没有复现此前约 15 mm 的超程。第二个旋钮双向通过。第四按钮到摇杆的导航用时 **10.93 s**，粗到站位置误差 74.7 mm、偏航误差 0.0714 rad，满足原 80 mm／0.10 rad 门限，随后另执行精确停靠。

摇杆两次停止后左臂最大回零误差分别 **0.000900／0.000949 rad**，工具距插口分别 **1.514／1.514 m**。取消过程中曾出现宽限期内未确认的中间状态，验收器继续等待，最终两次均得到 `backend_termination_confirmed=true`，未以最初的取消响应充当停止证据。

## 证据与复现

本轮运行目录：`log/retest_full_20261017_1725/`。

- `full/summary.json`：`success=true`，完整运行包含 A → B → A 切换。
- `full/*_navigation.json`：15 个自主导航结果。
- `full/*_web.json`、`*_turned.json`、`*_center.json`、`*_press.json`、`frb_repeat.json`：50 次动作检查及实际采样。
- `full/*_geometry.json`、`*_motion.jsonl`、`generator_contact.jsonl`：几何判定和原始位姿。
- `source.json`、`changes.patch`、`knob_axial_acceptance.json`、`final_readiness.json`：代码校验、补丁、旋钮行程与结束时 A 就绪状态。

本机复现：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
./run_all.sh --web
# 另一终端同样 source 环境、设置上述变量后执行；每次使用新的输出目录。
python3 -u scripts/validate/validate_two_scene_web.py --phase all --navigate \
  --drawer-cycles 2 --out log/two_scene_full_NEW
```

麒麟更新需要重建镜像；仅 `docker restart` 不会应用新代码：

```bash
cd /root/proj/work01-gitee
git pull --ff-only
GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 bash ./start
```

前端：**http://10.186.109.146:8888/**。

## 空间管理

验收结束后已正常关闭本机测试实例，避免空闲时 Gazebo 日志持续增长。仅压缩已关闭且无打开句柄的日志，校验解压内容 SHA-256 一致后移除未压缩副本，当前和失败的动作证据均保留。压缩文件位于 `log/archive/`，实测空间变化和校验值见两次运行目录的 `cleanup*.json`。
