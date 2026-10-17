# 两场景修复与全量复测（2026-10-17）

**本机分阶段验收：50 次动作任务请求全部通过。** 本机 amd64、ROS 2 Humble、Gazebo 11；麒麟 ARM64 尚未复测，必须拉取后重建镜像。全部通过 Web API 自主导航、执行动作，没有预定位机器人或直接设置控件终态。

## 验收结果

| 范围 | 动作请求 | 验证结果 |
| --- | ---: | --- |
| db1、dm1、ds2、ds3 抽拉柜 | 32 | 各两轮开闭及重复目标；29,989 帧几何检查通过 |
| 六个旋钮 | 12 | 拉出、双向旋转、插回、松爪、收臂及终态全部通过 |
| 四个按钮 | 4 | 实际按下行程、触发、释放与收臂通过 |
| 摇杆整圈及短转停止 | 2 | 两次均确认后端停止、退出插口、收臂与释放任务占用 |

最后的夹爪修改只影响 B 旋钮，沿用本轮已通过的 A 抽拉柜结果；最后恢复 B 导航旋转评分后，额外重走全部 **11 个发电机工位**，全部通过。538 帧导航采样最小工具—柜体净空 **69.7 mm**。另复现“按完第四个按钮→摇杆”的原失败路线：13.45 s 到站、零恢复重试，位置误差 64.6 mm、朝向误差 0.00354 rad，保持原 80 mm／0.10 rad 到站阈值。

这是一组修复过程中的分阶段验收结果；原先中途失败的整批 `summary.json` 保持失败，没有改写成通过。汇总依据在 `log/retest_20261017/final_acceptance.json`。

## 修复内容与依据

- **B 底盘保留关节相对速度。** 新平面移动插件和稳定器只修正底盘刚体速度增量；原模型级速度覆盖会抹掉机械臂、轮子及工具的相对运动。A 保留原底盘和稳定器行为。新插件保留上游 Apache 许可及来源说明。
- **B 导航里程计换轴。** 新增 `/xczs/nav_odom`，子坐标系为 `base_link`，同步转换位姿方向、线／角速度及完整速度协方差；原 `/xczs/odom` 和 `odom→body` 接口保留。`body→base_link` 固定偏航为 -π/2，Nav2 原先直接使用 body 轴速度，与指令轴不一致。1,139 条对照采样的速度指令差值均方根由 0.653 m/s 降至 1.61e-8 m/s；这是坐标一致性检查，不是轨迹误差测量。
- **毫米级对中保留完整路径。** 对小幅对中在本地生成未计时的笛卡尔路径，逐点检查当前碰撞场景和约束，再细粒度计时；避免默认 TOTG 将小于约 0.001 rad 的整段变化压成单个静止点。原完成比例和绝对关节跳变限制保留。机器人相对锚点的 2 m 修正护栏改在机器人附近计算，避免约 30 m 外的场景原点放大微小偏航差。
- **定位与避障。** 已知 Gazebo 位姿播种 AMCL 时使用 2 cm 位置标准差和 0.01 rad 偏航标准差；B 降低对真实仿真里程计额外注入的运动噪声。B 局部地图加入静态层，补足 180° 雷达盲区；完整工具外形参与碰撞检查。B 粗到站阈值为 8 cm、0.10 rad，预测时域为 1 s。
- **夹爪接触跟随有总行程上限。** 以夹紧验证目标为固定参考，每次补偿 0.15 mm，相对参考最多额外闭合 0.35 mm。保持和旋转共用这条规则，避免每次以新目标为参考无限累积。原双侧接触及 1 mm 持续／2 mm 瞬时穿入保护没有放宽。
- **夹爪轨迹使用仅位置点。** 原零终端速度会让 Humble 插值器引入接触求解器反馈的起始速度，张开指令反而先向内过冲。安装版插值器回归：起点 12 mm、反馈速度 20 mm/s、4 s 张开到零，旧方式先超过 18 mm；仅位置点始终位于起点和目标之间。保持、转盘及开闭轨迹均使用此方式。[控制器起点选择源码](https://github.com/ros-controls/ros2_controllers/blob/humble/joint_trajectory_controller/src/joint_trajectory_controller.cpp)
- **终点转向恢复正常评分。** B 保留基础 `Twirling.scale=5`。试验中的 100 在终点压过 `RotateToGoal.scale=32`、0.5 s 前瞻的转向收益，造成位置到站后仍拒绝转向。新增正反方向评分检查；碰撞净空继续由完整外形检查保障。[旋转惩罚源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/twirling.cpp)、[终点转向源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)

## 旋钮实测行程

下表取正反两次动作中的最大值，单位 mm；原轴向关节上限 10 mm，独立验收余量 1 mm。

| 旋钮 | 全过程轴向峰值 | 释放阶段峰值 |
| --- | ---: | ---: |
| fr135 | 7.915 | 0.503 |
| fr2222 | 7.933 | 0.498 |
| fr4332 | 7.971 | 0.498 |
| fr12452 | 7.895 | 0.502 |
| fr20422 | 7.850 | 0.498 |
| fr25452 | 7.724 | 0.498 |

旋钮／按钮整段 3,129 帧几何检查通过，柜体最小采样净空 1.395 mm；目标接触采样最小有符号距离 -0.025 mm，满足原 1 mm 数值接触容差。摇杆另有 462 帧几何检查；停止后左臂最大回零误差小于 0.001 rad，工具距插口大于 1.50 m。四个抽拉最不利时刻图和旋钮、按钮、摇杆代表接触图均已查看。

**边界：** 顶点及开放 CAD 网格距离采样不能证明完整连续无碰撞；验收还结合了规划碰撞检查、实际行程、接触保护、释放／退出和终态。仅计划成功、抓取附件建立或 API 返回成功均不单独作为通过依据。

## 证据与失败保留

所有本轮证据位于 `log/retest_20261017/`，生成日志不提交 Git。早期已不存在的 `log/generator_regression_20261015/` 不作为本轮证据。

| 记录 | 结论 |
| --- | --- |
| `full_fixed_hold/` 的四个 `*_web.json`、`*_geometry.json` | A 部分通过：db1 7948 帧、dm1 8018 帧、ds2 7170 帧、ds3 6853 帧；该整批后来在 B 失败 |
| `generator_linear_full/` | 六个旋钮双向与四个按钮的 16 次动作、3129 帧几何通过；该整批后来在摇杆导航失败 |
| `rocker_heading/`、`heading_transition_geometry.json` | 最终导航评分下摇杆两次完整运行／停止通过；原失败转场的 389 帧几何通过 |
| `all_nav_heading/` | 最终评分下 11 个工位导航及 538 帧净空检查全部通过 |
| `full_final/` | 首个旋钮夹紧后 0.14 s 出现 2.86 mm 穿入，被保护中止 |
| `full_fixed_hold/` 的 B 部分 | 固定保持目标时第二旋钮拉出后单侧分离 0.102 mm，失败 |
| `generator_bounded_full/`、`tail_bounded/` | 第四／第六旋钮 API 成功，但独立验收抓到释放时 15.206／15.131 mm 轴向超程，仍记失败 |
| `generator_linear_full/frb_navigation.json` | 旧旋转评分导致 64.6 s 超时；修复后的同一路线成功记录见 `rocker_heading/` |

专项证据：`second_bounded_1/` 至 `second_bounded_3/` 保留第二旋钮三轮双向结果；`fourth_linear/` 保留插值修复后第四旋钮双向及 486 帧几何结果。`jaw_controller_trace.jsonl`、`jaw_release_overshoot.json` 保存旧夹爪实际／目标速度、位置；`linear_release_comparison.json` 保存修复后对照。源码及补丁快照以 `*_source.json`、`*_changes.patch` 留存。

控制包测试 **217 项**、Gazebo 包测试 **2 项**均无失败；最终导航、播种相关 Python 检查 **10 项**通过，A/B 模型检查通过。夹爪专项插值测试另在显式 C++17 配置下通过。

## 复现与麒麟更新

本机完整重跑（使用新的输出目录）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
python3 scripts/validate/validate_two_scene_web.py --phase all --navigate \
  --drawer-cycles 2 --out log/two_scene_acceptance_NEW
```

麒麟必须重建，`docker start/restart` 不会应用源码更新：

```bash
cd /root/proj/work01-gitee
git pull --ff-only
GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 bash ./start
```

## 日志空间管理

只归档已结束且没有打开句柄的日志，压缩后逐文件或解压内容 SHA-256 一致才移除未压缩副本。当前验收及失败证据保留。

- `log/archive/retest_after_cleanup_20261016.tar.gz`：71 个文件校验通过，约 168 MB 压至 42 MB；清理当时实测净释放约 117 MiB，清单 `cleanup_20261017.json`。
- 关闭后的 Gazebo 日志分别保存在 `log/archive/gazebo_default_20261017_1212.log.gz`、`gazebo_bounded_hold.log.gz`、`gazebo_linear_before_heading.log.gz`，对应校验与空间测量清单同目录保存。活动日志和正在使用的 VS Code 索引保留。

需要读取旧复测文件时：

```bash
tar -xzf log/archive/retest_after_cleanup_20261016.tar.gz -C log
```
