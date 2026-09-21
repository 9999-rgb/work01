# 巡操机器人仿真

统一入口：`./run_all.sh`，Web：`http://localhost:8090/monitor.html`。
未保存资产选择时默认启动电气夹层；Web 可切换电气夹层、发电机层及 A/B 末端。

- 电气夹层 / A：db1、dm1、ds2、ds3 抽拉 5 cm；ds2/ds3 不使用支撑杆。ds1 暂不可操作。
- 发电机层 / B：六个旋钮拉出 8 mm、转到 0° 或 45°、插回；四个按钮用右侧原有按压头；frb 用左侧摇杆，Web 开始／停止并退出。
- 场景配色：抽拉柜蓝色、旋钮橙色、按钮绿色、摇杆插口紫色。灰色实体不表示已支持操作。

## 启动与构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
./run_all.sh
# 指定发电机层
SCENE=generator_plant TOOLSET=B ./run_all.sh
```

## 文件位置

| 目录 | 内容 |
| --- | --- |
| xczs_inspection_robot_control | 控制节点、运动逻辑、场景合同与工位配置 |
| xczs_inspection_robot_description | 运行模型、网格与场景可视材质 |
| xczs_inspection_robot_gazebo | 仿真插件与世界 |
| xczs_inspection_robot_bringup | ROS 启动文件 |
| xczs_inspection_robot_interfaces | 消息、服务和动作 |
| xczs_inspection_robot_moveit_config / xczs_inspection_robot_nav2 | 机械臂规划 / 导航 |
| jiang | Web、HTTP、任务编排、录制回放；tests 为回归测试 |
| scripts/validate | 验收脚本 |
| scripts/tools | 开发、工位校准与诊断工具 |
| model / 仿真场景20260831 | 原始机器人、末端与用户场景资产，保留溯源；COLCON_IGNORE 排除源导出工程 |
| docs | 架构、使用约定与验收说明 |
| log | 自动生成的运行日志与验收证据，不提交 Git |

场景唯一运行目录为 `xczs_inspection_robot_control/config/scenes.yaml`，实例注册为同目录 `cabinet_instances.yaml`。旧三柜场景已移除。共享柜体配置仍被适配器默认值、资产导入和通用测试使用，不属于可删除的运行垃圾。

验收说明：[按钮与摇杆专项](docs/generator_aux_verification.md)、[两场景整理与回归](docs/two_scene_verification.md)、[抽拉柜](docs/drawer_simulation_verification.md)、[发电机层](docs/generator_simulation_verification.md)。工位预定位测试不等于自主导航验收。

- [发电机层提速与 Web 操作列表验收](docs/generator_speed_verification.md)：摇杆快速插入、六旋钮两档与菜单清理。

## 两场景动作回归

启动仿真后，在相同 ROS_DOMAIN_ID 下执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/validate/validate_two_scene_web.py --out log/two_scene_validation/latest
```

该脚本会操作仿真机器人：经 Web 切换场景和末端，在预定位工位执行四柜重复开关、六旋钮往返、四按钮按压、摇杆启停和再次插入。`--phase electrical` 或 `--phase generator` 可单独复测某一层。开启鉴权时通过 `XCZS_CONTROL_TOKEN` 传入已有登录 token。

只复测按钮和摇杆（四按钮各两次、摇杆整圈停止及非零角度再次插入）：

```bash
python3 scripts/validate/validate_two_scene_web.py --phase generator_aux --out log/generator_aux_validation/latest
```

`--phase generator_buttons` 只测四个按钮；`--phase generator_rocker` 只测摇杆两轮。
