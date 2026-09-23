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
| docs/build | 遗留的 colcon 构建树（约 300 MB / 2518 文件），已被 `.gitignore` 排除，可直接删除 |
| log | 自动生成的运行日志与验收证据，不提交 Git |

场景来源按「显式优先」决出：显式 `SCENE=` 内置场景时用 `xczs_inspection_robot_control/config/scenes.yaml`（实例注册为同目录 `cabinet_instances.yaml`），否则启动脚本把资产库里的同名场景（`jiang/data/assets/scene/<name>/scenes.yaml`）经 `--print-env` 注入，已被显式设置的环境变量不被覆盖。注意「显式内置」只保证启动那一刻：切末端套装重启子栈时下发的是**活动场景**及其 scenes.yaml，而场景解析是 kind+name 查表，两个内置场景名在资产库里都有同名目录，因此切换后子栈实际读的是资产库那份。旧三柜场景已移除。共享柜体配置仍被适配器默认值、资产导入和通用测试使用，不属于可删除的运行垃圾。

验收说明：[按钮与摇杆专项](docs/generator_aux_verification.md)、[两场景整理与回归](docs/two_scene_verification.md)、[抽拉柜](docs/drawer_simulation_verification.md)、[发电机层](docs/generator_simulation_verification.md)。工位预定位测试不等于自主导航验收。

- [发电机层提速与 Web 操作列表验收](docs/generator_speed_verification.md)：摇杆快速插入、六旋钮两档与菜单清理。
- [定位重播种与场景地图同步验收](docs/localization_realign_verification.md)：长距离导航的定位发散恢复、工位门不掩盖错位、资产库地图不再过期。
- [场景随末端套装切换验收](docs/toolset_scene_switch_verification.md)：切场景后再切套装不再回滚、不再载入旧场景地图。
- [精确停靠断困加力](docs/docking_breakaway_verification.md)：停靠出现「有指令、车不动」时加力突破；附实测签名与未能复现的边界。

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
