# 电气夹层抽拉柜仿真修复

## 范围

用户确认采用视觉仿真联动，所有抽屉的打开档位统一为 **0.05 m**。保留模型外观。

- `db1`、`dm1`、`ds2`、`ds3`：第一套末端双臂操作。
- `ds1`：打开档位同样为 0.05 m，但尚未找到可执行的双臂工作姿态，仍保留不可操作提示。

## 动作与修复

打开：底盘到工位 → 收杆并到工作姿态 → 双臂各向外张开 4 cm → 伸钩 → 向内扣合 → 伸支撑杆 → 联动拉出 5 cm。

关闭：在已打开位置准备 → 伸钩与支撑杆 → 联动推回 → 收杆 → 双臂归位。

- 左右张臂方向相反，并行执行；不再在扣手后重新恢复另一套姿态。
- 抽拉中双臂后移 35 mm，钩杆回缩 15 mm，支撑杆伸长 35 mm，支撑端保持靠墙；现有 120 mm 电缸行程足够。
- 双臂、杆和抽屉采用相同总时长；维护操作租约，控制器报错和取消不再被忽略。
- 位置订阅与插件 `SensorDataQoS` 匹配；必须取得实测位置，最终误差不得超过 3 mm。
- `ds2`、`ds3` 按实际窄把手间距重新求解，工位距离由 1.1 m 调整为 0.9 m；`dm1` 校正了 12 mm 横向偏差。
- Web 结果中的 `simulation_outcome_confirmed`、`final_position` 表示仿真轨道到位证据，不表示真实接触力拖拽。

## 验证方法

统一入口：

```bash
ROS_DOMAIN_ID=42 SCENE=electrical_mezzanine TOOLSET=A \
SCENES_CONFIG="$PWD/xczs_inspection_robot_control/config/scenes.yaml" ./run_all.sh
```

机器人位于指定柜的工位后，用 Web 入口验证打开、保持 5 秒、关闭，并检查其他抽屉无超过 3 mm 的串动：

```bash
python3 scripts/validate/validate_taught_drawer.py --control db1 \
  --out log/drawer_validation/2026-09-19/db1_web.json
```

`--control` 可选择 `db1`、`dm1`、`ds2`、`ds3`。验收器本身不负责导航。

局部画面检查可用 `scripts/tools/render_drawer_contacts.py`：它读取 Gazebo 实测 link 位姿和原始 STL，在本机已有 VTK 环境下生成诊断图，不截取桌面，也不修改模型；图中颜色仅用于区分工具与柜体。

生成日志、轨道读数和诊断图保存在 `log/drawer_validation/2026-09-19/`，不提交 Git。
