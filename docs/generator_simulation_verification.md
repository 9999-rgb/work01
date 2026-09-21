# 发电机层新模型接入与仿真验证

最新按钮与摇杆专项修复、验收见 [按钮与摇杆验收](generator_aux_verification.md)。下列数值保留初次接入记录；旋钮全量复测目前按用户要求暂停。

## 范围

- 源模型：`仿真场景20260831/发电机层2026.09.20`。运行场景复用其 18 个网格；第二套末端的可见网格与外观不变。
- 六个旋钮：原有滑动轴拉出 8 mm，沿源模型正方向转至 45° 或回到 0°，插回后松开。
- 四个独立按钮：第二套右侧旋钮末端的原有按压头，机械臂前送按下、回退释放。源 URDF 没有独立的按压头伸缩关节。
- 连续旋转插口 `frb`：第二套左侧摇杆，Web 开始／停止；停止后退出。方孔源网格有 30° 偏角，插入前根据当前插口角度与腕部姿态补偿。
- 六个旋钮内部轴向状态仅用于反馈，不作为独立按钮操作。

## 已验证

统一入口：`SCENE=generator_plant TOOLSET=B ./run_all.sh`。本轮隔离运行使用 `ROS_DOMAIN_ID=42`，Web API 为 `http://127.0.0.1:8090`。

机器人通过 `scripts/tools/preposition_base.py` 放到工作站，再通过 Web API 执行动作；本报告不构成整段自主导航验收。

| 控件 | 有效按下峰值 | 释放残余位移 | 结果 |
| --- | ---: | ---: | --- |
| fbutton1 | 4.054 mm | 0.041 mm | 通过 |
| fbutton2 | 4.381 mm | 0.037 mm | 通过 |
| fbutton3 | 4.186 mm | 0.045 mm | 通过 |
| fbutton4 | 4.028 mm | 0.048 mm | 通过，另完成一次重复操作 |

`fbutton4` 重复操作的网格顶点审计：按压头／按钮最大采样穿入约 0.008 mm，机器人到柜体碰撞代理最小间隙约 15.5 mm。侧视实测位姿渲染已核对。盒体使用解析有符号距离；开放 CAD 网格仅计算无符号距离，避免把不可靠的面法向误判为内部。顶点采样不是所有三角形、所有时刻无碰撞的数学证明。

`fr135_knob` 最新版本 0°→45°→0° 往返通过，拔出峰值分别约 7.65 mm 和 8.20 mm，终态角度分别为 0.785398 rad 和接近 0 rad。均验证插回、松开、撤离、收臂。前三个旋钮释放后的目标离面距离为 4 cm；三个低位旋钮为 3 cm（相对夹持点直线退出 26 mm），均保留超过夹持重叠厚度的净空，避免冗长退路进入不可达分支。

连续摇杆已通过两轮：第一轮转动超过一整圈；第二轮从非零终态重新插入，转动后再次停止。两轮峰值转速分别为 0.143、0.142 rad/s；停止后左臂回到 home 的最大关节误差均小于 0.001 rad，摇杆端头距插口约 1.515 m。后台终止确认、机器人占用释放、插口碰撞恢复均完成。

校正工位跟踪偏差后，转动时末端横向中心误差约 0.002 mm，网格顶点采样穿入约 0.05 mm。采用实测位姿侧视检查配合外观。

## 物理稳定性处理

新按钮及连续插口原来连接到超大质量的世界固定柜体。运行模型改为等效地直接锚定世界，保持零位姿、运动轴和全部可见网格，避免不良质量比使约束求解发散。连续插口保留网格碰撞，并为约 0.05 mm 的网格离散配合误差设置小量接触容差和受限纠正速度。按实际 2 ms 物理步长匹配转动惯量、阻尼和耦合刚度，避免显式力更新数值发散。

插入与传动复用现有仿真耦合机制：在有效任务租约下，插入前核验实测横向偏差、轴线和方孔角度，满足门槛后暂时屏蔽插口自身配合网格；插入到位并通过抓取服务后才建立单轴柔性力矩耦合。其他柜体碰撞继续有效；停止退出且任务结束后恢复该网格碰撞。转角由物理力矩驱动，不直接写入目标关节角。这是仿真传动近似。

摇杆工位让左肩朝向插口，腕部绕插入轴偏转 45° 留出避碰空间；转子在自由空间补偿腕部转角、源方孔偏角及插口当前角度，未对准则禁止插入。

## 检查与证据

验收脚本使用当前工作空间的 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

- `scripts/validate/check_adapter_contract --instance-id generator_plant --toolset B`
- `python3 scripts/validate/check_generator_knob_geometry`：旋钮轴向合同、按压头源网格端点、摇杆方孔角度补偿。
- `colcon test --packages-select xczs_inspection_robot_control xczs_inspection_robot_gazebo`
- `python3 -m pytest -q jiang/tests/test_cabinet_client.py jiang/tests/test_task_runner.py jiang/tests/test_profile_contract.py`
- 逐项 Web 动作采样：`scripts/validate/validate_generator_operation.py`。
- 实测网格审计：`scripts/validate/generator_contact_audit.py`。
- 侧视／斜视回放：`scripts/tools/render_generator_contacts.py`。

本地证据目录：`log/generator_validation/20260920/`。失败尝试保存在 `attempts/`，不计入通过结果。该目录为生成证据，不提交 Git。

## 六个旋钮双向回归

六个旋钮的 0°→45°→0° 共 12 次 Web 操作全部通过。低位三个旋钮在回插轨迹中补偿 0.5 mm 轴向跟踪偏差，维持原有 1 mm 到位阈值，不增加往返小动作。源码构建完成后 194 项 C++ 检查通过。

| 旋钮 | 转到 45° 拔出峰值 / mm | 回到 0° 拔出峰值 / mm | 结果 |
| --- | ---: | ---: | --- |

| fr135_knob | 7.654 | 8.205 | 通过 |
| fr2222_knob | 7.689 | 8.197 | 通过 |
| fr4332_knob | 7.635 | 8.249 | 通过 |
| fr12452_knob | 7.773 | 8.178 | 通过 |
| fr20422_knob | 7.856 | 8.493 | 通过 |
| fr25452_knob | 7.840 | 8.233 | 通过 |
