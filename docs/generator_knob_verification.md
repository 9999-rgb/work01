# 发电机层第二套末端旋钮验收

## 最后两个旋钮接近与提速复验（2026-09-25）

本轮只验证 `fr20422_knob`、`fr25452_knob`。Web 档位沿用外观定义：
`turned` 是正中间 0°，`center` 是斜放 45°；下表按 Web 外观标注。

- 最后两个旋钮直接规划至距夹持点 50 mm 的预夹持位，再沿轴线接近；取消先到 120 mm 准备位、再重新求解预夹持姿态的重复动作。候选分支仍需通过接近、拉出、转动、插回和退出的碰撞检查。
- 保留已验证的预夹持关节解；在关节余量允许时优先选择较短的接近分支，避免临近旋钮再次大幅翻腕。
- 夹爪先在净空内快速合拢 8 mm，再按实测位置小步接触。闭合目标不再因瞬时读数偏低而反向递减；滞后时保持上一目标，不继续累加深入。双侧夹持、终态及释放校验仍保留。
- `v5`、`v6` 连续切换旋钮时出现单侧夹爪先闭合再逐步退回，任务被正确判失败；这些记录不计入通过。上述闭合修正后的 `v7` 六次连续操作全部通过。

| 顺序 | 旋钮 | Web 目标 | 完整任务耗时 | 最终直线接近耗时 |
| --- | --- | --- | ---: | ---: |
| 1 | fr20422 | 0° | 70.2 s | 3.9 s |
| 2 | fr20422 | 45° | 82.5 s | 5.3 s |
| 3 | fr25452 | 0° | 65.2 s | 3.8 s |
| 4 | fr25452 | 45° | 76.3 s | 5.2 s |
| 5 | fr20422 | 0° | 64.3 s | 3.7 s |
| 6 | fr20422 | 45° | 92.8 s | 5.1 s |

此前 `v2` 单次完整流程约 110–114 s，准备位后的重复接近约 23 s。
本轮接近段单关节最大变化 8.13°，未复现约 180° 的翻转；六次均完成
拉出、转动、插回、释放及收臂，拉出峰值 7.64–8.35 mm，插回残差
0.452–0.473 mm，未检测到其他控件串扰。

1141 帧几何采样通过：固定柜体最小采样间隙 1.066 mm，夹爪与旋钮
最大采样交叠 0.024 mm；Gazebo 双侧接触门控读数最大 0.925 mm，
均在既有 1 mm 数值接触容差内。离散顶点采样不能证明连续全过程绝对无碰撞。
操作由 Web `/task/operate` 发起；工位使用仿真预定位，本轮不计作导航验收。

证据：`log/knob_short_approach/v7/` 中的六份任务 JSON、`motion.jsonl`、
`geometry.json`、`summary.json`。构建及统一入口启动通过；控制包测试
202 项通过，旋钮几何合同与 generator_plant/B 适配合同通过。源码验证对应
`9626d97`；2026-09-26 核对保存结果时工作树源码与该版本一致。

## 启动与复测

```bash
ROS_DOMAIN_ID=42 SCENE=generator_plant TOOLSET=B ./run_all.sh
scripts/validate/check_generator_knob_geometry
scripts/validate/check_adapter_contract --instance-id generator_plant --toolset B
scripts/validate/validate_generator_knobs --controls fr135_knob fr2222_knob fr4332_knob fr12452_knob fr20422_knob fr25452_knob --output /tmp/generator_knobs.json
```

验收要求：每个旋钮完成左→中→右→中→左；Web 返回物理终态确认、释放、退离和回位成功；旁路控件无状态跳变。几何校验不能替代 Gazebo 实测。

## 修复范围

- 保留模型可见外观；工具碰撞代理、柜体碰撞代理与规划场景保持一致。
- 旋钮操作实际开合双钳口；双侧接触成立后才允许耦合，旋转中持续检查接触与穿入深度。
- 使用 Gazebo 实体相对机器人姿态校准操作坐标，并同步规划场景，避免定位修正导致操作与碰撞模型错位。
- 退离后恢复旋钮及其按钮的规划碰撞，再规划回位；恢复通知必须匹配当前租约，操作坐标保持锁定。
- 旋转前检查接近、旋转、退离整个路径；任何候选都无法完整通过时拒绝开始接近。
- `fr12452_knob`、`fr20422_knob` 后方存在障碍物。原 1.4 m 站位导致收拢左臂与障碍物接触；这两个旋钮改为 1.0 m；同高度的 `fr25452_knob` 也使用 1.0 m 以获得完整可达路径。
- 三个低位近站旋钮（另含 `fr25452_knob`）设置 `rotary_retreat_distance: 0.04`，相对 0.004 m 夹持偏置直线退离 36 mm，再执行回位。其余旋钮保持原退离距离。运行、预检和异常退离使用相同参数。

## 实测记录（2026-09-17）

- `fr135_knob`：四段往返通过，证据 `/tmp/generator_proxy_trial.json`。
- `fr2222_knob`：四段往返分段通过，证据 `/tmp/generator_proxy_remaining.json`、`/tmp/generator_fr2222_finish.json`。中途服务停机造成的未完成任务不计入通过。
- `fr4332_knob`：四段往返通过，证据 `/tmp/generator_four_finish.json`。
- `fr12452_knob`：新站位四段往返通过，证据 `/tmp/generator_short4_verified.json`。该轮通过临时场景出生点启动，不计作第三组到第四组的导航验收。
- 第四组到第五组导航通过，证据 `/tmp/generator_last_two_verified.json`。
- `fr20422_knob`：2026-09-18 四段往返通过，无同组按钮串扰；证据 `log/generator_knob_validation/2026-09-18/generator_resume5_verified.json`。
- 第五组到第六组原站位导航通过；第六组在接近前因无完整路径被拒绝，旋钮未转动。证据 `log/generator_knob_validation/2026-09-18/generator_resume6_verified.json`。近站位四段往返已通过，无串扰；证据 `log/generator_knob_validation/2026-09-18/generator_resume6_close_verified.json`。

接触审计：`/tmp/generator_close_station_contact.jsonl`。第四组成功任务中工具对固定柜体的最小采样间隙约 1.3 mm；该审计为离散网格采样，不能证明连续全过程的间隙。

构建：`colcon build --packages-select xczs_inspection_robot_control --symlink-install` 通过。
回归：`colcon test --packages-select xczs_inspection_robot_control xczs_inspection_robot_gazebo --return-code-on-test-failure` 通过；控制包 194 项，0 错误、0 失败。

`/tmp` 文件是当前机器的运行证据，可能在清理或重启后丢失；复测时使用上面的命令生成新证据。


## 前一轮结果与后续复验（2026-09-18）

- 默认 `ROS_DOMAIN_ID=42 SCENE=generator_plant TOOLSET=B ./run_all.sh` 启动通过。
- 默认出生点到第三组导航通过；地图修复前，第三组到第四组新站位在约 62 s 后导航超时，机械臂保持归位。该失败属于修复前记录；地图修复后的通过证据见下节。证据 `log/generator_knob_validation/2026-09-18/generator_station_navigation_verified.json`。
- 第五、六组八段成功操作的采样最小柜体间隙约 1.8 mm，钳口/方板最大采样交叠约 0.055 mm，未采到工具进入固定柜体。汇总：`log/generator_knob_validation/2026-09-18/contact_summary.json`。
- 2026-09-17 的 `/tmp` 原始记录已随环境清理；上节保留历史实测摘要，2026-09-18 的证据另存于项目忽略的 `log/` 下。

- 旋转中取消实测：第三组旋转时发送取消，后端约 27 s 完成恢复并释放任务占用；双臂 14 关节最大回位误差约 0.001 rad，钳口位置约 ±0.0015 mm，确认张开。证据 `generator_cancel_verified.json`、`generator_cancel_joint_evidence.json`（同一日志目录）。Web 的 `backend_termination_confirmed` 已变为 true，但消息文字仍保留最初 5 s 宽限期内未确认的提示，后续已修复该提示刷新，见下节。
- 取消后第三组再次执行左→中→左完整通过，含释放、回位和无串扰检查；证据 `generator_after_cancel_verified.json`（同一日志目录）。


## 导航与取消提示修复

- 发电机层 CPU 雷达扫描物理碰撞体，原导航地图却来自外观 STL；两者约 6.7 万个栅格不同，运行中出现米级定位偏差。地图生成工具现从该场景的静态碰撞盒生成地图，其他场景仍使用原 STL 流程，不修改模型外观。
- 进入较近的发电机工位时先调整 Y，再沿 X 横移，避免保留远侧站位的 Y 穿过后方设备区域。其余场景保持原规则。
- 默认启动可能优先使用资产库地图。本机已备份并同步 `jiang/data/assets/scene/generator_plant/maps/`；运行日志确认加载 841×568 新地图。旧副本备份在日志目录 `original_asset_map/`。重新导入旧资产后需要同步地图，不能只重建源码地图。
- 默认出生点→第三→第四→第五→第六组导航全部通过；证据 `log/generator_knob_validation/2026-09-18/generator_asset_map_navigation.json`。
- 取消超出宽限期后仍保留任务占用；收到后端终止确认时，同步更新 `message`、`failure_reason`，不再残留“未确认”的提示。没有后端确认时仍显示未确认。
- 地图、任务管理和路线回归共 138 项通过；场景校验和 Nav2/控制包构建通过。

- 取消提示实测通过：第六组实际旋转中取消，后端结束后自动显示 `Cancellation completed; backend termination confirmed.`，`backend_termination_confirmed=true` 且 `reservation_active=false`。证据 `generator_cancel_message_verified.json`（同一日志目录）。
- 取消后第六→第五→第四→第三组反向导航全部通过，证据 `generator_return_navigation.json`（同一日志目录）。至此此前未完成的导航与取消提示两项均已复验通过。
