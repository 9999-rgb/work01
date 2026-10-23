# 发电机层第二套末端旋钮验收

## 当前实现与遗留问题（2026-09-30）

旋钮现按“夹持—拉出 8 mm—独立转盘旋转—插回—松夹—退出—转盘归零”执行。
旋转轴为 `r_rotbtn_rotate_joint`，旋转期间机械臂保持姿态；外观 0° 对应 `turned`，
45° 对应 `center`。下文 09-25 及更早的接近路径、耗时数据属于对应历史版本。

09-28 六旋钮双向共 12 次成功，另有夹持失败后重试记录；09-30 fr12452 双向通过，
转盘采样行程 44.737° / 44.916°、机械臂最大单关节变化 0.000041° / 0.017°。
但 09-30 同样复现过旋转前夹爪不到位，偶发夹持问题未消除。
证据与边界见 [两场景回归](two_scene_verification.md)，当前转盘专项检查器为
`../scripts/validate/check_knob_rotor_motion.py`。近期工位预定位不计导航验收。

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

## 2026-10-22 夹爪末段减速与续测核对

第一旋钮 `fr135_knob` 在夹持阶段曾触发预夹持扰动保护：最大角速度
0.082248 rad/s，任务失败。将最终接触阶段的夹爪轨迹时长由 0.2 s
改为 0.4 s，保留闭合行程和扰动保护阈值。随后该旋钮转动、复位专项通过。

现存证据 `log/retest_solver_profile_20261022_144329/full3/` 显示，后续
六个旋钮共 12 项转动／复位通过；全量动作汇总及切回 A 后的 dm1 复测
均通过。四柜与发电机几何检查分别为 8365、9986、9039、8274、4076 帧，
切回 A 后另有 5284 帧通过。核对时 36 项源码／产物哈希与该轮记录一致。
本次续接先复核这些已完成结果，不重复从第一柜开始运行。

验收边界：该轮 Gazebo 加载的是 `/tmp/xczs-profile-install/` 下的仿真
插件，正式安装目录当时仍是旧产物，不能将其表述为默认安装环境已完成
全量验收。世界连续性汇总文件为空，原始 JSONL 最后一行截断；7488 条
完整记录中世界进程身份一致、未见大于 0.5 s 的采样时钟倒退，但不能
据此声称原逐消息连续性检查完整通过。保留原始文件，不覆盖失败证据。

后续修复按失败旋钮／阶段续测；专项通过后才进行全量回归。实际运行的
插件、控制程序和启动环境须与验收对象一致，不仅检查源码。

续接检查已完成控制包及 Gazebo 包的正式目录编译，均成功；夹爪保持、
旋钮夹持合同及控制器运动学三项 CTest 通过。当前世界仍加载原临时插件，
此次编译不会替换已经加载的库。正式安装产物的启动和运行验收仍待进行。

## 正式安装环境全量覆盖续测（2026-10-22—23）

使用正式编译产物通过 `run_all.sh --web` 启动，确认 Gazebo 实际加载
项目 `build/xczs_inspection_robot_gazebo/` 中的插件，不再使用临时安装
插件。证据目录：`log/retest_official_20261022_resume/`。

- 四柜各两轮开关及重复目标：32 项通过，记录在 `full/`。
- 六旋钮转动、复位：12 项通过。前四个在 `full/`，最后两个在 `continued/`。
- 四按钮：4 项通过，记录在 `continued/buttons/`。
- 摇杆整圈停止及短转停止：2 项通过，记录在 `continued/full/`；均等待
  后端终止确认，并验证实际退出插口和收臂。
- B→A 热切换后 dm1 开关及重复目标：4 项通过，记录在
  `continued/post_switch_A/`。

合计 54 项成功动作检查，9 份几何检查共 39,508 帧全部通过（包含中断前
采样）。切回 A 后 dm1 支撑杆最大转角 0.5715°。续测运行日志未检出
SIGSEGV／SIGABRT；结束时服务保留在 electrical_mezzanine／A 就绪状态。

本轮不是连续无中断验收：第五旋钮操作途中监督器收到 SIGTERM，Zenoh
退出状态为 143，Web 连接断开，原 `full/summary.json` 保留失败状态。
终止信号来源未确定，不能据此认定旋钮控制代码存在故障。恢复服务后从
第五旋钮续测，没有重跑已通过项目。原失败记录不覆盖；上面的 54 项是
分段证据合并覆盖，不能解释为原始全量脚本一次通过或世界连续性通过。

本轮未复现新的机械操作故障，没有追加功能或调整安全阈值。夹爪末段
0.4 s 修改在正式安装环境中完成六旋钮往返验证。离散几何采样仍不代表
连续全过程绝对无碰撞。

## 再次连续全量复测（2026-10-23）

证据：`log/retest_full_20261023/`。本轮全量脚本及切回 A 后的复验均
正常退出，未中断、未重试动作、未新增运行代码修改。54 项动作检查、
16 次自主导航全部通过：四柜 32 项、六旋钮往返 12 项、四按钮 4 项、
摇杆整圈与短转停止退出 2 项，以及 B→A 后 dm1 的 4 项。

六份几何检查共 42,445 帧通过，8 张记录时刻检查图已查看。切回 A 后
dm1 支撑杆最大转角 0.2339°。世界连续性监测收到 30,140 条时钟消息，
PID 895162、start_ticks 47052320 始终一致，无进程替换或超过 0.5 s
的时钟倒退；完整汇总保存于 `world_continuity.json`。控制源码、控制
程序及正式仿真插件三个 SHA-256 在本轮前后保持一致。

本轮未复现新的功能故障，无需追加修复。运行日志崩溃检查通过；结束后
保留仿真在 electrical_mezzanine／A 就绪状态，因此不包含最终关闭验收。
有限采样通过不代表对所有未来运行状态的保证。

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
