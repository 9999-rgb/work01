# 发电机层第二套末端旋钮验收

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


## 本轮边界（2026-09-18）

- 默认 `ROS_DOMAIN_ID=42 SCENE=generator_plant TOOLSET=B ./run_all.sh` 启动通过。
- 默认出生点到第三组导航通过；第三组到第四组新站位仍在约 62 s 后导航超时，机械臂保持归位。旋钮操作通过不代表这段导航已修复。证据 `log/generator_knob_validation/2026-09-18/generator_station_navigation_verified.json`。
- 第五、六组八段成功操作的采样最小柜体间隙约 1.8 mm，钳口/方板最大采样交叠约 0.055 mm，未采到工具进入固定柜体。汇总：`log/generator_knob_validation/2026-09-18/contact_summary.json`。
- 2026-09-17 的 `/tmp` 原始记录已随环境清理；上节保留历史实测摘要，2026-09-18 的证据另存于项目忽略的 `log/` 下。

- 旋转中取消实测：第三组旋转时发送取消，后端约 27 s 完成恢复并释放任务占用；双臂 14 关节最大回位误差约 0.001 rad，钳口位置约 ±0.0015 mm，确认张开。证据 `generator_cancel_verified.json`、`generator_cancel_joint_evidence.json`（同一日志目录）。Web 的 `backend_termination_confirmed` 已变为 true，但消息文字仍保留最初 5 s 宽限期内未确认的提示，这一显示问题尚未修改。
- 取消后第三组再次执行左→中→左完整通过，含释放、回位和无串扰检查；证据 `generator_after_cancel_verified.json`（同一日志目录）。
