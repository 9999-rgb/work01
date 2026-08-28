# scripts — 可执行脚本

## 总体介绍

面向用户与 CI 的验收「门」+ 开发工具集合：`validate/` 放验收/校验脚本，`tools/` 放
开发辅助工具。脚本用 `parents[N]` 定位工作区根并把自己的 `jiang/` 加入 `sys.path`，
直接读取跨层 YAML 合同并调用任务层 API，把「配置合同是否一致、仿真闭环是否通过」
变成可一条条执行的检查。

## 模块架构

- `scripts/validate/` —— 验收/校验脚本：
  - `check_adapter_contract` 跨文件 profile 合同校验（不起 ROS）
  - `check_cabinet_model` 柜体 Xacro/物理关节/插件/目录静态校验（`--asset` 资产模式）
  - `check_scene_config` 场景目录（`scenes.yaml`）与 Nav2 地图/参数校验
  - `validate_cabinet_simulation` 确定性底层控件检查（不跑 Nav2，需先预置位）
  - `validate_cabinet_web` Web→任务层→ROS 2→Gazebo 完整闭环
  - `validate_recording_replay` 录制/回放 HTTP 合同校验
- `scripts/tools/` —— 开发工具：
  - `xczs_import_asset` 资产 CLI 导入（`--select` / `--print-env` / `--list`）
  - `preposition_base.py` 预置位（`/set_entity_state` 遥移到工位 + 发布 AMCL 初值）
  - `classify_controls.py` 逐控件可达性分类（订阅 `control_catalog`，纯规划不执行）
  - `generate_scene_maps.py` 由 STL 场景网格生成 Nav2 地图（PGM+YAML）
  - `package_asset_samples.sh` 把样例资产打成上传用 zip
  - `cabinet_validation_targets.py` 控件目标选择纯函数（被 validate 与测试复用）

## 功能介绍

- **静态合同门**（不起 ROS）：`check_adapter_contract` 校验
  `cabinet_robot_adapter.yaml` / `cabinet_instances.yaml` / `cabinet_controls.yaml` /
  `cabinet_scene.yaml` / `cabinet_pose.yaml` 与 MoveIt `kinematics.yaml`；`check_scene_config`
  证明带 padding 的机器人足迹落在自由格内；`check_cabinet_model` 验物理与插件合同。
- **仿真闭环验收**：`validate_cabinet_simulation` 走 `OperateCabinetControl` Action、
  `control_catalog` 话题与 `get_planning_scene` 服务；`validate_cabinet_web` 打
  `http://127.0.0.1:8090` 的 `/task/navigate`、`/task/operate`，`--exhaustive` 全量。
- **录制/回放验收**：`validate_recording_replay` 覆盖 `/replay/*` 端点与只读互锁（409）；
  `--runtime` 只发一次零速度请求确认回放期间后端返回 409，任务重演须显式加
  `--allow-motion --task-recording`。
- **开发工具**：`preposition_base.py` 让物理验收前把底座放到目标控件工位；
  `generate_scene_maps.py` 生成已提交的 Nav2 地图；`package_asset_samples.sh` 产出
  `/assets/import` 端点可直接接收的 zip；`xczs_import_asset` 导入含 `manifest.yaml`
  的资产并记入 SQLite `assets` 表。

## 与项目的关系

- **依赖**：消费 `jiang/control_gateway`（`profile_contract`、`asset_manifest`）纯逻辑；
  `xczs_inspection_robot_control/config/*.yaml` 是跨层合同，按硬编码路径读取；
  `xczs_inspection_robot_moveit_config/config/kinematics.yaml`、description 的 meshes。
- **被消费**：作为用户/CI 的验收门；`xczs_import_asset` 在 bridge 启动时也会被调用；
  生成的地图被 Nav2 `map_server` 服务；打包 zip 经 8090 Web 控制台上传。
- **在架构里的位置**：处于三层适配架构之外的上层——用 `validate_cabinet_web` 把
  通用任务层（`jiang/control_server.py`）→ 机器人适配层 → 场景适配层整条启动链
  串起来做端到端验收；`check_*` 系列在启动前静态守住跨层 YAML 合同。
