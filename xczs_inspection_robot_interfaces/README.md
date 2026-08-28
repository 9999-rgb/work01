# xczs_inspection_robot_interfaces

## 总体介绍

本包是柜体操作相关接口的唯一来源：8 个 `msg/` / `srv/` / `action/` 定义，把
「操作一个柜体控件」这一跨层合同建模成一组类型——按钮/旋钮/开关的命令、操作租约
互斥、抓取点与末端工具切换都在这里定义。位于依赖最底层，接口一改，C++ 编译与
Python 侧同步受影响。

## 模块架构

- **内部组织**：`action/`（2 个）、`msg/`（3 个）、`srv/`（3 个）；`package.xml` 与
  `CMakeLists.txt` 描述构建与依赖。
- **生成**：由 `ament_cmake` + `rosidl_default_generators` 生成 C++ 头与 Python
  模块，消费方在编译期/运行时引用。
- **依赖**：仅 `action_msgs`、`geometry_msgs`、`std_msgs` 三个标准包，无自研运行时依赖。
- **协作方式**：本包只提供契约、自身不运行进程；消费方在 CMake / Python import
  中引用生成代码，接口变更会同时影响 C++ 编译与 Python 侧。

## 功能介绍

- `action/OperateCabinetControl.action`：柜体控件通用操作（`COMMAND_PRESS` /
  `COMMAND_SET_STATE` / `COMMAND_SET_POSITION` / `COMMAND_TOGGLE`），19 级错误码、
  物理结果确认标志（`physical_outcome_confirmed` 等）与分段诊断（`failure_reason`）。
- `action/PressCabinetButton.action`：按钮按压专用，反馈含按压/释放验证阶段。
- `msg/CabinetControl.msg`：控件目录条目——类型（按钮/旋钮/开关/门）、支持命令位掩码、
  力参数、所需末端（`required_toolset`）、可达性与适配器校验（`operable` /
  `adapter_validated`）。
- `msg/CabinetControlCatalog.msg`：控件目录（`CabinetControl[] controls`）。
- `msg/CabinetControlState.msg`：控件实时状态（位置/速度/归一化位置/是否触发/运动中）。
- `srv/ManageOperationLease.srv`：操作租约（`ACQUIRE` / `RENEW` / `RELEASE`），全局操作互斥。
- `srv/SetCabinetGrasp.srv`：设置/释放抓取点（`attach` 布尔），返回 `distance`。
- `srv/SwitchToolset.srv`：末端 A/B 无损切换，`expected_generation` 提供乐观并发保护。

## 与项目的关系

- **谁消费**：`xczs_inspection_robot_control`（C++，`src/cabinet_button_operator.cpp`
  服务上述两个 action 并作为租约服务客户端——`ManageOperationLease` 由
  `operation_lease_coordinator.cpp` 提供；另发布 `CabinetControlCatalog`）；
  `xczs_inspection_robot_gazebo`（`src/cabinet_state_plugin.cpp` 按控件发布
  `CabinetControlState`）；`jiang/` 任务层（`control_gateway/cabinet_client.py` /
  `ros_node.py` 发送 operate action 与 `SwitchToolset` 请求，
  `scripts/toolset_supervisor.py` 切换末端）；`scripts/tools/classify_controls.py` 读目录。
- **依赖方向**：位于依赖最底层，单向 `interfaces ← control ← gazebo`；`bringup` 只组装。
- **三层适配定位**：接口是跨层合同——`CabinetControl` 的 `required_toolset` / `operable`
  字段与 `xczs_inspection_robot_control/config/` 场景适配 YAML 呼应；改动接口须全量
  重建，并跑 `colcon test` 与 `pytest jiang/tests/`。

结构总览见 `../docs/architecture.md`。
