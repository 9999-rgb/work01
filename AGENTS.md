# 项目开发规范
- 优先在github上找类似的成熟的功能进行复用，且学习他们的架构。
- 代码编写必须规范、清晰并保持可维护性。文件摆放也要规范，不要到处乱放。切记要懂得模块化，结构清晰方便复用，可读性要强。
- 优先使用 ROS 2 的 `rclcpp`、`rclpy` 及社区成熟方案，避免重复造轮子。
- 节点间通信统一使用 ROS 2 的 topic、service 或 action，不使用裸 socket。
- 不准更改模型物理外观，非要改的话要向我询问。
- 遇到不明确的需求或技术问题时，先向用户确认再实施。
- 引入第三方依赖、插件或复杂方案前，先向用户确认。
- 功能节点优先使用 C++ 开发，仅在 C++ 不适合时使用 Python。
- 无法做到的事情请不要隐瞒，直接告诉我，我会尝试完成。
- 需要新的技术请告诉我，我会去查找对应资料并且引入。

## 文件放置规范

- 验收/校验脚本放入 `scripts/validate/`；开发工具脚本放入 `scripts/tools/`。
- 自定义消息、服务、动作（msg/srv/action）放入
  `xczs_inspection_robot_interfaces`。
- 启动文件放入 `xczs_inspection_robot_bringup/launch/`。
- Gazebo 插件与仿真世界（worlds）放入 `xczs_inspection_robot_gazebo`。
- 机器人模型与网格（URDF/Xacro、meshes）放入
  `xczs_inspection_robot_description`。
- 控制节点与纯逻辑头放入 `xczs_inspection_robot_control`（`src/`、`include/`）。
- 场景适配 YAML（instances/scene/controls/adapter/pose）留在
  `xczs_inspection_robot_control/config/`——它们是跨层合同，按硬编码路径读取。
- 通用任务层（Web/HTTP/SSE/任务管理/录制回放）放入 `jiang/`。
- 不提交 `build/`、`install/`、`log/`、Python 缓存和导出日志等生成文件。

## 项目约定

- 项目说明文档使用中文，命令、话题名称、文件名和代码标识符使用英文。
- ROS 2 启动文件统一使用 Python launch。
- 项目统一启动入口为 run_all.sh
- 修改源码后必须重新编译，并验证统一启动入口能够正常运行。

## 版本管理

- 全程使用 Git 规范管理代码。
- 提交前完成必要的构建、格式和运行检查。
- 提交信息应准确描述本次变更。

## 当前项目索引（2026-09-30）

- 用户启动入口为根目录 `./run_all.sh --web`，调用 `jiang/start_xczs_bridge.sh`；
  bringup launch 是 ROS 子栈入口，不能替代 Web 启动链。
- 现役场景为 `electrical_mezzanine`（末端 A）和 `generator_plant`（末端 B）。
  当前能力、构建命令及文档入口见 [README.md](README.md)。
- 旋钮采用独立转盘；档位 ID `turned` / `center` 分别对应外观 0° / 45°。
- 旧 `docs/build/` 和八个一次性脚本已清理，依据见
  [清理记录](docs/cleanup_20260930.md)。当前 `build/`、`install/`、源模型和运行数据保留。
- 验收结论须保留日期与范围；遗留夹持问题及当前测试结果见
  [两场景回归记录](docs/two_scene_verification.md)。
