# 项目开发规范

- 代码编写必须规范、清晰并保持可维护性。切记要懂得模块化，结构清晰方便复用。
- 优先使用 ROS 2 的 `rclcpp`、`rclpy` 及社区成熟方案，避免重复造轮子。
- 节点间通信统一使用 ROS 2 的 topic、service 或 action，不使用裸 socket。
- 遇到不明确的需求或技术问题时，先向用户确认再实施。
- 引入第三方依赖、插件或复杂方案前，先向用户确认。
- 功能节点优先使用 C++ 开发，仅在 C++ 不适合时使用 Python。

## 文件放置规范

- 可执行脚本放入 `scripts/`。
- 自定义消息放入 `msg/`。
- 启动文件放入 `launch/`。
- 配置文件放入 `config/`。
- 其他文件按照 ROS 2 功能包规范分类存放；无法确定时先询问用户。
- 机器人模型、网格和仿真环境统一放入
  `xczs_inspection_robot_description`。
- 控制节点和统一启动入口统一放入
  `xczs_inspection_robot_control`。
- 不提交 `build/`、`install/`、`log/`、Python 缓存和导出日志等生成文件。

## 项目约定

- 项目说明文档使用中文，命令、话题名称、文件名和代码标识符使用英文。
- ROS 2 启动文件统一使用 Python launch，不再新增 ROS 1 XML launch。
- 项目统一启动入口为
  `xczs_inspection_robot_control/launch/inspection_robot.launch.py`。
- 修改源码后必须重新编译，并验证统一启动入口能够正常运行。

## 版本管理

- 全程使用 Git 规范管理代码。
- 提交前完成必要的构建、格式和运行检查。
- 提交信息应准确描述本次变更。
