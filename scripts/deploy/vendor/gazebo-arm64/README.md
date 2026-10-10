# Gazebo Classic ARM64 依赖

Gazebo 11.10.2+dfsg-1osrf6 的五个 Debian 安装包来自 Open Robotics 的 gazebo11-non-amd64 PPA，适用于 Ubuntu Jammy ARM64（仅 common 为 all）。包摘要已逐项与 PPA Packages 索引比对。仅安装在 Ubuntu 容器，不安装到麒麟宿主机。

两个 ROS 插件源码归档按 Humble rosdistro 指定分支选择，再固定到具体提交；详细 URL、提交和 SHA256 见 provenance.json，归档内保留原始许可证。Gazebo Debian 包中包含 /usr/share/doc 下的版权及许可证说明。上游源码：https://github.com/gazebosim/gazebo-classic；PPA 源码发布目录：https://ppa.launchpadcontent.net/openrobotics/gazebo11-non-amd64/ubuntu/pool/main/g/gazebo/ 。

构建先校验 SHA256SUMS，安装 Gazebo Debian 包，其依赖由国内 Ubuntu/ROS 镜像解析；然后把 gazebo_ros_pkgs、gazebo_ros2_control 与项目一起编译。忽略上游演示包，不禁用项目所需的 Gazebo 插件。

此目录额外增加约 57 MiB，换取目标机不依赖 GitHub/Launchpad 下载。已验证文件校验值、架构与源码包清单；尚未在目标麒麟完成原生编译和仿真验收。
