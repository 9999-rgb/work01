# Gazebo 11.10.2 固定源码

归档来自 Open Robotics 的 `gazebo11-non-amd64` PPA，与现有 ARM64 Debian 包对应。文件、原始下载地址和 SHA256 见 `provenance.json`、`SHA256SUMS`。两个归档共约 45.3 MiB，目标机无需下载 GitHub 源码。

`build_gazebo_core.sh` 解压 DFSG 源码并应用 Debian 补丁，**仅跳过 `0010-armhf-singleton.patch`**。该补丁清空 `GZ_SINGLETON_DECLARE`，在启用隐藏符号的构建中使不同动态库拥有各自的管理对象；现有 ARM64 包未导出 ConnectionManager、TopicManager 和 ModelDatabase 的共享实例。恢复上游声明并用 GCC 编译后，通过动态符号检查和最小世界实际统计、CPU 雷达样本检查才能继续构建 ROS 插件。

保留其他打包补丁，包括恢复 GUI 的 Qt property browser 源文件、系统字体、tinyobjloader 等适配。安装完整 GUI 与资源到 `/opt/gazebo-fixed`，不覆盖 dpkg 管理的 `/usr` 文件；运行程序、原生库和 CMake 配置使用此安装前缀。镜像同时设置登录终端路径，防止 `bash -lc` 重新选中旧可执行文件。

主要许可证为 Apache-2.0，部分组件使用 BSD、LGPL 等许可证；完整声明保留在源码归档的 `COPYING`、各文件声明及 Debian 归档的 `debian/copyright` 中。归档未修改，补丁选择由构建脚本明确记录。

开发机验证不能替代麒麟 ARM64 运行验收。最小世界通过只证明原生统计和 CPU 雷达工作，仍须检查项目相机、ROS 雷达消息、真实 map→odom 定位与 Web 操作。
