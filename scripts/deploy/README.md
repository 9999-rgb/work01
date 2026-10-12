# 麒麟 V10 ARM64 部署

入口为项目根目录 `start`。目标环境：银河麒麟高级服务器 V10、aarch64、systemd；首次安装需要访问麒麟现有软件源、DaoCloud 镜像加速站及清华镜像站。目标机不再直接下载 GitHub 上的 Zenoh 发布包。

## 操作

从 Git 克隆完整项目到麒麟；仓库包含部署所需的两个场景资产、模型及 `scripts/deploy`，无需单独上传运行数据库。不要复制开发机的 `build`、`install`、`log`、`.venv`。在项目目录的桌面终端执行：

```bash
chmod +x start
./start --check
sudo ./start
```

root 用户直接运行 `./start`。脚本只从麒麟现有软件源查找 Docker Engine，不增加其他发行版源。如果源中没有 Docker 安装包，将明确停止；需要根据目标机器的软件源确定后续安装方式。不会卸载 Podman。

默认 `GUI=auto`：检测本地 X11 显示和授权，有授权则显示 Gazebo 窗口，否则使用 Xvfb。只看到桌面但终端没有 DISPLAY/XAUTHORITY 时，不等于容器具备桌面访问权限。强制要求窗口可用：

```bash
sudo env DISPLAY="$DISPLAY" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" GUI=1 ./start
```

仅后台运行：`sudo GUI=0 ./start`。软件渲染不要求 GPU 透传，但性能需在目标机器验证。X11 会话注销或重建后，需要从新桌面终端重新运行部署以更新授权。

部署完成后在麒麟浏览器打开 `http://localhost/monitor.html`。其他网络可达机器使用 `http://服务器IP/monitor.html`。`HTTP_PORT=8080 ./start` 可更换宿主机端口；脚本不修改防火墙。

## 运行结构

- 容器基础环境：`m.daocloud.io/docker.io/library/ros:humble-ros-base-jammy`，在 ARM64 主机上原生编译项目。
- 容器继续使用 `run_all.sh --web`，Nginx `/api/` 去前缀后代理到内部 8090。
- 仅发布 Nginx 端口，不发布 ROS、Zenoh 或后端端口。
- 桌面只传入 X11 socket 和授权 cookie，不使用 `xhost +`、特权容器或关闭宿主机安全策略。
- Docker 命名卷 `xczs-runtime-data`、`xczs-recordings` 保存运行数据，替换容器不删除卷。首次创建数据卷时使用镜像内资产；后续若修改源资产，需要另行同步卷内资产，重建镜像不会覆盖已有数据卷。
- 重新执行会构建镜像，然后替换本项目容器；构建失败不停止旧容器，启动或就绪失败尝试恢复旧容器。共享数据卷不是数据库回滚快照。

```bash
docker logs --tail 100 -f xczs-simulation
docker stop xczs-simulation
docker start xczs-simulation
docker inspect --format '{{.State.Health.Status}}' xczs-simulation
```

## 验证边界

已在开发机执行 ShellCheck、Bash/Python 语法检查及实际 Nginx 代理测试（去前缀、查询参数、重定向、三个前端地址替换）。没有在目标麒麟 ARM64 上完成镜像构建和机器人动作验收。安装过程的 healthy 只代表代理和机器人就绪状态通过，机械动作仍需另行测试。

## 国内下载源

- 容器基础镜像：DaoCloud 公开加速站，已检查 ARM64 清单。可用 `ROS_IMAGE=企业镜像地址 ./start` 覆盖，不改宿主机 Docker 全局配置。
- Ubuntu ARM64：清华 `ubuntu-ports`；ROS 2：清华 `ros2/ubuntu`，保留基础镜像原有签名密钥。
- Python：清华 PyPI；rosdep 规则和发行版索引：清华 rosdistro。
- Zenoh 1.9.0 ARM64：官方原始 ZIP 随 Git 分发，构建时校验 SHA256。额外增加约 9.3 MiB；来源与许可证见 `vendor/README.md`。
- Docker Engine 仍从麒麟本机配置的软件源安装，不使用 Ubuntu/CentOS 源替换麒麟系统源。

这些地址已在开发机做连通检查，不代表目标网络一定允许访问。国内源也可能限流或停服；脚本不自动回退国外源。若无需 Gazebo 桌面窗口，使用 `GUI=0 ./start`，通过浏览器控制。

## ARM64 Gazebo Classic

ARM64 软件源缺少 `ros-humble-gazebo-ros-pkgs` 与 `ros-humble-gazebo-ros2-control`，不能使用 x86 的二进制安装命令。当前镜像改为安装随仓库提供的 Open Robotics Gazebo 11 ARM64 Debian 包，再从固定源码编译 ROS 插件，与项目共享 `install/setup.bash`。上游来源及校验值见 `vendor/gazebo-arm64/`。

已导入离线基础镜像的目标机使用：

```bash
GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 ./start
```

首次构建时间取决于依赖下载和 ARM64 编译速度。Docker 数据目录应留足空间，不能因根分区很大就忽略 `/var` 的独立小分区。当前只完成包架构、归档校验及脚本静态检查，目标机器编译与机械动作仍待实测。

## 2026-10-12 构建故障修复

- 构建阶段使用宿主机网络，运行容器仍使用原有端口映射，不改变运行网络。
- rosdep 更新单独缓存，限定 IPv4、单请求 60 秒、每轮最多 300 秒，最多 3 轮；失败仍终止，不使用不完整缓存继续构建。后续 APT 请求采用 IPv4、60 秒超时和 5 次重试。
- 包清单单独用于依赖安装；机器人源码、版本检查、Web/场景配置分层。后续修改源码不再重下 rosdep 索引，修改版本检查或 Web 文件不再重编机器人。升级此分层布局时，旧层之后的部分步骤仍需重跑一次。
- `xczs_ros_global_args_guard` 补齐 `gazebo_ros` 目标依赖，适配独立安装前缀。
- Gazebo 版本检查限时 20 秒；只在输出包含 `version 11.` 且退出码为 0 或 255 时通过。此检查不能替代仿真启动及机械动作验收。

麒麟上继续构建并保存日志：

```bash
set -o pipefail
GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 bash ./start 2>&1 | tee deploy-kylin.log
```

本次验证包括下载失败重试、连续失败终止、超时上限、IPv4/请求超时设置、版本异常输出及退出码拒绝、Docker RUN 的 Bash 语法、构建层边界。目标机此前已报告 13 包编译通过；更新后的完整镜像构建、容器启动及 Web 操作仍需目标机验证。
