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

已在开发机执行 ShellCheck、Bash/Python 语法检查及实际 Nginx 代理测试（去前缀、查询参数、重定向、三个前端地址替换）。2026-10-12 目标麒麟 ARM64 已报告 13 包编译、镜像构建及 Nginx 配置检查通过；机器人动作验收尚未完成。安装过程的 healthy 只代表代理和机器人就绪状态通过，机械动作仍需另行测试。

## 国内下载源

- 容器基础镜像：DaoCloud 公开加速站，已检查 ARM64 清单。可用 `ROS_IMAGE=企业镜像地址 ./start` 覆盖，不改宿主机 Docker 全局配置。
- Ubuntu ARM64：清华 `ubuntu-ports`；ROS 2：清华 `ros2/ubuntu`，保留基础镜像原有签名密钥。
- Python：清华 PyPI；rosdep 规则和发行版索引：清华 rosdistro。
- Zenoh 1.9.0 ARM64：官方原始 ZIP 随 Git 分发，构建时校验 SHA256。额外增加约 9.3 MiB；来源与许可证见 `vendor/README.md`。
- Docker Engine 仍从麒麟本机配置的软件源安装，不使用 Ubuntu/CentOS 源替换麒麟系统源。

这些地址已在开发机做连通检查，不代表目标网络一定允许访问。国内源也可能限流或停服；脚本不自动回退国外源。若无需 Gazebo 桌面窗口，使用 `GUI=0 ./start`，通过浏览器控制。

## ARM64 Gazebo Classic

ARM64 软件源缺少 `ros-humble-gazebo-ros-pkgs` 与 `ros-humble-gazebo-ros2-control`，不能使用 x86 的二进制安装命令。镜像先安装随仓库提供的 Open Robotics Gazebo 11 ARM64 Debian 包作为依赖基础，再使用修正共享对象声明的固定源码重建 Gazebo 本体及 ROS 插件，与项目共享 `install/setup.bash`。上游来源及校验值见 `vendor/gazebo-arm64/`、`vendor/gazebo-source/`。

已导入离线基础镜像的目标机使用：

```bash
GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 ./start
```

首次构建时间取决于依赖下载和 ARM64 编译速度。Docker 数据目录应留足空间，不能因根分区很大就忽略 `/var` 的独立小分区。2026-10-12 目标机器已完成编译及镜像构建；修复后的服务启动与机械动作仍待实测。

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

本次验证包括下载失败重试、连续失败终止、超时上限、IPv4/请求超时设置、版本异常输出及退出码拒绝、Docker RUN 的 Bash 语法、构建层边界。目标机已报告 13 包编译及完整镜像构建通过；随后发现首次管理员引导的密码库兼容错误，修复后的容器就绪及 Web 操作仍需目标机验证。


## 首次 Web 启动失败：密码库兼容性

若日志出现 `Application startup failed`，调用栈指向 `bootstrap_admin` 和 Passlib 后端自检，随后出现 `password cannot be longer than 72 bytes`，不能据此认定用户实际密码过长。bcrypt 5 对长输入的处理变化使 Passlib 1.7.4 的后端自检失败；即使短测试密码也可复现。

依赖固定为 Passlib 1.7.4 与 bcrypt 4.0.1，继续使用原有 bcrypt_sha256 哈希格式。Docker 在安装 Python 依赖后测试长密码的哈希、正确密码验证和错误密码拒绝，失败会在编译机器人前停止。开发机已验证短密码、长密码、中文密码、首次管理员创建和重复引导保留现有账号；已确认 Python 3.10 的 ARM64 wheel 可下载。目标机仍须更新镜像才能使用修复，单独 `docker start` 会继续运行旧镜像。

后台 Xvfb 启动先用 glxinfo 实际检查 OpenGL 上下文（最长 20 秒）。检查通过后默认不再套用实体桌面的 180 秒冷启动等待；检查失败立即停止并保留错误输出。实体桌面分支仍保留原显示等待，显式配置的 XCZS_DISPLAY_SETTLE_SECONDS / XCZS_DISPLAY_SETTLE_MAX_WAIT 会继续生效。随后仍须通过 ROS 和 Web 就绪检查。`Ctrl+C` 中断根目录 start 的就绪等待，不代表已经停止后台容器。停止容器使用项目根目录 `bash ./stop_all.sh`，状态以 Docker 检查为准。


启动等待期间，根目录 `start` 会自动跟随容器日志，便于看到显示等待、Gazebo/ROS 启动与 Web 就绪进度；可能包含的管理员密码和显式密钥值会隐藏。脚本退出时关闭日志跟随器，后台容器状态仍由 Docker 管理。此改动仅涉及宿主机脚本，无需重新编译镜像。


虚拟显示等待修复已通过 8 项部署检查，并在开发机隔离的 Ubuntu 24.04 amd64 容器中实际创建 Xvfb/GLX 上下文，渲染器为 Mesa llvmpipe。该检查不等于目标麒麟 ARM64 运行验收；目标端仍须看到 OpenGL 检查通过、Web/ROS 就绪和真实传感器数据。

## 自动采样导航与传感器故障

在麒麟项目目录执行 `bash scripts/deploy/diagnose_runtime.sh`。脚本会先核对容器归属，再重启本项目容器一次，从同一轮启动自动采样 80 秒并打印进度。它通过 ROS 消息订阅收集 clock、里程计、雷达、相机元数据和 TF，不发送导航/机械动作，也不修改健康标准。

诊断覆盖标准及实际发现的同类型话题，能区分传感器无发布者、发布者存在但无数据、话题名称不一致，以及 odom→body、body→base_link、map→odom 三段连接缺失。静态 TF 采用可靠且持久的订阅以读取已发布的固定连接；传感器使用 best-effort 订阅兼容上游 QoS。输出只有消息计数、帧名、时间戳、尺寸和连接关系，不打印图像数据或凭据。

该脚本运行在宿主机，通过标准输入把探针送进现有容器；拉取它后即可诊断，无需重新构建镜像。若容器在采样期间退出，脚本会明确提示采样未完成，并打印最近故障及容器状态。

## 隔离检查 Gazebo 原生发布

若 ROS 中有里程计，但原生 `world_stats`、雷达话题的 `Publishers` 为空，执行：

```bash
bash scripts/deploy/diagnose_gazebo_native.sh
```

脚本使用 `xczs-simulation` 当前镜像创建临时容器，运行一个无项目插件、无网格模型的最小 CPU 雷达世界。它不停止现有容器、不映射宿主机端口、不挂载数据卷；结束或中断后清理临时容器。必须实际收到世界统计和雷达频率才算通过，退出码为 0 或话题名称存在不能代替数据验证。

通过说明该镜像在最小世界中能完成原生通信和 CPU 雷达更新，后续检查项目模型与插件加载；失败则优先检查目标机的 Gazebo 原生运行环境。该检查不验证相机渲染、ROS 导航或机械动作，无需重新构建镜像。

最小世界也失败时，脚本会打印主线程等待位置、线程等待状态汇总、实际加载的 Gazebo 核心库与原生发布注册。宿主机已安装 gdb 时，可作为 root 执行 `bash scripts/deploy/diagnose_gazebo_native.sh --stack`，在失败现场读取临时 gzserver 的主线程和各线程调用栈；最多额外等待约 30 秒，然后继续清理临时容器。gdb 通过临时容器 ID 查找宿主机 PID，使用该进程的根文件系统解析库，不读取主仿真进程，也不打印函数参数或环境凭据。

## 2026-10-13 Gazebo 原生发布修复

目标机最小世界也收不到统计和 CPU 雷达样本，关闭 seccomp 的对照结果相同。线程栈显示物理循环仍运行，却出现多条 ModelDatabase 更新线程。随仓库分发的 ARM64 库未导出共享 SingletonT 对象；对应发行包的 `0010-armhf-singleton.patch` 清空了上游显式声明。在开发机的两个动态库对照中，该补丁使对象分裂，恢复声明后对象共享。这支持原生管理对象分裂导致发布注册失败的判断，最终仍以麒麟实际数据为准。

构建跳过该补丁，用 GCC 重建完整 Gazebo（含 GUI、传感器和资源）到 `/opt/gazebo-fixed`，随后重编 ROS 插件。已有依赖下载层保留；新增原生依赖安装、源码编译和真实数据检查分别缓存。导出检查或最小世界实际统计/雷达检查不通过就终止，不以退出码 0、版本文本或静态 TF 代替运行验收。源码构建会比此前仅编译 ROS 包更久，旧容器在构建阶段保持原状态。

原生构建依赖使用 ROS 源配套的 `libignition-math6-dev`。不要改为 Ubuntu 的 `libignition-math-dev`：后者要求旧运行库 `libignition-math6-6`，会与已有的 `libignition-math6` 6.16 冲突，导致 APT 在源码编译前退出。更新此安装层后可继续使用之前的构建缓存，无需清理镜像或降级运行库。

更新麒麟仓库后执行，必须重建镜像；`docker start/restart` 仍会使用原镜像：

```bash
set -o pipefail
cd /root/proj/work01-gitee &&
git pull --ff-only &&
GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 bash ./start 2>&1 | tee deploy-kylin.log
```

构建中应看到三项“共享对象导出通过”及“原生世界统计与雷达均有实际数据”。安装结束后执行 `bash scripts/deploy/diagnose_runtime.sh`，确认雷达和相机收到实际消息、map→odom 连通，再验证网页与机械动作。

开发机 amd64 已完整编译、安装同一源码并确认新库加载；最小世界收到连续统计、约 10 Hz 的 CPU 雷达，以及设置软件渲染的原生相机消息。8 项部署检查和真实双动态库对象共享/分裂回归通过，原 ARM64 库与空输出均被拒绝。此结果不代表项目场景、ROS 相机或机械动作已通过，也不替代麒麟 ARM64 镜像构建与运行验收。

## 独立前端页面

`font_op/dist` 随一键部署打包，由容器 Nginx 的 8888 端口提供，访问 `http://10.186.109.146:8888/`。接口与 WebSocket 经同源 `/api/` 转发到容器后端；原 80 端口监控页保留。首次接入需重建镜像及容器，单独 `docker restart` 不会增加端口映射。防火墙需允许 TCP 8888，`HTTP_PORT` 不可设为 8888。
