# 麒麟 V10 ARM64 部署

入口为项目根目录 `start`。目标环境：银河麒麟高级服务器 V10、aarch64、systemd；首次安装需要访问麒麟软件源、Docker Hub、Ubuntu/ROS 软件源、PyPI 和 GitHub。

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

- 容器基础环境：`ros:humble-ros-base-jammy`，在 ARM64 主机上原生编译项目。
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
