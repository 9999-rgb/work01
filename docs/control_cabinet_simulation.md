# 控制柜自动操作仿真

本项目在 Gazebo、MoveIt 2、Nav2 和 Web 控制端之间提供控制柜自动操作闭环。自动操作必须由机械臂到达目标并触发 Gazebo 中的物理关节状态；仅完成轨迹规划不视为成功。

## 控制件清单

控制目录固定包含 33 个可操作件：

- 20 个按钮：行程 8 mm，超过 6 mm 进入 `pressed`，回到 3 mm 以下进入 `released`。
- 11 个旋钮：仿真三档 `left=-45°`、`center=0°`、`right=45°`，默认 `center`。
- 1 个背门总开关：仿真两档 `off=0°`、`on=90°`，默认 `off`。
- 1 个背门：仿真两档 `closed=0°`、`open=90°`，默认 `closed`。

旋钮档位、总开关角度和背门结构是现有 CAD 缺少业务定义或独立门板资产时采用的明确仿真假设，不代表真实设备的电气参数。柜壳因此使用可碰撞的程序化框体、独立背门和 U 形把手；原模块网格继续用于前面板外观。

## 各组件职责

- Gazebo 保存按钮、旋钮、开关和柜门的真实关节状态，并实现按钮弹簧、旋钮档位、开关/柜门锁止、近距工具约束和复位。
- MoveIt 2 负责机械臂的碰撞检查、到位、接近、操作和安全回撤，是自动操作的必需组件。
- Nav2 只负责把移动底盘送到控件所在柜面前的操作工位。Web 中可取消“先导航到操作工位”，此时机器人必须已经停在可达位置。
- Web 端负责选择控件、命令和目标状态，显示物理位置、离散状态、动作阶段，提供取消与复位。

旋钮、总开关和柜门采用仿真工具锁扣：只有 `button_press_tip` 到达配置的操作点并处于允许距离内，Gazebo 才建立临时跨模型约束。随后关节角度由机械臂执行的旋转或门把手弧线带动；操作接口不会直接写目标关节角。按钮仍由探针真实接触压下。

## 启动和 Web 验证

先构建并启动统一入口：

```bash
colcon build --symlink-install
source install/setup.bash
./run_all.sh --web --no-gui
```

浏览器打开 `http://localhost:8080/monitor.html`，进入“控制柜自动操作”：

1. 在按类型分组的目标列表中选择控制件。
2. 选择按下、档位、ON/OFF 或 OPEN/CLOSED。
3. 根据机器人当前位置决定是否勾选“先导航到控制柜操作工位”。
4. 提交后观察动作阶段、当前位置、目标位置和物理状态。
5. 可随时取消；复位仅在没有控制柜动作、Nav2 或普通 MoveIt 动作时允许。

## ROS 2 接口

- 通用动作：`/xczs/operate_cabinet_control`，类型 `OperateCabinetControl`。
- 按钮兼容动作：`/xczs/press_cabinet_button`，类型 `PressCabinetButton`。
- 控制目录：`/xczs/cabinet/control_catalog`，可靠、持久化 QoS。
- 单件状态：`/xczs/cabinet/<control_id>/state`，类型 `CabinetControlState`。
- 活动目标：`/xczs/cabinet/active_control`，供 MoveIt 动态碰撞场景临时开放当前目标。
- 安全复位：`/xczs/cabinet/reset_controls`，类型 `std_srvs/Trigger`。
- Gazebo 内部复位：`/xczs/cabinet/reset_physics`。
- Gazebo 工具约束：`/xczs/cabinet/grasp`，类型 `SetCabinetGrasp`。

Web API 保留 `/cabinet/press`，并增加：

- `POST /cabinet/operate`
- `POST /cabinet/cancel`
- `POST /cabinet/reset`
- `GET /cabinet/controls`
- `GET /cabinet/status`

## 自动验收

只检查 33 项目录和所有物理状态话题：

```bash
source install/setup.bash
scripts/validate_cabinet_simulation --inventory-only
```

执行每个控制件的代表动作：

```bash
scripts/validate_cabinet_simulation
```

执行全部按钮，并让旋钮、总开关、柜门遍历后回到默认状态：

```bash
scripts/validate_cabinet_simulation --exhaustive
```

让每次动作包含 Nav2 自动停靠：

```bash
scripts/validate_cabinet_simulation --navigate --exhaustive
```

也可用一个或多个 `--control <control_id>` 做定点回归。

## 仿真边界

控制柜位姿由启动参数和 `odom -> control_cabinet_frame` 静态变换提供，属于仿真真值定位。当前 RGB 相机没有深度、目标识别和手眼标定链路，因此本功能不宣称视觉定位；Gazebo 的近距约束与关节状态闭环也不等同于可直接迁移到真实机械臂的力控。

Ubuntu 22.04 / ROS 2 Humble 当前二进制 MoveIt 2.5.9 在 `move_group` 收到 Ctrl-C 后可能于第三方析构路径退出为 `-11`。统一启动脚本仍会取消动作并清理所有子进程；升级或回溯修补 MoveIt 属于第三方版本变更，需单独评估。
