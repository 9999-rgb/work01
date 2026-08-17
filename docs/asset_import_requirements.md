# 场景 / 柜体 资产导入 + 末端夹爪固定变体选择 —— 需求文档

> 状态:已确认 · 版本:v1 · 日期:2026-08-17
> 关联设计:三层适配架构(通用任务层 / 机器人适配层 / 场景适配层),
> 现有链路见 `docs/control_cabinet_simulation.md` 与本仓库 CLAUDE.md。

---

## 1. 背景与目标

现有系统采用「三层适配」架构,设计意图是「更换机器人 / 设备 / 场地只改适配包,
不动任务 API 和 Web 页面」。但当前「导入」只是把文件散落在各 ROS 包、再靠
环境变量 / launch 参数指路径:没有资产库、没有统一 manifest、没有 Web 导入 /
选择界面。用户换一台柜体、换一个场景,需要人工跨包改文件,门槛高且易错。

本需求为这套仿真系统补齐 **资产的导入、校验、选择与分发** 能力,让用户通过
Web 页面完成「导入一个场景 / 柜体资产、选择本次运行组合」,并且:
- **对现有任务 API 与 Web 监控页面零影响**(改动全部落在适配层与新增资产层);
- **启动时导入**(非运行时热插拔);
- **机器人本体固定**,用户可换的是:场景、柜体、末端夹爪(**固定变体,选择而非导入**)。

## 2. 需求范围

### 2.1 核心需求(已确认)

| # | 需求 | 说明 |
|---|---|---|
| R1 | 可导入「场景」资产 | 换地图、柜体摆放、场地模型与机器人出生位;机器人 + 柜体不变 |
| R2 | 可导入「柜体」资产 | 换柜体类型(新控件目录 / 新几何 / 新物理参数) |
| R3 | 末端夹爪**不导入**,选择 | 机器人模型内置「固定的几类夹爪变体」,Web 下拉选择 |
| R4 | 机器人本体固定 | 底座 + 机械臂 + MoveIt manipulator + ros2_control 不在导入范围 |
| R5 | 启动时导入 | 资产在启动前导入 / 校验 / 选择,启动进程消费选中结果 |
| R6 | Web 页面入口 | 在现有 8090 控制台加「资产」区块:上传、列表、校验、选择、应用 |
| R7 | 场景即组合根 | 场景资产 manifest 引用它需要的柜体资产与夹爪变体 |

### 2.2 非目标(明确不做)

- 运行时热插拔 / 动态增删资产(重启生效)。
- 导入「整个机器人」类型(仅夹爪可换)。
- 通用可达性自动求解(夹爪×柜体配对采用预写可达性表)。
- 跨机器 / 云端资产分发仓库(目录 / zip 分发即可)。

## 3. 资产模型

| 资产 | 处理方式 | 内容(映射现有文件) |
|---|---|---|
| 机器人本体(底座+机械臂) | 固定 | arm + MoveIt `manipulator` 组 + `ros2_controllers.yaml`(不含夹爪) |
| **末端夹爪** | **固定变体 + 选择** | `gripper_<variant>.xacro`(worklink / button_press_tip)+ 各自控制器 + MoveIt `gripper` 组 + `open`/`closed` 命名位姿 + `contact_tool_link` + 夹爪×柜体可达性表 |
| **柜体** | ✅ 可导入 | `cabinet_controls.yaml` + `cabinet_scene.yaml` + `cabinet_pose.yaml` + `control_cabinet.urdf.xacro` + `meshes/control_cabinet/*` |
| **场景** | ✅ 可导入 | `scenes.yaml` 条目 + Nav2 地图(yaml+pgm)+ 场地 SDF/mesh + 摆放(`cabinet_instances.yaml` + `robot_spawn`) |

### 3.1 组合关系

```
场景(组合根)
 ├─ 引用 → 机器人本体(固定)
 ├─ 引用 → 夹爪变体(固定集合内选择)
 └─ 引用 → 柜体资产…(可导入)
```

### 3.2 关键洞察:夹爪 × 柜体耦合

夹爪是「操作柜子按钮」的枢纽件,与柜体耦合最紧:

- `contact_tool_link = button_press_tip` —— 按按钮的接触点长在夹爪上;
- `operable` / `unreachable_control_ids` —— 「够不够得到某按钮」取决于夹爪几何;
- 旋钮 / 开关 / 门的抓取(`SetCabinetGrasp`)依赖夹爪。

因此「换夹爪变体」与「换柜体」都会触碰**夹爪 × 柜体耦合**,这是本设计的技术
核心。解决思路:夹爪侧是固定小集合,按 **(夹爪变体, 柜体类型)** 预写可达性表
(而非通用耦合求解器);新导入柜体需补一份对该柜体的可达性评价(手动,或后续用
现有 `operable=false` 规划验证自动推导)。

## 4. manifest 格式(示意)

每个资产目录根放 `manifest.yaml`,统一字段:

```yaml
kind: cabinet | scene       # 资产类型(夹爪不在其中,是机器人内选择项)
name: <a-z[a-z0-9_]{0,62}>  # 唯一名,复用现有 control_id / 实例名正则
version: 1.0.0
files:                      # 本目录内文件角色映射
  xacro: ./control_cabinet.urdf.xacro
  controls: ./cabinet_controls.yaml
  ...
```

场景资产额外含:
- `references`:引用的柜体资产名 + 夹爪变体名;
- `instances`:柜体摆放(覆盖默认 `cabinet_instances.yaml`);
- `robot_spawn`:机器人出生位;
- `nav2_map`:地图文件(资产库内相对路径或 `package://`)。

## 5. 导入 / 选择 / 分发流程

### 5.1 导入(Import)
用户上传 zip 或指定目录 → 读取并校验 manifest → 运行导入校验
(`check_scene_config` / `check_adapter_contract` 复用)→ 复制进资产库
(`jiang/data/assets/`)→ 登记 catalog(`assets_catalog.yaml`)。

### 5.2 选择(Select)
Web 选择场景 / 柜体 / 夹爪变体 → 持久化到 `jiang/data/selection.yaml` →
启动时 `start_xczs_bridge.sh` 读选中结果 → 映射到现有环境变量指针
(`SCENES_CONFIG` / `NAV2_MAP_PATH` / `CABINET_INSTANCES_PATH` / 夹爪相关)。

### 5.3 分发(Distribute)
资产目录或 zip 整体拷出即完成分发;接收方导入即可。

## 6. 改动面

### 6.1 复用(已有,不重造)

- `jiang/control_gateway/profile_contract.py::validate_profile()` +
  `scripts/check_adapter_contract` + `scripts/check_scene_config` —— 现成跨文件
  导入校验器,接受任意路径。
- `jiang/control_gateway/_package_resolver.py::resolve_package_uri()` —— URI 解析扩展点。
- 现有路径管道:`start_xczs_bridge.sh` 的 `CABINET_*_PATH` / `SCENES_CONFIG` /
  `NAV2_*_PATH` / `MOVEIT_*_PATH` → CLI / launch arg —— 选择层只需映射到这些指针。
- FastAPI + JWT + `monitor.html` —— Web 导入 / 选择地基。

### 6.2 新建(必须建)

1. **manifest schema + 校验器** —— `control_gateway/asset_manifest.py`(不可变
   dataclass + `yaml.safe_load` + 严格字段校验,风格对齐 `robot_adapter.py` / `inventory.py`)。
2. **资产库 + catalog + 选择持久化** —— `control_gateway/asset_library.py`;
   `jiang/data/assets/` + `assets_catalog.yaml` + `selection.yaml`。
3. **CLI 导入脚本** —— `scripts/xczs_import_asset`(脚本化 / CI)。
4. **Web 导入 / 选择后端 + 前端页** —— `app/api/assets.py` + `monitor.html`「资产」区块。
5. **URI 解析扩展** —— `package://` / `model://` 引用指向资产库目录。
6. **启动层消费选中资产集** —— `start_xczs_bridge.sh` 读 `selection.yaml` → 设 env/launch 指针。
7. **多夹爪变体重构**(阶段4) —— 机器人模型单夹爪 → N 变体 + 选择开关。

## 7. 分阶段计划(每阶段可独立验收)

| 阶段 | 内容 | 验收边界 |
|---|---|---|
| **1 · 后端核心 + 换场景** | 资产库 + manifest + 导入/校验 + 复用现有管道做「选择」。先只支持**换场景**(夹爪/柜体不变,零夹爪×柜体改动),验证 manifest 形状 | CLI 导入一个场景资产 → 校验 → 选择 → `XCZS_PREFLIGHT_ONLY` 预检通过 |
| **2 · Web 导入/选择页** | 阶段1 后端接 Web(上传、列表、校验、选择、应用),夹爪变体下拉先只有「当前夹爪」 | 登录后导入场景资产,列表/校验/选择/应用闭环;换场景后 `run_all.sh` 用新地图/摆放启动 |
| **3 · 换柜体** | 柜体资产导入 + 夹爪×柜体可达性配对(固定夹爪集合内预写);`scripts/check_cabinet_model` 参数化 | 导入新柜体资产,预检通过,柜体操作闭环验证 |
| **4 · 多夹爪变体重构** | 机器人模型 N 夹爪变体 + 选择开关 + 各变体×柜体可达性表 | 切换夹爪变体后,接触点/控制器/MoveIt 组/可达性表随之切换,`check_adapter_contract` 通过 |

## 8. 验证方式

1. 每种资产类型写样例资产目录 + manifest,端到端:导入 → 校验 → 选择 → 启动。
2. 复用 `check_adapter_contract` / `check_scene_config` 作导入校验;
   `XCZS_PREFLIGHT_ONLY=true ./run_all.sh` 验证启动预检。
3. `python3 -m pytest jiang/tests/ -q` 与
   `colcon test --packages-select xczs_inspection_robot_control` 回归。
4. Web 侧(阶段2):登录后导入场景资产,确认列表 / 校验 / 选择 / 应用闭环,
   并验证「换场景」后 `run_all.sh` 用新地图 / 摆放启动。
5. 阶段4:切换夹爪变体后,验证接触点 / 控制器 / MoveIt 组 / 可达性表随之切换,
   `check_adapter_contract` 通过。

## 8.5 实现状态

> 记录实现进度;验收方式见 §8。

| 阶段 | 状态 | 落点 |
|---|---|---|
| **1 · 后端核心 + 换场景** | ✅ 已实现 | `control_gateway/asset_manifest.py`(manifest schema + 校验器)、`control_gateway/asset_library.py`(资产库 + catalog + 选择持久化 + `selection_to_env` 映射)、`scripts/xczs_import_asset`(CLI 导入,复用 `check_scene_config` 语义校验)、`jiang/samples/scene_cabinet_operation/`(样例场景资产)、`jiang/start_xczs_bridge.sh`(启动时读 selection → 映射现有 env 指针) |
| 2 · Web 导入/选择页 | ⬜ 待实施 | `app/api/assets.py` + `monitor.html`「资产」区块 + `app/main.py` 挂载 |
| 3 · 换柜体 | ⬜ 待实施 | 柜体资产导入 + 夹爪×柜体可达性配对 + `check_cabinet_model` 参数化 |
| 4 · 多夹爪变体重构 | ⬜ 待实施 | 机器人模型单夹爪 → N 变体 + 选择开关 |

阶段1 验证结果(2026-08-17):

- 新增 pytest:`test_asset_manifest.py`(18)+ `test_asset_library.py`(15)+ `test_asset_import_cli.py`(5);全量 `python3 -m pytest jiang/tests/ -q` = **542 passed**(原 504 + 新增 38)。
- 端到端:CLI 导入样例场景资产 → 语义校验(`check_scene_config` 子进程)通过 → 选择 → `selection_to_env` 输出 `SCENES_CONFIG` / `SCENE` / `CABINET_INSTANCES_PATH` → `XCZS_PREFLIGHT_ONLY=true ./run_all.sh` 预检通过。
- 默认行为不变:selection 为空时 `--print-env` 无输出,启动脚本不注入任何覆盖,仍走 `cabinet_operation`。
- 设计要点:
  - `jiang/data/assets/` 已在 `.gitignore`(生成的资产库不提交);样例**源**资产放 `jiang/samples/`,可重新导入。
  - 导入时场景资产内相对 `nav2_map` / `model.file` 归一化为资产库内绝对路径,`package://` / `model://` 原样保留。
  - 导入失败不留任何痕迹(拷贝 + 校验 + catalog 登记同一事务,失败即清理)。
  - 校验器可注入(`validate=` 钩子),Web 端(阶段2)复用同一导入入口。

## 9. 风险与后续可选项

- **夹爪 × 柜体可达性**:新柜体 / 新夹爪变体需预写可达性表;后续可复用
  `operable=false` 的规划验证自动推导,减少手写。
- **`check_cabinet_model` 参数化**:当前冻结到 33 控件 / 20+13 关节,阶段3 需
  使其支持任意柜体端到端校验。
- **资产版本与兼容**:manifest 带 `version`,导入校验可回答「这柜体和这夹爪配不配」。
- **数据一致性**:机器人 / 柜体 / 场景三层配置本就跨文件强校验
  (`profile_contract`),导入即校验是守住一致性的入口。
