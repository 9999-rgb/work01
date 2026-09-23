# 场景随末端套装切换验收（2026-09-22）

## 变更与结论

- **背景缺陷**：监督器重启机器人子栈时沿用**启动期烘死的 `scene:=`**（它只追加
  toolset/spawn 参数、其余原样透传），所以「先切场景、再切套装」会把子栈里的场景换回
  启动时那个：`map_server` 载入旧场景地图 → Nav2 在配置阶段卡住、永不到 active →
  120 s 就绪门超时 → 监督器**回滚到上一套装**。同一份错误地图还会让目标场景的工位
  （用的是另一套坐标）落进旧图的占用带，导航被直接拒绝。
  现场三症状——「NAV2 导航未就绪 / 控制柜操作未就绪」「过一会自动变回套装 B」
  「操控抽拉柜失败」——是同一个根因的三种表现。
- **改法**（4 处，2 个包）：
  1. `SwitchToolset.srv` 新增 `string scene` 与 `string scenes_config`（空串表示沿用
     启动时的场景，兼容未升级的调用方；两字段必须成对给出，缺一即忽略并告警）；
  2. 网关 `ros_node.request_toolset_switch` 接受并填充这两个字段；
  3. runner 把**活动场景**与**该场景所在的那份 scenes.yaml** 随请求下发；
  4. 监督器在**目标启动**与**回滚**两条路径上把这对参数**追加到最后**
     （`ros2 launch` 同名参数后者生效）。
- **scenes.yaml 必须取「包含该场景」的那份**：资产库的场景是**逐场景自包含单文件**
  （`assets/scene/<name>/scenes.yaml` 只含该场景），内置场景在
  `control/config/scenes.yaml`。传错（拿别的场景的资产文件配 `scene:=`）会让子栈
  找不到该场景、启动即退出（`robot child exited before ready (code 1)`）。为此新增
  `AssetSceneProvider.scene_config_path()`，runner 按场景来源解析后再下发。

## 证据

| 步骤 | 修复前 | 修复后 |
| --- | --- | --- |
| 切场景 → 电气夹层 | `switched` | `switched` |
| 切套装 A→B | **120 s 就绪门超时 → 回滚到 A** | **`ready`，无回滚** |
| 运行时 `/map` | 发电机图（origin −40.512） | **夹层图（origin −33.177）** |
| 切套装 B→A | 同左 | **`ready`，无回滚** |
| 导航 db1（夹层工位） | `Navigation goal is inside an occupied map cell` | **`success`，33.1 s** |

修复前那次失败的日志：子栈按 `scene:=generator_plant` 载入发电机图，
`map_server` 配置之后再无进展，直到 +120 s 被销毁并回滚，报
`Toolset A could not become ready: actions unavailable: Nav2 lifecycle manager is not active`；
随后 3 分钟内子栈被重启三次（A → B → A），插件实例号 `robot_a_2 → robot_b_3 → robot_a_4`。

## 校验与复现

- 契约测试 `jiang/tests/test_toolset_gateway_contract.py` 新增一条：断言切换请求
  携带活动场景，且路径按来源解析（资产场景 → 资产内那份；内置场景 → 内置 catalog）。
  > 后续核对（2026-09-22）：解析实现是 `AssetSceneProvider.scene_config_path()` 的
  > **纯 kind+name 查表**（`asset_library.find("scene", name)`），不是"按来源"判定；
  > 两个内置场景名在资产库里都有同名目录，故它们同样会命中资产库那份。该契约测试用
  > 假 provider 把两条分支当成名字二分来断言，**未覆盖同名碰撞**这一实际情形。
- `jiang/tests` 全量通过；`colcon test`（control 包）202 项通过。
- 复现：`./run_all.sh --web` → `POST /scene/switch {"name":"electrical_mezzanine"}`
  → `POST /robot/toolset/switch {"toolset":"B"}` → 读 `/map` 的 origin 与
  `/robot/toolset/status`。
- **未做**：组合穷举（另一场景下的切换、用户导入的资产场景、场景与套装同时下发）。
  逻辑上同一条路径都成立，但未逐一实测。
- **失误记录**：本修复第一版把**启动时那份** scenes_config 直接下发，导致子栈找不到
  场景、启动即退出、回滚也失败、监督器进入 `failed` 终态（只能重启恢复）。现按
  「按场景来源解析」修正，并有测试覆盖两条分支。
