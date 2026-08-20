"""阶段3 柜体资产导入 / 参数化 测试。

覆盖三块：

1. **真实样例端到端**（需要 ROS workspace 提供 ``xacro`` CLI）——
   ``scripts/xczs_import_asset jiang/samples/demo_cabinet`` 触发真实
   ``check_cabinet_model --asset`` 语义校验，catalog 落盘 ``validated: true``，
   选中后 ``--print-env`` 输出 5 个 ``CABINET_*_PATH`` 指针。

2. **非 33 控件参数化证明**（需要 xacro）——从样例派生一台 32 控件合成柜体
   （删掉纯旋钮盒体 box_9），``check_cabinet_model --asset`` 必须接受
   （输出 32 controls），证明资产模式不冻结 33/20+13。

3. **校验器 hook 契约**（纯逻辑，无需 xacro）——``cabinet_validator`` 子进程
   命令构造与失败传播可注入替换。

样例柜体资产为内置 control_cabinet 的物理孪生，见
``jiang/samples/demo_cabinet/manifest.yaml`` 说明。
"""

from __future__ import annotations

import os
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from app.assets.store import SqlAssetStore
from control_gateway.asset_manifest import load_manifest
from control_gateway.asset_validators import cabinet_validator


WORKSPACE = Path(__file__).resolve().parents[2]
SAMPLE_CABINET = WORKSPACE / "jiang" / "samples" / "demo_cabinet"
CHECKER = WORKSPACE / "scripts" / "check_cabinet_model"
CLI = WORKSPACE / "scripts" / "xczs_import_asset"
CABINET_ENV_KEYS = (
    "CABINET_CONTROLS_PATH",
    "CABINET_SCENE_PATH",
    "CABINET_POSE_PATH",
    "CABINET_ROBOT_ADAPTER_PATH",
    "CABINET_XACRO_PATH",
)

# 从样例删掉 box_9_knob（纯旋钮盒体，场景/门/开关均不引用它）所需的精确文本块。
# 若样例文件改动导致任一块不匹配，测试立即失败 —— 这正是跨文件一致性的信号。
_CONTROLS_ID_LINE = "      - box_9_knob\n"
_CONTROLS_BLOCK = (
    "      box_9_knob:\n"
    "        type: knob\n"
    '        display_name: "9号模块旋钮"\n'
    "        joint_name: box_9_box_9_knob\n"
    "        pivot_position: [0.12494, 0.774526, 0.0295]\n"
    "        local_position: [0.13844, 0.774526, 0.0475]\n"
)
_ADAPTER_BLOCK = (
    "      box_9_knob:\n"
    "        navigation_station:\n"
    "          local_anchor: [0.488440, 1.115, 0.000]\n"
    "          outward_axis: [0.0, 0.0, 1.0]\n"
    "          standoff: 0.930\n"
    "          base_yaw_offset: 0.0\n"
    "          frame_id: map\n"
)
# 适配器的 unreachable_control_ids 全不可达清单必须与控件目录保持一致；
# 从 33 控件派生体删掉 box_9_knob 时，该清单也要同步删掉这一行，否则
# 契约校验会报「覆盖不完整」的未知 ID。
_UNREACHABLE_BOX9_LINE = "      - box_9_knob\n"
_GAZEBO_LINE = (
    '      <xacro:control_cabinet_knob_state control_id="box_9_knob" '
    'joint_name="box_9_box_9_knob" />\n'
)
_MODULES_BLOCK = (
    "    <!-- Box 9 has a knob but no buttons in the source assembly. -->\n"
    "    <xacro:control_cabinet_module\n"
    '      index="9"\n'
    '      body_mesh="box_9"\n'
    '      knob_mesh="box_9_knob"\n'
    '      body_xyz="0.005 0.721 0"\n'
    '      knob_xyz="0.12494 0.774526 0.0295"\n'
    '      knob_blade_xyz="0.0135 0 0.018"\n'
    '      knob_blade_size="0.065 0.011 0.036" />\n'
)


def _derive_trimmed_cabinet(target: Path) -> Path:
    """Derive a 32-control cabinet from the sample by dropping box_9_knob.

    Removes the knob consistently across all four coupled sources: catalog
    (control_ids + controls), robot adapter (reachability list), Gazebo
    state plugin (``<control>`` block) and URDF module (joint + link).  The
    scene profile only references the door / switch, which stay untouched.
    """
    shutil.copytree(SAMPLE_CABINET, target)

    controls = target / "cabinet_controls.yaml"
    text = controls.read_text(encoding="utf-8")
    assert _CONTROLS_ID_LINE in text
    assert _CONTROLS_BLOCK in text
    text = text.replace(_CONTROLS_ID_LINE, "").replace(_CONTROLS_BLOCK, "")
    controls.write_text(text, encoding="utf-8")

    adapter = target / "cabinet_robot_adapter.yaml"
    text = adapter.read_text(encoding="utf-8")
    assert _ADAPTER_BLOCK in text
    assert _UNREACHABLE_BOX9_LINE in text
    text = text.replace(_ADAPTER_BLOCK, "")
    text = text.replace(_UNREACHABLE_BOX9_LINE, "", 1)
    adapter.write_text(text, encoding="utf-8")

    gazebo = target / "control_cabinet" / "components" / "gazebo.xacro"
    text = gazebo.read_text(encoding="utf-8")
    assert _GAZEBO_LINE in text
    text = text.replace(_GAZEBO_LINE, "")
    gazebo.write_text(text, encoding="utf-8")

    modules = target / "control_cabinet" / "components" / "cabinet_modules.xacro"
    text = modules.read_text(encoding="utf-8")
    assert _MODULES_BLOCK in text
    text = text.replace(_MODULES_BLOCK, "")
    modules.write_text(text, encoding="utf-8")

    return target


def _run_checker(*arguments: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(CHECKER), *arguments],
        capture_output=True,
        text=True,
        cwd=str(WORKSPACE),
    )


@unittest.skipUnless(
    shutil.which("xacro"), "xacro CLI is required (source the ROS workspace)"
)
class CabinetAssetImportE2ETest(unittest.TestCase):
    """真实样例导入 + 选择 + 32 控件合成柜体参数化证明。"""

    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)
        # CLI 与库 API 现在把目录 / 选择写入 SQLite；逐用例注入独立 DB。
        self.db_url = f"sqlite+aiosqlite:///{self.directory / 'assets.db'}"

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _run_cli(self, *arguments: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(CLI), *arguments],
            capture_output=True,
            text=True,
            cwd=str(WORKSPACE),
            env={**os.environ, "XCZS_DATABASE_URL": self.db_url},
        )

    def _db_assets(self) -> list[tuple[str, str, str, bool]]:
        with sqlite3.connect(self.directory / "assets.db") as connection:
            rows = connection.execute(
                "SELECT kind, name, version, validated FROM assets ORDER BY id"
            ).fetchall()
        return [(str(k), str(n), str(v), bool(ok)) for k, n, v, ok in rows]

    def test_sample_import_validates_end_to_end(self) -> None:
        assets = self.directory / "assets"
        result = self._run_cli(str(SAMPLE_CABINET), "--assets-dir", str(assets))

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn(
            "imported cabinet/demo_cabinet v1.0.0 (validated)", result.stdout
        )
        self.assertEqual(
            [("cabinet", "demo_cabinet", "1.0.0", True)], self._db_assets()
        )
        # 样例整树（含 control_cabinet/components/）应随资产拷贝。
        self.assertTrue(
            (assets / "cabinet" / "demo_cabinet"
             / "control_cabinet" / "components" / "gazebo.xacro").is_file()
        )

    def test_sample_selection_maps_cabinet_env_pointers(self) -> None:
        assets = self.directory / "assets"
        self._run_cli(str(SAMPLE_CABINET), "--assets-dir", str(assets))

        # CLI 的 --select 只对 scene 生效；柜体选择走库 API 持久化。
        from control_gateway.asset_library import AssetLibrary, AssetSelection

        library = AssetLibrary(assets, store=SqlAssetStore(self.db_url))
        library.save_selection(AssetSelection(cabinet="demo_cabinet"))

        result = self._run_cli("--print-env", "--assets-dir", str(assets))
        self.assertEqual(0, result.returncode, result.stderr)
        env = dict(
            line.split("=", 1)
            for line in result.stdout.splitlines()
            if "=" in line
        )
        for key in CABINET_ENV_KEYS:
            self.assertIn(key, env)
            self.assertTrue(env[key].startswith(str(assets)))

    def test_synthetic_non_33_cabinet_passes_asset_mode(self) -> None:
        synthetic = _derive_trimmed_cabinet(
            self.directory / "synth_cabinet_32"
        )
        result = _run_checker(
            "--asset",
            "--controls",
            str(synthetic / "cabinet_controls.yaml"),
            "--scene",
            str(synthetic / "cabinet_scene.yaml"),
            "--adapter",
            str(synthetic / "cabinet_robot_adapter.yaml"),
            "--xacro",
            str(synthetic / "control_cabinet.urdf.xacro"),
            "--cabinet-name",
            "synth_cabinet",
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("32 controls", result.stdout)

    def test_trimmed_cabinet_with_drift_rejected(self) -> None:
        # 从 32 控件派生体上把 box_9_knob 重新塞回 adapter controls——
        # 与 catalog / xacro 不再一致，asset 校验必须拒绝（防参数化掩盖漂移）。
        synthetic = _derive_trimmed_cabinet(self.directory / "synth_drift")
        adapter = synthetic / "cabinet_robot_adapter.yaml"
        text = adapter.read_text(encoding="utf-8")
        marker = "      box_10_knob:\n"
        assert marker in text
        text = text.replace(
            marker,
            "      box_9_knob:\n"
            "        operable: false\n"
            "        unavailable_reason: drifted leftover\n"
            + marker,
            1,
        )
        adapter.write_text(text, encoding="utf-8")

        result = _run_checker(
            "--asset",
            "--controls",
            str(synthetic / "cabinet_controls.yaml"),
            "--scene",
            str(synthetic / "cabinet_scene.yaml"),
            "--adapter",
            str(synthetic / "cabinet_robot_adapter.yaml"),
            "--xacro",
            str(synthetic / "control_cabinet.urdf.xacro"),
            "--cabinet-name",
            "synth_drift",
        )
        self.assertEqual(1, result.returncode)
        self.assertIn("unknown controls", result.stderr)


class CabinetValidatorHookTest(unittest.TestCase):
    """``cabinet_validator`` hook 契约（纯逻辑，替换子进程）。"""

    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _validate(self, returncode: int, stderr: str = "") -> mock.Mock:
        manifest = load_manifest(SAMPLE_CABINET / "manifest.yaml")
        process = mock.Mock(returncode=returncode, stdout="", stderr=stderr)
        with mock.patch(
            "control_gateway.asset_validators.subprocess.run",
            return_value=process,
        ) as run:
            validate = cabinet_validator(checker_path="/fake/checker")
            validate(manifest, SAMPLE_CABINET)
        return run

    def test_hook_builds_asset_command_under_asset_name(self) -> None:
        run = self._validate(returncode=0)
        command = run.call_args.args[0]
        self.assertEqual([command[0], command[1]], [sys.executable, "/fake/checker"])
        self.assertIn("--asset", command)
        self.assertIn("--cabinet-name", command)
        self.assertEqual(
            command[command.index("--cabinet-name") + 1], "demo_cabinet"
        )
        # 五个角色路径按 manifest 角色解析到样例目录内。
        self.assertEqual(
            command[command.index("--controls") + 1],
            str(SAMPLE_CABINET / "cabinet_controls.yaml"),
        )
        self.assertEqual(
            command[command.index("--xacro") + 1],
            str(SAMPLE_CABINET / "control_cabinet.urdf.xacro"),
        )

    def test_hook_raises_value_error_on_failure(self) -> None:
        manifest = load_manifest(SAMPLE_CABINET / "manifest.yaml")
        process = mock.Mock(returncode=3, stdout="", stderr="boom: kaboom")
        with mock.patch(
            "control_gateway.asset_validators.subprocess.run",
            return_value=process,
        ):
            validate = cabinet_validator(checker_path="/fake/checker")
            with self.assertRaisesRegex(ValueError, "boom: kaboom"):
                validate(manifest, SAMPLE_CABINET)


if __name__ == "__main__":
    unittest.main()
