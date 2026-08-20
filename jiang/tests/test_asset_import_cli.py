"""Integration tests for the ``scripts/xczs_import_asset`` CLI (pure, no ROS).

Runs the real CLI as a subprocess against a scratch asset library, so these
tests exercise the whole import chain: manifest validation, whole-tree copy,
scene reference normalization, the real ``check_scene_config`` semantic
checker, catalog/selection persistence, and the selection -> env mapping the
launch script consumes.  No ROS workspace is required — the sample's
``nav2_map`` is normalized to an absolute path inside the scratch library.
"""

from __future__ import annotations

import os
import sqlite3
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CLI = WORKSPACE / "scripts" / "xczs_import_asset"
MAP_FIELDS = {
    "image": "inspection_map.pgm",
    "resolution": 0.05,
    "origin": [-5.0, -5.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
}


def _scene_document(name: str) -> Dict[str, Any]:
    return {
        "scenes": [
            {
                "name": name,
                "spawn_cabinet": True,
                "model": None,
                "nav2_map": "maps/inspection_map.yaml",
                "robot_spawn": {"x": 0.0, "y": 0.0, "z": 0.515, "yaw": 1.57079632679},
            }
        ]
    }


def _write_scene_asset(source: Path, name: str = "cli_scene") -> Path:
    asset = source / name
    (asset / "maps").mkdir(parents=True)
    (asset / "maps" / "inspection_map.yaml").write_text(
        yaml.safe_dump(MAP_FIELDS, sort_keys=False), encoding="utf-8"
    )
    (asset / "maps" / "inspection_map.pgm").write_bytes(b"\x00")
    (asset / "scenes.yaml").write_text(
        yaml.safe_dump(_scene_document(name), sort_keys=False), encoding="utf-8"
    )
    (asset / "cabinet_instances.yaml").write_text(
        "instances: []\n", encoding="utf-8"
    )
    (asset / "manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "kind": "scene",
                "name": name,
                "version": "1.0.0",
                "description": "CLI integration fixture.",
                "files": {
                    "scenes": "scenes.yaml",
                    "instances": "cabinet_instances.yaml",
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return asset


class AssetImportCliTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._temporary_directory.name)
        self.assets_dir = self.directory / "assets"
        # CLI 现在把目录 / 选择写入 SQLite；逐用例注入独立 DB，隔离于 conftest
        # 的会话级 DB 与其他用例。
        self.db_path = self.directory / "assets.db"

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _run(self, *arguments: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [sys.executable, str(CLI), *arguments],
            capture_output=True,
            text=True,
            cwd=str(WORKSPACE),
            env={**os.environ, "XCZS_DATABASE_URL": f"sqlite+aiosqlite:///{self.db_path}"},
        )

    def _db_assets(self) -> list[tuple[str, str, str, bool]]:
        with sqlite3.connect(self.db_path) as connection:
            rows = connection.execute(
                "SELECT kind, name, version, validated FROM assets ORDER BY id"
            ).fetchall()
        return [(str(k), str(n), str(v), bool(ok)) for k, n, v, ok in rows]

    def _db_selection(self) -> tuple[str | None, str | None]:
        with sqlite3.connect(self.db_path) as connection:
            row = connection.execute(
                "SELECT scene, cabinet FROM selection WHERE id=1"
            ).fetchone()
        return (None, None) if row is None else tuple(row)

    def test_import_select_print_env_end_to_end(self) -> None:
        source = _write_scene_asset(self.directory / "source")

        result = self._run(
            str(source),
            "--assets-dir",
            str(self.assets_dir),
            "--select",
            "--print-env",
        )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("imported scene/cli_scene v1.0.0 (validated)", result.stdout)

        # Catalog and selection persisted in SQLite (not YAML).
        self.assertEqual(
            [("scene", "cli_scene", "1.0.0", True)], self._db_assets()
        )
        self.assertEqual("cli_scene", self._db_selection()[0])

        # Env mapping points at the imported, normalized files.
        root = self.assets_dir / "scene" / "cli_scene"
        self.assertIn(f"SCENES_CONFIG={root / 'scenes.yaml'}", result.stdout)
        self.assertIn("SCENE=cli_scene", result.stdout)
        self.assertIn(
            f"CABINET_INSTANCES_PATH={root / 'cabinet_instances.yaml'}",
            result.stdout,
        )

        # The copy is self-contained: relative nav2_map became absolute.
        scenes = yaml.safe_load((root / "scenes.yaml").read_text())
        self.assertEqual(
            str((root / "maps" / "inspection_map.yaml").resolve()),
            scenes["scenes"][0]["nav2_map"],
        )

    def test_print_env_without_source_reads_persisted_selection(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        imported = self._run(
            str(source),
            "--assets-dir",
            str(self.assets_dir),
            "--select",
        )
        self.assertEqual(0, imported.returncode, imported.stderr)

        result = self._run("--assets-dir", str(self.assets_dir), "--print-env")

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("SCENE=cli_scene", result.stdout)

    def test_duplicate_import_is_rejected_and_force_replaces(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        self.assertEqual(
            0, self._run(str(source), "--assets-dir", str(self.assets_dir)).returncode
        )

        duplicate = self._run(str(source), "--assets-dir", str(self.assets_dir))
        self.assertEqual(1, duplicate.returncode)
        self.assertIn("already imported", duplicate.stderr)

        (source / "manifest.yaml").write_text(
            (source / "manifest.yaml").read_text().replace("1.0.0", "2.0.0"),
            encoding="utf-8",
        )
        forced = self._run(
            str(source), "--assets-dir", str(self.assets_dir), "--force"
        )
        self.assertEqual(0, forced.returncode, forced.stderr)
        self.assertIn("v2.0.0", forced.stdout)

    def test_invalid_manifest_is_rejected(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        (source / "manifest.yaml").write_text(
            "kind: scene\nname: Bad-Case\n", encoding="utf-8"
        )
        result = self._run(str(source), "--assets-dir", str(self.assets_dir))
        self.assertEqual(1, result.returncode)
        self.assertIn("FAIL:", result.stderr)

    def test_scene_checker_rejects_broken_map(self) -> None:
        source = _write_scene_asset(self.directory / "source")
        # Remove the map image the checker requires.
        (source / "maps" / "inspection_map.pgm").unlink()
        result = self._run(str(source), "--assets-dir", str(self.assets_dir))
        self.assertEqual(1, result.returncode)
        self.assertIn("FAIL", result.stderr)
        # A failed import leaves no trace in the library.
        self.assertFalse((self.assets_dir / "scene" / "cli_scene").exists())
        self.assertEqual([], self._db_assets())


if __name__ == "__main__":
    unittest.main()
