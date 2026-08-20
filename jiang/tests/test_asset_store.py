"""Unit tests for the SQLite asset store (pure, no ROS).

Cover the ``AssetStore`` protocol implemented by ``app.assets.store.SqlAssetStore``:
catalog CRUD (list / get / put with kind+name upsert / delete), the selection
singleton read / write, and the ``AssetNotFoundError`` raised for missing
assets.  Each test points the store at its own temporary SQLite file, so the
store's URL-keyed engine cache is exercised against a fresh database every time.
"""

from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path

JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

from app.assets.store import SqlAssetStore  # noqa: E402
from control_gateway.asset_library import (  # noqa: E402
    AssetNotFoundError,
    AssetRecord,
    AssetSelection,
)


def _record(**overrides: object) -> AssetRecord:
    values = {
        "kind": "scene",
        "name": "demo",
        "version": "1.0.0",
        "description": "fixture",
        "path": "scene/demo",
        "files": {"scenes": "scenes.yaml"},
        "references": {},
        "imported_at": "2026-08-18T00:00:00Z",
        "validated": True,
    }
    values.update(overrides)  # type: ignore[arg-type]
    return AssetRecord(**values)  # type: ignore[arg-type]


class SqlAssetStoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.db_path = Path(self._temporary_directory.name) / "assets.db"
        self.store = SqlAssetStore(f"sqlite+aiosqlite:///{self.db_path}")

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def test_list_empty(self) -> None:
        self.assertEqual([], self.store.list_assets())

    def test_put_then_list_and_get_roundtrip(self) -> None:
        self.store.put_asset(_record())
        self.assertEqual(["demo"], [r.name for r in self.store.list_assets()])

        fetched = self.store.get_asset("scene", "demo")
        self.assertEqual("scene", fetched.kind)
        self.assertEqual("1.0.0", fetched.version)
        self.assertTrue(fetched.validated)
        self.assertEqual({"scenes": "scenes.yaml"}, dict(fetched.files))
        self.assertEqual("scene/demo", fetched.path)

    def test_put_upserts_same_kind_and_name(self) -> None:
        self.store.put_asset(_record(version="1.0.0"))
        self.store.put_asset(_record(version="2.0.0"))
        records = self.store.list_assets()
        self.assertEqual(1, len(records))
        self.assertEqual("2.0.0", records[0].version)

    def test_put_distinct_names_are_separate_rows(self) -> None:
        self.store.put_asset(_record(name="a"))
        self.store.put_asset(_record(name="b"))
        self.assertEqual({"a", "b"}, {r.name for r in self.store.list_assets()})

    def test_delete_removes_row(self) -> None:
        self.store.put_asset(_record())
        self.store.delete_asset("scene", "demo")
        self.assertEqual([], self.store.list_assets())

    def test_delete_missing_is_noop(self) -> None:
        # Deleting an absent asset must not raise.
        self.store.delete_asset("scene", "nope")

    def test_get_missing_raises(self) -> None:
        with self.assertRaises(AssetNotFoundError):
            self.store.get_asset("scene", "nope")

    def test_selection_defaults_when_empty(self) -> None:
        self.assertEqual(AssetSelection(), self.store.load_selection())

    def test_selection_roundtrip(self) -> None:
        self.store.save_selection(
            AssetSelection(scene="demo", cabinet=None)
        )
        loaded = self.store.load_selection()
        self.assertEqual("demo", loaded.scene)
        self.assertIsNone(loaded.cabinet)

    def test_selection_overwrites_previous(self) -> None:
        self.store.save_selection(AssetSelection(scene="a", cabinet="b"))
        self.store.save_selection(AssetSelection())
        self.assertEqual(AssetSelection(), self.store.load_selection())


if __name__ == "__main__":
    unittest.main()
