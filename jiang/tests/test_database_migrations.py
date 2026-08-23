"""Alembic 启动迁移与历史无版本 SQLite 库接管测试。"""

from __future__ import annotations

import logging
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from alembic import command
from pydantic import ValidationError
from sqlalchemy import create_engine, inspect, text

JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

from app.database.migrations import (  # noqa: E402
    _alembic_config,
    migrate_database,
)
from app.config import Settings  # noqa: E402

HEAD_REVISION = "9c1d2e3f4a5b"
TASKS_REVISION = "5d7e8f9a0b1c"
EXPECTED_TABLES = {
    "alembic_version",
    "assets",
    "selection",
    "task_progress_events",
    "task_records",
    "users",
}


class DatabaseMigrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def _urls(self, name: str) -> tuple[str, str]:
        database = self.root / name
        return (
            f"sqlite+aiosqlite:///{database}",
            f"sqlite:///{database}",
        )

    def _revision(self, sync_url: str) -> str:
        engine = create_engine(sync_url)
        try:
            with engine.connect() as connection:
                return str(
                    connection.execute(
                        text("SELECT version_num FROM alembic_version")
                    ).scalar_one()
                )
        finally:
            engine.dispose()

    def test_fresh_database_is_created_at_head(self) -> None:
        async_url, sync_url = self._urls("fresh.db")

        migrate_database(async_url)

        engine = create_engine(sync_url)
        try:
            self.assertEqual(EXPECTED_TABLES, set(inspect(engine).get_table_names()))
        finally:
            engine.dispose()
        self.assertEqual(HEAD_REVISION, self._revision(sync_url))

    def test_known_unversioned_database_is_stamped_then_upgraded(self) -> None:
        async_url, sync_url = self._urls("legacy.db")
        command.upgrade(_alembic_config(async_url), TASKS_REVISION)
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(text("DROP TABLE alembic_version"))
        finally:
            engine.dispose()

        migrate_database(async_url)

        engine = create_engine(sync_url)
        try:
            selection_columns = {
                column["name"] for column in inspect(engine).get_columns("selection")
            }
        finally:
            engine.dispose()
        self.assertIn("toolset", selection_columns)
        self.assertEqual(HEAD_REVISION, self._revision(sync_url))

    def test_known_gripper_variant_legacy_shape_is_preserved_and_upgraded(
        self,
    ) -> None:
        async_url, sync_url = self._urls("legacy-gripper.db")
        command.upgrade(_alembic_config(async_url), TASKS_REVISION)
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(
                    text(
                        "ALTER TABLE selection ADD COLUMN "
                        "gripper_variant VARCHAR(64)"
                    )
                )
                connection.execute(
                    text(
                        "INSERT INTO selection "
                        "(id, scene, cabinet, gripper_variant) "
                        "VALUES (1, 'legacy_scene', 'legacy_cabinet', 'default')"
                    )
                )
                connection.execute(text("DROP TABLE alembic_version"))
        finally:
            engine.dispose()

        migrate_database(async_url)

        engine = create_engine(sync_url)
        try:
            columns = {
                column["name"] for column in inspect(engine).get_columns("selection")
            }
            with engine.connect() as connection:
                row = connection.execute(
                    text(
                        "SELECT scene, cabinet, gripper_variant, toolset "
                        "FROM selection WHERE id = 1"
                    )
                ).one()
        finally:
            engine.dispose()
        self.assertEqual(
            {"id", "scene", "cabinet", "gripper_variant", "toolset"}, columns
        )
        self.assertEqual(
            ("legacy_scene", "legacy_cabinet", "default", None), tuple(row)
        )
        self.assertEqual(HEAD_REVISION, self._revision(sync_url))

    def test_unknown_unversioned_schema_fails_closed(self) -> None:
        async_url, sync_url = self._urls("damaged.db")
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(text("CREATE TABLE users (id INTEGER PRIMARY KEY)"))
        finally:
            engine.dispose()

        with self.assertRaisesRegex(RuntimeError, "拒绝自动修改"):
            migrate_database(async_url)

        engine = create_engine(sync_url)
        try:
            self.assertNotIn("alembic_version", inspect(engine).get_table_names())
        finally:
            engine.dispose()

    def test_known_columns_with_wrong_type_fail_closed(self) -> None:
        async_url, sync_url = self._urls("wrong-type.db")
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(
                    text(
                        "CREATE TABLE users ("
                        "id INTEGER NOT NULL PRIMARY KEY, "
                        "username INTEGER NOT NULL, "
                        "hashed_password VARCHAR(128) NOT NULL, "
                        "role VARCHAR(16) NOT NULL, "
                        "is_active BOOLEAN NOT NULL, "
                        "created_at DATETIME NOT NULL, "
                        "updated_at DATETIME NOT NULL)"
                    )
                )
                connection.execute(
                    text(
                        "CREATE UNIQUE INDEX ix_users_username "
                        "ON users (username)"
                    )
                )
        finally:
            engine.dispose()

        with self.assertRaisesRegex(RuntimeError, "拒绝自动修改"):
            migrate_database(async_url)

    def test_known_columns_without_required_server_default_fail_closed(self) -> None:
        async_url, sync_url = self._urls("missing-default.db")
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(
                    text(
                        "CREATE TABLE users ("
                        "id INTEGER NOT NULL PRIMARY KEY, "
                        "username VARCHAR(64) NOT NULL, "
                        "hashed_password VARCHAR(128) NOT NULL, "
                        "role VARCHAR(16) NOT NULL, "
                        "is_active BOOLEAN NOT NULL, "
                        "created_at DATETIME NOT NULL, "
                        "updated_at DATETIME NOT NULL)"
                    )
                )
                connection.execute(
                    text(
                        "CREATE UNIQUE INDEX ix_users_username "
                        "ON users (username)"
                    )
                )
        finally:
            engine.dispose()

        with self.assertRaisesRegex(RuntimeError, "拒绝自动修改"):
            migrate_database(async_url)

    def test_empty_version_table_recovers_from_base(self) -> None:
        async_url, sync_url = self._urls("empty-version.db")
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(
                    text(
                        "CREATE TABLE alembic_version "
                        "(version_num VARCHAR(32) NOT NULL PRIMARY KEY)"
                    )
                )
        finally:
            engine.dispose()

        migrate_database(async_url)

        self.assertEqual(HEAD_REVISION, self._revision(sync_url))

    def test_repeated_migration_is_idempotent(self) -> None:
        async_url, sync_url = self._urls("repeat.db")
        migrate_database(async_url)
        migrate_database(async_url)
        self.assertEqual(HEAD_REVISION, self._revision(sync_url))

    def test_versioned_database_with_corrupt_schema_fails_before_upgrade(
        self,
    ) -> None:
        async_url, sync_url = self._urls("corrupt-versioned.db")
        command.upgrade(_alembic_config(async_url), TASKS_REVISION)
        engine = create_engine(sync_url)
        try:
            with engine.begin() as connection:
                connection.execute(text("DROP INDEX ix_task_records_status"))
        finally:
            engine.dispose()

        with self.assertRaisesRegex(RuntimeError, "版本与实际 schema 不一致"):
            migrate_database(async_url)

        # The pre-upgrade fingerprint check must not stamp or advance a
        # damaged database before the operator has inspected a backup.
        self.assertEqual(TASKS_REVISION, self._revision(sync_url))

    def test_unsupported_database_dialect_is_rejected_before_driver_load(
        self,
    ) -> None:
        with self.assertRaisesRegex(ValueError, "仅支持"):
            migrate_database("postgresql+asyncpg://user:pass@localhost/xczs")

    def test_sqlite_uri_memory_database_is_rejected(self) -> None:
        uri = (
            "sqlite+aiosqlite:///file:memdb_audit"
            "?mode=memory&cache=shared&uri=true"
        )
        with self.assertRaisesRegex(ValueError, "普通文件型 SQLite"):
            migrate_database(uri)

    def test_sqlite_authority_is_rejected_before_lock_creation(self) -> None:
        for url in (
            "sqlite+aiosqlite://localhost/hosted.db",
            "sqlite+aiosqlite://user:pass@localhost/hosted.db",
        ):
            with self.subTest(url=url), self.assertRaisesRegex(
                ValueError, "无 authority"
            ):
                migrate_database(url)

    def test_runtime_settings_reject_unsupported_or_memory_database(self) -> None:
        for database_url in (
            "postgresql+asyncpg://user:pass@localhost/xczs",
            "sqlite+aiosqlite:///:memory:",
            (
                "sqlite+aiosqlite:///file:memdb_settings"
                "?mode=memory&cache=shared&uri=true"
            ),
            "sqlite+aiosqlite://localhost/hosted.db",
            "sqlite:///tmp/xczs.db",
        ):
            with self.subTest(database_url=database_url), self.assertRaises(
                ValidationError
            ):
                Settings(
                    database_url=database_url,
                    secret_key="s" * 32,
                    _env_file=None,
                )

    def test_programmatic_migration_keeps_application_loggers_enabled(self) -> None:
        async_url, _sync_url = self._urls("logging.db")
        logger = logging.getLogger("app.database.migration-test")
        logger.disabled = False

        migrate_database(async_url)

        self.assertFalse(logger.disabled)

    def test_concurrent_first_start_is_serialized(self) -> None:
        async_url, sync_url = self._urls("concurrent.db")

        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(migrate_database, [async_url, async_url]))

        self.assertEqual([None, None], results)
        self.assertEqual(HEAD_REVISION, self._revision(sync_url))


if __name__ == "__main__":
    unittest.main()
