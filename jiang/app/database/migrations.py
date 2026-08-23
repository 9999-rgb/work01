"""数据库 schema 的唯一启动迁移入口。

运行时、资产 CLI 与任务记录 store 都必须先经过这里，再访问 ORM 表。这样可以
避免 ``metadata.create_all()`` 创建了“看起来存在、实际缺少新字段”的半升级
数据库。

项目早期版本没有写入 ``alembic_version``。对于这类历史 SQLite 数据库，仅当
表、列、关键索引与某个已发布 revision 精确匹配时才自动 stamp；未知结构拒绝
猜测，要求人工检查/备份后再处理。
"""

from __future__ import annotations

import fcntl
import os
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator, Optional

from alembic import command
from alembic.config import Config
from sqlalchemy import create_engine, inspect, text
from sqlalchemy.engine import Inspector, make_url
from sqlalchemy.sql.sqltypes import String

from app.config import settings, validate_sqlite_file_url

_JIANG_DIR = Path(__file__).resolve().parents[2]
_ALEMBIC_INI = _JIANG_DIR / "alembic.ini"

_REV_USERS = "75036a3d0043"
_REV_ASSETS = "3f1a2b4c5d6e"
_REV_TASKS = "5d7e8f9a0b1c"
_REV_HEAD = "9c1d2e3f4a5b"

_EXPECTED_COLUMNS: dict[str, frozenset[str]] = {
    "users": frozenset(
        {
            "id",
            "username",
            "hashed_password",
            "role",
            "is_active",
            "created_at",
            "updated_at",
        }
    ),
    "assets": frozenset(
        {
            "id",
            "kind",
            "name",
            "version",
            "description",
            "path",
            "files",
            "references",
            "imported_at",
            "validated",
        }
    ),
    "selection@assets": frozenset({"id", "scene", "cabinet"}),
    "selection@head": frozenset({"id", "scene", "cabinet", "toolset"}),
    "task_records": frozenset(
        {
            "id",
            "task_id",
            "type",
            "status",
            "phase",
            "progress",
            "message",
            "cabinet",
            "control_id",
            "control_name",
            "command",
            "target_state",
            "target_position",
            "force",
            "request",
            "result",
            "business_data",
            "failure_code",
            "failure_reason",
            "failure_details",
            "duration_seconds",
            "created_at",
            "started_at",
            "completed_at",
            "updated_at",
        }
    ),
    "task_progress_events": frozenset(
        {
            "id",
            "task_id",
            "sequence",
            "event",
            "status",
            "phase",
            "progress",
            "message",
            "business_data",
            "timestamp",
            "elapsed_seconds",
        }
    ),
}

# 项目早期曾由 ``metadata.create_all()`` 产生该字段，随后业务层将其作为死代码
# 移除，但现场数据库中可能仍保留。接管时只容忍这一种精确旧形状，且保留字段
# 和数据；不会把任意额外列误判为已知 revision。
_LEGACY_SELECTION_COLUMN = "gripper_variant"

_EXPECTED_INDEXES: dict[str, dict[str, tuple[tuple[str, ...], bool]]] = {
    "users": {"ix_users_username": (("username",), True)},
    "assets": {"ix_assets_kind": (("kind",), False)},
    "task_records": {
        "ix_task_records_task_id": (("task_id",), True),
        "ix_task_records_type": (("type",), False),
        "ix_task_records_status": (("status",), False),
        "ix_task_records_cabinet": (("cabinet",), False),
        "ix_task_records_control_id": (("control_id",), False),
        "ix_task_records_failure_code": (("failure_code",), False),
        "ix_task_records_created_at": (("created_at",), False),
    },
    "task_progress_events": {
        "ix_task_progress_events_task_id": (("task_id",), False),
        "ix_task_progress_events_timestamp": (("timestamp",), False),
    },
}

_THREAD_LOCKS: dict[str, threading.Lock] = {}
_THREAD_LOCKS_GUARD = threading.Lock()


def _database_url(database_url: Optional[str]) -> str:
    return database_url or os.environ.get("XCZS_DATABASE_URL") or settings.database_url


def _validate_supported_url(database_url: str) -> None:
    validate_sqlite_file_url(database_url, allow_sync_driver=True)


def _sync_sqlite_url(database_url: str) -> str:
    if database_url.startswith("sqlite+aiosqlite:"):
        return database_url.replace("sqlite+aiosqlite:", "sqlite:", 1)
    return database_url


def _migration_url(database_url: str) -> str:
    """Alembic 的 env 使用异步引擎；同步 SQLite URL 在这里补齐驱动。"""
    if database_url.startswith("sqlite:") and not database_url.startswith(
        "sqlite+aiosqlite:"
    ):
        return database_url.replace("sqlite:", "sqlite+aiosqlite:", 1)
    return database_url


def _sqlite_path(database_url: str) -> Path:
    _validate_supported_url(database_url)
    parsed = make_url(_sync_sqlite_url(database_url))
    assert parsed.database is not None
    return Path(parsed.database).expanduser().resolve()


@contextmanager
def _migration_lock(database_url: str) -> Iterator[None]:
    """同进程线程锁 + SQLite 文件锁，防止 CLI/Web 同时迁移。"""
    with _THREAD_LOCKS_GUARD:
        thread_lock = _THREAD_LOCKS.setdefault(database_url, threading.Lock())
    with thread_lock:
        parsed = make_url(_sync_sqlite_url(database_url))
        if parsed.get_backend_name() != "sqlite":
            yield
            return
        database_path = _sqlite_path(database_url)
        database_path.parent.mkdir(parents=True, exist_ok=True)
        lock_path = database_path.with_name(f".{database_path.name}.migrate.lock")
        with lock_path.open("a", encoding="utf-8") as lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _alembic_config(database_url: str) -> Config:
    config = Config(str(_ALEMBIC_INI))
    # env.py 优先读取该属性，避免测试/CLI 的显式 URL 被全局 settings 覆盖。
    config.attributes["database_url"] = _migration_url(database_url)
    return config


def _index_matches(
    inspector: Inspector,
    table: str,
    expected: dict[str, tuple[tuple[str, ...], bool]],
) -> bool:
    actual = {
        item["name"]: (
            tuple(item.get("column_names") or ()),
            bool(item.get("unique", False)),
        )
        for item in inspector.get_indexes(table)
    }
    return all(actual.get(name) == signature for name, signature in expected.items())


def _column_shapes_match(
    inspector: Inspector,
    table: str,
    expected_names: frozenset[str],
    *,
    legacy_selection_column: bool = False,
) -> bool:
    """比较列名、类型 affinity/长度、nullable 与主键标志。"""
    # 延迟导入，既注册完整 metadata，也避免 store 模块导入期间形成环。
    from app.assets.models import Asset, Selection  # noqa: F401
    from app.auth.models import User  # noqa: F401
    from app.database.base import Base
    from app.tasks.models import TaskProgressEvent, TaskRecord  # noqa: F401

    actual_columns = {
        column["name"]: column for column in inspector.get_columns(table)
    }
    actual_expected_names = expected_names
    if legacy_selection_column:
        actual_expected_names |= frozenset({_LEGACY_SELECTION_COLUMN})
    if frozenset(actual_columns) != actual_expected_names:
        return False
    expected_table = Base.metadata.tables[table]

    def normalize_default(value: object) -> str | None:
        if value is None:
            return None
        normalized = " ".join(str(value).strip().upper().split())
        while normalized.startswith("(") and normalized.endswith(")"):
            normalized = normalized[1:-1].strip()
        return normalized

    for name in expected_names:
        actual = actual_columns[name]
        expected = expected_table.c[name]
        actual_type = actual["type"]
        if actual_type._type_affinity is not expected.type._type_affinity:
            return False
        if isinstance(expected.type, String) and (
            getattr(actual_type, "length", None) != expected.type.length
        ):
            return False
        if bool(actual["nullable"]) != bool(expected.nullable):
            return False
        if bool(actual["primary_key"]) != bool(expected.primary_key):
            return False
        expected_default = (
            expected.server_default.arg.compile(dialect=inspector.bind.dialect)
            if expected.server_default is not None
            else None
        )
        if normalize_default(actual.get("default")) != normalize_default(
            expected_default
        ):
            return False
    if legacy_selection_column:
        legacy = actual_columns[_LEGACY_SELECTION_COLUMN]
        legacy_type = legacy["type"]
        if legacy_type._type_affinity is not String()._type_affinity:
            return False
        if getattr(legacy_type, "length", None) != 64:
            return False
        if not bool(legacy["nullable"]):
            return False
        if bool(legacy["primary_key"]):
            return False
        if normalize_default(legacy.get("default")) is not None:
            return False
    return True


def _matches_revision(
    inspector: Inspector,
    *,
    tables: frozenset[str],
    selection_columns: frozenset[str] | None,
    legacy_selection_column: bool = False,
) -> bool:
    actual_tables = frozenset(inspector.get_table_names()) - {"alembic_version"}
    if actual_tables != tables:
        return False
    for table in tables:
        key = (
            "selection@head"
            if table == "selection" and selection_columns == _EXPECTED_COLUMNS["selection@head"]
            else "selection@assets"
            if table == "selection"
            else table
        )
        if not _column_shapes_match(
            inspector,
            table,
            _EXPECTED_COLUMNS[key],
            legacy_selection_column=(
                table == "selection" and legacy_selection_column
            ),
        ):
            return False
        if not _index_matches(inspector, table, _EXPECTED_INDEXES.get(table, {})):
            return False
    if "assets" in tables:
        unique_constraints = {
            tuple(item.get("column_names") or ())
            for item in inspector.get_unique_constraints("assets")
        }
        if ("kind", "name") not in unique_constraints:
            return False
    return True


def _legacy_revision(inspector: Inspector) -> str:
    users = frozenset({"users"})
    assets = frozenset({"users", "assets", "selection"})
    tasks = frozenset(
        {"users", "assets", "selection", "task_records", "task_progress_events"}
    )
    candidates = (
        (_REV_USERS, users, None, False),
        (_REV_ASSETS, assets, _EXPECTED_COLUMNS["selection@assets"], False),
        (_REV_TASKS, tasks, _EXPECTED_COLUMNS["selection@assets"], False),
        (_REV_HEAD, tasks, _EXPECTED_COLUMNS["selection@head"], False),
        # 精确识别项目实际产生过的残留 gripper_variant 结构；先 stamp 到对应
        # revision，再由正常 Alembic 链补 toolset，旧列与旧数据原样保留。
        (_REV_ASSETS, assets, _EXPECTED_COLUMNS["selection@assets"], True),
        (_REV_TASKS, tasks, _EXPECTED_COLUMNS["selection@assets"], True),
        (_REV_HEAD, tasks, _EXPECTED_COLUMNS["selection@head"], True),
    )
    for revision, tables, selection_columns, legacy_selection_column in candidates:
        if _matches_revision(
            inspector,
            tables=tables,
            selection_columns=selection_columns,
            legacy_selection_column=legacy_selection_column,
        ):
            return revision
    actual_tables = sorted(
        set(inspector.get_table_names()) - {"alembic_version"}
    )
    raise RuntimeError(
        "检测到未受 Alembic 管理且结构不匹配任何已知版本的数据库；"
        f"拒绝自动修改（tables={actual_tables}）。请先备份并人工核对 schema。"
    )


def _revision_shape_matches(inspector: Inspector, revision: str) -> bool:
    """Require a versioned database to match the schema that revision owns."""

    users = frozenset({"users"})
    assets = frozenset({"users", "assets", "selection"})
    tasks = frozenset(
        {"users", "assets", "selection", "task_records", "task_progress_events"}
    )
    candidates: dict[
        str,
        tuple[frozenset[str], frozenset[str] | None, bool],
    ] = {
        _REV_USERS: (users, None, False),
        _REV_ASSETS: (assets, _EXPECTED_COLUMNS["selection@assets"], False),
        _REV_TASKS: (tasks, _EXPECTED_COLUMNS["selection@assets"], False),
        _REV_HEAD: (tasks, _EXPECTED_COLUMNS["selection@head"], False),
    }
    expected = candidates.get(revision)
    if expected is None:
        return False
    tables, selection_columns, _legacy = expected
    if _matches_revision(
        inspector,
        tables=tables,
        selection_columns=selection_columns,
    ):
        return True
    # ``gripper_variant`` is the one historical create_all residue this
    # project emitted.  Preserve it and its data, but tolerate no other shape.
    return "selection" in tables and _matches_revision(
        inspector,
        tables=tables,
        selection_columns=selection_columns,
        legacy_selection_column=True,
    )


def _assert_versioned_schema(
    inspector: Inspector,
    revision: str,
    *,
    expected_revision: str | None = None,
) -> None:
    if expected_revision is not None and revision != expected_revision:
        raise RuntimeError(
            "数据库迁移后的 Alembic 版本不正确；"
            f"expected={expected_revision}, actual={revision}。"
        )
    if not _revision_shape_matches(inspector, revision):
        raise RuntimeError(
            "数据库声明的 Alembic 版本与实际 schema 不一致；拒绝自动修改"
            f"（revision={revision}）。请先备份并人工核对 schema。"
        )


def _prepare_sqlite_baseline(database_url: str, config: Config) -> None:
    engine = create_engine(_sync_sqlite_url(database_url))
    try:
        inspector = inspect(engine)
        tables = set(inspector.get_table_names())
        if not tables:
            return
        if "alembic_version" in tables:
            with engine.connect() as connection:
                versions = connection.execute(
                    text("SELECT version_num FROM alembic_version")
                ).scalars().all()
            if len(versions) == 1 and versions[0]:
                _assert_versioned_schema(inspector, str(versions[0]))
                return
            if len(versions) > 1:
                raise RuntimeError("alembic_version 含多个版本行，拒绝自动迁移")
            if tables == {"alembic_version"}:
                # Alembic 在首次迁移开始前可能已创建空版本表；其余表为空时
                # 仍可安全地从 base 正常 upgrade。
                return
        revision = _legacy_revision(inspector)
    finally:
        engine.dispose()
    command.stamp(config, revision)


def _verify_head_schema(database_url: str) -> None:
    """Prove that Alembic really left a complete, internally consistent head."""

    engine = create_engine(_sync_sqlite_url(database_url))
    try:
        inspector = inspect(engine)
        if "alembic_version" not in inspector.get_table_names():
            raise RuntimeError("数据库迁移完成后缺少 alembic_version 表")
        with engine.connect() as connection:
            versions = connection.execute(
                text("SELECT version_num FROM alembic_version")
            ).scalars().all()
        if len(versions) != 1 or not versions[0]:
            raise RuntimeError("数据库迁移完成后没有唯一有效的 Alembic 版本")
        _assert_versioned_schema(
            inspector,
            str(versions[0]),
            expected_revision=_REV_HEAD,
        )
    finally:
        engine.dispose()


def migrate_database(database_url: Optional[str] = None) -> None:
    """把指定数据库升级到 Alembic head；失败时不启动上层服务。"""
    url = _database_url(database_url)
    _validate_supported_url(url)
    config = _alembic_config(url)
    with _migration_lock(url):
        _prepare_sqlite_baseline(url, config)
        command.upgrade(config, "head")
        _verify_head_schema(url)


__all__ = ["migrate_database"]
