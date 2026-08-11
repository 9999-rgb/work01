"""全测试模块共享的会话级临时数据库与认证环境变量。

Python ``settings`` 是模块级单例（首次 ``from app.config import settings``
时从环境变量读取并缓存）。所有需要 DB 的测试模块必须在导入
``app.main`` 之前共享同一组环境变量，否则后续模块的覆盖不会生效。

本 conftest.py 在 pytest 收集阶段（任何模块导入前）先设置 env，
确保全会话只有一份 settings，所有模块共用同一个临时 SQLite 库。
"""

from __future__ import annotations

import atexit
import os
import tempfile

# ── 全局临时数据库（会话级，pytest 退出后自动删除） ──────────────
_SESSION_TMP = tempfile.TemporaryDirectory(prefix="xczs_pytest_")

os.environ.setdefault("XCZS_AUTH_ENABLED", "true")
os.environ.setdefault("XCZS_SECRET_KEY", "pytest-global-secret")
os.environ.setdefault("XCZS_ADMIN_PASSWORD", "pytest-admin-pass")
os.environ.setdefault(
    "XCZS_DATABASE_URL",
    f"sqlite+aiosqlite:///{_SESSION_TMP.name}/session.db",
)


def _cleanup() -> None:
    try:
        _SESSION_TMP.cleanup()
    except Exception:
        pass


atexit.register(_cleanup)
