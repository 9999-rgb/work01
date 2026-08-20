"""FastAPI 应用配置。

使用 pydantic-settings 从环境变量读取，前缀 ``XCZS_`` 覆盖默认值。
数据库连接串通过 ``XCZS_DATABASE_URL`` 注入，默认 SQLite (WAL)；切换
PostgreSQL 仅需修改该变量，无需改代码。
"""

from __future__ import annotations

import logging
import secrets
from pathlib import Path
from typing import Literal, Optional

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

# ``jiang/`` 目录（本文件所在目录的上一级即为工作区根目录的相对基准）。
_JIANG_DIR = Path(__file__).resolve().parents[1]
_WORKSPACE = _JIANG_DIR.parent

logger = logging.getLogger(__name__)


def _generate_ephemeral_secret_key() -> str:
    logger.warning(
        "未配置 XCZS_SECRET_KEY，已为当前进程生成强随机 JWT "
        "密钥；服务重启后现有 token 会失效。"
    )
    return secrets.token_urlsafe(48)


class Settings(BaseSettings):
    """应用配置。

    环境变量名规则：``XCZS_DATABASE_URL``、``XCZS_SECRET_KEY`` 等。
    兼容旧启动脚本中的路径变量（``CABINET_INSTANCES_PATH`` 等不带前缀）。
    """

    model_config = SettingsConfigDict(
        env_prefix="XCZS_",
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    # ── 数据库 ──────────────────────────────────────────────────────
    # 默认 SQLite WAL；PostgreSQL 只需改为 postgresql+asyncpg://...
    database_url: str = f"sqlite+aiosqlite:///{_JIANG_DIR / 'data' / 'xczs.db'}"

    # ── 认证 ────────────────────────────────────────────────────────
    # 未配置时为当前进程生成强随机密钥，避免使用可预测的开发
    # 默认值。重启后旧 token 会失效；需要持久会话时应显式配置
    # XCZS_SECRET_KEY。
    secret_key: str = Field(
        default_factory=_generate_ephemeral_secret_key,
        min_length=32,
    )
    jwt_algorithm: Literal["HS256"] = "HS256"
    access_token_expire_minutes: int = 480  # 8 小时
    # 初始 admin 账号密码；为空时启动随机生成并打印。
    admin_password: str = ""
    # Swagger UI 访问 token；为空则不保护 /docs。
    swagger_token: str = ""
    # 是否启用 JWT 门禁。迁移过渡期可用 XCZS_AUTH_ENABLED=false 关闭。
    auth_enabled: bool = True

    # ── 端口（三合一后主网关端口） ──────────────────────────────────
    host: str = "127.0.0.1"
    port: int = 8090

    # ── 允许的浏览器 Origin（逗号分隔，兼容 XCZS_CONTROL_ORIGINS） ──
    # pydantic-settings 读取 XCZS_ALLOWED_ORIGINS；旧变量由 launcher 换算。
    allowed_origins: str = "http://localhost:8090,http://127.0.0.1:8090"

    # ── 数据目录 ────────────────────────────────────────────────────
    recordings_root: Optional[str] = str(_WORKSPACE / "recordings")

    # ── 任务记录保留 ────────────────────────────────────────────────
    # task_records 保留上限（条数）；0 = 不限。任务终态写入时若超过上限，
    # 删除最旧的超限记录及其进度明细（task_progress_events 级联清理）。
    task_record_retention: int = 10000

    @property
    def allowed_origin_list(self) -> list[str]:
        """解析逗号分隔的 Origin 列表。"""
        origins = [o.strip() for o in self.allowed_origins.split(",") if o.strip()]
        return origins or ["http://localhost:8090"]

    @property
    def is_production(self) -> bool:
        """启发式判断：生产模式关闭可写文档等暴露面。"""
        return len(self.secret_key) >= 32


settings = Settings()
