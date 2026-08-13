"""管理员账号引导与恢复。

系统没有启用状态 admin 时创建新的恢复账号：
- 密码来自 ``XCZS_ADMIN_PASSWORD``；为空则随机生成并打印到日志。
- 不覆盖、激活或提权任何已有账号。
"""

from __future__ import annotations

import logging
import secrets

from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from app.auth.models import ROLE_ADMIN, User
from app.auth.service import hash_password
from app.config import settings

logger = logging.getLogger(__name__)

DEFAULT_ADMIN_USERNAME = "admin"
RECOVERY_ADMIN_USERNAME_PREFIX = "recovery_admin"


async def bootstrap_admin(session: AsyncSession) -> None:
    """确保存在至少一个启用状态的 admin 账号。"""
    count_result = await session.execute(
        select(func.count())
        .select_from(User)
        .where(User.role == ROLE_ADMIN, User.is_active.is_(True))
    )
    if count_result.scalar_one() > 0:
        return

    username_result = await session.execute(select(User.username))
    existing_usernames = set(username_result.scalars())
    if not existing_usernames:
        username = DEFAULT_ADMIN_USERNAME
    else:
        suffix = 1
        username = RECOVERY_ADMIN_USERNAME_PREFIX
        while username in existing_usernames:
            suffix += 1
            username = f"{RECOVERY_ADMIN_USERNAME_PREFIX}_{suffix}"

    password = settings.admin_password
    if not password:
        # 单机场景：随机生成，仅打印到控制台，避免硬编码弱口令。
        password = secrets.token_urlsafe(16)
        logger.warning(
            "未配置 XCZS_ADMIN_PASSWORD，已随机生成 %s 密码：%s",
            username,
            password,
        )
    admin = User(
        username=username,
        hashed_password=hash_password(password),
        role=ROLE_ADMIN,
        is_active=True,
    )
    session.add(admin)
    await session.commit()
    logger.warning("系统无启用状态 admin，已创建恢复账号：%s", username)
