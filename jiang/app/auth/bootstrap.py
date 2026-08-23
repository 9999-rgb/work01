"""管理员账号引导与恢复。

系统没有启用状态 admin 时创建新的恢复账号：
- 密码来自 ``XCZS_ADMIN_PASSWORD``；为空则随机生成并打印到日志。
- 不覆盖、激活或提权任何已有账号。
"""

from __future__ import annotations

import logging
import secrets

from sqlalchemy import func, select, text
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from app.auth.models import ROLE_ADMIN, User
from app.auth.service import hash_password
from app.config import settings

logger = logging.getLogger(__name__)

DEFAULT_ADMIN_USERNAME = "admin"
RECOVERY_ADMIN_USERNAME_PREFIX = "recovery_admin"


async def bootstrap_admin(session: AsyncSession) -> None:
    """确保存在至少一个启用状态的 admin 账号。"""
    # SQLite 没有 SELECT FOR UPDATE；启动时先取得写事务，避免同一数据库
    # 的两个 worker 同时判断“没有 admin”并创建同名账号。
    bind = session.get_bind()
    if bind.dialect.name == "sqlite":
        if session.in_transaction():
            await session.rollback()
        await session.execute(text("BEGIN IMMEDIATE"))

    count_result = await session.execute(
        select(func.count())
        .select_from(User)
        .where(User.role == ROLE_ADMIN, User.is_active.is_(True))
    )
    if count_result.scalar_one() > 0:
        await session.rollback()
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

    password = settings.admin_password or secrets.token_urlsafe(16)
    admin = User(
        username=username,
        hashed_password=hash_password(password),
        role=ROLE_ADMIN,
        is_active=True,
    )
    session.add(admin)
    try:
        await session.commit()
    except IntegrityError:
        # 唯一用户名约束是最后一道并发仲裁；输家回滚后只在确有 active
        # admin 时安全返回，避免掩盖其他完整性错误。
        await session.rollback()
        winner = await session.execute(
            select(func.count())
            .select_from(User)
            .where(User.role == ROLE_ADMIN, User.is_active.is_(True))
        )
        if winner.scalar_one() > 0:
            await session.rollback()
            return
        raise
    if not settings.admin_password:
        # 仅公布真正提交成功的随机凭证；并发引导的输家不会打印假密码。
        logger.warning(
            "未配置 XCZS_ADMIN_PASSWORD，已随机生成 %s 密码：%s",
            username,
            password,
        )
    logger.warning("系统无启用状态 admin，已创建恢复账号：%s", username)
