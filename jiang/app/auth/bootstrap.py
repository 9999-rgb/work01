"""初始管理员账号引导。

首次启动（users 表为空）时创建默认 admin：
- 密码来自 ``XCZS_ADMIN_PASSWORD``；为空则随机生成并打印到日志。
- 幂等：已存在用户时不做任何事。
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


async def bootstrap_admin(session: AsyncSession) -> None:
    """确保存在至少一个启用状态的 admin 账号。"""
    count_result = await session.execute(
        select(func.count()).select_from(User)
    )
    if count_result.scalar_one() > 0:
        return

    password = settings.admin_password
    if not password:
        # 单机场景：随机生成，仅打印到控制台，避免硬编码弱口令。
        password = secrets.token_urlsafe(16)
        logger.warning(
            "未配置 XCZS_ADMIN_PASSWORD，已随机生成初始 admin 密码：%s",
            password,
        )
    admin = User(
        username=DEFAULT_ADMIN_USERNAME,
        hashed_password=hash_password(password),
        role=ROLE_ADMIN,
        is_active=True,
    )
    session.add(admin)
    await session.commit()
    logger.info("已创建初始 admin 账号：%s", DEFAULT_ADMIN_USERNAME)
