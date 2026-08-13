"""认证业务逻辑：密码哈希、JWT 签发与校验。"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Any

from jose import JWTError, jwt
from passlib.context import CryptContext

from app.config import settings

# bcrypt 哈希上下文。passlib 1.7.4 与 bcrypt>=4.1 存在 __about__ 告警，
# 用 ``bcrypt_sha256`` 作为兼容层消除运行时风险。
_pwd_context = CryptContext(schemes=["bcrypt_sha256"], deprecated="auto")


def hash_password(password: str) -> str:
    """生成密码哈希（bcrypt-sha256）。"""
    return _pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """校验明文密码与哈希是否匹配。"""
    return _pwd_context.verify(plain_password, hashed_password)


def create_access_token(
    user_id: int,
    username: str,
    role: str,
    *,
    expires_delta: timedelta | None = None,
) -> tuple[str, int]:
    """签发 JWT token，返回 ``(token, 有效期秒数)``。

    payload：``sub`` = user_id，``username``，``role``，``exp``。
    """
    expire_minutes = settings.access_token_expire_minutes
    # An explicit zero/negative duration is meaningful to callers testing or
    # enforcing immediate expiry; only ``None`` selects the configured TTL.
    delta = (
        timedelta(minutes=expire_minutes)
        if expires_delta is None
        else expires_delta
    )
    now = datetime.now(timezone.utc)
    # python-jose serializes NumericDate values with whole-second precision.
    # ``exp == iat`` can therefore remain valid for the rest of that second,
    # so place non-positive TTLs unambiguously in the past.
    expire_at = (
        now + delta
        if delta > timedelta(0)
        else now - timedelta(seconds=1)
    )
    payload: dict[str, Any] = {
        "sub": str(user_id),
        "username": username,
        "role": role,
        "exp": expire_at,
        "iat": now,
    }
    token = jwt.encode(payload, settings.secret_key, algorithm=settings.jwt_algorithm)
    return token, int(delta.total_seconds())


class InvalidTokenError(ValueError):
    """token 缺失、过期或签名不合法。"""


def decode_access_token(token: str) -> dict[str, Any]:
    """校验并解析 JWT，返回 payload；非法则抛出 InvalidTokenError。"""
    try:
        payload = jwt.decode(
            token,
            settings.secret_key,
            algorithms=[settings.jwt_algorithm],
            options={
                "require_exp": True,
                "require_iat": True,
                "require_sub": True,
            },
        )
    except JWTError as error:
        raise InvalidTokenError(str(error)) from error
    return payload
