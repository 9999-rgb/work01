"""认证与用户管理路由。

公开：
- ``POST /auth/login`` 登录获取 token
- ``GET /auth/me`` 当前用户信息

admin：
- ``POST /users`` 创建用户
- ``GET /users`` 列出用户
- ``GET /users/{id}`` 查看用户
- ``PATCH /users/{id}`` 更新用户（禁用/改角色/改密码）
- ``DELETE /users/{id}`` 删除用户
"""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import delete, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from app.auth.deps import get_current_user, require_admin
from app.auth.models import ROLE_ADMIN, User
from app.auth.schemas import (
    LoginRequest,
    TokenResponse,
    UserCreate,
    UserResponse,
    UserUpdate,
)
from app.auth.service import create_access_token, hash_password, verify_password
from app.database.engine import get_db

router = APIRouter(tags=["认证"])


@router.post(
    "/auth/login",
    response_model=TokenResponse,
    summary="用户登录",
    description="""校验用户名密码，返回 JWT Bearer Token。

调用方式：``Authorization: Bearer <access_token>``。
token 默认 8 小时过期；过期后重新登录。
""",
)
async def login(
    body: LoginRequest,
    session: Annotated[AsyncSession, Depends(get_db)],
) -> TokenResponse:
    result = await session.execute(
        select(User).where(User.username == body.username)
    )
    user = result.scalar_one_or_none()
    if user is None or not verify_password(body.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="用户名或密码错误。",
        )
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="该账号已被禁用。",
        )
    token, expires_in = create_access_token(user.id, user.username, user.role)
    return TokenResponse(
        access_token=token,
        expires_in=expires_in,
        user=UserResponse.model_validate(user),
    )


@router.get(
    "/auth/me",
    response_model=UserResponse,
    summary="当前用户信息",
)
async def me(
    current_user: Annotated[User, Depends(get_current_user)],
) -> UserResponse:
    return UserResponse.model_validate(current_user)


@router.post(
    "/users",
    response_model=UserResponse,
    status_code=status.HTTP_201_CREATED,
    summary="创建用户",
    description="仅 admin 可调用。",
)
async def create_user(
    body: UserCreate,
    session: Annotated[AsyncSession, Depends(get_db)],
    _admin: Annotated[User, Depends(require_admin)],
) -> UserResponse:
    exists = await session.execute(
        select(User).where(User.username == body.username)
    )
    if exists.scalar_one_or_none() is not None:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"用户名 {body.username!r} 已存在。",
        )
    user = User(
        username=body.username,
        hashed_password=hash_password(body.password),
        role=body.role,
    )
    session.add(user)
    await session.commit()
    await session.refresh(user)
    return UserResponse.model_validate(user)


@router.get(
    "/users",
    response_model=list[UserResponse],
    summary="列出用户",
    description="仅 admin 可调用。",
)
async def list_users(
    session: Annotated[AsyncSession, Depends(get_db)],
    _admin: Annotated[User, Depends(require_admin)],
    limit: Annotated[int, Query(ge=1, le=100)] = 100,
) -> list[UserResponse]:
    result = await session.execute(select(User).order_by(User.id).limit(limit))
    return [UserResponse.model_validate(u) for u in result.scalars()]


@router.get(
    "/users/{user_id}",
    response_model=UserResponse,
    summary="查看用户",
    description="仅 admin 可调用。",
)
async def get_user(
    user_id: int,
    session: Annotated[AsyncSession, Depends(get_db)],
    _admin: Annotated[User, Depends(require_admin)],
) -> UserResponse:
    user = await session.get(User, user_id)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"用户 {user_id} 不存在。",
        )
    return UserResponse.model_validate(user)


@router.patch(
    "/users/{user_id}",
    response_model=UserResponse,
    summary="更新用户",
    description="""仅 admin 可调用。可改密码、角色、启用状态。

保护：禁止禁用/删除最后一个 admin，避免锁定系统。
""",
)
async def update_user(
    user_id: int,
    body: UserUpdate,
    session: Annotated[AsyncSession, Depends(get_db)],
    _admin: Annotated[User, Depends(require_admin)],
) -> UserResponse:
    user = await session.get(User, user_id)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"用户 {user_id} 不存在。",
        )
    # 防止把最后一个 active admin 禁用或降级，导致系统无人可管理。
    # 必须在赋值前保留原角色，否则 admin -> operator 会绕过检查。
    was_active_admin = user.role == ROLE_ADMIN and user.is_active
    next_role = body.role if body.role is not None else user.role
    next_is_active = (
        body.is_active if body.is_active is not None else user.is_active
    )
    will_lose_admin = was_active_admin and not (
        next_role == ROLE_ADMIN and next_is_active
    )
    if will_lose_admin and await _last_admin_would_be_locked_out(
        session, user_id
    ):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="不能禁用或降级最后一个 admin 账号。",
        )
    if body.password is not None:
        user.hashed_password = hash_password(body.password)
    user.role = next_role
    user.is_active = next_is_active
    await session.commit()
    await session.refresh(user)
    return UserResponse.model_validate(user)


@router.delete(
    "/users/{user_id}",
    status_code=status.HTTP_204_NO_CONTENT,
    summary="删除用户",
    description="仅 admin 可调用。禁止删除最后一个 admin。",
)
async def delete_user(
    user_id: int,
    session: Annotated[AsyncSession, Depends(get_db)],
    _admin: Annotated[User, Depends(require_admin)],
) -> None:
    user = await session.get(User, user_id)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"用户 {user_id} 不存在。",
        )
    if user.role == "admin" and await _last_admin_would_be_locked_out(
        session, user_id
    ):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="不能删除最后一个 admin 账号。",
        )
    await session.execute(delete(User).where(User.id == user_id))
    await session.commit()


async def _last_admin_would_be_locked_out(
    session: AsyncSession,
    exclude_user_id: int,
) -> bool:
    """除 exclude_user_id 之外是否还存在其他启用状态的 admin。"""
    count_result = await session.execute(
        select(func.count())
        .select_from(User)
        .where(
            User.id != exclude_user_id,
            User.role == ROLE_ADMIN,
            User.is_active.is_(True),
        )
    )
    return count_result.scalar_one() == 0
