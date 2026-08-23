"""资产导入 / 选择 API（阶段2）。

上传 zip、校验 manifest、导入资产库、列出目录、选择当前运行组合。
目录与选择持久化到 SQLite（``assets`` / ``selection`` 表，经
:class:`app.assets.store.SqlAssetStore`）；启动时 ``start_xczs_bridge.sh`` 读入
并映射到现有环境指针 —— 本路由不改变已在 Gazebo 中加载的机器人模型。

末端工具套装是一个例外：当前机器人 URDF、Gazebo ``ros2_control`` 和
MoveIt 配置会在启动时共同选择 A 或 B，运行中把 Web 的选择直接应用到
控制层会让规划关节与物理模型不一致。因此工具套装选择只保存为下一次
人工启动时的待生效配置；绝不通过写标记触发整套仿真自动重启。

校验器与 CLI（``scripts/xczs_import_asset``）共用
:mod:`control_gateway.asset_validators`，保证 Web 与脚本两条入口行为一致。
"""

from __future__ import annotations

import asyncio
import io
import os
import shutil
import tempfile
import zipfile
from pathlib import Path
from typing import Annotated, Any

from fastapi import APIRouter, Depends, File, Form, HTTPException, UploadFile, status

from app.api.schemas import AssetSelectionRequest
from app.assets.store import SqlAssetStore
from app.auth.deps import require_admin
from app.auth.models import User

from control_gateway.asset_library import (
    AssetExistsError,
    AssetLibrary,
    AssetLibraryError,
    AssetNotFoundError,
    AssetSelection,
    default_library_root,
)
from control_gateway.asset_manifest import ManifestError, load_manifest
from control_gateway.asset_validators import kind_validator

router = APIRouter(tags=["资产"])

#: 可导入的资产类型。
_ASSET_KINDS = ("scene", "cabinet")

_ACTIVE_TOOLSET_ENV = "XCZS_ACTIVE_TOOLSET"


def _library() -> AssetLibrary:
    """构建资产库实例；根目录可用 ``XCZS_ASSETS_DIR`` 覆盖（与启动脚本一致）。

    目录 / 选择持久化到 SQLite（``SqlAssetStore``），与 CLI 和启动脚本共用。
    """
    root = os.environ.get("XCZS_ASSETS_DIR") or default_library_root()
    return AssetLibrary(root, store=SqlAssetStore())


def _normalized_toolset(value: object) -> str | None:
    """Return a valid A/B toolset name, or ``None`` for unavailable input."""
    if not isinstance(value, str):
        return None
    normalized = value.strip().upper()
    return normalized if normalized in {"A", "B"} else None


def _toolset_status(selection: AssetSelection) -> dict[str, object]:
    """Describe the live-vs-persisted toolset without changing runtime state.

    ``XCZS_ACTIVE_TOOLSET`` is exported only by the unified launcher after it
    has normalized the toolset used to build the current URDF.  A standalone
    Web process deliberately reports an unknown active set instead of claiming
    that a merely persisted selection is mounted.
    """
    active = _normalized_toolset(os.environ.get(_ACTIVE_TOOLSET_ENV))
    selected = _normalized_toolset(selection.toolset)
    pending = (
        selected
        if selected is not None and (active is None or selected != active)
        else None
    )
    return {
        "active_toolset": active,
        "pending_toolset": pending,
        # ``None`` means that this Web process is not attached to a unified
        # launcher, so it cannot verify which physical tool is mounted.
        "applied": None if active is None else pending is None,
        "manual_restart_required": pending is not None,
        "automatic_restart": False,
    }


def _selection_from_request(
    body: AssetSelectionRequest,
    library: AssetLibrary,
    previous: AssetSelection,
) -> AssetSelection:
    """Merge a partial selection request and validate its effective assets.

    ``AssetSelectionRequest`` deliberately uses ``null``/empty fields to mean
    "leave this dimension unchanged".  Merging before validation prevents a
    toolset-only Web save from accidentally clearing a persisted scene or
    cabinet selection.
    """
    scene = body.scene if body.scene is not None else previous.scene
    cabinet = body.cabinet if body.cabinet is not None else previous.cabinet
    toolset = body.toolset if body.toolset is not None else previous.toolset
    if scene is not None:
        library.find("scene", scene)  # raises AssetNotFoundError
    if cabinet is not None:
        library.find("cabinet", cabinet)
    return AssetSelection(
        scene=scene,
        cabinet=cabinet,
        toolset=toolset,
    )


@router.get(
    "/assets",
    summary="资产目录与当前选择",
    description="返回已导入资产、当前选择与资产库根目录。",
)
async def list_assets() -> dict[str, Any]:
    # Store 构造会执行首次 Alembic 迁移；与后续同步 SQLAlchemy 操作一样
    # 放在线程中，避免阻塞事件循环或在运行中的 loop 内调用迁移 runner。
    library = await asyncio.to_thread(_library)
    assets = await asyncio.to_thread(library.load_catalog)
    selection = await asyncio.to_thread(library.load_selection)
    return {
        "assets": [record.to_dict() for record in assets],
        "selection": selection.to_dict(),
        "toolset_status": _toolset_status(selection),
        "library_root": str(library.root),
    }


@router.get(
    "/assets/selection",
    summary="当前资产选择",
    description="返回当前选择的场景 / 柜体（可为 null）。",
)
async def get_selection() -> dict[str, Any]:
    library = await asyncio.to_thread(_library)
    selection = await asyncio.to_thread(library.load_selection)
    return selection.to_dict()


@router.post(
    "/assets/selection",
    summary="保存资产选择",
    description=(
        "持久化选择组合；下次人工启动时由启动脚本消费。scene / cabinet 必须已在"
        "资产库中。工具套装不会让当前 Gazebo 仿真自动重启；响应会返回当前、"
        "待生效与已应用状态。"
    ),
)
async def save_selection(body: AssetSelectionRequest) -> dict[str, Any]:
    library = await asyncio.to_thread(_library)

    def _run() -> dict[str, Any]:
        previous = library.load_selection()
        try:
            selection = _selection_from_request(body, library, previous)
        except AssetNotFoundError as error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail=str(error)
            ) from None
        except AssetLibraryError as error:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST, detail=str(error)
            ) from None
        library.save_selection(selection)
        result = library.load_selection().to_dict()
        # 保留 ``restart_required`` 兼容旧前端，但它现在恒为 false：当前
        # 仿真绝不能因为持久化一次选择而被自动终止。
        result["restart_required"] = False
        result["toolset_status"] = _toolset_status(
            AssetSelection(
                scene=result["scene"],
                cabinet=result["cabinet"],
                toolset=result["toolset"],
            )
        )
        return result

    return await asyncio.to_thread(_run)


@router.post(
    "/assets/import",
    status_code=status.HTTP_201_CREATED,
    summary="导入资产（zip）",
    description=(
        "上传包含 manifest.yaml 的 zip，经校验后复制进资产库。"
        "重复导入返回 409；--force 表单可覆盖同类型同名资产。"
    ),
)
async def import_asset(
    _admin: Annotated[User, Depends(require_admin)],
    file: Annotated[UploadFile, File(description="资产 zip")],
    force: Annotated[bool, Form(description="已存在时覆盖")] = False,
    skip_validate: Annotated[
        bool, Form(description="跳过语义校验（仅查 manifest 结构与文件存在）")
    ] = False,
) -> dict[str, Any]:
    library = await asyncio.to_thread(_library)
    try:
        data = await file.read()
    finally:
        await file.close()

    def _run() -> dict[str, Any]:
        with tempfile.TemporaryDirectory(prefix="xczs_asset_") as temp_dir:
            extract_root = _extract_zip_upload(data, Path(temp_dir))
            try:
                manifest = load_manifest(extract_root / "manifest.yaml")
            except ManifestError as error:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST, detail=str(error)
                ) from None
            validate = None if skip_validate else kind_validator(manifest.kind)
            try:
                record = library.import_asset(
                    extract_root, validate=validate, force=force
                )
            except AssetExistsError as error:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT, detail=str(error)
                ) from None
            except AssetLibraryError as error:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST, detail=str(error)
                ) from None
        return {"asset": record.to_dict()}

    return await asyncio.to_thread(_run)


@router.delete(
    "/assets/{kind}/{name}",
    summary="删除资产",
    description="删除资产目录与 catalog 条目；若该资产正被选择，对应选择字段一并清空。",
)
async def delete_asset(
    _admin: Annotated[User, Depends(require_admin)],
    kind: str,
    name: str,
) -> dict[str, Any]:
    if kind not in _ASSET_KINDS:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"不支持的资产类型：{kind}（仅 scene / cabinet）。",
        )
    library = await asyncio.to_thread(_library)

    def _run() -> dict[str, Any]:
        try:
            record = library.remove_asset(kind, name)
        except AssetNotFoundError as error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail=str(error)
            ) from None
        return {
            "removed": record.to_dict(),
            "selection": library.load_selection().to_dict(),
        }

    return await asyncio.to_thread(_run)


# ── zip 解压（zip-slip 安全） ───────────────────────────────────────────────


def _extract_zip_upload(data: bytes, destination: Path) -> Path:
    """安全解压上传的 zip，返回含 ``manifest.yaml`` 的资产根目录。

    - 拒绝空文件、非法 zip、绝对路径与 ``..`` 越界成员（zip-slip）；
    - 资产根目录 = 解压根下或唯一子目录下的 ``manifest.yaml``。
    """
    if not data:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail="上传文件为空。"
        )
    try:
        archive = zipfile.ZipFile(io.BytesIO(data))
    except zipfile.BadZipFile as error:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"不是有效的 zip 文件：{error}",
        ) from None
    members = archive.infolist()
    if not members:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail="zip 内没有任何文件。"
        )
    destination.mkdir(parents=True, exist_ok=True)
    for member in members:
        _extract_zip_member(archive, member, destination)
    return _find_asset_root(destination)


def _extract_zip_member(
    archive: zipfile.ZipFile,
    member: zipfile.ZipInfo,
    destination: Path,
) -> None:
    normalized = member.filename.replace("\\", "/")
    path = Path(normalized)
    if path.is_absolute() or ".." in path.parts:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"zip 包含非法路径：{member.filename!r}",
        )
    target = destination / path
    try:
        target.resolve().relative_to(destination.resolve())
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"zip 路径越界：{member.filename!r}",
        ) from None
    if member.is_dir():
        target.mkdir(parents=True, exist_ok=True)
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    with archive.open(member) as source, target.open("wb") as sink:
        shutil.copyfileobj(source, sink)


def _find_asset_root(extract_root: Path) -> Path:
    if (extract_root / "manifest.yaml").is_file():
        return extract_root
    candidates = [
        child
        for child in extract_root.iterdir()
        if child.is_dir() and (child / "manifest.yaml").is_file()
    ]
    if len(candidates) == 1:
        return candidates[0]
    raise HTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail="zip 根目录（或唯一子目录）下需有 manifest.yaml。",
    )
