"""资产导入 / 选择 API（阶段2）。

上传 zip、校验 manifest、导入资产库、列出目录、选择当前运行组合。
选择结果只持久化到 ``selection.yaml``；启动时 ``start_xczs_bridge.sh`` 读入并
映射到现有环境指针 —— 本路由不做任何运行时切换（启动时导入，重启生效）。

校验器与 CLI（``scripts/xczs_import_asset``）共用
:mod:`control_gateway.asset_validators`，保证 Web 与脚本两条入口行为一致。
"""

from __future__ import annotations

import io
import os
import shutil
import tempfile
import zipfile
from pathlib import Path
from typing import Annotated, Any

from fastapi import APIRouter, Depends, File, Form, HTTPException, UploadFile, status

from app.api.schemas import AssetSelectionRequest
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

#: 机器人内置的固定夹爪变体集。阶段4 将由机器人模型导出真实变体集；
#: 在此之前只有一个变体「当前夹爪」。
BUILTIN_GRIPPER_VARIANTS = ("default",)

#: 可导入的资产类型（夹爪不导入，是机器人内的选择项）。
_ASSET_KINDS = ("scene", "cabinet")


def _library() -> AssetLibrary:
    """构建资产库实例；根目录可用 ``XCZS_ASSETS_DIR`` 覆盖（与启动脚本一致）。"""
    root = os.environ.get("XCZS_ASSETS_DIR") or default_library_root()
    return AssetLibrary(root)


def _selection_from_request(
    body: AssetSelectionRequest, library: AssetLibrary
) -> AssetSelection:
    """把请求体校验为可持久化的选择；名称不合法时抛 ``AssetLibraryError``。"""
    if body.scene is not None:
        library.find("scene", body.scene)  # raises AssetNotFoundError
    if body.cabinet is not None:
        library.find("cabinet", body.cabinet)
    if body.gripper_variant is not None and body.gripper_variant not in BUILTIN_GRIPPER_VARIANTS:
        raise AssetLibraryError(f"Unknown gripper_variant: {body.gripper_variant}")
    return AssetSelection(
        scene=body.scene,
        cabinet=body.cabinet,
        gripper_variant=body.gripper_variant,
    )


@router.get(
    "/assets",
    summary="资产目录与当前选择",
    description="返回已导入资产、当前选择、可用夹爪变体与资产库根目录。",
)
async def list_assets() -> dict[str, Any]:
    library = _library()
    return {
        "assets": [record.to_dict() for record in library.load_catalog()],
        "selection": library.load_selection().to_dict(),
        "gripper_variants": list(BUILTIN_GRIPPER_VARIANTS),
        "library_root": str(library.root),
    }


@router.get(
    "/assets/selection",
    summary="当前资产选择",
    description="返回当前选择的场景 / 柜体 / 夹爪变体（可为 null）。",
)
async def get_selection() -> dict[str, Any]:
    return _library().load_selection().to_dict()


@router.post(
    "/assets/selection",
    summary="保存资产选择",
    description="持久化选择组合；重启后由启动脚本消费。scene / cabinet 必须已在资产库中。",
)
async def save_selection(body: AssetSelectionRequest) -> dict[str, Any]:
    library = _library()
    try:
        selection = _selection_from_request(body, library)
    except AssetNotFoundError as error:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND, detail=str(error)
        ) from None
    except AssetLibraryError as error:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, detail=str(error)
        ) from None
    library.save_selection(selection)
    return library.load_selection().to_dict()


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
    library = _library()
    try:
        data = await file.read()
    finally:
        await file.close()

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
    library = _library()
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
