"""把已导入的场景资产解析为经过校验的 SceneSpec（纯逻辑，无 ROS 依赖）。

场景资产的 ``scenes.yaml`` 在导入时已由 :func:`~.asset_library._normalize_scene_refs`
把相对 ``nav2_map`` / ``model.file`` 归一化为资产库内绝对路径（``package://`` /
``model://`` / ``file://`` 等 URI 原样保留），因此运行时只需加载并复用
:class:`~.scene_catalog.SceneCatalog` 的严格解析器，无需再次归一化。

解析是惰性且无状态的：每次调用都重读资产目录，Web 端导入 / 删除资产后，
下一次 ``/scenes`` 或场景切换即可反映，无需重启网关。``/scenes`` 不在监控页
轮询循环中（只在启动与切换后拉取），逐请求解析少量小 YAML 的开销可接受。
"""

from __future__ import annotations

from typing import Iterator, Optional

from .asset_library import (
    AssetLibrary,
    AssetLibraryError,
    AssetNotFoundError,
)
from .asset_manifest import ManifestError
from .scene_catalog import SceneCatalog, SceneError, SceneSpec

_SCENES_ROLE = "scenes"


class AssetSceneProvider:
    """把资产库 scene 资产映射为可切换的 SceneSpec。

    ``library`` 为 ``None``（或资产库为空）时不提供任何场景。单个资产解析
    失败（manifest 缺失、``scenes.yaml`` 畸形、role 缺失、目录被外部改动等）
    会跳过该资产而不中断其余资产，避免一个坏资产拖垮 ``/scenes`` 或场景切换。
    """

    def __init__(self, library: Optional[AssetLibrary] = None) -> None:
        self._library = library

    def scene_asset_names(self) -> list[str]:
        """返回资产库中全部 scene 资产的名称（按导入顺序）。"""
        if self._library is None:
            return []
        return [
            record.name
            for record in self._library.load_catalog()
            if record.kind == "scene"
        ]

    def resolve_scene(self, name: str) -> Optional[SceneSpec]:
        """解析名为 ``name`` 的资产场景为 SceneSpec。

        返回 ``None`` 表示资产不存在或解析失败；调用方应据此回退到内置场景
        注册表或给出 404。
        """
        if self._library is None:
            return None
        try:
            record = self._library.find("scene", name)
            manifest = self._library.asset_manifest(record)
            root = self._library.asset_root(record)
            catalog = SceneCatalog.load(manifest.file_path(root, _SCENES_ROLE))
            return catalog.get(name)
        except (
            AssetNotFoundError,
            AssetLibraryError,
            ManifestError,
            SceneError,
            OSError,
        ):
            return None

    def iter_scene_specs(self) -> Iterator[SceneSpec]:
        """依次产出每个资产场景的 SceneSpec；解析失败的资产被跳过。"""
        for name in self.scene_asset_names():
            spec = self.resolve_scene(name)
            if spec is not None:
                yield spec
