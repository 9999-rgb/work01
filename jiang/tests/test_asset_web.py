"""Web 资产管理已移除，场景和末端运行接口仍保留。"""
from pathlib import Path
import sys

from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.config import settings
from app.main import create_app


def test_asset_routes_removed_but_scene_and_toolset_routes_remain(monkeypatch):
    monkeypatch.setattr(settings, "auth_enabled", False)
    app = create_app(enable_db=False)
    paths = app.openapi()["paths"]
    assert not any(path.startswith("/assets") for path in paths)
    assert "/scene/switch" in paths
    assert "/robot/toolset/switch" in paths
    with TestClient(app) as client:
        for method, path in [
            ("GET", "/assets"),
            ("POST", "/assets/import"),
            ("GET", "/assets/selection"),
            ("POST", "/assets/selection"),
            ("DELETE", "/assets/scene/generator_plant"),
        ]:
            assert client.request(method, path).status_code == 404
