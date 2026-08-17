"""FastAPI 资产路由契约测试（阶段2）。

驱动真实 ``app.main.create_app`` + 真实资产库实现（``control_gateway.asset_library``），
用测试临时目录作资产库根（``XCZS_ASSETS_DIR``）。分三组：

- ``AssetWebReadContractTest``：auth 关闭，覆盖列表 / 选择（无 admin 依赖）；
- ``AssetWebImportContractTest``：auth 开启 + admin 登录，覆盖 zip 导入 / 校验 /
  重复 / 越权路径（导入与删除挂 ``require_admin``，与 ``/users`` 同姿态，始终强制鉴权）；
- ``AssetWebAuthGateTest``：auth 门禁 —— 匿名 401、operator 403、admin 放行。

不依赖 ROS：control_gateway 包注入假模块，纯子模块按真实路径加载。
"""

from __future__ import annotations

import io
import os
import sys
import tempfile
import types
import unittest
import zipfile
from pathlib import Path
from typing import Dict

import yaml

JIANG_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(JIANG_DIR))

# 兼容无 ROS 环境的纯模块注入（app 模块按真实路径加载 control_gateway 子模块）。
CONTROL_GATEWAY_PACKAGE = types.ModuleType("control_gateway")
CONTROL_GATEWAY_PACKAGE.__path__ = [str(JIANG_DIR / "control_gateway")]
sys.modules.setdefault("control_gateway", CONTROL_GATEWAY_PACKAGE)

_ADMIN_PASS = "pytest-admin-pass"

MAP_FIELDS = {
    "image": "inspection_map.pgm",
    "resolution": 0.05,
    "origin": [-5.0, -5.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.25,
}


def _scene_document(name: str) -> Dict[str, object]:
    return {
        "scenes": [
            {
                "name": name,
                "spawn_cabinet": True,
                "model": None,
                "nav2_map": "maps/inspection_map.yaml",
                "robot_spawn": {"x": 0.0, "y": 0.0, "z": 0.515, "yaw": 1.57079632679},
            }
        ]
    }


def _scene_manifest(name: str, version: str = "1.0.0") -> Dict[str, object]:
    return {
        "kind": "scene",
        "name": name,
        "version": version,
        "description": "Web import fixture.",
        "files": {
            "scenes": "scenes.yaml",
            "instances": "cabinet_instances.yaml",
        },
    }


def _scene_zip_bytes(name: str = "web_scene", version: str = "1.0.0") -> bytes:
    """Build a valid scene-asset zip with a nested asset directory."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        archive.writestr(
            f"{name}/manifest.yaml",
            yaml.safe_dump(_scene_manifest(name, version), sort_keys=False),
        )
        archive.writestr(
            f"{name}/scenes.yaml",
            yaml.safe_dump(_scene_document(name), sort_keys=False),
        )
        archive.writestr(f"{name}/cabinet_instances.yaml", "instances: []\n")
        archive.writestr(
            f"{name}/maps/inspection_map.yaml",
            yaml.safe_dump(MAP_FIELDS, sort_keys=False),
        )
        archive.writestr(f"{name}/maps/inspection_map.pgm", b"\x00")
    return buffer.getvalue()


def _zipped_bytes(members: Dict[str, bytes]) -> bytes:
    """Pack arbitrary ``path -> bytes`` members into a zip (for negative cases)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for path, content in members.items():
            archive.writestr(path, content)
    return buffer.getvalue()


class _AssetWebTestCase(unittest.TestCase):
    """共享 setUp/tearDown：临时资产库目录 + XCZS_ASSETS_DIR 覆盖。

    子类若对 TestClient 调用 ``__enter__``（触发 lifespan），须把
    ``self._client_entered`` 置 True，base tearDown 会对称调用 ``__exit__``。
    """

    _client_entered: bool = False

    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.scratch = Path(self._temporary_directory.name)
        self.assets_dir = self.scratch / "assets"
        self._previous_env = os.environ.get("XCZS_ASSETS_DIR")
        os.environ["XCZS_ASSETS_DIR"] = str(self.assets_dir)

    def tearDown(self) -> None:
        if self._client_entered:
            self._client_cm.__exit__(None, None, None)
        if self._previous_env is None:
            os.environ.pop("XCZS_ASSETS_DIR", None)
        else:
            os.environ["XCZS_ASSETS_DIR"] = self._previous_env
        self._temporary_directory.cleanup()


class AssetWebReadContractTest(_AssetWebTestCase):
    """列表 / 选择契约（这些端点无 admin 依赖，auth 关闭即可驱动）。"""

    def setUp(self) -> None:
        super().setUp()
        from fastapi.testclient import TestClient

        from app.main import create_app

        self._client_cm = TestClient(
            create_app(
                control_server=_FakeControlServer(),
                enable_db=False,
                auth_enabled=False,
            )
        )
        self.client = self._client_cm

    def test_list_assets_empty_library(self) -> None:
        response = self.client.get("/assets")
        self.assertEqual(200, response.status_code)
        data = response.json()
        self.assertEqual([], data["assets"])
        self.assertEqual(
            {"scene": None, "cabinet": None, "gripper_variant": None},
            data["selection"],
        )
        self.assertEqual(["default"], data["gripper_variants"])
        self.assertEqual(str(self.assets_dir), data["library_root"])

    def test_get_selection_initial(self) -> None:
        response = self.client.get("/assets/selection")
        self.assertEqual(200, response.status_code)
        self.assertEqual(
            {"scene": None, "cabinet": None, "gripper_variant": None},
            response.json(),
        )

    def test_selection_rejects_unknown_asset_and_variant(self) -> None:
        unknown = self.client.post(
            "/assets/selection", json={"scene": "missing_scene"}
        )
        self.assertEqual(404, unknown.status_code, unknown.text)

        bad_gripper = self.client.post(
            "/assets/selection", json={"gripper_variant": "claw_9000"}
        )
        self.assertEqual(400, bad_gripper.status_code, bad_gripper.text)

    def test_selection_roundtrip(self) -> None:
        # Valid scene selection persists and clears other fields.
        saved = self.client.post("/assets/selection", json={"scene": ""})
        self.assertEqual(200, saved.status_code, saved.text)
        self.assertIsNone(saved.json()["scene"])


class AssetWebImportContractTest(_AssetWebTestCase):
    """导入 / 删除契约：这些端点挂 ``require_admin``（与 /users 同姿态）。

    用 enable_db=True + 真实登录拿 admin token 驱动。
    """

    def setUp(self) -> None:
        super().setUp()
        from fastapi.testclient import TestClient

        from app.main import create_app

        app = create_app(
            control_server=_FakeControlServer(), enable_db=True, auth_enabled=True
        )
        self._client_cm = TestClient(app)
        self.client = self._client_cm
        self._client_cm.__enter__()
        self._client_entered = True
        login = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": _ADMIN_PASS},
        )
        self.assertEqual(200, login.status_code, login.text)
        self.admin_headers = {
            "Authorization": f"Bearer {login.json()['access_token']}"
        }

    def _import_scene(self, name: str = "web_scene") -> Dict[str, object]:
        response = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={"file": (f"{name}.zip", _scene_zip_bytes(name), "application/zip")},
            data={"skip_validate": "true"},
        )
        assert response.status_code == 201, response.text
        return response.json()["asset"]

    def test_import_scene_zip_end_to_end(self) -> None:
        record = self._import_scene()

        self.assertEqual("scene", record["kind"])
        self.assertEqual("web_scene", record["name"])
        self.assertEqual("1.0.0", record["version"])
        self.assertTrue(record["path"].startswith("scene/"))

        # Catalog persisted on disk inside the scratch library.
        catalog = yaml.safe_load(
            (self.assets_dir / "assets_catalog.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual("web_scene", catalog["assets"][0]["name"])

        # Listed back by GET /assets.
        listing = self.client.get(
            "/assets", headers=self.admin_headers
        ).json()
        self.assertEqual(["web_scene"], [a["name"] for a in listing["assets"]])

    def test_import_validates_scene_when_not_skipped(self) -> None:
        # Without skip_validate, the shared check_scene_config runs against the
        # normalized copy; a broken map fails the import with no trace.
        name = "web_scene"
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w") as archive:
            archive.writestr(
                f"{name}/manifest.yaml",
                yaml.safe_dump(_scene_manifest(name), sort_keys=False),
            )
            archive.writestr(
                f"{name}/scenes.yaml",
                yaml.safe_dump(_scene_document(name), sort_keys=False),
            )
            archive.writestr(f"{name}/cabinet_instances.yaml", "instances: []\n")
            archive.writestr(
                f"{name}/maps/inspection_map.yaml",
                yaml.safe_dump(MAP_FIELDS, sort_keys=False),
            )
            # missing inspection_map.pgm -> checker failure

        response = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={"file": (f"{name}.zip", buffer.getvalue(), "application/zip")},
        )
        self.assertEqual(400, response.status_code, response.text)
        self.assertIn("error", response.json())
        self.assertFalse((self.assets_dir / "scene" / name).exists())
        self.assertFalse((self.assets_dir / "assets_catalog.yaml").exists())

    def test_duplicate_import_conflicts_then_force_replaces(self) -> None:
        self._import_scene()
        duplicate = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={"file": ("dup.zip", _scene_zip_bytes(), "application/zip")},
            data={"skip_validate": "true"},
        )
        self.assertEqual(409, duplicate.status_code, duplicate.text)

        forced = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={
                "file": (
                    "v2.zip",
                    _scene_zip_bytes("web_scene", version="2.0.0"),
                    "application/zip",
                )
            },
            data={"skip_validate": "true", "force": "true"},
        )
        self.assertEqual(201, forced.status_code, forced.text)
        self.assertEqual("2.0.0", forced.json()["asset"]["version"])

    def test_rejects_zip_slip_members(self) -> None:
        response = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={
                "file": (
                    "evil.zip",
                    _zipped_bytes({"../escape.txt": b"boom"}),
                    "application/zip",
                )
            },
        )
        self.assertEqual(400, response.status_code, response.text)
        self.assertFalse((self.scratch / "escape.txt").exists())

    def test_rejects_zip_without_manifest(self) -> None:
        response = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={
                "file": (
                    "empty.zip",
                    _zipped_bytes({"some.txt": b"x"}),
                    "application/zip",
                )
            },
        )
        self.assertEqual(400, response.status_code, response.text)

    def test_rejects_invalid_manifest(self) -> None:
        response = self.client.post(
            "/assets/import",
            headers=self.admin_headers,
            files={
                "file": (
                    "bad.zip",
                    _zipped_bytes(
                        {"manifest.yaml": ("kind: scene\nname: Bad-Case\n").encode()}
                    ),
                    "application/zip",
                )
            },
        )
        self.assertEqual(400, response.status_code, response.text)

    def test_selection_validated_against_catalog(self) -> None:
        record = self._import_scene()

        saved = self.client.post(
            "/assets/selection",
            json={"scene": record["name"]},
            headers=self.admin_headers,
        )
        self.assertEqual(200, saved.status_code, saved.text)
        self.assertEqual(record["name"], saved.json()["scene"])
        selection = yaml.safe_load(
            (self.assets_dir / "selection.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(record["name"], selection["scene"])

        # GET /assets reflects the selection.
        listing = self.client.get(
            "/assets", headers=self.admin_headers
        ).json()
        self.assertEqual(record["name"], listing["selection"]["scene"])

    def test_delete_removes_asset_and_clears_selection(self) -> None:
        record = self._import_scene()
        self.client.post(
            "/assets/selection",
            json={"scene": record["name"]},
            headers=self.admin_headers,
        )

        removed = self.client.delete(
            f"/assets/scene/{record['name']}", headers=self.admin_headers
        )
        self.assertEqual(200, removed.status_code, removed.text)
        self.assertEqual(record["name"], removed.json()["removed"]["name"])
        # Selection field cleared so env mapping never points at a deleted asset.
        self.assertIsNone(removed.json()["selection"]["scene"])

        self.assertFalse((self.assets_dir / "scene" / record["name"]).exists())
        listing = self.client.get(
            "/assets", headers=self.admin_headers
        ).json()
        self.assertEqual([], listing["assets"])

    def test_delete_unknown_asset_404_and_bad_kind_400(self) -> None:
        unknown = self.client.delete(
            "/assets/scene/nope", headers=self.admin_headers
        )
        self.assertEqual(404, unknown.status_code, unknown.text)
        bad_kind = self.client.delete(
            "/assets/gripper/nope", headers=self.admin_headers
        )
        self.assertEqual(400, bad_kind.status_code, bad_kind.text)


class AssetWebAuthGateTest(_AssetWebTestCase):
    """auth 门禁：匿名 401、operator 403、admin 放行。"""

    def setUp(self) -> None:
        super().setUp()
        from fastapi.testclient import TestClient

        from app.main import create_app

        app = create_app(
            control_server=_FakeControlServer(), enable_db=True, auth_enabled=True
        )
        self._client_cm = TestClient(app)
        self.client = self._client_cm
        self._client_cm.__enter__()
        self._client_entered = True

        self._admin_token = self._login()
        created = self.client.post(
            "/users",
            json={
                "username": "op1",
                "password": "operator-pass",
                "role": "operator",
            },
            headers={"Authorization": f"Bearer {self._admin_token}"},
        )
        # 会话级共享 DB：op1 可能由本类前一个用例创建过。
        self.assertIn(created.status_code, (201, 409), created.text)
        operator_login = self.client.post(
            "/auth/login",
            json={"username": "op1", "password": "operator-pass"},
        )
        self.assertEqual(200, operator_login.status_code, operator_login.text)
        self._operator_token = operator_login.json()["access_token"]

    def _login(self) -> str:
        response = self.client.post(
            "/auth/login",
            json={"username": "admin", "password": _ADMIN_PASS},
        )
        self.assertEqual(200, response.status_code, response.text)
        return response.json()["access_token"]

    def test_read_endpoints_require_token(self) -> None:
        for path in ("/assets", "/assets/selection"):
            response = self.client.get(path)
            self.assertEqual(401, response.status_code, path)

    def test_import_requires_admin(self) -> None:
        payload = {
            "files": {
                "file": ("s.zip", _scene_zip_bytes("auth_scene"), "application/zip")
            },
            "data": {"skip_validate": "true"},
        }
        anonymous = self.client.post("/assets/import", **payload)
        self.assertEqual(401, anonymous.status_code, anonymous.text)

        operator = self.client.post(
            "/assets/import",
            headers={"Authorization": f"Bearer {self._operator_token}"},
            **payload,
        )
        self.assertEqual(403, operator.status_code, operator.text)

        admin = self.client.post(
            "/assets/import",
            headers={"Authorization": f"Bearer {self._admin_token}"},
            **payload,
        )
        self.assertEqual(201, admin.status_code, admin.text)

    def test_delete_requires_admin(self) -> None:
        seeded = self.client.post(
            "/assets/import",
            headers={"Authorization": f"Bearer {self._admin_token}"},
            files={"file": ("s.zip", _scene_zip_bytes("auth_scene"), "application/zip")},
            data={"skip_validate": "true"},
        )
        self.assertEqual(201, seeded.status_code, seeded.text)

        operator = self.client.delete(
            "/assets/scene/auth_scene",
            headers={"Authorization": f"Bearer {self._operator_token}"},
        )
        self.assertEqual(403, operator.status_code, operator.text)

        admin = self.client.delete(
            "/assets/scene/auth_scene",
            headers={"Authorization": f"Bearer {self._admin_token}"},
        )
        self.assertEqual(200, admin.status_code, admin.text)

    def test_authenticated_user_can_read_and_save_selection(self) -> None:
        listing = self.client.get(
            "/assets",
            headers={"Authorization": f"Bearer {self._operator_token}"},
        )
        self.assertEqual(200, listing.status_code, listing.text)
        saved = self.client.post(
            "/assets/selection",
            json={},
            headers={"Authorization": f"Bearer {self._operator_token}"},
        )
        self.assertEqual(200, saved.status_code, saved.text)


class _FakeControlServer:
    """其他路由所需的最小 stub（资产路由本身不触碰 control server）。"""

    def scenes(self) -> Dict[str, object]:
        return {"scenes": [], "active": None}

    def active_scene(self) -> Dict[str, object]:
        return {}

    def switch_scene(self, name: str) -> Dict[str, object]:
        del name
        return {}


if __name__ == "__main__":
    unittest.main()
