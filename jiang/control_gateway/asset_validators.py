"""Semantic validators for imported assets (pure, injectable).

The manifest schema (:mod:`control_gateway.asset_manifest`) and the library
(:mod:`control_gateway.asset_library`) validate structure and declared-file
existence.  This module supplies the *semantic* checkers that reuse the existing
cross-file checkers (``scripts/validate/check_scene_config`` for scenes; the
parametrized ``scripts/validate/check_cabinet_model --asset`` for cabinets).
They run as subprocesses
so the CLI and the Web gateway share one implementation, and the returned
``validate`` hook is injectable so tests can substitute fakes.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from .asset_manifest import AssetManifest


def _default_scene_checker_path() -> Path:
    """``jiang/control_gateway/asset_validators.py`` -> workspace ``scripts/validate/``."""
    workspace = Path(__file__).resolve().parents[2]
    return workspace / "scripts" / "validate" / "check_scene_config"


def _default_cabinet_checker_path() -> Path:
    workspace = Path(__file__).resolve().parents[2]
    return workspace / "scripts" / "validate" / "check_cabinet_model"


def scene_validator(
    checker_path: Optional[Path | str] = None,
) -> Callable[[AssetManifest, Path], None]:
    """Return a ``validate`` hook that runs ``check_scene_config``.

    The checker runs against the imported (normalized) ``scenes.yaml``; it must
    raise on failure, which the library wraps into an ``AssetLibraryError`` and
    cleans up the partially imported directory.
    """
    checker = Path(checker_path) if checker_path else _default_scene_checker_path()

    def validate(manifest: AssetManifest, root: Path) -> None:
        scenes = manifest.file_path(root, "scenes")
        proc = subprocess.run(
            [sys.executable, str(checker), "--scenes", str(scenes)],
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            message = (proc.stderr or proc.stdout or "").strip()
            raise ValueError(message or f"scene config check failed ({proc.returncode})")

    return validate


def cabinet_validator(
    checker_path: Optional[Path | str] = None,
) -> Callable[[AssetManifest, Path], None]:
    """Return a ``validate`` hook that runs ``check_cabinet_model --asset``.

    The parametrized cabinet checker validates the imported asset's structural
    self-consistency: controls <-> URDF <-> state plugin agreement, and the
    per-cabinet robot-adapter capability allowlist
    (``operable_control_ids`` / per-control ``navigation_station``) against
    the controls catalog.  Unlisted controls remain planning-only.  It expands
    the asset Xacro under the asset name, so
    ``xacro`` and the ROS workspace must be available (the launch/CLI run with
    them sourced).  It does *not* assert the built-in frozen physics or the
    fixed robot stack — physical closure is declared by the asset's positive
    capability allowlist.
    """

    checker = (
        Path(checker_path) if checker_path else _default_cabinet_checker_path()
    )

    def validate(manifest: AssetManifest, root: Path) -> None:
        def path_for(role: str) -> str:
            return str(manifest.file_path(root, role))

        proc = subprocess.run(
            [
                sys.executable,
                str(checker),
                "--asset",
                "--controls",
                path_for("controls"),
                "--scene",
                path_for("scene"),
                "--adapter",
                path_for("adapter"),
                "--xacro",
                path_for("xacro"),
                "--cabinet-name",
                manifest.name,
            ],
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            message = (proc.stderr or proc.stdout or "").strip()
            raise ValueError(
                message or f"cabinet model check failed ({proc.returncode})"
            )

    return validate


def kind_validator(
    kind: str,
    *,
    checker_path: Optional[Path | str] = None,
) -> Optional[Callable[[AssetManifest, Path], None]]:
    """Return the semantic validator for an asset kind, or ``None``.

    ``None`` means schema-only validation (manifest + declared files).
    Scenes run ``check_scene_config``; cabinets run the parametrized
    ``check_cabinet_model --asset`` (结构自洽 + 可达性配对, 不声称物理闭环).
    """
    if kind == "scene":
        return scene_validator(checker_path)
    if kind == "cabinet":
        return cabinet_validator(checker_path)
    return None
