"""Semantic validators for imported assets (pure, injectable).

The manifest schema (:mod:`control_gateway.asset_manifest`) and the library
(:mod:`control_gateway.asset_library`) validate structure and declared-file
existence.  This module supplies the *semantic* checkers that reuse the existing
cross-file checkers (``scripts/check_scene_config`` for scenes; the cabinet
adapter contract is wired in 阶段3).  They run as subprocesses so the CLI and the
Web gateway share one implementation, and the returned ``validate`` hook is
injectable so tests can substitute fakes.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from .asset_manifest import AssetManifest


def _default_checker_path() -> Path:
    """``jiang/control_gateway/asset_validators.py`` -> workspace ``scripts/``."""
    workspace = Path(__file__).resolve().parents[2]
    return workspace / "scripts" / "check_scene_config"


def scene_validator(
    checker_path: Optional[Path | str] = None,
) -> Callable[[AssetManifest, Path], None]:
    """Return a ``validate`` hook that runs ``check_scene_config``.

    The checker runs against the imported (normalized) ``scenes.yaml``; it must
    raise on failure, which the library wraps into an ``AssetLibraryError`` and
    cleans up the partially imported directory.
    """
    checker = Path(checker_path) if checker_path else _default_checker_path()

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


def kind_validator(
    kind: str,
    *,
    checker_path: Optional[Path | str] = None,
) -> Optional[Callable[[AssetManifest, Path], None]]:
    """Return the semantic validator for an asset kind, or ``None``.

    ``None`` means schema-only validation (manifest + declared files), which is
    what cabinet assets receive until the adapter-contract checks are wired in
    阶段3.
    """
    if kind == "scene":
        return scene_validator(checker_path)
    return None
