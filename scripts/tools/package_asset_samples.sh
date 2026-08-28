#!/usr/bin/env bash
# Package every importable asset under the configured roots into a
# ready-to-upload zip.  The archive keeps manifest.yaml at its root — the
# layout the Web /assets/import endpoint (jiang/app/api/assets.py
# _find_asset_root) accepts.
#
# Roots:
#   - jiang/samples/      portable sample assets
#   - model/场景-资产/     built-in scene + cabinet assets for import
#
# Usage:
#   bash scripts/tools/package_asset_samples.sh               # all assets
#   bash scripts/tools/package_asset_samples.sh demo_cabinet  # one asset by name
#
# Generated zips land next to their source directory (<root>/<name>.zip) and
# are meant to be uploaded through the 8090 Web console.  They are build
# artifacts and are not committed to git.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PACKAGE_ROOTS=("$ROOT/jiang/samples" "$ROOT/model/场景-资产")
EXCLUDES=(-x '*.pyc' '*__pycache__/*' '.DS_Store' '.git/*')

# Optional positional args restrict which assets to package (by name).  Each
# target is recorded as "name<DELIM>root" so a name present in several roots
# (e.g. demo_cabinet) is packaged in every root that holds it.
DELIM=$'\t'
targets=()
if (($# > 0)); then
    for name in "$@"; do
        found=0
        for root in "${PACKAGE_ROOTS[@]}"; do
            if [[ -f "$root/$name/manifest.yaml" ]]; then
                targets+=("$name$DELIM$root")
                found=1
            fi
        done
        if ((found == 0)); then
            echo "no asset named '$name' under ${PACKAGE_ROOTS[*]}" >&2
            exit 1
        fi
    done
else
    # Auto-discover: every directory under any root with a root manifest.yaml.
    # ``*_bk`` backups are excluded — they are preserved snapshots (often with
    # library-absolute references) and packaging them yields non-portable zips.
    for root in "${PACKAGE_ROOTS[@]}"; do
        [[ -d "$root" ]] || continue
        for dir in "$root"/*/; do
            name="$(basename "$dir")"
            [[ "$name" == *_bk ]] && continue
            [[ -f "$dir/manifest.yaml" ]] && targets+=("$name$DELIM$root")
        done
    done
fi

if ((${#targets[@]} == 0)); then
    echo "no importable assets found under ${PACKAGE_ROOTS[*]}" >&2
    exit 1
fi

for target in "${targets[@]}"; do
    name="${target%%$DELIM*}"
    root="${target#*$DELIM}"
    dir="$root/$name"

    dest="$root/$name.zip"
    (cd "$dir" && zip -q -r "$dest" . "${EXCLUDES[@]}")

    # Self-check: manifest.yaml must sit at the archive root, not nested.
    if ! unzip -Z1 "$dest" | grep -qx 'manifest.yaml'; then
        echo "packaging check failed: manifest.yaml not at root of $dest" >&2
        exit 1
    fi

    size=$(du -h "$dest" | cut -f1)
    echo "packaged: $dest ($size)"
done
