#!/usr/bin/env bash

set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
visual_mesh_dir="${package_dir}/meshes"
collision_mesh_dir="${visual_mesh_dir}/collision"
filter_script="${package_dir}/config/convex_hull.mlx"

if ! command -v meshlabserver >/dev/null 2>&1; then
  echo "meshlabserver is required to generate collision meshes." >&2
  exit 1
fi

mkdir -p -- "${collision_mesh_dir}"

for visual_mesh in "${visual_mesh_dir}"/*.STL; do
  mesh_name="$(basename -- "${visual_mesh}")"
  collision_mesh="${collision_mesh_dir}/${mesh_name}"

  echo "Generating convex collision mesh: ${mesh_name}"
  meshlabserver \
    -i "${visual_mesh}" \
    -o "${collision_mesh}" \
    -s "${filter_script}"
done
