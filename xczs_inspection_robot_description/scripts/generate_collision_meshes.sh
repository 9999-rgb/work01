#!/usr/bin/env bash

# Regenerate the dual-arm robot's collision meshes from its visual meshes.
#
# The LinkForge export (model/双臂机器人) ships empty (0-triangle) *_collision.stl
# files, so the robot has no collision geometry in Gazebo or MoveIt until these
# are generated.
#
# Two strategies, because a single one does not fit every link:
#   * ARM links (arm_0..arm_6, both sides): quadric edge-collapse DECIMATION of
#     the visual mesh (scripts/decimate_arm.py, VTK).  Convex hulls are
#     unsuitable here: the wrist (arm_6) is genuinely non-convex, so its hull
#     bulges into the elbow (arm_4) and creates phantom self-collisions that
#     make every MoveIt plan fail with START_STATE_INVALID.  A decimated surface
#     keeps the concavity (no phantom contacts) while staying cheap for FCL.
#   * Body / wheels / cameras / lasers: single convex hull (meshlabserver), the
#     same approach the legacy single-arm robot used.
#
# Usage: ./scripts/generate_collision_meshes.sh
# Requires: python3 + vtk (decimation), meshlabserver (convex hulls).

set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mesh_dir="${package_dir}/meshes/dual_arm"
filter_script="${package_dir}/config/convex_hull.mlx"

if ! python3 -c 'import vtk' >/dev/null 2>&1; then
  echo "python3-vtk is required to decimate the arm collision meshes." >&2
  exit 1
fi
if ! command -v meshlabserver >/dev/null 2>&1; then
  echo "meshlabserver is required to generate body/wheel/camera hulls." >&2
  exit 1
fi

shopt -s nullglob
visual_meshes=("${mesh_dir}"/*_visual.stl)

for visual_mesh in "${visual_meshes[@]}"; do
  base="$(basename -- "${visual_mesh}")"
  collision_mesh="${mesh_dir}/${base/_visual/_collision}"

  if [[ "${base}" =~ _arm_[0-6]_visual ]]; then
    echo "Decimating arm collision mesh: ${base}"
    python3 "${package_dir}/scripts/decimate_arm.py" \
      "${visual_mesh}" "${collision_mesh}"
  else
    echo "Generating convex collision mesh: ${base}"
    meshlabserver \
      -i "${visual_mesh}" \
      -o "${collision_mesh}" \
      -s "${filter_script}" >/dev/null 2>&1
  fi
done

echo "Done. Regenerated ${#visual_meshes[@]} collision meshes in ${mesh_dir}."
