#!/usr/bin/env bash
# Install upstream Humble's ABBA deadlock fix; never overwrite ROS setup.bash.
set -eo pipefail
vendor_dir=$(realpath "${1:?source archive directory required}")
prefix=${2:-/opt/ros/humble}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
jobs=${BUILD_JOBS:-2}
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'Invalid BUILD_JOBS' >&2; exit 1; }
work=$(mktemp -d /tmp/xczs-tf2-build.XXXXXX)
trap 'rm -rf "$work"' EXIT
(cd "$vendor_dir" && sha256sum -c SHA256SUMS)
tar -xzf "$vendor_dir/geometry2-0.25.24.tar.gz" -C "$work"
src="$work/geometry2-0.25.24"
mkdir -p "$prefix"
prefix=$(realpath "$prefix")
for package in tf2 tf2_ros; do
  cmake -S "$src/$package" -B "$work/$package" \
    -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
  cmake --build "$work/$package" --parallel "$jobs"
  cmake --install "$work/$package"
  export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
done
cmake -S "$script_dir/tf2_regression" -B "$work/test" \
  -DGEOMETRY2_SOURCE="$src" -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$work/test" --parallel "$jobs"
ldd "$work/test/test_buffer" > "$work/linkage.txt"
cat "$work/linkage.txt"
for library in tf2 tf2_ros; do
  grep -F "lib${library}.so => $prefix/lib/" "$work/linkage.txt"
done
ctest --test-dir "$work/test" --output-on-failure
mkdir -p "$prefix/share/xczs-tf2-fix"
printf '%s\n' 'geometry2 0.25.24; upstream Humble PR #990; test_buffer passed' \
  > "$prefix/share/xczs-tf2-fix/build.txt"
cp "$vendor_dir/SHA256SUMS" "$prefix/share/xczs-tf2-fix/"
