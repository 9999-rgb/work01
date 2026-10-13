#!/usr/bin/env bash
# 重建 Gazebo 11 本体，保留跨动态库的共享 SingletonT 实例。
set -Eeuo pipefail
SOURCE_DIR="${1:?需要 Gazebo 源码归档目录}"
PREFIX=/opt/gazebo-fixed
BUILD_JOBS="${BUILD_JOBS:-2}"
[[ "$BUILD_JOBS" =~ ^[1-9][0-9]?$ ]] || { echo 'BUILD_JOBS 应为 1～99。' >&2; exit 1; }
cd "$SOURCE_DIR"
sha256sum -c SHA256SUMS
WORK="$(mktemp -d -t xczs-gazebo-build.XXXXXX)"
trap 'rm -rf -- "$WORK"' EXIT
tar -xf gazebo_11.10.2+dfsg.orig.tar.xz --strip-components=1 -C "$WORK"
tar -xf gazebo_11.10.2+dfsg-1osrf6.debian.tar.xz -C "$WORK"
while IFS= read -r patch_name; do
    [[ "$patch_name" != 0010-armhf-singleton.patch ]] || continue
    [[ "$patch_name" =~ ^[a-zA-Z0-9_.-]+\.patch$ ]] || { echo '源码补丁名称无效。' >&2; exit 1; }
    patch --batch --forward -p1 -d "$WORK" < "$WORK/debian/patches/$patch_name"
done < "$WORK/debian/patches/series"
# GCC 使用上游显式导出声明；被清空的声明会使各动态库创建独立管理对象。
grep -Fq 'template class visibility ::SingletonT' "$WORK/gazebo/common/SingletonT.hh"
cmake -S "$WORK" -B "$WORK/build" \
    -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_TESTING=OFF \
    -DUSE_HOST_CFLAGS=False -DUSE_UPSTREAM_CFLAGS=False
cmake --build "$WORK/build" --parallel "$BUILD_JOBS"
# 上游安装钩子使用相对目录调用 man 目标，必须在构建目录执行安装。
cmake --build "$WORK/build" --target man --parallel "$BUILD_JOBS"
(cd "$WORK/build" && cmake --install .)
python3 /usr/local/lib/xczs/check_gazebo_singletons.py "$PREFIX/lib"
