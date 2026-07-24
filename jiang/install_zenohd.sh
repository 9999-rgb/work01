#!/usr/bin/env bash
# =============================================================================
# zenohd 自动安装脚本
# 根据当前操作系统和架构，从清华大学 TUNA 镜像站下载并安装 zenohd 1.9.0
#
# 用法:
#   chmod +x install_zenohd.sh
#   ./install_zenohd.sh [安装目录]
#
# 默认安装目录: /usr/local/bin  (需要 sudo)
# 自定义安装目录: ./install_zenohd.sh ~/.local/bin
# =============================================================================

set -euo pipefail

ZENOH_VERSION="1.9.0"
MIRROR_BASE="https://mirrors.tuna.tsinghua.edu.cn/eclipse/zenoh/zenoh/${ZENOH_VERSION}"
INSTALL_DIR="${1:-/usr/local/bin}"

# --------------- 颜色输出 ---------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; }
section() { echo -e "\n${CYAN}==>${NC} ${CYAN}$*${NC}"; }

# --------------- 检测操作系统 ---------------
detect_os() {
    case "$(uname -s | tr '[:upper:]' '[:lower:]')" in
        linux)  OS="linux" ;;
        darwin) OS="macos" ;;
        mingw*|msys*|cygwin*) OS="windows" ;;
        *)
            # 有些 Windows 环境 uname 返回 Windows_NT 等
            if [[ "$(uname -s)" == *"MINGW"* ]] || [[ "$(uname -s)" == *"MSYS"* ]] || [[ "$(uname -s)" == *"CYGWIN"* ]]; then
                OS="windows"
            else
                error "不支持的操作系统: $(uname -s)"
                exit 1
            fi
            ;;
    esac
    info "检测到操作系统: ${OS}"
}

# --------------- 检测架构 ---------------
detect_arch() {
    local arch
    arch=$(uname -m)
    case "${arch}" in
        x86_64|amd64)   ARCH="x86_64" ;;
        aarch64|arm64)  ARCH="aarch64" ;;
        armv7l|armv7)   ARCH="armv7" ;;
        arm*)           ARCH="arm" ;;
        *)
            error "不支持的CPU架构: ${arch}"
            exit 1
            ;;
    esac
    info "检测到CPU架构: ${ARCH}"
}

# --------------- 检测 libc (仅 Linux) ---------------
detect_libc() {
    if [[ "${OS}" != "linux" ]]; then
        LIBC=""  # macOS / Windows 不需要
        return
    fi

    # 检测是否为 musl (Alpine 等)
    if command -v ldd &>/dev/null && ldd --version 2>&1 | grep -qi "musl"; then
        LIBC="musl"
    elif [[ -f /etc/alpine-release ]]; then
        LIBC="musl"
    else
        LIBC="gnu"
    fi
    info "检测到 libc: ${LIBC}"
}

# --------------- 组合下载文件名 ---------------
build_package_name() {
    case "${OS}" in
        linux)
            # 包类型: standalone (自包含) 和 debian (需要系统依赖)
            # 优先选 standalone，更通用
            PKG_TYPE="standalone"

            case "${ARCH}" in
                x86_64)
                    TRIPLE="x86_64-unknown-linux-${LIBC}"
                    ;;
                aarch64)
                    TRIPLE="aarch64-unknown-linux-${LIBC}"
                    ;;
                armv7)
                    TRIPLE="armv7-unknown-linux-gnueabihf"
                    # armv7 只有 gnu 版本
                    if [[ "${LIBC}" == "musl" ]]; then
                        warn "armv7 没有 musl 版本，将使用 gnu 版本（可能不兼容 Alpine）"
                    fi
                    ;;
                arm)
                    TRIPLE="arm-unknown-linux-gnueabi"
                    if [[ "${LIBC}" == "musl" ]]; then
                        warn "arm 没有 musl 版本，将使用 gnu 版本（可能不兼容 Alpine）"
                    fi
                    ;;
            esac
            PKG_NAME="zenoh-${ZENOH_VERSION}-${TRIPLE}-${PKG_TYPE}"
            ;;

        macos)
            case "${ARCH}" in
                x86_64)  TRIPLE="x86_64-apple-darwin" ;;
                aarch64) TRIPLE="aarch64-apple-darwin" ;;
            esac
            PKG_NAME="zenoh-${ZENOH_VERSION}-${TRIPLE}-standalone"
            ;;

        windows)
            case "${ARCH}" in
                x86_64)
                    # 优先 MSVC，如果不可用则回退到 GNU
                    PKG_NAME="zenoh-${ZENOH_VERSION}-x86_64-pc-windows-msvc-standalone"
                    ;;
                *)
                    error "Windows 仅支持 x86_64 架构"
                    exit 1
                    ;;
            esac
            ;;
    esac

    info "目标包名: ${PKG_NAME}"
}

# --------------- 下载并安装 ---------------
download_and_install() {
    local url="${MIRROR_BASE}/${PKG_NAME}.zip"
    local tmpdir
    tmpdir=$(mktemp -d)
    trap 'rm -rf "${tmpdir}"' EXIT

    section "下载 zenohd"
    info "URL: ${url}"
    curl -fSL --progress-bar -o "${tmpdir}/${PKG_NAME}.zip" "${url}"

    section "解压"
    unzip -q -o "${tmpdir}/${PKG_NAME}.zip" -d "${tmpdir}/zenoh"

    section "安装到 ${INSTALL_DIR}"
    mkdir -p "${INSTALL_DIR}"

    # 查找 zenohd 可执行文件
    local zenohd_bin
    zenohd_bin=$(find "${tmpdir}/zenoh" -type f -name "zenohd" -o -name "zenohd.exe" | head -1)

    if [[ -z "${zenohd_bin}" ]]; then
        error "在解压后的文件中找不到 zenohd 可执行文件"
        info "解压内容:"
        find "${tmpdir}/zenoh" -type f | head -20
        exit 1
    fi

    cp "${zenohd_bin}" "${INSTALL_DIR}/"
    chmod +x "${INSTALL_DIR}/$(basename "${zenohd_bin}")"

    # 如果有插件目录(plugins)，一并复制
    if [[ -d "${tmpdir}/zenoh" ]]; then
        local plugin_dir
        plugin_dir=$(find "${tmpdir}/zenoh" -type d -name "plugins" -o -type d -name "lib" | head -1)
        # standalone 包通常是单文件的，直接完成
        :
    fi

    section "验证安装"
    if "${INSTALL_DIR}/$(basename "${zenohd_bin}")" --version 2>&1 || "${INSTALL_DIR}/$(basename "${zenohd_bin}")" -V 2>&1 || true; then
        info "zenohd 安装成功!"
        info "可执行文件: ${INSTALL_DIR}/$(basename "${zenohd_bin}")"
    fi
}

# --------------- 主流程 ---------------
main() {
    echo "============================================"
    echo "  zenohd ${ZENOH_VERSION} 自动安装脚本"
    echo "  镜像源: mirrors.tuna.tsinghua.edu.cn"
    echo "============================================"

    detect_os
    detect_arch
    detect_libc
    build_package_name
    download_and_install

    echo ""
    info "完成! 运行 zenohd 启动:"
    echo "    zenohd"
    echo ""
    info "或以后台模式运行:"
    echo "    zenohd &"
}

main