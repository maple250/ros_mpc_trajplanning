#!/usr/bin/env bash
# ============================================================
# 外部依赖安装脚本（在 src/mpc_planning/ 目录下运行）
#
# 产物布局（供 CMakeLists.txt 引用）：
#   External/Eigen/Eigen/...                 头文件库
#   External/Json/include/nlohmann/json.hpp  头文件库
#   External/matplotlib/matplotlibcpp.h      头文件库
#   External/blasfeo/lib/include/*.h         静态库头文件
#   External/blasfeo/lib/lib/libblasfeo.a    静态库
#   External/hpipm/lib/include/*.h           静态库头文件
#   External/hpipm/lib/lib/libhpipm.a        静态库
#
# 说明：blasfeo/hpipm 必须在与最终链接一致的工具链下编译
#       （即容器内 Ubuntu 20.04 + ROS Noetic），故本脚本通常由
#       Dockerfile 在镜像构建时调用，而非在 24.04 宿主机上运行。
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
EXT="$SCRIPT_DIR/External"
mkdir -p "$EXT"

NPROC="$(nproc 2>/dev/null || echo 4)"

# 幂等克隆：已是 git 仓库则跳过（注意：空目录不算，会被克隆填充）
clone_if_missing() {
    local url="$1" dir="$2" ref="${3:-}"
    if [ -d "$dir/.git" ]; then
        echo "[skip] clone: $dir 已是 git 仓库"
        return 0
    fi
    mkdir -p "$(dirname "$dir")"
    if [ -n "$ref" ]; then
        git clone --depth 1 --branch "$ref" "$url" "$dir"
    else
        git clone --depth 1 "$url" "$dir"
    fi
}

echo "########## 1/5 头文件库：Eigen ##########"
clone_if_missing https://gitlab.com/libeigen/eigen.git "$EXT/Eigen"

echo "########## 2/5 头文件库：nlohmann/json ##########"
clone_if_missing https://github.com/nlohmann/json.git "$EXT/Json"

echo "########## 3/5 头文件库：matplotlib-cpp ##########"
clone_if_missing https://github.com/lava/matplotlib-cpp.git "$EXT/matplotlib"

echo "########## 4/5 静态库：blasfeo (TARGET=X64_INTEL_HASWELL) ##########"
if [ -f "$EXT/blasfeo/lib/lib/libblasfeo.a" ]; then
    echo "[skip] blasfeo 已构建: $EXT/blasfeo/lib/lib/libblasfeo.a"
else
    clone_if_missing https://github.com/giaf/blasfeo.git "$EXT/blasfeo"
    cmake -S "$EXT/blasfeo" -B "$EXT/blasfeo/build" \
        -DCMAKE_INSTALL_PREFIX="$EXT/blasfeo/lib" \
        -DTARGET=X64_INTEL_HASWELL \
        -DLA=HIGH_PERFORMANCE \
        -DBLAS_API=OFF
    cmake --build "$EXT/blasfeo/build" -j"$NPROC"
    cmake --install "$EXT/blasfeo/build"
fi

echo "########## 5/5 静态库：hpipm (复用预编译 blasfeo) ##########"
if [ -f "$EXT/hpipm/lib/lib/libhpipm.a" ]; then
    echo "[skip] hpipm 已构建: $EXT/hpipm/lib/lib/libhpipm.a"
else
    if [ ! -f "$EXT/blasfeo/lib/lib/libblasfeo.a" ]; then
        echo "[error] 未找到 blasfeo，hpipm 依赖它，请先完成 blasfeo 构建" >&2
        exit 1
    fi
    clone_if_missing https://github.com/giaf/hpipm.git "$EXT/hpipm"
    # HPIPM_FIND_BLASFEO=OFF(默认): 用 BLASFEO_PATH 指向预编译 blasfeo，不重新编译
    cmake -S "$EXT/hpipm" -B "$EXT/hpipm/build" \
        -DCMAKE_INSTALL_PREFIX="$EXT/hpipm/lib" \
        -DBLASFEO_PATH="$EXT/blasfeo/lib" \
        -DHPIPM_FIND_BLASFEO=OFF
    cmake --build "$EXT/hpipm/build" -j"$NPROC"
    cmake --install "$EXT/hpipm/build"
fi

echo "########## 完成 ##########"
echo "blasfeo: $EXT/blasfeo/lib/lib/libblasfeo.a"
echo "hpipm:   $EXT/hpipm/lib/lib/libhpipm.a"
