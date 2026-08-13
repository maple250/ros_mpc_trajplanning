#!/usr/bin/env bash
# 容器入口：source ROS / 工作空间 / PX4 环境，再执行用户命令
set -e

# ---- ROS Noetic ----
source /opt/ros/noetic/setup.bash

# ---- 本 catkin 工作空间（若已构建）----
if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

# ---- PX4 SITL（若已安装）：使 $(find px4) 解析、设置 gazebo 插件/模型路径 ----
PX4=/opt/PX4-Autopilot
if [ -d "$PX4" ]; then
    PX4_BUILD="$(ls -d "$PX4"/build/px4_sitl* 2>/dev/null | head -n1 || true)"
    # setup_gazebo.bash 位置随版本变动，逐个尝试
    for s in \
        "$PX4/Tools/setup_gazebo.bash" \
        "$PX4/Tools/simulation/gazebo-classic/setup_gazebo.bash"; do
        if [ -f "$s" ]; then
            # shellcheck disable=SC1090
            source "$s" "$PX4" "$PX4_BUILD" >/dev/null 2>&1 || true
            break
        fi
    done
    # gazebo-classic 包目录随 PX4 版本变动（v1.13: Tools/sitl_gazebo-classic；
    # v1.14+: Tools/simulation/gazebo-classic/sitl_gazebo-classic），按 package.xml 实测定位
    GAZEBO_CLASSIC_PKG=""
    for g in \
        "$PX4/Tools/sitl_gazebo-classic" \
        "$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic"; do
        if [ -f "$g/package.xml" ]; then
            GAZEBO_CLASSIC_PKG="$g"
            break
        fi
    done
    export ROS_PACKAGE_PATH="${PX4}${GAZEBO_CLASSIC_PKG:+:$GAZEBO_CLASSIC_PKG}${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
fi

# ---- NVIDIA GL：glvnd 优先使用 NVIDIA vendor（由 nvidia-container-toolkit 注入驱动库）----
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"

exec "$@"
