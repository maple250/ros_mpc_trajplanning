#!/usr/bin/env bash
# 宿主机侧运行容器的辅助脚本
# 用法：
#   ./docker/run.sh                # 进容器 shell（默认挂载源码便于改代码）
#   ./docker/run.sh roslaunch mpcplanning intercept_mpc.launch gui:=false
#   ./docker/run.sh --no-mount ... # 用镜像内烘焙的源码，不挂载宿主机源码
set -e

IMG="${IMAGE:-mpc-noetic:full}"
DEV_MOUNT="${DEV_MOUNT:-1}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$REPO_ROOT/src/mpc_planning"

# 处理 --no-mount
if [ "${1:-}" = "--no-mount" ]; then
    DEV_MOUNT=0
    shift
fi

# 允许容器访问宿主机 X server（Gazebo GUI 需要）
xhost +local:docker >/dev/null 2>&1 || true

ARGS=(
    --rm -it
    --runtime=nvidia --gpus all
    --net=host
    --env DISPLAY="${DISPLAY:-:1}"
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
)

if [ "$DEV_MOUNT" = "1" ] && [ -d "$PKG" ]; then
    # 挂载源码便于实时编辑；External/ 用命名卷保留镜像内已构建的依赖
    ARGS+=(
        --volume "$PKG:/catkin_ws/src/mpc_planning:rw"
        --volume mpc_external:/catkin_ws/src/mpc_planning/External:rw
    )
fi

docker run "${ARGS[@]}" --name mpc-dev "$IMG" "$@"
