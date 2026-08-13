#!/usr/bin/env bash
# 镜像构建脚本（用 sg docker 免密、--network=host + 代理 build-arg 应对国内网络）
# 用法: build.sh mpc   -> 构建 mpc-noetic:mpc (mpc-built 阶段，验证编译，无 PX4)
#       build.sh full  -> 构建 mpc-noetic:full (final 阶段，含 PX4 SITL)
set -e
TARGET="${1:-mpc}"
cd /home/nvidia/drone_ws/path_planning/ros_mpc_trajplanning

if [ "$TARGET" = "full" ]; then
    TAG="mpc-noetic:full"; STAGE="final"
else
    TAG="mpc-noetic:mpc"; STAGE="mpc-built"
fi

exec sg docker -c "docker build --network=host \
  --build-arg HTTP_PROXY=http://127.0.0.1:10801 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:10801 \
  --build-arg NO_PROXY=localhost,127.0.0.1,::1 \
  --progress=plain --target ${STAGE} -t ${TAG} ."
