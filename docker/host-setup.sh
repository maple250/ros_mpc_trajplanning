#!/usr/bin/env bash
# ============================================================
# 宿主机（Ubuntu 24.04）一次性环境准备：安装 Docker + nvidia-container-toolkit
# 需以 root 运行：  sudo bash docker/host-setup.sh
# Docker CE 走清华镜像（国内稳定），nvidia 源加重试
# ============================================================
set -e

REAL_USER="${SUDO_USER:-$USER}"
CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
ARCH="$(dpkg --print-architecture)"
# 清华 Docker CE 镜像
DOCKER_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/docker-ce"
CURL_OPTS="-fsSL --retry 5 --retry-all-errors --retry-delay 2"
# nvidia.github.io 在国内需走代理（sudo 不继承用户代理环境变量，故显式指定）
PROXY="${https_proxy:-http://127.0.0.1:10801}"

echo "########## 1/4 安装 Docker Engine（清华镜像，Ubuntu 24.04 ${CODENAME}）##########"
# 清理上次失败可能留下的残文件
rm -f /etc/apt/sources.list.d/docker.list /etc/apt/keyrings/docker.gpg
apt-get update
apt-get install -y ca-certificates curl gnupg lsb-release
install -m 0755 -d /etc/apt/keyrings
curl $CURL_OPTS "${DOCKER_MIRROR}/linux/ubuntu/gpg" \
    | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/docker.gpg] ${DOCKER_MIRROR}/linux/ubuntu ${CODENAME} stable" \
    > /etc/apt/sources.list.d/docker.list
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

echo "########## 2/4 安装 nvidia-container-toolkit（走代理 $PROXY）##########"
# 临时 apt 代理配置（仅本步使用，结束后删除以恢复直连）
cat > /etc/apt/apt.conf.d/99proxy-tmp <<EOF
Acquire::http::Proxy "$PROXY";
Acquire::https::Proxy "$PROXY";
EOF
curl $CURL_OPTS --proxy "$PROXY" https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl $CURL_OPTS --proxy "$PROXY" https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update
apt-get install -y nvidia-container-toolkit
rm -f /etc/apt/apt.conf.d/99proxy-tmp   # 恢复直连（nvidia 源后续更新需手动开代理）

echo "########## 3/4 配置 Docker 使用 NVIDIA runtime ##########"
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

echo "########## 4/4 将用户 $REAL_USER 加入 docker 组 ##########"
usermod -aG docker "$REAL_USER"

echo ""
echo "================ 完成 ================"
echo "请【重新登录】(或执行 newgrp docker)使 docker 组生效。"
echo "验证 Docker:        docker run --rm hello-world"
echo "验证 GPU 直通:      docker run --rm --gpus all ubuntu nvidia-smi"
