#!/usr/bin/env bash
# ============================================================
# 为 docker daemon 配置代理（拉取 Docker Hub 镜像需要，因直连被墙）
# 需以 root 运行：  sudo bash docker/daemon-proxy.sh
# 代理写在 systemd drop-in，不触碰 nvidia-ctk 已写的 daemon.json
# ============================================================
set -e
PROXY="${https_proxy:-http://127.0.0.1:10801}"

mkdir -p /etc/systemd/system/docker.service.d
cat > /etc/systemd/system/docker.service.d/http-proxy.conf <<EOF
[Service]
Environment="HTTP_PROXY=${PROXY}"
Environment="HTTPS_PROXY=${PROXY}"
Environment="NO_PROXY=localhost,127.0.0.1,::1,172.16.0.0/12,192.168.0.0/16,10.0.0.0/8"
EOF

systemctl daemon-reload
systemctl restart docker

echo "Docker daemon 代理已配置: ${PROXY}"
echo "--- 验证 ---"
docker info 2>/dev/null | grep -iE 'Proxy|Server Version' || true
