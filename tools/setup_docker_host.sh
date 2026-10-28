#!/usr/bin/env bash
# 管理者が一度実行する、Ubuntu 22.04 用の Docker 導入スクリプト。
# 公式手順: https://docs.docker.com/engine/install/ubuntu/
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "使い方: sudo bash $0 <Dockerを利用するユーザー名>" >&2
    exit 2
fi
sim_user="$1"
if [[ $EUID -ne 0 ]]; then
    echo "管理者権限が必要です。sudo可能なアカウントから実行してください。" >&2
    exit 1
fi
id "$sim_user" >/dev/null
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
    echo "このスクリプトは Ubuntu 22.04 用です。" >&2
    exit 1
fi

# 既存環境のパッケージを自動削除しない。
conflicts=()
for package in docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc; do
    if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" == 'install ok installed' ]]; then
        conflicts+=("$package")
    fi
done
if (( ${#conflicts[@]} )); then
    printf '競合するパッケージがあります。削除せず停止します: %s\n' "${conflicts[*]}" >&2
    exit 1
fi

apt-get update
apt-get install -y --no-remove ca-certificates curl
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
cat > /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: jammy
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
apt-get update
apt-get install -y --no-remove docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
systemctl enable --now docker

# docker グループへの所属は実質的に root 相当の権限を付与する。
if ! getent group docker >/dev/null; then
    groupadd docker
fi
usermod -aG docker "$sim_user"
docker version
docker compose version
printf '\n導入完了。%s はログアウト・再ログイン後に docker info を確認してください。\n' "$sim_user"
