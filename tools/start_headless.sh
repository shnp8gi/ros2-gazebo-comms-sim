#!/usr/bin/env bash
# Docker導入後、一般ユーザーでイメージ作成・ビルド・ヘッドレス起動する。
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! command -v docker >/dev/null 2>&1; then
    echo "Dockerがありません。管理者に tools/setup_docker_host.sh の実行を依頼してください。" >&2
    exit 1
fi
if ! docker compose version; then
    echo "Docker Compose plugin が必要です。管理者に導入を依頼してください。" >&2
    exit 1
fi
if ! docker info >/dev/null; then
    echo "Dockerに接続できません。サービスの起動とdockerグループへの所属を確認してください。" >&2
    echo "グループ追加後はログアウト・再ログインが必要です。" >&2
    exit 1
fi

# GPU設定・ローカルoverrideを使わず、まずCPUで動作確認する。
compose=(docker compose -f "$repo_root/docker-compose.yml")
"${compose[@]}" config --quiet
"${compose[@]}" build sim

# ビルド成果物やCSVをホストの一般ユーザー所有で作成する。
# passwd登録のないUIDでもキャッシュやROSログを書けるようHOMEを指定する。
run_options=(--rm --no-deps --user "$(id -u):$(id -g)" -e HOME=/tmp)
"${compose[@]}" run "${run_options[@]}" build

echo "ビルド完了。ヘッドレスシミュレーションを起動します（中断: Ctrl+C）。"
# 任意UIDにはコンテナ内のユーザー名がないため、Gazeboが生成する既定の
# partitionがプロセスごとに異なる。全ノードに共通の値を明示する。
sim_partition="${GZ_PARTITION:-comms_sim_$(id -u)_$$}"
if [[ "${1:-}" == "--all" ]]; then
    shift
    sim_command=(python3 /workspace/tools/run_drone_scenarios.py "$@")
else
    sim_command=(ros2 launch comms_sim_pkg sim_launch.py headless:=true "$@")
fi
exec "${compose[@]}" run "${run_options[@]}" \
    -e DISPLAY= -e NVIDIA_VISIBLE_DEVICES=void -e "GZ_PARTITION=$sim_partition" sim \
    "${sim_command[@]}"
