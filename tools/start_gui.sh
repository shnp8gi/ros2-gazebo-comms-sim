#!/usr/bin/env bash
# Xauthorityで現在のデスクトップへ表示。Docker GPU連携がなくてもMesaで描画する。
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
if [[ -z "${DISPLAY:-}" ]]; then
    echo "GUIを表示するデスクトップのDISPLAYがありません。" >&2
    exit 1
fi
xauth_file="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -r "$xauth_file" ]]; then
    echo "Xauthorityを読み取れません: $xauth_file" >&2
    exit 1
fi
docker info >/dev/null
compose=(docker compose -f "$repo_root/docker-compose.yml")
run_options=(--rm --no-deps --user "$(id -u):$(id -g)" -e HOME=/tmp)
"${compose[@]}" run "${run_options[@]}" build
sim_partition="${GZ_PARTITION:-drone_gui_$(id -u)_$$}"
exec "${compose[@]}" run "${run_options[@]}" \
    -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/drone.Xauthority \
    -v "$xauth_file:/tmp/drone.Xauthority:ro" \
    -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e NVIDIA_VISIBLE_DEVICES=void -e "GZ_PARTITION=$sim_partition" sim \
    ros2 launch comms_sim_pkg sim_launch.py headless:=false pause_at_encounter:=true keep_gui_open:=true "$@"
