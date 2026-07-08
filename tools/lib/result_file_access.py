"""
result_file_access.py
---------------------
シミュレーション結果ファイルへのホスト側アクセス権を保証するアダプタ層。

背景: コンテナ内 (root) で生成された CSV はホストユーザーから書き込めない。
docker compose 固有の知識(サービス名・chown コマンド)はこのモジュールに
閉じ込め、呼び出し側は「書き込めるようにする」という意図だけを表明する。
"""
import os
import subprocess


def ensure_writable(path: str, workspace_root: str = None,
                    docker_service: str = "sim", runner=subprocess.run) -> bool:
    """
    path をホストの現ユーザーで書き込み可能な状態にする。

    既に書き込み可能なら何もしない。不可能な場合はコンテナ経由で chown を試みる。
    runner は subprocess.run 互換の呼び出し可能オブジェクト(テスト時に差し替え可能)。

    Returns:
        処理後に書き込み可能なら True
    """
    if os.access(path, os.W_OK):
        return True

    workspace_root = workspace_root or os.getcwd()
    rel_path = os.path.relpath(path, workspace_root)
    try:
        runner(
            ["docker", "compose", "exec", "-T", docker_service, "chown",
             f"{os.getuid()}:{os.getgid()}", f"/workspace/{rel_path}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=30
        )
    except Exception:
        pass
    return os.access(path, os.W_OK)
