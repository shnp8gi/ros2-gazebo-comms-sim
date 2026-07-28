"""
REM 状態 (α, P, S2, W, meta) の走行間永続化 (本番仕様 §5.3)。

「設置してまっさら開始 → 走行を重ねて収束」の自己組織化を実現する。
学習相 (sim.py learn) が状態を作り、評価相 (kkf_conv) が load して開始する。

責務: 直列化・復元・忘却適用のみ。学習も予測もしない。
"""
import hashlib
import json
import os
import tempfile

import numpy as np

STATE_VERSION = 1


def basis_hash(basis_config):
    """基底設定 dict の正規化ハッシュ。不一致の状態を黙ってロードしないための鍵。"""
    payload = json.dumps(basis_config, sort_keys=True)
    return hashlib.sha1(payload.encode()).hexdigest()[:16]


class RemStore:
    """RSU (BS) ごとの状態ファイル rem_state/bs<j>.npz の読み書き。"""

    def __init__(self, state_dir):
        self.state_dir = str(state_dir)

    def path(self, bs_id):
        """状態ファイルのパス。bs_id は int でも文字列キーでもよい
        (方向別地図では "0_p" / "0_m" のような複合キーを使う。単一方向では
        従来どおり "0" となり、既存の学習済み状態がそのまま読める)。"""
        return os.path.join(self.state_dir, f"bs{bs_id}.npz")

    def exists(self, bs_id):
        return os.path.isfile(self.path(bs_id))

    def save(self, bs_id, alpha, P, S2, W, meta):
        """アトミック保存 (tmp → rename)。run 途中の定期上書きに耐える。

        meta には少なくとも basis_hash を含めること (load 時の整合検証キー)。
        run_count は保存のたびに呼び出し側が進める。
        """
        if 'basis_hash' not in meta:
            raise ValueError("RemStore.save: meta に basis_hash が必要")
        os.makedirs(self.state_dir, exist_ok=True)
        full_meta = dict(meta)
        full_meta['state_version'] = STATE_VERSION
        fd, tmp = tempfile.mkstemp(suffix='.npz.tmp', dir=self.state_dir)
        try:
            with os.fdopen(fd, 'wb') as f:
                np.savez(f,
                         alpha=np.asarray(alpha, dtype=float),
                         P=np.asarray(P, dtype=float),
                         S2=np.asarray(S2, dtype=float),
                         W=np.asarray(W, dtype=float),
                         meta=json.dumps(full_meta))
            os.replace(tmp, self.path(bs_id))
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def load(self, bs_id, expected_basis_hash=None, q_forget=0.0, gamma=1.0):
        """状態を復元し、忘却を適用して返す。ファイルが無ければ None。

        忘却 (本番仕様 §5.3、load 時に一度だけ適用):
          P  += q_forget·I   (平均場の確信を緩め、環境変化に追従)
          S2 ·= γ, W ·= γ    (σ_ν² は不変のまま重みを減らす遅い割引)

        expected_basis_hash 不一致は ValueError (黙って壊れた状態で走らせない)。
        """
        p = self.path(bs_id)
        if not os.path.isfile(p):
            return None
        with np.load(p) as data:
            meta = json.loads(str(data['meta']))
            alpha = data['alpha'].astype(float)
            P = data['P'].astype(float)
            S2 = data['S2'].astype(float)
            W = data['W'].astype(float)
        if expected_basis_hash is not None:
            found = meta.get('basis_hash')
            if found != expected_basis_hash:
                raise ValueError(
                    f"RemStore: basis 設定不一致 (bs{bs_id}: 保存 {found}, "
                    f"期待 {expected_basis_hash})。基底設定を保存時と揃えるか "
                    f"状態ディレクトリを作り直すこと")
        if P.shape != (len(alpha), len(alpha)):
            raise ValueError(f"RemStore: P 形状不正 (bs{bs_id}: {P.shape})")
        if q_forget > 0.0:
            P = P + q_forget * np.eye(len(alpha))
        if gamma != 1.0:
            S2 = S2 * gamma
            W = W * gamma
        return {'alpha': alpha, 'P': P, 'S2': S2, 'W': W, 'meta': meta}
