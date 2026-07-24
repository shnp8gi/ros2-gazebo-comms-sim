#!/usr/bin/env python3
"""
RemStore 単体テスト: 保存/復元の往復一致・忘却適用・basis ハッシュ検証 (本番仕様 §5.3)。
コンテナ内で実行:
  docker compose exec -T sim bash -c \
    "source /workspace/install/setup.bash && python3 /workspace/tools/tests/rem_store_test.py"
"""
import os
import shutil
import sys
import tempfile

import numpy as np

from comms_sim_pkg.kkf_core import RemStore, basis_hash


def main():
    failures = []
    tmpdir = tempfile.mkdtemp(prefix='rem_store_test_')
    try:
        store = RemStore(os.path.join(tmpdir, 'rem_state'))
        rng = np.random.default_rng(0)
        p = 20
        alpha = rng.normal(size=p)
        P = np.eye(p) * 3.0
        S2 = rng.uniform(0, 100, size=101)
        W = rng.uniform(0.1, 50, size=101)
        bhash = basis_hash({'type': 'rbf', 's_min': 0.0, 's_max': 200.0,
                            'num_bases': 20, 'width_m': 10.5})
        meta = {'basis_hash': bhash, 'bs_id': 1, 'run_count': 42}

        # 1) 未保存は None
        if store.load(1) is not None:
            failures.append("未保存の load が None でない")

        # 2) 往復一致 (忘却なし)
        store.save(1, alpha, P, S2, W, meta)
        st = store.load(1, expected_basis_hash=bhash)
        if not (np.allclose(st['alpha'], alpha) and np.allclose(st['P'], P)
                and np.allclose(st['S2'], S2) and np.allclose(st['W'], W)):
            failures.append("往復で配列が一致しない")
        if st['meta']['run_count'] != 42 or st['meta']['bs_id'] != 1:
            failures.append(f"meta が保存されていない: {st['meta']}")

        # 3) 忘却の適用 (P += q·I, S2·γ, W·γ)
        st2 = store.load(1, q_forget=0.05, gamma=0.997)
        if not np.allclose(st2['P'], P + 0.05 * np.eye(p)):
            failures.append("q_forget が P に適用されていない")
        if not (np.allclose(st2['S2'], S2 * 0.997) and np.allclose(st2['W'], W * 0.997)):
            failures.append("γ が S2/W に適用されていない")
        ratio_before = S2 / np.maximum(W, 1e-12)
        ratio_after = st2['S2'] / np.maximum(st2['W'], 1e-12)
        if not np.allclose(ratio_before, ratio_after):
            failures.append("忘却で σ_ν²=S2/W が変化した (不変であるべき)")

        # 4) basis ハッシュ不一致はエラー
        try:
            store.load(1, expected_basis_hash=basis_hash({'type': 'constant'}))
            failures.append("basis 不一致の load がエラーにならない")
        except ValueError:
            pass

        # 5) 上書き保存 (定期スナップショットの模擬) と meta 必須チェック
        store.save(1, alpha * 2.0, P, S2, W, meta)
        st3 = store.load(1)
        if not np.allclose(st3['alpha'], alpha * 2.0):
            failures.append("上書き保存が反映されない")
        leftovers = [f for f in os.listdir(store.state_dir) if f.endswith('.tmp')]
        if leftovers:
            failures.append(f"一時ファイルが残留: {leftovers}")
        try:
            store.save(2, alpha, P, S2, W, {'bs_id': 2})
            failures.append("basis_hash なしの save がエラーにならない")
        except ValueError:
            pass

        # 6) BS 別ファイルの分離
        store.save(0, alpha + 1.0, P, S2, W, dict(meta, bs_id=0))
        if not np.allclose(store.load(0)['alpha'], alpha + 1.0):
            failures.append("bs0 の状態が独立に保存されていない")
        if not np.allclose(store.load(1)['alpha'], alpha * 2.0):
            failures.append("bs0 の保存が bs1 を汚染した")

        print(f"  state_dir: {sorted(os.listdir(store.state_dir))}")
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    if failures:
        print("FAIL:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("PASS: rem_store_test 全項目合格")


if __name__ == '__main__':
    main()
