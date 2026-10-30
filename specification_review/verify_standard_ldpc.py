#!/usr/bin/env python3
"""ユーザー提供IEEE 802.15.3-2023の検査行列定義を展開し、GF(2)で構造を照合する。
通信コアからは使用しない。符号器/復号器/送信順の適合性を保証するテストではない。
"""
import json
from pathlib import Path


def rank(rows):
    pivots = {}
    for row in rows:
        while row:
            col = row.bit_length()-1
            if col not in pivots:
                pivots[col] = row
                break
            row ^= pivots[col]
    return len(pivots)


def main():
    root = Path(__file__).resolve().parent
    spec = json.loads((root/'ieee_2023_ldpc_definitions.json').read_text())
    result = []
    for code in spec['codes']:
        n,k = code['n'],code['k']
        seeds = code['first_15_columns_one_rows']
        assert len(seeds)==15 and all(len(set(s))==3 for s in seeds)
        rows = [0]*(n-k)
        for j in range(n):
            for seed in seeds[j%15]:
                # 規定式h(i,j)=h(group*96+(i+floor(j/15))mod96,jmod15)を逆に解く。
                i = seed//96*96 + (seed%96-j//15)%96
                rows[i] |= 1<<j
        # 別表現の規定式で全要素を照合。modの符号/行グループ間違いを検出。
        for i in range(n-k):
            for j in range(n):
                expected = i//96*96+(i+j//15)%96 in seeds[j%15]
                assert bool((rows[i]>>j)&1)==expected
        full_rank = rank(rows)
        parity_rank = rank([r>>k for r in rows])
        assert full_rank==n-k and parity_rank==n-k
        result.append(dict(n=n,k=k,rank=full_rank,systematic_parity_rank=parity_rank,
            column_weight=3,nonzero_entries=sum(r.bit_count() for r in rows),
            equation_crosscheck=True,status='matrix_structure_only_not_decoder_validation'))
    (root/'ieee_2023_ldpc_matrix_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
