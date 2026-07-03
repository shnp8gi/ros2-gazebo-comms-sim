#!/bin/bash
# 3段階のスイープテストを順次実行するスクリプト

echo "=== 小規模テスト開始 (Smoke Test: 2条件) ==="
python3 tools/sweep_sim.py --sweep-config config/sweep/test_small.yaml

echo "=== 中規模テスト開始 (Functional Test: 10条件) ==="
python3 tools/sweep_sim.py --sweep-config config/sweep/test_medium.yaml

echo "=== 大規模テスト開始 (Robustness Test: 42条件) ==="
python3 tools/sweep_sim.py --sweep-config config/sweep/test_large.yaml

echo "=== 全スイープテスト完了 ==="
