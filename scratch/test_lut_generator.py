import sys
import os

# ワークスペースパスを追加
ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'comms_sim_pkg'))
sys.path.insert(0, ws_root)

from comms_sim_pkg.lut_generator import PairAllocationOptimizer

def test_pair_allocation_optimizer():
    print("Testing PairAllocationOptimizer...")

    # Dummy data setup
    # 2 vehicles, 3 rx_nodes
    vehicle_antennas = [{'name': 'v1'}, {'name': 'v2'}]
    rx_nodes = [{'name': 'r0'}, {'name': 'r1'}, {'name': 'r2'}]
    num_pairs = 2

    # rssi_matrix[tx_idx][rx_idx]
    # At t=0, let's say r0 and r1 are best for v1 and v2
    rssi_matrix_t0 = [
        [-50.0, -80.0, -90.0], # tx0 (v1): r0 is best (-50), r1 is bad
        [-80.0, -55.0, -90.0], # tx1 (v2): r1 is best (-55), r0 is bad
    ]

    best_perm_t0, best_pairs_t0, max_rssi_t0 = PairAllocationOptimizer.get_best_allocation(
        rssi_matrix=rssi_matrix_t0,
        num_pairs=num_pairs,
        vehicle_antennas=vehicle_antennas,
        rx_nodes=rx_nodes,
        prev_perm=None,
        ff_hysteresis_margin_db=2.0
    )

    print(f"t0: best_perm={best_perm_t0}, max_rssi={max_rssi_t0}")
    # permutations generates tuples. Since num_rx=3 and num_pairs=2, 
    # it generates (0,1,2), (0,2,1), ... wait, permutations(range(num_rx)) generates length num_rx=3
    # Wait! If num_pairs = 2, permutations(range(3)) returns (0,1,2), (0,2,1), (1,0,2)...
    # The code uses `perm[tx_idx]` for tx_idx in range(num_pairs).
    # So for `perm = (0, 1, 2)`, `tx_idx` goes 0, 1. It uses `perm[0]=0`, `perm[1]=1`.
    # Therefore best_perm is a 3-element tuple, e.g., (0, 1, 2) or (0, 1, X).
    assert best_perm_t0[0] == 0 and best_perm_t0[1] == 1
    assert max_rssi_t0 == -50.0

    # At t=1, RSSI changes slightly, but not enough to overcome hysteresis.
    rssi_matrix_t1 = [
        [-50.0, -49.0, -90.0], 
        [-80.0, -55.0, -55.0], 
    ]

    best_perm_t1_no_hysteresis, _, _ = PairAllocationOptimizer.get_best_allocation(
        rssi_matrix=rssi_matrix_t1,
        num_pairs=num_pairs,
        vehicle_antennas=vehicle_antennas,
        rx_nodes=rx_nodes,
        prev_perm=best_perm_t0,
        ff_hysteresis_margin_db=0.0
    )
    print(f"t1 (no hysteresis): best_perm={best_perm_t1_no_hysteresis}")
    assert best_perm_t1_no_hysteresis[0] == 1 and best_perm_t1_no_hysteresis[1] == 2

    best_perm_t1_with_hysteresis, _, _ = PairAllocationOptimizer.get_best_allocation(
        rssi_matrix=rssi_matrix_t1,
        num_pairs=num_pairs,
        vehicle_antennas=vehicle_antennas,
        rx_nodes=rx_nodes,
        prev_perm=best_perm_t0,
        ff_hysteresis_margin_db=2.0
    )
    print(f"t1 (with hysteresis): best_perm={best_perm_t1_with_hysteresis}")
    assert best_perm_t1_with_hysteresis == best_perm_t0 # Should NOT switch

    # At t=2, the new pair (1, 2) is significantly better (>2dB difference)
    rssi_matrix_t2 = [
        [-50.0, -45.0, -90.0],
        [-80.0, -55.0, -55.0],
    ]
    
    best_perm_t2_with_hysteresis, _, _ = PairAllocationOptimizer.get_best_allocation(
        rssi_matrix=rssi_matrix_t2,
        num_pairs=num_pairs,
        vehicle_antennas=vehicle_antennas,
        rx_nodes=rx_nodes,
        prev_perm=best_perm_t0,
        ff_hysteresis_margin_db=2.0
    )
    print(f"t2 (with hysteresis): best_perm={best_perm_t2_with_hysteresis}")
    assert best_perm_t2_with_hysteresis[0] == 1 and best_perm_t2_with_hysteresis[1] == 2

    print("All tests passed successfully!")

if __name__ == '__main__':
    test_pair_allocation_optimizer()
