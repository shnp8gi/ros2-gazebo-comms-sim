"""
kkf_core: KKF予測ハンドオーバーの純粋数理層。

ROS / gz-transport / protobuf に依存せず numpy のみで動作する。
C++ リファレンス実装 (include/comms_sim_pkg/kkf/*.hpp) と同一仕様であり、
ゴールデンテスト (tools/tests/golden_kkf_test.py) で数値一致を保証する。
"""
from .road_coordinate import RoadCoordinate
from .basis import ConstantBasis, LogDistanceBasis, RbfBasis
from .kkf import KkfParams, Observation, KrigedKalmanFilter
from .planner import solve_handover_plan
from .blockage_tracker import TrackerParams, BlockageTracker
from .scalar_kf import ScalarKfParams, ScalarRssiKF
from .a3 import A3Params, A3Controller
from .variance_map import VarianceMapParams, AleatoricVarianceMap
from .rem_store import RemStore, basis_hash
from .matching import solve_assignment, FORBIDDEN_UTILITY

__all__ = [
    'RoadCoordinate', 'ConstantBasis', 'LogDistanceBasis', 'RbfBasis',
    'KkfParams', 'Observation', 'KrigedKalmanFilter',
    'solve_handover_plan',
    'TrackerParams', 'BlockageTracker',
    'ScalarKfParams', 'ScalarRssiKF',
    'A3Params', 'A3Controller',
    'VarianceMapParams', 'AleatoricVarianceMap',
    'RemStore', 'basis_hash',
    'solve_assignment', 'FORBIDDEN_UTILITY',
]
