"""汎用回転ユーティリティ (C++ Utils.hpp の rpy_to_rotmat と同一定義)。"""
import numpy as np


def rpy_to_rotmat(roll, pitch, yaw):
    """RPY (Z-Y-X 順の外因性回転, R = Rz·Ry·Rx) から回転行列を生成する。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx
