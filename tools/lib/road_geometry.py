"""
道路断面と RSU/車載アンテナ幾何の純関数群 (仕様 docs/urban_2lane_scenario_spec.md §1)。

責務: 幾何パラメータ → 座標・角度の計算のみ。
シナリオ辞書の組み立て・yaml 入出力・交通生成は呼び出し側の責務であり、
本モジュールは ROS・Gazebo・yaml を一切知らない (stdlib のみ、単体テスト可能)。

座標系:
  x = 道路軸。**下り = +x**。
  y = 横方向。**RSU は y > 0 側の路側 (歩道の外側端)** に設置する。
  近車線 = RSU 側 = 下り (direction +1)、奥車線 = 反対側 = 上り (direction −1)。

角度規約 (既存 gen_road_scenario.py を踏襲):
  RSU  yaw = −90° + δ_rsu   (車線正対 −y が基準。δ_rsu > 0 = 下流 +x へ振る)
  車   relative yaw = ±δ_veh (車体基準。world +y = RSU 側へ振るため方向で符号反転)
"""
import math


# --- 道路断面 ---------------------------------------------------------------

def lane_layout(lanes_near, lanes_far, lane_width):
    """車線中心の y 座標を返す。

    Returns:
        (near_ys, far_ys) — near は +y 側 (direction +1)、far は −y 側 (direction −1)。
        いずれも中央線 (y=0) に近い順。
    """
    if lanes_near < 0 or lanes_far < 0:
        raise ValueError("車線数は 0 以上")
    near = [(k + 0.5) * lane_width for k in range(lanes_near)]
    far = [-(k + 0.5) * lane_width for k in range(lanes_far)]
    return near, far


def rsu_y_from_section(lanes_near, lane_width, shoulder_m, sidewalk_m):
    """RSU の横位置 = 歩道の外側端 (仕様 §1.1)。

        y_rsu = 近車線の車線帯 + 路肩 + 歩道
    """
    return lanes_near * lane_width + shoulder_m + sidewalk_m


def rsu_positions(num_rsu, spacing_m):
    """RSU の x 座標。原点中心の等間隔配置。"""
    x0 = -spacing_m * (num_rsu - 1) / 2.0
    return [x0 + i * spacing_m for i in range(num_rsu)]


# --- アンテナ角 -------------------------------------------------------------

def rsu_tilts_deg(num_rsu, tilt_deg, pattern='alternate'):
    """RSU ごとの δ_rsu [deg] を返す。半数を上流 (−), 半数を下流 (+) へ振る。

    符号の意味は serving_direction() を参照 — **どちらの走行方向に相互ボアサイトを
    与えるか**を選ぶスイッチであり、単なる空間ダイバーシチではない。

      alternate: [−δ, +δ, −δ, +δ, ...]  上流窓・下流窓を交互に配置
      grouped:   [−δ, −δ, +δ, +δ, ...]  区間前半で上流窓、後半で下流窓
    """
    if pattern == 'alternate':
        return [(-tilt_deg if i % 2 == 0 else tilt_deg) for i in range(num_rsu)]
    if pattern == 'grouped':
        half = num_rsu // 2
        return [(-tilt_deg if i < half else tilt_deg) for i in range(num_rsu)]
    raise ValueError(f"未知の rsu_tilt_pattern: {pattern}")


def rsu_yaw_rad(tilt_deg):
    """RSU の world yaw [rad]。車線正対 (−y, −90°) から下流 (+x) へ tilt_deg 振る。"""
    return math.radians(-90.0 + tilt_deg)


def rsu_boresight_xy(tilt_deg):
    """RSU ボアサイトの水平方向単位ベクトル (x, y)。"""
    yaw = rsu_yaw_rad(tilt_deg)
    return (math.cos(yaw), math.sin(yaw))


def veh_relative_yaw_rad(direction, veh_tilt_deg):
    """車載アンテナの車体基準 relative yaw [rad] (仕様 §1.4)。

    world +y (RSU 側) へ傾けたいが、車体 yaw が走行方向で 0 / π と異なるため
    **符号が反転する**。ここを +δ 固定にすると上り車線のアンテナが
    道路の外側を向く (現行 gen_road_scenario.py の制約)。
    """
    return math.radians(veh_tilt_deg if direction > 0 else -veh_tilt_deg)


def veh_body_yaw_rad(direction):
    """車体 yaw [rad]。下り (+1) は 0、上り (−1) は π。"""
    return 0.0 if direction > 0 else math.pi


def veh_boresight_xy(direction, veh_tilt_deg):
    """車載アンテナ ボアサイトの水平方向単位ベクトル (x, y)。"""
    yaw = veh_body_yaw_rad(direction) + veh_relative_yaw_rad(direction, veh_tilt_deg)
    return (math.cos(yaw), math.sin(yaw))


# --- 相互ボアサイト対向 (仕様 §1.3) -----------------------------------------

def veh_tilt_for_mutual_boresight(rsu_tilt_deg):
    """相互ボアサイト対向が成立する δ_veh。

        δ_rsu + δ_veh = 90°

    現行 gen_road_scenario.py の docstring にある「15°/15° では厳密対向は
    不成立」は、この関係を満たしていなかったことによる。
    """
    return 90.0 - abs(rsu_tilt_deg)


def boresight_meet_dx(lateral_m, rsu_tilt_deg):
    """対向が成立する道路軸上のオフセット |Δx| = L·tan(δ_rsu)。

    Args:
        lateral_m: RSU と対象車線の横距離 L (正)。
    """
    return lateral_m * math.tan(math.radians(abs(rsu_tilt_deg)))


def serving_direction(rsu_tilt_deg):
    """その RSU が相互ボアサイトを与える走行方向 (+1 = 下り / −1 = 上り)。

    幾何から導かれる性質 (仕様 §1.3):

      RSU→車 のベクトルが RSU ボアサイト (sin δ_rsu, −cos δ_rsu) と一致するには
      Δx = x_車 − x_RSU が sin δ_rsu と同符号でなければならない。
      一方 車→RSU が車のボアサイトと一致する条件は、下り車 (+x, 前方右向き) では
      RSU が前方 (Δx < 0 ⇔ x_RSU > x_車) にあることを要求する。
      両立するのは δ_rsu < 0、すなわち **上流へ振った RSU が下り車を捕まえる**。
      上り車は対称に **下流へ振った RSU** と対向する。

    直感: ビームは「これから自分に近づいてくる車」の方を向く。
    """
    return 1 if rsu_tilt_deg < 0 else -1


def mutual_boresight_pose(rsu_x, rsu_y, rsu_tilt_deg, lane_y):
    """その RSU と相互ボアサイトが成立する車の位置 x と、必要な δ_veh を返す。

    Returns:
        (veh_x, veh_tilt_deg, direction)
    """
    lateral = abs(rsu_y - lane_y)
    dx = boresight_meet_dx(lateral, rsu_tilt_deg)
    direction = serving_direction(rsu_tilt_deg)
    # 下り車 (direction +1) は RSU の上流側 (x が小さい側) で対向する
    veh_x = rsu_x - dx if direction > 0 else rsu_x + dx
    return veh_x, veh_tilt_for_mutual_boresight(rsu_tilt_deg), direction


# --- 視線幾何 (仕様 §1.2) ---------------------------------------------------

def los_height_at_y(y_query, rsu_y, rsu_h, veh_y, veh_h):
    """対象車アンテナ → RSU アンテナ の視線が y = y_query を横切る高さ [m]。

    視線は線分パラメータに対して線形で、y も線形に変化するため、**高さは y のみで
    決まり x 方向のずれによらない** (仕様 §1.2)。y_query が両端の外なら外挿値。
    """
    span = rsu_y - veh_y
    if abs(span) < 1e-12:
        raise ValueError("対象車と RSU の y が同一で視線が定義できない")
    t = (y_query - veh_y) / span          # 0 = 車側, 1 = RSU側
    return veh_h + t * (rsu_h - veh_h)


def blocks_los(blocker_h, blocker_y, blocker_width, rsu_y, rsu_h, veh_y, veh_h):
    """遮蔽車が視線を遮るか。車幅の範囲で視線が最も低くなる点で判定する。

    遮蔽車の車幅 [blocker_y − w/2, blocker_y + w/2] のうち、対象車に近い端
    (視線が低い側) が最も遮りやすい。
    """
    y_lo = blocker_y - blocker_width / 2.0
    y_hi = blocker_y + blocker_width / 2.0
    # 対象車に近い端 = 視線が低い端
    y_near = y_lo if abs(y_lo - veh_y) < abs(y_hi - veh_y) else y_hi
    return blocker_h >= los_height_at_y(y_near, rsu_y, rsu_h, veh_y, veh_h)
