import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_interfaces.srv import Cargo, GetTargetPose
from robocup_pkg.srv import ArmCommand
from std_srvs.srv import Trigger
import rbpodo as rb
import numpy as np
import time
import threading
import re


DEFAULT_ROBOT_IP = "10.0.2.8"

HOME_JOINT_DEG        = np.array([-90.0,  0.0,   90.0,  0.0, 90.0,  0.0])
MOVING_JOINT_DEG      = np.array([-90.0, -26.02, 140.8, 0.0, 65.22, 0.0])
VISION_LOAD_JOINT_DEG = np.array([-90.0, 13.70, 69.94, 0.0, 96.36, 0.0])

# 아이스크림/빅트리 조립 중 재료 픽업<->조립위치<->cap_slot 등 직행 이동마다 거쳐가는
# 경유 조인트. 안전한 중간 자세 하나만 거쳐서 충돌 위험을 줄인다.
ASSEMBLY_TRANSIT_JOINT_DEG = np.array([-238.13, 9.23, 31.32, 5.79, 116.41, -9.49])
# np.array([-90.0, 13.82, 83.37, 0.0, 82.82, 0.0]), z=280.28
# np.array([-90.0, 13.43, 80.49, 0.0, 86.09, 0.0]), z=301.28
# np.array([-90.0, 13.28,  75.45, 0.0, 91.27, 0.0]), z=331.28 main 학교
# np.array([-90.0, 13.70, 69.94, 0.0, 96.36, 0.0]), z=361.28 대회장
# np.array([-90.0, 14.26, 66.11, 0.0, 99.62, 0.0]),  z=381.28

# 슬롯 2~8 공통 경유점 (슬롯 1은 경로가 달라 별도 관리)
SLOT_COMMON_WPS = [
    np.array([-90.0,   -20.81,  107.71, 0.0,  93.11,  0.0]),
    np.array([-160.21, -8.27,  125.95, 0.46,  60.24,  0.0]),
    np.array([-220.0,  -11.96,   47.40, 0.0, 100.40,  0.0]),
]

# LOAD 시 슬롯별 최종 접근 위치 (슬롯 2~8)
LOAD_SLOT_JOINTS = {
    2: np.array([-267.47,   8.45, 34.61,  -1.11, 113.49,  1.83]),
    3: np.array([-252.14,  11.15, 31.41,  -7.71, 114.77, 13.17]),
    4: np.array([-239.48,  20.95, 17.82, -13.53, 120.01, 21.54]),
    5: np.array([-225.70,  -3.09, 48.50, -18.08, 116.14, 33.65]),
    6: np.array([-242.58, -10.98, 55.32, -11.60, 114.11, 20.65]),
    7: np.array([-284.66,   1.20, 43.65,   0.0,  135.15, -14.65]),
    8: np.array([-304.82,  11.95, 31.44,   0.0,  136.61, -34.83]),
}

# 슬롯별 웨이포인트: 슬롯 1은 독립 경로, 슬롯 2~8은 공통 경유점 + 슬롯별 최종 위치
SLOT_WAYPOINTS = {
    1: [
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-15.0, -36.42, 117.55, 0.0, 98.86, 0.0]),
        np.array([35.0, 15.0, 23.0, 0.0, 100.0, 0.0]),
        np.array([73.71, 31.20, 13.67, -2.18, 139.91, -17.89]),
    ],
    **{slot: SLOT_COMMON_WPS + [joint] for slot, joint in LOAD_SLOT_JOINTS.items()}
}

# 슬롯 7, 8 언로드 전용 웨이포인트 (로드 경로와 다름)
UNLOAD_SLOT_WAYPOINTS = {
    7: [
        np.array([-90.0,  -20.81, 107.71,   0.0,  93.11,   0.0]),
        np.array([-15.0,  -36.42, 117.55,   0.0,  98.86,   0.0]),
        np.array([ 89.71, -21.9,   43.02, -12.68, 116.79,  11.67]),
        np.array([156.75, -85.1,  122.9,  -71.25,  76.01,  35.48]),
    ],
    8: [
        np.array([-90.0,  -20.81, 107.71,   0.0,  93.11,   0.0]),
        np.array([-15.0,  -36.42, 117.55,   0.0,  98.86,   0.0]),
        np.array([-30.34, -34.83, 116.39,  68.02,  96.42,  73.61]),
        np.array([ -5.72, -34.3,  116.0,   90.83,  95.6,   98.3]),
    ],
}
# 인덱스 0~5: 내려놓는 순서에 따라 사용 (unload 전용)
DELIVERY_WAYPOINTS = {
    0: [
        np.array([-106.29, 23.23, 96.43, 0.0, 60.35, -16.28]),
    ],
    1: [
        np.array([-91.40, 20.078, 100.7, 0.0, 59.22, -1.39]),
    ],
    2: [
        np.array([-75.10, 22.13, 97.94, 0.0, 59.94, 14.91]),
    ],
    3: [
        np.array([-78.43, 43.34, 66.28, 0.0, 70.38, 11.58]),
    ],
    4: [
        np.array([-90.71, 41.46, 69.26, 0.0, 69.28, -0.71]),
    ],
    5: [
        np.array([-103.28, 44.63, 64.19, 0.0, 71.19, -13.28]),
    ],
    # 6번: 슬롯1 전용 내려놓기 포인트.
    #      완성품 unload 는 delivery_idx 와 무관하게 무조건 이 포인트로 간다.
    6: [
        np.array([-71.93, 28.31, 129.97, 19.60, -62.59, -4.07]),
    ],
}

# 완성품 층 그룹 — delivery 시 내려놓는 높이(z)가 그룹별로 다르다.
PRODUCT_FLOOR_1   = {34, 13, 81}               # 1층:   배터리, 마그넷, 이스탑
PRODUCT_FLOOR_2_5 = {442, 241, 462, 8518}      # 2층반: 당근, 신호등, 스몰트리, 버거
PRODUCT_FLOOR_2   = {711, 4482, 48132, 46262}  # 2층:   망치, 큰당근, 아이스크림, 빅트리

# 완성품(Products) ID 묶음 = 위 3개 층 그룹의 합집합.
# 이 집합에 속하면 unload 시 완성품 전용 경로를 탄다.
# (raw material 1~8 은 포함하지 않음 -> 기존 DELIVERY_WAYPOINTS 흐름 그대로)
FINISHED_PRODUCTS = PRODUCT_FLOOR_1 | PRODUCT_FLOOR_2_5 | PRODUCT_FLOOR_2

# 완성품 unload 시 사용할 고정 delivery 인덱스.
# 완성품은 원래 있던 슬롯(1이든 7/8이든)과 무관하게 항상 DELIVERY_WAYPOINTS[6] 으로 간다.
PRODUCT_DELIVERY_IDX = 6

# 워크벤치 스테이션: 이 station_id로 unload가 들어오면 sequence_unload_multi가 세어주는
# 배치 내 순번(0~5)으로 고정 웨이포인트에 순서대로 내려놓는다. cargo_manager는 이 값을
# 모르며 관여하지 않는다 — 그 외 스테이션(고객센터 등)은 비전(666)으로 실시간 빈 공간을
# 찾아 그 좌표에 내려놓는다.
WORKBENCH_STATION_IDS = {4, 10}

# UNLOAD 시 빈 공간 탐지용 비전 target_id. 물체 인식(object_id)과 달리
# "빈 공간 좌표를 달라"는 의미의 고정 ID (vision_pkg 쪽에 이미 구현되어 있음).
DELIVERY_EMPTY_SPACE_VISION_ID = "666"

# 비전(666)이 준 z값 보정용 오프셋(mm). LOAD 픽업면과 딜리버리 바닥면이 달라서
# 필요한 여유값 — 전체 z값에서 이만큼 뺀다.
DELIVERY_VISION_Z_OFFSET_MM = 20.0

# 조립 슬롯. target_slot은 더 이상 요청자(station_id 등)가 지정하지 않고,
# cargo_manager가 배정해준 슬롯을 그대로 쓴다. 우선순위 확인용으로만 순서를 들고 있는다
# (실제 "어느 슬롯을 줄지"는 cargo_manager 쪽 FIND_EMPTY_ASSEMBLY_SLOT이 결정한다).
ASSEMBLY_SLOTS = [7, 8]

# --- LOAD 비전/오프셋 상수 ---
CAM_X_OFF = -51.0
CAM_Y_OFF = 32.0
LOAD_Z_DOWN_MM = 55.0
LOAD_Z_UP_MM = -55.0
Z_OFFSET = -85.0
Z_MARGIN = 40.0
SCAN_Y_OFFSETS_MM = [0.0, 200.0, -200.0]
SCAN_Y_AXIS_INDEX = 1
SCAN_SETTLE_TIME_SEC = 0.3
SCAN_VISION_RETRIES_PER_POSE = 1
SCAN_MAX_CYCLES = 3
# 파지(grip) 직전, 이동 정지 후 기계 진동이 잦아들 시간(초). 최소값으로 잡음.
# 0 에 가까울수록 빠르지만, 흔들리는 중에 잡으면 파지 실패 위험 -> 0.05~0.1 권장.
GRIP_SETTLE_TIME_SEC = 0.1

# --- 파지 pos 기반 장축/단축 판정 (LOAD 전용) ---
# grip 완료 후 그리퍼 pos 값으로 어느 축을 잡았는지 판정한다.
# 주의: 장축 구간(357~358)이 오파지 구간(300~400) 안에 겹쳐 있으므로,
#       classify_grip_pos() 에서 반드시 장축 구간을 먼저 체크한다.
GRIP_POS_LONG_AXIS_MIN = 350    # 장축을 잡았을 때 pos 범위
GRIP_POS_LONG_AXIS_MAX = 360
GRIP_POS_SHORT_AXIS_MIN = 500   # 단축을 잡았을 때 pos 범위 (정상)
GRIP_POS_SHORT_AXIS_MAX = 600
GRIP_POS_FAIL_MIN = 300         # 잘못 잡힌(오파지) pos 범위
GRIP_POS_FAIL_MAX = 400

# 장축 판정 시 재파지를 위해 J6 를 회전시키는 각도(deg). 우선 +90 고정.
GRIP_REORIENT_J6_DEG = 90.0

# 오파지(fail) 판정 시 비전부터 다시 시도하는 최대 횟수.
MAX_LOAD_GRIP_ATTEMPTS = 3


def classify_grip_pos(pos):
    """grip 후 pos 값으로 'long' / 'short' / 'fail' / 'unknown' 을 반환한다."""
    if pos is None:
        return 'unknown'
    if GRIP_POS_LONG_AXIS_MIN <= pos <= GRIP_POS_LONG_AXIS_MAX:
        return 'long'
    if GRIP_POS_SHORT_AXIS_MIN <= pos <= GRIP_POS_SHORT_AXIS_MAX:
        return 'short'
    if GRIP_POS_FAIL_MIN <= pos <= GRIP_POS_FAIL_MAX:
        return 'fail'
    return 'unknown'

# # --- LOAD yaw(rz) 보정 상수 ---
# # 특정 완성품은 파지 방향을 맞추기 위해 비전 yaw 에 고정 오프셋(deg)을 더한다.
# # e_stop(81), burger(8518), big_tree(46262) -> -90도
# # 여기 없는 object_id 는 오프셋 0 (비전 yaw 그대로 사용).
# YAW_OFFSET_DEG = {
#     81: -90.0,     # e_stop
#     8518: -90.0,   # burger
#     46262: 0.0,  # big_tree
# }

# --- 제품별 파지 오프셋 (LOAD 전용) ---
# YAW_OFFSET_DEG 와 같은 방식으로 object_id 별로 파지 보정을 따로 준다.
# 비전이 준 좌표 위에 "더해지는" 추가 보정값. 전역 CAM/Z 오프셋은 그대로 두고
# 제품마다 미세 보정만 얹는다. (yaw 가 p.yaw + offset 으로 더해지던 것과 동일 패턴)
#
#   x   : 최종 접근 move_l_rel 의 tool x 병진에 더할 값 (mm)
#   y   : 최종 접근 move_l_rel 의 tool y 병진에 더할 값 (mm)
#   z   : 파지 깊이(tool z)에 더할 값 (mm, +면 더 깊이)
#   yaw : 파지 회전(rz)에 더할 값 (deg)
#
# 생략한 키는 0. 테이블에 없는 object_id 도 전부 0.
PICK_OFFSET_DEFAULT = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

PICK_OFFSET = {
    # --- Raw Materials ---
    1: {},  # 2x2_red
    2: {},  # 2x2_green
    3: {},  # 2x2_blue
    4: {},  # 2x2_yellow
    5: {'yaw': -90.0},  # 4x2_red
    6: {'yaw': -90.0},  # 4x2_green
    7: {'yaw': -90.0},  # 4x2_blue
    8: {'yaw': -90.0},  # 4x2_yellow
    # --- Products ---
    34:    {},               # battery
    13:    {},               # magnet
    81:    {'z': 15.0},   # e_stop
    442:   {},               # carrot
    241:   {},               # traffic_light
    462:   {'z': 20.0},               # small_tree
    711:   {'x': -25.0, 'z': 20.0},     # hammer 로봇베이스 기준 안쪽은 x+ 
    4482:  {'x': -20.0, 'z': 25.0},               # big_carrot
    8518:  {'y': -3.0, 'z': 30.0},   # burger
    48132: {'z': 25.0},               # ice_cream
    46262: {'z': 30.0},      # big_tree 벅서 빅트리 회전제한 -90~90
}


def get_pick_offset(object_id):
    """object_id 의 파지 오프셋을 (없는 키는 0 으로 채워) 반환한다."""
    off = dict(PICK_OFFSET_DEFAULT)
    off.update(PICK_OFFSET.get(object_id, {}))
    return off

# --- UNLOAD Z 상수 (슬롯에서 물체 집을 때, 슬롯 2~6) --q   -
UNLOAD_Z_DOWN_MM = 70.0
UNLOAD_Z_UP_MM = -70.0

# --- 슬롯 1 전용 Z 상수 (LOAD/UNLOAD 공통) ---
SLOT1_Z_DOWN_MM = 30.0
SLOT1_Z_UP_MM   = -30.0

# --- UNLOAD X 상수 (슬롯 7/8 언로드 시 Cargo Tool X 방향 이동) ---
UNLOAD_X_DOWN_MM = 90.0
UNLOAD_X_UP_MM = -90.0

# 슬롯별 Tool X 이동 방향 부호 (+1.0 = X+, -1.0 = X-)
UNLOAD_SLOT_X_DIR = {
    7:  1.0,
    8: -1.0,
}

# --- DELIVERY Z 상수 (배달 위치에서 물체 내려놓을 때, 일반 재료 전용 — 워크벤치,
#     비전 없이 고정 웨이포인트로 내려놓는 경로에서만 쓴다) ---
DELIVERY_Z_MM = 165.0 #학교 115 #대회 165


# --- 완성품(Products) 전용 delivery Z 상수 ---
# 완성품은 6번 포인트에서 손목이 꺾여 Tool z축이 수직이 아니므로,
# delivery 내려놓기는 Base 프레임(중력 방향) 기준으로 수행한다.
# point 6에서 내려놓을 때 층 그룹과 무관하게 항상 이 값만큼 내려갔다 올라온다.
# 비전으로 x,y 보정을 거치면 실제 도착 위치가 미세하게 달라질 수 있어서,
# 워크벤치(비전 없이 point 6 직행)와 그 외 스테이션(비전 보정)의 z를 따로 둔다.
PRODUCT_DELIVERY_FIXED_Z_MM = 240.0   # 워크벤치 (비전 없음)
PRODUCT_DELIVERY_VISION_Z_MM = 225.0  # 그 외 스테이션 (비전 x,y 보정)


# --- 모션 속도/가속 (이 4개 숫자가 로봇팔 속도를 전부 결정한다) ---
# move_j : J_VEL=deg/s,  J_ACC=deg/s^2   (관절 이동: 슬롯/홈/배달 등 긴 이동)
# move_l : L_VEL=mm/s,   L_ACC=mm/s^2    (직선 이동: 비전 접근/하강/상승 등)
#
# 값이 클수록 빠르다. 아래는 "빠르게" 세팅이다.
# 만약 너무 거칠거나(덜컹/오버슈트) 물건을 놓치면 숫자를 낮추면 된다.
#   - 더 빠르게 : J 500/1200, L 800/2000   (관절은 로봇 물리 한계에서 멈춤)
#   - 빠르게    : J 400/1000, L 700/1500    <- 현재값
#   - 보통      : J 300/600,  L 500/1000
#   - 느리게    : J 200/400,  L 400/800
# (가속 J_ACC 가 짧은 이동 속도를 가장 크게 좌우한다. 더 빠르게 하려면 J_ACC 먼저 올릴 것)
J_VEL, J_ACC = 400, 1000
L_VEL, L_ACC = 700, 1500

# --- 조립(ASSEMBLE) 상수 ---
# ASSEMBLY_WAYPOINTS: target_slot별 조립 위치까지의 경유 조인트 목록
#   - 딕셔너리 키 = target_slot (7 또는 8)
#   - 마지막 요소가 실제 조립 위치 (= ASSEMBLY_JOINT[target_slot])
#   - sequence_assemble 에서 순서대로 move_j 로 이동
ASSEMBLY_Z_DOWN_MM = 70.0   # layer 0 기준 블록 내려놓기 하강 거리 (mm)
ASSEMBLY_Z_UP_MM   = -70.0  # layer 0 기준 블록 내려놓기 상승 거리 (mm)
BLOCK_H_MM         = 18.0   # 블록 1개 높이 (mm)

# --- OLD BigTree (46262) 전용 조립 상수: 재료슬롯 2-6 내부 조립 버전 비활성화 ---
# BIG_TREE_STEP2_Z_OFFSET_MM = 19.0   # slot_x layer2: 다른 슬롯의 6을 결합 (70-19=51mm)
# BIG_TREE_STEP3_Z_OFFSET_MM = 38.0   # slot_x layer3: 다른 슬롯의 2를 결합, 그리퍼 유지 (70-38=32mm)
# BIG_TREE_FINAL_Z_OFFSET_MM = 47.0   # y0 layer0: 완성 스택을 4 위에 최종 결합 (70-47=23mm)

# --- BigTree (46262) 전용 조립 상수: slot 7/8 고정 조립 버전 ---
BIG_TREE_BASE_SLOT = 7
BIG_TREE_STAGE_SLOT = 8

BIG_TREE_BASE_X_MM = 0.0
BIG_TREE_STAGE_LEFT_X_MM = -32.0
BIG_TREE_STAGE_RIGHT_X_MM = 16.5
BIG_TREE_STAGE_MID_X_MM = 0.0
BIG_TREE_STAGE_PICKUP_X_MM = -16.5
BIG_TREE_MERGE_PLACE_X_MM = 0.0
BIG_TREE_TOP_2_X_MM = 0.0

BIG_TREE_STAGE_LAYER_TOP = 0.9

BIG_TREE_STAGE_PICKUP_Z_DOWN_MM = 51.0
BIG_TREE_STAGE_PICKUP_Z_UP_MM = -51.0
BIG_TREE_STAGE_MERGE_Z_DOWN_MM = 52.0
BIG_TREE_STAGE_MERGE_Z_UP_MM = -52.0
BIG_TREE_TOP_2_Z_DOWN_MM = 16.0
BIG_TREE_TOP_2_Z_UP_MM = -16.0

BIG_TREE_STAGE_GROUP_IDS = (6, 2, 6)
BIG_TREE_FINAL_STACK_IDS = (4, 6, 2, 6, 2)

# --- IceCream(48132) / BigTree(46262) 재료슬롯(2-6) 스테이징 조립 공통 ---
# 재료슬롯(2-6)에 임시로 쌓아둔 재료끼리 결합할 때, 그 슬롯에 실제로 몇 개가
# 쌓여있는지와 무관하게 항상 정해진 layer_index(=UNLOAD_SLOT_JOINTS의 근처 고정 조인트)로
# 접근한 뒤, ASSEMBLY_Z_DOWN_MM(70mm) 기준에서 아래 오프셋만큼 덜 내려가 정확히 접촉시킨다.
STAGING_MATERIAL_SLOTS = [2, 3, 4, 5, 6]

# 아이스크림 캡(3+1+2) 조립: 캡 스테이징 슬롯의 layer_index=1 위치에서 2(초록)를 결합
ICE_CREAM_CAP_LAYER_INDEX = 1
ICE_CREAM_CAP_Z_OFFSET_MM = 19.0   # z_down = ASSEMBLY_Z_DOWN_MM(70) - 19 = 51mm

# UNLOAD 픽업용 슬롯 조인트 (direct move_j, 중간 웨이포인트 없음)
# 키: slot 번호 (슬롯 2~6)
UNLOAD_SLOT_JOINTS = {
    20: np.array([-266.24, -9.60, 56.23, -1.59, 109.94, 2.91]),  # ref: [-266.24, -9.60, 56.23, -1.59, 109.94, 2.91]
    21: np.array([-266.49, -5.88, 51.62, -1.50, 110.83, 2.69]),  # ref: [-266.49, -5.88, 51.62, -1.49, 110.83, 2.69]
    22: np.array([-266.72, -1.93, 46.49, -1.41, 112.00, 2.49]),  # ref: [-266.72, -1.93, 46.49, -1.41, 112.01, 2.49]
    23: np.array([-266.91, 2.35, 40.65, -1.35, 113.55, 2.30]),  # ref: [-266.91, 2.35, 40.65, -1.35, 113.55, 2.30]
    24: np.array([-267.08, 7.17, 33.75, -1.29, 115.63, 2.12]),  # ref: [-267.08, 7.17, 33.75, -1.29, 115.63, 2.12]
    30: np.array([-246.77, -6.53, 53.50, -9.69, 111.00, 18.00]),  # ref: [-246.77, -6.53, 53.50, -9.69, 111.00, 18.00]
    31: np.array([-248.19, -2.84, 48.70, -9.18, 111.93, 16.71]),  # ref: [-248.19, -2.84, 48.70, -9.18, 111.93, 16.71]
    32: np.array([-249.46, 1.14, 43.29, -8.75, 113.20, 15.51]),  # ref: [-249.46, 1.14, 43.30, -8.75, 113.20, 15.51]
    33: np.array([-250.59, 5.55, 37.04, -8.40, 114.91, 14.36]),  # ref: [-250.59, 5.55, 37.04, -8.40, 114.91, 14.36]
    34: np.array([-251.61, 10.68, 29.39, -8.14, 117.30, 13.22]),  # ref: [-251.61, 10.68, 29.39, -8.14, 117.30, 13.22]
    40: np.array([-231.15, 0.67, 46.72, -15.76, 113.13, 30.14]),  # ref: [-231.15, 0.67, 46.72, -15.77, 113.13, 30.14]
    41: np.array([-233.11, 4.40, 41.34, -15.21, 114.33, 28.16]),  # ref: [-233.11, 4.40, 41.34, -15.21, 114.33, 28.16]
    42: np.array([-234.91, 8.62, 35.05, -14.76, 116.00, 26.21]),  # ref: [-234.91, 8.62, 35.05, -14.76, 116.01, 26.21]
    43: np.array([-236.56, 13.65, 27.25, -14.45, 118.41, 24.22]),  # ref: [-236.56, 13.65, 27.25, -14.45, 118.41, 24.22]
    44: np.array([-238.08, 20.63, 15.83, -14.46, 122.46, 21.87]),  # ref: [-238.08, 20.63, 15.83, -14.46, 122.46, 21.87]
    50: np.array([-211.52, -12.81, 59.52, -21.41, 119.66, 45.58]),  # ref: [-209.45, -14.62, 62.23, -23.19, 118.28, 46.92]
    51: np.array([-215.23, -10.37, 56.28, -20.43, 119.45, 42.37]),  # ref: [-213.31, -12.27, 59.05, -22.13, 117.95, 43.60]
    52: np.array([-218.62, -7.71, 52.72, -19.51, 119.47, 39.40]),  # ref: [-216.85, -9.70, 55.58, -21.13, 117.85, 40.53]
    53: np.array([-221.71, -4.82, 48.78, -18.66, 119.75, 36.65]),  # ref: [-220.08, -6.91, 51.78, -20.19, 117.99, 37.69]
    54: np.array([-224.50, -1.68, 44.44, -17.91, 120.29, 34.12]),  # ref: [-223.02, -3.87, 47.56, -19.33, 118.40, 35.04]
    60: np.array([-226.84, -23.79, 67.91, -17.83, 117.14, 32.37]),  # ref: [-226.84, -23.79, 67.91, -17.83, 117.14, 32.37]
    61: np.array([-231.08, -21.03, 64.89, -16.24, 116.54, 29.12]),  # ref: [-231.08, -21.03, 64.89, -16.24, 116.54, 29.12]
    62: np.array([-234.65, -18.13, 61.62, -14.88, 116.23, 26.36]),  # ref: [-234.66, -18.13, 61.62, -14.88, 116.22, 26.36]
    63: np.array([-237.69, -15.06, 58.07, -13.72, 116.18, 23.98]),  # ref: [-237.69, -15.06, 58.07, -13.72, 116.18, 23.98]
    64: np.array([-240.28, -11.83, 54.19, -12.73, 116.40, 21.91]),  # ref: [-240.28, -11.83, 54.19, -12.73, 116.40, 21.90]
    # 20: np.array([-266.65,-10.78, 60.26, -1.40, 107.07, 2.67]),
    # 21: np.array([-266.87, -7.18, 55.88, -1.31, 107.85, 2.47]),
    # 22: np.array([-267.06, -3.39, 51.06, -1.24, 108.88, 2.3]),
    # 23: np.array([-267.23, 0.65, 45.68, -1.18, 110.22, 2.14]),
    # 24: np.array([-267.38, 5.07, 39.51, -1.13, 111.96, 1.98]),
    # 30: np.array([-246.79, -8.18, 58.19, -9.5, 107.99, 18.52]),
    # 31: np.array([-248.19, -4.65, 53.71, -9.0, 108.77, 17.25]),
    # 32: np.array([-249.43, -0.91, 48.75, -8.56, 109.83, 16.07]),
    # 33: np.array([-250.55, 3.14, 43.16, -8.18, 111.23, 14.97]),
    # 34: np.array([-251.55, 7.64, 36.67, -7.88, 113.10, 13.91]),
    # 40: np.array([-231.78, -1.0, 51.34, -15.23, 110.13, 30.48]),
    # 41: np.array([-233.70, 2.51, 46.39, -14.67, 111.15, 28.58]),
    # 42: np.array([-235.45, 6.37, 40.76, -14.18, 112.53, 26.75]),
    # 43: np.array([-237.06, 10.76, 34.14, -13.78, 114.42, 24.94]),
    # 44: np.array([-238.53, 16.09, 25.78, -13.53, 117.13, 23.05]),
    # 50: np.array([-210.75, -15.79, 65.59, -22.38, 115.86, 46.85]),
    # 51: np.array([-214.49, -13.46, 62.51, -21.35, 115.50, 43.62]),
    # 52: np.array([-217.91, -10.93, 59.16, -20.37, 115.36, 40.63]),
    # 53: np.array([-221.04, -8.21, 55.51, -19.45, 115.44, 37.87]),
    # 54: np.array([-223.89, -5.26, 51.50, -18.61, 115.76, 35.31]),
    # 60: np.array([-228.30, -25.67, 71.33, -16.99, 114.78, 31.97]),
    # 61: np.array([-232.32, -22.33, 68.38, -15.49, 114.16, 28.85]),
    # 62: np.array([-235.12, -19.44, 65.21, -14.21, 113.81, 26.18]),
    # 63: np.array([-238.61, -16.42, 61.78, -13.11, 113.71, 23.89]),
    # 64: np.array([-241.08, -13.25, 58.07, -12.17, 113.86, 21.89]),
}

MATERIAL_NAMES = {
    # --- Raw Materials ---
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
    # --- Products ---
    34: "battery",
    13: "magnet",
    81: "e_stop",
    442: "carrot",
    241: "traffic_light",
    462: "small_tree",
    711: "hammer",
    4482: "big_carrot",
    8518: "burger",
    48132: "ice_cream",
    46262: "big_tree",
}

# 완성품별 조립 재료 시퀀스.
# 리스트 인덱스 = layer_index (0부터 시작).
# 값 = 재료 object_id.
# layer 0: ASSEMBLY_Z_DOWN_MM 그대로 하강.
# layer N: ASSEMBLY_Z_DOWN_MM - (BLOCK_H_MM * N) 만큼 하강.
ASSEMBLY_SEQUENCE = {
    34:   [3, 4],        # battery:       2x2파랑 → 2x2노랑
    13:   [1, 3],        # magnet:        2x2빨강 → 2x2파랑
    81:   [8, 1],        # e_stop:        4x2노랑 → 2x2빨강
    442:  [4, 4, 2],     # carrot:        2x2노랑 → 2x2노랑 → 2x2초록
    241:  [2, 4, 1],     # traffic_light: 2x2초록 → 2x2노랑 → 2x2빨강
    462:  [4, 6, 2],     # small_tree:    2x2노랑 → 4x2초록 → 2x2초록
    711:  [1, 1, 7],     # hammer:        2x2빨강 → 2x2빨강 → 2x2파랑
    4482: [4, 4, 8, 2],  # big_carrot:    2x2노랑 → 2x2노랑 → 4x2노랑 → 2x2초록
    # dict 형식: {'id': 재료id, 'layer': 높이레이어, 'x': Tool X 오프셋(mm)}
    # 같은 layer 값이 여러 번 나오면 같은 높이에서 x 위치만 달리해 배치한다.
    # 48132(ice_cream)는 재료슬롯 스테이징 방식(sequence_assemble_ice_cream)으로 대체되어
    # 여기서 제거함 — sequence_assemble()에서 46262처럼 별도 분기로 처리한다.
    8518: [   # burger
        {'id': 8, 'layer': 0, 'x':  0.0},
        {'id': 5, 'layer': 1, 'x': -17},
        {'id': 1, 'layer': 1, 'x':  32.0},
        {'id': 8, 'layer': 2, 'x': -1.0},
    ],
}


class AmrRobotNode(Node):
    """load / unload 통합 오케스트레이터.

    /arm_command 서비스 하나로 LOAD / UNLOAD 를 모두 처리한다.
    request.action 이 'LOAD' 이면 적재 시퀀스, 'UNLOAD' 이면 출고 시퀀스를 돈다.
    로봇 연결·busy 락·서비스는 모두 단일 인스턴스로 공유한다.
    """

    def __init__(self):
        super().__init__('amr_robot_node')
        self.cbg = ReentrantCallbackGroup()
        self.declare_parameter('robot_ip', DEFAULT_ROBOT_IP)
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.robot = None
        self.rc = None
        self.robot_data = None
        self.robot_ready = False

        try:
            self.robot = rb.Cobot(self.robot_ip)
            self.rc = rb.ResponseCollector()
            self.robot.set_operation_mode(self.rc, rb.OperationMode.Real)
            self.robot.set_speed_bar(self.rc, 0.8)
            self.robot.set_speed_multiplier(self.rc, 1.5)
            self.robot_ready = True
            self.get_logger().info(f'[AMR] robot connected: {self.robot_ip}')
        except Exception as e:
            self.robot = None
            self.rc = None
            self.robot_ready = False
            self.get_logger().error(
                f'[AMR] robot connection error ({self.robot_ip}): {e}')

        # 현재 조인트 각도 읽기용 데이터 채널 (HOME 도착 여부 판정에 사용)
        try:
            self.robot_data = rb.CobotData(self.robot_ip)
            self.get_logger().info(f'[AMR] data channel connected: {self.robot_ip}')
        except Exception as e:
            self.robot_data = None
            self.get_logger().warn(
                f'[AMR] data channel connect failed ({self.robot_ip}): {e}')

        self.vision_client = self.create_client(
            GetTargetPose, '/get_target_pose', callback_group=self.cbg)
        self.gripper_open_client = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cbg)
        self.gripper_grip110_client = self.create_client(
            Trigger, '/gripper/grip110', callback_group=self.cbg)
        self.gripper_grip_client = self.create_client(
            Trigger, '/gripper/grip', callback_group=self.cbg)
        self.cargo_client = self.create_client(
            Cargo, '/cargo', callback_group=self.cbg)
        self.srv = self.create_service(
            ArmCommand, '/amr_robot_command', self.arm_robot_command_cb, callback_group=self.cbg)

        self._busy_lock = threading.Lock()
        self._busy = False

        self.get_logger().info('[AMR] amr_robot_node started')

        if self.robot_ready:
            t = threading.Thread(target=self._startup_move, daemon=True)
            t.start()

    # --- 상태 확인 헬퍼 ---

    def is_robot_ready(self):
        if not self.robot_ready or self.robot is None or self.rc is None:
            self.get_logger().error('[AMR] robot is not connected')
            return False
        return True

    def is_at_home(self, tol_deg=1.0):
        """현재 측정 조인트 각도(jnt_ang)를 읽어 HOME과 비교한다.
        데이터 채널이 없거나 읽기 실패 시 False를 반환해, 안전하게 실제 이동으로 폴백한다."""
        if self.robot_data is None:
            return False
        try:
            data = self.robot_data.request_data(1.0)
            cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
            return bool(np.all(np.abs(cur - HOME_JOINT_DEG) <= tol_deg))
        except Exception as e:
            self.get_logger().warn(f'[AMR] is_at_home read failed: {e}')
            return False

    def is_at_moving_pose(self, tol_deg=1.0):
        """현재 조인트가 MOVING_JOINT_DEG와 일치하는지 확인한다."""
        if self.robot_data is None:
            return False
        try:
            data = self.robot_data.request_data(1.0)
            cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
            return bool(np.all(np.abs(cur - MOVING_JOINT_DEG) <= tol_deg))
        except Exception as e:
            self.get_logger().warn(f'[AMR] is_at_moving_pose read failed: {e}')
            return False

    # --- 서비스 호출 헬퍼 ---

    def call_service(self, client, request, timeout=10.0):
        """Call a ROS2 service from inside callbacks without nested spinning.
        This node runs under MultiThreadedExecutor with a ReentrantCallbackGroup.
        The current callback thread waits on an Event, while another executor
        thread processes the service response.
        """
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f'[AMR] service unavailable: {client.srv_name}')
                return None

            future = client.call_async(request)
            done_event = threading.Event()
            future.add_done_callback(lambda _: done_event.set())

            if not done_event.wait(timeout=timeout):
                self.get_logger().error(f'[AMR] service timeout: {client.srv_name}')
                return None

            return future.result()
        except Exception as e:
            self.get_logger().error(f'[AMR] service call failed: {client.srv_name}: {e}')
            return None

    def call_vision(self, target_color, retries=3):
        for i in range(retries):
            req = GetTargetPose.Request()
            req.target_color = target_color
            req.target_size = ""
            res = self.call_service(self.vision_client, req, timeout=30.0)
            if res and res.success:
                return res
            self.get_logger().warn(f'[AMR] vision retry {i + 1}/{retries}')
            time.sleep(0.5)
        return None

    def _scan_y_delta(self, dy_mm):
        delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        delta[SCAN_Y_AXIS_INDEX] = float(dy_mm)
        return delta

    def return_scan_center(self, current_y_offset_mm):
        if abs(current_y_offset_mm) < 1e-6:
            return True
        return self.move_l_rel_checked(
            self._scan_y_delta(-current_y_offset_mm),
            label='scan return center',
        )

    def call_vision_with_y_scan(self, target_color):
        current_y_offset = 0.0

        for cycle in range(SCAN_MAX_CYCLES):
            self.get_logger().info(f'[AMR] vision scan cycle {cycle + 1}/{SCAN_MAX_CYCLES}')
            for target_y_offset in SCAN_Y_OFFSETS_MM:
                delta_y = target_y_offset - current_y_offset
                if abs(delta_y) > 1e-6:
                    if not self.move_l_rel_checked(
                        self._scan_y_delta(delta_y),
                        label=f'scan y offset {target_y_offset:.0f}mm',
                    ):
                        self.get_logger().error(
                            f'[AMR] scan move failed: y={target_y_offset:.0f}mm')
                        self.return_scan_center(current_y_offset)
                        return None
                    current_y_offset = target_y_offset

                time.sleep(SCAN_SETTLE_TIME_SEC)
                self.get_logger().info(
                    f'[AMR] vision scan at y_offset={current_y_offset:.0f}mm')

                res = self.call_vision(
                    target_color,
                    retries=SCAN_VISION_RETRIES_PER_POSE,
                )
                if res:
                    self.get_logger().info(
                        f'[AMR] vision success at y_offset={current_y_offset:.0f}mm')
                    return res

            self.get_logger().warn(f'[AMR] vision scan cycle {cycle + 1} failed')

        self.get_logger().warn('[AMR] vision scan failed at all cycles')
        if not self.return_scan_center(current_y_offset):
            self.get_logger().error('[AMR] failed to return scan center')
        return None

    def call_gripper(self, grip: bool, client=None):
        if client is None:
            client = self.gripper_grip_client if grip else self.gripper_open_client

        req = Trigger.Request()
        res = self.call_service(client, req, timeout=6.0)
        action_name = 'grip' if grip else 'open'
        if res and res.success:
            self.get_logger().info(f'[GRIPPER] {action_name}')
            return True
        self.get_logger().error(f'[GRIPPER] {action_name} failed')
        return False

    @staticmethod
    def _parse_grip_pos(message: str):
        """gripper 서비스 응답 message(예: 'Gripped|pos=357')에서 pos 값을 뽑는다."""
        if not message:
            return None
        m = re.search(r'pos[:=]?\s*(\d+)', message, re.IGNORECASE)
        return int(m.group(1)) if m else None

    def call_gripper_grip_with_pos(self):
        """LOAD 장축/단축 판정용: grip 성공 여부와 pos 값을 함께 반환한다.
        기존 call_gripper()는 다른 호출부(UNLOAD/조립 등)에 영향 없도록 그대로 두고,
        이 메서드는 LOAD 파지 단계에서만 사용한다."""
        req = Trigger.Request()
        res = self.call_service(self.gripper_grip_client, req, timeout=6.0)
        if res and res.success:
            pos = self._parse_grip_pos(res.message)
            self.get_logger().info(f'[GRIPPER] grip pos={pos}')
            return True, pos
        self.get_logger().error('[GRIPPER] grip failed')
        return False, None

    def rotate_j6_checked(self, delta_deg, label='rotate J6'):
        """현재 자세에서 J6(마지막 조인트, index 5)만 delta_deg 만큼 회전시킨다.
        move_l_rel 의 tool rz 회전과 달리 조인트 공간에서 직접 회전시키므로
        wrist pose 에 따른 Tool 프레임 방향 문제와 무관하게 동작한다."""
        if self.robot_data is None:
            self.get_logger().error(f'[AMR] {label}: robot_data unavailable')
            return False
        try:
            data = self.robot_data.request_data(1.0)
            cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
        except Exception as e:
            self.get_logger().error(f'[AMR] {label}: failed to read current joints: {e}')
            return False
        target = cur.copy()
        target[5] += delta_deg
        return self.move_j_checked(target, label=label)

    def call_cargo(self, action, slot=0, object_id=0, station_id=0):
        req = Cargo.Request()
        req.action = action
        req.slot = slot
        req.object_id = object_id
        req.station_id = station_id
        return self.call_service(self.cargo_client, req)

    # --- 로봇 이동 헬퍼 ---

    def wait_move(self, timeout=10.0, label='move'):
        if not self.is_robot_ready():
            return False
        try:
            result = self.robot.wait_for_move_finished(self.rc, timeout=timeout)
            if result is False:
                self.get_logger().error(f'[AMR] {label} wait returned False')
                return False
            return True
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} wait failed: {e}')
            return False

    def move_j_checked(self, joints_deg, label='move_j', timeout=10.0):
        if not self.is_robot_ready():
            return False
        if self.robot_data is not None:
            try:
                data = self.robot_data.request_data(1.0)
                cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
                if np.all(np.abs(cur - joints_deg) <= 1.0):
                    self.get_logger().info(f'[AMR] {label} already at target, skip')
                    return True
            except Exception:
                pass
        try:
            self.robot.move_j(self.rc, joints_deg, J_VEL, J_ACC)
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} command failed: {e}')
            return False
        return self.wait_move(timeout=timeout, label=label)

    def move_l_rel_checked(self, delta, label='move_l_rel', timeout=10.0,
                           ref_frame=None):
        if not self.is_robot_ready():
            return False
        if ref_frame is None:
            ref_frame = rb.ReferenceFrame.Tool
        try:
            self.get_logger().info(f'[AMR] command start {label}: {delta} (frame={ref_frame})')
            self.robot.move_l_rel(
                self.rc,
                np.array(delta, dtype=float),
                L_VEL,
                L_ACC,
                ref_frame,
            )
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} command failed: {e}')
            return False
        ok = self.wait_move(timeout=timeout, label=label)
        if ok:
            self.get_logger().info(f'[AMR] command done {label}')
        return ok

    def _startup_move(self):
        """노드 시작 직후 MOVING_JOINT_DEG 자세로 직접 이동한다."""
        with self._busy_lock:
            if self._busy:
                return
            self._busy = True
        try:
            if self.is_at_moving_pose():
                self.get_logger().info('[AMR] already at moving pose, skip startup move')
                return
            self.go_moving_pose()
        finally:
            with self._busy_lock:
                self._busy = False

    def go_home(self):
        return self.move_j_checked(HOME_JOINT_DEG, label='go_home')

    def go_moving_pose(self):
        if not self.move_j_checked(MOVING_JOINT_DEG, label='go_moving_pose'):
            return False
        self.get_logger().info('[AMR] moving pose reached')
        return True

    # --- 웨이포인트 이동 (action별 테이블을 인자로 받음) ---

    def move_to_slot(self, slot, for_unload=False, layer_index=0):
        # 슬롯 7/8 언로드는 전용 웨이포인트 사용
        if for_unload and slot in UNLOAD_SLOT_WAYPOINTS:
            waypoints = UNLOAD_SLOT_WAYPOINTS[slot]
        else:
            waypoints = SLOT_WAYPOINTS.get(slot)

        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for slot={slot}')
            return False

        move_waypoints = list(waypoints)

        # UNLOAD_SLOT_WAYPOINTS 사용 시 마지막 WP가 이미 픽업 위치이므로 교체 안 함
        if for_unload and slot not in UNLOAD_SLOT_WAYPOINTS:
            unload_joint = UNLOAD_SLOT_JOINTS.get(slot * 10 + layer_index)
            if unload_joint is not None:
                move_waypoints[-1] = unload_joint

        for idx, wp in enumerate(move_waypoints, start=1):
            if not self.move_j_checked(wp, label=f'move_to_slot({slot}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] slot={slot} reached')
        return True

    def return_from_slot(self, slot, skip_last=False, for_unload=False):
        # 슬롯 7/8 언로드 복귀는 전용 웨이포인트 역순 사용
        if for_unload and slot in UNLOAD_SLOT_WAYPOINTS:
            waypoints = UNLOAD_SLOT_WAYPOINTS[slot]
        else:
            waypoints = SLOT_WAYPOINTS.get(slot)

        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for slot={slot}')
            return False

        # 역방향 첫 번째는 방금 도착한 최종 자세라서 스킵
        return_waypoints = list(reversed(waypoints))[1:]

        if skip_last:
            return_waypoints = return_waypoints[:-1]

        for idx, wp in enumerate(return_waypoints, start=2):
            if not self.move_j_checked(wp, label=f'return_from_slot({slot}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] returned from slot={slot}')
        return True

    def move_to_delivery(self, delivery_idx):
        waypoints = DELIVERY_WAYPOINTS.get(delivery_idx)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for delivery_idx={delivery_idx}')
            return False

        for idx, wp in enumerate(waypoints, start=1):
            if not self.move_j_checked(wp, label=f'move_to_delivery({delivery_idx}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] delivery position {delivery_idx} reached')
        return True

    def return_from_delivery(self, delivery_idx):
        # waypoint가 1개뿐이면 역순 복귀 없이 그대로 반환.
        # (move_j_checked가 zero-distance를 자동 skip하므로 별도 처리 불필요)
        # waypoint가 여러 개이면 마지막 자세를 제외한 경유점만 역순으로 탄다.
        waypoints = DELIVERY_WAYPOINTS.get(delivery_idx)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for delivery_idx={delivery_idx}')
            return False

        if len(waypoints) > 1:
            return_waypoints = list(reversed(waypoints))[1:]
            for idx, wp in enumerate(return_waypoints, start=1):
                if not self.move_j_checked(wp, label=f'return_from_delivery({delivery_idx}) wp{idx}'):
                    return False

        self.get_logger().info(f'[AMR] returned from delivery position {delivery_idx}')
        return True

    def pick_from_floor_by_vision(self, object_id, label_prefix='pick'):
        vision_target = str(object_id)

        if not self.call_gripper(False):
            return False

        if not self.go_home():
            return False

        p = self.call_vision_with_y_scan(vision_target)
        if not p:
            self.get_logger().error(f'[AMR] vision failed during {label_prefix}')
            self.go_home()
            return False

        off = get_pick_offset(object_id)
        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        z_move = (p.z * 1000.0) + Z_OFFSET
        yaw = p.yaw + off['yaw']

        tool_x = dy + off['x']
        tool_y = dx + off['y']
        tool_z = (z_move - Z_MARGIN) + off['z']

        if any(off[k] != 0.0 for k in ('x', 'y', 'z', 'yaw')):
            self.get_logger().info(
                f'[AMR] {label_prefix} offset applied: object_id={object_id}, '
                f'off={off}, vision_yaw={p.yaw:.2f} -> yaw={yaw:.2f}'
            )

        if not self.move_l_rel_checked(
            [tool_x, tool_y, tool_z, 0.0, 0.0, yaw],
            label=f'{label_prefix} yaw+xy+z approach',
        ):
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
            label=f'{label_prefix} z final approach',
        ):
            self.go_home()
            return False
        time.sleep(GRIP_SETTLE_TIME_SEC)

        if not self.call_gripper(True):
            self.get_logger().error(f'[AMR] {label_prefix} grip failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                label=f'{label_prefix} retreat after grip failure',
            )
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label=f'{label_prefix} lift after grip',
        ):
            self.go_home()
            return False

        return True

    def place_at_delivery(self, delivery_idx):
        """워크벤치 재료 전용(비전 없음, 고정 delivery_idx 웨이포인트). 완성품은
        항상 place_at_delivery_product_fixed / place_at_delivery_product_by_vision을
        쓰므로 이 함수는 재료만 다룬다."""
        if not self.move_to_delivery(delivery_idx):
            self.go_home()
            return False

        delivery_ref = rb.ReferenceFrame.Tool
        delivery_down = [0.0, 0.0, DELIVERY_Z_MM, 0.0, 0.0, 0.0]
        delivery_up = [0.0, 0.0, -DELIVERY_Z_MM, 0.0, 0.0, 0.0]

        if not self.move_l_rel_checked(
            delivery_down,
            label='delivery z down',
            ref_frame=delivery_ref,
        ):
            self.go_home()
            return False

        if not self.call_gripper(False):
            self.get_logger().error('[AMR] final gripper open failed')
            self.move_l_rel_checked(
                delivery_up,
                label='retreat after delivery open failure',
                ref_frame=delivery_ref,
            )
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            delivery_up,
            label='delivery z up',
            ref_frame=delivery_ref,
        ):
            self.go_home()
            return False

        return True

    def place_at_delivery_by_vision(self, label_prefix='delivery(vision)'):
        """워크벤치가 아닌 스테이션(고객센터 등) 전용. 고정 delivery_idx 대신
        비전(target_id=666)으로 실시간 빈 공간 좌표를 받아 그 자리에 물체를 내려놓는다.
        LOAD 쪽 pick_from_floor_by_vision과 동일한 카메라/좌표 변환을 쓰고,
        마지막 동작만 grip 대신 open(내려놓기)으로 바뀐다."""
        if not self.move_j_checked(
            VISION_LOAD_JOINT_DEG, label=f'{label_prefix} vision pose'
        ):
            return False

        p = self.call_vision_with_y_scan(DELIVERY_EMPTY_SPACE_VISION_ID)
        if not p:
            self.get_logger().error(f'[AMR] vision failed during {label_prefix}')
            self.go_home()
            return False

        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        z_move = (p.z * 1000.0) + Z_OFFSET - DELIVERY_VISION_Z_OFFSET_MM
        yaw = p.yaw

        tool_x = dy
        tool_y = dx
        tool_z = z_move - Z_MARGIN

        if not self.move_l_rel_checked(
            [tool_x, tool_y, tool_z, 0.0, 0.0, yaw],
            label=f'{label_prefix} yaw+xy+z approach',
        ):
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
            label=f'{label_prefix} z final approach',
        ):
            self.go_home()
            return False

        if not self.call_gripper(False):
            self.get_logger().error(f'[AMR] {label_prefix} gripper open failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                label=f'retreat after {label_prefix} open failure',
            )
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label=f'{label_prefix} lift after place',
        ):
            self.go_home()
            return False

        return self.go_moving_pose()

    def _place_product_at_point6(self, z_mm, label_prefix):
        """point 6에 도착한 이후 공통 동작: z 하강 -> open -> z 상승 -> moving pose 복귀
        (Base 프레임). place_at_delivery_product_by_vision / _fixed가 각자의 z_mm으로 공유한다."""
        delivery_ref = rb.ReferenceFrame.Base
        delivery_down = [0.0, 0.0, -z_mm, 0.0, 0.0, 0.0]
        delivery_up = [0.0, 0.0, z_mm, 0.0, 0.0, 0.0]

        if not self.move_l_rel_checked(
            delivery_down,
            label=f'{label_prefix} z down',
            ref_frame=delivery_ref,
        ):
            self.go_home()
            return False

        if not self.call_gripper(False):
            self.get_logger().error(f'[AMR] {label_prefix} gripper open failed')
            self.move_l_rel_checked(
                delivery_up,
                label=f'retreat after {label_prefix} open failure',
                ref_frame=delivery_ref,
            )
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            delivery_up,
            label=f'{label_prefix} z up',
            ref_frame=delivery_ref,
        ):
            self.go_home()
            return False

        return self.go_moving_pose()

    def place_at_delivery_product_by_vision(self, label_prefix='delivery(product,vision)'):
        """완성품 전용, 워크벤치가 아닌 스테이션. 슬롯(1 또는 7/8)과 무관하게 항상
        point 6(PRODUCT_DELIVERY_IDX) 고정 조인트로 가되, 비전(666)으로 측정한 x,y만
        미세 보정으로 얹는다. z는 층 그룹과 무관하게 항상 PRODUCT_DELIVERY_VISION_Z_MM만큼
        내려갔다 올라온다."""
        if not self.move_j_checked(
            VISION_LOAD_JOINT_DEG, label=f'{label_prefix} vision pose'
        ):
            return False

        p = self.call_vision_with_y_scan(DELIVERY_EMPTY_SPACE_VISION_ID)
        if not p:
            self.get_logger().error(f'[AMR] vision failed during {label_prefix}')
            self.go_home()
            return False

        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        tool_x = dy
        tool_y = dx

        product_joint = DELIVERY_WAYPOINTS[PRODUCT_DELIVERY_IDX][-1]
        if not self.move_j_checked(product_joint, label=f'{label_prefix} point6 pose'):
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [tool_x, tool_y, 0.0, 0.0, 0.0, 0.0],
            label=f'{label_prefix} xy correction',
        ):
            self.go_home()
            return False

        return self._place_product_at_point6(PRODUCT_DELIVERY_VISION_Z_MM, label_prefix)

    def place_at_delivery_product_fixed(self, label_prefix='delivery(product,fixed)'):
        """완성품 전용, 워크벤치 스테이션. 비전을 전혀 쓰지 않고 point 6 고정 조인트로
        바로 이동한 뒤, PRODUCT_DELIVERY_FIXED_Z_MM만큼 내려갔다 올라온다."""
        product_joint = DELIVERY_WAYPOINTS[PRODUCT_DELIVERY_IDX][-1]
        if not self.move_j_checked(product_joint, label=f'{label_prefix} point6 pose'):
            self.go_home()
            return False

        return self._place_product_at_point6(PRODUCT_DELIVERY_FIXED_Z_MM, label_prefix)

    # --- 서비스 콜백 (LOAD / UNLOAD 분기) ---

    def arm_robot_command_cb(self, request, response):
        response.slots = []
        response.object_ids = []

        action = request.action.upper()
        if action not in ('LOAD', 'UNLOAD', 'ASSEMBLE'):
            response.success = False
            response.message = f'unknown action: {request.action}'
            return response

        if not self.is_robot_ready():
            response.success = False
            response.message = 'robot not connected'
            return response

        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'busy'
                return response
            self._busy = True

        try:
            if action == 'LOAD':
                results = self.sequence_load_multi(list(request.object_ids))
            elif action == 'UNLOAD':
                results = self.sequence_unload_multi(
                    list(request.object_ids), request.station_id)
            else:
                results = self.sequence_assemble_multi(list(request.object_ids))

            success_all = bool(results) and all(r['success'] for r in results)
            response.success = success_all
            response.slots = [r['slot'] for r in results]
            response.object_ids = [r['object_id'] for r in results]
            response.message = ', '.join(r['message'] for r in results)
        except Exception as e:
            self.get_logger().error(f'[AMR] exception: {e}')
            response.success = False
            response.slots = []
            response.object_ids = []
            response.message = str(e)
        finally:
            with self._busy_lock:
                self._busy = False

        return response

    # --- LOAD 시퀀스 ---

    def sequence_load_multi(self, object_ids):
        results = []
        last_idx = len(object_ids) - 1
        for idx, object_id in enumerate(object_ids):
            is_last = (idx == last_idx)
            result = self.sequence_load(object_id, is_last=is_last)
            results.append(result)
            if not result['success']:
                self.get_logger().error(f'[AMR] load failed at object_id={object_id}, stopping')
                break

        self.go_moving_pose()
        return results

    def sequence_load(self, object_id, is_last=False):
        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'robot not connected',
            }

        target_color = MATERIAL_NAMES.get(object_id)
        if not target_color:
            self.get_logger().error(f'[AMR] unknown object_id: {object_id}')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': f'unknown object_id={object_id}',
            }

        vision_target = str(object_id)

        self.get_logger().info(f'[LOAD START] object_id={object_id}, target={target_color}')

        # 1. 빈 슬롯 확인
        res = self.call_cargo('FIND_EMPTY', object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] no empty slot')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'no empty slot',
            }
        slot = res.slot
        self.get_logger().info(f'[CARGO] empty slot: {slot}')

        # 2. 초기화: 그리퍼 open
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'initial gripper open failed',
            }

        # 3~6. 비전 탐색 -> 접근 -> grip -> pos 기반 장축/단축/오파지 판정
        #   - 오파지(fail): 비전부터 다시 시도한다 (최대 MAX_LOAD_GRIP_ATTEMPTS 회)
        #   - 장축(long)  : 비전으로 돌아가지 않고 그 자리에서 J6만 돌려 재파지한다.
        #                   재파지 후에도 여전히 fail/long 이면 재시도하지 않고
        #                   moving pose 로 복귀 후 실패 처리한다.
        #   완성품(FINISHED_PRODUCTS)은 이 pos 기반 판정을 하지 않는다 - grip 성공
        #   여부만 확인하고 바로 다음 단계로 넘어간다.
        skip_grip_judgment = object_id in FINISHED_PRODUCTS
        grip_attempts = 1 if skip_grip_judgment else MAX_LOAD_GRIP_ATTEMPTS
        grip_pos = None
        grip_type = None

        for attempt in range(1, grip_attempts + 1):
            if not self.move_j_checked(
                VISION_LOAD_JOINT_DEG, label=f'vision load pose (attempt {attempt})'
            ):
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': object_id,
                    'message': 'vision load pose failed',
                }

            # 3. 비전 자세 기준 center -> left -> right 순서로 탐색
            p = self.call_vision_with_y_scan(vision_target)
            if not p:
                self.get_logger().error('[AMR] vision failed')
                self.go_home()
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': object_id,
                    'message': 'OBJECT_NOT_FOUND',
                }

            # 4. YAW + XY + Z접근 동시 이동
            #    물체 바로 위(Z_MARGIN)까지 대각선으로 내려가고,
            #    최종 접근은 5번에서 수직으로 따로 한다. (대각선 최종접근은 파지 안정성 저하)
            off = get_pick_offset(object_id)
            dx = -(p.x * 1000.0) + CAM_Y_OFF
            dy = (p.y * 1000.0) + CAM_X_OFF
            z_move = (p.z * 1000.0) + Z_OFFSET

            yaw = p.yaw + off['yaw']

            tool_x = dy + off['x']
            tool_y = dx + off['y']
            tool_z = (z_move - Z_MARGIN) + off['z']

            if any(off[k] != 0.0 for k in ('x', 'y', 'z', 'yaw')):
                self.get_logger().info(
                    f'[LOAD] pick offset applied: object_id={object_id}, '
                    f'off={off}, vision_yaw={p.yaw:.2f} -> yaw={yaw:.2f}')

            if not self.move_l_rel_checked(
                [tool_x, tool_y, tool_z, 0.0, 0.0, yaw],
                label='yaw+xy+z approach',
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': object_id,
                    'message': 'yaw+xy+z approach failed',
                }

            # 5. 수직 최종 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
            if not self.move_l_rel_checked(
                [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
                label='z final approach',
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': object_id,
                    'message': 'z final approach failed',
                }
            time.sleep(GRIP_SETTLE_TIME_SEC)

            # 6. 그리퍼 grip (+ pos 기반 장축/단축/오파지 판정)
            grip_ok, grip_pos = self.call_gripper_grip_with_pos()
            if not grip_ok:
                self.get_logger().error('[AMR] grip failed')
                self.move_l_rel_checked(
                    [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                    label='retreat after grip failure',
                )
                self.go_home()
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': object_id,
                    'message': 'grip failed',
                }

            if skip_grip_judgment:
                self.get_logger().info(f'[AMR] product grip pos={grip_pos} (판정 생략)')
                break

            grip_type = classify_grip_pos(grip_pos)
            self.get_logger().info(
                f'[AMR] attempt {attempt}/{grip_attempts}: grip pos={grip_pos} -> {grip_type}'
            )

            if grip_type == 'fail':
                self.get_logger().warn(f'[AMR] mis-grasp detected (pos={grip_pos})')
                self.call_gripper(False)
                self.move_l_rel_checked(
                    [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                    label='retreat after mis-grasp',
                )
                if attempt == grip_attempts:
                    self.go_moving_pose()
                    return {
                        'success': False,
                        'slot': -1,
                        'object_id': object_id,
                        'message': f'mis-grasp, retries exhausted (pos={grip_pos})',
                    }
                # 비전부터 다시 시도
                continue

            if grip_type == 'long':
                self.get_logger().info(
                    f'[AMR] long-axis grasp (pos={grip_pos}) -> open, '
                    f'J6 +{GRIP_REORIENT_J6_DEG} reorient, re-grip (재비전 없이 그 자리에서)'
                )
                if not self.call_gripper(False):
                    self.go_home()
                    return {
                        'success': False,
                        'slot': -1,
                        'object_id': object_id,
                        'message': 'open before J6 reorient failed',
                    }

                if not self.rotate_j6_checked(GRIP_REORIENT_J6_DEG, label='J6 reorient +90'):
                    self.go_home()
                    return {
                        'success': False,
                        'slot': -1,
                        'object_id': object_id,
                        'message': 'J6 reorient failed',
                    }

                grip_ok, grip_pos = self.call_gripper_grip_with_pos()
                if not grip_ok:
                    self.get_logger().error('[AMR] re-grip after J6 reorient failed')
                    self.move_l_rel_checked(
                        [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                        label='retreat after re-grip failure',
                    )
                    self.go_home()
                    return {
                        'success': False,
                        'slot': -1,
                        'object_id': object_id,
                        'message': 're-grip after J6 reorient failed',
                    }
                grip_type = classify_grip_pos(grip_pos)
                self.get_logger().info(f'[AMR] re-grip pos={grip_pos} -> {grip_type}')

                if grip_type in ('fail', 'long'):
                    # 재파지 후에도 여전히 오파지/장축 -> 더 재시도하지 않고 moving pose 로 복귀 후 실패 처리
                    self.get_logger().error(
                        f'[AMR] re-grip still {grip_type} after J6 reorient (pos={grip_pos}), giving up'
                    )
                    self.call_gripper(False)
                    self.move_l_rel_checked(
                        [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                        label='retreat after reorient still bad',
                    )
                    self.go_moving_pose()
                    return {
                        'success': False,
                        'slot': -1,
                        'object_id': object_id,
                        'message': f're-grip still {grip_type} after J6 reorient (pos={grip_pos})',
                    }

            # grip_type == 'short'(정상) 이거나 'unknown', 혹은 장축 재파지 성공 -> 다음 단계로
            break
        else:
            # 이론상 도달하지 않음: 마지막 시도의 'fail' 은 위에서 이미 return 처리됨
            self.go_moving_pose()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'grip retries exhausted',
            }

        # 7. Z 상승
        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label='lift after grip',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'lift after grip failed',
            }

        # 8. 웨이포인트 순서대로 슬롯으로 이동
        if not self.move_to_slot(slot):
            self.get_logger().error('[AMR] move to slot failed')
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'move to slot failed',
            }

        # 9. Z 하강 -> open -> Z 상승
        place_z_down = SLOT1_Z_DOWN_MM if slot == 1 else LOAD_Z_DOWN_MM
        place_z_up   = SLOT1_Z_UP_MM   if slot == 1 else LOAD_Z_UP_MM
        if not self.move_l_rel_checked(
            [0.0, 0.0, place_z_down, 0.0, 0.0, 0.0],
            label='place z down',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'place z down failed',
            }

        if not self.call_gripper(False):
            self.get_logger().error('[AMR] final gripper open failed')
            self.move_l_rel_checked(
                [0.0, 0.0, place_z_up, 0.0, 0.0, 0.0],
                label='retreat after open failure',
            )
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'final gripper open failed',
            }

        if not self.move_l_rel_checked(
            [0.0, 0.0, place_z_up, 0.0, 0.0, 0.0],
            label='place z up',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'place z up failed',
            }

        # 10. 웨이포인트 역순으로 복귀
        #     마지막 물체면 복귀 경로의 끝점(SLOT_COMMON_WPS[0])을 생략하고 바로 이동 포즈로 간다.
        if not self.return_from_slot(slot, skip_last=is_last):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'return from slot failed',
            }

        # 11. 카고 기록
        res = self.call_cargo('SET', slot=slot, object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo SET failed')
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'loaded physically but cargo SET failed',
            }

        self.get_logger().info(f'[LOAD DONE] object_id={object_id}, slot={slot}')
        return {
            'success': True,
            'slot': slot,
            'object_id': object_id,
            'message': 'load success',
        }

    # --- UNLOAD 시퀀스 ---

    def sequence_unload_multi(self, object_ids, station_id=0):
        results = []
        # 워크벤치 station일 때만 쓰는 배치 내 순번 (0부터). 몇 개를 내려놓을지는
        # 이 요청의 object_ids 개수로 이미 정해지므로, cargo_manager에 따로 기억시킬
        # 필요 없이 여기서 세면서 0, 1, 2, ... 순서로 delivery_idx를 배정한다.
        workbench_delivery_idx = 0
        for object_id in object_ids:
            result = self.sequence_unload(
                object_id, station_id=station_id, workbench_delivery_idx=workbench_delivery_idx)
            results.append(result)
            if not result['success']:
                self.get_logger().error(f'[AMR] unload failed at object_id={object_id}, stopping')
                break
            if object_id not in FINISHED_PRODUCTS and station_id in WORKBENCH_STATION_IDS:
                workbench_delivery_idx += 1

        self.go_moving_pose()
        return results

    def sequence_unload(self, object_id, station_id=0, workbench_delivery_idx=0):
        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'robot not connected',
            }

        self.get_logger().info(f'[UNLOAD START] object_id={object_id}, station_id={station_id}')

        # 완성품 여부 판별: 완성품이면 delivery 단계에서 전용 포인트/전용 Z를 쓴다.
        is_product = object_id in FINISHED_PRODUCTS
        if is_product:
            self.get_logger().info(f'[UNLOAD] object_id={object_id} is a finished product')

        # 1. 슬롯 확인
        res = self.call_cargo('FIND_OBJECT', object_id=object_id)
        if not res or not res.success:
            self.get_logger().error(f'[AMR] object_id={object_id} not found in cargo')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': f'object not found: {object_id}',
            }
        slot = res.slot
        layer_index = res.layer_index
        self.get_logger().info(f'[CARGO] object found: slot={slot}, layer_index={layer_index}')

        # 1-1. 배달 위치 결정
        #   완제품(is_product)은 원래 있던 슬롯(1이든 7/8이든)과 무관하게 항상
        #   point 6(PRODUCT_DELIVERY_IDX) 고정 조인트로 가고, z는 층 그룹과 무관한
        #   고정값을 쓴다. 다만 워크벤치(WORKBENCH_STATION_IDS)는 비전을 아예 쓰지 않고
        #   point 6로 바로 가고(place_at_delivery_product_fixed, PRODUCT_DELIVERY_FIXED_Z_MM),
        #   그 외 스테이션(고객센터 등)은 비전(666)으로 x,y만 미세 보정한다
        #   (place_at_delivery_product_by_vision, PRODUCT_DELIVERY_VISION_Z_MM).
        #   재료는 station이 워크벤치면 이번 UNLOAD 배치 안에서 몇 번째로 내려놓는지
        #   (sequence_unload_multi가 세어서 넘겨주는 workbench_delivery_idx)를 그대로
        #   delivery_idx로 써서 0번부터 순서대로 고정 웨이포인트에 내려놓는다.
        #   워크벤치가 아닌 스테이션의 재료는 비전(666)으로 실시간 빈 공간을 찾아
        #   내려놓으므로 delivery_idx가 필요 없다.
        use_vision_delivery = False
        use_product_vision_delivery = False
        use_product_fixed_delivery = False
        delivery_idx = None
        if is_product:
            if station_id in WORKBENCH_STATION_IDS:
                use_product_fixed_delivery = True
            else:
                use_product_vision_delivery = True
        elif station_id in WORKBENCH_STATION_IDS:
            delivery_idx = workbench_delivery_idx
            if not (0 <= delivery_idx <= 5):
                self.get_logger().error(
                    f'[AMR] no empty delivery slot at station={station_id} (idx={delivery_idx})')
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': f'no empty delivery slot at station={station_id}',
                }
        else:
            use_vision_delivery = True

        # 2. 초기화
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'initial gripper open failed',
            }

        # 3. 웨이포인트 순서대로 슬롯으로 이동 (레이어별 UNLOAD 위치 사용)
        if not self.move_to_slot(slot, for_unload=True, layer_index=layer_index):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'move to slot failed',
            }

        # 4-6. 픽업 벡터 결정
        #   슬롯 7/8: Tool X 방향으로 이동
        #   슬롯 2~6: Tool Z 방향으로 이동 (완성품은 ASSEMBLY_Z, 재료는 UNLOAD_Z)
        if slot in UNLOAD_SLOT_WAYPOINTS:
            x_dir = UNLOAD_SLOT_X_DIR.get(slot, 1.0)
            pick_down = [UNLOAD_X_DOWN_MM * x_dir, 0.0, 0.0, 0.0, 0.0, 0.0]
            pick_up   = [UNLOAD_X_UP_MM   * x_dir, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif slot == 1:
            pick_down = [0.0, 0.0, SLOT1_Z_DOWN_MM, 0.0, 0.0, 0.0]
            pick_up   = [0.0, 0.0, SLOT1_Z_UP_MM,   0.0, 0.0, 0.0]
        else:
            pickup_z_down = ASSEMBLY_Z_DOWN_MM if is_product else UNLOAD_Z_DOWN_MM
            pickup_z_up   = ASSEMBLY_Z_UP_MM   if is_product else UNLOAD_Z_UP_MM
            pick_down = [0.0, 0.0, pickup_z_down, 0.0, 0.0, 0.0]
            pick_up   = [0.0, 0.0, pickup_z_up,   0.0, 0.0, 0.0]

        # 4. 하강
        self.get_logger().info('[AMR] start slot pick down')
        if not self.move_l_rel_checked(
            pick_down,
            label='slot pick down',
            ref_frame=rb.ReferenceFrame.Tool,
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'slot pick down failed',
            }

        # 5. 그리퍼 grip
        grip_client = self.gripper_grip110_client if (is_product and slot in (7, 8)) else None
        if not self.call_gripper(True, client=grip_client):
            self.get_logger().error('[AMR] grip failed')
            self.move_l_rel_checked(
                pick_up,
                label='retreat after grip failure',
                ref_frame=rb.ReferenceFrame.Tool,
            )
            self.return_from_slot(slot, for_unload=True)
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'grip failed',
            }

        # 6. 상승
        self.get_logger().info('[AMR] start slot pick up')
        if not self.move_l_rel_checked(
            pick_up,
            label='slot pick up',
            ref_frame=rb.ReferenceFrame.Tool,
        ):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'slot pick up failed',
            }

        # 7. 슬롯에서 물체를 들어 올렸으므로 cargo 상태를 먼저 비운다.
        # 이후 복귀 실패가 나도 cargo_manager의 슬롯 상태는 실제 물리 상태와 맞는다.
        res = self.call_cargo('CLEAR', slot=slot, object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo CLEAR failed')
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'object picked physically but cargo CLEAR failed',
            }

        # 8. 웨이포인트 역순으로 홈 복귀
        if not self.return_from_slot(slot, for_unload=True):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'cargo CLEAR done, but return from slot failed',
            }

        # 9. 배달 위치로 이동해 내려놓는다.
        #    완제품은 항상 point 6로 가되, 워크벤치는 비전 없이 바로, 그 외 스테이션은
        #    비전 xy 보정을 거친다. 워크벤치 재료는 1-1에서 정한 delivery_idx로 고정
        #    웨이포인트, 그 외 스테이션 재료는 비전(666)으로 바로 내려놓는다.
        if use_product_fixed_delivery:
            if not self.place_at_delivery_product_fixed():
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'delivery placement (product,fixed) failed',
                }
        elif use_product_vision_delivery:
            if not self.place_at_delivery_product_by_vision():
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'delivery placement (product,vision) failed',
                }
        elif use_vision_delivery:
            if not self.place_at_delivery_by_vision():
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'delivery placement (vision) failed',
                }
        else:
            if not self.place_at_delivery(delivery_idx):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'delivery placement failed',
                }

            # 10. 웨이포인트 역순으로 홈 복귀
            if not self.return_from_delivery(delivery_idx):
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'return from delivery failed',
                }

        self.get_logger().info(
            f'[UNLOAD DONE] object_id={object_id}, slot={slot}, delivery_idx={delivery_idx}'
        )
        return {
            'success': True,
            'slot': slot,
            'object_id': object_id,
            'message': 'unload success',
        }


    # --- ASSEMBLE 시퀀스 ---

    def sequence_assemble_multi(self, object_ids):
        results = []
        last_target_slot = None
        for product_id in object_ids:
            result = self.sequence_assemble(product_id)
            results.append(result)
            if result['success']:
                last_target_slot = result['slot']
            else:
                self.get_logger().error(
                    f'[AMR] assemble failed at product_id={product_id}, stopping')
                break

        # 마지막으로 성공한 조립의 target_slot 기준으로 복귀한다.
        # (실패로 끝난 경우 sequence_assemble 안에서 이미 go_home() 등으로 안전하게 빠져나온 상태)
        if last_target_slot is not None:
            self.return_from_slot(last_target_slot)
        self.go_moving_pose()
        return results

    def resolve_assembly_slot(self, product_id, expected_ids=None):
        """조립에 사용할 target_slot(7 또는 8)을 cargo_manager에게 물어서 정한다.
        station_id 등 호출자가 지정한 값을 슬롯 번호로 쓰지 않는다 — station_id와
        slot id는 서로 다른 개념이므로 이 판단은 항상 cargo_manager의 실제 슬롯
        상태를 기준으로 한다.

        1. expected_ids가 주어지면(일반 ASSEMBLY_SEQUENCE 기반 제품), 이미 이 제품의
           부분 조립이 진행 중인 슬롯이 있는지 먼저 확인해서 있으면 그 슬롯을
           재사용(재개)한다.
        2. 재개할 슬롯이 없으면 비어있는 조립 슬롯을 순서대로(7 -> 8) 배정받는다.
        3. 반환값: (target_slot, current_stack). 배정 실패 시 (None, None).
        """
        if expected_ids:
            for slot in ASSEMBLY_SLOTS:
                res = self.call_cargo('FIND_SLOT_STACK', slot=slot)
                if not res or not res.success:
                    self.get_logger().error(f'[AMR] cargo FIND_SLOT_STACK failed for slot={slot}')
                    return None, None
                stack = list(res.stack)
                if stack and stack == expected_ids[:len(stack)]:
                    self.get_logger().info(
                        f'[AMR] resuming existing partial build of product_id={product_id} '
                        f'in slot={slot} (stack={stack})')
                    return slot, stack

        res = self.call_cargo('FIND_EMPTY_ASSEMBLY_SLOT')
        if not res or not res.success:
            self.get_logger().error('[AMR] no available assembly slot (7, 8 occupied)')
            return None, None
        return res.slot, []

    def find_empty_material_slot(self):
        """재료슬롯(2-6) 중 완전히 빈 슬롯 하나를 찾아 반환한다.
        아이스크림/빅트리의 캡(재료 조합) 스테이징용으로 쓴다. 없으면 None."""
        for slot in STAGING_MATERIAL_SLOTS:
            res = self.call_cargo('FIND_SLOT_STACK', slot=slot)
            if not res or not res.success:
                self.get_logger().error(f'[AMR] cargo FIND_SLOT_STACK failed for slot={slot}')
                continue
            if len(res.stack) == 0:
                return slot
        return None

    def find_material_source(self, object_id, exclude_slot=None):
        """object_id가 있는 슬롯을 찾는다. exclude_slot이 주어지면 그 슬롯은 제외하고
        찾되(FIND_OBJECT_EXCLUDING), 실패하면 exclude_slot 포함 전체에서 다시 찾는다
        (단, 그 결과가 exclude_slot 자신이면 그대로 실패로 둔다 - 다른 슬롯에 없다는 뜻)."""
        if exclude_slot is None:
            return self.call_cargo('FIND_OBJECT', object_id=object_id)

        res = self.call_cargo('FIND_OBJECT_EXCLUDING', object_id=object_id, slot=exclude_slot)
        if res and res.success:
            return res

        fallback = self.call_cargo('FIND_OBJECT', object_id=object_id)
        if fallback and fallback.success and fallback.slot != exclude_slot:
            return fallback
        return res

    def pick_material_for_assembly(self, material_id, label_prefix, exclude_slot=None, direct=True):
        """재료슬롯(2-6)에서 material_id를 찾아 집어 든다 (그리퍼는 문 채로 반환).
        exclude_slot이 주어지면 그 슬롯은 검색에서 제외한다 (같은 object_id가 두
        슬롯에 나뉘어 있을 때, 이미 확보한 슬롯 말고 다른 슬롯에서 찾아야 하는 경우).
        direct=True(기본)면 ASSEMBLY_TRANSIT_JOINT_DEG를 거쳐 소스 슬롯 조인트(UNLOAD_SLOT_JOINTS)로
        직행하고, direct=False면 공통 웨이포인트를 전부 거치는 move_to_slot()을 사용한다
        (한 시퀀스의 맨 첫 픽업에서만 direct=False로 안전하게 진입한다)."""
        res = self.find_material_source(material_id, exclude_slot=exclude_slot)
        if not res or not res.success:
            return {
                'success': False,
                'slot': -1,
                'message': f'{label_prefix}: material {material_id} not found',
            }

        slot = res.slot
        layer_index = res.layer_index

        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': slot,
                'message': f'{label_prefix}: gripper open failed',
            }

        if direct:
            slot_joint = UNLOAD_SLOT_JOINTS.get(slot * 10 + layer_index)
            if slot_joint is None:
                self.get_logger().error(
                    f'[AMR] {label_prefix}: no unload joint for slot={slot} layer={layer_index}')
                return {
                    'success': False,
                    'slot': slot,
                    'message': f'{label_prefix}: no unload joint for slot={slot} layer={layer_index}',
                }
            if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label=f'{label_prefix} pickup transit'):
                self.return_from_slot(slot, for_unload=True)
                return {
                    'success': False,
                    'slot': slot,
                    'message': f'{label_prefix}: transit move failed',
                }
            if not self.move_j_checked(slot_joint, label=f'{label_prefix} to slot={slot}'):
                self.return_from_slot(slot, for_unload=True)
                return {
                    'success': False,
                    'slot': slot,
                    'message': f'{label_prefix}: move to source slot failed',
                }
        else:
            if not self.move_to_slot(slot, for_unload=True, layer_index=layer_index):
                return {
                    'success': False,
                    'slot': slot,
                    'message': f'{label_prefix}: move to source slot failed',
                }

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                       label=f'{label_prefix} z down'):
            self.return_from_slot(slot, for_unload=True)
            return {
                'success': False,
                'slot': slot,
                'message': f'{label_prefix}: source z down failed',
            }

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                                    label=f'{label_prefix} retreat after grip failure')
            self.return_from_slot(slot, for_unload=True)
            return {
                'success': False,
                'slot': slot,
                'message': f'{label_prefix}: grip failed',
            }

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                                       label=f'{label_prefix} z up'):
            self.return_from_slot(slot, for_unload=True)
            return {
                'success': False,
                'slot': slot,
                'message': f'{label_prefix}: source z up failed',
            }

        res = self.call_cargo('CLEAR', slot=slot, object_id=material_id)
        if not res or not res.success:
            self.return_from_slot(slot, for_unload=True)
            return {
                'success': False,
                'slot': slot,
                'message': f'{label_prefix}: cargo CLEAR {material_id} failed',
            }

        return {
            'success': True,
            'slot': slot,
            'layer_index': layer_index,
        }

    def place_gripped_to_assembly_slot(self, target_slot, z_down_mm, label_prefix,
                                       x_offset=0.0, cargo_set_ids=None, direct=True):
        """현재 그리퍼에 문 재료(또는 재료 묶음)를 조립슬롯(target_slot)에 x_offset만큼
        Tool X로 비켜서 내려놓는다. 놓고 나면 cargo_set_ids에 있는 각 object_id를
        target_slot에 SET한다 (묶음을 통째로 옮긴 경우 여러 id를 한 번에 등록).
        direct=True(기본)면 ASSEMBLY_TRANSIT_JOINT_DEG를 거쳐 target_slot 조인트(LOAD_SLOT_JOINTS)로
        직행하고, direct=False면 공통 웨이포인트를 전부 거치는 move_to_slot()을 사용한다."""
        if direct:
            target_joint = LOAD_SLOT_JOINTS.get(target_slot)
            if target_joint is None:
                self.get_logger().error(f'[AMR] {label_prefix}: no load joint for target_slot={target_slot}')
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: no load joint for target_slot={target_slot}',
                }
            if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label=f'{label_prefix} place transit'):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: transit move failed',
                }
            if not self.move_j_checked(target_joint, label=f'{label_prefix} to target_slot={target_slot}'):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: move to assembly slot failed',
                }
        else:
            if not self.move_to_slot(target_slot):
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: move to assembly slot failed',
                }

        if abs(x_offset) > 1e-6:
            if not self.move_l_rel_checked([x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                           label=f'{label_prefix} x offset',
                                           ref_frame=rb.ReferenceFrame.Tool):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: x offset failed',
                }

        if not self.move_l_rel_checked([0.0, 0.0, z_down_mm, 0.0, 0.0, 0.0],
                                       label=f'{label_prefix} z down',
                                       ref_frame=rb.ReferenceFrame.Tool):
            self.return_from_slot(target_slot)
            return {
                'success': False,
                'slot': target_slot,
                'message': f'{label_prefix}: assembly z down failed',
            }

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -z_down_mm, 0.0, 0.0, 0.0],
                                    label=f'{label_prefix} retreat after release failure',
                                    ref_frame=rb.ReferenceFrame.Tool)
            if abs(x_offset) > 1e-6:
                self.move_l_rel_checked([-x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        label=f'{label_prefix} x return after release failure',
                                        ref_frame=rb.ReferenceFrame.Tool)
            self.return_from_slot(target_slot)
            return {
                'success': False,
                'slot': target_slot,
                'message': f'{label_prefix}: release failed',
            }

        if not self.move_l_rel_checked([0.0, 0.0, -z_down_mm, 0.0, 0.0, 0.0],
                                       label=f'{label_prefix} z up',
                                       ref_frame=rb.ReferenceFrame.Tool):
            self.return_from_slot(target_slot)
            return {
                'success': False,
                'slot': target_slot,
                'message': f'{label_prefix}: assembly z up failed',
            }

        if abs(x_offset) > 1e-6:
            if not self.move_l_rel_checked([-x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                                           label=f'{label_prefix} x return',
                                           ref_frame=rb.ReferenceFrame.Tool):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: x return failed',
                }

        for material_id in cargo_set_ids or []:
            res = self.call_cargo('SET', slot=target_slot, object_id=material_id)
            if not res or not res.success:
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': target_slot,
                    'message': f'{label_prefix}: cargo SET {material_id} failed',
                }

        return {'success': True, 'slot': target_slot}

    def pickup_big_tree_stage_group_from_slot8(self):
        """BigTree 전용: slot 8(BIG_TREE_STAGE_SLOT)에 부분조립해 둔 6+2+6 묶음을
        BIG_TREE_STAGE_PICKUP_X_MM만큼 비켜서 통째로 집어 든다 (그리퍼는 문 채로 반환).
        시퀀스 중간에만 호출되므로(맨 첫 픽업이 아님) 항상 TRANSIT 경유 직행으로 slot 8까지 간다."""
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: gripper open failed',
            }

        stage_joint = LOAD_SLOT_JOINTS.get(BIG_TREE_STAGE_SLOT)
        if stage_joint is None:
            self.get_logger().error(
                f'[AMR] big_tree stage pickup: no load joint for slot={BIG_TREE_STAGE_SLOT}')
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': f'big_tree stage pickup: no load joint for slot={BIG_TREE_STAGE_SLOT}',
            }

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='big_tree stage pickup transit'):
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: transit move failed',
            }

        if not self.move_j_checked(stage_joint, label=f'big_tree stage pickup to slot={BIG_TREE_STAGE_SLOT}'):
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: move to slot8 assembly slot failed',
            }

        if not self.move_l_rel_checked([BIG_TREE_STAGE_PICKUP_X_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                       label='big_tree stage pickup x offset',
                                       ref_frame=rb.ReferenceFrame.Tool):
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: x offset failed',
            }

        if not self.move_l_rel_checked([0.0, 0.0, BIG_TREE_STAGE_PICKUP_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                       label='big_tree stage pickup z down',
                                       ref_frame=rb.ReferenceFrame.Tool):
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: z down failed',
            }

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, BIG_TREE_STAGE_PICKUP_Z_UP_MM, 0.0, 0.0, 0.0],
                                    label='big_tree stage pickup retreat after grip failure',
                                    ref_frame=rb.ReferenceFrame.Tool)
            self.move_l_rel_checked([-BIG_TREE_STAGE_PICKUP_X_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    label='big_tree stage pickup x return after grip failure',
                                    ref_frame=rb.ReferenceFrame.Tool)
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: grip failed',
            }

        if not self.move_l_rel_checked([0.0, 0.0, BIG_TREE_STAGE_PICKUP_Z_UP_MM, 0.0, 0.0, 0.0],
                                       label='big_tree stage pickup z up',
                                       ref_frame=rb.ReferenceFrame.Tool):
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: z up failed',
            }

        if not self.move_l_rel_checked([-BIG_TREE_STAGE_PICKUP_X_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                       label='big_tree stage pickup x return',
                                       ref_frame=rb.ReferenceFrame.Tool):
            self.return_from_slot(BIG_TREE_STAGE_SLOT)
            return {
                'success': False,
                'slot': BIG_TREE_STAGE_SLOT,
                'message': 'big_tree stage pickup: x return failed',
            }

        for material_id in BIG_TREE_STAGE_GROUP_IDS:
            res = self.call_cargo('CLEAR', slot=BIG_TREE_STAGE_SLOT, object_id=material_id)
            if not res or not res.success:
                self.return_from_slot(BIG_TREE_STAGE_SLOT)
                return {
                    'success': False,
                    'slot': BIG_TREE_STAGE_SLOT,
                    'message': f'big_tree stage pickup: cargo CLEAR {material_id} failed',
                }

        return {'success': True, 'slot': BIG_TREE_STAGE_SLOT}

    def sequence_assemble(self, product_id):
        if product_id == 46262:
            return self.sequence_assemble_big_tree()
        if product_id == 48132:
            return self.sequence_assemble_ice_cream()

        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': 'robot not connected',
            }

        sequence = ASSEMBLY_SEQUENCE.get(product_id)
        if sequence is None:
            self.get_logger().error(
                f'[AMR] no assembly sequence for product_id={product_id}')
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': f'no assembly sequence for product_id={product_id}',
            }

        expected_ids = [
            step['id'] if isinstance(step, dict) else step for step in sequence
        ]

        # target_slot 결정: 진행 중인 부분 조립이 있으면 그 슬롯을 재사용(재개)하고,
        # 없으면 cargo_manager가 배정해주는 빈 조립 슬롯을 쓴다 (7 -> 8 순서).
        target_slot, current_stack = self.resolve_assembly_slot(product_id, expected_ids)
        if target_slot is None:
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': 'no available assembly slot (7, 8 occupied by other builds)',
            }

        assembly_wps = SLOT_WAYPOINTS.get(target_slot)
        if assembly_wps is None:
            self.get_logger().error(
                f'[AMR] no slot waypoints for target_slot={target_slot}')
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': f'no slot waypoints for target_slot={target_slot}',
            }

        assembly_joint = assembly_wps[-1]  # SLOT_WAYPOINTS 마지막 WP = 조립 위치

        start_idx = len(current_stack)
        if start_idx > 0:
            self.get_logger().info(
                f'[ASSEMBLE RESUME] product_id={product_id}, target_slot={target_slot}, '
                f'resuming from enum_idx={start_idx} (already placed={current_stack})')
        else:
            self.get_logger().info(
                f'[ASSEMBLE START] product_id={product_id}, target_slot={target_slot}, steps={len(sequence)}')

        for enum_idx, step in enumerate(sequence):
            if enum_idx < start_idx:
                continue
            # int 형식: material_id만, layer=enumerate index, x_offset=0
            # dict 형식: {'id', 'layer', 'x'} — 같은 layer에 여러 블록 배치 가능
            if isinstance(step, dict):
                material_id = step['id']
                place_layer  = step['layer']
                x_offset     = step.get('x', 0.0)
            else:
                material_id = step
                place_layer  = enum_idx
                x_offset     = 0.0

            self.get_logger().info(
                f'[ASSEMBLE] enum={enum_idx}, material_id={material_id}, '
                f'place_layer={place_layer}, x_offset={x_offset}')

            # 1. 카고에서 재료 슬롯 확인
            res = self.call_cargo('FIND_OBJECT', object_id=material_id)
            if not res or not res.success:
                self.get_logger().error(
                    f'[AMR] material {material_id} not found in cargo')
                self.go_home()
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': product_id,
                    'message': f'material {material_id} not found in cargo at enum={enum_idx}',
                }
            slot = res.slot
            cargo_layer = res.layer_index
            self.get_logger().info(
                f'[ASSEMBLE] material_id={material_id} -> slot={slot}, cargo_layer={cargo_layer}')

            # 2. 그리퍼 열기
            if not self.call_gripper(False):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'gripper open failed at enum={enum_idx}',
                }

            # 3. 재료 슬롯으로 이동
            if enum_idx == start_idx:
                # 이번 호출에서 실제로 처음 수행되는 스텝: 이동 포즈에서 웨이포인트 경유
                # (재개(resume)인 경우에도, 로봇은 이 호출 시작 시점에 조립 위치 근처에
                #  있다는 보장이 없으므로 안전하게 웨이포인트를 거쳐간다.)
                if not self.move_to_slot(slot, for_unload=True, layer_index=cargo_layer):
                    self.return_from_slot(slot, for_unload=True)
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'move to slot={slot} failed at enum={enum_idx}',
                    }
            else:
                # 이후 재료: 조립위치에서 직접 이동 (웨이포인트 없음)
                slot_joint = UNLOAD_SLOT_JOINTS.get(slot * 10 + cargo_layer)
                if slot_joint is None:
                    self.get_logger().error(
                        f'[AMR] no unload slot joint for slot={slot} cargo_layer={cargo_layer}')
                    self.return_from_slot(slot, for_unload=True)
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'no assembly slot joint for slot={slot} cargo_layer={cargo_layer}',
                    }
                if not self.move_j_checked(
                    slot_joint, label=f'assemble to slot={slot} cargo_layer={cargo_layer}'
                ):
                    self.return_from_slot(slot, for_unload=True)
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'move to slot={slot} failed at enum={enum_idx}',
                    }

            # 5. Z 하강
            if not self.move_l_rel_checked(
                [0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                label=f'assemble slot={slot} z down',
            ):
                self.return_from_slot(slot, for_unload=True)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'slot z down failed at enum={enum_idx}',
                }

            # 6. 그리퍼 grip
            if not self.call_gripper(True):
                self.get_logger().error(
                    f'[AMR] assemble grip failed at slot={slot}')
                self.move_l_rel_checked(
                    [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                    label='retreat after grip failure',
                )
                self.return_from_slot(slot, for_unload=True)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'grip failed at enum={enum_idx}',
                }

            # 7. Z 상승
            if not self.move_l_rel_checked(
                [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                label=f'assemble slot={slot} z up',
            ):
                self.return_from_slot(slot, for_unload=True)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'slot z up failed at enum={enum_idx}',
                }

            # 8. 카고 슬롯 비우기
            res = self.call_cargo('CLEAR', slot=slot, object_id=material_id)
            if not res or not res.success:
                self.get_logger().error('[AMR] cargo CLEAR failed')
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'cargo CLEAR failed at enum={enum_idx}',
                }

            # 9. 조립 위치로 직접 이동 (경유 없이 assembly_joint 로)
            if not self.move_j_checked(
                assembly_joint,
                label=f'assemble return to assembly position enum={enum_idx}',
            ):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'return to assembly position failed at enum={enum_idx}',
                }

            # 10. place_layer 기준 z 하강 거리 계산 (높은 층일수록 덜 내려감)
            z_down = ASSEMBLY_Z_DOWN_MM - (BLOCK_H_MM * place_layer)

            # 10a. X 오프셋 이동 (dict 형식에서 같은 layer 내 위치 분리)
            if abs(x_offset) > 1e-6:
                if not self.move_l_rel_checked(
                    [x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                    label=f'assemble x offset={x_offset} enum={enum_idx}',
                    ref_frame=rb.ReferenceFrame.Tool,
                ):
                    self.return_from_slot(target_slot)
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'assembly x offset failed at enum={enum_idx}',
                    }

            # 10b. Z 하강 (Tool 기준)
            if not self.move_l_rel_checked(
                [0.0, 0.0, z_down, 0.0, 0.0, 0.0],
                label=f'assemble place z down place_layer={place_layer}',
                ref_frame=rb.ReferenceFrame.Tool,
            ):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'assembly z down failed at enum={enum_idx}',
                }

            # 11. 그리퍼 열기 (블록 내려놓기)
            if not self.call_gripper(False):
                self.get_logger().error('[AMR] assembly gripper open failed')
                self.move_l_rel_checked(
                    [0.0, 0.0, -z_down, 0.0, 0.0, 0.0],
                    label='retreat z after assembly open failure',
                    ref_frame=rb.ReferenceFrame.Tool,
                )
                if abs(x_offset) > 1e-6:
                    self.move_l_rel_checked(
                        [-x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                        label='retreat x after assembly open failure',
                        ref_frame=rb.ReferenceFrame.Tool,
                    )
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'assembly gripper open failed at enum={enum_idx}',
                }

            # 12. Z 상승 (Tool 기준)
            if not self.move_l_rel_checked(
                [0.0, 0.0, -z_down, 0.0, 0.0, 0.0],
                label=f'assemble place z up place_layer={place_layer}',
                ref_frame=rb.ReferenceFrame.Tool,
            ):
                self.return_from_slot(target_slot)
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'assembly z up failed at enum={enum_idx}',
                }

            # 12a. X 오프셋 복귀 (assembly_joint 기준 위치로 되돌림)
            if abs(x_offset) > 1e-6:
                if not self.move_l_rel_checked(
                    [-x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                    label=f'assemble x return enum={enum_idx}',
                    ref_frame=rb.ReferenceFrame.Tool,
                ):
                    self.return_from_slot(target_slot)
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'assembly x return failed at enum={enum_idx}',
                    }

            # 12b. 카고 기록: target_slot에 이 재료를 놓았음을 반영한다.
            # 조립이 도중에 실패해도 target_slot의 재료 스택이 곧 진행 상황이 되어,
            # 다음 ASSEMBLE 요청에서 이 지점부터 재개할 수 있다.
            res = self.call_cargo('SET', slot=target_slot, object_id=material_id)
            if not res or not res.success:
                self.get_logger().error(
                    f'[AMR] cargo SET (assembly progress) failed at enum={enum_idx}')
                return {
                    'success': False,
                    'slot': target_slot,
                    'object_id': product_id,
                    'message': f'material placed physically but cargo SET failed at enum={enum_idx}',
                }

        # 13. 조립 완료: target_slot에 낱개로 기록되어 있던 재료들을 지우고
        #     완성품 하나로 등록한다.
        for material_id in expected_ids:
            res = self.call_cargo('CLEAR', slot=target_slot, object_id=material_id)
            if not res or not res.success:
                self.get_logger().error(
                    f'[AMR] cargo CLEAR (assembly material {material_id}) failed')
                return {
                    'success': False,
                    'slot': target_slot,
                    'object_id': product_id,
                    'message': f'assembled physically but cargo CLEAR of material {material_id} failed',
                }

        res = self.call_cargo('SET', slot=target_slot, object_id=product_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo SET for assembled product failed')
            return {
                'success': False,
                'slot': target_slot,
                'object_id': product_id,
                'message': 'assembled physically but cargo SET failed',
            }

        self.get_logger().info(f'[ASSEMBLE DONE] product_id={product_id}, slot={target_slot}')
        return {
            'success': True,
            'slot': target_slot,
            'object_id': product_id,
            'message': 'assemble success',
        }

    def sequence_assemble_ice_cream(self):
        """48132 아이스크림 전용 조립 시퀀스.

        베이스    4(2x2노랑) -> 조립슬롯(7/8) layer0
        2층       8(4x2노랑) -> 조립슬롯 layer1
        캡 준비   빈 재료슬롯(cap_slot)에 3(2x2파랑) -> 1(2x2빨강) 순서로 쌓고,
                  2(2x2초록)를 cap_slot의 고정 layer_index(ICE_CREAM_CAP_LAYER_INDEX)
                  위치에서 z오프셋(ICE_CREAM_CAP_Z_OFFSET_MM)만큼 덜 내려가 결합한다.
                  그리퍼는 놓지 않고 캡(3+1+2) 전체를 그대로 든다.
        결합      캡을 조립슬롯의 4+8 위(layer3)로 옮겨 얹고 그리퍼를 연다.

        주의: BigTree와 마찬가지로 target_slot에 재료를 놓을 때마다 cargo SET을
        하지 않으므로 중단 후 재개 판정 대상이 아니다. 그래서 슬롯 배정은 항상
        "빈 조립 슬롯 찾기"만 한다 (resolve_assembly_slot에 expected_ids를 넘기지 않음).
        """
        product_id = 48132

        if not self.is_robot_ready():
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'robot not connected'}

        target_slot, _ = self.resolve_assembly_slot(product_id)
        if target_slot is None:
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'no available assembly slot (7, 8 occupied by other builds)'}

        assembly_wps = SLOT_WAYPOINTS.get(target_slot)
        if assembly_wps is None:
            self.get_logger().error(f'[ICE_CREAM] no slot waypoints for target_slot={target_slot}')
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'no slot waypoints for target_slot={target_slot}'}
        assembly_joint = assembly_wps[-1]

        self.get_logger().info(f'[ICE_CREAM START] product_id={product_id}, target_slot={target_slot}')

        # ── 베이스: 4(2x2노랑) 파지 -> 조립슬롯 layer0 ──────────────────────
        res = self.call_cargo('FIND_OBJECT', object_id=4)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] base: 4 not found')
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: 4 not found'}
        slot_4, layer_4 = res.slot, res.layer_index

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: gripper open failed'}

        if not self.move_to_slot(slot_4, for_unload=True, layer_index=layer_4):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: move to slot=4 failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='base 4 z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: 4 z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='base 4 retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: 4 grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='base 4 z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: 4 z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_4, object_id=4)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_4, 'object_id': product_id, 'message': 'base: cargo CLEAR 4 failed'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='base to assembly_joint transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'base: move to target_slot load pos failed'}

        if not self.move_j_checked(assembly_joint, label='base to assembly_joint'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'base: move to target_slot load pos failed'}

        if not self.move_l_rel_checked([0.0, 0.0, LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='base place z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: place z down failed'}

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='base place retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='base place z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'base: place z up failed'}

        # ── 2층: 8(4x2노랑) 파지 -> 조립슬롯 layer1 ──────────────────────────
        res = self.call_cargo('FIND_OBJECT', object_id=8)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] floor2: 8 not found')
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: 8 not found'}
        slot_8, layer_8 = res.slot, res.layer_index

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: gripper open failed'}

        slot_joint_8 = UNLOAD_SLOT_JOINTS.get(slot_8 * 10 + layer_8)
        if slot_joint_8 is None:
            self.get_logger().error(f'[ICE_CREAM] floor2: no unload joint for slot={slot_8} layer={layer_8}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'floor2: no unload joint for slot={slot_8} layer={layer_8}'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='floor2 to slot transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': f'floor2: move to slot={slot_8} failed'}

        if not self.move_j_checked(slot_joint_8, label=f'floor2 to slot={slot_8}'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': f'floor2: move to slot={slot_8} failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='floor2 8 z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: 8 z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='floor2 8 retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: 8 grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='floor2 8 z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: 8 z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_8, object_id=8)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_8, 'object_id': product_id, 'message': 'floor2: cargo CLEAR 8 failed'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='floor2 to assembly_joint transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: move to assembly_joint failed'}

        if not self.move_j_checked(assembly_joint, label='floor2 to assembly_joint'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: move to assembly_joint failed'}

        z_floor2 = ASSEMBLY_Z_DOWN_MM - BLOCK_H_MM * 1
        if not self.move_l_rel_checked([0.0, 0.0, z_floor2, 0.0, 0.0, 0.0],
                                        label='floor2 place z down', ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: place z down failed'}

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -z_floor2, 0.0, 0.0, 0.0],
                                     label='floor2 place retreat', ref_frame=rb.ReferenceFrame.Tool)
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -z_floor2, 0.0, 0.0, 0.0],
                                        label='floor2 place z up', ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'floor2: place z up failed'}

        # ── 캡 준비: 빈 재료슬롯(cap_slot) 찾기 ──────────────────────────────
        cap_slot = self.find_empty_material_slot()
        if cap_slot is None:
            self.get_logger().error('[ICE_CREAM] cap: no empty material slot for staging')
            return {'success': False, 'slot': target_slot, 'object_id': product_id,
                    'message': 'cap: no empty material slot for staging'}
        cap_load_joint = LOAD_SLOT_JOINTS[cap_slot]

        # ── 캡 1: 3(2x2파랑) -> cap_slot (LOAD 경로, 바닥) ──────────────────
        res = self.call_cargo('FIND_OBJECT', object_id=3)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] cap: 3 not found')
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 not found'}
        slot_3, layer_3 = res.slot, res.layer_index

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: gripper open failed (3)'}

        slot_joint_3 = UNLOAD_SLOT_JOINTS.get(slot_3 * 10 + layer_3)
        if slot_joint_3 is None:
            self.get_logger().error(f'[ICE_CREAM] cap: no unload joint for slot={slot_3} layer={layer_3}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'cap: no unload joint for slot={slot_3} layer={layer_3}'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap 3 to slot transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to slot=3 source failed'}

        if not self.move_j_checked(slot_joint_3, label=f'cap 3 to slot={slot_3}'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to slot=3 source failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 3 z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='cap 3 retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='cap 3 z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_3, object_id=3)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_3, 'object_id': product_id, 'message': 'cap: cargo CLEAR 3 failed'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap 3 to cap_slot transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'cap: move to cap_slot load pos failed (3)'}

        if not self.move_j_checked(cap_load_joint, label='cap 3 to cap_slot'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'cap: move to cap_slot load pos failed (3)'}

        if not self.move_l_rel_checked([0.0, 0.0, LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 3 place z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 place z down failed'}

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 3 place retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 3 place z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 3 place z up failed'}

        res = self.call_cargo('SET', slot=cap_slot, object_id=3)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] cargo SET (cap 3) failed')
            return {'success': False, 'slot': cap_slot, 'object_id': product_id, 'message': 'cap: cargo SET 3 failed'}

        # ── 캡 2: 1(2x2빨강) -> cap_slot, 3 위에 쌓기 ───────────────────────
        res = self.call_cargo('FIND_OBJECT', object_id=1)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] cap: 1 not found')
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 not found'}
        slot_1, layer_1 = res.slot, res.layer_index

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: gripper open failed (1)'}

        slot_joint_1 = UNLOAD_SLOT_JOINTS.get(slot_1 * 10 + layer_1)
        if slot_joint_1 is None:
            self.get_logger().error(f'[ICE_CREAM] cap: no unload joint for slot={slot_1} layer={layer_1}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'cap: no unload joint for slot={slot_1} layer={layer_1}'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap 1 to slot transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to slot=1 source failed'}

        if not self.move_j_checked(slot_joint_1, label=f'cap 1 to slot={slot_1}'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to slot=1 source failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 1 z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='cap 1 retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='cap 1 z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_1, object_id=1)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_1, 'object_id': product_id, 'message': 'cap: cargo CLEAR 1 failed'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap 1 to cap_slot transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'cap: move to cap_slot load pos failed (1)'}

        if not self.move_j_checked(cap_load_joint, label='cap 1 to cap_slot'):  # cap_slot의 기존 3 위에 쌓임
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'cap: move to cap_slot load pos failed (1)'}

        if not self.move_l_rel_checked([0.0, 0.0, LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 1 place z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 place z down failed'}

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 1 place retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 1 place z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 1 place z up failed'}

        res = self.call_cargo('SET', slot=cap_slot, object_id=1)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] cargo SET (cap 1) failed')
            return {'success': False, 'slot': cap_slot, 'object_id': product_id, 'message': 'cap: cargo SET 1 failed'}

        # ── 캡 3: 2(2x2초록) -> cap_slot의 고정 layer(=ICE_CREAM_CAP_LAYER_INDEX)에 결합,
        #          그리퍼는 놓지 않고 캡(3+1+2) 전체를 든다 ──────────────────
        res = self.call_cargo('FIND_OBJECT', object_id=2)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] cap: 2 not found')
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 2 not found'}
        slot_2, layer_2 = res.slot, res.layer_index

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: gripper open failed (2)'}

        slot_joint_2 = UNLOAD_SLOT_JOINTS.get(slot_2 * 10 + layer_2)
        if slot_joint_2 is None:
            self.get_logger().error(f'[ICE_CREAM] cap: no unload joint for slot={slot_2} layer={layer_2}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'cap: no unload joint for slot={slot_2} layer={layer_2}'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap 2 to slot transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to slot=2 source failed'}

        if not self.move_j_checked(slot_joint_2, label=f'cap 2 to slot={slot_2}'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to slot=2 source failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0], label='cap 2 z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 2 z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='cap 2 retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 2 grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0], label='cap 2 z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: 2 z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_2, object_id=2)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_2, 'object_id': product_id, 'message': 'cap: cargo CLEAR 2 failed'}

        cap_joint = UNLOAD_SLOT_JOINTS.get(cap_slot * 10 + ICE_CREAM_CAP_LAYER_INDEX)
        if cap_joint is None:
            self.get_logger().error(
                f'[ICE_CREAM] cap: no joint for cap_slot={cap_slot} layer={ICE_CREAM_CAP_LAYER_INDEX}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'cap: no joint for cap_slot={cap_slot} layer={ICE_CREAM_CAP_LAYER_INDEX}'}

        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap 2 to cap combine joint transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to cap combine joint failed'}

        if not self.move_j_checked(cap_joint, label='cap 2 to cap combine joint'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to cap combine joint failed'}

        z_cap = ASSEMBLY_Z_DOWN_MM - ICE_CREAM_CAP_Z_OFFSET_MM
        if not self.move_l_rel_checked([0.0, 0.0, z_cap, 0.0, 0.0, 0.0],
                                        label='cap 2 combine z down', ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: combine z down failed'}

        # 그리퍼는 유지한다 (캡 3+1+2 통째로 들고 이동) — cap_slot에 잠깐 등록해둔 3, 1을 정리
        res = self.call_cargo('CLEAR', slot=cap_slot, object_id=3)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': cap_slot, 'object_id': product_id, 'message': 'cap: cargo CLEAR 3 (cap_slot) failed'}

        res = self.call_cargo('CLEAR', slot=cap_slot, object_id=1)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': cap_slot, 'object_id': product_id, 'message': 'cap: cargo CLEAR 1 (cap_slot) failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -z_cap, 0.0, 0.0, 0.0],
                                        label='cap lift', ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: lift failed'}

        # ── 결합: 캡(3+1+2)을 조립슬롯의 4+8 위(layer3)에 얹기 ────────────────
        if not self.move_j_checked(ASSEMBLY_TRANSIT_JOINT_DEG, label='cap to assembly_joint transit'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to assembly_joint failed'}

        if not self.move_j_checked(assembly_joint, label='cap to assembly_joint'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: move to assembly_joint failed'}

        z_final = ASSEMBLY_Z_DOWN_MM - BLOCK_H_MM * 3
        if not self.move_l_rel_checked([0.0, 0.0, z_final, 0.0, 0.0, 0.0],
                                        label='cap final z down', ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: final z down failed'}

        if not self.call_gripper(False):
            self.get_logger().error('[ICE_CREAM] final release failed')
            self.move_l_rel_checked([0.0, 0.0, -z_final, 0.0, 0.0, 0.0],
                                     label='cap final retreat', ref_frame=rb.ReferenceFrame.Tool)
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: final release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -z_final, 0.0, 0.0, 0.0],
                                        label='cap final z up', ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'cap: final z up failed'}

        # ── cargo 등록 ────────────────────────────────────────────────────────
        res = self.call_cargo('SET', slot=target_slot, object_id=product_id)
        if not res or not res.success:
            self.get_logger().error('[ICE_CREAM] cargo SET failed')
            return {'success': False, 'slot': target_slot, 'object_id': product_id,
                    'message': 'assembled but cargo SET failed'}

        self.get_logger().info(f'[ASSEMBLE DONE] ice_cream product_id={product_id}, slot={target_slot}')
        return {'success': True, 'slot': target_slot, 'object_id': product_id, 'message': 'assemble success'}

    def sequence_assemble_big_tree(self):
        """46262 BigTree 전용 조립 시퀀스.

        slot 7에는 4를 베이스로 놓고,
        slot 8에는 6(x=-17), 2(x=32), 6(layer1)을 부분조립한 뒤
        그 6+2+6 묶음을 통째로 들어 slot 7 위로 합친다.
        마지막 2는 slot 7 상단에 별도로 올린다.
        """
        product_id = 46262

        if not self.is_robot_ready():
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'robot not connected'}

        self.get_logger().info(f'[BIG_TREE START] product_id={product_id}')

        for slot in (BIG_TREE_BASE_SLOT, BIG_TREE_STAGE_SLOT):
            res = self.call_cargo('FIND_SLOT_STACK', slot=slot)
            if not res or not res.success:
                return {
                    'success': False, 'slot': slot, 'object_id': product_id,
                    'message': f'big_tree: FIND_SLOT_STACK failed for slot={slot}',
                }
            if list(res.stack):
                return {
                    'success': False, 'slot': slot, 'object_id': product_id,
                    'message': f'big_tree: slot={slot} is not empty',
                }

        placements = [
            {'material_id': 4, 'target_slot': BIG_TREE_BASE_SLOT, 'x_offset': BIG_TREE_BASE_X_MM,
             'z_down_mm': ASSEMBLY_Z_DOWN_MM, 'cargo_set_ids': (4,), 'exclude_slot': None,
             'label': 'big_tree base 4'},
            {'material_id': 6, 'target_slot': BIG_TREE_STAGE_SLOT, 'x_offset': BIG_TREE_STAGE_LEFT_X_MM,
             'z_down_mm': ASSEMBLY_Z_DOWN_MM, 'cargo_set_ids': (6,), 'exclude_slot': None,
             'label': 'big_tree stage 6 left'},
            {'material_id': 2, 'target_slot': BIG_TREE_STAGE_SLOT, 'x_offset': BIG_TREE_STAGE_RIGHT_X_MM,
             'z_down_mm': ASSEMBLY_Z_DOWN_MM, 'cargo_set_ids': (2,), 'exclude_slot': None,
             'label': 'big_tree stage 2 right'},
            {'material_id': 6, 'target_slot': BIG_TREE_STAGE_SLOT, 'x_offset': BIG_TREE_STAGE_MID_X_MM,
             'z_down_mm': ASSEMBLY_Z_DOWN_MM - BLOCK_H_MM * BIG_TREE_STAGE_LAYER_TOP,
             'cargo_set_ids': (6,), 'exclude_slot': BIG_TREE_STAGE_SLOT, 'label': 'big_tree stage 6 top'},
        ]

        for idx, step in enumerate(placements):
            pick_res = self.pick_material_for_assembly(
                step['material_id'], step['label'], exclude_slot=step['exclude_slot'],
                direct=(idx != 0),  # 시퀀스의 맨 첫 픽업(베이스 4)만 전체 웨이포인트로 안전하게 진입
            )
            if not pick_res['success']:
                return {'success': False, 'slot': pick_res.get('slot', -1),
                        'object_id': product_id, 'message': pick_res['message']}

            place_res = self.place_gripped_to_assembly_slot(
                step['target_slot'], step['z_down_mm'], step['label'],
                x_offset=step['x_offset'], cargo_set_ids=step['cargo_set_ids'],
            )
            if not place_res['success']:
                return {'success': False, 'slot': place_res.get('slot', -1),
                        'object_id': product_id, 'message': place_res['message']}

        pickup_res = self.pickup_big_tree_stage_group_from_slot8()
        if not pickup_res['success']:
            return {'success': False, 'slot': pickup_res.get('slot', -1),
                    'object_id': product_id, 'message': pickup_res['message']}

        merge_res = self.place_gripped_to_assembly_slot(
            BIG_TREE_BASE_SLOT, BIG_TREE_STAGE_MERGE_Z_DOWN_MM, 'big_tree merge stage onto base',
            x_offset=BIG_TREE_MERGE_PLACE_X_MM, cargo_set_ids=BIG_TREE_STAGE_GROUP_IDS,
        )
        if not merge_res['success']:
            return {'success': False, 'slot': merge_res.get('slot', -1),
                    'object_id': product_id, 'message': merge_res['message']}

        top2_pick_res = self.pick_material_for_assembly(2, 'big_tree top 2', exclude_slot=BIG_TREE_BASE_SLOT)
        if not top2_pick_res['success']:
            return {'success': False, 'slot': top2_pick_res.get('slot', -1),
                    'object_id': product_id, 'message': top2_pick_res['message']}

        top2_place_res = self.place_gripped_to_assembly_slot(
            BIG_TREE_BASE_SLOT, BIG_TREE_TOP_2_Z_DOWN_MM, 'big_tree top 2 place',
            x_offset=BIG_TREE_TOP_2_X_MM, cargo_set_ids=(2,),
        )
        if not top2_place_res['success']:
            return {'success': False, 'slot': top2_place_res.get('slot', -1),
                    'object_id': product_id, 'message': top2_place_res['message']}

        for material_id in BIG_TREE_FINAL_STACK_IDS:
            res = self.call_cargo('CLEAR', slot=BIG_TREE_BASE_SLOT, object_id=material_id)
            if not res or not res.success:
                self.return_from_slot(BIG_TREE_BASE_SLOT)
                return {'success': False, 'slot': BIG_TREE_BASE_SLOT, 'object_id': product_id,
                        'message': f'big_tree: cargo CLEAR {material_id} failed during finalization'}

        res = self.call_cargo('SET', slot=BIG_TREE_BASE_SLOT, object_id=product_id)
        if not res or not res.success:
            self.return_from_slot(BIG_TREE_BASE_SLOT)
            return {'success': False, 'slot': BIG_TREE_BASE_SLOT, 'object_id': product_id,
                    'message': 'big_tree: cargo SET failed during finalization'}

        self.get_logger().info(f'[ASSEMBLE DONE] big_tree product_id={product_id}, slot={BIG_TREE_BASE_SLOT}')
        return {'success': True, 'slot': BIG_TREE_BASE_SLOT, 'object_id': product_id, 'message': 'assemble success'}


def main(args=None):
    rclpy.init(args=args)
    node = AmrRobotNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()