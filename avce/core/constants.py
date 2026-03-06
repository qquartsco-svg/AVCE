"""상수 (EQUATIONS.md §9, PRECISION_CONTROL_ANALYSIS)"""
from math import pi

# 퍼텐셜 필드
DEFAULT_K_GOAL = 0.5
DEFAULT_K_OBS = 1.0
RHO_0_DEFAULT = 500.0  # m, 장애물 영향 반경
GAUSSIAN_SIGMA_MIN = 50.0  # m

# 정밀도 (OrbitStabilizer/기술 백서 정렬)
PRECISION_DEG = 0.1  # 도, 위상 정밀도
PRECISION_RAD = PRECISION_DEG * pi / 180.0
PREDICTION_HORIZON_MS = 100.0  # ms
FAILSAFE_MS = 0.1

# 선박 3-DOF 단위
UNIT_POSITION_M = 1.0
UNIT_ANGLE_RAD = 1.0
UNIT_VELOCITY_MS = 1.0
UNIT_ANGULAR_RADS = 1.0

# 맥락 라벨 (쇄빙선)
CONTEXT_OPEN_WATER = "open_water"
CONTEXT_ICE_TRANSIT = "ice_transit"
CONTEXT_ICE_RAM = "ice_ram"
