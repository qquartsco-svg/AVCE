"""
운항 모드 정의 (AVCE §MODE).

VesselMode   : 운항 모드 Enum (8종).
ModeConfig   : 모드별 물리·제어 파라미터 묶음 (frozen).
ModeProfile  : 모드 전환 규칙 집합 (안전 임계값).

운항 모드 체계
──────────────────────────────────────────────────────────
SURFACE              수상 항행        3-DOF,  빙 없음
ICEBREAKER           수상 쇄빙        3-DOF,  표면 빙 파쇄
DIVE                 잠수 항행        6-DOF,  개방 해역
UNDER_ICE            빙하 하부 항행   6-DOF,  빙 천장 회피
SUBMERGED_ICEBREAKER 잠수 중 상향 파쇄 6-DOF, 빙 천장 파쇄
TRANSIT_DIVE         잠수 전환 중     6-DOF,  z 가속 구간
TRANSIT_SURFACE      부상 전환 중     6-DOF,  z 감속 구간
EMERGENCY_SURFACE    비상 부상        6-DOF,  최대 부력 + 수직 상승

설계 원칙
──────────
- VesselMode(str, Enum): 문자열 직렬화 가능.
- ModeConfig(frozen=True): 모드별 파라미터 불변.
- ModeProfile(frozen=True): 전환 임계값 불변.
- 하드코딩 없음: 전 파라미터 주입형.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, Optional, Tuple


# ── 운항 모드 ──────────────────────────────────────────────────────────────────

class VesselMode(str, Enum):
    """운항 모드 (8종).

    전환 그래프
    ──────────────────────────────────────────────────
    SURFACE ←→ ICEBREAKER   (표면 빙 존재 여부)
    SURFACE → TRANSIT_DIVE → DIVE
    DIVE    → TRANSIT_SURFACE → SURFACE
    DIVE    ←→ UNDER_ICE         (빙 천장 감지)
    UNDER_ICE → SUBMERGED_ICEBREAKER  (파쇄 명령)
    * → EMERGENCY_SURFACE    (위험 감지 시 우선)
    """
    SURFACE              = "surface"              # 수상 항행   (3-DOF)
    ICEBREAKER           = "icebreaker"           # 수상 쇄빙   (3-DOF + 빙)
    DIVE                 = "dive"                 # 잠수 항행   (6-DOF)
    UNDER_ICE            = "under_ice"            # 빙하 하부   (6-DOF + 빙 천장)
    SUBMERGED_ICEBREAKER = "submerged_icebreaker" # 잠수 쇄빙   (6-DOF + 상향 파쇄)
    TRANSIT_DIVE         = "transit_dive"         # 잠수 전환   (6-DOF 과도)
    TRANSIT_SURFACE      = "transit_surface"      # 부상 전환   (6-DOF 과도)
    EMERGENCY_SURFACE    = "emergency_surface"    # 비상 부상   (최우선)

    # ── 분류 프로퍼티 ────────────────────────────────────────────────────────

    @property
    def is_surface(self) -> bool:
        return self in (VesselMode.SURFACE, VesselMode.ICEBREAKER)

    @property
    def is_submerged(self) -> bool:
        return self in (
            VesselMode.DIVE,
            VesselMode.UNDER_ICE,
            VesselMode.SUBMERGED_ICEBREAKER,
            VesselMode.TRANSIT_DIVE,
            VesselMode.TRANSIT_SURFACE,
            VesselMode.EMERGENCY_SURFACE,
        )

    @property
    def is_6dof(self) -> bool:
        """True이면 6-DOF 동역학 사용."""
        return self.is_submerged

    @property
    def uses_ice(self) -> bool:
        return self in (
            VesselMode.ICEBREAKER,
            VesselMode.UNDER_ICE,
            VesselMode.SUBMERGED_ICEBREAKER,
        )

    @property
    def is_transition(self) -> bool:
        return self in (VesselMode.TRANSIT_DIVE, VesselMode.TRANSIT_SURFACE)

    @property
    def is_emergency(self) -> bool:
        return self is VesselMode.EMERGENCY_SURFACE


# ── 모드 설정 (불변) ──────────────────────────────────────────────────────────

@dataclass(frozen=True)
class ModeConfig:
    """모드별 물리·제어 파라미터 묶음 (불변).

    Attributes
    ----------
    mode : VesselMode
        적용 모드.
    max_depth_m : float
        허용 최대 잠수 깊이 [m]. 수상 모드=0.
    max_speed_ms : float
        허용 최대 선속 [m/s].
    max_pitch_rad : float
        허용 최대 피치각 [rad] (잠수 기동 한계).
    max_roll_rad : float
        허용 최대 롤각 [rad].
    ram_enable : bool
        쇄빙 파쇄(Ramming) 허용 여부.
    ram_max_force_n : float
        파쇄 최대 충격력 [N]. 0=제한 없음.
    propulsion_max_fraction : float
        추진 최대 출력 비율 [0~1]. 비상 모드=1.0.
    depth_control_k : float
        깊이 P 제어 게인 [N/m].
    pitch_control_k : float
        피치 P 제어 게인 [N·m/rad].
    """
    mode:                    VesselMode
    max_depth_m:             float = 0.0
    max_speed_ms:            float = 10.0
    max_pitch_rad:           float = math.radians(30.0)
    max_roll_rad:            float = math.radians(20.0)
    ram_enable:              bool  = False
    ram_max_force_n:         float = 0.0
    propulsion_max_fraction: float = 1.0
    depth_control_k:         float = 1.0e4
    pitch_control_k:         float = 5.0e5

    def validate(self) -> None:
        """파라미터 무결성 확인 (불변 생성 후 명시적 호출)."""
        if self.max_depth_m < 0:
            raise ValueError(f"ModeConfig.max_depth_m must be ≥ 0, got {self.max_depth_m}")
        if not (0.0 < self.propulsion_max_fraction <= 1.0):
            raise ValueError(f"propulsion_max_fraction ∈ (0, 1], got {self.propulsion_max_fraction}")


# ── 모드 전환 규칙 (불변) ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class ModeProfile:
    """모드 전환 안전 임계값 (불변).

    Attributes
    ----------
    dive_speed_ms : float
        잠수 시작 허용 최소 속도 [m/s].
    surface_z_thresh_m : float
        "수면 도달" 판정 깊이 [m]. z < 이 값이면 수상.
    ice_detect_thickness_m : float
        빙 감지 최소 두께 [m]. 이하면 무시.
    ram_velocity_ms : float
        쇄빙 파쇄 시작 최소 속도 [m/s].
    emergency_depth_m : float
        비상 부상 트리거 깊이 [m]. 최대 허용치 초과 시 발동.
    pitch_limit_emerg_rad : float
        이 피치각 초과 시 비상 안정화.
    """
    dive_speed_ms:           float = 1.0
    surface_z_thresh_m:      float = 1.0
    ice_detect_thickness_m:  float = 0.1
    ram_velocity_ms:         float = 2.0
    emergency_depth_m:       float = 500.0
    pitch_limit_emerg_rad:   float = math.radians(45.0)


# ── 기본 모드 설정 테이블 ─────────────────────────────────────────────────────

def default_mode_config(mode: VesselMode) -> ModeConfig:
    """모드별 기본 ModeConfig 반환 (교체 가능한 팩토리)."""
    _TABLE = {
        VesselMode.SURFACE: ModeConfig(
            mode=VesselMode.SURFACE,
            max_depth_m=0.0, max_speed_ms=15.0,
            ram_enable=False,
        ),
        VesselMode.ICEBREAKER: ModeConfig(
            mode=VesselMode.ICEBREAKER,
            max_depth_m=0.0, max_speed_ms=8.0,
            ram_enable=True, ram_max_force_n=5.0e6,
            propulsion_max_fraction=1.0,
        ),
        VesselMode.DIVE: ModeConfig(
            mode=VesselMode.DIVE,
            max_depth_m=600.0, max_speed_ms=12.0,
            max_pitch_rad=math.radians(25.0),
            ram_enable=False,
        ),
        VesselMode.UNDER_ICE: ModeConfig(
            mode=VesselMode.UNDER_ICE,
            max_depth_m=600.0, max_speed_ms=8.0,
            max_pitch_rad=math.radians(15.0),
            ram_enable=False,
        ),
        VesselMode.SUBMERGED_ICEBREAKER: ModeConfig(
            mode=VesselMode.SUBMERGED_ICEBREAKER,
            max_depth_m=200.0, max_speed_ms=5.0,
            max_pitch_rad=math.radians(20.0),
            ram_enable=True, ram_max_force_n=3.0e6,
            propulsion_max_fraction=1.0,
        ),
        VesselMode.TRANSIT_DIVE: ModeConfig(
            mode=VesselMode.TRANSIT_DIVE,
            max_depth_m=600.0, max_speed_ms=6.0,
            max_pitch_rad=math.radians(20.0),
            ram_enable=False,
        ),
        VesselMode.TRANSIT_SURFACE: ModeConfig(
            mode=VesselMode.TRANSIT_SURFACE,
            max_depth_m=600.0, max_speed_ms=6.0,
            max_pitch_rad=math.radians(20.0),
            ram_enable=False,
        ),
        VesselMode.EMERGENCY_SURFACE: ModeConfig(
            mode=VesselMode.EMERGENCY_SURFACE,
            max_depth_m=600.0, max_speed_ms=10.0,
            max_pitch_rad=math.radians(45.0),
            max_roll_rad=math.radians(40.0),
            ram_enable=False,
            propulsion_max_fraction=1.0,
            depth_control_k=3.0e4,     # 비상 시 3배 게인
            pitch_control_k=1.5e6,
        ),
    }
    return _TABLE[mode]


# ── 허용 전환 테이블 ──────────────────────────────────────────────────────────

_ALLOWED_TRANSITIONS: dict[VesselMode, FrozenSet[VesselMode]] = {
    VesselMode.SURFACE: frozenset({
        VesselMode.ICEBREAKER,
        VesselMode.TRANSIT_DIVE,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.ICEBREAKER: frozenset({
        VesselMode.SURFACE,
        VesselMode.TRANSIT_DIVE,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.DIVE: frozenset({
        VesselMode.TRANSIT_SURFACE,
        VesselMode.UNDER_ICE,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.UNDER_ICE: frozenset({
        VesselMode.DIVE,
        VesselMode.SUBMERGED_ICEBREAKER,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.SUBMERGED_ICEBREAKER: frozenset({
        VesselMode.UNDER_ICE,
        VesselMode.DIVE,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.TRANSIT_DIVE: frozenset({
        VesselMode.DIVE,
        VesselMode.SURFACE,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.TRANSIT_SURFACE: frozenset({
        VesselMode.SURFACE,
        VesselMode.ICEBREAKER,
        VesselMode.DIVE,
        VesselMode.EMERGENCY_SURFACE,
    }),
    VesselMode.EMERGENCY_SURFACE: frozenset({
        VesselMode.SURFACE,
        VesselMode.ICEBREAKER,
    }),
}


def allowed_transitions(mode: VesselMode) -> FrozenSet[VesselMode]:
    """현재 모드에서 허용된 전환 대상 모드 집합."""
    return _ALLOWED_TRANSITIONS.get(mode, frozenset())


def can_transition(from_mode: VesselMode, to_mode: VesselMode) -> bool:
    """전환 허용 여부 확인."""
    if to_mode is VesselMode.EMERGENCY_SURFACE:
        return True   # 비상 부상은 항상 허용
    return to_mode in allowed_transitions(from_mode)
