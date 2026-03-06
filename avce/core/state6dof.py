"""
6-DOF 선체 상태 (AVCE §6DOF).

VehicleState  : 12D 전 운항 상태 (위치 + 자세 + 속도 + 각속도).
IceLayer      : 3D 빙층 (수면 위/아래 모두 표현).
OceanCell     : 해양 환경 격자 (수온·밀도·조류).

설계 원칙
──────────
- VehicleState : 기존 VesselState (3-DOF) 완전 포함.
  to_vessel_state() / from_vessel_state() 변환 제공.
- IceLayer     : IceCell(2D) 상위 개념. 깊이 z_top/z_bottom 추가.
- OceanCell    : 조류·수온·염분 → 환경 외란 τ_env 계산에 사용.
- frozen 없음: VehicleState는 시뮬 루프에서 매 스텝 갱신됨.

좌표계 (NED — North-East-Down)
──────────────────────────────
  ξ (North+), η (East+), z (Down+)
  z > 0 : 수면 아래 (잠수)
  z < 0 : 수면 위 (공중, 이론상)

Euler 자세각
────────────
  φ (roll):  우현 기울 → 양수
  θ (pitch): 선수 올림 → 양수 (잠수 시 하강 → 음수)
  ψ (yaw):   북에서 시계 방향 → 양수
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .state import VesselState


# ── 12D 선체 상태 ─────────────────────────────────────────────────────────────

@dataclass
class VehicleState:
    """12D 선체 상태 (6-DOF 위치+자세 + 6-DOF 속도).

    NED 좌표계. z > 0 이면 수면 하부 (잠수 상태).

    Attributes
    ----------
    xi_m, eta_m : float
        수평 위치 [m] (North, East).
    z_m : float
        수심 [m]. 양수=잠수, 0=수면, 음수=수면 위.
    phi_rad : float
        롤각 [rad].
    theta_rad : float
        피치각 [rad]. 양수=선수 올림.
    psi_rad : float
        방위각 [rad]. VesselState.psi_rad 와 동일.
    u_ms, v_ms, w_ms : float
        Body-frame 속도: surge, sway, heave [m/s].
    p_rads, q_rads, r_rads : float
        Body-frame 각속도: roll, pitch, yaw [rad/s].
    """
    # 위치
    xi_m:     float = 0.0
    eta_m:    float = 0.0
    z_m:      float = 0.0
    # 자세 (Euler)
    phi_rad:   float = 0.0
    theta_rad: float = 0.0
    psi_rad:   float = 0.0
    # Body-frame 선속도
    u_ms:     float = 0.0
    v_ms:     float = 0.0
    w_ms:     float = 0.0
    # Body-frame 각속도
    p_rads:   float = 0.0
    q_rads:   float = 0.0
    r_rads:   float = 0.0

    # ── 파생 속성 ──────────────────────────────────────────────────────────

    @property
    def U_ms(self) -> float:
        """3D 합산 선속 [m/s]."""
        return math.sqrt(self.u_ms**2 + self.v_ms**2 + self.w_ms**2)

    @property
    def U_horizontal_ms(self) -> float:
        """수평 속력 [m/s] (surge + sway 합성)."""
        return math.sqrt(self.u_ms**2 + self.v_ms**2)

    @property
    def is_submerged(self) -> bool:
        """수면 하부 여부 (z > 0.5m)."""
        return self.z_m > 0.5

    def heading_deg(self) -> float:
        """방위각 [deg]."""
        return math.degrees(self.psi_rad)

    def depth_m(self) -> float:
        """잠수 깊이 [m]. z_m 의 별칭."""
        return self.z_m

    # ── 변환 ───────────────────────────────────────────────────────────────

    def to_vessel_state(self) -> VesselState:
        """3-DOF VesselState 변환 (수평 성분만). 기존 코드 호환용."""
        return VesselState(
            xi_m=self.xi_m,
            eta_m=self.eta_m,
            psi_rad=self.psi_rad,
            u_ms=self.u_ms,
            v_ms=self.v_ms,
            r_rads=self.r_rads,
        )

    @classmethod
    def from_vessel_state(
        cls,
        s: VesselState,
        z_m: float = 0.0,
        phi_rad: float = 0.0,
        theta_rad: float = 0.0,
        w_ms: float = 0.0,
        p_rads: float = 0.0,
        q_rads: float = 0.0,
    ) -> "VehicleState":
        """3-DOF VesselState → 6-DOF VehicleState 확장."""
        return cls(
            xi_m=s.xi_m,
            eta_m=s.eta_m,
            z_m=z_m,
            phi_rad=phi_rad,
            theta_rad=theta_rad,
            psi_rad=s.psi_rad,
            u_ms=s.u_ms,
            v_ms=s.v_ms,
            w_ms=w_ms,
            p_rads=p_rads,
            q_rads=q_rads,
            r_rads=s.r_rads,
        )

    def copy(self) -> "VehicleState":
        """얕은 복사 (불변 시뮬 패턴용)."""
        return VehicleState(
            xi_m=self.xi_m, eta_m=self.eta_m, z_m=self.z_m,
            phi_rad=self.phi_rad, theta_rad=self.theta_rad, psi_rad=self.psi_rad,
            u_ms=self.u_ms, v_ms=self.v_ms, w_ms=self.w_ms,
            p_rads=self.p_rads, q_rads=self.q_rads, r_rads=self.r_rads,
        )


# ── 3D 빙층 ───────────────────────────────────────────────────────────────────

@dataclass
class IceLayer:
    """3D 빙층 셀 (수평 위치 + 수직 범위).

    기존 IceCell(2D) 상위 개념.
    z_top_m < z_bottom_m 관계 (NED: 아래가 양수).

    Examples
    --------
    수면 빙  : z_top_m = -0.5,  z_bottom_m =  2.5  (빙하 2m, 수면 위 0.5m 노출)
    빙하 아래: z_top_m =  0.5,  z_bottom_m =  3.0  (수면 아래 빙)
    """
    xi_m:        float         # 격자 중심 North [m]
    eta_m:       float         # 격자 중심 East  [m]
    z_top_m:     float         # 빙 상면 깊이 [m]   (음수=수면 위, 양수=수면 아래)
    z_bottom_m:  float         # 빙 하면 깊이 [m]
    C_ice:       float = 1.0   # 빙 농도 [0~1]
    sigma_m:     float = 200.0 # 수평 Gaussian 반경 [m]
    hardness:    float = 1.0   # 경도 계수 [0~1] (1=다년빙, 0=신생빙)

    @property
    def thickness_m(self) -> float:
        """빙 두께 [m]."""
        return max(0.0, self.z_bottom_m - self.z_top_m)

    @property
    def is_surface_ice(self) -> bool:
        """수면 위 빙 (쇄빙선 대상)."""
        return self.z_top_m <= 0.0

    @property
    def is_ceiling_ice(self) -> bool:
        """수면 하부 빙 천장 (잠수함 대상)."""
        return self.z_top_m > 0.0

    def at_depth(self, z_m: float) -> bool:
        """주어진 깊이에서 이 빙층이 존재하는지 확인."""
        return self.z_top_m <= z_m <= self.z_bottom_m


# ── 해양 환경 격자 셀 ─────────────────────────────────────────────────────────

@dataclass
class OceanCell:
    """해양 환경 격자 셀 (수온·밀도·조류).

    환경 외란 τ_env 계산에 사용.
    조류 (current_u/v/w) 는 NED 글로벌 좌표계 기준.
    """
    xi_m:        float         # 격자 중심 [m]
    eta_m:       float         # 격자 중심 [m]
    z_m:         float         # 기준 깊이 [m]
    sigma_m:     float = 500.0 # 영향 반경 [m]

    # 해양 파라미터
    temp_c:      float = 4.0   # 수온 [°C]
    salinity_ppt: float = 35.0 # 염분 [ppt]
    density_kgm3: float = 1025.0  # 밀도 [kg/m³]

    # 조류 속도 (NED 글로벌)
    current_u_ms: float = 0.0  # North 방향 조류 [m/s]
    current_v_ms: float = 0.0  # East  방향 조류 [m/s]
    current_w_ms: float = 0.0  # Down  방향 조류 [m/s]

    @property
    def current_speed_ms(self) -> float:
        """조류 합산 속력 [m/s]."""
        return math.sqrt(
            self.current_u_ms**2 +
            self.current_v_ms**2 +
            self.current_w_ms**2
        )
