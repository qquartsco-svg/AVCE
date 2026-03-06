"""상태·계약 (EQUATIONS.md §3, CONCEPTS §3)"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List
import math

# 단위: ξ,η [m], ψ [rad], u,v [m/s], r [rad/s]


@dataclass
class Waypoint:
    """목표 waypoint"""
    xi_m: float
    eta_m: float
    psi_d_rad: Optional[float] = None  # 목표 방위각(선택)
    U_d_ms: Optional[float] = None     # 목표 속도(선택)


@dataclass
class Obstacle:
    """장애물 반발 우물 (가우시안 또는 거리 기반)"""
    xi_m: float
    eta_m: float
    A: float = 1.0       # 반발 강도 (가우시안)
    sigma_m: float = 100.0  # 영향 반경 [m]
    rho_0_m: Optional[float] = None  # None이면 가우시안, 설정 시 거리 기반


@dataclass
class VesselState:
    """선박 3-DOF 상태 (계약)"""
    xi_m: float
    eta_m: float
    psi_rad: float
    u_ms: float
    v_ms: float
    r_rads: float

    @property
    def U_ms(self) -> float:
        """전진 속도 근사 [m/s]"""
        return math.sqrt(self.u_ms ** 2 + self.v_ms ** 2)

    def heading_deg(self) -> float:
        """방위각 [deg]"""
        return math.degrees(self.psi_rad)


@dataclass
class AxisSetpoint:
    """단일 축 setpoint (정밀 제어용)"""
    value: float   # 위상/각도 [rad] 또는 [deg] (계약에 따름)
    axis_id: str   # "psi" | "phi" | "trim" | ...


@dataclass
class IceCell:
    """빙 격자 셀 (쇄빙선 U_ice)"""
    xi_m: float
    eta_m: float
    C_ice: float  # 0~1
    sigma_m: float = 200.0
