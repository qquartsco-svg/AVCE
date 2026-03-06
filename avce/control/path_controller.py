"""경로 제어: ψ_ref, U_ref (EQUATIONS.md §3.4, §8)"""
from __future__ import annotations
import math
from typing import List, Optional, Callable
from dataclasses import dataclass

from ..core.state import VesselState, Waypoint, Obstacle, IceCell
from ..core.constants import RHO_0_DEFAULT
from .potential_field import (
    U_total,
    gradient_U,
    psi_ref_from_gradient,
)


@dataclass
class PathOutput:
    """경로 계층 출력"""
    psi_ref_rad: float
    U_ref_ms: float
    F_pf_mag: float   # |−∇U| 크기 (가중용)


def path_controller(
    state: VesselState,
    waypoint: Waypoint,
    obstacles: List[Obstacle],
    memory_wells: Optional[Callable[[float, float], float]] = None,
    ice_cells: Optional[List[IceCell]] = None,
    k_goal: float = 0.5,
    k_ice: float = 1.0,
    rho_0: float = RHO_0_DEFAULT,
    U_default_ms: float = 5.0,
    F_to_speed_gain: float = 0.0,
) -> PathOutput:
    """Potential Field → ψ_ref, U_ref.

    ψ_ref = atan2(F_eta, F_xi)   (−∇U 방향).
    U_ref = waypoint.U_d_ms 또는 U_default_ms.

    Parameters
    ----------
    F_to_speed_gain : float
        |−∇U| 크기 비례 속도 보정 게인. 기본 0.0 (비활성).
        > 0 이면: U_ref += gain * min(|F_pf|, 100.0).
        활성화 시 최대 gain*100 m/s 추가 — 과도 증가 방지를 위해 0.01 이하 권장.

    Notes
    -----
    기본값 F_to_speed_gain=0.0 : U_ref = waypoint.U_d_ms 고정 (예측 가능).
    장거리·가속 전략이 필요한 경우만 작은 게인으로 활성화.
    """
    dxi, deta = gradient_U(
        state.xi_m, state.eta_m,
        waypoint, obstacles,
        memory_wells, ice_cells,
        k_goal, k_ice, rho_0,
    )
    psi_ref = psi_ref_from_gradient(dxi, deta)
    F_mag = math.sqrt(dxi ** 2 + deta ** 2)
    U_ref = waypoint.U_d_ms if waypoint.U_d_ms is not None else U_default_ms
    if F_to_speed_gain > 0 and F_mag > 1e-6:
        U_ref = U_ref + F_to_speed_gain * min(F_mag, 100.0)
    return PathOutput(psi_ref_rad=psi_ref, U_ref_ms=max(0.0, U_ref), F_pf_mag=F_mag)
