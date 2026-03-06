"""
해양 환경 모델 (AVCE §ENV).
OceanEnvironment : 수심별 수온·밀도·조류 → 환경 외란 τ_env.
BuoyancyModel    : 잠수 깊이별 부력 계산.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from ..core.state6dof import VehicleState, OceanCell

_Tau6 = Tuple[float, float, float, float, float, float]

@dataclass(frozen=True)
class BuoyancyParams:
    """부력·배수량 파라미터 (불변)."""
    displacement_m3: float = 100.0   # 배수량 [m³]
    rho_water_kgm3: float = 1025.0   # 기준 해수 밀도 [kg/m³]
    g_ms2: float = 9.81              # 중력 가속도 [m/s²]
    mass_kg: float = 1.0e5           # 선체 질량 [kg]
    ballast_fraction: float = 0.0    # 밸러스트 비율 [0~1] (0=최대 부력, 1=최대 중력)

    @property
    def W_n(self) -> float:
        """중력 [N]."""
        return self.mass_kg * self.g_ms2

    @property
    def B_n_max(self) -> float:
        """최대 부력 [N] (밸러스트 없을 때)."""
        return self.displacement_m3 * self.rho_water_kgm3 * self.g_ms2

    @property
    def net_buoyancy_n(self) -> float:
        """순 부력 [N] = B - W. 양수=부상력, 음수=침강력."""
        B = self.B_n_max * (1.0 - self.ballast_fraction)
        return B - self.W_n


class BuoyancyModel:
    """잠수 깊이·밸러스트 조절 → 부력 계산.

    밸러스트 조절 speed: ballast_rate [fraction/s].
    """
    def __init__(self, params: BuoyancyParams, ballast_rate_s: float = 0.01):
        self._params = params
        self._ballast: float = params.ballast_fraction
        self._rate: float = ballast_rate_s

    @property
    def params(self) -> BuoyancyParams:
        return self._params

    @property
    def ballast_fraction(self) -> float:
        return self._ballast

    def set_ballast(self, target: float, dt_s: float) -> None:
        """밸러스트 비율을 target으로 rate 제한하여 갱신."""
        delta = target - self._ballast
        max_delta = self._rate * dt_s
        self._ballast += max(-max_delta, min(max_delta, delta))
        self._ballast = max(0.0, min(1.0, self._ballast))

    def net_buoyancy_n(self, rho_local: float = 1025.0) -> float:
        """현재 밸러스트 + 로컬 밀도 기준 순 부력 [N]."""
        p = self._params
        B = p.displacement_m3 * rho_local * p.g_ms2 * (1.0 - self._ballast)
        return B - p.W_n

    def tau_buoyancy(self) -> _Tau6:
        """부력 → τ_env_w 성분 (수직 힘, NED 좌표)."""
        # 순 부력: 양수=부상(위) → NED에서 z 감소 방향 = -w
        F_z = -self.net_buoyancy_n()   # NED: z+는 아래. 부상력은 -z
        return (0.0, 0.0, F_z, 0.0, 0.0, 0.0)


class OceanEnvironment:
    """해양 환경 모델. OceanCell 목록 → 선체 위치의 환경 외란 τ_env 계산.

    가중 평균 Gaussian 보간으로 로컬 환경값 추정.
    """
    def __init__(self, cells: Optional[List[OceanCell]] = None):
        self._cells: List[OceanCell] = list(cells or [])

    def add_cell(self, cell: OceanCell) -> None:
        self._cells.append(cell)

    def interpolate(self, xi: float, eta: float, z: float) -> OceanCell:
        """주어진 위치의 환경값을 Gaussian 보간으로 추정."""
        if not self._cells:
            return OceanCell(xi_m=xi, eta_m=eta, z_m=z)
        total_w = 0.0
        temp = density = sal = cu = cv = cw = 0.0
        for c in self._cells:
            dx = xi - c.xi_m; dy = eta - c.eta_m; dz = z - c.z_m
            r2 = dx**2 + dy**2 + dz**2
            w = math.exp(-r2 / (2.0 * c.sigma_m**2))
            total_w += w
            temp    += w * c.temp_c
            density += w * c.density_kgm3
            sal     += w * c.salinity_ppt
            cu      += w * c.current_u_ms
            cv      += w * c.current_v_ms
            cw      += w * c.current_w_ms
        if total_w < 1e-12:
            return OceanCell(xi_m=xi, eta_m=eta, z_m=z)
        inv = 1.0 / total_w
        return OceanCell(
            xi_m=xi, eta_m=eta, z_m=z,
            temp_c=temp*inv, salinity_ppt=sal*inv,
            density_kgm3=density*inv,
            current_u_ms=cu*inv, current_v_ms=cv*inv, current_w_ms=cw*inv,
        )

    def tau_env_from_current(
        self,
        state: VehicleState,
        cd_u: float = 2.0e3,
        cd_v: float = 4.0e3,
        cd_w: float = 4.0e3,
    ) -> _Tau6:
        """조류 → 항력 외란 τ_env.

        조류와 선체 사이 상대 속도 → 항력 (D * Δv).
        cd_* : 조류 항력 계수 [N·s/m].
        """
        env = self.interpolate(state.xi_m, state.eta_m, state.z_m)
        # 글로벌 NED 조류를 Body frame으로 변환
        cp = math.cos(state.phi_rad); sp = math.sin(state.phi_rad)
        ct = math.cos(state.theta_rad); st = math.sin(state.theta_rad)
        cs = math.cos(state.psi_rad); ss = math.sin(state.psi_rad)
        # NED → Body (R_nb^T) — 3×3 회전 역변환 (transpose)
        cu_b = (env.current_u_ms * cs*ct +
                env.current_v_ms * ss*ct +
                env.current_w_ms * (-st))
        cv_b = (env.current_u_ms * (cs*st*sp - ss*cp) +
                env.current_v_ms * (ss*st*sp + cs*cp) +
                env.current_w_ms * (ct*sp))
        cw_b = (env.current_u_ms * (cs*st*cp + ss*sp) +
                env.current_v_ms * (ss*st*cp - cs*sp) +
                env.current_w_ms * (ct*cp))
        # 상대 속도 → 항력
        tau_u = cd_u * (cu_b - state.u_ms)
        tau_v = cd_v * (cv_b - state.v_ms)
        tau_w = cd_w * (cw_b - state.w_ms)
        return (tau_u, tau_v, tau_w, 0.0, 0.0, 0.0)
