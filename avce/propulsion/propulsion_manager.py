"""
추진 통합 관리자 (AVCE §PROP.MGR).

PropulsionType      : 추진 방식 Enum.
PropulsionManager   : 모드·추진 방식별 τ_u 통합 제공.
                      핵추진 / 디젤-전기 / 복합 지원.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
from .nuclear import NuclearReactor, ReactorParams, ReactorState, ReactorStatus

_Tau6 = Tuple[float, float, float, float, float, float]

class PropulsionType(str, Enum):
    NUCLEAR        = "nuclear"         # 핵추진 (단독)
    DIESEL_ELECTRIC = "diesel_electric" # 디젤-전기 (단독)
    HYBRID         = "hybrid"          # 핵 + 디젤 복합
    EMERGENCY_BATTERY = "emergency_battery"  # 비상 배터리 (무소음)

@dataclass(frozen=True)
class DieselParams:
    """디젤-전기 추진 파라미터 (불변)."""
    max_power_kw:   float = 5000.0   # 최대 출력 [kW]
    eta_electric:   float = 0.90     # 전기 효율
    tau_response_s: float = 5.0      # 응답 시정수 [s]
    R_prop_m:       float = 2.0      # 프로펠러 반경 [m]

@dataclass(frozen=True)
class BatteryParams:
    """비상 배터리 파라미터 (불변)."""
    capacity_kwh:   float = 500.0
    max_power_kw:   float = 1000.0
    eta:            float = 0.92

class PropulsionManager:
    """추진 통합 관리자.

    핵추진/디젤/배터리를 단일 인터페이스로 추상화.
    step() → tau_u [N] 반환.
    """
    def __init__(
        self,
        prop_type: PropulsionType = PropulsionType.NUCLEAR,
        reactor_params: Optional[ReactorParams] = None,
        diesel_params: Optional[DieselParams] = None,
        battery_params: Optional[BatteryParams] = None,
        omega_nominal_rads: float = 10.0,
    ):
        self._type = prop_type
        self._omega = omega_nominal_rads

        self._reactor: Optional[NuclearReactor] = None
        if prop_type in (PropulsionType.NUCLEAR, PropulsionType.HYBRID):
            rp = reactor_params or ReactorParams()
            self._reactor = NuclearReactor(rp)

        self._diesel_power_kw: float = 0.0
        self._diesel_params = diesel_params or DieselParams()
        self._battery_params = battery_params or BatteryParams()
        self._battery_soc: float = 1.0   # State of Charge [0~1]

        self._last_tau_u: float = 0.0

    def startup(self) -> None:
        """핵추진 기동 (해당 시)."""
        if self._reactor is not None:
            self._reactor.startup()

    def step(
        self,
        demand_frac: float,
        dt_s: float,
        omega_rads: Optional[float] = None,
    ) -> Tuple[float, Optional[ReactorState]]:
        """추진 한 스텝.

        Returns
        -------
        (tau_u_n, reactor_state_or_None)
            tau_u_n : 총 추력 [N]
            reactor_state : 핵추진 상태 (미사용 시 None)
        """
        omega = omega_rads if omega_rads is not None else self._omega
        reactor_state: Optional[ReactorState] = None
        tau_u = 0.0

        if self._type is PropulsionType.NUCLEAR and self._reactor is not None:
            reactor_state = self._reactor.step(demand_frac, omega, dt_s)
            if reactor_state.status is not ReactorStatus.SCRAMMED:
                tau_u = reactor_state.tau_u_n

        elif self._type is PropulsionType.DIESEL_ELECTRIC:
            dp = self._diesel_params
            P_target = dp.max_power_kw * demand_frac * 1e3
            self._diesel_power_kw += (P_target/1e3 - self._diesel_power_kw) / dp.tau_response_s * dt_s
            P_mech = self._diesel_power_kw * 1e3 * dp.eta_electric
            tau_u = P_mech / max(omega, 0.1) / dp.R_prop_m

        elif self._type is PropulsionType.HYBRID and self._reactor is not None:
            reactor_state = self._reactor.step(demand_frac * 0.8, omega, dt_s)
            tau_u = reactor_state.tau_u_n if reactor_state.status is not ReactorStatus.SCRAMMED else 0.0
            # 디젤 보조
            dp = self._diesel_params
            aux_frac = demand_frac * 0.2
            P_aux = dp.max_power_kw * aux_frac * 1e3 * dp.eta_electric
            tau_u += P_aux / max(omega, 0.1) / dp.R_prop_m

        elif self._type is PropulsionType.EMERGENCY_BATTERY:
            bp = self._battery_params
            if self._battery_soc > 0.05:
                P_avail = bp.max_power_kw * demand_frac * 1e3 * bp.eta
                tau_u = P_avail / max(omega, 0.1) / 2.0  # 소형 프로펠러 가정
                self._battery_soc -= P_avail / (bp.capacity_kwh * 3.6e6) * dt_s
                self._battery_soc = max(0.0, self._battery_soc)

        self._last_tau_u = tau_u
        return tau_u, reactor_state

    @property
    def propulsion_type(self) -> PropulsionType:
        return self._type

    @property
    def battery_soc(self) -> float:
        return self._battery_soc

    def tau6_surge(self, demand_frac: float, dt_s: float) -> _Tau6:
        """추진 → τ 벡터 (surge만, 나머지 0)."""
        tau_u, _ = self.step(demand_frac, dt_s)
        return (tau_u, 0.0, 0.0, 0.0, 0.0, 0.0)
