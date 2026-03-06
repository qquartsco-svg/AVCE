"""
핵추진 시스템 모델 (AVCE §PROP.NUCLEAR).

NuclearReactorParams : PWR 원자로 파라미터 (frozen).
NuclearReactor       : 원자로 → 증기 → 터빈 → 추력 변환.
                       1차 지연 응답 + SCRAM 보호 논리.

수식
────
dP/dt = (P_demand − P) / τ_reactor          [1차 지연]
τ_shaft = η · P_thermal · 1e6 / ω_shaft     [N·m]
τ_u     = τ_shaft / R_prop                  [N] (프로펠러 반경 R_prop)

SCRAM 조건: P > P_rated · scram_frac → 즉각 P → 0, 재기동 불가
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum

class ReactorStatus(str, Enum):
    SHUTDOWN   = "shutdown"
    STARTUP    = "startup"
    OPERATING  = "operating"
    SCRAMMED   = "scrammed"   # 비상 정지 (재기동 불가)

@dataclass(frozen=True)
class ReactorParams:
    """PWR 원자로 파라미터 (불변).

    참고: 러시아 OK-650B (Typhoon급) ≈ 190 MWth
         미국 S9G (Virginia급) ≈ 210 MWth
    """
    P_rated_mw:      float = 190.0   # 정격 열출력 [MW]
    eta_mechanical:  float = 0.35    # 열→기계 효율
    tau_response_s:  float = 30.0    # 출력 응답 시정수 [s]
    P_min_frac:      float = 0.15    # 최소 출력 비율 (안전 임계)
    P_max_frac:      float = 1.05    # 최대 허용 출력 배율
    P_scram_frac:    float = 1.15    # SCRAM 트립 배율
    R_prop_m:        float = 2.5     # 프로펠러 반경 [m]
    omega_nominal_rads: float = 10.0 # 공칭 프로펠러 각속도 [rad/s]
    n_shafts:        int   = 2       # 추진축 수

@dataclass(frozen=True)
class ReactorState:
    """원자로 현재 상태 (불변 스냅샷)."""
    status:       ReactorStatus
    P_thermal_mw: float   # 현재 열출력 [MW]
    demand_frac:  float   # 요구 출력 비율
    tau_u_n:      float   # 발생 추력 [N]
    scram_count:  int     # SCRAM 발생 횟수

class NuclearReactor:
    """PWR 원자로 추진 모델.

    사용법
    ------
    reactor = NuclearReactor(ReactorParams())
    reactor.startup()
    tau_u = reactor.step(demand_frac=0.7, omega_rads=10.0, dt_s=0.1).tau_u_n
    """
    def __init__(self, params: ReactorParams):
        self._p = params
        self._P_mw: float = 0.0
        self._status = ReactorStatus.SHUTDOWN
        self._scram_count: int = 0
        self._demand: float = 0.0

    def startup(self) -> None:
        """원자로 기동 (SHUTDOWN → STARTUP → OPERATING)."""
        if self._status is ReactorStatus.SCRAMMED:
            raise RuntimeError("SCRAM 상태: 원자로 재기동 불가 (수동 리셋 필요)")
        self._status = ReactorStatus.STARTUP
        self._P_mw = self._p.P_rated_mw * self._p.P_min_frac

    def scram_reset(self) -> None:
        """SCRAM 수동 리셋 (정비 후 재기동 전 호출)."""
        self._status = ReactorStatus.SHUTDOWN
        self._P_mw = 0.0

    def step(
        self,
        demand_frac: float,
        omega_rads: float,
        dt_s: float,
    ) -> ReactorState:
        """한 스텝: 출력 갱신 → 추력 계산.

        Parameters
        ----------
        demand_frac : float [0~1]
            요구 출력 비율.
        omega_rads : float
            프로펠러 각속도 [rad/s]. 0이면 최소 토크.
        """
        if self._status is ReactorStatus.SCRAMMED:
            return ReactorState(ReactorStatus.SCRAMMED, 0.0, 0.0, 0.0, self._scram_count)
        if self._status is ReactorStatus.SHUTDOWN:
            return ReactorState(ReactorStatus.SHUTDOWN, 0.0, 0.0, 0.0, self._scram_count)

        p = self._p
        # 클램핑
        demand_frac = max(p.P_min_frac, min(p.P_max_frac, demand_frac))
        P_target = p.P_rated_mw * demand_frac
        self._demand = demand_frac

        # SCRAM 체크
        if P_target > p.P_rated_mw * p.P_scram_frac:
            self._P_mw = 0.0
            self._status = ReactorStatus.SCRAMMED
            self._scram_count += 1
            return ReactorState(ReactorStatus.SCRAMMED, 0.0, demand_frac, 0.0, self._scram_count)

        # 1차 지연 응답: dP/dt = (P_target - P) / tau
        self._P_mw += (P_target - self._P_mw) / p.tau_response_s * dt_s
        self._P_mw = max(0.0, self._P_mw)

        if self._status is ReactorStatus.STARTUP and self._P_mw >= p.P_rated_mw * p.P_min_frac:
            self._status = ReactorStatus.OPERATING

        # 추력 계산: τ_shaft = η * P [W] / ω [rad/s]
        P_mech_w = self._P_mw * 1e6 * p.eta_mechanical
        omega_safe = max(abs(omega_rads), 0.1)
        tau_shaft = P_mech_w / omega_safe          # [N·m]
        tau_u = tau_shaft / p.R_prop_m * p.n_shafts  # [N] (전체 추력)

        return ReactorState(
            status=self._status,
            P_thermal_mw=self._P_mw,
            demand_frac=demand_frac,
            tau_u_n=tau_u,
            scram_count=self._scram_count,
        )

    @property
    def status(self) -> ReactorStatus:
        return self._status

    @property
    def P_thermal_mw(self) -> float:
        return self._P_mw
