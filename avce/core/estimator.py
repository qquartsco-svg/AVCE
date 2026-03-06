"""
상태 추정 (EQUATIONS §7).

StateEstimator  : 계약 Protocol — z → VesselState.
IdentityEstimator : 시뮬/테스트용 (no-op).
KalmanEstimator : EKF-style 선형화 KF — 잡음 센서 입력 → 추정 상태 x̂.

설계 원칙
──────────
- KalmanConfig : frozen=True, numpy 없이 숫자만 — 하드코딩 없음.
- 행렬 연산은 numpy (선택 의존). 미설치 시 ImportError + 안내 메시지.
- KalmanEstimator.estimate(z) : predict + update 한 번에 처리 (단일 인터페이스).
- set_control(tau_u, tau_v, tau_r) : 시뮬 루프에서 제어 입력 전달 (predict에 사용).
- DynamicsParams에 직접 의존하지 않고 덕 타이핑 수용 — 레이어 독립성 유지.

EKF 선형화 수식 (각 스텝에서 재선형화)
─────────────────────────────────────
상태 벡터 x = [ξ, η, ψ, u, v, r]ᵀ  (인덱스 0~5)

연속 Jacobian Fc = ∂f/∂x (현재 x̂에서):
  row ξ : [0, 0, -(u sinψ + v cosψ),  cosψ, -sinψ, 0]
  row η : [0, 0,  (u cosψ - v sinψ),  sinψ,  cosψ, 0]
  row ψ : [0, 0,  0,                  0,     0,     1]
  row u : [0, 0,  0,                 -d_u/m_u, 0,   0]
  row v : [0, 0,  0,                  0, -d_v/m_v,  0]
  row r : [0, 0,  0,                  0,  0, -d_r/m_r]

이산화: F = I + Fc · dt (Euler 근사, 각 스텝 재선형화)

입력 행렬 B = dt · [0₃; M⁻¹] (제어 입력 [τ_u, τ_v, τ_r])

프로세스 잡음 Q = diag(σ_p²) · dt
측정 잡음 R   = diag(σ_m²)
측정 행렬 H   = I₆ (전 상태 측정)

예측:  x̂⁻ = F x̂ + B u,   P⁻ = F P Fᵀ + Q
갱신:  K  = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
       x̂  = x̂⁻ + K (z - H x̂⁻),   P = (I - KH) P⁻
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Optional, Protocol, Tuple

from .state import VesselState

# numpy는 선택 의존 — KalmanEstimator 사용 시에만 필요
try:
    import numpy as np
    _NP_OK = True
except ImportError:
    np = None  # type: ignore
    _NP_OK = False


# ── 계약 ──────────────────────────────────────────────────────────────────────

class StateEstimator(Protocol):
    """계약: 센서 z → 추정 상태 x̂ (VesselState)."""

    def estimate(self, z: Any) -> VesselState:
        """센서 z → x̂."""
        ...


# ── 단순 전달 (시뮬/테스트용) ─────────────────────────────────────────────────

class IdentityEstimator:
    """z가 이미 VesselState인 경우 그대로 반환 (no-op, 하위 호환)."""

    def estimate(self, z: Any) -> VesselState:
        if isinstance(z, VesselState):
            return z
        raise TypeError(f"IdentityEstimator: z must be VesselState, got {type(z)}")


# ── KF 파라미터 (불변) ────────────────────────────────────────────────────────

@dataclass(frozen=True)
class KalmanConfig:
    """EKF-style 선형화 KF 잡음 파라미터 (불변).

    σ_p : 프로세스 잡음 표준편차 (상태 불확실성 per √s).
    σ_m : 측정 잡음 표준편차 (센서 오차).
    p0_scale : 초기 공분산 P₀ = p0_scale · R.

    실제 GPS/IMU 예시
    -----------------
    GPS    : σ_pos ≈ 1–3 m
    나침반 : σ_heading ≈ 0.005–0.02 rad
    DVL    : σ_vel ≈ 0.05–0.3 m/s
    """
    # 프로세스 잡음 ─────────────────────────────────────────
    sigma_p_xi:   float = 0.5      # ξ  [m/√s]
    sigma_p_eta:  float = 0.5      # η
    sigma_p_psi:  float = 0.005    # ψ  [rad/√s]
    sigma_p_u:    float = 0.05     # u  [m/s/√s]
    sigma_p_v:    float = 0.05     # v
    sigma_p_r:    float = 0.005    # r  [rad/s/√s]

    # 측정 잡음 ─────────────────────────────────────────────
    sigma_m_xi:   float = 2.0      # GPS ξ [m]
    sigma_m_eta:  float = 2.0      # GPS η
    sigma_m_psi:  float = 0.02     # 나침반 [rad]
    sigma_m_u:    float = 0.3      # 속도 [m/s]
    sigma_m_v:    float = 0.3
    sigma_m_r:    float = 0.02     # 회전율 [rad/s]

    # 초기 공분산 배율 ──────────────────────────────────────
    p0_scale: float = 10.0


# ── 동역학 계수 (내부용, 레이어 독립) ────────────────────────────────────────

@dataclass(frozen=True)
class _DynCoeffs:
    """Jacobian 계산용 동역학 계수. DynamicsParams 덕 타이핑으로 생성."""
    d_u_over_m_u: float
    d_v_over_m_v: float
    d_r_over_m_r: float
    inv_m_u: float
    inv_m_v: float
    inv_m_r: float


# ── KalmanEstimator ───────────────────────────────────────────────────────────

class KalmanEstimator:
    """EKF-style 선형화 Kalman Filter (numpy 의존).

    사용법 (시뮬 루프)
    ──────────────────
        kf_cfg = KalmanConfig()
        est    = KalmanEstimator.from_dynamics_params(kf_cfg, dyn_params, dt_s=0.1)
        est.initialize(initial_state)

        # 매 스텝:
        x_hat = est.estimate(noisy_state)        # predict + update
        tau_u, tau_v, tau_r = controller(x_hat)
        est.set_control(tau_u, tau_v, tau_r)     # 다음 predict용
    """

    _N = 6  # 상태 차원
    _I_XI = 0; _I_ETA = 1; _I_PSI = 2
    _I_U  = 3; _I_V   = 4; _I_R   = 5

    def __init__(
        self,
        config: KalmanConfig,
        dyn: _DynCoeffs,
        dt_s: float,
    ) -> None:
        if not _NP_OK:
            raise ImportError(
                "KalmanEstimator는 numpy가 필요합니다: pip install numpy"
            )
        self._cfg = config
        self._dyn = dyn
        self._dt  = dt_s

        # 고정 행렬 (스텝마다 변하지 않음)
        self._R = np.diag([
            config.sigma_m_xi  ** 2,
            config.sigma_m_eta ** 2,
            config.sigma_m_psi ** 2,
            config.sigma_m_u   ** 2,
            config.sigma_m_v   ** 2,
            config.sigma_m_r   ** 2,
        ])
        self._H = np.eye(self._N)   # 전 상태 측정 H = I₆

        self._x_hat:    Optional[np.ndarray] = None
        self._P:        Optional[np.ndarray] = None
        self._last_tau: np.ndarray           = np.zeros(3)

    # ── 팩토리 ───────────────────────────────────────────────────────────────

    @classmethod
    def from_dynamics_params(
        cls,
        config: KalmanConfig,
        dynamics_params: Any,   # DynamicsParams — duck typing
        dt_s: float,
    ) -> "KalmanEstimator":
        """DynamicsParams 인스턴스로부터 생성 (덕 타이핑, 직접 import 없음)."""
        dyn = _DynCoeffs(
            d_u_over_m_u = dynamics_params.d_u / dynamics_params.m_u,
            d_v_over_m_v = dynamics_params.d_v / dynamics_params.m_v,
            d_r_over_m_r = dynamics_params.d_r / dynamics_params.m_r,
            inv_m_u      = 1.0 / dynamics_params.m_u,
            inv_m_v      = 1.0 / dynamics_params.m_v,
            inv_m_r      = 1.0 / dynamics_params.m_r,
        )
        return cls(config, dyn, dt_s)

    # ── 초기화 ───────────────────────────────────────────────────────────────

    def initialize(self, state: VesselState) -> None:
        """초기 x̂₀, P₀ 설정. estimate() 전에 반드시 호출."""
        self._x_hat    = self._to_vec(state)
        self._P        = self._cfg.p0_scale * self._R.copy()
        self._last_tau = np.zeros(3)

    # ── 제어 입력 등록 ────────────────────────────────────────────────────────

    def set_control(self, tau_u: float, tau_v: float, tau_r: float) -> None:
        """시뮬 루프에서 τ 저장 — 다음 predict 스텝에 사용."""
        self._last_tau = np.array([tau_u, tau_v, tau_r])

    # ── 메인 인터페이스 ───────────────────────────────────────────────────────

    def estimate(self, z: Any) -> VesselState:
        """predict + update → x̂ (VesselState).

        Parameters
        ----------
        z : VesselState
            노이즈 포함 센서 측정값. 미초기화 시 자동 initialize.
        """
        if not isinstance(z, VesselState):
            raise TypeError(f"KalmanEstimator.estimate(): z must be VesselState, got {type(z)}")

        if self._x_hat is None:
            self.initialize(z)
            return z  # 첫 스텝: 측정값 그대로

        # 1. 예측 ──────────────────────────────────────────────────────────────
        F       = self._compute_F()
        Q       = self._compute_Q()
        B       = self._compute_B()

        x_prior = F @ self._x_hat + B @ self._last_tau
        x_prior[self._I_PSI] = self._wrap(x_prior[self._I_PSI])
        P_prior = F @ self._P @ F.T + Q

        # 2. 갱신 ──────────────────────────────────────────────────────────────
        z_vec      = self._to_vec(z)
        innovation = z_vec - self._H @ x_prior
        innovation[self._I_PSI] = self._wrap(innovation[self._I_PSI])

        S = self._H @ P_prior @ self._H.T + self._R
        K = P_prior @ self._H.T @ np.linalg.inv(S)

        x_post = x_prior + K @ innovation
        x_post[self._I_PSI] = self._wrap(x_post[self._I_PSI])

        P_post = (np.eye(self._N) - K @ self._H) @ P_prior

        self._x_hat = x_post
        self._P     = P_post

        return self._to_state(x_post)

    # ── 행렬 빌더 ─────────────────────────────────────────────────────────────

    def _compute_F(self) -> np.ndarray:
        """이산화 Jacobian F = I + Fc · dt (현재 x̂에서 재선형화)."""
        x   = self._x_hat
        psi = x[self._I_PSI]
        u   = x[self._I_U]
        v   = x[self._I_V]
        d   = self._dyn
        dt  = self._dt
        cp  = math.cos(psi)
        sp  = math.sin(psi)

        Fc = np.zeros((6, 6))
        Fc[0, 2] = -(u * sp + v * cp)  ;  Fc[0, 3] = cp   ;  Fc[0, 4] = -sp
        Fc[1, 2] =   u * cp - v * sp   ;  Fc[1, 3] = sp   ;  Fc[1, 4] =  cp
        Fc[2, 5] = 1.0
        Fc[3, 3] = -d.d_u_over_m_u
        Fc[4, 4] = -d.d_v_over_m_v
        Fc[5, 5] = -d.d_r_over_m_r

        return np.eye(6) + Fc * dt

    def _compute_B(self) -> np.ndarray:
        """입력 행렬 B = dt · M⁻¹ ∈ ℝ^{6×3}."""
        d  = self._dyn
        dt = self._dt
        B  = np.zeros((6, 3))
        B[3, 0] = dt * d.inv_m_u
        B[4, 1] = dt * d.inv_m_v
        B[5, 2] = dt * d.inv_m_r
        return B

    def _compute_Q(self) -> np.ndarray:
        """Q = diag(σ_p²) · dt."""
        cfg = self._cfg
        q = np.array([cfg.sigma_p_xi**2, cfg.sigma_p_eta**2, cfg.sigma_p_psi**2,
                      cfg.sigma_p_u**2,  cfg.sigma_p_v**2,   cfg.sigma_p_r**2])
        return np.diag(q) * self._dt

    # ── 유틸 ─────────────────────────────────────────────────────────────────

    @staticmethod
    def _to_vec(s: VesselState) -> np.ndarray:
        return np.array([s.xi_m, s.eta_m, s.psi_rad, s.u_ms, s.v_ms, s.r_rads])

    @staticmethod
    def _to_state(x: np.ndarray) -> VesselState:
        return VesselState(xi_m=float(x[0]), eta_m=float(x[1]), psi_rad=float(x[2]),
                           u_ms=float(x[3]), v_ms=float(x[4]), r_rads=float(x[5]))

    @staticmethod
    def _wrap(psi: float) -> float:
        return (psi + math.pi) % (2.0 * math.pi) - math.pi

    # ── 진단 ─────────────────────────────────────────────────────────────────

    @property
    def x_hat(self) -> Optional[np.ndarray]:
        """현재 추정 상태 벡터 (복사본). 미초기화 시 None."""
        return self._x_hat.copy() if self._x_hat is not None else None

    @property
    def P(self) -> Optional[np.ndarray]:
        """현재 공분산 행렬 (복사본). 미초기화 시 None."""
        return self._P.copy() if self._P is not None else None

    def std_devs(self) -> Optional[List[float]]:
        """추정 1-σ 표준편차 [σ_ξ, σ_η, σ_ψ, σ_u, σ_v, σ_r]."""
        if self._P is None:
            return None
        return [math.sqrt(max(0.0, float(self._P[i, i]))) for i in range(self._N)]
