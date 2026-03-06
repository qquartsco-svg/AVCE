"""
3-DOF 선박 동역학 (EQUATIONS §2).

M ν̇ + D ν = τ + τ_env
kinematic: ξ̇ = u cosψ − v sinψ,  η̇ = u sinψ + v cosψ,  ψ̇ = r

설계 원칙
──────────
- _ode_rhs() : 순수 함수 (부작용 없음). Euler / RK4 양쪽에서 공유.
- DynamicsParams : frozen=True — 파라미터 불변성 보장.
- IntegrationMethod : Enum — 하드코딩 없이 방식 선택.
- ψ 정규화는 최종 스텝에서만 수행 (중간 k_i 평가 시 미적용 → RK4 수치 안정).
- tau_from_heading_and_surge() : 단순 P 제어 (계약 수준). 하부 제어기 교체 가능.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Tuple

from ..core.state import VesselState


# ── 적분 방식 ─────────────────────────────────────────────────────────────────

class IntegrationMethod(str, Enum):
    """수치 적분 방식 선택.

    EULER : 1차 Euler — 빠름, 장기 시뮬에서 오차 누적.
    RK4   : 4차 Runge-Kutta — 정확, 실시간 제어·정밀 시뮬 권장.
    """
    EULER = "euler"
    RK4   = "rk4"


# ── 동역학 파라미터 (불변) ────────────────────────────────────────────────────

@dataclass(frozen=True)
class DynamicsParams:
    """선형화 3-DOF 파라미터 (불변 dataclass).

    diagonal M = diag(m_u, m_v, m_r),  D = diag(d_u, d_v, d_r)
    단위: 질량 [kg], 감쇠 [N·s/m 또는 N·m·s/rad].

    Notes
    -----
    실 선박 값으로 교체 시 frozen 특성으로 replace()를 사용:
        p2 = dataclasses.replace(p, m_u=2.0e5)
    """
    m_u: float = 1.0e5   # surge 질량 + 부가질량 [kg]
    m_v: float = 1.5e5   # sway
    m_r: float = 1.0e8   # yaw 관성 [kg·m²]
    d_u: float = 5.0e3   # surge 감쇠 [N·s/m]
    d_v: float = 8.0e3   # sway 감쇠
    d_r: float = 1.0e6   # yaw 감쇠 [N·m·s/rad]


# ── 내부 순수 함수 ────────────────────────────────────────────────────────────

_Nu  = Tuple[float, float, float]   # (u, v, r)
_Pos = Tuple[float, float, float]   # (ξ, η, ψ)


def _ode_rhs(
    nu:  _Nu,
    pos: _Pos,
    tau_u: float,
    tau_v: float,
    tau_r: float,
    params: DynamicsParams,
    tau_env_u: float = 0.0,
    tau_env_v: float = 0.0,
    tau_env_r: float = 0.0,
) -> Tuple[_Nu, _Pos]:
    """순수 ODE 우변 f(ν, pos, τ) — 부작용 없음.

    수식
    ────
    ν̇ = M⁻¹ (τ + τ_env − D ν)
    ξ̇ = u cosψ − v sinψ
    η̇ = u sinψ + v cosψ
    ψ̇ = r

    Returns
    -------
    (dnu, dpos) : 속도 미분 (du,dv,dr), 위치 미분 (dξ,dη,dψ)
    """
    u, v, r       = nu
    xi, eta, psi  = pos

    f_u = tau_u + tau_env_u
    f_v = tau_v + tau_env_v
    f_r = tau_r + tau_env_r

    du = (f_u - params.d_u * u) / params.m_u
    dv = (f_v - params.d_v * v) / params.m_v
    dr = (f_r - params.d_r * r) / params.m_r

    cp = math.cos(psi)
    sp = math.sin(psi)
    dxi  = u * cp - v * sp
    deta = u * sp + v * cp
    dpsi = r

    return (du, dv, dr), (dxi, deta, dpsi)


def _normalize_psi(psi: float) -> float:
    """ψ를 (−π, π] 로 정규화."""
    return (psi + math.pi) % (2.0 * math.pi) - math.pi


def _add3(a: _Nu, b: _Nu, s: float) -> tuple:
    """a + s*b (3-tuple)."""
    return (a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2])


def _rk4_combine(k1: _Nu, k2: _Nu, k3: _Nu, k4: _Nu, w: float) -> _Nu:
    """RK4 가중 합 (k1 + 2k2 + 2k3 + k4)/6 * dt."""
    return (
        w * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
        w * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]),
        w * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]),
    )


# ── 공개 적분 함수 ────────────────────────────────────────────────────────────

def step_3dof(
    state: VesselState,
    tau_u: float,
    tau_v: float,
    tau_r: float,
    dt_s: float,
    params: DynamicsParams,
    tau_env_u: float = 0.0,
    tau_env_v: float = 0.0,
    tau_env_r: float = 0.0,
) -> VesselState:
    """Euler 1차 적분 (하위 호환).

    정확도가 필요하면 step_3dof_rk4() 또는 step_3dof_method() 사용.
    """
    nu  = (state.u_ms, state.v_ms, state.r_rads)
    pos = (state.xi_m, state.eta_m, state.psi_rad)

    dnu, dpos = _ode_rhs(nu, pos, tau_u, tau_v, tau_r, params,
                         tau_env_u, tau_env_v, tau_env_r)

    return VesselState(
        xi_m    = pos[0] + dpos[0] * dt_s,
        eta_m   = pos[1] + dpos[1] * dt_s,
        psi_rad = _normalize_psi(pos[2] + dpos[2] * dt_s),
        u_ms    = nu[0]  + dnu[0]  * dt_s,
        v_ms    = nu[1]  + dnu[1]  * dt_s,
        r_rads  = nu[2]  + dnu[2]  * dt_s,
    )


def step_3dof_rk4(
    state: VesselState,
    tau_u: float,
    tau_v: float,
    tau_r: float,
    dt_s: float,
    params: DynamicsParams,
    tau_env_u: float = 0.0,
    tau_env_v: float = 0.0,
    tau_env_r: float = 0.0,
) -> VesselState:
    """Runge-Kutta 4차 적분.

    수식
    ────
    k₁ = f(t, y)
    k₂ = f(t+½dt, y + ½k₁·dt)
    k₃ = f(t+½dt, y + ½k₂·dt)
    k₄ = f(t+dt,  y + k₃·dt)
    y_new = y + (k₁ + 2k₂ + 2k₃ + k₄)/6 · dt

    Notes
    -----
    - τ, τ_env는 스텝 내 상수로 가정 (1차 hold).
    - ψ 정규화는 최종 스텝에서만 수행 (k₂·k₃·k₄ 평가 시 미적용 → 경계 근처 수치 일관성).
    """
    nu0  = (state.u_ms,   state.v_ms,   state.r_rads)
    pos0 = (state.xi_m,   state.eta_m,  state.psi_rad)
    kw   = {"params": params, "tau_env_u": tau_env_u,
            "tau_env_v": tau_env_v, "tau_env_r": tau_env_r}

    # k₁
    dnu1, dp1 = _ode_rhs(nu0, pos0, tau_u, tau_v, tau_r, **kw)

    # k₂
    nu2  = _add3(nu0,  dnu1, 0.5 * dt_s)
    pos2 = _add3(pos0, dp1,  0.5 * dt_s)
    dnu2, dp2 = _ode_rhs(nu2, pos2, tau_u, tau_v, tau_r, **kw)

    # k₃
    nu3  = _add3(nu0,  dnu2, 0.5 * dt_s)
    pos3 = _add3(pos0, dp2,  0.5 * dt_s)
    dnu3, dp3 = _ode_rhs(nu3, pos3, tau_u, tau_v, tau_r, **kw)

    # k₄
    nu4  = _add3(nu0,  dnu3, dt_s)
    pos4 = _add3(pos0, dp3,  dt_s)
    dnu4, dp4 = _ode_rhs(nu4, pos4, tau_u, tau_v, tau_r, **kw)

    # 가중 합
    w = dt_s / 6.0
    dnu_rk4  = _rk4_combine(dnu1, dnu2, dnu3, dnu4, w)
    dpos_rk4 = _rk4_combine(dp1,  dp2,  dp3,  dp4,  w)

    return VesselState(
        xi_m    = pos0[0] + dpos_rk4[0],
        eta_m   = pos0[1] + dpos_rk4[1],
        psi_rad = _normalize_psi(pos0[2] + dpos_rk4[2]),
        u_ms    = nu0[0]  + dnu_rk4[0],
        v_ms    = nu0[1]  + dnu_rk4[1],
        r_rads  = nu0[2]  + dnu_rk4[2],
    )


def step_3dof_method(
    state: VesselState,
    tau_u: float,
    tau_v: float,
    tau_r: float,
    dt_s: float,
    params: DynamicsParams,
    method: IntegrationMethod = IntegrationMethod.RK4,
    tau_env_u: float = 0.0,
    tau_env_v: float = 0.0,
    tau_env_r: float = 0.0,
) -> VesselState:
    """통합 인터페이스: IntegrationMethod로 적분 방식 선택.

    Parameters
    ----------
    method : IntegrationMethod
        EULER (1차) 또는 RK4 (4차, 기본값).
    """
    if method is IntegrationMethod.RK4:
        return step_3dof_rk4(
            state, tau_u, tau_v, tau_r, dt_s, params,
            tau_env_u, tau_env_v, tau_env_r,
        )
    return step_3dof(
        state, tau_u, tau_v, tau_r, dt_s, params,
        tau_env_u, tau_env_v, tau_env_r,
    )


# ── 단순 P 제어 (계약 수준) ───────────────────────────────────────────────────

def tau_from_heading_and_surge(
    psi_ref_rad: float,
    psi_rad: float,
    U_ref_ms: float,
    u_ms: float,
    v_ms: float,
    k_psi: float = 1.0e5,
    k_u: float   = 5.0e4,
) -> Tuple[float, float, float]:
    """단순 P 제어: τ_r = k_ψ·Δψ, τ_u = k_u·ΔU.

    Notes
    -----
    실제 선박에서는 이 함수를 PID / MPC 등으로 교체한다.
    Δψ는 최단 경로 각도차 (-π, π] 로 정규화.
    """
    U = math.sqrt(u_ms ** 2 + v_ms ** 2) if (u_ms or v_ms) else 0.0

    d_psi = _normalize_psi(psi_ref_rad - psi_rad)

    tau_u = k_u * (U_ref_ms - U)
    tau_r = k_psi * d_psi
    return tau_u, 0.0, tau_r
