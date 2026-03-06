"""
6-DOF 선체 동역학 — Fossen (AVCE §6DOF.DYN).

M_rb ν̇ + C_rb(ν) ν + M_a ν̇_r + C_a(ν_r) ν_r + D(ν_r) ν_r + g(η) = τ + τ_env

선형화 단순 모델 (대각 M, D):
  (M_rb + M_a) ν̇ + D ν_r + g(η) = τ + τ_env
  즉:  M_6 ν̇ = τ + τ_env − D ν_r − g(η)

운동학 (Euler 각도):
  η̇ = J(η) ν        (6×6 Jacobian = [R_nb | 0; 0 | T_ψθ])

상태 벡터:
  ν = [u, v, w, p, q, r]ᵀ     Body-frame 속도 / 각속도
  η = [ξ, η, z, φ, θ, ψ]ᵀ    NED 위치 / Euler 자세

복원력 g(η) (중성 부력 근사):
  g_z = (W - B)                  [N]   순 중력 (양수=가라앉음)
  g_φ = GM_L · W · sin(φ)       [N·m] roll 복원
  g_θ = GM_T · W · sin(θ)       [N·m] pitch 복원
  나머지 = 0

설계 원칙
──────────
- _ode_rhs_6dof(): 순수 함수, 부작용 없음.
- DynamicsParams6DOF: frozen=True.
- RK4 적분: IntegrationMethod 재사용 (3-DOF와 동일 Enum).
- ψ/φ/θ 정규화: 최종 스텝에서만 수행.
- 복원력은 선택적 (W=B=0 → 중성 부력 → 복원력 0).

참고문헌
────────
Fossen, T.I. (2011). Handbook of Marine Craft Hydrodynamics and Motion Control.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from ..core.state6dof import VehicleState
from .dynamics import IntegrationMethod


# ── 6-DOF 파라미터 (불변) ──────────────────────────────────────────────────────

@dataclass(frozen=True)
class DynamicsParams6DOF:
    """6-DOF 선박/잠수함 동역학 파라미터 (불변).

    단위: 질량/관성 [kg / kg·m²], 감쇠 [N·s/m 또는 N·m·s/rad].

    Notes
    -----
    실 선박 값 교체 시: dataclasses.replace(p, m_u=...) 사용.
    Coriolis(C_rb, C_a) 항은 선형화 모델에서 생략 (저속 근사).
    고속 기동 시 C_rb 보정이 필요하면 c_enabled=True 설정.

    쇄빙 핵잠 (참고치 — Typhoon급 기준 추정):
        m_u ≈ 3.0e7 kg (24,000ton + 부가질량)
        m_r ≈ 1.0e12 kg·m²
    """
    # ── 질량 + 부가질량 ────────────────────────────────────────────────────
    m_u: float = 1.0e5    # surge  [kg]
    m_v: float = 1.5e5    # sway
    m_w: float = 1.5e5    # heave
    m_p: float = 2.0e8    # roll   [kg·m²]
    m_q: float = 5.0e8    # pitch
    m_r: float = 1.0e8    # yaw

    # ── 선형 감쇠 ─────────────────────────────────────────────────────────
    d_u: float = 5.0e3    # surge  [N·s/m]
    d_v: float = 8.0e3    # sway
    d_w: float = 8.0e3    # heave
    d_p: float = 5.0e5    # roll   [N·m·s/rad]
    d_q: float = 1.0e6    # pitch
    d_r: float = 1.0e6    # yaw

    # ── 중력·부력 (복원력) ────────────────────────────────────────────────
    W_n:        float = 0.0   # 중력 [N]. 0 = 중성 부력 (복원력 없음)
    B_n:        float = 0.0   # 부력 [N]. W_n = B_n 이면 중성 부력
    GM_roll_m:  float = 0.5   # 롤 복원 팔 [m]  (GM_L)
    GM_pitch_m: float = 5.0   # 피치 복원 팔 [m] (GM_T)

    # ── Coriolis 보정 사용 여부 ────────────────────────────────────────────
    coriolis_enabled: bool = False


# ── 내부 타입 ─────────────────────────────────────────────────────────────────

_Nu6  = Tuple[float, float, float, float, float, float]  # (u,v,w,p,q,r)
_Pos6 = Tuple[float, float, float, float, float, float]  # (ξ,η,z,φ,θ,ψ)
_Tau6 = Tuple[float, float, float, float, float, float]  # (τ_u,τ_v,τ_w,τ_p,τ_q,τ_r)


# ── 순수 헬퍼 ─────────────────────────────────────────────────────────────────

def _wrap(angle: float) -> float:
    """각도를 (−π, π] 로 정규화."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _add6(a: _Nu6, b: _Nu6, s: float) -> _Nu6:
    """a + s·b (6-tuple)."""
    return (
        a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2],
        a[3] + s * b[3], a[4] + s * b[4], a[5] + s * b[5],
    )


def _rk4_combine6(k1: _Nu6, k2: _Nu6, k3: _Nu6, k4: _Nu6, w: float) -> _Nu6:
    """RK4 가중 합 (k₁+2k₂+2k₃+k₄)/6·dt."""
    return (
        w * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
        w * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]),
        w * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]),
        w * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]),
        w * (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]),
        w * (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]),
    )


# ── 복원력 ────────────────────────────────────────────────────────────────────

def _restoring(pos: _Pos6, p: DynamicsParams6DOF) -> _Tau6:
    """복원력 벡터 g(η) [6×1].

    중성 부력 (W=B=0) 이면 전부 0.
    g(η) 는 동역학 우변에서 음수로 빠짐:
      M ν̇ = τ + τ_env − D ν − g(η)

    반환: g = [g_u, g_v, g_w, g_p, g_q, g_r]
    """
    _xi, _eta, _z, phi, theta, _psi = pos
    net = p.W_n - p.B_n          # 순 중력 (양수=가라앉음)

    # 수직 복원: 음수(-) 부호 → 양의 net이면 수직 힘은 아래 방향
    g_w = net
    g_p = p.GM_roll_m  * p.W_n * math.sin(phi)     # roll 복원
    g_q = p.GM_pitch_m * p.W_n * math.sin(theta)   # pitch 복원

    return (0.0, 0.0, g_w, g_p, g_q, 0.0)


# ── 6-DOF ODE 우변 (순수 함수) ────────────────────────────────────────────────

def _ode_rhs_6dof(
    nu:    _Nu6,
    pos:   _Pos6,
    tau:   _Tau6,
    params: DynamicsParams6DOF,
    tau_env: _Tau6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> Tuple[_Nu6, _Pos6]:
    """6-DOF ODE 우변 f(ν, η, τ) — 부작용 없음.

    수식
    ────
    ν̇ = M⁻¹ [τ + τ_env − D ν − g(η)]
    η̇ = J(η) ν

    J(η) — Fossen kinematic Jacobian (Euler 각도):

      ẋ = u cψcθ + v (cψsθsφ - sψcφ) + w (cψsθcφ + sψsφ)
      ẏ = u sψcθ + v (sψsθsφ + cψcφ) + w (sψsθcφ - cψsφ)
      ż = -u sθ  + v cθsφ             + w cθcφ
      φ̇ = p + q sφ tθ + r cφ tθ
      θ̇ = q cφ - r sφ
      ψ̇ = (q sφ + r cφ) / cθ          (singular at θ = ±π/2)

    Returns
    -------
    (dnu, dpos) : (ν̇ 6-tuple, η̇ 6-tuple)
    """
    u, v, w, p, q, r = nu
    xi, eta, z, phi, theta, psi = pos
    tau_u, tau_v, tau_w, tau_p, tau_q, tau_r = tau
    e_u, e_v, e_w, e_p, e_q, e_r = tau_env

    # 삼각함수 사전 계산
    cp = math.cos(phi);   sp = math.sin(phi)
    ct = math.cos(theta); st = math.sin(theta)
    cs = math.cos(psi);   ss = math.sin(psi)

    # 복원력
    g = _restoring(pos, params)

    # 속도 동역학: M⁻¹ (τ + τ_env − D ν − g)
    def _dnu(tau_i, e_i, d_i, m_i, g_i, nu_i) -> float:
        return (tau_i + e_i - d_i * nu_i - g_i) / m_i

    du = _dnu(tau_u, e_u, params.d_u, params.m_u, g[0], u)
    dv = _dnu(tau_v, e_v, params.d_v, params.m_v, g[1], v)
    dw = _dnu(tau_w, e_w, params.d_w, params.m_w, g[2], w)
    dp = _dnu(tau_p, e_p, params.d_p, params.m_p, g[3], p)
    dq = _dnu(tau_q, e_q, params.d_q, params.m_q, g[4], q)
    dr = _dnu(tau_r, e_r, params.d_r, params.m_r, g[5], r)

    # 운동학: η̇ = J(η) ν
    dxi  = u * (cs * ct) + v * (cs * st * sp - ss * cp) + w * (cs * st * cp + ss * sp)
    deta = u * (ss * ct) + v * (ss * st * sp + cs * cp) + w * (ss * st * cp - cs * sp)
    dz   = u * (-st)     + v * (ct * sp)                + w * (ct * cp)

    # θ = ±90° 특이점 보호
    sec_theta = 1.0 / max(abs(ct), 1e-6) * (1.0 if ct >= 0 else -1.0)
    tt = st * sec_theta  # tan(θ)

    dphi   = p + (q * sp + r * cp) * tt
    dtheta = q * cp - r * sp
    dpsi   = (q * sp + r * cp) * sec_theta

    return (
        (du, dv, dw, dp, dq, dr),
        (dxi, deta, dz, dphi, dtheta, dpsi),
    )


# ── Euler 적분 ────────────────────────────────────────────────────────────────

def step_6dof(
    state:  VehicleState,
    tau:    _Tau6,
    dt_s:   float,
    params: DynamicsParams6DOF,
    tau_env: _Tau6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> VehicleState:
    """Euler 1차 적분 (legacy 호환)."""
    nu0  = (state.u_ms, state.v_ms, state.w_ms, state.p_rads, state.q_rads, state.r_rads)
    pos0 = (state.xi_m, state.eta_m, state.z_m, state.phi_rad, state.theta_rad, state.psi_rad)

    dnu, dpos = _ode_rhs_6dof(nu0, pos0, tau, params, tau_env)

    return VehicleState(
        xi_m    = pos0[0] + dpos[0] * dt_s,
        eta_m   = pos0[1] + dpos[1] * dt_s,
        z_m     = pos0[2] + dpos[2] * dt_s,
        phi_rad   = _wrap(pos0[3] + dpos[3] * dt_s),
        theta_rad = _wrap(pos0[4] + dpos[4] * dt_s),
        psi_rad   = _wrap(pos0[5] + dpos[5] * dt_s),
        u_ms    = nu0[0] + dnu[0] * dt_s,
        v_ms    = nu0[1] + dnu[1] * dt_s,
        w_ms    = nu0[2] + dnu[2] * dt_s,
        p_rads  = nu0[3] + dnu[3] * dt_s,
        q_rads  = nu0[4] + dnu[4] * dt_s,
        r_rads  = nu0[5] + dnu[5] * dt_s,
    )


# ── RK4 적분 ─────────────────────────────────────────────────────────────────

def step_6dof_rk4(
    state:  VehicleState,
    tau:    _Tau6,
    dt_s:   float,
    params: DynamicsParams6DOF,
    tau_env: _Tau6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> VehicleState:
    """Runge-Kutta 4차 적분.

    Notes
    -----
    τ, τ_env는 스텝 내 상수 (1차 hold).
    각도 정규화(φ, θ, ψ)는 최종 스텝에서만 수행.
    θ = ±90° 특이점 근방에서 dpsi 발산 가능 → 운영 범위 제한 권장.
    """
    nu0  = (state.u_ms, state.v_ms, state.w_ms, state.p_rads, state.q_rads, state.r_rads)
    pos0 = (state.xi_m, state.eta_m, state.z_m, state.phi_rad, state.theta_rad, state.psi_rad)

    kw = {"params": params, "tau_env": tau_env}

    # k₁
    dnu1, dp1 = _ode_rhs_6dof(nu0, pos0, tau, **kw)

    # k₂
    nu2  = _add6(nu0,  dnu1, 0.5 * dt_s)
    pos2 = _add6(pos0, dp1,  0.5 * dt_s)
    dnu2, dp2 = _ode_rhs_6dof(nu2, pos2, tau, **kw)

    # k₃
    nu3  = _add6(nu0,  dnu2, 0.5 * dt_s)
    pos3 = _add6(pos0, dp2,  0.5 * dt_s)
    dnu3, dp3 = _ode_rhs_6dof(nu3, pos3, tau, **kw)

    # k₄
    nu4  = _add6(nu0,  dnu3, dt_s)
    pos4 = _add6(pos0, dp3,  dt_s)
    dnu4, dp4 = _ode_rhs_6dof(nu4, pos4, tau, **kw)

    # 가중 합
    w = dt_s / 6.0
    dnu_rk4  = _rk4_combine6(dnu1, dnu2, dnu3, dnu4, w)
    dpos_rk4 = _rk4_combine6(dp1,  dp2,  dp3,  dp4,  w)

    return VehicleState(
        xi_m      = pos0[0] + dpos_rk4[0],
        eta_m     = pos0[1] + dpos_rk4[1],
        z_m       = pos0[2] + dpos_rk4[2],
        phi_rad   = _wrap(pos0[3] + dpos_rk4[3]),
        theta_rad = _wrap(pos0[4] + dpos_rk4[4]),
        psi_rad   = _wrap(pos0[5] + dpos_rk4[5]),
        u_ms      = nu0[0] + dnu_rk4[0],
        v_ms      = nu0[1] + dnu_rk4[1],
        w_ms      = nu0[2] + dnu_rk4[2],
        p_rads    = nu0[3] + dnu_rk4[3],
        q_rads    = nu0[4] + dnu_rk4[4],
        r_rads    = nu0[5] + dnu_rk4[5],
    )


# ── 통합 Dispatcher ───────────────────────────────────────────────────────────

def step_6dof_method(
    state:  VehicleState,
    tau:    _Tau6,
    dt_s:   float,
    params: DynamicsParams6DOF,
    method: IntegrationMethod = IntegrationMethod.RK4,
    tau_env: _Tau6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> VehicleState:
    """통합 6-DOF 적분 인터페이스.

    Parameters
    ----------
    tau : (τ_u, τ_v, τ_w, τ_p, τ_q, τ_r)
        제어 힘/모멘트 벡터 [N / N·m].
    method : IntegrationMethod
        EULER 또는 RK4 (기본).
    tau_env : 6-tuple
        환경 외란 (파랑, 조류, 빙 충격 등).
    """
    if method is IntegrationMethod.RK4:
        return step_6dof_rk4(state, tau, dt_s, params, tau_env)
    return step_6dof(state, tau, dt_s, params, tau_env)


# ── 제어 힘 변환 (단순 P 제어) ────────────────────────────────────────────────

def tau6_from_setpoints(
    state:      VehicleState,
    psi_ref:    float,
    U_ref:      float,
    z_ref:      float = 0.0,
    theta_ref:  float = 0.0,
    k_psi:   float = 1.0e5,
    k_u:     float = 5.0e4,
    k_z:     float = 1.0e4,
    k_theta: float = 5.0e5,
    k_phi:   float = 3.0e5,
) -> _Tau6:
    """setpoint → 6-DOF τ 벡터 (단순 P 제어).

    Parameters
    ----------
    psi_ref   : 목표 방위각 [rad]
    U_ref     : 목표 수평 속력 [m/s]
    z_ref     : 목표 깊이 [m]  (양수=잠수)
    theta_ref : 목표 피치각 [rad]
    k_*       : P 게인

    Returns
    -------
    (τ_u, τ_v, τ_w, τ_p, τ_q, τ_r)
    """
    # Δψ 최단 경로
    d_psi   = _wrap(psi_ref - state.psi_rad)
    # 수평 속력
    U_now   = state.U_horizontal_ms
    # 깊이·피치 오차
    d_z     = z_ref   - state.z_m
    d_theta = _wrap(theta_ref - state.theta_rad)
    # 롤 → 0 복귀
    d_phi   = _wrap(0.0 - state.phi_rad)

    tau_u = k_u     * (U_ref - U_now)
    tau_v = 0.0                          # sway — 별도 sideslip 제어 시 사용
    tau_w = k_z     * d_z               # heave → 깊이 제어
    tau_p = k_phi   * d_phi             # roll → 수평 유지
    tau_q = k_theta * d_theta           # pitch → 깊이 자세
    tau_r = k_psi   * d_psi             # yaw → 방위각

    return (tau_u, tau_v, tau_w, tau_p, tau_q, tau_r)
