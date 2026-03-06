"""퍼텐셜 필드 (EQUATIONS.md §3, §10).

스칼라 포텐셜
─────────────
U_goal         : 흡인 우물 — (1/2)*k*(Δξ²+Δη²)
U_obs_gaussian : 반발 가우시안 — A·exp(-ρ²/2σ²)
U_obs_distance : 반발 거리 기반 — (1/2)·A·(1/ρ - 1/ρ₀)²
U_ice_linear   : 빙 비용 — k_ice·max_c(w_c·C_c)
U_total        : 합산

Gradient 계산
─────────────
gradient_U_analytic : 해석적 −∇U   (O(N), 권장)
gradient_U_numeric  : 유한 차분 −∇U (legacy, eps 사용)
gradient_U          : 통합 dispatcher — GradientMethod 선택

설계 원칙
──────────
- GradientMethod(str, Enum): ANALYTIC / NUMERIC — 하드코딩 없음.
- _grad_* 순수 함수: state.py 계약 타입만 의존.
- memory_wells Callable: 해석적 미분 불가 → analytic 모드에서도 수치 보완.
- gradient_U(): 기본값 GradientMethod.ANALYTIC (eps 파라미터 backward compat).

해석적 Gradient 수식
──────────────────────────────────────────────
U_goal = (1/2)·k·(Δξ²+Δη²)          ,  Δξ = ξ - ξ_d
  ∇U_goal = k·(Δξ, Δη)

U_gauss = A·exp(−ρ²/2σ²)             ,  ρ² = Δξ²+Δη²
  ∇U_gauss = val·(−Δξ/σ², −Δη/σ²)   ,  val = U_gauss

U_dist = (1/2)·A·(1/ρ − 1/ρ₀)²      ,  valid ρ < ρ₀
  ∇U_dist = −A·(1/ρ − 1/ρ₀)·(Δξ,Δη) / ρ³

U_ice ≈ k_ice·w_{c*}·C_{c*}          ,  c* = argmax w_c·C_c
  ∇U_ice ≈ k_ice·C_{c*}·w_{c*}·(−Δξ_{c*}/σ_{c*}², −Δη_{c*}/σ_{c*}²)
"""
from __future__ import annotations

import math
from enum import Enum
from typing import Callable, List, Optional, Tuple

from ..core.constants import DEFAULT_K_GOAL, DEFAULT_K_OBS, RHO_0_DEFAULT
from ..core.state import IceCell, Obstacle, VesselState, Waypoint


# ── 순수 헬퍼 ────────────────────────────────────────────────────────────────

def _rho(xi: float, eta: float, xo: float, yo: float) -> float:
    """유클리드 거리."""
    return math.sqrt((xi - xo) ** 2 + (eta - yo) ** 2)


# ── 스칼라 포텐셜 ─────────────────────────────────────────────────────────────

def U_goal(
    xi: float, eta: float,
    wp: Waypoint,
    k_goal: float = DEFAULT_K_GOAL,
) -> float:
    """흡인 우물: U_goal = (1/2) k_goal · (거리²). [m²·(1/m²)] → 무차원 가중치."""
    d2 = (xi - wp.xi_m) ** 2 + (eta - wp.eta_m) ** 2
    return 0.5 * k_goal * d2


def U_obs_gaussian(
    xi: float, eta: float,
    obs: Obstacle,
) -> float:
    """반발 가우시안: A · exp(−ρ² / (2σ²))."""
    r = _rho(xi, eta, obs.xi_m, obs.eta_m)
    return obs.A * math.exp(-(r ** 2) / (2.0 * obs.sigma_m ** 2))


def U_obs_distance(
    xi: float, eta: float,
    obs: Obstacle,
    rho_0: float = RHO_0_DEFAULT,
) -> float:
    """반발 거리 기반: (1/2)·A·(1/ρ − 1/ρ₀)²,  ρ ≤ ρ₀에서만."""
    r = _rho(xi, eta, obs.xi_m, obs.eta_m)
    if r >= rho_0 or r < 1e-6:
        return 0.0
    return 0.5 * obs.A * (1.0 / r - 1.0 / rho_0) ** 2


def U_obs(
    xi: float, eta: float,
    obs: Obstacle,
    rho_0: float = RHO_0_DEFAULT,
) -> float:
    """장애물 포텐셜: rho_0_m 있으면 거리 기반, 없으면 가우시안."""
    if obs.rho_0_m is not None:
        return U_obs_distance(xi, eta, obs, obs.rho_0_m)
    return U_obs_gaussian(xi, eta, obs)


def U_ice_linear(
    xi: float, eta: float,
    cells: List[IceCell],
    k_ice: float = 1.0,
) -> float:
    """빙 비용: k_ice · max_c(w_c · C_c),  w_c = Gaussian 가중치."""
    if not cells:
        return 0.0
    best = 0.0
    for c in cells:
        r = _rho(xi, eta, c.xi_m, c.eta_m)
        w = math.exp(-(r ** 2) / (2.0 * c.sigma_m ** 2))
        best = max(best, w * c.C_ice)
    return k_ice * best


def U_total(
    xi: float, eta: float,
    wp: Waypoint,
    obstacles: List[Obstacle],
    memory_wells: Optional[Callable[[float, float], float]] = None,
    ice_cells: Optional[List[IceCell]] = None,
    k_goal: float = DEFAULT_K_GOAL,
    k_ice: float = 1.0,
    rho_0: float = RHO_0_DEFAULT,
) -> float:
    """U = U_goal + ΣU_obs + U_mem + U_ice."""
    u = U_goal(xi, eta, wp, k_goal)
    for obs in obstacles:
        u += U_obs(xi, eta, obs, rho_0)
    if memory_wells is not None:
        u += memory_wells(xi, eta)
    if ice_cells:
        u += U_ice_linear(xi, eta, ice_cells, k_ice)
    return u


# ── Gradient 메서드 Enum ──────────────────────────────────────────────────────

class GradientMethod(str, Enum):
    """Gradient 계산 방식."""
    ANALYTIC = "analytic"   # 해석적 (O(N), 권장)
    NUMERIC  = "numeric"    # 유한 차분 (legacy)


# ── 해석적 Gradient — 내부 순수 함수 ─────────────────────────────────────────

def _grad_U_goal(
    xi: float, eta: float,
    wp: Waypoint,
    k_goal: float,
) -> Tuple[float, float]:
    """∇U_goal = (k·Δξ, k·Δη).  (dU/dξ, dU/dη) 반환."""
    return k_goal * (xi - wp.xi_m), k_goal * (eta - wp.eta_m)


def _grad_U_obs_gaussian(
    xi: float, eta: float,
    obs: Obstacle,
) -> Tuple[float, float]:
    """∇U_gauss = val·(−Δξ/σ², −Δη/σ²).  (dU/dξ, dU/dη) 반환."""
    dx   = xi  - obs.xi_m
    dy   = eta - obs.eta_m
    r2   = dx * dx + dy * dy
    sig2 = obs.sigma_m ** 2
    val  = obs.A * math.exp(-r2 / (2.0 * sig2))
    return val * (-dx / sig2), val * (-dy / sig2)


def _grad_U_obs_distance(
    xi: float, eta: float,
    obs: Obstacle,
    rho_0: float,
) -> Tuple[float, float]:
    """∇U_dist = −A·(1/ρ−1/ρ₀)·(Δξ,Δη)/ρ³.  ρ ≥ ρ₀ 또는 ρ≈0 → (0,0).

    유도
    ────
    U   = (1/2)·A·(1/ρ − 1/ρ₀)²
    dU/dρ = A·(1/ρ − 1/ρ₀)·(−1/ρ²)
    dρ/dξ = Δξ/ρ
    dU/dξ = (dU/dρ)·(dρ/dξ) = −A·(1/ρ−1/ρ₀)·Δξ/ρ³
    """
    dx = xi  - obs.xi_m
    dy = eta - obs.eta_m
    r  = math.sqrt(dx * dx + dy * dy)
    if r >= rho_0 or r < 1e-6:
        return 0.0, 0.0
    coeff = -obs.A * (1.0 / r - 1.0 / rho_0) / (r * r * r)
    return coeff * dx, coeff * dy


def _grad_U_ice_argmax(
    xi: float, eta: float,
    cells: List[IceCell],
    k_ice: float,
) -> Tuple[float, float]:
    """∇U_ice (argmax 근사): 지배 Gaussian 셀 해석적 미분.

    U_ice ≈ k_ice · w_{c*} · C_{c*}   ,  c* = argmax_c(w_c · C_c)
    dU_ice/dξ ≈ k_ice · C_{c*} · w_{c*} · (−Δξ_{c*}/σ_{c*}²)

    비고: max() 연산의 불연속점 근방에서 근사 오차 발생 가능.
          일반적 운항 조건에서는 지배 셀이 자명하여 충분히 정확.
    """
    if not cells:
        return 0.0, 0.0

    best_val:  float             = -1.0
    best_cell: Optional[IceCell] = None
    best_w:    float             = 0.0

    for c in cells:
        r = _rho(xi, eta, c.xi_m, c.eta_m)
        w = math.exp(-r * r / (2.0 * c.sigma_m ** 2))
        contrib = w * c.C_ice
        if contrib > best_val:
            best_val  = contrib
            best_cell = c
            best_w    = w

    if best_cell is None or best_val <= 0.0:
        return 0.0, 0.0

    dx   = xi  - best_cell.xi_m
    dy   = eta - best_cell.eta_m
    sig2 = best_cell.sigma_m ** 2
    dU_dxi  = k_ice * best_cell.C_ice * best_w * (-dx  / sig2)
    dU_deta = k_ice * best_cell.C_ice * best_w * (-dy  / sig2)
    return dU_dxi, dU_deta


# ── 해석적 −∇U ────────────────────────────────────────────────────────────────

def gradient_U_analytic(
    xi: float, eta: float,
    wp: Waypoint,
    obstacles: List[Obstacle],
    memory_wells: Optional[Callable[[float, float], float]] = None,
    ice_cells: Optional[List[IceCell]] = None,
    k_goal: float = DEFAULT_K_GOAL,
    k_ice: float = 1.0,
    rho_0: float = RHO_0_DEFAULT,
    eps: float = 1e-4,
) -> Tuple[float, float]:
    """해석적 −∇U(ξ, η).

    각 항목 해석적 gradient 합산 후 부호 반전하여 인력/반발력 벡터 반환.
    memory_wells callable은 해석적 미분 불가 → 수치 전방 차분 보완 (eps 사용).

    Returns
    -------
    (F_xi, F_eta) : −∇U 벡터 성분 [N 또는 무차원 힘].
    """
    # ── goal gradient ────────────────────────────────────────────────────────
    g_xi, g_eta = _grad_U_goal(xi, eta, wp, k_goal)

    # ── obstacle gradient ────────────────────────────────────────────────────
    for obs in obstacles:
        rho_0_eff = obs.rho_0_m if obs.rho_0_m is not None else None
        if rho_0_eff is not None:
            dxi, deta = _grad_U_obs_distance(xi, eta, obs, rho_0_eff)
        else:
            dxi, deta = _grad_U_obs_gaussian(xi, eta, obs)
        g_xi  += dxi
        g_eta += deta

    # ── ice gradient ─────────────────────────────────────────────────────────
    if ice_cells:
        dxi, deta = _grad_U_ice_argmax(xi, eta, ice_cells, k_ice)
        g_xi  += dxi
        g_eta += deta

    # ── memory wells: 수치 보완 ───────────────────────────────────────────────
    if memory_wells is not None:
        m0    = memory_wells(xi, eta)
        dxi   = (memory_wells(xi + eps, eta) - m0) / eps
        deta  = (memory_wells(xi, eta + eps) - m0) / eps
        g_xi  += dxi
        g_eta += deta

    return -g_xi, -g_eta


# ── 수치 −∇U (legacy) ─────────────────────────────────────────────────────────

def gradient_U_numeric(
    xi: float, eta: float,
    wp: Waypoint,
    obstacles: List[Obstacle],
    memory_wells: Optional[Callable[[float, float], float]] = None,
    ice_cells: Optional[List[IceCell]] = None,
    k_goal: float = DEFAULT_K_GOAL,
    k_ice: float = 1.0,
    rho_0: float = RHO_0_DEFAULT,
    eps: float = 1e-4,
) -> Tuple[float, float]:
    """-∇U 유한 차분 (전방 차분).  O(2N_eval). Legacy 목적으로 유지."""
    U0   = U_total(xi,       eta,       wp, obstacles, memory_wells, ice_cells,
                   k_goal, k_ice, rho_0)
    dxi  = (U_total(xi + eps, eta,       wp, obstacles, memory_wells, ice_cells,
                    k_goal, k_ice, rho_0) - U0) / eps
    deta = (U_total(xi,       eta + eps, wp, obstacles, memory_wells, ice_cells,
                    k_goal, k_ice, rho_0) - U0) / eps
    return -dxi, -deta


# ── 통합 Dispatcher ───────────────────────────────────────────────────────────

def gradient_U(
    xi: float, eta: float,
    wp: Waypoint,
    obstacles: List[Obstacle],
    memory_wells: Optional[Callable[[float, float], float]] = None,
    ice_cells: Optional[List[IceCell]] = None,
    k_goal: float = DEFAULT_K_GOAL,
    k_ice: float = 1.0,
    rho_0: float = RHO_0_DEFAULT,
    eps: float = 1e-4,
    method: GradientMethod = GradientMethod.ANALYTIC,
) -> Tuple[float, float]:
    """-∇U 통합 인터페이스.

    Parameters
    ----------
    method : GradientMethod
        ANALYTIC (기본, O(N)) 또는 NUMERIC (유한 차분, legacy).

    Returns
    -------
    (F_xi, F_eta) : -∇U 벡터. path_controller 의 방향 계산에 직접 사용.
    """
    if method is GradientMethod.NUMERIC:
        return gradient_U_numeric(
            xi, eta, wp, obstacles, memory_wells, ice_cells,
            k_goal, k_ice, rho_0, eps,
        )
    return gradient_U_analytic(
        xi, eta, wp, obstacles, memory_wells, ice_cells,
        k_goal, k_ice, rho_0, eps,
    )


# ── 방향 변환 ─────────────────────────────────────────────────────────────────

def psi_ref_from_gradient(dU_dxi: float, dU_deta: float) -> float:
    """ψ_ref = atan2(F_eta, F_xi) [rad].  인자: −∇U 성분 (= 힘 벡터 F_pf)."""
    return math.atan2(dU_deta, dU_dxi)
