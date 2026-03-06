"""
깊이·피치·롤 제어 (AVCE §CTRL.DEPTH).

DepthControlConfig : 깊이/피치/롤 P 제어 파라미터 (frozen).
DepthController    : z_ref, θ_ref → (τ_w, τ_q, τ_p) 계산.
                     잠수 중 깊이 유지 / 부상 기동 모두 지원.

수식
────
τ_w = k_z     · (z_ref − z)      [N]     heave 제어 (깊이)
τ_q = k_theta · (θ_ref − θ)      [N·m]   pitch 제어 (자세)
τ_p = k_phi   · (0 − φ)          [N·m]   roll  제어 (수평 유지)
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple
from ..core.state6dof import VehicleState

_Tau6 = Tuple[float, float, float, float, float, float]

def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

@dataclass(frozen=True)
class DepthControlConfig:
    """깊이·피치·롤 P 제어 파라미터 (불변)."""
    k_z:         float = 1.0e4    # 깊이 게인    [N/m]
    k_theta:     float = 5.0e5   # 피치 게인    [N·m/rad]
    k_phi:       float = 3.0e5   # 롤  게인     [N·m/rad]
    k_z_d:       float = 2.0e3   # 깊이 미분 (heave 속도 감쇠)
    k_theta_d:   float = 1.0e5   # 피치 미분 (pitch rate 감쇠)
    max_tau_w:   float = 5.0e5   # 최대 heave 힘  [N]
    max_tau_q:   float = 2.0e6   # 최대 pitch 모멘트 [N·m]
    max_tau_p:   float = 1.0e6   # 최대 roll  모멘트 [N·m]
    z_deadband_m: float = 0.1    # 깊이 데드밴드 [m]

class DepthController:
    """깊이·피치·롤 PD 제어기.

    Parameters
    ----------
    config : DepthControlConfig
    """
    def __init__(self, config: DepthControlConfig = DepthControlConfig()):
        self._cfg = config

    def step(
        self,
        state: VehicleState,
        z_ref: float,
        theta_ref: float = 0.0,
    ) -> _Tau6:
        """제어 출력 계산.

        Returns
        -------
        τ_6 : (0, 0, τ_w, τ_p, τ_q, 0)
        """
        cfg = self._cfg
        # 깊이 오차 (데드밴드 적용)
        dz = z_ref - state.z_m
        if abs(dz) < cfg.z_deadband_m:
            dz = 0.0
        # 피치·롤 오차
        d_theta = _wrap(theta_ref - state.theta_rad)
        d_phi   = _wrap(0.0       - state.phi_rad)

        # PD 제어
        tau_w = cfg.k_z * dz - cfg.k_z_d * state.w_ms
        tau_q = cfg.k_theta * d_theta - cfg.k_theta_d * state.q_rads
        tau_p = cfg.k_phi   * d_phi

        # 포화
        tau_w = max(-cfg.max_tau_w, min(cfg.max_tau_w, tau_w))
        tau_q = max(-cfg.max_tau_q, min(cfg.max_tau_q, tau_q))
        tau_p = max(-cfg.max_tau_p, min(cfg.max_tau_p, tau_p))

        return (0.0, 0.0, tau_w, tau_p, tau_q, 0.0)

    def emergency_surface_tau(self, state: VehicleState) -> _Tau6:
        """비상 부상: 최대 부력 + 상향 피치."""
        cfg = self._cfg
        tau_w = -cfg.max_tau_w         # 최대 heave 상향
        tau_q = cfg.k_theta * _wrap(math.radians(-20.0) - state.theta_rad)
        tau_q = max(-cfg.max_tau_q, min(cfg.max_tau_q, tau_q))
        tau_p = cfg.k_phi * _wrap(-state.phi_rad)
        return (0.0, 0.0, tau_w, tau_p, tau_q, 0.0)
