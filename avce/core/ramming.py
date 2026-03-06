"""
쇄빙 램 외란 τ_ram(t) (EQUATIONS §10.2).
τ_ram(t) = T_ram * exp(-(t - t_impact)^2 / (2 σ_ram^2))
"""
from __future__ import annotations
import math
from typing import Optional
from dataclasses import dataclass


@dataclass
class RammingImpact:
    """단일 충돌 이벤트"""
    t_impact_s: float
    T_ram_nm: float   # 피크 토크 [N·m]
    sigma_ram_s: float = 1.0


def tau_ram_at(t_s: float, impact: RammingImpact) -> float:
    """τ_ram(t) = T_ram * exp(-(t - t_impact)^2 / (2 σ_ram^2))."""
    d = t_s - impact.t_impact_s
    if impact.sigma_ram_s <= 0:
        return 0.0
    return impact.T_ram_nm * math.exp(-(d ** 2) / (2.0 * impact.sigma_ram_s ** 2))


class RammingDisturbance:
    """
    여러 램 이벤트 합산. 현재 시각 t_s에 대해 Σ τ_ram(t_s; 각 impact).
    """
    def __init__(self):
        self._impacts: list[RammingImpact] = []

    def add_impact(self, impact: RammingImpact) -> None:
        self._impacts.append(impact)

    def tau_ram(self, t_s: float, cutoff_s: float = 10.0) -> float:
        """
        t_s에서 총 τ_ram. t_impact로부터 cutoff_s 이상 지난 이벤트는 무시(선택).
        """
        total = 0.0
        for imp in self._impacts:
            if abs(t_s - imp.t_impact_s) > cutoff_s * max(1.0, imp.sigma_ram_s):
                continue
            total += tau_ram_at(t_s, imp)
        return total

    def clear_before(self, t_s: float) -> None:
        """t_s 이전 충돌 이벤트 제거 (메모리 관리)."""
        self._impacts = [i for i in self._impacts if i.t_impact_s >= t_s]
