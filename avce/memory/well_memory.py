"""
맥락별 기억 우물 (EQUATIONS §5).
U_mem(ξ,η; c) = Σ w_j f(ρ_j/σ_j). 리콜 시 W(c)만 활성화.
"""
from __future__ import annotations
import math
from typing import Dict, List, Optional
from dataclasses import dataclass, field

from ..core.constants import CONTEXT_OPEN_WATER


@dataclass
class Well:
    """단일 우물 (x, y, w, σ)"""
    x_m: float
    y_m: float
    w: float   # 강도 (흡인 <0, 반발 >0)
    sigma_m: float


def _rho(xi: float, eta: float, x: float, y: float) -> float:
    return math.sqrt((xi - x) ** 2 + (eta - y) ** 2)


def _f_gaussian(rho_sigma: float) -> float:
    """f(ρ/σ) = exp(-(ρ/σ)^2)"""
    return math.exp(-rho_sigma ** 2)


def U_mem_at(
    xi: float, eta: float,
    wells: List[Well],
) -> float:
    """U_mem = Σ w_j f(ρ_j/σ_j)."""
    u = 0.0
    for wl in wells:
        r = _rho(xi, eta, wl.x_m, wl.y_m)
        u += wl.w * _f_gaussian(r / wl.sigma_m) if wl.sigma_m > 0 else 0.0
    return u


class WellMemory:
    """
    맥락 c → W(c) 우물 집합. 시간 감쇠 지원.
    """
    def __init__(self, decay_tau_s: float = 3600.0):
        self._wells: Dict[str, List[Well]] = {}
        self._decay_tau_s = decay_tau_s
        self._time_s: float = 0.0

    def set_wells(self, context: str, wells: List[Well]) -> None:
        self._wells[context] = list(wells)

    def add_well(self, context: str, well: Well) -> None:
        if context not in self._wells:
            self._wells[context] = []
        self._wells[context].append(well)

    def get_wells(self, context: str) -> List[Well]:
        return list(self._wells.get(context, []))

    def U_mem(self, xi: float, eta: float, context: str) -> float:
        return U_mem_at(xi, eta, self.get_wells(context))

    def make_callback(self, context: str):
        """(xi, eta) -> U_mem(xi, eta; context) 콜백."""
        def cb(xi: float, eta: float) -> float:
            return self.U_mem(xi, eta, context)
        return cb

    def decay(self, dt_s: float) -> None:
        """시간 감쇠: w_j <- w_j * exp(-dt/tau)."""
        if self._decay_tau_s <= 0:
            return
        k = math.exp(-dt_s / self._decay_tau_s)
        for ctx, wells in self._wells.items():
            for wl in wells:
                wl.w *= k
        self._time_s += dt_s
