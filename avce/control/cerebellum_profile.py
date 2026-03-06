"""
Cerebellum-style 운동 패턴 (EQUATIONS §6).
ψ_ref(t), U_ref(t) 단순 형태: 선형+감쇠. 맥락별 프로파일 재생.
"""
from __future__ import annotations
import math
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass

from ..core.constants import CONTEXT_ICE_RAM, CONTEXT_ICE_TRANSIT, CONTEXT_OPEN_WATER


@dataclass
class ProfilePoint:
    """시점 t에서의 참조"""
    t_s: float
    psi_ref_rad: float
    U_ref_ms: float


def exponential_profile(
    t_s: float,
    psi_0_rad: float,
    psi_goal_rad: float,
    U_0_ms: float,
    U_goal_ms: float,
    T_psi_s: float = 10.0,
    T_U_s: float = 15.0,
) -> Tuple[float, float]:
    """
    EQUATIONS §6.2:
    ψ_ref(t) = ψ_0 + (ψ_goal - ψ_0)(1 - exp(-t/T_ψ))
    U_ref(t) = U_0 + (U_goal - U_0)(1 - exp(-t/T_U))
    """
    if T_psi_s <= 0:
        T_psi_s = 1e-6
    if T_U_s <= 0:
        T_U_s = 1e-6
    psi_ref = psi_0_rad + (psi_goal_rad - psi_0_rad) * (1.0 - math.exp(-t_s / T_psi_s))
    U_ref = U_0_ms + (U_goal_ms - U_0_ms) * (1.0 - math.exp(-t_s / T_U_s))
    return psi_ref, max(0.0, U_ref)


class CerebellumProfile:
    """
    맥락별 (ψ_ref, U_ref) 프로파일.
    - 단순 모드: exponential_profile(t, ψ_0, ψ_goal, U_0, U_goal, T_ψ, T_U)
    - 재생 모드: 저장된 시퀀스 [ProfilePoint] 재생
    """
    def __init__(
        self,
        T_psi_s: float = 10.0,
        T_U_s: float = 15.0,
    ):
        self.T_psi_s = T_psi_s
        self.T_U_s = T_U_s
        self._sequences: Dict[str, List[ProfilePoint]] = {}

    def add_sequence(self, profile_id: str, points: List[ProfilePoint]) -> None:
        self._sequences[profile_id] = sorted(points, key=lambda p: p.t_s)

    def get_at_t(
        self,
        t_s: float,
        psi_0_rad: float,
        psi_goal_rad: float,
        U_0_ms: float,
        U_goal_ms: float,
        context: str = CONTEXT_OPEN_WATER,
        profile_id: Optional[str] = None,
    ) -> Tuple[float, float]:
        """
        t_s에서 (psi_ref, U_ref). profile_id가 있고 해당 시퀀스가 있으면 재생, 아니면 단순 감쇠.
        """
        if profile_id and profile_id in self._sequences:
            seq = self._sequences[profile_id]
            if not seq:
                return exponential_profile(t_s, psi_0_rad, psi_goal_rad, U_0_ms, U_goal_ms, self.T_psi_s, self.T_U_s)
            # 보간
            if t_s <= seq[0].t_s:
                return seq[0].psi_ref_rad, seq[0].U_ref_ms
            if t_s >= seq[-1].t_s:
                return seq[-1].psi_ref_rad, seq[-1].U_ref_ms
            for i in range(len(seq) - 1):
                if seq[i].t_s <= t_s <= seq[i + 1].t_s:
                    a = (t_s - seq[i].t_s) / (seq[i + 1].t_s - seq[i].t_s)
                    psi = seq[i].psi_ref_rad + a * (seq[i + 1].psi_ref_rad - seq[i].psi_ref_rad)
                    u = seq[i].U_ref_ms + a * (seq[i + 1].U_ref_ms - seq[i].U_ref_ms)
                    return psi, max(0.0, u)
        # 맥락별 기본 (쇄빙선)
        if context == CONTEXT_ICE_RAM:
            U_goal_ms = min(U_goal_ms, 2.0)  # 램 시 저속
        return exponential_profile(t_s, psi_0_rad, psi_goal_rad, U_0_ms, U_goal_ms, self.T_psi_s, self.T_U_s)
