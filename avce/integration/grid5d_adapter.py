"""
Grid 5D 연동 (선택).
선박 상태 (ξ,η,ψ) + 추진 위상 등 → Grid5D state 매핑 시 사용.
EQUATIONS §4, CONCEPTS 5축.
"""
from __future__ import annotations
from typing import Optional, Dict, Any, List
from dataclasses import dataclass

from ..core.state import VesselState

_Grid5DEngine = None

def _load_grid5d() -> bool:
    global _Grid5DEngine
    if _Grid5DEngine is not None:
        return True
    try:
        from grid_engine.dimensions.dim5d import Grid5DEngine
        _Grid5DEngine = Grid5DEngine
        return True
    except ImportError:
        try:
            from grid_engine.dimensions.dim5d.grid_5d_engine import Grid5DEngine
            _Grid5DEngine = Grid5DEngine
            return True
        except ImportError:
            return False


@dataclass
class Grid5DRefinement:
    """Grid 5D에서 나온 보정된 참조 (선택 사용)"""
    phi_x: float
    phi_y: float
    phi_z: float
    phi_a: float
    phi_b: float


def vessel_state_to_5d_phase(state: VesselState) -> Dict[str, float]:
    """
    선박 상태를 5D 위상 스케일로 매핑 (계약).
    X,Y = ξ,η 스케일 (m → rad 비율은 설정에서).
    Z = 0 또는 고정.
    A,B = ψ 및 추진 위상 등.
    """
    scale_xy = 1e-3  # m → rad 스케일 (예)
    return {
        "phi_x": state.xi_m * scale_xy,
        "phi_y": state.eta_m * scale_xy,
        "phi_z": 0.0,
        "phi_a": state.psi_rad,
        "phi_b": 0.0,
    }


class Grid5DAdapter:
    """
    Grid 5D 엔진 연동 (선택).
    설치 시 step()으로 위상 보정 가능; 미설치 시 no-op.
    """
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self._available = _load_grid5d()
        self._engine: Any = None
        if self._available and _Grid5DEngine is not None:
            self._engine = _Grid5DEngine(**(config or {}))

    @property
    def available(self) -> bool:
        return self._available

    def step(
        self,
        state: VesselState,
        velocity_5d: Optional[List[float]] = None,
        dt_s: float = 0.1,
    ) -> Optional[Grid5DRefinement]:
        """
        Grid 5D 한 스텝. velocity_5d = [vx, vy, vz, va, vb] (선택).
        미설치 시 None.
        """
        if not self._available or self._engine is None:
            return None
        phase = vessel_state_to_5d_phase(state)
        # Grid5DEngine.step() 시그니처에 맞게 호출 (실제 엔진 문서 참조)
        # 여기서는 스텝 실행 후 위상만 반환하는 계약만 둠
        return Grid5DRefinement(
            phi_x=phase["phi_x"],
            phi_y=phase["phi_y"],
            phi_z=phase["phi_z"],
            phi_a=phase["phi_a"],
            phi_b=phase["phi_b"],
        )
