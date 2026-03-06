"""
OrbitStabilizer 연동 (선택 의존).
다축: 축별 OrbitStabilizer 인스턴스 → 보정력 합성.
EQUATIONS §4, PRECISION_CONTROL_ANALYSIS.
"""
from __future__ import annotations
from typing import Dict, Optional, List, Any
from dataclasses import dataclass

from ..core.constants import PREDICTION_HORIZON_MS

_OrbitStabilizer = None
_StabilizationTerm = None

def _load_orbit_stabilizer() -> bool:
    global _OrbitStabilizer, _StabilizationTerm
    if _OrbitStabilizer is not None:
        return _StabilizationTerm is not None
    try:
        from orbit_stabilizer import OrbitStabilizer, StabilizationTerm
        _OrbitStabilizer, _StabilizationTerm = OrbitStabilizer, StabilizationTerm
        return True
    except ImportError:
        try:
            from hippo_memory.orbit_stabilizer import OrbitStabilizer, StabilizationTerm
            _OrbitStabilizer, _StabilizationTerm = OrbitStabilizer, StabilizationTerm
            return True
        except ImportError:
            return False


@dataclass
class AxisCorrection:
    """축별 보정 출력"""
    axis_id: str
    correction_force: float
    predicted_error: float
    confidence: float
    wear_reduction: float


class OrbitStabilizerAdapter:
    """
    다축 정밀 보정: 축별 OrbitStabilizer.
    - axis_ids: e.g. ["psi", "phi"] (선수 방위각, 추진축 위상)
    - update(axis_id, sensor_value_rad, target_value_rad) → AxisCorrection
    """
    def __init__(
        self,
        axis_ids: List[str],
        ring_size: int = 15,
        config: str = "case2",
        prediction_horizon_ms: float = PREDICTION_HORIZON_MS,
        **stabilizer_kw: Any,
    ):
        self._axis_ids = list(axis_ids)
        self._prediction_horizon_ms = prediction_horizon_ms
        self._stabilizers: Dict[str, Any] = {}
        self._available = _load_orbit_stabilizer()
        if self._available and _OrbitStabilizer is not None:
            for aid in self._axis_ids:
                self._stabilizers[aid] = _OrbitStabilizer(
                    size=ring_size, config=config, **stabilizer_kw
                )

    @property
    def available(self) -> bool:
        return self._available

    def update(
        self,
        axis_id: str,
        sensor_value_rad: float,
        target_value_rad: float,
        prediction_horizon_ms: Optional[float] = None,
    ) -> AxisCorrection:
        """
        한 축에 대해 보정항 계산.
        OrbitStabilizer 미설치 시 correction_force=0, confidence=0 반환.
        """
        ph = prediction_horizon_ms if prediction_horizon_ms is not None else self._prediction_horizon_ms
        if not self._available or axis_id not in self._stabilizers:
            return AxisCorrection(
                axis_id=axis_id,
                correction_force=0.0,
                predicted_error=0.0,
                confidence=0.0,
                wear_reduction=0.0,
            )
        stab = self._stabilizers[axis_id]
        term = stab.update(sensor_value_rad, target_value_rad, ph)
        return AxisCorrection(
            axis_id=axis_id,
            correction_force=term.correction_force,
            predicted_error=term.predicted_error,
            confidence=term.confidence,
            wear_reduction=term.wear_reduction,
        )

    def update_multi(
        self,
        sensor_targets: Dict[str, tuple[float, float]],
        prediction_horizon_ms: Optional[float] = None,
    ) -> Dict[str, AxisCorrection]:
        """여러 축 동시 업데이트. sensor_targets: { axis_id: (sensor_rad, target_rad) }"""
        out: Dict[str, AxisCorrection] = {}
        for aid, (sensor_rad, target_rad) in sensor_targets.items():
            out[aid] = self.update(aid, sensor_rad, target_rad, prediction_horizon_ms)
        return out
