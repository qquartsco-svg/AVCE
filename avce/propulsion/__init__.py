"""Propulsion: 추진 시스템 (핵추진, 디젤-전기, 통합 관리자)."""
from .nuclear import (
    ReactorStatus,
    ReactorParams,
    ReactorState,
    NuclearReactor,
)
from .propulsion_manager import (
    PropulsionType,
    DieselParams,
    BatteryParams,
    PropulsionManager,
)

__all__ = [
    # ── nuclear ──────────────────────────────────────────────────────────────
    "ReactorStatus",
    "ReactorParams",
    "ReactorState",
    "NuclearReactor",
    # ── propulsion manager ───────────────────────────────────────────────────
    "PropulsionType",
    "DieselParams",
    "BatteryParams",
    "PropulsionManager",
]
