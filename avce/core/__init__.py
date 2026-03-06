"""Core: 상태·상수·계약 (EQUATIONS.md §1–2, §7, §9, §10)
6-DOF 확장: VehicleState(12D), IceLayer(3D), OceanCell, VesselMode(8종), ModeConfig/Profile
"""
from .state import VesselState, Waypoint, Obstacle, AxisSetpoint, IceCell
from .constants import (
    DEFAULT_K_GOAL, DEFAULT_K_OBS, RHO_0_DEFAULT,
    GAUSSIAN_SIGMA_MIN,
    PRECISION_DEG, PRECISION_RAD, PREDICTION_HORIZON_MS,
    FAILSAFE_MS,
    CONTEXT_OPEN_WATER, CONTEXT_ICE_TRANSIT, CONTEXT_ICE_RAM,
)
from .ramming import RammingImpact, RammingDisturbance, tau_ram_at
from .estimator import (
    StateEstimator,
    IdentityEstimator,
    KalmanConfig,
    KalmanEstimator,
)
from .sequencer import (
    ArrivalConfig,
    SpeedConfig,
    SequencerStatus,
    SequencerResult,
    WaypointSequencer,
)
# ── 6-DOF 확장 ────────────────────────────────────────────────────────────────
from .state6dof import VehicleState, IceLayer, OceanCell
from .mode import (
    VesselMode,
    ModeConfig,
    ModeProfile,
    default_mode_config,
    allowed_transitions,
    can_transition,
)

__all__ = [
    # ── 3-DOF state ──────────────────────────────────────────────────────────
    "VesselState",
    "Waypoint",
    "Obstacle",
    "AxisSetpoint",
    "IceCell",
    # ── constants ────────────────────────────────────────────────────────────
    "DEFAULT_K_GOAL",
    "DEFAULT_K_OBS",
    "RHO_0_DEFAULT",
    "GAUSSIAN_SIGMA_MIN",
    "PRECISION_DEG",
    "PRECISION_RAD",
    "PREDICTION_HORIZON_MS",
    "FAILSAFE_MS",
    "CONTEXT_OPEN_WATER",
    "CONTEXT_ICE_TRANSIT",
    "CONTEXT_ICE_RAM",
    # ── ramming ──────────────────────────────────────────────────────────────
    "RammingImpact",
    "RammingDisturbance",
    "tau_ram_at",
    # ── estimator ────────────────────────────────────────────────────────────
    "StateEstimator",
    "IdentityEstimator",
    "KalmanConfig",
    "KalmanEstimator",
    # ── sequencer ────────────────────────────────────────────────────────────
    "ArrivalConfig",
    "SpeedConfig",
    "SequencerStatus",
    "SequencerResult",
    "WaypointSequencer",
    # ── 6-DOF state ──────────────────────────────────────────────────────────
    "VehicleState",
    "IceLayer",
    "OceanCell",
    # ── mode ─────────────────────────────────────────────────────────────────
    "VesselMode",
    "ModeConfig",
    "ModeProfile",
    "default_mode_config",
    "allowed_transitions",
    "can_transition",
]
