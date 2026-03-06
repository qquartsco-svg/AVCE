"""Core: 상태·상수·계약 (EQUATIONS.md §1–2, §7, §9, §10)"""
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

__all__ = [
    # state
    "VesselState",
    "Waypoint",
    "Obstacle",
    "AxisSetpoint",
    "IceCell",
    # constants
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
    # ramming
    "RammingImpact",
    "RammingDisturbance",
    "tau_ram_at",
    # estimator
    "StateEstimator",
    "IdentityEstimator",
    "KalmanConfig",
    "KalmanEstimator",
    # sequencer
    "ArrivalConfig",
    "SpeedConfig",
    "SequencerStatus",
    "SequencerResult",
    "WaypointSequencer",
]
