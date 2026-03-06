"""Autonomous Vessel Control Engine — 자율선박 제어 (확장·정밀)

개념·수식: docs/CONCEPTS.md, docs/EQUATIONS.md.
정밀도: docs/PRECISION_CONTROL_ANALYSIS.md.

공개:
  - core: VesselState, Waypoint, Obstacle, IceCell, 상수(맥락 라벨 포함)
           KalmanConfig, KalmanEstimator (EKF-style 추정)
           ArrivalConfig, SpeedConfig, WaypointSequencer (다중 waypoint)
  - control: potential_field (해석적/수치 gradient), path_controller → ψ_ref, U_ref
  - memory: WellMemory (맥락별 우물)
  - integration: OrbitStabilizerAdapter (다축), Grid5DAdapter (선택)
  - VesselController: 한 스텝에 경로 + 보정 합성
  - simulation: DynamicsParams, IntegrationMethod (RK4/Euler),
                simulation_loop_step, run_simulation
"""

from .core import (
    VesselState,
    Waypoint,
    Obstacle,
    AxisSetpoint,
    IceCell,
    DEFAULT_K_GOAL,
    DEFAULT_K_OBS,
    RHO_0_DEFAULT,
    GAUSSIAN_SIGMA_MIN,
    PRECISION_DEG,
    PRECISION_RAD,
    PREDICTION_HORIZON_MS,
    FAILSAFE_MS,
    CONTEXT_OPEN_WATER,
    CONTEXT_ICE_TRANSIT,
    CONTEXT_ICE_RAM,
    RammingImpact,
    RammingDisturbance,
    tau_ram_at,
    StateEstimator,
    IdentityEstimator,
    KalmanConfig,
    KalmanEstimator,
    ArrivalConfig,
    SpeedConfig,
    SequencerStatus,
    SequencerResult,
    WaypointSequencer,
)
from .control import (
    path_controller,
    PathOutput,
    CerebellumProfile,
    ProfilePoint,
    exponential_profile,
    GradientMethod,
    gradient_U_analytic,
    gradient_U_numeric,
    gradient_U,
    psi_ref_from_gradient,
    U_goal,
    U_obs,
    U_total,
)
from .memory import WellMemory, Well, U_mem_at
from .integration import (
    OrbitStabilizerAdapter,
    AxisCorrection,
    Grid5DAdapter,
    Grid5DRefinement,
    vessel_state_to_5d_phase,
)
from .controller import VesselController, VesselStepResult
from .simulation import (
    DynamicsParams,
    IntegrationMethod,
    step_3dof,
    step_3dof_rk4,
    step_3dof_method,
    tau_from_heading_and_surge,
    simulation_loop_step,
    run_simulation,
    SimulationStepResult,
)

__all__ = [
    # ── core: state ──────────────────────────────────────────────────────────
    "VesselState",
    "Waypoint",
    "Obstacle",
    "IceCell",
    "AxisSetpoint",
    # ── core: constants ──────────────────────────────────────────────────────
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
    # ── core: ramming ────────────────────────────────────────────────────────
    "RammingImpact",
    "RammingDisturbance",
    "tau_ram_at",
    # ── core: estimator ──────────────────────────────────────────────────────
    "StateEstimator",
    "IdentityEstimator",
    "KalmanConfig",
    "KalmanEstimator",
    # ── core: sequencer ──────────────────────────────────────────────────────
    "ArrivalConfig",
    "SpeedConfig",
    "SequencerStatus",
    "SequencerResult",
    "WaypointSequencer",
    # ── control: path ────────────────────────────────────────────────────────
    "path_controller",
    "PathOutput",
    # ── control: potential field ─────────────────────────────────────────────
    "GradientMethod",
    "gradient_U_analytic",
    "gradient_U_numeric",
    "gradient_U",
    "psi_ref_from_gradient",
    "U_goal",
    "U_obs",
    "U_total",
    # ── control: cerebellum ──────────────────────────────────────────────────
    "CerebellumProfile",
    "ProfilePoint",
    "exponential_profile",
    # ── memory ───────────────────────────────────────────────────────────────
    "WellMemory",
    "Well",
    "U_mem_at",
    # ── integration ──────────────────────────────────────────────────────────
    "OrbitStabilizerAdapter",
    "AxisCorrection",
    "Grid5DAdapter",
    "Grid5DRefinement",
    "vessel_state_to_5d_phase",
    # ── controller ───────────────────────────────────────────────────────────
    "VesselController",
    "VesselStepResult",
    # ── simulation ───────────────────────────────────────────────────────────
    "DynamicsParams",
    "IntegrationMethod",
    "step_3dof",
    "step_3dof_rk4",
    "step_3dof_method",
    "tau_from_heading_and_surge",
    "simulation_loop_step",
    "run_simulation",
    "SimulationStepResult",
]
