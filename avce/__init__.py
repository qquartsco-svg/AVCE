"""Autonomous Vessel Control Engine — 자율선박 제어 (확장·정밀)

개념·수식: docs/CONCEPTS.md, docs/EQUATIONS.md.
정밀도: docs/PRECISION_CONTROL_ANALYSIS.md.

공개 API (하이브리드 다중모드 자율선박 제어 시스템):
  ── core (3-DOF) ─────────────────────────────────────────────────────────────
  · VesselState, Waypoint, Obstacle, IceCell, AxisSetpoint
  · 상수: DEFAULT_K_GOAL, DEFAULT_K_OBS, RHO_0_DEFAULT, GAUSSIAN_SIGMA_MIN
          PRECISION_DEG/RAD, PREDICTION_HORIZON_MS, FAILSAFE_MS
          CONTEXT_OPEN_WATER, CONTEXT_ICE_TRANSIT, CONTEXT_ICE_RAM
  · KalmanConfig, KalmanEstimator (EKF-style)
  · ArrivalConfig, SpeedConfig, WaypointSequencer (다중 waypoint)
  ── core (6-DOF) ─────────────────────────────────────────────────────────────
  · VehicleState (12D: ξ,η,z,φ,θ,ψ,u,v,w,p,q,r)
  · IceLayer (3D: 수면/천장 빙, z_top/z_bottom)
  · OceanCell (수온·염분·밀도·3D 조류)
  ── mode ─────────────────────────────────────────────────────────────────────
  · VesselMode (8종: SURFACE/ICEBREAKER/DIVE/UNDER_ICE/
                SUBMERGED_ICEBREAKER/TRANSIT_DIVE/TRANSIT_SURFACE/EMERGENCY_SURFACE)
  · ModeConfig, ModeProfile, default_mode_config, can_transition
  · ModeManager, TransitionResult
  ── control ──────────────────────────────────────────────────────────────────
  · potential_field (해석적/수치 gradient), path_controller → ψ_ref, U_ref
  · DepthController (PD: z/θ/φ), IceInterface (3D 빙 분석·파쇄)
  ── simulation ───────────────────────────────────────────────────────────────
  · 3-DOF: DynamicsParams, IntegrationMethod (RK4/Euler), step_3dof_rk4/method
  · 6-DOF: DynamicsParams6DOF (Fossen), step_6dof_rk4/method, tau6_from_setpoints
  · BuoyancyParams, BuoyancyModel, OceanEnvironment
  · simulation_loop_step, run_simulation, SimulationStepResult
  ── propulsion ───────────────────────────────────────────────────────────────
  · NuclearReactor (PWR: 1차 지연 + SCRAM), ReactorParams, ReactorStatus, ReactorState
  · PropulsionType (NUCLEAR/DIESEL_ELECTRIC/HYBRID/EMERGENCY_BATTERY)
  · DieselParams, BatteryParams, PropulsionManager
  ── memory · integration · controller ────────────────────────────────────────
  · WellMemory, OrbitStabilizerAdapter, Grid5DAdapter, VesselController
"""

from .core import (
    # 3-DOF state
    VesselState,
    Waypoint,
    Obstacle,
    AxisSetpoint,
    IceCell,
    # constants
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
    # ramming
    RammingImpact,
    RammingDisturbance,
    tau_ram_at,
    # estimator
    StateEstimator,
    IdentityEstimator,
    KalmanConfig,
    KalmanEstimator,
    # sequencer
    ArrivalConfig,
    SpeedConfig,
    SequencerStatus,
    SequencerResult,
    WaypointSequencer,
    # 6-DOF state
    VehicleState,
    IceLayer,
    OceanCell,
    # mode
    VesselMode,
    ModeConfig,
    ModeProfile,
    default_mode_config,
    allowed_transitions,
    can_transition,
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
    # 6-DOF control
    DepthControlConfig,
    DepthController,
    IceInterfaceConfig,
    IceContactResult,
    IceInterface,
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
    # 3-DOF dynamics
    DynamicsParams,
    IntegrationMethod,
    step_3dof,
    step_3dof_rk4,
    step_3dof_method,
    tau_from_heading_and_surge,
    # loop
    simulation_loop_step,
    run_simulation,
    SimulationStepResult,
    # 6-DOF dynamics
    DynamicsParams6DOF,
    step_6dof,
    step_6dof_rk4,
    step_6dof_method,
    tau6_from_setpoints,
    # environment
    BuoyancyParams,
    BuoyancyModel,
    OceanEnvironment,
)
from .propulsion import (
    ReactorStatus,
    ReactorParams,
    ReactorState,
    NuclearReactor,
    PropulsionType,
    DieselParams,
    BatteryParams,
    PropulsionManager,
)
from .mode import (
    TransitionResult,
    ModeManager,
)

__all__ = [
    # ── core: 3-DOF state ────────────────────────────────────────────────────
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
    # ── core: 6-DOF state ────────────────────────────────────────────────────
    "VehicleState",
    "IceLayer",
    "OceanCell",
    # ── core: mode ───────────────────────────────────────────────────────────
    "VesselMode",
    "ModeConfig",
    "ModeProfile",
    "default_mode_config",
    "allowed_transitions",
    "can_transition",
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
    # ── control: 6-DOF ───────────────────────────────────────────────────────
    "DepthControlConfig",
    "DepthController",
    "IceInterfaceConfig",
    "IceContactResult",
    "IceInterface",
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
    # ── simulation: 3-DOF ────────────────────────────────────────────────────
    "DynamicsParams",
    "IntegrationMethod",
    "step_3dof",
    "step_3dof_rk4",
    "step_3dof_method",
    "tau_from_heading_and_surge",
    "simulation_loop_step",
    "run_simulation",
    "SimulationStepResult",
    # ── simulation: 6-DOF ────────────────────────────────────────────────────
    "DynamicsParams6DOF",
    "step_6dof",
    "step_6dof_rk4",
    "step_6dof_method",
    "tau6_from_setpoints",
    # ── simulation: environment ───────────────────────────────────────────────
    "BuoyancyParams",
    "BuoyancyModel",
    "OceanEnvironment",
    # ── propulsion ───────────────────────────────────────────────────────────
    "ReactorStatus",
    "ReactorParams",
    "ReactorState",
    "NuclearReactor",
    "PropulsionType",
    "DieselParams",
    "BatteryParams",
    "PropulsionManager",
    # ── mode manager ─────────────────────────────────────────────────────────
    "TransitionResult",
    "ModeManager",
]
