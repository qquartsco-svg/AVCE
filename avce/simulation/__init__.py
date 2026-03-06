"""Simulation: 3-DOF / 6-DOF 동역학, 폐루프 스텝, run_simulation
6-DOF 확장: DynamicsParams6DOF, step_6dof_rk4/method, OceanEnvironment, BuoyancyModel
"""
from .dynamics import (
    DynamicsParams,
    IntegrationMethod,
    step_3dof,
    step_3dof_rk4,
    step_3dof_method,
    tau_from_heading_and_surge,
)
from .loop import simulation_loop_step, run_simulation, SimulationStepResult
# ── 6-DOF 확장 ────────────────────────────────────────────────────────────────
from .dynamics6dof import (
    DynamicsParams6DOF,
    step_6dof,
    step_6dof_rk4,
    step_6dof_method,
    tau6_from_setpoints,
)
from .environment import (
    BuoyancyParams,
    BuoyancyModel,
    OceanEnvironment,
)

__all__ = [
    # ── 3-DOF dynamics ───────────────────────────────────────────────────────
    "DynamicsParams",
    "IntegrationMethod",
    "step_3dof",
    "step_3dof_rk4",
    "step_3dof_method",
    "tau_from_heading_and_surge",
    # ── loop ─────────────────────────────────────────────────────────────────
    "simulation_loop_step",
    "run_simulation",
    "SimulationStepResult",
    # ── 6-DOF dynamics ───────────────────────────────────────────────────────
    "DynamicsParams6DOF",
    "step_6dof",
    "step_6dof_rk4",
    "step_6dof_method",
    "tau6_from_setpoints",
    # ── environment ──────────────────────────────────────────────────────────
    "BuoyancyParams",
    "BuoyancyModel",
    "OceanEnvironment",
]
