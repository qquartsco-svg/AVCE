"""
통합 스텝: 경로 제어 + (선택) OrbitStabilizer 보정.
EQUATIONS §8 제어 합성.
"""
from __future__ import annotations
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

from .core.state import VesselState, Waypoint, Obstacle, IceCell
from .core.constants import RHO_0_DEFAULT, CONTEXT_OPEN_WATER, PREDICTION_HORIZON_MS
from .control.path_controller import path_controller, PathOutput
from .control.cerebellum_profile import CerebellumProfile
from .integration.orbit_stabilizer_adapter import OrbitStabilizerAdapter, AxisCorrection
from .memory.well_memory import WellMemory


@dataclass
class VesselStepResult:
    """한 제어 스텝 출력"""
    psi_ref_rad: float
    U_ref_ms: float
    F_pf_mag: float
    corrections: Dict[str, AxisCorrection]  # axis_id -> 보정
    path_output: PathOutput
    tau_ram_nm: float = 0.0  # 쇄빙 램 외란 (보고용, 제어기는 τ_env로 받음)


def _noop_wells(_xi: float, _eta: float) -> float:
    return 0.0


class VesselController:
    """
    확장 가능 자율선박 제어기.
    - Potential Field → ψ_ref, U_ref
    - (선택) OrbitStabilizer 다축 보정
    - (선택) WellMemory U_mem
    - (선택) 빙 격자 U_ice
    """
    def __init__(
        self,
        waypoint: Waypoint,
        obstacles: List[Obstacle],
        *,
        memory: Optional[WellMemory] = None,
        context: str = CONTEXT_OPEN_WATER,
        ice_cells: Optional[List[IceCell]] = None,
        orbit_stabilizer_axes: Optional[List[str]] = None,
        cerebellum_profile: Optional[CerebellumProfile] = None,
        profile_id: Optional[str] = None,
        k_goal: float = 0.5,
        k_ice: float = 1.0,
        rho_0: float = RHO_0_DEFAULT,
        U_default_ms: float = 5.0,
        prediction_horizon_ms: float = PREDICTION_HORIZON_MS,
        profile_blend: float = 0.0,
    ):
        self.waypoint = waypoint
        self.obstacles = obstacles
        self.memory = memory
        self.context = context
        self.ice_cells = ice_cells or []
        self.cerebellum_profile = cerebellum_profile
        self.profile_id = profile_id
        self.profile_blend = max(0.0, min(1.0, profile_blend))  # 0=path만, 1=프로파일만
        self.k_goal = k_goal
        self.k_ice = k_ice
        self.rho_0 = rho_0
        self.U_default_ms = U_default_ms
        self.prediction_horizon_ms = prediction_horizon_ms
        self._path_kw: Dict[str, Any] = {
            "k_goal": k_goal,
            "k_ice": k_ice,
            "rho_0": rho_0,
            "U_default_ms": U_default_ms,
        }
        self._os_adapter: Optional[OrbitStabilizerAdapter] = None
        if orbit_stabilizer_axes:
            self._os_adapter = OrbitStabilizerAdapter(
                axis_ids=orbit_stabilizer_axes,
                prediction_horizon_ms=prediction_horizon_ms,
            )

    def step(
        self,
        state: VesselState,
        phi_rad: Optional[float] = None,
        phi_ref_rad: Optional[float] = None,
        t_s: float = 0.0,
        tau_ram_nm: float = 0.0,
    ) -> VesselStepResult:
        """
        한 제어 스텝.
        phi_rad / phi_ref_rad: 추진축 위상 (OrbitStabilizer 'phi' 사용 시).
        t_s: Cerebellum 프로파일용 시각 [s]. tau_ram_nm: 램 외란(보고용).
        """
        memory_cb = None
        if self.memory is not None:
            memory_cb = self.memory.make_callback(self.context)

        path_out = path_controller(
            state,
            self.waypoint,
            self.obstacles,
            memory_wells=memory_cb,
            ice_cells=self.ice_cells if self.ice_cells else None,
            **self._path_kw,
        )
        psi_ref = path_out.psi_ref_rad
        U_ref = path_out.U_ref_ms

        if self.cerebellum_profile is not None and self.profile_blend > 0:
            psi_goal = path_out.psi_ref_rad
            U_goal = path_out.U_ref_ms
            psi_p, U_p = self.cerebellum_profile.get_at_t(
                t_s, state.psi_rad, psi_goal, state.U_ms, U_goal,
                self.context, self.profile_id,
            )
            a = self.profile_blend
            psi_ref = (1 - a) * psi_ref + a * psi_p
            U_ref = (1 - a) * U_ref + a * U_p

        corrections: Dict[str, AxisCorrection] = {}
        if self._os_adapter is not None and self._os_adapter.available:
            sensor_targets: Dict[str, tuple[float, float]] = {
                "psi": (state.psi_rad, psi_ref),
            }
            if phi_rad is not None and phi_ref_rad is not None:
                sensor_targets["phi"] = (phi_rad, phi_ref_rad)
            corrections = self._os_adapter.update_multi(
                sensor_targets,
                prediction_horizon_ms=self.prediction_horizon_ms,
            )

        return VesselStepResult(
            psi_ref_rad=psi_ref,
            U_ref_ms=U_ref,
            F_pf_mag=path_out.F_pf_mag,
            corrections=corrections,
            path_output=path_out,
            tau_ram_nm=tau_ram_nm,
        )
