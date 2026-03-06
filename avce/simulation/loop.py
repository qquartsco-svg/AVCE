"""
폐루프 시뮬레이션: 추정 → 제어 → 동역학 스텝 (EQUATIONS §8).

변경 이력 (v2)
──────────────
- IntegrationMethod 선택 지원 (기본 RK4).
  step_3dof_method() 사용 → Euler / RK4 선택 가능.
- WaypointSequencer 통합.
  sequencer.update(x) → 현재 waypoint 자동 전환 + U_ref_scale 적용.
- KalmanEstimator 연동.
  est.set_control(τ_u, τ_v, τ_r) → 다음 predict 스텝용 τ 저장.
  (set_control 미구현 estimator는 hasattr 검사로 안전하게 스킵.)
- run_simulation: estimator, integration_method, sequencer 파라미터 추가.

설계 원칙
──────────
- 하드코딩 없음: IntegrationMethod 기본값은 RK4 (명시적 Enum).
- SimulationStepResult.seq_result: Optional — 시퀀서 없이 사용해도 안전.
- VesselController.waypoint 직접 갱신 → 시퀀서와 계약 없이 연동.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional

from ..core.estimator import IdentityEstimator, StateEstimator
from ..core.ramming import RammingDisturbance
from ..core.sequencer import SequencerResult, WaypointSequencer
from ..core.state import IceCell, Obstacle, VesselState, Waypoint
from ..controller import VesselController
from .dynamics import (
    DynamicsParams,
    IntegrationMethod,
    step_3dof_method,
    tau_from_heading_and_surge,
)


# ── 스텝 결과 ─────────────────────────────────────────────────────────────────

@dataclass
class SimulationStepResult:
    """한 시뮬 스텝 결과."""
    state:          VesselState
    psi_ref_rad:    float
    U_ref_ms:       float          # 시퀀서 U_ref_scale 적용 후 값
    tau_u:          float
    tau_v:          float
    tau_r:          float
    tau_ram_r:      float          # 쇄빙 축 외란 (보고용)
    seq_result:     Optional[SequencerResult] = None   # 시퀀서 없으면 None


# ── 단일 스텝 ─────────────────────────────────────────────────────────────────

def simulation_loop_step(
    state:            VesselState,
    controller:       VesselController,
    dt_s:             float,
    t_s:              float,
    dynamics_params:  DynamicsParams,
    estimator:        Optional[StateEstimator]     = None,
    ramming:          Optional[RammingDisturbance] = None,
    sequencer:        Optional[WaypointSequencer]  = None,
    integration_method: IntegrationMethod          = IntegrationMethod.RK4,
    k_psi: float = 1.0e5,
    k_u:   float = 5.0e4,
) -> SimulationStepResult:
    """한 스텝: state → estimate → (sequencer) → controller → τ → dynamics → state_new.

    Parameters
    ----------
    state           : 현재 선박 상태.
    controller      : VesselController 인스턴스 (waypoint 직접 갱신).
    dt_s            : 타임 스텝 [s].
    t_s             : 현재 시각 [s] (Cerebellum / 외란 계산용).
    dynamics_params : 동역학 파라미터 (frozen).
    estimator       : 상태 추정기. None → IdentityEstimator.
    ramming         : 쇄빙 외란 생성기. None → 외란 없음.
    sequencer       : WaypointSequencer. None → controller.waypoint 고정.
    integration_method : 적분 방식 (기본 RK4).
    k_psi, k_u      : 선수각·surge 힘 변환 게인.

    Notes
    -----
    τ_ram은 yaw 축(N_r)에 가산됨.
    OrbitStabilizer 보정 (psi 축): τ_r에 가산됨.
    sequencer 사용 시: controller.waypoint를 매 스텝 seq_result.waypoint로 갱신.
    KalmanEstimator 사용 시: set_control()로 τ를 다음 스텝 predict에 전달.
    """
    # 1. 상태 추정 ─────────────────────────────────────────────────────────────
    est = estimator if estimator is not None else IdentityEstimator()
    x   = est.estimate(state)

    # 2. 시퀀서 갱신 ───────────────────────────────────────────────────────────
    seq_result: Optional[SequencerResult] = None
    if sequencer is not None and not sequencer.is_complete:
        seq_result = sequencer.update(x)
        controller.waypoint = seq_result.waypoint  # waypoint 자동 전환

    # 3. 쇄빙 외란 ─────────────────────────────────────────────────────────────
    tau_ram_r = ramming.tau_ram(t_s) if ramming is not None else 0.0

    # 4. 제어기 스텝 ───────────────────────────────────────────────────────────
    ctrl_result = controller.step(x, t_s=t_s, tau_ram_nm=tau_ram_r)
    psi_ref     = ctrl_result.psi_ref_rad
    U_ref_ms    = ctrl_result.U_ref_ms

    # 5. 시퀀서 속도 스케일 적용 ───────────────────────────────────────────────
    if seq_result is not None:
        U_ref_ms *= seq_result.U_ref_scale

    # 6. τ 변환 (방향 오차 + surge 오차 → 힘) ────────────────────────────────
    tau_u, tau_v, tau_r = tau_from_heading_and_surge(
        psi_ref, x.psi_rad, U_ref_ms, x.u_ms, x.v_ms, k_psi, k_u,
    )

    # 7. OrbitStabilizer psi 축 보정 → τ_r 가산 ──────────────────────────────
    if ctrl_result.corrections and "psi" in ctrl_result.corrections:
        tau_r += ctrl_result.corrections["psi"].correction_force

    # 8. Kalman Estimator 제어 입력 등록 (다음 predict 사용) ──────────────────
    if hasattr(est, "set_control"):
        est.set_control(tau_u, tau_v, tau_r)

    # 9. 동역학 적분 ───────────────────────────────────────────────────────────
    state_new = step_3dof_method(
        state, tau_u, tau_v, tau_r, dt_s,
        dynamics_params,
        method=integration_method,
        tau_env_r=tau_ram_r,
    )

    return SimulationStepResult(
        state=state_new,
        psi_ref_rad=psi_ref,
        U_ref_ms=U_ref_ms,
        tau_u=tau_u,
        tau_v=tau_v,
        tau_r=tau_r,
        tau_ram_r=tau_ram_r,
        seq_result=seq_result,
    )


# ── 전체 루프 ─────────────────────────────────────────────────────────────────

def run_simulation(
    initial_state:    VesselState,
    controller:       VesselController,
    T_s:              float,
    dt_s:             float,
    dynamics_params:  Optional[DynamicsParams]     = None,
    estimator:        Optional[StateEstimator]     = None,
    ramming:          Optional[RammingDisturbance] = None,
    sequencer:        Optional[WaypointSequencer]  = None,
    integration_method: IntegrationMethod          = IntegrationMethod.RK4,
    callback: Optional[Callable[[float, SimulationStepResult], None]] = None,
) -> List[SimulationStepResult]:
    """t=0 ~ T_s까지 폐루프 시뮬레이션.

    Parameters
    ----------
    initial_state     : 초기 선박 상태.
    controller        : VesselController.
    T_s               : 총 시뮬레이션 시간 [s].
    dt_s              : 타임 스텝 [s].
    dynamics_params   : None → DynamicsParams() 기본값 사용.
    estimator         : 상태 추정기. KalmanEstimator 권장.
    ramming           : 쇄빙 외란. None → 없음.
    sequencer         : WaypointSequencer. None → 단일 waypoint 고정.
    integration_method: 적분 방식. 기본 RK4.
    callback          : (t, step_result) 콜백 (로그·시각화용).

    Returns
    -------
    results : List[SimulationStepResult] — 각 스텝 결과 목록.

    Notes
    -----
    sequencer 완료(is_complete) 후에도 시뮬레이션은 T_s까지 계속 실행됨.
    (마지막 waypoint 정박 상태 유지. controller.waypoint 변경 없음.)
    """
    params  = dynamics_params if dynamics_params is not None else DynamicsParams()
    results: List[SimulationStepResult] = []
    state   = initial_state
    t       = 0.0

    while t < T_s:
        step_result = simulation_loop_step(
            state, controller, dt_s, t, params,
            estimator=estimator,
            ramming=ramming,
            sequencer=sequencer,
            integration_method=integration_method,
        )
        results.append(step_result)
        if callback is not None:
            callback(t, step_result)
        state = step_result.state
        t    += dt_s

    return results
