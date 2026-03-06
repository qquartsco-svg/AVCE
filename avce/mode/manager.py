"""
모드 전환 관리자 (AVCE §MODE.MGR).

ModeManager : 운항 모드 전환 요청 → 안전 검사 → 전환 실행.
              전환 불가 조건 감지 → 자동 EMERGENCY_SURFACE 발동.

안전 규칙
─────────
1. 깊이 초과 → EMERGENCY_SURFACE
2. 피치 초과 → EMERGENCY_SURFACE
3. 비허용 전환 요청 → 거부 (현재 모드 유지)
4. SCRAM 상태에서 잠수 금지
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional
from ..core.mode import (
    VesselMode, ModeConfig, ModeProfile,
    default_mode_config, can_transition,
)
from ..core.state6dof import VehicleState

@dataclass(frozen=True)
class TransitionResult:
    """모드 전환 결과 (불변)."""
    success:      bool
    prev_mode:    VesselMode
    new_mode:     VesselMode
    reason:       str

class ModeManager:
    """운항 모드 전환 관리자.

    Parameters
    ----------
    initial_mode : VesselMode
        초기 운항 모드.
    profile : ModeProfile
        전환 임계값 설정.
    mode_configs : dict
        모드별 ModeConfig 재정의 (None이면 기본값 사용).
    """
    def __init__(
        self,
        initial_mode: VesselMode = VesselMode.SURFACE,
        profile: ModeProfile = ModeProfile(),
        mode_configs: Optional[dict] = None,
    ):
        self._mode = initial_mode
        self._profile = profile
        self._configs: dict = {
            m: default_mode_config(m) for m in VesselMode
        }
        if mode_configs:
            self._configs.update(mode_configs)
        self._transition_log: list = []

    @property
    def mode(self) -> VesselMode:
        return self._mode

    @property
    def config(self) -> ModeConfig:
        return self._configs[self._mode]

    def request_transition(
        self,
        target: VesselMode,
        state: VehicleState,
    ) -> TransitionResult:
        """모드 전환 요청. 안전 검사 후 전환 또는 거부."""
        prev = self._mode
        # 비상 부상은 항상 허용
        if target is VesselMode.EMERGENCY_SURFACE:
            return self._do_transition(target, "비상 부상 요청")
        # 허용 전환 검사
        if not can_transition(prev, target):
            return TransitionResult(False, prev, prev,
                f"{prev.value} → {target.value} 전환 불허 (전환 그래프 위반)")
        # 상태 안전 검사
        err = self._safety_check(target, state)
        if err:
            return TransitionResult(False, prev, prev, err)
        return self._do_transition(target, f"정상 전환 요청: {prev.value} → {target.value}")

    def auto_safety_check(self, state: VehicleState) -> Optional[TransitionResult]:
        """매 스텝 자동 안전 점검. 위험 감지 시 EMERGENCY_SURFACE 발동."""
        pr = self._profile
        cfg = self.config
        # 최대 깊이 초과
        if state.z_m > pr.emergency_depth_m:
            return self._do_transition(VesselMode.EMERGENCY_SURFACE,
                f"최대 깊이 초과: z={state.z_m:.1f}m > {pr.emergency_depth_m}m")
        # 최대 깊이 (모드별) 초과
        if cfg.max_depth_m > 0 and state.z_m > cfg.max_depth_m * 1.1:
            return self._do_transition(VesselMode.EMERGENCY_SURFACE,
                f"모드 최대 깊이 초과: z={state.z_m:.1f}m")
        # 피치 초과
        if abs(state.theta_rad) > pr.pitch_limit_emerg_rad:
            return self._do_transition(VesselMode.EMERGENCY_SURFACE,
                f"피치 초과: θ={math.degrees(state.theta_rad):.1f}°")
        return None

    def _do_transition(self, target: VesselMode, reason: str) -> TransitionResult:
        prev = self._mode
        self._mode = target
        result = TransitionResult(True, prev, target, reason)
        self._transition_log.append(result)
        return result

    def _safety_check(self, target: VesselMode, state: VehicleState) -> Optional[str]:
        """전환 전 안전 검사. 문제 있으면 오류 메시지 반환, 없으면 None."""
        pr = self._profile
        # 잠수 시작: 최소 속도 확인
        if target in (VesselMode.TRANSIT_DIVE, VesselMode.DIVE):
            if state.U_horizontal_ms < pr.dive_speed_ms:
                return f"잠수 최소 속도 미달: {state.U_horizontal_ms:.2f} < {pr.dive_speed_ms} m/s"
        # 부상: 깊이 확인
        if target is VesselMode.TRANSIT_SURFACE:
            if state.z_m <= pr.surface_z_thresh_m:
                return f"이미 수면 근처 (z={state.z_m:.2f}m)"
        return None

    @property
    def transition_log(self) -> list:
        return list(self._transition_log)
