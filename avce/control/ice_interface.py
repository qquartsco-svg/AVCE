"""
3D 빙 인터페이스 제어 (AVCE §CTRL.ICE).

IceInterfaceConfig : 빙 감지·파쇄 파라미터 (frozen).
IceInterface       : 선체 위치 + IceLayer 목록 → 빙 외란 / 파쇄력 계산.

지원 시나리오
─────────────
1. 수상 쇄빙 (ICEBREAKER)    : 수면 빙 → 전방 램 충격
2. 빙 천장 회피 (UNDER_ICE)  : 빙 천장 감지 → 하방 이격 유지
3. 잠수 쇄빙 (SUBMERGED_ICE) : 빙 천장 → 상향 파쇄력

파쇄력 모델
───────────
F_ram = min(A_ram · C_ice · hardness · (v_approach)², F_max)
τ_ram → surge (표면) 또는 heave (잠수) 방향으로 적용.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from ..core.state6dof import VehicleState, IceLayer
from ..core.mode import VesselMode

_Tau6 = Tuple[float, float, float, float, float, float]

@dataclass(frozen=True)
class IceInterfaceConfig:
    """빙 인터페이스 파라미터 (불변)."""
    detect_radius_m:    float = 300.0  # 빙 감지 반경 [m]
    safety_clearance_m: float = 5.0   # 빙 천장 최소 이격 [m]
    ram_area_m2:        float = 50.0  # 램 충격 단면적 [m²]
    max_ram_force_n:    float = 5.0e6 # 최대 파쇄력 [N]
    ice_resist_coeff:   float = 2.0e4 # 빙 저항 계수 [N·s²/m²]
    ceiling_warn_m:     float = 20.0  # 빙 천장 경고 이격 [m]

@dataclass(frozen=True)
class IceContactResult:
    """빙 접촉 분석 결과 (불변)."""
    in_contact:         bool              # 접촉 여부
    nearest_layer:      Optional[IceLayer]  # 가장 가까운 빙층
    clearance_m:        float             # 빙까지 이격 거리 [m]
    tau_ram:            _Tau6             # 파쇄 외란 벡터
    risk_level:         float             # 위험 수준 [0~1]
    surface_ice_count:  int = 0           # 감지된 수면 빙층 수 (ICEBREAKER 모드)
    ceiling_ice_count:  int = 0           # 감지된 빙 천장 수 (UNDER_ICE 모드)

class IceInterface:
    """3D 빙 인터페이스 분석·파쇄력 계산.

    Parameters
    ----------
    config : IceInterfaceConfig
    layers : IceLayer 목록 (동적 추가 가능)
    """
    def __init__(
        self,
        config: IceInterfaceConfig = IceInterfaceConfig(),
        layers: Optional[List[IceLayer]] = None,
    ):
        self._cfg = config
        self._layers: List[IceLayer] = list(layers or [])

    def add_layer(self, layer: IceLayer) -> None:
        self._layers.append(layer)

    def analyze(self, state: VehicleState, mode: VesselMode) -> IceContactResult:
        """선체 위치 + 모드 → 빙 접촉 분석."""
        cfg = self._cfg
        relevant = self._relevant_layers(state, mode)

        if not relevant:
            return IceContactResult(False, None, 9999.0, (0,0,0,0,0,0), 0.0, 0, 0)

        # 수면/천장 빙층 카운트 (반경 내 전체, 모드 불문)
        in_radius = [
            layer for layer in self._layers
            if math.sqrt((state.xi_m - layer.xi_m)**2 + (state.eta_m - layer.eta_m)**2)
               <= cfg.detect_radius_m
        ]
        surf_count    = sum(1 for l in in_radius if l.is_surface_ice)
        ceiling_count = sum(1 for l in in_radius if l.is_ceiling_ice)

        # 가장 가까운 빙층 + 이격 계산
        nearest, clearance = self._nearest_clearance(state, relevant, mode)

        # 위험 수준
        risk = max(0.0, 1.0 - clearance / max(cfg.ceiling_warn_m, 1.0))
        risk = min(1.0, risk)

        in_contact = clearance < 0.1

        # 파쇄 외란 계산
        tau_ram = self._ram_force(state, nearest, mode, in_contact)

        return IceContactResult(
            in_contact=in_contact,
            nearest_layer=nearest,
            clearance_m=clearance,
            tau_ram=tau_ram,
            risk_level=risk,
            surface_ice_count=surf_count,
            ceiling_ice_count=ceiling_count,
        )

    def ceiling_avoidance_tau(
        self,
        state: VehicleState,
        contact: IceContactResult,
    ) -> _Tau6:
        """빙 천장 이격 유지 → 하방 τ_w 보정."""
        if contact.nearest_layer is None:
            return (0.0,) * 6
        cfg = self._cfg
        deficit = cfg.safety_clearance_m - contact.clearance_m
        if deficit <= 0:
            return (0.0,) * 6
        # 이격 부족 → 아래로 이동 (z 증가)
        k = cfg.ice_resist_coeff
        tau_w = k * deficit   # 양수 = NED 아래
        tau_w = min(tau_w, cfg.max_ram_force_n)
        return (0.0, 0.0, tau_w, 0.0, 0.0, 0.0)

    # ── 내부 ──────────────────────────────────────────────────────────────────

    def _relevant_layers(
        self,
        state: VehicleState,
        mode: VesselMode,
    ) -> List[IceLayer]:
        """모드에 따라 관련 빙층 필터링."""
        out = []
        for layer in self._layers:
            dx = state.xi_m - layer.xi_m
            dy = state.eta_m - layer.eta_m
            if math.sqrt(dx**2 + dy**2) > self._cfg.detect_radius_m:
                continue
            if mode.is_surface and layer.is_surface_ice:
                out.append(layer)
            elif mode.is_submerged and layer.is_ceiling_ice:
                out.append(layer)
        return out

    def _nearest_clearance(
        self,
        state: VehicleState,
        layers: List[IceLayer],
        mode: VesselMode,
    ) -> Tuple[IceLayer, float]:
        nearest = layers[0]
        min_clear = 9999.0
        for layer in layers:
            if mode.is_surface:
                # 수상: 수면 빙까지 수평 거리
                dx = state.xi_m - layer.xi_m
                dy = state.eta_m - layer.eta_m
                clear = math.sqrt(dx**2 + dy**2) - layer.sigma_m
            else:
                # 잠수: 빙 하면까지 수직 이격
                clear = state.z_m - layer.z_bottom_m
            if clear < min_clear:
                min_clear = clear
                nearest = layer
        return nearest, max(0.0, min_clear)

    def _ram_force(
        self,
        state: VehicleState,
        layer: IceLayer,
        mode: VesselMode,
        in_contact: bool,
    ) -> _Tau6:
        """접촉 시 파쇄 외란 계산."""
        if not in_contact:
            return (0.0,) * 6
        cfg = self._cfg
        # 접근 속도 (양수)
        if mode.is_surface:
            v_approach = max(0.0, state.u_ms)   # surge
            F = cfg.ram_area_m2 * layer.C_ice * layer.hardness * v_approach**2
            F = min(F, cfg.max_ram_force_n)
            return (-F, 0.0, 0.0, 0.0, 0.0, 0.0)   # surge 역방향 저항
        else:
            v_approach = max(0.0, -state.w_ms)  # heave 상향 속도
            F = cfg.ram_area_m2 * layer.C_ice * layer.hardness * v_approach**2
            F = min(F, cfg.max_ram_force_n)
            return (0.0, 0.0, F, 0.0, 0.0, 0.0)    # heave 하방 반력
