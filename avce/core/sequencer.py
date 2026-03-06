"""
다중 Waypoint 시퀀서 (EQUATIONS §3.5 확장).

WaypointSequencer : Waypoint 목록을 순서대로 추적.
  - 도달 판정 (arrival_radius) → 다음 waypoint 자동 전환.
  - 감속 프로파일 (decel_radius, min_speed_scale) → 근접 시 U_ref 자동 스케일.
  - 전체 완료 감지 (is_complete).

설계 원칙
──────────
- ArrivalConfig / SpeedConfig : frozen=True — 파라미터 불변.
- SequencerResult  : frozen=True — 출력 불변 (읽기 전용 결과).
- WaypointSequencer : 내부 상태(인덱스) 만 변경. 경로 목록은 생성 시 고정.
- 하드코딩 없음: 모든 임계값은 ArrivalConfig/SpeedConfig 주입.
- state.py(Waypoint, VesselState)만 import — 레이어 독립.

감속 수식 (decel zone)
───────────────────────
d = dist(vessel, wp_current)

U_scale(d) = {  1.0                                  if d >= decel_radius
             {  max(min_speed_scale, d/decel_radius)  if d < decel_radius

→ VesselController / path_controller의 U_ref_ms 에 곱해서 사용.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

from .state import VesselState, Waypoint


# ── 설정 (불변) ───────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class ArrivalConfig:
    """Waypoint 도달 판정 파라미터 (불변).

    Attributes
    ----------
    radius_m : float
        이 거리 이하이면 도달로 판정 [m].
    """
    radius_m: float = 50.0


@dataclass(frozen=True)
class SpeedConfig:
    """근접 감속 파라미터 (불변).

    Attributes
    ----------
    decel_radius_m : float
        감속 시작 거리 [m]. 이 거리부터 선형 감속.
    min_speed_scale : float
        최소 속도 배율 (0 < min ≤ 1). 극단적 저속 방지.
    final_speed_scale : float
        마지막 waypoint 도달 후 속도 배율 (보통 0 → 정지).
    """
    decel_radius_m:  float = 200.0
    min_speed_scale: float = 0.3
    final_speed_scale: float = 0.0


# ── 시퀀서 상태 Enum ──────────────────────────────────────────────────────────

class SequencerStatus(str, Enum):
    """시퀀서 현재 상태."""
    ACTIVE   = "active"    # 항행 중
    ARRIVING = "arriving"  # 현재 wp 근접 (decel_radius 이내)
    ARRIVED  = "arrived"   # 이번 스텝에 도달 → 다음 wp로 전환됨
    COMPLETE = "complete"  # 모든 wp 완료


# ── 결과 (불변) ───────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SequencerResult:
    """update() 반환값 (불변).

    Attributes
    ----------
    waypoint : Waypoint
        현재(또는 방금 전환된) 목표 waypoint.
    index : int
        현재 wp 인덱스 (0-based). complete 시 마지막 인덱스.
    distance_m : float
        현재 선박 위치 → 현재 wp 거리 [m].
    status : SequencerStatus
        현재 시퀀서 상태.
    U_ref_scale : float
        속도 배율 [0, 1]. path_controller U_ref_ms 에 곱해서 사용.
    remaining_count : int
        남은 waypoint 수 (현재 포함).
    """
    waypoint:        Waypoint
    index:           int
    distance_m:      float
    status:          SequencerStatus
    U_ref_scale:     float
    remaining_count: int

    @property
    def arrived(self) -> bool:
        return self.status is SequencerStatus.ARRIVED

    @property
    def is_complete(self) -> bool:
        return self.status is SequencerStatus.COMPLETE


# ── WaypointSequencer ─────────────────────────────────────────────────────────

class WaypointSequencer:
    """다중 Waypoint 순서 추적기.

    Parameters
    ----------
    waypoints : List[Waypoint]
        항로 순서대로 정렬된 waypoint 목록. 최소 1개.
    arrival : ArrivalConfig
        도달 판정 파라미터.
    speed : SpeedConfig
        감속 파라미터.

    Example
    -------
    >>> wps = [Waypoint(500, 0, U_d_ms=5.0), Waypoint(1000, 300, U_d_ms=3.0)]
    >>> seq = WaypointSequencer(wps)
    >>> result = seq.update(vessel_state)
    >>> U_ref = path_output.U_ref_ms * result.U_ref_scale
    """

    def __init__(
        self,
        waypoints: List[Waypoint],
        arrival: ArrivalConfig = ArrivalConfig(),
        speed:   SpeedConfig   = SpeedConfig(),
    ) -> None:
        if not waypoints:
            raise ValueError("WaypointSequencer: waypoints 목록이 비어있습니다.")
        self._wps:     List[Waypoint] = list(waypoints)  # 불변 복사
        self._arrival: ArrivalConfig  = arrival
        self._speed:   SpeedConfig    = speed
        self._idx:     int            = 0
        self._complete: bool          = False

    # ── 주요 인터페이스 ───────────────────────────────────────────────────────

    def update(self, state: VesselState) -> SequencerResult:
        """현재 선박 위치 기준 시퀀서 갱신.

        도달 판정 → 인덱스 전환 → U_ref_scale 계산 → SequencerResult 반환.
        complete 상태에서 계속 호출해도 안전 (마지막 wp + COMPLETE 반환).
        """
        if self._complete:
            wp = self._wps[-1]
            d  = _dist(state, wp)
            return SequencerResult(
                waypoint=wp,
                index=len(self._wps) - 1,
                distance_m=d,
                status=SequencerStatus.COMPLETE,
                U_ref_scale=self._speed.final_speed_scale,
                remaining_count=0,
            )

        wp = self._wps[self._idx]
        d  = _dist(state, wp)

        # 도달 판정
        arrived = d < self._arrival.radius_m
        if arrived:
            if self._idx < len(self._wps) - 1:
                self._idx += 1
                wp = self._wps[self._idx]
                d  = _dist(state, wp)
                status = SequencerStatus.ARRIVED
            else:
                # 마지막 wp 도달 → 완료
                self._complete = True
                return SequencerResult(
                    waypoint=wp,
                    index=self._idx,
                    distance_m=d,
                    status=SequencerStatus.COMPLETE,
                    U_ref_scale=self._speed.final_speed_scale,
                    remaining_count=0,
                )
        else:
            status = (SequencerStatus.ARRIVING
                      if d < self._speed.decel_radius_m
                      else SequencerStatus.ACTIVE)

        u_scale = self._speed_scale(d)
        remaining = len(self._wps) - self._idx

        return SequencerResult(
            waypoint=wp,
            index=self._idx,
            distance_m=d,
            status=status,
            U_ref_scale=u_scale,
            remaining_count=remaining,
        )

    # ── 조회 ──────────────────────────────────────────────────────────────────

    @property
    def current_waypoint(self) -> Waypoint:
        """현재 목표 waypoint."""
        return self._wps[min(self._idx, len(self._wps) - 1)]

    @property
    def current_index(self) -> int:
        return self._idx

    @property
    def is_complete(self) -> bool:
        return self._complete

    @property
    def total(self) -> int:
        return len(self._wps)

    def remaining_waypoints(self) -> List[Waypoint]:
        """현재 포함 남은 waypoint 목록 (복사본)."""
        return list(self._wps[self._idx:])

    def reset(self) -> None:
        """처음 waypoint로 초기화 (재항로 계획 시)."""
        self._idx      = 0
        self._complete = False

    def insert_waypoint(self, wp: Waypoint, after_index: int) -> None:
        """항로 중 긴급 waypoint 삽입 (after_index 다음에 삽입).

        완료 상태가 아닌 경우에만 동작.
        """
        if self._complete:
            return
        insert_at = min(after_index + 1, len(self._wps))
        self._wps.insert(insert_at, wp)
        # 삽입 위치가 현재 인덱스 이하이면 인덱스 보정
        if insert_at <= self._idx:
            self._idx += 1

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _speed_scale(self, d: float) -> float:
        """거리 d에 따른 속도 배율 계산.

        d >= decel_radius → 1.0
        d <  decel_radius → max(min_scale, d / decel_radius)  [선형 감속]
        """
        s = self._speed
        if d >= s.decel_radius_m:
            return 1.0
        if s.decel_radius_m <= 0.0:
            return s.min_speed_scale
        raw = d / s.decel_radius_m
        return max(s.min_speed_scale, min(1.0, raw))


# ── 순수 헬퍼 ────────────────────────────────────────────────────────────────

def _dist(state: VesselState, wp: Waypoint) -> float:
    """선박 위치 → waypoint 유클리드 거리 [m]."""
    return math.sqrt((state.xi_m - wp.xi_m) ** 2 + (state.eta_m - wp.eta_m) ** 2)
