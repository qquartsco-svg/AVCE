# AVCE — Autonomous Vessel Control Engine

**자율선박 하이브리드 제어 AI** — 인지과학 기반 다중모드 자율선박·잠수함·쇄빙선 통합 제어 엔진

[![Layer](https://img.shields.io/badge/layer-60_APPLIED-blue.svg)](.)
[![Status](https://img.shields.io/badge/status-implemented-green.svg)](.)
[![Version](https://img.shields.io/badge/version-0.4-orange.svg)](.)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](.)

> **전 지구 해양 어디서든 — 수면·수중·빙하 하부 — 뚫고 지나가는 하이브리드 자율 제어 시스템.**

---

## 목차

1. [철학과 목적](#1-철학과-목적)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [운항 모드 — 8종 하이브리드](#3-운항-모드--8종-하이브리드)
4. [인지과학 매핑](#4-인지과학-매핑)
5. [핵심 수식 — 제어 이론](#5-핵심-수식--제어-이론)
6. [모듈 구조](#6-모듈-구조)
7. [구현된 기능 전체](#7-구현된-기능-전체)
8. [공개 API — 92 exports](#8-공개-api--92-exports)
9. [사용 예시](#9-사용-예시)
10. [의존성 · 설계 원칙](#10-의존성--설계-원칙)
11. [블록체인 서명](#11-블록체인-서명)
12. [버전 이력](#12-버전-이력)

---

## 1. 철학과 목적

### 왜 이 엔진인가

AVCE는 **"전체 자율선박 시스템"이 아니라** 제어 계층의 **개념·수식·계약**을 제공하는 독립 모듈이다.

- CookiieBrain(우물 기억, Cerebellum, Potential Field, OrbitStabilizer)을 **자율선박 제어**로 확장할 때의 공통 스펙
- 실제 제어 루프·센서·시뮬레이터는 외부에서 붙인다 — **인터페이스만** 이 엔진에 정의
- **v0.4**부터: 3-DOF 수상뿐 아니라 **6-DOF 잠수함·쇄빙·핵추진**까지 단일 패키지로 커버

### 설계 원칙

| 원칙 | 내용 |
|------|------|
| **frozen dataclass** | 모든 파라미터 클래스는 `frozen=True` — 불변 보장, 실수 방지 |
| **순수 함수** | `_ode_rhs`, `gradient_U` 등 핵심 연산은 부작용 없는 순수 함수 |
| **Enum 선택** | 모든 선택지(모드·적분법·추진·기울기)는 `str Enum` — 하드코딩 금지 |
| **레이어 격리** | `core` → `control` → `simulation` 단방향. 역방향 임포트 없음 |
| **계약 우선** | 구현보다 `CONCEPTS.md`, `EQUATIONS.md` 수식 고정 후 코드 작성 |

---

## 2. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                        AVCE — 제어 파이프라인                         │
│                                                                     │
│  Sensor/SLAM → Estimator → Path Controller → Vessel Controller      │
│                   │              │                  │               │
│              KalmanEstimator  Potential        VesselController     │
│              (EKF 6-state)    Field + Hippo   + OrbitStabilizer     │
│                                 + Cerebellum  + Cerebellum          │
│                                                                     │
│  ─────────────────────── Mode Manager ──────────────────────────── │
│    SURFACE ↔ ICEBREAKER                                             │
│    SURFACE → TRANSIT_DIVE → DIVE ↔ UNDER_ICE ↔ SUBMERGED_ICE      │
│    * → EMERGENCY_SURFACE  (항상 허용)                               │
│                                                                     │
│  ─────────── 동역학 레이어 ────────────────────────────────────── │
│    3-DOF (수상): M ν̇ + D ν = τ + τ_env        (Euler / RK4)       │
│    6-DOF (잠수): M ν̇ + D ν + g(η) = τ + τ_env  (Fossen / RK4)    │
│                                                                     │
│  ─────────── 추진 레이어 ──────────────────────────────────────── │
│    NuclearReactor (PWR) → PropulsionManager → τ_u [N]              │
│    dP/dt = (P_demand − P) / τ_reactor   (1차 지연)                  │
│    SCRAM 보호: P > P_rated × 1.15 → 즉각 정지                       │
└─────────────────────────────────────────────────────────────────────┘
```

**핵심 데이터 흐름:**

```
State(12D) → ModeManager → {3-DOF 또는 6-DOF 동역학 선택}
                ↓
      Potential Field −∇U → ψ_ref, U_ref
      DepthController → τ_w, τ_q, τ_p
      IceInterface → τ_ram (수면/천장 파쇄)
      NuclearReactor → τ_u (추진력)
                ↓
         step_6dof_rk4() → 다음 VehicleState(12D)
```

---

## 3. 운항 모드 — 8종 하이브리드

| 모드 | 값 | 동역학 | 설명 | 최대 수심 | 쇄빙 |
|------|----|--------|------|-----------|------|
| `SURFACE` | `"surface"` | 3-DOF | 수상 항행 — 일반 | 0 m | ✗ |
| `ICEBREAKER` | `"icebreaker"` | 3-DOF | 수상 쇄빙 — 표면 빙 파쇄 | 0 m | ✓ |
| `TRANSIT_DIVE` | `"transit_dive"` | 6-DOF | 잠수 전환 — 하강 과도 구간 | 600 m | ✗ |
| `DIVE` | `"dive"` | 6-DOF | 잠수 항행 — 개방 해역 | 600 m | ✗ |
| `UNDER_ICE` | `"under_ice"` | 6-DOF | 빙하 하부 항행 — 천장 회피 | 600 m | ✗ |
| `SUBMERGED_ICEBREAKER` | `"submerged_icebreaker"` | 6-DOF | 잠수 중 상향 파쇄 | 200 m | ✓ |
| `TRANSIT_SURFACE` | `"transit_surface"` | 6-DOF | 부상 전환 — 상승 과도 구간 | 600 m | ✗ |
| `EMERGENCY_SURFACE` | `"emergency_surface"` | 6-DOF | 비상 부상 — 최우선, 최대 부력 | 600 m | ✗ |

### 전환 그래프

```
SURFACE ←────────────→ ICEBREAKER
    │                        │
    └──→ TRANSIT_DIVE ←──────┘
              │
              ↓
            DIVE ←──────────→ UNDER_ICE
              │                   │
              │           SUBMERGED_ICEBREAKER
              │
         TRANSIT_SURFACE
              │
              ↓
           SURFACE / ICEBREAKER

   * 모든 모드 → EMERGENCY_SURFACE  (항상 허용, 비상)
```

**자동 비상 트리거:**
- 깊이 초과: `z > emergency_depth_m` → `EMERGENCY_SURFACE` 자동 전환
- 피치 초과: `|θ| > pitch_limit_emerg_rad` → `EMERGENCY_SURFACE` 자동 전환

---

## 4. 인지과학 매핑

AVCE는 인간 인지 구조를 자율선박 제어에 대응한다.

| 인지/엔진 레이어 | 제어 역할 | AVCE 구현 |
|------------------|-----------|-----------|
| **L0 Ring Attractor** | 회전 위상 φ의 국소 기억·안정 궤적 | OrbitStabilizer 래핑 |
| **L1 Potential Field** | 공간 위험·목표 표현 → 경로/회피 | `U_goal`, `U_obs`, `gradient_U` |
| **L1 Grid Engine** | 공간 상태 표현 (해역 그리드) | `Grid5DAdapter` |
| **L2 Hippo Memory** | 맥락/장소 기억 → 우물로 저장·리콜 | `WellMemory`, `U_mem_at` |
| **L3 Cerebellum** | 운동 패턴 — 반복 기동·선회 학습 | `CerebellumProfile`, `exponential_profile` |
| **Prefrontal** | 모드 전환 의사결정·안전 규칙 | `ModeManager`, `VesselMode` |
| **Brainstem** | 저수준 반사 — 비상 부상·SCRAM | `auto_safety_check()`, `ReactorStatus.SCRAMMED` |
| **상태 추정** | 센서 → 위치·속도 추정 | `KalmanEstimator` (EKF-style) |

**제어 흐름:**

```
센서 → KalmanEstimator → {
    Potential Field (L1) + WellMemory (L2) → ψ_ref, U_ref  [경로]
    WaypointSequencer → U_ref_scale                         [순차 waypoint]
    CerebellumProfile (L3) → ψ_ref(t), U_ref(t) 보정       [운동 패턴]
    ModeManager → VesselMode                                [모드 결정]
    DepthController → τ_w, τ_q, τ_p                         [수심 제어]
    IceInterface → τ_ram                                    [빙 파쇄]
    NuclearReactor → τ_u                                    [추진]
} → step_6dof_rk4() → 다음 State
```

---

## 5. 핵심 수식 — 제어 이론

### 5.1 3-DOF 수상 동역학

상태: **ν** = [u, v, r]ᵀ (surge, sway, yaw rate), **η** = [ξ, η, ψ]ᵀ

```
M ν̇ + D ν = τ + τ_env

운동학:
  ξ̇ = u cosψ − v sinψ
  η̇ = u sinψ + v cosψ
  ψ̇ = r
```

수치 적분: **RK4** (기본) 또는 Euler (`IntegrationMethod.EULER`)

### 5.2 6-DOF 잠수함 동역학 (Fossen)

상태: **ν** = [u, v, w, p, q, r]ᵀ, **η** = [ξ, η, z, φ, θ, ψ]ᵀ

```
M₆ ν̇ + D₆ ν + g(η) = τ + τ_env

운동학 — Euler 자세각 Jacobian J(η):
  [ξ̇, η̇, ż]ᵀ = R_nb · [u, v, w]ᵀ
  φ̇ = p + (q sinφ + r cosφ) tanθ
  θ̇ = q cosφ − r sinφ
  ψ̇ = (q sinφ + r cosφ) secθ   [θ = ±90° 특이점 보호: max(|cosθ|, 1e-6)]

복원력 g(η):
  g_z = W − B               [N]    순 중력 (중성 부력이면 0)
  g_φ = GM_L · W · sinφ    [N·m]  roll 복원
  g_θ = GM_T · W · sinθ    [N·m]  pitch 복원
```

### 5.3 퍼텐셜 필드 (해석적 기울기)

```
U_total = U_goal + Σ U_obs,i + U_mem(c) + U_ice

흡인 우물 (목표):
  U_goal = ½ k_goal [(ξ−ξ_d)² + (η−η_d)²]
  ∇U_goal = k_goal · (Δξ, Δη)

반발 우물 — 가우시안:
  U_obs = A · exp(−ρ²/2σ²)
  ∇U_obs = U_obs · (−Δξ/σ², −Δη/σ²)

반발 우물 — 거리 기반:
  U_obs = ½ A (1/ρ − 1/ρ₀)²  [ρ < ρ₀]
  ∇U_obs = −A (1/ρ − 1/ρ₀) · (Δξ, Δη) / ρ³

경로 명령:
  ψ_ref = atan2(−∂U/∂η, −∂U/∂ξ)
  U_ref = U_d_ms  (waypoint 목표 속도)
```

해석적 기울기(`GradientMethod.ANALYTIC`)는 수치 차분 대비 O(N) — 정확도 동일, 연산 절반.

### 5.4 깊이·자세 PD 제어

```
τ_w = clip(k_z·(z_ref − z) − k_z_d·ẇ,  ±max_τ_w)    [N]    heave
τ_q = clip(k_θ·(θ_ref − θ) − k_θ_d·q,  ±max_τ_q)    [N·m]  pitch
τ_p = clip(−k_φ·φ,                        ±max_τ_p)   [N·m]  roll (수평 유지)

비상 부상: τ_w = −max_τ_w (최대 상방), τ_q = 0
```

### 5.5 빙 파쇄력 (IceInterface)

```
F_ram = min(A_ram · C_ice · hardness · v_approach², F_max)    [N]

수상 쇄빙 (ICEBREAKER):   τ_ram → surge (전방 파쇄)
잠수 쇄빙 (SUBMERGED_ICE): τ_ram → heave (상향 파쇄)
빙 천장 회피 (UNDER_ICE):  τ_w = k_avoid / clearance (반발)
```

### 5.6 핵추진 (PWR)

```
dP/dt = (P_demand − P) / τ_reactor               [1차 지연, τ≈30s]
P_mech = η_mechanical · P_thermal                 [기계 출력 W]
τ_shaft = P_mech / ω_shaft                        [N·m]
τ_u = τ_shaft / R_prop · n_shafts                 [N] 총 추진력

SCRAM 조건: P / P_rated > 1.15 → P = 0, 재기동 불가
최소 출력: P_min_frac = 0.15 (정상 운전 하한)

참고치: OK-650B (Typhoon급) 190 MWth / S9G (Virginia급) 210 MWth
```

### 5.7 쇄빙 램 외란 (τ_ram)

```
τ_ram(t) = T_ram · exp(−(t − t_impact)² / 2σ_ram²)   [N·m]

τ_env에 가산 → OrbitStabilizer / PropulsionManager가 외란 흡수
맥락: open_water | ice_transit | ice_ram → 프로파일 전환
```

### 5.8 KalmanEstimator (EKF-style)

```
6-상태: x = [ξ, η, ψ, u, v, r]ᵀ

예측: x_pred = f(x, τ, dt)     [3-DOF 동역학으로 선형화]
      P_pred = F·P·Fᵀ + Q

갱신: K = P_pred·Hᵀ·(H·P_pred·Hᵀ + R)⁻¹
      x = x_pred + K·(z − H·x_pred)
      P = (I − K·H)·P_pred

σ_ξ 수렴: 6.325m (초기) → 0.560m (30스텝 후)
```

---

## 6. 모듈 구조

```
avce/                               # AVCE 루트
├── README.md                       # 이 문서 (v0.4)
├── run_demo.py                     # 3-DOF 데모
├── run_demo_extended.py            # Cerebellum + 램 + 폐루프
├── docs/
│   ├── CONCEPTS.md                 # 인지 매핑, 상태공간, 제어 계층
│   ├── EQUATIONS.md                # 전체 수식 (§1–10)
│   └── PRECISION_CONTROL_ANALYSIS.md
├── blockchain/                     # SHA256 무결성 서명
└── avce/                           # Python 패키지 (92 공개 API)
    ├── __init__.py                 # 92 exports
    ├── controller.py               # VesselController (3-DOF 통합)
    │
    ├── core/                       # 상태·상수·계약 [레이어 0]
    │   ├── state.py                # VesselState(3-DOF), Waypoint, Obstacle, IceCell
    │   ├── state6dof.py            # VehicleState(12D), IceLayer(3D), OceanCell ★NEW
    │   ├── mode.py                 # VesselMode(8종), ModeConfig, ModeProfile ★NEW
    │   ├── constants.py            # k_goal, ρ₀, CONTEXT_*, PRECISION_*
    │   ├── ramming.py              # RammingImpact, τ_ram(t)
    │   ├── estimator.py            # KalmanEstimator (EKF), StateEstimator 계약
    │   └── sequencer.py            # WaypointSequencer (다중 waypoint)
    │
    ├── control/                    # 경로·회피·자세 [레이어 1]
    │   ├── potential_field.py      # U_total, gradient_U (해석/수치), psi_ref
    │   ├── path_controller.py      # path_controller() → PathOutput
    │   ├── cerebellum_profile.py   # CerebellumProfile, exponential_profile
    │   ├── depth_controller.py     # DepthController (PD: z/θ/φ) ★NEW
    │   └── ice_interface.py        # IceInterface (3D 빙 분석·파쇄) ★NEW
    │
    ├── simulation/                 # 동역학·루프 [레이어 2]
    │   ├── dynamics.py             # 3-DOF: step_3dof_rk4, IntegrationMethod
    │   ├── dynamics6dof.py         # 6-DOF Fossen: step_6dof_rk4, tau6_from_setpoints ★NEW
    │   ├── environment.py          # BuoyancyModel, OceanEnvironment ★NEW
    │   └── loop.py                 # simulation_loop_step, run_simulation
    │
    ├── propulsion/                 # 추진 시스템 [레이어 2] ★NEW
    │   ├── nuclear.py              # NuclearReactor (PWR), ReactorParams, SCRAM
    │   └── propulsion_manager.py   # PropulsionManager (NUCLEAR/DIESEL/HYBRID/BATTERY)
    │
    ├── mode/                       # 모드 관리 [레이어 2] ★NEW
    │   └── manager.py              # ModeManager, TransitionResult
    │
    ├── memory/                     # 기억 우물 [레이어 1]
    │   └── well_memory.py          # WellMemory, Well, U_mem_at
    │
    └── integration/                # 선택 연동 [레이어 3]
        ├── orbit_stabilizer_adapter.py
        └── grid5d_adapter.py
```

> ★NEW = v0.4에서 새로 추가된 파일

---

## 7. 구현된 기능 전체

### 7.1 핵심 기능

| 기능 | 위치 | 상세 |
|------|------|------|
| **3-DOF 수상 동역학** | `simulation/dynamics.py` | Euler / RK4, DynamicsParams (frozen) |
| **6-DOF 잠수함 동역학** | `simulation/dynamics6dof.py` | Fossen 방정식, RK4, 복원력 g(η) |
| **퍼텐셜 필드** | `control/potential_field.py` | U_goal / U_obs(가우시안·거리) / U_ice, **해석적 기울기** |
| **경로 제어** | `control/path_controller.py` | ψ_ref, U_ref, F_pf_mag |
| **깊이·피치 제어** | `control/depth_controller.py` | PD: τ_w/τ_q/τ_p, deadband, 포화 |
| **3D 빙 인터페이스** | `control/ice_interface.py` | 수면/천장 빙, F_ram = A·C·h·v², 천장 회피 |
| **운항 모드** | `core/mode.py` | VesselMode 8종, ModeConfig (frozen), 전환 그래프 |
| **모드 관리자** | `mode/manager.py` | 안전 전환 검사, 자동 비상 발동 |
| **핵추진** | `propulsion/nuclear.py` | PWR 1차 지연, SCRAM, τ_u = P_mech/ω·n |
| **추진 통합** | `propulsion/propulsion_manager.py` | NUCLEAR / DIESEL / HYBRID / BATTERY |
| **부력 모델** | `simulation/environment.py` | BuoyancyModel (밸러스트 제어), OceanEnvironment |
| **KalmanEstimator** | `core/estimator.py` | EKF-style 6-state, set_control() |
| **WaypointSequencer** | `core/sequencer.py` | 다중 waypoint 순차 처리, 도달 감지, U_ref_scale |
| **기억 우물** | `memory/well_memory.py` | 맥락 c별 우물 집합, 시간 감쇠 |
| **Cerebellum 프로파일** | `control/cerebellum_profile.py` | 감쇠형 ψ_ref(t), U_ref(t) |
| **쇄빙 램** | `core/ramming.py` | τ_ram(t) 가우시안 펄스, 다중 충돌 |
| **OrbitStabilizer 연동** | `integration/orbit_stabilizer_adapter.py` | 다축 위상 보정 |

### 7.2 성능 수치

| 항목 | 수치 |
|------|------|
| 공개 API | **92 exports** |
| 지원 운항 모드 | **8종** |
| 추진 방식 | **4종** (NUCLEAR / DIESEL_ELECTRIC / HYBRID / EMERGENCY_BATTERY) |
| 동역학 자유도 | **3-DOF** (수상) / **6-DOF** (잠수) 자동 전환 |
| 상태 벡터 | **12D** (ξ,η,z,φ,θ,ψ,u,v,w,p,q,r) |
| 적분 방식 | **RK4** (기본) / Euler (`IntegrationMethod`) |
| 기울기 방식 | **해석적** O(N) / 수치 O(2N) (`GradientMethod`) |
| KalmanEstimator 수렴 | σ_ξ: 6.325m → **0.560m** (30 스텝) |
| 핵추진 참고 출력 | **7.6 MN** (190 MWth PWR, 80% 출력, τ_reactor=30s) |
| 빙 파쇄력 (최대) | **5 MN** (수상 쇄빙), **3 MN** (잠수 쇄빙) |

---

## 8. 공개 API — 92 exports

```python
import avce

# ── core: 3-DOF ──────────────────────────────────────────────────────
avce.VesselState, Waypoint, Obstacle, IceCell, AxisSetpoint
avce.DEFAULT_K_GOAL, DEFAULT_K_OBS, RHO_0_DEFAULT
avce.GAUSSIAN_SIGMA_MIN, PRECISION_DEG, PRECISION_RAD
avce.PREDICTION_HORIZON_MS, FAILSAFE_MS
avce.CONTEXT_OPEN_WATER, CONTEXT_ICE_TRANSIT, CONTEXT_ICE_RAM
avce.RammingImpact, RammingDisturbance, tau_ram_at
avce.StateEstimator, IdentityEstimator, KalmanConfig, KalmanEstimator
avce.ArrivalConfig, SpeedConfig, SequencerStatus, SequencerResult, WaypointSequencer

# ── core: 6-DOF ──────────────────────────────────────────────────────
avce.VehicleState           # 12D 전 운항 상태
avce.IceLayer               # 3D 빙층 (z_top, z_bottom)
avce.OceanCell              # 해양 환경 격자

# ── mode ─────────────────────────────────────────────────────────────
avce.VesselMode             # 8종 Enum
avce.ModeConfig, ModeProfile, default_mode_config
avce.allowed_transitions, can_transition
avce.ModeManager, TransitionResult

# ── control ──────────────────────────────────────────────────────────
avce.GradientMethod, gradient_U_analytic, gradient_U_numeric, gradient_U
avce.U_goal, U_obs, U_total, psi_ref_from_gradient
avce.path_controller, PathOutput
avce.CerebellumProfile, ProfilePoint, exponential_profile
avce.DepthControlConfig, DepthController
avce.IceInterfaceConfig, IceContactResult, IceInterface

# ── simulation: 3-DOF ────────────────────────────────────────────────
avce.DynamicsParams, IntegrationMethod
avce.step_3dof, step_3dof_rk4, step_3dof_method
avce.tau_from_heading_and_surge
avce.simulation_loop_step, run_simulation, SimulationStepResult

# ── simulation: 6-DOF ────────────────────────────────────────────────
avce.DynamicsParams6DOF
avce.step_6dof, step_6dof_rk4, step_6dof_method
avce.tau6_from_setpoints
avce.BuoyancyParams, BuoyancyModel, OceanEnvironment

# ── propulsion ───────────────────────────────────────────────────────
avce.ReactorStatus, ReactorParams, ReactorState, NuclearReactor
avce.PropulsionType, DieselParams, BatteryParams, PropulsionManager

# ── memory · integration · controller ────────────────────────────────
avce.WellMemory, Well, U_mem_at
avce.OrbitStabilizerAdapter, AxisCorrection
avce.Grid5DAdapter, Grid5DRefinement, vessel_state_to_5d_phase
avce.VesselController, VesselStepResult
```

---

## 9. 사용 예시

### 9.1 수상 경로 제어 (3-DOF)

```python
from avce import VesselState, Waypoint, Obstacle, path_controller

state = VesselState(xi_m=0.0, eta_m=0.0, psi_rad=0.0, u_ms=2.0, v_ms=0.0, r_rads=0.0)
wp    = Waypoint(xi_m=1000.0, eta_m=500.0, U_d_ms=5.0)
obs   = [Obstacle(xi_m=400.0, eta_m=200.0, A=2.0, sigma_m=150.0)]
out   = path_controller(state, wp, obs)
# out.psi_ref_rad, out.U_ref_ms, out.F_pf_mag
```

### 9.2 폐루프 시뮬레이션

```python
from avce import VesselController, run_simulation, DynamicsParams, RammingDisturbance, RammingImpact

ctrl    = VesselController(waypoint=wp, obstacles=obs, orbit_stabilizer_axes=["psi"])
ramming = RammingDisturbance()
ramming.add_impact(RammingImpact(t_impact_s=5.0, T_ram_nm=2e5, sigma_ram_s=0.5))
results = run_simulation(state, ctrl, T_s=30.0, dt_s=0.1, ramming=ramming)
```

### 9.3 6-DOF 잠수 동역학

```python
from avce import VehicleState, DynamicsParams6DOF, step_6dof_rk4

params = DynamicsParams6DOF(m_u=1e5, m_w=1.5e5, W_n=0.0, B_n=0.0)
state  = VehicleState(xi_m=0, eta_m=0, z_m=50.0)   # 50m 수심
tau    = (1e5, 0, 0, 0, 0, 0)                        # surge 추진
for _ in range(100):
    state = step_6dof_rk4(state, tau, dt_s=0.1, params=params)
# state.u_ms, state.xi_m, state.z_m
```

### 9.4 핵추진 반응기

```python
from avce import NuclearReactor, ReactorParams, ReactorStatus

reactor = NuclearReactor(ReactorParams(P_rated_mw=190.0, tau_response_s=30.0))
reactor.startup()
for _ in range(30):
    rs = reactor.step(demand_frac=0.8, omega_rads=5.0, dt_s=10.0)
# rs.status == ReactorStatus.OPERATING
# rs.tau_u_n  → ~7.6e6 N
```

### 9.5 운항 모드 전환

```python
from avce import ModeManager, ModeProfile, VesselMode, VehicleState

mgr = ModeManager(initial_mode=VesselMode.SURFACE, profile=ModeProfile())
state = VehicleState(z_m=0.0, u_ms=3.0)

r = mgr.request_transition(VesselMode.TRANSIT_DIVE, state)
# r.success, r.new_mode

# 자동 비상: 깊이 초과 시
danger = VehicleState(z_m=350.0)          # emergency_depth_m=300m 초과
emerg  = mgr.auto_safety_check(danger)
# emerg.new_mode == VesselMode.EMERGENCY_SURFACE
```

### 9.6 깊이 제어

```python
from avce import DepthController, DepthControlConfig, VehicleState

ctrl  = DepthController(DepthControlConfig(k_z=1e4, max_tau_w=5e5))
state = VehicleState(z_m=10.0, w_ms=0.0)
tau   = ctrl.step(state, z_ref=50.0, theta_ref=0.0)
# tau[2] > 0  →  heave 하강 (NED: Down+)
```

### 9.7 WaypointSequencer (다중 waypoint)

```python
from avce import WaypointSequencer, ArrivalConfig, Waypoint

waypoints = [
    Waypoint(xi_m=500,  eta_m=0,   U_d_ms=5.0),
    Waypoint(xi_m=1000, eta_m=500, U_d_ms=4.0),
    Waypoint(xi_m=1500, eta_m=500, U_d_ms=3.0),
]
seq = WaypointSequencer(waypoints, ArrivalConfig(radius_m=20.0))
result = seq.update(state)
# result.current_waypoint, result.U_ref_scale, result.status
```

---

## 10. 의존성 · 설계 원칙

### 의존성

| 구분 | 패키지 | 용도 |
|------|--------|------|
| **필수** | 없음 | 순수 Python 3.10+, stdlib만 사용 |
| **선택** | `orbit_stabilizer_sdk` | 다축 위상 보정 (설치 시 자동 활성화) |
| **선택** | `grid_engine` | Grid 5D 위상 매핑 |

### 레이어 의존 방향 (단방향)

```
core  →  control  →  simulation
 ↑           ↑            ↑
(하위)    (중간)        (상위)

propulsion  →  simulation  (추진 → 시뮬)
mode        →  core        (모드 정의 ← core에서)
mode.manager→  core + simulation (모드 관리자)
```

역방향 임포트 없음. `core`에서 `simulation` 임포트 없음.

---

## 11. 블록체인 서명

핵심 모듈 파일의 **SHA256**을 기록해 변조 추적을 지원한다.

```bash
python3 blockchain/sign_avce.py   # signature.json, SIGNATURE.md 생성
```

→ `blockchain/README.md`, `blockchain/SIGNATURE.md` 참고.

---

## 12. 버전 이력

| 버전 | 내용 |
|------|------|
| **v0.1** | 개념·수식 (CONCEPTS.md, EQUATIONS.md) |
| **v0.2** | 구현 — core/control/memory/integration, VesselController, run_demo |
| **v0.3** | 확장 — Cerebellum 프로파일, 쇄빙 τ_ram(t), KalmanEstimator(EKF), WaypointSequencer, RK4, 해석적 기울기, 블록체인 서명 |
| **v0.4** | **하이브리드 다중모드** — VesselMode(8종), VehicleState(12D), 6-DOF Fossen, NuclearReactor(PWR+SCRAM), PropulsionManager(4종), DepthController(PD), IceInterface(3D), ModeManager(자동 비상), BuoyancyModel, OceanEnvironment. 총 **92 exports** |

---

*AVCE — 어디든 뚫고 지나간다. 수면이든, 얼음 아래든, 심해든.*
