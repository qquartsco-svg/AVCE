# Autonomous Vessel Control Engine

**자율선박 제어 AI — 인지과학 기반 제어 시스템 (확장 모듈)**

[![Layer](https://img.shields.io/badge/layer-60_APPLIED-blue.svg)](.)
[![Status](https://img.shields.io/badge/status-implemented-green.svg)](.)

---

## 목적

- **지금 만드는 것**: “전체 자율선박 시스템”이 아니라, **확장 가능한 독립 모듈 엔진**의 **개념·수식·계약**.
- 기존에 만든 로직을 **자율선박 제어**로 확장할 때의 **공통 스펙**을 이 폴더에 둔다.
  - CookiieBrain: 상태공간, 우물, Hippo, Cerebellum, Potential Field.
  - 마린 SDK: OrbitStabilizer, Marine Propulsion (추진축 위상 안정화·마모 감소).

---

## 원칙

1. **개념·수식 먼저**: 구현보다 **CONCEPTS.md**, **EQUATIONS.md**를 고정한 뒤, 코드는 그에 맞춰 채운다.
2. **완전 독립 모듈**: 이 폴더만으로도 “무엇을 입력받고 무엇을 내보내는지” 계약이 읽힌다.
3. **확장**: 실제 제어 루프·센서·선박 시뮬레이터는 외부에서 붙인다. 이 엔진은 **인지과학 기반 제어 레이어**의 스펙과 수식이다.

---

## 폴더 구조

```
avce/   # 자율선박·쇄빙선 제어 모듈 (Autonomous Vessel Control Engine)
├── README.md
├── run_demo.py                  # 데모: 경로 제어 + (선택) OrbitStabilizer
├── run_demo_extended.py         # 확장 데모: Cerebellum, 램 외란, 폐루프 시뮬
├── docs/
│   ├── CONCEPTS.md
│   ├── EQUATIONS.md
│   └── PRECISION_CONTROL_ANALYSIS.md
└── avce/                        # Python 패키지
    ├── __init__.py
    ├── controller.py            # VesselController (Cerebellum·램 옵션)
    ├── core/
    │   ├── state.py             # VesselState, Waypoint, Obstacle, IceCell
    │   ├── constants.py
    │   ├── ramming.py           # τ_ram(t) 쇄빙 램 외란
    │   └── estimator.py         # 상태 추정 계약 (z → x̂)
    ├── control/
    │   ├── potential_field.py   # U_goal, U_obs, U_ice, −∇U
    │   ├── path_controller.py   # ψ_ref, U_ref
    │   └── cerebellum_profile.py # Cerebellum-style (ψ_ref, U_ref) 프로파일
    ├── memory/
    │   └── well_memory.py       # 맥락별 우물 U_mem
    ├── simulation/              # 3-DOF 동역학·폐루프 시뮬
    │   ├── dynamics.py          # step_3dof, tau_from_heading_and_surge
    │   └── loop.py              # simulation_loop_step, run_simulation
    └── integration/             # 선택 의존
        ├── orbit_stabilizer_adapter.py  # 다축 OrbitStabilizer
        └── grid5d_adapter.py            # Grid 5D (선택)
```

---

## 인지과학 매핑 (요약)

| 레이어 | 역할 |
|--------|------|
| **Ring Attractor / OrbitStabilizer** | 위상 궤적 기억 + 예측 보정 → PID 앞단 |
| **Marine Propulsion** | 추진축에 적용 → 마모 감소 |
| **Potential Field (우물)** | 목표·장애물 → 경로/회피 (−∇U) |
| **Grid Engine** | 공간 상태 표현 (해역 그리드) |
| **Hippo Memory** | 맥락/장소 기억 → 항로·이벤트 우물 리콜 |
| **Cerebellum** | 운동 패턴 학습 → 조타/추진 프로파일 |

자세한 정의는 **docs/CONCEPTS.md**.

**쇄빙선**: 같은 엔진으로 확장 가능. 빙 농도→퍼텐셜 \(U_{ice}\), 램 시 축 충격→OrbitStabilizer 외란 흡수, 맥락 `ice_ram`/`ice_transit`. → **docs/CONCEPTS.md §6**, **docs/EQUATIONS.md §10**.

---

## 수식 요약

- **선박 동역학**: 3-DOF Mν̇ + C(ν)ν + Dν = τ + τ_env, 위치·자세 적분.
- **퍼텐셜**: U = U_goal + Σ U_obs,i + U_mem(c) → ψ_ref, U_ref from −∇U.
- **위상 안정화**: φ_pred, e_pred → F_correction (OrbitStabilizer).
- **기억 우물**: 맥락 c → 우물 집합 W(c) → U_mem.

전부 **docs/EQUATIONS.md** 에 정리.

---

## 의존성

- **없음**: 퍼텐셜·경로 제어·WellMemory·VesselController는 순수 Python만 사용.
- **선택**: OrbitStabilizer (`orbit_stabilizer` 또는 `hippo_memory.orbit_stabilizer`) 설치 시 다축 보정; Grid 5D (`grid_engine`) 설치 시 Grid5DAdapter 사용 가능.

OrbitStabilizer/Grid 미설치 시에도 경로 계층(ψ_ref, U_ref)은 그대로 동작한다.

이 엔진은 “개념·수식·계약”만 두었으므로, **코드 의존성은 구현 단계에서** 추가한다.

---

## 사용

```python
from avce import (
    VesselState, Waypoint, Obstacle,
    VesselController, path_controller,
)

state = VesselState(0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
wp = Waypoint(1000.0, 500.0, U_d_ms=5.0)
obstacles = [Obstacle(400.0, 200.0, A=2.0, sigma_m=150.0)]

# 경로만
out = path_controller(state, wp, obstacles)
# out.psi_ref_rad, out.U_ref_ms

# 통합 (OrbitStabilizer 축 지정 시, 설치되어 있으면 보정 포함)
ctrl = VesselController(waypoint=wp, obstacles=obstacles, orbit_stabilizer_axes=["psi"])
result = ctrl.step(state)
# result.psi_ref_rad, result.U_ref_ms, result.corrections

# Cerebellum 프로파일 보정 + 램 외란 + 폐루프 시뮬
from avce import (
    CerebellumProfile, RammingDisturbance, RammingImpact,
    run_simulation, DynamicsParams,
)
profile = CerebellumProfile(T_psi_s=8.0, T_U_s=12.0)
ctrl = VesselController(waypoint=wp, obstacles=obstacles, cerebellum_profile=profile, profile_blend=0.3)
ramming = RammingDisturbance()
ramming.add_impact(RammingImpact(t_impact_s=5.0, T_ram_nm=2e5, sigma_ram_s=0.5))
results = run_simulation(state0, ctrl, T_s=30.0, dt_s=0.1, ramming=ramming)
```

데모: `python run_demo.py`, `python run_demo_extended.py` (패키지 루트에서 실행).

---

## 버전

- **v0.1**: 개념·수식 (CONCEPTS.md, EQUATIONS.md).
- **v0.2**: 구현 — core/control/memory/integration, VesselController, run_demo. OrbitStabilizer/Grid5D 선택 연동.
- **v0.3**: 확장 — Cerebellum 프로파일(ψ_ref, U_ref 시퀀스), 쇄빙 램 τ_ram(t), 상태 추정 계약(StateEstimator), 3-DOF 시뮬(dynamics·run_simulation), run_demo_extended.
