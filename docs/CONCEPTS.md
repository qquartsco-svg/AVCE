# Autonomous Vessel Control Engine — 개념 정리

**완전 독립 모듈 엔진.**  
기존 CookiieBrain/마린 SDK 로직을 **자율선박 제어 AI**로 확장할 때의 개념·계약을 정의한다.  
“지금 바로 선박을 띄우는 것”이 아니라, **확장 가능한 개념·수식·인터페이스**를 먼저 고정한다.

---

## 1. 범위와 위치

### 1.1 이 엔진이 하는 일

- **자율선박 제어 시스템**의 **개념·수식·계약**을 한 곳에 정의한다.
- 구현은 다른 독립 엔진(marine_propulsion_engine, orbit_stabilizer_sdk, WellFormation, PotentialField, Hippo, Cerebellum 등)을 **호출**하는 형태로 확장한다.
- 따라서 이 폴더는 **스펙 + 수식 + 인터페이스**가 중심이고, 실제 제어 루프/시뮬레이터는 이후 단계에서 채운다.

### 1.2 독립 모듈로서의 계약

- **입력**: 센서/추정 상태(위치, 속도, 코스, 목표 waypoint, 장애물 정보 등) — 계약으로 정의.
- **출력**: 추진/조타 명령 또는 하부 제어기(OrbitStabilizer, Marine Propulsion)에 넘길 setpoint.
- **의존성**: orbit_stabilizer_sdk, marine_propulsion_engine은 **선택 의존** (없으면 경로·의사결정만 제공).

---

## 2. 인지과학 매핑 — “누가 무슨 역할을 하는가”

CookiieBrain/ENGINE_HUB에 이미 있는 로직을 **자율선박 제어**에 대응시키면 아래와 같다.

| 인지/엔진 레이어 | 제어 시스템에서의 역할 | 상태/입출력 |
|------------------|------------------------|-------------|
| **L0 Ring Attractor** | 회전 위상(각도)의 **국소 기억** · 안정 궤적 | 위상 φ, 각속도 ω → “정상 궤적” 기억 |
| **OrbitStabilizer** | 위상 궤적 기억 + **예측 보정** → PID 앞단 | φ, 목표 → 보정력 F_correction |
| **Marine Propulsion** | 선박 추진축에 OrbitStabilizer 적용 → **마모 감소** | 축 토크/각도 → 보정된 토크 명령 |
| **L1 Potential Field / 우물** | **공간 위험·목표** 표현 → 경로/회피 | 장애물=반발 우물, 목표=흡인 우물 → ∇U |
| **L1 Grid Engine** | **공간 상태 표현** (해역 그리드, occupancy) | (x,y) 또는 (x,y,ψ) 그리드 |
| **L2 Hippo Memory** | **맥락/장소 기억** — 항로·이벤트를 우물로 저장·리콜 | 맥락 c → 우물 집합 {W_i(c)} |
| **L3 Cerebellum** | **운동 패턴 최적화** — 반복 기동·선회 패턴 학습 | (목표, 맥락) → 토크/조타 프로파일 |
| **상태 추정** | 센서 → 위치·자세·속도 추정 (필터/관측기) | z_sensor → x_est (본 엔진 외부 계약) |

- **제어 흐름**:  
  `센서 → 상태 추정 → (Grid + Potential Field) 경로/회피 → (Hippo) 맥락 리콜 → (Cerebellum) 운동 패턴 → (OrbitStabilizer + Marine Propulsion) 하부 제어 → 액추에이터`

---

## 3. 상태공간 정의

자율선박 제어가 다루는 **상태 벡터**를 하나로 고정한다 (계약).

### 3.1 선박 상태 (물리)

- **x** = [ξ, η, ψ, u, v, r]ᵀ  
  - ξ, η: 지구좌표계 위치 (m)  
  - ψ: 선수 방위각 (rad)  
  - u: surge 속도 (m/s)  
  - v: sway 속도 (m/s)  
  - r: yaw rate (rad/s)  
- 필요 시 3-DOF로 축약: (ξ, η, ψ) 또는 (ξ, η, ψ, U) (U = 대문 advance 속도).

### 3.2 제어/참조 상태

- **목표**: waypoint (ξ_d, η_d) 또는 코스 ψ_d, 속도 U_d.
- **궤적 위상** (OrbitStabilizer/추진축): φ ∈ [0, 2π) — 프로펠러/축 회전 위상.
- **맥락** (Hippo): c — 현재 항로·수로·이벤트 라벨 (정수 또는 벡터).

### 3.3 “인지” 상태 (엔진 내부)

- **Potential Field**: 우물 파라미터 { (x_i, y_i, strength_i, type_i) } → 스칼라장 U(ξ, η).
- **Hippo 우물**: 맥락 c에 따른 우물 집합 → 리콜 시 보정/가중 경로.
- **Cerebellum**: (목표, 맥락) → 학습된 토크/조타 시퀀스.

---

## 4. 제어 계층 (레이어)

1. **경로/의사결정**  
   Potential Field + (선택) Grid + (선택) Hippo 리콜 → “지금 가야 할 방향/속도” ψ_ref, U_ref.
2. **궤적/위상 안정화**  
   OrbitStabilizer: ψ, ψ_ref (또는 축 위상 φ, φ_ref) → 예측 보정 → PID 입력.
3. **추진 실행**  
   Marine Propulsion: 축 각도/토크 → 마모 최소화 보정 → 최종 토크/ RPM 명령.
4. **학습/기억** (선택)  
   Hippo: 통과 구간·이벤트를 우물로 저장. Cerebellum: 반복 기동 패턴 업데이트.

---

## 5. 확장 방향 (지금 구현하지 않아도 되는 것)

- COLREG 규칙 반영 (우선권·회피 각도).
- AIS/레이더 연동 (장애물 목록 → Potential Field 우물).
- 실선/시뮬레이터 연동 (ROS, NMEA, 시뮬레이터 API).
- 인증/안전 (Fail-Safe, 감시 타이머).

이들은 “개념·수식”이 정해진 뒤, **인터페이스**만 이 엔진에 두고 구현은 별도 모듈/팀으로 확장하면 된다.

---

## 6. 쇄빙선 확장 (Icebreaker)

같은 엔진을 **쇄빙선**으로 확장할 수 있다. 추가되는 것은 “얼음”을 상태·퍼텐셜·추진에 반영하는 계약뿐이다.

### 6.1 쇄빙선에서의 역할 대응

| 기능 | 기존 엔진 활용 | 확장 내용 |
|------|----------------|-----------|
| **얼음 회피/경로** | Potential Field + Grid | 얼음 두께·농도를 **비용 또는 반발 우물**로 표현. 빙량이 큰 구역 = U_ice 높음 → −∇U가 얇은 빙/열린 물로 유도. |
| **충격·램(Ramming)** | OrbitStabilizer + Marine Propulsion | 쇄빙 시 축에 가해지는 **순간 토크 스파이크**를 “외란”으로 흡수. 위상 기억 + 예측 보정으로 램 직후 복귀, 마모·피로 감소. |
| **전진력(저속 고추진)** | Marine Propulsion / Cerebellum | “램 모드” vs “통항 모드”를 **맥락 c**로 구분. c=ice_ram → 고토크·저속 프로파일 재생. |
| **빙해역 경로 학습** | Hippo Memory | 통과한 빙 분포·통로를 우물로 저장 → 같은 구역 재진입 시 리콜로 경로 보정. |

### 6.2 상태·입력 확장

- **빙 정보** (센서/외부): 격자별 빙 농도 \(C_{ice}(i,j)\) 또는 두께 \(h_{ice}(i,j)\).  
  → Potential Field의 \(U_{ice}(\xi,\eta)\) 또는 Grid 비용으로 사용.
- **맥락 c**: `open_water` | `ice_transit` | `ice_ram` 등.  
  → Cerebellum 프로파일·OrbitStabilizer gain 전환.

### 6.3 제어 흐름 (쇄빙선)

`센서(위치·빙 맵) → 상태 추정 → U = U_goal + U_obs + U_ice + U_mem(c) → ψ_ref, U_ref (또는 램 모드 시 토크 프로파일) → OrbitStabilizer + Marine Propulsion → 액추에이터`

- **U_ice**: 빙 농도/두께에 비례하는 비용 또는 반발. 상세는 EQUATIONS.md §10.

---

## 7. 문서 구성

- **CONCEPTS.md** (이 파일): 개념, 인지 매핑, 상태공간, 제어 계층, **쇄빙선 확장**.
- **EQUATIONS.md**: 수식 전부 (동역학, 퍼텐셜, 위상 보정, 우물, 운동 패턴, **빙 퍼텐셜·램**).
- **README.md**: 엔진 목적, 폴더 구조, 의존성, 사용 예시(계약 수준).

구현 코드는 `core/`, `control/`, `memory/` 등 서브폴더로 나누되, **개념·수식이 먼저 고정**된 뒤 채워 넣는다.
