# Autonomous Vessel Control Engine — 수식 정리

모든 식은 **계약** 수준으로 정의한다. 부호·좌표계·단위를 고정해 두고, 구현 시 참조한다.

---

## 1. 좌표계·부호 규칙

- **지구 좌표계**: North-East-Down (NED) 또는 동쪽 ξ, 북쪽 η. ψ = 0 = 북쪽, ψ 증가 = 시계반대(선수 우현).
- **선박 고정 좌표계**: 선수-우현-하방. surge = u, sway = v, yaw rate = r.
- **각도**: rad 기본. 필요 시 ° 명시.

---

## 2. 선박 동역학 (3-DOF)

### 2.1 운동 방정식

질량·부가질량·감쇠를 묶은 3-DOF 모델:

$$
\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}\boldsymbol{\nu} = \boldsymbol{\tau} + \boldsymbol{\tau}_{env}
$$

- **상태**: \(\boldsymbol{\nu} = [u,\,v,\,r]^\top\)
- **위치·자세**: \(\dot{\xi} = u\cos\psi - v\sin\psi,\quad \dot{\eta} = u\sin\psi + v\cos\psi,\quad \dot{\psi} = r\)
- **M**: 질량+부가질량 행렬 (대각 또는 비대각)
- **C(ν)**: 코리올리·구심 항
- **D**: 선형 감쇠 (비선형 감쇠는 별도 항)
- **τ**: 제어 입력 \([F_u,\,F_v,\,N_r]^\top\) (추진력·조타)
- **τ_env**: 바람·조류·파 외력

### 2.2 단순화 (경로 제어용)

- **선형화**: 정상 직진 (u≈U0, v≈0, r≈0) 근처에서 \(\mathbf{M}\dot{\boldsymbol{\nu}} + \mathbf{D}\boldsymbol{\nu} \approx \boldsymbol{\tau}\) 로 쓰고, 전진 속도 U = √(u²+v²) 또는 u로 근사.
- **1차 전달함수** (선수 방위각 ψ ↔ 조타각 δ):
  $$
  \frac{\psi}{\delta}(s) = \frac{K}{s(T s + 1)}
  $$
  K, T는 선형화 계수 (선형/선박별).

---

## 3. 퍼텐셜 필드 (Potential Field)

### 3.1 스칼라 장

목표·장애물을 우물(well)로 두고 합산:

$$
U(\xi,\eta) = U_{goal}(\xi,\eta) + \sum_i U_{obs,i}(\xi,\eta)
$$

### 3.2 흡인 우물 (목표)

$$
U_{goal}(\xi,\eta) = \frac{1}{2} k_{goal} \big[ (\xi-\xi_d)^2 + (\eta-\eta_d)^2 \big]
$$

- \(k_{goal} > 0\). 목표 \((\xi_d,\eta_d)\) 쪽으로 −∇U가 끌어당김.

### 3.3 반발 우물 (장애물)

가우시안형 (CookiieBrain 우물→가우시안과 동일 개념):

$$
U_{obs,i}(\xi,\eta) = A_i \exp\left( -\frac{ (\xi-x_i)^2 + (\eta-y_i)^2 }{ 2\sigma_i^2 } \right)
$$

- \(A_i > 0\): 반발 강도.  
- \((x_i,y_i)\): 장애물 중심.  
- \(\sigma_i\): 영향 반경.

또는 거리 기반:

$$
U_{obs,i} = \frac{k_{obs,i}}{2}\left( \frac{1}{\rho_i} - \frac{1}{\rho_0} \right)^2 \quad (\rho_i \le \rho_0),\qquad \rho_i = \sqrt{(\xi-x_i)^2 + (\eta-y_i)^2}
$$

- \(\rho_0\): 영향 반경. \(\rho_i > \rho_0\) 이면 \(U_{obs,i}=0\).

### 3.4 제어력으로의 변환

경로 제어에서 “가야 할 방향”은 퍼텐셜의 음의 그래디언트 방향:

$$
\mathbf{F}_{pf} = -\nabla U = -\left( \frac{\partial U}{\partial \xi},\, \frac{\partial U}{\partial \eta} \right)^\top
$$

- **ψ_ref**: \(\psi_{ref} = \mathrm{atan2}(-\partial U/\partial \eta,\, -\partial U/\partial \xi)\) (또는 속도 벡터 방향).
- 크기: \(\|\mathbf{F}_{pf}\|\) 또는 상수 gain으로 속도/추진 명령으로 변환.

---

## 4. 궤적/위상 안정화 (OrbitStabilizer)

### 4.1 위상 기억

Ring Attractor 기반: “정상 회전 위상” φ를 국소적으로 기억.  
이산 시간에서:

$$
\phi_{k+1} = \phi_k + \omega_k \Delta t + \frac{1}{2} \alpha_k (\Delta t)^2
$$

- \(\omega\): 위상 속도 (각속도에 대응).  
- \(\alpha\): 위상 가속도.

### 4.2 예측 오차·보정력

목표 위상 \(\phi_{ref}\), 예측 위상 \(\phi_{pred}(\phi, \omega, \alpha, \Delta t)\):

$$
e_{pred} = \phi_{ref} - \phi_{pred},\qquad
F_{correction} = k_{stab}\, e_{pred}\, \mathrm{sign}(\Delta\phi),\quad \Delta\phi = \phi_{ref} - \phi
$$

- **최종 제어**: 기존 PID 출력 + \(F_{correction}\) (트로이 목마: PID 앞단 보조).

### 4.3 선박 적용

- **추진축**: φ = 프로펠러/축 회전 위상 → Marine Propulsion이 OrbitStabilizer 출력으로 마모 감소.
- **선수 방위각**: ψ를 “위상”으로 취급하면, ψ_ref 대비 예측 오차로 조타 보정 추가 가능 (확장).

---

## 5. 기억 우물 (Hippo-style)

### 5.1 맥락별 우물

맥락 \(c\) (항로 ID, 수로 구간 등)에 따라 우물 집합:

$$
\mathcal{W}(c) = \{ (x_j, y_j, w_j, \sigma_j) \mid j \in J(c) \}
$$

- \(w_j\): 강도 (흡인/반발).  
- 위치 \((x_j,y_j)\), 폭 \(\sigma_j\).

### 5.2 퍼텐셜에의 반영

$$
U_{mem}(\xi,\eta; c) = \sum_{j \in J(c)} w_j \, f\big( \rho_j/\sigma_j \big),\quad \rho_j = \sqrt{(\xi-x_j)^2 + (\eta-y_j)^2}
$$

- \(f\): 가우시안 또는 역거리 등.  
- 리콜 시 \(\mathcal{W}(c)\)만 활성화 → 해당 구역에서만 \(U_{mem}\)이 경로에 영향.

### 5.3 우물 강화·감쇠 (에피소드)

- 통과 이벤트: 해당 위치에 우물 추가 또는 \(w_j\) 증가.
- 시간 감쇠: \(w_j \leftarrow w_j \cdot \exp(-\Delta t/\tau_w)\).

---

## 6. 운동 패턴 (Cerebellum-style)

### 6.1 목표–맥락 → 프로파일

학습된 매핑:

$$
(\psi_{ref}(t), U_{ref}(t)) \big|_{[0,T]} = \mathcal{C}(\psi_{goal},\, U_{goal},\, c)
$$

- \(\mathcal{C}\): Cerebellum 엔진 또는 룩업 테이블.  
- 반복 기동(출항·입항·선회)에서 \((\psi_{ref}, U_{ref})\) 시퀀스를 저장·재생.

### 6.2 단순 형태 (선형 + 감쇠)

$$
\psi_{ref}(t) = \psi_0 + (\psi_{goal} - \psi_0)(1 - e^{-t/T_\psi}),\qquad
U_{ref}(t) = U_0 + (U_{goal}-U_0)(1 - e^{-t/T_U})
$$

- \(T_\psi, T_U\): 시정수 (학습 또는 튜닝).

---

## 7. 상태 추정 (계약만)

- **입력**: 센서 \(z\) (GPS, IMU, AIS, 레이더 등).
- **출력**: \(\hat{\mathbf{x}} = (\hat{\xi},\hat{\eta},\hat{\psi},\hat{u},\hat{v},\hat{r})\).
- **수식**: 본 엔진에서는 **계약**만 둠. (EKF, UKF, 관측기 등은 별도 모듈.)

---

## 8. 제어 합성 (한 줄 요약)

1. **Potential Field**: \(U = U_{goal} + \sum U_{obs,i} + U_{mem}(c)\) → \(-\nabla U\) → \(\psi_{ref}, U_{ref}\).
2. **Cerebellum** (선택): \((\psi_{ref}, U_{ref})\) 시퀀스 보정.
3. **OrbitStabilizer**: (ψ, ψ_ref) 또는 (φ, φ_ref) → \(F_{correction}\).
4. **Marine Propulsion**: 축 토크/각도 + \(F_{correction}\) → 최종 추진/조타 명령.

---

## 9. 단위·상수 요약

| 기호 | 의미 | 단위 |
|------|------|------|
| ξ, η | 위치 | m |
| ψ | 방위각 | rad |
| u, v, r | 속도, 각속도 | m/s, rad/s |
| U | 전진 속도 | m/s |
| \(k_{goal}, k_{obs}\) | 퍼텐셜 gain | 계약 |
| \(A_i, \sigma_i\) | 반발 강도, 반경 | 계약 |
| \(k_{stab}\) | OrbitStabilizer 보정 gain | 계약 |
| \(\rho_0\) | 장애물 영향 반경 | m |

이 문서는 **개념·수식 고정**용이다. 구현 시 파라미터는 설정 파일 또는 캘리브레이션으로 둔다.

---

## 10. 쇄빙선 확장 (Icebreaker)

### 10.1 빙 퍼텐셜 (U_ice)

빙 농도·두께를 **비용** 또는 **반발**로 쓰면, 경로가 얇은 빙/열린 물을 선호한다.

**격자 기반** (Grid와 연동):  
격자 \((i,j)\)의 빙 농도 \(C_{ice}(i,j) \in [0,1]\) 또는 두께 \(h_{ice}(i,j) \ge 0\)에 대해

$$
U_{ice}(\xi,\eta) = k_{ice} \sum_{i,j} C_{ice}(i,j)\, g\big( \rho((\xi,\eta), (x_i,y_j))/\sigma \big)
$$

또는 단순히 **선형 비용**:

$$
U_{ice}(\xi,\eta) = k_{ice}\, C_{ice}(\xi,\eta),\qquad C_{ice} \in [0,1]
$$

- \(k_{ice} > 0\). \(C_{ice}\)는 보간된 연속장.  
- 합성 퍼텐셜: \(U = U_{goal} + \sum U_{obs,i} + U_{mem}(c) + U_{ice}\).

### 10.2 램(Ramming) 시 추진축 외란

쇄빙 시 축에 걸리는 **토크 스파이크**를 \(\tau_{env}\)에 포함:

$$
\tau_{env} \leftarrow \tau_{env} + \tau_{ram}(t),\qquad \tau_{ram}(t) = T_{ram}\, e^{-(t-t_{impact})^2/(2\sigma_{ram}^2)}
$$

- \(t_{impact}\): 충돌 시각.  
- OrbitStabilizer / Marine Propulsion은 이 \(\tau_{ram}\)을 **외란**으로 받아, 위상 예측 보정으로 축 마모·피로를 줄인다 (기존 “31.9% 마모 감소”와 동일 메커니즘).

### 10.3 맥락별 모드 (c = ice_ram / ice_transit)

- **ice_ram**: 고토크·저속 프로파일. Cerebellum 또는 룩업: \(U_{ref} = U_{ram}\), \(\tau_{ref}(t)\) = 램 사이클.
- **ice_transit**: 얇은 빙 통과 — 일반 경로 제어 + \(U_{ice}\) 반영.
- **open_water**: \(U_{ice}=0\), 기존 자율항해와 동일.

단위: \(C_{ice}\) 무차원, \(h_{ice}\) [m], \(k_{ice}\) [계약], \(\tau_{ram}\) [N·m].
