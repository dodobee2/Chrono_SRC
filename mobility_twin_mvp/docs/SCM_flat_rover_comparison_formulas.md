# SCM 평지 정찰-메인 로버 비교 수식 정리

정찰로버가 먼저 지형을 주행해서 얻은 데이터를 이용해, 메인로버가 같은 지형을 지나갈 때의 위험도를 예측하기 위한 수식 정리입니다.

핵심은 **정찰로버 결과를 그대로 메인로버 결과라고 주장하는 것**이 아니라, 질량/바퀴/하중 차이를 반영해서 메인로버의 slip, sinkage, drawbar pull, energy를 예측하고, 실제 PyChrono SCM 결과와 비교하는 것입니다.

모든 단위는 SI로 고정합니다.

---

## 1. 실험 목적

같은 SCM 평지에서 다음 두 로버를 비교합니다.

- 정찰로버 scout rover
- 메인로버 main rover

기본 실험 순서는 다음과 같습니다.

1. 정찰로버가 SCM 평지를 먼저 주행한다.
2. 정찰로버의 slip, sinkage, 속도, 토크, 에너지, drawbar pull을 기록한다.
3. 정찰로버 데이터와 두 로버의 물리 파라미터 차이를 이용해 메인로버 결과를 예측한다.
4. 메인로버를 같은 지형에서 실제 SCM으로 주행시킨다.
5. 예측값과 실제값의 오차를 비교한다.

이 단계의 목표는 최종 위험도 모델 완성이 아니라, **정찰로버 데이터로 메인로버 상태를 예측할 수 있는지**를 확인하는 것입니다.

---

## 2. 기본 기하와 하중

로버 전체 중량:

```text
W = m g
```

바퀴 하나당 평균 수직하중:

```text
W_wheel = W / n_wheels
```

근사 접지면적:

```text
A_contact ≈ b l_contact
```

근사 접지압:

```text
p ≈ W_wheel / A_contact
```

변수 의미:

| 기호 | 의미 | 단위 |
|---|---|---|
| m | 로버 질량 | kg |
| g | 중력가속도, 9.81 | m/s² |
| W | 로버 중량 | N |
| n_wheels | 전체 바퀴 수 | - |
| W_wheel | 바퀴 하나당 평균 수직하중 | N |
| b | 바퀴 폭 | m |
| l_contact | 접촉 길이 | m |
| A_contact | 접지면적 | m² |
| p | 접지압 | Pa |

---

## 3. 정찰로버와 메인로버의 하중 비율

정찰로버 데이터로 메인로버를 예측할 때 가장 먼저 봐야 하는 값은 바퀴당 하중 비율입니다.

```text
pressure_ratio = W_wheel_main / W_wheel_scout
```

두 로버의 질량과 바퀴 수로 쓰면:

```text
pressure_ratio = (m_main g / n_main) / (m_scout g / n_scout)
```

현재 MVP에서 쓰기 좋은 형태:

```text
pressure_ratio = (m_main g / n_main) / scout_wheel_load_N
```

주의할 점:

- 이 값은 kg/kg 비율이 아닙니다.
- 반드시 N/N, 즉 힘의 비율이어야 합니다.
- `pressure_ratio > 1`이면 메인로버가 정찰로버보다 지면을 더 강하게 누릅니다.
- 일반적으로 하중 비율이 커지면 sinkage와 slip이 증가할 가능성이 큽니다.

---

## 4. Slip 정의

바퀴 반지름이 `r`, 바퀴 각속도가 `omega`, 차체 전진 속도가 `v`일 때 slip은 다음처럼 정의할 수 있습니다.

```text
s = (r omega - v) / max(|r omega|, epsilon)
```

전진 주행이고 `r omega > 0`이면 다음 형태도 자주 사용합니다.

```text
s = 1 - v / (r omega)
```

변수 의미:

| 기호 | 의미 | 단위 |
|---|---|---|
| s | slip ratio | - |
| r | 바퀴 반지름 | m |
| omega | 바퀴 각속도 | rad/s |
| v | 차체 전진 속도 | m/s |
| epsilon | 0 나눗셈 방지용 작은 값 | - |

해석:

- `s = 0`이면 이상적인 구름에 가깝습니다.
- `s`가 커질수록 바퀴가 헛도는 비율이 커집니다.
- `s ≈ 1`에 가까우면 차체는 거의 안 가는데 바퀴만 도는 상태입니다.

현재 MVP의 단순 예측식:

```text
s_main_pred = clip(
    s_scout * sqrt(pressure_ratio) * sqrt(r_scout / r_main),
    0,
    1
)
```

이 식의 의미:

- 메인로버 바퀴당 하중이 더 크면 slip 증가
- 메인로버 바퀴 반지름이 더 크면 같은 조건에서 slip 완화 가능
- 아직 경험식이므로 SCM 결과로 보정해야 함

---

## 5. Bekker pressure-sinkage 관계

SCM에서 지반 침하 sinkage를 설명하는 대표식은 Bekker pressure-sinkage 관계입니다.

```text
p = (k_c / b + k_phi) z^n
```

이를 침하 `z`에 대해 풀면:

```text
z = [ p / (k_c / b + k_phi) ]^(1/n)
```

정찰로버와 메인로버의 침하 비율은 다음처럼 비교할 수 있습니다.

```text
z_main / z_scout
= [
    (p_main / p_scout)
    * ((k_c / b_scout + k_phi) / (k_c / b_main + k_phi))
  ]^(1/n)
```

변수 의미:

| 기호 | 의미 | 단위 |
|---|---|---|
| p | 접지압 | Pa |
| z | sinkage, 침하량 | m |
| k_c | cohesive modulus | N/m^(n+1) |
| k_phi | frictional modulus | N/m^(n+2) |
| n | sinkage exponent | - |
| b | 바퀴 폭 | m |

현재 MVP의 단순 예측식:

```text
z_main_pred = z_scout * sqrt(pressure_ratio) * sqrt(b_scout / b_main)
```

해석:

- 메인로버가 더 무거우면 침하 증가
- 메인로버 바퀴 폭이 더 넓으면 침하 감소
- 정확한 모델은 SCM soil parameter를 이용한 Bekker 식으로 교체해야 함

---

## 6. Janosi-Hanamoto 전단 모델

바퀴가 지반을 밀 때 발생하는 전단응력은 Janosi-Hanamoto 모델로 표현할 수 있습니다.

```text
tau = (c + sigma tan(phi)) * (1 - exp(-j / K))
```

접촉면 전체의 추진력은:

```text
F_x = integral_A tau dA
```

단순 근사:

```text
F_x ≈ tau_avg A_contact
```

변수 의미:

| 기호 | 의미 | 단위 |
|---|---|---|
| tau | 전단응력 | Pa |
| c | cohesion | Pa |
| sigma | 수직응력 | Pa |
| phi | internal friction angle | rad |
| j | shear displacement | m |
| K | shear deformation modulus | m |
| F_x | 추진력 | N |

이 모델이 필요한 이유:

- slip이 커진다고 무조건 전진력이 커지는 것이 아닙니다.
- 지반 전단 한계에 도달하면 바퀴가 더 헛돌 수 있습니다.
- 메인로버 위험도 예측에는 slip뿐 아니라 traction 한계도 필요합니다.

---

## 7. 평지 주행 저항

평지에서 일정 속도로 주행한다고 가정하면 요구 힘은 주로 rolling resistance와 지반 변형 저항입니다.

단순 rolling resistance:

```text
F_rr = C_rr W
```

즉:

```text
F_req ≈ C_rr m g
```

변수 의미:

| 기호 | 의미 | 단위 |
|---|---|---|
| F_rr | rolling resistance force | N |
| C_rr | rolling resistance coefficient | - |
| F_req | 필요한 주행 힘 | N |

---

## 8. 토크, 추진력, Drawbar Pull

바퀴 하나의 토크가 `tau_wheel`이면 바퀴 하나가 낼 수 있는 힘은 대략 다음과 같습니다.

```text
F_wheel ≈ tau_wheel / r
```

구동 바퀴가 `n_driven`개이면 전체 구동력은:

```text
F_drive ≈ n_driven tau_wheel / r
```

하지만 실제 사용 가능한 힘은 여러 한계 중 가장 작은 값입니다.

```text
F_avail = min(F_friction_max, F_torque_max, F_soil_shear_limit)
```

Drawbar pull은 다음처럼 볼 수 있습니다.

```text
DBP = F_traction - F_resistance
```

해석:

- `DBP > 0`: 전진 여유 있음
- `DBP ≈ 0`: 한계 상태
- `DBP < 0`: 현재 조건에서 지속 주행 어려움

메인로버 예측 위험도에서는 `DBP`가 매우 중요합니다.

---

## 9. 에너지와 Cost of Transport

모터 출력:

```text
P = sum_i |tau_i omega_i|
```

누적 에너지:

```text
E = integral P dt
```

거리당 에너지:

```text
E_per_m = E / distance
```

Cost of Transport:

```text
COT = E / (m g distance)
```

해석:

- 같은 거리에서 에너지가 많이 들수록 주행 조건이 나쁩니다.
- COT는 질량 차이를 어느 정도 보정한 효율 지표입니다.
- 이동거리가 너무 작으면 COT가 비정상적으로 커질 수 있으므로 최소 이동거리 조건이 필요합니다.

---

## 10. 저장해야 할 시계열 값

정찰로버와 메인로버 모두 같은 컬럼 이름과 SI 단위로 저장하는 것이 중요합니다.

필수 추천 컬럼:

| 컬럼명 | 의미 | 단위 |
|---|---|---|
| time_s | 시간 | s |
| x_m | x 위치 | m |
| y_m | y 위치 | m |
| z_m | z 위치 | m |
| body_speed_mps | 차체 속도 | m/s |
| wheel_angular_speed_radps | 바퀴 각속도 | rad/s |
| wheel_torque_nm | 바퀴 토크 | N m |
| slip | slip ratio | - |
| sinkage_m | 침하량 | m |
| drawbar_pull_n | drawbar pull | N |
| normal_load_per_wheel_n | 바퀴당 수직하중 | N |
| contact_area_m2 | 접지면적 | m² |
| motor_power_w | 모터 출력 | W |
| energy_j | 누적 에너지 | J |
| cot | cost of transport | - |

---

## 11. 저장해야 할 요약값

각 실험 run마다 다음 summary 값을 저장하면 비교가 쉬워집니다.

| 요약값 | 의미 |
|---|---|
| mean_slip | 평균 slip |
| max_slip | 최대 slip |
| mean_sinkage_m | 평균 침하 |
| max_sinkage_m | 최대 침하 |
| mean_drawbar_pull_n | 평균 drawbar pull |
| mean_wheel_torque_nm | 평균 바퀴 토크 |
| peak_wheel_torque_nm | 최대 바퀴 토크 |
| travel_distance_m | 이동거리 |
| mean_speed_mps | 평균 속도 |
| energy_j | 총 에너지 |
| cot | cost of transport |
| completed | 주행 완료 여부 |

---

## 12. 비교할 예측 모델

처음부터 복잡한 모델 하나만 쓰기보다 baseline을 나눠 비교하는 것이 좋습니다.

### Baseline A: Identity transfer

정찰로버 값을 메인로버 예측값으로 그대로 사용합니다.

```text
s_main_pred = s_scout
z_main_pred = z_scout
```

의미:

- 가장 단순한 기준선입니다.
- 이 기준선보다 좋아야 모델링의 의미가 있습니다.

### Baseline B: MVP pressure scaling

현재 MVP에서 바로 쓸 수 있는 하중 기반 보정입니다.

```text
s_main_pred = s_scout * sqrt(pressure_ratio) * sqrt(r_scout / r_main)
```

```text
z_main_pred = z_scout * sqrt(pressure_ratio) * sqrt(b_scout / b_main)
```

의미:

- 메인로버가 더 무겁고 바퀴가 불리하면 slip/sinkage 증가
- 바퀴가 크거나 넓으면 완화
- 단, 아직 보정 전 경험식입니다.

### Baseline C: Bekker-based sinkage scaling

SCM soil parameter가 확보되면 Bekker 식으로 침하를 예측합니다.

```text
z = [ p / (k_c / b + k_phi) ]^(1/n)
```

의미:

- SCM 지반 파라미터를 직접 반영합니다.
- 평지 실험에서 가장 먼저 검증하기 좋은 물리 기반 모델입니다.

---

## 13. 예측 오차 계산

메인로버 실제 SCM 결과가 나오면 다음 오차를 계산합니다.

```text
error_slip = |s_main_pred - s_main_actual|
```

```text
error_sinkage = |z_main_pred - z_main_actual|
```

```text
error_energy = |E_main_pred - E_main_actual|
```

상대오차:

```text
relative_error = |pred - actual| / max(|actual|, epsilon)
```

여러 run의 평균 오차:

```text
MAE = mean(|pred_i - actual_i|)
```

```text
RMSE = sqrt(mean((pred_i - actual_i)^2))
```

---

## 14. 위험도 계산 방향

현재 MVP의 위험도는 아직 calibration된 값이 아닙니다.

초기 위험도는 다음 요소의 weighted sum으로 볼 수 있습니다.

```text
risk = w_s slip_risk
     + w_z sinkage_risk
     + w_d dbp_risk
     + w_e energy_risk
```

예시:

```text
slip_risk = clip((mean_slip - slip_safe) / (slip_fail - slip_safe), 0, 1)
```

```text
sinkage_risk = clip((mean_sinkage - z_safe) / (z_fail - z_safe), 0, 1)
```

```text
dbp_risk = clip((DBP_safe - mean_DBP) / (DBP_safe - DBP_fail), 0, 1)
```

중요:

- threshold는 임의로 확정하면 안 됩니다.
- 먼저 SCM 평지 결과로 예측 오차를 확인합니다.
- 이후 slope, single rock, rock field로 확장하면서 threshold를 보정합니다.

---

## 15. 팀원에게 요청할 파일

SCM 평지 비교를 위해 팀원에게 요청할 handoff 파일은 다음과 같습니다.

### rover parameter 파일

정찰로버와 메인로버 각각 필요합니다.

```yaml
rover_id: scout_v01
mass_kg: 1.0
wheel_radius_m: 0.05
wheel_width_m: 0.03
wheel_count: 4
driven_wheel_count: 4
max_wheel_torque_nm: 0.2
```

```yaml
rover_id: main_v01
mass_kg: 5.0
wheel_radius_m: 0.10
wheel_width_m: 0.06
wheel_count: 4
driven_wheel_count: 4
max_wheel_torque_nm: 1.0
```

### SCM soil parameter 파일

```yaml
terrain_id: scm_flat_v01
model: SCM
k_c: null
k_phi: null
n: null
cohesion_pa: null
friction_angle_rad: null
janosi_shear_m: null
```

### trajectory.csv

정찰로버와 메인로버 모두 같은 스키마로 저장합니다.

```text
time_s,x_m,y_m,z_m,body_speed_mps,wheel_angular_speed_radps,wheel_torque_nm,slip,sinkage_m,drawbar_pull_n,normal_load_per_wheel_n,contact_area_m2,motor_power_w,energy_j,cot
```

### summary.json

```json
{
  "rover_id": "scout_v01",
  "terrain_id": "scm_flat_v01",
  "mean_slip": 0.0,
  "max_slip": 0.0,
  "mean_sinkage_m": 0.0,
  "max_sinkage_m": 0.0,
  "mean_drawbar_pull_n": 0.0,
  "travel_distance_m": 0.0,
  "mean_speed_mps": 0.0,
  "energy_j": 0.0,
  "cot": 0.0,
  "completed": true
}
```

---

## 16. 팀원에게 보낼 짧은 요청 문구

아래 문구를 그대로 보내도 됩니다.

> SCM 평지에서 정찰로버와 메인로버를 같은 제어 입력으로 각각 주행시켜 비교하려고 합니다.  
> 우선 목표는 최종 위험도 계산이 아니라, 정찰로버 데이터로 메인로버의 slip/sinkage/energy/drawbar pull을 예측할 수 있는지 확인하는 것입니다.  
> 각 로버의 mass, wheel radius, wheel width, wheel count, driven wheel count, max torque를 yaml로 공유해주세요.  
> 지형은 SCM flat으로 두고 k_c, k_phi, n, cohesion, friction angle, Janosi shear parameter를 공유해주세요.  
> 결과는 trajectory.csv와 summary.json으로 저장하고, 모든 단위는 SI로 맞춰주세요.

---

## 17. 지금 단계에서 하지 말아야 할 것

- 최종 위험도 threshold를 임의로 확정하지 않기
- 정찰로버 결과를 보정 없이 메인로버 결과라고 주장하지 않기
- SCM soil parameter를 임의로 추정해서 결론 내리지 않기
- slope, rock field부터 시작하지 않기
- 먼저 평지에서 scout-to-main prediction error를 확인하기

---

## 18. 결론

평지 SCM 실험에서 가장 먼저 확인해야 할 질문은 다음입니다.

```text
정찰로버에서 관측한 slip/sinkage/energy를
메인로버의 하중, 바퀴 반지름, 바퀴 폭 차이로 보정했을 때
실제 메인로버 SCM 결과에 가까워지는가?
```

이 질문에 대해 identity transfer보다 좋은 결과가 나오면, MVP의 핵심 연구질문인 **정찰로버 데이터 기반 메인로버 위험 예측**의 가능성을 보여줄 수 있습니다.