# Handoff Folder Structure

이 문서는 팀원이 정해진 위치에 파일만 넣고 Streamlit에서 선택해 실험할 수 있도록 정리한 폴더 규칙입니다.

## 기본 원칙

- 기존 샘플 `T01_flat`, `T02_slope`, `T03_single_rock`, `T04_rock_field`는 삭제하지 않습니다.
- 새 로버/지형/제어/관측 데이터는 새 ID 폴더 또는 새 YAML 파일로 추가합니다.
- 모든 수치는 SI 단위로 저장합니다.
- 실제 PyChrono 모델 코드는 `app.py`에 직접 넣지 않습니다.
- PyChrono 생성 코드는 각 handoff 폴더의 `chrono_factory.py`에 둡니다.

## 지형 담당자

새 지형은 아래 구조로 넣습니다.

```text
terrain_scenarios/<terrain_id>/
  terrain.yaml
  chrono_factory.py        # PyChrono terrain builder, 필요할 때만
  assets/
    map.py                 # 전달받은 원본 또는 보조 코드
    *.obj                  # mesh asset, 필요할 때만
    *.csv                  # heightmap/log, 필요할 때만
```

`terrain.yaml`에서 factory는 이렇게 연결합니다.

```yaml
geometry:
  source_type: code_factory
  asset_uri: assets/map.py
  factory_uri: chrono_factory.py:build_terrain
```

`chrono_factory.py`는 다음 함수를 제공해야 합니다.

```python
def build_terrain(system, terrain, material):
    ...
    return artifact
```

현재 종민님 지형은 다음 위치에 등록되어 있습니다.

```text
terrain_scenarios/jongmin_arena_v01/
  terrain.yaml
  chrono_factory.py
  assets/map.py
```

## 로버 담당자

새 로버는 아래 구조로 넣습니다.

```text
rover_models/<rover_id>/
  rover.yaml
  chrono_factory.py        # 전용 생성기가 필요할 때만
  assets/
    *.obj
    *.json
    *.yaml
```

필수 파일은 `rover.yaml`입니다. 실제 CAD나 collision asset이 아직 없으면 `model_uri`에는 전달 위치만 적고, 임의 CAD를 만들지 않습니다.

## 제어 입력 담당자

제어 profile은 파일 하나로 추가합니다.

```text
control_profiles/<profile_id>.yaml
```

예:

```yaml
profile_id: nominal_traverse
display_name: Nominal Traverse
target_speed_mps: 0.2
duration_s: 10.0
throttle: 0.5
steering_deg: 0.0
drive_mode: velocity_hold
```

## 정찰 관측 데이터 담당자

정찰로버가 실제 또는 시뮬레이션으로 먼저 주행한 결과는 아래에 넣습니다.

```text
observations/<observation_id>/
  observation.yaml
  trajectory.csv           # 권장
  summary.json             # 권장
```

`observation.yaml`의 `terrain_id`, `scout_rover_id`, `control_profile_id`가 앱에서 선택한 값과 맞아야 자동 후보로 뜹니다.

## 지형 재료와 접촉쌍

SCM/rigid 지형 재료는 아래에 넣습니다.

```text
terrain_materials/<material_id>.yaml
```

바퀴-지형 접촉값은 아래에 넣습니다.

```text
contact_pairs/<contact_pair_id>.yaml
```

`contact_pair.yaml`의 `wheel_material_id`는 선택한 로버의 `wheel_material_id`와 같아야 하고, `terrain_material_id`는 선택한 지형의 `material_id`와 같아야 합니다.

## 검증 명령

파일을 넣은 뒤 아래 명령으로 registry와 참조 관계를 확인합니다.

```bash
cd C:\K_SRC\mobility_twin_mvp
python tools\validate_handoff.py
```

특정 조합만 확인하려면:

```bash
python tools\validate_handoff.py --rover main_v01 --terrain jongmin_arena_v01 --control nominal_traverse --observation O_jongmin_arena_nominal
```

## Streamlit 확인

```bash
conda activate chrono
cd /d C:\K_SRC\mobility_twin_mvp
python -m streamlit run app.py --browser.gatherUsageStats false
```

`통합 실험` 탭에서 다음을 선택합니다.

- 예측 대상 메인로버
- 정찰로버 모델
- 지형 시나리오
- 제어 프로파일
- 정찰 관측 데이터
- 바퀴-지형 접촉값

선택 후 `실험 실행`을 누르면 `SimulationResult`가 생성됩니다.

## 주의

- 임시 mock observation은 UI 연결 확인용입니다. 물리적 결론에 쓰면 안 됩니다.
- 실제 신뢰성 평가는 정찰로버 trajectory와 메인로버 ground truth가 들어온 뒤 prediction error로 판단합니다.
- `source_type: code_factory` 지형은 PyChrono가 있는 환경에서만 실제 생성됩니다.