# Mobility Twin MVP 인수인계

대상: 종민·호진 팀원  
기준일: 2026-07-20  
개발 폴더: `C:\K_SRC\mobility_twin_mvp`

## 1. 프로젝트 한 줄 요약

정찰(Scout) 로버가 지형을 주행하며 얻은 관측값을 이용해 메인(Main) 로버의 주행 가능성과 위험도를 예측하고, 이를 **Main-Rover Mobility Risk Map(로버별 주행성 지도)** 으로 보여 주는 Streamlit 기반 분석 도구입니다.

현재는 다음 세 단계를 한 저장소에서 다룹니다.

1. CSV와 수식 기반의 기존 휴리스틱 위험도 지도
2. 로버·지형·관측·재료·제어 파일을 연결하는 Integration Contract v2
3. PyChrono 기반 실제 물리 시뮬레이션으로 교체하기 위한 로버/지형 factory와 실험 파일럿

> 중요: 현재 앱에서 표시하는 휴리스틱 위험도는 개념 검증용입니다. 실제 정찰 로그와 메인 로버 ground truth로 보정된 최종 안전 판정값이 아닙니다.

## 2. 폴더 역할과 원본 파일 위치

```text
C:\K_SRC\
├─ README.md                   # 이 인수인계 문서
├─ handoff\                    # 팀원이 전달한 원본/기준 자료(가급적 직접 수정 금지)
│  ├─ map.py                  # 종민 지형 원본: 5-zone Chrono arena
│  └─ rover_module_v01\       # 호진 로버 원본
│     ├─ specs\               # scout_v01, main_v01 원본 사양
│     ├─ code\                # 로버 builder, schema, 검증/렌더 스크립트
│     ├─ results\             # 원본 검증 결과
│     ├─ images\              # 로버/검증 이미지
│     └─ 01~04 문서           # 소개, 설계 이유, 검증 결과, 용어
└─ mobility_twin_mvp\          # 실제 앱 및 프로그램 개발 폴더
```

원본을 앱에 직접 import하는 방식이 아니라, 개발 폴더 안에 등록 가능한 형태로 옮겨 사용합니다.

- 종민 지형 등록본: `mobility_twin_mvp\terrain_scenarios\jongmin_arena_v01\`
- 종민 지형 보존 사본: `...\assets\map.py`
- 종민 지형 adapter: `...\chrono_factory.py`
- 호진 로버 등록 사양: `mobility_twin_mvp\rover_models\scout_v01\rover.yaml`, `main_v01\rover.yaml`
- 호진 builder 이식본: `mobility_twin_mvp\src\chrono\vendor\rover_module_v01\`

`handoff`는 원본과 근거를 확인하는 곳, `mobility_twin_mvp`는 통합·수정·실행하는 곳으로 생각하면 됩니다. 원본이 갱신되면 복사본과 adapter를 갱신하고 테스트해야 합니다.

## 3. 실행 방법

### 3.1 로그인(사인인)

현재 앱에는 계정, 로그인, 권한 관리 기능이 없습니다. 별도 사인인 없이 로컬 PC에서 실행한 뒤 브라우저의 Streamlit 주소(보통 `http://localhost:8501`)로 접속합니다.

`conda activate chrono`는 로그인 명령이 아니라 PyChrono가 설치된 Python 환경을 활성화하는 명령입니다.

### 3.2 일반 앱 실행(Chrono 없이)

PowerShell에서:

```powershell
cd C:\K_SRC\mobility_twin_mvp
python -m pip install -r requirements.txt
python -m streamlit run app.py --browser.gatherUsageStats false
```

휴리스틱 분석, mock backend, YAML registry, 일반 테스트는 이 환경으로 실행할 수 있습니다.

### 3.3 PyChrono 기능을 포함한 실행

현재 확인된 환경 이름은 `chrono`입니다.

```powershell
conda activate chrono
cd C:\K_SRC\mobility_twin_mvp
python -c "import pychrono; print(pychrono.__file__)"
python -m streamlit run app.py --browser.gatherUsageStats false
```

PyChrono는 pip 패키지로 추가하지 말고 `projectchrono` conda 채널을 사용해야 합니다. 새 환경이 꼭 필요한 경우에만 아래처럼 생성합니다(용량이 수 GB일 수 있으므로 C: 여유 공간 확인).

```powershell
conda create -n chrono-mvp -c projectchrono -c conda-forge python=3.12 pychrono streamlit pandas numpy matplotlib pytest pyyaml
conda activate chrono-mvp
```

자세한 환경 장애 기록은 `mobility_twin_mvp\docs\ENVIRONMENT_SETUP.md`를 참고하십시오.

### 3.4 기본 검증

```powershell
cd C:\K_SRC\mobility_twin_mvp
python tools\validate_handoff.py
python -m pytest -q -m "not pychrono"
python -m compileall src
```

2026-07-20 현재 일반 테스트 결과는 **47 passed, 9 deselected**입니다. 제외된 9개는 실제 PyChrono 실행 마커가 붙은 테스트입니다.

## 4. 앱 사용 흐름

앱은 크게 `통합 실험`과 `기존 휴리스틱 위험도 지도` 탭으로 구성됩니다.

통합 실험의 데이터 흐름은 다음과 같습니다.

```text
Main RoverSpec + Scout RoverSpec
TerrainScenario + TerrainMaterialSpec + ContactPairSpec
ScoutObservation + ControlProfile
                  ↓
            MobilityBackend
                  ↓
           SimulationResult
                  ↓
예측 지표 / 주행 궤적 / 로버별 주행 위험도 시각화
```

통합 실험 사용 순서:

1. 메인 로버, 정찰 로버, 지형, 제어 프로파일을 선택합니다.
2. 조건과 일치하는 정찰 관측 및 바퀴-지형 접촉쌍을 선택합니다.
3. `heuristic` 또는 `mock_chrono` 계산 방식을 선택합니다.
4. 실험 실행 후 예측값, 지형 평면도, 경로, 저장된 `SimulationResult`를 확인합니다.
5. PyChrono 설치 확인은 box-drop smoke 버튼으로 별도 실행합니다. 이 결과는 로버 위험 판정에 포함되지 않습니다.

backend 의미:

- `heuristic`: 정찰 관측과 단순 물리/경험식을 이용해 위험도를 계산합니다.
- `mock_chrono`: UI와 데이터 계약 연결 확인용입니다. 물리를 계산하지 않으며 결과는 `NOT_EVALUATED`입니다.
- `pychrono_smoke`: 실제 PyChrono core로 상자 낙하/접촉만 확인합니다. 로버 주행 해석이 아닙니다.

기존 휴리스틱 탭은 `data\sample_patches.csv`의 격자별 경사, 거칠기, 장애물, slip, sinkage 등을 편집해 위험도 지도를 빠르게 확인하는 데 사용합니다.

## 5. 개발 폴더 구조

```text
mobility_twin_mvp\
├─ app.py                         # Streamlit UI와 실행 진입점
├─ requirements.txt               # 일반 Python 의존성(PyChrono 제외)
├─ src\
│  ├─ schemas.py                 # 통합 데이터 계약
│  ├─ registries.py              # 폴더/YAML 검색 및 로딩
│  ├─ backends.py                # heuristic/mock/smoke backend 선택
│  ├─ mobility_physics.py        # 견인력·전복·장애물 관련 계산
│  ├─ risk_fusion.py             # 위험도 결합
│  ├─ terrain_classifier.py      # 지형 분류
│  ├─ chrono\                    # Chrono system/rover/terrain/result 계층
│  └─ experiments\              # SCM 및 rigid transfer 파일럿
├─ rover_models\                 # 로버별 rover.yaml
├─ terrain_scenarios\            # 지형별 terrain.yaml/factory/assets
├─ terrain_materials\            # 지형 재료/SCM 가정값
├─ contact_pairs\                # 바퀴-지형 유효 접촉값
├─ observations\                 # 정찰 관측 YAML(향후 trajectory/log 포함)
├─ control_profiles\             # 속도·시간·조향 제어 조건
├─ data\                         # 입력 샘플 및 실행 결과
├─ scripts\                      # 파일럿/실시간 Chrono 실행
├─ tools\                        # handoff 검증 및 진단
├─ tests\                        # 일반/Chrono 테스트
└─ docs\                         # 환경, 구조, 공식, 실험 계획 상세 문서
```

새 모델을 추가할 때는 `app.py`에 하드코딩하지 말고 각 폴더의 `_template`을 복사해 YAML을 등록합니다. 등록 후 `python tools\validate_handoff.py`로 ID와 참조 관계를 검사합니다.

## 6. 현재까지 구현된 내용

### 완료·동작 확인

- Streamlit 기반 통합 실험 UI 및 기존 격자 위험도 지도
- Rover/Terrain/Observation/Material/Contact/Control/Result schema와 registry
- 휴리스틱 backend, mock backend, PyChrono core box-drop smoke backend
- 호진 `scout_v01`/`main_v01` 사양과 4륜 rover builder 통합
- 로버 mass, CG, wheel metadata 변환 및 테스트
- rigid flat terrain과 SCM terrain factory 골격 및 단위 테스트
- 종민 5-zone arena를 선택 가능한 `jongmin_arena_v01`로 등록
- 종민 arena의 rigid/mesh 구간을 headless factory로 연결하는 adapter
- Scout-to-Main 예측 미리보기와 로버/지형 시각화 진입점
- rigid terrain transfer pilot 구현
- rigid pilot 7개 조건(flat, 5도 경사, 마찰 3단계, 장애물 2단계) end-to-end 실행 기록
- Chrono 프로세스 hang에 대비한 조건별 subprocess, timeout, retry

최근 검증된 rigid pilot에서는 7개 조건이 rollover 없이 완료되었고 평균 slip MAE가 약 0.008이었습니다. 다만 동일 설정과 소수 조건에 대한 결과이므로 일반화된 성능 검증으로 보고하면 안 됩니다.

### 부분 구현

- 종민 arena: 등록 및 일부 rigid/mesh 생성은 연결됐지만 SCM particle zone은 `pychrono.vehicle` 문제로 완전 실행되지 않습니다.
- SCM pilot: scenario, runner, predictor, evaluator와 테스트 골격은 있으나 end-to-end는 환경 문제로 막혀 있습니다.
- 앱의 Chrono 관련 버튼/뷰어: 실행 진입점은 있으나 `pychrono.irrlicht`가 현재 환경에서 로드되지 않아 사용이 제한됩니다.
- 실시간 rigid 로버 주행: 실험용 경로가 존재하지만 최종 제품 backend 및 안정적인 분석 흐름으로 통합되지는 않았습니다.

## 7. 알려진 문제와 보완 우선순위

### P0 — 개발 재개 전에 해결

1. **PyChrono native 환경 복구**  
   현재 `pychrono` core는 동작하지만 `pychrono.vehicle`과 `pychrono.irrlicht`는 DLL 초기화 오류가 납니다. SCM과 native viewer가 이 문제로 막혀 있습니다. 재설치 전 디스크 공간을 확인하고, 기존 환경 백업/재현 정보를 남겨야 합니다.

2. **간헐적 native loading hang**  
   Chrono core만 사용하는 새 프로세스도 약 3~4회 중 1회 60초 이상 멈춘 기록이 있습니다. 자동 실행에서는 반드시 subprocess + timeout + retry를 유지하십시오. 백신 실시간 검사 영향 가능성이 있으나 확정되지 않았습니다.

3. **한글 인코딩 정리**  
   현재 `app.py`와 일부 Markdown의 한글 문자열이 환경에 따라 mojibake(깨진 글자)로 보입니다. 파일 인코딩을 UTF-8로 통일하고 Streamlit 실제 화면을 확인해야 합니다. 로직 수정과 인코딩 일괄 변환을 한 커밋에 섞지 않는 편이 안전합니다.

4. **작업 트리 정리**  
   현재 `mobility_twin_mvp`에는 수정/신규 파일과 대용량 실행 결과가 아직 정리되지 않은 상태입니다. 팀 공유 전에 소스·문서·필수 소형 fixture만 커밋하고, 재생성 가능한 `data\rigid_transfer_pilot` 결과와 `output` 정책을 정해 `.gitignore`를 보완해야 합니다. 기존 변경을 임의로 reset하지 마십시오.

### P1 — 실제 분석 도구로 만들기 위한 핵심 과제

1. `ControlProfile`을 Chrono motor/steering 입력으로 변환하는 `control_adapter` 구현
2. pose, 속도, slip, sinkage, wheel torque, contact, energy, stall/rollover를 표준 `SimulationResult`로 만드는 `result_extractor` 구현
3. 실제 rover/terrain을 실행하는 `pychrono_physics` backend를 `make_backend()`에 등록
4. 종민 arena 전 구간(rock, uneven, gate, slope, SCM)을 하나의 안정적인 scenario로 실행
5. 실제 정찰 로버 trajectory와 센서 로그 수집 및 `ScoutObservation` 변환
6. 같은 조건의 메인 로버 ground truth 수집
7. 가정값으로 표시된 토양/마찰/contact parameter를 측정값 또는 근거 있는 calibration 값으로 교체
8. train/calibration 조건과 validation 조건을 분리해 예측식 MAE, rank correlation, 실패 탐지율 평가

### P2 — 제품화 과제

- 실행 큐, 진행률, 취소, timeout/retry 상태를 앱에서 명확히 표시
- 실험 ID와 입력 YAML snapshot을 결과에 함께 저장해 재현성 확보
- CSV/JSON/PDF 보고서 내보내기
- 여러 run 비교, 조건 필터, 지도 overlay 및 이력 관리
- 사용자 로그인/권한 관리가 필요하면 별도 요구사항으로 설계(현재 미구현·불필요)
- CI에서 일반 테스트와 Chrono 전용 테스트를 분리

## 8. 팀원별 권장 인수인계 작업

### 종민 — 지형

- `handoff\map.py`가 지형 원본입니다.
- 앱 등록본은 `terrain_scenarios\jongmin_arena_v01`입니다.
- 좌표계는 `X forward, Y left, Z up`, arena 진행은 `-X → +X`로 기록돼 있습니다.
- zone별 시작/끝 좌표, mesh 생성 규칙, collision 재료, seed를 문서화해 주십시오.
- SCM zone의 필수 soil parameter와 기대 지형 변형량을 확정해 주십시오.
- 원본 변경 시 `assets\map.py`, `terrain.yaml`, `chrono_factory.py`의 차이를 함께 검토하십시오.

### 호진 — 로버

- `handoff\rover_module_v01`이 로버 원본 및 검증 근거입니다.
- 앱 등록 사양은 `rover_models\scout_v01`, `rover_models\main_v01`입니다.
- 앱 builder 이식본은 `src\chrono\vendor\rover_module_v01`입니다.
- mass/CG/wheel radius·width/wheelbase/track/torque 및 최대 wheel speed를 원본과 대조하십시오.
- CAD/collision mesh가 생기면 `model_uri`와 실제 단위·좌표계·원점을 명시하십시오.
- 4륜만 지원하는 현재 builder의 범위와 suspension/steering 모델 필요 여부를 확정하십시오.

### 공동

- 동일 terrain/control에서 Scout → 예측 → Main ground truth 순서로 데이터를 생성하십시오.
- 성공 조건(completed), 전복 각도, stall, slip 정의, sinkage 기준점을 실험 전에 합의하십시오.
- assumed parameter 결과는 실측 결과처럼 보고하지 마십시오.

## 9. 주요 명령 모음

```powershell
# handoff 참조 검증
cd C:\K_SRC\mobility_twin_mvp
python tools\validate_handoff.py

# 특정 조합만 검증
python tools\validate_handoff.py --rover main_v01 --terrain jongmin_arena_v01 --control nominal_traverse --observation O_jongmin_arena_nominal

# 일반 테스트
python -m pytest -q -m "not pychrono"

# rigid Scout-to-Main 파일럿(PyChrono core 필요)
python scripts\run_rigid_transfer_pilot.py

# SCM 파일럿(pychrono.vehicle 복구 후)
python scripts\run_scm_pilot.py --slope flat --soil loose

# 앱
python -m streamlit run app.py --browser.gatherUsageStats false
```

## 10. 결과 해석 시 주의

- `mock_chrono`와 `pychrono_smoke`는 `NOT_EVALUATED`이며 Safe/Caution/Risk 통계에 넣지 않습니다.
- `pychrono_smoke` 성공은 PyChrono core와 접촉 검출 확인일 뿐 로버 모델 검증이 아닙니다.
- 휴리스틱의 slip/sinkage scaling과 최종 risk weight는 calibration 전입니다.
- `O_jongmin_arena_nominal` 등 mock/가정 관측을 실제 센서 관측으로 오해하지 마십시오.
- 10도·15도 rigid slope 조건은 반복 전복 원인이 아직 규명되지 않아 기본 조건에서 제외돼 있습니다.
- 결과를 공유할 때 입력 rover/terrain/control/observation ID, commit, seed, PyChrono 버전을 반드시 함께 남기십시오.

## 11. 먼저 읽을 문서

1. `mobility_twin_mvp\README.md` — 기존 구현 상세와 공식
2. `mobility_twin_mvp\docs\ENVIRONMENT_SETUP.md` — PyChrono 설치/장애 기록
3. `mobility_twin_mvp\docs\HANDOFF_STRUCTURE.md` — 모델 추가 규칙
4. `mobility_twin_mvp\docs\HANDOFF_CHECKLIST.md` — 팀별 전달 체크리스트
5. `mobility_twin_mvp\docs\SCOUT_MAIN_SCM_PILOT_PLAN.md` — SCM 파일럿 계획과 차단 사유
6. `mobility_twin_mvp\docs\SCM_flat_rover_comparison_formulas.md` — Scout/Main 비교 공식

인수인계의 핵심은 **원본은 `handoff`, 개발과 실행은 `mobility_twin_mvp`, 실측 전 결과는 MVP/가정값으로 명확히 표시**하는 것입니다.
