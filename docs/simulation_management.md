# 표준 시뮬레이션 관리

실행 진입점은 `bash scripts/open_simulation.command`이며 Python에서는 `python -m scripts.sim_cli`다. 기본 창·설정 실행, 공동 출하, 통신 비교, 주행, ACT·교사·학습과 평가 도구를 같은 카탈로그에서 선택하고 같은 실행 기록으로 추적한다. 실행 대상과 예산은 현재 프로젝트 지침과 해당 workflow의 요구사항을 따른다.

Colab에서도 같은 소스·CLI를 사용하는 [L4 적용 검토](colab_standard_simulation_review_20260923.md)를 참고한다. 현재 전송기의 CPU 렌더링 강제 설정과 GPU 학습 경로는 별도 연결이 필요하며 L4 실제 실행 완료를 뜻하지 않는다.

## 한곳에서 선택하고 실행하기

```bash
# 등록된 연구 실행과 요구사항
bash scripts/open_simulation.command workflow list

# 실제 실행 없이 명령·입력·출력·버전 확인
bash scripts/open_simulation.command workflow plan stage-sync

# 외부 모델/물리 실행 없는 동기화 fixture
bash scripts/open_simulation.command workflow run stage-sync

# 기존 설정 실행도 같은 공통 관리 기록에 연결
bash scripts/open_simulation.command run configs/simulation/drive.json --headless

# 관리된 실행 목록과 특정 실행 기록
bash scripts/open_simulation.command workflow runs
bash scripts/open_simulation.command workflow show RUN_ID
```

`workflow plan/run <ID> -- <기존 인자>` 형식으로 연구별 옵션을 전달한다. plan은 모델·훈련·시뮬레이션을 시작하지 않는다. 외부 모델, 원격 제출, 교사 데이터와 학습은 해당 workflow와 인자를 명시해 선택한다. 카탈로그에 표시된 실행 요구사항과 연구별 문서를 확인한다. 서버·하드웨어 및 기록 분석 도구의 실행 완료는 시뮬레이션 성공 판정이 아니다.

## 지도·학습·실행 결과를 잇는 절차

사람이 바로 장면을 보거나 지원된 출하장 임무에 자연어 지시를 주려면 [터미널 간편 메뉴](local_simulation.md#터미널-간편-메뉴)를 쓸 수 있다. 실행 방식·맵·관찰 속도·모델은 번호로 고르고 자연어 지시만 텍스트로 입력한다. 기존 `run`/`dispatch`를 호출하므로 아래 공통 기록과 검증 경계는 같다. ACT 지도 미리보기와 LLM 출하 실행의 지원 맵은 별개다.

먼저 [로컬 첫 실행](local_simulation.md#설치와-첫-실행)의 모델 없는 명령으로 MuJoCo 기본 창과 로봇 RGB 저장을 확인한다. 연구용 맵은 아래처럼 기존 suite를 준비한다. `plan`은 실제 작업을 시작하지 않으며, `run`은 새 출력 폴더와 공통 manifest를 만든다. `--render`는 정적 장면·카메라 미리보기이며 창을 통한 자율 제어가 아니다.

개별 지도는 `bash scripts/open_simulation.command run configs/simulation/local.json --scene act/train-open-1 --paused --capture`로 MuJoCo 기본 창에서 바로 볼 수 있다. 이것도 정적 장면 확인이며 ACT 정책 실행은 아니다.

```bash
bash scripts/open_simulation.command workflow plan act-map-suite -- \
  --spec maps/act_generalization/suite_v1.json --render
bash scripts/open_simulation.command workflow run act-map-suite -- \
  --spec maps/act_generalization/suite_v1.json --render
bash scripts/open_simulation.command workflow runs
bash scripts/open_simulation.command workflow show RUN_ID
```

`show`의 `output` 아래 `summary.json`과 `manifest.json`에서 생성된 지도·분할·기하 검사·렌더 오류를 확인한다. `geometric_candidate`나 카메라 미리보기는 교사 주행, 학생 운반, 새 지도 일반화의 성공 판정이 아니다. 지도 출력은 ACT 학습 데이터셋이 아니다. 교사 성공 시연과 학생 입력을 별도로 생성·검증하고 train/dev/test 맵 분할을 고정한다. 시험 맵은 학습이나 체크포인트 선택에 사용하지 않은 뒤에만 다음 학습을 선택한다. 지도 준비 계약과 범위는 [ACT 맵 안내](act_map_suite.md)를 따른다.

학습은 검증된 데이터셋 경로, 이미지 크기, 기억 길이, 장치와 실행 예산을 명시한다. 아래 `DATASET_JSON`은 이미 준비·검증한 파일로 바꾼다. 이 명령은 읽기 전용 계획 확인이다. 자원·종료 시점을 정하고 소스를 커밋해 고정한 뒤에만 같은 인자로 `workflow run act-input-training -- ...`을 실행한다.

```bash
bash scripts/open_simulation.command workflow plan act-input-training -- \
  --dataset DATASET_JSON --size 256 --history 4 --steps 8000 --seed 20260921 --device cpu
```

학습 실행이 완료되면 `workflow show RUN_ID`의 `output`에 있는 `report.json`에서 완료 여부·개발 지표·선택된 체크포인트를 확인한다. 학습 완료나 개발 점수는 새 조건에서 학생의 독립 실행 성공이 아니다. 별도 평가 workflow를 같은 소스·모델·지도·물리·입력 조건으로 실행해 결과와 실패를 기록한다. 최종 실행의 `manifest.json`에서 `output`, 소스/입력 해시, 종료 상태를 확인하고 해당 출력의 `result.json`과 평가 파일을 읽는다. `exit_code=0`, `protocol_complete`, `physical_success`는 뜻이 다르며 실제 성공 주장은 평가 근거에 한정한다.

완료한 **실행·학습** 결과는 [TensorBoard 안내](tensorboard.md)에 따라 새 스냅샷으로 변환한다. 다음 예시는 `workflow show`의 `output`으로 확인한 완료 실행 폴더를 사용한다. worktree에서 실행해도 변환 결과는 기본 체크아웃의 공유 `outputs/tensorboard`에 둔다. `REVIEW_ID`는 새 이름으로 바꾸고, 여러 완료 실행은 `--source`를 반복한다. 지도 준비의 `summary.json`만 있는 폴더는 현재 변환기가 지원하는 실행·학습 원본이 아니므로 원본에서 검토한다.

```bash
run_output=/absolute/path/from/workflow-show/output
primary_root="$(dirname "$(git rev-parse --path-format=absolute --git-common-dir)")"
review_dir="$primary_root/outputs/tensorboard/REVIEW_ID"
bash scripts/open_simulation.command workflow run tensorboard -- \
  --source "$run_output" --output "$review_dir"
```

변환 전에 `requirements-observability.txt`의 선택 의존성을 현재 시뮬레이션 Python 환경에 설치한다. 화면을 열기 전 포트 6006/6007의 기존 서버 PID·명령·TensorBoard logdir와 소유 작업을 확인한다. 기존 logdir에 새 스냅샷이 포함되면 그 서버를 새로고침한다. 자신의 서버가 필요하고 두 포트가 비어 있으면 `UGRP_TENSORBOARD_LOGDIR="$review_dir" bash scripts/open_tensorboard.command`로 실행한다. 다른 작업의 서버는 중지하지 않는다. TensorBoard 화면에서 실제 run과 값이 로드되는지 확인한다. 기본 체크아웃의 로컬 `outputs/tensorboard-view.json`이 있으면 그 설정의 HParams 열·Time Series 카드·원본 영상 링크를 적용한다. 새 clone에 이 파일이 없으면 현재 결과의 관련 지표를 직접 선택한다. 원본의 해시와 스냅샷 manifest를 대조하고 실행 중이거나 회수되지 않은 원격 결과를 완료로 표시하지 않는다. 화면과 영상은 공식 TensorBoard 및 원본 MP4 링크로 확인하며 별도 대시보드는 사용하지 않는다. 결과·자료는 로컬 프로젝트에 남기고 Google Drive에 조회·업로드·동기화하지 않는다.

## 관리하는 것

| 공통 기록 | 용도 |
|---|---|
| workflow ID·버전·카탈로그 해시 | 어떤 연구 실행을 선택했는지 식별 |
| Git SHA·dirty 상태·실제 소스 해시 | 같은 커밋에서도 로컬 수정 여부 구분 |
| 실행 인자·설정·입력 파일 해시 | 지도·모델·계획·입력 변경 추적 |
| `inputs_changed_during_run` | 설정이 참조한 지도·확장 파일이나 명시 입력이 실행 중 바뀌었는지 확인 |
| Python·패키지 환경 | 소스가 같아도 환경이 다른 실행 구분 |
| 시작·종료·중단·시간 제한·exit code | 실패와 중단을 포함한 실행 수명 기록 |
| 결과 위치·파일 해시 | 원본 결과와 실행 조건 연결 |

기존 실행기가 저장하는 장면 XML·실제 물리 설정·모델 요청·영상·평가 결과는 그대로 보존한다. 공통 기록은 이를 감싼 실행 이력이다. `exit code 0`은 프로세스 종료 상태이며 물리적 운반 성공은 각 실행의 평가 자료로 확인한다. 실제 실험은 소스를 먼저 커밋하고 고정하며, 원본과 과거 실행 폴더를 덮어쓰지 않는다. 로컬 해시는 원격 백업을 뜻하지 않는다.

## 하나의 관리 체계와 여러 실험 프로필

표준 설정 실행과 기존 공동 출하·공통 RGB adapter는 `sim.session_scenes.Scene`의 출하장 XML 생성·초기화를 공유한다. 기존 출하장 실행기의 전역 XML 생성기 교체를 제거하고 세계 인스턴스마다 같은 장면 정의를 적용한다. 카메라 배치/FOV, 초기 servo 명령과 안정화 시간, weld OFF를 보존한다.

접촉 설정과 제어기까지 무조건 하나의 기본값으로 합치지는 않는다. 과거 `local_contact_fine` 성공과 `legacy` adapter 진단은 서로 다른 실험 조건이다. 실행 시 선택한 프로필과 실제 적용값을 기록하고 [실행 번들 버전](execution_versioning.md)으로 검증한다. 현재 표준 장면 연결 후보는 `rgb-standard-dispatch-v5`; 표준 dispatch의 촬영 최적화·독립 경로 자동 선택을 새 버전으로 기록한다. 기존 v1/v2/v3/v4 번들 JSON은 변경하지 않고 각 원래 소스로 재현한다.

나머지 연구 실행기는 같은 관리 계층에 등록된 호환 어댑터다. 전체 제어 루프를 `Simulation.step()`으로 재작성했다는 의미는 아니다. 기존 스크립트 직접 호출과 Python API 직접 사용도 호환을 위해 남아 있지만 그 자체로 공통 실행 기록이 생성되지는 않는다. 신규 실험과 신규 workflow는 표준 관리 진입점에 연결하며 독립적인 버전·결과 관리 체계를 추가하지 않는다.

## 새 실행 경로 추가

`configs/simulation_workflows.json`에 ID·버전·진입점·출력 규칙·실행 요구사항을 등록하고 관련 테스트를 추가한다. 공통 장면·초기화 구현을 재사용하며, 다른 물리·카메라·명령 조건이 필요하면 명시적 프로필/버전으로 추가한다. 기존 성공 조건의 변경 전후 차이와 검증 범위를 기록한다. 카탈로그 등록이나 자동 검사 통과는 연구 과제 완주 검증을 대신하지 않는다.
