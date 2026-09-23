# 표준 시뮬레이션 지도·로컬 실행 검증 — 2026-09-23

기존 `scripts/open_simulation.command`의 관리 카탈로그에 ACT 지도 묶음 생성·검사와 입력 이력 ACT 학습을 연결했다. `run --scene`으로 같은 표준 장면을 MuJoCo 기본 창에서 바로 열 수 있다. 관리 실행은 정적 지도·확장 설정·동작 플러그인·제어기 파일의 경로와 해시를 실행 전후에 기록한다. 사용 순서는 [표준 관리 문서](../../docs/simulation_management.md)와 [로컬 실행 문서](../../docs/local_simulation.md)에 있다.

## 실행 소스와 환경

- 실제 두 실행의 소스: `029709de2ba768a8e26b8e68da21bb97d25859a4` (깨끗한 작업 트리). Python 3.12.13, MuJoCo 3.12.0, macOS 27.2 arm64.
- 소스·입력 변경 감지: 두 실행 모두 `false`. 관리자 상태 `process_completed`, exit 0, `physical_success=null`.
- 개별 설정·입력·출력 해시와 원본 경로는 [verification.json](verification.json)에 있다. 결과 기록 추가 커밋은 실제 실행 소스 커밋과 다르다.

## 실제 실행과 판정

| 실행 | 범위와 결과 | 비용·한계 |
|---|---|---|
| `workflow run act-map-suite --timeout 240 -- --spec maps/act_generalization/suite_v1.json --render` | [22개 지도 결과](case-results.json): 기하 경로 후보 19, 의도한 불가능 대조군 2, `regression_fully-blocked`의 격자 경로 미확정 1. 22개 모두 렌더 오류·초기 충돌 0. [요약](map-summary.json)·[지도 매니페스트](map-manifest.json). | 정적 기하·초기 카메라 검사만 수행. 기존 장면 6개의 상단 카메라 FOV 제한이 남음. 명령·정책 호출·운반 시도 0, 모델 비용 $0, 성공률 미정. 소요 11.83초. |
| `run configs/simulation/local.json --scene act/train-open-1 --sim-seconds 3 --wall-seconds 45 --camera cctv_warehouse --capture --video` | MuJoCo 창에서 `act/train-open-1` 로드, 3초 시뮬레이션과 31프레임 영상 생성, 정상 종료. [결과](native-result.json): 실행 9.28초, 명령 0, 모델 호출 0. | 로봇이 작업을 수행하지 않은 장면·기록 확인이며 물리 운반 성공 아님. `physical_success=null`. |

전체 오프라인 CI는 **1,821 통과, 8 건너뜀, 205 subtests 통과**였다. 관리·지도·장면 집중 검사 101개 통과. `rgb-standard-dispatch-v2` 정적 번들 검증은 통과했으나 상태는 `experimental_unqualified`이다. 새 입력 이력 학습 workflow는 매개변수·입력 검사와 기록 경로까지만 검증했다. 학습 실행·학생의 새 지도 통과·단독/공동 RGB 운반 재현은 수행하지 않았다. 연결된 병목 분석의 RGB 파지·방향 실패가 이 검증으로 해소됐다는 판정은 없다.

## 결과 열람과 보관

- 22개 장면을 한눈에 보는 원본: `/Users/changmin/projects/ugrp/outputs/simulation-runs/20260923-141800-act-map-suite-2a1400e3/artifacts/top-overview.jpg`; 개별 지도·자기 RGB와 평가 JSON은 같은 실행 폴더에 있다.
- 네이티브 영상: `/Users/changmin/projects/ugrp/outputs/simulation-runs/20260923-141843-local-f64cce22/artifacts/motion.mp4`.
- 새 TensorBoard 스냅샷: `/Users/changmin/projects/ugrp/outputs/tensorboard/0923-standard-preview`. 공유 기본 logdir에서 새 실행과 `0922-native-simulation/07-window-final`을 함께 열어 Time Series 4개, HParams 열, 영상 링크와 실제 scalar 값(`wall_s=9.2827`, `commands=0`, `model_calls=0`, `protocol_complete=1`)을 확인했다. 지도 묶음은 정책·작업 실행이 아닌 정적 검사라 운반 성공 지표를 만들지 않았다.
- 원본 295개 지도 파일과 42개 네이티브 실행 파일을 작업용 worktree에서 기본 프로젝트의 `outputs/simulation-runs/`로 복사했고 **337개 파일의 SHA-256이 모두 일치**했다. 기존 스냅샷은 보존했다. 이 로컬 보관은 원격 raw 백업이 아니다. Google Drive는 사용하지 않았다.
