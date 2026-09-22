# 2026-09-22 — RGB 통신 기반 최소 물리 재생 2회

단독·공동 deterministic replay 각각 1회를 Colab CPU에서 실행·회수했다.
**두 시행 모두 `invalid_artifact`이며 물리 성공·통신 효과를 입증하지 못했다.**
실행기는 정상 종료했지만 내부 runtime은 `aborted/RUNTIME_ERROR`였다.
LLM 호출·입출력 토큰·메시지는 각각 0이다. 승인된 2회는 소진됐고 재시도하지 않았다.

## 동결 소스·설정·환경

- 실행 SHA: `bfe478f1209375d774b6dc2881300198f967f5ca`, 별도 `codex/advanced-rgb-validation`.
- A `ca32f280a77b484f299059f381e7ea2c8926850d`, B `be819a9e63db41ceec04b28d2369e60177e1deb6`, C `44e831de52158f604a78d8c31fd270263dc37784`, D `7fe2b15` 조합.
- manifest canonical SHA256: `3eebd939fe81c80133b9b5c332c1af445d824d3e9f713d127dfbe456635b017b`.
- config SHA256: `777c9c9bbef09c970f9c21b289b34db34c9dee1968c13d36e8ad4245a0daff0b`.
- A preflight certificate SHA256: `133207b2a5ea5a1833228cabc8876d1a6993b7fa65a227fac8ed558d99ce651c`. 505 source/11 cases/9 artifacts의 pre-physics 경계 감사이며 physical pass가 아니다.
- Colab 신규 CPU/default, Python 3.12.3, Linux x86_64, OSMesa. 패키지 원본은 `recovered/packages.txt`이며 해시는 별도 inventory에 있다.
- `dispatch_open`, seed 11, weld OFF, 고정 RGB 카메라. map instance `733b5ee3320f0230c199afaefe2c2a6b8baa6a57a152417a6e8022e3bc0f9bab`, group `fa0944a500de62caa2947ab321513c1063ab565c482016ed4e16645e21ab5987`.
- 기존 모델의 학습 맵 출처는 unknown. 기존 22-map suite와 alias하지 않은 development diagnostic이다. Test A/B·held-out 일반화 주장은 없다.
- 상한: 시행당 SIM 180초/wall 600초/발행 명령 6000, 전체 준비·회수·종료 1800초, 외부 모델 0회, 자동 재시도 없음.
- 전체 실제 작업 170.715346초. 신규 세션 `ugrp-d2-06e86e6cb059`만 stop(exit 0)했고 이후 `colab sessions`에서 활성 세션 없음 확인.

## 전체 결과와 판정 경계

| 지표 | 단독 box | 공동 beam |
|---|---:|---:|
| canonical outcome | invalid_artifact | invalid_artifact |
| runtime outcome | aborted / RUNTIME_ERROR | aborted / RUNTIME_ERROR |
| child process wall, 초 | 46.901877 | 35.877640 |
| runtime wall, 초 | 27.571217 | 25.518374 |
| 발행 LOCAL_COMMAND | 5 | 0 |
| planner decision | 3 | 3 |
| 외부 모델 호출/토큰/메시지 | 0 / 0 / 0 | 0 / 0 / 0 |
| runtime terminal 요청 SIM, 초 | 0.8 | 0.8 |
| evaluator snapshot SIM, 초 | 0.75 | 0.75 |
| 원본 raw mission_complete | false | false |
| 원본 replay_goal_complete | null | null |

evaluator가 runtime 종료보다 앞서므로 `evaluator snapshot predates the runtime terminal event` guard가 두 결과를 거절했다. 원본 시각을 고치거나 `replay_goal_complete=null`을 false로 치환하지 않았다. raw referee의 대상 물체 physical_success는 false이고 이동량 0, lift/carry/delivery 없음, weld 0이다. 이는 각각의 원본 관측이며 모순된 최종 증거를 유효한 성공/실패 점수로 바꾸지 않는다. 확정 성공 수 0은 물리 성능의 유효 성공률 0/2와 다르다.

단독은 r2 task 접수 후 팔 자세 명령 5개를 발행했으나 SIM .75에서 stale RGB worker result로 스킬이 종료됐다. 공동은 r1/r3 동의를 받았지만 .25에서 같은 오류로 스킬 종료됐고 저수준 명령이 없다. 그 뒤 두 runtime이 요청 .8에서 RuntimeError로 중단됐다. 배우별 종료 주장·스킬 종료·독립 물리 판정을 분리한다.

## 진단과 독립 감사

- A의 [회수 감사](a2-postreplay-audit.json): 69 파일 해시, 두 실행 각 10개의 RGB 참조, 단독 발행 5개 모두 observation/decision/consent 연결 검증. live gate closed.
- C의 [clock 진단](c2-clock-diagnosis.md): 실패한 요청 tick을 종료 시각으로 쓰는 기록 결함은 fault injection으로 직접 재현했다. last acknowledged .75와 요청 .8이 혼동된다.
- B/C는 수정하지 않은 clock 경로로 `2.049999999999996`과 `2.0500000000000007`의 차이 `-4.88498e-15`가 엄격한 역행 검사에 걸리는 같은 시점·수치도 독립 재현했다. 원본에는 cause chain이 없으므로 최종 오류의 강한 일치 후보이며 원문에 확정 원인이 기록됐다고 주장하지 않는다.
- 선행 stale worker 오류는 별도다. 입력의 SIM age가 단독 .25/공동 .15초여서 고정 소스의 wall-age >2초 분기가 발동한 것으로 추론한다. 이 wall-age는 계산뿐 아니라 owner의 rendering/I/O/poll 대기도 포함한다. 실제 계산 지연과 늦은 회수를 구분하는 원본 계측은 없다.
- 진단에 물리·렌더러·외부 모델을 새로 실행하지 않았다. 이후 clock 수정은 별도 후보이며 이 원본 재분류·추가 Colab 실행을 승인하지 않는다.

## 검증과 TensorBoard

- 동결 실행 소스: RGB 163 passed/4 skipped/14 subtests, 전체 offline 1336 passed/7 skipped/198 subtests.
- 진단용 TensorBoard exporter `ab22751`: 관련 34 tests passed. 시간 모순 guard는 그대로 두고, 유효성 0 및 원본별 비용/종료/시각을 별도 표시한다. 모순된 `result/sim_s`, 미확정 false-finish 점수, 적용되지 않는 모델 지연을 만들지 않는다.
- 공통 logdir의 새 snapshot `0922-RGB재생-단독`, `0922-RGB재생-공동`. 기존 snapshot/원본은 보존했다. manifest SHA256은 각각 `7baaaa97bf32876ee56bdd2cf1e9bacb2514e38e5cf174db98254b6c433f755b`, `66657846025d604bd7bd9e5e3bd5487cded1e61360603366481ab2b6bdf48dee`.
- EventAccumulator 및 실행 중인 기본 TensorBoard의 두 run 로딩 확인. success/evidence_valid 0, commands 5/0, external calls 0/0, process wall, planner decisions를 고정하도록 저장했다. `reported_success=0`은 확정 성공 없음이지 `replay_goal_complete=false`가 아니다.
- 화면 설정: 기본 체크아웃 `outputs/tensorboard-view.json`의 `rgb_communication_replay_20260922`. 기존 default/다른 작업 view를 덮어쓰지 않았다. [TensorBoard 결과](http://127.0.0.1:6006/?runFilter=%5E0922-RGB%EC%9E%AC%EC%83%9D-#timeseries).
- 새 영상 2개 등록 및 HTTP 200/Range 206 확인. 두 MP4는 같은 SHA256, 960×720, 4fps, 5프레임/1.25초다. 5프레임 전체를 decode해 setup absolute 1.30~2.05초 장면만 확인했으며 운반 동작 성공을 보여주지 않는다. 영상 시간은 SIM과 자동 정렬하지 않는다.
- **실제 화면 검증:** 기존 Chrome 강 탭 `963999421`의 소유 작업 `01a0c7a0-7425-7353-b6cf-0ee551a13a13`이 정상 재연결 후 같은 탭에서 대행했다. 두 run, Time Series 고정 6개 수치, HParams outcome/success/runtime wall/commands 4열, 원본 영상 두 URL의 로딩·재생을 확인하고 저장된 Time Series URL로 돌아와 표시를 유지했다. 강제 소유권 우회·중복 탭·다른 작업 서버 변경은 없다.
- **표시 한계:** 공통 HParams는 첫 experiment 정의를 골라 신규 evidence_valid/process_wall_s/external_model_calls/planner_decisions 열을 제공하지 않는다. 새 snapshot에는 metric을 등록했으나 기존 메타데이터를 덮어쓰지 않았으며 해당 값은 Time Series 고정 카드에서 확인했다. HParams wall_s는 runtime 시간으로 process wall과 다르다. Text의 validation 카테고리 본문은 비어 있어 해당 본문·Text 안의 영상 링크까지 화면 검증했다고 주장하지 않는다. 두 원본 영상 URL은 직접 재생했다.

정확한 6-card 고정 URL/4열/실제 검증 범위는 [dashboard.json](dashboard.json)에 보존했다.
검증 종료 후 제가 시작한 `d2-replay-media-review`(PGID 65529, 포트 6007)만
종료하고 listener 부재를 확인했다. 기존 TensorBoard PID 9293와 포트 6009 미디어
PID 9291은 보존했으므로 기본 대시보드는 유지된다. 두 직접 영상 URL은 다음
읽기 전용 companion을 다시 켠 동안만 열린다(새 실험을 실행하지 않음).

```sh
cd /Users/changmin/.codex/worktrees/cf5f/ugrp
python3 scripts/ugrp_session.py run d2-replay-media-review -- \
  /Users/changmin/Project-Runtimes/ugrp/.venv-sim-worker-mac/bin/python -c \
  'from scripts.tensorboard_tools.media import make_server; make_server("/Users/changmin/projects/ugrp/outputs/tensorboard",6007).serve_forever()'
# 열람 종료 후 별도 터미널:
python3 scripts/ugrp_session.py stop d2-replay-media-review
```

## 보존 위치·남은 조건

원본 root: `/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/rgb-communication-colab-bfe478f`.
capsule root: `/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/study-inputs`.
원본 archive SHA256: `c708440c3ec2aa95076449c97b142c8b5e6c02d0ad71692917970a5749459ab7`.
입력 archive SHA256: `87c8ab7245a20cc1559331f52cc1a83e1fd2799b2e3ad51cc231b73e7051f8a1`.
[원본 파일 inventory](raw-artifact-hashes.json), [제출/회수/종료](submission.json), [변경하지 않은 전체 report](original-report.json)를 저장했다. raw 영상·로그·모델은 로컬 보관이며 이 해시/요약을 원격 raw 백업으로 표현하지 않는다. Google Drive는 사용하지 않았다.

물리 pass, clock 수정 후보의 새 실행, A의 수정 영향 감사, provider의 input/output hard token cap 검증 및 최종 live admission은 미완료다. 추가 실행·LLM은 NO-GO이며 승인된 두 시행을 자동 확장하지 않는다. 새 PR #100–103은 별도 사용자 승인 전 병합하지 않는다.

후속 D `30f2409`는 clock-only supervisor 연결과 신규 terminal schema 검사만 추가했다. unknown/malformed clock 9개 RED 후 통과, 부분 진행 .77/last-ack .75 분리, requested/actual roundoff 보존, 구 원본 .8/.75 invalid 유지, actor/evaluator 비노출 seam을 포함해 관련 70 tests를 오프라인으로 검사했다. 이 수정은 원본 실행 소스가 아니며 새 Colab 실행을 포함하지 않는다.

이후 nullable raw clock 표시 보완과 B의 180초 종료 반례 수정을 포함한
`f139672`의 [후속 오프라인 영향 검증](clock-followup/README.md)은 별도 기록이다.
전체 1423 tests와 A의 영향 감사는 통과했지만 execution admission/물리 pass는 false다.
