# UGRP 프로젝트 작업 지침

이 파일은 UGRP 저장소에서 작업하는 모든 Claude 세션의 공통 진입 지침이다. 연구 방향을 새로 추측하지 말고, 저장소에 기록된 현재 상태에서 작업을 이어간다.

## 세션 시작 시 반드시 읽을 순서

1. `README.md` — 연구 배경과 공식안·수정안·참고 자산의 구분
2. `ROADMAP.md` — 현재 실행 순서와 단계별 완료 기준
3. `docs/decision_log.md` — 현재 상태, 결정, 문제 해결, 마지막 세션 인계

세 문서를 읽은 뒤 `docs/decision_log.md`의 **현재 상태**, **미해결 항목**, **가장 최근 세션 인계의 다음 행동**을 먼저 확인한다.

## 문서별 권위와 역할

- `README.md`는 연구 배경과 설계 변천을 설명한다.
- `ROADMAP.md`는 실행 순서와 완료 기준을 설명한다.
- `docs/decision_log.md`는 실행 중인 결정, 관찰된 문제, 시도 결과, 세션 간 인계를 기록하는 현재 상태의 기준 문서(canonical source)다.
- `docs/masterpi_technical_architecture.md`와 `docs/software_tasks.md`는 참고 설계·작업 명세다. 이 문서만으로 연구 방향이나 승인 상태를 확정하지 않는다.

공식 제출계획, Isaac Lab 수정안, 기존 브라우저·MasterPi 구현 자산을 자동으로 하나의 설계로 합치지 않는다. 승인 여부가 기록되지 않은 제안은 확정된 사실처럼 다루지 말고 사용자에게 확인한다.

## 작업 중 기록 규칙

- 작업을 시작하기 전에 현재 상태와 미해결 항목을 읽고, 이번 작업의 목표를 결정 로그와 모순되지 않게 정한다.
- 새로운 연구 방향·환경·로봇 수·비교군·프로토콜을 정할 때는 결정 로그에 결정 내용, 근거, 영향, 승인 상태를 추가한다.
- 문제가 생기면 문제, 실제로 확인한 증거, 시도한 방법, 결과, 해결 여부, 다음 행동을 기록한다.
- 과거 기록은 지우거나 덮어쓰지 않는다. 상태가 바뀌면 새 항목을 추가해 변경 이력을 남긴다.
- 세션을 마치기 전 실제로 확인한 사실과 아직 확인하지 못한 사항을 구분해 세션 인계 항목을 추가한다. 추정·약속·예상은 확인 결과로 기록하지 않는다.
- 연구 결과가 아닌 예상 수치, 시뮬레이션 데모의 동작, 문서에 적힌 API 예시는 실험 결과로 기록하지 않는다.


## Git 운영 원칙

- 이 프로젝트의 canonical source는 `/home/ubuntu/projects/ugrp`의 Git `master` 이력이다. 코드·설정·문서 변경은 가능하면 작은 단위로 커밋한다.
- SIM과 REAL은 별도 코드 branch로 갈라 관리하지 않는다. 같은 commit을 기준으로 환경별 설정/배포 절차만 구분하고, 배포 시 commit SHA 또는 content signature를 기록한다.
- `outputs/`, `stress_logs/`, 가상환경, OS 이미지, 모델/대형 바이너리, 인증정보는 Git에 넣지 않는다. `.gitignore`와 pre-commit size guard를 유지한다.
- 별도 worktree는 격리 실험/PR 준비가 필요한 경우에만 사용한다. live source 변경은 commit한 뒤 필요한 worktree에서 merge/rebase한다.
- 원격 저장소가 설정되지 않은 상태에서는 임의로 push하지 않는다.

## 대화와 worktree의 연속성

- 두 대화가 **같은 저장소 경로**를 열고 있으면 `docs/decision_log.md`를 통해 상태를 공유한다. 동시에 같은 항목을 수정하지 말고, 각 작업 후 파일을 다시 읽는다.
- 별도 Git worktree를 사용하면 파일이 즉시 공유되지 않는다. 한쪽의 기록을 다른 쪽에서 사용하려면 commit 후 해당 worktree에 merge/rebase하거나 같은 checkout으로 전환한다.
- 채팅 기록 자체는 다른 대화에서 자동으로 읽을 수 없다. 대화에서 결정·문제·해결이 나오면 반드시 이 저장소의 결정 로그에 옮긴 뒤 다음 작업을 진행한다.

## 비밀정보 보호

Wi-Fi SSID·비밀번호, 사용자 비밀번호, API key, OAuth token, SSH private key, 계정 식별자와 같은 인증정보는 저장소·결정 로그·커밋·출력에 기록하지 않는다. 필요한 경우 `설정 완료` 또는 `인증정보는 로컬 입력`처럼 비밀값을 제외해 기록한다.

## 현재 연구의 기본 안전장치

- `README.md`와 `ROADMAP.md`의 공식안·수정안 승인 상태를 확인하기 전에는 어느 한쪽을 최종 연구 사양으로 단정하지 않는다.
- 추상 환경과 rule baseline의 재현성이 확인되기 전에는 LLM, Isaac Lab, 실물 하드웨어를 연구 결과의 근거로 확장하지 않는다.
- 실제 장비를 조작할 때는 대상 장치와 현재 상태를 먼저 확인하고, 파괴적 작업이나 저장장치 덮어쓰기는 사용자 승인을 받은 범위에서만 수행한다.

## SIM 실행 문제를 고칠 때의 강제 관찰 절차

SIM에서 동작 실패, 이상한 움직임, 물리적으로 부자연스러운 결과, 성공/실패 판정 오류를 수정할 때는 **코드부터 추측해서 고치지 않는다.** 다음 순서를 기본 절차로 사용한다.

1. 사용자와 동일한 seed/명령 순서를 가능한 한 그대로 재현한다.
2. `outputs/sim_traces/`의 해당 action trace를 확인한다. 각 trace에는 시작 전/실행 중/종료 직후의 robot-camera 및 human observer(3인칭) 프레임과 같은 시점의 privileged MuJoCo state가 저장된다.
3. 먼저 `analysis.json` / `analysis.md`의 self-observer 판정을 읽고, `scripts/analyze_sim_trace.py <trace_id|latest>`로 contact sheet를 생성해 실행 장면의 시간 순서를 검토한다.
4. 로봇 actor가 본 정보(WorldState/robot camera)와 SIM 디버거만 볼 수 있는 privileged truth를 구분한다. privileged truth는 원인 규명과 simulator 검증에만 사용하고 planner 입력이나 REAL-equivalent 성공 판정에 섞지 않는다.
5. 수정 전 반드시 **관찰된 현상 → 첫 이상 시점 → 원인 가설 → 증거**를 명시한다. 단순 exit code, 최종 `ok`, 테스트 통과만으로 실제 동작이 정상이라고 판단하지 않는다.
6. 수정 후 동일 replay를 다시 실행해 trace를 비교한다. self-observer의 critical/warning이 없어졌는지뿐 아니라 3인칭 프레임에서 사용자가 보기에 동작이 실제로 개선됐는지도 확인한다.
7. self-observer가 `ACTION_FAILED_UNCLASSIFIED`를 내거나 영상상 명백한 이상을 분류하지 못하면, 그 실패 유형을 observer 규칙/분석기에 추가한다. 즉 **새로운 SIM 문제를 발견할 때마다 진단 능력 자체도 함께 확장**한다.

조회 API는 bridge의 `GET /sim/traces`와 `GET /sim/traces/latest`이다. 이 forensic 경로는 SIM 개선용이며 robot actor의 관측 채널이 아니다.
