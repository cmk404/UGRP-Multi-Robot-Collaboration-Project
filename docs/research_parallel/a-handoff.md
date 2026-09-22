# A · R0/R1 handoff

2026-09-22. 담당 task `01a0c783-4c16-7fa2-a517-57a2060c0a11`.
브랜치 `codex/r0-r1-information-boundary-audit`.

## 버전과 산출물

- 감사 production SHA: `120cc821b6a1d5c104aab8c8ef2260cf7f8c9a7b` (착수 당시 origin/main 일치).
- 감사 코드·프로토콜 SHA: `1f4ac13049815c5230848cfdf51afe8588227f81`.
  이후 handoff 기록 커밋은 production/test 동작을 바꾸지 않는다.
- 실제 모델·물리 실행 SHA: 없음. 기존 요청 표본의 실행 SHA `e099a4a38c8da094c8c1946ffca2e7ab02f16a83`는
  과거 자료이며 이번 실행 버전이 아니다.
- [프로토콜](a-protocol.md): 2+1 임무, none/structured/natural, 동기화·지표·유한 6회 후보.
- [경계 감사](a-boundary-audit.md): actor×출처, 현행 경로 분류, A-01–06과 B/C 최소 수정 요구.
- [실행 전 gate](a-go-no-go.md): 현재 NO-GO 및 통합 후 반례.
- [오프라인 테스트](../../tests/test_rgb_communication_boundary_audit.py): 14개 독립 감사 검사.
- [fixture와 출처](../../tests/fixtures/rgb_communication_audit/README.md): source hash,
  실제 요청 표본 경로/해시, 합성 입력 및 실행하지 않은 protocol candidate.

공용 README/current_status/TODO/실험 인덱스/CI 및 production 파일은 수정하지 않았다.
기존 다른 worktree·실행·원본도 변경하지 않았다. 기록은 이 로컬 프로젝트와 Git PR에 둔다.

## 검증 및 판정

Python 3.12.13, 기존 `/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python` 사용.
로컬 오프라인 집중 검증 **141 passed, 40 subtests passed**. 새 테스트만 stdlib unittest로
실행하여 **14 passed**도 확인했다. `git diff --check` 통과. 호출·시계·명령은 fake이며
MuJoCo 실행·렌더링·학습·외부 LLM·원격 실험·실물 구동을 시작하지 않았다.

통과의 의미는 현재 경계와 결함의 재현이다. A-01 공유 계획 우회, A-02 중첩
semantic oracle 키 방어 공백, A-03 RGB intent 만료 미적용, A-04 공동 request 미연결을
해결 완료로 표시하지 않았다. 수신자/자기 기억 격리, none의 입력/호출 시점 비간섭,
저장 이미지 해시, message 없는 기존 consent matching을 확인했다.

기존 실제 RGB 요청 1건의 context/이미지 라벨/해시 및 기존 false-grasp 이미지 3장을
읽었다. 이미지에서 한 로봇만 아는 파지 실패를 입증하지 못하여 복구 후보는 no-go다.
새 실험·학습·평가 결과가 없어 TensorBoard 스냅샷을 추가하거나 대시보드를 재시작하지
않았다. fixture/테스트 성적을 연구 성능 결과로 등록하지 않는다.

이 handoff 작성 시점에는 원격 push/PR/CI 전이다. 최종 전달 시 PR URL·원격 HEAD·
CI 확인 상태를 별도로 보고한다. CI test 목록에 새 A 모듈은 아직 포함되지 않아 기존
CI의 green이 새 감사 검사 실행을 뜻하지 않는다.

## B/C/D에 전달한 계약과 후속 의존

| 담당 | 전달/회신 상태 | 통합 이후 남은 일 |
|---|---|---|
| B `01a0c783-4c8d-74e3-ac99-297dcb21cd12` | own request tuple와 최소 GO/HOLD/lease, generic 충돌 응답 요청. B는 own/top+static+명령 및 evaluator 분리, 독립 matching·peer role 비노출 fake 검사를 보고 | 확정 SHA로 actor event·거부·완료·resource 시점까지 독립 재감사 |
| C `01a0c783-4c1c-7af0-8c4f-af217b4d88cd` | nested allowlist·TTL·stale response·none joint request·공유 plan 금지 요구 전달. 신규 runtime은 별도 파일로 진행 중 | 실제 serializer/clock/port와 정확한 message→decision→action trace 확인 |
| D `01a0c783-4cde-74b2-8c66-4f98bad3fd40` | 두 scenario ID와 예산 일치 확인. recovery/hash/model/message-token cap/readiness는 미확정 유지 | source/물리/모델·장면·예산 강제 고정 후 단일 실행 manifest로 admission |

이 PR은 main 위의 독립 감사여서 B/C/D 코드를 import하거나 다른 PR을 병합하지 않는다.
#93 문서는 계획 참고이고 코드 의존성은 없다. #89/#91/#92의 필요한 변경 채택은
기존 담당·코디네이터가 별도 결정한다. 위 회신은 담당자가 보고한 진행 상태이며
A가 그 worktree의 전체 테스트나 실제 통합을 실행했다는 뜻은 아니다.

## 바로 이어서 할 명령과 완료 기준

저장소 root에서 다음 명령은 네트워크/시뮬레이터 없는 감사 재현이다.

```sh
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python -m unittest discover \
  -s tests -p test_rgb_communication_boundary_audit.py -v

/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python -m pytest -q \
  tests/test_rgb_communication_boundary_audit.py tests/test_camera_policy.py \
  tests/test_camera_runtime.py tests/test_camera_robot_port.py tests/test_coela_runtime.py \
  tests/test_mixed_warehouse_protocol.py tests/test_dispatch_research.py \
  tests/test_dispatch_execution.py tests/test_task_stage_sync.py

git diff --check
```

코디네이터의 다음 단계는 확정 B/C/D 커밋을 별도 통합 branch에서 연결하고
[go/no-go 반례](a-go-no-go.md)를 실제 새 포트·trace에 적용하는 것이다.
공용 `scripts/run_ci_tests.py`에 `tests/test_rgb_communication_boundary_audit.py`를
등록해야 한다. 기존 characterization이 보존될지, 신규 연결의 regression으로
전환할지는 수정된 경로를 명시해 결정한다. 결함 재현 green을 compliance로 세지 않는다.

후속 실제 실행의 기본 대상은 Colab CLI, 단일 제출자, 6회·432모델호출·입력3.6M/
출력331,776토큰·36,000명령·전체6,900 wall초의 **후보 상한**이다. D의 통합 manifest가
단위를 확정하고 실제 강제 기능·모델·해시·관측 차이를 검증한 뒤에만 실행 명령을
고정할 수 있다. 아직 없는 통합 runner의 원격 명령을 만들어 실행하지 않는다.
실제 회수 담당자는 성공/실패·미실행 전체 목록과 원본 해시를 기록하고 TensorBoard
공통 경로의 새 스냅샷/로딩/화면을 확인한다. 사용자 명시 승인 전 main 병합은 금지다.
