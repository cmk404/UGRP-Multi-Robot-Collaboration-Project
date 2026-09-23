# D3 — 로컬 2회 준비와 변경 경계

공통 출발점은 `357e1f2e66dcae20c54f669711308ed18cf03966`(#98)이다.
후속 브랜치 `codex/d3-local-study`는 #98에 쌓고, 사용자 검토·명시적 승인 전 병합하지 않는다.
기존 D2 원본 `bfe478f`와 offline 수정 검증 `f139672`는 별도 기록으로 보존한다.

## 범위와 admission

사용자가 승인한 새 실제 실행은 `d3-local-solo`, `d3-local-joint` 정확히 2회,
순차 1개이며 시행당 SIM 180초, wall 600초, 명령 6000개 이내다.
준비·회수·정리를 포함한 실행 할당은 wall 1800초이며 실패도 횟수에 포함한다.
외부 LLM·통신 메시지·새 학습·재시도·원격 세션은 0회다.
ACT 소유자의 종료/자식 정리 확인과 최종 고정 source/config에 대한 A3 독립 감사가
모두 있어야 시작한다. 준비·offline 테스트는 실제 물리 성공의 증거가 아니다.

기존 #98 `scripts.run_rgb_communication_study prepare/check/run/trial`과
소유 subprocess 종료·reap 경계를 그대로 사용한다. 모델/렌더링 작업자가 늦게 끝나도
전체 child 종료 뒤에만 부모가 원본 해시를 수집한다. 단순 `bundle.close()`를
모든 파일 쓰기의 종료로 해석하지 않는다.

로컬 부모 SIGTERM도 KeyboardInterrupt와 같은 소유 child TERM→5초→KILL→reap
경로로 처리한다. exit130 뒤 다음 시행을 시작하지 않고 `unrun/study_interrupted`로
분모에 남긴다. 바깥 launcher는 같은 `bounded_process`로 기존 study CLI를 직접
감싼다(1780초+최대10초정리+기록 여유10초, study allocation1750초).
중첩 세션 wrapper의 동일 grace 경쟁은 추가하지 않는다.
이 runner는 자신이 시작한 trial의 별도 process group을 회수한다. 기존 공유 viewer나
ACT 프로세스를 종료 대상으로 검색하지 않는다.

| 부모가 관찰한 종료 | 다음 배정 시행 |
|---|---|
| 정상 process exit0 + 물리 목표 실패/런타임 실패 기록 | 원래 배정된 다음 시행 진행(재시도 아님) |
| 부모 SIGTERM/KeyboardInterrupt, child exit130 또는 signal exit | `unrun/study_interrupted` |
| child wall timeout | `unrun/previous_trial_wall_timeout` |
| 기타 child 비정상 exit, leader 종료 뒤 잔류 자식 정리 필요 | `unrun/previous_trial_process_failed` |
| child reap 또는 process group 소멸 미확인 | `unrun/child_cleanup_unconfirmed`; raw 최종 해시 생성 금지 |

리더 종료만으로 전체 그룹 종료를 추정하지 않는다. 정상·중단 모두 소유 PGID의 실제
소멸을 검사하며 남은 자식은 같은 유한 cleanup 안에서 정리한다. TERM을 무시하면
KILL로 올리고 drain 시간을 남긴다. 미확인 상태를 성공이나 완료된 회수로 바꾸지 않는다.

A의 `755dca3` 독립 감사는 Popen 반환 직후 wait 보호 구간 전의 SIGTERM에서 남는
자식을 재현해 해당 후보를 NO-GO로 판정했다. 후속은 INT/TERM handler가 예외를 던지지
않고 요청 flag만 기록하며, 0.1초 이하의 유한 wait poll에서 이를 소비한다. 생성 중과
cleanup 중 반복 신호도 같은 방식으로 처리한다. 과거 후보의 검사/입력은 보존한다.

`report.json.child_cleanup_confirmed`는 자식 정리만 뜻한다. inventory 작성이 끝난 뒤
별도 `artifact-finalization.json`에 그 inventory 해시를 기록한다. 회수 완료는 outer
exit0·reap/group 소멸·이 receipt와 inventory의 실제 재해시를 모두 확인해야 한다.
report만 존재하거나 inventory 쓰기 도중 종료된 실행은 해시 완료로 표시하지 않는다.
회수가 유예 시간 내 끝나지 않아도 cleanup 유예를 다시 시작하지 않는다. 단일 시도 후
미확인 상태를 반환하며, 후속 시행·원본 해시 완료 판정은 계속 차단한다.

## 로컬 자산 참조 준비

`scripts.prepare_rgb_communication_replay`의 명시적 `--asset-mode reference`는
기존 모델/학습 보고서/manifest와 고정 TOP 기준 이미지를 읽기전용으로 재사용한다.
새 evidence 폴더에는 경로·해시를 담은 metadata만 만들며 모델 파일을 복사하지 않는다.
기존 원격용 `copy` 모드는 별도 선택으로 보존한다.
절대경로 descriptor 고정은 `reference`에만 적용해 원격 capsule의 경로 이동을 막지 않는다.
D3는 `copy` 선택 자체를 거부한다. 기존 복사 모드는 D2의 상대경로 계약을 유지한다.
공동 스킬의 위·아래 역할은 준비할 때 명시하며, 실행 중 자기 움직임으로 확인한 TOP 영상의 빔 상대 위치와 대조한다. 아래 배정은 `dispatch/open` seed 11 진단용이다.

```sh
python -m scripts.prepare_rgb_communication_replay \
  --assets-root /EXISTING/READ_ONLY/assets \
  --output outputs/d3-local-study-inputs \
  --asset-mode reference --run-prefix d3-local --submitter D3 \
  --pair-role-assignment r1-upper-r3-lower
```

결과는 `ready=false`이며 `config-for-audit.json`에 독립 감사가 없다.
`local-assets.json`은 `rgb-local-assets.v1`의 절대 실제 경로→SHA-256 목록이다.
parent와 child의 preflight는 모든 파일 해시와 backend input 전체 포함 여부를 확인한다.
일반 evidence `checked_reference`의 상대경로/루트/심볼릭 링크 제한은 완화하지 않는다.
추가된 `backend_descriptor_evidence`는 모델과 manifest가 함께 바뀌는 경우도 검출한다.
실제 runtime output 경로는 기존 계약처럼 trial별로만 바뀐다.

D3는 `rgb-offline-boundary-review.v2`, `independent_reviewer=A3`를 요구한다.
구 v1/A2 자료가 해시상 유효하더라도 신규 admission으로 승계하지 않는다.
최종 source/config를 바꾸면 새 준비/감사가 필요하다.

## 바꾸지 않는 연구 경계

진단 장면은 backend가 지원하는 `dispatch_open`, seed 11이며 native scene selector의
`dispatch/open`과 연결한다. native 기본 `dispatch/shared_crossing`을 대신 쓰지 않는다.
58개 scene 등록/초기화나 ACT 22조건 미리보기는 운반 지원·성공이 아니다.
미지원 map은 `unrun`이며 open으로 대체하지 않는다. 이전 학습 맵 출처는 unknown으로
남기고 진단 장면은 노출된 regression으로 취급한다.

카메라/FOV·weld OFF·자기 RGB+TOP+자기 발행 명령 경계는 그대로다.
평가 장면/좌표·접촉·물리 성공·reset 원문은 actor에게 전달하지 않는다.
worker freshness는 제출부터 **소비까지 total wall 2초**, SIM age 1초이고,
consent/revision/cancel을 소비 시점에 다시 확인한다. completion→poll 지연을 분리한
계측은 이 기준을 대체하거나 완화하지 않는다.

6회 LLM pilot은 정상/경쟁 2 scenario × none/structured/natural의 준비만 허용된다.
세 독립 equal LLM의 통신 효과가 연구 질문이며 ACT 검증 결과와 합산하지 않는다.
현재 provider hard input/output/call/time/cost bound가 입증되지 않아 `ready=false`다.
새 모델·예산의 명시적 선택과 물리/관측/비용 gate 없이 POST를 하지 않는다.

## 결과 기록

실제 실행 후 source/config·환경·모든 실패·원본·해시·자식 정리 결과를 새 실험 폴더에
기록한다. TensorBoard는 기존 공통 logdir에 원본 해시 중복 확인 후 새 snapshot으로
추가하고, 로딩과 기존 native UI·고정 링크까지 확인한다. 합성 테스트는 공용
대시보드에 올리지 않는다. Google Drive/새 상시 서버/자동 감시는 사용하지 않는다.

완료된 실제 2회의 고정 SHA·실패·회수·독립 감사·TensorBoard 화면 확인은
[D3 로컬 재생 기록](../../experiments/2026-09-22-rgb-communication-local-replay/README.md)에 있다.
이 기록은 추가 실행이나 live LLM pilot 승인이 아니다.
