# B의 D3 완료 raw 독립 로그 분석

2026-09-22. 실행 소스 `8c1b64a95ca9dc3196e80ea6d47ea7c98d2776f5`.
raw: `/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/d3-local-replay-8c1b64a`.
B 분석 소스 작업 폴더는 `6f6c/ugrp`, 제품 코드는 `1d46f7a`에서 변경하지 않았다.

## 핵심 판정

- 이번 두 실행의 최초 실패 경계는 **wall/SIM stale 판정이 아니다**. 양쪽 모두
  `RGB_WORKER_REJECTED`가 0이고 기존 제출→소비 2초/SIM 나이 1초 제한을 유지했다.
- Solo는 worker가 정상 응답해도 `approach`를 벗어나지 못했다. 첫 이상 구간으로 특정할 수 있는
  지속 반복은 **44.65~179.55 SIM초의 servo3 508↔500 교대 270명령**이다. 마지막까지 접근 중이며
  grasp/carry/release 성공 근거는 없다. 종료 원인은 `MAX_TICKS`, 실제 clock 179.95000000013732초다.
- Joint는 0.1 SIM초의 첫 두 worker에서 오류, 0.2 SIM초에 작업 해제, 명령 0이다.
  1.05 SIM초의 `ALL_ACTORS_FINISHED`는 각 replay actor의 후속 `cannot_continue`에 따른 종료로,
  성공 선언 또는 최초 오류 원인이 아니다.
- A가 supervisor 감사에서 별도로 확인해 공유한 정확한 오류는
  `ValueError: RGB outside saved approach support`다. 아래 소스 대조에 따르면 첫 coarse 영상 특징의
  지원 조건이 성립하지 않았다. 학습된 yaw/lateral/forward/grasp predictor나 실제 공동 파지까지
  도달한 실패로 해석하면 안 된다.

## 검증 범위와 원본 보존

B는 worker-timing/runtime/skill-inputs만 의미 분석했다. 1488개 inventory 파일은 내용 해석 없이
SHA256를 전수 대조했고 전부 일치했다. inventory/finalization/outer launcher receipt 해시도 D 전달값과
일치한다. `analysis.json`에 receipt 원문·검증값을 저장했다. 실행 source clean은 읽기 전용 Git 검사로
확인했고, B backend 소스의 SHA256는 실행 커밋 내용과 동일한
`15632076a99d21891d65243d6fcae2f1e132e16b3bb910efaedb924700b64cba`다.

실행/모델 추론/렌더/파라미터 변경/재시도/코드 수정은 하지 않았다. evaluator 재채점과 영상 의미 판정은
D/A 범위이며 B가 수행했다고 주장하지 않는다. 실제 실행물은 invalid_artifact가 아니라 완료된 실패/시간초과
증거라는 D 회수 판정과 모순되는 로그는 발견하지 못했다. 프로세스 reap/groupgone은 확인서에 근거하며
B가 종료 시점에 직접 관찰했다고 표현하지 않는다. 해시와 로컬 보관은 원격 백업이 아니다.

## Solo: 정상 처리와 접근 명령의 비수렴을 분리

- 350회 worker 계산이 모두 `returned`; 349회 소비, 1회는 마지막 close 때 권한 회수.
- 발행 명령 463개를 observation/skill-input history에서 command_id로 중복 제거했다.
  463개 모두 `stage=approach`다. 명령 발행은 측정 관절 상태나 실제 이동을 뜻하지 않는다.
- command 190~192: 43.6~43.8초 마지막 전진 명령, 193: 44.1초 servo4=2496.
  command 194(44.65초, observation r2-000170)부터 463(179.55초, r2-000709)까지
  servo3=508/500이 정확히 교대한다. 긴 접근 체류를 deadline 문제와 구분하는 직접 증거다.
- `VisualBoxSkill._approach`의 세로 영상 오차→손목 ±8→500 하한 클립 경로와 모양이 일치하지만,
  허용 로그에는 해당 decide의 pixel_centroid/target/분기 evidence가 없다. 그러므로 실제 원인을
  영상 흔들림, 좌표계/해상도 보정, 접촉/가림 중 하나로 확정하지 않는다. source: `harness/visual_box_skill.py:337`.
- 마지막 r2-000710은 제출179.85초, 계산 완료 후 close가 179.95초에 회수했다.
  compute=0.102292초, 제출→회수=0.110213초, 완료→회수=0.007724초다.
  이 1회를 stale 거절/계산 timeout/늦은 답의 행동 실행으로 세지 않는다.
- MAX_TICKS=3600, 첫 tick=0이므로 마지막 요청179.95초다. terminal clock은 verified이고
  primary_error/close_error가 없다. 과거 float-clock invalid artifact와 다르다.

### Solo 시간 분포 (초)

| 항목 | 중앙값 | p95 | 최대 | 표본 |
|---|---:|---:|---:|---:|
| worker queue | 0.000221 | 0.000801 | 0.019647 | 350 |
| worker decide | 0.143305 | 0.341288 | 0.839568 | 350 |
| 제출→소비 poll | 0.185231 | 0.439033 | 1.420463 | 349 |
| 완료→소비 poll | 0.044406 | 0.117915 | 0.740680 | 349 |
| 이미지 SIM 나이 | 0.15 | 0.25 | 0.45 | 349 |
| 신규 입력 capture/pack | 0.126977 | 0.255972 | 0.782475 | 350 |
| 입력 저장 I/O | 0.000975 | 0.002366 | 0.011386 | 350 |

최대 제출→poll은 r2-000167: 입력43.9초→소비44.05초, queue0.000491 +
compute0.679292 + 완료→poll0.740680 = 1.420463초. worker-timing.jsonl **1290행**.
따라서 owner 지연은 존재하지만 이번 소비 시점에는 제한을 넘지 않았다.

owner 3599회 advance의 누적 physics27.436초, referee0.740초, video capture54.022초다.
중복 capture_id를 제외한 own RGB702회=51.229초, TOP698회=49.512초다.
worker compute 누적58.316초는 owner 작업과 겹칠 수 있고, capture/pack에는 RGB capture가 포함된다.
이 값을 단순 합산해 총 시간 또는 배타적 병목 비율로 해석하지 않는다. `completion_to_poll`도
완료 로그 I/O를 포함하며 순수 scheduler 대기와 같지 않다.

## Joint: 오류 발생 → 해제 → actor 종료

1. 0.05초: r1 lower와 r3 upper의 독립 요청이 모두 수락되어 같은 pair lease 활성화.
2. 0.1초: r1/r3 각 첫 skill input(각 observation `*-000003`) 준비 후 제출.
3. r3 worker 0.037623초, r1 worker 0.040230초 계산 후 `worker_outcome=error`.
4. 0.2초: `rgb_skill_unavailable`로 두 Future 권한 회수. 둘 다 done=true, cancelled=false.
   제출→회수는 각각 약0.0597/0.0598초로 wall2보다 작다. 이 값은 poll 시각 자체가 아니라
   poll 이후 회수까지의 상한이다. SIM 나이는0.1초이고 stale rejection은 없다.
5. 1.0초: runtime local_status에 STOPPED/TERMINATED 표시.
6. 1.05초: r1/r3가 cannot_continue 반환, r2는 이미0.05초에 비참여자로 종료.
   ALL_ACTORS_FINISHED는 이 흐름의 마지막 사건이다.

### 오류 문구의 소스 대조와 한계

A가 `backend/backend-evaluation.json → coordination_audit`에서 확인해 공유한
SKILL_FAILED는0.2초, ValueError, `RGB outside saved approach support`다.
B는 이 평가 파일의 내용을 직접 읽지 않았으며, 해당 교차 근거는 `cross-review.json`에 분리했다.

첫 controller phase는 coarse이고 `PairActorSkill.decide`에서 canonical TOP 처리 뒤
`coarse_approach(...).ok == False`이면 이 문구를 발생시킨다. 실행 source의
`harness/camera_goal_transport.py:98`에 해당 false 사유는 다음뿐이다.

- `lane_or_payload_unresolved`: current 또는 reference의 lane/payload 특징을 못 찾음.
- `wheel_heading_unresolved`: current의 wheel heading을 못 찾음.

선택된 개별 reason/특징/변환은 오류 때 raw에 보존되지 않아 둘 중 무엇인지 확정할 수 없다.
재추론으로 빈칸을 채우지 않았다. 문구가 saved support라고 해도 학습된 stage predictor의
분포 밖 거절이라고 단정하지 않는다. 현재 지점은 그 predictor 호출 전이다.

또한 `_service`는 `[r1,r3]` 순서로 Future.result를 꺼내고 `_fail`은 첫 예외 하나를 참여자 둘의
SKILL_FAILED에 복제한다. 따라서 r1의 최초 상세 오류를 좁힐 수 있지만, 동일한 감사 레코드2개가
r3의 독립 예외 문구까지 증명하는 것은 아니다. r3 자체의 error 완료는 worker 로그로 확인된다.

## 원본 프레임 재사용 검사

- Solo: frame read714, own hit12/TOP hit16, unique capture1400.
- Joint: frame read12, own hit0/TOP hit7, unique capture17.
- 같은 capture_id의 metadata/원본 시각/SHA/own 로봇 소유권 변화0, read SIM과 capture SIM 불일치0.
- 모든 skill input의 JPEG700+4개를 실제 파일 hash와 capture hash에 대조해 전부 일치했다.
- 이는 이번 raw의 기록 일관성 검사다. 모든 가능한 reset/외부 in-place 변경을 검증했다는 뜻이 아니다.
  cache hit 수가 작고 통제된 before/after cohort가 없으므로 B3의 인과적 속도 개선 크기도 주장하지 않는다.

## 산출물과 다음 판단 경계

- `analysis.json`: 통계, 원본 hash, 최종 runtime, 상태 변화, 반복 명령 경계.
- `evidence.json`: 원본 경로/1-based 행번호를 붙인 핵심 이벤트와 명령.
- `worker-calls.json`: observation_id별 completion/consume/revoke 연결.
- `issued-commands.json`: 중복 제거한 발행 명령과 최초 발견 위치.
- `cross-review.json`: A의 별도 감사 공유 내용, B 직접 증거와 구분.
- `file-hashes.json`: 위 산출물 및 분석 코드·보고서의 SHA256.

코드 수정/새 실행은 별도 승인 전까지 하지 않는다. 후속 진단이 승인된다면 필요한 증거는
joint 실패 prediction.reason/current-reference feature 여부와 solo 접근 세로오차/선택분기다.
현재는 기록의 빈칸을 명시하는 데서 멈춘다. 성공률·통신 효용·일반화를 이 두 scripted 시행으로
판정하지 않는다. TensorBoard와 실제 영상 검토는 D/A가 별도 진행하며 B는 새 스냅샷을 중복 생성하지 않는다.
