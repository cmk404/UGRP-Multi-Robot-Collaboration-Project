# A3 독립 사후 검토 — 8c1b64a

2026-09-22. 판정: **완료된 유효 실패 기록 2건, 운반 목표 성공 0/2. Live LLM pilot NO-GO 유지.**

실행 소스는 `8c1b64a95ca9dc3196e80ea6d47ea7c98d2776f5`다. D만 실제 두 실행을 수행했고,
A는 완료된 자료의 읽기 전용 감사만 했다. 추가 물리 실행·시뮬레이터 렌더·정책 추론·외부 호출·재시도·제품 수정은 없다.

## 결과

| 실행 | 판정 | 상대 SIM초 | 프로세스 벽시계초 | 발행 명령 | 외부 모델 호출 |
|---|---|---:|---:|---:|---:|
| 단독 | timeout / MAX_TICKS | 179.950 | 302.358 | 463 | 0 |
| 공동 | failure / ALL_ACTORS_FINISHED | 1.050 | 5.097 | 0 | 0 |

- 단독: 소비된 349개 제어 결정은 모두 approach이며, 파지·운반·하역으로 넘어가지 않았다.
  B의 독립 명령 분석은 44.65~179.55 SIM초의 servo3 508↔500 교대 270명령을 확인했다.
  이는 발행 명령의 비수렴 증거이며 실제 관절 상태나 정확한 영상 오차 원인을 뜻하지 않는다.
  마지막 350번째 완료 결과는 종료 시 회수됐고 소비·명령으로 이어지지 않았다.
- 공동: 0.05초 독립 동의 → 0.1초 첫 worker 두 개 오류 → 0.2초 권한 해제 → 1.05초 actor 종료다.
  첫 r1 예외는 `ValueError: RGB outside saved approach support`이며 coarse 특징 지원 조건 실패다.
  `_fail`이 이 예외를 두 참가자 감사 행에 복제하므로 r3의 구체적 예외까지 동일하다고 단정하지 않는다.
  r3도 error 완료한 사실은 별도 worker 기록으로 확인된다. lane/payload와 wheel heading 중 무엇이
  미해결이었는지는 저장되지 않았으며 재추론으로 보충하지 않았다. 학습 stage predictor 진입 전 실패다.
- 두 실행 모두 stale 거절 0. 단독 소비 시 최대 SIM 나이 0.45초, 제출→소비 1.420463초로
  각각 1초/2초 제한 이내다. 공동의 짧은 종료 시간은 성능 향상이나 성공이 아니다.
- referee에서 목표 물체 평면 변위는 모두 0, 상승은 약 1e-13m 수치 수준, weld는 계속 OFF다.
  runtime의 공동 `completed`는 제어 루프 종료이지 물리적 성공 판정이 아니다.

## 독립 검증과 보존

- 원본 inventory 1,488개 전수 SHA256, source 508개, 설정·receipt를 대조했다. 재검사도 불변이었다.
- 저장 결과와 독립 evaluator 재채점의 모든 반환 필드가 일치했다. runtime terminal clock,
  backend timestamp, evaluator snapshot이 일치하고 primary/close 오류가 없다.
- 전체 309.298초, 계획된 solo 1회와 joint 1회만 존재한다. outer/child 종료 영수증은 exit 0,
  reap/groupgone이며 A가 세 process group의 부재도 별도 확인했다.
- 원본 JPEG 1,452개를 디코딩·해시·캡처 기록에 결박했다. own RGB·자기 명령·기억 격리,
  동일 SIM/generation의 TOP 공유와 캐시 원시각 보존에서 이 범위의 결함은 발견하지 못했다.
- 원본 base64 문자열과 실제 worker wrapper는 별도 저장되지 않았다. JPEG 재구성과 frozen 생성 소스를
  대조한 범위다. 동일 SIM은 동시 벽시계 노출을 뜻하지 않으며 2초 guard는 첫 촬영이 아니라 제출부터다.
  외부 모델 호출은 0이지만 일반 `model_calls`는 null이다. 원래 D 실행의 로컬 skill 계산과 구분한다.

## 영상·환경

- 저장 solo 영상은 전체 디코딩 후 721프레임 중 9개를 직접 검토했고, joint는 전 6프레임을 검토했다.
  원본 actor/skill JPEG 5개도 직접 확인했다. 영상은 단독 접근 후 정체, 공동 무명령 종료와 일치한다.
  단독 영상 전체 프레임을 육안으로 확인했다고 주장하지 않는다.
- video는 별도 감사 카메라이며 beam z 평가 오버레이가 들어간다. actor 입력 JPEG와 분리돼 있다.
  영상의 `fixture_setup`은 갱신되지 않은 고정 라벨이다. 영상 시각은 1.3초 settling을 포함한 절대 SIM,
  결과 시각은 이를 뺀 상대 SIM이다. 시작·끝 강제 캡처 때문에 영상 길이는 180.25/1.50초다.
- 정적 준비 XML과 실제 XML은 **바이트 동일하지 않다**. 차이는 seed로 정해진 비가시·충돌 OFF
  legacy placeholder 9개 요소의 속성뿐이다. 카메라·로봇·실제 beam/box·지도는 동일하다.
  초기 로봇 평면 차이는 settling 약 3.42e-6m, 목표 물체 평면 차이는 0이다.

## TensorBoard

공통 TensorBoard에 `0922-RGB로컬-단독`, `0922-RGB로컬-공동`이 등록됐다.
A는 snapshot manifest와 원본 연결 및 live 12개 scalar 값을 독립 재확인했다.
D가 기존 Chrome 강 탭에서 고정 지표 6개·HParams 4열·Text 본문/영상 링크·영상 재생을 확인하고
저장된 고정 URL로 화면을 복원했다. A의 UI 직접 검증으로 바꿔 표현하지 않는다.
공유 viewer를 재시작하거나 기존 결과를 재변환하지 않았다. 상세 URL·수치·UI 증거는 package의
`dashboard-verified.json`과 `a3-dashboard-readback.json`에 있다.

## 판단 경계와 남은 문제

이는 고정 역할·기존 노출 환경의 개발용 연결 진단 두 건이다. 통신 유무 비교, 통신 효용,
held-out 일반화, 로컬 계산 속도 개선에 대한 결론은 내릴 수 없다.
제어기의 기본 운반 성공이 아직 입증되지 않았으므로 live LLM pilot은 계속 NO-GO다.
별도 provider/실행 예산 승인도 이 감사로 대체하지 않는다.

다음 진단이 승인된다면 먼저 필요한 증거는 공동 coarse 실패 reason/특징과 단독 접근 오차·분기다.
현재 권한으로 재실행·재추론·파라미터 수정은 하지 않았다. 승인된 두 실행은 모두 소진됐다.
결과는 로컬 보관이며 원격 백업이나 Drive 업로드가 아니다. PR 병합도 하지 않았다.

## 감사 자료

패키지 `a3-postrun-independent-review-8c1b64a/`에 core/input/visual 보고서와 검토 코드,
표본 프레임, B 보조 분석, D dashboard 확인서, 종합 판정, 출처 및 해시 목록을 바이트 보존해 모았다.
`file-hashes.json`은 자신을 제외한 패키지 전 파일을 검증한다. 원본 D 자료는 복사본과 구분해 보존한다.
systematic-debugging 절차에 따라 관측 사실·원인 추정·기록 한계를 분리했다.
