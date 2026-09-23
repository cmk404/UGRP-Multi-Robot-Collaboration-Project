# 2026-09-22 — D3 로컬 RGB 물리 연결 진단 2회

승인된 단독 box·공동 beam deterministic replay를 각각 1회, 로컬에서 순차 실행했다.
두 대상 모두 운반 완료는 false다. 단독은 `timeout/MAX_TICKS`, 공동은
`failure/ALL_ACTORS_FINISHED`다. **통신 효과·LLM/ACT 모델 비교·미관측 환경
일반화 실험이 아니라, 노출된 개발 장면에서의 연결 진단 실패 2건**이다.
외부 LLM 호출·입출력 토큰·통신 메시지는 0이며 재시도하지 않았다.

## 동결 소스와 범위

- 실행 SHA `8c1b64a95ca9dc3196e80ea6d47ea7c98d2776f5`, `codex/d3-local-validation`.
- 공통 #98 `357e1f2e66dcae20c54f669711308ed18cf03966` 위 A `c81b7d4`,
  B `1d46f7a`, C `ca6e099`, root `46c4897`, D `8545801`의 소유 커밋을 보존한 조합이다.
- config SHA256 `ae941c5ecdcc78d537f6d52499ab1f553ee91e26835f02c8f4bce8fc386952e7`.
- canonical manifest SHA256 `87d5e51d072cee9bb3df840db716b15cedd37076e3f6d086c3dbb41a050f916a`.
- A3 v2 offline 인증 18/18, 인증 파일 SHA256
  `6d348b8b06630ac09fbe47446616eff0fbf50a47302fe56895b64088d2ba6c1c`.
  인증의 config-without-certificate digest는
  `0e86c79efe53c2b765c992f12f680ab4c70620d9ef2f6fbae18199aeec3c0334`다.
  이 인증은 실행 전 경계 감사이지 물리 성공이나 live admission이 아니다.
- Python 3.12.13, macOS-27.2-arm64, 기존 `.venv-sim-worker-mac` 사용.
  모델·학습 보고서·기준 이미지 등 참조 파일 13개는 기존 로컬 자산을 read-only로
  재사용하고 새 모델/환경을 복사하지 않았다.
- `dispatch_open`, seed 11, regression/development_connection_smoke. 고정 RGB/FOV,
  weld OFF, 자기 RGB·공용 TOP·자기 발행 명령 경계를 유지한다. 이전 학습 맵 출처 unknown.
- 시행당 SIM 180초/wall 600초/명령 6000개, 작업 전체 1800초. 실제 launcher는
  study allocation 1750초, outer deadline 1780초+단일 cleanup 10초+기록 여유 10초다.
  각 trial은 deadline 595초+cleanup 5초이며, 기존 소유 프로세스 회수 경로를 쓴다.
- ACT/native/console 소유자들의 종료 확인 뒤 지정 PID 부재를 직접 확인하고 시작했다.
  기존 공유 TensorBoard 9285/9291/9293은 변경하거나 종료하지 않았다.

## 두 시행 전체 결과

| 지표 | 단독 box | 공동 beam |
|---|---:|---:|
| canonical outcome | timeout / MAX_TICKS | failure / ALL_ACTORS_FINISHED |
| 원본 replay_goal_complete | false | false |
| full mission_complete | false | false |
| child process wall, 초 | 302.358123 | 5.096612 |
| runtime wall, 초 | 298.765120 | 2.586017 |
| 검증된 최종 SIM, 초 | 179.950000 | 1.050000 |
| 발행 저수준 명령 | 463 | 0 |
| planner 결정 수 | 182 | 5 |
| high-level action | 1 | 2 |
| 외부 모델 호출 / 입력 토큰 / 출력 토큰 | 0 / 0 / 0 | 0 / 0 / 0 |
| 통신 메시지 / 메시지 bytes | 0 / 0 | 0 / 0 |
| actor 성공 주장 / false-finish 판정 | 없음 / false | 없음 / false |

전체 launcher wall은 309.298014초다. 두 child·study·outer는 exit0로 끝났지만
이는 파이프라인 정상 종료이지 로봇 성공이 아니다. 단독의 `timeout`은 SIM tick 한도이며
부모 wall timeout은 false다. 공동의 runtime `completed`도 모든 actor의
`cannot_continue` 종료를 뜻하고, 물리 완료는 false다.

두 clock terminal은 supervisor가 확인한 실제 SIM과 evaluator snapshot이 같으며
원본 재채점이 일치한다. D2의 `.8/.75` 모순 원본은 수정하거나 재분류하지 않았다.
적용되지 않는 외부 모델 응답 시간과 미측정 `model_calls/message_tokens`는 null로 남겼다.
오프라인 planner 응답 시간은 모델 응답 시간으로 합산하지 않는다.

## 회수와 보존

완료 inventory 1,488개 파일 전부 재해시하고 inventory 밖의 추가 원본이 없음을 확인했다.
별도 `artifact-finalization.json`과 launcher receipt를 검증했으며,
outer PGID 27214, solo 27225, joint 29640의 소멸도 직접 확인했다.

- raw root: `/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/d3-local-replay-8c1b64a`
- input root: `/Users/changmin/.codex/worktrees/cf5f/ugrp/outputs/d3-local-study-inputs-v3`
- inventory SHA256 `a7e591c1d0e29f7a0ed347932729e736c5d5fe05f1d884952c9ce2bddc3d0a5e`
- finalization receipt SHA256 `9823b07eb37340e662294634943af886ef5b309c11c52236e08834880c94f890`
- outer receipt SHA256 `976e874b27aa704af7a74331f7dd918e7ee158cc18f9cbe5821d2e80c679f155`

영상·RGB·전체 로그 약 127 MiB는 로컬 원본이다. Git의 기록/해시는 raw 원격 백업이 아니다.
UGRP 예외에 따라 Google Drive 조회·업로드·재시도는 하지 않았다.
실행 예산 2회는 소진됐고 새 물리·학습·LLM·재시도는 없다.

## 검증과 과거 후보

최종 실행 소스는 offline 134 modules, **1569 passed, 7 skipped, 198 subtests**를 통과했다.
JUnit 원본 `outputs/d3-integrated-8c1b64a-tests.xml` SHA256
`2744abe32968060017e0a4695dc185a846a23e65d99215db589729d9fec21b0f`.
실행 전 PR104 D8545801의 CI 12/12와 통합 branch run35711353991의 6개 job SUCCESS를 확인했다.

`systematic-debugging` 절차로 process signal/reap/단일 cleanup grace/최종 해시 receipt의
경계를 먼저 재현·수정·오프라인 검증했다. A가 NO-GO로 판정한 `755dca3`의 launch/cleanup
signal 누수와 `de69eb3`의 cleanup grace 중복 후보 및 증거는 별도 outputs에 보존한다.
이 후보에서는 실제 물리 실행을 하지 않았고 최종 결과의 실행 SHA와 섞지 않는다.

## 독립 사후 감사와 대시보드

B의 [독립 로그 분석](b3-review/REPORT.md)은 1,488개 raw 해시 및 skill 입력 JPEG
704개의 파일/캡처 해시·SIM 시각을 전수 대조했다. 이번 두 시행에서
`RGB_WORKER_REJECTED`는 0이다. 제한 완화 없이 단독의 제출→소비 최대 1.420463초,
이미지 SIM 나이 최대 0.45초였으며 고정 wall 2초/SIM 1초 안이다.

- 단독: 계산 350회 returned, 349회 소비, 마지막 1회는 정상 close에서 회수했다.
  발행 명령 463개 모두 approach였고 44.65~179.55 SIM초의 마지막 270개는
  servo3 508↔500 교대였다. 이는 접근 비수렴의 명령 증거다. 영상 오차/특징/선택 분기가
  해당 로그에 남지 않아 가림·영상 흔들림·보정 문제 중 어느 것이 원인인지는 확정하지 않는다.
- 공동: 독립 동의는 0.05 SIM초에 완료됐다. 0.1초에 제출된 첫 worker 2개가
  약 37.6/40.2ms 만에 error로 끝났고 0.2초에 작업을 해제했다.
  supervisor 원문의 첫 r1 예외는 `ValueError: RGB outside saved approach support`다.
  이는 coarse 영상 특징의 `ok=false`이며 lane/payload 또는 wheel heading의 미해결 중
  개별 reason은 보존되지 않았다. **학습된 stage predictor 호출 전**의 실패다.
  `_fail`이 첫 예외를 참여자 둘에게 복제하므로 두 SKILL_FAILED를 서로 독립적인
  동일 상세 예외 2개로 세지 않는다. r3도 error로 끝난 것은 확인됐지만 상세 원인은 미확정이다.
- 새 추론·시뮬레이션 없이 원본과 동결 소스만 비교했다. 캐시의 인과적 속도 향상 크기는
  통제된 before/after cohort가 없으므로 주장하지 않는다.

A의 [최종 독립 감사](a3-review/REPORT.md)는 source 508개, raw 1,488개와 원본 JPEG
1,452개를 전수 대조했고 원본 재채점의 모든 반환 필드·종료 clock이 일치했다.
허용 입력/자기 기억/자기 명령 격리에서 이번 범위의 결함은 발견하지 못했다.
원본 base64 문자열과 실제 worker wrapper는 별도 보존되지 않아 JPEG 재구성과 동결 생성
소스를 대조한 범위다. 같은 SIM은 같은 wall 시각 촬영을 뜻하지 않으며 2초 guard는 제출부터다.

저장 영상 solo 721프레임은 전체 디코딩 후 9개를 육안 검토하고 joint는 6프레임 전부,
actor/skill JPEG 5개도 직접 확인했다. 단독 접근 후 정체와 공동 무명령 종료가 원본과 일치한다.
영상 카메라·beam z 오버레이는 평가용이며 actor 입력과 분리된다. 영상의 `fixture_setup`은
갱신되지 않은 고정 라벨이고, 영상 시각은 1.3초 settling을 포함한 절대 SIM이다.
결과 시각은 이를 뺀 상대 SIM이며 시작·끝 강제 캡처 때문에 영상 길이는 180.25/1.50초다.

정적 준비 XML과 실제 XML은 바이트 동일하지 않다. 차이는 비가시·충돌 OFF legacy placeholder
9개 요소 속성이고 실제 카메라·로봇·beam/box·지도는 동일했다. 초기 로봇 평면 차이는
settling 약 3.42e-6m, 대상 물체 평면 차이는 0이었다. A의 최종 판정은 유효 실패 2건,
진단 목표 0/2이며 live LLM pilot NO-GO다.

새 snapshot은 공통 logdir의 `0922-RGB로컬-단독`, `0922-RGB로컬-공동`이다.
기존 manifest에서 같은 source가 없음을 확인하고 1회만 변환했다. 원본·기존 snapshot·archive는 보존했다.
각 19 scalar/7 text, image 0, warnings 0이며 EventAccumulator와 live6006 수치가 일치한다.
영상 두 개는 기존 media6009에서 200, `/raw/<id>`의 Range는 206을 확인했다.
별도 HTML 요약/상시 서버/자동 감시는 만들지 않았다. 변환 뒤 raw 1,488개 해시도 일치했다.

성공·유효성·명령·외부 호출·process wall·planner 결정 6개 카드의 고정 URL과
HParams 4열 설정은 `dashboard.json` 및 기본 체크아웃 `outputs/tensorboard-view.json`의
`rgb_communication_local_replay_20260922`에 저장한다. HParams `wall_s`는 runtime 시간이다.
공통 experiment 메타데이터의 누락 열을 임의 덮어쓰지 않으며 필요한 6개 수치는 Time Series에서 본다.

**실제 화면 검증은 D가 수행했다.** 기존 Chrome 강 탭 `963999421`에서 두 run과
고정 6개 카드 수치를 확인하고 HParams의 outcome/runtime wall/commands/success 네 열을
재적용했다. Text의 measurements·termination 본문과 원본 영상 링크 2개도 표시됐다.
공동 1.5초 영상은 끝까지, 단독 180.25초 영상은 재생과 173.385초 후반 표본을 확인했다.
영상 이동 뒤 HParams를 다시 적용하고 저장된 Time Series URL로 돌아와 표시를 유지했다.
단독 전 구간을 D가 직접 연속 재생 검토했다고 주장하지 않는다. UI 도구의 Chrome content export는
지원되지 않아 별도 페이지 덤프는 없으며, 실제 AX/스크린샷 검증과 범위는 dashboard 기록에 남긴다.
기존 임시 viewport 제약과 축소 배율은 기본 화면 크기·100%로 복원했다.

LLM pilot은 모델 미선택·승인 호출0·금액 상한null·ready=false로 유지한다.
provider hard input/output/call/time/cost bound, 물리 capability, 최종 live gate가 해결되기 전
실제 호출은 금지다. 세 독립 equal LLM의 통신 효과라는 본 연구 질문에 이 결과를 대신 답하지 않는다.

## 기록 패키지와 PR 경계

이 폴더에는 주요 원본 JSON/전체 raw 해시와 A/B 감사의 핵심 문서를 byte-exact 보존한다.
감사의 전체 패키지(표본 PNG/상세 worker별 전개 포함)는 D의
`outputs/d3-local-review-8c1b64a/a3-review`, `b3-review`에도 보존한다.
`copy-provenance.json`의 포함/미포함 경로를 구분한다. 여기의 감사 file-hashes는 **전체 로컬
감사 패키지** 목록이므로 일부 파일은 Git 기록 폴더가 아니라 그 로컬 패키지에 있다.
`inputs/`는 감사용 원본 사본이지 이동 가능한 실행 캡슐이 아니다. 원래 상대 evidence 참조와
절대 모델 참조는 원래 input root에서 검증했으며 이 기록 폴더를 새 실행 대상으로 쓰지 않는다.

D 제품 변경은 PR104의 소유 브랜치에 있고 실행 통합 SHA와 결과 기록 커밋은 별개다.
사후 감사와 export가 끝난 뒤에만 D 브랜치로 돌아와 이 기록을 추가했다.
PR은 사용자 확인·명시적 승인 전 병합하지 않으며 기본 main이나 다른 작업의 소스를 갱신하지 않았다.
