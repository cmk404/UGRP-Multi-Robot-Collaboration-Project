# review-r6 — Codex 6차 검토(`codex-194-r6`)와 통합 #229 발견 수정 (2026-09-26)

- **상태:** 수정 완료. 오프라인 스모크를 **새 버전 [v5](../v5/README.md)로 재실행했다.** 재질문 규칙이 채널 3조건의 SIM trace를 바꾸기 때문이다. v1~v4는 그대로 둔다.
- **검토 기준:** Codex 6차 검토 `d13dccee`. 판정은 not-ready(실제 LLM 파일럿 보류)이고 P1 1건·P2 1건이다. 통합 PR #229(이슈 #222 코멘트)는 두 문제를 더 찾았다. 재질문 타이머가 쌓이는 문제와, 계약이 tags_v2 지도를 거부하는 문제다.
- **수정 커밋:** `8ab8f7edbc78e7cb27985dfe81dde092823e01b1` (브랜치 `kiro/zone-study-core`, PR 194). 기준은 `444603f5`(`d13dccee` + main 병합)다.
- **회귀 테스트:** `tests/test_zone_study_review_r6.py`(SIM 회계·재질문)와 `tests/test_zone_study_review_r6_paths_contract.py`(logdir·계약 v2), 합계 85개다. 600줄 규칙에 맞춰 두 파일로 나눴다. `scripts/run_ci_tests.py` `TEST_PATTERNS`에 `tests/test_zone_study_review_r6*.py`로 등록했다.
- 모델 호출·물리 실행은 0회다.

## 수정 (`8ab8f7ed` 기준 file:line)

| # | 반례 | 수정 | 회귀 테스트 |
|---|---|---|---|
| P1 (Codex) | 어댑터가 `submit()`에서 요청을 보낸 뒤 `TransportFailure(attempts=(), usage_known=False)`를 던진다. 처리기는 예약을 돌려주고 다시 던진다. 그 호출은 완료·censor 기록에 없고 SIM 비용이 0이다. HTTP 상한 1에서 호출자가 예외를 처리하고 다시 실행하면 **실제 전송 2회·장부 0회**다 | `harness/zone_event_scheduler.py:929` `_submit`: **`NotSent`(`:217`)만** "요청을 보내지 않았다"는 증명으로 받는다. 그때만 호출의 예약을 모두 돌려주고, `unsent_calls`(`:483`)에 적고, 다시 던진다. 그 밖의 예외는 `_SubmitFailed`(`:255`) 토큰이 된다. 이 호출은 `reply()` 실패와 같은 규칙으로 정산된다(`:836` `_fetch_reply`, `:852` `_failure_reply`, `stage='submit'`). 시도를 세고, SIM 비용을 내고, hold하고, 호출·censor 기록에 남는다. 사용량은 미상으로 표시한다. 루프는 계속 돈다. `KeyboardInterrupt` 같은 중단은 예약을 남기고 ledger에 `interrupted`를 적는다. `reply()`가 던진 `NotSent`는 반환되지 않는다 | `test_r6_p1_*` 11개 사례(함수 8). 반례 3종(일반 예외·`attempts=()`·부분 사용량), Codex 시나리오(상한 1, 호출자 재실행), horizon censor, `submit` 안에서 재시도 예약, 팀 상한. 경계: `NotSent`만 반환(예약 1·2개)되고 반환된 예산은 실제로 다시 쓸 수 있다, `reply()`의 `NotSent`, 중단 |
| P1 후속 (Codex 권고) | 예약만 하고 보내지 않은 재시도가 있으면 5차 규칙은 실제 1회를 2회·1.0 SIM 초로 센다 | `CallReply.sent_attempts`(`:153`)와 `TransportFailure(sent_attempts=…)`(`:213`): 보낸 요청 수를 아는 전송 계층이 밝힌다. `_reply_of`(`:798`)는 값이 있으면 예약 대신 그 수를 세고 청구한다. 쓰지 않은 예약은 반환한다. `None`이면 5차 규칙(예약한 시도를 모두 보낸 것으로 셈)을 유지한다. `_sent_count`(`:233`)는 1 이상의 정수만 받는다. 0회는 `NotSent`로만 표현한다. 보고한 시도 수보다 작으면 거절하고, 사용량이 확정이면 보고한 시도 수와 같아야 한다 | `test_r6_sent_*` 15개 사례(함수 6). 선언하면 1회·재시도 허용, 선언이 없으면 보수적으로 2회(대조군), 보고보다 큰 선언은 채움, 예약보다 큰 선언은 예산 위반, 잘못된 값 9종(0·-1·bool 2·1.5·문자열·NaN·Inf·list), 일치 규칙 |
| P2 (Codex) | logdir 안의 부모 디렉터리가 외부를 가리키는 심볼릭 링크이고 새 run은 아직 없다. 문자열 검사와 `exists()`는 통과하고 `mkdir`·writer는 외부에 쓴다 | `scripts/zone_study_report.py:310` `_run_paths`: logdir와 run 사이의 경로 요소가 링크이면 거절한다(`:343`). 끊어진 링크와 logdir 안을 가리키는 링크도 거절한다. `Path.resolve()`한 경로가 실제 logdir 안에 있어야 한다(`:347`). 중복은 NFC·casefold한 실제 경로로 비교한다(`:349`, macOS 기본 파일 시스템). 한 run이 다른 run을 포함하면 거절한다(`:361`). logdir가 파일이면 거절한다. 반환값은 실제 경로다. `write_events`는 run마다 만들기 직전에 `_still_inside`(`:368`, `:388`)로 다시 확인한다. logdir 자체가 링크인 것은 허용하고 실제 위치에 쓴다 | `test_r6_p2_*` 16개 사례(함수 11). Codex 반례, 보고서 파일도 안 씀, 링크 3종(끊어진·내부 별칭·외부), logdir 링크의 실제 경로(쓰기 포함), 중복·중첩 4종, 파일 logdir, 확인 뒤 생긴 링크(직접·`write_events` 경쟁), 링크 검사를 속여도 포함 검사가 막음, r5 동작 유지(대조군) |
| 통합 #229 | 재질문 타이머가 호출마다 하나씩 쌓여 모든 조건이 445–478 SIM s에 호출 예산 90회를 다 썼다. 오프라인 v4 채널 조건도 18/18 `budget_exhausted` | `harness/zone_event_scheduler.py:537` `arm_reask`: **로봇마다 대기 중인 재질문 타이머는 1개다.** 대기 중이면 아무것도 걸지 않고 `False`를 돌려주며 `reask_counts`에 센다. 울리면 표지를 지운다(`:1156`). 일반 `timer()`는 표지를 건드리지 않는다. 시각은 유한한 수이고 과거가 아니어야 한다. 라벨·actor를 검사한다. `REASK_POLICY = 'single_pending_own_timer.v1'`(`:57`)은 통합 우회와 같은 식별자다. `harness/zone_study_offline.py:493`이 이 API를 쓰고, 번들 ID를 `zone_study_offline_v2`(`:62`)로 올렸다 | `test_r6_reask_*` 14개 사례(함수 5). 단일 대기·actor별·일반 타이머와 분리, 잘못된 시각 7종(None·NaN·±Inf·bool·문자열·list), 라벨·actor·과거, **4조건 오프라인 시행에서 한 로봇의 재질문 간격 ≥ `idle_reask_s`**(통합 게이트 P9와 같음)·예산 거절 0·`sim_horizon`, 번들 ID |
| 통합 #229 | #194 계약이 tags_v2 지도를 거부한다(`landmarks.placement`의 `near_door_spacing_m`·`near_door_radius_m`·`door_posts`) | `harness/zone_study_contract.py:56` `CONTRACT_VERSION = …v2`. 통합 PR과 **같은 키·같은 검사**(`:641` `DOOR_POST_KEYS`, `:644` `_door_posts`, `:660` `PLACEMENT_KEYS`)다. **v1은 남긴다**(`:55`, `:658` `PLACEMENT_KEYS_V1`, `:663` `PLACEMENT_KEYS_BY_VERSION`). `registry_sha256(version)`(`:251`)·`condition_manifest(…, contract_version=)`(`:227`)·`payload_violations(…, contract_version=)`(`:1226`)·`boundary_manifest(version)`(`:1131`)가 버전을 받는다. 모르는 버전은 `ValueError`다(`:244`). v1 registry 해시 `f1ff6a49…`(v1~v4 기록)를 재현하고, v2 해시 `c9bb5556…`은 통합 브랜치가 계산한 값과 같다 | `test_r6_contract_*` 28개 사례(함수 8). tags_v2 payload 통과(v1은 계속 거절), tags_v1은 두 버전 모두 통과, **투영 해시를 다시 계산한** 변조 13종, NaN/Inf, 경계값 4종(0·빈 목록·빈 객체·키 없음), v1 해시 재현, manifest, 모르는 버전 5종 |

## 수정 전 실패 / 수정 후 통과 (`test_results.json`)

- **수정 전:** `444603f5`의 `harness`·`scripts`·`tests`·`configs`·`maps`·`sim`를 `git archive`로 /tmp에 풀고 최종 테스트 파일 2개만 올려 돌렸다. **82 failed, 3 passed**다. 통과 3건은 대조군이다. 선언 없는 전송 계층의 보수적 2회 계산(5차 규칙 유지), logdir 링크 자체의 실제 경로 쓰기, r5 경로 검사 동작이다.
  - Codex P1 시나리오는 `assert 2 == 1`로 실패한다. 실제 전송은 2회인데 장부는 비어 있다. 다른 P1 반례는 예외가 루프 밖으로 나와 실패한다.
  - P2 반례는 `DID NOT RAISE ValueError`로 실패한다. 외부에 쓰였다.
  - 재질문 반례는 한 로봇의 재질문 간격 3.5–6.4 s(< 10 s)로 실패한다.
  - 계약 반례는 tags_v2 payload에서 `ContractViolation`으로 실패한다.
- **수정 후:** 85/85 통과. 관련 13개 모듈 **709 passed**다(r5 기준 606 + r6 85 + `test_simulation_workflow_manager.py` 18). tensorboard를 막은 실행(CI `offline-regressions` 흉내)은 83 passed·2 skipped다. 건너뛴 2건은 실제 이벤트 파일 쓰기이며 경로 검사는 tensorboard 없이 돈다.
- **CI 수집:** 패턴이 217개 모듈을 고르고 r6 두 파일을 포함한다. 두 파일에서 85개가 수집된다. 테스트 `test_r6_both_files_of_this_round_are_collected_by_the_ci_patterns`가 이것을 스스로 확인한다.

## 변이 검사 (`mutations.json`)

/tmp 사본에서 수정 조각 38개를 하나씩 되돌렸다. **38/38이 r6 테스트를 실패시켰다.** 원복하면 85/85 통과한다. 첫 실행에서 2개가 살아남았다. `door_posts` 닫힘 검사와 파일 logdir 거절이다. 원인은 테스트 약점이었다. 변조 키 `live_pose_m`은 금지 키 검사가 먼저 잡았고, 파일 logdir는 `mkdir`이 대신 실패했다. 중립 이름 키(`bracket_depth_m`)와 `_run_paths` 직접 검사로 테스트를 보강한 뒤 다시 돌렸다.

| 영역 | 변이 수 | 예 |
|---|---:|---|
| P1 `submit` 정산 | 6 | 모든 예외 반환(옛 규칙), `NotSent` 미반환·삼킴, `_SubmitFailed`를 `reply()`로 조회, stage 표지, 중단 기록 |
| `sent_attempts` | 7 | 선언 무시, 실패·응답에서 선언 소실, 0·bool·보고보다 작은 값·확정 사용량 불일치 허용 |
| 재질문 | 8 | 대기 중에도 추가, 표지 안 지움, 모든 타이머가 표지 지움, 비유한·라벨·과거 허용, 오프라인 옛 `timer()`, 번들 ID |
| P2 경로 | 8 | 링크 검사·포함 검사 제거, 대소문자 구분 중복, 중첩 허용, 쓰기 직전 재확인 제거·약화, 문자열 경로 반환, 파일 logdir |
| 계약 v2 | 9 | v1 확장, v2 미확장(통합 버그), registry 해시 버전 무시, 모르는 버전 허용, door_posts 타입·닫힘, manifest 누락, 버전 전달 누락 |

## 스모크 영향 — v5를 만든 이유

- **P1·sent:** fixture 전송 계층(`harness/zone_study_offline.py` `_FixtureTransport`)은 `submit()`에서 예외를 내지 않고, 사용량 확정·시도 1개 응답만 낸다. 그래서 이 두 수정은 스모크 출력을 바꾸지 않는다.
- **P2:** 스모크 실행기는 TensorBoard를 쓰지 않는다. 보고서 변환만 영향을 받으며, v5 변환은 새 스냅샷 디렉터리에 했다.
- **계약 v2:** 스모크 시나리오 6개는 모두 `*_tags_v1` 지도를 쓴다. v1·v2 모두 통과하므로 입력 payload는 같다. 바뀌는 것은 `provenance.registry_sha256`뿐이다.
- **재질문:** 채널 3조건의 SIM trace·호출 수·종료 이유가 바뀐다. 그래서 새 버전 v5로 실행했다. `no_comm`·`reference_R` 12개 시행은 v4와 SIM trace·요청 입력 해시가 같다(`../v5/control_runs.json`).

## 파일 길이 (600줄·80줄 규칙)

- `harness/zone_event_scheduler.py` 1224줄(r5 1066), `harness/zone_study_contract.py` 1511줄(r5 1453), `harness/zone_study_offline.py` 947줄이다. 세 파일 모두 이미 600줄을 넘는 패키지 파일이다(main에서 scheduler 534, contract 988). 이번 라운드에 나누지 않은 이유는 두 가지다. PR #229가 이 브랜치 위에 쌓여 같은 파일을 고치고 있어 분할하면 충돌이 커진다. 수정 범위도 네 반례로 정해져 있다. 분할은 #194·#229 병합 뒤 별도 작업으로 한다.
- 새로 만들거나 고친 함수는 모두 80줄 이하다: `_run_paths` 58, `_reply_of` 38(설명 포함), `_submit` 30, `arm_reask` 27, `_failure_reply` 25, `_sent_count` 21, `_on_call_start` 49(`_submit`을 따로 빼서 73에서 줄임).
- 테스트는 600줄 규칙에 맞춰 두 파일(421줄·289줄)로 나눴다.

## 남은 범위

- 실제 LLM 어댑터는 아직 연결하지 않았다. 어댑터는 두 가지를 지켜야 한다. 보내기 전 실패만 `NotSent`로 던지고, 보낸 요청 수를 알면 `sent_attempts`로 밝혀야 한다. 둘 다 파일럿에서 실제 공급자 응답으로 확인해야 한다. `NotSent`를 잘못 던지는 어댑터는 과소 기록을 만들 수 있다. 계약상 "증명"은 어댑터 코드 검토로만 확인된다.
- 통합 브랜치(#229)는 이제 `EventScheduler.arm_reask`·`REASK_POLICY`와 계약 v2를 이 PR에서 가져올 수 있다. 지역 우회와 계약 패치를 지우는 것은 #229 쪽 작업이다.
- TensorBoard 공용 logdir의 병합 HParams 열 제한은 그대로다.

## 파일

| 파일 | 내용 |
|---|---|
| `test_results.json` | r6 85개의 수정 전·후 결과(사례별 메시지), tensorboard 없는 실행, 관련 13개 모듈 709 통과, CI 수집 |
| `mutations.json` | 변이 38개의 원문·변이 문자열·결과 |

JUnit 원본(`/tmp/kiro-r6-*.xml`)은 임시 파일이다. 해시는 `test_results.json`에 남겼다.

## 참고 자료

- 논문: 이번 라운드를 위해 새로 찾은 논문은 없다. 미상 사용량을 결측 표지와 하한으로 따로 두는 근거는 5차와 같다. Donald B. Rubin, "Inference and missing data", *Biometrika* 63(3), 1976. https://doi.org/10.1093/biomet/63.3.581
- OSS
  - CPython 3.12 표준 라이브러리 `pathlib`(`Path.resolve`, `Path.is_relative_to`, `Path.is_symlink`), `os.path.lexists`, `unicodedata.normalize`, `math.isfinite` (PSF License). 경로 검사는 새 의존성 없이 표준 라이브러리로 했다.
  - pytest 9.1.1 (MIT), https://github.com/pytest-dev/pytest — `tmp_path`, `monkeypatch`, `parametrize`, `importorskip`
  - TensorBoard 2.21.0 (Apache-2.0) — 기존 `write_events`를 그대로 썼다
- 내부 모듈·PR
  - `harness/zone_event_scheduler.py`(`AttemptBudget`, `PendingCall.reserve`, `TransportFailure`, `_reply_of`), `harness/zone_sim_cost.py`(`call_cost`, `censored_call_record`), `harness/zone_study_contract.py`, `harness/zone_study_offline.py`, `scripts/zone_study_report.py`, `scripts/run_ci_tests.py`
  - PR #229 `kiro/zone-study-integration`: `harness/zone_study_integration.py` `_arm_reask`·`REASK_POLICY`(재질문 규칙과 식별자를 그대로 채택), `harness/zone_study_contract.py`의 계약 v2 패치(`DOOR_POST_KEYS`·`_door_posts`·`PLACEMENT_KEYS`를 같은 키·같은 검사로 채택), `tests/test_zone_study_integration_seams.py`의 계약 v2 변조 사례(8종을 투영 해시 재계산과 함께 확장)
- 문서·웹
  - Codex 6차 검토 `codex-194-r6`, 이슈 #222 코멘트(2026-09-26 11:34), `docs/zone_sim_cost.md`, `docs/zone_study_contract.md`, `docs/zone_study_metrics.md`, `AGENTS.md`
  - Python `pathlib` 문서(`resolve`는 심볼릭 링크를 풀어 절대 경로를 만든다): https://docs.python.org/3.12/library/pathlib.html
- 채택하지 않은 대안
  - **`submit()` 예외를 모두 반환(기존):** 보낸 요청이 장부에서 사라진다. 이것이 반례였다.
  - **`submit()` 예외를 모두 다시 던지고 예약만 유지:** 호출 기록·SIM 비용이 여전히 0이고 루프가 멈춘다. 호출 기록에 남기려면 결국 실패 호출로 정산해야 한다.
  - **예외 종류(`ConnectionError` 등)로 전송 여부 추측:** 연결 오류도 요청을 쓴 뒤 날 수 있다. 추측 대신 어댑터가 명시하는 `NotSent`만 믿는다.
  - **`sent_attempts` 없이 예약 계약만 유지:** Codex가 지적한 과대 청구가 남는다. 선언은 선택이며, 선언이 없으면 5차 규칙을 그대로 쓴다.
  - **경로를 문자열 `os.path.realpath`로만 비교:** `resolve()`와 같지만 끊어진 링크나 logdir 안을 가리키는 별칭을 걸러내지 못한다. 요소별 링크 거절을 더했다.
  - **`O_NOFOLLOW`/`openat`으로 경쟁 조건까지 완전히 막기:** TensorBoard `EventFileWriter`가 경로 문자열을 받아 직접 열기 때문에 적용할 수 없다. 만들기 직전 재확인으로 창을 줄였고, 남은 경쟁 창은 공용 logdir에 쓰기 권한이 있는 로컬 사용자로 한정된다.
  - **재질문 타이머를 새 행동마다 뒤로 미루기(debounce):** 규칙이 통합 우회와 달라지고, 계속 행동하는 로봇은 재질문을 받지 못한다. 통합과 같은 "대기 1개, 시각 유지"를 택했다.
  - **계약 v1을 그 자리에서 넓히기:** v1~v4 기록의 registry 해시와 v1 경계 판정이 바뀐다. 버전별 키 표로 v1을 남겼다.
