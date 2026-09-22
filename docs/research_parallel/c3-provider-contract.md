# C3 · 기존 프록시 상한 감사와 차단된 파일럿 계획

2026-09-22. 출발 `357e1f2e66dcae20c54f669711308ed18cf03966` (#98),
후속 `codex/c3-provider-bound-contract`. PR base는 `codex/simulation-live-view`다.
기존 #103/main·개인 프록시·계정·서비스·ACT·native UI·장면 코드는 바꾸지 않는다.
이번 범위는 읽기 전용 조사·오프라인 transport 검사·계획 패킷이며 실제 모델/물리 실행이 아니다.

## 결론: ready=false 유지

현재 로컬 `127.0.0.1:8391`은 저장소의 CLIProxyAPI 설치 예제가 아니라 별도의 개인
`gemini_subscription_proxy.py`다. 환경변수 endpoint override는 없었다. `/health`,
`/v1/models` GET 2개만 HTTP 200을 확인했다. 세 등록 별칭은
`gemini-3.1-pro-preview`, `gemini-3.7-flash`, `gemini-3.8-flash`다.
GET는 로컬 등록 목록일 뿐 로그인·할당량·upstream 모델 존재·생성·비용의 증거가 아니다.

조사한 **디스크 소스** SHA-256:
`7d4c4e8c2addf9e0dc159de6ba0f4ec4374f876d915a6bc0752d3fd18db3e556`.
이는 실행 프로세스가 로딩한 bytes의 원격/메모리 attestation이 아니다. 원문이나 개인
인증 설정은 이 저장소에 복사하지 않는다. 개인 서비스는 시작·종료·수정하지 않았다.

| 경계 | 안전하게 추출한 정적 근거 | 파일럿 제약 |
|---|---|---|
| 입력 | 이미지·텍스트·추가 wrapper를 포함한 count/bound 계약을 찾지 못함 | bytes/문자수 추정을 토큰 hard bound로 사용하지 않음 |
| 출력 | `normalize_max_tokens(512, Flash)` → 8192; Pro → 12288 | 요청한 512 상한이 보존되지 않음 |
| 호출 수 | `fetch_generate_content`는 특정 429에서 최대 2 upstream 시도 | C의 client 1회가 upstream 1회임을 증명하지 못함 |
| 시간 | upstream 시도별 timeout 300초, retry 대기 가능 | C 30초 timeout/cancel이 원격 작업 종료라는 증거가 아님 |
| 사용량 | 누락 usage → 0, completion은 candidates만, thoughts 별도 미계수 | C가 이미 0으로 바뀐 값의 실제 미측정을 복원할 수 없음 |
| reasoning | Flash 요청 `none` → upstream `LOW` | reasoning 없음으로 해석하거나 숨은 비용을 0으로 두지 않음 |

허용 상수와 순수 함수만 AST로 추출해 출력 변환을 계산했다. 프록시 전체 모듈은 import하지
않았으며, credential 파일/로그 조회·생성 POST·인증 시험·새 provider·서비스 변경은 없었다.
합성 usage `prompt=100,candidate=10,thought=500,total=610`에서 completion=10만 나오는 것도
확인했다. 이것은 변환 함수의 재현이며 실제 모델의 토큰 측정치가 아니다.

`ProviderSettings.readiness()`의 두 blocker와 D의 callback 주입 거부를 유지한다.
현재 source에는 등록 가능한 검증 capability가 없으므로 resolver를 새로 연결하지 않는다.
증거 문자열/JSON 선언만으로 `ready=true`로 승격하지 않는다. 공통 proxy 클라이언트와
개인 어댑터 수정을 맡거나 구현하지 않았으며, 별도 사용자 범위 승인이 필요하다.

## D 인계: 계획이며 실행 manifest가 아님

[c3-provider-packet.json](c3-provider-packet.json)은 명시적으로 `ready=false`,
`execution_admission=false`, `approved_model_calls=0`, `selected_model=null`이다.
6개 자리는 D와 합의한 `normal-supported-mission`/`public-workspace-contention` ×
none/structured/natural이다. seed·최종 controller와 환경 증거는 D/A가 별도 고정하며
경쟁 역할을 로봇에게 사전 할당하지 않는다. 등록 별칭 중 현재 C 기본값을 **미승인 후보**로만 써
정확한 policy hash를 보여주며 모델을 자동 선택하지 않는다.

- 후보 정책: `gemini-3.7-flash`, 출력 요청 512, temperature .2, reasoning `none`,
  HTTP timeout 30초, 현재 원본 RGB 2장 + 과거 원본 1쌍(최대 4장).
- 예산 **제안**: 로봇별 최대 12 client 판단(시행당36), 입력120,000·출력12,000,
  요청별 입력10,000·출력512, 180 SIM초/600 wall초. 6회 합계216 client 호출,
  입력720,000·출력72,000, job wall4,200초. 동시 시행1개, actor별 in-flight1개/전체3개,
  자동 재시도0. 이 수치는 승인·provider 보장·충분한 성능의 증거가 아니다.
- 토큰 cap은 admission의 상한일 뿐 계산식/추정 bound가 아니다. 실제 proven bound가
  요청별 cap을 넘거나 없으면 전송 전에 거부해야 한다. monetary cap은 아직 `null`이며
  요금/구독/할당량·원격 잔여 작업을 포함하는 비용 합의 없이는 실행하지 않는다.
- prompt/planner/serializer/async/runtime/clock/tool/skill/evaluator 해시는 출발 소스 기준으로
  패킷에 있다. B/D 변경을 합친 **최종 SHA에서 전부 재생성**해야 하며 이 출발 hash를
  새 소스로 자동 전이하지 않는다. 미래 실제 이미지 bytes/hash는 아직 없어 `null`이다.
  실행 때 원래 capture 시각·이미지 SHA·실제 serialized body를 별도로 보존해야 한다.
- D `provider_settings`/`policy_manifest`/clock API와 C production 코드는 변경하지 않는다.
  기존 physical/observation·E0·freshness·예산 gate도 그대로다. 첫 local solo/joint 각1회
  승인과 아직 승인되지 않은 LLM 6회를 섞지 않는다.

## 오프라인 검증

`tests/test_rgb_communication_provider_budget.py`는 byte transport만 주입한다. fake bound=100은
테스트 전용이며 D resolver에 등록하지 않는다. 실제 네트워크/모델 호출은 0이다.

- history 0/1/3쌍 각각에서 bound callback·실제 transport bytes·저장 SHA 일치 및 원본
  2/4/8 image parts 검사. 파일럿 후보는 별도로 history1에 고정한다.
- 사용량 없음은 입력·출력 예약 유지, C transport timeout은 재시도 없이 예약 유지.
- 지연/취소 요청은 slot을 계속 점유하고 다른 두 actor/단일 SIM clock은 진행한다.
  종료 뒤 돌아온 답은 동작하지 않으며 실행 결과의 미측정 예약을 소급 제거하지 않는다.
- JSON의 capability/증거 주입을 거부하고 `ready=false` 유지.
- 새 취소 테스트의 최초1건은 0.01초 wall에 archive I/O가 포함된 flaky 기대값이었다.
  실제 peer 완료 event를 기다리고 해당 테스트의 clock을 주입해 pending/drain 경계만
  검사한다. production deadline·budget은 바꾸지 않았고 기존 deadline 회귀는 그대로다.

기존 C와 합친 검사 명령:

```sh
python -m pytest -q tests/test_rgb_communication_provider_budget.py \
  tests/test_rgb_communication_planner.py tests/test_rgb_communication_async.py \
  tests/test_rgb_communication_clock.py tests/test_rgb_communication_runtime.py
```

결과는 PR의 최종 SHA CI와 함께 인계한다. 단위 fixture는 통신 효과/실제 운반 성공으로
분류하거나 TensorBoard 연구 결과로 등록하지 않는다. raw 기존 실패 자료도 수정하지 않는다.

## 실제 연결 전에 필요한 결정/근거

1. 기존 개인 프록시를 연구용 계약에 맞춰 수정·검증할 별도 범위 승인 여부.
2. 동일 모델/endpoint에서 image·text·wrapper·숨은 reasoning을 포함한 사전 hard bound,
   출력 상한 보존, retry/attempt 수, timeout/cancel 뒤 잔여 작업과 비용의 보장 근거.
3. 정확한 모델·입출력·호출·wall·금액/구독 비용 상한의 명시적 선택/승인.
4. D의 실제 물리·관측 gate, A의 새 source/config별 preflight와 이후 단일 실행자 승인.

이 결정이 없으면 그대로 차단한다. 새 provider/공개 endpoint/유료 구매로 우회하지 않는다.
