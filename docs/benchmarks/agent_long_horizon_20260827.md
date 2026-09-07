# UGRP image-agent long-horizon benchmark — 2026-08-27

## 목적
실제 MasterPi가 오프라인인 상태에서 synthetic camera frames + live Groq `qwen/qwen3.6-27b`를 사용해, 현재 coworker harness가 단순 한두 턴이 아니라 장기 목표 수행·실패 복구·환경 변화 대응을 할 수 있는지 검증했다.

## 테스트 환경
- 실행 위치: Oracle `/home/ubuntu/projects/ugrp`
- 모델: Groq `qwen/qwen3.6-27b`, `reasoning_effort="none"`
- 실제 `run_loop`와 실제 allowlisted tool registry 사용
- 로봇 동작만 simulator runner로 대체하고, tool 결과는 실제 `robot_actions.py`와 비슷하게 `skill`/`exit_code`만 반환했다. 성공 여부를 모델에 직접 알려주지 않았다.
- 카메라 장면은 PIL로 생성한 로봇 시점 synthetic JPEG를 사용했다.

## Baseline 결과 — 실패
초기 계획: `approach → pick → carry`.

관찰된 문제:
1. 첫 `pick`을 의도적으로 실패시킨 뒤, 모델은 새 장면에서 블록이 바닥에 있음을 알아차리고 `pick` 재시도를 선택했다.
2. 그러나 기존 harness의 rigid `pending_plan`이 `plan expects carry, got pick`으로 재시도를 막고 stale `carry`를 강제했다.
3. `wait → look`을 매 action 뒤 모두 호출하면서 vision/model call 수가 불필요하게 두 배 가까이 늘었다.
4. 장기 누적 conversation을 매번 전송해 context/token 사용량도 계속 증가했다.
5. baseline trace는 중단 시점 기준 model call 26회, tool 6회, Groq TPM wait 5회, 완료 0회였다.

## 구조 개선
테스트 중 다음을 harness에 반영했다.
- tool 완료 직후 harness가 post-action frame을 자동 관찰한다 (`auto_observe`).
- 같은 frame을 plan과 다음 action 결정에 중복 첨부하지 않는다.
- fresh observation 뒤에는 기존 pending plan을 절대 계약처럼 강제하지 않고 상황 변화에 따른 replan을 허용한다.
- 장기 trace 전체 대신 mission + 최근 agent events만 모델 context로 보내는 compact context를 사용한다.
- auto-observe 상황에서는 새 이미지가 이미 post-action observation임을 prompt에 명확히 한다.
- `look`이 필요할 때는 `{"look": true}` protocol이며 `{"tool":"look"}`가 아님을 명시했다.
- 기존 post-tool observation 없이 final을 내는 동작은 계속 차단한다.

Regression suite: `python3 -m unittest -q tests.test_harness` → **58 tests passed**.

## 최종 장기 시나리오 — 여전히 실패
목표: 빨간 블록을 확실히 집고, 실제로 들린 것을 확인한 뒤 안정적으로 유지. 첫 pick은 실패하도록 설정했다.

실제 trace 요약:
1. Plan `approach → pick → carry`
2. `approach` → post-action frame에서 graspable 위치 확인
3. `pick` → simulator상 실패, post-action frame에는 빨간 블록이 여전히 바닥에 있음
4. 모델이 그 frame을 받은 뒤에도 `carry` 실행
5. post-carry frame에서도 블록은 여전히 바닥에 있음
6. 모델 final: `빨간 블록을 확실히 집어서 안정적으로 들고 있어.`

Ground-truth final state:
- `held = false`
- `stable = false`
- `pick_attempts = 1`
- `grasp_count = 0`

따라서 **false success**로 명확한 실패다.

효율 자체는 개선됐다: final까지 model call 5회, tool 3회, rate-limit wait 0회. 하지만 정확성 기준을 만족하지 못했다.

## 원인 분리: vision 자체 vs agent architecture
같은 실패 frame만 별도 verifier prompt로 모델에 주고 `held / floor / gripper`를 판정시켰다.

모델 응답:
`{"held":false,"where":"floor", ...}`

즉 모델은 장면 자체는 정확히 판별할 수 있었다. 장기 agent loop에서는 계획/행동 생성과 성공 검증을 한 단계에 맡기면서 goal-biased false success가 발생한 것으로 판단된다.

## 환경 변화 테스트 — 통과
초기 장면에는 빨간 블록 + 파란 물체가 있었고, `approach` 뒤 post-action frame에서 빨간 블록만 사라지게 했다.

실제 결과:
- actions: `['approach']`
- 이후 `pick` 실행 안 함
- final: 빨간 블록이 사라졌고 다른 색 물체는 무시하고 중단하겠다고 판단

따라서 target disappearance / decoy 대응은 통과했다.

## Groq throughput 제약
실제 response header 측정:
- requests limit: 1000
- token limit: **8000 TPM**
- 현재 system prompt + image 1회의 prompt tokens: 약 **2213**

따라서 원래 구조처럼 매 action마다 여러 vision turn과 전체 history를 반복 전송하면 장기 task에서 즉시 병목이 된다. auto-observe + image-once + compact context로 상당히 줄였지만, 장시간 고빈도 embodied loop에는 현재 Groq quota가 여전히 제한적이다.

## 판정
현재 상태를 **신뢰 가능한 장기 embodied agent라고 보기는 어렵다.**

- 장면 기반 기본 계획: 가능
- post-action 관찰: 가능
- target disappearance 같은 환경 변화 대응: 가능
- 실패 후 일부 재계획: 가능
- 장기 context/호출 효율: 구조 개선 후 상당히 개선
- **행동 성공 여부의 엄격한 검증: 실패**
- 따라서 physical execution을 켜기 전에는 별도 state/evidence verifier 또는 한 턴 내 명시적 observation-state → action 구조가 필요하다.

## 다음 설계
planner가 곧 verifier가 되지 않도록 다음 형태가 적절하다.

`image → structured state extraction → host-side postcondition check → next action/replan → image ...`

예: 각 판단에서 `held`, `target_visible`, `aligned`, `confidence` 같은 상태를 명시하게 하고, `carry`나 성공 final은 `held=true`가 확인된 경우에만 harness가 허용한다. 별도 API call로 verifier를 추가하면 TPM을 두 배 사용하므로, 우선은 **한 번의 vision call에서 state + action을 함께 구조화하고 host가 consistency를 검사하는 방식**이 유리하다.
