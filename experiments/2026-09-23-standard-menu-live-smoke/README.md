# 터미널 메뉴·MuJoCo 직접 실행 확인 — 2026-09-23

## 범위와 방법

Mac의 기존 `.venv-sim-worker-mac`에서 `bash scripts/open_simulation.command`를 PTY로 실행했다. 메뉴에서 `장면 미리보기` → `act/train-open-1` → 관찰 속도 `2×`를 골랐다. MuJoCo 기본 창에 해당 장면이 열린 것을 화면에서 확인하고, Space 키로 `Paused`에서 `Running`으로 바뀌는 것을 확인한 뒤 Ctrl-C로 종료했다. 두 실행 모두 모델 호출이나 로봇 명령은 없고 사용자가 중단한 미리보기다. 속도 `2.0`과 맵 선택은 `artifacts/config.json` 및 `artifacts/scene.json`에 기록됐다.

| 실행 | 소스 SHA | 직접 관찰 및 결과 |
| --- | --- | --- |
| `20260923-154912-local-d1a42a44` | `408059bb19266025057581110a09f020f1607d87` | 창 열림·재생 확인. Ctrl-C 때 종료 신호가 겹쳐 traceback이 발생했고 `artifacts/result.json`이 누락됐다. `manifest.json`은 `interrupted`, exit 130. 실패 증거를 보존했다. |
| `20260923-160022-local-b1f7631b` | `97825fab5c47f5fbb2c308e0dd15b6fac8204e22` | 종료 신호 처리 수정 후 같은 선택으로 재실행. 창 열림·재생 확인. Ctrl-C 뒤 traceback 없이 `result.json`과 관리 manifest가 저장됐고 소유 세션·자식 PID가 사라졌다. `result.json`: `stop_reason=interrupted`, `protocol_complete=false`, `sim_s=7.78`, `wall_s=57.38`, `commands=0`, `model_calls=0`. 관리 manifest는 비정상 종료 코드 130을 `process_failed`로 표시한다. |

## 원본·해시

원본 두 디렉터리는 작업 worktree의 `outputs/simulation-runs/`에 생성했고, 동일한 이름으로 기본 체크아웃의 `/Users/changmin/projects/ugrp/outputs/simulation-runs/`에 복사해 파일별 SHA-256을 대조했다(각 28개·31개 파일). 원본은 로컬 보관이며 원격 백업이 아니다.

| 실행 | 파일 | SHA-256 |
| --- | --- | --- |
| 첫 실행 | `manifest.json` | `98146d27914cef7dd8381d6d07d4730f5a3043b530f80907e7a9599d9b923885` |
| 첫 실행 | `artifacts/config.json` | `0b4187bded5ca93bf70b1a4369e91a2752c877850745e1b30104036dc06bdb42` |
| 첫 실행 | `artifacts/scene.json` | `ab459e714cb2148846869aca9637a069e181c2602b7959ad788951a053c0b508` |
| 재실행 | `manifest.json` | `b7966e86593a155b5d5447426a6c967a19752c84cbadaf187f1d33d483f32b07` |
| 재실행 | `artifacts/result.json` | `9e2fcbda6ace846d6140d73e5b4e043ba1c8a8e66afd3c1e87a7d649dcd7936a` |
| 재실행 | `artifacts/config.json` | `0b4187bded5ca93bf70b1a4369e91a2752c877850745e1b30104036dc06bdb42` |
| 재실행 | `artifacts/scene.json` | `ab459e714cb2148846869aca9637a069e181c2602b7959ad788951a053c0b508` |

수정 후 로컬 전체 검사: 1,857 passed, 8 skipped, 205 subtests passed. 재실행의 실패·중단 결과는 기본 체크아웃의 TensorBoard 새 스냅샷 `outputs/tensorboard/0923-terminal-menu-live-smoke`로 변환했다. 첫 실행에는 `result.json`이 없어 변환 대상에 포함하지 않았다. EventAccumulator와 공유 TensorBoard 서버(기본 체크아웃의 `outputs/tensorboard`, 포트 6006)가 같은 다섯 scalar를 반환했다: `result/wall_s=57.38168`, `result/sim_s=7.78`, `result/commands=0`, `result/model_calls=0`, `claims/protocol_complete=0`. [해당 run의 Time Series](http://127.0.0.1:6006/?runFilter=%5E0923-terminal-menu-live-smoke%2F#timeseries)에서도 선택·표시를 확인했다. 이는 중단 결과이며 성공 지표가 아니다.

## 판정 경계

직접 확인한 것은 터미널 메뉴의 장면·속도 선택, MuJoCo 기본 창 표시·재생, 중단 시 기록 최종화·소유 프로세스 정리다. 미리보기 중단을 시뮬레이션 완주나 연구 성공으로 간주하지 않는다.

## 자연어 지시·모델 호출 직접 확인

같은 터미널 메뉴에서 `1. LLM 공동 계획` → `open` 지도 → 관찰 속도 `1×` → 기본 `gemini-3.8-flash` → `beam과 box를 dock_b로 옮겨`를 입력해 실제 실행했다. 소스 SHA는 `7c1bb783dc3a168414d144a107723eb9867779d3`, 실행 중 소스는 깨끗하고 불변이었다. MuJoCo 관찰 창이 열렸고, 원본 12개 모델 요청/응답 쌍과 3대 로봇의 합의 로그가 남았다. 모든 요청의 user 텍스트에는 해당 `operator_instruction`이 Unicode 이스케이프 형태로 포함됐다. 합의된 계획은 `dock_b`, beam 담당 `r1+r3`/north, box 담당 `r2`/south다. 로봇별 프로그램 생성 후 접근 단계에서 발행 명령이 기록됐다.

약 75초 후 테스트 수행자가 Ctrl-C로 중단했다. `result.json`은 `plan_committed=true`, `phase=APPROACH`, `protocol_complete=false`, `physical_success=false`, `wall_s=75.1704`, `llm_calls=12`, 입력 115,637 tokens/출력 2,997 tokens/합계 123,136 tokens를 기록했다. `issued-commands.json`은 r1 20행, r2 2행, r3 20행으로 총 42행이다. 이 중 로봇별 첫 `SETUP` 목표 스냅샷 3행을 제외한 실제 **발행 명령은 39개**이며, TensorBoard의 `result/commands`도 39다. 발행 명령은 실제 이동 성공이 아니다. 비용 필드는 `null`로 금액을 산정할 수 없다. 종료 중 ffmpeg가 exit 255/Broken pipe를 기록했지만 남은 7.5초 MP4는 ffprobe와 전체 디코딩이 통과했다. 이는 부분 영상이며 전체 실행 영상은 아니다. 관리 manifest는 exit 1/`process_failed`다. 소유 세션과 자식 프로세스는 종료됐다.

원본은 worktree의 `outputs/simulation-runs/20260923-161558-dispatch-cca0ae6c`에 있고 기본 체크아웃의 같은 `outputs/simulation-runs/`에 242파일/31,107,436바이트를 복사해 전수 SHA-256을 대조했다. 파일 목록·크기·해시 집계는 `5fb4a9baa371c6404364d1d02c5bdfa2730c509925674192222c0bb82581aab0`이다. 이는 로컬 보관이지 원격 백업이 아니다. 핵심 SHA-256: `manifest.json` `4491f03f9276b5c669a8d7b3f040fa6d2828e5c3dab36992d704c2c1e5254e57`; `artifacts/result.json` `08dc747a780eadacceffd76846e7c9ef8802ef7bc6447bb579a088130d0f99ef`; `artifacts/actor-mission.json` `6dae1f9c65919ed3e047b56f0ad6b164a71c23ac6882fc78f4d07390ec0eff6a`; `artifacts/committed-plan.json` `fa2da6d7f27d0d85dd65b4e4874912424c81c64a7480b4a1a12f0354cb1b7e07`; `artifacts/issued-commands.json` `b201f315c89ddddcb26ce6bd91cb1228686d6aae28d6b74b06b842dc12587b47`.

기본 체크아웃의 새 TensorBoard 스냅샷 `outputs/tensorboard/0923-terminal-menu-llm-smoke`는 변환 실패 0이다. EventAccumulator와 기존 포트 6006 서버에서 `evaluation/reported_success=0`, `result/wall_s=75.1704`, `result/commands=39`, `result/model_calls=12`, `result/model_latency_s=110.4104`, `claims/plan_committed=1`을 대조했다. 원본 부분 MP4의 6009 서버 Range 요청은 206이었다. 스냅샷 manifest SHA-256은 `4c2cbb30bd7d6e3a10bd2d1ae076887c23c9d593447c7575f262544f5059504f`이다.

이 실행은 자연어 입력 → 실제 모델 응답 → 계획 합의 → RGB 스킬의 초기 명령까지 확인했다. 운반 완주, 다른 지도에서의 일반화, 학습, 성공률·시간·비용 비교는 확인하지 않았다. 중단한 실행을 물리 성공으로 집계하지 않는다.
