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

수정 후 로컬 전체 검사: 1,857 passed, 8 skipped, 205 subtests passed. 재실행의 실패·중단 결과는 기본 체크아웃의 TensorBoard 새 스냅샷 `outputs/tensorboard/0923-terminal-menu-live-smoke`로 변환했다. 첫 실행에는 `result.json`이 없어 변환 대상에 포함하지 않았다.

## 판정 경계

직접 확인한 것은 터미널 메뉴의 장면·속도 선택, MuJoCo 기본 창 표시·재생, 중단 시 기록 최종화·소유 프로세스 정리다. 자연어 지시의 외부 LLM 응답, 로봇 행동·운반 성공, 다른 지도에서의 실행, 학습, 성능 비교는 이번 실행으로 확인하지 않았다. 미리보기 중단을 시뮬레이션 완주나 연구 성공으로 간주하지 않는다.
