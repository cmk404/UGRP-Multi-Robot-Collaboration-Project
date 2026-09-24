# 봉 접근 동기화: 저장 계획 재생 (2026-09-23)

## 질문과 조건

사용자가 `dispatch` 실행에서 본 봉 양 끝 로봇의 선행 접근을 재현하고 수정했다. 비교 조건은 `open` 지도, seed 11, `local_contact_fine`, weld OFF, 동일한 저장 계획 입력(SHA-256 `1b6268175d15d410b6bb9319db665f09dcc6a22e9690bfe6b94d629a123596d9`), 동일한 RGB grasp/stage 모델이다. 두 후보 재생은 모델 호출 없이 저장 계획을 사용했다. 컨트롤러에는 자기 RGB·공용 TOP RGB·자기 명령 이력만 제공했고 좌표·접촉·성공 판정은 사후 평가에만 사용했다. 원본 실행은 MuJoCo 창을 띄운 사용자 세션이고 후보는 headless 재생이므로 wall time을 성능 비교로 해석하지 않는다.

| 실행 / 고정 소스 SHA | 봉 접근 | 봉 부분 물리 판정 | 전체 결과 | wall / 명령 / 모델 호출 |
| --- | --- | --- | --- | --- |
| 원본 `20260923-170300-dispatch-dff8ade9` / `b41ec686889be2e48f408cb4de16eb432a7f49bd` | 물리 r3 첫 전진 SIM 5.2s, r1 12.2s; 최대 전후 차이 0.376301m | 미완료 | 뷰어 종료로 `process_failed`, `protocol_complete=false` | 187.605s / 347 / 12 |
| v3 `20260923-pair-approach-sync-b055042` / `b05504268e340cf1a905afc19ef19299e8012c4d` | 양측 첫 전진 12.2s, 최대 차이 0.114139m | 미완료 | fine RGB 지원 범위 이탈, `process_failed` | 88.823s / 323 / 0 |
| v3 반복 `20260923-pair-approach-sync-b055042-repeat` / 동일 SHA | 명령·물리 궤적과 같은 지원 범위 이탈 | 미완료 | `process_failed` | 99.821s / 323 / 0 |
| v4 `20260923-pair-approach-sync-c3178cf` / `c3178cf7b82de2e86dced5841f3f6ab3e0f75e73` | 양측 첫 전진 12.2s, 최대 차이 0.114139m; 7/7 fine 단계·dock 완료 | `cargo.beam.physical_success=true` | 상자 운반 중 360s 제한으로 `process_failed`; 전체 `physical_success=false`, `protocol_complete=false` | 362.095s / 919 / 0 |

v3은 두 로봇의 RGB 전진 간격을 제한해 한쪽이 먼저 달리지 않게 했지만 fine 단계의 일시적인 영상 지원 범위 이탈에서 정지했다. v4는 **봉 접근 경로에만** 양쪽 0명령 0.25s와 추가 정지 1.3s를 허용하고 새 RGB에서 양쪽 유효 판정을 두 번 확인한 뒤 같은 단계로 복귀한다. 지속적인 무효 판정은 중단한다. 실제 재생에서는 forward 단계 frame 166에서 이 경로가 작동했고 frame 167·168 모두 유효했다. 저장 모델의 지원 임계값이나 다른 실행기의 기본 실패 동작은 바꾸지 않았다. v3 번들은 불변이며 v4 새 번들 SHA-256은 `16cf1053ff07372cb86921d52dfebb7f2a938e6d275d9923f4f11320eb158379`이다.

v4의 봉은 사후 평가에서 1.68043m 이동, 최대 0.07776m 상승, 목적 슬롯 안에서 안정적으로 지지된 것으로 기록됐다. 운반 구간 237개 표본 중 바닥 접촉은 0이고 weld 사용은 0이다. 봉과 **어느 로봇이든** 접촉한 표본은 237/237이며, 이것만으로 양측의 동시 접촉을 증명하지는 않는다. [정지·재관측 장면](recovery-overview.jpg), [운반 장면](carry-overview.jpg), [배치 장면](placement-overview.jpg)은 검토용 정지 이미지다. 실시간 정답은 제어에 사용하지 않았다.

전체 실행에서는 상자가 목적 슬롯에 놓이지 않았고 `RELEASE` 단계에서 wall budget이 소진됐다. `simultaneous_loaded_motion_s=0`은 봉과 상자의 동시 적재 이동이 없었다는 전체 임무 지표다. 이번 한 조건의 **봉 부분 성공 1/1**을 전체 임무 성공률이나 다른 맵의 일반화로 확대하지 않는다. 원본의 모델 비용은 기록이 `null`이라 비교할 수 없으며 저장 계획 재생은 호출 비용이 발생하지 않았다.

## 보존과 검증

각 실행의 원본은 `records.json`에 절대 위치와 `manifest.json`, `result.json`, 발행 명령, referee 기록, 확정 계획의 SHA-256으로 연결했다. 원본은 로컬 worktree에 보존돼 있으며 이 PR이나 TensorBoard가 원본 전체의 원격 백업은 아니다. 실행 관리 manifest에는 소스·카탈로그·환경·입력·결과 해시가 있고 v4 workflow version은 `1.2.0`이다. v4 실행 전 소스는 커밋·고정됐고 종료 뒤 변경 없음이 확인됐다.

v4 소스에서 전체 오프라인 검사 1,869 passed, 8 skipped, 205 subtests passed; 번들 현재 검증과 기존 번들 불변 검사가 통과했다. 물리 재생은 `scripts.sim_cli workflow run dispatch-skills`로 실행했고 원본과 실패 사례도 보존했다. 기본 체크아웃의 새 TensorBoard 스냅샷 `outputs/tensorboard/0923-pair-approach-v4-physical`에는 네 실행이 들어 있다. 원본 해시·이벤트 scalar/text/HParams 로딩·기존 6006 서버와 6009 영상 응답을 확인했다. TensorBoard의 `evaluation/reported_success=0`은 **전체 실행 실패**이고 봉 부분 성공은 원본 `result.json`의 `evaluation.cargo.beam`에서 확인한다. Images는 현재 변환기 범위 때문에 0개이며 원본 RGB는 실행 폴더에 있다.
