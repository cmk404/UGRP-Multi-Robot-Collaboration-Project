# 표준 시뮬레이션 관리·Colab 포장 검증 — 2026-09-23

19개 연구 실행을 기존 표준 CLI의 공통 카탈로그·실행 기록으로 연결하고, 표준 설정 실행과 공동 출하/RGB backend의 장면 생성·초기화를 공유했다. 실행 번들은 `rgb-standard-dispatch-v2`로 분리하며 기존 번들 JSON을 변경하지 않았다. 모든 제어 루프를 재작성하거나 과거 물리 완주 성능을 재검증한 결과는 아니다.

## 실행 소스와 검증

- `64d09959b7066d5e2b2848d3a540a41b4bbee458`: 관리·장면 통합. 실제 CLI `workflow run stage-sync`에서 9/9 fixture가 예상과 일치하고, 목록/상세 조회·10개 결과 파일 해시를 확인했다. HOLD 조건도 포함하며 `physical_success=null`이다.
- `267ada94c60c9c74541e7d5d06be1e2771d7ce3f`: Colab 포장에 추적된 examples를 포함. 같은 커밋의 sparse source를 로컬 collector에서 실행해 9개 fixture와 관리 manifest가 든 15개 파일 ZIP을 회수하고 내부 해시 전부를 확인했다. 원격 Colab/GPU 실행은 아니다.
- 포장 수정 전 전체 회귀: **1,812 통과, 8 건너뜀, 205 subtests 통과**. 관리 CLI 최종 집중 검사 43 통과/1 건너뜀, mock 기록 격리 후 28 통과/1 건너뜀. 마지막 포장 수정·번들 해시 갱신 후 관련 10개 통과. RGB/장면 관련 집중 검사 74개 통과.
- MuJoCo 장면 비교는 legacy/contact/fine 및 ACT 미리보기의 XML·초기 qpos/qvel·카메라·접촉 설정 비교다. 무렌더 초기화 검사이며 새 운반 성능 근거가 아니다.

## 실패와 검증 경계

초기 포장 소스는 `examples/task_stage_sync/plan.json`이 누락돼 exit 1이었다. 실패 ZIP·run.json·로그를 보존하고 examples 포함 후 exit 0을 확인했다. sparse 묶음은 dashboard Python 파일 3개를 계속 제외하며, 공통 source manifest가 이를 누락으로 표시한다. 이번 fixture에서 필요하지 않은 파일이며 모든 선택적 workflow 자산의 포함을 보증하지 않는다. 모델 ZIP/외부 데이터는 해당 작업에서 명시적으로 포함·해시 확인해야 한다.

실행 중 소스 변경 없음, 기록 마무리 오류 없음. LLM 호출 0회, GPU 할당 없음, 로봇 물리 운반 재평가 없음. 성공률·운반 시간·행동 수·모델 비용의 연구 비교는 수행하지 않았다. 기존 RGB 후보의 실패 판정에 새로운 성공을 부여하지 않는다. 버전·환경·시간·모든 원본 식별값은 [verification.json](verification.json)에 있다.

## 보관

원본은 작업 worktree의 `outputs/unified-simulation-management-20260923`에 보존했고 필요한 로그·소스 압축·실패/성공 ZIP·fixture·감사 기록을 기본 프로젝트의 `/Users/changmin/projects/ugrp/outputs/unified-simulation-management-20260923`로 복사해 모든 해시를 대조했다. 이 기록과 해시는 raw 원격 백업이 아니다. Google Drive를 사용하지 않았다.

[표준 관리 사용법](../../docs/simulation_management.md), [Google AI Pro·Colab L4 검토](../../docs/colab_standard_simulation_review_20260923.md)를 참고한다. L4의 실제 NVIDIA EGL 렌더링과 단독/공동 완주 재검증은 남아 있다.
