# 실험 인덱스

- [2026-09-25 두 로봇 접근 동시성](2026-09-24-pair-coarse-concurrency/protocol.md): v53 기본 OFF 후보. 동료 방향 보정 중 가능한 전진과 RGB·실제 port 명령 연결 기록, 물리 A/B 미실행.
- [2026-09-25 정지 후 RGB 재관측 후보](2026-09-25-settled-view-recovery/README.md): v43의 정상 주행 뒤 기준 영상 무효화 실패를 재현하고, 정확한 HOLD·안정화 뒤 새 영상으로 재관측하는 v46 후보. 실제 물리 검증 전.

- [2026-09-25 후속 RGB 진단·ACT 교사 수집 검토](2026-09-25-action-act-followup/README.md): v40 물리 0/1·수정 사후 감사 통과, 남쪽 교사 물리 실패/인프라 중단/미실행 각 1건, 세 번째 학습 미시작과 새 TensorBoard 2개 실패 기록.
- [2026-09-25 계산 후 재생 확인](2026-09-25-post-run-replay/README.md): 표준 dispatch 재생 기록 ON/OFF 동기 실행 2회 모두 성공, 명령·평가 동일(SIM 132.26초), Mac MuJoCo 창 재생 끝 프레임 확인.
- [2026-09-24 rolling 접근 영상 시야 복구](2026-09-24-rolling-view-recovery/physical-protocol.md): v40의 365회 연속 약한 직접 RGB 관측·600결정 한도 실패를 근거로 만든 v43 기본 OFF 후보와 사전 물리 비교 계약. v43의 물리 결과는 아직 없다.

- [2026-09-24 실시간 명령 끊김과 빔 미세 정렬 시간](2026-09-24-realtime-stop-gap/README.md): 기존 기록 오프라인 분석. 실시간 39개에서 임대 만료 뒤 정지 감쇠 끊김 비율이 명령당 이동 감소(순위 상관 -0.88)·미세 정렬 시간(+0.79)과 연관; 새 물리 실행 없음.

- [2026-09-24 빔 미세 정렬 이득 스케줄](2026-09-24-fine-gain-schedule/README.md): 임계 경로 분석(빔 미세 정렬 51–54 SIM초), 기본 OFF 옵션·번들 v30, 저장 RGB 668건 재생 불일치 0. 물리 A/B는 사전 고정만 했고 결과 없음.
- [2026-09-24 ACT 학습·행동 개선](2026-09-24-action-act/refinement.md): 첫 8,000 update 학습과 6회 비교의 실패를 보존하고, 영상 접근·추적·ACT 학습 및 추론을 수정한 후보를 고정 비교.

- [2026-09-23 로컬 dispatch 속도·병렬 실행](2026-09-23-local-dispatch-performance/README.md): 동일 계획 headless 367→285초, 최종 MuJoCo 창331초 완주; 동시 적재 이동·중간 실패·SIM/wall 시간과 원본 해시를 구분.

현재 코드의 진입점은 [현재 상태](../docs/current_status.md)를 참고한다. 아래는 실행 SHA별 보존 기록이며 과거 실패와 후속 결과를 함께 남긴다. 필요한 ID만 골라 읽는다.

- [2026-09-23 표준 시뮬레이션 지도·로컬 실행 검증](2026-09-23-standard-simulation-refinement/README.md) — 관리 CLI에서 지도 22종 정적 검사·MuJoCo 창/영상 실행, 원본 해시와 TensorBoard 스냅샷; 학습·운반 성공 판정은 미실행.
- [2026-09-23 표준 시뮬레이션 관리·Colab 포장 검증](2026-09-23-unified-simulation-management/README.md) — 19개 실행 공통 관리·공유 장면, 회귀 1,812개·fixture 9/9·로컬 포장/회수 검증; L4·운반 재검증은 미실행.
- [2026-09-23 RGB 실행 계약 수정·4회 재생](2026-09-23-rgb-execution-contract/README.md) — 초기 해상도·공동 식별 결함 수정 후 Colab·Mac 단독/공동 모두 물리 완주 실패; 네 실패 경계·원본 해시·독립 감사 보존.
- [2026-09-23 공통 RGB 물리 완주 복구](2026-09-23-rgb-common-physical-recovery/README.md) — v1~v8 개발 후보를 같은 seed11에서 분리 재생; 단독 상자 대상은 v3~v8 성공, 공동 빔은 v8에서 회전을 통과하고 횡방향 영상 지원 범위 밖에서 파지 전 중단. 전체 임무·통신 효과는 미입증.
- [2026-09-22 native 시뮬레이션 CLI/API](2026-09-22-native-simulation/README.md) — Mac·Ubuntu 공통 설정/API, native 창·headless·RGB·reset, Linux 종료 오류 수정과 검증 한계.
- [2026-09-22 브라우저 뷰어 검증 이력 (구현 폐기)](2026-09-22-local-simulation-live/README.md) — 네 카메라·수동 이동·일시정지·초기화·종료 통합 검사; 초기 검사 실패 포함, 자율 운반 평가는 아님.
- [2026-09-22 RGB 통신 기반 최소 물리 재생](2026-09-22-rgb-communication-replay/README.md) — Colab 단독/공동 2회 회수, LLM 0회; 모두 종료·평가 clock 모순으로 invalid_artifact. 물리 성공·통신 효과 미입증.
- [2026-09-22 2+1 병렬 운반 복구](2026-09-22-parallel-transport/README.md) — 순차 잠금·도착 오판 수정, 원 ACT 전체 성공·동시 운반 영상 확인. 개발 실패 포함 5회와 영상 판정 재생 11건 보존.
- [2026-09-22 병렬 시연 추가와 ACT 재학습](2026-09-22-parallel-act-study/README.md) — 기존 시연 8개의 운반 중첩 0 확인, 추가 시연→128px/4프레임 학습→세 조건별 9회 평가. 완료 여부는 원본 study.json으로 구분.
- [2026-09-22 ACT 영상 처리 속도](2026-09-22-act-render-speed/README.md) — 관측 처리 1.586배; 전체 운반 437초 성공, 전체 명령·공통 2177장 동일. 동시 실행 수가 달라 전체 wall 비율은 분리해 해석.
- [2026-09-22 ACT 방출 연결·기준선 재검증](2026-09-22-act-release-revalidation/README.md) — 원 교사 2조건·원 ACT 성공 재현, 새 ACT의 추적 오류 해소·배치 실패 확인; 서로 다른 조건의 실패율로 합산하지 않음.

- [2026-09-21 시뮬레이션 전 환경·통신 검사](2026-09-21-cloud-preflight/README.md) — 과부하 복구 기록, 의존성·진행률·사전 차단, 새 시뮬레이션 없음.

- [2026-09-21 Colab 중계 최적화](2026-09-21-relay-performance/README.md) — 연결 재사용·4슬롯 독립 전달, 중단 기록 보존과 남은 시행 재개.

- [2026-09-21 반복 실패 측정](2026-09-21-carry-failure-estimation/README.md) — ACT 108회 고정 평가 제출, 단일 진단 성공 전제 제거. 별도 Colab RGB 비교는 17:07 KST 스냅샷 90/231건이며 미완료·오류를 구분해 보존.

- [2026-09-21 Kaggle CLI CPU 실행·결과 회수](2026-09-21-kaggle-cli-smoke/README.md) — private·인터넷 OFF, 최종 물리·카메라 데모 1/1, 앞선 setup 실패 3회 별도 보존.
- [2026-09-21 Colab CLI CPU 실행·결과 회수](2026-09-21-colab-cli-smoke/README.md) — 실제 원격 물리·카메라 데모 1/1, ZIP 회수와 전체 파일 해시 확인; 자율 운반 검증 아님.

- [2026-09-21 Jev 의미 상태 폐루프 195회](2026-09-21-jev-semantic-motion/README.md): 새 자세 Jev 5/36 → 35/36(원판정 33/36), Gemini 32/36 → 36/36. 시간 반올림 검산과 원판정 모두 보존.
- [2026-09-21 Jev 직접 이동·표현 진단](2026-09-21-jev-direct-motion/README.md): 동일 RGB 접근 9회, 저장 상태 표현 비교 72회, 공개 제어 설계 검토. 후속 폐루프 195회는 위 별도 기록에서 검증.
- [공동 출하 복구: 목적지·장애물 6조건 새 LLM 계획과 물리 E2E 성공](dispatch-adaptive-recovery-20260917/README.md) — 이전 1/6 이후의 최종 비교, 조건당 1회.
- [2026-09-21 Colab ACT 학습·CLI 복구](2026-09-21-colab-carry-training/README.md) — 8개 × 8000 updates, 50파일 해시 및 Mac native 32개 대조 통과.
- [2026-09-21 ACT 입력 해상도·이력 2×2 비교](2026-09-21-carry-input-ablation/README.md) — 8개 Colab 학습 모델 회수·Mac 검증 완료, 고정 36-run 물리 비교 진행 중.
- [2026-09-18 ACT 공동 운반 비교](2026-09-18-act-pair-carry/README.md) — 운반 진입 조건 교사 3/3, ACT 1/3·0/3, 네 번째 조건 접근 중단.

- [2026-09-17 목적지·지형 확대 E2E: 기본 A 성공, B 영상 인식 실패, 4개 지형 실행 거부](dispatch-variation-e2e-20260917/README.md)
- [2026-09-17 공동 출하 스킬 통합: 실제 새 LLM 계획부터 두 화물 방출까지](dispatch-skill-integration-20260917/README.md)

| ID | 코드 연결 | 범위 | 결과 |
|---|---|---|---|
| [three-robot-e2e-20260916](three-robot-e2e-20260916/README.md) | 실행 `d79c97d`, 결과·원본 해시·대표 영상 | 3대 계획 승인, R1/R3 운반·R2 정지 관찰; 준비 지연·보고 단절 | 최종 실제 모델 3/3·연결 fixture 2/2; 첫 협상 실패 보존, 자유로운 역할 분담 아님 |
| [research-e2e-heading-fix-20260916](research-e2e-heading-fix-20260916/README.md) | 실행 `d863e74`, 전체 결과·원본 해시 | 먼 거리 RGB 방향 보정·정지 재정렬·마지막 전 축 복구, 같은 19배치 × 두 조건 | 로컬 19/19·LLM 19/19; 중간 18/19·17/19 실패와 개발 6회 보존 |
| [research-e2e-varied-start-20260916](research-e2e-varied-start-20260916/README.md) | 실행 `fd5d68e`, 전체 결과·원본 해시 | 거리 30–70 cm·yaw ±10°·좌우 ±6 cm의 19배치, 두 조건 38회 | 로컬 15/19·LLM 15/19; 4배치 모두 파지 전 영상 정렬 실패 |
| [research-e2e-local-skills-20260916](research-e2e-local-skills-20260916/README.md) | 실행 `11da613`, 전체 결과·원본 해시 | 고정 역할·평지, RGB 제어·시연 팔, 두 LLM 단계 허가 | 최종 로컬 3/3·LLM 3/3; 첫 코호트 4/6과 개발 실패 포함 |
| [2026-09-09-markerless-n7](2026-09-09-markerless-n7/README.md) | [커밋](2026-09-09-markerless-n7/code-version.json), [소스 해시](2026-09-09-markerless-n7/source-manifest.json) | 로봇1대, 표식 없음, Gemini, 시드42~46 | 5/5; 효율 개선 필요 |
| [2026-09-10-grasp-recovery](2026-09-10-grasp-recovery/report.md) | 보고서의 학습/최종 SHA | RGB 국소 파지 복구 | 새 20/20, 기존 14/14 |
| [2026-09-10-rgb-short-approach](2026-09-10-rgb-short-approach/report.md) | 보고서의 최종 SHA | 20–30cm 직진 접근 후 파지 | 학생 19/20, 고정 주행 2/20 |
| [2026-09-10-rgb-varied-start](2026-09-10-rgb-varied-start/README.md) | 보고서의 최종 SHA | 거리·옆 오차·방향 변동 접근 | 새 29/30, 기존 20/20 |
| [2026-09-13-rgb-short-transport](2026-09-13-rgb-short-transport/README.md) | 보고서의 실행 SHA | 20cm 운반·시연 내려놓기 | 고정 10/10, 다양한 시작 6/10 |
| [2026-09-13-pair-carry-sync](2026-09-13-pair-carry-sync/README.md) | protocol 및 manifest | 고정 fixture의 운반 지연·보고 누락 | 비교군 20/26, 동기화 26/26 |
| [2026-09-13-known-map-navigation](2026-09-13-known-map-navigation/README.md) | 보고서의 실행 SHA | 정적 지도·RGB 무부하 주행 | 통행 가능 4/4 도착, 좁은 통로 2/2 거부 |
| [2026-09-13-heading-map-navigation](2026-09-13-heading-map-navigation/README.md) | 보고서의 실행 SHA | 회전 후 전진과 기존 옆걸음 비교 | 두 방식 모두 4/4 도착·2/2 거부 |
| [2026-09-14-pr-integration](2026-09-14-pr-integration/README.md) | 통합 검증 기록 | PR 9개 조합의 회귀검사 | 595 tests + 154 subtests, 기록 감사 55/55, 새 실행 9/9 예상 일치 |
| [2026-09-16-task-stage-sync](2026-09-16-task-stage-sync/README.md) | 실행 SHA와 원본 JSON | 다섯 단계 동기화·팀원 JSON 계약; 합성 프로토콜 fixture | 9/9 예상 일치, 655 tests + 154 subtests; 물리 검증 아님 |
| [2026-09-15-pair-grasp-retention](2026-09-15-pair-grasp-retention/README.md) | `cdeee5f`, 전체 진단 SHA 기록 | 접촉 수치 처리와 RGB 조기 감지·한 번 재파지 | 제자리·왕복 300초, 지형 5/5·차단 정지 1/1, 재발 시 방출·종료 |

새 실험은 별도 ID 폴더에 코드 SHA·실행 환경·설정·성공과 실패 전부·판정 기준·자동/영상 검토 범위·원본 저장 위치와 식별값을 남긴다. 소스가 달라지면 별도 후보로 구분한다. 실험 결과 파일을 추가한 커밋과 실제 실행 코드의 커밋은 다를 수 있다.

현재 raw 영상·로그는 로컬 보관이다. 이 인덱스와 해시만으로 raw 자료를 내려받거나 완전히 재현할 수는 없다. 이전 상세 기록은 `docs/`에 있고 로컬 경로를 포함할 수 있다.

- [2026-09-21 ACT/Jev Colab·Kaggle 재개](2026-09-21-cloud-continuation/README.md): 중단 원본 보존, 원격 진단·학습·기준선 재개; 완료 성능 미확정.

## 이전 실험과 실패 기록

| 단계 | 기록 | 해석 범위 |
|---|---|---|
| 초기 탐색 C1~D2 | [시행착오](../docs/navigation_trials_20260908.md) | 단일 시드41, 실패와 두 완료 기록 |
| 새 시드 검증 | [시드 검증](../docs/navigation_seed_validation_20260908.md) | 수정 전 조건별 결과 |
| 운반 수정 E 계열 | [수정 과정](../docs/navigation_generalization_repair_20260908.md) | E7 장애 중단과 E7r1 5/5를 구분 |
| 표식 제거 M 계열 | [표식 제거](../docs/markerless_blocks_20260909.md) | M4 2/5, 집기 진단과 운반 구분 |
| 표식 없는 운반 N 계열 | [26회 결과와 대조 기록](2026-09-09-markerless-trials/README.md) · [원인과 변경](../docs/markerless_improvement_trials_20260909.md) | 실패 6회 포함, N2 폐기·진단은 별도 |
| 성공 후 효율 분석 | [분석](../docs/markerless_success_analysis_20260909.md) | 접근 보정 반복과 운반 비용 |

이 표는 기존 보고서의 탐색 경로를 보완한 것으로, 모든 과거 실험의 소스·환경·원본을 현재 형식으로 이관했다는 뜻은 아니다. 이전 후보의 커밋 연결 및 raw 원격 보관은 미완료다. 원본 복구 경로는 [이슈 #3](https://github.com/kcm0127-dotcom/ugrp/issues/3)에서 추적한다.

- [2026-09-21 TensorBoard 기록 열람 검증](2026-09-21-tensorboard-review/README.md): 기존 24개 기록 변환, 이벤트·원본 해시·로컬 화면 확인. 새 로봇 실험 아님.

- [2026-09-22 로컬 연구 장면 구성 검토](2026-09-22-simulation-scenes/README.md): 기존 58개 항목 연결·Mac/Linux reset/RGB/native/기록 검사; 운반 성능 비교 아님.
