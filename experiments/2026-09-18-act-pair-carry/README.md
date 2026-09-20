# ACT loaded pair-carry feasibility — 2026-09-18

이 작은 데이터·경량 ACT 구성은 교사의 일부 공동 운반을 학습했지만, 현재 코드의 대체 성능에는 못 미쳤다. **운반에 진입한 3개 조건에서 교사 3/3, ACT seed18 1/3, seed19 0/3**이다. 네 번째 조건은 세 정책 모두 기존 접근 단계에서 중단돼 ACT 운반을 평가하지 못했다. 더 강한 일반화나 교사를 넘는 복구 능력은 입증하지 못했다.

| 조건 | 교사 | ACT seed18 | ACT seed19 |
|---|---|---|---|
| open-minus | 성공 | 성공 | 실패 |
| open-plus | 성공 | 실패 | 실패 |
| turn-minus | 성공 | 실패 | 실패 |
| turn-plus | 운반 진입 전 중단 | 운반 진입 전 중단 | 운반 진입 전 중단 |
| 합계 | 3/4 | 1/4 | 0/4 |
| 운반 진입 후 엄격한 빔 성공 | 3/3 | 1/3 | 0/3 |

최종 후보는 네 조건을 세 정책으로 새로 실행한 12회다. 첫 시도의 완료·중단 결과는 `all-attempt-results.json`에 별도로 보존했으며 성공률에 섞지 않았다. 개발 2회는 원 학습 기록에서 재사용했다.

| 정책 / 조건 | 엄격한 빔 성공 | 운반 구간 SIM초 | ACT 판단 횟수 | 장애물 접촉 step | 종료 오류 |
|---|---|---:|---:|---:|---|
| 교사 / open-minus | 성공 | 23.7 | — | 0 | 없음 |
| ACT seed18 / open-minus | 성공 | 20.6 | 104 | 0 | 없음 |
| ACT seed19 / open-minus | 실패 | 97.8 | 490 | 131229 | RuntimeError: visual placement confirmation failed |
| 교사 / open-plus | 성공 | 22.7 | — | 0 | 없음 |
| ACT seed18 / open-plus | 실패 | 20.4 | 103 | 0 | 없음 |
| ACT seed19 / open-plus | 실패 | 178.4 | 893 | 0 | RuntimeError: ACT carry RGB attachment guard stopped |
| 교사 / turn-minus | 성공 | 83.6 | — | 0 | 없음 |
| ACT seed18 / turn-minus | 실패 | 180.0 | 900 | 988 | RuntimeError: ACT carry decision budget exhausted |
| ACT seed19 / turn-minus | 실패 | 180.0 | 900 | 439366 | RuntimeError: ACT carry decision budget exhausted |
| 교사 / turn-plus | 실패 | — | — | 0 | RuntimeError: fine docking outside saved support |
| ACT seed18 / turn-plus | 실패 | — | 0 | 0 | RuntimeError: fine docking outside saved support |
| ACT seed19 / turn-plus | 실패 | — | 0 | 0 | RuntimeError: fine docking outside saved support |

실패할 때까지 걸린 시간은 속도 개선으로 해석하지 않는다. 장애물 접촉 step은 접촉 사건 개수가 아니라 물리 적분 step 수다. 명령 수·실제 벽시계 시간·외부 모델 호출 비용은 diagnostics.json에 별도로 기록했다.


## 범위

기존 3대 로봇 실행기에서 두 로봇이 빔을 실제로 집어 든 뒤의 운반 행동만 ACT로 바꿨다. 계획 합의, 자원 예약, 접근·파지, 단독 박스 운반, 내려놓기·영상 확인은 기존 경로다. 계획은 기존 LLM의 확정 계획을 재생했다. 이번 실행에서 새 외부 LLM 호출은 0건이다. 실행기의 cost_usd 필드는 null(미기록)이며, 로컬 컴퓨팅 비용은 계산하지 않았다. 새 LLM 계획부터 끝까지의 독립 E2E 검증으로 해석하지 않는다.

학생 입력은 각 로봇의 자기 RGB, 원래 공용 top RGB, 고정 목표·경로·역할, 자기 직전 발행 명령이다. 실시간 정답 위치·관절·접촉·referee 결과는 입력 또는 행동 보정에 쓰지 않는다. 모델에서 사용하는 environment_state라는 키는 고정 작업 정보와 발행 명령 8개 숫자의 구현상 이름이며 측정 상태가 아니다. 두 로봇이 같은 가중치를 사용하지만 개별 입력과 명령 이력을 갖는다. 로봇 간 협상·통신을 새로 학습한 실험은 아니며 기존 합의·동기화 규칙 아래 로컬 운반 행동을 학습했다.

## 모델과 데이터

기존 성공 교사 F1–F6 전체 궤적 6개를 학습에 사용하고 새 교사 실행 2개를 개발 세트로 분리했다. 로봇별 시퀀스를 합쳐 학습 3,590개 관측, 개발 1,084개 관측이다. 학습 종료 표적은 12개, 개발 종료 표적은 4개뿐이다. 가까운 프레임을 무작위로 나눠 평가하지 않았다. 실패 복구 궤적을 새로 수집한 실험은 아니다.

LeRobot ACT 고정 버전, ImageNet ResNet18 동결, 두 영상 128px, transformer dim 64, chunk 8이다. 매 0.2 SIM초 새로운 영상을 보고 첫 행동을 실행한다. 원 ACT 논문의 전체 크기나 학습량을 재현한 실험은 아니다. 두 seed를 각각 8,000 updates 학습했으며 개발 점수로 checkpoint를 선택했다. 한 로봇이라도 완료를 예측하면 둘 다 정지하고, 둘 모두 3회 연속 완료를 예측해야 기존 방출 경로에 진입한다. 자기 RGB의 물체 부착 연속성 확인과 기존 동기화도 유지한다.

## 고정 비교

open / shared_crossing 각각 로봇 출발점을 두 방향으로 옮긴 4개 조건에 교사, ACT seed18, ACT seed19를 실행했다. 네 조건 × 세 정책 = 최종 12회이며 ACT 8회는 서로 다른 8개 환경이 아니라 같은 4개 조건을 두 학습 seed로 실행한 것이다. 개발 2회와 최종 12회를 모두 남겼다. 각 코호트 전에 소스·프로토콜·두 모델의 해시를 고정했다. 첫 코호트에서 실행기와 ACT 출력 범위 불일치를 발견해 중단하고, 허용 명령 범위만 고친 뒤 같은 가중치로 최종 12회를 모두 다시 실행했다. 학습이나 완료 기준은 바꾸지 않았다. AMENDMENT.md와 첫 시도 기록을 함께 보존한다.

공통 물리 성공 조건은 빔과 박스가 실제로 들려 이동하고, 운반 중 샘플링된 바닥 비접촉, 목표 영역 안에 전체 모서리 포함, 방출 후 바닥 지지·로봇 비접촉·1초 안정, weld 0이다. 보고하는 엄격한 전체 성공은 여기에 프로토콜 완료, 실행 오류 없음, 장애물 접촉 0을 추가한다. 제어기의 완료 선언과 사후 실제 성공을 구분한다. 카메라/FOV와 외관은 고정했다. local_contact_fine 프로파일은 모든 비교에 동일하며 실물 보정 결과가 아니다.

## 해석 한계

출발점 변화를 기존 접근·파지가 흡수하므로 ACT가 시작하는 실제 하중 상태 변화는 작다. 개발 실행의 빔 위치 변화는 원 학습 궤적 대비 약 0.13mm / 1.73mm였다. 새 출발점 성과를 새로운 복합 복구 상태 또는 새 지형 일반화로 주장하지 않는다. 최종 성공한 경우끼리만 속도를 비교하며 실패까지 걸린 시간을 속도 개선으로 세지 않는다. 입력 화면 검토에서 빔이 자기 카메라 대부분을 가리는 것을 확인했다. 공용 RGB의 128px 축소, 동결된 영상 특징, 작은 데이터 규모는 각각 가능한 제약이며 영향 분리를 위한 실험 없이 단일 원인으로 단정하지 않는다.

raw 로그·영상·모델은 로컬 outputs에만 보존한다. GitHub의 결과 기록과 해시는 raw 백업이 아니다. Drive는 사용하지 않았다.

## 검증과 원본

실행 소스 `92b984e9d518fbdd878d0e1260b527cb5aad6a1a`, 학습 소스 `e987d4a7f5bea5d08fd4100f23be6fce9b1251f0`. audit.json은 원본 데이터 추출 재현, 모든 최종 이미지·요청·문맥·발행 명령, 표본 native ACT 추론, 사후 referee 결과 재계산과 초기 상태 비교를 기록한다. 최종 12회와 원 개발 2회의 입력 경계를 확인했다. 영상은 원본 SIM 시각 OCR로 정렬했고 원본·합성 영상 전체 디코딩, 선택 프레임의 시각 대조, 화면 검토를 수행했다.

영상·전체 raw·가중치는 `outputs/experiment-v2` 및 `outputs/video-v2`에 있고 Git에는 기록과 해시만 있다. dataset-manifest.json.gz에는 원본 이미지 경로·해시와 학습 표적이 있으며 이미지 자체는 없다. 새 clone만으로 실험을 재현하려면 이 raw 데이터와 기존 grasp/stage 모델이 추가로 필요하다.

## 실패와 가능성의 구분

- open-minus의 seed18은 빔 운반 20.6 SIM초로 교사 23.7초보다 약 13% 짧았고, 전체 임무와 엄격한 실제 판정도 통과했다. 성공한 한 조건의 비교다. 최종 목표 경계 여유는 교사 27.2mm, ACT 11.2mm로 같지 않으므로 동일한 최종 자세의 속도 우위라고 해석하지 않는다.
- open-plus의 seed18은 프로토콜을 완료했지만 빔 모서리가 목표 구역을 21.7mm 벗어났다. 완료 선언을 실제 성공으로 세지 않았다.
- seed19는 open-minus에서 잘못 배치했고, open-plus에서는 893번 판단 중 660번 한 로봇만 완료를 예측했다. 후자는 237.2 SIM초부터 바닥 접촉이 나타났고 자기 RGB 보호 장치가 중단했다. 물체를 목표 영역에 떨어뜨린 것을 성공으로 세지 않았다.
- turn-minus의 두 ACT는 각각 900회 판단 / 180 SIM초 한도를 소진했다. 두 경우 모두 운반 구간의 바닥 비접촉 샘플은 유지했지만 목표 정렬을 끝내지 못했다. 장애물 접촉은 seed18 988 step, seed19 439,366 step이었다. seed18은 목표 부근까지 이동했으나 자세를 맞추지 못했고, seed19는 다른 경로로 벗어나는 모습이 영상에 나타난다. 완료 분류기의 기준만 낮출 문제는 아니다.
- turn-plus의 세 정책은 모두 APPROACH에서 `fine docking outside saved support`로 중단됐다. ACT 추론은 한 번도 실행되지 않았다. 전체 임무 지표에는 실패로 남기되 로컬 ACT 운반 성공률의 분모에서는 분리했다.

최종 시험의 빔 운반 시작 위치는 원 학습 궤적에서 정상 공간 약 0.28–0.40mm, 회전 조건 약 3.00mm만 달랐다. 따라서 새로운 출발점을 썼다는 이유로 큰 하중 상태 변화에 일반화했다고 주장할 수 없다. 정책 간에는 같은 초기 상태와 같은 운반 진입 RGB를 확인했다.

두 seed의 개발 점수는 0.06466 / 0.06248로 비슷하고 seed19가 약간 낮았지만 실제 실행은 더 나빴다. 개발용 교사 프레임에서의 오차만으로 closed-loop 성공을 예측하기 어렵다. 이번 결과를 보고 가중치·체크포인트·완료 기준을 다시 고르지는 않았다.

다음 실험은 현재 ACT가 만든 이탈 상태에서 교사가 회전·정렬·재접근하는 복구 시연, 목표 근처의 성공/실패 경계 자료, 한쪽만 완료한 상황을 우선 수집하는 것이 타당하다. 개발용 학생 실행 성공률을 별도로 사용하고 최종 시험은 다시 분리해야 한다. 해상도 증가는 카메라 배치/FOV를 유지한 별도 비교로 검사할 수 있다. 이는 후속 제안이며 이번에 수행한 실험이 아니다.

## 검증 범위

- 수정된 실행 코드: 로컬 888 tests + 160 subtests 통과, 선택 의존성 1 test skipped. 실제 native ACT forward/backward/save/readback 별도 통과.
- 최종 감사: 14개 기록(새 최종 12회 + 재사용 개발 2회)의 referee 결과 재계산; ACT 이미지 참조 13,560건·실제 요청 6,780건·문맥·발행 명령 확인; native ACT 결정 164개 표본 재현. 이미지 참조 수는 중복된 공용 영상 참조를 포함한다.
- 기존 계획 요청·접근·파지·단독 운반·교사 운반의 저장 입력/결정 재생도 14개 기록 모두 통과했다. 구체적인 범위와 수는 existing-stage-input-audits.json에 있다.
- 입력 제한, 카메라/FOV 유지, weld OFF 확인. 이 감사 통과는 로봇의 임무 성공이나 실물 성능을 뜻하지 않는다.
- 비교 영상 두 개는 원본 전체 프레임의 시각 인식, 원본/합성 영상 전체 디코딩, 선택 프레임 시각 24건 대조를 통과했다. 시작/중간/마지막 총 8개 화면에서 레이아웃·시각·결과·정지 화면 표시를 직접 확인했다.

## 재실행

원 실행 소스는 92b984e9d518fbdd878d0e1260b527cb5aad6a1a, 학습 소스는 e987d4a7f5bea5d08fd4100f23be6fce9b1251f0이다. 원본 Mac에서 아래처럼 별도 출력 디렉터리를 사용한다. 다른 호스트에는 raw 데이터와 기존 grasp/stage 모델을 먼저 제공하고 프로토콜의 로컬 원본 경로를 맞춰야 한다. 새 실행의 실제 SHA는 보고서에 별도로 기록된다. 이 결과가 실물 또는 다른 플랫폼에서도 재현된다는 검증은 하지 않았다.

```sh
SIM_PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python
ACT_PY=/Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python
MJ_PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython
"$SIM_PY" scripts/ugrp_session.py run carry-act-reproduction -- \
  "$SIM_PY" scripts/run_carry_act_experiment.py \
  --out outputs/carry-act-reproduction-NEW-ID \
  --act-python "$ACT_PY" --mjpython "$MJ_PY" \
  --grasp /Users/changmin/projects/ugrp-worktrees/dispatch-e2e/outputs/dispatch-transfer-t4/models/grasp \
  --stages /Users/changmin/projects/ugrp-worktrees/dispatch-e2e/outputs/dispatch-transfer-t4/models/varied \
  --reuse-training outputs/experiment-v1
```

학습부터 새로 실행하려면 새 출력 경로에서 `--reuse-training`을 생략한다. 결과 감사는 scripts/audit_carry_act_experiment.py, 사후 분석은 scripts/analyze_carry_act_experiment.py, 영상 비교는 scripts/render_carry_act_comparison.py의 도움말을 따른다. 실행기는 깨끗한 Git 상태를 요구한다. 로그·모델·영상의 로컬 원본과 GitHub의 기록을 구분한다.
