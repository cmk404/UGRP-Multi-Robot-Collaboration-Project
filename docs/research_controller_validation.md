# 연구 제어기 검증과 사용 경계

통신 연구의 기본 제어기는 같은 RGB 실행 경로로 고정한다. ACT는 독립 구성요소 비교로 취급한다. 기본 제어기의 접근/파지 실패, ACT 운반 실패, 통신 계획 실패, 실행 환경 오류를 한 성공률에 섞어 모델 순위를 만들지 않는다.

## 세 실행 조건

| 조건 | 이동 | 내려놓기 진입 | 해석 |
| --- | --- | --- | --- |
| RGB | 기존 RGB 제어기 | 기존 RGB 도착 판정 | 연구 환경 기준선 |
| ACT-guarded | ACT | ACT 완료 합의와 별도 RGB 도착 확인 | 잘못된 완료는 `premature_model_stop`으로 기록하고 방출하지 않음 |
| ACT-RGB-refined | ACT, 필요하면 RGB 최종 정렬 | RGB 도착 및 기존 방출 후 확인 | 명시적인 혼합 제어기, ACT 단독 성공으로 세지 않음 |

`--carry-act-stop-mode learned`는 이전 완료 판단의 비교 재현용이다. 기본은 `rgb_guarded`다. 한쪽 ACT가 종료를 제안하면 즉시 RGB 도착을 검사한다. `rgb_guarded`는 목표 밖의 제안을 바로 실패로 기록하며, 목표 안에서는 기존 3회 공동 합의를 유지한다. `rgb_refined`는 첫 제안 시 목적 구역 부근(최대 80px)에서 기존 RGB 최종 waypoint를 사용하고 로그에 전환을 남긴다. 두 슬롯의 합의를 기다리다 다시 출발하는 동작은 하지 않는다. 어떤 조건도 시뮬레이터 정답을 제어에 사용하지 않는다. 모든 조건은 같은 카메라·물리·weld OFF·파지·후처리 평가를 사용한다.

## 검증 순서

1. `tests/test_research_entry_stop.py`와 관련 회귀검사로 저장된 접근 실패, 부정확한 종료, 정상 도착을 재생한다. 부분 바퀴·과대 배경 마스크는 계속 거부한다.
2. `scripts/audit_carry_termination.py --predictions <model>/development-predictions.json --out <new-audit.json>`으로 에피소드 단위 조기 종료를 확인한다. 두 로봇 중 한쪽의 오판도 공동 정지를 일으키므로 프레임 평균 오차만으로 통과시키지 않는다. 오프라인 통과는 물리 성공이 아니다.
3. 알려진 실패 조건은 회귀 시험으로 별도 보존한다. 변경 코드를 커밋한 뒤 실행하고 중간에 수정하지 않는다.
4. 새 시험 조건과 모델/파지/설정 해시를 프로토콜에 먼저 고정한다. `expected_source_sha`가 실행 체크아웃과 같아야 한다. `run_matched_carry_cohort.py`는 매 실행 전후 소스·프로토콜·자료 해시를 확인한다.
5. RGB 기준선, ACT-guarded, ACT-RGB-refined를 같은 배치와 교대 순서로 전부 실행한다. 실패가 발생해도 뒤의 사례는 계속한다. 누락/디스크 부족은 미실행으로 기록한다.
6. 전체 작업 실패율과 운반 진입 이후의 조건부 실패율을 같이 확인한다. 기본 제어기로 채택하려면 해당 조건의 모든 사전 지정 사례가 물리 완료되고 충돌·허위 완료가 없어야 한다. 표본 수와 검증 지도 범위를 함께 명시한다.
   병렬 상자 스킬의 오류로 빔 운반이 중단되면 전체 작업 실패는 유지하되, 빔 운반 모델의 조건부 실패율 분모에서는 중도 중단으로 구분한다. `failed_component`와 `carry_censored_by_other_component`를 확인한다.
7. 성공과 실패의 원본, 입력 이미지, 발행 명령, 모델 응답, 평가 전용 자료, 영상 및 해시를 보존한다. 새 TensorBoard 스냅샷의 이벤트와 영상 링크, 실제 기본 화면을 확인한다.

완료 후 `scripts/assess_research_cohort.py --report <cohort>/report.json --out <new-assessment.json>`으로 원본을 다시 읽어 조건별 사용 가능 여부를 확인한다. 종료 코드만 성공하거나 영상·접촉 측정·소스가 빠진 결과는 채택하지 않는다. ACT가 호출된 경우 `audit_saved_act_inputs.py`로 모든 요청을 원본 JPEG·로봇별 인과적 기록·정적 목표와 직전 발행 명령에서 재구성하고 실제 요청 해시와 대조한다. 이 감사는 새 추론이나 시뮬레이션을 실행하지 않는다. 결과의 `qualified_for_declared_cases`는 지정된 시험 조건 안에서만 유효하다.

이 세션의 Mac 실행은 사용자의 앞선 명시적 선택에 따른다. 일반 신규 실행 대상은 프로젝트의 Colab CLI 지침을 따른다. 실행 중 프로세스는 `ugrp_session.py`로 소유하며, 종료·중단 시 해당 세션과 자식만 정리한다. raw 자료는 로컬에 있고 해시·Git push가 raw 백업을 뜻하지 않는다.

기존 아홉 번의 시험은 이미 진단에 사용했으므로 이후 성공률 추정용 새 시험으로 재사용하지 않는다. 종료 라벨이나 가중치를 바꾸면 새 후보 모델로 식별하고 개발 데이터에서 선정한 뒤 새 조건 전체를 비교한다.

`train_carry_input_act.py --termination-objective episode`는 도착 전 마지막 8개 명령의 음성 표적을 별도 표본군으로 뽑고, 개발 에피소드의 조기 공동 정지율·종료 누락률·동작 오차로 체크포인트를 선정한다. 기존 방식 재현은 `--termination-objective legacy`로 명시한다. 목적 함수가 다르면 기존 체크포인트의 학습을 그대로 이어서 처리하지 않는다. 개발 통과 여부와 새 조건의 독립 실행 결과를 모두 보고한다.

## 한 코호트 실행과 판정

깨끗한 실행 체크아웃에서 모델·프로토콜을 고정하고 아래 순서로 실행한다. 프로토콜에는 `conditions`, `test`, `controls`, `grasp`, `stages`, `asset_sha256`, `expected_source_sha`를 모두 기록한다. 이번 Mac 검증의 실제 경로와 명령은 [실험 기록](../experiments/2026-09-22-research-controller-qualification/README.md)에 남긴다. 다른 컴퓨터에서는 해당 호스트의 런타임과 검증된 자료 경로를 사용한다.

```sh
python3 scripts/ugrp_session.py run research-cohort -- \
  /absolute/path/to/sim-python scripts/run_matched_carry_cohort.py \
  --protocol /absolute/path/to/frozen-protocol.json \
  --out /absolute/path/to/new-raw-directory \
  --mjpython /absolute/path/to/mjpython \
  --act-python /absolute/path/to/reference-act-python \
  --tensorboard-dir /absolute/path/to/primary/outputs/tensorboard/new-snapshot

python3 scripts/assess_research_cohort.py \
  --report /absolute/path/to/new-raw-directory/report.json \
  --out /absolute/path/to/new-assessment.json
```

실행기의 종료 코드 0은 예정한 실행을 모두 시도했다는 뜻이다. 제어기 성공은 `conditions.<조건>.qualified_for_declared_cases`와 각 원본 물리 판정을 확인한다. 디스크 부족·런타임 실행 실패와 물리 실패를 구분하고, 미완료 원본 위에 재실행하거나 기존 실패 기록을 덮어쓰지 않는다. 조건을 바꾸면 새 프로토콜·새 디렉터리로 시작한다. 중단은 `python3 scripts/ugrp_session.py stop research-cohort`로 자신이 시작한 세션에만 적용한다.


독립 실행을 둘로 나눌 때는 전체 프로토콜을 먼저 저장하고 각 shard의 `conditions` 또는 명시적인 `trial_ids`를 분할한다. `trial_ids`는 전체 예정 실행의 중복 없는 부분집합이어야 하며 기존 교대 순서를 유지한다. 소스·모델·환경·사례·반복 수는 같아야 한다. 이 세션은 이 작업이 소유한 시뮬레이터가 동시에 최대 두 개가 되도록 직접 배치하며, 기존 코호트가 끝난 뒤 빈 자리에 다음 shard를 시작한다. `scripts/combine_matched_carry_shards.py --protocol <전체> --report <첫 보고서> --report <둘째 보고서> --out <새 통합 보고서>`는 완료 여부·동일 설정·누락·중복을 확인하고 원본 해시를 보존한다. 통합 후 다시 `assess_research_cohort.py`로 원본 판정을 검증한다. 자원을 공유하므로 실행 시간을 통제된 속도 향상으로 해석하지 않는다.

`run_matched_carry_cohort.py`는 SIGINT/TERM/HUP을 받으면 별도 프로세스 그룹의 실행 자식까지 정리하고, 중단된 시도를 `interrupted`로 기록하며 남은 사례를 미완료로 남긴다. 일반적인 모델/물리 실패는 예정한 다음 사례로 계속 진행한다. 따라서 명시적 중단과 평범한 실패의 동작을 구분한다.
