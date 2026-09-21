# 다중 물건 계획·예약 프로토콜 파일럿

2026-09-20, 실행 소스 `7f4211e6c4b7b9aa9027ad298969437de54a25ae`.
실행 전 소스/설정 커밋과 깨끗한 작업 트리를 확인하고 전체 fixture 동안 소스를 유지했다.
관련 작업: [#50](https://github.com/kcm0127-dotcom/ugrp/issues/50), [#51](https://github.com/kcm0127-dotcom/ugrp/issues/51).

**결과:** 대표 12조건 × 로봇 배분 회전 3가지 = 36개 합성 프로토콜 실행이 모두 완료됐다.
계획 검증 → 세 로봇 ACK → 자원 예약 → 모든 담당자 완료 보고를 재생한 결과다.
물리 성공, 모델의 자발적 계획, 실제 이미지 식별이나 운반 성능의 증거가 아니다.

| 항목 | 결과 |
| --- | --- |
| 물건 수 | 1·2·3·4·5·6·8개 |
| 대표 조건 | 단독/공동/혼합, 공용 통로, 정리 선행, 임시 적치 후 최종 배송 |
| 물건 수와 작업 수 분리 | staging_three: 물건 3개, 필수 작업 5개 |
| 합성 trace | 36/36 전체 작업 완료 보고, 누락·중복 완료 0 |
| 자원 감사 | 36개 전체 trace에서 활성 예약 중복 0 |
| 파일/임무 hash | 파일 53개와 임무 12개 일치 |
| 관련 테스트 | 55 passed |
| 전체 오프라인 회귀 | 1,065 passed, 1 skipped, 184 subtests passed (54.65초) |
| 물리 장면 배치·운반·영상 검토 | 미실행 |
| 모델/API 호출·API 비용 | 0회 / $0 |

전체 fixture 출력 생성 wall 시간은 약 0.05초다. 각 trace의 `protocol_ticks`는 합성
스케줄 진행 횟수로 실제 운반 시간·SIM 시간·속도 성능이 아니다. `whole_mission_claimed`는
테스트가 생성한 보고를 집계하며 `physical_success`는 모두 `not_evaluated`다.

## 부정 조건과 검토 범위

테스트는 없는 작업·중복 작업·DAG 순환·선행 관계 제거·잘못된 역할/경로·임무 hash,
학생 과제의 정답/좌표 추가 필드, 한 명의 ACK 누락, 한쪽 완료 대기, 오래된 관측/sequence,
다른 작업 ticket, 임시 적치의 잘못된 최종 완료 집계, 공유 통로·슬롯 중복 예약을 검사한다.
계획 무효화 이후 과거 보고가 예약을 해제하지 않는 것도 확인했다.

같은 적치 슬롯에 필요한 공간이 부족해 논리 교착이 생기는 별도 테스트는 대기를 유지한다.
DAG 유효성만으로 자원 교착 부재나 물리 안전을 보장하지 않는다. 예약 인계·교착 복구는 후속이다.
두 로봇의 READY/DONE 프레임 참조는 fixture 문자열이다. 이미지 내용의 정확성·추적 일관성은
검증하지 않았고, 실행 가능한 모터 포트나 ACT 입력으로 새 논리 경로를 연결하지 않았다.

## 보존과 재현

로컬 원본: `/Users/changmin/projects/ugrp/outputs/multi-object-pilot-20260920-v1/`.
그 안에 임무 12개와 합의/계획/예약 이벤트 파일 36개가 있다. 파일별 해시는
[artifact-hashes.json](artifact-hashes.json), 모든 조건 결과는 [results.json](results.json),
소스·조건·임무 hash는 [manifest.json](manifest.json), 환경은 [environment.json](environment.json)에 보존했다.
원본은 로컬 보관이며 원격 백업으로 표현하지 않는다. PR CI는 별도 실행 증거를 14일 보관한다.
지속 서비스·시뮬레이터·워커를 시작하지 않았다.

```sh
.venv-sim-worker-mac/bin/python scripts/prepare_multi_object_pilot.py \
  --output outputs/multi-object-pilot-NEW
.venv-sim-worker-mac/bin/python scripts/run_ci_tests.py
```

출력 경로가 있거나 작업 트리가 변경된 경우 생성기는 실행을 거부한다.
후속 구현은 [파일럿 가이드](../../docs/multi_object_pilot.md)의 물리 장면·RGB 식별·단계 실행 연결 순서를 따른다.
main 병합과 원격 CI 상태는 이 로컬 결과와 별도로 PR에서 확인한다.
