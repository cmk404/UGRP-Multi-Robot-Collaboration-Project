# ACT 진입 전 RGB 차단 수정

관련 이슈: #51, #30. 기존 원격 104회에서 ACT carry 진입은 0회였다.
54회는 preclose RGB support, 50회는 TOP box ambiguity에서 실패했다.
실패 104회와 미시도 4회는 보존한다. 이 개발 진단을 과거 성공률에 합치지 않는다.

## 원인과 변경

- 기존 전체 TOP 특징은 먼 solo 로봇의 움직임을 파지 자세의 novelty로 판정했다.
  기존 교사 RGB/정답 표적만 사용해 own 전체 + canonical TOP의 기존 파지 영역
  (pooled x=[12,20), y=[6,19))으로 새 모델을 학습한다.
  PCA, support, novelty limit 모두 해당 표현에서 다시 계산한다.
  실패 장면은 진단 입력이며 학습 입력이 아니다. 기존 전체 영상 모델은 유지한다.
- 상자 운반 시작 때 같은 색 바닥 4개와 실제 상자 1개가 후보였다.
  기존 own attachment 검사를 통과한 후 같은 arm pan의 TOP 4장을 사용한다.
  좌우 반전 이동, 시작점 복귀, 유일한 대응이 모두 확인된 후보만 최초로 연결한다.
  정지/단방향/다중 후보는 계속 거부한다.

## 사전 검증 계획

1. 원격 두 실패 case와 교사 자료를 파일별 SHA 검증 후 로컬 회수한다.
2. 원본 장면 회귀 + 먼 배경 불변성 + own/파지 영역 novelty 거부를 확인한다.
3. 소스 커밋 후 모델 refit. 새 출력 폴더를 사용한다.
4. Mac의 기존 환경에서 open-minus, shared_crossing-minus 개발 실행.
   동일 plan/seed/offset, weld OFF, unchanged camera/geometry.
5. ACT 128px·1프레임 seed 20260921의 원래 모델 해시를 검증해 사용한다.
   ACT 추론과 실제 명령 발행, 물리적 운반 성공을 각각 확인한다.
6. 개발 확인 후 최종 소스/모델을 고정한 반복 비교를 별도 실행한다.
   개별 실패는 계속 기록하며 이후 trial을 진행한다.

raw: 이 worktree의 outputs/entry-diagnosis. 로컬 보관이며 백업 주장이 아니다.
현재 진행 중인 act-cloud-resume Mac 모델 비교는 소스와 실행을 변경하지 않는다.

## 개발 검증 결과

실행 소스는 `0e2c64448a3281ac1eedf77d97c726f8e29c265c`로 고정했다.
전체 회귀 1,267 passed / 3 skipped / 191 subtests 및 해당 SHA의 원격 CI가 통과했다.
수치·모델/원본 SHA·환경·실제 입력 wire 재구성·발행 명령 대조는
`development-verification.json`에 기록한다.

| 개발 실행 | ACT 응답/발행 명령 일치 | 전체 성공 | 종료/실행 시간 |
|---|---:|---:|---|
| open-minus, 가져온 Kaggle 보정 | 0 | 실패 | APPROACH support, 55.3초 |
| shared_crossing-minus, 가져온 Kaggle 보정 | 0 | 실패 | APPROACH support, 354.3초; 상자 성공 |
| open-minus, Mac 보정 + ACT128/h1 | 1,228 / 1,228 | 실패 | TRANSIT 시간 한도, 904.0초 |
| shared_crossing-minus, Mac 보정 + ACT128/h1 | 850 / 850 | 실패 | TRANSIT 시간 한도, 904.0초; 상자 성공 |
| open-minus, Mac 보정 + RGB 기준선 | 0 / 해당 없음 | 성공 | FINISHED, 421.2초 |

가져온 stage 모델은 동일 scene/robot/beam XML에서도 Mac의 영상 배경 차이를
지지 영역 밖으로 판정했다. 렌더러 하위 구현의 정확한 원인은 미확정이다.
같은 카메라/물리 설정으로 **교사 전용 보정 자료**를 Mac에서 새로 생성했고,
교사 nominal grasp와 180개 grasp 학습 예제를 확인했다(369.6초).
실패한 학생 장면은 학습에 넣지 않았고 ACT 모델은 재학습하지 않았다.
원래 Kaggle 보정과 새 Mac 보정은 별도 폴더에 보존했다.

Mac 보정 후 두 ACT 실행 모두 공동 파지와 carry 진입을 통과했다.
2,078개 응답이 저장된 실제 발행 명령과 일치했고, 막대는 각각 1.074m/2.027m 이동했다.
한 응답 추론 중앙값은 0.0429/0.0434초다. 두 실행 모두 전체 작업 성공은 아니다.
카메라 geometry unchanged, weld steps=0이다. 첫/중간/마지막 입력의 원본 이미지 SHA와
요청 wire SHA를 검증했으며, 명령 발행을 실제 성공으로 동일시하지 않는다.
이는 진입 수리의 개발 검증이고 최종 실패율 추정용 분모에 합치지 않는다.

## 첫 반복 비교

`first-wave-protocol.json`: 기존 네 ACT 입력 조건(128/256px × 1/4프레임),
RGB 기준선, 두 plus-offset case, 각 3회로 총 30회다. 모델 seed는 20260921 하나로
고정하고 검증된 가중치를 재사용한다. 각 실행 900초, 최대 동시 2개다.
가용 공간이 8GiB 미만이면 새 trial을 시작하지 않고 미시도로 남긴다.
실패 trial은 기록하고 다음 trial을 계속한다. 같은 seed의 반복은 재현성 측정이며
독립된 환경 30개로 해석하지 않는다. 다른 seed 및 minus-offset 최종 비교는 미완료다.

원본 저장 위치: `outputs/entry-diagnosis/first-wave-30`. 완료 여부는 그 안의
`report.json`의 attempted/pending과 개별 결과로 확인한다. 프로토콜 저장은 실행 완료가 아니다.

보고/운영 보완 후 검증: 전체 1,273 passed / 3 skipped / 191 subtests,
TensorBoard·반복 실패/공간 부족 회귀 29 passed. ACT 응답·지연·명령은 실제 원본에서
집계하며 `llm_calls=0`을 로컬 ACT 호출 0회로 오인하지 않는다.
wall/decision budget 소진은 timeout으로 분류하되 원문 오류와 phase를 보존한다.
