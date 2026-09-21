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
