# RGB 실행 계약 수정 후 유한 재생 (2026-09-23)

실행 소스는 `6089f80519cab7b9c3c886f92ca71668249a41ea`로 고정했다. 단독 자기 RGB의 960×720→640×480 정규화와 공동 자기 움직임 식별·국소 바퀴 마스크를 공통 실행기에 연결한 뒤, `dispatch_open` seed 11에서 단독 box와 공동 beam을 Colab CPU와 Mac에서 각각 한 번씩 실행했다. **물리 운반 완주는 Colab 0/2, Mac 0/2**다. 아래 네 건은 실행·회수된 유효한 실패이며, 코드 테스트 통과나 초기 인식 진전이 운반 성공을 뜻하지 않는다.

| 환경·조건 | SIM초 | 실행 wall초 | 명령 | 실제 진행과 종료 경계 |
|---|---:|---:|---:|---|
| Colab 단독 | 1.05 | 36.657 | 5 | `completed_staleness`: worker 제출→소비 2.360초로 고정 wall 안전 제한 초과 |
| Colab 공동 | 2.05 | 67.860 | 13 | `completed_staleness`: 두 worker 제출→소비 약 3.260초; upper 식별·coarse 미진입 |
| Mac 단독 | 73.05 | 100.005 | 701 | 접근·파지·올리기·운반까지 진행, `VISUAL_ATTACHMENT_UNCONFIRMED` 종료 |
| Mac 공동 | 27.05 | 48.786 | 410 | 두 자기 identity 식별 후 coarse 188결정, r3 `own_wheel_heading_unresolved` 종료 |

[기계 판독 요약](results.json)에는 네 결과의 정확한 실행 ID·소스/설정 SHA·경로·비용 지표를 담았다. 원본 `result.json`·`process.json`·계획·backend provenance는 [`runs/`](runs/)에 조건별로 복사했다. 조건마다 SIM 180초, wall 600초, 명령 6,000개의 상한을 사용했다. 외부 LLM 호출·통신 메시지·추가 학습은 0, weld는 OFF였다. 따라서 이 기록은 통신 방식 비교나 새 지도·실물 로봇 일반화 평가가 아니다.

두 초기 결함의 수정 효과와 남은 실패 경계를 구분했다. Mac 단독에서는 실제 960×720 자기 영상의 정규화 해시를 153개 `SKILL_DECISION` 모두 재계산해 일치시켰고, 154개 worker 소비가 모두 fresh였다. Mac 공동에서는 r1/r3의 자기 움직임 claim이 각각 valid/fresh였고, 양쪽에서 coarse 94결정씩 실제 실행했다. r3의 94개 예측과 마지막 거절도 원본 TOP 영상으로 재계산해 일치했다. 마지막 local yellow mask 422px의 최소 사각형은 약 74×63px, 각도 −90°로 기존 four-corner heading 지원 범위를 벗어났다. 인식 임계값이나 Colab의 2초 안전 제한은 완화하지 않았다. [Mac 독립 감사 보고서](mac/independent-review.md), [정형 감사](mac/independent-review.json), [Colab 독립 감사](colab/independent-review.json)가 각각의 검토 범위를 기록한다. Colab에서 소비되지 않은 worker 출력의 오프라인 재생은 실제 행동으로 세지 않는다.

오프라인 전체 검사는 1,783 passed·8 skipped·205 subtests passed, 실행 소스 SHA의 GitHub CI는 12/12 성공했다. [검사 로그](study/ci.log), [CI SHA 영수증](study/ci-execution-sha.json), [경계 검토](study/offline-boundary-review.json)를 보존했다. Mac 원본 1,284개 파일의 해시와 Colab 회수 결과의 해시는 독립 검토에서 확인했다. [Mac 원본 inventory](mac/artifact-hashes.json), [Colab 원본 inventory](colab/artifact-hashes.json), [패키지 목록](package-manifest.json)에 파일별 원본 절대 경로·크기·SHA-256이 있다. [설정](study/config-reviewed.json), [Mac 설정](mac/config-reviewed.json), 각 환경 기록([Colab](colab/environment.json), [Mac](mac/environment.json)) 및 실행별 provenance로 실행 조건을 추적할 수 있다.

원본 전체는 `/Users/changmin/projects/ugrp/outputs/rgb-contract-fix-20260923/`에 로컬 보관한다. 본 패키지는 작은 설정·결과·감사 파일만 포함하며 모델, JPEG, MP4, raw 전체를 복사하지 않았다. Colab 입력 ZIP SHA-256은 `05fccd9e03168651ac3f7d66961cf2a2c076f1f26c7fd4d740554137eb973f95`, 회수 ZIP SHA-256은 `e034014b31cb6d0f00598526831d0e256e05a9632eedf7b9099c81e3b4092259`다. 원본 위치와 영상 해시는 [패키지 목록](package-manifest.json)에 있고, 로컬 보관은 원격 raw 백업이 아니다.

TensorBoard의 `0923-RGB수정-Colab`·`0923-RGB수정-Mac` 스냅샷에 실패 결과를 등록했다. [대시보드 확인](dashboard/dashboard-verification.json), [Colab 수치 대조](dashboard/colab-dashboard.json), [Mac 수치 대조](dashboard/local-dashboard.json), [Mac 영상 표본 검토](mac/video-review.json) 영수증이 있다. 화면 검증은 여섯 비교 run과 일곱 고정 카드·HParams 네 열을 확인했으며, 영상은 조건별 처음·중간·끝 프레임만 확인했다. 과거 `0922-RGB로컬-*` run은 이전 소스의 비교 기록으로, 이 네 실행의 성공률에 합산하지 않는다.
