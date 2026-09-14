# 길 쪽으로 몸을 돌린 뒤 전진

사용자 요청: “좀 사람같게.. 고개를 돌려서 앞으로 이동하면 안되나?”

기존 항법은 `turn=0`을 유지하고 전진/옆걸음을 섞어 경로를 따라갔다. 차체가 이동 방향을 바라보지 않아 옆으로 미끄러지는 모습이었다. 기본 실행을 `heading`으로 바꾸어, 공용 RGB 영상으로 방향을 알아낸 뒤 차체를 실제로 돌리고 양의 전진만 발행하도록 했다. 이전 방식은 `holonomic`으로 보존한다. 카메라 장착 위치·FOV, 로봇 외형, 접은 팔, 지도와 물리 환경은 그대로다.

## 확인된 결과

최종 실행 SHA: `3abedb6e776b960359033b3371f38275e1bdfda1`.

동일한 두 시작 조건에서 open/slalom/narrow × heading/holonomic, 총 12실행을 완료했다. 이동 가능한 네 조건은 두 방식 모두 4/4 충돌 없이 도착했다. 좁은 통로 두 조건은 두 방식 모두 2/2 진입 전 거부했다. 거부는 도착 성공률에 섞지 않는다. 전체 12실행에서 장애물 접촉·weld 활성화 0, 카메라/외형 불변 조건 통과. 각 비교 쌍은 최초 자기/공용 JPEG 바이트, 사전 지도, private setup, scene/robot/camera invariant가 동일함을 별도로 확인했다.

| 실행 | 결과 | 결정 수 | SIM 초 | 옆걸음 명령 | 회전 명령 |
|---|---|---:|---:|---:|---:|
| heldout_r1-open-heading | arrived | 58 | 18.046 | 0 | 2 |
| heldout_r1-open-holonomic | arrived | 70 | 21.65 | 56 | 0 |
| heldout_r1-slalom-heading | arrived | 142 | 56.35 | 0 | 49 |
| heldout_r1-slalom-holonomic | arrived | 138 | 38.65 | 124 | 0 |
| heldout_r1-narrow-heading | no_map_route | 1 | 0.25 | 0 | 0 |
| heldout_r1-narrow-holonomic | no_map_route | 1 | 0.25 | 0 | 0 |
| heldout_r3-open-heading | arrived | 62 | 20.058 | 0 | 4 |
| heldout_r3-open-holonomic | arrived | 72 | 22.3 | 57 | 0 |
| heldout_r3-slalom-heading | arrived | 152 | 60.388 | 0 | 53 |
| heldout_r3-slalom-holonomic | arrived | 147 | 41.05 | 132 | 0 |
| heldout_r3-narrow-heading | no_map_route | 1 | 0.25 | 0 | 0 |
| heldout_r3-narrow-holonomic | no_map_route | 1 | 0.25 | 0 | 0 |

벽 우회에서 차체 방향과 실제 이동 방향의 거리 가중 차이는 r1 50.30°→1.82°, r3 47.19°→1.26°였다. 옆걸음 명령은 r1 124→0, r3 132→0이다. 이 각도는 종료 후 별도 평가 기록으로 계산했으며 제어 입력에 넣지 않았다. 0.1초 간격 위치/방향 샘플의 0.2mm 이상 이동을 계산하므로 완전한 연속 시간 증명은 아니다.

대신 회전 시간이 추가되어 우회 시간은 r1 38.65→56.35 SIM초, r3 41.05→60.388 SIM초로 늘었다. 이번 결과는 방향을 바라보며 움직이는 동작의 개선이며 최단 시간 제어의 증거가 아니다. 실제 정지/회전 전환에는 관성 이동이 조금 남을 수 있다. 화면 방향 추정 오차는 우회 두 실행에서 평균 4.01°/3.71°, 최대 16.42°/15.43°였다.

## 어떤 관측으로 결정했는가

1. 시작 구역 근처 로봇의 기존 황금색 바퀴/구조물을 top RGB에서 찾아 지도에 투영한다. 사전에 준 지도와 고정 카메라 보정 정보만 사용한다.
2. 전진 0.6초 후 새 정지 영상을 비교한다. 누적 영상 이동 5cm가 확인되면 앞 방향과 전진 명령의 효과를 추정한다. 명령만으로 이동 성공을 인정하지 않는다.
3. 지도에서 차체 반경·여유를 포함한 경로를 찾고 다음 경유점 방향과 추정 앞 방향을 비교한다. 10°보다 차이가 크면 차체 회전 명령을 발행한다.
4. 회전 전후 차체 색 영역의 큰 형태를 비교한다. 작은 바퀴 무늬 변화를 완화하고, ±3px 이동과 ±12° 회전을 함께 탐색한다. 회전 신호가 불충분하거나 명령 반대 방향으로 관측되면 정지한다.
5. 방향이 맞으면 전진만 한다. 실제 영상 변위로 전진 시간을 보정하고, 회전 없는 구간에서 누적 8cm가 관측될 때 방향을 다시 추정한다. 위치/차체 형태를 4회 연속 잃으면 종료한다.
6. 목표 구역은 두 번의 새 영상으로 확인한다. 실제 도착·접촉·자세의 정답 평가는 종료 후 별도 결과에만 남긴다.

입력은 승인된 사전 지도, 자기 RGB, 공용 top RGB, 프레임 번호, 자기 발행 명령 이력이다. 현재 위치/방향 추정에는 공용 RGB를 쓴다. 자기 RGB는 저장·검증하지만 제어에 쓰지 않는다. 시뮬레이터 위치/방향, 관절 측정, 접촉, 평가 결과를 제어에 전달하지 않는다. 모델 호출 0회·모델 비용 0달러인 고전 영상 항법이며, LLM 협력 성공으로 부르지 않는다.

## 실패와 개발 기록

- `heading-turn-probe-v1`, `8512cd9`: 고정 전진/회전 명령으로 실제 RGB 회전 관측을 확보했다. 항법 성공 시험이 아니다.
- `heading-development-v1`, `cdc1479`: open 도착, narrow 거부. slalom은 10결정 후 바퀴 무늬 변화로 회전 유사도 부족 정지. 충돌 없음.
- `heading-development-v2`, `e102983`: 작은 무늬를 완화한 뒤 slalom 15결정까지 진행했으나, 황금색 픽셀의 무게중심이 바뀌어 다시 회전 인식 실패. 충돌 없음.
- `heading-development-v3`, `3abedb6`: 차체 외곽 중심과 이동/회전 공동 비교, 긴 직진 구간 방향 갱신으로 slalom 165결정/68.73 SIM초에 충돌 없이 도착.
- 최종 비교는 같은 `3abedb6`로 전체 12조건을 새로 실행했다. 이 두 시작 설정은 앞선 #36 실험에도 쓰였으므로 새 환경 일반화 증거로 해석하지 않는다.

실패를 포함한 17개 항법 실행 모두 실행 당시 소스에서 재생 감사 통과: 총 1,067프레임, 그중 최종 비교 845프레임. 이전 style 필드가 없는 기록은 원래 holonomic으로 재생한다. 감사는 기록된 입력/결정과 코드 경계의 검증이며 숨은 상태 부재의 암호학적 증명은 아니다.

## 영상과 검사

- [r1 이전/이후 비교](media/r1-before-after.mp4): 같은 SIM 배속, 왼쪽 완료 후에는 마지막 프레임을 유지하며 표시한다. 원본은 4fps로 기록되어 약 0.25초 시간 양자화가 있다.
- [r1 방향/경로 그림](media/r1-direction-comparison.png): 화살표는 제어 입력이 아닌 사후 평가 방향이다.
- [r1 새 주행](media/r1-slalom-heading.mp4), [r3 새 주행](media/r3-slalom-heading.mp4), [r1 기존 주행](media/r1-slalom-holonomic.mp4), [r3 기존 주행](media/r3-slalom-holonomic.mp4).
- 개발 성공 영상은 5초 간격, 최종 r1/r3 새 주행은 전체 구간 4초 간격 프레임을 직접 검토했다. 초기 회전 실패 전후 RGB와 비교 영상 합성 결과도 직접 확인했다. 매 물리 단계 접촉 검사는 별도 평가기가 수행했다.
- 로컬 전체 검사: `python scripts/run_ci_tests.py` → **563 passed, 152 subtests passed**. 최종 실행 SHA의 GitHub `offline-regressions`·`ubuntu-simulation` 통과: [실행 34760842645](https://github.com/kcm0127-dotcom/ugrp/actions/runs/34760842645).

## 저장 범위와 재현

[사전 계획](protocol.md), [전체 결과](all-results.json), [비교 CSV](comparison.csv), [원본 위치·해시](raw-locations.json), `records/`의 원본 메타데이터·결정/평가 로그(gzip, 손실 없음)를 보관한다. 대표 영상·재현용 실제 JPEG도 Git에 포함한다. 전체 RGB와 나머지 raw 영상은 `/Users/changmin/projects/ugrp/outputs/`의 로컬 보관이며 원격 백업으로 주장하지 않는다. 알려진 장애물·고정 관측·접은 팔·짐 없는 한 대의 차체 주행 범위다. 카메라만 별도로 돌리는 목 동작, 이동 장애물, 가림 복구, 하중 운반과 실물 로봇은 검증하지 않았다.

```sh
# 해당 소스로 체크아웃한 깨끗한 작업 브랜치에서 새 output 이름을 사용한다.
PYTHONPATH=. /absolute/path/to/python scripts/ugrp_session.py run heading-comparison-NEW -- \
  /absolute/path/to/python -m scripts.run_known_map_cohort \
  --matrix heading-comparison --python /absolute/path/to/mjpython \
  --out-dir /absolute/path/to/outputs/heading-comparison-NEW
PYTHONPATH=. /absolute/path/to/python -m scripts.report_heading_navigation \
  /absolute/path/to/outputs/heading-comparison-NEW
```

작업 브랜치는 #36 `codex/known-map-navigation` 위에 쌓았다. 이슈 #37을 다루며 main 병합은 사용자 확인 후 별도 승인으로 진행한다. 이 프로젝트는 Drive를 사용하지 않는다.
