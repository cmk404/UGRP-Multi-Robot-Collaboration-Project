# 공동 파지 유지와 RGB 미끄러짐 회복

실행 소스 `cdeee5fb22fe7ccbbb90b0bde424e7692a94aa16`. PR52의 `b1924bb` 위에서 수정했으며 main 병합은 별도 승인 대상이다.

## 결과

| 시험 | 결과 | 높이 감소 / 비고 |
|---|---|---|
| stationary 300초 | 통과 | 최대 6.908mm; 바닥 방출 성공 |
| shuttle 300초 | 통과 | 최대 6.908mm; 바닥 방출 성공 |
| narrow-door | 운반·방출 통과 | 449회 판단; 재파지 0회 |
| l-corner | 운반·방출 통과 | 264회 판단; 재파지 0회 |
| s-bends | 운반·방출 통과 | 339회 판단; 재파지 0회 |
| staggered-obstacles | 운반·방출 통과 | 419회 판단; 재파지 0회 |
| blocked-branch | 운반·방출 통과 | 384회 판단; 재파지 0회 |
| fully-blocked | 경로 없음 공동 정지 | 10회 판단; 재파지 0회 |
| 기존 접촉 + 조기 감지·회복 | 별도 회복 평가 통과 | 한 번 재파지·재개 후 재발 시 내려놓고 종료; 연속 유지 실패 |

왕복은 평가 출력에서 빔의 x방향 이동 범위 17.42cm와 중간 지점의 같은 방향 통과 25회를 확인했다.

두 300초 시험의 180/300초 checkpoint는 같은 궤적을 나눠 평가한 것이며 독립 반복 두 번이 아니다. 여섯 지형은 통행 가능 5개 운반·방출과 차단 대조 1개 정지를 구분한다. 장시간 유지의 성공 기준은 높이 감소 1cm, 간격 변화 2cm, 기울기 10도, 모든 표본의 양측 파지·들림, 충돌 없음, weld OFF 및 카메라·형상 불변이다. 기준을 완화하지 않았다.

## 시간과 행동 수

| 시험 | 전체 SIM초 | 주행 판단 | 실제 실행 초 |
|---|---:|---:|---:|
| baseline-recovery | 63.32 | 46 | 46.9 |
| blocked-branch | 107.61 | 384 | 153.2 |
| fully-blocked | 26.56 | 10 | 32.0 |
| l-corner | 83.61 | 264 | 111.1 |
| narrow-door | 120.61 | 449 | 168.9 |
| s-bends | 98.61 | 339 | 139.8 |
| shuttle | 329.81 | 3000 | 708.7 |
| staggered-obstacles | 114.61 | 419 | 164.8 |
| stationary | 329.81 | 3000 | 692.6 |

각 판단 라운드에는 두 로봇의 독립 판단이 있다. 파지 간격 감시는 별도로 초기 104라운드이며 회복 대조의 재파지 때 다시 실행한다. 실시간 비용은 해당 Mac의 관측값으로, 일부 감사 작업과 동시에 실행해 속도 우월성 비교에 쓰지 않는다. 모델 호출과 비용은 모든 조건에서 0이다. 더 작은 물리 시간 간격은 연산량을 늘린다.

## 원인과 변경

기존에는 접촉이 계속 있어도 빔이 집게 사이로 천천히 내려갔다. 기존 최종 stationary/shuttle은 약 94초에서 영상 외형 변화로 멈췄고, 90초 표본에서는 빔 높이가 약 25.85mm 내려간 반면 손가락 높이는 약 0.35mm만 변했다. 이번 30초 기준 진단의 마지막 접선 힘은 마찰 한계의 최대 약 1.81%였다. 단순히 집게 힘의 한계를 넘는 현상보다 연성 접촉의 수치적 미끄러짐이 주원인이라는 근거다.

MuJoCo의 연성 접촉 마찰은 접선 방향에서 감쇠에 의존하므로 마찰 원뿔 안에서도 미끄러짐이 남을 수 있다. [공식 설명](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip)을 참고했다. 전역 NoSlip은 바퀴 접촉에도 영향을 주어 간격 제어 실패가 생겼고, 국소 감쇠를 큰 시간 간격으로 계산하면 수치 불안정이 발생했다.

채택한 retention 프로필은 손가락과 빔의 네 접촉에만 `solreffriction="0 -3000"`을 적용하고 물리 시간 간격을 2ms에서 0.25ms로 줄인다. 기존 impratio=10, NoSlip=0, normal solref/solimp, 형상·질량·마찰계수, 카메라/FOV, PD·바퀴 명령 상한, lift 명령은 유지한다. baseline과 retention의 초기 geometry hash와 camera 기록이 동일함을 확인했다. 이 변경은 시뮬레이터 접촉 모델의 수치 처리 변경이며 제어 소프트웨어만의 개선이나 실물 마찰 보정 증명이 아니다.

기존 own RGB의 빔 하단 경계가 지속적으로 내려가는지 추가 감시한다. 2초 영상 창에서 처음 대비 높이 .035 이상 변화와 .008/s 초과 변화율이 .3초 지속되면 공동 정지한다. 기존 카메라를 옮기지 않고 정답 높이를 입력하지 않는다. 한 프레임의 외형 변화는 지속 미끄러짐으로 취급하지 않는다.

감지되면 기존 내려놓기·방출 명령을 실행하고 한 번만 기존 RGB 파지 절차로 다시 잡는다. 새로운 영상과 공동 허가를 받은 뒤 재개하며 재발하면 내려놓고 종료한다. 새 지형의 임의 위치·방향에서 성공하는 범용 재파지를 검증한 것은 아니다. 회복 물리 검증은 기존 고정 시작 stationary 대조에서 수행했다.

## 진단과 검토 범위

전체 진단 10회, 예비 시험 2회, 최종 코호트 9회를 보존했다. NoSlip1/3, lift 중 위치 보정, diagexact, 1ms 국소 감쇠, 적분 제어·횡방향 명령 확대의 실패도 records/에 포함한다. 채택 후보의 최초 30초 높이 감소는 .224mm, 60초는 1.05mm였다. 30/60초 예비 결과의 endurance success=false는 180/300초 checkpoint 미실행 또는 의도된 회복 중단을 뜻하며 성공으로 덮어쓰지 않았다.

기존 raw stationary/shuttle 영상을 오프라인 재생하면 새 감지기는 각각 2.2초에 경고했다. 별도 물리 회복 실행에서도 2.2초에 감지했다. 이 값은 특정 고정 시작 조건에서의 결과이며 일반 감지 지연을 뜻하지 않는다.

최종 9회와 예비 2회의 저장된 RGB, 발행 명령, 동기화 허가를 전부 재생 감사해 11/11 통과했다. 회복 보고서·팔 명령·신선한 영상 재개·별도 평가도 통과했다. 로컬 CI 664 tests + 154 subtests가 통과했다. 정답 좌표·접촉·관절 측정은 평가 출력에만 있으며 행동 및 전환 입력으로 사용하지 않는다. 전체 모델 호출 0회, 모델 비용 $0. 고정 초기화·파지/방출 시연과 고전 RGB 제어의 결과이며 적응형 LLM 협업이나 새 조건 학습 성공이 아니다.

원본 관찰자 영상의 초기 파지, 유지 중간·종료, 바닥 방출 및 회복 전후 대표 프레임을 눈으로 확인했다. own RGB의 시작·중간·후반도 비교했다. 모든 동영상 프레임을 사람이 하나씩 검토한 것은 아니다. 영상의 시간/높이 표시는 평가용이며 actor 영상 입력에는 넣지 않았다.

## 자료

- [높이 비교](media/height-retention.png), [유지·회복 영상](media/retention-and-recovery.mp4), [여섯 지형 영상](media/terrain-retention.mp4), [관찰자 프레임](media/observer-review.jpg), [실제 own RGB](media/own-rgb-review.jpg)
- [최종 및 예비 결과](records/summary.json), [전체 진단](records/diagnostic-summary.json), [사전 계획](protocol.md)
- [검증 기록](validation/verification.json), [원본 위치와 저장 범위](records/raw-manifest.json)

압축 결과·명령·평가, 선택한 이미지/동영상과 해시를 Git에 보관한다. 대량 RGB·원본 영상 및 파지 모델은 기존 로컬 폴더에만 있으며 해시가 원격 백업이나 다운로드 주소를 뜻하지 않는다. Google Drive는 사용하지 않는다.

## 실행

현재 Mac 시뮬레이션 Python은 기본 프로젝트의 `.venv-sim-worker-mac`이다. 전체 파지 모델 폴더는 기존 로컬 자산으로 별도 필요하다. 새 출력 폴더 이름을 사용한다.

```sh
python3 scripts/ugrp_session.py run retention-NEW -- \
  /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python \
  experiments/2026-09-15-pair-grasp-retention/run_cohort.py \
  --mjpython /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython \
  --grasp-model-dir /absolute/path/to/models/grasp \
  --out-dir outputs/retention-NEW
```

지도 운반은 `--vision-mode robust --impratio 10 --contact-profile retention`을 명시한다. 기존 비교는 `--contact-profile baseline`으로 구분한다. endurance의 기본 프로필은 retention이며 조기 감지·한 번 회복이 켜져 있다. 실험 중 소스는 고정하고, 실행 완료 뒤 시작한 세션과 자식을 정리한다.

5분을 넘는 무기한 유지, 새 하중·재질·시작 자세, 임의 지형에서의 재파지, 실물 로봇은 미검증이다. 남은 느린 미끄러짐이 있으므로 이 설정을 보편적 최적값이라고 부르지 않는다.
