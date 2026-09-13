# 사전 지도와 영상으로 주행하기

사용자가 2026-09-13에 승인한 **사전 지도 직접 제공** 실행 경로다. `maps/navigation/*.json` 하나를 검증한 뒤 같은 자료로 물리 장애물과 항법 지도를 만든다. 지도에 로봇의 실시간 정답 위치를 넣지 않는다.

현재 범위는 평평한 바닥의 정적 벽, 접은 팔, 짐을 들지 않은 로봇 한 대다. r1/r3는 각각 별도 실행한다. 두 카메라 원본을 저장하지만 현재 위치 추정과 제어에는 공용 top RGB를 사용한다. 로봇 자기 카메라에 의한 장애물 판독, LLM 판단, 경사로·턱 통과, 동적 장애물과 여러 로봇의 공동 운반은 별도 후속 검증 대상이다.

## 지도와 판단

- `bounds_m`: 지도 범위. 카메라가 볼 수 있는 영역 안에 구성한다.
- `zones`: 사전에 정한 시작 구역과 목표 구역. 실제 시작 위치는 모델 입력에 포함되지 않는다.
- `obstacles`: 실제 충돌 상자와 같은 위치·크기·높이의 고정 장애물. v1은 `traversable: false`만 허용한다.
- `footprint`: 접은 로봇의 반경 0.18m와 안전 여유 0.04m. 원래 장애물보다 큰 금지 영역을 만들고, 그 바깥에서도 벽에 붙지 않는 경로를 선호한다.
- `top_camera`: 이전 실험에서 사용한 고정 카메라의 지도 좌표계 보정값. 이 실행기는 다른 위치/방향/FOV를 거부한다.

로봇 외형의 색 특징으로 자신의 위치를 영상에서 추정하고, 발행 명령 전후 영상 변위로 이동 방향을 보정한다. 명령 발행 자체를 움직였다는 증거로 쓰지 않는다. 경로가 없거나 위치/보정 결과를 신뢰할 수 없으면 정지한다. 목표 도착도 새로운 영상으로 다시 확인한다. 물리 정답을 읽는 별도 평가기는 결과 파일만 기록한다.

지도 범위는 계획 영역이며 그 선에 물리 벽을 추가하지 않는다. 보정은 0.6초 동작과 영상 정지 확인을 축별 최대 6회 반복하며, 누적 영상 변위 5cm를 요구한다. 각 동작의 여유 공간 검사는 현재 시뮬레이터와 시험한 시작 조건에서 검증한 것으로, 임의의 실물 구동기에 대한 최대 이동 거리 보장은 아니다. `direct`도 같은 지도 기반 초기 보정 검사를 통과하므로 비교 결과는 주행 중 경로계획의 효과를 보여준다.

## 실행

실험 소스와 설정을 먼저 커밋한다. 작업 중인 변경이 있으면 실행기가 거부한다. Mac 예시(저장소 루트에서 실행):

```sh
PYTHONPATH=. .venv-sim-worker-mac/bin/python scripts/ugrp_session.py run known-map-example -- \
  .venv-sim-worker-mac/bin/mjpython -m scripts.run_known_map_navigation \
  --map-file maps/navigation/slalom.json \
  --case-json '{"case_id":"example","robot_id":"r1","start_xy_m":[-0.5,-2.55],"start_yaw_deg":0}' \
  --out-dir outputs/known-map-example-NEW
```

분리된 worktree에서는 기존 시뮬레이션 환경 실행 파일의 절대 경로를 사용한다. Ubuntu에서는 설치된 환경의 `python`과 기존 headless 렌더 설정을 사용한다. `--condition direct`는 같은 영상 위치 추정/보정을 사용하되 목표로 직행하는 비교 조건이다. 비교 실험에서 실제 벽 접촉이 생길 수 있으므로 성공 여부는 별도 평가 결과를 확인한다.

```sh
python scripts/audit_known_map_navigation.py outputs/known-map-example-NEW
```

감사는 실행 당시 항법 코드로 수행한다. 원본 이미지·지도·명령을 다시 넣었을 때 결정이 같은지, 해시·입력 항목·카메라 불변 조건이 맞는지 검사한다. 감사 통과와 충돌 없는 실제 도착은 다른 결과다.

`run.json`에는 실행 SHA/환경/설정, `actor-decisions.jsonl`에는 원본 입력 참조와 결정, `rgb/`에는 실제 JPEG, `evaluation-only.jsonl`에는 제어와 분리한 정답 평가, `motion.mp4`에는 관찰용 영상, `manifest.json`에는 원본 해시가 남는다. 원본의 로컬 보관을 원격 백업으로 해석하지 않는다.

검증 설계와 전체 결과는 [실험 기록](../experiments/2026-09-13-known-map-navigation/)을 따른다.
