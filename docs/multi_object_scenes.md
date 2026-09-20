# 다중 물건 정적 장면·공간 검토 v1

[12조건 파일럿](multi_object_pilot.md)을 같은 개방 train 맵에 실제 MuJoCo 물체로 배치한다.
대상은 물건 1·2·3·4·5·6·8개이며, `staging_three`는 봉 1개와 상자 2개로 작업 5개를 요구한다.
전체 조건을 별도 물리 맵 12개나 holdout 평가로 세지 않는다.

## 장면과 입력 경계

[배치 설정](../maps/act_generalization/multi_object_layout_v1.json)은 초기 물건·로봇 배치와
작성 목표·적치 슬롯을 재현한다. 기존 `train-open-1`의 경계와 고정 TOP을 재사용한다.
물건별 body/free joint/geom 이름을 분리하고 기존 plain beam(0.196kg)과 cyan box(0.030kg)의
형상·색·질량·마찰·자식 body를 복제한다. 이름과 초기 위치를 제외한 XML 서브트리 hash를 비교한다.
로봇 XML, 실제 자기 카메라 보정·fisheye 변환, TOP 위치/FOV를 유지하고 weld를 끈다.
기존 production API에 필요한 봉/상자 이름은 비가시·비충돌·중력 보상 placeholder로 남기며
임무 물건 수에 포함하지 않는다. 새 물건은 각각 독립된 동적 body다.

`scene-setup-only.json`에 정확한 초기 배치를, `geometry-evaluation-only.json`에 경로·순서와
평가 결과를 둔다. `static-task.json`에는 작성 지도·목적 슬롯·임무만 내보낸다.
setup, simulator ID, segmentation mask, 기하 경로, 예상 결과는 학생 입력이 아니다.
물리 ID를 분리해도 학생이 같은 모양 물건의 영상 ID를 추적한다는 뜻은 아니다.
`lane_a/lane_b`는 아직 실제 모터/ACT 경로에 연결되지 않았으며 실행 가능하다고 표시하지 않는다.

## 검사 범위

- 실제 모델을 컴파일해 개수·독립 free joint·크기·질량·충돌 활성·목적 paint·맵 벽을 확인한다.
- 초기화 직후와 기존 1초 접힌 팔 초기화 후에 서로 다른 물건/로봇/벽의 침투 접촉을 검사한다.
  바닥 접촉과 같은 로봇 내부 링크 접촉은 별도로 취급한다. 물건 XY 이동이 2mm를 넘으면 표시한다.
- 기존 TOP과 로봇 세 대의 실제 RGB를 저장한다. 평가용 segmentation으로 각 물건의 가시 픽셀을 센다.
  자기 영상의 mask에는 같은 fisheye 좌표 변환을 nearest 방식으로 적용한다.
- TOP의 전체 silhouette 투영 면적 대비 가시 비율을 근사하고, 물건마다 16px 이상·85% 이상을
  정적 검토 문턱으로 사용한다. 이는 인식 성공률이 아니다. 초기 자기 영상에서 안 보이는 물건도
  기록하며, 모든 로봇이 주차 위치에서 모든 물건을 봐야 한다고 요구하지 않는다.
- 미운반 물건과 이미 적치한 물건을 모두 장애물로 유지하는 배치 순서 탐색을 수행한다.
  선행 관계를 지키고, 이전 물건을 지우거나 현재 물건 외의 장애물을 무시하지 않는다.
- 운반 footprint는 기존 pair 값, 별도로 명시한 solo 값으로 고정한다. yaw=0의 평행 이동 경로와
  연속 구간 샘플 clearance를 검사하고 출발/목표의 90도 회전 공간도 별도로 기록한다.
  미하중 로봇의 접근은 다른 두 로봇을 주차 위치의 장애물로 유지한 상태에서 주차점부터
  파지 대기점까지의 **독립 도달성 검사**다.

배치 순서 탐색은 평가용 좌표 ledger만 갱신한다. 실제 물체를 이동시키거나 모터로 재생하지 않는다.
세 로봇의 동시 충돌 회피·이전 배송 위치부터의 연속 로봇 경로·실제 팔/파지 궤적은 미검증이다.
따라서 모든 정적 문턱을 넘어도 `static_candidate`이며 실제 임무 성공이 아니다.
회전 probe 실패는 기록하되 yaw=0 경로의 통과와 구분한다. 제한된 탐색의 실패도 물리 불가능의 증명이 아니다.
불합격/미해결 조건은 모든 결과와 이유를 보존하며 개수·외관·카메라를 바꿔 숨기지 않는다.

## 실행

소스/설정을 먼저 커밋한 깨끗한 작업 트리에서 새 출력 경로로 실행한다.

```sh
.venv-sim-worker-mac/bin/python -m pytest -q tests/test_multi_object_scene.py
.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run multi-object-preview-NEW -- \
  .venv-sim-worker-mac/bin/mjpython scripts/prepare_multi_object_scenes.py \
  --case staging_three --render --output outputs/multi-object-preview-NEW
# --case를 생략하면 12조건 전체. --render를 생략하면 기하 검사만 수행.
```

산출물은 조건별 실제 TOP·r1/r2/r3 영상·장면 XML·배치 도면·전체 검사 결과와 overview,
source/map/mission/setup/scene hash manifest, 환경, 파일별 hash다.
이미지/세그멘테이션 검토와 기하 검사를 구분하고 최종 실험 기록에 시각 검토 범위를 남긴다.
외부 모델 호출·학습·실제 운반 시도는 0회다. 작업 종료 시 세션과 렌더 자원을 정리한다.

다음 단계는 후보 배치에서 RGB 물건 ID 추적을 구현하고, 계획의 작업 ID를 실제 단계별 실행에 연결하는 것이다.
