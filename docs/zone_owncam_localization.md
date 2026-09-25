# 자기 카메라 위치 추정 (구역 지도, 벽 AprilTag)

2026-09-25 사용자 결정에 따른 경로다. 최종 연구의 로봇은 **자기 wrist RGB, 정적 지도, 자기 발행 명령 이력**만 입력으로 쓴다. TOP 카메라는 평가·점수에만 쓰고, 시뮬레이터 전용 `nav_cam`은 로봇 입력이 아니다. 정답 교사는 시연, 학습 표적, 평가에만 쓴다. AprilTag tag36h11은 벽과 문기둥에만 붙이며, 버전이 있는 정적 지도 특징으로 다룬다. 화물과 로봇에는 붙이지 않는다.

M1(zone_wide_door에서 로봇 1대가 wrist 카메라만으로 청록 상자 1개를 문 너머로 배달)의 1단계다. 실험 기록: [experiments/2026-09-25-zone-owncam-loc](../experiments/2026-09-25-zone-owncam-loc/README.md).

## 구성

| 부분 | 파일 | 로봇 입력 여부 |
|---|---|---|
| 태그 지도 버전 | `maps/zones/<base>_tags_v1.json`, `sim/zone_landmarks.py` | 정적 지도 (허용) |
| MuJoCo 태그 | `sim/zone_landmarks.add_tag_geoms`, `TaggedZoneScene` | 장면(물리 불변, 시각 전용) |
| 검출·PnP | `harness/wall_tags.py` | 자기 RGB, 발행 PWM, 고정 카메라 보정 |
| 위치 추정 | `harness/owncam_localizer.py` | 발행 명령, 검출, 지도, 오프라인 보정값 |
| 데이터 기록 | `scripts/record_owncam_localization.py` (workflow `zone-owncam-loc-record`) | 교사 주행. `inputs/`와 `eval_only/` 분리 |
| 오프라인 평가 | `scripts/eval_owncam_localization.py` | detect·localize는 입력만 읽고, calibrate·score만 정답을 읽음 |

## 지도 버전과 해시

- 새 지도: `zone_wide_door_tags_v1`(태그 70개), `zone_wide_two_doors_tags_v1`(65개), `zone_wide_corridor_tags_v1`(87개).
- 기존 `zone_wide_door`, `zone_wide_two_doors`, `zone_wide_corridor` JSON은 바이트 그대로 둔다. 테스트가 SHA-256을 고정한다.
- 태그 지도는 base 지도의 모든 키를 그대로 복사한다. 여기에 `map_id`, `version`, `base_map.static_map_sha256`(base의 `digest`), `landmarks` 블록을 더한다.
  - `landmarks` 블록: schema `ugrp.zone_landmarks.v1`, 태그 가족, OpenCV 사전 이름, 사용한 ID들의 비트 해시, 배치 파라미터, 태그 목록 `{id, wall, normal_xy, center_m, yaw_rad, size_m}`.
- `tagged_map(name)`은 파일이 정의와 다르거나 base 지도가 바뀌었으면 거절한다.
- 실행 기록(`manifest.json`)에는 다음 해시를 모두 남긴다: `static_map_sha256`, `base_static_map_sha256`, `landmarks_sha256`, `scene_xml_sha256`, 저장한 `scene.xml` 파일 해시.

## 태그 배치 기본값과 변경 방법

`sim/zone_landmarks.DEFAULT_PLACEMENT` 기본값:

| 항목 | 값 | 비고 |
|---|---|---|
| 검은 사각 | 0.072 m | 칸 9 mm |
| 흰 판 | 0.090 m | 한 칸 여백 |
| 중심 높이 | 0.05 m | 판이 z 0.005–0.095에 있어 0.10 m 벽 위로 나오지 않음 |
| 간격 | 0.5 m 이하 | 모든 자유 벽면의 양 끝(문 가장자리·모서리 옆)에 한 장씩 |
| 판 두께 | 1 mm | |
| 검은 칸 두께 | 1 mm | |

- 바깥 벽은 안쪽 면에만, 내부 벽은 양면에 붙인다.
- 문기둥 태그는 문 가장자리에서 0.075 m 떨어진 벽면에 양쪽 방향으로 하나씩 있다. 벽 끝면은 0.05 m 두께라 붙이지 않는다.
- wrist 카메라 높이: search 자세(`SEARCH_POSE`)에서 0.21 m, −21°. 수평 운반 자세(`CARRY_POSE`)에서 0.23 m, +0.5°. 두 자세 모두 0.4 m 밖부터 이 높이의 태그를 본다.
- 가시성 연구에서 권장값이 나오면 `TAGGED_MAPS`에 `_tags_v2`를 새로 추가한다. 기존 v1 JSON은 덮어쓰지 않는다(`write_tagged_maps`가 거절).

물리 불변 조건:
- 태그는 `contype=0 conaffinity=0 mass=0`인 얇은 box geom이다.
- `tests/test_zone_landmarks_sim.py`는 같은 명령에서 base 장면과 태그 장면의 `qpos` 궤적이 비트 단위로 같은지 확인한다.

## 검출·PnP·외부 파라미터

- OpenCV 5.0 `cv2.aruco` `DICT_APRILTAG_36h11`로 raw 어안 영상에서 검출한다(`CORNER_REFINE_SUBPIX`).
- 모서리는 실측 K/D로 `cv2.fisheye.undistortPoints`를 거친다. 시뮬레이터 영상은 핀홀 렌더를 `raw_fisheye_remap`한 것이므로 같은 모델로 역변환된다.
- 태그마다 `SOLVEPNP_IPPE_SQUARE`로 두 해를 구한다. 이동량은 재투영 오차가 작은 해에서 쓰고, 면 법선은 두 해를 모두 후보로 둔다.
- **카메라→차체 변환은 `harness.visual_arm.camera_extrinsics(발행 PWM)`만 쓴다.** 명령한 서보 펄스의 FK이며, 측정 관절은 쓰지 않는다.
  - 발행 PWM 상태는 로봇 자신의 명령 기록을 재생해 만든다. 기록기가 프레임에 적은 값과 매 프레임 일치하는지 검사한다.
- 측정 결과: FK 카메라는 실제 렌더 카메라보다 pitch가 약 0.97° 다르다. 팔이 중력으로 처지기 때문이다. 수평 방위각 오차는 0.1° 이하다.
  - 그래서 측정 모델은 방위각(좁음)과 고도각(넓음)을 나누고, 외부 파라미터 보정항은 두지 않는다.

## 파티클 필터

- PythonRobotics `Localization/particle_filter/particle_filter.py`(commit `b2020cd`, MIT)를 옮겼다. 저작권·허가 문구는 `harness/owncam_localizer.py` 머리말에 있다.
  - 원본에서 가져온 것: 입력 잡음 예측, 가우시안 가중치, 가중 공분산, 유효 입자 수, 저분산 재표집.
- 운동 모델: 발행한 `mecanum` 명령 `(forward, left, turn)`을 lease 동안 유지한다. `hold` 등은 정지로 본다.
  - 차체 속도 = 이득 행렬 × 명령, 1차 지연 τ를 둔다.
  - 입자마다 느리게 변하는 미끄러짐 배율을 둔다.
  - 이득·τ·잡음은 **dev 교사 로그로 오프라인 보정**한다. 출처와 파일 해시는 보정 JSON에 남는다.
- 측정 모델: 태그 PnP의 방위각, 고도각, log 거리, 면 법선(두 IPPE 해 중 최소)이다.
  - 태그마다 이상치 상한을 둔다.
  - 한 프레임의 태그들은 같은 FK 오차를 공유하므로 합친 로그우도를 태그 수로 완화한다(`tag_temper`).
- 지도: 벽(로봇 여유 0.07 m 포함) 안이나 경계 밖에 있는 입자는 로그가중치를 −8 깎는다.
- 초기화와 재설정: 첫 태그 관측에서 yaw를 원 전체에 퍼뜨리고, 태그 이동량으로 위치를 정한다. 최고 우도가 너무 낮으면 일부 입자를 같은 방식으로 다시 뿌린다.
- 출력: 평균 `(x, y, yaw)`, 3×3 공분산, 유효 입자 수, 마지막 태그 이후 시간.

## 경계 검사 (테스트)

`tests/test_owncam_localizer.py`가 다음을 확인한다.
- 실행 모듈 두 개가 허용 목록 밖의 모듈을 import하지 않는다.
- `xpos`, `qpos`, `base_xyz`, `eval_only` 같은 시뮬레이터 상태 이름을 쓰지 않는다.
- `mujoco` import를 막아도 동작한다.
- 오프라인 `localize`가 `eval_only/`를 열지 않는다.

## 실행

```sh
PY=.venv-sim-worker-mac/bin/python
E=experiments/2026-09-25-zone-owncam-loc/episodes.json
$PY scripts/record_owncam_localization.py --episodes $E --only dev-ls-s11 --output outputs/owncam-loc/raw
$PY scripts/eval_owncam_localization.py detect    --data outputs/owncam-loc/raw --episodes dev-ls-s11
$PY scripts/eval_owncam_localization.py calibrate --data outputs/owncam-loc/raw --episodes <dev ids> --calibration cal.json
$PY scripts/eval_owncam_localization.py localize  --data outputs/owncam-loc/raw --episodes <ids> --calibration cal.json
$PY scripts/eval_owncam_localization.py score     --data outputs/owncam-loc/raw --episodes <ids> \
    --thresholds experiments/2026-09-25-zone-owncam-loc/thresholds.json --output metrics.json
```

기록은 동기 SIM 전용이며 weld OFF, `local_contact_fine`을 쓴다. 교사가 주행하므로 기록 자체는 학생 성공이 아니다.
