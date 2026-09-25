# 구역 화물 종류 인식 (TOP RGB, 프로필 `top_cargo_v1`·`top_cargo_v2`)

> **연동에는 `top_cargo_v2`를 쓴다.** v1은 평행 빔 병합과 틀 안쪽 상자 억제 결함이 있어 평가 기준으로만 바이트 그대로 남긴다. v2의 차이는 [아래 절](#top_cargo_v2)과 [v2 실험 기록](../experiments/2026-09-25-zone-cargo-perception-v2/README.md)을 따른다.

2026-09-25 요청: [화물 목록](zone_cargo.md)의 새 종류(can·tile·long_beam·heavy_crate·tri_frame)를 TOP RGB에서 알아보게 해 LLM 로봇이 볼 수 있게 한다. **이 문서는 인식기와 출력 계약만 다룬다.** 제어기·프롬프트 연결은 팀 운반 연동 담당이 한다. 평가 수치는 [실험 기록](../experiments/2026-09-25-zone-cargo-perception/README.md)에 있다.

## 입력 경계

- 입력: TOP JPEG 네 장(`zone_wide`), 작성된 TOP 보정(`static_map['top_cameras']`), 정적 화물 목록(`sim.zone_cargo`: 모양·치수·파지 역할). 정적 목록은 지도와 같은 허용 정적 정보다.
- 쓰지 않음: 시뮬레이터 자세, 분할 영상, 접촉, 화물 설정 pose. 평가 정답은 `eval-labels/`에만 있고 검출기에 가지 않는다.
- 기존 프로필(`zone_perception_v1`, `top_zone_v2`, `own_*`)과 `harness/zone_perception.py`·`harness/zone_color_boxes.py`는 바꾸지 않았다. 새 프로필은 `harness/zone_cargo_perception.py`에 있고 명시적으로 불러야 켜진다. 실행 번들 소스 closure 밖이다.

## 호출

```python
from harness.zone_cargo_perception import detect_all_cargo, detect_cargo_top, grasp_handles
view = detect_all_cargo(tops, static_map)          # tops: {camera_name: jpeg bytes}
for item in view['items']:
    handles = grasp_handles(item)                  # 화물만; 상자는 빈 목록
```

`detect_cargo_top(jpeg, camera)`는 TOP 한 장의 결과(카메라별)다. `detect_all_cargo`는 네 장을 합친다. 겹친 시야에서는 경계에 잘리지 않고 신뢰도가 높으며 화면 중심에 가까운 시야를 남긴다. 빔은 두 시야에 걸친 조각을 바닥 좌표에서 하나로 합친다.

## 출력 스키마 `ugrp.zone_cargo_perception.v1`

```json
{"schema": "ugrp.zone_cargo_perception.v1", "profile": "top_cargo_v1",
 "source": "TOP RGB colour+shape and authored TOP calibration; static cargo catalogue; not simulator state",
 "items": [
  {"kind": "heavy_crate", "colour": "pink", "floor_xy_m": [3.0, -0.2], "yaw_rad": 0.30,
   "yaw_symmetry_deg": 180, "confidence": 0.99, "camera": "cctv_top_north_east",
   "pixel": [0.54, 0.24], "clipped": false,
   "evidence": {"area_px": 1180, "aspect": 1.4, "lug_dark_fraction": 0.97, "axis_source": "body_long_axis"}},
  {"kind": "box", "colour": "red", "floor_xy_m": [1.6, -2.45], "yaw_rad": null, "yaw_symmetry_deg": null,
   "confidence": 1.0, "camera": "cctv_top", "evidence": {"source_profile": "top_zone_v2"}}]}
```

| 필드 | 뜻 |
|---|---|
| `kind` | `box`, `can`, `tile`, `long_beam`, `heavy_crate`, `tri_frame` (`sim.zone_cargo` 종류 이름, 상자는 `EXISTING_SOLO['box']`) |
| `colour` | 상자는 칠 색(`cyan/green/red/yellow`), 화물은 목록 색(violet, magenta, lime, pink, cream) |
| `floor_xy_m` | 물체 원점(목록 정의: 무게중심 아래 바닥)의 세계 좌표. 종류별 윗면 높이로 원근을 보정한다 |
| `yaw_rad` | 물체 x축의 세계 방향. `yaw_symmetry_deg` 주기로만 정해지며 [-주기/2, 주기/2)로 돌려준다. can·box는 `null` |
| `yaw_symmetry_deg` | tile·long_beam·heavy_crate 180, tri_frame 120, can·box `null` |
| `confidence` | 형태 게이트 점수(0–1). 보정된 확률이 아니다. 잘림 0.6배, 빔 한쪽 끝만 보임 0.6, 빔 양끝 모두 안 보임 0.3 |
| `clipped` | 화면 경계에 닿음(자세가 덜 믿을 만함) |
| `evidence` | 판단 근거(면적, 가로세로비, 빔 `centre_mode`·`visible_length_m`·조각 수, 틀 `vertex_mode`·변 길이, 손잡이 어두운 비율) |
| 빔 전용 | `segment_floor_m`(보이는 구간 양끝), `segment_end_clipped` |

필요 대수·formation은 인식 결과가 아니라 정적 목록에서 온다(`grasp_handles`의 `required_carriers`, `formations`).

## 파지 손잡이 위치 (연동 담당용)

`grasp_handles(item)`은 **RGB 자세 추정 + 정적 목록 파지 역할**만으로 손잡이 위치와 접근 base pose를 만든다(시뮬레이터 상태 없음).

```
grip_world    = floor_xy + R(yaw) · grasp.grip_xyz[:2]        (높이는 grasp.grip_xyz[2])
approach_base = floor_xy + R(yaw) · grasp.approach_base()[:2], heading = yaw + grasp.approach_yaw
```

- yaw가 대칭 주기로만 알려지므로 역할 이름(`end_neg`/`end_pos`, `west`/`east`, `v0`/`v1`/`v2`)은 서로 바뀔 수 있다. **손잡이 위치의 집합은 대칭 아래 같다**(테스트로 확인). 어느 로봇이 어느 손잡이를 맡을지는 팀이 위치로 정해야 하며 이름으로 정하면 안 된다(`role_names_interchangeable_under_symmetry`).
- can은 방향이 없어 `approach_heading_free=true`, `approach_base_xyyaw=null`이다. 아무 방향에서 중심을 잡는다.
- 오차는 자세 오차를 그대로 물려받는다. 손잡이 오차 ≈ xy 오차 + (손잡이 반경 × yaw 오차). 빔 끝 손잡이(반경 0.27 m)에서 yaw 1°는 약 4.7 mm다. 평가의 `grip_error_m`이 이 값이다.
- `confidence`가 낮거나 `clipped`인 항목, 빔 `centre_mode`가 `partial_midpoint`인 항목은 손잡이 위치를 쓰기 전에 다른 시점에서 다시 보는 것이 안전하다.
- 목록의 손잡이 형상이 바뀌면(PR #164 미끄러짐 수정) `grasp_handles`는 새 목록 값을 그대로 쓴다. 인식 상수는 테스트(`test_geometry_constants_match_the_catalogue`)가 목록과 대조하므로, 모양이 바뀌면 테스트가 실패해 재검증을 알린다.

## 한계

- held-out test(110장면) 요약: 완전 가시 화물 검출 can 98.2%·tile·beam·frame 100%·crate 98.6%, 종류 혼동 0, 칠·바닥 오검출 0. 수치와 실패는 [실험 기록](../experiments/2026-09-25-zone-cargo-perception/README.md)을 따른다.
- **can 위치는 서쪽 과노출 영역에서 +x로 2–2.6 cm 치우친다**(p90 19.6 mm). 한쪽 집게 여유(약 11.5 mm)보다 크므로, can은 접근 전에 자기 RGB 근거리 관측으로 다시 맞춰야 한다.
- 틀 삼각형 **안쪽**에 놓인 상자는 화물 위 상자로 보고 지워진다(test 1건). 틀 안쪽 상자는 이 프로필로는 보이지 않을 수 있다. → `top_cargo_v2`에서 수정.
- v1은 약 4.5 cm 떨어진 평행 빔 두 개를 하나로 합칠 수 있고, 한 시야에서 붙은 평행 빔은 둘 다 놓친다 → `top_cargo_v2`에서 수정.
- 한 시점 정지 영상이다. 들린 화물, 움직이는 중의 흐림, 기울어진 화물은 평가하지 않았다.
- 같은 색 물체끼리 닿으면(예: can 두 개) 한 덩어리가 될 수 있다.
- 빔의 먼 끝이 로봇에 가려지고 다른 끝도 경계에 잘리면 중심을 알 수 없다(`partial_midpoint`).
- 임계값은 `zone_wide` dev 장면에서 정했다. 다른 조명·지도로 일반화된다는 근거는 없다.

## top_cargo_v2

`harness/zone_cargo_perception_v2.py` (PR #168 검토 결함 수정). 입력 경계, 출력 스키마 `ugrp.zone_cargo_perception.v1`, `grasp_handles`는 v1과 같다. `profile` 값만 `top_cargo_v2`다.

```python
from harness.zone_cargo_perception_v2 import detect_all_cargo, detect_cargo_top, grasp_handles
```

- **빔 병합**
  - 같은 카메라의 검출은 합치지 않는다.
  - 다른 카메라 조각은 다음을 모두 만족할 때만 합친다.
    - 각도 차 5° 이하, 측면 거리 2 cm 이하, 축 방향 겹침 -3 cm 이상.
    - 합친 길이가 정적 길이 + 8 cm 이하.
    - 경계에 잘리지 않은 끝을 6 cm 넘게 지나지 않을 것.
  - 비빔 종류의 중복 제거도 다른 카메라끼리만 한다.
- **붙은 평행 빔 분리**
  - 폭이 1.6배를 넘는 빔 덩어리는 윗면 화소로 둘로 나눈다.
  - 나눈 결과에는 `evidence.split_from_wider_blob=true`를 표시한다.
  - 폭 0.6배 미만 조각은 버린다.
- **틀 근처 상자.** 막대 띠(반폭 + 1 cm)와 손잡이 돌기 4.5 cm 이내에서만 억제한다. 삼각형 안쪽 빈 곳의 상자는 남는다.
- **알려진 절충.** 서로 다른 빔을 합치지 않는 대신, 같은 빔의 짧은 잘린 조각이 따로 남을 수 있다.
  - 새 test에서 180장면 중 중복 4개가 나왔고, 신뢰도는 모두 0.5 미만이었다.
  - 신뢰도 0.5 미만 빔은 다른 시점에서 다시 확인한다.
  - 같은 축 위 가까운 두 빔은 길이 합을 정적 길이와 비교한다.
- **평가 요약**(새 test 12시드·180장면, 완전 가시)
  - 빔: v1 293/297 → v2 297/297.
  - 붙은 평행 빔 쌍: v1 5/11 → v2 11/11.
  - 틀 안쪽 상자: v1 0/40 → v2 37/40. 놓친 3개는 기본 상자 검출기도 놓쳤다.
  - 그 밖의 종류: v1과 같다.
