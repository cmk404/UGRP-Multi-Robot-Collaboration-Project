# 구역 화물 목록 (solo · pair · trio)

2026-09-25 사용자 요청: 역할 배분과 팀 구성이 실제로 의미 있도록, 한 대가 드는 물건·정확히 두 대가 필요한 물건(긴 빔, 무거운 상자)·세 대가 필요한 물건(삼각 틀)을 둔다.

**필요 대수는 규칙이 아니라 SIM 로봇의 물리에서 나온다.** 질량·길이는 로봇 모델에서 잰 한 대의 들기 한계와 집게 형상에 맞춰 정했고, 근거는 [실험 기록](../experiments/2026-09-25-zone-cargo-catalogue/README.md)의 정답 교사 검사(weld OFF)다. 질량은 SIM MasterPi 기준 값이며 실물 적재 능력 주장이 아니다.

## 목록 (`sim/zone_cargo.py`, 목록 v1)

| 종류 | 치수 (mm) | 질량 | 필요 대수 | 물리 근거 | TOP 식별 |
|---|---|---|---|---|---|
| box (기존) | 34×40×32 | 0.030 kg | 1 | 기존 상자, 4색 | 기존 4색 |
| can | Ø38×50 | 0.080 kg | 1 | 한 대 한계의 12% | 보라, 작은 원 |
| tile | 60×40×12 | 0.025 kg | 1 | 낮은 물건, 바닥 위 7 mm에서 잡음 | 자홍, 작은 직사각형 |
| long_beam | 600×40×32 | 0.300 kg | 2 | 팔에 손목 회전이 없어 긴 축 위에서만 잡힘(끝 9 cm 안). 한쪽만 들면 먼 끝이 바닥에 남음 | 연두, 15:1 막대, 양끝 검은 띠 |
| heavy_crate | 240×100×60 (본체 140×100) | 0.900 kg | 2 | 한 대 한계(0.66–0.68 kg) 초과, 본체 폭 100 mm > 집게 최대 61 mm | 분홍 직사각형 + 양끝 검은 손잡이 |
| tri_frame | 한 변 0.35 m 삼각 틀 | 1.500 kg | 3 | 두 대 합계 한계 초과, 두 손잡이만 잡으면 셋째 꼭짓점이 바닥에 남음 | 크림색 삼각 윤곽 + 꼭짓점 검은 손잡이 |

- 모든 손잡이는 기존 상자 단면(집게 방향 40 mm, 같은 마찰·같은 손가락 접촉 pair)이라 기존 상자 파지(반경 0.155 m, 높이 24 mm, 집게 1500)를 그대로 쓴다. pair/trio 손잡이는 46 mm로 높여 느린 미끄러짐 여유를 늘렸다(56 mm는 손목 충돌).
- 정적 명세(`CargoKind`: `required_carriers`, 파지 역할별 물체 좌표 grasp frame·접촉 geom, 역할별 접근 base pose, 충돌 형상, 질량·관성, 접촉 profile, 허용 formation, 착지 영역·방향 대칭, 시각 명세)와 실행 인스턴스(`CargoInstance`: body/joint 이름, 설정 전용 pose)를 분리했다. 이 항목은 Codex 읽기 전용 분석(2026-09-25)이 제안한 팀 운반 연동 계약을 따른다.
- `cal_block`은 한 대 한계 측정 전용(상자 모양, 질량 가변)이며 목록·에피소드에 들어가지 않는다.

## 시각 명세 (탐지기는 바꾸지 않음)

색은 네 상자 색(빨강·노랑·초록·청록), 로봇(주황 집게, 노랑 롤러, 알루미늄, 검정), 바닥 칠(적재 파랑, 구역 A 주황, B 파랑, C 보라)과 겹치지 않게 골랐다. `scripts/render_zone_cargo_catalogue.py`가 TOP 렌더(960×720)에서 각 물체 화소를 분할(평가 전용)해 다음을 확인한다.

- 상자 색 HSV 범위(main `zone_perception_v1`, PR #163 `top_zone_v2` 사본)에 들어가는 화소 비율, 가장 가까운 CIELAB 색 거리, 로봇 화소와의 근접 수.
- pair/trio는 색만이 아니라 모양(막대 15:1, 손잡이 달린 직사각형 2.4:1, 삼각 윤곽)으로 구분된다. 크림색 틀의 색상은 노랑과 가깝지만 채도가 낮아(렌더 S≈60, 노랑 범위 S≥140) 노랑 범위에 들지 않는다.
- can(지름 약 10 px)과 tile(약 17×11 px)은 TOP에서 작다. 같은 색 물체가 닿으면 한 덩어리로 보일 수 있으니 같은 종류끼리는 떨어뜨려 둔다.
- 종류별 `visual`(TOP 크기 px, 가로세로비, 무조명 HSV, 표시)은 `catalogue_record()`에 있다.

## 장면 연결 (opt-in)

```python
from sim.zone_cargo_scene import CargoZoneScene
scene = CargoZoneScene.from_cargo_config('zone_wide', 11, goal={'A': {'red': 1}},
    cargo=[{'item_id': 'crate1', 'kind': 'heavy_crate', 'pose': [2.5, .2, 0.]}])
```

화물이 없으면 `ZoneScene`과 바이트 단위로 같은 XML을 만든다(`zone_open` v2, `zone_wide` v1의 XML 해시를 테스트로 고정). 화물 pose는 상자 pose처럼 설정 전용이며 로봇에게 가지 않는다. weld/equality는 추가하지 않는다.

## 정답 교사와 검사

`scripts/cargo_formation_teacher.py`는 1–3대 **교사** 실행기다. 정답 pose·IK·접촉력을 쓰므로 학생/RGB 성공으로 보고하지 않는다. 운반은 virtual structure(공통 기준 화물 궤적, 역할별 offset, 공통 포화 비율, 오차가 크면 기준 정지)이고, 파지 뒤 양쪽 손가락 접촉력(각 ≥0.5 N)을 확인해야 들기로 넘어간다. 매 physics step마다 equality 활성화가 0인지 검사한다.

```bash
.venv-sim/bin/python -m scripts.probe_zone_cargo --probe pair_crate --output outputs/zone-cargo/pair_crate
.venv-sim/bin/python -m scripts.probe_zone_cargo --sweep cal_block --output outputs/zone-cargo/sweep
.venv-sim/bin/python -m scripts.render_zone_cargo_catalogue --output outputs/zone-cargo/catalogue
```

검사 목록: `solo_box`·`solo_can`·`solo_tile`(1 m 운반), `pair_beam`·`pair_crate`·`trio_frame`(0.8 m → 90° 회전 → 0.5 m), 긴 경로 `*_long`(4.0 m, 90° 회전 두 번), 한 대 적은 `solo_beam`·`solo_crate`·`duo_frame`(들기 후 운반 시도). `--static-hold --hold-s 60`은 제자리 버티기, `--drive-check`는 구동 부작용 검사다.

## 접촉 profile과 미끄러짐

- `local_contact_fine`의 마찰은 감쇠형(`solreffriction 0 -6000`)이라, 들고 있는 동안 물체가 집게 사이로 천천히 내려간다. 수치적 soft-contact creep이며, 약 0.9–1.2 mm/s·kg를 측정했다.
- 화물 장면은 opt-in profile **`cargo_noslip_v1`**(`sim/zone_cargo_contact.py`)을 고를 수 있다. `local_contact_fine`에 MuJoCo `noslip_iterations 10`만 더한 것이다. 60초 정지 미끄러짐은 0.51 mm 이하, 4 m 운반은 1.2 mm 이하였다. 마찰 계수·손가락 힘·관절 한계는 같고 weld도 OFF다. 전역 옵션이지만 구동 응답과 쉬는 상자에 차이가 없음을 측정했다.
- 원인 분리와 합격 검사는 실험 기록 8절에 있다.

```python
CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=[...], contact_profile='cargo_noslip_v1')
```

```bash
.venv-sim/bin/python -m scripts.probe_zone_cargo --probe trio_frame_long --contact-profile cargo_noslip_v1 --output outputs/zone-cargo/tri-long
.venv-sim/bin/python -m scripts.probe_zone_cargo --probe pair_crate --static-hold --hold-s 60 --contact-profile cargo_noslip_v1 --output outputs/zone-cargo/hold
```

## 알려진 물리 한계

- 뒤로 가는 로봇의 명령 한계(forward ≥ −0.05)가 팀 속도를 제한한다.

## 아직 안 한 것

구역 교사(`scripts/zone_teacher.py`)·구역 실행기·지도 연동, RGB 탐지기 지원, 벽·문이 있는 경로의 팀 운반. 제안은 실험 기록의 "다음 단계"를 본다.
