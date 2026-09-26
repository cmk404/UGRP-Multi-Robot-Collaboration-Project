# M2 2인 long_beam 운반 (실행 중 GT 없음)

workflow `zone-m2-pair` → `scripts/run_m2_pair.py`. 실험 기록: [experiments/2026-09-26-zone-m2-pair](../experiments/2026-09-26-zone-m2-pair/README.md)

## 로봇 입력

| 허용 | 금지(평가 전용 파일에만 기록) |
|---|---|
| 자기 `robot_cam` JPEG, 자기 발행 명령 | 로봇·빔의 실시간 자세 |
| 정적 태그 지도 `zone_wide_door_tags_v2`, loop-v2 위치 추정 보정(#201 e10f88d) | 손가락 접촉 힘, 빔 높이·기울기 |
| 개략 주문서: 빔 자세를 0.10 m / 10° 격자로 반올림한 값, 역할, 운반 구간·속도 | 빔 설정 자세 자체 |
| 승인된 쌍 장벽 `PairCarrySync`(자기 frame id) | 상대 로봇의 위치 |
| (플래그) 후보 상태 채널 `--status-channel on` — 사용자 결정 대기 | |

## 단계(로봇마다)

1. `approach`: `harness/pair_owncam_approach.PairApproachDriver`(M1 `OwnCamDriverV2` 하위 클래스). 둘러보기로 초기화 → 자기 추정 A* 주행(최종 heading, 대칭 envelope, 주문서 빔 발자국과 상대의 주문서 정류장을 정적 keep-out으로) → 사전 정류장(주문서 정류장 0.30 m 뒤)에서 위치·heading 허용치 + 도착 다시 보기
2. `wait_approach`: 장벽 `approach`. 두 채널 조건 모두 제한 150 s. ON이면 상대 abort/silent 조기 종료만 추가
3. pair study v3의 자기 RGB 단계(정렬·파지·들기·운반·내리기·놓기), `#200 29700fd`를 읽기 전용으로 사용

## 실패 처리

`--on-failure continue`(기본): 러너가 상대 로봇을 멈추지 않는다. 실패한 로봇만 자기 정지하고, 상대는 자기 입력으로 끝까지 행동한다. 평가 전용 `failure_propagation`에 상대의 종료 상태·지연·실패 뒤 명령 수·빔 기울기를 남긴다. `halt_all`은 pair study 동작(첫 실패에 러너가 둘 다 정지)이며 비교용이다. 러너의 정지는 어떤 로봇도 가진 정보가 아니다.

## 실행

```bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
python3 scripts/ugrp_session.py run m2 -- .venv-sim/bin/python scripts/run_m2_pair.py \
  --seed 701 --status-channel off --output outputs/zone-m2-pair/<run>
```

weld OFF. `cargo_noslip_v1`이 주 프로필이며 사용자 결정을 기다린다.
