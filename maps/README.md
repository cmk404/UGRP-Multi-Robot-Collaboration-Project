# 실행용 지도 목록

ACT 맵 확장을 위한 [맵·분할·정적 검토 v1](../docs/act_map_suite.md)도 제공한다.
새 맵 16개와 기존 회귀 6개를 생성/로딩하고 같은 production 물리 장면에서 검사한다.
새 지도는 아직 dispatch/ACT 운반 실행에 연결하지 않았으며 기하 경로와 실제 성공을 구분한다.

[다중 물건 파일럿](../docs/multi_object_pilot.md)은 1~8개 물건의 대표 12조건과
선행 작업·임시 적치·반복 배분을 정의한다. [물리 장면·정적 검토](../docs/multi_object_scenes.md)까지 제공하며 실제 다중 물건 운반은 미연결이다.

앞서 만든 예시 지형 5종과 완전 차단 대조를 `maps/`에서도 바로 불러올 수 있다.
같은 벽 배치를 기존 단독 주행용과 공동 운반용 형식으로 등록했다.

| 지형 | 단독 주행용 | 공동 운반용 |
| --- | --- | --- |
| 좁은 문·통로 | [narrow-door.json](navigation/narrow-door.json) | [narrow-door.json](pair_navigation/narrow-door.json) |
| L자 모퉁이 | [l-corner.json](navigation/l-corner.json) | [l-corner.json](pair_navigation/l-corner.json) |
| S자 연속 굴곡 | [s-bends.json](navigation/s-bends.json) | [s-bends.json](pair_navigation/s-bends.json) |
| 엇갈린 장애물 | [staggered-obstacles.json](navigation/staggered-obstacles.json) | [staggered-obstacles.json](pair_navigation/staggered-obstacles.json) |
| 막힌 길·우회 분기 | [blocked-branch.json](navigation/blocked-branch.json) | [blocked-branch.json](pair_navigation/blocked-branch.json) |
| 완전 차단 대조 | [fully-blocked.json](navigation/fully-blocked.json) | [fully-blocked.json](pair_navigation/fully-blocked.json) |

기존 단독 지도 [open](navigation/open.json), [slalom](navigation/slalom.json),
[narrow](navigation/narrow.json)도 그대로 사용할 수 있다. 기존 `narrow`는 통과 불가능한
좁은 통로 대조이고, 새 `narrow-door`는 폭 56 cm의 별도 예시다.

![예시 지형 도면](../experiments/2026-09-14-pair-terrain-examples/gallery/overview.png)

## 두 형식의 차이

| 항목 | `navigation/` | `pair_navigation/` |
| --- | --- | --- |
| 실행기 | `scripts/run_known_map_navigation.py` | `scripts/run_pair_navigation.py` |
| 지도 인자 | `--map-file` | `--map` |
| 사용 조건 | 짐 없는 로봇 1대 | 두 로봇과 공동 하중 |
| 계획 공간 | 기존 반경 0.18 m + 여유 0.04 m | 기존 직사각형 0.45 × 1.07 m, 여유 포함 |
| 목표 | 중심에서 반경 0.18 m | 목표 중심과 최종 회전 방향 |

장애물 위치·크기·높이, 경계, 카메라 보정, 시작 구역과 목표 중심은 두 형식에서
같다. 단독 주행용에는 기존 2.5 cm 격자와 원형 footprint를 유지한다.
공동 운반용에는 예시의 6 cm 격자·하중 footprint·목표 방향을 보존한다.
따라서 같은 지형에서도 계산 경로나 회전 방식은 달라질 수 있다.
공동 운반은 이후 여섯 지형을 실제 시험했다. `impratio=10`, 고정 시작 각 1회에서
통과 가능 지형 완주 0/5, 완전 차단의 경로 없음·공동 정지 1/1이었다.
네 지형은 영상 인식 중단, 엇갈린 장애물은 경유점 정체·파지 이탈·예산 소진으로 실패했다.
[실제 운반 시험 결과와 영상](../experiments/2026-09-14-pair-terrain-trials/README.md)을 참고한다.
이후 영상 추적·제한된 재획득을 추가한 두 후보는 같은 조건에서 각각 완주 1/5,
완전 차단 정지 1/1이었다. 좁은 문만 운반·내려놓기를 완료했으며,
나머지는 가림·바퀴 재획득 실패 또는 정체·파지 이탈이 남았다.
[후보 비교 결과와 영상](../experiments/2026-09-14-pair-vision-recovery/README.md)에 전체 기록을 보존했다.
최종 `robust` 후보는 같은 고정 시작·물리 조건에서 **운반·내려놓기 5/5, 완전 차단 정지 1/1**을
통과했다. 네 바퀴의 기하 추적·중앙 빔 인식·자기 카메라 운반 감시를 사용한다.
[최종 전체 결과와 영상](../experiments/2026-09-14-pair-transport-robustness/README.md)을 참고한다.
원본 예시·정적 갤러리 기록의 시험 전 상태는 보존한다. 단독 주행 완주와 LLM 경로 선택은
이 시험의 검증 범위가 아니다.

## 불러오기

저장소 루트에서, 기존 시뮬레이션 환경으로 실행한다. 출력 이름은 매번 새로 정한다.
실행 전 소스와 설정을 커밋한다. worktree에서는 기본 프로젝트 가상환경의 절대 경로를
사용하고, Ubuntu에서는 설치된 환경의 `python`과 기존 headless 렌더 설정을 사용한다.

단독 주행에서 S자 지형을 선택하는 예:

```sh
.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run s-bends-solo-NEW -- \
  .venv-sim-worker-mac/bin/mjpython -m scripts.run_known_map_navigation \
  --map-file maps/navigation/s-bends.json \
  --case-json '{"case_id":"s_bends_solo","robot_id":"r1","start_xy_m":[0.58,-2.0],"start_yaw_deg":0}' \
  --out-dir outputs/s-bends-solo-NEW
```

`case-json`은 평가·초기 배치 전용이며 실행 중 정답 위치 입력이 아니다.
실제 단독 주행과 공동 운반의 시작 장면·표시 방식은 각 기존 실행기를 따른다.

공동 운반의 실행 지도 선택은 다음과 같다. `--grasp-model-dir`은 기존 실행기에서
사용하던 **전체 grasp 모델 폴더**로 지정한다. 정적 갤러리의 `_setup_metadata`는
초기화 JSON만 들어 있으므로 실제 파지 모델 폴더로 사용할 수 없다.

```sh
.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run s-bends-pair-NEW -- \
  .venv-sim-worker-mac/bin/mjpython scripts/run_pair_navigation.py \
  --map maps/pair_navigation/s-bends.json \
  --grasp-model-dir /absolute/path/to/models/grasp \
  --vision-mode robust --impratio 10 --contact-profile retention --budget 750 \
  --out-dir outputs/s-bends-pair-NEW
```

위 공동 운반 명령은 `robust`, `impratio=10`, `retention` 조건을 명시한다.
retention은 네 손가락 접촉의 수치적 미끄러짐을 줄이고 물리 계산 간격을 0.25ms로 한다.
같은 고정 시작에서 5분 유지와 여섯 지형을 검증한 [결과와 한계](../experiments/2026-09-15-pair-grasp-retention/README.md)를 참고한다.
RGB로 지속 미끄러짐이 보이면 내려놓고 한 번 재파지하며, 재발하면 내려놓고 종료한다.
기존 접촉 조건은 `--contact-profile baseline`으로 구분한다. robust + impratio=10은
프로필을 생략하면 retention을 선택한다.
옵션을 생략했을 때의 기존 CLI 기본값은 `legacy`, `impratio=1`로 유지한다.
이전 영상 후보를 재현하려면 모드를 `temporal` 또는 `temporal-edges`로 지정한다.

## 지도와 정적 장면만 확인하기

[카탈로그](pair_navigation/catalog.json)에 제목·지도 경로·단독 주행 대응 파일·
원본 예시 출처가 있다. 갤러리 생성기는 이제 이 카탈로그를 기본으로 읽는다.

```sh
.venv-sim-worker-mac/bin/python scripts/build_pair_terrain_gallery.py \
  --geometry-only --out-dir outputs/terrain-plans-NEW

.venv-sim-worker-mac/bin/python scripts/verify_terrain_maps.py \
  --out-dir outputs/terrain-map-check-NEW

.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run terrain-map-preview-NEW -- \
  .venv-sim-worker-mac/bin/mjpython scripts/verify_terrain_maps.py \
  --render --out-dir outputs/terrain-map-preview-NEW
```

마지막 명령은 단독 주행용 6개 지도의 초기 정지 장면만 렌더한다. 이동이나 파지는
실행하지 않는다. 공동 운반 정적 장면은 같은 세션 래퍼로
`scripts/build_pair_terrain_gallery.py --out-dir outputs/pair-preview-NEW`를 실행한다.

원본 예시와 당시 검증 기록은 [예시 지형 기록](../experiments/2026-09-14-pair-terrain-examples/README.md)에
보존한다. 공동 운반 등록 지도는 원본과 바이트 단위로 같고, 단독 주행 지도는
그 벽 배치에서 파생했다. 두 버전을 수정할 때 `verify_terrain_maps.py`로 출처·배치
일치를 검사하고, 새 실험 결과는 새 SHA와 함께 기록한다.

## 구역 벽 AprilTag 지도 (2026-09-25)

`zones/zone_wide_door_tags_v1.json`, `zones/zone_wide_two_doors_tags_v1.json`, `zones/zone_wide_corridor_tags_v1.json`은 같은 이름의 base 지도에 벽·문기둥 tag36h11 목록(`landmarks`)을 더한 새 버전이다.
- base 지도 파일은 바꾸지 않는다. 태그 지도는 base의 `static_map_sha256`을 기록한다.
- 태그는 벽면에 붙인 시각 전용 geom이며 물리는 같다. 화물과 로봇에는 태그를 붙이지 않는다.
- 배치 파라미터와 새 버전을 만드는 방법은 [자기 카메라 위치 추정](../docs/zone_owncam_localization.md)을 따른다.
