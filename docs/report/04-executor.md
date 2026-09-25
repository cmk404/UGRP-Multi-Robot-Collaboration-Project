# 04 실행기

**기준 커밋: `origin/main` `1cd9ea1` (2026-09-26 확인).**

## 4.1 결정: 자기 카메라만으로 성공해야 한다

사용자 결정(2026-09-25)의 원문은 "본인 카메라만 보고 성공해야지"다
([docs/decision_log.md](../decision_log.md) "최종 연구의 실행 입력" 항목).
같은 항목이 교사의 역할을 한정한다.

> 정답 교사 실행기(`scripts/zone_teacher.py`, 시뮬레이터 `xpos`를 읽음)는 시연과 학습 표적
> 생성에만 쓴다. 연구 실행기가 아니며, 교사 주행은 로봇 성공으로 세지 않는다.

## 4.2 출발점: 자기 카메라 실행기가 없었다

두 읽기 전용 조사가 같은 결론을 냈다.

> **자기 RGB와 정적 지도만으로 지도 위 위치를 추정하는 코드와 실험은 저장소에 없습니다.**
> 지도 기반 주행은 모두 TOP으로 위치를 추정합니다(`harness/known_map_navigation.py:3-6`,
> `harness/heading_map_navigation.py:146`, `harness/pair_navigation.py:256-262`,
> `harness/map_goto.py:7`).
> — [자기 카메라 역량 목록](../design/2026-09-25-own-camera-inventory-claude.md)

오픈소스 조사도 "현재 A* 계획기 `harness/map_goto.py`는 현재 위치를 호출자에게서 받는다.
그 위치를 공급하는 `heading_map_navigation.py`와 `run_known_map_navigation`은 공용 TOP
카메라로 위치를 잡는다"고 확인했다
([OSS survey](../design/2026-09-25-own-camera-oss-survey-claude.md)).

자기 카메라만으로 증명된 것은 **벽·문·지도가 없는 장면의 단독 상자 파지·운반·내려놓기**뿐이고,
그 N7 5/5(2026-09-09-markerless-n7)조차 주행에는 시뮬레이터 전용 `nav_cam`을 썼다. 공동 운반은
모두 TOP에 의존하고, trio는 정답 교사로만 성공했다(같은 역량 목록).

권장 해법은 혼합 방식이었다. 벽·문 AprilTag + 자기 명령을 운동 모델로 쓰는 파티클 필터 +
기존 A*·RGB 스킬 + LeRobot ACT이며, "새로 필요한 의존성은 거의 없다"(OSS survey).
라이선스·플랫폼 차단 목록도 남겼다: YOLO(AGPL), ORB-SLAM3·ViSP(GPL),
FoundationPose(CUDA 필요), pi0(GPU 24GB 이상).

## 4.3 M1 계획

첫 마일스톤 M1은 사용자 결정으로 고정됐다.

> `zone_wide_door`에서 로봇 1대가 청록 상자 1개를 손목 카메라만으로 문 너머까지 배달한다.
> TOP은 채점과 영상에만 쓰고, weld는 OFF다.
> ([docs/decision_log.md](../decision_log.md) 2026-09-25)

M1은 두 트랙으로 나뉘어 진행됐다.

| 트랙 | 담당 PR | 범위 |
|---|---|---|
| 위치 추정 | #177(병합), #178(진행) | 벽 AprilTag 지도, wrist 어안 PnP, 발행 명령 파티클 필터, 폐루프 문 통과 |
| 시야·스킬 | #176(병합), #181(진행) | 운반 자세 probe, `wrist_zone_skill_v1/v2`, 슬롯 배치 |

## 4.4 위치 추정 (PR #177, 병합)

[experiments/2026-09-25-zone-owncam-loc](../../experiments/2026-09-25-zone-owncam-loc/README.md),
설계는 [docs/zone_owncam_localization.md](../zone_owncam_localization.md).

입력 경계: `robot_cam` 640×480 raw 어안(JPEG q90)만. **TOP·nav_cam은 입력으로 쓰지 않았다.**
외부 파라미터는 `harness.visual_arm.camera_extrinsics(발행 PWM)`로 만들고 측정 관절은 쓰지
않는다. 주행은 정답 교사가 했고, 이는 **오프라인 평가**다.

### 사전 등록 게이트 결과

`thresholds.json`은 결과가 나오기 전에 커밋됐다(`13c64c4`). 보정·조정은 dev만 사용하고
`calibration_frozen.json`과 코드를 `efdf5f4`로 고정한 뒤 test는 한 번만 채점했다.

| 게이트 | dev | test |
|---|---|---|
| G1 문 근처 정지 둘러보기 p90 (<5 cm/5°) | 16.0 cm / 3.07° **실패** | 9.1 cm / 2.68° **실패** |
| G2 문 근처 전체 p50 (<5 cm/5°) | 3.2 cm / 0.46° 통과 | 3.2 cm / 0.27° 통과 |
| G3 문 근처 전체 p90 (<10 cm/10°) | 16.5 cm / 4.37° **실패** | 26.2 cm / 3.53° **실패** |

**원인은 상자를 든 carry 자세에서 태그가 한 번도 보이지 않은 것이다**(가시율 0%). 이때 위치는
명령 적분만으로 추정되어 문 근처 오차가 16–35 cm까지 커졌다. 수평 carry에서는 들고 있는 청록
상자가 영상 아래 약 60%를 가리고, 0.05 m 높이 벽 태그는 수평선 아래에 있어 전부 가려진다.
좌우 pan을 해도 상자가 카메라와 같이 돌아 효과가 없다.

### 사후 분해 (사전 등록 아님)

look 모드만 골라 다시 계산하면 **test 문 근처 오차는 중앙값 1.9 cm / 0.19°,
p90 5.8 cm / 0.48°**다. 자세별 태그 가시율은 search(−21°) 0.92–0.93, 수평·상자 없음
0.98–0.99, **수평 carry(상자 있음) 0.00**, 교사 low carry(−57°) 0.00이다.

사후 진단(test 채점 뒤 1회, 에피소드 1개): 상자를 든 채 손목을 **20° 숙이면** 가시율 97%,
오차 p50 2.7 cm / p90 5.4 cm(문 근처 p50 3.0 cm / p90 5.8 cm)이고 상자는 떨어지지 않았다.
30° 숙이면 가시율 7%로 떨어진다.

명령 적분만으로는 한계가 뚜렷하다. 마지막 태그 이후 10–30 s면 p90 20 cm, 30 s 이상이면
p90 28 cm / 7°다. 적재 여부로 운동 모델을 나눴고(판정 기준은 파지 높이에서 집게를 닫으라고
명령했는지), 회전 이득은 적재 0.74·무적재 1.49다.

테스트는 `tests/test_owncam_localizer.py` 12개(하위 48개), `tests/test_zone_landmarks_sim.py`
2개이며 기존 테스트까지 합쳐 72개 통과했다. `mujoco` import를 막은 subprocess에서의 합성 위치
추정과 `eval_only` 열기 차단을 포함한다.

## 4.5 폐루프 문 통과 (PR #178, 진행 중)

"로봇은 실행 중에 정답을 전혀 받지 않고, 자기 추정값만으로 `zone_wide_door_tags_v2`의 문을
지난다. 먼저 상자 없이, 다음에 상자를 공용 carry 자세로 든 채 진행한다."
학생은 `harness/owncam_drive.py`(위치 추정 + `map_goto` A*에 추정 시작점 + 추종 +
멈춰 둘러보기)이고, 사전 등록은 `experiments/2026-09-25-zone-owncam-loop/prereg.json`
(dev 31–33, test 41–43, 게이트 R1–R4, 중단 규칙)이다.
**dev 실행·소스 고정·test 실행·기록은 아직 미완료다**(PR
[#178](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/178) 체크리스트).

## 4.6 손목 스킬 v1 (PR #176, 병합)

[experiments/2026-09-25-zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md).

`harness/wrist_zone_skill.py`의 `wrist_zone_skill_v1`은 N7 `VisualBoxSkill`을 서브클래스로
재사용하고 N7 클래스와 기본값은 그대로 둔다. 추가한 것은 주문서 픽업 칸까지 자세 추정 기반
주행, carry 자세, 슬롯 앞 정렬, 놓은 뒤 자기 RGB로 다시 보기다.

### gt_stub 한계 — 이 결과는 M1이 아니다

> 이 기록의 스킬 결과는 모두 `pose_source=gt_stub_eval_only`다. 위치 추정기 자리에 시뮬레이터
> 정답을 넣은 **스킬 격리 시험**이며, 문을 통과하지 않았다. **M1 성공이 아니다.**

### 운반 자세 권고 (probe 2회)

발행 PWM 3/4/5, 집게 1500, pan 1500 기준이다.

| 자세 | PWM | 광축 | 보이는 범위 | 용도 |
|---|---|---|---|---|
| `carry_p30` | 777/2053/1646 | −22.6° | 파지 직후 바닥 0.67–3.7 m, 1.0 m 앞 벽 높이 0.005–0.15 m | **운반 기본** |
| `look_p20` | 1072/2400/1482 | −12.5° | 1.0 m 앞 벽 높이 0.15–0.32 m | 멈춰서 보기 |
| `look_p10` | 897/1998/1598 | −2.5° | 0.27–0.54 m | 높은 곳 보기 |
| 교사 carry(hover) | 도구 pitch −90° | – | 바닥 0.25–0.32 m만, **벽 태그 0개** | 교사 전용 |

정책 권고: 운반은 `carry_p30`, 문 1.5 m·0.6 m 앞에서 멈춰 `look_p20`으로
pan {1230, 1500, 1770}을 훑는다. 상자 유지는 모든 자세·0.4 m 후진·전진·90° 제자리 회전에서
놓치지 않았으나(weld OFF), probe2 기준 파지 후 약 480 SIM s에 1.6–1.8 cm 누적 미끄러짐이 있었다.

### v1 사전 등록 코호트 (501–505, `d016c04`, 각 1회, SIM 한도 300 s)

| seed | 슬롯 | 결과 | GT 파지 | GT 슬롯 | SIM s |
|---|---|---|---|---|---|
| 501 | B1 | SIM_LIMIT(접근 교착) | ✗ | ✗ | 300.4 |
| 502 | A3 | SIM_LIMIT(접근 교착) | ✗ | ✗ | 300.4 |
| 503 | A1 | OWN_RGB_PLACEMENT_IN_SLOT | ✓ | ✓ (−5.3, −12.6 mm) | 127.3 |
| 504 | C3 | SIM_LIMIT(접근 교착) | ✗ | ✗ | 300.6 |
| 505 | B2 | CARRY_VISUAL_GRASP_DRIFT | ✓ | ✗ (들고 있는 채 정지) | 173.3 |

**파지 2/5, 슬롯 배치 1/5.** 스킬의 자기 판정과 GT는 5/5 일치했고 거짓 성공 0,
weld eq_active 최댓값 0, 벽 접촉 0이다. 개발 시드 401·402는 성공률에 합산하지 않았다.

실패 원인 두 가지가 기록돼 있다.

1. **접근 교착(501·502·504)** — zone 천장 조명 때문에 상자 앞면 아래쪽이 회색으로 렌더되어
   청록 실루엣의 바닥 직육면체 적합이 흔들린다. 투영 IoU 중앙값 0.70, 가로 추정 표준편차
   6–7 mm(성공 사례는 IoU 0.85–0.93, 2.7–3.5 mm). N7 접근은 pan과 수직 오차 게이트가 같은
   프레임에서 둘 다 통과해야 전진하는데, 두 게이트가 번갈아 걸려 x≈0.25–0.28 m에서 멈췄다.
   기록은 조정자 가설 중 "대기 거리가 팔 도달 밖"이 **틀렸다**고 적고, "진행이 없을 때의 시간
   한도나 후퇴가 없다"가 맞았다고 적는다.
2. **운반 중 grasp drift(505)** — 가장 긴 운반(약 2.5 m, 86 SIM s)에서 상자가 손가락 안에서
   천천히 밀려 자기 RGB anchor 중심이 N7 한계 25.6 px를 넘어 정지했다.

## 4.7 손목 스킬 v2 (PR #181, 진행 중)

v2 사전 등록은 시험 실행 전에 커밋됐고 v1 파일은 바이트 그대로다(테스트가 `d016c04` 해시 확인).
바꾼 것은 접근(자기 RGB 추정치 퓨전, pan 이력 30 PWM, 보정만 4회 이어지면 짧게 전진, 진행 없으면
후퇴 후 재접근)과 운반(anchor 이동 18 px 초과 또는 mask IoU 0.85 미만이면 내려놓고 재장착,
짐 든 속도 상한 0.08→0.12)이다.

시험 시드 511–520, 각 1회, SIM 한도 420 s, 주 지표는 GT 슬롯 배치 성공 수/10이다.

> **GT 슬롯 배치 9/10**, GT 파지 10/10. 자기 판정과 GT 일치 9/10, 거짓 성공 0.
> 재장착 7/10(각 1회), 후퇴 0. weld 0, 벽 접촉 0.
> — PR [#181](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/181)

남은 실패는 518(물리 배치는 성공했으나 구역 B 칠 위에서 다시 보기가 상자를 검출하지 못한
보수적 거짓 음성)과 519(재장착 뒤 N7 운반 검사가 `TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`로 정지,
상자는 들린 상태)다. 개발 시드 403–406은 합산하지 않았고, v1 기준 1/5는 시나리오가 달라
참고용이다. 테스트 27개 통과.

**이 9/10도 `pose_source=gt_stub_eval_only`이고 문을 통과하지 않았으므로 M1이 아니다**
(같은 PR 범위 절).

## 4.8 nav_cam·TOP 사용처 감사

`wrist` 전용 실행기를 만들면서 N7 체인의 카메라 사용처를 표로 정리했다
([zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md) 2절).
쓰지 않기로 한 모듈은 `scripts/evaluate_gemini_team.py`(N7 러너, LLM 주행에 `nav_cam`),
`visual_drive_guard`(nav_cam 전용), `visual_navigation`·`visual_transport`(nav_cam 전용),
`visual_placement.inspect_placement`(wrist와 nav_cam 둘 다 필요)다.
재사용하는 것은 `VisualBoxSkill`(robot_cam만), `markerless_box`, `visual_box_surface`,
`visual_attachment`, `markerless_face`, `visual_floor`, `zone_color_boxes.detect_own`이다.
`solo_box_transport`(dispatch v61)는 조작은 own이지만 목표 서보가 TOP이므로 파지 설정만
재사용했다.

## 4.9 교사의 역할과 남은 경계 위반

교사는 시연·학습 표적·평가에만 쓴다([4.1](#41-결정-자기-카메라만으로-성공해야-한다)).
설계 v1은 교사를 공통 실행기로 남긴 비교의 지위를 명확히 한다.

> A2 교사는 실제 자세·접촉력·화물 높이로 단계와 행동을 결정한다. 따라서 이를 공통 실행기로
> 남긴 예비 비교는 **교사 위의 고수준 조정 실험**이며, 사용자 지시를 준수하는 로봇 본실험으로
> 보고할 수 없다.
> — [설계 v1](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/180) 4절

현재 러너에는 아직 고쳐야 할 경계가 남아 있다. Codex PR 검토는 PR #169를 **blocker**로 표시했다.

> 정답 기반 교사 결과가 모델 입력·깨우기·목표 차감으로 직접 돌아간다. 접촉력·높이로 정한
> 결과(`scripts/zone_team_teacher.py:379–428,892–902`)를 `own_jobs.status`, 동료 게시판,
> 재질문에 넣는다(`scripts/zone_dispatch_v2.py:179–193`). 실패 시나리오: RGB로 확인하지 못한
> 파지 실패가 로봇과 동료에게 통보된다.
> — [PR 검토](../design/2026-09-25-zone-pr-review-codex.md)

같은 검토는 PR #170의 RGB 결과 판정도 "평가 입력과 제안 훅에 교사 상태 의존이 남는다.
**이 훅만으로 L4는 제거되지 않는다**"고 적었다. 이 때문에 PR #170의 역할은 2026-09-25 결정으로
"로봇 입력용 역할은 폐기, 평가 보조·향후 자기 카메라판 참고용"으로 바뀌었다
([experiments/README.md](../../experiments/README.md) zone-rgb-outcome 항목).

## 4.10 M1까지 남은 차단 요인

[zone-owncam-skill](../../experiments/2026-09-25-zone-owncam-skill/README.md) 마지막 절 그대로다.

1. 자세 추정기 연결 — PR #177의 파티클 필터를 `PoseEstimate` 인터페이스에 붙여야 한다.
   지금은 stub이다.
2. 문 통과 정책 — `carry_p30` 주행과 `look_p20` 정지 관찰, 문기둥 태그(`_tags_v2`).
3. 접근 교착과 운반 drift(v2에서 개선했으나 gt_stub 조건).
4. 위치 추정 오차가 슬롯 배치(허용 ±6 cm)와 놓은 뒤 확인에 미치는 영향 측정.

설계 v1 4절은 그 뒤 필요한 단계도 적어 두었다. 자기 RGB로 판단해야 하는 네 가지는
"올바른 물건을 들었는가", "앞이 막혔는가", "pickup slot에 물건이 있는가",
"구역에 내려놓았는가"이며, 각각 남겨야 할 불확실성이 명시돼 있다(예: "집게 닫기 명령만으로
파지 확정 금지", "검출 실패를 빈 slot으로 바꾸지 않음").
