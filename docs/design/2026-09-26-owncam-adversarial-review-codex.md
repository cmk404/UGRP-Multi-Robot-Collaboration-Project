# 2026-09-26 자기 카메라 PR 반대 검토 — Codex 검토

- **작성:** Codex CLI (xhigh, 읽기 전용). 파일 변경·브랜치 생성·시뮬레이션·댓글 작성은 하지 않았다(본문 둘째 문단).
- **날짜:** 2026-09-26
- **검토 대상:** PR #177 `a9f5d6e`(자기 카메라 위치 추정), PR #176 `edd075d`(wrist 구역 스킬). 두 PR은 모두 병합됐다(2026-09-26 `gh pr view`로 `MERGED` 확인). 본문 행 번호는 검토 시점의 해당 브랜치 기준이며 이후 커밋에서 바뀌었을 수 있다.
- **상태:** 검토 결과; 수정은 PR 181(스킬)·PR 178(폐루프)에서 추적. 이 문서의 지적이 해결됐다는 뜻은 아니다.
- **링크 주의:** 본문의 근거 링크는 검토 당시 로컬 worktree의 절대 경로(`/Users/changmin/projects/ugrp-wt/zone-owncam-skill/...`, `.../zone-owncam-loc/...`)와 공용 `outputs/` 경로를 가리킨다. 이 worktree 경로는 보존되지 않으므로 링크가 열리지 않을 수 있다. 원문 보존을 위해 그대로 남겼다. 같은 파일은 저장소의 `harness/`·`scripts/`·`sim/`·`tests/`·`experiments/` 상대 경로에서 찾는다.
- 아래 본문은 Codex 출력 원문 그대로다. 비밀정보·토큰은 없었다. 끝의 Codex 세션 ID는 인증 정보가 아니라 로컬 재개용 식별자라서 출처로 남겼다.

---

**판정: 두 PR의 결과를 현재 상태에서 “wrist-only M1 성공”으로 보고하면 안 됩니다.** 문서에는 이 한계가 대체로 명시돼 있지만, 성공 판정·입력 검증·결과 집계에서 이를 강제하지 못하는 경로가 있습니다.

검토 대상은 로컬 ref의 **#177 `a9f5d6e`**, **#176 `edd075d`**입니다. 로컬 `origin/main`에는 두 PR의 병합 기록이 이미 있습니다. 읽기 전용 제한으로 fetch는 생략했고, GitHub 조회는 실패했습니다. 파일 변경·브랜치 생성·시뮬레이션·댓글 작성은 하지 않았습니다.

아래 심각도는 **최종 M1 채택 기준**입니다.

1. **Blocker — GT pose가 성공 판정까지 도달하며, M1 제외 표시가 전체 결과 경로에 유지되지 않습니다.**

   러너는 `base_xyz()`·`base_rpy()`를 매번 읽어 제어기에 전달합니다. `PoseEstimate.source`는 비어 있지 않은 문자열이면 통과하고, 스킬은 출처를 기록하기만 합니다. GT pose도 슬롯 판정과 `OWN_RGB_PLACEMENT_IN_SLOT` 종료에 사용됩니다.  
   근거: [run_zone_owncam_skill.py:88](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:88), [wrist_zone_skill.py:71](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:71), [wrist_zone_skill.py:206](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:206), [wrist_zone_skill.py:306](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:306).

   원본 러너의 `counts_as_m1=False`는 올바른 보호입니다. 하지만 실제 TensorBoard 파생 파일에는 이를 제외한 채 일반적인 `success:true`가 기록됩니다. v2 seed 518은 로봇의 배치 확인 실패인데도 이 값이 참입니다. 설명 문자열에는 NOT M1이 있지만, 일반 성공 지표만 집계하면 구분이 사라집니다.  
   근거: [러너:283](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:283), [v1 파생 결과:5](/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/tensorboard-view/owncam-v1-s503/result.json:5), [v2 seed 518 파생 결과:5](/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/tensorboard-view-v2/v2-s518/result.json:5).

   **실패 시나리오:** 기존 스킬 종료 사유나 `success`를 그대로 M1 대시보드에 연결하면 GT 도움을 받은 배치가 성공으로 집계됩니다.  
   **수정:** 진단 성공과 M1 성공을 별도 필드로 만들고, M1 실행기·판정기·exporter 모두에서 GT 출처를 거부하십시오. `counts_as_m1`, pose 출처, 입력 계약을 파생 결과에도 필수로 보존해야 합니다.

2. **High — 최종 배치 확인은 wrist 카메라 검증을 우회합니다.**

   `look_back → confirm_placement`는 관측의 camera·robot_id·frame_id·영상 해시를 검사하지 않고 이미지와 PWM만 추출합니다. 기존 N7의 엄격한 `_validate_observation()`을 거치지 않습니다. 카메라 거부 테스트도 grasp 단계만 검사합니다.  
   근거: [wrist_zone_skill.py:283](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:283), [wrist_zone_skill.py:298](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:298), [test_wrist_zone_skill.py:89](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/tests/test_wrist_zone_skill.py:89).

   **확인:** 시뮬레이터 없이 검출기만 대체한 검사에서 `camera=nav_cam`과 `camera=cctv_top` 모두 `OWN_RGB_PLACEMENT_IN_SLOT`을 반환했습니다. 이는 입력 경계 우회 재현이며, 실제 TOP 영상의 검출 정확도를 시험한 것은 아닙니다.

   **실패 시나리오:** 연결 오류로 다른 카메라나 오래된 프레임이 들어와도 마지막 성공 확인에 사용됩니다.  
   **수정:** 모든 단계의 공통 진입점에서 wrist 소유권·프레임 신선도·해시·명령 이력을 검증하고, 특히 `look_back`의 음성 테스트를 추가하십시오.

3. **High — 정적 지도에 없는 정확한 화물 위치가 주문서로 유입됩니다.**

   같은 `scenario['pickup_xy']`가 화물의 `qpos` 설정과 `OrderSheet` 생성에 사용됩니다. 제어기는 이를 이용해 상자 40cm 앞까지 이동합니다. 따라서 GT pose를 정상 localizer로 교체해도 **시나리오의 정확한 화물 배치 정보**가 남습니다.  
   근거: [run_zone_owncam_skill.py:107](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:107), [run_zone_owncam_skill.py:119](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:119), [wrist_zone_skill.py:218](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:218). 정적 지도 정의에는 구역·벽·슬롯이 있지만 이 시나리오별 화물 좌표는 없습니다: [zone_arena.py:212](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/sim/zone_arena.py:212).

   추가로 RGB 면 추정이 실패하면 “동쪽을 향해 잡는다”는 접근 관례를 실제 물체 면 법선으로 대체하고 `ready=True`를 반환합니다. 접근 방향 관례가 물체의 실제 방향을 보장하지는 않습니다.  
   근거: [wrist_zone_skill.py:104](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:104), [wrist_zone_skill.py:228](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:228).

   **실패 시나리오:** 상자를 옮기거나 회전시키면, 시야로 확인하지 않은 위치·법선을 계속 신뢰합니다.  
   **수정:** 지도상의 픽업 **구역**과 목적지만 제공하고 실제 화물 위치·방향은 wrist RGB로 추정하십시오. 면 추정 실패는 재관측 또는 중단으로 처리해야 합니다.

4. **High — 위치 추정의 불확실성과 관측 공백을 제어기가 표현할 수 없습니다.**

   localizer는 공분산과 `since_tag_s`를 출력하지만 `PoseEstimate`에는 위치·yaw·출처만 있습니다. 주행과 슬롯 성공 판정에는 추정 시각·태그 부재 시간·불확실성 검사가 없습니다.  
   근거: [owncam_localizer.py:319](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/harness/owncam_localizer.py:319), [wrist_zone_skill.py:63](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:63), [wrist_zone_skill.py:319](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/wrist_zone_skill.py:319).

   실제 carry 조건은 태그 가시율 0%였고, 사전 등록 test G3 위치 오차는 p90 **26.2cm**입니다. 슬롯 판정 허용치는 ±6cm입니다.  
   근거: [위치 추정 보고서:40](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/README.md:40), [같은 보고서:58](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/README.md:58).

   **실패 시나리오:** 가려진 상태에서 오래 추측 항법을 한 뒤 잘못된 위치로 문을 통과하거나 슬롯 배치를 선언합니다.  
   **수정:** 추정 시각·초기화 상태·불확실성·마지막 유효 관측을 인터페이스에 포함하고, 한계 초과 시 정지·재관측하도록 연결하십시오.

5. **Medium — 현재 물리 기록은 무충돌·비관통 파지의 증거로 부족합니다.**

   접촉·weld·상자 높이 통계는 physics step 반복문 **밖에서** 갱신됩니다. 매크로 중간의 짧은 충돌은 놓치며, 벽 접촉도 `r1`과 벽만 세므로 운반 화물과 벽의 접촉은 빠집니다.  
   근거: [run_zone_owncam_skill.py:164](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:164).

   실제 XML에는 손가락–상자 마찰계수 3.4와 `solreffriction="0 -6000"`이 적용됩니다. 이는 기존 프로필을 재사용한 것이며 이번 PR의 몰래 변경은 아닙니다. 그러나 관통 깊이·접촉력·실물 마찰 보정 자료는 이 실험에 없습니다. probe의 유지 판정도 높이와 gripper 상대 위치로 합니다.  
   근거: [dispatch_contact_profile.py:37](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/sim/dispatch_contact_profile.py:37), [probe_owncam_carry_view.py:272](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/probe_owncam_carry_view.py:272).

   **실패 시나리오:** 높은 마찰이나 연성 접촉의 관통에 의존한 유지가 정상 파지로 해석되거나, 순간 충돌이 “벽 접촉 0”으로 보고됩니다.  
   **수정:** 평가 전용으로 매 physics step의 화물·손가락·벽 접촉, 최소 접촉 거리, 힘과 상대 미끄러짐을 기록하고 허용 기준을 사전에 고정하십시오. **현재 증거로 아티팩트 악용을 확정할 수는 없습니다.**

6. **Medium — 실행 번들·소스 식별이 실행 버전 관리 계약에 못 미칩니다.**

   스킬 PR은 번들과 workflow 미등록을 명시합니다. 러너의 `source_sha`는 실행 **종료 시점 HEAD**이며, 의존 소스 해시와 패키지 환경을 묶지 않습니다. 실제 501 결과는 `9d3e654`를 기록하지만 README는 `d016c04`에서 실행했다고 설명합니다. 두 커밋 사이 런타임 변경은 없음을 확인했으므로 행동 변경 증거는 아니지만, 기록 SHA가 시작 소스를 직접 증명하지 못합니다.  
   근거: [스킬 보고서:19](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/experiments/2026-09-25-zone-owncam-skill/README.md:19), [러너:288](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:288), [501 결과:28](/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/cohort-d016c04/501/result.json:28).

   위치 추정 최초 16개 기록도 종료 시점 SHA·dirty 상태를 기록했습니다. 설명은 있지만 당시 미커밋 파일의 바이트까지 복원할 수는 없습니다.  
   근거: [위치 추정 보고서:22](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/README.md:22), [실행 버전 계약:25](/Users/changmin/projects/ugrp/docs/execution_versioning.md:25).

   **실패 시나리오:** 같은 skill 이름이나 종료 SHA만 보고 서로 다른 실행기·보정값·환경의 결과를 합칩니다.  
   **수정:** 시작 시점 SHA와 의존 소스·지도·보정·물리·명령 타이밍·환경 해시를 새 번들에 고정하고 종료 때 다시 검증하십시오.

7. **Medium — “실행 전 사전 등록”은 항목별로 성립 범위가 다릅니다.**

   커밋 시각과 로컬 파일 생성 시각을 대조한 결과입니다. 모두 KST이며, 파일 시각은 독립적인 변조 방지 증명은 아닙니다.

   | 항목 | 커밋 | 관측된 시작 | 판정 |
   |---|---|---|---|
   | 위치 추정 seed·episode | `4af81db`, 22:24:13 | 첫 raw 22:24:21 | 녹화 전 고정 |
   | 위치 추정 게이트 | `13c64c4`, 22:27:52 | 첫 raw 22:24:21 | **녹화 시작 후 고정** |
   | localizer 최종 보정 | `efdf5f4`, 23:33:15 | 첫 test 검출 23:33:46 | 보존된 test 처리보다 앞섬 |
   | v1 seed·300초/900단계 | `a868328`, 22:31:53 | 501 시작 22:42:05 | 실행 전 고정 |
   | v2 seed·판정·중단 규칙 | `edd075d`, 23:55:10 | 511/512 시작 23:55:18 | 실행 전 고정 |

   위치 추정 문서는 “위치 추정 결과 전 고정”이라고 정확히 좁혀 설명합니다. 이를 **모든 실행 전 등록**이라고 확대하면 안 됩니다. v1의 완전한 코호트 설명은 실행 종료 후인 `59b736d`에서 처음 추가됐습니다.  
   근거: [위치 추정 보고서:30](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/README.md:30), [스킬 러너:38](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/scripts/run_zone_owncam_skill.py:38), [v2 등록:122](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/experiments/2026-09-25-zone-owncam-skill/README.md:122).

   **실패 시나리오:** 부분적으로 사전 고정한 개발 실험이 완전한 사전 등록 시험으로 인용됩니다.  
   **수정:** 데이터 수집 전 등록과 test 채점 전 동결을 구분하고, 다음 코호트에는 seed·주지표·중단·재시도·제외 규칙을 하나의 사전 커밋으로 남기십시오.

8. **Medium — 집계기가 누락된 시험 seed를 조용히 제외합니다.**

   `build_results.py`는 결과 파일이 존재하는 seed만 수집하고 그 길이를 `n`으로 사용합니다.  
   근거: [build_results.py:100](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/experiments/2026-09-25-zone-owncam-skill/build_results.py:100).

   **실패 시나리오:** 실패 실행이 결과 파일을 남기지 못하면 사전 등록 분모 5가 자동으로 줄어듭니다. **이번 보존 자료에서는 501–505 모두 존재하므로 실제 누락은 발견하지 않았습니다.**  
   **수정:** 등록 seed 전체를 강제 대조하고, 결과 없는 seed는 `missing/infrastructure_failure`로 남기며 완료 코호트 보고를 차단하십시오.

9. **Low — 위치 추정 보고서의 timestep이 실제 실행과 다릅니다.**

   README는 0.002초지만 manifest와 XML은 **0.00025초**입니다. 8배 차이입니다.  
   근거: [README:18](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/README.md:18), [실제 manifest:42](/Users/changmin/projects/ugrp/outputs/owncam-loc-20260925/raw/dev-tc-s11/manifest.json:42).

   **실패 시나리오:** 문서의 timestep으로 재현하면 접촉과 유지 결과가 달라질 수 있습니다.  
   **수정:** 실제 적용값으로 정정하고 보고서 조건표를 manifest에서 생성하십시오.

재실행·제외 여부에서는 다음까지 확인했습니다.

- **501–505:** 완료 결과는 각각 1개입니다. 502–505의 첫 시작은 dirty-tree 검사에서 거절된 뒤 재시도됐습니다. 제어 시작 전 거절이므로 완료 시험을 다시 돌린 증거는 아닙니다. [cohort.log:3](/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/cohort-d016c04/cohort.log:3).
- **511–520:** 로컬 raw에 10개 모두 있으며 GT 슬롯 배치 **9/10**, 로봇 자체 배치 확인 **8/10**, 모두 `counts_as_m1=false`입니다. 이 결과는 검토 ref 이후 생성된 로컬 증거입니다. [cohort.log:15](/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925/cohort-v2-edd075d/cohort.log:15).
- dev seed 반복 조정은 별도 표시돼 있습니다. 위치 추정의 look-only 분석과 pitched-carry 진단도 **사후 분석**으로 명시돼 있습니다. 완료 test의 반복·삭제 증거는 발견하지 못했지만, 보존되지 않은 실행까지 배제할 수는 없습니다.

입력 경계와 버전 보존에서 확인된 정상 사항도 있습니다.

- **localizer/PnP 자체에서 runtime GT·측정 관절·접촉·TOP 프레임 읽기는 발견하지 못했습니다.** 외부 파라미터는 발행 PWM의 FK를 사용합니다. `sim.masterpi_camera_profile`은 고정 보정값이며, `plan_path` 재사용도 전달받은 지도·좌표만 사용하는 함수입니다. [wall_tags.py:30](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/harness/wall_tags.py:30), [visual_arm.py:134](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/visual_arm.py:134).
- GT 기반 운동·측정 보정은 dev 자료의 **오프라인 보정**으로 문서화됐고, 출처 32개 파일 해시가 일치했습니다. 허용된 예외에 해당합니다. [calibration_frozen.json:214](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/calibration_frozen.json:214).
- 기존 지도 3개, N7 스킬, 물리 프로필은 기준 커밋과 바이트가 같고, v1 스킬도 v2에서 보존됐습니다. 태그는 world의 `contype=0`, `conaffinity=0`, `mass=0` geom으로 추가됩니다. 물리 변경 코드는 발견하지 못했습니다. 기존 물리 동등성 테스트의 범위는 한 지도·seed의 1.5초 궤적입니다. [zone_landmarks.py:185](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/sim/zone_landmarks.py:185), [test_zone_landmarks_sim.py:32](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/tests/test_zone_landmarks_sim.py:32).
- **`eval_only`는 접근 차단이 아니라 디렉터리 구분입니다.** 같은 사용자 프로세스에서 읽을 수 있습니다. 현재 localizer가 읽는 경로는 발견하지 못했지만, 경계 테스트의 파일-open 감시는 `load_inputs()`에만 적용되고 `localize()` 호출 전 해제됩니다. [test_owncam_localizer.py:78](/Users/changmin/projects/ugrp-wt/zone-owncam-loc/tests/test_owncam_localizer.py:78).
- `TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`의 TOP은 **상자 윗면 기하**를 뜻합니다. 해당 종료 사유 자체는 TOP 카메라 누출 증거가 아닙니다. [visual_box_skill.py:614](/Users/changmin/projects/ugrp-wt/zone-owncam-skill/harness/visual_box_skill.py:614).

원본 해시 검사에서는 위치 추정 기록 162개 항목·영상 **8,770장**, 스킬 15회 결과 묶음·영상 **6,018장**이 기록과 일치했습니다. 물리 재실행, 관통·힘 검증, 전체 영상 시청, TensorBoard 화면 검증, 실제 wrist-only 폐루프 문 통과는 수행하지 않았습니다. 따라서 현재 입증 범위는 **교사 주행의 오프라인 위치 추정과 GT pose를 사용하는 스킬 격리 시험**까지입니다.

Codex session ID: 01a0d963-1e2d-71c0-bcbd-f98ef3616566
Resume in Codex: codex resume 01a0d963-1e2d-71c0-bcbd-f98ef3616566
