# 참고 자료 통합 목록 (2026-09-26 기준)

열린 draft PR들이 **실제로 쓰거나 인용한 자료**를 한곳에 모은 목록이다. 최종 보고서의 참고문헌과 재사용 근거로 쓴다. 사용자 요청(2026-09-26): "PR할 때 뭐 참고했는지 기록하고! 보고서 쓸 때 필요할듯". 작성은 Kiro(`kiro/references-0926`)가 했다.

- **대상:** 2026-09-26 작업을 시작할 때 열려 있던 draft PR 15개의 본문·diff·코드 주석·실험 README·설계 문서다. 그중 #184–#187은 작업 중(07:39–07:40Z) main에 병합됐다. 기준은 origin/main `a67b6e3`과 [5절](#5-pr별-색인)의 PR head다. 그 뒤의 변경은 반영하지 않았다.
- **넣은 기준:** import, 명시적 인용, "재사용/adapted" 문구처럼 diff에 근거가 있는 것만 넣었다. 서지 정보는 원문에서 확인한 것만 적었다.
- **확인:** 2026-09-26에 외부 URL이 모두 열리는지(HTTP 200) 확인했다. 논문의 제목·저자·연도는 arXiv API, NCBI esummary, 출판사 페이지에서 가져왔다. 라이선스와 커밋은 GitHub API로, 버전은 PyPI로 확인했다.
- **나중에(작업 중이라 제외):** #194 `kiro/zone-study-core`, #207 `kiro/zone-teacher-fix`, #208 `kiro/zone-map-v3`, #209 `kiro/sim-speed`, #210 `kiro/markerless-research`, #211 `kiro/zone-owncam-memory`, #212 `kiro/disk-rules`.
  - #209와 #211의 본문에는 자체 `## 참고 자료` 절이 이미 있다.
  - #210과 #212는 작업 중 병합됐고 처리하지 않았다.
  - 작업 중 새로 열린 #227 `kiro/zone-vision-loc`도 진행 중인 작업이다. 본문에 자체 `## 참고 자료` 절이 있어 대상에서 뺐다.

## 1. 논문

| ID | 서지 | URL | 사용 PR | 쓰인 곳 |
|---|---|---|---|---|
| P1 | Y. Chen, J. Arkin, Y. Zhang, N. Roy, C. Fan, "Scalable Multi-Robot Collaboration with Large Language Models: Centralized or Decentralized Systems?", ICRA 2024. arXiv:2309.15943 | https://arxiv.org/abs/2309.15943 | #191 | 보고서 01장 선행연구 표, 06장. 원 인용은 [결정 로그](decision_log.md) 2026-09-25 절이다. TOP 카메라를 로봇 입력에서 뺀 결정의 참고 선행연구로 적혀 있다 |
| P2 | X. Guo, K. Huang, J. Liu, W. Fan, N. Vélez, Q. Wu, H. Wang, T. L. Griffiths, M. Wang, "Embodied LLM Agents Learn to Cooperate in Organized Teams", 2024. arXiv:2403.12482 | https://arxiv.org/abs/2403.12482 | #191 | 같은 표. 원 인용은 결정 로그 2026-09-25 절의 두 곳이다. TOP 제거 결정의 참고 문헌이고, 지휘자 겸임 결정의 근거("지정 지휘자 설정")다 |
| P3 | H. Zhang, W. Du, J. Shan, Q. Zhou, Y. Du, J. B. Tenenbaum, T. Shu, C. Gan, "Building Cooperative Embodied Agents Modularly with Large Language Models"(CoELA), ICLR 2024. arXiv:2307.02485 | https://arxiv.org/abs/2307.02485 | #191 | 모듈 구성(4장)과 통신 제거 실험(5.4절)을 참고했다. 원 인용은 [CoELA 통신 연구](coela_communication_study.md)와 [레퍼런스 차이표](reference_alignment.md)다 |
| P4 | J. Y. Despature, K. Shibata, T. Matsubara, "CoLF: Learning Consistent Leader-Follower Policies for Vision-Language-Guided Multi-Robot Cooperative Transport", 2026. arXiv:2602.07776 | https://arxiv.org/abs/2602.07776 | #191 | 설계 참고(두 로봇, 자기 영상 + 언어 기반 리더–팔로워). 원 인용은 [OSS 조사](design/2026-09-25-own-camera-oss-survey-claude.md)다. 조사 표에는 ID 없이 "CoLF (2026)"만 있고 URL은 출처 목록에 따로 있다. arXiv 제목으로 같은 논문임을 확인했다 |
| P5 | L. Zhang, H. Xiong, O. Ma, Z. Wang, "Multi-robot Cooperative Object Transportation using Decentralized Deep Reinforcement Learning", 2020. arXiv:2007.09243 | https://arxiv.org/abs/2007.09243 | #191 | 협동 운반 참고. 원 인용은 OSS 조사다 |
| P6 | C. P. Bechlioulis, K. J. Kyriakopoulos, "Collaborative Multi-Robot Transportation in Obstacle-Cluttered Environments via Implicit Communication", Frontiers in Robotics and AI 5:90, 2018. doi:10.3389/frobt.2018.00090 | https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7806111/ | #191 | 암묵적 통신 운반 참고. 원 인용은 OSS 조사다 |

나머지 14개 PR은 논문을 인용하지 않았다. 각 PR의 추가 줄과 본문에서 `arXiv|doi|et al|논문|paper|Proceedings|IEEE|ICRA|IROS|https?://`를 찾았고 0건이었다.

보고서를 쓸 때 채워야 할 빈칸:
- #191이 `[출처 확인 필요]`로 표시한 RoCo와 V2X·협력 인지 문헌은 저장소 어디에도 인용이 없다. 넣으려면 서지를 새로 확인해야 한다.
- 다음 두 가지에는 문헌이나 측정 출처가 적혀 있지 않다.
  - #185의 통계 방법: 결정론적 percentile bootstrap, 짝 Cohen's d_z, matched-pairs rank-biserial, "PAR-style" 벌점.
  - #186의 기본 추론 속도 가정: prefill 약 5,000 tok/s, decode 약 50 tok/s.

## 2. 공개 코드·라이브러리

| ID | 이름·URL | 버전·커밋 | 라이선스 | 재사용·적응한 것 | 사용 PR |
|---|---|---|---|---|---|
| O1 | PythonRobotics https://github.com/AtsushiSakai/PythonRobotics | commit `b2020cd613d0`(2026-09-22). `Localization/particle_filter/particle_filter.py` sha256 `a649994b0c3e2423…`(다시 받아 일치 확인) | MIT(LICENSE 확인) | `harness/owncam_localizer.py`(#177)가 필터 구조를 옮겨 적응했다. 옮긴 부분은 잡음 입력 예측, Gaussian 중요도 가중, 가중 공분산, 유효 입자 수, 저분산 재표본이다. 머리말에 출처와 MIT 전문이 있다(`owncam_localizer.py:11-36`) | 수정: #201(모션 프로필 선택, 축별 지연, 정지 프레임 kidnap reset. 출처 문구는 유지). 그대로 사용: #203·#205(#201 파일을 바이트 그대로 복사), #206(#201 병합) |
| O2 | OpenCV https://github.com/opencv/opencv (PyPI `opencv-python-headless`) | `5.0.0.93`(`requirements-sim.txt`, `requirements-test.txt`) | Apache-2.0 | 태그: `cv2.aruco.ArucoDetector` + `DICT_APRILTAG_36h11` + `CORNER_REFINE_SUBPIX`, `cv2.solvePnPGeneric(SOLVEPNP_IPPE_SQUARE)`(`harness/wall_tags.py`, #177). 어안: `cv2.fisheye.undistortPoints`/`distortPoints`(`harness/markerless_box.py`, `sim/masterpi_camera_profile.py`, #193 v3). 인식: HSV `inRange`, `morphologyEx`, `findContours`, `connectedComponentsWithStats`, `convexHull`, `Sobel`, `GaussianBlur`(#193). 빔: HSV 마스크(#200 파일을 #203이 가져옴, #205 `owncam_pair_lift_v3.py`). 러너: `imdecode`/`imencode` | #193 #201 #203 #205 #206 |
| O3 | MuJoCo https://github.com/google-deepmind/mujoco | `3.12.0`(`requirements-sim.txt`. 실행 manifest에도 기록) | Apache-2.0 | 시뮬레이터다. 러너·평가 쪽에서만 `mj_contactForce`, `mj_name2id`/`mj_id2name`, `mj_forward`, `data.contact`, `data.eq_active`(weld 감시), `opt.noslip_iterations`를 쓴다. `noslip_iterations`는 `cargo_noslip_v1`이고 `sim/zone_cargo_contact.py`(#167)에 있다. 로봇 쪽 모듈에 `mujoco` import가 없음은 테스트로 검사한다 | #193 #201 #203 #205 #206 |
| O4 | NumPy https://github.com/numpy/numpy | `2.5.2` | BSD-3-Clause(PyPI 표기 `BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0`) | 배열 계산 전반 | #193 #201 #203 #205 #206 |
| O5 | Pillow https://github.com/python-pillow/Pillow | `12.3.0` | MIT-CMU | `harness/zone_map_schematic.py`가 `Image`, `ImageDraw`, `ImageFont.load_default`로 정적 지도 도식 PNG를 결정론적으로 그리고 `PIL.__version__`을 기록한다 | #187, #190(같은 파일을 import) |
| O6 | TensorBoard https://github.com/tensorflow/tensorboard | `2.21.0`(`requirements-observability.txt`) | Apache-2.0 | 쓰기: `EventFileWriter`, `event_pb2`/`summary_pb2`, hparams plugin(`api_pb2`, `metadata`, `plugin_data_pb2`), `make_tensor_proto`. #199는 main `scripts/tensorboard_tools/export.py`의 `Writer`를 import하고, #185는 같은 API를 직접 부른다. 읽기 검증: `EventAccumulator`(#195·#199), TensorBoard HTTP `/data/runs`·`/data/plugin/...`(#195) | #185 #195 #199. 스냅샷 생성만: #201 #203 #205 #206(`scripts/export_tensorboard.py` 경유) |
| O7 | pytest https://github.com/pytest-dev/pytest | `9.1.1`(`requirements-test.txt`) | MIT | 테스트. #185·#201은 표준 `unittest`를 쓴다 | 그 밖의 PR |
| O8 | LeRobot https://github.com/huggingface/lerobot | commit `89236ea0f4f8`(`requirements-reference-act.txt`) | Apache-2.0 | #191 04장이 OSS 조사가 권한 조합의 하나(LeRobot ACT)로 서술했다. 15개 PR의 코드는 쓰지 않는다 | #191(서술만) |

표준 라이브러리만 쓴 PR도 있다.
- #184: `copy`, `inspect`, `re`, `dataclasses`, `base64`, `json`
- #186: `heapq` 사건 큐, `hashlib`, `math`
- #188: `argparse`, `hashlib`, `json`. 덮어쓰기는 `open('xb')`로 막는다

### 검토했지만 쓰지 않은 공개 코드

main OSS 조사의 차단 목록이며 #191 04장이 인용했다.

| 이름 | URL | 라이선스(확인) | 쓰지 않은 이유(조사 문서) |
|---|---|---|---|
| ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 | GPL-3.0 | 연동하면 GPL이 전파된다. 2021 v1.0 이후 정체 |
| ViSP | https://github.com/lagadic/visp | GPL-2.0 | GPL, C++ 빌드. 제어 법칙만 참고 |
| Ultralytics YOLO | https://github.com/ultralytics/ultralytics · https://www.ultralytics.com/license | AGPL-3.0 | 학습한 모델까지 AGPL이 적용된다 |
| FoundationPose | https://github.com/NVlabs/FoundationPose | NVIDIA 자체 라이선스(GitHub SPDX 미지정) | CUDA 필수 |
| openpi(pi0) | https://github.com/Physical-Intelligence/openpi | Apache-2.0 | 미세조정에 GPU 24 GB 이상 필요 |
| CoELA 코드 | https://github.com/UMass-Embodied-AGI/CoELA | LICENSE 파일 없음 | 코드는 쓰지 않고 논문(P3)만 참고 |

## 3. 저장소 안에서 재사용한 모듈·PR

"도입 PR"은 main에 처음 들어온 PR이다. "미병합"은 기준 시점 `a67b6e3`에 main에 없던 파일이다. "쓴 PR"은 조사한 15개 PR 가운데 그 모듈을 쓴 PR이다.

| 모듈 | 도입 PR | 쓴 PR | 용도 |
|---|---|---|---|
| `harness/owncam_localizer.py`(O1 적응) | #177 | #201(수정), #203, #205, #206 | 자기 카메라 위치 추정 |
| `harness/wall_tags.py`(O2), `sim/zone_landmarks.py`, `scripts/record_owncam_localization.py`(`LoggingPort`) | #177 | #201 #203 #205 #206 | 태그 관측과 PnP, 태그 지도, 발행 명령 기록 |
| `harness/map_goto.py`(8-연결 격자 A*) | #148 | #201 #203 #205 #206 | 경로 계획. #203은 창고 칸 배정과 seed 필터에도 쓴다 |
| `harness/owncam_drive.py` / `harness/owncam_drive_v2.py` | #178 / #197 | #201 #203 #205 #206 | 주행·둘러보기. #203 `PairApproachDriver`의 부모 클래스 |
| `harness/wrist_zone_skill.py`, `wrist_zone_skill_v4`–`v9`, `harness/m1_contract.py` | #176, #181 | #201 #206(#193은 읽기만) | 손목 카메라 파지 스킬, M1 모드 계약 |
| `harness/m1_owncam_delivery.py`, `m1_owncam_contract.py`, `owncam_pose_source.py` | #201(미병합) | #206 | M1 배달 제어기와 자세 출처 |
| `harness/zone_own_perception*.py`, `zone_own_outcome*.py` | #193(미병합) | #206 | 경로 막힘·들고 있음 판단 |
| `harness/owncam_pair_beam*.py`, `owncam_pair_hold_v3.py`, `team_carry_status.py`(짝 상태 채널), `scripts/study_owncam_pair_beam.py` | #200 | #203 #205 | `imports.json`에 출처 커밋과 SHA-256을 붙여 바이트 그대로 가져왔다 |
| `harness/zone_scenario_feasibility.py`, `scripts/check_zone_scenarios.py` | #202 | #203 #205 | seed 실현 가능성 필터 |
| `harness/pair_carry_sync.py` | #43 | #203 #205 | 장벽 동기화 |
| `sim/zone_cargo_contact.py`(`cargo_noslip_v1`) | #167 | #193 #201 #203 #205 #206, #190(허용값 검사) | 접촉 프로필 |
| `sim/zone_cargo.py` | #164 | #190 #193 #203 #205 | 화물 카탈로그. #190은 고정 사본과 대조한다 |
| `scripts/cargo_formation_teacher.py`(`FormationTeacher`) | #164 | #193 | 교사 파지로 한 파지 유지 탐침(평가 전용) |
| `sim/zone_arena.py` | #149 | #190 #193 #201 #203 #206 | 지도 변형·배치 |
| `scripts/zone_teacher.py`의 `ArmSequence`(발행 서보 목표 보간)·`FOLDED` | #149 | #193 #203 #205 | 팔 명령 시퀀스 도우미 |
| `sim/dispatch_contact_profile.py` | #65 | #190 | 접촉 프로필 허용값 |
| `harness/zone_color_boxes.py`(`detect_own`) | #163 | #193 #201 | 4색 상자 검출 |
| `scripts/eval_zone_color_detection.py` | #163 | #193 | 렌더·분할 보조(평가 쪽) |
| `harness/markerless_box.py` | main 초기 모듈 | #193 #206 | 어안 투영 K·D, 바닥 직육면체 적합 |
| `harness/visual_floor.py` | main 초기 모듈 | #193(아이디어만, import 없음) | 바닥 측정 |
| `harness/visual_arm.py` | main 초기 모듈 | #193, #203(가져온 파일 경유) | FK·도구 자세 |
| `sim/masterpi_camera_profile.py` | main 초기 모듈 | #193 #206 | 고정 카메라 보정, `raw_fisheye_remap` |
| `sim/camera_robot_port.py`, `sim/multi_masterpi_production.py` | main 초기 모듈 | #193 #201 #203 #205 #206 | 장면·로봇 포트 |
| `harness/zone_rgb_outcome.py` | #170 | #193 | 고정 주기·안정성 구조만 참고했다(코드 복사 없음) |
| `harness/zone_dialogue_metrics.py` | #172 | #184 #185(읽기), #188(수정: 규칙 v1 동결, v2 신설) | 한국어 비율, 영어 단어, ID 검사, 대화 행위 규칙 |
| `harness/three_robot_plan.py`(`parse`) | #62 | #184 | 코드 울타리 JSON 파싱 |
| `harness/zone_goal_v2.py` | #165 | #187(#190은 #187 경유) | 화물 편성 대조 |
| `harness/zone_study_contract.py`, `zone_study_inputs.py`, `zone_map_schematic.py` | #187(작업 중 병합) | #190(병합해 사용), #184(로컬 어댑터), #185(잠정 schema), #206(#194의 값을 고정 복사) | 연구 계약, 주문서, 지도 도식 |
| `harness/zone_sim_cost.py`, `zone_event_scheduler.py` | #186(작업 중 병합) | #206(`TRIGGERS` 고정 복사) | SIM 비용, 사건 큐 |
| `scripts/tensorboard_tools/export.py`, `scripts/export_tensorboard.py` | #87 | #195 #199(`Writer` import), #201 #203 #205 #206(스냅샷) | TensorBoard 변환 |

변환에 쓴 원본 결과:
- #195: `experiments/2026-09-25-zone-owncam-loop`(#178)
- #199: `experiments/2026-09-26-zone-own-perception`(#193), [`experiments/2026-09-26-noslip-side-effects`](../experiments/2026-09-26-noslip-side-effects/README.md)(#189)

## 4. 문서·웹 페이지

### 저장소 문서

| 문서 | 도입 PR | 쓴 PR |
|---|---|---|
| [Codex 통합 설계](design/2026-09-25-zone-dialogue-study-design-codex.md) | #180 | #184(경로·PR 인용), #186(5절 비용식·실행 의미), #187(이름만), #190(본문에서 6절), #192(9절 작업 패키지), #191. #185는 인용 없이 조건 이름만 같다. `r1` 고정 지휘자, 별도 commander, 교사 실행기 권고는 사용자 결정으로 대체됐다(#184·#187에 명시) |
| [Codex PR 검토](design/2026-09-25-zone-pr-review-codex.md) | #175 | #188(PR #172 절의 후속), #191 |
| [자기 카메라 반대 검토(Codex)](design/2026-09-26-owncam-adversarial-review-codex.md) | #196 | #201. 검토 항목 번호(#1–#4, #7, #9)만 적고 경로는 없어 대응은 추정이다 |
| [OSS 조사](design/2026-09-25-own-camera-oss-survey-claude.md), [자기 카메라 역량 목록](design/2026-09-25-own-camera-inventory-claude.md) | #175 | #191(플랫폼 사실, 차단 목록, 선행연구) |
| [결정 로그](decision_log.md) 2026-09-25 절들, 2026-09-26 절(`cargo_noslip_v1` 채택, 짝 상태 채널 전 조건) | – , #204 | #191 #192 |
| [CoELA 통신 연구](coela_communication_study.md), [레퍼런스 차이표](reference_alignment.md) | main 초기, #58 | #191 |
| [실행 버전 관리](execution_versioning.md) | #113 | #187 #190 |
| [TensorBoard 안내](tensorboard.md) | #87 | #185(절차), #199(절 추가) |
| [자기 카메라 위치 추정](zone_owncam_localization.md) | #177 | #191 #192 |
| [한국어 대화 파일럿](../experiments/2026-09-25-zone-dialogue-ko-pilot/README.md) | #172 | #184 #185 #188 |

### 웹 페이지

| ID | 페이지 | 쓴 곳 |
|---|---|---|
| W1 | MuJoCo Modeling, "Preventing slip": https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip (앵커 확인) | `sim/zone_cargo_contact.py`(#167)는 `noslip_iterations`를 "MuJoCo's documented remedy"라고만 쓰고 URL을 적지 않았다. 이 URL은 main의 `experiments/2026-09-14-pair-grasp-endurance/protocol.md`와 `experiments/2026-09-15-pair-grasp-retention/protocol.md`가 인용한다. `cargo_noslip_v1`을 쓰는 #193 #201 #203 #205 #206의 간접 근거다 |
| W2 | TensorBoard 안내: https://www.tensorflow.org/tensorboard/get_started · https://www.tensorflow.org/tensorboard/hyperparameter_tuning_with_hparams | 기존 `docs/tensorboard.md`가 인용한다. 이 문서를 따르거나 고친 PR은 #185와 #199다 |
| W3 | Hiwonder MasterPi 제품 페이지: https://www.hiwonder.com/products/masterpi | #191 03장의 플랫폼 사실(OSS 조사 경유) |
| W4 | PythonRobotics 원본 파일(고정 커밋): https://github.com/AtsushiSakai/PythonRobotics/blob/b2020cd613d0709e9c0f38a7579f1a681cf7a227/Localization/particle_filter/particle_filter.py | O1 |

## 5. PR별 색인

| PR | head | 논문 | 공개 코드 | 저장소 재사용 | 문서 |
|---|---|---|---|---|---|
| #184 | `e8c80eb` | – | 표준 라이브러리, O7 | `zone_dialogue_metrics`(#172), `three_robot_plan`(#62) | Codex 설계, 파일럿 README |
| #185 | `a68ce05` | –(방법 출처 미기재) | O6, `unittest` | `zone_dialogue_metrics`(#172), #184 조건 이름, #187 잠정 schema | TensorBoard 안내, 파일럿 README |
| #186 | `6be1395` | –(속도 가정 출처 미기재) | 표준 라이브러리(`heapq`), O7 | 없음(러너 코드 줄만 읽음) | Codex 설계 5절 |
| #187 | `8587bb9` | – | O5, O7 | `zone_goal_v2`(#165), `maps/zones`(#149·#173·#177) | Codex 설계(이름), 실행 버전 관리 |
| #188 | `d6557d7` | – | 표준 라이브러리, O7 | #172 지표·파일럿 스크립트 수정 | Codex PR 검토 |
| #190 | `3186100` | – | O5(간접), O7 | #187(`eab3f67c`), `zone_cargo`(#164), `zone_arena`(#149), 접촉 프로필(#65·#167), `tags_v1` 지도(#177) | Codex 설계 6절, 실행 버전 관리 |
| #191 | `6b582ee` | P1–P6 | O1–O4·O8 서술, 차단 목록 | 요약 대상 PR 다수 | OSS 조사, 결정 로그, CoELA 문서, W3 |
| #192 | `0b8cb27` | – | MuJoCo `noslip_iterations` 서술 | #176–#191 요약 | 결정 로그, Codex 설계 9절 |
| #193 | `4249ace` | – | O2 O3 O4 O7 | #163 #164 #167 #170(구조만) #149, 초기 모듈 | W1(간접) |
| #195 | `425b560` | – | O6 | #87 변환기, #178 결과 | – |
| #199 | `e19c88a` | – | O6 | #87 `Writer`, #193·#189 결과 | TensorBoard 안내(절 추가), W2 |
| #201 | `22c8484`(→`1b836a1`) | – | O1(수정) O2 O3 O4, `unittest` | #177 #178 #197 #181 #176 #163 #148 #167 | 반대 검토(번호만), W1(간접) |
| #203 | `c9a3662`(→`78c2979`) | – | O1(간접) O2 O3 O4 O7 | #200 #201 #202 바이트 복사, #43 #148 #149 #164 #167 | W1(간접) |
| #205 | `6990a6e` | – | O2 O4 O7(O3는 #203에서 이어받음) | #203, #200 파일 | W1(간접) |
| #206 | `c8a2355` | – | O2 O3 O4 O7(O1은 #201에서 이어받음) | #201 #193 #181 #178·#197 #148 #167, #194·#186 값 고정 복사 | W1(간접) |

#201·#203은 추출 뒤 head가 바뀌었다(괄호 안). 새 커밋은 origin/main 병합과 `scripts/run_ci_tests.py` 등 CI 목록 수정뿐이다. #203의 `imports.json` 32개 파일 해시와, #201·#203의 `cargo_noslip_v1` 표기 줄·PythonRobotics 출처 문구는 새 head에서 다시 확인했고 그대로다.

## 6. `cargo_noslip_v1` 승인 표기

`cargo_noslip_v1`은 2026-09-26 사용자 승인으로 연구 전체의 접촉 프로필이 됐다(#204, [결정 로그](decision_log.md) 2026-09-26 절). 일부 PR의 본문·README·러너 문자열에는 승인 전 표기가 남아 있다("pending user approval", "사용자 결정을 기다린다", "보류 중인 사용자 결정"). 그래서 해당 PR 본문에 "cargo_noslip_v1은 2026-09-26 사용자 승인됨(#204)" 한 줄을 더했다. 커밋된 기록, 동결 소스, 사전 등록은 규칙대로 고치지 않았다.
- 본문에 추가: #201, #203, #205, #206, #192
- 나중에: #207(본문과 README에 "사용자 승인 대기")

## 7. 확인 방법과 한계

- **추출:** 각 PR에서 다음을 확인했다.
  - `git diff <merge-base>..<head>`와 본문(`gh pr view`)
  - 추가 줄의 import
  - `arXiv|doi|et al|논문|paper|https?://|adapted|upstream|license|재사용` 검색
- **상속 구분:** 다른 브랜치를 병합한 PR은 고유 변경과 상속을 나눴다(#206 ← #201·#193, #205 ← #203, #190 ← #187).
- **PR 본문:** 15개 PR의 본문 끝에 같은 근거로 `## 참고 자료` 절을 붙였다. 기존 본문은 그대로 두고 덧붙이기만 했다. #184–#187은 병합된 뒤에 본문만 덧붙였다.
- **버전:** 대부분 PR의 커밋 기록에는 라이브러리 버전이 없다. 위 버전은 requirements의 고정값이다. #201·#206의 로컬 raw manifest(`outputs/`, Git 밖)에는 python 3.12.13, mujoco 3.12.0, opencv 5.0.0, numpy 2.5.2가 기록돼 있다.
- **범위 밖:** main에 이미 병합된 문서의 다른 인용은 여기 모으지 않았다(예: `docs/reference_alignment.md`의 ACT·Diffusion Policy, OSS 조사의 다른 후보). 목록에 없다고 해서 쓰지 않았다는 뜻은 아니다.
