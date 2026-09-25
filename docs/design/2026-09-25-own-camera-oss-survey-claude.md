# 자기 RGB·정적 지도만으로 동작하는 로봇: 오픈소스 재사용 조사

- 작성: Claude subagent (읽기 전용 웹 조사). 후보 라이브러리는 설치하거나 시험하지 않았으며, 적합도와 공수는 추정치다.
- 날짜: 2026-09-25
- 읽은 코드: 기본 체크아웃 `/Users/changmin/projects/ugrp`의 작업 트리, HEAD `0add360d00c5f1f274b10a6c1ec578ed93280dc9`(main, 깨끗한 상태). 로컬 `origin/main` ref는 `6d80e3e401f6936bc90cd3ccfc00249018a81748`이고 fetch하지 않았다. HEAD와 다르며, 그 사이 변경분은 읽지 않았다.
- 범위: 저장소 파일, 설치된 가상환경, AGENTS.md를 직접 읽었고 오픈소스 후보는 웹 검색 결과로만 조사했다. 저장소 수정·커밋·시뮬레이션 실행은 하지 않았다.

## 가장 중요한 결론
- **지도 주행이 자기 카메라로 자기 위치를 추정하지 못한다.** 현재 A* 계획기 `harness/map_goto.py`는 현재 위치를 호출자에게서 받는다. 그 위치를 공급하는 `heading_map_navigation.py`와 `run_known_map_navigation`은 공용 TOP 카메라로 위치를 잡는다. 이제 TOP은 금지 입력이므로, 목표 조건에서는 자기 카메라로 위치를 추정하는 부품이 빠진 셈이다.
- **짐을 들면 실물 카메라가 가려진다.** 실물 MasterPi의 카메라는 팔 끝(집게)에 달린 어안 카메라 1대뿐이다. 상자를 들면 이 카메라가 대부분 가려진다(`docs/camera_team_llm.md`). 차체 전방 카메라 `nav_cam`은 시뮬레이터에만 추가한 장치이고 실물 검증이 없다.
- **추천 조합:** 벽·문에 붙인 AprilTag + 자기 명령을 운동 모델로 쓰는 파티클 필터 + 기존 A*·RGB 스킬 + LeRobot ACT. 새로 필요한 의존성은 거의 없다.
- **라이선스·플랫폼 차단:** YOLO(AGPL), ORB-SLAM3·ViSP(GPL), FoundationPose(CUDA 필요), pi0(GPU 메모리 24GB 이상)는 이 환경에 맞지 않는다.

## 1. 저장소에서 확인한 플랫폼·스택
- **로봇:** Hiwonder MasterPi다(`sim/masterpi_geometry.py`). 차체는 메카넘 바퀴 4개(지름 65 mm, 축간 0.12 m)다. 문의한 SO-100/SO-101이나 LeKiwi가 아니다. 저장소에는 LeKiwi·Feetech 관련 언급이 없다.
- **팔:** 5자유도 PWM 서보(500–2500 µs 범위)이며 1번이 집게, 3·4·5번이 관절, 6번이 받침 회전이다. 탐색 자세 `SEARCH_POSE={1:2000,3:740,4:2320,5:1320,6:1500}`(`sim/masterpi_production_v2.py:18`)가 질문의 서보값이다. 실물은 명령 PWM만 알고 관절 측정값은 없다(`harness/real_geometry.py`, `real_odometry.py`).
- **자기 카메라(실물):**
  - 팔 끝에 달린 icspring 어안 카메라, 640×480.
  - 측정 내부 파라미터 fx≈619.5, fy≈622.2, 어안 왜곡 계수 4개.
  - 설치 위치는 "잠정·미검증" 상태다(`sim/masterpi_camera_profile.py`).
  - 시뮬레이터는 핀홀 렌더를 실물 어안 기하로 다시 매핑한다(`raw_fisheye_remap`). 그래서 `cv2.fisheye` 모델을 시뮬레이터와 실물에 그대로 쓸 수 있다.
- **`nav_cam`(시뮬레이터 전용):** 640×480 핀홀, 세로 FOV 70°, 차체 기준 (0.05, 0, 0.32) m, 25° 아래로 기울어져 있고 `camera_team` 배치에서만 켜진다(`sim/navigation_camera_profile.py`). 문서에 "이 추가 장치는 실물에서 검증되지 않았다"고 적혀 있다.
- **시뮬레이터:** MuJoCo 3.12.0, Python 3.12.13, numpy 2.5.2, opencv-python-headless 5.0.0이다. `.venv-sim-worker-mac`에서 버전을 직접 확인했다. 호스트는 Apple M3, 메모리 16 GB, CUDA 없음이다. Ubuntu 24.04 경로는 `docs/ubuntu_quickstart.md`에 있다.
- **이미 쓰고 있는 외부 코드:**
  - LeRobot ACT를 `requirements-reference-act.txt`에서 커밋 `89236ea`로 고정해 두었다(torch 2.11). 다만 확인한 4개 가상환경 어디에도 torch·lerobot이 설치되어 있지 않다. 별도 환경이 필요하다.
  - Depth Anything V2 metric 연결 모듈이 이미 있다(`harness/depth_anything_metric.py`, Colab GPU용).
  - 현재 OpenCV 5.0에 ArUco와 AprilTag 36h11 사전이 들어 있어, 추가 설치 없이 검출할 수 있다(직접 확인).
- **재사용할 수 있는 저장소 자산:**
  - `map_goto.py`: 정적 지도 A*, 자기 RGB로 추정한 장애물도 반영.
  - `visual_floor.py`: 어안 영상에서 바닥·장애물 측정.
  - `camera_pixel_grasp.py`, `visual_box_skill.py`, `visual_placement.py`: 영상 기반 파지·배치.
  - `camera_landmark_tracker.py`: 영상 속 표식 추적.
  - `pair_transport_vision.py`와 2026-09-14 "robust" 공동 운반: 운반·내려놓기 5/5.
  - `maps/navigation/narrow-door.json`: 폭 56 cm 문이 있는 지도.
- **과거 결정:** 화물의 ArUco 식별판은 2026-09-09 사용자 요청으로 제거했다(`docs/cargo_marker_removal_20260909.md`). 따라서 화물에는 표식을 다시 붙이지 않는 것이 전제다.

## 2. 기능별 오픈소스 후보
열 설명: "Mac" = M3·CPU·MPS에서 실행 가능한지, "MJ" = MuJoCo 연결 공수, "입력" = 필요한 입력과 허용 여부.

| 기능 | 후보 | 라이선스 / 유지 상태 | Mac | MJ | 입력 | 판단 |
|---|---|---|---|---|---|---|
| 위치 추정 | **OpenCV ArUco/AprilTag + `solvePnP`/fisheye** | Apache-2.0 / 활발 | ◎ 설치 완료 | 낮음 | 자기 RGB, 지도에 적은 태그 위치 | **1순위** |
| 위치 추정 | pupil-apriltags (apriltag3 바인딩) | MIT(원본 BSD-2) / 유지 | ◎ arm64 wheel | 낮음 | 자기 RGB | OpenCV 검출이 부족할 때 대안 |
| 위치 추정 | PythonRobotics 파티클 필터·EKF | MIT / 활발 | ◎ 순수 Python | 낮음 | 태그 관측 + **자기 발행 명령**(운동 모델로만) | 약 150줄을 옮겨 쓰기 |
| 위치 추정 | ORB-SLAM3 (단안·어안 지원) | **GPLv3** / 2021 v1.0 이후 정체 | △ Mac 포크 빌드 | 높음 | 자기 RGB; 단안은 축척이 불확실 | 보류. 연동하면 GPL이 전파됨 |
| 위치 추정 | RTAB-Map | BSD-3 / 활발 | △ | 높음 | 기본은 RGB-D·스테레오. 단안만으로는 약함(이슈 다수) | 부적합 |
| 위치 추정 | Duckietown 방식(태그 + 지도) | 공개 교육 플랫폼 | 참고용 | – | 단안 + 고정 태그 | 설계 근거로만 참고 |
| 계획·추종 | 기존 `map_goto` A* + PythonRobotics pure pursuit | MIT | ◎ | 낮음 | 추정 위치 + 지도 | **재사용** |
| 계획·추종 | Nav2 (AMCL·MPPI·costmap) | Apache/LGPL 혼재 / 활발 | ✗ ROS 2가 Mac에서 사실상 비지원 | 매우 높음 | 라이다 기반 AMCL | 개념만 참고(장애물 여유 반경, 복구 행동, 전방향 MPPI) |
| 장애물·빈 공간 | 기존 `visual_floor.py` + 지도와 영상 차이 비교 | 자체 코드 | ◎ | 없음 | 자기 RGB | **1순위** |
| 장애물·빈 공간 | Depth Anything V2 **Small** | Apache-2.0 (Base/Large/Giant는 **CC-BY-NC**) | ○ CPU·MPS 저빈도 | 중간 | 자기 RGB에서 상대 깊이 추정 | 2순위. 저장소가 쓰는 metric-hypersim 체크포인트의 라이선스는 별도 확인 필요 |
| 물체 검출 | **RF-DETR Nano/Small** | Apache-2.0 / 활발(ICLR 2026) | ○ MPS·CPU 학습 공식 지원 | 중간 | 자기 RGB | 교사 정답으로 자동 라벨한 시뮬 영상으로 미세조정 |
| 물체 검출 | Grounding DINO + SAM 2 | Apache-2.0 / DINO는 정체, SAM 2 활발 | △ 느림(CPU가 MPS보다 빠른 사례) | 중간 | 자기 RGB + 문장 | 오프라인 라벨 생성용으로만 |
| 물체 검출 | Ultralytics YOLOv8 이상 | **AGPL-3.0** | ○ | 낮음 | 자기 RGB | **회피**(학습된 모델까지 AGPL 적용) |
| 물체 자세 | FoundationPose | NVIDIA 라이선스 / 유지 | ✗ **CUDA 필수** | 높음 | 기본은 RGB-D + CAD | **차단** |
| 파지 | 기존 `camera_pixel_grasp` (IBVS류) | 자체 코드 | ◎ | 없음 | 자기 RGB + 명령 이력 | **재사용** |
| 파지 | ViSP IBVS | **GPLv2/3** / 활발 | △ C++ 빌드 | 높음 | 자기 RGB | 제어 법칙만 참고(약 50줄) |
| 파지 | Contact-GraspNet 계열 | 다양 | ✗ | 높음 | 깊이 카메라 필요(실물에 없음) | 부적합 |
| 모방 학습 | **LeRobot ACT / Diffusion Policy** | Apache-2.0 / 매우 활발 | ○ `policy.device=mps` 공식 | 낮음(이미 연동) | 자기 RGB + 명령 이력 | **재사용** |
| 모방 학습 | SmolVLA (약 450M) | Apache-2.0 | △ MPS 미세조정 가능하나 느림 | 중간 | 자기 RGB + 한국어 지시 | 한국어 대화 연구와 연결되는 후보 |
| 모방 학습 | pi0 / pi0.5 (openpi) | Apache-2.0 | ✗ 미세조정 GPU 24GB 이상(LoRA 22.5GB) | – | – | **차단**(M3 16GB, Kaggle/Colab T4 16GB 모두 부족) |
| 모방 학습 | LeKiwi (LeRobot) | Apache-2.0 | – | – | 하드웨어가 다름(SO-101 팔 + 옴니 바퀴 3개) | 드라이버는 쓸 수 없고, 데이터 형식(차체 속도 + 팔 명령 + 카메라)만 참고 |
| 협동 운반 | CoLF (2026, 사족로봇 2대, 자기 영상 + 언어 기반 리더–팔로워, 통신 없음) | 논문 | – | – | 자기 RGB | 설계 참고. 코드 공개 여부 미확인 |
| 협동 운반 | 분산 강화학습 운반(arXiv 2007.09243), 암묵적 통신 운반(PMC7806111) | 논문 | – | – | – | 참고용 |

협동 운반은 바로 가져다 쓸 라이브러리가 없다. 가장 검증된 출발점은 저장소 자체의 "robust" 공동 운반(5/5)과 `zone_team_formation`이다.

## 3. 환경 추가물과 AGENTS.md 해석
- **벽·문에 붙인 AprilTag (추천):** AGENTS.md의 지도 경로 예외는 미리 만든 정적 지도(고정 장애물·지형 형상, 목적 구역)와 고정 카메라 보정을 허용한다. 태그는 **고정 벽의 외형 특징**이므로 "버전·해시로 연결한 정적 지도 특징"으로 해석할 수 있다. 필요한 작업은 다음과 같다.
  - 지도 JSON에 `landmarks:[{id, family:"tag36h11", size_m, pose}]`를 추가하고, 같은 자료로 MuJoCo 텍스처를 생성한다.
  - 지도·장면 해시를 실행 번들에 기록한다.
  - 현장 일치(태그 위치 대 실제 배치)를 따로 검증한다.
  - 실물에서도 인쇄해 붙이면 되므로 실물과의 괴리가 작다.
- **태그를 붙이면 안 되는 곳:** 로봇과 화물이다. AGENTS가 로봇 외관·물체 유지를 요구하고, 2026-09-09 제거 결정도 있다.
- **바닥 무늬·구역 도색:** 이미 구역 도색을 "nav_cam이 볼 수 있는 장면 근거"로 쓰고 있다(`multi_masterpi_production.py:277`). 통로마다 다른 바닥 무늬도 같은 방식으로 정적 지도 특징에 넣을 수 있다.
- **사용자 확인이 필요한 결정 두 가지:**
  - (a) 태그는 AGENTS.md에 명시된 지도 항목이 아니다. 채택 여부를 decision_log에 남기는 것이 안전하다.
  - (b) 운반 중 시야를 어떻게 확보할지다. 선택지는 실물 그대로 팔 끝 카메라만 쓰고 팔 자세로 시야를 확보하는 것, 또는 시뮬레이터 전용 `nav_cam`을 "로봇의 실제 카메라"로 인정하는 것이다. `nav_cam`을 인정하면 실물 카메라 배치를 유지하라는 원칙과 부딪힌다.

## 4. 추천 조합과 첫 목표
**조합:**
- 위치 추정: OpenCV 태그 검출 + `cv2.fisheye` 왜곡 제거 + PnP로 태그 관측을 만들고, PythonRobotics 파티클 필터(MIT)로 합친다. 운동 모델은 자기 발행 명령만 쓴다. 필터 출력은 위치 추정일 뿐이고 이동 성공 판정에는 쓰지 않는다.
- 계획: 추정 위치를 기존 `map_goto` A*에 넣고, 메카넘용 pure pursuit로 따라간다.
- 장애물: `visual_floor`로 검사하고, 지도와 영상 차이를 `map_goto`의 RGB 장애물 입력으로 넣는다.
- 물체 검출: 처음에는 기존 색·markerless 검출을 쓰고, 부족하면 RF-DETR Nano를 도입한다. 라벨은 교사 정답으로 자동 생성하되 학습 데이터에만 쓰고 출처를 표시한다.
- 파지·배치: 기존 RGB 스킬을 쓰고, 이후 LeRobot ACT로 대체할 수 있는지 비교한다.
- 공동 운반: 기존 robust 공동 운반 경로에 같은 위치 추정기를 로봇별로 붙인다.

**첫 목표:** `narrow-door` 지도에서 로봇 1대가 상자 1개를 자기 카메라만으로 옮긴다. 순서는 파지 → 태그로 위치 추정 → 문 통과 → 목표 구역에 배치 → 자기 영상으로 배치 확인이다. TOP 카메라와 weld는 OFF다. 정답 좌표는 별도 평가 출력에만 기록하고, 위치 추정 오차도 따로 보고한다.

| 부품 | 공수(추정) | 위험 |
|---|---|---|
| 지도 JSON 태그 형식 + MuJoCo 텍스처 생성 + 해시 연결 | 1–2일 | 낮음 |
| 태그 검출·어안 PnP·파티클 필터(명령 운동 모델) + 오프라인 오차 감사 | 3–5일 | 중간. 팔 끝 카메라 높이·시야 때문에 태그가 잘 안 보일 수 있음 |
| 위치 추정 → `map_goto` → 메카넘 경로 추종 연결, 복구·정지 규칙 | 2–3일 | 중간 |
| 운반 중 가림 대응(팔 자세로 시야 확보, 또는 `nav_cam` 결정) | 2–4일 | **높음. 사용자 결정 필요** |
| 파지·배치 기존 스킬 연결 | 2–3일 | 중간 |
| 실행 번들·워크플로 등록·실험 기록·TensorBoard | 1–2일 | 낮음 |
| **합계** | **약 2–3주** | – |

이후 단계의 공수(추정):
- 미지 장애물 대응(DAv2 Small 추가 비교): 약 1주
- RF-DETR 도입: 약 1주
- ACT 학습·비교: 1–2주
- 2대 긴 빔 운반 이식: 약 2주
- 3대 삼각 프레임 운반: 2주 이상. 참고할 선례가 없다.

**주의할 차단 요인:**
- AGPL: YOLO
- GPL: ORB-SLAM3, ViSP. 연동하면 소스 공개 의무가 전파된다.
- CUDA 필수: FoundationPose
- GPU 메모리 부족: pi0/pi0.5
- 비상업 라이선스: Depth Anything V2 Base/Large
- ROS 2가 Mac에서 사실상 비지원: Nav2
- 추가 확인 필요: 저장소가 쓰는 Depth Anything metric 체크포인트의 라이선스, LeRobot용 환경(torch)이 어디 설치되어 있는지.

## 출처
- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3 · Mac 포크 https://github.com/zhoujoey/ORB-SLAM3-Mac
- RTAB-Map 단안 이슈: https://github.com/introlab/rtabmap/issues/1395 · https://github.com/introlab/rtabmap/discussions/1284 · https://arxiv.org/pdf/2403.06341
- pupil-apriltags: https://github.com/pupil-labs/apriltags · https://pypi.org/project/pupil-apriltags
- Duckietown: https://people.csail.mit.edu/jalonsom/docs/17-paull-duckietown-icra.pdf · https://duckietown.com/localization-with-sensor-fusion-in-duckietown/
- PythonRobotics: https://github.com/atsushisakai/pythonrobotics · https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/particle_filter/particle_filter.py
- Nav2: https://github.com/ros-navigation/navigation2 · https://github.com/ros-planning/navigation2/issues/1035 · https://navigation.ros.org/configuration/packages/configuring-mppic.html
- Depth Anything V2 라이선스: https://github.com/DepthAnything/Depth-Anything-V2/issues/162 · https://github.com/DepthAnything/Depth-Anything-V2/issues/320
- RF-DETR: https://github.com/roboflow/rf-detr · https://rfdetr.roboflow.com/latest/learn/train/
- YOLO 라이선스: https://www.ultralytics.com/license · https://www.libreyolo.com/articles/best-ultralytics-alternatives
- Grounding DINO / SAM 2: https://github.com/idea-research/groundingdino · https://github.com/idea-research/grounded-sam-2 · https://stephencowchau.medium.com/using-groundingdino-object-detection-on-apple-mac-mini-m1-1db9fa5a07e7
- FoundationPose: https://github.com/NVlabs/FoundationPose · https://github.com/NVlabs/FoundationPose/blob/main/LICENSE · https://huggingface.co/nvidia/foundationpose
- ViSP: https://github.com/lagadic/visp · https://visp-doc.inria.fr/doxygen/visp-daily/tutorial_python.html
- LeRobot·LeKiwi·SmolVLA: https://arxiv.org/html/2602.22818v1 · https://wiki.seeedstudio.com/lerobot_lekiwi/ · https://huggingface.co/blog/zuoxingdong/mobile-manipulation-lekiwi-pincopen · https://arxiv.org/pdf/2506.01844
- pi0/openpi: https://huggingface.co/docs/lerobot/pi0 · https://github.com/Physical-Intelligence/openpi · https://github.com/huggingface/lerobot/issues/2216
- 협동 운반: https://arxiv.org/abs/2602.07776 · https://arxiv.org/pdf/2007.09243 · https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7806111/ · https://arxiv.org/pdf/2305.01614
- MasterPi: https://www.hiwonder.com/products/masterpi · https://docs.hiwonder.com/projects/MasterPi/en/latest/docs/1.getting_ready.html
