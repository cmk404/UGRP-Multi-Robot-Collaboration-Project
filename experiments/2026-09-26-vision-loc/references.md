### 논문 (직접 확인한 것만)

- A. Howard, M. Sandler, G. Chu, L.-C. Chen, B. Chen, M. Tan, W. Wang, Y. Zhu, R. Pang, V. Vasudevan, Q. V. Le, H. Adam, "Searching for MobileNetV3", ICCV 2019. https://arxiv.org/abs/1905.02244 — LR-ASPP 분할 머리와 백본(채택).
- L.-C. Chen, G. Papandreou, F. Schroff, H. Adam, "Rethinking Atrous Convolution for Semantic Image Segmentation" (DeepLabv3), 2017. https://arxiv.org/abs/1706.05587 — 비교 후보(기각).
- F. Boniardi, A. Valada, R. Mohan, T. Caselitz, W. Burgard, "Robot Localization in Floor Plans Using a Room Layout Edge Extraction Network", IROS 2019. https://arxiv.org/abs/1903.01804 — 학습한 배치 가장자리 + 평면도 PF(이번 설계의 가장 가까운 선행 연구).
- C. Chen, R. Wang, C. Vogel, M. Pollefeys, "F³Loc: Fusion and Filtering for Floorplan Localization", CVPR 2024. https://arxiv.org/abs/2403.03370 — 평면도 깊이 + 필터(기각: 재학습 필요).
- T. Sattler, Q. Zhou, M. Pollefeys, L. Leal-Taixé, "Understanding the Limitations of CNN-based Absolute Camera Pose Regression", CVPR 2019. https://arxiv.org/abs/1903.07504 — 자세 회귀 기각 근거.
- A. Kendall, M. Grimes, R. Cipolla, "PoseNet: A Convolutional Network for Real-Time 6-DOF Camera Relocalization", ICCV 2015. https://arxiv.org/abs/1505.07427 — 자세 회귀 후보(기각).
- A. Kirillov 외, "Segment Anything", ICCV 2023. https://arxiv.org/abs/2304.02643 — 범용 분할 후보(기각: 비용).
- R. Grompone von Gioi, J. Jakubowicz, J.-M. Morel, G. Randall, "LSD: a Line Segment Detector", IPOL 2012. https://www.ipol.im/pub/art/2012/gjmr-lsd/ — 선분 후보(기각).
- J. Tobin, R. Fong, A. Ray, J. Schneider, W. Zaremba, P. Abbeel, "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World", IROS 2017. https://arxiv.org/abs/1703.06907 — 실물 이전 한계의 근거.
- PR #210 설계 문서의 조사(visual sonar, 평면도 PF, RoboCup 가장자리 MCL, VPR 등)를 그대로 이어받는다: [docs/design/2026-09-26-markerless-localization-and-memory.md](../../docs/design/2026-09-26-markerless-localization-and-memory.md) 2절.

### OSS·라이브러리

| 이름 | 버전 | 라이선스 | 재사용한 것 |
|---|---|---|---|
| torchvision ([github.com/pytorch/vision](https://github.com/pytorch/vision)) | 0.26.0 | BSD-3-Clause | `lraspp_mobilenet_v3_large` 모델 정의, `MobileNet_V3_Large_Weights.IMAGENET1K_V1` 백본 가중치(`mobilenet_v3_large-8738ca79.pth`, sha256 `8738ca79…a04`) |
| PyTorch | 2.11.0 (MPS) | BSD-3-Clause | 학습·추론, DataLoader |
| OpenCV (`opencv-python-headless`) | 4.13.0 (학습 환경) / 5.0.0 (sim 환경) | Apache-2.0 | 어안 복원 `fisheye.initUndistortRectifyMap`, JPEG/PNG 입출력 |
| MuJoCo | 3.12.0 | Apache-2.0 | 교사 렌더의 분할 영상(`Renderer.enable_segmentation_rendering`) |
| PythonRobotics ([github.com/AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)) | b2020cd (M1 PF가 인용) | MIT | M1 PF 구조(예측·가중치·저분산 재표본화) — M1 localizer를 통해 간접 상속 |
| TensorBoard | 2.21.0 (sim 환경) | Apache-2.0 | 스냅샷 보기(저장소 `scripts/tensorboard_tools/export.py` Writer로 이벤트 파일 작성) |
| Ultralytics YOLO | — | AGPL-3.0 ([라이선스](https://www.ultralytics.com/license)) | 사용 안 함(라이선스·출력 형식 이유로 기각) |

### 내부 모듈·PR

- PR #210 `experiments/2026-09-26-markerless-probe/markerless_probe.py`: `ColumnModel`(열 궤적), `MapGeometry`(벽 발자국·슬랩 광선 추적), `undistort`, `load_m1_localizer`/`load_m1_calibration`(해시 확인), 손 검출기 `detect_boundaries`/`boundary_loglik`(기준선).
- M1 PF `harness/owncam_localizer.py` @ `22c84842`(PR #201, sha256 `0304d7c4…`), M1 보정 `experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json`.
- M1 실행기·제어기(교사 렌더): `scripts/run_m1_owncam.py`, `harness/m1_owncam_delivery.py`, `harness/wrist_zone_skill_v9.py`(PR #181 고정본) @ `kiro/zone-map-v3` `7cedb049`.
- 환경 v3 벽 프로필 `sim/zone_arena.py` `WALL_PROFILES['walls_v3']`/`apply_wall_profile`, `sim/zone_scene.py` `ZoneScene`(PR #208, `kiro/zone-map-v3`).
- 운동 모델 재적합 변형(기각): `scripts/eval_owncam_localization.py` `fit_motion`.
- TensorBoard: `scripts/tensorboard_tools/export.py` `Writer`; 빌더 구조는 PR #210 `build_tensorboard.py`.
- 태그 참조: PR #210 `results/metrics_test.json`(M1 test, tags_v2), `kiro/zone-map-v3` 환경 v3 루프 test 원본(`outputs/zone-env-v3-20260926/loop/test`, 기록 커밋 `007949bf`).

### 문서·웹 페이지

- torchvision LRASPP 문서: https://pytorch.org/vision/stable/models/lraspp.html
- MuJoCo Python 렌더링(분할 렌더): https://mujoco.readthedocs.io/en/stable/python.html
- Ultralytics 라이선스: https://www.ultralytics.com/license
- 저장소 `docs/model_artifacts.md`(모델 배포 절차), `docs/tensorboard.md`.
