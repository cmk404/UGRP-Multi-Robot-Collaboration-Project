# 테스트 인덱스

테스트 파일은 구현 영역과 같은 용어를 사용한다. 전체 검사는 저장소 루트에서
`python scripts/run_ci_tests.py`로 실행한다. 개별 파일의 직접 실행보다 이 진입점을 우선한다.

## 영역별 찾기

| 파일 이름 | 검증 영역 |
|---|---|
| `test_camera_*`, `test_visual_*`, `test_markerless_*` | 카메라 입력, 시각 제어, 표식 없는 상자 작업 |
| `test_real_*`, `test_masterpi_*` | 실물 로봇 경로, 기구·동역학, 대시보드 |
| `test_sim_*`, `test_grasp_*`, `test_physics_*` | MuJoCo 브리지, 집기 환경, 물리 충실도 |
| `test_warehouse_*`, `test_*_warehouse_*` | 창고 임무, 평가, 관측, 다중 로봇 실행 |
| `test_gemini_*`, `test_llm_*`, `test_groq_*` | 모델 transport, 예산, 복구, 실행 피드백 |
| `test_coela_*`, `test_crew_*`, `test_cooperative_*` | 협력 정책과 통신·이동 메트릭 |
| `test_navigation_*`, `test_approach_*`, `test_placement_*` | 탐색, 접근, 배치 기하와 시간 증거 |
| `test_task_recovery.py`, `test_*_recovery.py` | 실패 감지와 복구 |

## Fixture

[`fixtures/`](fixtures/)에는 CI에서 재현 가능한 작은 이미지와 메타데이터만 둔다.
각 fixture 묶음은 `manifest.json` 또는 `metadata.json`으로 출처·의도를 설명한다.
raw 영상, 전체 실행 로그, 모델 파일은 fixture로 추가하지 않는다.

새 테스트 파일은 `test_<기능>.py` 형식을 따르고, 같은 기능의 테스트는 가능한 한 기존 파일에
추가한다. 실제 네트워크·로봇·비밀키가 필요한 동작은 기본 CI에서 실행하지 않는다.
