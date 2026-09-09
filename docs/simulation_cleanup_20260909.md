# 시뮬레이션 환경 정리 — 2026-09-09

요청: 오래된 시뮬레이션 환경을 제거하고 클라우드 시뮬레이션은 작은 기록으로 남긴다. 사용자가 확인한 범위는 프로젝트 안의 파일·환경이며 외부 서버·계정 삭제는 제외한다.

## 보존 기준

현재 Mac의 `MultiMasterPiProductionV2(warehouse_layout="camera_team")`와 Gemini 운반 실행 경로, 실물 로봇 코드, 기존 실험 결과·영상은 보존한다. `masterpi_scene.xml`은 현재 V2 빌더의 입력이며 `masterpi_scene_v2.xml`은 회귀검사 기준이므로 오래된 이름만 보고 삭제하지 않는다. 태그 PNG와 이전 실패 영상에서 추출한 fixture도 현재 호환·판정 검사에 필요하다.

## 로컬 환경

- 옛 Linux `.venv-sim`, `.venv-lightning`, 중복 Mac `.venv-sim-mac` 실제 설치 폴더를 삭제했다. 삭제 전 할당 크기 합계 1,207,734,272 bytes(약 1.21 GB), Python·패키지 목록은 [구성 기록](retired_sim_environments_20260909.json)에 보존했다.
- 현재 `.venv-sim-worker-mac`과 실물 로봇용 `.venv-real-mac`은 유지했다. `.venv-sim`은 기존 실행기 호환을 위해 현재 워커로 연결되는 symlink로 바꿨다. 별도 설치가 아니다.
- [클라우드 퇴역 요약](cloud_simulation.md)을 작게 남기고 예전 코드 복원 기준을 Git `76aeba36461ddae2044e8b732adb00b8365dde67`로 지정한다. Git 이력을 삭제하거나 대용량 백업을 새로 만들지 않는다.
- 이름상 혼동을 주던 `CLOUD_SYNC.md`는 GitHub 수동 동기화 설명이므로 `GITHUB_SYNC.md`로 바꾸고 유지했다.

## 검증

- [제거한 47개 파일의 목록·크기·SHA-256](retired_sim_sources_20260909.json): 구형 물리 엔진 8개, 전용 학습·평가·렌더·stress 도구, 클라우드 배포 코드와 전용 테스트·서비스를 함께 제거했다. 파일명 변경 `CLOUD_SYNC.md`→`GITHUB_SYNC.md`는 이 삭제 수에서 제외했다.
- Mac 워커 복구는 공유 진입점을 유지하되 클라우드 제공자 선택·배포를 제거했다. 예전 cloud opt-in 설정이 남아도 Azure·Colab·Lightning을 실행하지 않는다. 로컬 `.env.gpu`에서도 폐기한 설정·인증 항목 5개를 제거했다. 비밀값은 기록·Git에 복사하지 않았다.
- Mac 복구·UI·다중 로봇 라우팅·Bridge·하네스 관련 217개 검사 통과. 최종 구성 파일명 호환 보완 후 복구/UI 25개를 다시 확인했다. 기존 `.env.gpu` 파일명은 유지한다.
- 삭제된 Python 모듈의 살아 있는 직접 import가 없는지 AST로 검사했다. `git diff --check` 통과.
- 기본 오프라인 회귀 188개 + 세부 검사 107개 통과. 현재 장면/공유 제어 관련 48개와 Bridge 35개 통과. 구형 엔진 전용 테스트는 기능과 함께 제거했으며 현재 운반 판정 검사를 삭제한 것은 아니다.
- 현재 camera_team 단일 화물 장면을 생성하고 100 물리 스텝(0.2 SIM초)을 진행했다. 상태가 유한함을 확인하고 NAV 및 관찰 카메라 JPEG를 생성·직접 검토했다. 이 검사는 초기 장면과 렌더링 확인이며 새 전체 운반 성공 검증이 아니다. 기록은 `outputs/simulation-cleanup-20260909/smoke.json`, 이미지는 같은 폴더에 있다. 실행 객체는 종료했다. 새로운 유료 모델 운반 실험이나 원격 서버 조작은 하지 않는다. 오프라인 CI는 코드 검증으로 유지하며 클라우드 시뮬레이션 배포와 구분한다.
