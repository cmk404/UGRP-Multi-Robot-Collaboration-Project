# 표준 시뮬레이션 관리

실행 진입점은 `bash scripts/open_simulation.command`이며 Python에서는 `python -m scripts.sim_cli`다. 기본 창·설정 실행, 공동 출하, 통신 비교, 주행, ACT·교사·학습과 평가 도구를 같은 카탈로그에서 선택하고 같은 실행 기록으로 추적한다. 실행 대상과 예산은 현재 프로젝트 지침과 해당 workflow의 요구사항을 따른다.

Colab에서도 같은 소스·CLI를 사용하는 [L4 적용 검토](colab_standard_simulation_review_20260923.md)를 참고한다. 현재 전송기의 CPU 렌더링 강제 설정과 GPU 학습 경로는 별도 연결이 필요하며 L4 실제 실행 완료를 뜻하지 않는다.

## 한곳에서 선택하고 실행하기

```bash
# 등록된 연구 실행과 요구사항
bash scripts/open_simulation.command workflow list

# 실제 실행 없이 명령·입력·출력·버전 확인
bash scripts/open_simulation.command workflow plan stage-sync

# 외부 모델/물리 실행 없는 동기화 fixture
bash scripts/open_simulation.command workflow run stage-sync

# 기존 설정 실행도 같은 공통 관리 기록에 연결
bash scripts/open_simulation.command run configs/simulation/drive.json --headless

# 관리된 실행 목록과 특정 실행 기록
bash scripts/open_simulation.command workflow runs
bash scripts/open_simulation.command workflow show RUN_ID
```

`workflow plan/run <ID> -- <기존 인자>` 형식으로 연구별 옵션을 전달한다. plan은 모델·훈련·시뮬레이션을 시작하지 않는다. 외부 모델, 원격 제출, 교사 데이터와 학습은 해당 workflow와 인자를 명시해 선택한다. 카탈로그에 표시된 실행 요구사항과 연구별 문서를 확인한다. 서버·하드웨어 및 기록 분석 도구의 실행 완료는 시뮬레이션 성공 판정이 아니다.

## 관리하는 것

| 공통 기록 | 용도 |
|---|---|
| workflow ID·버전·카탈로그 해시 | 어떤 연구 실행을 선택했는지 식별 |
| Git SHA·dirty 상태·실제 소스 해시 | 같은 커밋에서도 로컬 수정 여부 구분 |
| 실행 인자·설정·입력 파일 해시 | 지도·모델·계획·입력 변경 추적 |
| Python·패키지 환경 | 소스가 같아도 환경이 다른 실행 구분 |
| 시작·종료·중단·시간 제한·exit code | 실패와 중단을 포함한 실행 수명 기록 |
| 결과 위치·파일 해시 | 원본 결과와 실행 조건 연결 |

기존 실행기가 저장하는 장면 XML·실제 물리 설정·모델 요청·영상·평가 결과는 그대로 보존한다. 공통 기록은 이를 감싼 실행 이력이다. `exit code 0`은 프로세스 종료 상태이며 물리적 운반 성공은 각 실행의 평가 자료로 확인한다. 실제 실험은 소스를 먼저 커밋하고 고정하며, 원본과 과거 실행 폴더를 덮어쓰지 않는다. 로컬 해시는 원격 백업을 뜻하지 않는다.

## 하나의 관리 체계와 여러 실험 프로필

표준 설정 실행과 기존 공동 출하·공통 RGB adapter는 `sim.session_scenes.Scene`의 출하장 XML 생성·초기화를 공유한다. 기존 출하장 실행기의 전역 XML 생성기 교체를 제거하고 세계 인스턴스마다 같은 장면 정의를 적용한다. 카메라 배치/FOV, 초기 servo 명령과 안정화 시간, weld OFF를 보존한다.

접촉 설정과 제어기까지 무조건 하나의 기본값으로 합치지는 않는다. 과거 `local_contact_fine` 성공과 `legacy` adapter 진단은 서로 다른 실험 조건이다. 실행 시 선택한 프로필과 실제 적용값을 기록하고 [실행 번들 버전](execution_versioning.md)으로 검증한다. 현재 공통 RGB 후보는 `rgb-adapter-contact-fine-boundary-identity-v5`이며 공동 물리 완주를 재검증해야 한다. 이전 번들 JSON은 변경하지 않고 원래 소스로 재현한다.

나머지 연구 실행기는 같은 관리 계층에 등록된 호환 어댑터다. 전체 제어 루프를 `Simulation.step()`으로 재작성했다는 의미는 아니다. 기존 스크립트 직접 호출과 Python API 직접 사용도 호환을 위해 남아 있지만 그 자체로 공통 실행 기록이 생성되지는 않는다. 신규 실험과 신규 workflow는 표준 관리 진입점에 연결하며 독립적인 버전·결과 관리 체계를 추가하지 않는다.

## 새 실행 경로 추가

`configs/simulation_workflows.json`에 ID·버전·진입점·출력 규칙·실행 요구사항을 등록하고 관련 테스트를 추가한다. 공통 장면·초기화 구현을 재사용하며, 다른 물리·카메라·명령 조건이 필요하면 명시적 프로필/버전으로 추가한다. 기존 성공 조건의 변경 전후 차이와 검증 범위를 기록한다. 카탈로그 등록이나 자동 검사 통과는 연구 과제 완주 검증을 대신하지 않는다.
