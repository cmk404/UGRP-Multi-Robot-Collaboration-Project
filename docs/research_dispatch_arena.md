# 공동 출하 연구 환경

연구 질문은 **동등한 세 LLM의 자연어 합의가 역할 분담·협력 이송·예외 대응에 도움이 되는가**이다.
공동 목표는 빔과 상자를 같은 출하 구역의 각 슬롯에 놓아 한 주문을 완성하는 것이다.

- 실제 MuJoCo 공간: 픽업 구역, 중앙 섬, 북/남 공용 통로, 대기 공간, 공용 출하 앞마당, 선택 가능한 출하 구역 A/B.
- 다섯 조건: 기본 공간, 공용 통로, 지도 밖 북쪽 장애물, 남쪽 39cm 협소부, 남쪽 8mm 턱.
- 계획이 담당 로봇·파트너·경로·목적지·선행 작업을 정한다. 세 로봇이 동일한 계획 ID/hash를 승인해야 프로그램을 생성한다.
- 각 로봇의 접근 준비는 독립적이며 공동 파지·들기·내리기는 해당 팀의 동기화 대상이다.
- 출하 앞마당/통로 예약은 명시적이다. 응답 끊김·재계획이 기존 점유를 자동 해제하지 않는다.

## 실행

이 문서 경로 기준의 저장소 루트에서 실행한다. macOS는 기존 `.venv-sim-worker-mac/bin/mjpython`, Ubuntu는 `.venv-dev/bin/python`을 사용한다.
다른 worktree에 가상환경을 새로 만들 필요 없이 기본 프로젝트의 절대 경로 실행기를 사용할 수 있다.

```sh
# 환경 생성 + 각 카메라 프레임. 모델 호출 없음.
python scripts/ugrp_session.py run dispatch-preview -- \
  .venv-sim-worker-mac/bin/mjpython scripts/run_research_dispatch.py \
  --output outputs/dispatch-preview-NEW --variant shared_crossing

# 세 로봇 모터 진단: 합의 계획 실행이나 운반 성공 검사가 아님.
python scripts/ugrp_session.py run dispatch-smoke -- \
  .venv-sim-worker-mac/bin/mjpython scripts/run_research_dispatch.py \
  --output outputs/dispatch-smoke-NEW --variant north_blocked --motion-smoke

# 실제 모델 호출: 공동 계획 합의와 로봇별 프로그램 생성까지만.
python scripts/ugrp_session.py run dispatch-plan -- \
  .venv-sim-worker-mac/bin/mjpython scripts/run_research_dispatch.py \
  --output outputs/dispatch-plan-NEW --variant shared_crossing --planner llm
```

`--variant`는 `open`, `shared_crossing`, `north_blocked`, `narrow_south`, `rough_south` 중 하나다.
`--seed`는 로봇 ID와 세 시작 슬롯의 대응을 바꾼다. 조건 간 비교는 같은 seed를 사용한다.
기본 실행은 무료 환경 검사이며, `--planner fixture`는 외부 모델 없이 계획 계약을 검사한다.
`--planner llm`은 기존 개인 모델 프록시를 사용하고 비용이 발생할 수 있다. 최대 8라운드/24호출, 요청별 60초 한도다.
출력 폴더를 재사용하면 시작 전에 거부한다.

## 경계와 연결 상태

| 부분 | 상태 |
|---|---|
| 실제 충돌 지형·동적 화물·3대 로봇·RGB | 구현 및 로딩/이동 진단 검증 |
| 정적 지도와 물리 환경 연결 | 지도/scene/로봇 XML hash 기록 |
| 실제 세 LLM의 공동 계획 | 기본/북쪽 장애물 조건에서 실행 기록 |
| 계획 → 로봇별 담당/목적지/순서 | 프로그램 생성 및 계약 검사 |
| 단계·자원 허가 | `DispatchCoordinator` 계약 검사; 물리 포트 미연결 |
| 새 환경의 로봇별 운반 스킬 | 미연결: 모든 프로그램에 `unbound_new_arena_skill` 명시 |
| 실제 운반 E2E·실시간 비동기·실물·재계획 성능 | 아직 검증하지 않음 |

기존 #62의 고정 역할 운반은 별도 기준선으로 보존했다. 새 환경을 기존 고정 레인 학생에
몰래 투입하거나 좌표로 보정하지 않는다. 현재의 협의 결과는 물리적인 운반 성공이 아니며,
모델이 상대 로봇 위치를 잘못 추정해도 형식상 합의는 가능하다. 합의와 지각 정확도를 분리해서 평가한다.

`harness/dispatch_plan.py`는 계획/프로그램/상위 자원 계약을 담당한다. 실제 파지 단계의
관측 최신성·하중 지지·명령 허가는 기존 `TaskStageSync`/`TaskStageExecution`에 연결해야 한다.
이 환경 작업은 팀원의 로컬 제어/계획 역할을 완료했다고 표시하지 않는다.

## 자료

- [실제 환경·카메라·합의 결과 뷰어](../experiments/research-dispatch-arena-20260917/index.html)
- [검증 결과](../experiments/research-dispatch-arena-20260917/README.md)
- [통제 조건과 후속 비교 실험](../experiments/research-dispatch-arena-20260917/protocol.md)

뷰어는 외부 라이브러리나 서버가 필요 없는 HTML이다. 각 조건/카메라 버튼을 눌러 실제 저장 프레임을 비교한다.
실시간 관제 화면이 아니며 아직 실행하지 않은 운반 성공률을 표시하지 않는다.
