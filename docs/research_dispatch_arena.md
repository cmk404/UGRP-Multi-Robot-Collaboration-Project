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

# 검증된 모델 번들 복원 (저장소에 포함, 교사 raw 자료 불필요)
unzip experiments/dispatch-skill-integration-20260917/models.zip -d outputs/dispatch-models

# 실제 세 LLM 합의 + 기존 RGB 스킬 + 물리 E2E
python scripts/ugrp_session.py run dispatch-e2e -- \
  .venv-sim-worker-mac/bin/mjpython scripts/run_dispatch_e2e.py \
  --executor skills --variant open --seed 11 \
  --grasp-model-dir outputs/dispatch-models/models/grasp \
  --stage-model-dir outputs/dispatch-models/models/varied \
  --output outputs/dispatch-e2e-NEW --max-wall-s 900
```

`--variant`는 `open`, `shared_crossing`, `north_blocked`, `narrow_south`, `rough_south` 중 하나다.
`--seed`는 로봇 ID와 세 시작 슬롯의 대응을 바꾼다. 조건 간 비교는 같은 seed를 사용한다.
기본 실행은 무료 환경 검사이며, `--planner fixture`는 외부 모델 없이 계획 계약을 검사한다.
`--planner llm`은 기존 개인 모델 프록시를 사용하고 비용이 발생할 수 있다. 최대 8라운드/24호출, 요청별 60초 한도다.
출력 폴더를 재사용하면 시작 전에 거부한다.

## 경계와 연결 상태

| 부분 | 상태 |
|---|---|
| 실제 충돌 지형·동적 화물·3대 로봇·RGB | 다섯 환경의 로딩/이동 진단 검증 |
| 지도와 물리 환경 연결 | 정적 지도/최종 scene/로봇 XML hash 및 접촉 프로필 기록 |
| 세 LLM 공동 계획 → 실제 담당 로봇 | 새 F1 합의에서 R1/R3 빔, R2 상자, dock_a 및 선행 작업을 실제 포트에 연결 |
| 단계·자원 허가 | 접근 병행, 집기 전 운반 자원 예약, 선행 작업·계획 hash 검사, 공동 운반 RGB 동기화 |
| 운반 스킬 | 기존 접근/파지 학습기·시연 팔 동작·VisualBoxSkill/VisualMacroExecutor 재사용. 새 배경은 승인된 교사 자료로 재학습 |
| 새 환경 실제 E2E | open/seed11 최종 1회 완주: 빔 1.702m·상자 2.369m, 둘 다 슬롯 내 방출·안정 |
| 좁은 통로·미지 배치·실시간 비동기·주행 중 재계획·실물 | 미검증. 0.685m 통로는 기존 0.93m 평행 대형의 한계 때문에 진입 거부 |

기본 E2E 실행기는 `--executor skills`이며 모델 디렉터리를 요구한다. `--executor raw`로만
이전 저수준 LLM 진단을 선택한다. `--plan-replay`는 기록 계획을 사용하는 명시적인 fixture 진단이다.
계획 전용 `run_research_dispatch.py`가 출력하는 `unbound_new_arena_skill`은 실행기를 연결하지 않는
그 경로의 범위를 뜻한다. 실제 연결은 `run_dispatch_skills.py`와 `dispatch_pair_skill.py`에 있다.

`--contact-profile local_contact`는 집게/화물 접촉의 마찰 방향 계산을 0.5ms 간격으로 해석한다.
질량·형상·기존 마찰계수·모터 힘·법선 접촉 파라미터·카메라는 유지하며 weld/접착력은 사용하지 않는다.
이 수치 프로필은 실물 보정 결과가 아니다. 과거 물리 조건은 `legacy`, 전역 NoSlip은 별도 비교 진단이다.
수정 후보와 제외한 불안정 조건을 [전체 기록](../experiments/dispatch-skill-integration-20260917/README.md)에 남겼다.

학생 입력은 자기 RGB, 공용 TOP RGB, 승인된 정적 지도, 자기 발행 명령과 동료 메시지다.
영상에서 추정한 위치·모양과 실제 평가 좌표를 구분하며, 접촉/관절 측정/평가 성공으로 제어를 보정하지 않는다.
자기 명령은 실제 이동·관절 상태의 증명이 아니다. 최종 요청 재구성과 상자 517개 결정 재실행을 확인했다.
교사 시연 성공과 학생의 독립 실행 성공은 별도로 기록한다.

LLM 추론 중에는 SIM이 정지한다. 이 결과를 실시간 분산 성능, 자유로운 재계획 성능이나
모든 역할 조합의 물리 일반화로 해석하지 않는다. 팀원의 연구 비교/계획/로컬 제어 역할이 완료됐다는 뜻도 아니다.

## 자료

- [최신 스킬 통합 E2E·재현 모델·전체 실패 기록](../experiments/dispatch-skill-integration-20260917/README.md)

- [E2E 진단 결과·실패 원인·후속 연결 지점](../experiments/dispatch-e2e-20260917/README.md)
- [실제 환경·카메라·합의 결과 뷰어](../experiments/research-dispatch-arena-20260917/index.html)
- [검증 결과](../experiments/research-dispatch-arena-20260917/README.md)
- [통제 조건과 후속 비교 실험](../experiments/research-dispatch-arena-20260917/protocol.md)

뷰어는 외부 라이브러리나 서버가 필요 없는 HTML이다. 각 조건/카메라 버튼을 눌러 실제 저장 프레임을 비교한다.
실시간 관제 화면이 아니며 아직 실행하지 않은 운반 성공률을 표시하지 않는다.
