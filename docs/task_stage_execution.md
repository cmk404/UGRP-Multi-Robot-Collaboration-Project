# 공동 단계 관리기와 실제 실행 포트 연결

2026-09-16. PR55의 [단계 계약](task_stage_sync_contract.md)에
`harness/task_stage_execution.py`의 `TaskStageExecution`을 추가한다.
창민 담당인 보고 수신·명령 허가·공동 정지·재개를 실제 `CameraRobotPort`와 연결한다.
상위 계획·경로 계산·파지/지지 영상 판별기를 새로 구현하는 작업은 아니다.

## 연결한 것

- 각 로봇에 실제 자기 RGB·공용 top RGB, 자기 발행 명령 이력, 고정 작업 설명과 단계 문맥을
  JSON으로 전달한다. 두 이미지 원본·해시와 전달한 JSON 전체를 보존한다.
- 실행 담당의 영상 판단을 해당 요청의 원래 관측 시각/단계/epoch에 묶어 받는다.
  늦게 도착한 판단을 새 영상처럼 취급하지 않는다. 추가 필드·다른 로봇/이전 요청·재사용은 거부한다.
- 양쪽 명령의 형식·범위·제한 시간을 먼저 검사한다. 허가 확인 후 같은 SIM 시점에서 순서대로
  제출하며, 도중 한쪽 제출이 실패하면 두 포트 모두 정지시킨다.
- HOLD/보고 만료/계획·단계 변경은 바퀴 0과 팔의 대기 중 보간 취소로 연결한다.
  팔은 **마지막으로 발행한 보간 목표**를 유지한다. 측정 관절을 읽지 않으며, 물리적 정지나
  하중 유지 성공을 뜻하지 않는다. 남은 관성·침하·낙하 가능성은 별도 평가가 필요하다.
- 각 포트도 최대 0.25초 lease를 집행한다. 명령 시간이 끝나면 미완료 팔 이동까지 취소한다.
  조정기가 새 보고를 받지 않아도 물리 소유자가 포트 `tick()`을 호출하면 만료된다.
- 시간 경과/명령 접수는 DONE이 아니다. 새 양쪽 영상 보고가 있어야 재개/다음 단계로 넘어간다.
  판별기가 아직 없으면 `uncertain_reply()`가 UNCERTAIN을 반환하고 움직이지 않는다.

## 실행 담당 연결 예시

물리 소유자만 어댑터와 포트를 가진다. 영상 판단 함수에는 `capture()`가 반환한 JSON 사본만
전달한다. 그 함수에 world, 포트, 평가 결과, 실시간 좌표/관절/접촉을 주지 않는다.

```python
execution = TaskStageExecution(plan, local_ports, map_path=static_map_path,
                               output=evidence_path, now_s=sim_time)
request = execution.capture(robot_id, own_rgb=own_jpeg, top_rgb=top_jpeg, now_s=sim_time)
reply = visual_stage_producer(request)  # 실행 담당 구현; 없으면 uncertain_reply(request)
execution.receive(robot_id, reply, now_s=sim_time)
# 양쪽 보고를 받은 뒤:
permission = execution.authorize(now_s=sim_time)["permission"]
execution.dispatch_pair(permission, commands, now_s=sim_time)
# 매 물리 step 직전: execution.tick(sim_time)
# 새 DONE 보고를 모두 받은 뒤: execution.advance(now_s=sim_time)
# 종료 시 finally: execution.close(now_s=sim_time)
```

`reply`는 `request_id, status, confidence, checks, command_id, reason, decided_at_s`만 받는다.
판단 시각은 동일 SIM 시계를 사용한다. `checks`는 기존 단계 계약의 영상 확인 항목이다.
`command_id`는 해당 로봇의 자기 발행 이력에서 참조한다. DONE은 명령 제한 시간 이후의
새 영상이어야 한다. `receive(robot_id, …)`의 로봇 ID는 연결자가 정한 엔드포인트에서 온다.

`commands`는 각 참여자의 `{command_id, action, duration_s}`다. CARRY는 drive/mecanum/wait,
다른 단계는 arm/wait를 받으며, drive의 시간은 봉투의 시간과 같아야 한다. 목표 pulse가
0.25초 안에 도달하지 않으면 중간 발행 목표에서 멈춘다. 재개는 새 허가·새 명령으로 한다.
이 제약은 stage의 물리적 의미나 잡는 힘을 검증하는 기능은 아니다.

## 동작 조건과 남은 경계

현재 **하나의 프로세스, 직렬 호출, 공통 SIM 시계** 연결이다. 실제 네트워크 버스, 로봇별 별도
프로세스, 시계 동기화, OS 재시작 복구, 실물 watchdog을 구현한 것으로 보고하지 않는다.
기존 `team_bus`의 텍스트/깨우기 채널은 건드리지 않는다. JSON 경계는 별도 프로세스 연결 시
사용할 입력 계약이며 지금은 로컬 함수로 전달한다.

어댑터가 연결된 포트는 유일한 명령 발행자여야 한다. 같은 로봇에 기존 `scene.replay()`나
직접 servo/wheel 쓰기를 섞으면 보호를 우회한다. 기존 운반 실행기는 자동으로 이 경로로
바뀌지 않는다. 기존 파지·내려놓기 시연 명령을 영상 DONE으로 포장하지 않는다.

영상 판별 중 SIM은 진행하지 않는 방식으로 사용한다. 실제 시간이 흐르는 독립 워커에서는
워커 측 타이머·종료 시 정지와 시계 변환이 추가로 필요하다. `tick()` 없는 물리 실행은 금지한다.
HOLD 이후 양쪽 새 READY, 마지막 명령 뒤 DONE, 명령 실패 시 재계획 등은 계약 테스트로 검증한다.

## 재현과 판정 범위

```sh
python scripts/ugrp_session.py run stage-execution -- \
  /absolute/path/to/mjpython scripts/probe_task_stage_execution.py \
  --output outputs/stage-execution-new
```

Ubuntu에서는 mjpython 대신 python을 사용한다. 외부 모델/키/학습 파일은 필요 없다.
실행 코드를 먼저 커밋해야 하고 결과 폴더를 덮어쓰지 않는다. 6개 조건은 다음과 같다.

1. 판별기 미연결: 실제 RGB를 저장하지만 UNCERTAIN으로 정지.
2. 5단계: **합성 READY/DONE**으로 실제 팔/바퀴 포트의 발행·만료·전환 확인.
3. 팔 이동 중 HOLD: 대기 이동 취소, 한쪽 READY 차단, 양쪽 새 READY 후 재개.
4. CARRY 보고 단절: 실제 바퀴 명령 만료, 보고 만료 HOLD, 새 보고 뒤 재개.
5. 두 번째 포트 제출 오류: 먼저 제출한 팔 이동도 취소.
6. 회수된 허가 재사용: 양쪽 보고가 새로 모여도 이전 허가로는 움직이지 않음.

RGB, 입력/응답, 사건, 평가 전용 위치/관절 기록, 영상, 모델 XML, 소스 SHA와 해시를 저장한다.
기본 장면의 카메라·외관·물리 설정을 바꾸지 않고 weld를 켜지 않는다.
이 시험의 합성 판정은 카메라 이해 능력 검증에 사용할 수 없다.
**실제 공동 파지·운반·하중 유지·놓기 성공 및 영상에 의한 전체 자율 완주는 이 시험 범위 밖이다.**

다음 연결은 실행 담당의 실제 단계별 영상 판별기를 이 계약에 붙이고, 그 뒤 적재 상태에서
정지·지지 확인·해제·실패 복구를 검증하는 것이다. 이후 별도 워커/통신 끊김 시험으로 확장한다.
