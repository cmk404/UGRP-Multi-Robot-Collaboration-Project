# REAL MasterPi Causal Trace / RCA System

## 목적

REAL 로봇 실행에서 단순히 `approach failed` 같은 마지막 오류만 남기지 않고, **어떤 입력을 보고 어떤 상태에서 어떤 판단을 내려 어떤 명령을 보냈으며 그 직후 무엇이 달라졌는지**를 한 사용자 요청 단위로 재구성한다.

이 로거는 제어 경로와 분리되어 있다. 로그 파일 생성·JPEG 저장·원격 trace 복사에 실패해도 그 실패가 모터/서보 동작의 성공·실패를 바꾸지 않도록 설계한다.

## 실행 단위

REAL + execute + agent 요청마다 하나의 `run_id`를 만든다.

```text
outputs/real_traces/<run_id>/
├── run.json                 # 사용자 명령, 시작 상태, 시작 commanded pose
├── events.jsonl             # harness/executive/planner timeline
├── result.json              # 최종 LoopResult 또는 harness 오류
├── analysis.json            # 자동 RCA 기계 판독 결과
├── analysis.md              # 사람이 읽는 RCA 요약
├── harness_frames/          # planner/observe에 실제 사용한 프레임
└── skills/
    └── <span_id>/
        ├── result.json      # skill, argv, code hash, 시작/종료/exit code
        ├── events.jsonl     # Pi-side camera/detection/pose/actuator timeline
        ├── stdout.log       # controller 판단 출력 전체
        ├── stderr.log       # 실제 오류 원문
        ├── debug-failure.jpg
        └── frames/
            └── frame-*.jpg  # controller가 실제 소비한 카메라 입력
```

각 `search`, `track`, `approach`, `pick`, `place`, `put_down`, primitive 실행은 별도 `span_id`를 가지지만 같은 `run_id` 아래에 저장된다. 따라서 재시도나 recovery가 있어도 요청이 서로 섞이지 않는다.

## 기록하는 증거

### Harness / planner

- 사용자 원문 명령
- 요청 시작 시 `world_state`
- executive/LLM step의 raw output, parsed action, tool result, error
- planning / tool-start activity
- **LLM/VLM에 실제 전달된 정확한 이미지 파일**
- 그 planner call 당시의 compact world state
- planner 이미지 당시 streamed commanded pose, pose age, 안정 여부
- 각 tool 뒤 harness가 실제 관측한 post-action 이미지와 pose

### Pi skill

- controller가 `read()`로 **실제로 소비한 모든 카메라 프레임**
  - `UGRP_REAL_TRACE_ALL_FRAMES=1`인 REAL production이 기본이다.
  - MJPEG에서 decode됐지만 controller가 사용하지 않은 중간 프레임까지 무의미하게 저장하지는 않는다.
- frame sequence / source / image shape / JPEG 경로
- 각 frame 시점의 commanded PWM pose와 pose-state age
- detection과 정확한 `frame_seq` 연결
  - color, visible, cx/cy, nx/ny, area, width/height, box points, rectangularity, detector
- servo 단일/배치 명령
  - 이전 pulse, 목표 pulse, duration, mode
- chassis 명령
  - direction, speed, duration, wheel command
- command completion / motor stop
- hardware probe
- controller stdout 전체
  - face angle, range, progress, chosen direction, threshold reason 등 기존 판단 출력 포함
- 실패 stderr 원문
- 실패 debug frame을 span마다 별도 보존하여 다음 실패가 덮어쓰지 못하게 함

정밀 접근 경로의 `LiveVideo`도 recorder에 직접 연결되어 있어 `face-align -> pre-capture -> fixed capture -> grasp` 구간의 입력 프레임도 빠지지 않는다.

## 자동 RCA

한 run이 끝나면 `harness.trace_analysis`가 자동으로 `analysis.json`과 `analysis.md`를 만든다. 현재 자동 추출 항목은 다음과 같다.

- 첫 실패 skill/span
- failure code 및 실제 stderr reason
- 실패 직전 마지막 detection
- 실패 직전 마지막 commanded pose
- 실패 직전 마지막 actuator command
- 마지막/이전 입력 frame
- `visible=true -> false`가 발생했다면 정확한 before/after frame과 두 시점 pose
- controller decision stdout 최근 구간
- 각 skill의 duration / exit code / trace 위치

한 번 실패했다가 recovery로 최종 루프가 종료된 경우에는 `COMPLETED_WITH_FAILURES`로 표시해, 성공적인 recovery가 있었던 run을 단순 `FAILED`로 오분류하지 않는다.

수동 재분석:

```bash
.venv-sim/bin/python scripts/analyze_real_trace.py latest
.venv-sim/bin/python scripts/analyze_real_trace.py <run_id>
```

## 물리적으로 알 수 없는 것

현재 MasterPi 하드웨어/스택의 센서 한계는 trace 안에서도 명시적으로 구분한다.

- `servo 4 = 2200`은 **명령된 PWM pose**다. 독립적인 joint encoder가 없으므로 실제 관절이 정확히 그 위치에 도달했음을 직접 측정한 값이 아니다.
- `right, speed=35, duration=0.20s`는 **차체에 보낸 명령**이다. 검증된 wheel odometry가 없으므로 실제 이동 거리를 로그만으로 cm 단위 확정할 수 없다.
- 따라서 영상 변화, detector 결과, commanded pose, command timing을 결합해 소프트웨어/인지/제어 전이 원인을 분석할 수 있지만, 미끄러짐·서보 부하 같은 물리 오차를 센서가 없는 상태에서 사실처럼 단정하지 않는다.

## 운영 주의

- Oracle 서버가 장기 보관소다. Pi의 `/tmp/ugrp-real-traces/<run>/<span>`은 Oracle로 복사가 성공한 span만 정리한다.
- trace 저장/복사 실패는 robot control을 실패시키지 않으며, 가능한 경우 누락 사유를 별도로 기록한다.
- 모든 controller-consumed frame을 보존하므로 장기 실험에서는 `outputs/real_traces/` 용량을 주기적으로 확인한다. 자동 삭제는 하지 않는다. forensic evidence를 임의로 지우지 않기 위함이다.
- 소스 변경은 현재 실행 중인 장기 프로세스에 자동 주입되지 않는다. REAL 서비스가 안전하게 재시작된 이후 새 요청부터 이 시스템이 활성화된다.

## 진단 API와 대시보드

REAL backend는 trace evidence를 읽기 전용으로 노출한다.

- `GET /api/real-traces` — 최근 run 목록, 상태, 첫 실패, RCA class, 실패 경계, 저장 용량
- `GET /api/real-traces/<run_id>` — 한 run의 meta/result/automatic analysis
- `GET /api/real-traces/<run_id>/asset/<path>` — 해당 run 내부의 JPEG/PNG/WebP evidence만 제공

asset endpoint는 absolute path와 `..` traversal을 거부하고 이미지 확장자만 허용한다. 따라서 trace API를 통해 arbitrary project/server file을 읽을 수 없다.

메인 dashboard에서 REAL 탭을 열면 `최근 REAL 실행 분석` 패널이 표시된다. 최근 6개 run의 상태와 최초 실패 이유를 보여주며 최신 run에는 가능한 경우 다음을 같이 표시한다.

- `root_cause.category` / confidence
- 마지막 성공 skill → 첫 실패 skill 경계
- `visible -> lost` 전후 프레임 또는 마지막 두 input frame
- 마지막 commanded pose / actuator command / detection
- controller decision tail
- Pi-side recorder 평균/최대 ms per consumed frame와 post-skill trace copy 시간

실행 SSE의 `done`에도 `real_run_id`, trace status/failure가 포함되어 채팅 실행 카드에서 `진단` 행으로 즉시 연결된다.

## RCA 분류

자동 분석기는 기록된 사실보다 더 강한 물리 원인을 주장하지 않는다. 현재 결정론적으로 구분하는 대표 class는 다음과 같다.

- `CAMERA_TRANSPORT_FAILURE`
- `EXECUTIVE_STATE_CONFLICT`
- `CAUSAL_HANDOFF_INVALID`
- `RANGE_INCONSISTENCY_AFTER_MOTION`
- `FACE_ALIGNMENT_NONCONVERGENCE`
- `CAMERA_ALIGNMENT_LIMIT`
- `GRASP_VISUAL_MISS`
- `TARGET_LOST_AFTER_ARM_POSE_CHANGE`
- `TARGET_LOST_AFTER_CHASSIS_MOTION`
- `TARGET_LOST_WITHOUT_RECORDED_ACTUATION`
- structured `failure_code` fallback
- `UNCLASSIFIED_FAILURE`

예를 들어 `visible -> lost` 사이에 servo batch command가 기록되어 있으면 `TARGET_LOST_AFTER_ARM_POSE_CHANGE`로 분류할 수 있다. 이것은 **arm pose change 뒤 target loss가 관측되었다**는 인과 경계까지의 주장이고, servo stall/블록 이동/occlusion 중 어느 물리 메커니즘인지 센서 없이 임의 확정하지 않는다.

## 로깅 오버헤드 자체의 계측

관측 시스템이 제어 지연의 원인이 되는지도 같은 trace에서 검증한다.

Pi-side skill process가 정상/예외 종료될 때 `recorder_summary`를 best-effort로 남겨 다음을 기록한다.

- consumed/saved frame 수
- `frame_record_avg_ms`, `frame_record_max_ms`
- JPEG save 평균/최대 시간
- commanded-pose file read 평균/최대 시간
- JSONL event write 평균/최대 시간

Oracle deploy wrapper는 물리 skill/SSH 실행 구간과 trace 회수 구간도 분리한다.

- `execution_duration_s`: remote watchdog + physical skill가 반환하기까지의 구간
- `trace_copy_duration_s`: skill이 끝난 뒤 Pi evidence를 Oracle로 회수하는 시간
- `duration_s`: wrapper 전체 wall time

따라서 향후 “로봇 제어가 느려졌다”는 문제가 생겨도 controller 실행 자체가 느린지, Pi-side recorder가 프레임마다 오래 걸리는지, 아니면 skill 종료 후 SCP 회수가 오래 걸리는지 분리해서 볼 수 있다.

Oracle 개발 VM에서 640×480 random BGR / JPEG quality 55 / all-consumed-frame 저장을 120회 측정했을 때 참고값은 JPEG encode 약 2.22 ms/frame, 전체 recorder 약 2.71 ms/frame였다. **이 값은 Raspberry Pi 성능 검증값이 아니며**, 실제 판단에는 첫 physical trace의 Pi `recorder_summary`를 사용한다.
