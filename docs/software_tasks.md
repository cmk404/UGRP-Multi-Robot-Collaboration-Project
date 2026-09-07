# UGRP 소프트웨어 담당자(SW) 역할 및 태스크 명세

> 기준일: 2026-08-14  
> 대상: UGRP 다중로봇 LLM 협력 시스템 소프트웨어 개발  
> 목표: MasterPi 로봇 하드웨어 제어부터 P2P 자연어 통신, LLM 에이전트 연동, 실험 로깅까지의 SW 아키텍처 구축

---

## 1. 소프트웨어 전체 아키텍처 개요

소프트웨어 스택은 하위 물리 제어부터 상위 LLM 추론까지 5계층(Layer)으로 구성됩니다.

```text
[Layer 4] LLM Agent Layer (의사결정, 자연어 P2P 협상, JSON Action 파싱)
   │
[Layer 3] P2P Communication Bus (로봇 간 메시지 큐, 동기화, 브로드캐스트)
   │
[Layer 2] Task & Behavior Execution (상태 머신, Visual Servoing, 예외 감지)
   │
[Layer 1] Hardware Abstraction Layer (HAL) (바퀴 주행, 로봇팔 역기구학, 카메라)
   │
[Layer 0] OS & Network Infrastructure (라즈베리파이 OS, Wi-Fi 핫스팟, SSH/VNC)
```

---

## 2. 모듈별 핵심 담당 업무 및 산출물

### 1) [L0-L1] 하드웨어 추상화 및 기본 제어 (HAL &amp; Robot SDK)

하드웨어의 저수준 모터/센서 제어를 상위에서 쓰기 편한 Python API로 캡슐화합니다.

- **주요 태스크**:
  - [ ] **메카넘 휠 주행 제어 API**:
    - `move(vx, vy, vyaw)`: 평행 이동 및 회전 속도 제어
    - `move_distance(dx, dy, speed)`: 지정 거리만큼 이동
    - `rotate_angle(degree, speed)`: 지정 각도 회전
  - [ ] **4축 로봇팔 &amp; 그리퍼 제어 API**:
    - `arm_move_xyz(x, y, z)`: 기구학/역기구학(IK) 기반 엔드이펙터 위치 제어
    - `gripper_open()`, `gripper_close()`: 물체 파지 및 전류/센서 기반 파지 확인
    - `arm_home()`: 대기 기본 자세 복귀
  - [ ] **안전 및 상태 모니터링**:
    - `get_battery_level()`: 배터리 전압 모니터링
    - `emergency_stop()`: 예외 발생 시 전 모터 정지
- **산출물**: `src/hal/masterpi_driver.py`, `src/hal/arm_controller.py`
- **완료 기준**: 단일 Python 스크립트 실행으로 "주행 -&gt; 물체 앞 정지 -&gt; 파지 -&gt; 회전 -&gt; 내려놓기"가 오차 없이 동작.

---

### 2) [L2] 비전 인식 및 환경 관측 모듈 (Perception &amp; Observation)

카메라 영상으로부터 로봇의 현재 상황과 대상 물체 정보를 텍스트/수치 데이터로 변환합니다.

- **주요 태스크**:
  - [ ] **카메라 스트리밍 &amp; 보정**: 왜곡 보정(Calibration) 및 실시간 프레임 캡처
  - [ ] **물체 탐지 및 좌표 추정**:
    - Color/Blob 또는 ArUco 마커 기반으로 대상 상자의 상대 좌표 $(X, Y, Z, \theta)$ 계산
    - 목표 하차 지점(Goal Zone) 인식
  - [ ] **Visual Servoing (정밀 접근)**:
    - 물체가 카메라 중심에 오도록 로봇 위치를 미세 조정하는 피드백 제어
  - [ ] **텍스트 관측 생성기 (Observation Generator)**:
    - LLM 프롬프트에 주입할 표준 관측 텍스트 생성  
    *(예: `"detected_objects": [{"id": "box_large", "rel_x": 0.45, "rel_y": 0.05, "status": "heavy"}]`)*
- **산출물**: `src/perception/camera.py`, `src/perception/detector.py`, `src/perception/observer.py`
- **완료 기준**: 카메라 시야 내 물체의 거리와 각도를 95% 이상 정확도로 추정하여 관측 JSON을 반환.

---

### 3) [L3] 분산 P2P 통신 인프라 (Networking &amp; Messaging)

중앙 서버 없이 로봇 간(Robot-to-Robot) 메시지와 상태를 비동기로 교환합니다.

- **주요 태스크**:
  - [ ] **P2P 통신 프로토콜 선정 및 구현** (MQTT Broker 또는 Zenoh / WebSocket P2P / ROS2)
  - [ ] **메시지 스키마 및 직렬화**:
    - 협력 요청(`request_partner`), 동의(`accept_partner`), 동기화(`sync_ready`), 상태 보고(`status_update`), 위험 알림(`alert`)
  - [ ] **메시지 수신함(Inbox) 관리**:
    - 비동기로 들어오는 상대방 메시지를 큐에 쌓고, LLM 추론 시점의 Context에 포함
- **산출물**: `src/comm/p2p_bus.py`, `src/comm/message_schema.py`
- **완료 기준**: 로봇 A가 보낸 자연어/구조화 메시지가 100ms 이내에 로봇 B의 inbox에 정상 수신되고 파싱됨.

---

### 4) [L4] LLM 에이전트 및 의사결정 파이프라인 (Agent &amp; Reasoning)

관측 정보와 P2P 수신 메시지를 LLM에 입력하여 다음 행동(Action)과 협력 대화(Speech)를 도출합니다.

- **주요 태스크**:
  - [ ] **LLM API 클라이언트 구현**: OpenAI / Anthropic / Local LLM (Ollama, vLLM) 연동
  - [ ] **프롬프트 템플릿 설계 (System &amp; Context)**:
    - 로봇의 역할, 현재 물리 상태, 주변 물체, 통신 수신함, 사용 가능한 Action 스킬 명세
  - [ ] **Structured Output &amp; 파싱**:
    - LLM 응답을 엄격한 JSON 스키마로 강제 (`action`, `target`, `speech`, `intent`)
    - 파싱 실패 시 재시도 또는 Fallback 안전 행동 정의
  - [ ] **실험 비교군 에이전트 구현**:
    - `RuleAgent`: 고정 규칙 기반
    - `LLMNoCommAgent`: 통신 기능이 차단된 LLM
    - `LLMPeerCommAgent`: P2P 자연어 협상이 가능한 LLM
- **산출물**: `src/agents/llm_agent.py`, `src/agents/rule_agent.py`, `src/agents/prompt_templates.py`
- **완료 기준**: LLM이 상황에 맞는 유효한 Action JSON을 99% 이상 파싱 오류 없이 반환.

---

### 5) 실험 제어기 및 데이터 로거 (Runner &amp; Logging)

실험을 자동으로 구동하고, 모든 측정 지표(성공률, 토큰 비용, 지연 시간, 실패 원인)를 기록합니다.

- **주요 태스크**:
  - [ ] **Episode Runner**:
    - Episode 시작(초기화) -&gt; Step 반복 -&gt; 종료(성공/실패/타임아웃) 루프
  - [ ] **실험 로깅 (JSONL 기반)**:
    - Step별 관측, LLM 입출력 원문, 전송/수신 메시지, 실행된 물리 액션, 토큰 수, 소요 시간 저장
  - [ ] **지표 분석 스크립트**:
    - 성공률, 복구율, 평균 latency, 교착(deadlock) 횟수 집계
- **산출물**: `src/runner/run_experiment.py`, `src/logging/logger.py`, `scripts/evaluate_results.py`
- **완료 기준**: 1회 실험 종료 후 생성된 `run_xxx.jsonl` 파일만으로 당시의 모든 대화와 행동 재현 가능.

---

## 3. 소프트웨어 개발 마일스톤 (우선순위 순서)


| 주차          | 마일스톤                            | 목표 산출물                                              |
| ----------- | ------------------------------- | --------------------------------------------------- |
| **W1** (현재) | **기본 제어 드라이버 및 접속 환경**          | 라즈베리파이 SSH/VNC 완성, 파이썬 기반 바퀴/로봇팔/카메라 개별 구동 검증       |
| **W2**      | **단독 태스크(L1) 파이프라인**            | 카메라로 색상/마커 탐지 -&gt; 접근 -&gt; 파지 -&gt; 이송 완수 스크립트    |
| **W3**      | **P2P 통신 &amp; 메시지 버스**         | 두 대의 로봇(또는 PC와 로봇) 간 MQTT/소켓 기반 비동기 메시지 송수신         |
| **W4**      | **LLM 에이전트 연동 &amp; Action 파서** | 관측 데이터 텍스트화 -&gt; LLM 호출 -&gt; 로봇 물리 액션 자동 매핑       |
| **W5**      | **다중로봇 협력(L2) 및 예외(L3) 구현**     | 두 로봇의 공동 파지 동기화 및 장애물 발생 시 자연어 재협상                  |
| **W6**      | **자동화 로거 &amp; 본실험 데이터 수집**     | 조건별(Rule vs NoComm vs PeerComm) 반복 실험 및 결과 JSONL 수집 |


---

## 4. SW 개발 체크리스트 (Phase 1 기준)

- [ ] 라즈베리파이 OS 기본 세팅 및 Python 3.10+ 환경 구축
- [ ] MasterPi 제조사 제공 Python 제어 라이브러리 정상 동작 확인
- [ ] Git 저장소 클론 및 로컬 가상환경(`venv`) 구성
- [ ] 모터 및 서보 제어 테스트 스크립트(`tests/test_hardware.py`) 작성
- [ ] 카메라 프레임 읽기 테스트 스크립트(`tests/test_camera.py`) 작성

