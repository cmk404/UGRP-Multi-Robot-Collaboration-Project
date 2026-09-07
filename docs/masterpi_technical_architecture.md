# MasterPi 하드웨어 제어 및 시스템 연동 기술 명세서

> 기준일: 2026-08-14  
> 목적: MasterPi의 물리 하드웨어 인터페이스, 네트워크 연결 방식, Python SDK API, 분산 통신 및 LLM 연동 아키텍처에 대한 구체적인 기술 명세

---

## 1. 네트워크 연결 및 원격 접속 기술 스펙

MasterPi는 라즈베리파이 4B를 메인 연산 보드로 사용하며, 두 가지 네트워크 모드로 접속할 수 있습니다.

### 1.1 접속 모드별 네트워크 구성

```text
[모드 A: Direct AP Mode (단독 로봇 디버깅)]
  MasterPi (Wi-Fi AP: HW-MasterPi-xxxx) ──(192.168.149.1)──> 개발 PC (192.168.149.x)

[모드 B: Infrastructure/STA Mode (다중 로봇 UGRP 환경)]
                       ┌── 스마트폰 핫스팟 / Wi-Fi 공유기 ──┐
                       │       (예: 172.20.10.1 / 24)        │
                       ▼                                    ▼
       MasterPi #1 (172.20.10.2)             MasterPi #2 (172.20.10.3)
      (또는 masterpi1.local)                (또는 masterpi2.local)
```

| 항목 | Direct AP 모드 | STA 모드 (권장: 다중 로봇 실험) |
|---|---|---|
| **용도** | 단일 로봇 하드웨어 테스트 및 초기 설정 | 2대 이상의 로봇 P2P 통신 및 외부 LLM API 호출 |
| **IP 주소** | 기본 `192.168.149.1` 고정 | 핫스팟/공유기가 DHCP로 할당 (`172.20.10.x` 등) |
| **인터넷 연결** | 불가 (오프라인) | 가능 (OpenAI/Anthropic 등 외부 LLM API 호출 가능) |
| **호스트명(mDNS)** | `raspberrypi.local` | 로봇별로 `masterpi1.local`, `masterpi2.local` 변경 권장 |

### 1.2 프로토콜별 접속 포트
- **SSH (터미널 제어)**: TCP Port `22` (`ssh pi@<IP_or_mDNS>`)
- **VNC (GUI 데스크톱 스트리밍)**: TCP Port `5900` (`vnc://<IP_or_mDNS>:5900` 또는 RealVNC)
- **카메라 HTTP 스트림 (MJPEG)**: TCP Port `8080` (기본 비전 서버 구동 시)
- **Robot REST/WebSocket API (자체 구축 시)**: TCP Port `8000` 또는 `5000`

---

## 2. 하드웨어 인터페이스 및 제어 원리

MasterPi는 라즈베리파이 4B와 Hiwonder 전용 다기능 확장 보드(Raspberry Pi Expansion Board)가 GPIO 40핀 헤더로 결합되어 동작합니다.

```text
               ┌────────────────────────────────────────────────────────┐
               │              Raspberry Pi 4B (Linux / Python)          │
               └───┬───────────────────┬────────────────────┬───────────┘
                   │ I2C Bus           │ UART (/dev/ttyAMA0)│ CSI/USB
                   ▼                   ▼                    ▼
        ┌─────────────────────┐ ┌──────────────────┐ ┌───────────────┐
        │ I2C 모터 드라이버   │ │ 직렬 버스 서보 제어│ │ HD Camera     │
        │ (메카넘 휠 4채널 DC)│ │ (로봇팔 5/6채널)  │ │ (OpenCV 캡처) │
        └──────────┬──────────┘ └────────┬─────────┘ └───────────────┘
                   │                     │
          4× 메카넘 휠 모터       5-DOF 로봇팔 + 그리퍼
```

### 2.1 메카넘 휠 (Mecanum Chassis) 구동 메커니즘
- **물리 구조**: 45도 각도의 롤러가 장착된 4개의 DC 모터 + 엔코더
- **제어 방식**: 라즈베리파이의 **I2C 통신**을 통해 확장 보드의 모터 드라이버 IC에 PWM 및 방향 신호 전송
- **기구학 공식 (Omnidirectional Velocity Kinematics)**:
  - 전진/후진: 4개 바퀴가 동일 방향 회전
  - 좌/우 평행이동(Strafing): 대각선 바퀴 쌍이 서로 반대 방향으로 회전
  - 제자리 회전(Yaw): 좌측 바퀴와 우측 바퀴가 서로 반대 방향으로 회전

### 2.2 직렬 버스 서보모터 (Serial Bus Servos) 제어
- **물리 구조**: 로봇팔 관절 및 엔드이펙터(그리퍼)에 직렬 버스 서보(ID 1~6)가 데이지 체인(Daisy-chain)으로 연결
- **제어 방식**: 라즈베리파이의 **하드웨어 시리얼 포트(`/dev/ttyAMA0` 또는 `/dev/ttyUSB0`)**를 통해 패킷 송수신
- **통신 패킷 프로토콜**:
  `[0x55, 0x55, ID, Length, Instruction, Parameters..., Checksum]`
  - 관절 각도 제어(목표 펄스 `0~1000`, 이동 시간 `ms`)
  - 현재 관절 각도, 전압, 온도 피드백 읽기 가능

---

## 3. Python SDK 및 핵심 제어 API 명세

MasterPi 환경(기본 경로 `/home/pi/MasterPi/` 또는 패키지 `HiwonderSDK`)에서 제공하는 핵심 API 활용법입니다.

### 3.1 메카넘 섀시 주행 API (`HiwonderSDK.mecanum`)

```python
from HiwonderSDK.mecanum import MecanumChassis

chassis = MecanumChassis()

# 1. 속도와 방향 벡터로 주행
# set_velocity(speed, direction_angle, yaw_rate)
# speed: 0 ~ 100 (이동 속도)
# direction_angle: 0 (전진), 90 (우측 이동), 180 (후진), 270 (좌측 이동)
# yaw_rate: -100 ~ 100 (회전 속도, 양수=시계방향, 음수=반시계방향)

chassis.set_velocity(50, 0, 0)     # 50 속도로 직진
chassis.set_velocity(50, 90, 0)    # 50 속도로 우측 게걸음(평행이동)
chassis.set_velocity(0, 0, 30)     # 제자리 시계방향 회전
chassis.set_velocity(0, 0, 0)      # 정지
```

### 3.2 로봇팔 역기구학 API (`kinematics.arm_move_ik` / `ArmIK`)

```python
from kinematics.arm_move_ik import ArmIK

ak = ArmIK()

# 3차원 데카르트 좌표 (X, Y, Z) cm 단위로 로봇팔 끝점 이동
# setPitchRangeMoving((x, y, z), pitch, pitch_range, move_time_ms)
# pitch: 그리퍼 각도 (수평 기준 지면과 이루는 각도, 예: -90도는 수직 아래 보기)
result = ak.setPitchRangeMoving((0, 15, 10), -60, (-90, 0), 1500)
if result:
    print(f"역기구학 계산 성공: 서보 각도 {result}")
else:
    print("도달할 수 없는 좌표 (Out of Workspace)")
```

### 3.3 로우레벨 보드 및 서보/그리퍼 제어 (`HiwonderSDK.Board`)

```python
import HiwonderSDK.Board as Board
import time

# 1. 그리퍼 서보 제어 (ID: 1번 서보가 그리퍼)
# Board.setBusServoPulse(servo_id, pulse, run_time_ms)
# pulse 범위: 500(열림) ~ 150~200(닫힘/파지) - 모델 보정값에 따름
Board.setBusServoPulse(1, 500, 500)   # 그리퍼 열기 (500ms 동안)
time.sleep(0.5)
Board.setBusServoPulse(1, 200, 500)   # 그리퍼 닫기 (물체 파지)

# 2. RGB LED 제어 (상태 표시용)
# UGRP 실물 제어에서는 가청 버저 피드백을 사용하지 않는다.
Board.setRGB(255, 0, 0)              # 빨간색 LED (에러/차단 상태 표시)
```

---

## 4. 다중 로봇 통신 및 LLM 연동 아키텍처

UGRP 연구의 핵심인 **"동등한 LLM 에이전트 간 분산 통신"**을 실물에 적용하는 구체적인 소프트웨어 파이프라인입니다.

```text
┌───────────────────────── MasterPi #1 ─────────────────────────┐
│                                                               │
│  [Camera/Sensors] ──> [Observer] ──> (텍스트 관측 생성)       │
│                                              │                │
│  [P2P Inbox]      ───────────────────────────┤                │
│                                              ▼                │
│                                      [LLM Reasoning Engine]  │
│                                      (OpenAI/Anthropic API)   │
│                                              │                │
│                                 (Structured JSON Output)      │
│                                              │                │
│                       ┌──────────────────────┴──────────────┐ │
│                       ▼                                     ▼ │
│              [Speech / Outbox]                      [Action Runner]   │
│                       │                                     │ │
└───────────────────────┼─────────────────────────────────────┼─┘
                        │ MQTT / UDP JSON                     ▼
                        ▼                              [Mecanum / ArmIK]
┌───────────────────────── MasterPi #2 ─────────────────────────┐
│  [P2P Inbox] <────────┘                                       │
│                                                               │
└───────────────────────────────────────────────────────────────┘
```

### 4.1 P2P 메시지 프로토콜 설계 (MQTT or ZeroMQ)
- **브로커리스 P2P (ZeroMQ / WebSocket)** 또는 **경량 로컬 브로커 (Eclipse Mosquitto MQTT)** 활용
- **토픽(Topic) 구조**:
  - `robot/broadcast`: 전체 로봇 공통 채널 (파트너 탐색, 위험 알림)
  - `robot/masterpi1/inbox`: 로봇 #1 전용 수신 채널
  - `robot/masterpi2/inbox`: 로봇 #2 전용 수신 채널

### 4.2 메시지 교환 JSON 스키마 (표준 포맷)
```json
{
  "sender_id": "masterpi_1",
  "receiver_id": "masterpi_2",
  "timestamp": 1755148000.123,
  "msg_type": "REQUEST_PARTNER",
  "speech": "box_large 물체가 너무 무거워 혼자 들 수 없습니다. 같이 이송할 파트너가 필요합니다.",
  "intent": "request_partner",
  "payload": {
    "target_object": "box_large",
    "object_pos": [1.2, 0.4],
    "destination": "goal_zone_A"
  }
}
```

### 4.3 LLM 연동 방식 비교 및 권장안

| 방식 | 구조 | 장점 | 단점 | 적용성 |
|---|---|---|---|---|
| **A. 로컬 Pi 직접 호출** | 라즈베리파이 내부 Python이 `requests`/`openai`로 클라우드 API 호출 | 분산 구조 완벽 구현 (중앙 PC 불필요) | Pi의 Wi-Fi 연결 필수, API 지연(1~2s) 발생 | **1순위 (추천)** |
| **B. 워크스테이션 브리지** | 메인 PC에서 2개 LLM 프로세스 구동 후 로봇에 gRPC/ZeroMQ로 액션만 하달 | 디버깅 편리, 로깅 집중 관리 용이 | 실물 분산성과 약간의 괴리 발생 | 프로토타입 개발용 |
| **C. On-device SLM** | Pi 4B 내부에서 Llama.cpp / Ollama 직접 구동 | 완전 오프라인 독립 동작 | Pi 4B(ARM Cortex-A72)에서 토큰당 1~3초로 극도로 느림 | 현실적으로 불인정 |

---

## 5. 단계별 개발/검증 기술 체크리스트

1. **Step 1 (네트워크 & 드라이버 검증)**:
   - 라즈베리파이 터미널에서 Python 콘솔 실행 후 `chassis.set_velocity(30, 0, 0)` 실행 시 모터 구동 확인
   - `Board.setBusServoPulse(1, 500, 500)` 실행 시 그리퍼 개폐 확인
2. **Step 2 (비전 & 좌표 변환)**:
   - OpenCV로 카메라 프레임 취득 및 ArUco 마커 상대 거리 $(x, y)$ 계산
3. **Step 3 (단일 로봇 FSM 액션 래퍼)**:
   - `navigate_to(x, y)`, `pick(object_id)`, `deliver()` 고수준 함수화
4. **Step 4 (2대 로봇 간 P2P 통신)**:
   - Pi #1에서 메시지 발행 시 Pi #2의 콜백 함수에서 100ms 이내 수신 확인
5. **Step 5 (LLM 프롬프트 & Structured Output)**:
   - `Pydantic` 또는 OpenAI Function Calling / Json Mode로 하드웨어 실행 함수와 매핑
