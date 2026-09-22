# 로컬 시뮬레이션: 설정 파일 · CLI · Python API

같은 설정을 터미널에서 실행하고 MuJoCo 기본 3D 창으로 확인한다. Python에서는 `Simulation`을 불러와 관측·명령·물리 스텝을 직접 제어한다. 별도 웹 서버나 모델 계정은 필요 없다.

## 설치와 첫 실행

Python 3.12를 사용한다. 기존 Mac 환경 `.venv-sim-worker-mac`을 재사용하며, 새 clone에서는 다음과 같이 설치한다.

```bash
python3.12 -m venv .venv-dev
.venv-dev/bin/python -m pip install -r requirements-sim.txt
.venv-dev/bin/python -m pip check
bash scripts/open_simulation.command
```

Ubuntu 24.04는 먼저 [설치 안내](ubuntu_quickstart.md)를 따른다. 창을 띄우려면 데스크톱의 X11/Wayland 그래픽 세션이 필요하다. WSL은 WSLg가 필요하며 각 PC의 그래픽 지원은 따로 확인한다. 서버에서는 `--headless`를 사용한다. Linux에서 RGB를 저장하는 headless 실행에는 `libosmesa6`와 `MUJOCO_GL=osmesa`를 사용할 수 있다. 네이티브 창에는 기본 GLFW를 사용한다.

실행기는 저장소 또는 기본 worktree의 `.venv-sim-worker-mac`, `.venv-dev`를 찾는다. 외부 환경은 `UGRP_SIM_PYTHON=/absolute/path/bin/python`으로 지정한다. macOS의 네이티브 창은 MuJoCo가 제공하는 **`mjpython`**으로 실행해야 하며 실행기가 자동 선택한다. 환경을 활성화했다면 `mjpython -m scripts.sim_cli ...`(Mac 창), `python -m scripts.sim_cli ...`(Linux 또는 headless)를 직접 써도 된다.

## 터미널에서 구성하기

저장소 루트에서 실행한다. 인자 없는 실행은 `configs/simulation/local.json`을 읽는다.

```bash
# 설정 생성 → 편집 → 오류/기본값 확인
bash scripts/open_simulation.command init my-scene.json --layout camera_team --seed 42
bash scripts/open_simulation.command inspect my-scene.json
bash scripts/open_simulation.command layouts

# MuJoCo 기본 창. 마우스로 회전/이동/확대하며 물리를 관찰한다.
bash scripts/open_simulation.command run my-scene.json --paused

# 같은 설정으로 무화면 실행; 모델 호출 없음
bash scripts/open_simulation.command run my-scene.json --headless --sim-seconds 10

# 예제 명령 스케줄과 카메라 입력 저장
bash scripts/open_simulation.command run configs/simulation/drive.json --capture

# seed/시간을 코드 편집 없이 변경하고 결과 폴더 지정
bash scripts/open_simulation.command run my-scene.json --seed 43 --sim-seconds 60 --output outputs/my-run-43
```

`init`과 결과 저장은 기존 파일/폴더를 덮어쓰지 않는다. 설정 오타·잘못된 명령은 실행 전에 거부한다. 화물 ID가 실제 장면에 존재하는지는 세계 구성 때 확인한다. `--headless`는 가능한 속도로 진행하고, 창을 사용하는 실행은 `run.realtime_factor`에 맞춰 속도를 제한한다. 컴퓨터가 느리면 실제 시간보다 느려질 수 있다.

MuJoCo 창에서 **Space**는 일시정지/재개, **N**은 정지 중 물리 한 스텝, **R**은 동일 설정으로 초기화 후 정지다. 정지는 물리 시간을 멈추므로 남은 명령도 재개 시 이어진다. 창 닫기 또는 터미널 Ctrl-C로 종료한다. 기본 SIM 30초 또는 실제 30분에 도달해도 종료한다. `--sim-seconds`, `--wall-seconds`로 변경할 수 있고 실제 시간 제한은 일시정지/초기화해도 초기화되지 않는다.

관찰 카메라는 `--camera cctv_top`, `--camera cctv_warehouse`, `--camera r1__robot_cam` 등으로 선택한다. 창의 Rendering 카메라 선택도 사용할 수 있다. 자유 시점과 물체 드래그는 사람이 장면을 살펴보는 도구다. GUI에서 물리를 조작한 실행은 무인 평가와 구분한다.

## 설정 계약 (version 1)

필수 항목은 `version`이며 나머지는 `inspect`로 확인할 수 있는 기본값을 사용한다.

| 항목 | 의미 |
|---|---|
| `scene.layout` | 기존 `standard`, `mixed`, `arena`, `camera_team` 장면 생성기 |
| `scene.seed` | 정적 장면과 초기 배치 seed |
| `scene.cargo_ids` | `null`이면 전체 화물, 배열이면 선택한 화물만 구성. 실제 ID는 실행의 `physics.json` 참고 |
| `scene.robots` | 초기화 시에만 적용할 로봇별 `xyz_m: [x,y,z]`, `yaw_deg` |
| `camera.width/height` | 자기 RGB 출력 해상도. 공용 top은 기존 관찰자 최소 해상도를 유지 |
| `control.allow_reverse/allow_mecanum` | 허용할 저수준 명령 범위 |
| `run` | `sim_seconds`, `wall_seconds`, `realtime_factor` |
| `scene.objects/builder/params` | 추가 형상 목록 또는 독립 Python 장면 생성기와 인자 |
| `controllers` | 로봇별 독립 Python 제어기 factory·period_s·params |
| `action_plugins` | 사용자 action 이름 → 독립 Python 변환 함수 |
| `actions` | `{at_s, robot, command}` 배열. 같은 시각은 파일 순서로 적용 |

`configs/simulation/drive.json`은 3초 명령 스케줄 예제다. 다음은 직접 배치하고 한 로봇을 움직이는 설정 예시다. 배치의 충돌 여부와 적합성은 연구자가 창에서 확인한다.

```json
{
  "version": 1,
  "scene": {
    "layout": "camera_team", "seed": 41,
    "robots": {"r1": {"xyz_m": [-0.5, 0.3, 0.08], "yaw_deg": 45}}
  },
  "run": {"sim_seconds": 5},
  "actions": [
    {"at_s": 0, "robot": "r1", "command": {"kind": "drive", "forward": 0.12, "turn": 0, "duration_s": 0.5}}
  ]
}
```

로봇은 현재 기존 MasterPi 3대(r1/r2/r3)다. 장애물·경사면·동적 물체, 제어기, 사용자 action은 실험 폴더의 파일로 추가한다. `bash scripts/open_simulation.command new outputs/my-experiment`로 시작하며 [확장 안내](simulation_extensions.md)를 따른다. 새 로봇 기종·로봇 수·관절/센서 자체를 바꾸는 것은 여전히 엔진 개발 범위다.

## 연구 코드에 붙이기

```python
from sim.session import Simulation
from sim.session_config import load_config

config = load_config("configs/simulation/local.json")
with Simulation(config, render=True) as sim:
    sim.reset()                         # 같은 설정 재현; seed 변경은 새 Simulation
    observation = sim.observe("r1")     # 자기 JPEG + 공용 top JPEG + 자기 발행 명령
    sim.apply("r1", {"kind": "drive", "forward": .12, "turn": 0, "duration_s": .5})
    sim.step(100)                       # 정확히 100개의 엔진 tick
    next_observation = sim.observe("r1")
```

`sim.timestep`은 한 tick의 초, `sim.time`은 reset의 초기 안정화 시간을 제외한 에피소드 SIM 시간이다. `step()`은 매 tick 명령 만료와 서보 보간을 처리하고 전체 세 로봇의 물리를 한 번 진행한다. `apply()`는 시간을 진행하지 않고 발행 ACK만 반환한다. 여러 로봇의 명령을 `apply()`한 뒤 `step()`하면 같은 물리 세계에서 함께 실행된다. 설정의 스케줄과 직접 `apply()`를 동시에 쓰면 함께 적용되므로 자체 제어기는 보통 `actions: []`를 사용한다.

명령은 기존 `CameraRobotPort` 계약을 그대로 쓴다. `drive`(forward/turn/duration_s), `mecanum`(forward/left/turn/duration_s), `look`(pan_pulse), `arm`(servo_id/pulse), `wait`가 있다. 바퀴 입력은 속도 측정값이 아닌 정규화 명령이다. drive forward 범위는 -0.05~0.15, turn은 -0.2~0.2, 지속은 0~1초다. mecanum left는 -0.10~0.10, turn은 -0.15~0.15다. 서보 pulse는 500~2500이며 arm servo_id는 1/3/4/5다. `wait`는 바퀴 정지 명령이며 진행 중인 팔 명령을 취소하지 않는다.

`sim.launch_viewer()`로 같은 세계에 기본 창을 붙이고 `sim.sync_viewer()`를 호출해 갱신한다. reset은 model/data 객체를 유지해 창의 참조가 유효하다. [실행 가능한 Python 예제](../examples/simulation_session.py)는 두 에피소드를 실행한다. `python -m examples.simulation_session`, Mac에서 창을 붙이려면 `mjpython -m examples.simulation_session --viewer`를 사용한다. 세션은 한 소유 스레드에서 순차 사용하며 병렬 제어기는 명령을 소유 스레드에 전달한다.

`observe()`는 `render=True`일 때 기존 보정·왜곡을 적용한 자기 RGB와 공용 top RGB를 반환한다. 네이티브 고정 카메라 미리보기는 MuJoCo 투영이며 이 보정 RGB와 픽셀 단위로 동일하지 않다. 실제 모델 입력은 `observe()` 결과를 사용한다. 초기 위치·평가 좌표·관절 측정·접촉·성공 판정은 관측에 포함하지 않는다. `evaluation_state()`는 별도 평가 전용이며 제어기에 전달하지 않는다. 창을 사용하는 사람이 물체를 조작해도 이 경계가 자동으로 연구 프로토콜을 보장하는 것은 아니다.

## 기록과 코드 위치

CLI는 매번 `outputs/sim-<날짜>-<ID>/`를 만든다. `config.json`은 적용된 전체 설정, `session.json`은 소스 SHA/dirty 상태와 실행 환경, `model.mjb`는 컴파일된 모델, `physics.json`은 물리 설정·화물 목록, `commands.jsonl`은 초기화와 실제 발행 명령, `*-evaluation.json`은 별도 정답 진단, `result.json`은 종료 이유·시간·파일 해시다. `--capture`는 시작/종료의 실제 입력 RGB와 관측 JSON을 추가한다. `extensions.json`과 `extensions/`에는 실행한 확장 진입 파일·해시·최종 추가 형상을 보관한다. 제어기를 쓰면 `controller-decisions.jsonl`에 매 호출의 실제 RGB 입력·응답·발행 명령을 저장한다. 연속 영상 녹화나 전체 상태 replay는 아직 제공하지 않는다.

| 파일 | 책임 |
|---|---|
| `sim/session_config.py` | 설정 버전·기본값·검증 |
| `sim/session.py` | 세계 수명·reset/step·명령·관측·native viewer 연결 |
| `sim/camera_robot_port.py` | 기존 로봇별 RGB/발행 명령 경계 |
| `scripts/sim_cli.py` | new/init/inspect/layouts/run과 실행 기록 |
| `scripts/open_simulation.command` | 기존 Python 환경 선택과 프로세스 세션 관리 |
| `scripts/check_simulation.py` | 실제 물리·초기화·카메라·설정 통합 검사 |

명령 스케줄은 고정 입력이며 자율 운반·LLM 협력 성공을 뜻하지 않는다. 기존 하네스는 별도 경로로 유지되며 이 API로 전부 이관된 것은 아니다. 완료된 연구 결과 비교는 [TensorBoard](tensorboard.md) 절차를 따른다. 실행기는 개별 PID 기반 세션을 만들며 Ctrl-C/창 종료/시간 제한 시 자신이 시작한 세계와 창을 닫는다.

검증 명령:

```bash
python -m pytest -q tests/test_simulation_session.py tests/test_camera_robot_port.py
python -m scripts.check_simulation --output outputs/simulation-check-NEW
```

MuJoCo passive viewer의 스레드·macOS 실행 규칙은 [공식 Python 문서](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)를 따른다.

## 플랫폼 검증

| 환경 | 검사 |
|---|---|
| macOS arm64 · Python 3.12 · MuJoCo 3.12 | native 창 실행/유한 종료, headless, 보정 RGB, 설정/API |
| Ubuntu 24.04 x64 · Python 3.12 · MuJoCo 3.12 | Xvfb에서 native 창 3회 연속 실행/종료, OSMesa RGB, 설정/API |

두 OS에서 같은 JSON과 명령을 사용한다. Linux CI의 가상 디스플레이 검증은 각 PC의 그래픽 드라이버 확인을 대체하지 않는다. Mac 실제 창의 마우스/키보드 자동화는 도구 접근 시간 초과로 미확인이다. [검증 원본과 실패 기록](../experiments/2026-09-22-native-simulation/README.md)을 참조한다.
