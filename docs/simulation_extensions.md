# 독립 파일로 환경·제어기·action 확장하기

실험 폴더를 복사한 뒤 그 안의 파일만 편집한다. 등록 목록이나 시뮬레이터 코어를 수정할 필요가 없다. 설치는 [로컬 실행 안내](local_simulation.md)를 따른다. Mac과 Linux의 명령은 같다.

```bash
bash scripts/open_simulation.command new outputs/my-experiment
bash scripts/open_simulation.command inspect outputs/my-experiment/config.json
bash scripts/open_simulation.command run outputs/my-experiment/config.json --paused
# Space로 시작. 2 SIM초 후 종료되며, 길게 보려면 --sim-seconds 30을 추가한다.

# 파일을 교체하는 대신 같은 파일의 다른 factory를 선택할 수도 있다.
bash scripts/open_simulation.command run outputs/my-experiment/config.json \
  --controller r1=controller.py:create_idle_controller
```

| 생성 파일 | 바꾸는 내용 |
|---|---|
| `config.json` | 기본 레이아웃·seed·초기 배치·실행 시간·확장 선택·인자 |
| `scene.py` | 고정 장애물/경사면·움직이는 물체 추가 |
| `controller.py` | 로봇별 `act(observation)` 정책 및 상태 |
| `actions.py` | 직접 이름 붙인 명령을 기존 구동 명령으로 변환 |

`new`는 기존 폴더 덮어쓰기를 거부한다. `outputs/`는 Git에서 제외되므로 공유할 실험은 추후 자신의 작업 브랜치에 `examples/` 또는 별도 연구 패키지로 옮겨 커밋한다. 기본 예제는 짧은 전진/정지 동작을 보여주는 고정 정책이며 자율 주행 정책은 아니다.

## 환경

`scene.builder: "scene.py:build_scene"` 함수는 `build_scene(*, seed, params) -> list[dict]` 계약이다. `scene.params`는 이 함수에만 전달한다. 반환 목록과 `scene.objects`의 직접 선언 목록을 합쳐 검증한 뒤 기존 MJCF에 추가하고 물리 모델을 한 번 컴파일한다. 예제의 `barrier_half_width_m`을 바꾸면 장애물 폭이 바뀐다. seed 기반 변형에는 `random.Random(seed)`를 쓴다.

```python
def build_scene(*, seed, params):
    return [
        {"name": "ramp", "shape": "box", "xyz_m": [0, 1, .08],
         "size_m": [.4, .2, .03], "euler_deg": [0, 10, 0]},
        {"name": "ball", "shape": "sphere", "xyz_m": [.6, 1, .08],
         "size_m": [.05], "dynamic": True, "mass_kg": .08}
    ]
```

형상은 `box`, `sphere`, `cylinder`다. `size_m`은 MuJoCo의 **반 길이** 규칙으로 box는 `[반폭, 반깊이, 반높이]`, sphere는 `[반지름]`, cylinder는 `[반지름, 반높이]`다. `xyz_m`은 중심 좌표(m), `euler_deg`는 고정 축 x/y/z 회전(도), `mass_kg`은 kg이다. `dynamic: true`면 free joint를 가진다. `rgba`와 `friction: [미끄럼, 비틀림, 구름]`을 지정할 수 있다. 나머지 기본값은 `inspect`로 확인한다(생성기 반환 형상은 실행 후 `extensions.json`).

이름은 영문자로 시작하는 영문/숫자/밑줄, 최대 64자이며 중복을 거부한다. 엔진에는 `user__<name>`으로 들어간다. 충돌과 중력이 적용되며 reset 때 원래 배치로 돌아간다. 경사면의 실제 주행/운반 성능과 배치 충돌은 별도 검증한다. 추가 물체는 자동으로 기존 화물 임무의 성공 판정 대상이 되지 않는다. 기존 로봇·카메라·물리 설정·weld OFF는 유지한다.

## 제어기

`controllers`는 `r1`/`r2`/`r3`별 사전이다. 로봇마다 다른 파일·인자를 쓸 수 있고, 없는 로봇에는 자동 제어기를 붙이지 않는다.

```json
"controllers": {
  "r1": {"factory": "controller.py:create_controller", "period_s": 0.2, "params": {"power": 0.08}},
  "r2": {"factory": "other_policy.py:create_controller", "period_s": 0.5, "params": {}}
}
```

factory 계약은 `create_controller(*, robot_id, seed, params) -> object`다. 반환 객체의 `act(observation)`이 명령 dict 하나를 반환한다. 매 reset마다 새 객체를 생성하므로 상태는 모듈 전역 대신 객체에 보관한다. 호출은 SIM 0초부터 `period_s`마다 해당 시각에 도달한 첫 물리 tick에 이루어진다. 실행은 한 소유 스레드에서 동기식이며 느린 제어기는 전체 시뮬레이션의 실제 시간을 늦춘다. 비협조적인 외부 호출의 중단/timeout은 제어기에서 구현해야 한다. CLI의 wall limit은 호출을 강제 선점하지 않는다.

입력은 자기 보정 RGB JPEG(base64 `image`), 공용 top JPEG(`top_rgb.image`), 자기 발행 상태(`actuator_state`), 현재 에피소드의 자기 `command_history`, 에피소드/SIM 시간이다. 장면 좌표·측정 관절·접촉·성공 판정·다른 로봇의 명령은 전달하지 않는다. 관측 dict는 복사본이다. 공장 함수에도 전체 환경 설정이나 세계 객체를 전달하지 않는다. 자기 명령은 실제 이동 성공이나 측정 관절을 뜻하지 않는다.

제어기가 있으면 RGB 렌더링을 자동 활성화하고 CLI는 매 호출의 실제 입력·응답·오류를 `controller-decisions.jsonl`에 보존한다. 긴 실행은 RGB 로그가 커질 수 있다. 외부 API를 호출하는 제어기의 비용·모델 호출 수는 자동 계측하지 않으며 `controller_calls`와 구분한다. Linux headless RGB는 `MUJOCO_GL=osmesa`로 실행할 수 있다.

자동 제어기에 배정한 로봇은 `actions` 스케줄이나 직접 `sim.apply()`로 동시에 조작할 수 없다. 다른 로봇의 스케줄은 함께 사용할 수 있다. 제어기를 해제하려면 새 설정에서 해당 로봇의 `controllers` 항목을 지운다.

## 사용자 action

`action_plugins: {"nudge": "actions.py:nudge"}`로 등록하면 `{"kind":"nudge", "params":{"power":0.1}}`을 스케줄·제어기·직접 API에서 쓸 수 있다. 함수는 `nudge(params) -> raw_command`이며 세계나 관측을 받지 않는다.

```python
def nudge(params):
    return {"kind": "drive", "forward": params.get("power", .08),
            "turn": 0, "duration_s": .2}
```

반환 명령은 기존 drive/mecanum/look/arm/wait 계약으로 다시 검증하고 실제 actuator에 적용한다. 기존 이름 덮어쓰기, 다른 사용자 action으로 재귀 호출, 명령 목록 반환은 허용하지 않는다. 여러 단계 동작은 제어기의 상태/다음 SIM 호출로 구현해 물리 진행 시각을 명확히 한다. 이는 기존 액추에이터의 새 동작 조합을 위한 확장이다. 새 모터·센서·로봇 기종은 별도 엔진 개발이 필요하다.

## 경로·기록·Python API

모든 `file.py:callable`은 **설정 JSON이 있는 폴더 기준**이다. 절대 경로도 가능하며 CLI `--controller`, `--scene-builder`도 같은 규칙이다. `inspect`는 Python 파일을 실행하지 않고 설정 구조만 검사한다. Python 확장은 신뢰하는 로컬 코드이며 보안 sandbox가 아니다. 제어 입력 경계는 제공 API의 계약이지 임의 Python 코드의 부정 접근을 막는 격리는 아니다.

진입 파일은 하나의 독립 Python 파일로 작성한다. 다른 라이브러리를 쓸 때는 실행 환경에 설치 가능한 패키지로 관리한다(형제 파일의 상대 import를 자동 연결하지 않는다). 실행 결과에는 진입 파일의 실제 바이트·SHA256, config·최종 형상·컴파일 모델·물리 설정·발행 명령·별도 평가가 저장된다. 간접 import한 패키지나 외부 모델/파일까지 자동 복사하지 않으므로 그 버전·출처는 연구자가 함께 기록한다.

```python
from pathlib import Path
from sim.session_config import load_config
from sim.session import Simulation

path = Path("outputs/my-experiment/config.json").resolve()
with Simulation(load_config(path), base_dir=path.parent,
                decision_sink=lambda record: print(record["at_s"])) as sim:
    sim.step(100)  # 이 안에서 제어기와 모든 로봇의 물리가 함께 진행
```

직접 API의 기록은 `decision_sink`로 받을 수 있다. CLI는 자동 저장한다. 실행 중 파일 변경은 hot reload하지 않으므로 수정 후 다시 실행한다. `scripts/check_simulation_extensions.py`는 복사본의 장애물 폭 변경·동적 물체 reset·제어기 교체·실제 물리 차이·RGB 입력 보존을 검사한다.
