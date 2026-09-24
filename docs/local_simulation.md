# 로컬 시뮬레이션: 설정 파일 · CLI · Python API

같은 설정을 터미널에서 실행하고 MuJoCo 기본 3D 창으로 확인한다. Python에서는 `Simulation`을 불러와 관측·명령·물리 스텝을 직접 제어한다. 기본 연구 실행은 기존 `run_dispatch_e2e --executor skills`를 사용한다. 자연어 지시를 받은 세 로봇이 plan에 합의하고, 각자의 프로그램으로 기존 RGB 스킬을 실행한다. 수동·설정 실행에는 모델 계정이 필요 없다.

**실행·버전·결과 관리는 [표준 시뮬레이션 관리](simulation_management.md)로 통일한다.** `workflow list/plan/run/runs/show`에서 연구별 실행을 선택하고 이력을 확인한다. 아래 기본 `run/console/dispatch`도 공통 실행 기록에 연결된다.

## 터미널 간편 메뉴

설치된 환경에서 저장소 루트의 다음 명령을 실행하면 메뉴가 열린다. **실행 방식·맵·속도**(LLM 출하는 재생 속도, 장면 미리보기는 관찰 속도)를 고르고, LLM 협업 출하에서는 **모델 ID·자연어 지시**도 입력한다. 실제 세계는 MuJoCo 기본 창에서 열린다. 자세한 물리·접촉·카메라 설정은 아래 CLI와 설정 파일에서 관리한다.

```bash
bash scripts/open_simulation.command
```

선택 메뉴는 번호를 표시한다. 실행 방식은 `1` LLM 공동 계획, `2` 장면 미리보기, `3` 기타 실행이다. 출하 지도 5종은 번호로 고른다. 장면 미리보기는 먼저 번호로 그룹을 고른 다음 그룹 안의 장면 번호를 고른다. 속도는 `1`=0.5×, `2`=1×(기본), `3`=2×, `4`=4×다. 모델은 `1`=현재 기본 모델, `2`=모델 ID 직접 입력이다. Enter는 각 단계의 기본값을 고른다. 자연어 지시는 텍스트로 입력하며, 장면 ID 직접 입력과 `/검색어`도 계속 사용할 수 있다.

**LLM 협업 출하**는 기존 빔·상자 출하 임무와 지원 지도 5종에 한정된다. 자연어 지시를 비우면 기본 출하 임무가 전달된다. 지시는 실행 전 계획에 전달되며, 실행 도중 임의의 새 물체 작업을 추가하지 않는다. 모델 프록시가 준비되지 않았다면 모델 요청은 실패하며 메뉴가 로그인하거나 프록시를 시작하지 않는다. **지도 장면 보기**는 등록된 장면을 MuJoCo 창에서 정지 상태로 확인한다. ACT 지도도 볼 수 있지만 LLM 제어나 운반 평가가 아니다. 새 지도 구조를 생성하려면 지도 suite/설정 파일과 표준 workflow를 사용한다. 저장 plan 재생·수동 명령·설정 파일 실행은 메뉴의 **기타 실행**에서 계속 사용할 수 있다.

MuJoCo 창에서 Space는 재개/일시정지, R은 초기화다. 창을 닫거나 시작한 터미널에서 Ctrl-C를 누르면 실행을 종료한다. 메뉴 선택은 기존 `run`/`dispatch`와 같은 실행 기록을 사용한다.

## 설치와 첫 실행

Python 3.12를 사용한다. 기존 Mac 환경 `.venv-sim-worker-mac`을 재사용하며, 새 clone에서는 다음과 같이 설치한다.

```bash
python3.12 -m venv .venv-dev
.venv-dev/bin/python -m pip install -r requirements-sim.txt
.venv-dev/bin/python -m pip check
bash scripts/open_simulation.command run configs/simulation/drive.json --paused --capture
```

첫 명령은 모델 계정 없이 예제 명령 스케줄을 MuJoCo 기본 창에 연다. **Space**로 재개하면 3초의 SIM 시간 동안 로봇 명령을 실행하고 종료한다. `--capture`는 시작·종료의 로봇 RGB를 결과 폴더에 저장한다. 터미널에 표시된 `result.json` 경로와 `bash scripts/open_simulation.command workflow runs`의 실행 기록을 확인한다. 창을 열었다는 사실과 예제 명령 완료는 연구용 운반 성공이 아니다. 인자 없이 실행하면 선택 메뉴가 열리며 기본 선택 1은 모델 프록시가 필요한 공동 계획이다.

Ubuntu 24.04는 먼저 [설치 안내](ubuntu_quickstart.md)를 따른다. 창을 띄우려면 데스크톱의 X11/Wayland 그래픽 세션이 필요하다. WSL은 WSLg가 필요하며 각 PC의 그래픽 지원은 따로 확인한다. 서버에서는 `--headless`를 사용한다. Linux에서 RGB를 저장하는 headless 실행에는 `libosmesa6`와 `MUJOCO_GL=osmesa`를 사용할 수 있다. 네이티브 창에는 기본 GLFW를 사용한다.

실행기는 저장소 또는 기본 worktree의 `.venv-sim-worker-mac`, `.venv-dev`를 찾는다. 외부 환경은 `UGRP_SIM_PYTHON=/absolute/path/bin/python`으로 지정한다. macOS의 네이티브 창은 MuJoCo가 제공하는 **`mjpython`**으로 실행해야 하며 실행기가 자동 선택한다. 환경을 활성화했다면 `mjpython -m scripts.sim_cli ...`(Mac 창), `python -m scripts.sim_cli ...`(Linux 또는 headless)를 직접 써도 된다.

출하 실행의 실험 옵션 `dispatch --realtime-control`은 일반 Python에서 물리 owner를 실행하고, 별도 `mjpython` 프로세스에 MuJoCo 창을 연다. 기존 실행기에 이 옵션을 추가하면 된다. RGB의 실제 촬영 SIM 시각과 판단 지연을 저장하고, 오래된 판단은 정지 처리한다. 녹화가 밀리면 이전 프레임을 표시하고 화면에 반복 표시 및 `execution.frames.json`에 횟수를 남기므로 영상 길이를 줄여 빠르게 보이게 하지 않는다. 이 옵션은 open RGB 경로의 별도 검증 후보이며 ACT·회전 운반의 검증을 대신하지 않는다. `result.json`의 `timing`은 물리 진행/실제 시간과 준비·정리 시간을 나눠 기록하며 최종 `wall_s`는 정리까지 포함한다. 실행 구조와 기록 해석은 [실시간 출하 실행](realtime_dispatch.md), 상세 근거는 [실시간 실행 검증](../experiments/2026-09-23-realtime-dispatch/README.md)을 따른다.

## 터미널에서 구성하기

저장소 루트에서 실행한다. 인자 없는 실행은 LLM 공동 계획·지도 장면 보기·기타 기존 실행을 선택하는 메뉴를 연다. 공동 계획의 기본 장면은 **기존 공동 출하장(dispatch/shared_crossing, seed11)**이다. 수동 기본 설정은 `configs/simulation/local.json`이다. `init`과 `new`도 같은 연구 장면을 기본으로 사용한다. [전체 구성·누락 검토](simulation_inventory.md)에 기존 자산과 연결 범위를 정리했다. 지도 준비·ACT 학습·실행 이력·TensorBoard를 연결하는 명령은 [표준 관리 절차](simulation_management.md#지도학습실행-결과를-잇는-절차)를 따른다.

```bash
# 설정 생성 → 편집 → 오류/기본값 확인
bash scripts/open_simulation.command init my-scene.json --scene dispatch/shared_crossing --seed 11
bash scripts/open_simulation.command inspect my-scene.json
bash scripts/open_simulation.command scenes
bash scripts/open_simulation.command workflows
bash scripts/open_simulation.command doctor

# 등록된 지도를 설정 파일 수정 없이 바로 관찰
bash scripts/open_simulation.command run configs/simulation/local.json \
  --scene act/train-open-1 --paused --capture

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

MuJoCo 창에서 **Space**는 일시정지/재개, **N**은 정지 중 물리 한 스텝, **R**은 동일 설정으로 초기화 후 정지다. 정지는 물리 시간을 멈추므로 남은 명령도 재개 시 이어진다. 창 닫기 또는 터미널 Ctrl-C로 종료한다. `run`은 설정의 SIM 제한(기본 30초), `console`은 기본 SIM 30분이며 실제 시간 제한은 둘 다 기본 30분이다. `--sim-seconds`, `--wall-seconds`로 변경할 수 있고 실제 시간 제한은 일시정지/초기화해도 초기화되지 않는다.

MuJoCo 패널은 표시 옵션 외에 물리·actuator 상태도 바꿀 수 있다. 패널의 초기화나
수치 불안정으로 시간이 되돌아가면 창을 닫지 않고 물리를 정지하며 안내를 표시한다.
**R로 새 에피소드를 초기화한 뒤 Space로 재개**한다. 잘못된 상태에서는 Space/N이
동작을 재개하지 않는다. R은 장면·동작 상태를 초기화하지만 사용자가 패널에서 바꾼
물리 모델 설정까지 복원하지 않으므로, 그 설정 때문에 반복되면 실행을 다시 시작한다.
`runtime-events.json`과 결과에 해당 사건을 남기며, 복구 후에도 그 실행을 정상 완료로
집계하지 않는다(종료 코드 2). headless/API는 명확한 `SimulationStateError`로 중단한다.

관찰 카메라는 `--camera cctv_top`, `--camera cctv_warehouse`, `--camera r1__robot_cam` 등으로 선택한다. 창의 Rendering 카메라 선택도 사용할 수 있다. 자유 시점과 물체 드래그는 사람이 장면을 살펴보는 도구다. GUI에서 물리를 조작한 실행은 무인 평가와 구분한다.

## 기존 공동 계획과 로봇별 실행

```bash
bash scripts/open_simulation.command
# 1 LLM 공동 계획 → 기존 RGB 스킬 / 2 장면 미리보기 / 3 기타 실행
# 1에서는 지원 출하 맵·재생 속도·계획 모델·자연어 지시를 고른다.
# 2에서는 등록된 장면을 그룹 또는 /검색어로 찾아 MuJoCo 기본 창에서 본다.
# 저장된 plan 재생·수동·설정 실행은 3에서 고른다.

# 같은 기존 실행기를 직접 선택: 자연어는 세 로봇의 계획 협상에 전달
bash scripts/open_simulation.command dispatch \
  --task '서로 역할과 순서를 합의해서 beam과 box를 dock_b로 옮겨' \
  --required-dock dock_b --variant shared_crossing --seed 11

# 자신이 저장한 plan을 명시적으로 재생: 새로운 LLM 협상 아님
bash scripts/open_simulation.command dispatch --plan-replay outputs/my-run/committed-plan.json
```

`dispatch`는 **기존** `run_dispatch_e2e.py --executor skills`의 진입점이다. 세 로봇의 자기 RGB·공용 TOP·자기 명령 이력·허용 정적 지도·동료 메시지로 계획을 협상하고, 동일 plan ID/hash에 전원 합의해야 `SkillBindings`가 로봇별 프로그램을 만든다. 담당 물체, 운반 파트너, 경로, 목적지, 선행 작업은 plan에서 가져온다. pair RGB 접근·파지·운반과 solo box 스킬을 공통 물리 시계에서 진행하며, 공동 동작·점유 자원·선행 작업의 기존 허가를 유지한다. 별도 프로세스 세 개의 실시간 분산 제어를 뜻하지 않는다.

`--live-view` 관찰 창 왼쪽 아래에서 최근 동료 메시지와 발신·수신 로봇을 볼 수 있다(계산 후 재생 창에는 대화 표시가 없다). 전체 대화는 출력 폴더의 `team/conversation.jsonl`과 터미널 `peer_delivery` 행에 남는다. 현재 새 자연어 대화는 주로 계획 협상에서 발생하며, 저장 계획 재생과 RGB 운반 중에는 새 대화가 없을 수 있다. [대화 관찰·원문 기록](communication_observer.md)을 참고한다.

자연어는 **실행 전 계획 지시**다. 현재 계약은 기존 beam 1개·box 1개, 로봇 3대, dock_a/b, north/south 경로다. 자유로운 새 작업이나 임의 맵에 필요한 스킬을 자동으로 만드는 기능은 없다. 목적지처럼 반드시 지켜야 하는 조건은 `--required-dock`으로도 지정한다. 실행 중 새 자연어 지시로 이미 승인된 plan을 바꾸는 기능은 아직 없다. 새 작업은 종료 후 다시 실행한다. 저장된 plan 재생에는 새 `--task`를 함께 넣을 수 없다.

출하 스킬에 필요한 기존 모델은 저장소의 `experiments/dispatch-skill-integration-20260917/models.zip`에서 `outputs/dispatch-models/<bundle-hash>/`로 복원한다. 기존 파일을 덮어쓰거나 새로 학습하지 않는다. 자기 모델은 `--grasp-model-dir DIR --stage-model-dir DIR`를 함께 지정한다. 이 모델의 과거 성공 범위가 임의 조건의 성공을 보장하지는 않는다.

새 계획에는 접근 가능한 기존 모델 프록시가 필요하다. `GEMINI_PROXY_URL`은 자신의 `/v1/chat/completions` 주소이며 기존 기본은 로컬 8391이다. 모델은 `--model` 또는 `UGRP_SIM_MODEL`, 기본은 `gemini-3.8-flash`다. 실행기는 계정이나 프록시 서버를 자동으로 만들지 않는다. 저장된 계획 재생은 새 모델 협상을 하지 않는다.

주요 옵션은 `--variant open|shared_crossing|north_blocked|narrow_south|rough_south`, `--seed`, `--max-wall-s`(기본 1200초), `--timeout`(요청당 기본 60초), `--planning-rounds`(합의 시도당 기본 8라운드), `--max-replans`(실행 전 적합성 재협상 기본 2회), `--max-input-tokens`다. 기존 실행기의 나머지 옵션도 전달할 수 있다. headless는 `--headless`, 재생 속도는 `--realtime-factor`로 바꾼다. 속도는 물리 timestep이나 제어기를 바꾸지 않는다.

표준 `dispatch`는 pair 판단에 쓰지 않는 카메라 촬영을 생략한다. 로봇이 실제로 받은 자기·TOP 영상과 연속 기록 영상의 조건은 유지한다. 합의한 계획이 열린 맵의 독립된 경로·자원을 사용하면 기존 병렬 운반 경로도 자동 선택한다. 선행 작업이나 공유 경로가 있으면 직렬 실행을 유지하고, 병렬 실행에서도 공용 하역 구역은 한 팀씩 사용한다. 실제 선택과 이유는 `skill-bindings.json`과 `result.json`의 `overlap_selection`에 남는다. 계획의 역할·경로·선행 조건은 자동 수정하지 않는다.

이전 촬영·자원 일정과 비교하려면 `dispatch --full-capture --serial-route`를 사용한다. 연구 실행기 `run_dispatch_e2e.py`와 `dispatch-skills` workflow의 기본값은 그대로이며 새 자동 선택은 `--auto-route-overlap`, 촬영 최적화는 `--efficient-capture`로 명시한다. 기본 병렬 진입은 봉의 운반 명령 이후이고, `--overlap-start grasp`는 파지부터 겹치는 별도 조건이다. 창의 배속, 렌더 처리량, 계획 협상 시간, 실제 동시 운반 시간은 각각 구분해 측정한다.

**2026-09-24부터 표준 `dispatch`는 계산을 먼저 끝낸 뒤 재생한다.** 창 없이 끝까지 최대 속도로 계산하고, 그동안 관찰 전용 상태(관절·mocap 위치, SIM 30Hz)를 `replay/`에 기록한다. 끝나면 MuJoCo 기본 창에서 선택한 속도로 재생한다. 성공·실패와 관계없이 기록된 구간을 재생하며 종료 코드는 원래 실행의 값이다. 재생은 저장된 위치를 운동학적으로 보여 줄 뿐 물리를 다시 계산하지 않는다. 기록은 로봇·제어기·평가의 입력이 아니며 제어 동작은 바뀌지 않는다. 재생 창은 마우스 회전/확대, **Space** 일시정지/재개, **←/→** 5초 이동, **R** 처음부터, **Q 또는 창 닫기** 종료를 지원한다. 지난 실행은 `bash scripts/open_simulation.command replay outputs/<실행 폴더> --speed 2`로 다시 볼 수 있다. `replay/replay.json`의 파일 해시가 달라지면 재생을 거부한다. `--headless`는 기록도 창도 만들지 않으며, 필요하면 `--record-replay`를 추가한다.

계산 중에 보려면 `dispatch --live-view`를 쓴다. 이전 기본 **dispatch 관찰 창**이며 마우스 회전/확대, **Space** 일시정지/재개, **Q 또는 창 닫기** 종료를 지원한다. 모델 대기 중 물리는 멈춘다. 관찰 창은 model/data의 별도 복사본을 사용하므로 패널 초기화·actuator 조작·물체 드래그가 실제 제어 세계에 전달되지 않는다. 실제 물체 조작은 아래 수동 경로를 사용한다. 종료 중 이미 진행된 모델 요청은 제한 시간 안에 끝날 때까지 기록을 회수하며, 서버 측 취소를 보장하지 않는다.

결과 폴더에는 원래 실행기가 만드는 `actor-mission.json`, `team/`의 로봇별 실제 요청/응답, `committed-plan.json`, `robot-programs.json`, `skill-bindings.json`, 발행 명령·RGB·영상·별도 평가·`result.json`이 남는다. plan 합의, 스킬 프로토콜 완료, 실제 운반 성공은 각각 다른 판정이다. 출력 폴더는 덮어쓰지 않는다. 기존 연구 실행기와 동일하게 실제 실행 전 변경 소스를 커밋해야 한다.

## 저수준 수동·설정 콘솔

```bash
bash scripts/open_simulation.command console configs/simulation/local.json --mode manual
bash scripts/open_simulation.command console configs/simulation/drive.json --mode script
```

명령은 터미널에 입력하고 MuJoCo 창에서 확인한다. 수동 모드의 `r1 앞으로 0.5초`, `r2 왼쪽 0.3초`는 고정 문법이며 왼쪽/오른쪽은 제자리 회전이다. 평행 이동은 `r1 mecanum 0 0.05 0 0.5`, 팔은 `r1 arm 1 1800`, 사용자 action은 `/raw r1 {"kind":"wait"}`처럼 입력한다. 자유로운 자연어 계획은 위의 `dispatch`를 사용한다.

`/pause`는 물리를 정지하고 `/run`으로 잇는다. `/stop` 또는 `정지`는 작업·남은 actuator 보간을 취소한다. `/reset` 또는 `초기화`는 같은 장면을 초기화한다. `/mode manual|script` 변경도 초기화한다. `/status`, `/help`, `/quit` 또는 `종료`를 쓸 수 있다. script에는 actions/controllers가 있는 설정이 필요하다. 사용자 제어기를 바꾸려면 [확장 안내](simulation_extensions.md)를 따른다.

앞서 추가했던 `--mode llm-single|llm-independent|llm-peer`는 **별도 raw action 진단**으로만 보존한다. 기본 메뉴에는 표시하지 않는다. 이 경로는 LLM이 매번 저수준 명령을 고르므로 기존 plan→skill 실행 또는 같은 실행기를 고정한 통신 비교로 취급하지 않는다. 모델 요청 한도는 `--max-calls`, `--max-rounds`, `--model-timeout`; 기록은 `console-events.jsonl`, `model-calls/`에 남는다. 이 진단의 `protocol_complete`는 항상 false이며 모델의 완료 주장은 물리 성공이 아니다.

## 설정 계약 (version 1)

필수 항목은 `version`이며 나머지는 `inspect`로 확인할 수 있는 기본값을 사용한다.

| 항목 | 의미 |
|---|---|
| `scene.layout` | `scenes`의 연구 장면 ID 또는 기존 4개 엔진 예제 |
| `scene.seed` | 엔진 초기화·배치 seed. ACT 지도의 `map_seed`는 선택 case 명세를 따름 |
| `scene.cargo_ids` | 기존 4개 엔진 예제의 화물 필터. 연구 장면은 작성된 inventory 사용 |
| `scene.map_file` | `navigation/file`·`pair_navigation/file`의 지도 JSON; 설정 폴더 기준 |
| `scene.contact_profile` | 출하장의 명시적 접촉 solver 프로필; 기본은 원래 생성기 설정 |
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

CLI는 매번 `outputs/sim-<날짜>-<ID>/`를 만든다. `config.json`은 적용된 전체 설정, `session.json`은 소스 SHA/dirty 상태와 실행 환경, `model.mjb`는 컴파일된 모델, `physics.json`은 물리 설정·실제 선택 장면의 화물 목록, `commands.jsonl`은 초기화와 실제 발행 명령, `*-evaluation.json`은 별도 정답 진단, `result.json`은 종료 이유·시간·파일 해시다. `--capture`는 시작/종료의 실제 입력 RGB와 관측 JSON을 추가한다. `extensions.json`과 `extensions/`에는 실행한 확장 진입 파일·해시·최종 추가 형상을 보관한다. 제어기를 쓰면 `controller-decisions.jsonl`에 매 호출의 실제 RGB 입력·응답·발행 명령을 저장한다. `--video`는 ffmpeg로 관찰용 `motion.mp4`를 녹화한다. `--video-camera cctv_top --video-fps 10`처럼 시점을 지정할 수 있다. `video-frames.jsonl`에 프레임별 에피소드/SIM 시각을 남기며 pause 시간은 생략한다. 전체 상태·제어기 기억의 checkpoint replay는 제공하지 않는다.

| 파일 | 책임 |
|---|---|
| `sim/session_config.py` | 설정 버전·기본값·검증 |
| `sim/session.py` | 세계 수명·reset/step·명령·관측·native viewer 연결 |
| `sim/camera_robot_port.py` | 기존 로봇별 RGB/발행 명령 경계 |
| `scripts/sim_cli.py` | new/init/inspect/layouts/run과 실행 기록 |
| `scripts/open_simulation.command` | 기존 Python 환경 선택과 프로세스 세션 관리 |
| `scripts/check_simulation.py` | 실제 물리·초기화·카메라·설정 통합 검사 |

명령 스케줄은 고정 입력이며 자율 운반·LLM 협력 성공을 뜻하지 않는다. 기존 하네스는 표준 관리 계층의 workflow 어댑터로 연결된다. 제어 루프 전체를 이 Python API로 이관한 것은 아니다. 완료된 연구 결과 비교는 [TensorBoard](tensorboard.md) 절차를 따른다. 실행기는 개별 PID 기반 세션을 만들며 Ctrl-C/창 종료/시간 제한 시 자신이 시작한 세계와 창을 닫는다.

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


## 기존 연구 맵 선택

`scenes`는 기존 4개 엔진 예제, 출하장 5변형, 단독 지도 9개, 공동 운반 지도 6개,
ACT 22조건(새 맵 16 + 기존 회귀 6), 다중 물건 12조건을 표시한다. 기존 JSON·생성기를 그대로
사용한다. `layouts`도 같은 목록을 표시한다. 아래 명령은 외부 모델을 호출하지 않는다.

```bash
bash scripts/open_simulation.command init navigation.json --scene navigation/s-bends
bash scripts/open_simulation.command init pair.json --scene pair_navigation/narrow-door
bash scripts/open_simulation.command init act.json --scene act/train-open-1
bash scripts/open_simulation.command init multi.json --scene multi_object/mixed_eight
bash scripts/open_simulation.command run multi.json --paused --capture --video
```

새 지도는 기존 형식의 JSON을 복사/수정한 후 `--scene navigation/file --map-file my-map.json`
또는 `pair_navigation/file`로 불러온다. `map_file`과 Python 확장 경로는 **설정 파일의 폴더** 기준이다.
`inspect`는 지도 계약·고정 TOP 보정을 검사하고 설정과 장면 출처를 출력한다. Python 확장은 실행하지 않는다.

출하장은 기존 빔/상자·로봇 배치·예고하지 않은 장애물까지 복원한다. 공동 운반 지도와 ACT 지도는
기존 정적 미리보기처럼 빔을 바닥에 배치한다. 파지된 상태나 훈련된 ACT 제어기가 자동으로 생성되지 않는다.
단독 지도는 r1을 시작 구역에 두고 r2/r3는 코스 밖에 주차한다. 원하는 시작점은 `scene.robots`로 명시한다.
`R`/`reset()`은 선택한 장면의 로봇·화물·추가 물체를 같은 model/data에 복원한다.

`scene.contact_profile`은 출하장에 한해 `legacy`, `global_noslip`, `local_contact`,
`local_contact_fine`을 받는다. 기본은 기존 장면 생성기의 물리다. 특정 운반 결과를 재현하려면
해당 실행의 프로필과 모델 파일을 함께 지정한다. weld는 계속 OFF다.

`scene.json`에는 선택 항목·원래 지도/초기화·실제 화물·경계·확장 여부를, `scene-sources/`에는
사용한 지도/프로토콜 원본과 해시를 저장한다. 이 초기화 기록은 평가 전용이다. `scene.xml`은 생성된
XML이며 reset 시 적용한 카메라·배치까지 포함한 컴파일 상태는 `model.mjb`에 있다.
예외로 종료해도 `result.json`에 오류를 남기고, 확장 실행 전 `input-files/`에 진입 소스를 보존한다.
headless에서 SIM 시간에 도달하지 못한 wall timeout은 종료 코드 2를 반환한다.

기존 버전 1 설정에서 `scene.layout`을 생략하면 호환성을 위해 `camera_team`이 유지된다.
새 설정은 `init`으로 생성해 선택 장면을 명시한다. 원래 장애물/공 예제는
`new DIR --template extensions-demo`로 생성할 수 있다.


유한 통신 연구 실행도 `python -m scripts.run_rgb_communication_study prepare/check/run/trial`로
로컬 Mac/Linux에서 호출할 수 있다. 인자는 해당 subcommand의 `--help`와
[연구 실행기 계약](research_parallel/d2-study-runner.md)을 따른다. 출처가 동결된 manifest와
증거·예산·모델 admission이 필요하며, native 장면을 연 것만으로 이 조건이 충족되지는 않는다.

### 관찰 창 속도와 시뮬레이션 시간

`--realtime-factor`는 SIM 시간의 목표 진행 속도이며, 컴퓨터가 물리·영상 처리·화면 표시를 따라가지 못하면 그 속도를 보장하지 않는다. 예를 들어 SIM140초에 현실331초가 걸리면 실제 진행률은 약0.42배속이다. 이는 저장 영상을 느리게 재생하는 것이 아니라 시뮬레이션 진행 자체가 늦은 상태다. 결과의 SIM 시간과 `wall_s`를 나눠 확인한다.

표준 관찰 창은 별도 복제 모델의 그림자·반사를 생략하고 state-only sync 및 묶음 pacing을 사용한다. 로봇이 입력으로 받는 RGB와 저장 원본 영상, 물리 timestep은 이 관찰 창 설정에 영향받지 않는다. 검증 범위는 [속도·병렬 실행 기록](../experiments/2026-09-23-local-dispatch-performance/README.md)을 따른다.
