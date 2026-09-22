# 로컬에서 시뮬레이션 켜고 실시간으로 보기

MuJoCo의 세 MasterPi와 전체 장면·로봇 카메라를 브라우저에서 본다. 모델 계정·프록시·Colab·Kaggle·실물 로봇은 필요 없다. 각 참여자가 자신의 컴퓨터에서 실행한다. 저장소는 비공개이므로 먼저 GitHub collaborator 초대를 수락해야 한다.

## 최초 설치

Ubuntu 24.04 또는 Windows의 WSL2 Ubuntu는 [설치 안내](ubuntu_quickstart.md)의 1번을 따른다. 브라우저는 Windows에서 열어도 된다. Ubuntu/WSL에서 필요한 OS 패키지:

```bash
sudo apt update
sudo apt install -y git python3.12 python3.12-venv libosmesa6 libgl1 libglfw3
```

Mac은 Python 3.12가 필요하다. 기존 개발 환경 `.venv-sim-worker-mac`이 있으면 재사용한다. **새 clone에서만** 저장소 루트에서 설치한다.

```bash
python3.12 -m venv .venv-dev
.venv-dev/bin/python -m pip install -r requirements-sim.txt
.venv-dev/bin/python -m pip check
```

기존 환경을 복사하거나 특정 개발자의 파일을 가져올 필요가 없다. 대량 학습 가중치도 사용하지 않는다.

## 실행

저장소 루트에서:

```bash
bash scripts/open_simulation.command
```

터미널에 표시되는 **http://127.0.0.1:8765**를 브라우저에서 연다. Mac은 Finder에서 `scripts/open_simulation.command`를 더블 클릭해도 된다. 실행기는 브라우저·프로필을 임의로 선택하지 않는다. 첫 카메라 준비까지 잠시 걸릴 수 있다.

1. 전체 장면과 R1·R2·R3 카메라가 모두 표시되는지 확인한다.
2. **R1 이동 데모**를 눌러 실제 물리 이동과 카메라 변화를 본다.
3. 로봇을 골라 전진·후진·회전한다. 클릭당 0.5 SIM초 명령이며 자동 정지한다.
4. **일시정지**는 물리 시간을 멈추고 이동 명령을 취소한다. **시작**으로 재개한다.
5. **초기화**로 같은 seed를 재현하거나 다른 seed의 배치를 만든다. 초기화 후에는 일시정지 상태다.
6. **시뮬레이션 종료** 또는 실행 터미널의 Ctrl-C로 종료한다.

기본 최대 실행 시간은 30분이다. 브라우저를 닫거나 다른 탭으로 이동해 상태 조회가 멈추면 15초 후 일시정지하고, 최대 실행 시간이 되면 서버도 종료한다. 브라우저를 다시 열면 시작 버튼으로 재개한다. 서버를 종료한 뒤에는 명령을 다시 실행한다.

영상은 현재 물리 세계의 JPEG를 반복 전송하며 기본 상한은 초당 4회다. 실제 갱신 속도는 컴퓨터 성능에 따라 낮아진다. 화면의 SIM 시간은 실제 경과 시간과 다를 수 있다. 네 개의 카메라가 같은 물리 시점에서 생성된다. 끊기면 마지막 이미지를 유지하고 **연결 끊김**으로 표시한다.

## 실행 옵션과 문제 해결

```bash
# 포트 충돌 시 자동으로 빈 포트 선택; 다른 서버를 종료하지 않는다.
bash scripts/open_simulation.command --port 0

# 더 가벼운 렌더링, 10분 세션, 특정 초기 배치
bash scripts/open_simulation.command --fps 2 --duration 600 --seed 42

# 별도로 관리하던 기존 Python 환경
UGRP_SIM_PYTHON=/absolute/path/to/bin/python bash scripts/open_simulation.command
```

출력은 매번 새 `outputs/sim-live-<날짜>-<ID>/`에 저장한다. `session.json`은 실행 SHA·설정, `commands.jsonl`은 실제 적용한 명령, `result.json`은 종료 상태·시간·갱신 횟수, JPEG 네 장은 마지막 화면이다. 종료 전의 폴더를 완료 결과로 보지 않는다. 전체 영상 녹화는 제공하지 않는다.

`simulation-live`가 이미 실행 중이면 기존 터미널이나 기존 주소를 사용한다. 자신의 뷰어를 다른 터미널에서 종료할 때만 선택한 Python으로 `scripts/ugrp_session.py stop simulation-live`를 실행한다. 화면 접근은 localhost로 제한되며 다른 사람의 컴퓨터에서 이 주소로 접속할 수는 없다. 팀원은 자신의 clone에서 같은 실행 명령을 사용한다.

Linux 렌더링 오류는 `libosmesa6` 설치를 확인한다. 실행기가 기본으로 OSMesa를 선택한다. 기존 환경 변수가 다른 렌더러를 지정했다면 `MUJOCO_GL=osmesa bash scripts/open_simulation.command`로 실행한다. macOS는 기존 MuJoCo 렌더링 경로를 사용한다. WSL의 localhost 전달·그래픽 환경은 각 PC에서 확인해야 한다.

## 연구 실행과의 관계

이 화면은 **수동 물리·카메라 체험 환경**이다. 고정 이동 데모를 자율 운반이나 에이전트 협력 성공으로 해석하지 않는다. 물리·카메라 배치와 FOV, weld OFF를 유지하며, 제어에는 기존 `CameraRobotPort`의 제한된 바퀴 명령을 사용한다. LLM 요청이나 교사 정답 보정은 없다.

실험 결과 비교는 기존 [TensorBoard](tensorboard.md), 자율 출하는 [현재 실행 경로](current_status.md)를 사용한다. 이 뷰어는 실행 중인 다른 실험에 붙는 관제창이 아니라 독립된 로컬 세계다.
