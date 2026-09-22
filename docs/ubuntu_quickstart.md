# Ubuntu에서 처음 실행하고 PR 보내기

팀원용 기준 환경은 **Ubuntu 24.04 LTS, x86_64, Python 3.12**다. 처음에는 GPU나 모델 계정 없이 CPU 소프트웨어 렌더링으로 설치를 확인한다. 명령은 Ubuntu 터미널에서 실행한다. Windows 사용자는 Ubuntu를 설치하거나 [WSL2에 Ubuntu를 설치](https://learn.microsoft.com/en-us/windows/wsl/install)한 뒤 Ubuntu 터미널을 사용할 수 있다. Windows PowerShell에서 아래 Linux 명령을 직접 실행하지 않는다.

이 경로는 기본 물리 시뮬레이션과 코드 기여를 위한 것이다. 실제 Gemini 운반 실험에는 별도 모델 서비스가 필요하다. 과거 Mac N7 실험 결과를 Ubuntu에서 그대로 재현했다고 간주하지 않는다.

## 1. 도구 설치와 저장소 받기

저장소는 비공개다. 먼저 저장소 관리자에게 GitHub 계정의 collaborator 초대와 코드 기여 권한을 받고 초대를 수락한다.

```bash
sudo apt update
sudo apt install -y git gh python3.12 python3.12-venv libosmesa6 libgl1 libglfw3 ffmpeg fonts-noto-cjk

gh auth login
```

인증 안내에서 GitHub.com, HTTPS, 웹 브라우저 로그인을 선택한다. 자신의 GitHub 계정으로 인증한다. 이어서:

```bash
mkdir -p ~/projects
cd ~/projects
gh repo clone kcm0127-dotcom/ugrp
cd ugrp
python3.12 -m venv .venv-dev
.venv-dev/bin/python -m pip install --upgrade pip
.venv-dev/bin/python -m pip install -r requirements-sim.txt -r requirements-test.txt
.venv-dev/bin/python -m pip check
```

이후 모든 명령은 `~/projects/ugrp`에서 실행한다. 가상환경 활성화 없이 `.venv-dev/bin/python`을 직접 사용하므로 새 터미널에서도 같은 환경을 사용한다. 기존 Mac의 `.venv-sim-worker-mac` 폴더를 복사하지 않는다.

## 2. 모델 비용 없이 시뮬레이션 확인

브라우저에서 직접 움직이며 실시간으로 보려면 `bash scripts/open_simulation.command`를 실행하고 표시되는 localhost 주소를 연다. [로컬 카메라 뷰어](local_simulation.md)에 조작·종료 방법이 있다. 아래 GIF 경로는 설치 확인용 유한 실행이다.

```bash
export MUJOCO_GL=osmesa
.venv-dev/bin/python scripts/ugrp_session.py run quickstart -- \
  .venv-dev/bin/python -m scripts.sim_quickstart \
  --output outputs/quickstart-01
```

로봇 R1에 짧은 바퀴 이동 명령을 보내 실제 물리 상태를 진행시키고, 창을 띄우지 않고 장면과 로봇 카메라를 GIF로 저장한다. 모델·프록시·실물 로봇은 사용하지 않는다. 실행이 끝나면 세션도 종료된다.

완료 기준:

- 마지막 JSON에 `"ok": true`, `frame_count: 12`, 양수인 `robot_displacement_m`이 나온다.
- `outputs/quickstart-01/quickstart.gif`를 파일 관리자나 브라우저로 열면 왼쪽 장면과 오른쪽 R1 카메라가 보인다.
- `outputs/quickstart-01/summary.json`에 이동 거리와 시뮬레이션 시간이 기록된다.

GIF는 설치 확인용 고정 명령 데모다. 자율 운반·LLM 판단·다중 로봇 협업의 성공 증거는 아니다. 재실행할 때는 `quickstart-02`처럼 새 출력 폴더명을 쓴다. CPU 속도에 따라 실제 실행 시간은 달라진다.

중단하려면 실행 터미널에서 `Ctrl-C`를 누른다. 다른 Ubuntu 터미널에서 같은 작업만 중단하려면:

```bash
cd ~/projects/ugrp
.venv-dev/bin/python scripts/ugrp_session.py stop quickstart
```

## 3. 테스트하고 PR 보내기

```bash
.venv-dev/bin/python scripts/run_ci_tests.py
```

통과하면 작업 브랜치를 만든다. `your-topic`은 수정할 기능 이름으로 바꾼다.

```bash
git switch main
git pull --ff-only
git switch -c your-topic
```

코드를 수정하고 관련 테스트와 위 기본 데모를 다시 실행한다. Git 작성자 정보가 아직 없다면 이 저장소에서 사용할 자신의 이름과 이메일을 `git config user.name`, `git config user.email`로 설정한다. 아래 파일 경로는 **실제로 수정한 파일들로 바꾼다**.

```bash
git diff
git add path/to/changed_file.py
git commit -m "변경한 동작을 설명"
git push -u origin HEAD
gh pr create --base main --web
```

PR 양식에 문제, 변경 후 동작, 테스트 결과와 데모 검토 결과를 적는다. GitHub의 `offline-regressions`와 `ubuntu-simulation` 결과를 확인한다. `@kcm0127-dotcom`의 Approve 리뷰가 있어야 병합할 수 있다. 승인 후 코드를 바꾸면 다시 승인받는다. `main`에 직접 push하지 않는다. 권한 오류가 나면 관리자에게 저장소 쓰기 권한을 확인한다. 이 비공개 저장소를 공개 저장소로 복사하지 않는다.

모델 행동을 바꾼 PR은 기본 데모만으로 운반 성능을 검증했다고 쓰지 않는다. [실험 관리 절차](../CONTRIBUTING.md)에 따라 실행 코드 SHA와 전체 결과를 별도로 남긴다. `outputs/` 영상·로그는 자동으로 GitHub에 업로드되지 않는다.

## 4. 실제 Gemini 운반 실험 — 선택 사항

기본 데모와 테스트가 통과한 뒤 **[각자 PC에서 Gemini 로그인 프록시 설치](gemini_subscription_proxy.md)**를 따른다. 각 팀원이 본인 PC에서 서버를 실행하고 본인 Google 계정으로 로그인한다. 개발자 Mac이나 개발자 계정에 연결할 필요가 없다.

위 안내의 설치·로그인·모델 목록·실제 짧은 응답 확인을 완료하고 로컬 프록시를 실행한 상태에서 진행한다. 아래 명령은 CLIProxyAPI의 `gemini-3.8-flash-high`를 명시하므로 본인 계정에서 해당 이름으로 응답할 수 있어야 한다. 기존 기본값 `gemini-3.8-flash`와 구분해 실험 manifest에 기록한다. 지원되지 않으면 기존 비교 실험을 그대로 실행할 수 없으며, 기본 데모·테스트·PR 작업은 계속할 수 있다.

`127.0.0.1`은 이 Ubuntu 자신을 뜻한다. UGRP 저장소 루트의 터미널에서:

```bash
export MUJOCO_GL=osmesa
export GEMINI_PROXY_URL='http://127.0.0.1:8391/v1/chat/completions'
ffmpeg -hide_banner -encoders | grep libx264
```

`libx264` 행이 보여야 영상 기록이 가능하다. 실제 모델 호출과 비용을 수반하는 **한 시드** 실험:

```bash
.venv-dev/bin/python scripts/ugrp_session.py run gemini-trial -- \
  .venv-dev/bin/python -m scripts.run_gemini_seed_validation \
  --execute --output outputs/gemini-trial-01 --seeds 45 \
  --model gemini-3.8-flash-high \
  --reasoning-effort medium --request-timeout 60
```

현재 예산은 시드당 최대 30회 모델 호출·120,000 입력 토큰·300 시뮬레이션 초다. CPU 렌더링과 모델 대기 때문에 실제 시간은 더 길 수 있다. 실패 결과도 보존하고, 전체 비교 실험 중 소스를 수정하지 않는다. 위 한 시드 확인은 N7의 5개 시드 비교를 대체하지 않는다.

```bash
.venv-dev/bin/python -m scripts.verify_gemini_budget_run \
  outputs/gemini-trial-01/solo-45 --seed 45 --max-input-tokens 120000
.venv-dev/bin/python -m scripts.review_navigation_trial \
  outputs/gemini-trial-01/solo-45
```

자동 결과와 생성된 영상을 함께 검토한다. 과거 N7 raw 로그는 저장소에 없으므로 새 clone만으로 모든 과거 실패 재생 도구를 실행할 수 없다.

## 막힐 때

| 증상 | 확인할 내용 |
|---|---|
| `Repository not found` | 비공개 저장소 초대 수락 여부와 `gh auth status`의 로그인 계정 |
| `No matching distribution found` | Ubuntu 24.04 x86_64인지, `.venv-dev/bin/python --version`이 3.12인지 확인. 임의로 버전을 풀기 전에 오류를 PR/이슈에 첨부 |
| `DISPLAY`, GLFW, OpenGL 오류 | 실행하는 터미널에서 `export MUJOCO_GL=osmesa`를 다시 실행하고 `libosmesa6` 설치 확인 |
| `FileExistsError` 또는 출력 폴더가 이미 존재 | 기존 결과를 지우지 말고 새 출력 이름 사용 |
| `ffmpeg`가 없거나 `libx264` 오류 | `sudo apt install ffmpeg`와 위 인코더 검사. 무료 GIF 데모에는 ffmpeg가 필요 없음 |
| 모델 연결 거부·401·모델 없음 | 본인 PC의 프록시 실행·로그인 상태와 지원 모델 확인. 기본 데모와는 별도 문제 |
| `fcntl` 없음, `.venv-dev/bin/python` 없음 | Windows Python이 아닌 Ubuntu 터미널인지, 저장소 루트에서 환경을 만들었는지 확인 |

문제를 공유할 때 OS, Python 버전, `git rev-parse HEAD`, 실행 명령과 오류를 첨부한다. 인증정보는 제외한다.

## 지원·검증 범위와 참고

Ubuntu 24.04의 설치·회귀검사·OSMesa 카메라 렌더링과 이동 데모는 [GitHub workflow](../.github/workflows/tests.yml)에서 검사한다. 실제 개인 PC/WSL 드라이버 환경과 외부 Gemini 운반은 별도 확인 대상이다. 설치 파일은 개발 환경을 만들기 위한 것이며, 과거 Mac N7의 패키지 전체를 정확히 복원하는 lockfile은 아니다.

- [Ubuntu의 WSL 설치 안내](https://ubuntu.com/wsl/docs/latest/howto/install-ubuntu-wsl2/)
- [MuJoCo의 렌더링 백엔드 설명](https://mujoco.readthedocs.io/en/3.8.0/programming/): Linux OSMesa는 GPU 없이 소프트웨어 렌더링을 제공한다.
