# RGB varied-start 팀원 재현 안내

이 안내는 구현 커밋 `b4c9535d24efc6747ed4f6df244b902976efb00f`와 별도 배포된 `models.zip`을 사용해 새 출력에서 실행하는 절차다. 실행 전후 저장소는 clean 상태를 유지하고, 출력·케이스 파일·모델은 저장소 밖에 둔다. 같은 이름의 출력 폴더가 이미 있으면 실행기가 덮어쓰지 않으므로 새 이름을 사용한다.

## 1. Ubuntu 24.04 환경 설치

기준 환경은 Ubuntu 24.04 x86_64와 Python 3.12다. CPU 소프트웨어 렌더링과 영상 기록에 필요한 OS 패키지를 먼저 설치한다.

```bash
sudo apt update
sudo apt install -y \
  git gh python3.12 python3.12-venv \
  libosmesa6 libgl1 libglfw3 ffmpeg fonts-noto-cjk unzip

mkdir -p ~/projects
cd ~/projects
gh repo clone kcm0127-dotcom/ugrp
cd ugrp
git fetch origin
git switch --detach b4c9535d24efc6747ed4f6df244b902976efb00f
test "$(git rev-parse HEAD)" = b4c9535d24efc6747ed4f6df244b902976efb00f
test -z "$(git status --porcelain=v1)"

python3.12 -m venv .venv-dev
.venv-dev/bin/python -m pip install --upgrade pip
.venv-dev/bin/python -m pip install -r requirements-sim.txt
.venv-dev/bin/python -m pip check
```

`requirements-sim.txt`의 실제 설치 항목은 MuJoCo 3.12.0, NumPy 2.5.2, `opencv-python-headless` 5.0.0.93, Pillow 12.3.0, websockets 15.0.1, GLFW 2.10.2, PyOpenGL 3.1.10이다. RGB 학생은 OpenCV와 NumPy를 직접 사용한다. 이 JSON 모델의 학습·추론은 저장소 자체 구현이며 SciPy와 scikit-learn을 import하지 않는다. Torch, Stable-Baselines3, Gymnasium도 이 실행에는 필요하지 않다. 전체 회귀 테스트도 수행하려면 별도로 `requirements-test.txt`를 설치한다.

기준 결과를 만든 Mac 환경 기록은 Python 3.12.13, MuJoCo 3.12.0, NumPy 2.5.2, `opencv-python-headless` 5.0.0.93, Pillow 12.3.0이었다. 그 환경의 websockets는 17.1이었지만 현재 개발용 `requirements-sim.txt`는 15.0.1을 고정한다. SciPy와 scikit-learn은 설치되지 않았다. 따라서 위 설치는 현재 Ubuntu 개발 의존성으로 실행하는 재현이며, 과거 Mac 환경의 바이트 단위 lockfile 복원은 아니다. 플랫폼 간 결과가 같다고 사전에 가정하지 말고 새 출력과 audit를 보존한다.

```bash
.venv-dev/bin/python -m pip install -r requirements-test.txt
.venv-dev/bin/python scripts/run_ci_tests.py
```

Ubuntu에서는 창을 여는 `mjpython` 대신 일반 venv Python과 OSMesa를 사용한다. 코호트 실행기의 `--mjpython` 인자에도 `.venv-dev/bin/python`을 전달한다. macOS의 GUI 렌더링 실행만 `.venv-sim/bin/mjpython` 또는 현재 호환 경로의 `mjpython`이 필요하다.

```bash
export MUJOCO_GL=osmesa
ffmpeg -hide_banner -encoders | grep libx264
```

## 2. 모델을 저장소 밖에 풀고 검증

`models.zip`과 `models-manifest.json`을 받은 위치를 아래 `PACKAGE_DIR`로 지정한다. ZIP의 기준 SHA-256은 `d6d421c2d35ed448c4492592220b3e3141d346f5e576ebbdf5a7a3c787b7e635`다.

```bash
export PACKAGE_DIR="$HOME/Downloads/varied-start-evidence-stage-20260910-v1"
export MODEL_ROOT="$HOME/ugrp-models/b4c9535"

printf '%s  %s\n' \
  d6d421c2d35ed448c4492592220b3e3141d346f5e576ebbdf5a7a3c787b7e635 \
  "$PACKAGE_DIR/models.zip" | sha256sum -c -

test ! -e "$MODEL_ROOT.tmp"
mkdir -p "$MODEL_ROOT.tmp"
unzip -q "$PACKAGE_DIR/models.zip" -d "$MODEL_ROOT.tmp"
test ! -e "$MODEL_ROOT"
mv "$MODEL_ROOT.tmp" "$MODEL_ROOT"
find "$MODEL_ROOT/models" -maxdepth 2 -type f | sort
```

추출 결과에는 다음 세 런타임 모델 묶음이 있어야 한다.

- `models/varied/`: yaw → lateral → yaw → forward → yaw → lateral → forward 단계용 RGB varied-start 모델과 `varied-start-skill.json`
- `models/grasp/`: RGB 파지 복구 모델, `student-skill.json`, 고정 평가 fixture
- `models/straight/`: 비교용 직진 RGB 접근 모델과 `approach-skill.json`

각 skill JSON은 내부 모델 파일의 상대 경로와 SHA-256을 갖고 있으며 실행기가 이를 다시 확인한다. `models-manifest.json`에는 ZIP 내 21개 파일의 개별 SHA-256도 기록돼 있다.

## 3. 단일 케이스 실행과 독립 audit

단일 케이스 입력은 `r1`, `r3` 각각에 `distance_m`, `lateral_m`, `yaw_deg`를 정확히 하나씩 지정한다. 아래 값은 형식과 경계값을 확인하기 위한 예시이며 새 성능 수치를 주장하는 기준 케이스가 아니다.

```bash
cd ~/projects/ugrp
export RUN_ROOT="$HOME/ugrp-runs/varied-start-b4c9535"
mkdir -p "$RUN_ROOT/cases"

cat > "$RUN_ROOT/cases/one.json" <<'JSON'
{
  "case_id": "reproduction-01",
  "start_poses": {
    "r1": {"distance_m": 0.15, "lateral_m": 0.06, "yaw_deg": -10},
    "r3": {"distance_m": 0.40, "lateral_m": -0.06, "yaw_deg": -10}
  }
}
JSON

test "$(git rev-parse HEAD)" = b4c9535d24efc6747ed4f6df244b902976efb00f
test -z "$(git status --porcelain=v1)"

.venv-dev/bin/python scripts/ugrp_session.py run varied-one -- \
  .venv-dev/bin/python scripts/run_camera_varied_start_student.py \
  --stage-model-dir "$MODEL_ROOT/models/varied" \
  --straight-model-dir "$MODEL_ROOT/models/straight" \
  --grasp-model-dir "$MODEL_ROOT/models/grasp" \
  --case-json "$RUN_ROOT/cases/one.json" \
  --condition visual \
  --out-dir "$RUN_ROOT/one-visual"
```

실행기는 `result.json`, `execution-trace.json`, `evaluation-only.jsonl`, RGB 프레임과 `motion.mp4`를 남긴다. 성공 여부와 무관하게 원본을 보존한다. 실행 오류가 없는 결과는 같은 HEAD와 같은 모델로 독립 audit한다.

```bash
.venv-dev/bin/python scripts/audit_camera_varied_start_student.py \
  --run-dir "$RUN_ROOT/one-visual" \
  --stage-model-dir "$MODEL_ROOT/models/varied" \
  --straight-model-dir "$MODEL_ROOT/models/straight" \
  --grasp-model-dir "$MODEL_ROOT/models/grasp" \
  --out "$RUN_ROOT/one-visual/audit.json"

.venv-dev/bin/python - <<'PY'
import json
from pathlib import Path
p = Path.home() / "ugrp-runs/varied-start-b4c9535/one-visual/audit.json"
assert json.loads(p.read_text())["ok"] is True
print(p)
PY
```

audit는 `result.json`의 `source_sha`가 audit 시점의 현재 HEAD와 같은지 먼저 검사한다. 따라서 다른 커밋으로 이동한 뒤 과거 결과를 audit하면 실패한다. 코호트 실행기는 이보다 더 엄격하게 시작 전과 매 실행 사이에 HEAD, clean working tree, 세 모델 디렉터리의 모든 JSON 해시가 고정됐는지 검사한다.

중단할 때는 실행 터미널에서 `Ctrl-C`를 누르거나 다른 터미널에서 아래 명령을 사용한다.

```bash
cd ~/projects/ugrp
.venv-dev/bin/python scripts/ugrp_session.py stop varied-one
```

## 4. 전체 paired cohort 실행

전체 코호트 파일은 최상위에 `cases` 배열을 두고, 각 원소는 위 단일 케이스와 같은 형식이어야 한다. 모든 `case_id`는 영문자·숫자·`_`·`-`만 사용하며 중복되면 안 된다. 이번 새 시작 비교를 재현하려면 커밋된 `experiments/2026-09-10-rgb-varied-start/heldout-cases.json`을 `$RUN_ROOT/cases/full-cases.json`에 복사한다. 기존 시작 검증에는 `retention-cases.json`을 사용하고 `--conditions visual`로 실행한다. 출력 폴더와 세션 이름은 각각 새로 지정한다.

```bash
cd ~/projects/ugrp
test "$(git rev-parse HEAD)" = b4c9535d24efc6747ed4f6df244b902976efb00f
test -z "$(git status --porcelain=v1)"

.venv-dev/bin/python scripts/ugrp_session.py run varied-full -- \
  .venv-dev/bin/python scripts/run_camera_varied_start_cohort.py \
  --cases-json "$RUN_ROOT/cases/full-cases.json" \
  --stage-model-dir "$MODEL_ROOT/models/varied" \
  --straight-model-dir "$MODEL_ROOT/models/straight" \
  --grasp-model-dir "$MODEL_ROOT/models/grasp" \
  --out-dir "$RUN_ROOT/full-paired-01" \
  --mjpython "$PWD/.venv-dev/bin/python" \
  --conditions visual straight
```

코호트 실행기는 각 케이스를 `visual`, `straight` 순서로 실행하고, 각 실행 직후 audit하며, 두 조건의 초기 물리 상태와 첫 RGB 쌍이 일치하는지도 검사한다. 중간 실패도 원본 폴더와 stdout/stderr를 남긴다. 완료 판정은 `$RUN_ROOT/full-paired-01/cohort-report.json`의 `complete`와 각 run의 `audit_ok`를 확인하고, 생성된 영상도 직접 검토해야 한다.

```bash
cd ~/projects/ugrp
.venv-dev/bin/python scripts/ugrp_session.py stop varied-full
```

## 5. 관측 경계와 재현 범위

학생 접근 정책의 런타임 입력은 각 로봇의 자기 `robot_cam` RGB, 고정 `cctv_top` RGB, 자기 발행 명령 이력과 단계 이력뿐이다. 실행 중 교사의 좌표, base pose, 관절 측정, 접촉, 평가 결과는 접근·파지 명령이나 단계 전환 입력으로 사용하지 않는다. 물리 정답과 접촉은 별도 평가·audit 출력에만 쓰인다.

학습 데이터 생성용 teacher는 예외적으로 fixture 기준 위치와 base xyz/rpy를 사용해 접근 명령과 offline label을 만든다. 이후 집기는 기존 RGB 모델을 실행하며 접촉과 높이로 성공을 별도 평가한다. 이는 RGB 학생의 독립 실행 입력이 아니다. teacher 성공, offline label 적합도, 학생의 새 조건 실행 성공을 서로 같은 주장으로 취급하면 안 된다.

이 모델과 실행 프로토콜의 범위는 두 로봇의 시작 거리 0.15–0.40 m, lateral −0.06–+0.06 m, yaw −10–+10도다. 실행기는 이 범위를 벗어난 입력을 거부한다. 또한 `evaluation-fixture.json`의 seed와 기준 base pose, 고정 lane/beam 장면, 실제 코드에 고정된 960×720 렌더링, 기존 로봇 카메라와 고정 top 카메라 위치·자세·FOV를 전제로 한다. 다른 물체 외형·조명·카메라·장면·차선·로봇 배치로 일반화됐다고 볼 수 없다. 관찰용 MP4 카메라는 정책 카메라와 별개다. weld는 `False`다.

`models.zip`은 추론 모델만 담는다. 모델 파일에 포함된 압축 특성과 일부 기준 이미지만으로 동일 모델을 다시 학습할 수 없다. 재학습에는 성공한 teacher 수집 전체의 `report.json`, `actor-samples.jsonl`, `teacher-labels.jsonl`과 각 sample이 가리키는 원본 own/top RGB가 모두 필요하다. 일부 raw RGB, 실행 영상, 개발 결과만으로는 샘플 ID·offline label·제외 사유·입력 해시를 복원할 수 없으므로 재학습 재현 자료로 충분하지 않다.

오류를 공유할 때는 Ubuntu 버전, Python·MuJoCo·OpenCV 버전, `git rev-parse HEAD`, `git status --porcelain=v1`, 모델 ZIP SHA-256, 실행 명령과 stderr를 함께 남긴다. 기존 결과를 수정하거나 실패를 삭제하지 않는다.
