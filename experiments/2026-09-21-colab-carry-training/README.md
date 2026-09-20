# ACT 입력 비교 학습의 Colab 이식

관련 이슈: #83. 입력 비교와 로컬 물리 평가의 기본 설계는
`experiments/2026-09-21-carry-input-ablation/protocol.json`을 그대로 사용한다.
변경 범위는 학습 실행 환경(CPU → CUDA), 자료 이동 검증, 중단·재개다.
모델 교체, 추가 데이터, 시뮬레이션 이전, Drive 사용은 포함하지 않는다.

## 실행

1. 깨끗한 커밋 상태에서 `python3 scripts/colab_carry_bundle.py pack --output outputs/colab/carry.zip`.
2. `notebooks/carry_input_colab.ipynb`의 셀을 Colab 임시 노트북에 복사한다.
   Chrome **강 / kcm0127@gmail.com**, T4 GPU를 사용한다. Drive에 복사하지 않는다.
3. ZIP과 manifest를 런타임에 직접 업로드한다. Python 3.12 별도 연구 환경과
   기존 pinned LeRobot/PyTorch 의존성을 설치한다. 이어서 `scripts/patch_reference_act.py`로
   로컬에서도 사용하는 두 줄 device 호환 패치를 적용한다. 원본/패치 해시를 검증하며
   관절 상태 입력을 추가하지 않는다. 누락 시 학습 전에 거부한다.
4. `--diagnostic`으로 256px/4시점 조건의 2 update, CNN cache 오차 및 CPU 배포 오차를 확인한다.
   이 실행은 8000-step scheduler를 유지하지만 완성된 모델이나 비교 결과가 아니다.
5. 새 출력 폴더에서 전체 4조건 × 2seed 학습을 실행한다. 각 모델 8000 step, batch32,
   동일한 동결 backbone과 development 선택 기준을 유지한다.
6. 결과 ZIP을 VM이 살아 있을 때 내려받아 보존한다. 원본 ZIP·SHA256,
   canonical/relocated manifest·파일 해시·source manifest를 함께 보존한다.
   결과만으로 물리 성공을 주장하지 않는다. 로컬 평가 작업에 모델과 report를 전달한다.

## 중단·검증 경계

- `--resume`은 source SHA, canonical data SHA, arm, seed, 전체 steps, batch,
  upstream revision, 기록된 환경이 정확히 일치해야 한다.
- `resume.pt`는 모델·optimizer·scheduler·샘플러·CPU/CUDA RNG·최선 checkpoint·진행 이력을
  원자적으로 저장한다. 1/500 step, 완료/진단 중지 지점에서 저장한다.
- Colab 연결 해제와 VM 삭제는 다르다. VM 삭제 시 다운로드하지 않은 체크포인트는 사라진다.
  자체 keepalive/자동 VM 복구나 Drive 백업을 구현하지 않는다. 이후 공식 CLI의 세션 유지 기능을 사용했지만 VM 보존을 보장하지 않는다.
- CUDA TF32/혼합정밀도를 사용하지 않는다. CPU와 GPU의 부동소수점 비트 동일성을 주장하지 않는다.
  native/cache feature 오차와 action chunk 오차, GPU→CPU 배포 오차를 실제 측정하고 제한을 넘으면 실패시킨다.
- 데이터 이동은 **episode root만 치환**한다. 원래 manifest를 그대로 보존하고
  분할·순서·context·action 및 모든 참조 파일 해시를 검증한다. 원본 해시를 단순 덮어쓰지 않는다.
- 재개 체크포인트는 이 실행에서 생성된 신뢰할 수 있는 파일만 사용한다.

## 시뮬레이션 가능 여부 (구현 범위 밖)

Colab의 Linux VM에서 MuJoCo headless 렌더링/영상 저장은 기술적으로 가능하다.
프로젝트의 Ubuntu CI는 Linux 실행 근거지만 **Colab 실험 성공 근거가 아니다**.
macOS `mjpython` 실행 경로, OpenGL/EGL 또는 OSMesa 설정, 런타임 시간 제한을 별도로 맞추고
짧은 물리·영상·행동 회귀를 검증해야 한다. 이번 작업에서는 클라우드 시뮬레이션을 실행하지 않는다.

참고: https://mujoco.readthedocs.io/en/stable/python.html ,
https://research.google.com/colaboratory/faq.html

## 2026-09-21 초기 실행 기록 (01:26–01:51 KST)

실행 소스 `9aef6bb0fb7d05e6b419b2f79b361ebd29099948`을 별도 ZIP으로 고정했다.
Colab Tesla T4에서 전체 고정 데이터의 256px/4시점 2-update 진단이 통과했다.
feature cache/native 최대 차이 1.5903e-4, 전체 action chunk 최대 차이 2.2650e-6,
GPU→CPU 최대 차이 8.0466e-7(train/dev 각 2표본)이다.
로컬과 같은 ACT 패치 파일 해시 `2ea0acbdf210acee9d08f1f306afbda1e935bce0604b3b67b8d87c57c3f79a19`를 확인했다.
진단·실패 로그·설치 버전·manifest를 로컬로 내려받아 ZIP 해시까지 대조했다.

본 학습은 같은 소스로 01:26 KST에 시작했으며 `colab-carry-cohort` 세션의 종료점은
8개 모델 각 8000 step 완료 또는 첫 오류다. 아직 전체 학습 완료나 물리 평가 성공을 뜻하지 않는다.
실행 소스는 Colab `/content/ugrp-source-v3`, 데이터는 `/content/ugrp-carry/data/dataset.json`,
출력은 `/content/carry-training`, 로그는 `/content/carry-training.log`다.
`verification.json`에 검증 범위와 원본 위치를 기록했다.

초기 실행은 **강 / kcm0127@gmail.com**의 scratchpad 탭에서 확인했다. 이후 실행은 아래 CLI로 전환했다.
새로고침하지 않고 다음 셀로 상태를 읽을 수 있다.

```python
from pathlib import Path
import json
print([(p.parent.name, json.loads(p.read_text()).get('completed_steps', 0),
        json.loads(p.read_text()).get('complete', False))
       for p in Path('/content/carry-training').glob('model-*/report.json')])
print(Path('/content/carry-training.log').read_text()[-2000:])
```

끝난 모델과 report·예측 파일·cohort.json·로그를 내려받아 검증한 뒤
로컬 물리 평가 작업에 전달한다. 실행 중인 체크포인트는 `resume.pt`가 atomic rename으로
저장되므로 파일 복사 시 온전한 이전/최신 checkpoint 중 하나를 얻는다.
Colab VM이 종료되기 전에 회수해야 하며 자동 영구 백업은 아니다.
명시적인 중지 요청 때만 `/content/act-env/bin/python /content/ugrp-source-v3/scripts/ugrp_session.py stop colab-carry-cohort`로 이 세션을 정리한다.

최종 재현 번들은 로컬 `outputs/colab/carry-final.zip` 및 `carry-final.manifest.json`이다.
실행 소스9aef6bb와 원본 데이터를 함께 묶고 다시 풀어 전체 해시를 검증했다.
01:28 KST 마지막 확인은 첫 모델 feature cache 준비 중(프로세스 생존)이었다.
이후 Mac 잠금으로 Chrome 접근이 막혀 후속 진행률은 확인하지 못했다.
중지 명령은 보내지 않았으며 최종 모델 회수는 아직 0개다.


## CLI 전환 및 첫 결과 회수

Mac 잠금 해제 후 `google-colab-cli 0.6.0`을 kcm0127 계정의 기존 T4 런타임에 연결했다.
새 VM 생성·학습 재시작·Drive 조회/마운트 없이 기존 커널의 `fullproc`과 파일을 확인했다.
PyPI 기본 의존성 `jupyter-kernel-client 1.0.2`에는 CLI가 요구하는 `KernelClient`가 없어
첫 `exec`는 실패했다. 공식 저장소 `uv.lock`의 Google Colab fork를 고정해 해결했다.

```sh
uv tool install google-colab-cli==0.6.0 --with 'jupyter-kernel-client @ git+https://github.com/googlecolab/jupyter-kernel-client.git@f18e982c3265df5e923aa9def101ab3fd737e139'
colab sessions
colab ls -s ugrp-carry /content/carry-training
colab download -s ugrp-carry /content/carry-training/cohort.json /tmp/ugrp-cohort.json
```

`ugrp-carry`는 이 Mac의 로컬 CLI 세션 이름이며 새 설치에서 자동 생성되지 않는다.
0.6.0에는 웹 런타임 attach 명령이 없어 공식 패키지의 `Client.list_assignments()`에서
조회한 기존 assignment를 `StateStore`에 등록하고 Jupyter의 기존 kernel ID를 연결했다.
프록시 자격증명은 CLI의 사용자 설정에만 저장하며 Git·실험 기록에는 포함하지 않는다.
새 런타임의 경우 이름을 재사용하기 전에 계정과 endpoint를 다시 확인한다.

128px/1시점 두 seed 모델을 먼저 Mac으로 회수했다. ZIP·내부 12파일·모델 해시와
원본 데이터/소스/seed/8000step/batch32/adapter를 대조했다. Mac CPU에서 각 모델의
train/dev 처음·마지막 4표본 예측을 Colab 출력과 비교했고, 8개 검사 모두
atol/rtol 1e-4 이내(최대 3.7551e-6)였다. 이는 전송·모델 로드 검증이며 물리 성공률이 아니다.
원본과 `mac-verification.json`은 `outputs/colab-results/recovery-2-1789922566/`에 있다.

01:50 KST CLI 확인 시 3개 학습 완료, 네 번째 모델 7000/8000step으로 기존 프로세스가 살아 있었다.
남은 결과는 `colab-carry-results` 소유 세션에서 공식 CLI 파일 API로 회수한다.
종료점은 8개 모델의 해시·설정 및 최종 cohort 검증 완료, 연속 오류 3회, 또는 2시간이다.
상태는 `outputs/colab-results/collection-status.json`, 결과는
`outputs/colab-results/carry-training/`에 저장한다. 이 기록 시점에는 8개 전체 회수와 물리 평가는 미완료다.

01:51 KST CLI 회수기가 128px 두 history × 두 seed, 총 4개 모델의 다운로드와 해시·설정 검증을 완료했다. 나머지 256px 4개 모델을 기다리며 계속 실행 중이다.


## CLI 런타임 복구 (02:03 KST 이후)

최초 T4 assignment가 서버 목록에서 사라져 회수기가 중지됐다. 종료 원인은 확인되지 않았다.
이미 Mac에 회수한 5개 모델은 해시가 일치했고, 해당 모델의 train/dev 처음·마지막
20개 native 예측 검사도 통과했다(최대 차이 5.1260e-6).

같은 kcm0127 계정에 CLI 세션 `ugrp-carry-recovery`를 생성해 T4를 다시 할당했다.
소스 `9aef6bb`, canonical 데이터, 전체 설치 패키지 freeze가 최초 런타임과 일치한다.
소스/데이터 ZIP과 5개 완성 모델 ZIP을 업로드해 원격 해시를 대조한 뒤 복구했다.
CLI의 큰 파일 단일 업로드는 HTTP 400으로 실패했고, Jupyter Contents API의
4 MiB 순차 chunk 업로드가 성공했다. 나머지 3개 모델은 기존 seed에서 다시 학습했다.
사라진 VM의 미회수 optimizer 상태를 이어받았다고 주장하지 않는다.

마지막 `r256-h4 / seed 20260922`는 8000 update를 완료한 뒤 최종 평가/저장 중
SIGKILL로 종료됐다. cgroup의 oom/oom_kill 카운터는 0이어서 원인은 미확정이다.
8000-step 체크포인트, 저장 후보, 실패 로그를 ZIP으로 보존하고 Mac에서 모든 파일 해시와
`torch.load(weights_only=True)`를 검증했다. ZIP SHA256은
`f51c9b3a150126cf46f670e10d7ecb57053ece0c8109cd9879c457da487d5826`이다.
같은 소스의 `--resume`으로 추가 update 없이 최종 검증·저장을 완료했다.
프로세스 종료 코드는 0이며, 중단 전 저장 후보와 복구된 최종 모델의 SHA256이
`58abfd79305554332af9a490b3e0c4cbdfeb4d3d511def7e2e60ea59bd711baf`로 동일하다.

원본은 로컬 `outputs/colab-results/`와 `outputs/colab-verification/runtime-recovery.json`에
보관한다. Drive나 원격 raw 백업을 사용하지 않았다. 물리 평가는 별도 고정 checkout
`/Users/changmin/.codex/worktrees/carry-input-physics/ugrp`의 소스
`e00435b82470e722196858728bc22c1123ef730c`에서 기존 36-run 프로토콜을 사용한다.
8개 모델의 전체 회수·해시 및 Mac native 검증을 통과해야 시작하며, 시작 전 소유 Colab VM을 종료한다.


## 8개 모델 회수 및 Mac 검증 완료

전체 8개 모델이 각 8000 update를 완료했다. `cohort.json`, 학습 로그와 모델별
6파일, 총 50파일의 SHA256을 최종 manifest와 대조했다. Mac CPU에서 train/dev
처음·마지막 native 출력 32개를 대조했고 최대 절대 차이는 **5.6625e-6**이었다
(atol/rtol 1e-4). 8개 모델 모두 파라미터 수 **11,421,588** 및 동결 backbone
state 해시가 같았다. 모델별 선택 step·오차·시간·환경·해시는 `training-results.json`에 기록했다.

검증 뒤 `ugrp-carry-recovery`의 정상 종료 코드 0을 확인하고 해당 Colab VM을
`colab stop`으로 정리했다. 고정 소스 e00435b의 Mac 36-run 물리 비교가 시작됐다.
이 절의 완료 범위는 **학습·파일 회수·배포 일치 검사**이며 물리 성공률은 후속 audit에서 판정한다.
원시 모델·영상·로그는 로컬에만 보관되며 Git의 기록/해시가 원시 자료의 원격 백업을 의미하지 않는다.
