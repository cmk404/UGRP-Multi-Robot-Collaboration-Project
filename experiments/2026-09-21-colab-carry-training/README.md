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
  자동 keepalive/재접속이나 Drive 백업을 구현하지 않는다.
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

## 2026-09-21 실행 상태

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

Colab을 다시 확인할 때는 **강 / kcm0127@gmail.com**의 이미 열린 scratchpad 탭을 사용한다.
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
