# 레퍼런스 적용 기준과 현재 차이

2026-09-16. 기준 main `60622bb`. 연구 질문은 자연어 통신이 역할 분담과
협력 이송에 주는 효과다. 로봇 실행 기반은 가능한 한 공개 구현을 재사용하고,
우리 실험에서는 입력 경계·물리 동기화·통신 효과를 검증한다.

## CoELA와 현재 코드

| 항목 | 원 CoELA | 현재 UGRP | 처리 |
|---|---|---|---|
| 상위 구조 | Perception/Memory/Communication/Planning/Execution | `coela_modules.py`에 5모듈 | 유지 |
| 메시지 선택 | 후보 메시지 생성 후 계획 선택 | 한 응답에 action/message | 변경점으로 명시; 원본 방식과 비교 전 동등하다고 주장하지 않음 |
| 관측 | RGB-D 인식과 의미 지도, 환경별 symbolic 설정 | 실행 경로별 RGB; 지도 주행은 승인된 정적 지도 | 센서 계약에 맞춘 어댑터; 원본 벤치마크와 직접 점수 비교 금지 |
| 저수준 실행 | 고수준 계획을 환경별 primitive로 변환 | 과거 ideal-SIM/직접 PWM/최근 RGB 학생이 병존 | 연구 경로를 구분하고 비교군의 실행기를 고정 |
| 통신 비교 | 통신 제거 등 ablation | 무통신/정형/자연어 A/B/C | 같은 관측·예산·스킬에서 반복 |
| 물리 동기화 | TDW/VirtualHome 과제에 맞는 실행 | 공동 파지/들기/운반/해제의 준비·허가·중단 | 프로젝트 고유 검증 대상으로 유지 |

원본: [논문](https://arxiv.org/html/2307.02485v2),
[공식 코드](https://github.com/UMass-Embodied-AGI/CoELA).
이는 전체 CoELA 재현표가 아니라 현재 구현을 설명하는 차이표다.
기존 모듈을 이름만 맞춰 원본 구현이라고 부르지 않는다.

## 첫 재사용 대상: LeRobot ACT

`requirements-reference-act.txt`는 공식 LeRobot의 커밋
`89236ea0f4f81a81ca566081e20dd1ff5f823cbe`를 고정한다.
`harness/reference_act.py`는 공식 `ACTPolicy`를 직접 호출하고 신경망을 복제하지 않는다.
해당 버전은 `observation.state` 없이 두 RGB 입력을 설정할 수 있다.
다만 실제 학습/추론 시험에서 두 곳이 없는 state의 `.device`를 읽는 upstream
오류를 확인했다. `scripts/patch_reference_act.py`가 원본 파일 SHA를 검증하고
두 device 참조만 모델 parameter의 device로 바꾼다. 네트워크 구조나 입력을
추가하지 않는다. 패키지를 재설치하면 되돌릴 수 있다. 따라서 정확한 표기는
**공식 ACT + 두 줄 device 호환 패치**이며, 무수정 공식 실행 성공이 아니다.
일반 ACT 설명에 등장하는 관절값은 여기서 제공하지 않는다.
출처: [고정 버전 설정 코드](https://github.com/huggingface/lerobot/blob/89236ea0f4f81a81ca566081e20dd1ff5f823cbe/src/lerobot/policies/act/configuration_act.py).

이번은 공개 모델 연결을 확인하는 소규모 실험이다. RGB 96×96 전처리,
ResNet18 무작위 초기화, 축소 Transformer(64차원/encoder 2층), 4개 명령 chunk,
매번 새 영상으로 첫 명령만 실행, VAE KL 가중치 1을 사용한다.
원본 ACT의 학습 규모·성능 재현이나 최적 설정이 아니다. 원본 카메라 위치/FOV는
바꾸지 않으며 입력 영상 축소는 모델 전처리다.

## 비교 계약

- 같은 교사 데이터의 25개 사례를 20개 학습/5개 시험으로 미리 분할한다.
  두 로봇과 한 궤적의 모든 프레임은 같은 분할에 둔다. 시험 프레임으로 PCA,
  학습 범위, 정규화, early stopping, 임계값을 맞추지 않는다.
- 기존 커널을 학습 사례 전체로 다시 학습한다. 기존 400개 표본 설정과 달라
  예전 19/20 기록과 직접 합산하지 않는다. ACT와 같은 RGB/label 전체를 사용한다.
- 양쪽 모두 동일한 학습용 목표 RGB와 정지 label을 하나 추가한다.
  ACT의 미래 action label은 같은 학습 궤적 안에서만 구성한다.
- ACT 입력은 own/top RGB뿐이다. forward와 stop은 교사 학습 표적이며
  실행 입력에 넣지 않는다. 명령 history도 이번 모델은 사용하지 않는다.
- 출력은 전진 명령 0..0.15와 정지 score다. score≥0.65, 전진≤0.003일 때
  준비 후보로 보고 실행기가 정지 명령 후 새 RGB 두 장으로 재확인한다.
- 기존 커널은 학습 범위 밖 입력을 거부한다. ACT에는 그 기능이 없고 finite/output
  bound 검사만 있다. 거부 수와 false-ready를 기록하며 같은 OOD 보장을 주장하지 않는다.
- 준비/완료를 실제 접촉·성공 판정과 혼동하지 않는다. 물리 평가는 실행 뒤 별도 기록한다.

## 실행

시뮬레이션 Python에는 ML 패키지를 설치하지 않는다. 별도 Python 3.12 환경에
`requirements-reference-act.txt`를 설치한다. 클라우드/GPU 시뮬레이션 복원은 없다.

```sh
python3.12 -m venv /path/to/reference-act-env
/path/to/reference-act-env/bin/pip install -r requirements-reference-act.txt
/path/to/reference-act-env/bin/python scripts/patch_reference_act.py
/path/to/reference-act-env/bin/python -m pytest -q tests/test_reference_act_optional.py

# 실행 코드를 먼저 커밋한 뒤, 새 출력 디렉터리를 사용한다.
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 scripts/ugrp_session.py run reference-act -- \
  /path/to/reference-act-env/bin/python scripts/compare_reference_approach.py \
  --teacher-dir /path/to/rgb-approach-teacher-20260910-v1 \
  --protocol experiments/2026-09-16-reference-act/protocol.json \
  --out-dir outputs/reference-act-NEW
```

교사 원본은 `experiments/2026-09-10-rgb-short-approach/`의 teacher evidence ZIP과
manifest에서 복원하거나 동일 교사 실행기로 새로 생성한다. 보고서의 파일 해시로
자료 일치를 확인한다. 현재 원본 25개 사례를 재분할한 시험은 새 물리 장면 검증이 아니다.

물리 실행은 기존 `scripts/run_camera_approach_student.py`의 visual 조건에
`--act-python /path/to/reference-act-env/bin/python --act-model-dir outputs/reference-act-NEW`를
추가한다. `--approach-model-dir`도 같은 비교 출력 폴더를 사용하며, 커널 조건은 ACT
옵션을 생략한다. 파지 모델·시작 조건·명령 주기·확인 절차는 양쪽에서 같다.
각 ACT subprocess는 로봇별로 모델을 한 번 읽고 RGB만 받아 예측하며 실행기 종료 시 정리된다.

## 다음 채택 기준

첫 실행 결과는 [ACT 비교 기록](../experiments/2026-09-16-reference-act/README.md)에 있다.
소규모 실제 접근·파지는 커널 3/3, ACT 0/3이므로 이번 설정은 기본으로 채택하지 않았다.

기본 제어기는 이번 비교만으로 바꾸지 않는다. 작은 학습 실행이 실패해도 보존한다.
실제 접근·파지 성공률, false-ready/범위 이탈, 시간·명령 수·학습 연산비를 함께 보고,
새 조건 반복 결과가 나아진 후보만 별도 PR로 채택한다. 현재 단계 실행 계약의
합성 준비/완료 판정은 실제 RGB 판별기의 성능을 입증하지 않는다.
