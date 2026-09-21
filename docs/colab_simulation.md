# Colab CLI 시뮬레이션·평가

2026-09-21 사용자 요청: 시뮬레이션·렌더링·교사 데이터 생성·학습·평가는 **Colab CLI**로 실행한다. 브라우저 노트북·Colab Secrets·GitHub 토큰 등록을 기본 절차로 요구하지 않는다. Kaggle CLI는 추가 배치 실행 대상으로 사용할 수 있다. 코드 편집·단위 테스트·Git·결과 검토와 실물 MasterPi 연결은 로컬에 남긴다.

## Colab CLI에서 실행

이 Mac의 `colab`은 `google-colab-cli 0.6.0`이며, 기존 OAuth 인증으로 세션 목록 조회·CPU 세션 생성·원격 Python 실행을 확인했다. 명령과 플래그는 설치 버전의 `colab --help`, `colab skill`을 우선한다. 내장 안내의 인증 기본값과 실제 help가 다를 수 있으므로 이미 동작하는 인증을 임의로 바꾸지 않는다.

```sh
colab sessions
colab new -s ugrp-sim-작업ID
colab status -s ugrp-sim-작업ID

python3 scripts/colab_simulation_cli.py \
  --session ugrp-sim-작업ID --output outputs/colab-작업ID
```

`new`는 새 CPU 런타임을 만든다. 이미 본 작업이 소유한 세션이 있으면 상태를 확인하여 재사용한다. 다른 작업의 busy 세션에서 실행하거나 세션을 재시작하지 않는다. GPU가 필요한 승인된 학습 작업에서만 해당 세션을 `colab new -s <이름> --gpu T4`처럼 지정한다. 사용량·할당 가능 여부는 계정에 따라 다르며 유료 플랜/컴퓨팅 유닛을 자동 구매하지 않는다.

위 프로젝트 실행기는 아래를 순서대로 수행한다.

1. 추적 파일의 미커밋 변경을 거부하고 HEAD의 **실제 Git SHA를 보존하는 shallow sparse checkout**을 만든다. 실행 코드·설정·테스트와 작은 실험 JSON만 포함한다. `.env`, 토큰, 기존 로컬 환경과 raw 출력은 복사하지 않는다. 업로드용 Git remote도 제거한다.
2. `source-manifest.json`에 포함/제외 파일과 압축본 해시를 기록한다. 큰 과거 증거·모델은 기본 제외한다. 필요한 추적 파일은 `--include 경로`로 추가하며, 로컬 전용 학습 데이터·가중치는 별도 `colab upload`와 해시 검증을 거친다. 포함하지 않은 Git 객체·과거 커밋을 읽는 감사는 원본 로컬 checkout에서 수행한다.
3. `colab upload`로 소스를 전송하고 `colab exec -f`로 원격 해시·압축 경로를 검증한다. 매번 고유한 `/content/ugrp-cli-<ID>` 아래에 풀고 Python 3.12 시뮬레이션 환경을 만든다. Mac 환경을 복사하거나 Colab 커널 패키지를 교체하지 않는다.
4. CPU OSMesa로 무료 물리·카메라 데모를 실행한다. 다른 실행기는 `--module`과 `--` 뒤의 인자로 선택한다. `{output}`은 원격의 새로운 결과 경로다.
5. 정상·실패 모두 작업별 로그·소스 SHA·패키지·종료 코드·파일 해시·ZIP을 보존한다. `colab download`로 로컬에 회수하여 ZIP과 내부 모든 파일 해시, 소스 SHA를 검증한다. 결과의 종료 코드로 로컬 명령도 종료한다.

다른 조건 예시:

```sh
python3 scripts/colab_simulation_cli.py \
  --session ugrp-sim-작업ID --output outputs/colab-traffic-작업ID \
  --module scripts.run_rgb_traffic --timeout 1800 \
  -- --scenario crossing --out-dir '{output}'
```

로컬 `verified-result.json`은 회수·해시·프로세스 종료 검증이다. 로봇 과제 성공은 결과 내부 평가·영상·입력 감사를 따로 검토한다. 성공률·시간·행동 수·비용과 전체 실패는 `experiments/<ID>/`에 기록한다. raw ZIP의 로컬 보관은 원격 백업이 아니다.

## 정리·복구

결과 회수 후 **자신이 만든 세션만** 정리한다.

```sh
colab stop -s ugrp-sim-작업ID
colab sessions
```

프로젝트 실행기는 세션을 임의 생성·중지하지 않는다. `colab exec`의 연결/timeout 실패 시 자동 재실행하지 않고 세션·원격 경로를 남긴다. 로컬 `remote.json`, `setup.py`, `job.py`를 이용해 `colab status`, `colab ls`, `colab exec`, `colab download`로 상태와 결과를 먼저 확인한다. 아직 실행 중인 작업을 중복 실행하지 않는다. 필요한 경우 그 원격 checkout의 `scripts/ugrp_session.py stop colab-cli-job`으로 자신이 시작한 작업만 정리한다.

Colab VM이 삭제되면 미회수 파일은 사라질 수 있다. 일반 시뮬레이션의 중간 물리 상태 복원은 제공하지 않는다. 조건별로 유한 실행·회수하고 실패/미완료 조건을 새 출력 폴더에서 실행한다. 자동 재접속·Drive 사용·상시 브리지·기존 Mac 프로세스 중단은 하지 않는다. [공식 Colab FAQ](https://research.google.com/colaboratory/faq.html).

## 학습과 모델 연결

ACT의 `requirements-reference-act.txt`는 시뮬레이션과 NumPy/OpenCV 버전이 달라 별도 환경에 설치한다. `scripts/patch_reference_act.py` 적용 후 CUDA·실제 forward/backward·checkpoint를 확인하고 학습 실행기의 Python 경로 인자를 명시한다. 기존 ACT 입력 비교/재개 코드는 [PR #84](https://github.com/kcm0127-dotcom/ugrp/pull/84)의 별도 작업이다.

실제 LLM 운반에는 Colab에서 접근 가능한 인증된 모델 endpoint와 입력 가중치/데이터가 필요하다. Mac `127.0.0.1`은 원격에서 접근할 수 없다. endpoint·인증을 임의로 공개하거나 키를 로그에 기록하지 않는다. 기본 CLI 전송·무료 데모는 GitHub 키와 모델 키가 필요 없다. 모델 호출·학습·EGL·전체 코호트는 별도 실행 검증 대상이다.

## Kaggle CLI 병행

이 Mac에 `kaggle 2.2.4`를 독립 uv tool로 설치했다. 계정 인증과 원격 실행은 아직 확인하지 않았다. CLI 로그인 진입점은 다음과 같다. 로그인 URL의 사용자 계정 인증은 사용자가 완료하며 토큰을 채팅이나 저장소에 붙여 넣지 않는다.

```sh
kaggle auth login --no-launch-browser
kaggle quota
kaggle kernels init -p outputs/kaggle-작업ID
kaggle kernels push -p outputs/kaggle-작업ID
kaggle kernels status 계정/실험이름
kaggle kernels output 계정/실험이름 -p outputs/kaggle-results-작업ID
```

`kernels push`는 Python 스크립트 또는 ipynb를 올려 배치 실행한다. `kernel-metadata.json`은 **`is_private: true`**로 설정하며 GPU/인터넷은 필요한 경우에만 활성화한다. 입력 코드는 기본으로 파일 하나만 전달되므로 저장소·학습 데이터는 비공개 Dataset 등의 입력 자료로 따로 제공하고 해시를 검증해야 한다. Colab용 실행기를 그대로 Kaggle API에 연결했다고 주장하지 않는다. Kaggle 계정 인증 후 별도 private kernel의 설치·물리·결과 회수 진단을 먼저 수행한다. 공개 업로드는 이 작업의 범위가 아니다.

Kaggle은 여러 조건을 묶은 배치 실행에, Colab CLI는 같은 세션에서 반복 진단하는 작업에 쓰는 운영 구성이 적합하다. GPU 종류/할당량은 계정의 실제 `quota`와 제출 결과로 확인한다. 설치된 2.2.4의 `kernels push --help`에는 웹 문서의 `--no-run`이 없으므로 이를 사용하지 않는다.

참고: [Kaggle 공식 CLI](https://github.com/Kaggle/kaggle-cli), [실행·상태·결과 명령](https://github.com/Kaggle/kaggle-cli/blob/main/docs/kernels.md), [private kernel 메타데이터](https://github.com/Kaggle/kaggle-cli/blob/main/docs/kernels_metadata.md).

## 검증 범위

로컬 관련 테스트 12개와 최초 CI 4개 job이 통과했다. CLI 경로 수정 후 검증은 PR의 최신 head와 연결된 CI 및 로컬 `outputs/` 기록을 따른다. 세션 조회·생성·원격 Python 출력은 실제 Colab CLI에서 확인했다. [실제 Colab CPU 진단](../experiments/2026-09-21-colab-cli-smoke/README.md)에서 물리 이동 0.138548m·카메라 12프레임·결과 다운로드·전체 파일 해시가 확인되었고 소유 세션도 종료했다. 교사/학생·정적 지도 경계, 카메라 FOV와 weld OFF는 기존 지침을 유지한다.
