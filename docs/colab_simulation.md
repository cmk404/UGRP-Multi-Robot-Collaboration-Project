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

### 결과 회수 오류와 실행 수명 분리

`scripts/resume_colab_comparison.py`는 명시한 런타임 한 개에서 유한한 비교를
실행한다. 회수기의 파일/통신 오류나 종료 코드는 런타임 종료 조건이 아니다.
원격 실행 완료와 회수 완료가 모두 확인됐거나, 처음 정한 최대 시간이 끝났을
때에만 소유 세션을 종료한다. 회수 불가를 실행 중이나 완료로 표시하지 않는다.

`scripts/colab_live_contents.py`는 Colab CLI의 현재 할당 목록에서 **같은 endpoint**의
전송 인증을 만료 전에 갱신한다. 다른 런타임으로 전환하거나 새 VM을 할당하지
않는다. 로그에는 토큰/전체 인증 URL 대신 작업 종류·상대 파일 경로·오류 종류를
남긴다. 누락 파일은 제한 시간 내 재조회하고, 체크포인트 무결성 오류는 별도로
차단한다.

재개 목록은 기존 ZIP·내부 파일 해시·실험 SHA·전체 조건 목록을 검증해 만든다.
`scripts/run_frozen_skill_resume.py`는 고정된 기존 actor checkout을 그대로 사용하고
완료된 성공/실패 모두를 건너뛴다. 제어 스크립트 SHA/해시, 원래 프로토콜,
이전 결과와 새 실행 범위를 별도로 기록한다. 중단된 물리 상태를 이어 붙이지는
않으며 완료 결과가 없는 조건만 처음부터 수행한다. 기존·재개 결과 전체를
회수하기 전에는 전체 비교 완료로 보고하지 않는다.

ACT의 `requirements-reference-act.txt`는 시뮬레이션과 NumPy/OpenCV 버전이 달라 별도 환경에 설치한다. `scripts/patch_reference_act.py` 적용 후 CUDA·실제 forward/backward·checkpoint를 확인하고 학습 실행기의 Python 경로 인자를 명시한다. 기존 ACT 입력 비교/재개 코드는 [PR #84](https://github.com/kcm0127-dotcom/ugrp/pull/84)의 별도 작업이다.

실제 LLM 운반에는 Colab에서 접근 가능한 인증된 모델 endpoint와 입력 가중치/데이터가 필요하다. Mac `127.0.0.1`은 원격에서 접근할 수 없다. endpoint·인증을 임의로 공개하거나 키를 로그에 기록하지 않는다. 기본 CLI 전송·무료 데모는 GitHub 키와 모델 키가 필요 없다. 모델 호출·학습·EGL·전체 코호트는 별도 실행 검증 대상이다.

## Kaggle CLI 병행

[전용 Kaggle CLI 실행 안내](kaggle_simulation.md)를 따른다. `scripts/kaggle_simulation_cli.py`의 prepare → submit → status → collect가 private Dataset/CPU kernel 제출과 결과 검증을 담당한다. CLI 설치·로컬 테스트와 실제 Kaggle 인증/원격 실행은 구분한다.

## 검증 범위

로컬 관련 테스트 12개와 최초 CI 4개 job이 통과했다. CLI 경로 수정 후 검증은 PR의 최신 head와 연결된 CI 및 로컬 `outputs/` 기록을 따른다. 세션 조회·생성·원격 Python 출력은 실제 Colab CLI에서 확인했다. [실제 Colab CPU 진단](../experiments/2026-09-21-colab-cli-smoke/README.md)에서 물리 이동 0.138548m·카메라 12프레임·결과 다운로드·전체 파일 해시가 확인되었고 소유 세션도 종료했다. 교사/학생·정적 지도 경계, 카메라 FOV와 weld OFF는 기존 지침을 유지한다.

## Mac 모델 연결을 사용하는 비공개 Colab 비교

`run_jev_skill_cohort.py --model-mailbox /content/<작업>/mailbox`는 인증된 Colab Contents 파일 전송으로 모델 요청/응답만 전달한다. Mac의 `relay_colab_models.py`가 고정된 Jev API와 기존 loopback Gemini 서비스에 호출한다. 외부 공개 포트·터널은 만들지 않는다. Jev 키는 Mac 키체인(`ugrp.typesafe.ai` / `jev`)에서 프로세스 메모리로만 읽으며 원격 소스·큐·로그에 넣지 않는다. `scripts/macos_model_keychain.swift`로 빌드한 helper의 읽기 출력은 relay가 직접 캡처한다. 터미널에서 helper의 읽기 출력을 표시하지 않는다.

relay는 Colab CLI가 설치된 Python에서 `--session`, `--remote`, `--keychain-helper`, `--output`, `--seconds`를 지정해 `ugrp_session.py run`으로 실행한다. 최대 4시간/지정 호출 수까지만 동작하며 종료 시 해당 소유 세션을 정리한다. 요청마다 30초 한도와 고유 ID를 사용하고, 응답 전송 재시도 시 모델 호출을 반복하지 않는다. 임의 URL·모델·만료된 요청을 거부한다.

`latency_s`는 파일 전송을 포함한 실제 대기 시간이며 `provider_latency_s`가 API 응답 시간이다. 이 경로의 continuous 결과를 직접 HTTP 경로의 지연 성능과 합쳐 비교하지 않는다. 동일 소스·동일 Colab 환경에서 rule/Jev/Gemini를 함께 실행하며, 다른 Kaggle/과거 Mac 결과는 별도 실험으로 둔다.

### T4가 배정됐지만 실제 렌더러가 llvmpipe인 경우

`nvidia-smi`의 GPU 배정은 MuJoCo 렌더링 사용 증거가 아니다. Colab의 기존
`/usr/lib64-nvidia` 라이브러리와 GLVND 등록이 누락된 경우에만 다음 명령으로
등록하고 새 프로세스의 실제 OpenGL vendor/renderer를 확인한다. 기존의 다른
등록 내용은 덮어쓰지 않는다. 드라이버 설치나 실행 중 실험 변경은 하지 않는다.

```sh
python scripts/setup_colab_egl.py --python /content/ugrp-repair/sim-env/bin/python \
  --output /content/egl-verification-NEW.json
```

검증된 새 실행에만 출력의 `environment` 세 값(`MUJOCO_GL`,
`PYOPENGL_PLATFORM`, `__EGL_VENDOR_LIBRARY_FILENAMES`)을 적용한다. renderer가
NVIDIA인 것을 확인한 후 같은 소스·조건으로 새 코호트를 시작한다. CPU/OSMesa
결과와 NVIDIA 결과는 렌더러가 다른 진단으로 구분한다. GPU 영상에도 다른 JPEG
색 무늬가 있으므로 이전 CPU 프레임 재생만으로 GPU 제어 검증을 대신하지 않는다.
