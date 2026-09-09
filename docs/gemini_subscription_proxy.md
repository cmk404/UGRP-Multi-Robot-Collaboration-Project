# 현재 사용하는 Gemini 구독 프록시 연결 방식

2026-09-09 현재 개발자의 실행 중인 서비스와 소스를 확인한 기록이다. **현재 방식을 설명하는 문서이며, 공식 Gemini API 키 방식으로 전환하는 안내가 아니다.**

## 연결 구조

```text
UGRP 시뮬레이션 / GeminiTransportPlanner
    ↓ GeminiProxyCompleter: 텍스트 + 카메라 이미지
http://127.0.0.1:8391/v1/chat/completions
    ↓ 개인 gemini_subscription_proxy.py
저장된 Google OAuth 로그인 인증 + 구독 계정의 모델 사용 권한
    ↓ 모델별 백엔드 요청
Gemini 응답 → OpenAI 호환 응답·토큰 사용량 → UGRP 행동 실행
```

UGRP에는 [harness/gemini_proxy.py](../harness/gemini_proxy.py)의 클라이언트가 들어 있다. 이 클라이언트는 `GEMINI_PROXY_URL`로 주소를 정하고, 현재 요청에는 별도 API 키나 Authorization 헤더를 붙이지 않는다. **Google 인증은 프록시 서버가 담당한다.**

실행 중인 프록시는 `~/.local/bin/gemini_subscription_proxy.py`이고, `~/.gemini/antigravity_creds.json`의 OAuth 인증을 읽고 갱신한다. 로컬 로그는 `~/.hermes/logs/gemini-subscription-proxy.log`에 남는다. 이 경로는 프록시를 운영하는 컴퓨터의 홈 디렉터리 기준이다. 현재 Mac에서는 기존 launchd 서비스가 이를 실행하고 있다.

이 개인 서버 소스와 로그인 파일은 UGRP 저장소에 포함돼 있지 않다. **저장소를 clone하는 것만으로 동일한 프록시 서버가 설치되지는 않는다.** 인증 파일이나 개인 설정을 복사해서 공유하는 방식으로 설명하지 않는다.

## API 키와 로그인 인증의 구분

| 항목 | 현재 방식 |
|---|---|
| UGRP에서 설정하는 값 | `GEMINI_PROXY_URL` (전체 chat completions URL) |
| UGRP의 `GEMINI_API_KEY` | 현재 클라이언트는 읽지 않음 |
| 프록시 → Google 인증 | 프록시 소유자의 저장된 OAuth 로그인 인증 |
| 사용량의 주체 | 프록시에 로그인된 계정의 사용 권한·할당량 |
| 별도 프록시 API 키 | 현재 로컬 HTTP 서버에는 클라이언트 키 검증이 없음 |

다른 도구의 OpenAI 호환 설정 화면이 API 키 입력을 요구하더라도, 그 입력란이 현재 프록시의 Google 로그인 인증을 대신하지 않는다. UGRP 자체에는 임의의 키를 추가할 필요가 없다.

구독 이름만으로 다른 계정에도 같은 모델·할당량이 제공된다고 단정할 수 없다. 현재 프록시의 주석상 기반은 Google One AI Pro / Antigravity이지만, 실제 계정 권한은 별도 확인 대상이다.

## 현재 모델 매핑

개인 프록시 소스에 설정된 매핑이다. `/v1/models`는 이 목록을 반환하며, Google 측 실시간 모델 사용 권한을 조회한 결과는 아니다.

| UGRP 요청 모델 | 프록시가 요청하는 모델 | 백엔드 호스트 |
|---|---|---|
| `gemini-3.1-pro-preview` | `gemini-3.1-pro-low` | `cloudcode-pa.googleapis.com` |
| `gemini-3.7-flash` | `gemini-3.7-flash-tiered` | `daily-cloudcode-pa.googleapis.com` |
| `gemini-3.8-flash` | `gemini-3.8-flash-medium` | `daily-cloudcode-pa.googleapis.com` |

현재 [시드 검증 실행기](../scripts/run_gemini_seed_validation.py)는 `gemini-3.8-flash`를 요청한다. `/health`의 기본 `model` 필드가 `gemini-3.1-pro-preview`여도, 개별 요청의 모델 이름이 3.8이면 위 3.8 매핑을 사용한다. 이 별칭을 공식 Gemini API의 모델 이름과 자동으로 동일시하지 않는다.

프록시는 OpenAI 형식의 `messages`, `image_url`에 들어 있는 base64 카메라 이미지, `reasoning_effort`, 토큰 사용량을 변환한다. 현재 이미지 입력은 `data:image/...;base64,...` 형식이며 원격 이미지 URL을 가져오는 방식은 지원하지 않는다.

## Ubuntu에서 연결하기

먼저 [Ubuntu 기본 설치와 무료 데모](ubuntu_quickstart.md)를 완료한다. 아래 두 경우 중 프록시 위치에 맞는 경로를 사용한다.

### A. 같은 Ubuntu 컴퓨터에 호환 프록시가 이미 준비된 경우

본인 계정으로 동작하는 서버 소스와 로그인 절차가 준비돼 있어야 한다. 이 문서는 새 계정의 OAuth 로그인·서버 설치 프로그램을 제공하지 않는다. 기존 개인 소스를 단순 복사하면 계정별 프로젝트 설정이나 인증이 맞지 않을 수 있다.

서버가 `127.0.0.1:8391`에서 실행되고 있다면, UGRP 저장소 루트에서:

```bash
export GEMINI_PROXY_URL='http://127.0.0.1:8391/v1/chat/completions'
curl --fail --silent --show-error http://127.0.0.1:8391/health
curl --fail --silent --show-error http://127.0.0.1:8391/v1/models
```

이미 실행 중인 서버를 중복 실행하지 않는다. 별도로 준비된 서버를 UGRP 작업용으로 처음 시작할 때는 프로젝트의 세션 관리 절차를 따른다. Mac의 기존 launchd 서비스는 Ubuntu 설치 방법이 아니다.

### B. 허가받은 기존 프록시 컴퓨터에 연결하는 경우

현재 개발자 Mac의 서버는 `127.0.0.1`에만 바인딩돼 있어 다른 컴퓨터에서 Mac IP의 8391 포트로 직접 접속할 수 없다. 관리자가 사용을 허가하고 SSH 접속을 준비해 준 경우, **SSH 로컬 포트 전달**로 연결한다. 서버를 공개 주소에 노출하거나 인증 파일을 옮길 필요가 없다.

Ubuntu 터미널 1에서 다음의 `SSH_USER`, `PROXY_HOST`를 관리자가 제공한 SSH 계정과 호스트로 바꾼다. 이 명령은 SSH 접근 권한을 새로 만들어주지는 않는다.

```bash
ssh -N -T -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18391:127.0.0.1:8391 SSH_USER@PROXY_HOST
```

이 터미널을 유지한다. Ubuntu 터미널 2에서:

```bash
cd ~/projects/ugrp
export GEMINI_PROXY_URL='http://127.0.0.1:18391/v1/chat/completions'
curl --fail --silent --show-error http://127.0.0.1:18391/health
curl --fail --silent --show-error http://127.0.0.1:18391/v1/models
```

18391은 Ubuntu의 로컬 포트이고, 8391은 원격 프록시 컴퓨터의 로컬 포트다. 실험이 끝나면 터미널 1에서 `Ctrl-C`로 터널을 종료한다. 여러 사람이 같은 프록시를 사용하면 같은 계정의 할당량과 지연에 영향을 주므로 관리자와 실행 시간·예산을 맞춘다.

이번 문서 작업에서 SSH 접속을 개설하거나 프록시 공유 권한을 변경한 것은 아니다.

## 연결 확인 후 한 시드 실행

`/health`의 정상 응답은 HTTP 서버가 떠 있다는 뜻이다. `/v1/models`는 등록된 별칭 목록이다. **두 GET 요청만으로 로그인 유효성·상위 모델 응답·남은 할당량이 검증되지는 않는다.**

관리자에게 모델 사용을 허가받고 예산을 확인한 뒤, 앞에서 설정한 `GEMINI_PROXY_URL`을 유지한 같은 Ubuntu 터미널에서:

```bash
export MUJOCO_GL=osmesa
.venv-dev/bin/python scripts/ugrp_session.py run gemini-trial -- \
  .venv-dev/bin/python -m scripts.run_gemini_seed_validation \
  --execute --output outputs/gemini-proxy-trial-01 --seeds 45 \
  --reasoning-effort medium --request-timeout 60
```

시드당 설정 예산은 최대 30회 모델 호출·120,000 입력 토큰·300 SIM초다. 실제 모델 호출은 프록시 계정의 사용량을 소비하며, 입력 토큰 추정은 제공자 측 과금의 절대 상한이 아니다. 새 결과 폴더명을 사용하고 실행 중 코드를 바꾸지 않는다.

[Ubuntu 안내의 결과 검증 단계](ubuntu_quickstart.md#4-실제-gemini-운반-실험--선택-사항)를 따를 때 출력 경로는 `outputs/gemini-proxy-trial-01/solo-45`로 맞춘다. 모델 응답에 성공했다고 실제 운반까지 성공한 것은 아니므로 결과와 영상을 함께 검토한다.

## 오류를 구분하는 방법

| 증상 | 확인할 곳 |
|---|---|
| 연결 거부 | 같은 컴퓨터인지, SSH 터널이 유지되는지, Ubuntu 쪽 포트가 8391/18391 중 어느 것인지 |
| `/health`는 성공하지만 모델 요청 실패 | 프록시의 로그인 갱신, 계정 권한, 상위 서버 오류를 운영자가 확인 |
| 429 또는 시간 초과 | 계정 할당량·동시 요청·모델 지연. 자동 계정 교체로 처리하지 않음 |
| 이미지 입력 오류 | `data:image/...;base64,...` 형식인지 확인 |
| `GEMINI_API_KEY`를 넣었는데 변화 없음 | 현재 클라이언트는 그 환경변수를 읽지 않음. 위 URL 연결 구조 확인 |

오류를 공유할 때 인증 파일·토큰·전체 개인 로그를 첨부하지 않는다. 실행 코드 SHA, 모델 별칭, 오류 상태, 출력 폴더의 필요한 검증 결과를 남긴다.

## 이 문서에서 확인한 범위

개발자 Mac의 8391 리스너, 실제 실행 스크립트 경로, 프록시 소스의 인증·매핑 구조, `/health`와 `/v1/models` 응답을 직접 확인했다. 인증 파일의 내용은 수집하지 않았고 모델 생성 요청도 보내지 않았다. Ubuntu에서의 SSH 터널·새 계정 로그인·실제 모델 호출은 이번 문서 검증 범위에 포함하지 않는다.
