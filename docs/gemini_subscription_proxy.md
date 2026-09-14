# 각자 PC에서 Gemini 로그인 프록시 사용하기

**팀원마다 자기 Ubuntu PC에 프록시를 설치하고 자기 Google 계정으로 로그인한다.** 개발자의 Mac이 켜져 있거나 개발자 계정의 프록시가 실행 중일 필요가 없다. 계정·인증 파일·프록시 서비스를 팀원끼리 공유하지 않는다.

```text
본인 PC의 UGRP → 본인 PC의 127.0.0.1:8391 → 본인 Google 로그인 → Gemini
```

기존 안내에 있던 개발자 Mac으로의 SSH 터널은 이 목적에 맞지 않아 제거했다. `127.0.0.1`은 명령을 실행하는 각자의 컴퓨터다.

## 사용할 서버

UGRP 저장소의 `harness/gemini_proxy.py`는 HTTP 클라이언트다. 서버는 별도로 설치한다. 여기서는 공개 도구 [CLIProxyAPI v7.2.155](https://github.com/router-for-me/CLIProxyAPI/releases/tag/v7.2.155)의 Antigravity OAuth 로그인을 사용한다. 개발자 Mac의 개인 Python 어댑터와는 다른 구현이며, 개인 스크립트·인증·프로젝트 설정을 복사할 필요가 없다. Google 공식 배포 도구는 아니다.

이 경로는 별도 Gemini API 키 대신 본인의 Google 로그인을 사용한다. 구독 이름만으로 Antigravity 접근 권한이나 특정 모델의 사용 가능 여부가 보장되지는 않는다. 본인 계정으로 로그인하고 실제 응답을 확인해야 한다.

## 1. Ubuntu 24.04 x86_64에 설치

먼저 [Ubuntu 기본 설치와 무료 데모](ubuntu_quickstart.md)를 완료한다. 아래 명령은 UGRP 저장소 루트에서 실행한다. 같은 버전이 이미 설치됐다면 재설치하지 않아도 된다.

```bash
sudo apt-get install -y curl ca-certificates
ugrp_proxy_dir="$HOME/.local/share/ugrp/gemini-proxy"
mkdir -p "$ugrp_proxy_dir/bin" "$ugrp_proxy_dir/auth"
chmod 700 "$ugrp_proxy_dir" "$ugrp_proxy_dir/auth"
(
  set -eu
  test "$(uname -m)" = x86_64
  cd "$ugrp_proxy_dir/bin"
  release='https://github.com/router-for-me/CLIProxyAPI/releases/download/v7.2.155'
  archive='CLIProxyAPI_7.2.155_linux_amd64.tar.gz'
  curl -fL "$release/$archive" -o "$archive"
  curl -fL "$release/checksums.txt" -o checksums.txt
  grep " $archive$" checksums.txt | sha256sum --check --strict -
  tar -xzf "$archive"
  test -x ./cli-proxy-api
)
# 처음 설정할 때만 복사: 이미 있는 개인 설정은 보존한다.
if [ ! -f "$ugrp_proxy_dir/config.yaml" ]; then
  cp configs/gemini-proxy.example.yaml "$ugrp_proxy_dir/config.yaml"
fi
```

[설정 예제](../configs/gemini-proxy.example.yaml)는 `127.0.0.1:8391`에만 바인딩하고 인증을 본인 홈 디렉터리에 저장한다. UGRP 클라이언트는 Authorization 헤더를 보내지 않으므로 로컬 HTTP API 키는 비워 둔다. 이 설정은 같은 PC의 프로세스가 접근할 수 있으므로 개인 PC용이다. `host`를 외부 공개 주소로 바꾸지 않는다. 관리 패널과 자동 계정·모델 전환도 사용하지 않는다.

## 2. 본인 Google 계정으로 로그인

```bash
ugrp_proxy_dir="$HOME/.local/share/ugrp/gemini-proxy"
"$ugrp_proxy_dir/bin/cli-proxy-api" \
  --config "$ugrp_proxy_dir/config.yaml" --antigravity-login
```

열린 브라우저에서 **본인 계정**을 선택하고 안내를 완료한다. 브라우저 자동 실행이 안 되면 명령에 `--no-browser`를 추가하고 표시된 URL을 같은 PC의 브라우저에서 연다. 로그인 콜백은 기본 51121 포트를 사용한다. WSL에서는 Windows 브라우저에서 WSL의 localhost 콜백에 접근 가능한지도 확인한다. 이 포트는 로그인용이며 UGRP 요청 포트 8391과 다르다.

토큰은 본인의 `~/.local/share/ugrp/gemini-proxy/auth`에 저장된다. 이 디렉터리나 토큰을 Git·PR·메신저에 올리지 않는다. 로그인 오류는 본인 계정 권한과 CLIProxyAPI 로그인 출력을 기준으로 확인한다.

## 3. 필요할 때만 로컬 서버 실행

UGRP 저장소 루트의 터미널 1에서:

```bash
ugrp_proxy_dir="$HOME/.local/share/ugrp/gemini-proxy"
.venv-dev/bin/python scripts/ugrp_session.py run gemini-proxy -- \
  "$ugrp_proxy_dir/bin/cli-proxy-api" --config "$ugrp_proxy_dir/config.yaml"
```

터미널 2도 저장소 루트에서 열고:

```bash
export GEMINI_PROXY_URL='http://127.0.0.1:8391/v1/chat/completions'
curl --fail --silent --show-error http://127.0.0.1:8391/v1/models \
  | .venv-dev/bin/python -m json.tool
```

응답의 `data`에서 사용할 모델 ID를 확인한다. 빈 배열이면 서버는 실행됐지만 사용 가능한 계정·모델이 등록되지 않은 상태다. CLIProxyAPI에서 확인하는 경로는 `/v1/models`이며, 기존 개인 서버의 `/health` 응답을 기대하지 않는다.

**모델 목록 응답만으로 로그인 유효성·실제 생성·남은 할당량은 입증되지 않는다.** 이 버전의 Antigravity 카탈로그에서 3.8 모델 ID는 `gemini-3.8-flash-high`다. 아래 명령은 이 ID가 본인 목록에도 있을 때 사용한다. 기존 개인 프록시의 `gemini-3.8-flash` 별칭과는 다르므로, 시드 실행기에 `--model`로 정확한 ID를 전달한다. 선택한 모델은 결과 manifest에도 기록된다. 동일한 과거 모델 조건이라고 간주하지 않는다.

## 4. 실제 응답과 한 시드 확인

`gemini-3.8-flash-high`가 목록에 있는 본인 계정에서, 실제 모델 사용량을 소비하는 짧은 요청:

```bash
curl --fail-with-body --silent --show-error --max-time 90 \
  "$GEMINI_PROXY_URL" -H 'Content-Type: application/json' \
  -d '{"model":"gemini-3.8-flash-high","messages":[{"role":"user","content":"Reply with OK."}],"max_tokens":32}'
```

`choices` 안의 모델 응답을 확인한 다음 [Ubuntu 안내의 한 시드 실행과 결과 검증](ubuntu_quickstart.md#4-실제-gemini-운반-실험--선택-사항)을 따른다. 텍스트 응답 성공은 카메라 입력이나 운반 성공의 증거가 아니므로 결과와 영상을 함께 검토한다. 실험에 사용한 코드 SHA, 프록시 버전, 모델 이름과 실패 결과도 보존한다.

작업이 끝나면 터미널 1에서 `Ctrl-C`로 프록시 세션을 종료한다. 다른 터미널에서는 다음 명령으로 이 세션만 종료할 수 있다.

```bash
.venv-dev/bin/python scripts/ugrp_session.py stop gemini-proxy
```

## 오류 구분과 검증 범위

| 증상 | 확인 |
|---|---|
| 연결 거부 | 본인 PC에서 프록시 세션이 실행 중인지, 설정 포트가 8391인지 |
| `data: []` 또는 모델 없음 | 본인 계정 로그인 완료 여부, 설정의 auth-dir, 지원 모델 ID |
| 401·403·로그인 갱신 실패 | 본인 계정 권한과 로그인 상태; 토큰을 공유하지 않고 본인 PC에서 재로그인 |
| 429·시간 초과 | 본인 계정 할당량과 모델 지연; 다른 사람 계정으로 우회하지 않음 |
| 포트 사용 중 | 이미 실행된 본인 프록시인지 확인; 관련 없는 프로세스를 종료하지 않음 |

자동 검증은 고정 릴리스의 다운로드·체크섬·빈 인증 디렉터리로 서버 실행·무키 `/v1/models` 응답·종료까지 확인한다. 새 Google 계정 OAuth 로그인, 실제 Gemini 생성, 이미지 입력과 운반은 자동 검증에 포함하지 않는다. 개발자의 기존 프록시와 동일한 모델 매핑·성능을 검증한 것도 아니다.

참고: [공식 Antigravity 로그인 안내](https://help.router-for.me/configuration/provider/antigravity), [고정 버전의 설정 예제](https://github.com/router-for-me/CLIProxyAPI/blob/v7.2.155/config.example.yaml). 여기서 '공식'은 CLIProxyAPI 프로젝트 문서를 뜻한다.
