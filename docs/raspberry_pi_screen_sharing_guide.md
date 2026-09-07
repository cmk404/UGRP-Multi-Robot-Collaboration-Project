# Raspberry Pi 무선 화면 공유 가이드

> **구성:** Raspberry Pi `ugrp1` + WayVNC + noVNC/websockify
>
> **목적:** Mac의 Safari 또는 Chrome에서 Raspberry Pi의 GUI 화면을 열고 조작한다.
>
> **실제 성공 경로:** iPhone 개인용 핫스팟에 Mac과 Raspberry Pi를 모두 연결한 뒤, Mac에서 Pi의 `6080` 포트로 접속한다.

---

## 0. 이 문서의 기준

2026-08-14 실제 연결 기록(H-012–H-015)에서 다음을 확인했다.

- Mac과 Raspberry Pi가 같은 iPhone 핫스팟에 연결됐다.
- Raspberry Pi 호스트명은 `ugrp1`, 사용자 계정은 `ugrp1`이다.
- 당시 Pi 주소는 `172.20.10.2`였다.
- WayVNC가 VNC 포트 `5900`에서 실행됐다.
- `novnc.service`가 websockify 포트 `6080`에서 실행됐다.
- Mac Safari에서 noVNC 화면을 표시하고 조작했다.

`172.20.10.2`는 당시 DHCP가 할당한 주소일 뿐이다. 핫스팟을 다시 연결하거나 Pi를 재부팅하면 달라질 수 있으므로, 아래의 `<PI_IP>`를 현재 주소로 바꿔 사용한다.

> **인증 상태:** 현재 성공한 noVNC 구성은 `enable_auth=false`, `enable_pam=false`인 무인증 HTTP 구성이다. 개인 핫스팟 안의 임시 개발용으로만 사용한다. 같은 핫스팟에 연결된 다른 장치도 화면과 키보드를 제어할 수 있다. 장기 사용은 SSH 터널 방식(7장)을 권장한다.

이 문서는 화면 공유 절차만 다룬다. SD 카드 초기 설정과 Wi-Fi 저장은 [`raspberry_pi_headless_setup_guide.md`](./raspberry_pi_headless_setup_guide.md)를 따른다.

---

## 1. 무선 연결: iPhone 핫스팟 경로

### 1.1 iPhone에서

1. **설정 → 개인용 핫스팟**으로 이동한다.
2. **다른 사람의 연결 허용**을 켠다.
3. Pi가 연결되지 않으면 **호환성 최대화**도 켠다.
4. 핫스팟 비밀번호는 Pi와 Mac에 직접 입력한다. 비밀번호는 저장소나 로그에 기록하지 않는다.

### 1.2 Mac에서

1. Mac의 Wi-Fi를 같은 iPhone 핫스팟에 연결한다.
2. 연결된 인터페이스와 주소를 확인한다.

```bash
ipconfig getifaddr en0
```

3. mDNS가 동작하면 다음처럼 Pi를 찾는다.

```bash
ping -c 1 ugrp1.local
ssh ugrp1@ugrp1.local
```

이 프로젝트에서는 `~/.ssh/config`에 `ugrp1` 별칭을 만들어 두었으므로 다음도 사용할 수 있다.

```bash
ssh ugrp1
```

mDNS가 안 되면 iPhone 핫스팟의 연결 기기 목록 또는 현재 네트워크에서 Pi의 IP를 확인한다. 성공 당시 주소를 예로 들면 다음과 같다.

```bash
ssh ugrp1@172.20.10.2
```

이 주소를 고정 주소로 간주하지 않는다. SSH가 먼저 연결되어야 화면 공유 설정을 진행한다.

### 1.3 Mac 인터넷 공유와 혼동하지 않기

Mac의 **인터넷 공유(Internet Sharing)**는 iPhone 핫스팟과 별도의 AP 경로다. 결정 로그의 D-005/I-007은 Mac AP를 채택했던 과거 계획이고, 실제 화면 공유 성공 기록 H-012–H-015는 iPhone 핫스팟 경로다.

현재처럼 iPhone 핫스팟으로 연결할 때는 Mac의 인터넷 공유를 별도로 켜지 않는다. Mac AP로 전환할 때는 Pi가 새 SSID에 연결되어 있는지 확인하고 IP를 다시 찾은 뒤, 이 문서의 `<PI_IP>`와 포트 접속을 새로 확인한다.

---

## 2. Pi의 GUI와 WayVNC 확인

WayVNC는 화면을 새로 만들어 주는 서버가 아니라 **이미 로그인된 그래픽 세션을 공유하는 서버**다. SSH 연결만 되고 GUI 세션이 없으면 검은 화면이 나올 수 있다.

SSH로 Pi에 들어가 다음을 한 번 확인한다.

```bash
cat /etc/os-release
printf 'session=%s\n' "$XDG_SESSION_TYPE"
printf 'wayland=%s\n' "$WAYLAND_DISPLAY"
command -v wayvnc
systemctl list-unit-files '*wayvnc*'
```

다음 조건을 만족해야 한다.

- Desktop 환경이 설치되어 있다.
- 실제 GUI 세션이 로그인되어 있다.
- Wayland 세션에서 WayVNC가 실행된다.
- `command -v wayvnc`가 실행 파일을 찾는다.

이 문서의 성공 기록은 system-wide `wayvnc.service`를 사용한 환경이다. 다른 이미지에서는 user service 또는 다른 VNC backend를 사용할 수 있으므로, `wayvnc.service not found`가 나오면 같은 명령을 반복하지 말고 실제 유닛을 먼저 확인한다.

```bash
systemctl --user --no-pager --full status wayvnc
```

---

## 3. Pi에 noVNC 설치

Pi의 SSH 셸에서 실행한다.

```bash
sudo apt update
sudo apt install -y novnc websockify
```

이 명령은 noVNC와 websockify를 설치한다. WayVNC 또는 Desktop 환경까지 설치한다고 가정하지 않는다.

설치 경로가 현재 문서와 같은지 확인한다.

```bash
command -v wayvnc
command -v websockify
websockify --help | sed -n '1,60p'
test -f /usr/share/novnc/vnc.html && echo 'noVNC files OK'
```

`websockify` 또는 noVNC의 경로가 다르면 5장의 systemd 유닛에서 해당 경로를 사용한다.

---

## 4. WayVNC 설정

### 4.1 현재 실제 성공 구성

기존 설정을 먼저 백업한다.

```bash
sudo install -d -m 0755 /etc/wayvnc
if [ -f /etc/wayvnc/config ]; then
  sudo cp -a /etc/wayvnc/config "/etc/wayvnc/config.bak.$(date +%Y%m%d-%H%M%S)"
fi
```

현재 성공한 설정은 다음과 같다.

```bash
sudo tee /etc/wayvnc/config >/dev/null <<'EOF'
use_relative_paths=true
address=::
enable_auth=false
enable_pam=false
EOF
```

서비스를 재시작한다.

```bash
sudo systemctl restart wayvnc
sudo systemctl --no-pager --full status wayvnc
```

### 4.2 이 설정의 의미

| 설정 | 의미 |
| --- | --- |
| `address=::` | IPv6 wildcard 주소에 바인딩한다. IPv4 접근 가능 여부는 실제 listener와 TCP 접속으로 확인해야 한다. |
| `enable_auth=false` | VNC 인증을 사용하지 않는다. |
| `enable_pam=false` | PAM 인증을 사용하지 않는다. |
| `use_relative_paths=true` | WayVNC가 상대 경로 설정을 사용할 수 있게 한다. |

따라서 이 구성은 **비밀번호 없이 안전하게 로그인하는 기능**이 아니다. VNC `5900`이 네트워크에 노출되고, noVNC `6080`도 기본 HTTP로 노출된다. 공용 Wi-Fi, 학교망, 사무실망에서는 사용하지 않는다.

결정 로그 H-014의 PAM 인증 기록은 이 무인증 noVNC 구성과 다른 시점 또는 다른 설정을 가리킬 수 있다. 현재 활성 인증 방식을 문서에서 PAM이라고 단정하지 않는다.

---

## 5. noVNC systemd 서비스 등록

### 5.1 구조

```text
Mac 브라우저 :6080
        │ HTTP + WebSocket
        ▼
Pi의 noVNC/websockify :6080
        │ localhost:5900
        ▼
Pi의 WayVNC :5900
        │
        ▼
현재 로그인된 GUI 세션
```

noVNC는 화면을 직접 생성하지 않는다. `localhost:5900`에 WayVNC가 먼저 떠 있어야 한다. `After=wayvnc.service`는 시작 순서만 지정하며, WayVNC나 GUI 세션을 자동으로 만들지는 않는다.

### 5.2 유닛 생성

`User=`는 실제 GUI 로그인 사용자와 같아야 한다. 현재 구성은 `ugrp1`이다.

```bash
sudo tee /etc/systemd/system/novnc.service >/dev/null <<'EOF'
[Unit]
Description=noVNC Web GUI Service
Wants=network-online.target
After=network-online.target wayvnc.service

[Service]
Type=simple
User=ugrp1
ExecStart=/usr/bin/websockify --web /usr/share/novnc 6080 localhost:5900
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
```

경로가 다르면 `ExecStart`를 실제 `command -v websockify`와 noVNC 설치 경로에 맞춘다. 이후 서비스를 등록하고 시작한다.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now novnc
sudo systemctl --no-pager --full status novnc
```

상태를 간단히 확인한다.

```bash
sudo systemctl is-active novnc
sudo ss -ltnp | grep -E ':(5900|6080)\b'
```

- `5900`: WayVNC 원본 VNC 포트
- `6080`: noVNC 웹 페이지와 WebSocket 포트

---

## 6. Mac 브라우저에서 열기

현재 Pi 주소를 `<PI_IP>`에 넣는다.

```bash
PI_IP='<PI_IP>'
nc -vz "$PI_IP" 6080
open "http://$PI_IP:6080/vnc.html?autoconnect=true&resize=scale"
```

브라우저 주소창에는 다음 형식으로 입력한다.

```text
http://<PI_IP>:6080/vnc.html?autoconnect=true&resize=scale
```

mDNS가 안정적으로 동작하면 IP 대신 다음도 가능하다.

```text
http://ugrp1.local:6080/vnc.html?autoconnect=true&resize=scale
```

성공 당시의 실제 주소를 넣은 예시는 다음과 같다.

```text
http://172.20.10.2:6080/vnc.html?autoconnect=true&resize=scale
```

URL 옵션은 다음 의미다.

| 옵션 | 의미 |
| --- | --- |
| `autoconnect=true` | Connect 버튼을 생략하고 바로 연결을 시도한다. 인증을 제공하는 옵션은 아니다. |
| `resize=scale` | 브라우저 창 안에 화면을 맞춰 표시한다. |

완료 여부는 다음 네 가지로 판단한다.

1. noVNC 페이지가 열린다.
2. VNC 연결 상태가 된다.
3. Pi의 GUI 화면이 보인다.
4. 마우스와 키보드 입력이 Pi에서 반응한다.

---

## 7. 권장 접속 방식: SSH 터널

현재의 직접 HTTP 방식은 편하지만, 무인증 화면과 입력을 핫스팟에 노출한다. 장기 사용 시에는 WayVNC와 websockify를 loopback 주소에만 바인딩하고 SSH 터널을 사용한다.

구조는 다음과 같다.

```text
Mac 127.0.0.1:6080
        │ SSH 암호화 터널
        ▼
Pi 127.0.0.1:6080 (websockify)
        │
        ▼
Pi 127.0.0.1:5900 (WayVNC)
```

WayVNC와 websockify의 listen 주소를 실제 설치 버전의 도움말에 맞춰 `127.0.0.1`로 변경한 뒤, 외부 listener가 사라졌는지 `ss -ltnp`로 확인한다. 버전마다 websockify의 listen-address 옵션 표기가 다를 수 있으므로 `websockify --help`를 기준으로 한다.

Mac에서 터널을 연다.

```bash
ssh -N -L 6080:127.0.0.1:6080 ugrp1@<PI_IP>
```

터널을 유지한 채 다른 Mac 터미널에서 브라우저를 연다.

```bash
open 'http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale'
```

SSH 세션을 종료하면 터널과 화면 공유도 종료된다. 비밀번호·인증서·개인키는 이 저장소에 기록하지 않는다.

---

## 8. 문제 해결

### 8.1 Pi를 찾지 못함

- iPhone의 **다른 사람의 연결 허용**이 켜져 있는지 확인한다.
- Mac과 Pi가 같은 Wi-Fi에 연결되어 있는지 확인한다.
- Pi 전원과 부팅 상태를 확인한다.
- `ugrp1.local`이 안 되면 핫스팟 연결 기기 목록에서 IP를 다시 찾는다.
- `172.20.10.2`를 계속 사용하지 말고 `<PI_IP>`를 갱신한다.

### 8.2 SSH는 되지만 6080이 닫힘

Pi에서:

```bash
sudo systemctl status novnc --no-pager -l
sudo journalctl -u novnc -n 80 --no-pager
sudo ss -ltnp | grep ':6080' || true
```

경로 오류, 포트 충돌, websockify 실행 실패를 확인한 뒤 해당 서비스만 재시작한다.

```bash
sudo systemctl restart novnc
```

다른 프로세스까지 종료할 수 있는 `fuser -k 6080/tcp`는 사용하지 않는다.

### 8.3 페이지는 열리지만 VNC 연결 실패

```bash
sudo ss -ltnp | grep ':5900' || true
sudo systemctl status wayvnc --no-pager -l
sudo journalctl -u wayvnc -n 80 --no-pager
```

websockify는 `localhost:5900`으로 연결하므로 WayVNC가 먼저 실행되어야 한다. WayVNC가 user service라면 다음을 확인한다.

```bash
systemctl --user --no-pager --full status wayvnc
journalctl --user -u wayvnc -n 80 --no-pager
```

### 8.4 검은 화면 또는 입력 무응답

- Pi에 실제 Desktop GUI 세션이 로그인되어 있는지 확인한다.
- `XDG_SESSION_TYPE`, `WAYLAND_DISPLAY`가 현재 세션에 맞는지 확인한다.
- WayVNC가 system service인지 user service인지 확인한다.
- 브라우저 화면에 포커스를 둔 뒤 입력을 다시 확인한다.

서비스 재시작이 필요하면 다음 순서로 실행한다.

```bash
sudo systemctl restart wayvnc
sudo systemctl restart novnc
```

유닛이 없다는 오류가 나오면 user service 여부를 확인하고, 같은 명령을 반복하지 않는다.

### 8.5 재부팅 후 주소가 바뀜

DHCP 환경에서는 정상일 수 있다. 재부팅 후 다음 순서로 다시 접속한다.

1. 핫스팟 연결 기기 목록 또는 `ugrp1.local`에서 새 IP를 찾는다.
2. SSH 접속을 확인한다.
3. 새 IP로 `:6080` 브라우저 주소를 연다.

---

## 9. 사용 후 중지와 원상 복구

무인증 직접 접속을 더 이상 사용하지 않으면 noVNC를 중지한다.

```bash
sudo systemctl disable --now novnc
```

서비스 파일을 제거할 때는 먼저 현재 상태를 확인한 뒤 실행한다.

```bash
sudo systemctl stop novnc
sudo rm /etc/systemd/system/novnc.service
sudo systemctl daemon-reload
```

WayVNC 설정을 되돌릴 때는 백업 목록을 확인하고 원하는 파일만 복원한다.

```bash
ls -1t /etc/wayvnc/config.bak.* 2>/dev/null | head
```

---

## 10. 최종 사용 순서 요약

매번 다음 순서만 따르면 된다.

1. iPhone **개인용 핫스팟**에서 **다른 사람의 연결 허용**을 켠다.
2. Mac Wi-Fi를 그 핫스팟에 연결한다.
3. Pi 주소를 `ugrp1.local` 또는 핫스팟 연결 목록에서 찾는다.
4. `ssh ugrp1@<PI_IP>`로 SSH가 되는지 확인한다.
5. 브라우저에서 `http://<PI_IP>:6080/vnc.html?autoconnect=true&resize=scale`을 연다.
6. 화면과 마우스·키보드 입력을 확인한다.
7. 사용 후 `novnc`를 중지하거나 SSH 터널 방식으로 전환한다.

이 절차는 UGRP 연구 결과가 아니라 Raspberry Pi 원격 개발 환경 구축 절차다.

---

## 참고 링크

- [WayVNC upstream README](https://github.com/any1/wayvnc/blob/master/README.md)
- [WayVNC configuration reference](https://github.com/any1/wayvnc/blob/master/wayvnc.scd)
- [noVNC](https://github.com/novnc/noVNC)
- [websockify](https://github.com/novnc/websockify)
