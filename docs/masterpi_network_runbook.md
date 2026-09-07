# MasterPi 네트워크 운영 런북

> 상태: 운영 제안
>
> 목적: MasterPi 연결을 “어쩌다 발견됨”이 아니라, 어느 단계에서 실패했는지 재현 가능하게 확인한다.

## 0. 핵심 정책

1. **주 네트워크는 전용 Wi‑Fi AP/공유기 하나로 고정한다.** 교내 Wi‑Fi와 아이폰 핫스팟은 개발용 임시 경로로만 사용한다.
2. **한 번에 한 경로만 켠다.** 전용 AP, 아이폰 핫스팟, Mac 인터넷 공유를 동시에 바꾸지 않는다.
3. **Pi의 Wi‑Fi 설정은 하나의 네트워크 관리자만 관리한다.** 새 프로비저닝 후에는 Pi에서 실제 활성 관리자를 확인하고, `wpa_supplicant.conf`, Netplan, Imager 설정을 무심코 중복 운용하지 않는다.
4. **mDNS는 편의 기능이고 SSH가 연결 증거다.** `ugrp1.local`이 안 되더라도 DHCP 주소로 SSH가 되면 네트워크는 살아 있다.
5. **noVNC는 SSH 터널로만 연다.** 무인증 HTTP 포트를 교내망·공용망에 노출하지 않는다.

## 1. 권장 토폴로지

```text
                    인터넷(선택)
                        │
                 전용 Wi‑Fi AP/공유기
                 2.4 GHz / WPA2-PSK
                   │       │       │
                 Mac     Pi #1   Pi #2
                          masterpi1  masterpi2
```

- 두 Pi와 Mac은 항상 같은 로컬 서브넷에 둔다.
- 공유기에서 두 Pi의 DHCP 예약을 설정한다. 저장소와 명령어에는 실제 IP나 인증정보를 기록하지 않는다.
- 한 대만 시험할 때는 Pi의 Direct AP 모드를 사용할 수 있지만, 두 대·카메라·LLM 실험의 주 경로로 사용하지 않는다.
- 전용 AP가 당장 없으면 아이폰 핫스팟을 **부트스트랩용**으로 사용하고, 장기 실험 전에 전용 AP로 전환한다.

## 2. 연결 상태 모델

| 상태 | 의미 | 다음 확인 |
| --- | --- | --- |
| `MAC_WIFI_NOT_READY` | Mac에 IPv4가 없음 | Mac을 선택한 AP에 연결 |
| `PI_NOT_VISIBLE` | 같은 망에서 Pi 주소가 발견되지 않음 | Pi 전원·SSID·AP 연결 기기 목록 확인 |
| `PI_VISIBLE_SERVICES_UNAVAILABLE` | 주소는 찾았지만 SSH가 닫힘 | Pi 부팅, SSH, 방화벽 확인 |
| `SSH_READY` | 원격 셸 가능 | Pi 내부에서 Wi‑Fi와 서비스 확인 |
| `REMOTE_DESKTOP_READY` | SSH와 noVNC 포트 모두 응답 | SSH 터널로 GUI 확인 |

`mDNS` 실패만으로 `PI_NOT_VISIBLE`이라고 단정하지 않는다. 주소를 알고 있으면 다음처럼 직접 확인한다.

```bash
scripts/masterpi_connection_status.sh --host <PI_IP>
```

## 3. Mac에서 실행하는 표준 순서

매번 아래 순서를 처음부터 실행한다.

```bash
# 1) 현재 Mac 네트워크와 Pi 발견 상태
scripts/masterpi_connection_status.sh

# 2) 부팅 직후 최대 120초 동안 재확인
scripts/masterpi_connection_status.sh --wait 120 --interval 2

# 3) 주소를 확보한 뒤 계층별 포트 확인
scripts/masterpi_connection_status.sh --host <PI_IP>

# 4) SSH가 열려 있을 때만 Pi 상태 확인
ssh -o ConnectTimeout=3 ugrp1@<PI_IP> 'hostname; ip -4 addr show wlan0; ip route; systemctl is-active ssh'
```

현재 `scripts/find_pi.sh`는 기존 호출 호환용 별칭이다. 무한 대기하지 않고 bounded 진단기를 실행하며, 비밀번호를 출력하지 않는다.

## 4. Pi에서 확인할 단일 진실 공급원

SSH가 된 뒤 먼저 어떤 네트워크 관리자가 실제로 동작하는지 확인한다.

```bash
systemctl is-active NetworkManager 2>/dev/null || true
systemctl is-active wpa_supplicant 2>/dev/null || true
nmcli device status 2>/dev/null || true
nmcli connection show 2>/dev/null || true
ip -4 addr show wlan0
ip route
```

그 결과에 따라 **실제로 활성인 관리자 하나만** 기준으로 삼는다. 새 프로필을 추가할 때도 SSID와 비밀번호는 로컬 터미널에서만 입력하고 파일·로그·채팅에 남기지 않는다.

검증이 끝나기 전에는 기존 부팅 파티션의 설정 파일을 삭제하거나 SD 카드를 다시 플래싱하지 않는다. 먼저 현재 Pi 내부의 연결 실패 원인과 로그를 보존한다.

## 5. SSH·GUI·제어의 검증 순서

연결 성공을 한 가지 신호로 판단하지 않는다.

1. **네트워크**: Pi에 IPv4 주소와 기본 경로가 있음
2. **SSH**: TCP 22와 실제 셸 로그인 확인
3. **GUI**: WayVNC/noVNC 서비스 상태와 포트 확인
4. **제어 API**: `/api/drive` 또는 실제 제어 엔드포인트의 응답 확인
5. **하드웨어**: 저속·짧은 펄스의 정지 가능한 테스트 후 즉시 정지

GUI는 가능하면 SSH 터널로 연다.

```bash
ssh -N -L 6080:127.0.0.1:6080 ugrp1@<PI_IP>
```

그 뒤 Mac 브라우저에서 `http://127.0.0.1:6080/vnc.html`을 연다. 연결이 끊기면 Pi 내부의 watchdog이 모터를 정지해야 한다.

## 6. 두 대로 확장할 때 지켜야 할 것

- Pi마다 고유 호스트명과 고유 DHCP 예약을 사용한다.
- `masterpi1`, `masterpi2`를 임의의 고정 IP 대신 이름과 공유기 예약으로 관리한다.
- P2P 메시지는 두 Pi가 같은 로컬 AP에 붙은 뒤 MQTT/WebSocket 중 하나로 검증한다.
- 카메라·VNC·LLM 트래픽과 모터 제어 명령을 같은 “연결 성공” 판정으로 묶지 않는다.
- 제어 명령에는 heartbeat와 만료 시간을 두고, 링크 단절 시 로컬 정지를 우선한다.

## 7. 2026-08-20 현재 상태

- Mac은 `postech` Wi‑Fi에 연결되어 있다.
- 현재 Mac에서 `ugrp1.local`과 Raspberry Pi MAC이 발견되지 않는다.
- Raspberry Pi USB 직결 주소와 SD 카드 마운트도 확인되지 않는다.
- 따라서 현재는 Pi 내부의 Wi‑Fi 연결 실패 원인을 원격으로 읽을 수 없다.

다음 실제 검증은 아이폰 핫스팟을 무작정 재시도하는 것이 아니라, 하나의 선택된 AP를 켠 상태에서 Pi 전원·SSID 연결 표시·DHCP 주소·SSH를 위 순서로 각각 확인하는 것이다.
