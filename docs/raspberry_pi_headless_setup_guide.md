# Raspberry Pi 헤드리스 Wi-Fi & SSH 무선 연결 성공 가이드

> 대상: Raspberry Pi 4B / MasterPi (Raspberry Pi OS Bookworm & Bullseye 64-bit)  
> 목적: 모니터, 키보드 연결 없이(Headless) SD 카드 사전 설정만으로 Wi-Fi 자동 연결 및 SSH 원격 제어를 100% 성공시키는 표준 절차

---

## 1. 성공 원리 및 핵심 포인트

라즈베리파이 최신 OS(Bookworm)와 이전 OS(Bullseye)는 네트워크 및 사용자 초기화 방식이 다릅니다:
- **구형/표준 방식**: `/boot/wpa_supplicant.conf`, `/boot/userconf.txt`, `/boot/ssh`
- **최신 cloud-init/Netplan 방식**: `/boot/network-config`, `/boot/user-data`, `/boot/meta-data`

💡 **성공 비결**: 두 방식의 설정 파일을 `bootfs` 파티션에 모두 완벽하게 주입하여 **OS 버전(Bookworm/Bullseye)에 상관없이 무조건 100% 계정 생성 + Wi-Fi 연결 + SSH 활성화**가 되도록 구성합니다.

---

## 2. SD 카드 `bootfs` 필수 설정 파일 모음

SD 카드를 Mac/PC에 연결했을 때 인식되는 `bootfs` (FAT32 파티션)의 루트 경로에 아래 파일들을 배치합니다.

### 2.1 SSH 활성화
* 파일명: `ssh` (확장자 없음, 빈 파일)
```bash
touch /Volumes/bootfs/ssh
```

---

### 2.2 사용자 계정 생성 (`userconf` & `userconf.txt`)
Raspberry Pi 기본 계정(`pi`)이 폐지되었으므로 사전 계정 주입이 필수입니다.

1. **비밀번호 SHA-512 해시 생성** (터미널 명령):
   ```bash
   HASH=$(openssl passwd -6 -salt $(openssl rand -hex 8) "원하는비밀번호")
   echo "ugrp1:$HASH" > /Volumes/bootfs/userconf.txt
   cp /Volumes/bootfs/userconf.txt /Volumes/bootfs/userconf
   ```

---

### 2.3 Wi-Fi 설정 1: `wpa_supplicant.conf` (표준/Bullseye 호환)
다중 네트워크(예: Mac AP 및 모바일 핫스팟)를 우선순위(`priority`)와 함께 등록합니다.

* 파일 경로: `/Volumes/bootfs/wpa_supplicant.conf`
```ini
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=KR

network={
    ssid="UGRP-MAC"
    psk="AP비밀번호"
    priority=10
}

network={
    ssid="changmin-iPhone"
    psk="핫스팟비밀번호"
    priority=5
}
```

---

### 2.4 Wi-Fi 설정 2: `network-config` (Netplan / Bookworm 호환)
* 파일 경로: `/Volumes/bootfs/network-config`
```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      dhcp6: true
      optional: true
  wifis:
    wlan0:
      dhcp4: true
      regulatory-domain: "KR"
      access-points:
        "UGRP-MAC":
          password: "AP비밀번호"
        "changmin-iPhone":
          password: "핫스팟비밀번호"
      optional: true
```

---

### 2.5 초기화 설정: `user-data` (Cloud-init 호환)
* 파일 경로: `/Volumes/bootfs/user-data`
```yaml
#cloud-config
manage_resolv_conf: false

hostname: ugrp1
manage_etc_hosts: true
packages:
- avahi-daemon
apt:
  preserve_sources_list: true
  conf: |
    Acquire {
      Check-Date "false";
    };
timezone: Asia/Seoul
keyboard:
  model: pc105
  layout: "kr"
user:
  name: ugrp1
  shell: /bin/bash
  lock_passwd: false
  passwd: "생성한_SHA512_비밀번호_해시"
ssh_pwauth: true
runcmd:
  - [ systemctl, enable, --now, ssh ]
```

---

## 3. 부팅 및 네트워크 탐색 절차

1. **SD 카드 장착 및 부팅**:
   * SD 카드를 Pi에 꽂고 전원 인가
   * 녹색 ACT LED가 약 1분간 불규칙하게 점멸(파티션 확장 및 초기 설정) 후 꺼질 때까지 대기
2. **모바일 핫스팟/AP 준비**:
   * 아이폰 핫스팟의 경우 `다른 사람의 연결 허용` ON, `호환성 최대화` ON 유지
   * Mac의 Wi-Fi도 동일한 핫스팟/AP에 연결
3. **IP 자동 탐색**:
   ```bash
   # 서브넷 핑 스캔 (예: 172.20.10.0/28)
   for i in {1..14}; do ping -c 1 -W 500 172.20.10.$i &>/dev/null & done; wait
   
   # 라즈베리파이 MAC 주소(dc:a6:32 / b8:27:eb / e4:5f:01) 확인
   arp -a | grep -i -E "dc:a6:32|b8:27:eb|e4:5f:01"
   ```

---

## 4. SSH 접속

```bash
# IP로 직접 접속
ssh ugrp1@172.20.10.2

# 또는 호스트 별칭(mDNS)으로 접속
ssh ugrp1
# 또는
ssh ugrp1@ugrp1.local
```
* **기본 사용자**: `ugrp1`
* **초기 설정 비밀번호**: `1234` (또는 Imager에서 맞춤설정한 비밀번호)

---

## 5. 향후 편의를 위한 권장 후속 조치 (SSH Key 등록)

매번 비밀번호를 치지 않고 자동 접속하려면 Mac 터미널에서 다음을 실행합니다:
```bash
ssh-copy-id -i ~/.ssh/id_rsa.pub ugrp1@172.20.10.2
```
