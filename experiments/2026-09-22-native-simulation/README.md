# 로컬 native 시뮬레이션 CLI/API 검증 (2026-09-22)

실행 소스는 `535ba3c26dcdf986cd71d760fcde86ddbae7be69`다. [사용법·설정·API](../../docs/local_simulation.md), [원본 경로와 해시](verification.json).

## 확인한 범위

- Mac arm64, Python 3.12.13, MuJoCo 3.12.0: API 통합, native 창 실행·유한 종료, headless+보정 RGB 저장 통과.
- Ubuntu 24.04 x64, Python 3.12.14, MuJoCo 3.12.0: API 통합과 Xvfb 데스크톱에서 native 창 3회 연속 실행·정상 프로세스 종료 통과. [CI artifact](https://github.com/cmk404/UGRP-Multi-Robot-Collaboration-Project/actions/runs/35695487261).
- 단위 검사 30 passed + 27 subtests: 설정 검증, reset, tick별 명령 만료, 관측 경계, native 렌더 소유자 정리 순서.
- 네 장면 `standard/mixed/arena/camera_team`, seed41, 화물 subset·로봇 초기 위치 설정. 동일 model/data로 reset 후 초기 qpos 재현. 세 자기 RGB·공용 top, FOV 유지와 weld OFF.
- 고정 스케줄 3초/명령 3회, 모델 호출 0. 자동 검사에서 R1 약 0.0233m 변위 확인. 자율 운반·협력·연구 성공률 검증이 아니다.

## 실패와 제한

첫 Ubuntu native 실행은 물리가 완료된 뒤 프로세스 종료 시 exit139였다. `Handle.close()`가 종료 요청만 하고 native 렌더 정리를 기다리지 않는 MuJoCo 3.12 구현을 확인했다. `_sim` weak owner 소멸을 기다리는 처리를 한 곳에 한정했고 Linux 3회 연속·Mac 실행으로 다시 검사했다. 이 private API 의존성은 고정 MuJoCo 버전을 올릴 때 재검증한다. 원본의 종료 직전 `protocol_complete=true`는 보존하고, 별도 dashboard 파생 기록은 프로세스 실패로 표시한다.

Mac GUI 자동화에서 `org.mujoco.mjpython` 접근이 시간 초과되어 키보드/마우스와 실제 화면의 수동 검증은 미완료다. 두 paused 확인 실행은 에이전트가 종료했으며 완료 실행으로 세지 않는다. native render loop/physics 진행/프로세스 종료 검증과 화면 확인을 구분한다.

기존 MasterPi 3대/네 장면을 공통 API로 묶은 범위다. 임의 로봇/MJCF 구성, 모든 기존 하네스 이관, 연속 영상 녹화는 포함하지 않는다. 이전 [브라우저 구현 기록](../2026-09-22-local-simulation-live/README.md)은 폐기한 접근의 이력으로 보존한다.

## 원본·대시보드

원본은 worktree `outputs/native-*`에 있으며 개별 경로·SHA256은 verification.json에 있다. `outputs/native-records`는 원본을 수정하지 않고 CI 종료 실패·수동 확인 중단의 맥락을 더한 파생 기록이다. 원본·파일 해시를 검증한 뒤 기본 체크아웃 `outputs/tensorboard/0922-native-simulation`, `0922-native-linux`의 새 snapshot으로 변환했다. 기존 결과는 덮어쓰지 않았다.

TensorBoard 고정 링크는 기본 체크아웃의 `outputs/tensorboard-view.json` → `local_native_simulation.url`이다. 완료 주장·실행 시간·명령 수·모델 호출 수를 표시한다. raw Mac 자료는 로컬 보관이며 Git 기록/해시는 원격 백업이 아니다. Linux artifact는 GitHub에서 14일 보존된다.
