# 로컬 실시간 시뮬레이션 뷰어 검증

> 이 기록은 폐기한 브라우저 구현의 검증 이력이다. 현재 제공 경로는 [native CLI/API](../../docs/local_simulation.md)이며 아래 자료를 해당 경로의 검증으로 해석하지 않는다.

2026-09-22. [실행 안내](../../docs/local_simulation.md). 다른 참여자가 자신의 컴퓨터에서 모델 계정 없이 세 로봇을 조작하고 네 카메라를 보는 진입점을 검증했다. 자율 운반·통신 효과·실물 성공 평가는 아니다.

## 실행 소스와 전체 시도

| 실행 | 소스 | 결과 |
|---|---|---|
| v1 브라우저 수동 확인 | `69a2704` | Chrome 강 프로필에서 카메라 표시·R1 이동·화면 종료를 확인. 세션 종료 확인. 레이아웃의 빈 공간은 후속 수정 |
| v2 통합 검사 | `599922f` | 검사 실패: observer는 기존 고정 최소 640×480인데 검사에서 모든 카메라를 384×288로 가정. 카메라를 바꾸지 않고 검사를 수정. 자식 서버 정상 정리 |
| v3 통합 검사 | `a9194b1` | 네 JPEG의 크기·영상 분산, 실제 시간 진행과 R1/전체 장면 변화, R2 회전 명령, 일시정지 시간 고정, seed42 초기화, HTTP 종료 후 프로세스 종료와 결과 저장 모두 확인 |

Mac Python 3.12.13, MuJoCo 3.12.0. 관련 자동 검사 26개와 27 subtests 통과. v3은 seed41→42, 2개 세계, 적용 명령 5개, 물리 진행 합계 1.978 SIM초, 10개 카메라 묶음, 9.041초 실제 실행이다. HTTP 종료 명령은 물리 명령 수에 포함하지 않는다. 브라우저 새로고침·영상 갱신 시간은 별도이며 실시간 1배속을 보장하지 않는다.

Ubuntu의 실제 OSMesa 통합 검사는 PR의 `ubuntu-simulation` CI에 포함했다. CI 통과 전에는 이 로컬 기록을 Ubuntu 실행 증거로 사용하지 않는다. 개인 Windows/WSL 환경은 별도 확인 대상이다.

## 재현

```bash
python -m pytest -q tests/test_sim_live.py tests/test_sim_quickstart.py tests/test_camera_robot_port.py
python -m scripts.check_sim_live --output outputs/live-check-NEW-ID
bash scripts/open_simulation.command
```

`python`은 requirements-sim.txt와 requirements-test.txt가 설치된 환경을 사용한다. 통합 검사는 자기 서버만 켜고 종료하며 실제 모델 서비스를 호출하지 않는다. 브라우저 뷰어는 localhost에만 바인딩한다. 30분의 실행 상한과 브라우저 조회가 없을 때 15초 후 일시정지를 제공한다.

[verification.json](verification.json)에 모든 시도의 소스·결과·실패와 로컬 원본 경로·SHA256을 보존했다. raw JPEG와 로그는 로컬 보관이며 이 요약의 Git 업로드를 raw 전체 백업으로 표현하지 않는다. 성공/실패 기록은 기본 체크아웃 `outputs/tensorboard/0922-local-live-viewer`에 별도 스냅샷으로 표시한다. `protocol_complete`는 뷰어 통합 검사의 완료이며 로봇 과제 성공 지표가 아니다.
