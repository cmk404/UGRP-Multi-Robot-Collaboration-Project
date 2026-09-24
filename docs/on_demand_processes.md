# UGRP 프로젝트 프로세스: 필요할 때만 실행하기

`scripts/ugrp_session.py`는 기존 실행 명령을 포그라운드 세션으로 감싼다. 세션에서 생성된 모든 자식은 별도 프로세스 그룹에 들어가며, `Ctrl-C`, `SIGTERM`, 정상 종료, 또는 `stop` 명령 때 함께 정리된다. 실행 중인 실험을 시간 기준으로 끄는 idle timeout은 없다.

로컬 시뮬레이션은 표준 진입점과 MuJoCo 기본 창을 사용한다. 설정·메뉴·기록은
[로컬 시뮬레이션](local_simulation.md)과 [표준 관리](simulation_management.md)를 따른다.

```bash
cd /Users/changmin/projects/ugrp
bash scripts/open_simulation.command
```

브리지나 워커를 별도 터미널에서 실행해야 한다면 각각 이름을 준다.

```bash
python3 scripts/ugrp_session.py run sim-bridge -- .venv-sim/bin/python -m sim.bridge --host 127.0.0.1 --port 8091 --worker-port 8092
python3 scripts/ugrp_session.py run sim-worker -- <현재 실험에서 검증한 워커 실행 명령과 옵션>
```

실험별 워커 옵션은 기존 실험 명령을 그대로 사용해야 한다. 래퍼가 seed, 원격 주소, 속도나 모델 설정을 추정하지 않는다.

별도 명령을 세션 래퍼로 실행한 경우 상태 확인과 명시적 종료는 다음과 같다.

```bash
python3 scripts/ugrp_session.py status my-session
python3 scripts/ugrp_session.py stop my-session
```

세션 기록은 기본적으로 `/tmp/ugrp-$UID/sessions/`에만 저장된다. `stop`은 기록된 리더 PID, 프로세스 그룹, 시작 시각이 모두 현재 프로세스와 일치할 때만 해당 그룹을 종료한다. 기록이 오래되어 신원을 확인할 수 없으면 종료를 거부하므로, 다른 UGRP 실험이나 래퍼 밖에서 시작된 프로세스에는 영향을 주지 않는다.

구형 SIM/REAL/TEAM 브라우저 대시보드 런처는 2026-09-23 퇴역했다.
실물 장치 연결·기록은 [네트워크 안내](masterpi_network_runbook.md)와
[trace 안내](real_trace_system.md)를 따른다. 세션 시작 자체가 실제 로봇 동작 권한을 바꾸지는 않는다.
