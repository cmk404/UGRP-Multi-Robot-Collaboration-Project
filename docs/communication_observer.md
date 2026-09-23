# 시뮬레이션 동료 대화 관찰

`ThreeRobotRuntime`은 검증된 모델 응답의 `message`가 다른 로봇의 inbox에 실제 추가된 직후 `team/conversation.jsonl`에 전달 영수증을 덧붙인다. 각 행에는 발신자, 수신자, 턴, 계획/실행 단계, 시뮬레이션 시각, 요청 ID, 원문 및 `llm`/`fixture` 출처가 있다. `reason`은 로봇의 판단 설명으로 `decision_explanation`, `accept`와 제안 ID는 `proposal_vote`로 따로 기록한다. 모터 명령은 동료 대화에 포함하지 않는다.

터미널의 `peer_delivery` 행에는 전달 원문 전체가 JSON 문자열로 출력된다. 제어 문자와 줄바꿈은 터미널 안전을 위해 이스케이프하지만 UTF-8 JSONL의 `text` 값은 원문을 보존한다. `team/latest-dialogue.json`은 최근 실제 모델 메시지 최대 3개와 누적 건수를 담는 8 KiB 이하의 원자적 교체 파일이다. 오래된 메시지와 전체 문장은 JSONL에서 확인한다. 최신 파일은 새 메시지나 fixture에서 실제 모델로 전환될 때만 갱신한다.

`--viewer`의 MuJoCo 관찰 창은 이 최신 파일만 5 Hz 이하로 읽어 최근 대화와 전체 로그 위치를 작은 이미지로 표시한다. `--realtime-control`에서는 독립 프로세스의 복제 모델, 기본 비실시간 실행에서는 기존 복제 모델의 창에 표시한다. 두 창 모두 팀 생성·계획 협상 전에 열린다. Headless 실행은 터미널·JSONL 기록을 제공한다. macOS Apple SD Gothic Neo 또는 Linux Noto/Nanum 한글 폰트를 사용한다. 폰트가 없으면 창은 영문 안내로 돌아가며 원문은 UTF-8 터미널과 JSONL에서 볼 수 있다. 관찰용 이미지/파일은 로봇 RGB, 물리 상태, 판단 요청으로 되돌아가지 않는다. 별도 웹 UI나 추가 모델 요청은 없다.

`--plan-replay`의 합의 투표는 scripted fixture다. 이때 새 자연어 메시지 수는 0으로 표시한다. fixture가 시험용 문장을 inbox에 전달하더라도 `fixture_peer_message`로 기록하고 실제 모델 대화 건수에는 넣지 않는다. 저장 계획의 RGB 물리 실행도 새로운 자연어 대화를 만들어내지 않는다.

TensorBoard의 현재 generic export는 이 JSONL을 Text 카드로 변환하지 않는다. 완료된 실행을 별도 불변 snapshot으로 내보낼 때 `peer_message`와 `fixture_peer_message`를 분리해 Text event를 추가할 수 있다. 실시간 관찰의 기준 원본은 계속 JSONL이다.

실제 저장 LLM 대화를 새 호출 없이 창에서 확인하려면, 물리 실험이 종료된 뒤 아래의 유한한 관찰 전용 명령을 저장소 루트에서 실행할 수 있다. 이 명령은 저장된 `team.json`의 마지막 세 문장을 **기록 재생**으로 표시한다. 실행 결과와 최신 sidecar를 건드리지 않고 원본 SHA-256·건수·폰트 상태를 새 감사 파일에 기록한다.

```bash
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython -m scripts.smoke_communication_overlay \
  --source outputs/simulation-runs/20260923-161558-dispatch-cca0ae6c/artifacts/team/team.json \
  --audit outputs/communication-observer/saved-dialogue-native-smoke.json \
  --duration-s 10
```
