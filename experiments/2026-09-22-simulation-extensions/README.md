# 로컬 환경·제어기·action 확장 검증

실행 코드: `32a88effa8599349ac43595891e5244afeec5585`. Linux PR CI는 main과의 시험 병합 `987843569c522f5a6e8a33308873a0790333bd93`에서 실행했으며 두 tree SHA는 `b6839c381c35d1db898d6a71d187b0d5578c6fec`로 동일하다. [확장 사용법](../../docs/simulation_extensions.md), [원본 위치와 해시](verification.json).

- 실행 소스의 PR/push CI 12개 모두 통과.
- 관련 단위 검사: 69 passed + 27 subtests. 설정 경로, 복사본 실행, raw 명령 재검증, 소유권 충돌, reset, 제어기 오류 시 입력 보존, 관측 경계와 기존 RGB 실행 경로.
- Mac arm64/Python 3.12.13/MuJoCo 3.12.0: headless 확장 통합, 공통 launcher를 통한 native 창+RGB 실행/정상 종료.
- Ubuntu 24.04 x64/Python 3.12.14/MuJoCo 3.12.0: 같은 통합 검사와 공통 launcher를 통한 Xvfb native 창/정상 종료. [CI](https://github.com/cmk404/UGRP-Multi-Robot-Collaboration-Project/actions/runs/35698033271)의 `simulation-runtime-verification` artifact를 회수하고 파일 해시를 확인했다.
- 생성한 실험 폴더에서 장애물 반폭 .35→.6m 변경, free-joint 구 초기화, 제어기 함수 교체, 각 호출의 자기 RGB/top/응답 보존을 확인했다. 동일 .35m 환경에서 active/idle 제어기만 바꾼 두 실행의 최종 R1 위치 차이는 두 OS 모두 약 .02105m. 이는 실제 물리에 명령 변경이 반영됨을 확인하는 진단이다.
- 각 정책 실행은 2 SIM초, controller 10회, 발행 명령 11회(제어기 10+R2 스케줄 1)다. 예제는 외부 모델을 호출하지 않는다. 임의 사용자 제어기의 내부 모델 호출은 자동 계측하지 않는다.
- 기존 Linux 4개 장면/API 검사와 native 3회 연속 종료도 통과했다. 해당 원본은 회수 묶음의 api/native-1..3에 보존했다.

이번 확장 검증에는 실패한 물리 실행이 없다. 이전 native 종료 race 실패·paused 확인 중단은 [이전 기록](../2026-09-22-native-simulation/README.md)에 그대로 남긴다. Mac native 창의 키보드/마우스 자동화는 기존 AX 도구 접근 시간 초과로 미확인이다. 창 실행/렌더/종료와 실제 UI 조작 확인은 구분한다.

추가 형상은 고정/동적 box·sphere·cylinder이며 기존 화물 임무에 자동 등록하지 않는다. 기존 MasterPi 3대, 정상 접촉 물리와 weld OFF를 유지한다. action 확장은 기존 액추에이터 명령으로 내려가는 한 명령이고 여러 단계 정책은 제어기에 작성한다. Python 진입 파일은 신뢰 코드다. 새로운 로봇/센서·전체 MJCF 교체·LLM 통신 정책·운반 성공률은 이번 검증 범위가 아니다.

TensorBoard: 기본 체크아웃 `outputs/tensorboard/0922-extensions-mac`, `0922-extensions-linux`. 기존 원본/스냅샷을 유지하고 원본 해시와 EventAccumulator 로딩을 확인했다. 완료 주장·시간·명령·존재하는 모델 호출 지표를 표시한다. 12개 실행과 HParams 열·고정 카드 4개를 실제 화면에서 확인했다. Chrome 연결이 끊겨 기존 Chrome 탭 대신 Codex 앱 내 브라우저에 같은 대시보드를 표시했다. 필터/고정 링크는 `outputs/tensorboard-view.json`의 `simulation_extensions`다. 영상은 생성하지 않았으며 제어 입력 JPEG는 raw JSONL에 있다. Mac raw는 로컬 보관, Linux artifact는 GitHub 14일 보존이다. Git에 기록한 해시는 raw의 원격 백업이 아니다.
