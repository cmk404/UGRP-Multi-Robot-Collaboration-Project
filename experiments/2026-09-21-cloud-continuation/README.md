# ACT·Jev 후속의 Colab/Kaggle 재개

사용자가 두 기존 작업의 후속을 Colab/Kaggle에서 이어 달라고 요청했다. 기존 Mac 원본은 보존하고 새 원격 실행을 별도 코호트로 기록한다. 이 문서는 실행 착수 기록이며 성능 결론이 아니다.

## 재개 시 확인

- 128/256px ACT 학습 모델 8개는 회수되어 있다. 기존 Mac 물리 비교는 36회 중 결과 18개를 남기고 중단됐다. source `e00435b`의 원본 결과를 이번 원격 결과와 한 코호트로 합산하지 않는다.
- 512px 본 학습은 시작되지 않았다. 기존 2-update 기능 진단 중 Colab 세션이 목록에서 사라졌으며 원인은 확정되지 않았다. 캐시 텐서 보유 수정은 현재 main에 포함되어 있다.
- Jev skill 후보 `150554b`의 replacement holdout은 108개 중 결과 7개만 존재했다. 이전 `7dce220`의 32개 진단 결과는 별도 보존한다. 두 결과를 최종 228회 완료로 보고하지 않는다.
- 이 작업 시작 시 Colab 활성 세션 없음, Kaggle GPU 잔여 30h를 조회했다. 이후 상태는 변할 수 있다.

## 실행과 보존

- Colab T4 세션: `ugrp-resume-512-20260921`, ACT 실행 SHA `d13cda20834d039184bf90d5d9c817c1579b494c`.
- 순서: 전체 자료 512px/h4 2-step export 진단 → 512px/h1,h4 × 두 seed 각 8000 updates → 128/256px 기존 8모델+교사 36회 → 512px 4모델+교사 20회. 단계 실패 시 이후 단계는 실행하지 않는다. 전체 원격 작업은 23시간 한도다. 진단은 학습량과 scheduler가 다르므로 본 모델로 재사용하지 않는다.
- Colab의 시뮬레이션과 ACT Python 환경은 분리한다. NumPy/OpenCV 버전 차이를 유지하고 LeRobot 고정 소스 및 기존 device-only patch를 검증했다. batch32, float32, 카메라/FOV, 학생 입력, weld OFF를 유지한다.
- 원본 canonical dataset SHA: `3db730eaf9d827a40b355d7b53188993e14b75d10fd10b9b213a67f830fba167`. 이전 bundle의 자료를 사용하되 실행 소스는 새 Git snapshot이다. 경로 재배치는 canonical/root map/파일 해시를 검증한다.
- Kaggle CPU private/offline: `changmin2026/ugrp-simulation-ed4ad53401e0`, version1, SHA `06c4150364ea14f70cc10c9802446ed278723dde`. 새 holdout의 rule 기준선 12배치 × 3반복=36회, 2 worker. 제공자 종료 한도 1800초. 동일 물리 반복이며 독립 배치 36개가 아니다.
- Jev/Gemini 모델 비교는 인증/원격 endpoint가 미확보라 제출하지 않았다. 로컬 Gemini 주소를 공개하거나 임의 외부 터널을 만들지 않았다.
- 이 작업의 원격 실행에는 외부 모델 호출이 없다. 실제 청구액은 확인 전 null이며 무료 실행 또는 비용 0으로 단정하지 않는다.

로컬 원본/상태 위치:
`/Users/changmin/.codex/worktrees/act-cloud-resume/ugrp/outputs/cloud-resume/`
및 `outputs/kaggle-jev-rule-20260921/`.

`collection-status.json`은 수집 상태, `continuation.json`은 단계 상태다. 완료 ZIP은 분할 회수 후 ZIP/내부 모든 파일의 해시와 source SHA를 확인한다. 회수·해시 확인은 과제 성공이나 모델 성능 감사와 다르다. 새 물리 코호트가 끝난 뒤 엄격 성공·접근 실패·거짓 완료·시간·명령 수·추론 지연과 입력/영상 감사를 수행해야 한다. 선택된 조건의 새 오프셋 확인도 아직 실행하지 않았다.

수집기는 이 작업의 세션만 종료하며 Colab 자동 재접속/재할당·작업 재제출은 하지 않는다. 23.5시간 수집 한도가 있다. Mac에서는 업로드·수집·단위 검사만 수행한다. 자료는 로컬과 비공개 실행 사본으로 보존하며 Drive를 사용하지 않는다. 원본 전체의 영구 원격 백업을 주장하지 않는다.

## 코드와 검사

- 기존 Jev 후보를 별도 작업 브랜치에 통합했다. 충돌은 CI 테스트 목록에서 양쪽 항목을 보존해 해결했다. main은 변경하지 않았다.
- Kaggle 인자에서 원격 Python 경로를 전달하도록 하고 ACT 평가기가 검증된 데이터 경로 재배치를 허용하도록 수정했다.
- `run_cloud_carry_continuation.py`에 진단→학습→물리 평가 및 실패 보존을 연결했다.
- 로컬 오프라인 CI: 1195 passed, 3 skipped, 184 subtests passed. 관련 묶음 51개, 경로/프로토콜 관련 추가 실행 23개와 18개 통과(중복 포함, 합산하지 않음).
- actual GPU forward/export, 모델 학습 완료, 물리 결과 회수와 시각 검토는 각 상태/후속 기록에서 별도로 확인한다.

## 비용 우선순위 변경 — 2026-09-21 11:45 KST 이후

사용자의 512px 계산 비용 우려에 따라 512px 전체 학습과 이후 512px 평가는 보류했다. 앞서 제시한 10~20시간은 실제 512px 본 학습 속도를 측정한 예측이 아니므로 성능/시간 근거로 사용하지 않는다. 진단의 2-update는 완료했지만 export/cache 검증이 끝나지 않은 상태에서 중지했다. 본 8000-update 학습은 시작하지 않았다. 중지 시점 원본 및 체크포인트는 보존하고 별도 interruption manifest와 압축본을 만든다.

Colab에서는 이미 회수·검증한 128/256px 8모델+교사 36회 평가를 먼저 수행하도록 전환한다. 학습·모델·프로토콜은 바꾸지 않고 새 출력 `outputs/cloud-priority/physics128256`을 사용한다. 이 평가의 실행 한도는 4시간, 수집 한도는 4.5시간이다. 이는 완료 예상이 아닌 중단 한도다. Kaggle 규칙 기준선은 그대로 유지한다.

현재 상태는 로컬 `outputs/cloud-resume/priority-collection-status.json`, `physics128256-report.json`, `priority-intervention.json`을 확인한다. 이전 `collection-status.json`과 초기 실행 순서는 과거 기록이다. 512px 재시작은 현재 계획에 포함하지 않는다.

전환 첫 시도는 trial 시작 전 `ModuleNotFoundError: scripts`로 종료됐다. 원격 launcher가 기존 stage의 `PYTHONPATH`/OSMesa 환경을 전달하지 않은 것이 원인이다. 실패 원본은 `outputs/cloud-priority`에 유지하고 환경을 복원한 별도 `outputs/cloud-priority-v2`에서 같은 실행 SHA/프로토콜로 시작한다. 평가 entrypoint 자체도 저장소 루트를 import 경로에 추가하도록 보완해 격리 Python의 직접 실행 import를 검증했다. 기존 모델이나 평가 로직은 변경하지 않았다.

512px 중단 원본은 로컬 `outputs/cloud-resume/collected/diagnostic/result.zip`으로 회수했다. 압축본 SHA `34443c4161dcc7951bdbd292ef76913c5cb9d509f8f10b01e9516bb78addc781`, 내부 8파일과 source SHA를 검증했다. exit143은 비용 우선순위에 따른 명시적 중지다. 최신 평가 수집기는 `priority-v2-collection-status.json`을 사용한다.

2026-09-21 11:50 KST: Colab kernel exec 응답이 두 번 없어 출력 디렉터리 부재를 확인하고, 동일 생성 가드가 있는 launcher를 CLI의 piped console에서 실행했다. 새 supervisor PID10103/평가 부모 PID10105의 실행과 8개 기존 모델 검증을 확인했다. 새 환경은 PYTHONPATH·OSMesa·OMP_NUM_THREADS=2를 명시한다. 원격 평가 소스 SHA는 계속 `d13cda2`로 고정되어 있다.

## 모델 연결 후속

사용자가 제공한 Jev 키를 Mac 키체인에 저장하고 read-back을 확인했다. 실제 합성 의미 상태 입력으로 Jev 1.13.0 HTTP 200/0.710s, 기존 Gemini 3.8 Flash loopback HTTP 200/2.158s 및 스킬 응답 형식 검증이 통과했다. 이는 로봇 성공 결과가 아니다. 원본 요청/응답은 로컬 `outputs/cloud-resume/model-probe.json`, 비밀 제외 요약은 `model-connectivity.json`에 있다.

이 시점 재조회에서 기존 ACT Colab 세션은 서버에서 사라졌고 Kaggle rule 코호트는 CANCEL_ACKNOWLEDGED다. 기존 ACT collector를 종료했으며 이 둘을 계속 실행 중으로 보고하지 않는다. 미회수 결과·취소 원인은 확인되지 않았다. 새 CPU Colab 세션 `ugrp-model-compare-20260921`은 생성/IDLE을 확인했다. 비공개 파일 중계로 같은 Colab에서 세 정책을 비교하도록 준비한다. 512 학습은 계속 보류한다.

### Colab 왕복 확인 및 비교 시작

실행 SHA `b5ca7ca6d494dab76cf926c7af3f22dd2146ad08`. 새 Colab CPU에서 합성 의미 상태 요청을 비공개 mailbox로 보내 두 모델 모두 HTTP 200을 회수했다. Jev 총 2.897s/API 0.875s, Gemini 총 4.624s/API 2.617s였다. `colab-model-connectivity.json`에 원본 위치·해시를 기록했다. 키는 Mac 키체인에만 있고 원격에 업로드하지 않았다.

동일 Colab에서 dev_open의 rule/Jev/Gemini 3회 진단을 시작했다. 작성 시점에는 초기 물리 처리 중이며 완료한 로봇 trial은 0회다. 진단의 모델 전송에 실패가 없으면 108회 holdout으로 이어지되 두 단계 합계 3.5시간으로 제한한다. 실제 과제 성공 여부로 조건을 선별하지 않는다. 결과 collector는 중간 결과 JSON/turns와 종료 단계 ZIP을 회수하고 SHA·내부 파일 해시를 검사한 후 이 작업의 relay와 Colab만 종료한다. 로컬 상태: `outputs/cloud-model-compare/collection-status.json`, 중계 기록: `outputs/cloud-model-compare/relay/status.json`. 전체 로컬 검사 1199 passed / 3 skipped / 191 subtests passed. 모델 연결 성공과 로봇 성공률은 별개다.

## Kaggle 재개 점검

기존 rule36 작업의 서버 취소 원인은 로그에 기록되지 않았다. 로그에는 sitecustomize/wrapt 경고만 있으며 이것만으로 취소 원인으로 단정하지 않는다. `CANCEL_ACKNOWLEDGED`의 밑줄을 처리하지 못해 collector가 종료 상태를 거부한 버그를 수정했다. 실제 종료 작업의 다운로드·private 설정을 확인했으나 result ZIP은 없었다. 원본은 `outputs/kaggle-jev-rule-20260921/download-68fd6790`, 서버 로그는 같은 출력 폴더의 `execution-logs.txt`다. 관련 테스트 20개 통과. 신규 제출은 기존 기록을 덮어쓰지 않고 dev_open rule 1회부터 점검한다.

Kaggle 새 자료 업로드가 12분 넘게 완료 응답 없이 머물러 그 로컬 업로드 프로세스만 중단했다. 원격 Dataset 생성 여부는 미확정으로 보존했고 중복 생성하지 않았다. 검증된 기존 private Dataset을 재사용하도록 별도 kernel/고유 작업 ID를 만드는 경로를 추가했다. 새 작업 `changmin2026/ugrp-simulation-cabb25fb9ad7` version 1 제출 및 RUNNING을 확인했다. 실제 실행 소스는 기존 고정 SHA `06c4150364ea14f70cc10c9802446ed278723dde`, 새 제출/회수 코드 `e81ff1c`다. dev_open rule 1회이며 정식 36회 재개 완료는 아니다. 관련 테스트 21개 통과. 서버 제한 1800초, 로컬 회수 감시는 최대 2400초 후 종료하며 결과 ZIP·내부 해시와 실패 로그를 구분한다. 원본·현재 상태는 `outputs/kaggle-rule-reuse-20260921`에 보존한다.
