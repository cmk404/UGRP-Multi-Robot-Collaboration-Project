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

## 회수된 원격 진단 결과 및 미완료 항목

Colab dev_open 3회(Jev/Gemini/rule) 전부 `vision_recovery_exhausted`, 실제 주행 모델 호출 0회다. 24회 관측에서 모두 `cyan_target_missing_or_ambiguous`가 발생했다. 합성 상태 API 왕복 성공은 이 주행의 모델 성공 근거가 아니다. supervisor의 `development_transport_failure` 명칭은 정확하지 않다. 주행에서 모델 호출에 도달하지 못한 공통 RGB 관측 실패이며, 108회 holdout은 시작되지 않았다. 원본 ZIP 431개 파일 해시를 검증해 회수했고 relay/Colab을 종료했다.

Kaggle 재개 dev_open rule 1회도 동일 관측 실패로 끝났다. wall 202.456초, 9.8 SIM초, 발행 명령 31개, 과제 성공 false다. 서버 COMPLETE 및 프로세스 exit0은 실행·회수 성공이며 과제 성공을 뜻하지 않는다. ZIP/내부 해시 검증을 완료했다. `remote-diagnostic-outcome.json`에 결과·원본 위치·해시를 기록했다. 현재 새 Colab/Kaggle 본 비교 실행은 없고, ACT 128/256 재평가와 Jev/Gemini holdout은 여전히 미완료다. 512는 사용자 요청대로 보류다.

## 중단 복구와 실제 주행 재개 (2026-09-21, 후속)

RGB 청록 목표 검출이 바닥의 JPEG 무늬를 두 번째 목표로 해석하는 원인을 재현했다.
3×3 내부 검사와 밝은 청록 내부 확인을 추가하고 원래 중심 좌표·진짜 두 목표의
모호성 거부를 유지했다. GPU 78장과 Kaggle 71장 재생은 모두 단일 목표로 검출됐다.
T4가 배정됐지만 Mesa llvmpipe였던 EGL을 기존 NVIDIA 라이브러리 등록으로 복구했다.
실제 새 context는 NVIDIA/Tesla T4로 확인했다. `setup_colab_egl.py`는 그 절차를
명시적으로 재현하고 다른 기존 등록을 덮어쓰지 않는다.

고정 소스 `9c6c36d`의 Colab dev_open은 rule/Jev/Gemini 각각 1/1 성공했다.
시간은 rule 89.279초, Jev 123.335초, Gemini 177.037초이며 모두 명령 77개다.
Jev/Gemini는 각각 실제 API 12회 호출했다. 접근·정렬 범위이며 파지/운반 성공이
아니다. 한 개발 배치의 결과로 모델의 일반화 우열을 확정하지 않는다.
결과 ZIP SHA256 `15bda218629efec87adc5c0daa35ba34ded0dbd3a445374735b176bc52443bf3`,
내부 1168파일 해시를 검증했다. 이 고정 소스로 holdout 108회와 후속 단계가
실행 중이다. 전체 3.5시간 상한은 완료 예상이 아니며 끝나지 않은 단계를 완료로
표시하지 않는다.

Kaggle 수정 진단(source `2761693`) rule 1/1 성공, 430.991초/명령76개다.
기존 private Dataset을 재사용하는 Git object pack 경로를 실제 완료/회수로 검증했다.
초기 Git bundle의 shallow prerequisite 오류도 실패 로그로 보존했다.
Kaggle holdout rule36은 `b070939`로 고정해 새 private kernel에 제출했고 서버
RUNNING을 확인했다. CPU/OSMesa와 Colab/NVIDIA의 시간을 동등 조건으로 합산하지
않는다. `--timeout-seconds 14400`의 유한한 제한과 결과 회수를 별도로 기록한다.

ACT 128/256은 기존 8000-step 가중치 8개와 canonical 데이터의 해시를 확인해
Colab에서 36회 평가를 재개했다(source `04f129c`). 재학습은 없고 512는 보류다.
현재 부분 결과에는 교사/학생 모두 APPROACH의 RGB support 거부가 있다.
프레임과 저장된 배경을 비교하면 새 전경 영역이 학습 범위를 크게 벗어난다.
지원을 임의로 넓히거나 정답 보정을 넣지 않았다. 아직 ACT 운반에 도달하지 않은
실패를 ACT 운반 성공률로 해석하지 않는다. 전체 결과/영상 회수는 진행 중이다.

`repair-20260921.json`은 원본 위치·결과 해시·비용 미기록·진행 상태를 보존한다.
완료된 새/이전 실패 진단 11회를 기본 체크아웃 `outputs/tensorboard/0921-복구`에
중복 해시 검사 후 변환했고 실제 이벤트 수치·이미지 로딩과 native HParams 표를
확인했다. 기존 기준선 9회와 코호트를 나누었다. 고정 카드 5개와 저장된 HParams
열을 화면에 적용했다. 다른 작업 소유 미디어 서버의 새 영상 등록은 미완료다.
새 원본 MP4 11개와 manifest는 보존되어 있다.

## 원격 수집 복구 — 2026-09-21 14:50 KST 이후

이 절은 위 시점별 RUNNING 설명을 대체하는 후속 점검이다. **전체 비교 완료는 아니다.**

- Kaggle OAuth의 만료 판정이 실제 만료 30분 뒤에 갱신하는 SDK 동작을 확인했다. 제출/조회 전 만료 5분 이내 갱신하도록 수정했고 실제 인증을 확인했다.
- Mac은 13:44:49에 덮개 닫힘 잠자기에 들어가 14:02:40에 깨었다. 해당 구간에 기존 Colab 연결이 끊겼다. 그러나 Oracle에서 실행한 별도 세션도 약 12분 뒤 서버에서 사라졌다. **Mac 잠자기만을 종료 원인으로 확정하지 않는다.** 정상 keep-alive 호출 이후 404가 기록됐으며 정확한 서버 종료 사유는 제공되지 않았다.
- 실행·중계·회수를 기존 사용자 소유 Oracle 서버로 옮겼다. 무거운 시뮬레이션은 Colab/Kaggle에서만 한다. 작업별 유한 시간/자식 정리/종료 후 임시 인증 사본 삭제를 적용했다. 인증 값은 소스/결과/Git에 없다.
- 완료 trial마다 원본 ZIP과 내부 파일 해시를 즉시 회수한다. 단순 RUNNING/연결 불능/회수 종료/코호트 완료를 구분한다. 미회수 trial을 완료로 세지 않는다.
- Oracle 첫 비교(source `10582ae`)에서 개발 3회와 holdout 11회를 검증 회수했다. 개발은 세 정책 모두 성공, holdout의 규칙 2회 성공과 모델 9회 실패를 보존했다. **모델 9회 실패는 MailboxTimeout이며 모델 판단력 비교가 아니다.** 응답 파일 전송 시간이 API 30초 제한에 포함된 문제를 수정했다. 모델 자체 응답 제한은 유지하고 전달 여유를 분리했다.
- 새 비교(source `5a51a35`)는 실제 Jupyter 실행이 끝날 때까지 Colab CLI exec 연결을 유지한다. 개발 초기의 잘못된 구버전 relay 선택도 발견해 해당 프로세스만 교체했으며 초기 기록을 버리지 않는다. 이 개발 단계의 실행 시간은 공정한 속도 순위로 쓰지 않는다.
- Kaggle GPU 두 실행은 환경 진단에서 실패했다. 별도 짧은 두 probe에서 명시적 T4와 GPU 이미지/노트북 요청을 저장했지만 실제 장치 0개, NVIDIA 라이브러리 없음, `torch 2.10.0+cpu`를 회수했다. 설정상 GPU를 실제 GPU로 보고하지 않는다. 기존 rule36 CPU 작업은 별도 유한 collector가 계속 회수한다.
- ACT는 Colab의 실제 NVIDIA T4/EGL을 확인했다. 첫 설치는 시스템 ensurepip 부재로 실패하여 원본을 회수했다. `uv venv --seed`로 수정한 source `f1090df`를 준비했다. 동시 GPU 생성은 TooManyAssignmentsError로 거부되어 현재 비교 종료 뒤 한 번 할당하는 유한 작업 큐를 사용한다. 자동 재접속/반복 재제출/유료 자원 구매는 없다. 512px는 계속 제외한다.
- TensorBoard에는 개발/부분 holdout과 환경 실패를 나누어 추가했다. 설치 실패는 `process/exit_code`이며 로봇 실패 지표를 만들지 않는다. 18개 새 스냅샷의 실제 수치, native HParams 열, MP4 16개 HTTP 206 범위를 확인했다. 뒤이어 회수한 ACT 설치 실패도 별도 추가한다. 다른 작업의 6006 뷰어는 유지하고 작업 소유 6009 미디어 서버가 완료 manifest만 읽는다.

세부 원본/해시는 `independent-recovery/status.json`에 있다. Oracle 상태 위치는
`/home/ubuntu/ugrp-jobs/recovery-20260921/{colab-c,act-colab-b,rule}`이며 `act-queue.json`은 GPU 인계 대기 상태다. 이 문서 작성 뒤의 진행은 실제 상태 JSON을 다시 확인해야 한다.

추가 검증: 최신 수정의 전체 오프라인 회귀 1218 passed, 3 skipped, 191 subtests passed(73.02초). 관련 클라우드/중계/TensorBoard 검사 29 passed. 기존 push의 GitHub CI 여섯 종류는 모두 성공했으며, 이후 커밋의 CI는 별도 재확인 대상이다.

### Kaggle GPU 미배정 원인 확인 — 14:58 KST

강 프로필의 기존 Google 계정으로 Kaggle `changmin2026`에 로그인해 실제 Notebook Session options를 확인했다. **전화번호 미인증으로 GPU/TPU·인터넷 접근이 잠겨 있다.** `Internet off`는 disabled였고 `Get phone verified` 안내가 표시됐다. 진단 노트북의 GPU 요청 메타데이터와 실제 CPU 실행이 다른 이유를 설명하는 계정 차원의 차단이다. 본인 인증 폼만 열었으며 전화번호·문자 코드 입력이나 CAPTCHA 처리, 새 계정 생성은 하지 않았다. 사용자에게 인증 완료를 요청했다. Draft Session은 off이며 별도 브라우저 실행은 시작하지 않았다. 인증 전에는 Kaggle GPU 재제출을 반복하지 않는다. Colab 비교와 이후 ACT 인계는 이 인증과 독립적으로 진행한다.

15:00 KST 보고용 스냅샷: 전송 수정 후 개발 3회와 본 평가 6회를 회수해 모두 접근/정렬 성공을 확인했다. 기존 전송 실패였던 Jev 네 조건과 Gemini 가림 조건도 여기에 포함된다. 전체 108회 holdout 완료나 ACT 운반 성공은 아니다. 개발 중 relay 교체가 있어 개발 시간은 성능 순위에 쓰지 않는다. 새 9개 원본/해시·영상 HTTP 206·native TensorBoard 전체 40행과 비교 열을 확인했으며 `independent-recovery/comparison-checkpoint.json`에 고정했다. Kaggle 인증 화면은 사용자가 완료하도록 열어 두었다.

## Kaggle 인증 이후 GPU 검증 및 RGB 보정 복구

- `kaggle-authenticated.json`에 실제 T4 2개·CUDA·인터넷 확인과 실패/수정 이력을 보존했다. GPU 설치 오류와 학생의 물리 실패는 분리한다.
- NVIDIA 라이브러리 경로 수정 후 GPU 렌더링과 ACT 의존성 설치·접근 보정 완료를 확인했다. 접근 진단은 지원 범위 이탈로 실패했고, 파지 보정은 새 모델에 이전 마스크를 적용하던 오류로 중단됐다.
- 새 파지 보정 모델은 학습한 전체 RGB 특징을 유지하도록 수정했다. 입력 범위 검사·교사/학생 경계는 유지한다. 관련 37개, 전체 1,220개 테스트 및 191개 하위 검사 통과(3개 건너뜀).
- `feae35e09c441f438f87da6cf7d8f4acc4fef024` 소스로 비공개 Kaggle 재실행을 제출했다. 전체 ACT 평가 시작·완료나 모델 우열을 아직 주장하지 않는다. 512px는 제외한다.
