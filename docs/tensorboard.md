# TensorBoard로 UGRP 기록 보기

기존 ACT 학습·로봇 실행 기록을 **읽기 전용 스냅샷**으로 변환한다. 원본 JSON·영상·가중치를 수정하지 않고, 재학습·시뮬레이션·외부 모델 호출을 하지 않는다. 화면은 공식 TensorBoard이며 원본 MP4 링크만 작은 로컬 서버에서 제공한다.

## 설치와 실행

Mac에서는 기존 `.venv-sim-worker-mac`을 사용한다. 별도 시뮬레이션 환경은 만들지 않는다. Ubuntu는 기존 개발 환경 Python으로 같은 명령을 실행한다. TensorFlow와 PyTorch는 변환/뷰어 실행에 필요하지 않다.

```sh
.venv-sim-worker-mac/bin/python -m pip install -r requirements-observability.txt
.venv-sim-worker-mac/bin/python scripts/export_tensorboard.py \
  --source /absolute/path/to/act-training-model \
  --source /absolute/path/to/jev-cohort/rule-straight \
  --source /absolute/path/to/multi-object-run \
  --output outputs/tensorboard/review-NEW-ID --max-images 8

python3 scripts/ugrp_session.py run tensorboard-review -- \
  .venv-sim-worker-mac/bin/python scripts/run_tensorboard.py \
  --logdir outputs/tensorboard/review-NEW-ID
```

[TensorBoard](http://127.0.0.1:6006)을 연다. 원본 영상은 Text의 `media/` 링크에서 열린다. 기본 미디어 포트는 6007이다. 두 서버 모두 127.0.0.1에만 바인딩한다. 사용 후 실행 터미널에서 Ctrl-C 또는 `python3 scripts/ugrp_session.py stop tensorboard-review`로 종료한다. 로그인 시 자동 시작하거나 백그라운드 서비스를 설치하지 않는다.

Mac에서는 `scripts/open_tensorboard.command`를 Finder에서 실행해도 된다. 기본 체크아웃의 Python을 사용하며, `outputs/tensorboard` 아래에 여러 export collection이 있으면 `collection.json` 수정 시각이 가장 최근인 **export snapshot 하나**를 기본으로 연다. 이것은 편의를 위한 뷰어 선택이며 실제 실험 수행 날짜를 뜻하지 않는다. 특정 위치를 열려면 `UGRP_TENSORBOARD_LOGDIR=/path/to/review scripts/open_tensorboard.command`처럼 지정할 수 있다. 모든 collection을 한꺼번에 보려면 `scripts/run_tensorboard.py`를 `--prefer-latest-collection` 없이 직접 실행한다.

TensorBoard의 기존 run selector는 run이 40개를 넘으면 기본 선택을 모두 해제한다. 이 상태에서는 Text 탭에 `decisions`, `evaluation`, `provenance` 같은 그룹 제목만 보이고 실제 카드 내용은 비어 보일 수 있다. Finder launcher가 최신 export collection 하나를 기본으로 고르는 이유가 이것이다. 전체 logdir를 일부러 열었다면 왼쪽에서 run을 명시적으로 선택하거나 특정 collection으로 다시 실행한다.

미디어 포트가 이미 사용 중이면 launcher는 traceback 대신 포트 충돌을 명시적으로 보고한다. 기존 review 서버가 남아 있으면 그 서버를 종료한 뒤 다시 실행한다. 다른 미디어 포트를 사용하려면 이벤트 안의 `media/` 링크도 같은 포트를 가리키도록 해당 snapshot을 그 포트로 다시 export해야 한다.

## 무엇을 어디서 보는가

| TensorBoard 탭 / 태그 | 내용과 경계 |
|---|---|
| Scalars / `training`, `development` | 기록된 학습 step별 loss·개발 오차·완료 분류 지표. 전체 step을 보간하지 않음 |
| Scalars / `result` | 실행별 실제 시간·SIM 시간·발행 명령·호출·토큰·비용. 없는 값은 기록하지 않음 |
| Scalars / `rgb_estimate` | RGB에서 추정한 거리·방향. 정답 좌표가 아님 |
| Scalars / `claims` | 제어기/프로토콜 완료 주장. 사후 성공과 분리 |
| Scalars / `evaluation/reported_success` | result.json의 명시적인 bool 성공 필드. 어떤 필드인지 provenance에 기록. 실험별 판정 범위가 다름 |
| Images / `observations` | 결정에 연결된 자기·TOP 입력. 기본 최대 8개 결정을 균등 표본 추출하며 첫/마지막 포함 |
| Text / `decisions` | 결정 step별 상태·요청·행동/응답. 실행한 명령과 실제 물리 결과를 동일시하지 않음 |
| Text / `result`, `evaluation` | 종료 이유와 사후 판정 원문. 시뮬레이터 정답은 이 사후 평가 영역에만 존재 |
| Text / `provenance` | 원본 폴더·실행 SHA·조건·누락·변환 시각 |
| Text / `media` | 변환하지 않은 원본 MP4를 로컬 재생 화면으로 연결 |
| HParams | 정책·case·SHA·seed·clock·설정 해시와 개별 실행 지표 표. 조건을 필터링한 뒤 비교 |

HParams의 **session status=success는 이벤트 가져오기 완료**를 뜻한다. 로봇 성공은 `evaluation/reported_success`와 그 출처 필드로 확인한다. 서로 다른 조건의 성공률을 자동 합산하지 않는다. 빠르게 실패한 실행의 시간을 성능 개선으로 해석하지 않는다. ACT 설정이 있어도 운반 결정에 진입하지 않았다면 `act_carry_decision_rows=0`이며, ACT 운반 성공률의 분모로 자동 포함하지 않는다.

### 시간축

기본으로 **STEP 축**을 사용한다. 학습은 원래 optimizer step, 실행은 기록된 결정 순번이다. SIM 시각은 별도의 `execution/sim_time_s` 곡선으로 제공한다. 화면의 기본 smoothing은 원자료를 시각적으로 평활하므로 정확한 값 비교에는 0으로 설정한다. 뷰어는 태그당 최대 10,000 scalar 표본을 읽고, 이벤트 파일에는 변환한 전체 값을 보존한다.

과거 기록에 실제 시작 시각이 없으므로 **TensorBoard의 WALL/RELATIVE 시간은 변환 시각 기준**이다. 파일 수정 시각에서 실행 시작 시각을 만들어내지 않는다. 연속 영상의 플레이어 시각도 SIM과 자동 정렬하지 않는다.

### 지원되는 원본

- ACT 학습: `report.json`의 `progress` 배열 또는 로봇별 `progress.json`.
- Jev 직접 운동: `result.json` + `turns.json` + 참조 RGB.
- 다중 물건: `result.json` + `actor-static-task.json` + `turn-*.json` + `runtime/<request_id>.json`.
- ACT 공동 운반: `result.json` + `pair-decisions.json`의 `act_carry` 행. 물리 로봇 ID와 모델 슬롯 구분.
- ACT 종료 오프라인 감사: `termination-audit.json`과 해시가 일치하는 `predictions.json`. 예측을 다시 감사해 `offline/episodes`, 조기 정지·종료 누락 횟수, `offline/termination_pass`를 표시한다. `offline_pass`도 물리 성공이나 기본 제어기 채택을 뜻하지 않는다.
- 클라우드 실행: 종료된 `run.json`의 프로세스 종료 코드와 실제 시작/종료 시간. GPU 설치 실패는 로봇 실패로 바꾸지 않는다.
- 기타 `result.json`: 명시된 결과 지표와 출처만 변환. 형식을 모르는 내부 로그를 임의로 해석하지 않음.

`--source`는 한 실행/학습 폴더를 지정한다. 코호트 상위 폴더의 `results.json`이나 `report.json.runs`를 자동으로 따라가지 않는다. 원하는 하위 실행을 명시적으로 반복 지정한다. 현재 실행 중인 폴더보다 결과가 완성된 폴더를 선택한다.

## 저장과 갱신

- 기존 출력 폴더에 덮어쓰기·추가 쓰기를 거부한다. 새 스냅샷 ID로 변환한다.
- `manifest.json`에 변환기 SHA·dirty 여부, 실행 메타데이터, 읽은 원본 파일의 SHA256·크기·mtime, 이미지 누락/해시 불일치를 남긴다.
- 읽은 원본이 변환 도중 바뀌면 이벤트를 게시하지 않고 실패 manifest를 남긴다. 미완성·손상 JSON은 경고 또는 변환 실패로 구분한다.
- 알려진 인증 필드는 가리고, 로그는 HTML을 실행하지 않는 텍스트로 기록한다. 원본 영상 서버는 완료된 export manifest에 등록된 두 파일명만 허용하고 범위 요청을 지원한다. 크기/mtime이 바뀐 영상은 재변환 전 재생을 거부한다.
- JSON은 파일당 64 MiB 상한, 이미지 결정 표본은 `--max-images 0..100`. `0`은 숫자·텍스트만 변환한다.
- 카메라·영상 원본은 로컬에 그대로 있다. manifest의 해시와 TensorBoard 이벤트는 raw 전체의 원격 백업이 아니다. 원본 영상을 다른 컴퓨터로 옮기면 manifest의 로컬 경로도 달라진다.
- TensorBoard 자체의 새로고침은 이벤트 폴더 갱신을 읽는다. **현재 구현은 스냅샷 변환이며 JSON을 자동 감시하지 않는다.** 로봇 실행·학습 중인 소스에 로깅 코드를 삽입하지 않는다.

## 검증

```sh
.venv-sim-worker-mac/bin/python -m pytest -q tests/test_tensorboard_export.py tests/test_tensorboard_launcher.py
```

이벤트를 TensorBoard EventAccumulator로 다시 읽어 실제 scalar·text·image, HParams 메타데이터, 원본 불변성, 시간·누락값·완료 주장 분리, 영상 Range/경로 제한을 확인한다. launcher 회귀는 여러 export collection이 있을 때 최신 **export snapshot**만 고르는지와 손상·빈 collection을 무시하는지 확인한다. CI의 `tensorboard-export`가 선택 의존성을 설치해 실행하며 일반 회귀 환경에서는 선택 의존성이 필요한 테스트만 건너뛴다.

설계 참고: [TensorBoard 시작](https://www.tensorflow.org/tensorboard/get_started), [HParams 비교](https://www.tensorflow.org/tensorboard/hyperparameter_tuning_with_hparams), [PyTorch SummaryWriter](https://docs.pytorch.org/docs/stable/tensorboard.html). 실제 변환은 TensorBoard 2.21.0의 event protobuf를 사용한다.
