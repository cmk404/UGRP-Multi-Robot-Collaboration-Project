# 공동 하중 회전·고정 벽 우회

두 로봇이 빔을 든 상태에서 좌·우 90도 회전하고, 고정 벽을 돌아 목적지에 내려놓는
고정 조건 검증을 마쳤다. **성공 3건 모두 명시적인 `--impratio 10` 조건**이다.
기존 기본값 `impratio=1`에서는 짐이 미끄러져 실패했다. 기본 실행기와 기존 물리 설정은
변경하지 않았으며 새 실행기의 기본값도 1이다.

[검토 영상: 좌회전 → 우회전 → 벽 우회](media/pair-navigation-demo.mp4)는 약 61초다.
원본 관찰 영상의 4배속이며 작업 영역만 잘랐다. 보간·생성 동작은 없고 원본의 SIM 시각
표시를 유지했다. 실제 제어에 사용한 카메라와 화각은 바꾸지 않았다.

## 최종 결과

실행 코드: `13d833964335b04766cc8c7c8b72ddc5b70f4192`.
기반 main: `1b3b576292e4a3c0b4a66c0af6e56152fc3aed21` (통합 PR #43).
최종 6조건은 같은 실행 SHA와 고정 파지 후 8초 대기를 사용했다.
지도별 1회 시험이며 반복·새 시작 상태 일반화 실험이 아니다.

| 조건 | impratio | 결과 | 운반 SIM초 | 판단 회차 | 목표 위치 오차 | 목표 방향 오차 | 최소 lift |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 좌 90도 | 10 | 도착·내려놓기 통과 | 32.1 | 156 | 0.11 cm | 0.13도 | 7.33 cm |
| 우 90도 | 10 | 도착·내려놓기 통과 | 31.9 | 155 | 0.35 cm | 0.46도 | 7.34 cm |
| 고정 벽 우회 | 10 | 도착·내려놓기 통과 | 96.7 | 479 | 2.22 cm | 0.47도 | 5.48 cm |
| 막힌 통로 | 10 | 경로 없음으로 공동 정지 | 1.9 | 10 | 도착하지 않음 | — | 7.84 cm |
| 좌 90도 기본 설정 | 1 | 미끄러짐 후 영상 이탈 감지·정지 | 2.1 | 11 | 도착하지 않음 | — | 0.49 cm |
| 정지 유지 후 공중 해제 | 10 | 60초 유지·해제 진단 통과 | 별도 유지 60초 | 300회 정지 명령 쌍 | — | — | 6.64 cm |

세 성공 실행의 실제 빔 회전은 각각 +90.127도, -90.460도, 우회 후 원방향 대비 +0.468도다.
벽 조건은 공동 회전하여 벽 아래쪽을 지나고 반대쪽에서 방향을 되돌린다. 벽 위치는
`(0.1, -2.0) m`, 크기는 `0.1 × 0.2 × 0.3 m`이며 기존 창고 물체는 그대로다.
막힌 통로는 벽 폭만 1.2 m로 늘렸다. 통로 거부는 작은 영상 보정 전진 후 정지한 결과로,
목표 도착이나 내려놓기 성공으로 합산하지 않는다.

- 세 성공 실행의 운반 10 Hz 샘플 322/320/968개 모두 두 로봇의 양손 접촉, lift 3 cm 이상,
  기울기 10도 이하, 지도 경계를 통과했다. 운반의 모든 2 ms 물리 tick에서 벽·다른 물체
  충돌은 0이었다. 모든 단계에서 weld OFF, 마지막 내려놓기 후 바닥 접촉·손가락 해제를 확인했다.
- 공중 해제 진단은 600개 유지 샘플 전부 양손 접촉을 유지했고, 팔을 내리지 않고 집게를
  열자 6.65 cm 떨어졌다. 마지막 10개 해제 샘플은 바닥 접촉과 손가락 접촉 없음이다.
- 최종 내비게이션 5건의 **811회 × 두 로봇** 판단·허가·발행 명령을 저장된 RGB에서
  재계산해 모두 일치했다. 5건의 파지 입력과 별도 해제 진단의 파지 입력 감사도 통과했다.
- 전체 SIM 시간/실제 실행 시간은 좌회전 62.01/47.31초, 우회전 61.81/37.37초,
  벽 우회 126.61/105.88초다. 동시 실행·하드웨어 부하가 달라 벽시계 시간을 속도 우위로
  해석하지 않는다. 진단 실행기의 벽시계 시간은 계측하지 않았다.
- 외부 LLM 호출·토큰·비용은 전 조건 0이다. 판단 회차는 두 로봇이 한 쌍으로 판단한 횟수이며
  회차마다 각자 명령 1개를 발행한다. 파지·도착 후 정지·내려놓기는 이 회차에서 제외한다.

정확한 수치·설정·모든 실패는 [results.json](results.json), 최종 요약은
[final-results.csv](final-results.csv), 사전 조건 변경 기록은 [protocol.md](protocol.md)에 있다.

## 구현과 관측 경계

`harness/pair_navigation.py`의 두 로컬 실행기는 별도 기억과 자기 명령을 유지한다.
고정 top RGB의 원래 노란 바퀴 외관과 주황 빔을 추적하고, 짧은 전진의 영상 변화를 통해
초기 방향을 추정한다. 각자 own RGB도 받고 검증·보관하지만 현재 고전 영상 추적기는
공용 top RGB를 사용한다. 자체 카메라로 파지 상태를 인식했다고 주장하지 않는다.

승인된 사전 정적 지도와 고정 카메라 보정으로 두 차체·팔·짐 전체의 회전 공간을 검사한다.
격자 경로와 모든 지름길은 이동·회전 중 차지하는 영역을 확인한다. 같은 지도·경로 해시와
양쪽 최신 준비 보고가 있을 때 `PairCarrySync`가 GO를 허가한다. 현재 위치·방향·진행은
RGB로 보정하며 발행 명령을 실제 위치로 취급하지 않는다. 가려진 빔은 보이는 조각의
중심 대신 가능한 전체 중심 구간과 축 방향을 사용해 잘못된 이탈 판정을 줄였다.

정답 위치, 측정 관절, 접촉, 충돌, 성공 평가는 `evaluation-only.jsonl`과 종료 후 평가에만
기록한다. 배우 실행기는 이를 읽지 않는다. 정확한 입력·판단 재생은
`scripts/audit_pair_navigation.py`, 순수 사후 평가는 `scripts/evaluate_pair_navigation.py`다.
공중 해제는 정해진 명령의 별도 물리 진단이며 학생 정책 성공 건수에 넣지 않는다.

[사후 기하 비교](validation/geometric-comparison.json)에서 벽으로 향하는 직선은 하중 전체
공간 검사를 통과하지 못했고, 선택한 우회의 각 구간은 통과했다. 두 차체가 각자 중심에서만
90도 회전하면 이번 강체 대형 회전에 필요한 차체 중심과 약 38 cm 차이가 난다. 이는
기하 비교이며 두 단순 방식의 물리 실행·성공률 비교는 수행하지 않았다.

## 접촉 설정과 실패 이력

최초 회전은 로봇만 회전하고 빔이 떨어졌다. 같은 파지의 정지 유지도 8.9초 후 실패했고,
손목 +100/-100 pulse 진단도 7.8/8.5초 후 실패했다. 손목 변경은 최종 조건에서 제외했다.
이에 물성·집게 힘과 분리하여 MuJoCo 연성 접촉의 마찰 임피던스를 비교했다.
[MuJoCo solver 설명](https://mujoco.readthedocs.io/en/latest/modeling.html#solver-settings)에
근거한 수치 모델 비교이며, 실제 로봇 접촉에 맞춘 검정·보정 결과는 아니다.

`impratio=10`은 접촉의 수치적 마찰 임피던스를 바꾼다. 마찰계수, 질량, 형상, 팔/집게 힘,
관절 제한, actor 카메라는 유지했다. NoSlip 반복은 0이고 weld는 꺼져 있다. 기본값 1과
10의 빈 공간 최종 실행 사이에서 카메라·질량·관성·형상·마찰 해시는 모두 동일하며,
기록된 solver 옵션은 impratio만 다르다. 성공의 물리 조건 차이를 숨기지 않는다.

| 기록 | 당시에 관찰한 결과·후속 조치 |
| --- | --- |
| `dev-left-01`, `dev-hold-01`, `dev-hold-wrist100`, `dev-hold-wristminus100` | 기본 설정에서 회전·정지 모두 미끄러짐. 손목 변화로 해결되지 않음 |
| `dev-hold-impratio10`, `dev-hold-impratio100` | 60초 높이는 유지했으나 초기 접촉 누락 각각 3/1샘플로 엄격 기준 실패 |
| `dev-left-impratio10` | 파지 후 차체 간격이 달라져 초기 외관 인식 실패. 실제 RGB 외관 탐색으로 수정 |
| `dev-left-impratio10-v2`, `dev-right-impratio10` | 이전 5초 대기·당시 운반 기준 통과. 최종 코호트와 분리 |
| `dev-wall-impratio10`, `dev-wall-hold` | 운반은 통과했으나 출발 전 21.1/21.2/21.5 SIM초 접촉 누락. 동일 8초 고정 대기와 직전 2초 안정 판정을 사전 고정 |
| `final-left`, `final-wall` (`89cee62`) | 좌회전 통과. 벽 조건은 실제 파지·높이를 유지했지만 빔 가림 때문에 55.3초에 잘못 정지. 중심 구간 판정으로 수정 후 전체 최종 조건 재실행 |
| `final-*-v2` (`13d8339`) | 위 표의 6조건. 과거 실패를 삭제하거나 최종 성공률에 섞지 않음 |

기본 설정 실패에서는 RGB 정지가 낙하 자체를 예방하지 못했다. 미끄러짐의 조기 인식·복구,
다양한 시작 파지, 미지 장애물, 다양한 벽·하중, 지연 주입, 재시작, 메시지 버스·공유 자원
통합은 아직 남았다. 이번 내려놓기는 기존 시연 재생이다. 독립 LLM 협업이나 실물 성공은
검증하지 않았다. Issue #39 T4/T5의 고정 조건 증거이며 전체 작업 목록 완료가 아니다.

## 검증·환경·저장

- Python 3.12.13 / MuJoCo 3.12.0 / macOS 27 arm64. 기존 `.venv-sim-worker-mac` 사용.
- 실행 SHA에서 `python scripts/run_ci_tests.py`: **615 passed, 154 subtests passed**.
  [로컬 CI 로그](validation/local-ci.log). 회전 중 모서리 충돌, 막힌 경로, 외관 가림,
  명령과 관측 구분, 오래된 보고·다른 계획 거부, 위조 입력 해시·평가 판정 등을 포함한다.
- 실제 영상의 시작·중간·끝과 비교 진단을 확인했다. 정확한 추출 시각과 검토 범위는
  [visual-review.md](visual-review.md), [선택 프레임 목록](media/selected-frames.json)을 따른다.
- 원본은 `/Users/changmin/projects/ugrp-worktrees/pair-loaded-navigation/outputs/pair-navigation/`에
  로컬 보관한다. [raw-manifest.json](raw-manifest.json)과 압축 JSONL
  [전체 파일 해시](raw-hashes.jsonl.gz)는 모든 19회 원본과 검토 산출물 위치·크기·SHA256을 연결한다.
  **원본 RGB·전체 영상·로그는 원격 백업하지 않았다.** GitHub에 포함하는 영상은 약 1.1 MB
  검토 편집본 하나다. 기존 모델 ZIP은 이미 버전 관리되어 있으며 중복 추출물은 원본 목록에서 제외했다.
- 이 작업에서 시작한 실험 세션은 종료했다. 다른 작업의 프로세스는 정리 대상에 포함하지 않았다.

## 재현

실행 소스·프로토콜을 커밋한 깨끗한 체크아웃에서 실행한다. 정확한 후보는 위 실행 SHA이며
그 뒤 결과 보관 커밋은 실행 소스를 바꾸지 않는다. Ubuntu 환경 준비는
[프로젝트 안내](../../docs/ubuntu_quickstart.md)를 따른다. 이번 긴 공동 운반 코호트는 Mac에서
검증했으며 Ubuntu의 기본 렌더링 CI를 이 코호트 재현 성공으로 대신하지 않는다.

모델 아카이브와 각 파일 해시를 검증하고 grasp 부분만 추출한다.

```sh
python3 - <<'PY'
from pathlib import Path
import hashlib, json, zipfile
p = Path('experiments/2026-09-10-rgb-varied-start')
m = json.loads((p/'models-manifest.json').read_text())
assert hashlib.sha256((p/'models.zip').read_bytes()).hexdigest() == m['archive_sha256']
assert m['archive_sha256'] == 'd6d421c2d35ed448c4492592220b3e3141d346f5e576ebbdf5a7a3c787b7e635'
with zipfile.ZipFile(p/'models.zip') as z:
    for rec in m['files']:
        name = rec['archive_path']
        if name.startswith('models/grasp/'):
            raw = z.read(name)
            assert hashlib.sha256(raw).hexdigest() == rec['sha256']
            target = Path('outputs/pair-navigation/runtime')/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
PY
```

Mac에서는 기존 환경의 python과 mjpython을 사용한다. Ubuntu에서는 아래 두 경로를
설치한 `.venv-dev/bin/python`으로 바꾼다. 출력 이름은 매번 새 이름을 사용한다.

```sh
TASK_PY=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python
TASK_SIM=/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython
run_case() {
  "$TASK_PY" scripts/ugrp_session.py run "pair-$1" -- "$TASK_SIM" scripts/run_pair_navigation.py \
    --map "experiments/2026-09-14-pair-navigation/maps/$2.json" \
    --grasp-model-dir outputs/pair-navigation/runtime/models/grasp \
    --out-dir "outputs/pair-navigation/$1" --impratio "$3" --budget "$4"
}
run_case left-NEW open-left-90 10 500
run_case right-NEW open-right-90 10 500
run_case wall-NEW wall-detour 10 650
run_case blocked-NEW blocked-passage 10 500
run_case default-NEW open-left-90 1 500
```

기본 설정 비교의 이탈 정지는 exit 1이다. 결과의 실패·정지 원인을 확인한다.
각 내비게이션 실행을 다음처럼 감사한다(감사는 물리를 다시 실행하지 않는다).

```sh
"$TASK_PY" scripts/audit_pair_navigation.py outputs/pair-navigation/left-NEW \
  --grasp-model-dir outputs/pair-navigation/runtime/models/grasp
"$TASK_PY" scripts/ugrp_session.py run pair-release-NEW -- "$TASK_SIM" scripts/probe_pair_navigation_hold.py \
  --map experiments/2026-09-14-pair-navigation/maps/open-left-90.json \
  --grasp-model-dir outputs/pair-navigation/runtime/models/grasp \
  --out-dir outputs/pair-navigation/release-NEW \
  --impratio 10 --seconds 60 --post-grasp-settle 8 --drop-test
```

`collect_results.py`는 원래 로컬 19회 기록을 묶는 사후 전용 도구다. 새 실험의 결과를
기존 코호트에 덮어쓰는 데 사용하지 않는다. `result.json`의 물리 성공 여부와 관측 경계를
확인한 뒤 영상도 검토한다.
