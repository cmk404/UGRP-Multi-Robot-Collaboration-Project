# 2026-09-26 자기 카메라 폐루프 문 통과 TensorBoard 스냅샷 (0926-zone-owncam-loop)

PR #178([`claude/zone-owncam-loop`](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/pull/178)) 자기 카메라 폐루프
문 통과 코호트 **22회 전체(실패 포함)**를 새 스냅샷 `outputs/tensorboard/0926-zone-owncam-loop`
(22 run)으로 변환했다. 새 실험·시뮬레이션·모델 호출은 없고 원본·기존 스냅샷·PR #178의 파일은
수정하지 않았다. 전체 수치와 검증 기록은 [verification.json](verification.json)에 있다.

- 원본(읽기 전용): `outputs/owncam-loop-20260925/{dev-a1,dev-a2,dev-a3,dev-a4,test}/<에피소드>/{result.json,manifest.json}` — **로컬 보관만이며 원격 백업이 아니다**
- 실험 기록(읽기 전용): PR #178 `experiments/2026-09-25-zone-owncam-loop/{results.json,raw_index.json,prereg.json,prereg_amendments.json,frozen_source.json}`
- 파생 뷰: `outputs/tb-owncam-loop-20260926/derived`(로컬) + `index.json`
- 변환·검증 소스: `84899d6`(`scripts/build_owncam_loop_views.py`, `scripts/verify_owncam_loop_snapshot.py`, 변환기는 main의 `scripts/tensorboard_tools/export.py` 그대로)
- 실행 소스: 고정 `9361a8d`(test는 기록만 추가한 `521ae7b`에서 실행, 실행 파일 바이트 동일)
- 뷰 설정: `outputs/tensorboard-view.json`의 `zone_owncam_loop_20260926` 키 (다른 키는 읽고 그대로 유지)

## run 이름과 코호트 경계

| run | 수 | 코호트 | 경계 |
|---|---|---|---|
| `dev-a1-*`, `dev-a2-*`, `dev-a3-*` | 10 | `dev-amended` | test 전에 dev 전용 수정(교정·둘러보기·이득)이 이어진 시도. 보고 dev가 아니며 test와 합산하지 않는다 |
| `dev-a4-*` | 6 | `dev-frozen` | 고정 소스 `9361a8d`의 마지막 dev. PR #178이 보고한 dev |
| `test-*` | 6 | `test-preregistered` | 사전 등록 test 분할, 고정 소스에서 1회 실행·재실행 없음 |

이름은 `<시도>-<box|nobox>-s<seed>`다(dev seed 31–33, test seed 41–43). 사후 분석(멈춤 관성
재적합 후보)은 사전 등록 분석이 아니고 원본 JSON이 없어 **수치로 변환하지 않았다**. 조건 문자열과
`limits`에 그 사실을 남겼다.

## 스냅샷이 보여주는 값

| 태그 | 뜻 |
|---|---|
| `evaluation/reported_success` | 사전 등록 게이트 `episode_pass` = R1(도착 선언 시 GT–W ≤ 0.10 m) ∧ R2(벽 접촉 0) ∧ R3(문 영역 추정 p90 < 0.06 m) (+box R4 상자 z > 0.045 m). GT는 평가 전용 |
| `claims/protocol_complete` | 학생이 스스로 낸 도착 선언. 사후 게이트 통과와 분리한다(선언 20/22, 통과 17/22) |
| `result/sim_s` | **학생 구간** SIM 시간. 교사 구간(box 조건 20.0–34.1 s)은 Text에만 두고 성공에 넣지 않는다 |
| `result/commands` | 학생이 발행한 명령 수. 실제 관절 상태·이동 성공이 아니다 |
| `result/model_calls` | 0. 이 폐루프에 외부 모델 호출이 없다(결정적 입자 필터 + A* 재계획). 모델 응답 시간 없음 |
| `result/wall_s` | 프로세스 wall 시간(부하 평균은 Text `evaluation/referee_only`) |
| `evaluation/robot_robot_contact_samples` | 학생 구간 동료 로봇 접촉 기록 수(22회 모두 0) |
| Text `evaluation/referee_only` | R1 거리·R3 p90/프레임·R4 상자 z·둘러보기 사유별 횟수·태그 가시율·접촉·교사 SIM·부하 평균·해시·환경 |
| Text `result/summary`, `provenance/source` | 종료 이유(`arrived`/`time_limit`)와 입력 경계·코호트·원본 경로·해시 |
| HParams | `case`=split\|시도\|carry\|코호트, `policy`=`owncam_drive(particle_filter+map_goto A*)|pose_source=own_wrist_fisheye_tags_v2+particle_filter`, `seed`, `outcome`, `source_sha`, `clock` |

## 변환한 결과 (원본 값 그대로)

| 시도/조건 | 통과 | 학생 SIM (s) | 명령 수 |
|---|---|---|---|
| dev-a1 box / nobox | 0/1 · 1/1 | 140.1 · 60.2 | 2216 · 807 |
| dev-a2 box / nobox | 0/1 · 1/1 | 101.2 · 68.7 | 1582 · 892 |
| dev-a3 box | 2/3 | 116.5 / 154.9 / 240.1 | 1840 / 2424 / 3719 |
| dev-a3 nobox | 3/3 | 76.3 / 84.7 / 102.2 | 1063 / 1147 / 1322 |
| **dev-a4 box** (보고 dev) | 3/3 | 75.3 / 104.4 / 187.5 | 1164 / 1589 / 2825 |
| **dev-a4 nobox** (보고 dev) | 3/3 | 76.5 / 83.1 / 120.4 | 1065 / 1131 / 1624 |
| **test box** | 1/3 | 74.0 / 102.2 / 240.1 | 1151 / 1565 / 3637 |
| **test nobox** | 3/3 | 85.1 / 111.1 / 76.3 | 1151 / 1471 / 1063 |

test box 실패는 s42(R3 p90 8.0 cm, 도착은 함)와 s43(240 s 한도, 도착 선언 없음)이다. 분할·조건·
코호트의 성공률을 합산하지 않는다.

## 검증 범위

- 22/22 run을 `EventAccumulator`로 다시 열고 **scalar 154개를 파생 뷰가 아니라 원본
  `result.json`·`manifest.json`에서 다시 계산해 대조**했다. Text 카드의 게이트·둘러보기·접촉·교사 SIM
  값 198개도 원본과 대조했다. 불일치 0, 변환 경고 0, 미완성 manifest 0.
- 파생 뷰 생성 단계에서 원본 `result.json`·`manifest.json`의 SHA-256이 PR #178 `raw_index.json`과
  같은지, 기록 표(`results.json`)의 27개 항목이 원본과 같은지 확인했다(다르면 변환 거부).
- 실행 중인 **공용 서버(PID 9291/9293, 2026-09-22 13:58 시작, logdir `outputs/tensorboard`,
  포트 6006/미디어 6009)**가 새 22 run을 모두 나열하고(전체 548) 같은 값을 제공한다. scalar 154개,
  Text 카드 66개, HParams 세션 22개를 서버 API로 확인했다. **서버를 시작·재시작·종료하지 않았다.**
- 저장한 고정 링크(`outputs/tensorboard-view.json`의 `zone_owncam_loop_20260926.url`)의 파라미터가
  `pinned_tags`·`run_filter`·`smoothing=0`과 일치하고, 필터가 정확히 이 22 run만 고르며 고정 카드
  7개 × 22 run 모두 값이 있고 링크 HTTP 200임을 확인했다.
- 회귀: `.venv-sim-worker-mac/bin/python -m pytest -q tests/test_owncam_loop_views.py` 11 passed,
  `tests/test_tensorboard_export.py tests/test_tensorboard_launcher.py` 59 passed.

## 남은 한계

- 브라우저 화면 확인은 하지 않았다(EventAccumulator 재읽기와 서버 HTTP API 확인만).
- 현재 main 변환기는 실행 기록의 `condition`을 HParams 열로 내보내지 않아 그 열은 `unrecorded`로
  보인다. split·carry·코호트는 `case`, pose_source는 `policy` 열에서 본다. 게이트 수치(R1 거리·R3
  p90 등)도 스칼라 태그가 없어 Text `evaluation/referee_only`에 있다. 파생 뷰에는 `offline_scalars`
  선언을 함께 넣어 뒀으므로, 선언된 오프라인 수치를 내보내는 변환기 변경(PR #183)이 병합되면 같은
  파생 뷰를 새 스냅샷으로 다시 변환해 `offline/*` 스칼라와 `condition` 열을 추가할 수 있다.
- 이미지·영상은 변환하지 않았다(`--max-images 0`). 학생이 본 JPEG 프레임과 `eval_only` 기록은 원본
  폴더에만 있고 로컬 보관이다.
- 이 스냅샷은 시뮬레이션 기록이며 실물 MasterPi 검증이 아니다.

## 다시 만드는 방법

```sh
python3 scripts/build_owncam_loop_views.py \
  --records <PR178 worktree>/experiments/2026-09-25-zone-owncam-loop \
  --raw /Users/changmin/projects/ugrp/outputs/owncam-loop-20260925 \
  --output /Users/changmin/projects/ugrp/outputs/tb-owncam-loop-<새 ID>/derived
.venv-sim-worker-mac/bin/python scripts/export_tensorboard.py \
  --source <파생 뷰 1>  ... --source <파생 뷰 22> \
  --output outputs/tensorboard/<새 스냅샷 ID> --max-images 0 --media-port 6009
python3 scripts/build_owncam_loop_views.py \
  --rename-snapshot outputs/tensorboard/<새 스냅샷 ID> --index <파생 뷰>/index.json
.venv-sim-worker-mac/bin/python scripts/verify_owncam_loop_snapshot.py \
  --snapshot outputs/tensorboard/<새 스냅샷 ID> --index <파생 뷰>/index.json \
  --raw /Users/changmin/projects/ugrp/outputs/owncam-loop-20260925 \
  --server http://127.0.0.1:6006 --out experiments/<기록 ID>/verification.json
```

대시보드: <http://127.0.0.1:6006>에서 run 필터 `^0926-zone-owncam-loop/`, smoothing 0, HParams 열은
위 표대로 다시 적용한다. 고정 링크 전체는 `outputs/tensorboard-view.json`의
`zone_owncam_loop_20260926.url`에 있다.
