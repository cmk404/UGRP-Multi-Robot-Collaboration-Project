# 2026-09-26 태그 없는 비전 위치 추정 (Kiro)

**질문(사용자, 2026-09-26).** "벽과 문기둥에 표식을 남기지 않고, vision을 이용해서 인식하게 하면 안되나?" 최종 연구 환경은 AprilTag가 전혀 없는 환경 v3(`walls_v3` 0.40 m 벽 + 기본 지도)이다. 손목 어안 RGB의 비전 인식만으로 문 근처(짐 운반 중) 위치를 태그 수준에 가깝게 추정할 수 있는가?

<!-- ANSWER -->

- 작성: Kiro(`kiro/zone-vision-loc`, `kiro/`는 Kiro 작업 표시). PR #210(마커 없는 탐침, main 병합됨)의 후속. 설계·결과 문서: [docs/design/2026-09-26-vision-localization-tagfree.md](../../docs/design/2026-09-26-vision-localization-tagfree.md).
- 성격: 태그 없는 환경의 **교사 주행 렌더를 오프라인으로 재생**해 위치 추정만 평가했다. 추정으로 로봇을 몬 폐루프 결과가 아니다. 새 LLM/API 호출 없음.

## 입력 경계

| 입력 | 학생(위치 추정) | 비고 |
|---|---|---|
| 자기 손목 프레임 `frames/*.jpg` | 사용 | 원시 어안 JPEG |
| 자기 발행 명령 `inputs/commands.jsonl` | 사용 | 운동 모델, 서보 상태, 짐 상태, 안정 시간 |
| 자기 조작 단계 전환 `inputs/motion_profile.jsonl` | 사용 | M1 PF와 같은 운동 프로필 전환 |
| 정적 지도 `maps/zone_wide_door_walls_v3_notags.json` | 사용 | 벽·문·경계, 태그 0개 |
| 고정 보정 | 사용 | K·D, M1 운동 모델(해시 확인), `calibration_train.json`(train GT로 오프라인 적합) |
| 분할 모델 가중치 | 사용 | train 렌더로 학습, 해시 고정 |
| `teacher/`(교사 자세 보고·사건) | 사용 안 함 | 테스트로 차단 |
| `eval_only/`(GT 궤적, 분할 라벨, 카메라 자세) | 사용 안 함 | 학습 표적·보정 적합(train)·오라클 진단·채점만 |

## 재현

```sh
# 0) v3 소스(렌더 전용, 희소 worktree): git worktree add --detach ../kiro-vision-loc-v3src 7cedb049
# 1) 교사 렌더(최대 2 sim, 스레드 1, df >= 30 GiB, 기계 전체 sim < 6 대기)
experiments/2026-09-26-vision-loc/run_render.sh kiro-vl-renderA <out>/render vl-dev-s909,vl-train-s901,...
# 2) 보정(train), 학습(train, dev 검증), 관측(학생 입력만), 필터, 채점
PY=.venv-sim-worker-mac/bin/python; TPY=/Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py calibrate --episodes vl-train-s901 ... --settled-s 0.3 --output calibration_train.json
$TPY experiments/2026-09-26-vision-loc/vision_loc_cli.py train --output <out>/model/seg-v2 --epochs 4 --every 3
$TPY experiments/2026-09-26-vision-loc/vision_loc_cli.py segment --episodes <eps> --checkpoint <ckpt> --output <out>/obs-v2
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py oracle --episodes <eps> --output <out>/oracle-obs     # 진단 전용
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py localize --episodes <eps> --calibration calibration_train.json \
       --config selected_config.json --filters vision,boundary,deadreck,oracle --obs <out>/obs-v2 --oracle-obs <out>/oracle-obs --output <est>
$PY  experiments/2026-09-26-vision-loc/vision_loc_cli.py score --episodes <eps> --estimates <est> --obs ... --oracle-obs ... --output metrics.json
```

test 에피소드는 `prereg.json`이 있고 동결 파일·모델·설정·보정 해시가 모두 같을 때만 실행된다(`require_frozen`).

<!-- BODY -->
