# 2026-09-25 미스냅샷 결과의 TensorBoard 스냅샷 (0925-zone-supplement)

2026-09-25에 main에 병합됐지만 TensorBoard 스냅샷이 없던 기록 6개를 새 스냅샷
`outputs/tensorboard/0925-zone-supplement`(116 run)으로 변환했다. 새 실험·시뮬레이션·모델 호출은
없고 기존 스냅샷·원본은 수정하지 않았다. 전체 수치·검증 기록은
[verification.json](verification.json)에 있다.

- 변환 소스: `3bac186` (`scripts/tensorboard_tools/export.py`, `scripts/build_zone_supplement_views.py`)
- 파생 뷰: `outputs/tb-supplement-20260925/derived` (로컬), 원본은 읽기 전용
- 뷰 설정: `outputs/tensorboard-view.json`의 `zone_supplement_20260925` 키 (다른 키는 바이트 그대로 유지)

## 포함한 결과와 코호트

| run 접두사 | 원본 | run | 코호트 경계 |
|---|---|---|---|
| `hr-` | `2026-09-25-zone-hard-routes/results.json` (PR #173) | 60 | 경로 42 + zone_wide 기준선 18. fixture 응답(LLM 0회)·**교사 실행기**·weld off. `reported_success`는 심판 구역 개수이며 RGB·자기 카메라 성공이 아니다 |
| `loc-` | `2026-09-25-zone-owncam-loc/results.json` (PR #177) | 14 | 사전 등록 게이트 8(dev/test × G1·G2·G3·I1) + **사후** look-only 6(`ph-`). 성공 = 게이트 통과이며 운반 성공이 아니다 |
| `ko-` | `2026-09-25-zone-dialogue-ko-pilot/results.json` (PR #172) | 5 | V0–V3 오프라인 재질의 + 수동 라벨 일치 1. SIM 미실행·결정 미집행. V3는 고정 스키마 대조군이라 자유 한국어 지표가 없다. 성공 필드 없음 |
| `slip-` | `2026-09-25-zone-cargo-catalogue/results-slip.json` (PR #167) | 21 | 60초 정지 파지 12(cargo_noslip_v1 vs local_contact_fine × 6품목) + 장거리 6 + 로봇 부족 3. GT 교사·weld off, 성공 = 해당 절의 수락 기준 |
| `perc-cargo-` | `2026-09-25-zone-cargo-perception-v2/results.json` (PR #174) | 8 | split 2 × 프로필 4(v1/v2/top_zone_v2/zone_perception_v1), 렌더 프레임 오프라인 평가 |
| `perc-colour-` | `2026-09-25-zone-rgb-color/results.json` (PR #163) | 8 | split 2 × 프로필 2 × own/top_merged(primary_zone_wide). 로봇 미실행 |

`offline/*` 스칼라는 파생 뷰가 **명시적으로 선언한** 원본 기록의 오프라인 측정값이다. 변환기는 원본
경로·SHA-256을 다시 확인하고 태그 형식·유한 수치·범위 설명이 없으면 변환을 거부한다. 로봇 임무
성공이나 실행 시간이 아니다.

## 검증 범위

- 116/116 run이 EventAccumulator로 다시 열렸고, **813개 스칼라를 원본 JSON에서 독립적으로 재계산해
  대조**했다. 불일치 0, 경고 0, 미완성 manifest 0, 원본 해시 불일치 0.
- 실행 중인 공용 서버(PID 9291/9293, 2026-09-22 시작, logdir `outputs/tensorboard`)가 새 116 run을
  모두 나열하고(총 468) 값을 제공한다. **서버를 재시작하지 않았다.**
- HParams에 새 세션 116개가 등록되고 각 세션에 `condition`이 들어 있다. 다만 공용 experiment의 열
  목록에는 `condition`과 `offline/*`가 없어 이 둘은 Time Series 고정 카드로 본다.
- 6개 뷰(전체/hr/loc/ko/slip/perc)의 모든 고정 태그가 서버에서 값을 반환했다.
- 한계: 브라우저 화면 확인은 하지 않았다(서버 HTTP API·EventAccumulator 재확인만). 서로 다른 코호트의
  성공률을 합산하지 않는다.

## 건너뛴 기록과 이유

- 이미 변환됨: `0925-zone-owncam-skill`, `0925-zone-owncam-skill-v2`, zone-dispatch, zone-communication
  ZC2, zone-wide, dynamic-coordination/team-recovery, beam-regrasp, map-goto, plan-guidance,
  post-run-replay, settled-view v50, view-recovery v43 (기존 collection의 source 경로로 확인).
- zone-cargo-catalogue `results.json`(PR #164)과 화물 인식 v1 기록(PR #168): 여기 포함한 무슬립 수락
  표와 v2 평가가 같은 행을 갱신한다. v1 프로필은 `perc-cargo-*`의 프로필 열로 남아 있다.
- zone-team-jobs(#165), zone-comm-boundary-audit(#161), zc3-prereg-draft(#160): 단위 테스트·입력 경계
  감사·사전 등록 초안이라 변환할 실행/평가 행이 없다.
- r3-regrasp-pose(#162): 이미 변환된 v61 실행의 사후 재분석이며 새 실행 행이 없다.
- action-act-followup, host-scheduling, settled-view v46: 2026-09-25 기록에 완료된 물리 A/B 행이 없다.
