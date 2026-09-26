# 2026-09-26 병합 worktree 제거 중 무시된 raw 손실 — 사고 기록

작성: 2026-09-26, Kiro(`kiro/disk-rules`, PR #212). 조사와 기록만 추가했다. 기존 실험의 결과·판정은 바꾸지 않았다.

## 요약

- **일시·작업:** 2026-09-26 13:50경 관리 세션이 병합된 UGRP worktree 28개를 `git worktree remove`로 제거했다. 제거 목록은 관리 세션의 `wt-remove-ugrp.tsv`다.
- **원인:** 삭제 전에 `git status --porcelain`만 확인했다. 이 명령은 `.gitignore`에 걸린 `outputs/`·`MUJOCO_LOG.TXT`를 보여 주지 않는다. `git worktree remove`는 무시 파일을 경고 없이 함께 지운다. 그래서 worktree 안에만 있던 raw가 지워졌다.
- **백업:** 없다. Time Machine 대상이 설정돼 있지 않고(`tmutil destinationinfo`), 데이터 볼륨에 APFS 로컬 스냅샷도 없다(`tmutil listlocalsnapshots`, `diskutil apfs listSnapshots`, 14:3x 확인).
- **영향:** main의 기록 7건이 지워진 경로를 가리킨다. 상태는 다음과 같다.
  - 1건(r3 재파지 분석의 stage 모델)은 추적 중인 ZIP에서 완전히 복원할 수 있다.
  - 1건(한국어 대화 파일럿)은 응답 본문과 요청 이미지가 모두 남았고 wire 파일만 없다.
  - 5건은 원본 일부 또는 전부를 복구할 수 없다. 요약·해시는 기록에 남아 있다.
- **재발 방지:** `scripts/agent_worktree.py retire`를 만들었다. 무시 항목을 옮기고 개수·바이트를 검증한 뒤에만 제거한다. 규칙은 AGENTS.md와 [디스크 관리](../../docs/disk_management.md)에 적었다.

## 제거된 worktree 28개

| worktree | 브랜치 | 병합 PR | 무시 자료(09-25 인벤토리) | 제거 직전 du |
|---|---|---|---|---|
| ugrp-worktrees/agent-coordination | claude/agent-coordination | #136 | 없음 | 1.0G |
| ugrp-worktrees/beam-regrasp | claude/beam-regrasp-binding | #157 | 2.2M (22) | 1.0G |
| ugrp-worktrees/beam-regrasp-coarse | claude/beam-regrasp-coarse | #159 | 미조사 | 1.0G |
| ugrp-worktrees/current-status | claude/current-status-0925 | #156 | 없음 | 1.0G |
| ugrp-worktrees/dynamic-coordination | claude/dynamic-team-recovery | #147 | 12M (22) | 1.0G |
| ugrp-worktrees/fine-gain-schedule | claude/fine-gain-schedule | #133 | 5.0M (19) | 1.0G |
| ugrp-worktrees/plan-objective-preview | claude/plan-objective-preview | #144 | 22M (44) | 1.1G |
| ugrp-worktrees/post-run-replay | claude/post-run-replay | #140 | 238M (3266) | 1.3G |
| ugrp-worktrees/r3-pose-record | claude/beam-regrasp-r3-pose-record | #162 | 미조사 | 1.0G |
| ugrp-worktrees/realtime-stop-gap | claude/realtime-stop-gap | #138 | 없음 | 1.0G |
| ugrp-worktrees/records-0925 | claude/records-0925 | #171 | 미조사 | 1.0G |
| ugrp-worktrees/team-recovery-fix | claude/team-recovery-longer-backoff | #153 | 14M (22) | 1.0G |
| ugrp-worktrees/zc3-prereg | claude/zc3-prereg-draft | #160 | 없음 | 1.0G |
| ugrp-worktrees/zone-cargo | claude/zone-cargo-noslip | #167 | 미조사 | 1.2G |
| ugrp-worktrees/zone-cargo-perception | claude/zone-cargo-perception | #168 | 미조사 | 1.1G |
| ugrp-worktrees/zone-cargo-perception-v2 | claude/zone-cargo-perception-v2 | #174 | 미조사 | 1.1G |
| ugrp-worktrees/zone-comm-audit | claude/zone-comm-audit | #161 | 미조사 | 1.0G |
| ugrp-worktrees/zone-communication | claude/zone-communication | #158 | 없음 | 1.0G |
| ugrp-worktrees/zone-dialogue-ko | claude/zone-dialogue-ko-pilot | #172 | 미조사 | 1.1G |
| ugrp-worktrees/zone-dispatch | claude/zone-dispatch-z3-record | #154 | 없음 | 1.0G |
| ugrp-worktrees/zone-rgb-color | claude/zone-rgb-color | #163 | 미조사 | 1.2G |
| ugrp-worktrees/zone-rgb-outcome | claude/zone-rgb-outcome | #170 | 미조사 | 1.0G |
| ugrp-worktrees/zone-team-jobs | claude/zone-team-jobs | #165 | 미조사 | 1.0G |
| ugrp-worktrees/zone-wide-arena | claude/zone-wide-arena | #155 | 없음 | 1.0G |
| ugrp-wt-goto | claude/map-goto-navigation | #148 | 13M (21) | 1.0G |
| ugrp-wt/records-0925b | claude/records-0925b | #175 | 미조사 | 1.0G |
| ugrp-wt/zone-owncam-loc | claude/zone-owncam-loc | #177 | 미조사 | 1.0G |
| ugrp-wt/zone-owncam-skill | claude/zone-owncam-skill | #176 | 미조사 | 1.0G |

- 무시 자료 열은 2026-09-25 16:29 인벤토리(`worktree-inventory.md`)의 `outputs/` 크기(파일 수)다. 그 뒤 만들어진 worktree는 "미조사"다.
- 제거 직전 du는 관리 세션의 `wt-status.tsv`(13:50)에 있는 `du -h` 반올림값이다. 전체 checkout이 약 1.04 GiB이므로, 1.1–1.3G 항목에 0.1–0.3 GiB씩 무시 자료가 있었다고 추정한다. 목록이 남지 않아 무엇이 지워졌는지 전부 알 수는 없다.

## 참조 스캔 방법

1. `scan_references.py`: 로컬·원격 브랜치 217개 ref의 텍스트 blob 4,625개에서, 제거된 worktree의 절대 경로를 가리키는 참조 34개를 찾았다(`references.json`).
   - 무시 자료(`outputs/…`)를 가리키는 참조는 5개다. 모두 origin/main에 있다.
   - 나머지는 추적 파일 경로(Git에 남아 있음)나 실행 당시 cwd 기록이다.
   - 스캔 당시 main 밖에만 있던 참조는 `kiro/records-owncam-review`(PR #196) 검토 문서의 추적 파일 인용뿐이었다. 그 파일들은 Git에 남아 있다.
2. `scan_missing_outputs.py`: origin/main(`b7fedcee`) 기록에 나오는 상대 경로 `outputs/<a>/<b>` 참조 3,655개를 찾아봤다. 검색 위치는 기본 체크아웃, 등록된 모든 worktree, `outputs/retired-worktrees/*/outputs`(합 50곳)다(`missing_outputs_summary.json`).
   - 찾지 못한 872개에는 자리표시자·CI 경로·원격 경로, 이전에 승인된 정리로 지운 자료가 섞여 있다.
   - 09-24~26 기록의 270개를 직접 확인했다. 이번 제거와 연결되는 것은 아래 표의 4개 worktree뿐이었다(`zone-cargo`, `zone-cargo-perception`, `zone-cargo-perception-v2`, `zone-dialogue-ko`).
3. TensorBoard manifest(`outputs/tensorboard*`) 중 제거된 worktree를 원본으로 가리키는 것은 `0925-plan-guidance/R-user-boxfirst`와 `0925-dynamic-coordination/A-planfirst` 두 개다.

## 영향받은 기록과 남은 증거

각 기록 폴더에 `raw_status.json`을 추가했다. 한국어 파일럿 README는 열린 PR #188이 수정 중이라 README 메모를 넣지 않았다.

| 기록 | 지워진 경로 | 남은 것 | 상태 |
|---|---|---|---|
| [2026-09-25-dynamic-coordination](../2026-09-25-dynamic-coordination/raw_status.json) | `post-run-replay/…/20260925-000852-dispatch-ee5eac42/artifacts` (baseline-plan-first) | 요약값, result.json·team.json 해시, TensorBoard 스냅샷 2개(원본 5개 파일의 해시 포함) | 복구 불가. 영상 링크도 끊김 |
| [2026-09-25-plan-guidance](../2026-09-25-plan-guidance/raw_status.json) | 같은 실행(참고 실행) | 위와 같음. 코호트 15회 원본은 기본 체크아웃에 온전함 | 복구 불가 |
| [2026-09-25-r3-regrasp-pose](../2026-09-25-r3-regrasp-pose/raw_status.json) | `team-recovery-fix/outputs/dispatch-models/e78a5a5777f5bc48/models/varied` | 추적 중인 `models.zip`과 해시 7/7 일치. 추출본 4벌 | **복원 가능** |
| [2026-09-25-zone-cargo-catalogue](../2026-09-25-zone-cargo-catalogue/raw_status.json) | `zone-cargo/outputs/zone-cargo/{final-d81514a,slip-8886cf5,dev…}` | 영상·그림 20개가 `media/`에 바이트 그대로 있음. 나머지 231개는 해시만 | 일부 손실 |
| [2026-09-25-zone-cargo-perception](../2026-09-25-zone-cargo-perception/raw_status.json) | `outputs/zone-cargo-perception-20260925/` | manifest·summary·records 해시, 검출 그림 1장 | 복구 불가(재렌더 가능성, 미검증) |
| [2026-09-25-zone-cargo-perception-v2](../2026-09-25-zone-cargo-perception-v2/raw_status.json) | `outputs/zone-cargo-perception-v2-20260925/` | 해시, 재현 명령, TensorBoard 파생 뷰 | 복구 불가(재렌더 가능성, 미검증) |
| [2026-09-25-zone-dialogue-ko-pilot](../2026-09-25-zone-dialogue-ko-pilot/raw_status.json) | `outputs/zone-dialogue-ko-pilot-20260925/` wire·response 248개 | 응답 본문 124/124(`calls.jsonl`, 해시 일치). 요청 이미지 620/620이 원 요청에 있음 | wire 46.7 MB만 손실. 재구성 가능성 있음(미검증) |

- 한국어 파일럿의 원 요청은 기본 체크아웃 `outputs/zone-communication-20260925/`에 있다. 124개 파일이 모두 있고, 기록된 해시와 같다.
- 미끄러짐·최종 영상 18개와 카탈로그 그림 2개는 `media/`의 커밋 사본과 sha256이 같다.

## 모델 `e78a5a5777f5bc48`는 남아 있다

- 폴더 이름은 `experiments/dispatch-skill-integration-20260917/models.zip`의 sha256 앞 16자다. `scripts/sim_dispatch.py`의 `bundled_models()`가 이 ZIP을 `outputs/dispatch-models/<sha16>/`에 풀어 만든다. 따라서 지워진 폴더는 원본이 아니라 압축 해제 사본이었다.
- r3 기록의 stage 모델 해시 7개는 ZIP 구성원과 모두 같다.
- 같은 추출본 4벌(15/15 파일이 ZIP과 같음)이 Codex worktree에 있었다. 이번 작업에서 `outputs/retired-worktrees/codex-{simulation-performance,pair-approach-progress-sync,simulation-realtime,faster-dispatch}/outputs/dispatch-models/e78a5a5777f5bc48/`로 옮겼다.
- `configs/model_artifacts.json`에는 이 모델이 없다. GitHub Release 2개(`models-20260923-v1`, `models-20260924-action-act-v1`)에는 ACT 모델만 있다. 이 모델이 살아 있는 근거는 Git에 추적되는 ZIP이다.

## 원인

- 실험 드라이버가 raw를 worktree 자신의 `outputs/`에 썼다. 그 worktree의 수명이 raw의 수명이 됐다.
- 삭제 판단에 `git status --porcelain`만 썼다. 무시 파일은 보이지 않는다. 09-25 인벤토리는 이 함정을 경고했지만(`outputs/ caveat`), 제거 단계에는 반영되지 않았다.
- `git worktree remove`에는 무시 파일을 보호하는 단계가 없다(`--force` 없이도 지운다).

## 재발 방지 (PR #212)

- `scripts/agent_worktree.py retire <경로> --execute`는 아래 순서를 강제한다.
  1. 병합(HEAD가 origin/main에 포함 또는 PR이 같은 head로 MERGED)을 확인한다.
  2. 추적 변경·미추적 파일이 없는지 확인한다.
  3. `lsof`로 사용 중인 프로세스가 없는지, gh로 열린 PR이 없는지 확인한다.
  4. 무시 항목 중 캐시(`__pycache__` 등)가 아닌 것을 모두 기본 체크아웃 `outputs/retired-worktrees/<label>/`로 옮긴다.
  5. 개수·바이트를 대조한다.
  6. 그다음에만 `git worktree remove`(`--force` 없음)를 실행한다.
- AGENTS.md 규칙: 직접 `git worktree remove`를 쓰지 않는다. raw는 기본 체크아웃 `outputs/`에 쓴다.

## 이번 작업의 안전 정리: 병합된 Codex-app worktree 14개

- 대상: `act-finalization-recovery`, `action-act-refinement`, `dispatch-action-act`, `faster-dispatch`, `github-model-artifacts`, `merge-approved-20260923`(detached, 옮길 자료 없음), `pair-approach-progress-sync`, `pair-coarse-concurrency`, `rolling-cargo-recovery`, `rolling-view-recovery`, `rolling-visual-servo`, `settled-view-recovery`, `simulation-performance`, `simulation-realtime`.
- 모두 HEAD가 origin/main에 포함됐고, 미추적·추적 변경이 없었다. 사용 중인 프로세스와 열린 PR도 없었다. dry run 뒤 실행했다.
- 옮긴 자료: 13개 worktree에서 6,200개 항목, 876,932,998 바이트(0.82 GiB)를 `outputs/retired-worktrees/codex-<이름>/`로 옮겼다.
  - 항목마다 개수·바이트가 일치했다(`verified: true`).
  - `git worktree remove` 14회가 모두 exit 0이었다.
  - 영수증: 각 폴더의 `RETIRED.json`, 공용 `outputs/retired-worktrees/retirements.jsonl`, 사본 `footprint/retirements-20260926.jsonl`.
- 효과: checkout 삭제로 추정 14.38 GiB를 확보했다. 파일시스템 여유는 32.47 → 46.88 GiB였다(다른 작업의 쓰기와 섞인 값).
- 참조 검증(`check_relocations.py`, `relocations.json`): 모든 ref에서 이 14개 worktree를 가리키는 참조는 74개다. 그중 무시 자료를 가리키는 64개는 이동 전 옛 경로 64/64, 이동 뒤 새 경로 64/64가 존재했다.
- 부작용: TensorBoard 스냅샷 `0923-pair-approach-v4-physical`(3 run)·`0923-pair-approach-v3-review`(2 run)의 영상 링크가 옛 경로를 가리켜 재생되지 않는다. 스칼라·텍스트는 그대로다. 새 경로로 새 스냅샷을 만들지는 사용자가 정한다.
- worktree 폴더의 부모 `~/.codex/worktrees/<이름>/`(`.codex-worktree-name`만 있음)와 브랜치는 그대로 두었다.

## 검증 범위와 한계

- 삭제 전 파일 목록이 없어 "미조사" worktree에서 지워진 개발·진단 자료의 내용과 크기는 알 수 없다. 이 기록은 Git·TensorBoard에서 참조되는 자료만 판정한다.
- 재렌더·wire 재구성은 시도하지 않았다. 가능성만 적었다.
- 스캔은 텍스트 blob 안의 경로 문자열 기준이다. 코드가 실행 중에 경로를 조합하는 경우는 잡지 못한다.

## 파일

- `scan_references.py`, `references.json`: 제거된 worktree의 절대 경로 참조
- `scan_missing_outputs.py`, `missing_outputs_summary.json`: 상대 `outputs/` 참조 중 어디에도 없는 것
- `check_relocations.py`, `relocations.json`: Codex worktree 14개 이동 전후 참조 해석
- `footprint/`: 측정 보고서(before/after), 은퇴 영수증, sparse worktree 실측(`sparse-probe.json`)
