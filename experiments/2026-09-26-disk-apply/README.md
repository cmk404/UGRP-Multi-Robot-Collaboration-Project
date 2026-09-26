# 2026-09-26 사용자 승인 디스크 감축 4항목 적용

작성: 2026-09-26, Kiro(`kiro/disk-apply-0926`, PR #231; 도구는 병합된 PR #212). Refs #226.

같은 날 오전 사고([2026-09-26-disk-incident](../2026-09-26-disk-incident/README.md))를 되풀이하지 않도록, 모든 이동은 옮기기 전후 **파일 개수·바이트·sha256을 모두 비교**했다. 같을 때만 worktree를 제거했다. raw 파일은 하나도 삭제하지 않았다. 지운 것은 git이 다시 만들 수 있는 추적 checkout 사본(은퇴·sparse)과 캐시(`__pycache__`, `.pytest_cache`)뿐이다.

## 결과

| 항목 | 대상 | 확보(GiB) | 검증 |
|---|---|---|---|
| 1. 병합된 다른 에이전트 worktree 은퇴 | 22개 (Claude 12, Kiro 10) | 23.01 (checkout 추정) | 무시 자료 1,538개 0.24 GiB 이동, 전부 sha256 일치, `git worktree remove` 22/22 exit 0 |
| 2. 열린 PR·동결 소스 worktree sparse 전환 | 17개 (열린 PR 11, 동결 detached 6) | 15.59 (18.09 → 2.50, 실측) | 17/17 HEAD 그대로, 추적 변경 없음, `git status --ignored` 전후 동일, 기본 체크아웃은 full 유지 |
| 3. 미병합 Codex worktree 보존 후 은퇴 | 9개 | 9.20 (checkout 추정) | HEAD 9/9를 `codex/archive-<이름>-0926`에 push하고 `ls-remote` SHA 일치 확인, bundle 9개. 커밋 안 된 변경 0건(patch 없음). 무시 자료 31,276개 2.60 GiB 이동, 전부 sha256 일치 |
| 4a. 같은 내용 파일 APFS clone 중복 제거 | `outputs/` 4 KiB 이상, 141,699그룹 653,736개 | 26.50 계획, `df` +26.77 | 모든 경로를 hashlib sha256·크기·mode·mtime으로 재검사: 문제 0 |
| 4b. 기록된 dev raw 솎기 | 후보 54폴더 | **0 (적용 안 함)** | 삭제 조건을 확인하지 못해 목록만 남겼다(아래) |

`df -k /` 여유 공간은 16:49 **48.53 GiB → 19:00 114.47 GiB**(+65.94)다. `du` 기준 worktree 합계는 64.31 GiB(60개) → 16.81 GiB(32개)다([전](footprint/disk-report-before.txt)·[후](footprint/disk-report-after.txt)). 기본 체크아웃 `outputs/`는 68.27 → 72.76 GiB로 늘었다. 옮겨 온 무시 자료 2.84 GiB와 다른 작업의 새 실행이 더해졌고, clone은 `du`에 전체 크기로 잡히기 때문이다.

- 항목별 `df` 변화: 항목 1 +23.06(18:24:42→18:26:34), 항목 2 +13.53(18:28:56→18:29:59), 항목 3 +9.30(18:35:02→18:36:09), 항목 4a +26.77(18:50:49→18:54:28). 합계는 +72.66이다.
- 항목 사이 구간의 변화: 16:49–18:24 −7.93(다른 작업의 쓰기), 항목 2 직후 18:30–18:35 +2.66, 18:36–18:50 −1.38(fclones 그룹 파일 0.1 GiB 포함), 나머지 −0.08. 합계는 −6.73이고, 16:49→19:00 전체는 +65.94다.
- 항목 2 직후의 +2.66은 이 5분 동안 이 작업이 지운 것이 없으므로(보관 push·bundle 2.4 MB만 씀), sparse로 지운 파일 공간이 APFS에서 늦게 반환된 것으로 본다. 확인하지는 않았다. 이 값을 더하면 항목 2는 +16.2로 파일 크기 합 15.59와 비슷하다.
- 순수 절감량은 항목별 `df` 변화 합 약 72.7 GiB(늦은 반환 포함 약 75)로 본다. 각 1–4분 구간에도 다른 작업의 쓰기가 섞여 있어 추정치다.
- `du` 기준 보고서(`scripts/disk_report.py`)는 APFS clone을 전체 크기로 센다. 그래서 4a 절감은 `du`에 나타나지 않는다. 실제 확보량은 `df`로 본다.

## 제외 목록 (손대지 않음)

| worktree | 해당 항목 | 이유 |
|---|---|---|
| `ugrp-worktrees/zone-hard-routes` (PR #173 병합) | 1 | 사용 중: Claude 셸 53386·53824의 cwd가 그 안의 `outputs/`다 |
| `ugrp-worktrees/zone-team-a2` (PR #169 병합) | 1, 2 | 사용 중: 실행 중인 kiro-teacher-fix(PR #207) 스크립트가 그 `outputs/zone-team-a2-20260925`를 절대 경로로 읽는다 |
| `ugrp-wt/kiro-vision-loc` (병합) | 1 | 실행 중인 Kiro 작업, 미추적 파일, 60분 내 수정 |
| `ugrp-wt/kiro-memory-literature`, `kiro-references-0926` (병합) | 1 | 실행 중인 Kiro 작업(사용자 지정), 60분 내 수정. 이미 sparse(각 0.13) |
| `ugrp-wt/kiro-study-core` (PR #194) | 2 | 60분 내 수정(git index, `AGENTS.md`, `maps/zones/*`). `sparsify`가 거부 |
| `ugrp-wt/kiro-map-v3` (#208), `kiro-owncam-memory` (#211), `kiro-teacher-fix` (#207) | 2 | 실행 중인 Kiro 작업, 미추적·추적 변경, 60분 내 수정 |
| `ugrp-wt/kiro-sim-speed` (#209) | 2 | `sim_profile` 프로세스 67205·67209의 cwd |
| `ugrp-wt/kiro-vision-loc-v3src` (#208 소스) | 2 | 60분 내 수정, 이미 sparse(0.01) |
| `ugrp-wt/kiro-disk-rules` | 2 | 이 작업의 worktree, 이미 sparse |

- `claude-todo-dr-sim2real`(PR #215)은 18:27 dry run에서 60분 조건을 통과했다. 18:29 실행 때는 거부됐는데, 원인은 이 작업이 1초 전에 실행한 `git status`가 index를 갱신한 것이었다(HEAD 16:32, 작업 트리 60분 내 변경 없음, 프로세스 없음). 사유를 `item2/override-claude-todo-dr-sim2real.txt`에 적고 `--idle-minutes 0`으로 sparse 전환했다. 이후 상태 확인은 `--no-optional-locks`로 했다.
- `kiro-own-perception`은 sparse 전환 뒤 0.40 GiB다. 무시된 `outputs/` 0.25 GiB가 worktree 안에 남아 있다(항목 2는 무시 파일을 건드리지 않는다).

## 항목별 방법

### 1·3. 은퇴 (`scripts/agent_worktree.py retire`)

- 무시된 `outputs/<이름>`은 기본 체크아웃의 **같은 상대 경로**(`/Users/changmin/projects/ugrp/outputs/<이름>`)로 옮겼다. 이미 있는 이름(`outputs/simulation-runs`)과 `outputs/` 밖의 무시 파일(`MUJOCO_LOG.TXT`, venv 링크)은 `outputs/retired-worktrees/<주인>-<이름>/`으로 옮겼다.
  - 예: `rgb-common-physical-recovery-20260923`(31,252개)과 `zone-team-a2-v2-20260925`는 `outputs/` 바로 아래로 옮겼다. `rgb-common-physical-recovery`의 `outputs/simulation-runs`(18개)는 `retired-worktrees/codex-rgb-common-physical-recovery/outputs/simulation-runs/`로 옮겼다.
- 파일별 목록(경로·종류·크기·sha256)은 각 `outputs/retired-worktrees/<라벨>/MANIFEST.tsv`, 영수증은 `RETIRED.json`과 `outputs/retired-worktrees/retirements.jsonl`이다.
- 미병합 Codex 9개(`archive_codex_worktree.py`)
  - 각 worktree의 `git status --porcelain --ignored`를 기록했다.
  - 커밋 안 된 변경을 확인했다(9개 모두 없음).
  - `git bundle`(merge base 이후)을 만들어 `bundle verify`했다.
  - HEAD를 `refs/heads/codex/archive-<이름>-0926`에 push하고 `ls-remote` SHA가 HEAD와 같은지 확인했다. worktree에는 아무것도 커밋하지 않았다.
  - `codex/**` push가 CI를 띄워 9개 실행을 바로 취소했다. 9개 모두 `cancelled`로 확인했다.
  - 은퇴는 `--archive-ref`로 원격 SHA를 다시 확인한 뒤에만 진행했다.
- 참조 확인(`check_moves.py`)
  - 로컬·원격 ref 242개의 텍스트 파일에서 옛 worktree 경로 참조 63개를 찾았다.
  - 그중 옮긴 자료를 가리키는 43개는 새 위치에서 43/43 존재한다.
  - 나머지 20개는 worktree 루트·추적 파일만 가리킨다. 경로 문자열은 기록에 그대로 남아 있다(기록은 고치지 않았다).

### 2. sparse (`scripts/agent_worktree.py sparsify`)

- 기존 프로필 `agent-media-v1`: `experiments/`의 zip·gz·영상·이미지·pdf·html·npz를 checkout에서 빼고, 실행에 쓰는 `models.zip` 2개는 남긴다.
- 대상 worktree의 `harness/`·`sim/`·`scripts/`·`tests/`·`experiments/*.py`에서 빼는 종류의 `experiments/` 파일을 읽는 코드는 없다(grep, 17개 모두 확인).
- 파일은 `.git`에 그대로 있다. `git sparse-checkout disable`로 언제든 되돌릴 수 있다.

### 4a. 중복 제거 (`dedupe_outputs.py` + fclones)

- `fclones group outputs --min 4KiB --hash-fn sha256`로 그룹 145,399개, 중복 28.71 GiB를 찾았다.
- `dedupe_outputs.py plan`으로 다음을 뺐다.
  - 60분 내 수정된 파일 2,103개
  - 최근 활동이 있는 폴더(실행 중 작업·이 작업의 영수증)의 파일 4,275개
  - 하드 링크 파일 3,978개(clone이 링크를 끊으므로)
  - 열린 파일
- 계획 전 manifest(`item4/dedupe-manifest-before.tsv`: 그룹·경로·바이트·sha256·mode·mtime_ns)를 먼저 쓰고 `fclones dedupe --modified-before <계획 시각−60분>`을 실행했다.
- 동작: fclones는 사본을 임시 이름으로 옮기고 `cp -c`(clonefile)로 새로 만든 뒤 임시본을 지운다. 경로마다 같은 바이트를 읽고, mode·mtime·xattr도 유지된다(`/tmp` 시험으로 확인).
- 검증: `dedupe_outputs.py verify`가 653,736개 경로를 hashlib로 다시 해시했다. 크기·sha256·mode·mtime 문제는 0이었다(`item4/dedupe-verify.json`).
- 주의: clone끼리는 블록을 공유한다. 한 사본을 지워도 공간이 돌아오지 않고, 마지막 사본까지 지워야 돌아온다.

### 4b. dev raw 솎기: 적용하지 않음

사용자 조건 (c)는 "재생성 가능하거나 중복 파생본"이다. 이를 만족한다고 확인한 폴더가 없다.

- 큰 dev 폴더의 JPEG은 로봇 자기 카메라·TOP 프레임이다(예: `plan-guidance-20260925/objective_preview-*/rgb/`, `m1-owncam-20260926/dev-*/*/frames/`). 규칙 기반 제어기의 입력 원본이다.
- `execution.mp4`는 이 프레임을 인코딩한 것이 아니다. `cctv_warehouse` 카메라를 4 fps로 따로 렌더링한 영상이다(`harness/rgb_skill_execution.py:1261` → `scripts/probe_dual_grasp_sync.py:212`). 따라서 프레임은 "영상에 이미 담긴 파생본"이 아니다.
- 다시 만들려면 시뮬레이션을 재실행해야 하는데, 과거 실행의 바이트 단위 재현성은 확인하지 않았다.
- 같은 내용의 중복 사본은 4a에서 clone으로 이미 합쳤다.

사용자 결정용 후보(기록 참조가 있는 dev·진단 폴더 54개, 이미지 5.93 GiB, 5 Hz→1 Hz 솎기 추정 4.74 GiB, 논리 크기)는 `outputs/disk-apply-20260926/item4/thinning-candidates.json`에 있다. 4a 뒤라 실제 확보량은 이보다 작다. 큰 순서는 다음과 같다.

| 폴더 | JPEG/PNG 수 | 이미지 GiB | 1 Hz 솎기 추정 |
|---|---|---|---|
| `zone-rgb-outcome-v2-20260925/dev-final` | 19,920 | 1.10 | 0.88 |
| `zone-rgb-outcome-v2-20260925/dev-b` | 10,037 | 0.56 | 0.45 |
| `zone-rgb-outcome-v2-20260925/dev-a2` | 6,191 | 0.37 | 0.30 |
| `zone-rgb-outcome-v2-20260925/dev-a` | 5,212 | 0.31 | 0.24 |
| `zone-rgb-outcome-20260925/dev` | 4,141 | 0.22 | 0.18 |
| `dispatch-cargo-recovery-20260924/original-candidate-diagnostic-1` | 3,231 | 0.19 | 0.15 |
| `plan-guidance-20260925/objective_preview-1`…`-5` | 2,884 × 5 | 0.16 × 5 | 0.13 × 5 |

- 기록 참조가 없는 dev 폴더 18개(이미지 4.69 GiB, 대부분 `jev-skills-dev-*`)는 열린 PR의 기록인지 먼저 확인해야 한다. 이번에는 다루지 않았다.

## 파일

- 커밋한 기록: `applied.json`(항목별 행·합계·제외 목록·원본 영수증 sha256)
- 스크립트
  - `classify_worktrees.py`: 사전 전수 분류(읽기 전용)
  - `archive_codex_worktree.py`: 항목 3 보존
  - `check_moves.py`: 참조 확인
  - `dedupe_outputs.py`: 항목 4a 계획·검증·요약
  - `summarize.py`: `applied.json` 생성
- raw(로컬, 커밋 안 함): `/Users/changmin/projects/ugrp/outputs/disk-apply-20260926/`
  - `census-before.json`, `disk-report-before.*`, `disk-report-after.*`, `before-df.txt`, `after-df.txt`, `moves-check.json`
  - `item1/`·`item3/`: dry run과 실행 영수증, 항목 전후 `df`
  - `item2/`: 영수증, 전후 status
  - `item4/`: fclones 그룹 105 MB, 계획, 사전 manifest, 로그, 검증, 요약, 솎기 후보
- 보존본(로컬): `/Users/changmin/projects/ugrp/outputs/archive/codex-worktrees/<이름>-0926.{bundle,json}`. 원격 사본은 origin의 `codex/archive-*-0926` 브랜치 9개다.
- 로컬 raw만으로는 백업이 아니다. 이번 작업은 raw를 원격에 올리지 않았다.

## 남은 문제

- 은퇴한 worktree를 가리키는 기록의 절대 경로 문자열은 고치지 않았다. 새 위치는 `moves-check.json`에 있다.
- 병합된 PR #212의 원격 브랜치 `kiro/disk-rules`: 병합 뒤 삭제된 브랜치에 이 작업이 실수로 커밋 `429f7448`을 push해 다시 생겼다. 같은 내용은 `47c4b0dc`(이 PR)에 있다. 원격 브랜치 삭제는 사용자 확인 뒤에 한다.
- 제외한 worktree 12개는 작업이 끝난 뒤 주인이 같은 도구로 정리할 수 있다.
- 4b 솎기, `retired-worktrees` 등 raw 외부 보관, 예산 수치는 사용자 결정으로 남는다.

## 참고 자료

- 논문: 없음(운영 작업).
- OSS
  - fclones 0.35.0, https://github.com/pkolaczk/fclones , MIT. Homebrew bottle `fclones--0.35.0.arm64_golden_gate`로 설치했다. `group`(sha256)과 `dedupe`(APFS clone)를 그대로 썼고, 필터·검증만 직접 작성했다.
  - 검토 후 쓰지 않은 것:
    - jdupes 1.31.2(https://codeberg.org/jbruchon/jdupes , MIT): macOS clone 지원이 있으나 JSON 그룹 출력과 `--modified-before` 같은 안전 필터가 fclones보다 약하다.
    - 직접 `cp -c` 루프: fclones가 이미 원자적 교체와 메타데이터 보존을 한다.
- 내부
  - `scripts/agent_worktree.py`(retire·sparsify, 이 PR에서 sha256·같은 경로·`--archive-ref` 추가)
  - `scripts/worktree_guard.py`(신규)
  - `scripts/tree_manifest.py`(신규)
  - `scripts/disk_report.py`
  - `experiments/2026-09-26-disk-incident/check_relocations.py`(`check_moves.py`의 원형)
  - PR #212
- 문서
  - `man cp`(`-c`: clonefile(2) 사용), `man 2 clonefile`
  - `git help sparse-checkout`, `git help bundle`, `git help worktree`
