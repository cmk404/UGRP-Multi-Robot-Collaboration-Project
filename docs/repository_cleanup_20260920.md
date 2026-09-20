# UGRP 정리 기록 — 2026-09-20

## 범위와 기준

사용자 요청에 따라 GitHub 브랜치·로컬 Git 등록·문서 탐색 경로를 정리했다. 기준 main은 `7855e1a9991daa31ab42766b47be4dfb5c42eb4b`다. 실제 연구 코드·물리 환경·입력 경계·실험 원본·모델·서비스는 변경하지 않았다. 다른 작업이 진행 중이므로 별도 worktree에서 문서를 수정했다.

## 완료한 로컬·GitHub 정리

- 사라진 `/private/tmp/ugrp-*` 경로의 worktree 등록 15개를 `git worktree prune`으로 제거했다. 실제 존재하는 작업 폴더는 삭제하지 않았다. 정리 전 등록 48개에서 기존 33개가 남았고, 이번 정리용 worktree를 추가해 34개다.
- 아래 브랜치 17개를 GitHub와 로컬에서 삭제하고 양쪽에서 부재를 확인했다. 삭제 직전 PR MERGED, 원격 tip의 main 포함, 보호되지 않은 브랜치, 열린 PR 부재, 등록 worktree 부재, 로컬/원격 SHA 일치를 확인했다. 강제 push는 사용하지 않았다.
- 원격 브랜치 57개에서 40개가 남았다(이 정리 PR의 브랜치 push 전). 그중 main 외 39개는 worktree 사용·미통합 커밋·PR 상태 등의 이유로 유지했다. 실제 폴더의 clean 상태만으로 무시 파일·raw 자료를 지워도 된다고 판단하지 않았다.
- 기본 체크아웃은 main과 같고 소스 파일 변경이 없다. 이 문서 정리의 main 반영은 별도 PR 승인 뒤 수행한다.

| 삭제한 브랜치 | 복구용 tip SHA | 병합 PR |
|---|---|---|
| `codex/approach-side-grasp` | `72b2843162603d842922ba37715a940aae1418a0` | #24 |
| `codex/current-gemini-proxy-guide` | `d6f1ef75945b604680871ec77bbd1ffade070109` | #19 |
| `codex/default-unassisted-grasp` | `837fc374f79d4395c603d07114fc1eafca7e2f34` | #16 |
| `codex/dual-grasp-sync-probe` | `3b0688c57653edd620684502adbb33d532fd731d` | #9 |
| `codex/github-management-baseline` | `e98cb877310d46765838a233e939d05fedf7fb89` | #4 |
| `codex/grasp-recovery-curriculum` | `c23602e2e17727165aaa5fb65773e8b40c365f88` | #31 |
| `codex/grasp-weld-ablation` | `a3d53d8c9d836cc2b4f0888f660318d2caf7cd8d` | #14 |
| `codex/individual-gemini-proxy` | `650f9222793fb1863091491a6eaa8f1a283a6f70` | #27 |
| `codex/integrate-remaining-prs-20260920` | `3bcabcd0571eb14f6348965d1e88f0f2f3caca11` | #69 |
| `codex/owner-approval` | `d19e400841c4cc0251400aa0eff913aad5277ecc` | #22 |
| `codex/plain-payload-grasp` | `bbf5707d44df2043756d716836cb472f2b87d3d3` | #12 |
| `codex/post-merge-local-sync` | `17d0c4ce4727df169cdc8f5fe24b611d2755feb1` | #28 |
| `codex/record-audit-20260909` | `6bec0fbdf0f5b4fd21d2a2f9918a783df7fbbcb8` | #5 |
| `codex/remove-cargo-markers` | `b4eb83d0a983b53e0c164deb54c6baae0211aa57` | #25 |
| `codex/retire-legacy-simulation` | `9659f7a464ab1a73e5f160d7142208eeba45848c` | #7 |
| `codex/side-grasp-probe` | `525c009ab22769f0caa1760e9af3f85ffbc6c454` | #20 |
| `codex/ubuntu-quickstart` | `79c156c41a89e519c667ac8f0a074002c4c7b704` | #11 |

모든 위 SHA는 기준 main에서 도달 가능하다. 필요하면 해당 SHA로 로컬 브랜치를 복원하고 명시적으로 원격에 push할 수 있다. worktree 등록 백업과 전후 목록·명령 결과는 원본 Mac의 `/Users/changmin/projects/ugrp/outputs/repository-audit-20260920/`에 있다. 이 로컬 백업을 원격 백업으로 표현하지 않는다.

## 문서 정리

- `AGENTS.md`: 날짜별 누적 지시를 현재 유효한 규칙으로 통합했다. 10,440 → 7,023바이트. 기본 입력 제한과 교사/지도 예외, 카메라/FOV·외관·weld OFF, 증거 보존, Drive 제외, 프로세스 수명, PR 승인과 병합 후 기본 체크아웃 동기화를 유지했다. 정리 전 원문은 기준 main의 AGENTS.md에 보존돼 있다.
- `README.md`: 10,207 → 3,122바이트. 최신 main에 포함된 후속 공동 출하 6조건 검증과 ACT 한계를 앞에 두고 이전 검증 본문은 `docs/archive/validation_summary_20260917.md`로 옮겼다. 본문은 상대 링크 경로만 조정했다.
- `docs/current_status.md`, `docs/README.md`: 목적별 실행/문서 진입점과 실험 증거를 필요한 만큼 읽는 순서를 추가했다. 파일 검색 제외를 접근 차단으로 표현하지 않는다.
- 과거 아키텍처·결정 이력·출하 환경 문서에 기록 시점과 후속 근거를 표시했다. 기존 경로와 실험 자료는 유지했다.

## 검증과 남은 범위

변경 Markdown의 상대 링크 82개·새 진입점의 코드 경로 8개·이전 README 검증 본문 보존·`git diff --check` 검사를 통과했다. 문서 변경이므로 새 로봇/모델 실험은 실행하지 않는다. PR CI 결과는 GitHub check 상태에서 별도로 확인한다.

기존 worktree의 무시 파일·모델·영상·데이터 중복 제거와 미통합 브랜치의 삭제는 수행하지 않았다. 앞으로 문서를 정리할 때도 실행 SHA·실험 원본 위치와 해시를 유지하고 과거 실패를 지우지 않는다.
