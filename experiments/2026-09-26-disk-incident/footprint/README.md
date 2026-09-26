# 2026-09-26 디스크 사용량 측정·적용 기록

[디스크 관리](../../../docs/disk_management.md)의 수치 원본이다. 모두 `du` 기준 할당 크기이며, 측정 중에도 다른 에이전트의 실행이 계속됐다.

| 파일 | 내용 |
|---|---|
| `disk-report-before.txt` | `scripts/disk_report.py` 첫 실행(15:26, 부하 평균 8.18/12.58/27.89) |
| `disk-report-after.txt` | Codex worktree 14개 은퇴와 이 PR worktree sparse 전환 뒤(16:05, 부하 평균 17.96/48.48/62.67) |
| `retirements-20260926.jsonl` | `agent_worktree.py retire --execute` 영수증 15개(Codex 14 + 측정용 probe 1) |
| `sparse-probe.json` | 실제 새 sparse worktree의 크기와 그 안에서 돌린 테스트 |

- before 보고서의 `[virtual environments] 0.01 GiB`는 보고서 버그 때문에 잘못 나온 값이다. 같은 대상을 가리키는 링크 두 개가 합계를 덮어썼다. after 전에 고쳤다.
- 올바른 값은 1.12 GiB다(14:49 `du`, after 보고서와 같음). 그래서 before 합계는 보고서의 148.29가 아니라 149.4 GiB다.
