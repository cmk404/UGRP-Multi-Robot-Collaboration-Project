# 2026-09-25 구역 작업 결과의 RGB 전용 판정 (L4 대체 후보) — 작업 중

[경계 감사](../2026-09-25-zone-comm-boundary-audit/README.md)의 L4(작업 영수증·깨우기가 교사의 정답 검사에서 나옴)를 대체할 **RGB 전용 작업 결과 판정** 모듈을 만든다. 러너(`scripts/run_zone_dispatch.py`), 교사(`scripts/zone_teacher.py`), `harness/zone_coordination.py`는 다른 작업(`claude/zone-team-a2`)이 고치는 중이라 건드리지 않는다.

- 분할: [split.json](split.json)을 test 채점 전에 고정했다. dev = 스크립트 fixture 실행 22개, test = LLM 실행 30개(ZC2 18, Z1–Z3 12).
- 상태: 작업 중. 결과는 아직 없다.
