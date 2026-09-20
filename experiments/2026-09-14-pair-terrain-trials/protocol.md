# 예시 지형 실제 공동 운반 시험

사용자 요청: 이론적 통과 가능성을 확인한 여섯 예시 지형에서 실제 시험을 실행한다.

- `maps/pair_navigation/catalog.json` 순서대로 다섯 통과 가능 지형과 완전 차단 대조를 각 1회 실행한다.
- 기존 파지 유지가 확인된 명시적 `impratio=10` 비교 조건을 사용한다. 실행기 기본값 1은 변경하지 않는다.
- `scripts/run_pair_navigation.py`, 기존 grasp 학생 모델, 동일 파지 후 고정 8초 대기,
  조건당 750회(회당 0.2 SIM초) 판단 예산을 사용한다. 벽시계 제한은 조건당 600초다.
- 카메라·FOV·물체·질량·마찰계수·집게 힘·지도·계획기·제어기·평가 기준을 수정하지 않는다.
  weld OFF, noslip_iterations=0. 이전 이론 분석의 대안 경유점은 제어기에 넣지 않는다.
- 배우 입력은 기존 RGB, 승인된 정적 지도·고정 보정, 자기 발행 명령 이력이다.
  정답 위치·관절·접촉·평가는 출력 기록과 사후 진단에만 사용한다.
- 실행 코드와 프로토콜을 먼저 커밋하고 여섯 조건 종료까지 소스를 고정한다.
- 도착·파지 유지·높이·기울기·전 tick 충돌·지도 경계·내려놓기를 기존 평가기로 판정한다.
  완전 차단은 경로 없음에 따른 공동 정지를 별도 기대 결과로 기록하고 운반 성공률에 넣지 않는다.
- 성공 여부와 무관하게 전 조건, RGB 입력/명령 재생 감사, 영상 검토 범위, 원본 해시를 남긴다.
  조건당 한 번의 고정 시작 시험이며 반복 성공률이나 새로운 파지의 일반화로 해석하지 않는다.
- 물리 결과에 영향을 주는 오류나 시간 초과가 있으면 그대로 남긴다. 설정을 바꾼 진단은 별도 코호트다.
- LLM 호출과 외부 모델 비용은 0이며, 파지에는 저장된 고전 영상 학생 모델을 사용한다.
- 원본 영상·로그는 `outputs/pair-terrain-trials-2026-09-14/`에 로컬 보관한다.
  버전 관리할 요약·입력 설정·원본 해시·선택 영상은 이 실험 폴더에 저장한다.

실행:

```sh
/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run terrain-trials-20260914 -- \
  /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python experiments/2026-09-14-pair-terrain-trials/run_cohort.py \
  --mjpython /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/mjpython \
  --grasp-model-dir /Users/changmin/projects/ugrp-worktrees/pair-loaded-navigation/outputs/pair-navigation/runtime/models/grasp \
  --out-dir outputs/pair-terrain-trials-2026-09-14
```

다른 PC에서는 해당 환경과 복원된 모델 경로를 명시한다. 종료 후 같은 세션과 자식 프로세스가
정리되었는지 확인한다. 결과 기록에는 실제 실행 SHA와 환경을 자동 저장한다.
