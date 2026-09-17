# 실행 순서와 원시 자료

이 실험은 기존 로컬 MuJoCo 환경과 RGB 파지 스킬 원본을 사용한다. Git에는 설정·코드·작은 결과 요약만 포함한다. 원시 시연과 체크포인트가 없는 새 clone만으로 최종 모델을 재평가할 수 있다고 주장하지 않는다.

- 실험 worktree: `/Users/changmin/projects/ugrp-worktrees/act-recovery-generalization`
- 시뮬레이션 Python: `/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python`
- macOS MuJoCo 실행기: 같은 환경의 `bin/mjpython`
- ACT Python: `/Users/changmin/Project-Runtimes/ugrp/.venv-reference-act/bin/python`
- 공통 파지 스킬: `/Users/changmin/projects/ugrp/outputs/grasp-recovery-model-20260910-v1`

모든 긴 작업은 `scripts/ugrp_session.py run <고유 세션 이름> -- <명령>`으로 실행한다. 결과 디렉터리는 매번 새로 지정한다. 시뮬레이션은 하나씩, CPU 학습은 최대 두 개(각 torch 2 threads)씩 실행했다. 클라우드 GPU나 외부 모델 API는 사용하지 않았다.

1. `scripts/collect_recovery_curriculum.py --protocol-dir experiments/2026-09-16-act-recovery --out outputs/curriculum-v1 --mjpython <실행기> --grasp-model <파지 스킬>`로 전체 교사 파일럿, 개발 12, 정상 48, 단일 오류 24를 수집한다.
2. `scripts/build_recovery_datasets.py --curriculum outputs/curriculum-v1 --out outputs/datasets-v1`로 명시적인 궤적 목록을 만든다.
3. ACT Python으로 `scripts/train_recovery_act.py --dataset outputs/datasets-v1/bootstrap.json --out outputs/bootstrap-seed16-v1 --seed 20260916 --steps 5000`을 실행한다. 부트스트랩은 최종 비교 대상이 아니다.
4. `scripts/run_recovery_cases.py --cases experiments/2026-09-16-act-recovery/aggregation-cases.json --out outputs/aggregation-v1 --policy act --takeover --model outputs/bootstrap-seed16-v1 --act-python <ACT Python> --mjpython <실행기> --grasp-model <파지 스킬>`로 학생 prefix 뒤 실제 물리 상태에서 교사 tail을 수집한다.
5. 2번 데이터셋 생성기에 `--aggregation outputs/aggregation-v1`을 추가한다. 제외된 시연 자리는 새 사례로 채우지 않는다.
6. `train_recovery_act.py`를 nominal/recovery/aggregated 각각 seed 20260916/20260917, `--steps 10000`으로 실행한다. 동일 구조·샘플링 규칙·업데이트 수, 새 초기화, 동일 개발 세트 기반 체크포인트 선택이다.
7. 모든 모델이 완료된 뒤 `scripts/run_recovery_final.py --spec outputs/final-spec-v1.json --mjpython <실행기> --act-python <ACT Python> --grasp-model <파지 스킬>`로 8조건×16사례를 실행한다. final spec은 모델·결과 폴더의 절대 경로를 명시한다.
8. `scripts/audit_recovery_experiment.py --spec outputs/final-spec-v1.json --grasp <파지 스킬> --out <새 JSON>`으로 원본 해시, 시작 상태/영상 일치, 실제 RGB 요청, 교사 명령, 일부 학생 판단, 전체 공통 채점을 검증한다.
9. `scripts/compose_recovery_comparison.py --spec outputs/final-spec-v1.json --seed 16 --out <새 폴더>`로 사전 선택한 각 그룹 첫 사례의 네 정책을 실제 SIM 시각에 맞춰 영상으로 비교한다. seed 17도 동일하게 생성한다. 원본 프레임 사이에는 직전 화면을 유지하고, 조기 종료한 영상은 마지막 화면을 유지한다.

## 데이터 구조

각 시연 폴더의 `actor_samples.json`에는 두 RGB 파일 참조만, `teacher_labels.json`에는 연속 3축 명령·정지 표적·제외 영역 여부를 기록한다. 전체 교사 상태와 판정은 별도 `result.json`/`evaluation-only.jsonl`에 있다. 학생은 실행 중 이 상태 파일을 읽지 않는다. 모델별 `report.json`에는 모든 학습/개발 원본 경로와 해시, 선택한 체크포인트, seed·환경·시간, 모델 파일 해시를 남긴다.

`outputs`의 영상·이미지·가중치·로그는 이 Mac의 로컬 보관이다. 원격 백업이나 Drive 업로드는 하지 않았다. CI는 Ubuntu 환경에서 코드·실제 ACT forward/backward/저장·로드·입력 경계와 기본 시뮬레이션을 확인하며, 이 전체 실험의 다른 OS 재현을 의미하지 않는다.
