# 시뮬레이션 속도: 동작 불변 설정과 대기열

2026-09-26 사용자 요청("좀 빨리 할 수 있는 방법이 없을까")에 따라 로봇 행동·결과를 바꾸지 않고 CPU를 줄이는 방법을 정리한다. 측정·동등성 근거는 [실험 기록](../experiments/2026-09-26-sim-speed/README.md)에 있다. 물리·timestep·solver·접촉·카메라 해상도/FOV/외관·렌더러 설정·제어 주기는 바꾸지 않는다.

## 1. 비트 동일 가속 `exact-v1` (M1 러너)

```sh
.venv-sim-worker-mac/bin/python scripts/run_m1_owncam.py --prereg experiments/2026-09-26-zone-m1-owncam/prereg.json \
  --only m1dev-s93 --output /Users/changmin/projects/ugrp/outputs/<새 폴더> --speedups exact-v1
```

- `sim/exact_speedups.py`의 두 항목만 켠다. 구동 산술(`ExactDriveKernel`, 기존 `PhysicsDriveKernel` 훅 재사용)은 `np.dot`의 합산 순서를 그대로 따르고, 시작 시 `np.dot`과 비교하는 자체 검사가 실패하면 원래 경로로 돌아간다(`manifest.json`의 `speedups.drive_kernel`에 기록). 접촉 사전 필터(`ContactPrefilter`)는 러너가 어차피 건너뛰는 접촉만 벡터 연산으로 뺀다.
- 기본값은 `none`(기존 경로)이다. 적용 여부는 `manifest.json`의 `speedups`에 남고 `result.json`은 바뀌지 않는다.
- 다른 러너에서 구동 산술만 쓰려면 월드 생성 뒤 `from sim.exact_speedups import install_drive_kernel; install_drive_kernel(world)`를 호출한다. 러너마다 동등성 검증을 따로 한다. 훅에 이미 다른 커널(비트 동일이 아닌 `PhysicsDriveKernel`, 다른 버전, 다른 월드)이 있으면 유지·교체하지 않고 `RuntimeError`로 거부한다. 같은 버전의 정확 커널이면 `exact_drive_kernel_already_installed`를 돌려준다.
- **연구 코호트 기본값은 아직 `none`이다.** 2026-09-26 Codex 검토 판정에 따라, 최종 소스를 고정한 전체 M1 A/B에서 파지·상승 이후까지 qpos 해시를 비교하기 전에는 `exact-v1`을 연구 공통 기본값으로 채택하지 않는다(지금까지 qpos 동일성은 seed별 첫 120 SIM s만 확인).
- 다른 플랫폼(Ubuntu/OpenBLAS 등)이나 패키지 버전에서는 자체 검사 결과와 동등성을 다시 확인한다. Mac 결과와 Linux 결과는 같은 코호트로 합치지 않는다(렌더러·부동소수점이 다름).

## 2. 동등성 확인

```sh
python3 scripts/sim_equivalence.py <기준 run 폴더> <새 run 폴더>                 # 전체 실행
python3 scripts/sim_equivalence.py <절단 run> <전체 run> --until-sim-s 120       # 절단 실행과 앞부분 비교
```

명령·제어 입력 프레임 행과 JPEG 바이트·제어기/스킬/매크로 로그·평가 전용 로그·`result.json` 전 필드를 비교한다. `scripts/sim_profile.py` 출력끼리는 qpos/qvel/act SHA-256 체크포인트(기본 2,000 `mj_step`마다)도 비교한다. 증거가 부족하면 통과가 아니다(`verdict: insufficient_evidence`, 종료 코드 2): 두 경로가 폴더여야 하고, `inputs/commands.jsonl`·`inputs/frames.jsonl`(전체 비교는 `result.json`·`manifest.json`도)이 있어야 하며, 모든 행이 JSON 객체여야 하고, `--until-sim-s T`면 양쪽 제어 프레임이 T 이후까지 이어져야 하며, 비교한 명령·프레임·JPEG(양쪽이 프로파일 폴더면 체크포인트) 수가 0이면 안 된다. 종료 코드 0 = 동일, 1 = 다름. 새 가속 항목은 두 seed 이상의 전체 실행에서 모두 일치해야 채택한다. 하나라도 다르면 버리거나 [실행 버전 관리](execution_versioning.md)에 따라 새 실행 버전으로 표시한다. qpos 체크포인트는 `sim_profile.py`로 돌린 구간에만 있으므로, 전체 임무의 상태 동일성을 주장하려면 양쪽 모두 전체 임무를 프로파일러로 실행해야 한다.

## 3. CPU 측정

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1 \
python3 scripts/ugrp_session.py run kiro-prof -- .venv-sim-worker-mac/bin/python scripts/sim_profile.py m1 \
  --episode m1dev-s93 --sim-limit 120 --speedups exact-v1 --output /Users/changmin/projects/ugrp/outputs/<새 폴더> [--sections|--cprofile]
```

비교는 wall이 아닌 CPU 초와 **retired instruction 수**로 한다. Apple M3(성능 코어 4 + 효율 코어 4)에서는 같은 구간의 CPU 초도 부하에 따라 2배 이상 달라진다(효율 코어 배치·경합). `profile.json`의 `counters`(instructions, P 코어 비율)와 `cpu_at_sim_mark`(고정 SIM 시각까지의 CPU)를 함께 본다. wall 비교가 필요하면 `scripts/agent_lock.py` 잠금을 잡는다.

## 4. 머신 전체 sim 대기열 `scripts/sim_slots.py`

```sh
python3 scripts/sim_slots.py status
python3 scripts/ugrp_session.py run kiro-m1 -- python3 scripts/sim_slots.py run --owner kiro --label m1-s93 -- \
  .venv-sim-worker-mac/bin/python scripts/run_m1_owncam.py ...
```

- 슬롯 수 = 머신 전체 sim 상한(기본 6, 사용자 규칙; `UGRP_SIM_SLOTS`·`--slots`로 조정하되 모든 에이전트가 기본값을 쓴다). 슬롯은 `outputs/sim-slots/`의 파일 잠금(`fcntl.flock`)이며, 보유 프로세스가 끝나거나 죽으면 커널이 잠금을 푼다. 자식 명령도 잠금 파일 서술자를 물려받는다.
- 입장은 원자적이다. 전역 `admission.lock`을 잡은 채 "실행 중 sim 수 집계 → 상한 비교 → 빈 슬롯 예약"을 한 번에 한다. 실행 중 = 잡힌 슬롯(모든 슬롯 파일) + 대기열 밖 sim. sim은 argv 문자열이 아니라 **MuJoCo(`libmujoco`)를 실제로 불러온 프로세스**다(macOS `lsof`, Linux `/proc/<pid>/maps`). 슬롯 보유자와 그 자손은 슬롯으로 이미 세고, 기다리는 러너(`queue.lock`)는 아직 실행 중이 아니다. 집계를 할 수 없으면(`lsof`·`/proc` 없음) 시작하지 않고 실패한다(종료 코드 70). 예전 `--ps-cap`은 경쟁 조건 때문에 없앴다.
- Python 러너는 `with sim_slot(owner='kiro', label='...'):`로 감쌀 수 있다. 다른 프로세스를 멈추거나 신호를 보내지 않는다. `status`는 보유자 기록과 대기열 밖 sim PID를 보여 준다.

## 5. 개발용 앞부분 절단(dev slice)

```sh
... scripts/sim_profile.py m1 --episode m1dev-s93 --speedups exact-v1 --stop-at-phase skill:to_carry_posture --output ...
```

제어기가 지정 단계(`phase_times` 키 형식)에 들어가면 다음 결정 전에 끝낸다. 그 전까지는 전체 임무와 같은 궤적·명령·프레임이다. 출력에 `DEV_SLICE_NOT_A_RESULT.txt`가 생기며 M1 결과로 보고하지 않는다. 동결·시험 판단은 전체 임무로 한다. 문 통과·배치 같은 뒤쪽 단계부터 시작하는 slice는 제어기 진입점이 필요해 아직 없다(실험 기록 참조).

## 6. 원격 병렬 코호트(실행하지 않음)

| 경로 | 병렬성·비용 | 절차·주의 |
|---|---|---|
| Ubuntu 팀원 PC ([설치](ubuntu_quickstart.md)) | 코어 수만큼 동시 실행, 추가 비용 없음 | `MUJOCO_GL=osmesa` 기본(소프트웨어 렌더링). EGL GPU 렌더링은 별도 검증. Mac과 렌더러·부동소수점이 달라 **별도 실행 환경 코호트**로 기록 |
| Kaggle CLI ([안내](kaggle_simulation.md)) | 비공개 CPU kernel, 무료. 이 CLI 경로의 작업당 최대 요청 1,800초, 실제 할당은 `kaggle quota` | `prepare → submit → status → collect`. 인터넷 OFF, OSMesa. M1 한 에피소드(Mac 저부하 CPU 약 490초, 렌더 약 150초)가 OSMesa에서 1,800초 안에 끝나는지 먼저 짧은 slice로 측정해야 함 |
| Colab CLI ([안내](colab_simulation.md), [L4 검토](colab_standard_simulation_review_20260923.md)) | 무료 CPU 런타임 또는 계정의 CCU. 유료 자원 자동 구매 금지 | 전송기가 OSMesa를 강제. L4 EGL 연결·검증 미완. 세션당 유한 배치 후 회수·해시 검증, 자기 세션만 정리 |

세 경로 모두 결과 회수·해시 검증까지 해야 완료이며 Mac 결과와 섞지 않는다. 이번 작업은 원격 자원을 만들거나 실행하지 않았다.
