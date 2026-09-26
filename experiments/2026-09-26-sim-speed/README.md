# 2026-09-26 시뮬레이션 속도: 동작 불변 CPU 절감 (Kiro, PR #209)

사용자 요청(2026-09-26 "좀 빨리 할 수 있는 방법이 없을까"): 로봇 행동·결과를 바꾸지 않고 sim wall 시간을 줄인다. 대상은 M1 러너(`scripts/run_m1_owncam.py`, PR #201)이고 pair 러너(`scripts/run_m2_pair.py`, PR #203/#205)는 보조 대상이다. 첫 Kiro 세션(계정 1)이 프로파일 도구와 첫 프로파일까지 진행하고 멈춘 뒤, 계정 2가 같은 worktree·브랜치·PR에서 이어받았다. 사용법은 [docs/sim_speed.md](../../docs/sim_speed.md)에 있다.

## 결론

- **`exact-v1`(구동 산술 + 접촉 사전 필터)은 M1에서 retired instruction을 16.6–18.3% 줄였다.** 같은 부하에서 CPU 초도 18.3% 줄었다(s93, 첫 120 SIM s). pair 러너에는 구동 산술만 해당해 −4.5%였다. 2개 seed의 **전체 임무**에서 명령 6,541/6,910개, 제어 입력 프레임 2,080/2,309장의 JPEG 바이트, 모든 로그, `result.json` 전 필드가 기존 dev-a8 실행과 같았다. 첫 120 SIM s의 qpos/qvel/act 해시 체크포인트 240개도 같았다.
- 이 Mac의 렌더링은 드물게 실행마다 달라진다: pair 기준 두 번 사이에 785장 중 1장이 최대 1 단계 달랐다(궤적·결정 동일). M1은 비교한 약 1만 장 전부 같았다.
- 남은 CPU는 `mj_step`(약 37%)과 robot_cam 렌더(약 28%, 그중 그림자 약 75%)다. 둘 다 물리·영상을 바꾸지 않고는 줄일 방법을 찾지 못했다.
- **가장 큰 wall 요인은 머신 과부하였다.** 같은 코드·같은 120 SIM s 구간이 부하 3–5에서 CPU 156초, 부하 12→90에서 361초였다(2.3배). instruction 수는 같고 성능 코어 비율이 떨어진다(0.95 → 0.61–0.75). 동시 sim을 제한하는 `scripts/sim_slots.py`를 추가했다.

## 환경·코드

- Mac Apple M3(성능 코어 4 + 효율 코어 4), Python 3.12.13, MuJoCo 3.12.0, NumPy 2.5.2(Accelerate), OpenCV 5.0.0, Pillow 12.3.0. `OMP/OPENBLAS/VECLIB/MKL_NUM_THREADS=1`. OpenCV는 이 변수와 무관하게 GCD 작업자 8개를 쓴다.
- 기준: `outputs/m1-owncam-20260926/dev-a8/{m1dev-s93,m1devdiag-s95}` (PR #201, `16b41e27`, v9 스킬, `cargo_noslip_v1`, weld OFF). 이 브랜치 HEAD와 실행 파일(RUNTIME_FILES) 해시가 같다.
- 새 실행: 브랜치 `kiro/sim-speed`의 `b718da5`·`493bc0f`·`fb9c451`·`37955f6`·`92c5ea1`. 실행 파일은 `scripts/run_m1_owncam.py`의 `speedups` 인자와 새 `sim/exact_speedups.py` 외에는 기준과 같다. 모든 실행은 `code.dirty=false`다(`exact1-full-s95`는 추적 안 된 도구 파일 때문에 dirty로 기록돼 수 초 만에 멈추고 `exact1-full-s95b`로 다시 실행했다. 첫 세션의 `prof-s93-120-sections-base`도 dirty다).
- raw: `/Users/changmin/projects/ugrp/outputs/sim-speed-20260926/` (실행별 `profile.json`, `qpos_checkpoints.jsonl`, `run/`, 동등성 보고 `equivalence/*.json`, 실행 시각·부하 `logs/launch_load.txt`).

## CPU 분해 (기준, s93 첫 120 SIM s, 부하 3–5)

`base120i-s93`: 프로세스 CPU 157.2초 = 주 스레드 107.9 + 렌더 스레드 43.4 + 기타(OpenCV GCD 등) 6.0, instruction 2,178 G, `mj_step` 482,948회. 주 스레드 항목은 같은 장면의 스텝당 마이크로벤치(저부하)에 호출 수를 곱한 값이다.

| 항목 | CPU 초(추정) | 비율 | 비고 |
|---|---:|---:|---|
| `mj_step` 물리 | 58 | 37% | 120.7 µs/step, noslip 10·Newton·elliptic, timestep 0.25 ms. 변경 불가 |
| robot_cam 렌더(GL) | 43 | 28% | 프레임 597장, 그림자 광원 4개·shadowsize 4096·MSAA 4. 그림자 끄면 렌더 −74%지만 300,548 px 변경 → 불가 |
| 러너의 스텝별 접촉 루프 | 33 | 21% | 68 µs/step, 매 스텝 `data.contact[i]` 40개 순회 → **exact-v1** |
| `_physics_step_for` 구동 산술 | 11 | 7% | 23 µs/step, `np.dot`·`base_rpy` 잠금 → **exact-v1** |
| 제어기(PF·태그·색 검출), JPEG 인코딩/디코딩, base64, 디스크, JSONL | 약 12 | 8% | 첫 세션 구간 계측(고부하): on_frame 7.1초, AprilTag 2.4, PF 3.7, PIL JPEG 0.8, imdecode 0.6, 쓰기 0.4, JSONL 0.05 |

## exact-v1 결과

| seed | 비교 | 결과 |
|---|---|---|
| s93 | dev-a8 전체 vs `exact1-full-s93` | 명령 6,541·프레임 2,080(JPEG 바이트)·controller/skill/macro 로그·frames_eval·gt 7,919·retention 3,999·`result.json` **모두 동일** (`equivalence/s93_full_deva8_vs_exact1.json`) |
| s93 | `base120i-s93` vs `exact1-full-s93` (<120 s) | qpos 체크포인트 240, 명령 1,485, 프레임 594 **동일** |
| s95 | dev-a8 전체 vs `exact1-full-s95b` | 명령 6,910·프레임 2,309·로그·gt 8,468·retention 3,645·`result.json` **모두 동일** |
| s95 | `base120i-s95` vs `exact1-full-s95b` (<120 s) | qpos 체크포인트 240, 명령 1,614, 프레임 631 **동일** |
| 둘 다 | 원 러너 `base120-*` vs `--speedups none` `base120i-*` | 120 s 절단 전체 동일(기본 경로 불변) |

첫 120 SIM s CPU (`cpu_at_sim_mark`):

| seed | 조건 | 프로세스 CPU 초 | 주 스레드 | instruction (G) | P 코어 CPU 비율 | 부하 |
|---|---|---:|---:|---:|---:|---|
| s93 | none | 156.3 | 107.3 | 2,166 | 0.95 | 2.7→4.8 |
| s93 | exact-v1 | 127.7 (−18.3%) | 80.0 (−25.5%) | 1,770 (−18.3%) | 0.95 | 동시 실행 |
| s95 | none | 165.7 | 110.8 | 2,195 | 0.93 | 4.9→5.1 |
| s95 | exact-v1 | 169.8 | 105.8 | 1,831 (−16.6%) | 0.75 | 5.6→44 (다른 작업 부하) |

s95 exact-v1의 CPU 초는 실행 중 다른 작업의 부하(1분 평균 최대 156)로 효율 코어에 밀려 늘었다. 이 경우 instruction 수로 비교한다. 전체 임무 CPU: s93 491.7초(SIM 397 s, 저부하), s95 733.9초(SIM 425 s, 고부하, P 비율 0.64). wall은 잠금 없이 잰 참고값이다.

`--cv-threads 1`(OpenCV 단일 스레드)은 동등성은 유지했으나 instruction이 +0.2%로 이득이 없어 권장하지 않는다(`exact1-cv1-120-s93`).

## 채택하지 않은 후보

| 후보 | 판정·이유 |
|---|---|
| 기존 `sim/physics_drive_kernel.py` 그대로 사용 | `allclose` 수준만 같다. `np.dot`(Accelerate)은 4개 합을 `(v0+v2)+(v1+v3)` 순서로 더해 임의 입력의 약 25%에서 마지막 비트가 다르다. 같은 훅을 쓰는 `ExactDriveKernel`로 대체했고, 기존 커널은 비트 비교 시험에서 실패함을 확인했다 |
| 보이지 않는 AprilTag 칸 geom 제외(렌더) | 시제품에서 렌더 CPU −34%, 무작위 자세 120개 중 4개가 1–3 px 달라 바이트 동일성 위반. 채택하려면 새 실행 버전·재검증 필요 |
| 그림자·반사·MSAA·shadowsize 변경 | 영상이 바뀜(그림자 끄기 300,548 px, 반사 끄기 18,996 px) — 금지 |
| 렌더와 물리 겹치기 | `on_frame`이 제어기·스킬 단계를 바꿀 수 있고 러너가 매 스텝 그 단계를 읽음(평가 기록·pose 매크로 대기 시간) → 순서가 바뀌면 동일성 깨짐 |
| 같은 SIM 시각 중복 프레임 재사용 | s93 2,080장 중 67장(3%). 원리상 동일하지만 이득이 작아 보류 |
| PF 벡터화 | `harness/owncam_localizer.py`는 다른 작업 소유, 부동소수점 순서 위험, PF 약 2% |
| JPEG 비동기·일괄 저장, 로그 축소 | 합계 약 2%(JSONL은 종료 시 한 번 쓰기, 0.05초) |
| MuJoCo island/멀티스레드 | solver 결과가 바뀔 수 있어 물리 변경에 해당 |

## pair 러너(보조)

PR #205 `6990a6e`(`kiro/zone-m2-pair-v3`)의 `scripts/run_m2_pair.py`를 이 PR에서 고치지 않고, 분리된 sparse worktree에서 `pair_prof.py`(이 폴더, 원 러너를 그대로 호출)로 dev seed 701 open_floor를 실행했다. 두 로봇 모두 `done`, SIM 120.4 s, 프레임 785장.

- cProfile(`pair701-base-cprof-b`, process_time, 계측 부하 포함): `mj_step` 99.4초(35%), `mjr_render` 78.7초(28%, 785회), `_physics_step_for` 구동 산술·`base_rpy`·`_quat_to_rpy` 약 58초(계측 과대), 포트 tick 10.8초, `detectMarkers` 7.1초. pair 러너는 접촉 검사를 0.2 s마다 하므로 스텝별 접촉 루프 비용이 없다.
- 구동 산술만 `install_drive_kernel(world)`로 적용(`pair701-kernel`) vs 기준(`pair701-base`), 동시 실행(부하 52→85, P 비율 0.57 둘 다): instruction 2,011 → 1,921 G(**−4.5%**), 주 스레드 CPU 132.2 → 125.4초(−5.1%).
- 동등성: qpos 체크포인트 241개, 명령, `result.json`(wall·부하·SHA 제외 전 키), 프레임 784/785장이 같다. 다른 1장(`r2-00144.jpg`, 디코딩 후 최대 1 단계)은 **기준 두 번(`pair701-base` vs `pair701-base-cprof-b`) 사이에서도 똑같이 다르다.** 커널과 cProfile 기준은 792개 파일 전부 같다. 즉 커널 영향이 아니라 이 Mac의 렌더링이 드물게 실행마다 달라지는 현상이다(M1에서는 비교한 프레임 약 1만 장 전부 동일). 궤적·결정에는 영향이 없었지만, "프레임 바이트 동일"을 동등성 기준으로 쓸 때 이 기준선 변동을 먼저 확인해야 한다.
- 적용 방법: pair 러너가 월드 생성 뒤 `install_drive_kernel(world)` 한 줄을 부르면 된다. 러너 소유 PR에서 적용·재검증한다.

## sim 대기열과 원격 병렬

- `scripts/sim_slots.py`: `fcntl.flock` 기반 N 슬롯 세마포어(기본 코어 − 1 = 7). 보유자가 죽으면 커널이 해제한다. `--ps-cap 6`로 사용자 규칙(머신 전체 6개)도 지킨다. 기존 `grep -c` 규칙은 kiro-cli 프롬프트 문구를 세어 실제 sim 3개일 때 12로 보고했다. 테스트 7개.
- 성능 코어 4개 머신에서 5번째 이후 sim은 효율 코어에서 더 느리다(같은 instruction에 CPU 초 +60%, P 비율 0.61). 효율 코어까지 쓰면 전체 처리량은 늘 수 있지만(미측정) 개별 wall은 길어진다. 빠른 개발 반복은 동시 4개 이하가 유리할 것으로 본다.
- Ubuntu·Kaggle·Colab 병렬 코호트의 비용·절차는 [docs/sim_speed.md §6](../../docs/sim_speed.md)에 정리했다. OSMesa 렌더러와 Linux 부동소수점은 Mac과 달라 같은 코호트로 섞을 수 없고, 에피소드 시간(특히 OSMesa 렌더)은 측정하지 않았다. 원격 자원은 만들지 않았다.

## 개발용 slice

- 구현: `scripts/sim_profile.py m1 ... --stop-at-phase skill:to_carry_posture` (DEV 전용 앞부분 절단). 제어기가 지정 단계에 들어가면 다음 결정 전에 끝내고, 출력에 `DEV_SLICE_NOT_A_RESULT.txt`를 남긴다.
- `slice-s93-to-carry`(exact-v1): SIM 173.8 s에 파지 직후 단계에서 정지(정착 포함 175.4 s, 전체 397.2 s의 44%, instruction 2,750 G = 전체의 41%). 정지 시각 전까지 전체 실행과 qpos 체크포인트 347·명령 2,642·프레임 927장 JPEG 바이트·로그가 **동일**(`equivalence/s93_slice_to_carry_vs_exact1_full.json`). 탐색·접근·파지 단계 개발 반복에 쓴다.
- 제안(미구현): 문 앞에서 상자를 든 채 시작, 슬롯 앞에서 배치부터 시작. 둘 다 (1) weld 없이 파지 상태를 물리적으로 만드는 초기화, (2) 제어기·스킬 v9의 중간 단계 진입점(현재 위치 추정·보유 상태)이 필요하다. 해당 파일은 PR #201·자기 카메라 위치 추정 작업 소유여서 이 PR에서 고치지 않았다. 대안으로 전체 실행의 MuJoCo 상태(`mj_getState`)와 제어기 상태를 같은 SIM 시각에 저장·복원하는 checkpoint slice가 있으나, 러너 closure 구조 변경이 필요하다. 어떤 slice도 동결·시험 판단을 대신하지 않는다.

## TensorBoard

`outputs/tensorboard/0926-sim-speed`(14 run, `make_tb_snapshot.py`, `d28a36a`): `speed/*`(120 s 시점·전체 CPU, instruction, P 코어 비율, 동등성), `result/sim_s·commands·wall_s`(wall은 참고값), 전체 임무 M1만 `evaluation/reported_success`. 중단 2건은 텍스트만. EventAccumulator 값 37개가 원본 `profile.json`/`result.json`과 일치했고, 공용 서버(PID 9291, 재시작 없음)의 `/data/runs`·scalars API에서 14 run을 확인했다. `outputs/tensorboard-view.json`에는 `sim_speed_20260926` 키만 추가했다(다른 키 값·순서 불변, 파일은 indent 2로 다시 저장). [대시보드](http://127.0.0.1:6006/?pinnedCards=%5B%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22speed%2Finstructions_g_at_120s%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22speed%2Fcpu_process_s_at_120s%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22speed%2Fequivalent%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22evaluation%2Freported_success%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fsim_s%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fcommands%22%7D%2C%7B%22plugin%22%3A%22scalars%22%2C%22tag%22%3A%22result%2Fwall_s%22%7D%5D&smoothing=0&runFilter=%5E0926-sim-speed%2F#timeseries). 브라우저 화면은 열어 보지 않고 API로 값을 확인했다.

## 권장 설정

1. M1 개발 실행은 `--speedups exact-v1`. 새 동결·시험 코호트도 이 설정을 고정해 쓸 수 있다(동일성 검증됨, manifest에 기록).
2. 모든 sim을 `sim_slots.py run --ps-cap 6`으로 감싸 머신 과부하를 막는다.
3. CPU 비교는 `scripts/sim_profile.py`의 instruction 수와 `cpu_at_sim_mark`로 하고, 새 가속 항목은 `scripts/sim_equivalence.py`로 2개 seed 전체 임무 동일성을 확인한 뒤 채택한다.

## 참고 자료

- 논문
  - E. Todorov, T. Erez, Y. Tassa, "MuJoCo: A physics engine for model-based control", IROS 2012. https://doi.org/10.1109/IROS.2012.6386109
  - D. Goldberg, "What every computer scientist should know about floating-point arithmetic", ACM Computing Surveys 23(1), 1991. https://doi.org/10.1145/103162.103163 (합산 순서가 결과 비트를 바꾸는 근거)
  - 이번 세션에는 웹 조회 도구가 없어 두 논문의 원문을 다시 열지 않았다. 알고리즘을 논문에서 가져오지 않았다.
- OSS·라이브러리
  - MuJoCo 3.12.0 (Apache-2.0): `mj_step`, `MjData.contact.geom` 배열, `mujoco.Renderer`, `mjtRndFlag`(진단에만 사용). 설치본 `mujoco/cgl/__init__.py`에서 CGL 가속 컨텍스트를 확인했다.
  - NumPy 2.5.2 (BSD-3): `np.dot`의 Accelerate BLAS 합산 순서를 경험적으로 확인하고 자체 검사로 고정. `np.flatnonzero` 마스크 필터.
  - OpenCV 5.0.0 (Apache-2.0): GCD 스레드 진단(`setNumThreads`). TensorBoard 2.21 이벤트 protobuf — 기존 `scripts/tensorboard_tools/export.py`의 `Writer` 재사용.
  - Python 표준 라이브러리 (PSF): `fcntl.flock`, `cProfile`, `resource.getrusage`, `time.thread_time`.
  - macOS `proc_pid_rusage(RUSAGE_INFO_V6)` (libproc): instruction·cycle·P 코어 시간. 구조체는 Xcode SDK `sys/resource.h`의 `rusage_info_v6`로 확인.
  - 채택하지 않음: `filelock`(Unlicense, 단일 잠금이라 계수 세마포어 아님, 미설치), `posix_ipc`(BSD, 이름 있는 세마포어가 비정상 종료 시 해제되지 않음), GNU parallel `sem`(GPL-3, 미설치), util-linux `flock(1)`(macOS에 없음), pyinstrument(BSD-3, 미설치)·py-spy(MIT, macOS에서 root 필요) — 대신 cProfile과 스레드 CPU 구간 계측을 사용.
- 내부 모듈·PR
  - `sim/physics_drive_kernel.py`(`_fast_drive_kernel` 훅, `scripts/run_dispatch_skills.py`에서 사용): 재사용·비트 동일 버전으로 확장.
  - `sim/contact_audit_kernel.py`, `scripts/run_zone_owncam_skill_v9.py`의 `d.contact.geom` 벡터 필터: 접촉 사전 필터의 패턴.
  - `scripts/agent_lock.py`(기본 경로·기록 형식), `scripts/ugrp_session.py`(프로세스 그룹 실행) — 대기열 설계에 재사용.
  - PR #201 `scripts/run_m1_owncam.py`, dev-a8 실행 결과(기준), PR #203/#205 `scripts/run_m2_pair.py`.
- 문서: `docs/execution_versioning.md`, `docs/ubuntu_quickstart.md`, `docs/kaggle_simulation.md`, `docs/colab_simulation.md`, `docs/colab_standard_simulation_review_20260923.md`, `AGENTS.md`.
