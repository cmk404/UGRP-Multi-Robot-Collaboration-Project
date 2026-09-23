# 표준 시뮬레이션의 Colab L4 사용 검토 — 2026-09-23

판정: **같은 표준 시뮬레이션을 Colab의 유한한 headless 작업으로 실행할 수 있는 구조다. L4 GPU 렌더링까지 기본 전송 명령으로 완료된 상태는 아니다.** 현재 소스·설정·결과 관리 계층을 재사용하고 실행 환경만 Colab으로 선택한다. 별도 Colab용 시뮬레이터나 별도 연구 버전을 만들 필요는 없다.

## Google AI Pro 혜택

사용자가 말한 혜택은 Google AI Pro에 추가된 Colab 제공이다. 조회일의 [Google AI Pro 공식 도움말](https://support.google.com/googleone/answer/14534406?hl=en)은 **200 CCU**를 명시한다. 무료 체험에는 제공하지 않으며 가족 요금제 관리자 등 적용 조건이 있다. [Colab FAQ](https://research.google.com/colaboratory/faq.html)는 해당 유료 AI 요금제의 월별 컴퓨팅 단위 지급과 가용성에 따른 GPU 제공을 설명한다.

200 CCU는 GPU 200시간 또는 L4 100시간의 고정 보장이 아니다. 예상 시간은 `남은 CCU / 해당 런타임의 시간당 CCU`다. 예를 들어 실제 표시 소모량이 2 CCU/h일 때만 200 CCU로 약 100시간이다. 이 2 CCU/h는 계산 예시이며 현재 L4 요율을 측정한 값이 아니다. 계정의 구독·잔액·소모량·GPU 할당은 실제 화면/런타임에서 별도로 확인해야 한다. 일반 런타임은 최대 12시간, 충분한 단위가 있는 Pro+는 연속 실행 최대 24시간이라는 FAQ 조건도 있어 한 번의 100시간 연속 작업을 전제하지 않는다.

[NVIDIA L4 사양](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/l4/PB-11316-001_v01.pdf)의 명목 GPU 메모리는 24GB다. 사용자에게 표시된 22.5GB와 계정의 실제 사용 가능 메모리는 이번에 런타임을 할당해 확인하지 않았다.

## 현재 코드와 맞는 부분·보완할 부분

| 항목 | 확인 결과 |
|---|---|
| CLI 접근 | 이 Mac의 `colab` 0.6.0에서 세션 조회 성공, 활성 세션 0개. `colab new --help`에 L4 지원 표시. 생성·업로드·GPU 실행은 하지 않음 |
| 소스·버전 | `colab_simulation_cli.pack`이 커밋 SHA를 유지한 sparse checkout과 포함/제외 파일·압축 해시를 보존. 표준 CLI 코드/config도 포함. 로컬 포장 검사에서 `examples/task_stage_sync/plan.json` 누락을 발견해 추적된 examples를 기본 묶음에 포함하도록 수정 |
| 기본 시뮬레이션 | Linux headless 경로 사용 가능. native Mac 창을 원격에서 그대로 여는 방식은 아니며 RGB/MP4/평가 자료를 회수 |
| GPU 렌더링 | `colab_simulation_cli.job_code`가 `MUJOCO_GL=osmesa`, `PYOPENGL_PLATFORM=osmesa`를 강제. L4 할당만으로 GPU 렌더링이 되지 않음 |
| EGL 준비 | `setup_colab_egl.py`가 기존 NVIDIA 라이브러리 등록과 새 GL context의 vendor 확인을 구현. 전송 실행기의 EGL 선택·검증·실패 차단에 연결해야 함 |
| 물리 계산 | 현재 `MultiMasterPiProductionV2`는 `mujoco.mj_step` 기반. L4가 이 CPU 물리 계산을 자동으로 CUDA로 바꾸지 않음. 접촉 solver/time step은 같은 실험 버전으로 유지 |
| 학습 | 표준 카탈로그의 과거 `train_carry_act` 경로는 reference policy가 CPU로 구성됨. 별도 `train_carry_input_act --device cuda` 경로는 존재. GPU 학습 대상으로 쓸 workflow·데이터·CUDA 의존성·checkpoint 회수의 명시 연결 필요 |
| 모델·지도 | 기본 전송은 큰 ZIP/가중치/raw 자료를 제외. dispatch 기본 모델 ZIP은 `--include experiments/dispatch-skill-integration-20260917/models.zip` 등으로 명시하고 외부 파일은 별도 해시 검증 |
| 결과 회수 | `run_colab_simulation`은 자기 output 트리만 ZIP으로 회수. 공통 manager의 `--record '{output}'`를 사용해 manifest와 artifacts를 같은 회수 트리에 넣어야 함 |
| LLM 연결 | Colab의 localhost와 Mac의 localhost는 다름. 기존 비공개 모델 중계 또는 원격에서 접근 가능한 승인된 endpoint가 필요. L4 혜택에 외부 모델 호출 비용이 포함되는 것은 아님 |

MuJoCo는 [EGL을 GPU 기반 headless 렌더링 경로로 제공](https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/egl/__init__.py)한다. 따라서 영상 생성에는 L4의 이점을 기대할 수 있지만, 현재 2초 worker 제한과 전체 운반 시간이 개선되는지는 실제 동일 조건 비교가 필요하다. CPU OSMesa와 NVIDIA EGL의 영상 차이가 RGB 판단에 미치는 영향도 검사한다.

## 권장 연결과 검증 순서

1. 표준 manager/장면 통합 소스를 커밋하고 해당 SHA·번들·지도·가중치를 고정한다.
2. Colab 전송기에 명시적인 EGL 선택을 추가한다. 실제 GL vendor/renderer가 NVIDIA이고 필요한 GPU가 배정됐는지 확인한 뒤에만 작업을 실행하며, 실패하면 CPU로 조용히 대체하지 않는다.
3. 같은 표준 CLI로 짧은 headless·RGB·결과 회수 검사를 한다. manager manifest, 장면 XML, 실행 환경, 전송 SHA와 ZIP 내부 해시를 확인한다.
4. 단독·공동 실행을 같은 Colab 환경/고정 조건에서 각각 검사하고 wall 지연·명령·실제 물리 성공을 따로 기록한다. 과거 CPU/다른 후보의 성공률을 새 환경 결과로 사용하지 않는다.
5. 통과 후 조건별 유한 배치로 확대하고 매 배치 결과/checkpoint를 회수한다. 종료된 물리 상태를 이어 붙일 수 있다고 가정하지 않는다.

기존 CPU 전송기에서 **공통 관리 기록까지 회수하는 연결 형식**은 아래와 같다. 이 명령을 이번 검토에서 원격 실행하지 않았으며, 아직 L4 렌더링 명령은 아니다.

```sh
python3 scripts/colab_simulation_cli.py \
  --session OWN_SESSION_NAME --output outputs/colab-standard-NEW \
  --module scripts.sim_cli --timeout 300 \
  -- workflow run local --record '{output}' --timeout 180 \
  -- run configs/simulation/drive.json --headless --capture
```

이번 검토는 공식 안내·현재 소스·설치 CLI와 세션 조회에 근거한다. 개인 계정의 유료 혜택 활성화, 실제 L4 할당·렌더링·학습·운반·결과 회수는 미실행이다. 계정 구매/변경, Drive 사용, 자동 재접속·상시 서버는 포함하지 않는다.
