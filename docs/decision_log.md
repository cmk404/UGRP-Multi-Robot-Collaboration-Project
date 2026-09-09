# UGRP 결정 로그 (canonical)

> 최종 갱신: 2026-09-02 (클라우드 GPU 퇴역·코드 감사 수정·sim-to-real 병목 정리)  
> 역할: **현재 상태·결정·문제·세션 인계**의 기준 문서.  
> 읽는 순서: `README.md` → `ROADMAP.md` → 이 파일.  
> 규칙: 과거 항목을 지우거나 덮어쓰지 않는다. 상태가 바뀌면 **새 항목을 추가**한다.  
> 비밀정보(비밀번호, API key, SSH key 등)는 기록하지 않는다.

---

## 현재 상태 (2026-09-02 스냅샷)

> 08-25 스냅샷은 git 이력(`53f63dc` 이전)에 남아 있다. 아래는 09-02 기준으로 다시 쓴 요약이며, 세부 근거는 본문 09-02 항목.

- **저장소**: Oracle `/home/ubuntu/projects/ugrp`가 canonical Git(master). Mac `~/projects/ugrp`는 Syncthing 양방향 미러(`.git`은 stub, 커밋은 Oracle에서). 09-02에 WIP 스냅샷 `53f63dc` 뒤 감사 수정 커밋. 전체 `tests/` 798개 모듈 실행 기준 **0 실패**.
- **SIM compute**: MuJoCo는 CPU-bound. **Mac M3 워커만 자동 provider**(`UGRP_GPU_PROVIDERS='mac'`), Oracle은 브리지·coworker·failover만. Azure PAYG 구독/VM 없음, Colab·Lightning 자동 경로 차단, azure idle timer disable. `docs/cloud_simulation.md` 09-02 개정. → D-20260902-1.
- **SIM 물리**: `V2_STRUCTURAL_UNCALIBRATED`, `training_ready=false`. `calibration/masterpi/*.jsonl` 측정값 전부 null(계획만 있음). 카메라 K/D만 실측(ugrp1), 나머지 동역학·마운트·그리퍼는 명목/추정. → O6.
- **REAL**: ugrp1만 실동작(`ugrp-real.service`, 8083). ugrp2 ACT LED 무응답(O5), ugrp3 Pi 3B 부팅 미해결. 최신 REAL 실패 원인은 grasp reach(17.5 cm 임시), face 샘플 0, stale pose trust — SIM이 예측하지 못한 REAL-only 실패 다수. → O7, 09-02 병목 항목.
- **harness**: per-robot public-skill queue(09-01), TEAM 공유 채팅, `goal_achieved`/`tool_budget`/`planner_budget`/`protocol_error` 조기 종료(09-01~02). SIM/REAL 동일 스킬 소스. 연구 조건 비교(D1–D3)·실험 로거(D4)는 **미구현**.
- **사양**: O1(공식안 vs Isaac 수정안) **여전히 미승인**. MasterPi 플랫폼 작업은 C단계 자산이며 확정 연구 사양으로 읽지 않는다.
- **팀**: ugrp 톡방 킥오프(2026-08-22). 역할 — docs / masterpi / protocol / eval. 리드 ugrp. 크로스컷·사용자 확인만 main.

---

## 미해결 항목

| ID | 항목 | 막힌 이유 / 다음 |
| --- | --- | --- |
| O1 | 연구 사양 고정 (공식안 vs 수정안) | 사용자·리드 승인 필요. decision_log에 결정 기록 전엔 확정으로 취급 금지 |
| O2 | L1–L3 태스크 명세 값 | 골격 있음 (`docs/l1_l2_l3_task_spec.md`). 값·층 선택은 O1 승인 후 |
| O3 | 메시지 스키마·행동 스킬 확정 | 1차 대조 완료(하드 충돌 없음, 별칭만). 병합·확정은 B3 |
| O4 | 비교군·메트릭·로거 확정 | eval 초안+§4.2 별칭 병기. 병합은 B3, 본구현은 B 이후 |
| O5 | MasterPi 실작업 블로커 | ugrp2: PWR ON·SD 재장착·전원교체 후에도 ACT 무응답. bootfs 확인 대기. `outputs/masterpi_ugrp2_act_led_diag.md`. 연구 가설과 혼선 금지 |
| O6 | 디지털 트윈 동역학 캘리브레이션 캠페인 | (09-02) `calibration/masterpi/` 시험 계획 108+60+24+32건의 측정값이 전부 null. ugrp1 + 오버헤드 ArUco 측정 세션이 필요. **사용자 결정**: 지금 캠페인 vs REAL 경험적 튜닝 계속 |
| O7 | REAL 로봇 수 (1/3) | (09-02) ugrp1만 동작. **사용자 결정**: ugrp2/3 수리·교체 vs 1 REAL + SIM peer로 진행 vs 플랫폼 전환(O1) |
| O8 | REAL 오도메트리/외부 계측 | (09-02) REAL pose는 commanded PWM만, 섀시 오도메트리 없음. SIM spatial memory는 MuJoCo base pose 사용(REAL에 대응물 없음). **사용자 결정**: ArUco/VO 추가 vs ego-frame-only 유지 |

---

## 결정

> 아래는 **승인된 결정만** 채운다. 킥오프 시점에는 비워 둔다.

<!-- 템플릿
### D-YYYYMMDD-N — (제목)

- **결정**:
- **근거**:
- **영향** (문서·코드·실험):
- **승인 상태**: 미승인 | 리드 승인 | 사용자 승인
- **관련**: O?, ROADMAP Phase?
-->

### D-20260902-1 — SIM compute: 클라우드 GPU 전부 퇴역, Mac M3 워커 + Oracle 브리지

- **결정**: Azure A10 / Colab / Lightning을 SIM 자동 provider에서 제거한다. Mac M3 MuJoCo 워커만 자동 복구 대상으로 남기고, Oracle CPU 워커는 수동 fallback으로 둔다.
- **근거**: MuJoCo 3-robot 월드는 CPU-bound(스텝 수백 µs, 렌더는 offscreen 640×480). 클라우드 GPU는 명령/프레임마다 30–120 ms WAN 왕복을 추가했고, 비용·quota·cost-guard·복구 스크립트만 늘렸다. Azure PAYG 구독은 09-02 시점에 이미 존재하지 않았다.
- **영향**: `.env.gpu` provider=`mac`, Azure/Colab allow=0; `ugrp-azure-idle-controller.timer` disable; `docs/cloud_simulation.md` 개정; `scripts/{azure,colab,lightning}_worker_recover.py`·`cloud/lightning/`은 참고용으로만 잔존. `ugrp-sim-gpu-recover.service`는 `mac_worker_recover.py`만 실행.
- **승인 상태**: 사용자 승인 (2026-09-02 세션 지시)
- **관련**: `docs/cloud_simulation.md`, ROADMAP Phase C. 추후 선택지: Mac 전체 로컬 스택(0 WAN hop) — 미결정.

---

## 문제·시도

> 관찰된 문제, 증거, 시도, 결과, 해결 여부.

### P-20260822-1 — ugrp2 ACT LED 무응답

- **관찰**: ugrp2(Pi 4B). 어제 SSH 확인 후, 패키지 설치 중 네트워크 이탈. PWR만 점등, ACT 완전 무응답. boot 파티션은 Mac에서 멀쩡해 보였다는 보고. 슬롯 접촉 의심.
- **증거**: 사용자·Codex 세션 보고(직접 재현 전).
- **시도**:
  - 전원: 빨간 PWR ON. 다른 전원/케이블도 시도.
  - SD 재장착: 수행.
  - ACT: 여전히 무응답.
  - bootfs/`start4.elf`/`kernel8.img` 확인: **대기 중** (사용자 Mac).
  - 카드/보드 교차: 아직 안 함. **재플래시 금지**.
- **결과**: 미해결. 1·2번 실패 → 3번 bootfs 대기.
- **관련**: O5. `outputs/masterpi_ugrp2_act_led_diag.md`. 연구 사양과 무관.

---

## 세션 인계

### 2026-08-22 — ugrp-docs (킥오프)

- **한 일**: `ROADMAP.md`, `docs/decision_log.md` 초안 골격 작성. 현황만 반영, **결정 본문 비움**. L1–L3 명세는 다음.
- **확인함**: README에 공식안/수정안 미승인 구분·다음 구현 순서 존재. ROADMAP/decision_log 파일은 이전에 없었음.
- **확인 안 함**: 수정안의 공식 변경 승인 여부; MasterPi 현재 블로커 상세(masterpi 브리핑 대기).
- **다음 행동 (docs)**: L1–L3 명세 골격(값 비움). protocol/eval 초안과 필드·용어 충돌 여부 점검.
- **다음 행동 (팀)**: masterpi 한줄 브리핑; protocol 범위; eval 초안. 사양 확정은 리드·사용자.

---



### 2026-08-22 — ugrp-docs (L1–L3 골격)

- **한 일**: `docs/l1_l2_l3_task_spec.md` 작성. 공통 필드·L1/L2/L3 칸·공식안/수정안 **분리 표기**. 값은 전부 `_TBD_`.
- **확인함**: README L1–L3 라벨, ROADMAP A3, protocol/eval이 맞출 필드명(`seed`/`episode_id`/`step`, message/action/cost, `failure_reason`).
- **확인 안 함**: 실제 수치·이벤트 스케줄; protocol/eval 초안 파일 본문 대조(아직 범위 선언만).
- **다음 행동 (docs)**: protocol·eval 초안 문서가 생기면 §3 충돌표만 대조. O1 승인 전 값 채우지 않음.



### 2026-08-22 — ugrp-docs (필드 1차 대조 기록)

- **한 일**: L1–L3 §3에 protocol/eval 1차 대조 결과 반영 (하드 충돌 없음, 별칭 병기). O3/O4 미해결 문구 갱신.
- **확인함**: protocol·eval 톡 합의 및 eval §4.2 별칭 표기.
- **다음 행동**: 사양(O1) 승인·B3 전엔 필드 병합하지 않음. masterpi 실작업과 문서 층 분리 유지.

## 기록 규칙 (요약)

1. 새 연구 방향·환경·로봇 수·비교군·프로토콜을 정하면 **결정** 섹션에 추가하고 승인 상태를 명시한다.
2. 추정·약속·예상은 확인 결과로 쓰지 않는다.
3. 브라우저 sandbox·생성 초안의 수치를 실험 결과로 쓰지 않는다.

### 2026-08-22 — ugrp-eval (A5 초안)

- **한 일**: `docs/eval_spec_draft.md` 작성. 비교군(C-*)·메트릭(M-*)·로거 필드 후보만. 수치·구현 없음. 공식안/수정안 분리 표기.
- **확인함**: README §3.3·§7·§9, ROADMAP A5/B4/D*, decision_log O4.
- **확인 안 함**: L1–L3 골격·protocol 스키마 파일(아직 없음)과의 필드명 충돌 — 도착 후 §4 체크.
- **다음 행동 (eval)**: docs/protocol 골격 대기 후 용어 맞춤. 본구현은 B4 승인 후.

### 2026-08-22 — ugrp-masterpi (ugrp2 ACT)

- **한 일**: ugrp2 ACT LED 무응답 이슈 오너. 관찰만 `outputs/masterpi_ugrp2_act_led_diag.md`·본 로그 P-20260822-1에 기록. 최소 진단 순서 정의(재플래시 루프 금지).
- **확인함**: 보고 증상(PWR만, ACT 무응답). 연구 사양과 분리.
- **확인 안 함**: 전원/SD/bootfs/교차 실측.
- **다음 행동**: 사용자에게 **전원 一手**만 요청. 결과 보고 후 다음 단계.

### 2026-08-22 — ugrp-masterpi (ugrp2 ACT · 전원/SD 후)

- **한 일**: 사용자 보고 반영(전원·SD·케이블 후에도 ACT 무응답). outputs/decision_log 관찰 갱신. 재플래시 금지 유지.
- **다음**: bootfs에서 `start4.elf`/`kernel8.img` 존재 여부. 결과에 따라 교차만 제안.

### 2026-08-23 — allowlist LLM 도구 하네스

- **한 일**: `harness/`에 지정 Python만 호출하는 루프를 넣음. 모델은 JSON `tool`/`final`만 냄. 임의 `exec` 없음. 기본은 dry-run. 기본 도구는 `scripts/red_skills.py`의 approach/pick/carry/fetch/track. `tests/test_harness.py` 13개 통과.
- **확인함**: 미등록 도구·여분 인자·파이썬 원문 거절. dry-run에서 handler 미호출.
- **확인 안 함**: Qwen/Gemma 실모델 연결, `--execute`로 로봇 실구동, 카메라 프레임 입력.
- **다음 행동**: 모델 백엔드(mlx-vlm 또는 Ollama)를 붙일지, 지금 도구 목록을 그대로 쓸지 사용자 확인. 연구 사양(O1/B3)으로 승격하지 않음.

### 2026-08-23 — 연구용 스킬 시퀀스 채팅

- **한 일**: 채팅 기본을 도구 재생이 아니라 Ollama 대화로 바꿈 (`harness/talk.py`). `--chat`은 `gemma4:e2b`와 멀티턴 대화. Docker 없음. Open WebUI는 `scripts/serve_open_webui.sh` (uvx, 포트 3000).
- **확인함**: replay talker 히스토리 테스트. Ollama tags에 `gemma4:e2b`.
- **확인 안 함**: Open WebUI 실기동·첫 계정 생성, 공개 배포.
- **다음 행동**: `python3 -m harness --chat` 또는 Open WebUI 스크립트. 도구 호출은 대화가 된 뒤에.

### 2026-08-23 — 장면 조율 동료 창

- **한 일**: 잡담용 Gemma/Open WebUI를 제품 경로에서 뺌. `--chat`은 Qwen3-VL이 장면을 보고 `say`로 맞춘 뒤 allowlist 스킬을 호출. 기본 dry-run. 카메라/사진 입력. `scripts/serve_coworker.sh`.
- **확인함**: protocol `say`+tool, 동료 창 replay 테스트. 연구 사양(O1)으로 승격하지 않음.
- **확인 안 함**: Qwen3-VL 실모델 첫 턴 체감, `--execute` 로봇 실구동.
- **다음 행동**: `scripts/serve_coworker.sh` 로 창을 연 뒤, 장면+일로 조율. 실제로 움직이려면 `--execute`.

### 2026-08-23 — MasterPi 카메라만 사용

- **한 일**: 맥 웹캠(`getUserMedia`) 제거. 동료 창은 SSH로 MasterPi ustreamer(`:8080/snapshot`) JPEG를 가져와 `/api/camera`로 보여 주고, 턴마다 그 장면을 모델에 넣음.
- **확인함**: 로봇에서 snapshot JPEG 수신. 하네스 테스트에 카메라 프록시·턴 시 자동 캡처 추가.
- **확인 안 함**: 브라우저에서 MasterPi 라이브 프리뷰 체감, `--execute` 실구동.
- **다음 행동**: `python3 -m harness --chat` 후 창의 MasterPi 화면을 보고 일로 조율.

### 2026-08-23 — 동료 창 좌우 분할, 도구 파일 하나

- **한 일**: 브라우저를 좌(카메라·모터/팔 마지막 명령값·조정, 실행 가능 Python 도구) / 우(채팅)로 나눔. 카메라는 MasterPi MJPEG 스트림. 보내기 버튼은 검정, 설명 문장은 제거, IME 중복 전송 방지. 실행 도구 목록은 `scripts/robot_actions.py` 하나이며 `--actions`로 바꿔 끼움.
- **확인함**: 하네스 테스트 34개. `/api/camera/stream` MJPEG 수신. `/api/tools`가 robot_actions.py의 approach/pick/carry/fetch/track을 반환.
- **확인 안 함**: 브라우저에서 팔 슬라이더 실구동, Qwen 실모델 턴.
- **다음 행동**: 브라우저를 새로고침한 뒤 왼쪽 화면으로 조율. 도구를 바꾸려면 `scripts/robot_actions.py`를 고치거나 `--actions`로 다른 파일을 지정.

### 2026-08-24 — 동료 창을 목표 루프 에이전트로

- **한 일**: 오른쪽 창을 한 턴 채팅이 아니라 계획→허용 도구 실행→대기→장면 재확인→완료 루프로 바꿈. 모델은 Python을 쓰지 않는다. `plan`/`wait`/`look`은 프로토콜이고, 실행 코드는 `scripts/robot_actions.py`만. 진행은 SSE로 한 줄씩 그린다. 연구 사양(O1)으로 승격하지 않음.
- **확인함**: 하네스 테스트에 plan/wait/look, 실행 후 observe, `/api/turn` 스트림 추가.
- **확인 안 함**: Qwen 실모델이 plan을 먼저 내는지, `--execute`로 실제 집기까지 가는지.
- **다음 행동**: `http://127.0.0.1:8080/` 새로고침 후 목표를 한 문장으로 주기. 실제로 움직이려면 `--execute`와 오른쪽 `실제로 실행`.

### 2026-08-24 — 에이전트 루프 가드

- **한 일**: 도구 직후 장면 저장을 빼고 `wait`/`look` 뒤에만 새 프레임을 쓴다. 계획은 도구 이름(또는 한글 별칭) 순서를 강제한다. 실제 실행은 서버 `--execute`와 창의 체크박스가 둘 다 켜졌을 때만. `max_steps`는 도구 횟수만 센다. 다음 목표에는 실행 로그를 남긴다. 창에 중지 버튼과 한도 초과 표시를 넣었다. 연구 사양으로 올리지 않음.
- **확인함**: 계획 순서 위반 거부, 도구 한도와 wait/look 비계수, 취소, 브라우저 execute 단독 무효, `/api/status`.
- **확인 안 함**: Qwen 실모델이 plan을 도구 이름으로 내는지, `--execute`로 실제 집기까지 가는지.
- **다음 행동**: `http://127.0.0.1:8080/` 새로고침. 실제로 움직이려면 서버 `--execute`와 오른쪽 체크박스.

### 2026-08-24 — 동료 창 기본 모델을 Groq 비전으로

- **한 일**: 키가 있으면 Groq `qwen/qwen3.6-27b` JSON 모드로 장면을 본다. 키는 gitignore된 `.groq_keys` 또는 `GROQ_API_KEY(S)`만 쓴다. 같은 조직의 여러 키는 RPM을 곱하지 않고, 401/429일 때만 다음 키로 넘긴다. 키 없으면 로컬 mlx. 연구 사양으로 올리지 않음.
- **확인함**: 키 로드·한도 시 키 순환 단위 테스트. 실제 키 값은 로그·git에 넣지 않음.
- **확인 안 함**: Groq 실모델이 plan을 도구 이름으로 내는지, Pi Tailscale이 살아 있는지.
- **다음 행동**: `scripts/serve_coworker.sh` 로 창을 연 뒤 목표를 한 문장으로. 카메라/로봇은 Pi가 온라인일 때만.

### 2026-08-24 — 로봇 없으면 대화만

- **한 일**: Enter가 안 나가던 원인은 창 스크립트에서 `loadTools`가 깨진 것이었다. 고쳤다. 장면이 없으면 SSH 스냅샷을 기다리지 않고 Groq와 대화만 한다. 도구/plan/wait/look은 장면이 있을 때만. 연구 사양으로 올리지 않음.
- **확인함**: 스크립트에 `loadTools` 복구, 장면 없는 `/api/turn`은 `final`만, 하네스 테스트.
- **확인 안 함**: Pi가 다시 온라인일 때 도구 루프가 자동으로 돌아오는지.
- **다음 행동**: `http://127.0.0.1:8080/` 새로고침 후 Enter로 대화. 카메라가 살아나면 다시 도구를 쓴다.

### 2026-08-25 — Groq JSON 400 수정

- **한 일**: `qwen/qwen3.6-27b`가 `<think>`를 내면서 Groq `response_format=json_object`가 HTTP 400(`json_validate_failed`)을 냈다. `reasoning_format=hidden`으로 바꾸고 json_object 강제를 제거했다. 연구 사양으로 올리지 않음.
- **확인함**: 대화만 모드에서 `{"final":...}` 정상 수신.
- **확인 안 함**: 장면(이미지) 첨부 시에도 동일하게 안정적인지.
- **다음 행동**: `http://127.0.0.1:8080/` 새로고침 후 다시 보내기.


### 2026-08-27 — Oracle 상시 coworker + Groq vision/재관찰 가드

- **한 일**: 작업 기준을 Oracle `/home/ubuntu/projects/ugrp`로 옮긴 뒤 Mac과 Syncthing 양방향 동기화를 연결했다. Oracle에서 `ugrp-coworker.service`를 user systemd 서비스로 상시 실행하고, Tailscale Serve를 통해 tailnet 내부 HTTPS로 `127.0.0.1:8080`을 프록시한다.
- **Groq 수정**: `qwen/qwen3.6-27b` 요청에 `reasoning_effort="none"`을 추가했다. 기존 `reasoning_format="hidden"`만 사용하면 짧은 completion budget이 hidden reasoning에 소모되어 이미지 요청이 `finish_reason=length`로 끝나고 실질 content가 비는 현상을 확인했다. 수정 후 이미지 입력 실호출 정상 응답 확인.
- **에이전트 가드 수정**: 카메라 `observe`가 있는 상태에서 tool을 실행한 뒤 새 장면을 한 번도 관찰하지 않은 채 `final`을 내면 하네스가 거부한다. `wait` 또는 `look`이 실제 새 observation을 만든 뒤에만 완료 판단을 허용한다.
- **확인함**: `python3 -m unittest -q tests.test_harness` 54개 통과. live Groq synthetic 2-frame 시험에서 `approach` 직후 모델의 성급한 final을 거부하고, `look`으로 두 번째 장면을 본 뒤 final을 수락하는 흐름 확인. Mac에서 Oracle Tailscale Serve HTTPS 페이지 HTTP 200 확인. Oracle talk-only live Groq 턴 정상.
- **현재 서비스**: `ugrp-coworker.service` active, Groq backend, dry-run. Tailscale Serve: `https://instance-20260627-1243.taileb87bd.ts.net/` (tailnet only).
- **현재 블로커**: `ugrp1`은 Tailscale에서 offline(last seen 1d)이라 실제 MasterPi frame과 실제 motor/arm end-to-end는 아직 미검증.
- **다음 행동**: `ugrp1` 전원/네트워크 복귀 후 (1) `/api/status`가 talk_only=false로 전환되는지, (2) 실제 camera frame으로 Groq plan/tool/look 루프, (3) 서버 `--execute` + UI 실행 체크 이중 가드로 실제 `approach` 1회부터 검증.

### 2026-08-27 — Synthetic 장기 agent benchmark

- **목적**: 실제 로봇 없이 synthetic camera frames + live Groq로 장기 목표 수행/실패 복구/환경 변화 대응을 검증.
- **baseline**: 첫 pick 실패를 모델이 시각적으로 알아채고 재시도하려 했지만 rigid pending plan이 `carry`를 강제. wait/look 중복과 전체 trace 재전송 때문에 model call 26, tool 6, TPM wait 5에도 완료 못 함.
- **loop 개선**: post-tool auto-observe, image-once, fresh observation 기반 adaptive replan, compact long context, auto-observe protocol prompt를 추가. regression 58 tests pass.
- **최종 정확성 테스트**: 효율은 model call 5/tool 3/rate wait 0으로 크게 개선됐지만, 실패한 pick 뒤에도 carry를 하고 `held=false` 상태에서 성공 final을 내는 false success 발생. 따라서 장기 agent 판정은 실패.
- **원인 분리**: 같은 실패 frame을 별도 state-verifier prompt로 물으면 모델은 `held=false`, `where=floor`를 정확히 판정. vision 자체보다 planner와 verifier가 분리되지 않은 agent architecture가 핵심 문제.
- **환경 변화**: 접근 후 빨간 target이 사라지고 blue decoy만 남는 테스트에서는 pick 없이 중단하여 통과.
- **다음 행동**: vision call 한 번에서 structured scene state + action을 함께 받되, harness가 `held=true` 같은 postcondition을 확인해야 carry/final을 허용하도록 state/evidence gate 추가. 상세: `docs/benchmarks/agent_long_horizon_20260827.md`.

### 2026-08-27 — Live Groq long-agent synthetic vision benchmark

- Current web-agent path (`auto_observe=True`) was tested against five stateful synthetic camera worlds. Each next frame depended on the tool actually selected, so this was not a fixed image slideshow.
- Results and traces: `docs/benchmarks/long_agent_vision_2026-08-27.md` and `.results.json`; reusable runner: `scripts/benchmarks/long_agent_vision.py`.
- Agent recovered from off-center pick failure and from two forced pick failures, preserved red-vs-blue target identity, and correctly abstained when no target existed.
- Hard slip recovery exposed a weakness: the model recognized that arm geometry needed adjustment but repeatedly issued `pick` instead of `track`, then safely stopped after repeated failures rather than claiming false success.
- No unknown-tool hallucinations occurred in the five auto-observe cases.

### 2026-08-30 — Generic REAL object-colour manipulation API (handoff)

- **Implemented**: REAL public tools now use `search`/`track`/`approach`/`pick` with runtime `target_color` and `place` with runtime `target_color` plus `destination_color`. `place_on_blue` and `place_on_yellow` remain compatibility aliases for saved/operator plans.
- **State and safety**: planner-visible state now separately records `target.selected_color`, `grasp.held_object_color`, and `task.destination_color`. A placement follow-up while grasp is `PROBABLE_HELD` or `HELD` compiles directly to one guarded `place` call; it does not restart search/track/approach/pick. Mismatched held/source identity and same-colour destinations are rejected before dispatch.
- **Detector and causal handoffs**: red keeps its calibrated LAB+HSV detector. Blue/yellow use `detect_color_blob` through the new selected-target adapter. Precision-pick and carry handoffs bind the selected colour so a plan/evidence token cannot be reused for another object identity. Existing red handoffs without the field remain interpreted as red for compatibility.
- **Verification**: focused unit tests cover generic blue planning, Korean pronoun follow-up placement, direct-place execution, held/destination state separation, existing REAL precondition gates, handoff guards, and compatibility aliases. `py_compile` also passed. This is code-level verification only; no REAL endpoint or hardware was actuated in this change.
- Practical bottleneck observed: Groq qwen3.6 vision requests consumed ~2.2k prompt tokens per visual decision under an 8k TPM organization limit, causing repeated 429/backoff during long traces.

### 2026-08-27 — synthetic long-agent benchmark + verifier/bounded tools

- 실제 UI 조건(`auto_observe=True`)과 production Groq를 사용해 stateful synthetic camera world 5개를 테스트했다. 최종 결과 5/5 기대 동작, unknown tool 0회.
- 복구 case: off-center는 `track→pick`, grasp 후 slip은 fresh frame에서 감지해 `track→pick`, pick 2회 강제 실패 case는 3번째 pick에서 성공.
- actor의 stochastic false-success를 실제 재현했다. 독립 visual verifier가 최신 이미지만 보고 성공 final을 검증하도록 추가했고, 강제 false final test 및 double-failure benchmark에서 잘못된 완료를 차단했다.
- Groq 8k TPM에서 장기 turn이 429로 죽지 않도록 `Retry-After` 기반 production retry와 256-token action budget을 추가했다.
- 실제 Python action contract를 점검해 blocking semantics를 수정했다: agent `carry` 제거(`pick`이 이미 carry pose까지 들어 올림), `fetch --no-hold`, `track --seconds 1.5` 자동 적용.
- 관련 regression: `tests.test_harness + tests.test_track_red_block` 71 tests 통과.
- 상세: `docs/agent_benchmark_2026-08-27.md`, 결과: `outputs/agent_benchmark_2026-08-27.json`.

### 2026-08-27 — Cloud 3D simulation architecture

- Chose Lightning AI Studio as the primary live MuJoCo host; Oracle remains the Groq/UI/log orchestrator.
- Kaggle is retained as a secondary free batch RL/regression backend rather than the live serving path.
- Isaac Sim is deferred to a separate higher-end RT-GPU compatibility test; it is not assumed to work on free T4-class hardware.
- Added a MasterPi-like MuJoCo world, authenticated cloud API, Oracle remote proxy and Lightning deployment script.
- Physical coworker (:8080) and simulation coworker (:8082) remain isolated.
- Simulation browser UI is exposed tailnet-only on HTTPS :8444.
- Local exact-package smoke test verified authenticated 640x480 render and `approach -> pick -> carry` ending with `held=true`, `stable=true`.
- Existing harness tests remain green: 64 tests, 12.619 s.
- Remaining external prerequisite: Lightning account authentication before the Studio can actually be created.

### 2026-08-27 — PPO grasp skill + authoritative completion + reliable bridge delivery (historical; superseded 2026-08-28)

- **PPO skill**: added `grasp_rl`. The trained `outputs/rl/grasp_ppo_v2.zip` controls the live MuJoCo arm/gripper through the same 21-D observation / 7-D continuous action interface used during training. No object pose attach/teleport is used.
- **Runtime isolation**: loading Torch and the MuJoCo renderer in the same Oracle ARM process reproducibly caused SIGSEGV. PPO inference was therefore isolated into `ugrp-ppo-policy.service` on `127.0.0.1:8093`; the MuJoCo worker sends observations to `/predict` and receives actions. Both services are persistent systemd user services.
- **Completion facts**: `scripts/sim_actions.py` now returns `task_complete`, `authoritative_completion`, block height, bilateral contact, stability and finger normal forces. Strict completion requires `stable && bilateral_contact && lifted && z > 0.16m`.
- **Do not undo success**: after strict completion, disruptive scripted `approach/track/pick/fetch` actions are refused; `carry` is a no-op hold. The harness treats authoritative physical telemetry as sufficient final verification, preserves a fresh local frame for UI/history, but does not spend another VLM image call or independent visual verifier just to reconfirm an already verified grasp.
- **Bridge reliability**: `/worker/next` now retains an `inflight` command until `/worker/result` arrives. A broken long-poll response therefore redelivers the same command instead of losing it. The worker caches the last command result so a redelivery resends the result rather than executing robot motion twice. Five immediate post-restart `reset` trials completed 5/5 with zero 504s.
- **Integrated validation**: harness selected `grasp_rl` once, PPO reached strict completion in 15 policy steps, red block z=0.578m, bilateral contact=true, stable=true, contact forces L=19.639N/R=20.341N. The next model turn received no image (`[initial_image, None]`), visual verifier calls=0, and the loop stopped with final while the physical state remained stable.
- **Regression**: `tests.test_harness + tests.test_sim_bridge` = 67 tests OK; `tests.test_grasp_physics` = 4 tests OK. All four live services (`ugrp-sim-bridge`, `ugrp-sim-worker`, `ugrp-ppo-policy`, `ugrp-sim-coworker`) are active.
- **Current external limitation**: Groq text calls are healthy, but Groq vision is currently returning 429 under the shared 8k TPM bucket. The integrated control path is verified provider-independently; a true live Groq-vision run still depends on vision TPM becoming available. Successful future runs should consume only the initial vision decision plus a cheap text final because authoritative completion suppresses redundant vision calls.


### 2026-08-28 — WorldState + Task Executive production path

- **Architecture change**: active coworker path is no longer intentionally LLM-centric. Added `harness/state.py` (`WorldState`, temporal `StateEstimator`), `harness/perception.py` (robot-RGB red-target detector), and `harness/executive.py` (skill preconditions/recovery). The planner receives an explicit machine-readable `world_state` every decision. Known sensor facts live there rather than in chat history.
- **Temporal perception**: target visibility, center, apparent range and area are accumulated with streak counters and EMA. The initial robot-camera JPEG is processed before the first planner call, and post-action observations update the estimator again. This uses RGB pixels only, not MuJoCo object coordinates.
- **Skill contracts**: active simulation tools expose machine-readable `CONTRACTS` with preconditions, expected postconditions and recovery mappings. `/api/tools` exposes these contracts. `pick` requires visible/centered/PREGRASP when those facts are known.
- **Task executive**: deterministic precondition failures are handled before dispatch. Known recoveries (`search`, `track`, `approach`) can execute without another LLM decision and are emitted as `executive_recovery:*` events. Unknown facts remain UNKNOWN rather than being guessed.
- **Result semantics**: actor-facing simulation results now separate `command_status`, `execution_status`, and `outcome_status`. `pick`/`carry` motor completion returns `outcome_status=UNKNOWN`; it is no longer equivalent to physical task success. Structured `failure_code`, `required_state`, and `recommended_recovery` replace natural-language-only recovery. Legacy `ok` remains only for transport/backward compatibility.
- **Ground-truth boundary**: `scripts/sim_actions.py` continues to strip MuJoCo coordinates, contact forces, block height and evaluator completion. WorldState contains only information intended to be obtainable on the real MasterPi.
- **Runtime graph correction**: active actor tools are `search/track/approach/pick/carry`. `grasp_rl` is no longer enabled by the bridge unless `UGRP_ENABLE_LEGACY_GRASP_RL=1`. `ugrp-ppo-policy.service` is disabled/inactive and removed from the current simulation worker's required dependencies. The 2026-08-27 PPO/authoritative-completion entry is historical, not the current production graph.
- **Verifier**: same-model visual verifier remains as a fallback for ambiguous final claims; it is no longer treated as an independent sensor. Future completion should preferentially use transferable WorldState goal predicates once grasp sensing/visual-hold evidence is reliable.
- **API/UI**: `/api/status` now exposes the current transferable `world_state` and architecture version. CCTV observer views remain user-only; only the robot camera feeds perception/planning.
- **Tests**: added `tests/test_state_executive.py` for temporal estimation, motor-vs-outcome separation, contracts, RGB perception and deterministic recovery chaining.
- **Remaining**: reliable camera-only held-object evidence is still unresolved because the current eye-in-hand camera may lose the cube after carry. Do not promote hidden MuJoCo contact truth into WorldState to solve this. Retune camera/visual hold detection or add a real transferable sensor abstraction.

#### 2026-08-28 validation addendum

- **Live deterministic recovery**: from a reset scene where the target was not initially visible, a replay planner issued only `pick`. The Task Executive executed `search -> approach -> pick` without another planner tool decision. Total wall time was 20.54 s, with no precondition/recovery error. The resulting actor-visible state correctly remained `last_action.execution_status=COMPLETED`, `last_action.outcome_status=UNKNOWN`, `grasp.state=UNKNOWN`; motor completion did not become a false grasp success.
- **Rendering/control isolation**: adding four CCTV renders to every intermediate frame had stretched `search` beyond the 35 s bridge timeout. Worker rendering is now rate-bounded: robot-camera intermediate frames are throttled and CCTV views update one-at-a-time round-robin, with all views synchronized at action boundaries. A fresh `reset -> search` completed in 7.82 s and returned camera evidence `pixels=4471`, `area_ratio=0.01455`.
- **JPEG perception hardening**: the harness JPEG detector previously accepted a 122-pixel compression/artifact cluster as a target. It now requires target-scale evidence (`max(18, 0.0008 * frame_area)`); the same reset frame is correctly `visible=false` while the post-search target is well above threshold. Invisible observations clear current cx/cy/area/range instead of leaving stale PREGRASP state.
- **Goal predicate gate**: on contract-enabled structured execution paths, GRASP/LIFT/PLACE success claims are blocked unless transferable WorldState says the goal is achieved. When outcome remains unknown, the agent may stop only with failure/uncertainty rather than claiming success. Legacy/simple runners and dry-run behavior remain compatible.
- **Physical adapter parity**: `scripts/robot_actions.py` now exposes contracts and the same `command_status / execution_status / outcome_status` schema. A bounded `search` action reuses the existing arm-camera scan in `track.py`. Physical skill exit code 0 means execution completed only; physical outcome remains UNKNOWN until the camera/state estimator verifies it.
- **Regression**: `tests.test_harness + tests.test_sim_bridge + tests.test_state_executive` = 74 tests OK.
- **Continuous sensor state**: camera frames now update the persistent StateEstimator independently of planner turns (throttled in `ChatState.remember_frame`). WorldState therefore exists before the user asks for an action; the LLM reads state rather than creating it. Simulation startup now waits for the worker's first bridge snapshot so simultaneous restarts do not leave the coworker in a false talk-only state.

### 2026-08-28 — Eye-in-hand self-mask + simulation speed controls

- **Self/gripper false-positive fix**: simulation skill RGB detection now uses the same minimum target evidence as the harness (`max(18, 0.0008 * frame_area)`) instead of the old 0.00004 threshold. A reset-frame `track` now returns `TARGET_NOT_VISIBLE` with `camera_red.visible=false, pixels=0` rather than treating self/reflection pixels as the red block.
- **Planner self-mask**: the current simulation eye-in-hand camera has a persistent gripper strip around normalized y=0.785..0.865. The original user-facing frame and deterministic perception remain untouched, but the VLM planning image masks that calibrated strip. The system prompt also explicitly states that orange/black fingers/wrist/arm are SELF and that `world_state.target.visible=false` overrides a visual guess that self geometry is the target. This mask is simulation-camera-specific; a physical camera should be separately calibrated.
- **Speed UI/API**: simulation coworker exposes 1x/2x/3x buttons backed by `POST /api/sim/speed` and `/tmp/ugrp_sim_speed`. Physical coworker hides the controls.
- **Speed implementation**: accelerated modes preserve approximately the same MuJoCo physics time while reducing control/render interpolation. 2x renders at 480x360 and uses 4.5 cm visual-approach checkpoints; 3x renders at 320x240 and uses 6 cm checkpoints. Search scans 360 degrees at 20/30/40-degree visual checkpoints for 1x/2x/3x. CCTV refresh is round-robin rather than four renders per command cycle.
- **Measured wall-clock**: same seeded live path: search 1x 3.92 s vs 3x 2.51 s; approach 1x 10.66 s vs 3x 4.21 s. 2x measured search 2.71 s and approach 6.34 s. Speed labels are simulation modes, not promises of exact wall-clock ratios because camera rendering/HTTP overhead is fixed.
- **Physics sample check**: seeds [1,3,5,7,11] produced strict hidden-evaluator grasp/lift success 3/5 at both 1x and 3x; accelerated mode did not reduce this small-sample success rate.
- **Regression**: `tests.test_harness + tests.test_sim_bridge + tests.test_state_executive` = 74 tests OK.

## 2026-08-28 — Platform/demo: object-centric spatial memory + head-first search
- Classification: **platform/demo improvement**, not an approved change to the official research specification.
- Observed issue: SIM `search` rotated the chassis for a 360-degree visual sweep even though the eye-in-hand camera is downstream of `arm_yaw`; repeated blue placement also failed intermittently because a carried block could slip during a large target-facing chassis rotation.
- Implemented an object-centric spatial memory inferred only from robot-camera RGB, calibrated camera pose/FK, ground-plane ray intersection, and robot odometry. No target-body simulator coordinates are used to create the memory.
- A single RGB render now segments red/blue/yellow together and can update all remembered object landmarks. Memory preserves position when an object leaves view, reports relative bearing/distance/confidence, and rejects low-quality near-horizon/extreme-edge position updates.
- Search policy is now head-first: current view -> remembered bearing -> arm-yaw sweep -> chassis yaw only as a rear-hemisphere fallback. `track` also uses camera/arm yaw; `approach` reconciles head bearing with chassis heading once before translation.
- Head servo interpolation was changed to start from measured joint positions rather than stale actuator targets, fixing a target-loss failure after search.
- Placement uses the remembered target landmark and reduced carried-turn angular speed/acceleration.
- Measured local SIM results at 3x: blue placement seeds 600–609 = 10/10, yellow placement seeds 610–619 = 10/10. In these blue episodes, `search` chassis-yaw change was approximately 0 degrees when the target was within head range. Off-axis red targets near +/-40 degrees were both reacquired with head yaw and effectively zero chassis rotation.
- Regression: spatial/physics/bridge tests 14/14 passed; harness/Groq tests 13/13 passed.
- Unverified at time of this log entry: transfer of the spatial-memory estimator to the physical MasterPi camera/servo calibration. Real hardware will need measured camera intrinsics/extrinsics and odometry error characterization.
- Follow-up: verified self-actions now update spatial memory too. Successful `carry` marks red as `HELD` at gripper-FK position; successful placement marks it `ON_BLUE`/`ON_YELLOW` at the remembered target location. This avoids retaining a stale pre-pick landmark after the robot itself moves the object.


## 2026-08-28 — Platform/demo: user-visible semantic radar + obstacle memory
- Classification: **platform/demo improvement**, not an approved change to the official research specification.
- Added a `semantic_map` state contract beside object-centric `spatial_memory`. In SIM, obstacle cells are inferred from sparse `robot_cam` metric depth samples transformed by calibrated camera pose/FK and robot odometry; the map is not populated from MuJoCo body IDs or obstacle poses.
- Obstacle memory is a bounded sparse 2.5D occupancy representation (8 cm cells) with estimated height, confidence, observation age, and source=`robot_camera_depth`. Floor, near-camera/self geometry, and tiny block-height surfaces are filtered.
- Added two configured semantic destination landmarks, `BLUE DELIVERY` and `YELLOW DELIVERY`, to the map. These are configured workcell zones, not visually detected objects and not simulator truth promoted into actor state.
- Added `scan_world`: a high survey pose sweeps only the eye-in-hand `arm_yaw` through five headings, forces semantic/depth observations at each view, and returns the head forward. The chassis remains fixed unless another skill explicitly requires chassis motion.
- The SIM coworker UI now has a Three.js `Semantic Radar` panel with range rings, animated camera sweep/FOV, remembered red/blue/yellow objects, orange obstacle pillars, confidence/live state, and the two delivery zones.
- Live local-worker validation through the production bridge: `scan_world` returned `objects=['blue', 'red', 'yellow']`, `obstacles=100`, `chassis_delta_deg=0.00`; both bridge health and `/api/sim/state` exposed 100 obstacle cells and 2 zones. Tailnet UI returned HTTP 200 and served the new `semantic-radar` panel.
- Regression after the final survey-head change: semantic/spatial/color-search/physics/bridge suite 17/17 passed; state-executive/Groq suite 13/13 passed; both external and inline browser JavaScript syntax checks passed. Required SIM services were active.
- Transfer boundary: this verifies the semantic-map architecture in MuJoCo only. The physical MasterPi does **not** yet have a verified metric obstacle provider; REAL needs calibrated depth/stereo/sonar or another transferable range source before obstacle mapping/avoidance can be claimed on hardware.

## 2026-08-29 — Platform/demo: Colab T4 self-healing worker + single-remote bridge
- Classification: **platform/demo reliability improvement**, not an approved change to the official research specification and not a research result.
- Added `scripts/colab_worker_recover.py` and `ugrp-sim-colab-recover.service`. Recovery packages the current Oracle runtime files, uploads the bridge token without printing it, restores SIM speed, installs the required Colab packages idempotently, and launches the persistent WebSocket worker. If the named Colab runtime no longer exists, it creates a new T4 session first.
- Extended `scripts/sim_worker_failover.py`: after 15 s without a remote GPU it starts the Oracle CPU worker; after 30 s it requests Colab self-recovery. Failed recovery attempts are rate-limited for 5 minutes, while a successful GPU restore clears the prior recovery episode and stops the CPU fallback.
- Fault injection (existing runtime): deliberately killed `run_mujoco_ws_worker.py` inside Colab. Observed CPU fallback start after the remote outage, Colab recovery request at the 30 s threshold, worker redeployment into the existing T4, remote WebSocket restoration, then automatic CPU fallback shutdown.
- Fault injection (runtime absent): `colab sessions` reported no active sessions. The recovery service logged `creating T4 session ugrp-gpu` and then `session ready (created=True)`, verifying the no-runtime creation branch. A later deliberate token/bridge test interrupted that first recovery process after session creation; a following recovery reused the new session and restored the remote worker. This interruption was caused by the test procedure, not by the recovery script itself.
- Hardened `sim/bridge.py` so exactly one remote WebSocket worker is authoritative. A newer authenticated GPU worker increments a generation, replaces/closes an older ghost connection, and stale generations cannot send commands/results/state. This prevents duplicate simulation execution when Colab leaves a short-lived socket after a stopped runtime.
- Rotated the simulation bridge token during the fault test; the token value was not written to logs or this document. Local CPU and Colab deployment were synchronized to the new token.
- Added compute visibility to the Semantic Radar. `/api/sim/state` now exposes remote-worker status, and the browser shows `COLAB T4 GPU` or `ORACLE CPU FALLBACK` in the radar status line. Headless browser validation showed `Semantic Radar · COLAB T4 GPU · ...` against the live page.
- Live canonical validation after recovery: active named T4 session, bridge `remote_ws_connected=true`, `remote_ws_count=1`, CPU fallback inactive. `reset` and `scan_world` both returned `transport=websocket`; `scan_world` produced 105 obstacle cells. The production `/api/turn` command `빨간 블럭을 파란 블럭 위에 올려줘` completed on the remote path with semantic relation `ON_BLUE`, XY stack error 8.3 mm, and vertical delta 49.8 mm. Public SIM UI returned HTTP 200.
- Regression after the final changes: `python -m unittest discover -s tests -v` in `.venv-sim` ran **193 tests, all passed**. New recovery/bridge tests cover bundle secret exclusion, session parsing, recovery thresholds/cooldown, stale state fencing, and remote-generation replacement.


### 2026-08-29 — SIM failover authority/state-epoch correction

- **Observed production fault**: with Colab unavailable, the Oracle CPU fallback itself completed the canonical `red -> blue` task correctly, but a direct simulator reset left the coworker `WorldState` at the previous `PLACED` episode. The physical simulation and planner-side state could therefore disagree after reset/replacement.
- **Root cause in self-healing handoff**: the local HTTP CPU worker and newly reconnected GPU WebSocket worker both consumed the same bridge `queue/inflight`. Because an inflight command is intentionally redelivered until a result arrives, there was a race window where two independent MuJoCo worlds could execute the same command before the failover poll stopped CPU. The two workers also have no physical-world migration, and their `state_seq=time.time_ns()` values must not be ordered across Oracle/Colab host clocks.
- **Bridge correction**: a newly connected remote GPU is now parked (`remote_authoritative=false`) and cannot consume commands or publish actor state. The failover controller grants authority only at an idle command boundary, after stopping the coworker and CPU owner. Authority changes clear the bridge publication epoch (`state_seq`, cached state/frames), then the remote worker is explicitly asked for a fresh sync frame.
- **Fallback correction**: when a fresh CPU fallback episode starts, the coworker is restarted at the same boundary so persistent `StateEstimator/WorldState` cannot retain the prior GPU episode. A stale CPU process cannot claim new commands while a remote worker is authoritative.
- **Current external condition**: Colab currently has no active session and T4 allocation attempts return `Service Unavailable`; therefore the corrected GPU handoff could not be revalidated end-to-end against a live Colab runtime in this check. Oracle CPU fallback remains active and usable.
- **Live validation after correction**: production `/api/turn` `빨간 블럭을 파란 블럭 위에 올려줘` completed on Oracle CPU fallback in 19.87 s through `map_blue -> search -> track -> approach -> pick -> carry -> place_on_blue`. Final relation was `ON_BLUE`, XY stack error 6.5 mm, vertical delta 49.9 mm, queue empty and no inflight command.
- **Regression**: targeted bridge/recovery suite = 11/11 passed, including new parked-GPU/explicit-authority tests. Full `.venv-sim` suite = **199 tests, all passed**. The live simulator was reset afterward and the coworker `WorldState` was returned to a clean initial episode; bridge/worker/coworker/failover services were all active.
- **Known limitation**: authority handoff is now coherent but not state migration. A CPU<->GPU switch starts a fresh simulation episode rather than transferring MuJoCo qpos/qvel/contact state. Seamless mid-episode compute migration would require an explicit world snapshot/restore protocol and is not claimed here.

### 2026-08-29 — Provider-neutral GPU recovery; Colab demoted from automatic backend

- **Root policy finding**: free managed Google Colab is not a dependable substrate for the persistent UGRP WebSocket simulator. Google documents dynamic/non-guaranteed GPU availability and explicitly restricts remote-control/UI-bypass/distributed-worker patterns on free managed runtimes. Repeated T4 creation requests therefore cannot make the current architecture reliable.
- **Recovery policy**: automatic provider order now defaults to `lightning` only. Colab remains available for deliberate experiments only when both `UGRP_GPU_PROVIDERS` includes `colab` and `UGRP_ALLOW_COLAB_WORKER=1` are set. Oracle CPU remains the always-available fallback.
- **Lightning production worker**: added `scripts/lightning_worker_recover.py`, which deploys the current `MasterPiPhysicsWorld` + `run_mujoco_ws_worker.py` transport rather than the historical `cloud/lightning` HTTP simulator. Package smoke test confirmed no legacy cloud files, correct T4/provider metadata, and valid launcher syntax.
- **Generic recovery service**: added `scripts/gpu_worker_recover.py` and `ugrp-sim-gpu-recover.service`. Failover probes provider availability at most once/minute, starts remote recovery only when credentials are actually configured, and uses a 30-minute failed-recovery cooldown instead of repeatedly hammering allocation APIs.
- **Provider identity**: remote WS auth now includes provider/machine metadata. Bridge health and `/api/sim/state` expose it, and Semantic Radar shows a remote backend only when that remote is authoritative. This removes the prior hard-coded `COLAB T4 GPU` label and supports `LIGHTNING T4`, `COLAB T4`, etc.
- **Current external blocker**: Lightning CLI/SDK 2026.8.26 and T4/L4 machine definitions are present on Oracle, but `lightning auth whoami` reports no credentials. Actual Lightning T4 allocation cannot be truthfully claimed until one-time user account authentication is completed.
- **Validation**: targeted provider/bridge/harness suite passed; full `.venv-sim` regression ran **205 tests, all passed**. Production canonical placement also completed on Oracle CPU after the provider refactor, and the live world was reset afterward. Core services remain active.


### 2026-08-29 — Browser-controlled deterministic SIM seed reset

- Added a SIM-only browser control for entering an integer seed, resetting directly to that seed, or generating a random seed. The active seed is read back from the simulator and shown in the control.
- Seed now travels through `POST /api/sim/reset` -> bridge `/command` -> the authoritative CPU/GPU worker. This works identically for Oracle CPU fallback and a future Lightning GPU authority.
- `MasterPiPhysicsWorld` stores the active seed and a reset with the same seed reconstructs the same initial randomized block layout. Seed range is bounded to 0..2147483647.
- A browser seed reset is also an episode boundary for the coworker: temporal WorldState and prior chat/action history are cleared so state from the previous episode cannot contaminate the new seeded trial.

### 2026-08-29 — Platform/demo: REAL MasterPi spatial-memory transfer scaffolding

- Classification: **platform/demo transfer work**, not an approved change to the official research specification and not yet a real-hardware accuracy result.
- Physical RGB perception now has a reusable REAL geometry layer (`harness/real_geometry.py`) derived from the previously tuned MasterPi pickup controller rather than from MuJoCo. It preserves the physical constants (9.30/6.50/6.20 cm arm links, 7.0 cm camera link, 2.5 cm camera-z offset, PWM deviations, 48-degree VFOV) and projects the lower-center of a detected block bbox onto the floor.
- REAL coordinates use the explicit convention `+x forward, +y left, +z up`. Metric landmarks are labeled `reference_frame=robot_base_at_observation`; they are not called persistent world coordinates because calibrated chassis odometry is not yet available.
- Commanded servo pose is read non-destructively from `/tmp/ugrp-masterpi-pose.json`. Metric projection is promoted only after the same commanded pose is observed repeatedly and has settled for at least 0.18 s. Unstable observations remain image-relative rather than publishing dubious XYZ.
- Formula transfer verification: 1000 randomized pose/pixel cases were compared against the previous physical state-machine equations. 209 cases were geometrically valid in both implementations; all 209 matched exactly, with max XY difference 0.0 m and max FK difference 0.0 in native units. This proves code parity, **not** tape-measure calibration accuracy on the current physical scene.
- `WorldState` now carries a REAL `semantic_map`. Metric blue/yellow landmarks generate 10 cm `BLUE STACK TARGET` / `YELLOW STACK TARGET` regions. REAL maps explicitly report `persistent_world_map=false` and `odometry_available=false` until a calibrated odometry provider is installed.
- REAL Semantic Radar support is prepared: REAL mode polls `/real/api/status`, keeps the radar visible, and does not use the SIM hard-coded fallback delivery zones. When no real depth backend is configured it displays `DEPTH 준비 전` rather than implying zero obstacles.
- Added read-only `observe_scene`: it reads camera + commanded pose and can return color detections and stable metric estimates without moving any actuator. This is intentionally separate from active head scanning while another physical controller owns the robot.
- Added an odometry provider contract (`harness/real_odometry.py`). No wheel-command dead reckoning is promoted to world coordinates because it is not calibrated; the current provider status is explicitly unavailable.
- Prepared, but did **not** activate, a REAL obstacle-depth path based on Depth Anything V2 Metric Hypersim Small. The lazy backend, depth-to-base projection, coarse occupancy-cell conversion, and Colab bootstrap script exist. Depth-derived obstacle cells must pass physical scale validation before `validated=true` or live obstacle mapping is claimed.
- Concurrency boundary: actuator commands use an exclusive flock-based lease on the Pi, but at the user's request this setup phase does not terminate or replace any currently competing physical control process. The changes in this entry were prepared on Oracle only; no servo/chassis command or Pi service restart is required to build/test them.
- Final regression for this REAL setup batch: `.venv-sim/bin/python -m unittest discover -s tests -q` ran **233 tests in 63.378 s, all passed**. The run used local/fake transports; no physical actuator command was sent. Added a guarded `scripts/activate_real_spatial_stack.sh` that refuses activation while the Pi actuator flock is owned and never kills/stops the competing controller.


### 2026-08-29 — Browser SIM visual source-of-truth fixed

- Root cause of the visibly inconsistent SIM: the browser observer scene was a separately hand-authored Three.js workcell, while control/perception used MuJoCo. The two had drifted: delivery pad signs were flipped, wall geometry differed, and rack/crate geometry was missing or simplified.
- `sim/masterpi_scene.xml` is now the canonical visual workcell source loaded by both MuJoCo (`sim/mujoco_world.py`) and the browser (`/sim-scene.xml`). Browser world geoms are generated from direct MuJoCo `worldbody` geoms instead of duplicated coordinates.
- The browser robot first-person tile now shows the actual MuJoCo RGB stream used by the agent rather than a synthetic Three.js POV. WebGL remains only for low-cost observer/CCTV views and the semantic radar.
- Lightning/Colab worker bundles now include the canonical XML so remote GPU physics uses the same source.

### 2026-08-29 — REAL camera low-latency pipeline
- Root cause measurement: Pi-local ustreamer `/snapshot` took ~0.9–1.1 ms, while a fresh Oracle->Pi SSH snapshot took median **2.458 s**. The old REAL `/api/camera` inherited that full handshake cost, and REAL RGB spatial-memory pose polling also opened a fresh SSH approximately every 0.3 s inside camera-frame processing, periodically stalling the stream and allowing stale MJPEG bytes to queue.
- Reworked physical camera transport to one persistent SSH port-forward + one upstream MJPEG reader. Oracle now keeps only the newest decoded JPEG; browser clients and actor snapshots read this latest-frame cache. Slow clients skip stale intermediate frames rather than replaying a backlog.
- Moved REAL colour/FK perception off the camera receive path. Commanded-pose telemetry now uses one persistent read-only SSH process that only cats `/tmp/ugrp-masterpi-pose.json`; it never sends actuator commands. Camera receive, pose telemetry, and perception are independent.
- Optimized the SIM coworker's `/real/...` streaming proxy to use `HTTPResponse.read1()` so it relays currently available MJPEG bytes rather than waiting to fill a fixed read buffer.
- Live before/after: REAL `/api/camera` ~2.45 s -> **0.8–1.0 ms**; internal `/real/api/camera` ~2.8–3.8 s -> **1.8–3.6 ms**. Direct REAL MJPEG first frame **16.8 ms / 19.3 fps**. Combined UI proxy first frame improved from **103 ms / 16.3 fps** to **61.5 ms / 18.8 fps**. Public HTTPS test measured **42.3 ms first frame**, with live latest-frame age 47.5 ms in that sample.
- Deployment boundary: actuator lock was checked and free before restarting Oracle `ugrp-real.service`; Pi-side ustreamer/controller services were not restarted or killed. ustreamer PID remained unchanged. `ugrp-sim-coworker.service` was restarted only to load the proxy optimization.
- Regression: full `.venv-sim` suite ran **241 tests in 81.097 s, all passed**.
- Correction: the assistant did not issue any Pi-side ustreamer restart/kill command, but during later concurrent validation the observed ustreamer PID changed from 11910 to 19438, apparently due to another process/session. Therefore the stronger statement that the PID remained unchanged across the full interval is withdrawn; what is verified is that this camera-latency fix itself only restarted Oracle-side services and sent no Pi actuator or camera-service restart command.


### 2026-08-29 — Production SIM changed to GPU-only execution

- User explicitly rejected Oracle CPU simulation fallback. Production now treats Oracle as bridge/UI/orchestration only; `ugrp-sim-worker.service` is not a valid production compute path.
- The bridge has a `UGRP_SIM_GPU_ONLY=1` gate: local HTTP workers cannot claim commands or publish frames/results, and user commands fail immediately with `GPU_OFFLINE` when no authoritative remote GPU exists rather than waiting on a hidden CPU fallback.
- GPU watchdog polling is 1 s, configured-provider recovery begins after about 3 s, transient recovery failures retry after 60 s, and provider availability is reprobed every 15 s.
- Remote workers now identify their process instance, use WebSocket protocol ping/pong timeouts, reconnect after 1 s, and publish the actual MuJoCo robot RGB during motion at a bounded ~10 fps target on GPU.
- Tailscale Funnel on public TLS TCP :8443 remains the ingress to the authenticated bridge WebSocket :8093.
- Current blocker at the time of this change: Lightning SDK/CLI is installed but the Oracle host has no Lightning account credentials, so the correct production state is `GPU OFFLINE`, not Oracle CPU fallback.

- Follow-up FPS correction: the bridge MJPEG loop had a separate hard 0.20 s sleep (5 FPS ceiling). Production default is now 0.05 s (20 FPS ceiling), so the browser no longer discards/duplicates GPU camera updates behind an artificial 5 FPS cap. Actual first-person FPS remains bounded by GPU MuJoCo render + action callback cadence.

### 2026-08-29 — REAL chat/LLM responsiveness
- Diagnosed a UI-only failure where direct REAL `/api/turn` worked but `/real/api/turn` SSE stalled after the MJPEG latency optimization. Root cause: the shared REAL proxy used large `read1()` chunks for both multipart MJPEG and `text/event-stream`. SSE now uses upstream `readline()` + immediate flush + `X-Accel-Buffering: no`; MJPEG retains `read1()`.
- Added an immediate SSE `status` event (`요청을 받았습니다`) and UI rendering for it, so input acknowledgement appears before the model completes. Broken browser SSE connections now set the loop cancellation flag instead of leaving invisible long-running work.
- Bounded aggregate Groq rate-limit sleep to `GROQ_RATE_LIMIT_MAX_WAIT` (default 2.0 s), preventing old cooldown state from causing tens of seconds of silent waiting. Five configured Groq keys were independently probed healthy during diagnosis (values never exposed).
- Added explicit turn routing: normal conversation -> `conversation_only` (no tools, no world-state injection, no automatic camera attachment); visual questions -> conversation-only with the current camera frame; robot manipulation/search intent -> embodied agent/tool loop. This prevents arbitrary text like `공개 경로 확인` from entering repeated search/track dry-runs.
- In conversation-only mode, a non-JSON model reply is safely treated as the final text rather than surfacing a protocol error. Tool/actuator mode remains strict JSON protocol.
- Public 8444 verification showed normal chat as `status -> final -> done` with zero tools, and visual questions also `status -> final -> done` with zero tools. Input acknowledgement was ~15–17 ms in sampled public HTTPS runs; normal text final was observed around 0.27–0.8 s and a camera/VLM question around 1.8 s.
- No Pi actuator/controller process was stopped or commanded for this work; only Oracle-side REAL/UI services were restarted.

### 2026-08-29 — Robot-command acknowledgement vs execution start
- REAL UI previously acknowledged the request immediately but emitted no progress while a blocking planner/tool call was pending; one observed request had about a 23-second silent gap before physical `search` began.
- Added pre-dispatch status events (`로봇 실행을 준비하고 있습니다`, `명령을 해석하고 있습니다`, `<tool> 실행 시작`) and an explicit `실제 실행 꺼짐` status for dry-runs.
- Added deterministic `SEARCH` routing for explicit red-block search requests. `빨간 블럭 찾아줘` now compiles directly to `search`; fake REAL runner verification reached tool start in ~0.116 ms without invoking the LLM. Red-block GRASP already compiles directly to `search -> track -> approach -> pick`.
- Closed the SIM/REAL tab-switch race by disabling Send until status loading finishes and applying server `execute_default` exactly. Public REAL currently reports execute_allowed=true, execute_default=true, talk_only=false.
- REAL placement on blue/yellow remains unsupported in the active physical adapter because `map_blue/map_yellow`, `carry`, and `place_on_blue/place_on_yellow` are not currently exposed by `scripts/robot_actions.py`.

### 2026-08-29 — REAL camera stale-frame / SIM carryover fix
- **Observed symptom**: after switching the browser from SIM to REAL, the first-person panel could appear to keep showing the simulation view. Backend byte comparison showed the current SIM frame, the REAL server's cached frame, and a fresh MasterPi snapshot were all different, so the REAL API was not actually wired to the SIM camera.
- **Root cause 1 — dead persistent camera path**: the REAL service had retained a physical camera frame for about 15 minutes (`camera_frame_age_ms` ~903,000 ms). Its startup SSH camera tunnel had died, while the camera pump kept retrying the same dead local endpoint and the cache had no freshness expiry.
- **Root cause 2 — browser carryover**: SIM and REAL reuse the same `<img id="preview">`. Mode switching replaced its URL without clearing/hiding the previously rendered bitmap, so while REAL waited for a new frame the last SIM image could remain visible and look like a SIM->REAL source mix-up.
- **Backend fix**: REAL cached frames now expire for sensor/UI use after 2.5 s by default (`UGRP_CAMERA_STALE_AFTER_S`). `/api/camera/stream`, action image resolution, and `talk_only` no longer treat an arbitrarily old physical frame as current evidence. Raw cached-frame diagnostics remain available through `cached_frame_info()`.
- **Reconnect fix**: physical camera transport now recreates a dead SSH port-forward on demand and falls back to one persistent direct SSH MJPEG stream when necessary. The pump can return to a newly created tunnel on the next reconnect rather than being permanently pinned to the dead startup port.
- **UI fix**: switching to REAL clears the previous `preview.src`, hides the image until REAL status reports a fresh camera, and polls REAL status once per second. If the camera becomes stale, the prior bitmap is hidden and the UI says the latest MasterPi frame is unavailable instead of displaying stale/SIM-looking imagery.
- **Safety boundary**: only Oracle `ugrp-real.service` and Oracle-side SSH camera transport were restarted/fault-injected. No Pi ustreamer, motor, servo, or actuator controller was stopped or commanded.
- **Live validation**: after deployment, direct REAL samples showed frame ages 19.7/46.8/90.1 ms with sequence 1002->1013->1025; the combined `/real/api/status` path showed 21.4 ms. Killing only the Oracle-side camera SSH stream froze sequence 1374 for ~2.2 s, then a new SSH forward appeared and sequence resumed 1376->1470 with ~14-192 ms frame ages, before the 2.5 s stale threshold.
- **Policy separation**: strict positive target confirmation for destructive physical `pick` is now REAL-only; generic/SIM dry-run executive semantics remain compatible.
- **Regression**: focused camera/REAL/state suite = 40 tests OK (1 OpenCV-environment skip); broader harness subset = 105 tests OK (1 skip); final `.venv-sim` suite = **291 tests in 62.424 s, all passed (1 skip)**.

## 2026-08-29 — MasterPi physics fidelity correction (platform/demo, not research result)

- User correctly challenged the existing MuJoCo simulation as unsuitable for learning. Audit confirmed that software-test success had been masking major physics-fidelity defects.
- Deployed `MasterPiPhysicsWorld` is now explicitly classified **NOT TRAINING READY**. Critical failures include direct generalized `x/y/yaw` chassis actuation, decorative/non-contact mecanum wheels, disabled robot/workcell collisions, incorrect compiled robot mass (~3.10 kg vs manufacturer 1.10 kg), arm-link geometry mismatch, 62° SIM camera vs 48° current physical calibration, and simulator-only chassis action semantics.
- Historical PPO `sim/grasp_physics.py` is also explicitly non-transfer: mocap base, 150 mm block, different six-axis arm topology, and privileged block/contact observations. `train_grasp_ppo.py` and `eval_grasp_ppo.py` now refuse to run by default unless `--allow-legacy-physics` is supplied for historical reproduction only.
- Added `scripts/benchmarks/physics_fidelity.py` and tests. A green unit/integration suite no longer implies physical validity; the separate fidelity gate must be green before sim-to-real training is permitted.
- Added isolated candidate `sim/masterpi_dynamics_v2.py`; production Lightning worker is intentionally unchanged until calibration/promotion gates pass.
- v2 structural changes: free 6-DoF chassis, four wheel hinge/contact bodies, ABAB reduced mecanum motor-to-wrench model, simplified robot/workcell collision geometry, explicit 1.10 kg total mass budget, 4-DOF+gripper arm using current physical FK constants, 48° camera VFOV, and hardware-domain motor/PWM commands instead of direct world-frame chassis motion.
- Found and fixed a MuJoCo joint-axis sign error while testing v2. After the fix, search/grasp/observe PWM poses match the existing physical FK camera-axis and gripper-tip positions to floating-point precision.
- v2 directionality/contact stress tests now show stable four-wheel support, forward/lateral/yaw motion in the expected ABAB directions, collision stop at the back wall, and physical block displacement on chassis contact. These are structural tests only; their motion magnitudes are **not** claimed to match the real robot yet.
- Added `sim/masterpi_dynamics_calibration.json`, `docs/masterpi_physics_calibration.md`, and `scripts/benchmarks/fit_masterpi_dynamics.py`. Force, damping, slip, servo, gripper and inertia-distribution values remain provisional. Calibration manifest remains `validated:false` until repeatable real trials and held-out acceptance pass.
- Candidate v2 fidelity currently passes all structural checks and intentionally fails only `dynamics_calibrated_against_real`. Therefore sim-to-real/RL training remains blocked.
- This work belongs to the MasterPi platform/demo track and does not amend or claim completion of the approved research specification.
- Final regression after v2 structural/contact tests and calibration tooling: **305 tests in 68.852 s, all passed (1 environment skip)**. Production Lightning T4 remained connected/authoritative and Oracle CPU worker remained inactive+masked throughout.

### 2026-08-29 — REAL search floor-first policy
- Physical usage assumption: manipulation targets are usually on the floor, so REAL `search` now spends the majority of its camera budget on downward floor views before any high-view fallback.
- Replaced the generic 3x3 `tilt=700/880/1040` sweep with ordered floor scan points covering close/mid/far floor (`tilt=620..860`) and center/left/right pans.
- Higher `tilt=960` fallback is sparse (only after selected chassis pulses) instead of being paid at every heading.
- Kept strict clipped-border/self-artifact rejection; pointing farther downward is used to bring close floor blocks into a bounded image region rather than weakening the self mask.
- Live MasterPi validation: floor-first search detected red at `pan=1500, tilt=620`, then confirmed it at `pan=1250, tilt=660` with `nx≈0.267, ny≈0.835, area≈5273`; no high-view fallback or chassis rotation was needed.
- Focused regression after the change: 67 tests passed, 1 OpenCV-environment skip.

### 2026-08-29 — SIM `map_blue` 503 / GPU episode recovery hardening
- User observed `작업을 완료하지 못했어 (map_blue): sim bridge unavailable: HTTP Error 503: Service Unavailable` during a red->blue stacking request.
- Root cause: Lightning T4 Studio itself remained Running and `nvidia-smi` was healthy, but both `supervise_worker.sh` and `run_mujoco_ws_worker.py` had disappeared. The GPU-only bridge therefore had no authoritative remote worker and correctly rejected `/command` with `503 GPU_OFFLINE`; the old caller incorrectly treated this transient outage as a permanent task failure.
- Bridge now tracks `remote_last_instance_id`, `remote_episode_id`, and WS generation. Same-process reconnect preserves the MuJoCo episode/inflight command. A different worker process is an explicit new episode; old queued/inflight commands are aborted with `SIM_EPISODE_RESET` and are never replayed into a fresh world.
- `scripts/sim_actions.py` now waits for a transient GPU reconnect (default 30 s). If the same process returns it retries the command once; if a replacement process returns it reports `SIM_EPISODE_RESET` to the executive rather than replaying a middle primitive.
- Structured executive handles `SIM_EPISODE_RESET` by resetting transferable state and restarting the original deterministic goal plan from step 1 once. This prevents stale `carry/place` execution in a fresh world.
- GPU watchdog no longer restarts the coworker service on a new remote process, so the HTTP/SSE request waiting for recovery survives. `ChatState` synchronizes episode boundaries in-process and clears stale history/camera/state evidence between turns.
- Lightning recovery now first fast-restarts the existing bundle in a Running Studio and launches the supervisor using durable `nohup`; full archive upload/dependency setup is only a fallback. Duplicate Lightning auth probing in the child recovery path was removed.
- Controlled live fault injection: killed only remote Lightning supervisor+worker while leaving T4 Studio alive, then immediately submitted `빨간 블럭 잡아서 파란 블럭 위에 올려줘`. First `map_blue` observed `SIM_EPISODE_RESET`, the executive restarted from `map_blue`, then ran `search -> track -> approach -> pick -> carry -> place_on_blue`. Final relation was `RED=ON_BLUE`; no HTTP 503 was exposed to the user. End-to-end request including recovery took about 17.9 s in that trial.
- Oracle CPU MuJoCo fallback remained inactive and masked throughout.
- Focused recovery/bridge tests: 29 tests OK after updating one stale REAL pick assertion to the already-required `--preserve-gaze` contract.
- Final full regression: **327 tests in 84.838 s, all passed (1 environment skip)**.

### 2026-08-29 — composable REAL chassis primitives
- Added a general low-level locomotion layer so the agent is not limited to red-block task macros: `move_forward`, `move_backward`, `strafe_left`, `strafe_right`, `turn_left`, `turn_right`, and `stop_motion`.
- Primitive motions use the existing MasterPi mecanum `DRIVE_MAP`; raw individual motor control is still not exposed to the model.
- AI-facing motion bounds are deliberately tighter than the low-level driver: speed `10..30`, duration `0.10..0.80 s`, default `20 / 0.30 s`. Each call is one bounded pulse followed by stop and a fresh post-action observation.
- Extended the action catalog with typed `ACTION_PARAMETERS`, keeping legacy zero-argument action files backward compatible. Tool prompt/API metadata now expose primitive parameter type/default/description.
- Primitive motor completion reports `motion_executed=true` but leaves `outcome_status=UNKNOWN`; task success must come from the fresh camera/sensor state, not wheel command completion.
- Any real chassis transform sets `chassis_motion=true`. With no chassis odometry, the state estimator invalidates target visibility/centering/range plus robot-relative spatial/semantic memory before the next fresh observation.
- Added Korean plan aliases (`전진`, `후진`, `좌회전`, `우회전`, `왼쪽 이동`, etc.) for open-ended agent planning.
- Physical smoke test: `turn_right speed=10 duration=0.10s` executed successfully on MasterPi with controller probe ~7.825 V and bounded pre/final STOP behavior.

### 2026-08-29 — REAL camera/search regression: intent, mixed Pi code, and track recovery
- Physical camera transport itself was healthy; direct Pi MJPEG measured ~22.5 fps. The browser REAL preview was fragile because an endless multipart `<img>` stream relied on load/error behavior and was sometimes hidden/reconnecting. REAL UI now polls the server's latest cached JPEG; the upstream HTTP reader uses `read1` when available. After restart, cache publication measured ~25 fps with ~67 ms frame age.
- Bare spatial statements were incorrectly treated as actuator intent because nouns/colors/directions were in `_AGENT_HINTS`. `오른쪽에 있어` previously produced `agent=True` and started track. Intent classification now requires actual action wording; location statements remain visual/context only.
- A simple location correction also exposed VLM hallucination (`오른쪽에 있어` produced an invented person/clothing/laptop description). Declarative spatial corrections now bypass the VLM and return a deterministic acknowledgement while being retained as conversational context.
- Internal tool/debug logs were previously appended to assistant conversation history. History now stores only user-facing speech/final text, preventing later model turns from imitating tool traces and raw result dictionaries.
- `track.py` previously turned into a hidden search routine on the first missing frame, moving the camera away after nearly centering the target. Track now preserves gaze over bounded transient misses and fails in place after four consecutive misses. Threshold is 500 px, center confirmation is two frames, agent track window is 3.0 s.
- `search.py` now checks the current gaze before moving, accepts right-edge floor targets, keeps bottom/self clipping protection, and refuses to claim a current-view success when the target lies farther outward at a saturated pan servo. It then reacquires from a centered floor scan.
- Found a deployment-integrity failure: the shared Pi `/tools/red_block` directory had `search.py` from the new package but `camera.py`/`track.py` overwritten later by another legacy workflow while the old aggregate stamp remained unchanged. This caused `save_debug_frame(..., label=...)` to fail only on Pi even though Oracle source supported it.
- New deployment is content-addressed: every package signature runs from `/home/ugrp1/MasterPi/tools/.ugrp_versions/<sha256>/red_block`, includes a sha256sum manifest for all Python files plus `masterpi_control.py`, and verifies the manifest before every execution. Other legacy jobs may still overwrite the shared directory without creating a mixed version for UGRP actions.
- Physical validation after versioned deployment: search reacquired red from centered floor scan (`pan=1500, tilt=500`), then track verified centered at approximately `nx=0.470, ny=0.506`, area ~1145, with two fresh-frame confirmations.
- Existing competing/legacy Pi processes were observed but not stopped or killed.

### 2026-08-29 — REAL camera self-heal and no-frame execution gate
- Confirmed the final camera outage was on MasterPi itself: port 8080 had no listener and `ugrp-camera.service` was inactive after a clean SIGTERM. Its `Restart=on-failure` policy does not recover a successful/SIGTERM shutdown.
- Added `ensure_camera_service()` on the Oracle side. It first probes `127.0.0.1:8080/snapshot` on the Pi and is a no-op if any existing camera process already serves it. Only when 8080 is absent does it request `systemctl --user start ugrp-camera.service`, then polls briefly. It contains no stop/kill/pkill behavior, so legacy camera owners are not displaced.
- REAL server startup, stream reconnect, and snapshot fallback now call this non-destructive camera self-heal.
- Added a REAL execution gate: if an actuator command is requested but no current camera frame can be obtained, no tool is executed and the response explicitly says the robot was not moved. This removes the prior ambiguous '동작을 수행했지만...' message during camera startup/outage.
- Restart validation with camera already healthy: self-heal was a 0.416 s no-op, REAL became camera-ready in 4.86 s, frame age 41.7 ms.
- Steady-state camera validation: ~23.34 cached frames/s with 16.3 ms age and talk_only=false.
- Final live API validation: location correction `빨간 블록이 오른쪽에 있어` produced the deterministic acknowledgement and zero tools; `빨간 블록 찾아줘` executed exactly one live search and returned ACHIEVED in 4.691 s. Camera age at completion was ~1.6 ms.

### 2026-08-29 — REAL MasterPi buzzer call removal
- User requested that audible buzzer activation be removed from the code paths used around the MasterPi setup.
- Oracle repository active execution code already contained no buzzer call; `scripts/red_block/physical_state_machine_reference.py` explicitly avoids the colour-demo modules that use it.
- On the physical MasterPi, removed the three actual runtime buzzer callsites found under `/home/ugrp1/MasterPi`: `rpc_server.py`, `functions/color_sorting.py`, and `functions/color_detect.py`.
- Kept the SDK `set_buzzer()` method definition because the definition itself does not activate hardware and removing a vendor API would create unrelated compatibility risk. No non-comment runtime callsite remains in the scanned MasterPi Python tree.
- Removed the `Board.setBuzzer(...)` example from `docs/masterpi_technical_architecture.md` so future copy/paste does not reintroduce audible feedback.
- Validation: the three edited Pi files passed `python3 -m py_compile`; no actuator command, process stop, or service restart was performed for this change.

### 2026-08-29 — MasterPi wheel dead zone is a system-wide hardware constraint
- Physical operator observation supersedes earlier assumptions: this chassis does not move for nonzero wheel/chassis command magnitudes <=30. Earlier logs that called low-speed commands "completed" only proved command delivery, not mechanical motion.
- Live chassis command policy is now: `0` is reserved for STOP; nonzero chassis/wheel magnitude must be `31..40`; preferred/default motion command is `35` to leave margin above the static-friction dead zone.
- Fine motion is controlled by shorter pulse duration, never by reducing speed into the dead zone.
- Updated every REAL motion path: agent primitives, search body turns, approach forward/back/align/recovery turns, pick correction, place search/align/approach, legacy physical reference state machine, browser manual controls, dashboard controls, and transport command generation.
- Defense in depth: `scripts/masterpi_control.py` drive parser + `drive_speeds()` reject <=30; `MotorTransport.write_motor()` rejects every nonzero magnitude <31 while preserving zero STOP; `red_block/Robot.drive`, dashboard payload validation, and dashboard transport validate the same constraint.
- UI defaults changed to speed 35 and ranges to 31..40. Offline dynamics fitter still accepts historical 1..40 calibration rows explicitly because those may contain measured dead-zone trials; it never drives hardware.
- Source audit found no remaining executable REAL hard-coded wheel command in 1..30 after the changes.
- Pi content-addressed deployment verified the new driver rejects speed 30 before I2C. A conflict-free physical smoke test at `rotate-right speed=35 duration=0.10s` completed; before/after 640x480 camera frames had mean absolute RGB differences about 22/255 (downscaled luminance mean ~10.98), confirming a substantial scene change rather than command-only success.
- Focused regression: 198 tests OK (1 environment skip). Full regression: 363 tests OK (1 environment skip).

### 2026-08-29 — SIM coworker UI cleanup and manual-control removal
- Removed the obsolete browser-side `모터 / 팔` manual-control panel entirely, including its status/note boxes, motor display, directional buttons, arm sliders, `renderRobot`/`loadRobot`/`command` JavaScript, and startup/mode-switch robot polling. The chat/action system remains the robot-control path.
- Reworked the desktop SIM monitor area to use the available stage height instead of leaving a stretched utility card: 2 columns × 3 rows, with the final robot-first-person/Semantic Radar row slightly taller. At a measured 1440-wide headless layout, the stage was 911 px wide, the monitor grid used 810 px of vertical space, CCTV cells measured ~439×253 px, and the final two cells ~439×288 px.
- Converted `실행 가능한 Python 도구` into a collapsed `<details>` panel (~41 px closed height in the same measurement) so it no longer consumes the lower half of the stage; expanded tools use a compact two-column scrollable grid.
- Increased the desktop workspace share relative to chat and tightened spacing. Added responsive fallbacks for narrower windows while preserving REAL's two-view layout.
- Tightened SIM CCTV framing by changing the local Three.js observer FOV from 50° to 44° and moving the four observer presets closer to the workcell, reducing empty-floor framing and making the robot/blocks easier to read.
- Added UI regression assertions to `tests/test_sim_ui_consistency.py` covering removal of the manual control panel, SIM layout behavior, collapsed tools, cache-busted `sim3d.js`, and the new CCTV presets.
- Validation: `node --check harness/static/sim3d.js` passed; `tests.test_sim_ui_consistency` + `tests.test_real_radar_ui` passed 9 tests; combined `tests.test_harness` + those UI tests also passed. The live server serves HTML/JS with `Cache-Control: no-store`, so no coworker service restart is required for these static changes.

### 2026-08-29 — Correction: SIM UI combined regression result
- Correction to the immediately preceding SIM UI log: the first combined `tests.test_harness + UI` run did **not** pass because one legacy HTML assertion still required the removed `/api/robot` manual-control wiring to appear in the page.
- Updated only that stale HTML expectation to require `/api/robot` to be absent from the served UI. Backend `/api/robot` endpoint tests remain intact, so this does not remove or weaken the lower-level API itself.
- Re-ran `tests.test_harness`, `tests.test_sim_ui_consistency`, and `tests.test_real_radar_ui`: **78 tests passed in 11.712 s**. `node --check harness/static/sim3d.js` also passed.
- Live `:8082` HTML contains no `/api/robot`, `robot-status`, `data-drive`, or `모터 / 팔` UI references and is served with `Cache-Control: no-store`.

### 2026-08-29 — restore known-good precision approach/grasp pipeline from uploaded MasterPi code
- Compared the uploaded `masterpi_red_pick_code1` controller against active REAL actions. The active `approach.py` had been reduced to image `nx/ny/area` heuristics and the active `pick.py` used a fixed hover/grasp pose derived largely from one image-x sample; this discarded the known-good controller's calibrated metric pipeline.
- Restored the uploaded control sequence to the active split actions: inherited target gaze -> multi-frame reacquisition -> camera centering -> chassis-to-gaze alignment -> STOP -> multi-frame metric range estimate -> one short chassis pulse -> STOP -> camera delay flush -> remeasure/recenter -> final stopped metric estimate -> FK/IK pick plan.
- `approach.py` now writes a short-lived `/tmp/ugrp-red-pick-plan.json` containing the measured block coordinate and dynamically calculated hover/grasp poses. `pick.py` consumes that exact plan, executes the blind grasp, returns to the same hover view, verifies pickup-site disappearance, and on the first failure remeasures/recalculates once before a second attempt.
- Added defense against stale geometry: the precision handoff expires after 60 s and is invalidated by any chassis motion or servo 3/4/5/6 camera/arm movement between approach and pick. Pick also rejects a handoff if the current persisted arm/camera pose differs from the measurement pose.
- Fixed immutable-package imports in `physical_state_machine_reference.py`: sibling modules are now loaded from `Path(__file__).resolve().parent`, preventing a content-addressed execution from accidentally importing the legacy shared `/tools/red_block` package.
- Found a separate critical physical bug while comparing the uploaded code: the active global `DRIVE_MAP` had semantic directions permuted. Restored the physically verified MotorTransport patterns: forward `(-,+,+,-)`, backward `(+,-,-,+)`, left `(-,-,-,-)`, right `(+,+,+,+)`, rotate-left `(-,+,-,+)`, rotate-right `(+,-,+,-)`. The previously established nonzero wheel dead-zone constraint remains 31..40 with default 35.
- Unified search/track handoff with the precision controller's calibrated camera envelope: pan 1300..1700, tilt 620..1200. Wide scene coverage is achieved by bounded chassis rotation rather than leaving the arm at an extreme base yaw that the calibrated grasp geometry does not support. Search retains its feet-first vertical scan, with horizontal scan points kept inside pan 1300..1700.
- Added/updated tests for precision call ordering, metric-plan persistence/rebuild, stale/pose-changed plan rejection, same-view grasp verification path, calibrated track/search envelope, and exact physical DRIVE_MAP signs.
- Physical actuation test for the new approach/pick was intentionally not run while a separate existing Pi process `/home/ugrp1/MasterPi/tools/red_block_pick_state_machine.py` was active. Per operator requirement, that competing process was neither stopped nor killed. The new immutable package was deployed/read-only verified on Pi and loaded the restored DRIVE_MAP, speed floor, and same-version precision modules correctly.

### 2026-08-29 — REAL grasp failure gating, near-field centering, and Groq TPM control
- Reproduced a physical `approach -> pick -> observe_scene -> Groq 413 TPM` failure from REAL logs. The physical approach actually moved the chassis at speed 35: stationary metric range decreased 28.71 -> 27.71 -> 27.05 -> 26.32 -> 25.71 -> 24.89 -> 24.07 cm, so insufficient wheel output was not the direct cause of this episode. Battery telemetry during the run was approximately 7.43-7.51 V.
- The direct approach failure was a contradictory visual deadband: `track.DEADBAND` stopped correcting at 0.05 image error while the precision state machine required 0.04. At the observed near-floor pose (`nx~0.450`, `ny~0.696`, `pan~1520`, `tilt=620`) the tracker intentionally stopped but `centre_gaze()` waited 35 frames and failed. Precision coarse centering now uses at least 0.055 while the later final lateral/FK-IK alignment keeps its separate much tighter tolerance.
- REAL pick now requires `task.phase=PREGRASP_READY`, which is created only by a successful approach/FK-IK handoff. Camera-only PREGRASP/centered estimates cannot authorize a destructive pick after approach failure. Any chassis motion invalidates this planner phase, matching the Pi-side handoff invalidation.
- Follow-up commands such as `집어보라고` with omitted colour compile to the deterministic red pipeline when the active toolset exposes only the generic red `search/track/approach/pick` stack. This prevents an LLM from inventing `approach failed but it looks close enough, so pick` and avoids repeated VLM calls for a known manipulation program.
- Planner feedback sent to the LLM is now compact: full UI/debug world state is retained internally, while model feedback contains only current target/grasp/action/task essentials, live metric landmarks, and a bounded subset of tool-result evidence. Debug payloads, argv, full semantic maps, and observation paths are not re-injected.
- Groq HTTP 413 messages explicitly reporting organization/model TPM exhaustion are treated as shared TPM failures: do not rotate API keys and do not issue another large fallback-model request, because doing so only amplifies the same quota failure.
- Added regression tests for colorless deterministic grasp, failed-approach no-pick/no-LLM behavior, mandatory PREGRASP_READY handoff, chassis invalidation, planner payload compaction, exact near-floor centering regression, and Groq TPM no-key-rotation/no-fallback behavior.
- Relevant REAL/harness regression: 371 tests passed in the venv. A separate `test_sim_ui_consistency` module was excluded because its CCTV assertions were stale relative to a concurrently modified SIM UI file; that unrelated concurrent work was not overwritten.

## 2026-08-29 — REAL approach straight-line control and controller isolation

- Physical logs explained the reported “sideways tiny movements”: the active precision approach made image-derived block-face alignment a mandatory stage at ~25.5 cm and spent up to 10 lateral orbit pulses chasing a noisy face-angle estimate. A separate manually launched `/home/ugrp1/MasterPi/tools/red_block_pick_state_machine.py` also mixed `forward`, `right`, and `yaw-left` raw-I2C vectors while repeatedly logging no progress.
- Removed mandatory lateral `align_block_face()`/orbit from the active REAL `approach.py`. The active approach is now camera centering -> in-place body yaw alignment -> stopped metric range -> pure forward pulse -> stopped remeasurement/recentering.
- Physical validation on MasterPi confirmed the new path: after one in-place body-yaw pulse, approach used pure forward wheel vectors `[-35,+35,+35,-35]` and measured 22.00 -> 18.48 -> 16.44 -> 14.43 cm, reaching a valid pregrasp and FK/IK plan without any lateral orbit. No pick was executed during this validation.
- The 16.44 -> 14.43 cm final pulse exposed unnecessary near-goal overshoot. Added `FINAL_APPROACH_TOLERANCE_CM=0.8`, so a final radius within 16.2..17.0 cm is accepted instead of chasing sub-centimeter error with a multi-centimeter wheel pulse.
- Fixed search/track camera-envelope mismatch: search could acquire at tilt 500 while track previously clamped immediately to 620 and lost the target. Both track and precision control now use `TILT_MIN=500`.
- Added pose trust after reboot/external raw-I2C control: current-view fast path requires all metric servos 3/4/5/6 plus fresh pose telemetry; stale pose forces the calibrated search pose to be physically reasserted. Direct metric approach rejects incomplete/stale pose.
- Added a non-destructive guard against an active legacy raw-I2C pickup controller. It refuses concurrent actuation and never kills the competing process.
- Replaced the Pi manual `start_masterpi_red_pick.sh` entrypoint (old copy backed up) so future manual launches use the verified immutable search -> track -> straight approach -> pick pipeline and no longer `pkill` robot/camera processes or invoke the legacy 137 KB state machine.
- Focused REAL regressions: 69 tests passed in `.venv-sim`. Final immutable package: `8faff21230101205d5fd307a143b5cbf1ec2e381e08cba089fc30b66fd1389ad`.

## 2026-08-29 — SIM pick teleport correction: wall-clock pacing + servo-board duration interpolation

- User-observed failure was real: during `pick_red`, the browser-facing state jumped the red block by ~9.1 cm between published states, making the cube appear to teleport onto the arm.
- Root cause 1: production MuJoCo advanced controller sleeps as fast as compute while the WS publisher was wall-clock throttled. Added `MasterPiProductionV2.step_realtime()` and routed the migrated REAL `SimClock`, camera waits and drive durations through it. At 1x, simulated motion is now paced against wall time; 2x/3x remain accelerated.
- Root cause 2: `pick_red()` called `physical_state_machine_reference.move_arm_together()` outside `_patched_modules`, so that helper used real `time.sleep()` and froze MuJoCo during the arm motion. `pick_red()` now executes the physical arm sequence inside the same SimClock patch used by search/approach.
- Root cause 3: the SIM servo adapter treated the Hiwonder duration parameter as an instantaneous final actuator target. On physical hardware the servo board interpolates PWM over that duration. Added duration-aware pending servo trajectories: commands are interpolated every 20 ms of simulated time, can overlap like the physical asynchronous board, and continue progressing during camera/controller sleeps and chassis motion.
- Browser online polling was changed from 250 ms to 100 ms to match the ~10 Hz authoritative worker stream (`sim3d.js?v=20260829p`).
- SIM command timeout was increased from a hard 35 s to a bounded configurable default of 180 s (`UGRP_SIM_COMMAND_TIMEOUT_S`) because real-time 1x actions can legitimately exceed 35 s while streaming progress.
- Before servo interpolation, a controlled closed-grasp lift produced a ~13.2 cm published block jump and lost the object. After interpolation, the same MuJoCo test preserved bilateral contact and lifted continuously from ~2.49 cm to ~9.1 cm; maximum 10 Hz published block displacement was 6.9 mm (p95 ~6.1 mm).
- Added `tests/test_real_stack_sim_servo.py` to enforce: (1) nudge duration does not apply the final PWM instantaneously, and (2) a closed grasp lifted through the migrated REAL arm helper remains bilateral with no published jump >= 15 mm.
- Full regression after the correction: 402 tests in 80.716 s, OK.
- Deployment caveat: latest corrected bundle could not be restarted on Lightning because the Lightning API returned `insufficient balance`. The T4 worker is currently disconnected; GPU-only remains enabled and the Oracle CPU worker remains masked/inactive. The code is staged on Oracle and will deploy through the normal recovery path once Lightning can start the Studio again.

### 2026-08-29 — REAL pre-capture overlap gate fixed from physical failure
- Physical run failed in `approach` even though the locked red block remained centered and continuously moved closer: far-view `ny` progressed `0.527 -> 0.552 -> 0.579 -> 0.608 -> 0.635 -> 0.665 -> 0.696 -> 0.729 -> 0.760`.
- Root cause was an arbitrary `PRECAPTURE_TARGET_NY=0.80` plus an 8-pulse budget. The target was not lost; the controller timed itself out before switching camera geometry.
- FK hand-eye geometry shows that at about 15 cm block radius the far pose projects near `ny=0.719`, while the close capture pose projects near `ny=0.107`, safely inside the close view and still before the final TCP capture target `ny~=0.442` at 12 cm.
- Changed `PRECAPTURE_TARGET_NY` to `0.72` and bounded budget to 12 pulses; the exact physical failure trajectory is now a regression test and switches at `ny=0.729` instead of failing.
- REAL remote skill stderr is now propagated into structured `reason` so UI failure messages show the actual driver cause instead of only `one step did not satisfy its success condition`.
- Relevant regression suite: 128 tests passed. No physical rerun was performed during the fix.

## 2026-08-29 — REAL red-block search latency + hand-eye pick audit

### Why this pass was needed
- User observed that a red block directly in front of MasterPi was visible for a long time before `search` accepted it.
- User also correctly pointed out that the camera optical centre is not the gripper centre/TCP because the eye-in-hand camera sits above/behind the jaws. Final grasp alignment must therefore use a camera→gripper hand-eye target, not "put the block in image centre and lower the arm".
- Per user request, no further physical actuation was performed during this code/audit pass. The robot later became unreachable, so all post-change verification below is offline/unit/integration verification unless explicitly described as pre-change physical evidence.

### Physical evidence captured before disconnect
- 12:49 search run repeatedly detected the block but kept sweeping:
  - pan=1125 area=1627
  - pan=1450 area=1762
  - pan=1700 area=1968
  - pan=1400 area=2046
  - pan=1175 area=1886
  - later pan=1500 area=1886, pan=1750 area=2005
- An earlier 12:44 run similarly saw many 5k–6.9k px red components while still continuing the scan. This showed that the dominant problem was not HSV failing to see red; search confirmation/scan policy was failing to accept already-visible evidence quickly enough.
- A stale commanded-pose timestamp triggered a full calibrated-pose reset using five serialized ~1.5 s servo moves before the first search frame, adding roughly eight seconds even with a block already nearby.
- Latest complete physical pick run reached the close capture window at approximately `nx=0.503, ny=0.427`, with the nominal target `ny≈0.442`. It generated a 12.5 cm fingertip-radius, near-vertical blind descent and closed the jaws, but the post-grasp whole-view verifier still detected red in 9/9 frames and aborted.
- The copied debug frame `/tmp/ugrp1-pick-red-debug.jpg` contained the dominant red component around normalized centre `(0.506, 0.429)`, essentially unchanged from the pre-grasp capture location. Therefore the current physical hand-eye calibration is NOT proven correct; the latest real grasp was a clean miss.

### Search changes
- `scripts/red_block/search.py`
  - obvious cube-sized/trackable evidence now uses a 2-hit gate (initial frame + one consistent fresh frame), while weak/distant candidates retain the stricter 3-hit confirmation. Single-frame red is still never accepted.
  - current-view obvious candidates use the same fast confirmation path.
  - if the target is already visible and the full metric pose is known but the pose timestamp is stale, search now reasserts the same current 3/4/5/6 pose once and re-confirms the same gaze instead of deliberately looking away and starting a fresh sweep.
  - stale full-pose recovery uses a concurrent safe pose reassertion instead of clearing pose state and paying five sequential 1.5 s waits.
  - scan reads now wait for the actual 180–400 ms servo interpolation to finish before sampling the camera; previously `nudge_servos()` returned after ~50 ms and search could evaluate a frame while the camera was still moving.
- `scripts/red_block/robot.py`
  - added `reassert_pose_together(...)` for exactly this trusted-pose recovery. It rewrites even equal saved pulses, starts the joint interpolations together, waits once for completion, updates commanded pose state, and invalidates any precision pick handoff.

### Hand-eye / final grasp changes
- `scripts/red_block/physical_state_machine_reference.py`
  - made the final capture target explicitly two-dimensional: `CAPTURE_TARGET_NX` + `CAPTURE_TARGET_NY`.
  - added `capture_target_pixel_from_hand_eye()`; the capture controller now deals in an explicit *TCP target pixel*, not an implicit camera-centre assumption.
  - `fine_align_horizontal()` now takes `target_nx` rather than hard-coding `0.5`; this allows a measured lateral camera→jaw offset to be applied later without architectural changes.
  - `establish_capture_pose()` and `visual_capture_approach()` pass the TCP target through explicitly, and logs now print `TCP-target=(nx,ny)` and state that camera centre is not the grasp point.
  - IMPORTANT: `(0.50, ~0.442)` remains only the current nominal model value. The 2026-08-29 real 9/9 miss proves that the physical camera→TCP extrinsic is not yet calibrated well enough. No arbitrary replacement offset was invented while the robot was offline.
- Public Hiwonder MasterPi colour-sorting material was checked. Its lesson has the user physically place the block in front of the gripper before pickup; it does not provide the autonomous camera→gripper extrinsic calibration needed to recover the exact missing offset from documentation alone.

### Failure/recovery changes
- `scripts/robot_actions.py`
  - remote child-controller errors are no longer all flattened to `SKILL_FAILED`.
  - unambiguous track/approach target-loss errors map to `TARGET_NOT_VISIBLE` with `search` recovery.
  - approach final x drift maps to `TARGET_NOT_CENTERED` with `track` recovery.
  - a visually proven pick miss (red still on the floor / grasp postcondition failed) maps to `GRASP_NOT_ACQUIRED` so the structured executor can restart a fresh search→track→approach→pick goal once instead of assuming success or doing a blind closer retry.
  - missing/stale/invalidated precision pick handoffs map to `PREGRASP_PLAN_MISSING` with `approach` recovery.
  - unknown transport/driver failures remain generic `SKILL_FAILED`.
- Existing safety boundaries were rechecked:
  - approach invalidates old pick plans before work and on failure.
  - pick plans are short-lived and rejected after arm/camera/chassis pose changes.
  - strict REAL pick preconditions still require successful `PREGRASP_READY`, visible target, centered target, and PREGRASP range state.
  - visible red after a grasp remains a hard failure; the gripper is reopened and carry is not performed.
  - floor disappearance alone remains only `PROBABLE_HELD` at confidence <0.85; it is never promoted to `HELD` without stronger evidence.

### Regression coverage added
- Search tests now cover obvious 2-hit confirmation, weak 3-hit confirmation, scan-settle timing, stale-visible-current-gaze reassertion, and concurrent pose reassertion.
- Hand-eye tests cover explicit TCP pixel projection and a non-centre `target_nx` to ensure final alignment does not silently revert to image centre.
- Adapter tests cover target-loss recovery, visually proven grasp-miss classification, and stale pick-plan recovery.
- A loop-level REAL regression verifies that an approach target-loss performs bounded `search -> approach -> pick` recovery rather than ending immediately.
- One unrelated stale SIM UI cache-bust assertion expected `sim3d.js?v=20260829q` while current HTML already uses `20260829r`; the test expectation was updated to the current revision.

### Verification
- `tests/test_real_search_latency.py`: 40/40 passed.
- `tests/test_real_failure_gates.py`: 25/25 passed after loop-level recovery coverage was added.
- `tests/test_real_place_actions.py`: 30/30 passed.
- `tests/test_sim_ui_consistency.py`: 9/9 passed.
- Full suite: **432/432 tests passed** in 71.720 s using `.venv-sim`.
- Changed runtime files compile cleanly with `py_compile`.
- Versioned red-block deployment manifest contains `search.py`, `robot.py`, `physical_state_machine_reference.py`, `approach.py`, `pick.py`, `precision_handoff.py`, `camera.py`, and `track.py`.
- Current package signature after this pass: `e067b1c02f280eef6da0d355da94d2172c88951c09b1265de6aff119b84c2e95`.

### Remaining physical verification / handoff
- Post-change hardware behavior is **UNVERIFIED** because the robot disconnected and no further motion was intentionally sent.
- On the next live session, do not start by guessing another capture `ny`. First observe one controlled final-capture approach and measure the actual pixel location at which the jaw centre/TCP is geometrically over the cube. Use that to calibrate `CAPTURE_TARGET_NX/NY` (and, if needed, lateral/yaw hand-eye offset), then repeat a single verified grasp.
- Search changes are much more strongly supported by existing physical logs because the prior detector was already seeing the block; the remaining unknown is real-world timing improvement after deployment.
- Operational reload after the audit: `ugrp-real.service` was restarted on Oracle at 13:14 UTC so the in-memory `robot_actions.py` registry uses the new failure classifications. Service returned `active` with a new MainPID. This was a web/harness service reload only; no robot action or actuator command was issued. The Pi red-block package itself remains staged for automatic content-addressed sync on the next robot action after reconnect.
- Follow-up service check: REAL listens on `127.0.0.1:8083` (not 8081). Because the Pi was disconnected, startup spent ~31 s in the bounded SSH camera-availability check before opening the HTTP listener. Afterwards `/api/tools` returned 15 tools including `search`, `track`, `approach`, and `pick`; service remained active. No actuator action was invoked during this check.

## 2026-08-29 — REAL execution recorder for sim-to-real feedback loop

### Goal
- Turn the existing REAL-on-MuJoCo shared-controller architecture into a data-producing loop: every Oracle-launched physical red-block skill should leave enough structured evidence to calibrate SIM and build later REAL failure replay cases.

### Implementation
- Added `scripts/red_block/recorder.py`, opt-in only through `UGRP_REAL_TRACE_ENABLE=1` + a per-run trace directory. Without those variables it is a no-op, so SIM/unit execution does not silently become REAL tracing.
- Instrumented the physical camera boundary in `scripts/red_block/camera.py`:
  - every returned frame gets a monotonically increasing frame sequence and timestamp metadata;
  - red/blue/yellow detector outputs record visibility plus blob centroid, normalized coordinates, area, bbox dimensions/oriented points and rectangularity;
  - normal JPEG snapshots are throttled to 4 Hz by default to limit controller perturbation;
  - existing debug-frame callsites force an important trace JPEG.
- Instrumented `scripts/red_block/robot.py` at the REAL hardware wrapper:
  - robot initialization + commanded pose state/age;
  - hardware battery probe result;
  - servo target PWM, previous PWM and duration;
  - concurrent servo batches/reassertions and updated commanded pose;
  - semantic chassis direction/speed/duration plus the exact four `(motor_id, speed)` values produced by the physical `drive_speeds()` mapping;
  - STOP and command-completion events.
- `scripts/red_block/deploy.py` now gives each remote physical skill a unique trace ID, scopes recorder activation only to that Pi child process, copies the completed remote trace back to `outputs/real_traces/<trace_id>/`, and writes `result.json` with immutable package hash, argv, wall-clock timing, exit code/status and trace-copy status. Remote stderr and the existing failure debug image are retained when available.
- `scripts/robot_actions.py` exposes `real_trace_id` and `real_trace_dir` in the structured tool result so a later failure-corpus builder can link planner outcomes directly to physical evidence.
- Added `docs/real_execution_recorder.md` documenting the schema and promotion purpose.

### Safety / performance boundaries
- Recorder exceptions are swallowed by design and cannot become actuator failures.
- Camera JPEG encoding is throttled; metadata-only frame recording measured ~0.13 ms/call on the Oracle test host after the first sampled JPEG. This is NOT a Pi timing measurement and must not be represented as physical latency validation.
- No simulator-only body/contact truth is inserted into REAL traces.
- No physical robot actuation was performed during this implementation pass.

### Verification
- New recorder tests cover disabled no-op behavior, JSONL frame/detection/pose/chassis events, sampled JPEG creation, immutable deployment packaging, and `robot_actions` trace-reference propagation.
- Focused recorder/REAL/SIM regression: 107 tests passed.
- Full suite after final changes: **436 tests passed in 70.819 s**.
- Touched runtime files pass `py_compile`.
- Current immutable package signature: `a8ff41087ef69734b2ab92299460ad2a61baaeb0ee949b64f687b2c513d32ee6` and includes `recorder.py`.

### Physical status / remaining verification
- Tailscale reports `ugrp1` offline, last seen about one hour before this check. Therefore the new recorder has NOT yet produced a real physical trace and has NOT yet been timing-profiled on Raspberry Pi hardware.
- On the next physical skill launched through the normal Oracle REAL path, the immutable package will deploy automatically and that skill should create the first `outputs/real_traces/<trace_id>/` corpus item. Validate that `events.jsonl`, sampled frames and `result.json` are present before proceeding to REAL-failure replay automation.

## 2026-08-29 — Digital-twin calibration pipeline connected

- Corrected scope: the requested first stage is physical digital-twin calibration, not REAL failure recording. Automatic REAL trace activation was removed again from the normal deploy path so it does not perturb calibration runs.
- Found and fixed a critical gap: `fit_masterpi_dynamics.py` could write fitted chassis values to `sim/masterpi_dynamics_calibration.json`, but `MasterPiDynamicsV2` did not consume them. V2 now loads numeric manifest dynamics automatically while preserving explicit status separation: `STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED`, `REAL_FITTED_UNVALIDATED`, or `REAL_CALIBRATED_VALIDATED`. Training remains blocked unless fully validated.
- Updated the physically executable chassis calibration protocol from historical 10/20/30/40 commands to the live-safe measured-effective range 31/35/40, with 0.20 s and 0.55 s pulses. Generated 108 shuffled trials: 72 fit + 36 held-out.
- Added `scripts/benchmarks/masterpi_calibration_plan.py` and generated `calibration/masterpi/chassis_trials.jsonl`.
- Added fixed-overhead two-ArUco video measurement (`analyze_masterpi_overhead_video.py`), marker generator, and generated marker IDs 10/11 under `calibration/masterpi/markers/`. The actual mounted marker centre separation must be measured once and is used as per-frame metric scale.
- Added `apply_masterpi_trial_measurement.py` to write analyzer/manual results into one plan row and `run_masterpi_calibration_trial.py` to preview/execute exactly one planned bounded chassis pulse. It never chains trials because the robot must be reset to the external reference between trials.
- Synthetic overhead-video check recovered a programmed ~0.20 m straight motion as 0.189 m with zero lateral/yaw and zero stop distance; this only validates the analysis code path, not physical measurement accuracy.
- Current v2 remains honestly `STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED`; all manifest dynamics remain null because no physical measurements have been collected yet. Current fidelity gate's only v2 critical blocker remains `dynamics_calibrated_against_real`.
- Physical measurement has not started: `ugrp1` was offline during this pass. Next physical trial preview is `chassis-049`: left, speed 40, 0.20 s drive, 0.50 s coast.
- Full regression after these changes: **441/441 tests passed in 72.010 s**.

## 2026-08-30 — Hard-gated calibrated MasterPi task digital twin pipeline

- Scope: MasterPi platform/debug digital twin for the current `search -> track -> approach -> pick/place` task family, not a research claim of universal physical equivalence.
- The clean nominal V2 model remains the baseline, but calibration fields are no longer decorative. The generated MuJoCo plant now consumes fitted wheel radius/wheelbase/track, block mass/friction, gripper KP/finger friction, actual camera link/z/pitch, servo-6 physical center, and calibrated servo deadband/rate. Chassis running and stop/braking dynamics are separately identifiable.
- Added a dependency-free calibration schema and required-parameter gate. A manifest that claims `validated=true` while missing any required plant parameter remains unvalidated when loaded.
- Added physical evidence plans/tools for static dimensions/object properties, 60 servo trials (40 fit/20 holdout), 24 hand-eye trials (12/12), and 32 physical pick cases (12 gripper fit/20 final holdout), alongside the existing 108 chassis trials (72/36).
- Added fitters for servo deadband/rate, hand-eye sensor plant parameters, and gripper contact parameters, plus task-level REAL/SIM outcome evaluation.
- Added `validate_masterpi_digital_twin.py` as the only supported promotion path. It rejects SIM/MuJoCo/synthetic/mock evidence, checks dataset hashes and evidence counts, requires all held-out metrics to pass, and records immutable validation provenance before setting `validated=true`.
- `ugrp1` was offline during this implementation (Tailscale last-seen about 12 hours earlier), so no new physical calibration data was fabricated or inferred. The real manifest therefore remains correctly `validated=false`, with zero completed physical evidence rows.
- Synthetic calibration data is permitted only in unit tests to verify fitter recovery and the promotion gate; it is never written to the live manifest.

### Final verification for calibrated-twin infrastructure

- Remote SIM worker contract bumped to `masterpi-v2-calibrated-task-twin-v1`; older nominal/legacy workers cannot authenticate as the authoritative production SIM.
- Colab T4 was force-redeployed with the final bundle and is authoritative. The remote state exposes `physical_parameters`; because no physical calibration has yet been collected, it correctly reports `digital_twin_profile=nominal_real_contract_v1`, `physics_fidelity=V2_STRUCTURAL_UNCALIBRATED`, `training_ready=false`.
- Deliberate `validate_masterpi_digital_twin.py --promote` attempt with zero REAL evidence returned exit code 2 and left `validated=false`, proving that promotion cannot be faked by the current empty dataset.
- Full repository regression after all calibration/twin changes: 460 tests, all passing (`Ran 460 tests in 71.745s — OK`).
- Physical blocker remains external to the code: `ugrp1` is offline (last seen about 13 hours earlier), so the required physical evidence counts remain 0 and a truthful calibrated-twin promotion cannot be completed in this session.

## 2026-08-30 — Proactive REAL tool-contract hardening before next physical test

### Why this pass was needed
- User requested that the REAL tool layer be audited proactively so software/control-contract failures are found before the next MasterPi test rather than discovered by moving hardware.
- No physical robot actuator command was issued during this pass. Validation below is code/unit/integration/service-readback only unless explicitly stated otherwise.

### Legacy grasp bypass removed
- Found a critical alternate execution path: the public `fetch` tool still reached a historical single-frame/fixed-pose `grasp_and_lift()` implementation instead of the active `approach -> precision handoff -> pick` path. This meant the guarded `pick` implementation could be correct while one composite tool silently reintroduced the old failure mode.
- Removed `fetch` from the AI-facing REAL action/contract registry. The supported public manipulation sequence is now the explicit primitive pipeline.
- Kept `fetch.py` only as a compatibility CLI; it contains no independent grasp policy and delegates strictly to `run_approach()` followed by `run_precision_pick()`.
- Disabled standalone `carry.py` pickup entirely. `pick.py` already ends in carry pose, so a second grasp implementation is unnecessary and unsafe.
- Removed the dead fixed-pose `grasp_and_lift()` implementation from `pick.py`. Legacy `--grasp-pulse` and `--chassis-align` knobs are parsed only to fail explicitly; they can no longer silently suggest that they tune the precision grasp.

### Causal pick -> place handoff
- Added `scripts/red_block/carry_handoff.py`. A live `place` no longer trusts only "servo 1 happens to be closed" as proof that a block should be released.
- A successful precision pick writes a short-lived Pi-local handoff only after the pickup floor is visually clear and the robot has moved to the carry pose. The handoff records provenance, timestamp, commanded gripper pulse, and robot pose.
- `place` requires all of the following before beginning a live release sequence: fresh handoff, closed gripper command, and exact match of current commanded arm/camera servos 3/4/5/6 to the post-pick carry pose. Missing/stale/corrupt evidence or intervening arm movement rejects the place.
- Opening the gripper invalidates the carry handoff. A new destructive pick invalidates any older carry evidence before it starts.
- This remains deliberately `PROBABLE_HELD`, not physical proof of `HELD`: MasterPi has no force/current object-presence sensor.

### Planner/executive isolation while carrying
- In strict REAL execution, `search`, `track`, `approach`, and `pick` are now blocked while grasp state is `PROBABLE_HELD` or `HELD` (`OBJECT_ALREADY_CARRIED`). This prevents the eye-in-hand camera from rediscovering the carried red block as a new floor target and prevents accidental arm reconfiguration through a second pickup pipeline.
- `place_on_blue` / `place_on_yellow` remain permitted when the object may be carried; their Pi-side causal handoff is the final release gate.

### Bounded execution / infrastructure failures
- Added `remote_watchdog.py` to every deployed red-block skill. It owns the remote skill process group, applies a per-skill wall-clock budget, sends SIGTERM on timeout, then SIGKILL after a short grace period if needed. Oracle SSH also has a larger outer timeout so a dead transport cannot wait forever after the remote watchdog budget.
- Deployment package-current checks, package sync, and failure debug-copy subprocesses now have explicit time bounds.
- Direct `track.py` CLI is bounded by default (8 s); AI-facing track keeps its tighter 3 s budget.
- Infrastructure failures are no longer misclassified as target loss. Added explicit classes including `ROBOT_UNAVAILABLE`, `SKILL_TIMEOUT`, `CONTROLLER_BUSY`, `CAMERA_UNAVAILABLE`, `ACTUATOR_IO_FAILED`, and `CARRY_STATE_UNVERIFIED`, all without an unsafe automatic search retry.

### Lowest-level actuator I/O fail-fast
- Found that individual raw-I2C motor/servo writes in `scripts/masterpi_control.py` had no subprocess timeout. Added a 0.5 s timeout around each `i2ctransfer` write and convert timeout/exit/exec failures into `ControlError`.
- This gives three software timeout layers: individual actuator I/O -> remote skill watchdog -> Oracle SSH outer bound.
- Limitation: software still cannot guarantee a physical stop if the hardware/I2C path itself wedges after a motor command has already reached the controller. This change prevents indefinite software blocking; it is not a hardware E-stop claim.

### SIM/fidelity issues discovered by the same audit
- Full-suite probing exposed a deterministic fidelity-audit bug: candidate V2 had 15 canonical collision proxies enabled, but the gate divided them by all 76 robot geoms, including 61 intentionally decorative/non-colliding visual geoms. The collision gate now checks the canonical simplified physical envelope directly; all 15/15 required chassis/wheel/arm/finger collision geoms are enabled.
- The V2 fidelity gate's only remaining critical blocker is again the truthful external one: `dynamics_calibrated_against_real`.
- Two first full-suite failures were non-reproducible at fixed seed (12/12 drivetrain probes and 8/8 grasp-ready camera probes passed). Audit found `tests/test_masterpi_dynamics_v2.py` leaked its `setUp()` world after every test plus several local worlds. Added deterministic cleanup; the combined affected suites and final full suite are stable afterward.

### Verification
- Focused hardening regressions reached 176/176 passing before the final full run; carry/place follow-up regressions also passed.
- Deterministic SIM/fidelity subset after cleanup/fidelity-gate fix: 19/19 passed.
- Final repository regression: **483/483 tests passed in 73.250 s (`OK`)** using `.venv-sim`.
- Runtime files touched in the REAL path pass `py_compile`.
- No live `search`, `track`, `approach`, `pick`, `place`, chassis, arm, servo, or motor action was invoked during this audit.

### Remaining physical blocker
- None of these software gates resolves the known physical hand-eye error by itself. The latest real grasp before disconnect still missed with red visible in 9/9 post-grasp frames.
- The nominal capture target `(nx=0.50, ny≈0.442)` is still only a model value. Do not guess a replacement. The next physical manipulation session should first measure/calibrate the real camera->TCP target, then perform one controlled verified grasp before broader autonomous testing.

### Operational readback after the hardening pass
- Restarted only the Oracle `ugrp-real.service` harness so its in-memory tool registry uses the hardened contracts; no robot action endpoint was invoked.
- Service is `active`, MainPID `1921077`, listening on `127.0.0.1:8083`.
- Read-only `GET /api/tools` now exposes exactly 14 tools: `approach`, `move_backward`, `move_forward`, `observe_scene`, `pick`, `place_on_blue`, `place_on_yellow`, `search`, `stop_motion`, `strafe_left`, `strafe_right`, `track`, `turn_left`, `turn_right`. Legacy `fetch` is no longer AI-facing.
- Current content-addressed red-block package signature is `e80cdb55f3e9b1d5ab6945a01cbc2b48e3387479bc430081685cce84fce942c0`; it will sync through the normal verified deployment path on the next actual robot skill invocation.


## 2026-08-30 — REAL grasp-miss recovery corrected after live failure

### Live evidence
- A user-initiated REAL `search -> track -> approach -> pick` run reached the final visual capture at approximately `nx=0.497, ny=0.427` for the still-nominal TCP target `(0.500, ~0.442)` and completed one blind grasp/lift.
- Post-lift whole-view verification saw the red block in **9/9 frames**, so the first grasp was a definite physical miss. The gripper reopened as designed; no held-object state was claimed.
- The harness then incorrectly started a second autonomous `search -> track -> approach` cycle because an older `GRASP_NOT_ACQUIRED` recovery rule still allowed one full task restart even in strict REAL mode.
- That second search accepted the still-visible block from the post-pick close/verification arm pose rather than returning to the calibrated floor-search kinematic family. The inherited pose reached roughly `[3:612,4:2200,5:1900,6:1484]`; with the observed blob (`ny≈0.502`, area≈28k, height≈211 px), the metric floor ray evaluates past vertical (`~ -91.6 deg`) and therefore cannot produce a valid range sample. The reported `insufficient valid range samples (0/3)` was a deterministic secondary recovery-state error, not the original grasp failure.

### Corrections
- `harness/loop.py`: strict REAL execution now **stops immediately after a visually proven `GRASP_NOT_ACQUIRED`**. A failed destructive grasp is never followed by an autonomous second pickup cycle. The previous one-retry behavior remains available only to non-strict/SIM execution.
- `scripts/robot_actions.py`: a visually proven pick miss keeps `failure_code=GRASP_NOT_ACQUIRED` but has no automatic recovery recommendation. A new `APPROACH_POSE_INVALID` classification routes non-destructive invalid-pose entry back through `search`.
- `scripts/red_block/search.py`: the current-view shortcut now requires the fixed search/track arm geometry (`servo4=2320`, `servo5=1320`). A large red blob seen from a close/post-pick geometry is not allowed to become a SEARCH handoff; search first restores the calibrated floor-search pose.
- `scripts/red_block/approach.py`: defense-in-depth entry guard rejects any pose whose fixed arm geometry is not the search/track family before opening the video/ranging loop. This prevents direct/manual `approach` calls from repeating the same invalid-ray failure.

### Verification / scope
- Focused REAL recovery/search/approach/place/tool-hardening regression: **114/114 passed**.
- The new tests explicitly cover: no second strict-REAL pipeline after a grasp miss; post-pick close pose cannot satisfy current-view search; approach rejects the exact observed invalid pose before opening video; and invalid-pose failure classification.
- Repository-wide run executed 486 tests and exposed **five pre-existing/unrelated SIM-V2 geometry regressions** (static generated scene drift, an old exact-controller-FK expectation versus the newer physical arm placement, and three SIM grasp/contact expectations). The touched REAL files are not involved in those failures; the four representative SIM failures reproduce in isolation. Do not claim a clean repository-wide suite until that separate SIM-V2 contract drift is reconciled.
- No additional physical motor/servo action was issued while making or testing these corrections.

### Remaining primary physical blocker
- The original first-grasp failure remains the important physical issue: the nominal camera->jaw/TCP transform is still not calibrated from real measurements. The next manipulation test should not be another uncontrolled full retry. First calibrate/observe the real TCP pixel/hand-eye offset, then perform exactly one controlled verified grasp.

## 2026-08-30 — MasterPi nominal geometry correction verified and redeployed

- Continued the pre-calibration geometry pass after separating the arm into `arm_base -> yaw -> shoulder mount -> shoulder -> elbow -> wrist`. The final source compiles and MuJoCo loads the generated V2 XML successfully.
- Verified the corrected pitch-axis floor heights at the straight reference pose: shoulder **125.5 mm**, elbow **190.5 mm**, wrist **252.5 mm**. Independent scaling of the saved Hiwonder official side photograph gives approximately **127.8 / 189.8 / 249.1 mm**, so the respective nominal/photo differences are about **-2.3 / +0.7 / +3.4 mm**. The yaw-to-shoulder vertical separation is explicitly **30.5 mm**; the two axes are no longer co-located.
- Confirmed the former REAL-FK/SIM discrepancy is exactly one 65 mm wheel radius: the historical REAL math treats `LINK_1=9.3 cm` as floor-to-shoulder, while the corrected nominal mechanism treats it as axle/base-to-shoulder. SIM regression now records this **+32.5 mm** frame offset explicitly instead of lowering the SIM arm to satisfy legacy FK. The deployed REAL kinematic constants were intentionally NOT changed before physical calibration.
- Verified rigid eye-in-hand camera behavior at three widely separated arm poses. Camera position in the wrist frame remains exactly **(70, 0, 25) mm** for all tested poses; the old pose-dependent camera sliding cannot regress silently.
- Re-rendered the robot and compared the side view with the saved official photograph. The first corrected-height render still had unrealistic full-height orange plate links, so the inter-joint visual members were narrowed to approximately **11 mm full height** while preserving broad servo/joint cheeks and all joint locations. Render review artifacts are under `outputs/masterpi_geometry_review/final_arm_v2/`.
- Regenerated `sim/masterpi_scene_v2.xml` from the runtime V2 XML. The static V2 snapshot and runtime-generated scene are again byte-identical under the nominal-twin regression.
- Updated the nominal grasp regression rather than preserving fixtures that only worked with the arm 32.5 mm too low. A robust corrected nominal floor grasp is centered near **150 mm** radius. The tested open-hover -> descend -> close -> lift PWM sequence physically achieves bilateral contact and lifts the 30 mm cube; the old ~175 mm training fixture was near servo-5 saturation after the geometry correction.
- `grasp_ready` curriculum now samples **150 +/- 2 mm** with small lateral/yaw jitter. The observed REAL pre-capture command remains only a transferable camera-start command; it is not used as evidence that the legacy REAL height frame is nominal geometry.

### Verification
- Focused geometry/grasp/REAL-stack regression after the arm and visual changes: **28/28 passed**.
- Training-env regression after retuning the corrected nominal grasp-ready stage: **6/6 passed**, including camera visibility across reset seeds and a full `env.step()`-only grasp/lift.
- Final repository regression: **490/490 tests passed in 81.367 s**.
- Live Colab recovery redeployed the post-regression bundle into the existing T4 runtime. Bridge verification: one remote WS, `provider=colab`, `machine=T4`, `remote_authoritative=true`, worker contract `masterpi-v2-calibrated-task-twin-v1`, fresh 640x480 JPEG snapshots.
- Remote source SHA-256 values for `sim/masterpi_geometry.py`, `sim/masterpi_dynamics_v2.py`, `sim/masterpi_training_env_v2.py`, `sim/masterpi_scene.xml`, and `scripts/run_mujoco_ws_worker.py` were checked against Oracle; all checked files match the current Oracle bundle. A remote constant probe reports shoulder/elbow/wrist **125.5/190.5/252.5 mm**, yaw-to-shoulder **30.5 mm**, visual link full height **11.0 mm**, and grasp-ready radius **150.0 mm**.
- The authoritative remote state still correctly reports `digital_twin_profile=nominal_real_contract_v1`, `physics_fidelity=V2_STRUCTURAL_UNCALIBRATED`, and `training_ready=false`. No physical calibration evidence was invented and no REAL robot actuator command was issued during this pass.

### Next gate
- The nominal SIM geometry/runtime is now internally verified and redeployed. The remaining blocker is physical calibration/held-out validation against the actual MasterPi. Do not promote `training_ready` or rewrite REAL FK/IK from the nominal offset until those measurements pass the existing calibration gate.

## 2026-08-30 — REAL search no longer discards visible peripheral red + provisional forward grasp reach correction

### Physical evidence that triggered this pass
- In the user-initiated REAL search around 05:45 UTC, the user reported that the red cube was directly below/in front of the robot and visibly present in the camera, while `search` logged `red=no` through repeated floor scan points and multiple body sectors. Only after roughly three chassis sector turns did the normal detector finally report a large cube (`nx≈0.823`, area≈10k px).
- This was not merely an over-strict confirmation gate. `camera.py` discarded contours touching image borders before search could see them, and normal search additionally zeroed the left 100 px. Therefore genuinely user-visible red could be converted to `None` at the observation boundary.
- The latest physical approach/pick also showed that the chassis itself did continue into the close visual window (`ny≈0.248 -> 0.327 -> 0.404 -> 0.504`), but the blind grasp then always descended to the same legacy fingertip radius **12.5 cm** and again left red visible in **9/9** post-grasp frames. This supports the user's observation that the jaws were closing behind/short of the cube rather than the chassis simply stopping far away.

### Search observation fix
- Added `detect_red_suspicion()` in `scripts/red_block/camera.py` as a low-confidence observation channel. It deliberately keeps red components touching any image border and does not apply the normal left 100 px crop.
- This auxiliary channel is **not** allowed to claim a block, steer the chassis, enter metric approach, or authorize a pick. Its only role is to preserve the fact that red is visibly present and tell search where to look next.
- `scripts/red_block/search.py` now distinguishes:
  - normal bounded `candidate` -> may become confirmed SEARCH success;
  - peripheral/clipped `suspicion` -> immediately move the eye-in-hand camera toward it and re-run the normal detector.
- If the red hint is at the bottom while search tilt is already saturated at 500, search no longer proceeds as if the view were empty. It temporarily uses the already-supported fixed close camera geometry (`CAPTURE_ARM_POSE`) to inspect the immediate floor, preserving pan, and requires repeated normal detections there.
- A fresh close near-look result can pass through track/approach without being forced back to the far metric camera and losing the nearby cube again.

### Causal near-look handoff safety
- The close camera geometry overlaps a historical post-pick/verification arm family, so pose values alone are insufficient provenance.
- Added `scripts/red_block/near_look_handoff.py`: a live near-look SEARCH writes a short-lived token only after multi-frame red confirmation. `approach` requires and consumes this token before accepting the close-camera family.
- The token is bound to fixed servos 4/5, expires after 30 s, and is invalidated by chassis motion or by changing servos 4/5. Track may still refine servo 3/6 between search and approach.
- Therefore the old failure state `[3:612,4:2200,5:1900,6:1484]` cannot become a valid close approach merely because it resembles the near-look geometry; without fresh SEARCH provenance it is rejected before video/manipulation.
- Missing/invalid close handoff keeps the existing structured classification `APPROACH_POSE_INVALID -> search`.

### Provisional longitudinal grasp correction
- The close visual stop and the final jaw reach are now explicit separate quantities. The already-observed close-image target remains nominally `CAPTURE_TARGET_NY≈0.442`, based on `CAPTURE_VISUAL_BLOCK_RADIUS_CM=12.0`; this avoids compensating a short jaw by unnecessarily driving the chassis farther forward.
- The final grasp plan now applies an explicit **+2.5 cm provisional longitudinal reach correction**:
  - visual nominal block radius: **12.0 cm**
  - grasp block radius: **14.5 cm**
  - fingertip/TCP radius: **15.0 cm** (previously 12.5 cm)
- Offline current-V2 contact probing independently showed that, with a cube around 15 cm forward, 12.5–14.0 cm grasp radii miss bilateral finger contact while the ~15 cm region produces bilateral contact. The corrected nominal SIM also already uses a robust floor-grasp region around 150 mm.
- This **is not physical calibration** and must not be labeled calibrated/validated. It is a bounded provisional correction supported by two clean physical 12.5 cm misses, direct user observation of short reach, and nominal physical-SIM contact evidence. The next live attempt must remain a single controlled verified grasp; do not auto-loop on failure.

### Regression / operational verification
- Added explicit regressions for right-border red, normal-left-crop red, gaze refinement toward peripheral evidence, bottom-saturated near-look, preservation of near-look into approach, causal near-look token freshness/expiry/pose binding, and visual-stop-vs-jaw-reach decoupling.
- Focused search/approach/failure tests passed (96/96); subsequent search/approach/tool-hardening subset passed (90/90).
- Final repository regression: **502/502 tests passed in 83.900 s (`OK`)**. Expected libEGL `/dev/dri` warnings and argparse negative-test stderr are non-failures.
- Changed REAL runtime files compile cleanly.
- Current content-addressed red-block package signature: **`580dfe791a1dfd3515c68deb06d26b4d1e8689fae1a15092ca650e184ecc49a3`**. Package contains the new `near_look_handoff.py` and 21 Python files total.
- `ugrp-real.service` remains active (MainPID 2022060), listening on `127.0.0.1:8083`; read-only `/api/tools` exposes the expected 14 public tools.
- No physical motor/servo action was issued during this code/test pass. The new package will content-address-sync on the next actual REAL skill invocation.

## 2026-08-30 — REAL `search 실행 시작` UI stall fixed at the process boundary

### Incident
- Two user-initiated REAL requests at 06:03:55 UTC and 06:06:36 UTC stopped visually at `search 실행 시작`.
- The robot was not actually searching. `journalctl` shows `search.py` failed immediately during import with:
  `ImportError: cannot import name 'detect_red_suspicion' from 'camera'`.
- The on-disk `scripts/red_block/camera.py` already contained `detect_red_suspicion()` and a clean interpreter imported it successfully. The long-lived `ugrp-real.service` process had cached the older `camera` module in `sys.modules`; `robot_actions.py` then executed the newly edited `search.py` with `runpy.run_path()` in that same process, producing an inconsistent new-search/old-camera module set.
- Because the exception escaped the tool handler boundary, the HTTP request thread terminated before the UI received a tool completion/failure event, hence the apparent indefinite `search 실행 시작` state.

### Fix
- `scripts/robot_actions.py` no longer runs REAL red-block skill CLIs with `runpy` in the persistent chat-harness interpreter.
- Each `search/track/approach/pick/place/primitive` invocation is now launched in a fresh child interpreter, using the project's `.venv-sim/bin/python` when present. This is required because OpenCV (`cv2`) is installed in the project venv, not the system Python that hosts the web harness.
- The child inherits stdout/stderr so physical skill progress remains visible in the service journal; its process return code is converted into the normal structured tool result.
- `harness/loop.py::dispatch()` now catches unexpected implementation exceptions at the tool boundary, prints the traceback to the service journal, and returns a structured `TOOL_EXCEPTION` failure instead of tearing down the HTTP request thread. Future tool bugs therefore must not leave the UI stuck at a start event.

### Verification / operation
- Clean child smoke: `.venv-sim/bin/python scripts/red_block/search.py --help` imports the full current search dependency graph successfully and exits normally; no robot actuation occurs.
- Focused REAL/search/process-isolation regression: **97/97 passed**.
- Expanded harness + REAL relevant regression after the exception boundary was added: **168/168 passed**.
- Before each service reload, no active `search/track/approach/pick/place/primitive` or `remote_watchdog` process existed.
- `ugrp-real.service` was restarted only to discard the stale in-memory module set and load the new adapter/harness boundary. No motor or servo action was issued by this repair.
- Final runtime: service `active`, MainPID **2049484**, `127.0.0.1:8083` listening, and read-only `/api/tools` exposes the expected **14** tools including `search`.

## 2026-08-30 — REAL forward grasp reach increased from 15.0 cm to 17.5 cm after another clean physical miss

### New physical evidence
- The next user-run REAL pipeline successfully reached final visual capture at approximately `nx=0.491, ny=0.467` for target `(0.500, 0.442)`.
- The new 15.0 cm fingertip plan was definitely deployed and executed: hover `{3:816,4:2100,5:1915}` around `r=15.00,z=8.00`, final grasp `{3:776,4:1723,5:2486}` around `r=14.99,z=-0.01`, followed by the full five-waypoint descent and gripper close.
- Post-grasp verification still saw the red cube in **9/9 frames**, and the user again observed that the grasping distance was still short. This rules out the earlier hypothesis that only the legacy 12.5 cm reach was responsible; 15.0 cm is also physically insufficient in the current hand-eye geometry.
- The automatic second destructive pick remained disabled. The reported failure was the expected safe `MISS_RED_STILL_VISIBLE` / `GRASP_NOT_ACQUIRED` termination.

### Bounded reach correction
- Kept the close-camera visual stop unchanged: `CAPTURE_TARGET_NY≈0.442`, visual nominal block radius 12.0 cm.
- Increased only the provisional longitudinal jaw/TCP correction from **+2.5 cm to +5.0 cm**:
  - nominal block-centre radius used for capture grasp: **17.0 cm**
  - fingertip/TCP radius: **17.5 cm**
- 17.5 cm remains below the existing 18.0 cm calibrated-envelope hard edge and leaves **4.5 cm** margin from the 22.0 cm staging radius.
- Offline IK verifies a continuous radial descent at 17.5 cm through z=4,3,2,1,0 cm; final grasp is approximately `{3:1085,4:1843,5:2485}`, all within the MasterPi PWM envelope.

### Contact sweep used to choose 17.5 rather than 18.0 cm
- In the current nominal V2 contact model, a 15.0 cm reach loses bilateral contact once the cube centre is around 16 cm forward.
- 16.5/17.0 cm reaches extend stable bilateral contact to around 17 cm.
- **17.5 cm** provides stable final bilateral contact through roughly an 18 cm cube-centre location without non-finger collision in the tested sweep.
- 18.0 cm reaches farther but becomes marginal at ~19 cm: bilateral contact can occur transiently and then fail to remain bilateral. Therefore 17.5 cm is the bounded next physical probe, not 18.0 cm.
- This remains provisional physical tuning, not completed hand-eye calibration.

### Verification
- Focused REAL/failure-gate/search/tool-hardening/SIM-servo suite: **96/96 passed**.
- Final full repository regression: **506/506 passed in 100.704 s**.
- Current red-block package signature: `ea924fa7301b0f63af611864d3da8a55ecb694b519598ad417d7c02680b61d69`.
- No physical actuator action was issued while making/testing this correction. The content-addressed bundle will sync on the next actual REAL skill invocation.

## 2026-08-30 — Merged SIM/REAL dashboard lag traced to inactive-stream work and MJPEG disconnect leakage

### Observed runtime pressure
- The merged dashboard intentionally serves SIM and REAL from one page, but the inactive side was not actually quiescent.
- During the reported lag, the REAL harness was around 22–27% CPU, the host had about 7.7–7.8 GiB of 11 GiB RAM in use and ~1.5 GiB swap in use, and several Chromium renderers were holding hundreds of MiB each. This means host/browser memory pressure can amplify UI latency even when the robotics services themselves are not CPU-saturated.
- The four SIM CCTV `<img>` elements kept direct MJPEG `src` attributes even after their panels were hidden in REAL mode. CSS hiding does not reliably tear down MJPEG networking/decoding. Each bridge stream is eligible to publish every 0.05 s (20 fps), so an inactive REAL view could retain four unnecessary SIM streams.
- `sim3d.js` also kept a requestAnimationFrame renderer plus polling alive in both modes: SIM state at 10 Hz, REAL radar state at 2 Hz, and rendering effectively at browser rAF rate.
- REAL's dashboard image loop requested the cached camera endpoint at 10 Hz. This is presentation-only; physical perception and the REAL skills use the independent low-latency camera pump / Pi skill camera path.

### UI suspension / rate fixes
- SIM CCTV images now keep their URLs in `data-stream-src` and are connected only while SIM is the active, visible browser mode. Entering REAL removes all four stream `src` attributes instead of merely hiding their panels.
- A browser `visibilitychange` handler now suspends camera display refresh, SIM CCTV streams, and the Three.js/radar worker when the tab is hidden, and resumes the appropriate active mode when visible again.
- REAL dashboard camera refresh was reduced from 10 fps to 4 fps (`100 ms -> 250 ms`) without changing the physical camera pump or robot skill sensor cadence.
- SIM state polling was reduced from 10 Hz to 4 Hz (`100 ms -> 250 ms`); REAL radar polling is now 1 Hz instead of 2 Hz.
- Three.js presentation is capped at ~30 fps in SIM and ~15 fps in REAL. REAL skips rendering the hidden SIM POV and renders only the semantic radar.
- Static asset cache key bumped to `sim3d.js?v=20260830perf1` so a normal page reload receives the optimized script.

### MJPEG server leak fix
- Before the fix, `sim.bridge` had 14 threads and four observer-stream sockets stuck in `CLOSE-WAIT` (plus another stale connection) after clients had gone away. When no GPU observer frame exists, the old stream loop performs no write and therefore never receives `BrokenPipeError`; disconnected handlers could sleep forever and accumulate across tab/mode changes.
- Added `socket_peer_closed()` using non-consuming socket readiness/peek. Every bridge MJPEG loop checks the peer even when there is no frame to write and terminates immediately after client FIN.
- The SIM bridge was restarted only after confirming GPU remote disconnected, queue depth 0, no inflight SIM command, and no active REAL action. This cleared the existing stale sockets; REAL service was not restarted or actuated.
- Live frameless-stream disconnect probe after restart: bridge stayed at 3 threads / 9 fds and `CLOSE-WAIT=0`, proving the no-frame disconnect path now cleans up.

### Verification
- HTML inline JS and `sim3d.js` pass `node --check`.
- UI/harness focused regression: **83/83 passed**.
- SIM bridge + UI/radar regression after socket fix: **28/28 passed**.
- Live `:8080` static readback contains lazy `data-stream-src`, the 250 ms REAL UI refresh, and cache key `20260830perf1`; no main/REAL service restart is required for these static changes.
- No physical motor/servo command was issued by this performance work.

## 2026-08-30 — Face-normal grasping restored as vision-closed-loop mecanum base placement

### Why this change
- The operator correctly pointed out that MasterPi should not try to compensate a rotated square block by pretending the gripper has a free wrist-roll/yaw DOF. The useful extra DOFs are in the mobile mecanum base: choose a good grasp face, move the chassis to a compatible approach bearing, visually re-check, and only then execute arm IK/grasp.
- Inspection showed that `physical_state_machine_reference.py` already contained this idea (`estimate_face_alignment()` + `align_block_face()`), but the active split `scripts/red_block/approach.py` had accidentally bypassed it and approached directly to the 22 cm capture-staging radius. The capability existed in reference code but was not on the live normal approach path.

### Active REAL path now
- `ColorBlob` already retains `cv2.minAreaRect` oriented `box_points` and `rectangularity`; those are used as the block-face observation rather than treating the target as only a centroid.
- The normal active approach is now explicitly:
  1. keep the selected runtime `target_color` visually locked and centre gaze;
  2. align the chassis under the gaze;
  3. approach only to `FACE_ALIGN_START_RADIUS_CM = 25.5 cm`, leaving room around the block;
  4. estimate the lower visible block-face edge by floor-projecting the oriented contour corners;
  5. if the block is diagonal, use bounded left/right mecanum translation to change **base position**, then re-face the block and re-detect/re-estimate after every pulse;
  6. proceed only when the face normal is inside the guarded alignment envelope;
  7. continue straight to `CAPTURE_STAGING_RADIUS_CM = 22.0 cm`;
  8. perform the existing visual pre-capture/capture correction, make the IK plan, then allow one guarded pick.
- A 45-degree synthetic diagonal case now exercises the intended behavior: lateral base motion is commanded, the target is re-faced, the face is re-measured, and the example converges to a 4-degree square error before continuing.
- If the face remains too diagonal after the bounded orbit budget, approach fails before pre-capture/IK/pick planning. It does not knowingly perform a corner/diagonal blind grasp.

### Relation to a future explicit base-pose planner
- Conceptually this implements the operator's `p_base = p_object - d*n` idea, but **not as a one-shot absolute world-coordinate command yet**. REAL base odometry/slip and hand-eye/world calibration are not sufficiently validated to justify blindly trusting one computed absolute base pose.
- The current implementation is therefore a safer visual closed loop: estimate face -> translate base -> re-face/re-detect -> estimate again. Once held-out odometry/hand-eye calibration passes, this can be promoted to explicit grasp candidates and scored absolute base-pose targets, with the current visual loop retained as final correction.
- The logic is runtime-object generic: red/blue/yellow are data (`target_color`) routed through the same manipulation pipeline rather than separate hard-coded grasp algorithms.

### Important near-look exception
- A target already recovered in the very-close `near_look` camera geometry still preserves the existing close path and does **not** attempt the 25.5 cm lateral orbit. Orbiting around an object at roughly the near capture distance could be unsafe without first backing away.
- Therefore do not claim that every possible entry path is face-normalized yet. A future guarded improvement should either prove the visible face is already acceptable in the close view or retreat/reacquire to the safe face-planning radius before lateral placement. No uncalibrated autonomous retreat was added in this pass.

### Verification / safety
- `tests/test_red_block_approach.py`: 24/24 passed, including ordering `body-align -> 25.5 cm staging -> face/base placement -> 22 cm staging -> capture`, and a hard gate that diagonal-alignment failure prevents capture/pick planning.
- Face/near-field subset: 22/22 passed, including face-normal projection, 45-degree diagonal detection, lateral base translation, and re-facing before re-measurement.
- Expanded REAL/object/harness regression: 184/184 passed; harness/state contract regression: 83/83 passed.
- Final repository regression after synchronizing the generic runtime-object test contracts: **525/525 passed in 102.615 s (`OK`)**.
- No REAL search/track/approach/pick/place, motor, chassis, arm, or servo action was invoked while implementing or testing this change. Physical face-placement performance remains unvalidated until the operator runs a controlled REAL trial.

## 2026-08-30 — REAL/SIM task policy unified to one source

### Architecture correction
- Removed the active SIM-only manipulation-policy fork. `scripts/red_block/search.py`, `track.py`, `approach.py`, `pick.py`, and `place.py` are now the task/control source of truth for both REAL and SIM.
- `sim/real_stack_adapter.py` is restricted to the environment boundary: MuJoCo-backed clock, camera, motor/servo transport, and post-action physical evaluation. In particular, SIM `pick` now calls `pick_mod.run_precision_pick(...)` directly and SIM `place` calls `place_mod.execute_place(...)` directly; the previous separately coded hover/descent/close/lift sequence was removed.
- MuJoCo bilateral contact/lift and stack geometry remain evaluator-only facts. They may reject a false apparent success, but they do not choose motion, add a retry, open the gripper, or grant the actor stronger positive grasp evidence than the physical MasterPi has.
- Removed active V2 SIM-only `_track_color`, `_approach_color`, `_pick_color`, `_carry`, `_navigate_to_memory`, and `_place_on` manipulation implementations. Runtime object colour is data passed as `target_color`/`destination_color` through the shared REAL functions.
- Standalone `carry` is no longer an active SIM tool, matching REAL: the shared `pick.py` already ends in the carry pose.

### Shared public contract
- `scripts/sim_actions.py` now imports `ACTIONS`, `ACTION_PARAMETERS`, and `CONTRACTS` from `scripts/robot_actions.py`; the SIM and REAL agent-facing catalogs are therefore the same source rather than duplicated dictionaries.
- Live `/api/tools` after service reload exposes the same 15 tools as REAL and contains no `carry` or `map_*` agent tool.
- `sim/bridge.py` and the Azure WebSocket worker now preserve typed runtime action parameters end-to-end instead of collapsing every command to an action name.
- Because REAL skills run in fresh Python interpreters while the SIM worker is persistent, the adapter restores any temporary colour-detector binding after each shared skill. This emulates process isolation only; it is not an alternate controller.

### Deployment / verification
- Deployed the corrected canonical GPU bundle to the Azure A10 worker and restarted only the SIM bridge/coworker. No REAL service or physical actuator was touched.
- Local and Azure SHA-256 hashes match for the updated adapter, production V2 worker path, WebSocket worker, and current REAL `pick.py`.
- Live Azure smoke after reset: `search` returned `REAL search.py confirmed red target on MuJoCo camera`; `track` returned `REAL track.py PID centered red target on MuJoCo camera`.
- The same live `approach` then failed the current shared REAL face-normal guard after 10 orbit pulses (`current error 34.5deg, best 26.6deg`). This failure was intentionally preserved: SIM no longer substitutes an easier policy merely to make the task pass.
- Added `tests/test_sim_real_shared_source.py` to lock catalog/contract parity and direct delegation for search/track/approach/pick/place, plus persistent-worker detector restoration and absence of the removed SIM manipulation policies.
- Final repository regression after the guard test: **526/526 passed in 119.569 s (`OK`)**.

## 2026-08-30 — Operational activation and immutable Pi package verified

- Fixed `scripts/activate_real_spatial_stack.sh` so its actuator-lease safety probe uses the same verified MasterPi SSH identity/route as the live camera/controller transport (`User=ugrp1`, Tailscale host `100.119.44.65`, `HostKeyAlias=ugrp1.local`). The previous bare `ssh ugrp1` resolved to the wrong login and falsely reported that the lease could not be verified.
- Kept the safety gate intact: activation still refuses to restart the REAL harness unless the Pi actuator lock can be acquired non-blockingly. No motor, servo, search, track, approach, pick, place, or put-down command is used by activation.
- Replaced the brittle fixed 2 s startup sleep with an `/api/status` readiness loop. The REAL harness normally becomes HTTP-ready about 4–6 s after systemd reports it active because camera/SSH initialization happens first.
- REAL harness was reloaded through the guarded script and is serving the generic object API: `search/track/approach/pick(target_color)`, `place(target_color,destination_color)`, plus guarded `put_down`.
- Final immutable MasterPi package signature: `222c5c2e8c1b84a315df47da9113ca3fb2817afa36e89c5e205532539dc5026c`. The Pi copy passed the complete `.ugrp-files.sha256` manifest and Python compilation, and its `approach.py` contains the active face-normal mecanum base-placement stage.
- `put_down.py` was added to the shared GPU recovery bundle and `sim/masterpi_scene_v2.xml` was re-synchronized from the runtime `masterpi_dynamics_v2.XML`, restoring SIM/recovery single-source consistency.
- Final repository regression on the state that produced the deployed package: **535/535 passed in 96.852 s (`OK`)**. A second REAL-critical run on the exact stable package signature passed **192/192** and the package signature was unchanged before/after that run.
- Final verification found no active physical manipulation child on the Pi. This work validates deployment/integration only; it does not claim a successful physical grasp trial.

## 2026-08-30 — REAL wrong-object carry reconciliation and safe put-down

### Incident / live evidence
- A live REAL session became stuck in `grasp.state=PROBABLE_HELD` even though the operator could see that the gripper was empty. Subsequent `search/track/approach/pick` requests were rejected with `OBJECT_ALREADY_CARRIED`.
- The operator statement `너 손에 빨간 블럭 없는데` was previously conversation-only: the assistant could acknowledge the image, but `StateEstimator.grasp` remained unchanged, so the next manipulation request immediately hit the stale carry gate again.
- During the same incident, compatibility `place_on_blue` attempts never reached physical placement: the running REAL process invoked `place.py` without the required `--destination-color`, producing an argparse failure. The on-disk alias adapter already contained the corrected arguments, but `ugrp-real.service` had loaded older modules before those files were changed.
- The resulting contradictory state caused repeated placement/recovery attempts, then downstream planner/protocol symptoms such as `no JSON object in model output` and stale `plan expects ...` mismatches. These were secondary effects, not the primary physical-control fault.

### Deterministic wrong-object recovery
- Added public `put_down` as a shared REAL/SIM action. When the current goal is to GRASP/LIFT a known colour and a different colour is already `PROBABLE_HELD|HELD`, `TaskExecutive` now compiles the task deterministically as:
  `put_down -> search(requested) -> track(requested) -> approach(requested) -> pick(requested)`.
- Example locked by regression: red held + `파란 블럭 집어봐` becomes `put_down`, then the normal blue generic manipulation pipeline. The LLM is not asked to invent a recovery plan, so this path cannot oscillate between `place_on_blue` and acquisition tools.
- `put_down` is permitted only while a grasp may actually be present. Normal `search/track/approach/pick` remain blocked while carrying, preserving the existing no-double-pick safety boundary.

### Safe physical put-down contract
- `put_down` is not implemented as an arbitrary mid-air gripper-open action. A successful precision `pick` now stores a short-lived causal return path containing the exact original base-yaw pulse, hover arm pose, and grasp-descent waypoints used at the pickup site.
- Immediate wrong-object recovery reverses that saved path: carry pose -> original base yaw -> original hover -> original descent -> open gripper -> retreat through the reversed descent -> hover.
- If the chassis moves after the pick, the pickup-site return path is revoked because REAL has no sufficiently validated world odometry to guarantee that the old pickup coordinates are still below the gripper. Carry evidence itself is preserved so a separately requested guarded placement can still use it. `put_down` then refuses rather than guessing a floor drop.
- Older carry handoffs that predate the saved return path remain valid for guarded placement but cannot authorize autonomous `put_down`.

### Stale-grasp reconciliation
- Added a deterministic operator correction path for unambiguous empty-gripper statements such as `너 손에 빨간 블럭 없는데`, `집게에 아무것도 없어`, and `집게 비었어`.
- Such a statement performs no actuator/tool call. It may clear weak/unknown carry evidence (`PROBABLE_HELD` etc.) to `EMPTY`, but plain text does not override a future stronger `HELD` state backed by positive sensing.
- Live REAL verification after service reload: posting `너 손에 빨간 블럭 없는데` produced no tools, no execution intent, returned `알겠어. 현재 집게는 빈 상태로 정정했어.`, and changed grasp state to `EMPTY` with `held_object_color=null`.

### Alias / runtime corrections
- `place_on_blue` is regression-locked to supply `--target-color red --destination-color blue`; the corresponding yellow alias remains equivalent. This prevents the argparse failure seen in the incident.
- The hard-coded `red-target acquisition/manipulation...` rejection text was made colour-neutral.
- REAL, SIM bridge, and SIM coworker services were restarted after the code changes so the long-lived harnesses no longer retain stale imported action/state/executive modules.

### SIM parity / deployment
- SIM continues to use the same manipulation-policy source as REAL. `sim/real_stack_adapter.py` delegates `put_down` to the shared REAL `scripts/red_block/put_down.py`; MuJoCo is used only to evaluate the post-release physical condition.
- Added `put_down.py` to the canonical GPU worker bundle. Before deployment the local and Azure production bundle digests differed; the current bundle was deployed to the active Azure GPU worker, the remote file was verified present, and the local/remote bundle digests then matched.
- After bridge reload the Azure worker reconnected as the authoritative GPU worker with queue depth 0 and no inflight command.

### Verification / scope
- Affected REAL/harness/SIM regression set: **84/84 passed**. This includes wrong-colour deterministic recovery, state clearing after verified release, operator reconciliation, safe reverse-path bookkeeping, chassis invalidation of only the return path, compatibility alias arguments, SIM/REAL shared-source delegation, bridge delivery, transport recovery, and SIM servo behavior.
- Live `/api/tools` for both REAL and SIM exposes the same **16 tools** and includes `put_down`; both services came up without post-restart error/traceback entries during the readback check.
- REAL stale state was cleared by service reload (`PROBABLE_HELD` no longer persisted across the old process), and the explicit empty-gripper correction was live-verified without actuator execution.
- **No physical REAL search/track/approach/pick/place/put_down was executed as part of this repair.** The software control path and safety gates are verified, but the new reverse-path `put_down` motion itself still requires one controlled physical observation before it can be described as physically validated.

## 2026-08-30 — REAL face-normal base-placement oscillation fixed after physical trial

- A live operator trial exposed a real base-placement failure: the face estimator stayed near a diagonal square (`~39–47 deg` error) while `approach` repeatedly alternated left/right 0.20 s mecanum pulses. After 10 pulses it still reported `block remains diagonal`; this was the abnormal motion seen by the operator.
- Root cause 1: the initial tangent-motion sign was inverted. In the active floor-edge coordinate convention, a negative diagonal edge (for example `-45 deg`) must move the base right and re-face the object so the visible edge progresses toward `-90 deg`; a positive edge is the mirror case and moves left.
- Root cause 2: the controller reversed direction whenever one sample was only 1.5 deg worse than the previous sample. Real min-area-rectangle/floor projection estimates jitter by several degrees, so measurement noise caused left/right ping-pong and almost zero accumulated viewpoint change.
- `align_block_face()` now keeps a sticky tangent direction, accumulates real base displacement, re-faces/re-measures after every stopped pulse, and permits at most one direction correction only after at least three pulses and an unmistakably bad trend (>=8 deg worse with no meaningful best-error progress). Existing 12 deg ideal / 18 deg guarded continuation gates remain unchanged.
- Added regressions for both edge signs, noisy measurements that must not bounce, and the single robust-reversal case.
- The web Cancel path was also hardened because the same physical trial showed `/api/cancel` stopped the harness queue but left the already-running Pi skill alive. The actions registry now retains the cancellation hook from the exact loaded module; `robot_actions.cancel_current()` terminates only the active action's private local process group; and the remote watchdog converts SSH HUP/TERM/INT into termination of its own skill process group. A local integration test confirms watchdog SIGTERM removes the child skill process.
- Verification: REAL-focused suite **267/267 passed**; full repository suite **540/540 passed in 86.911 s (`OK`)**.
- Deployed immutable Pi package: `e9f80fef3e24c478231c5d51b0cceeae1eeb06cc8d5c49c1fc9676e9ff9568d2`. Pi SHA manifest and Python compilation passed; the reloaded REAL harness reports the generic runtime-color tools and has the cancellation hook loaded.
- No autonomous physical success claim is made from these tests. The next operator trial should show sustained motion in one chosen orbit direction rather than left/right oscillation; physical convergence still needs to be observed on the MasterPi.

## 2026-08-30 — REAL face-placement oscillation and cancellation propagation fixed

- Live trace review showed the diagonal-block face-placement loop was making short tangential pulses while reversing direction on noisy per-frame rectangle angles, producing left/right oscillation with little net viewpoint change.
- `align_block_face` now keeps the initial orbit direction through ordinary vision jitter, re-faces/re-measures after every stopped mecanum pulse, and allows at most one reversal only after a sustained, materially worse trend. The signed edge direction is covered for both mirror cases.
- The web `/api/cancel` path now propagates beyond the executive loop into the currently running action subprocess. Physical actions run in a private local process group; cancellation terminates only that group. The Pi `remote_watchdog` converts SSH HUP/TERM into termination of its guarded remote skill group so a cancelled request cannot continue independently on the robot.
- REAL-critical regression: 210/210 passed. Full repository regression: 543/543 passed.
- Immutable Pi package verified/deployed without running a skill: `ffb936ca035a581c7b27e78c48260ffa750dd2421d2e993928f11e9675d7f805`.
- Oracle `ugrp-real.service` was restarted only after verifying the MasterPi actuator lease was free. No live physical manipulation was executed during this fix.

## 2026-08-30 — Diamond re-face ping-pong and destination-only placement routing fixed

- REAL trace from the operator's diagonal/diamond-block run showed the tangential face-orbit itself stayed on one side, but the nested chassis re-face loop repeatedly crossed the target bearing and reversed yaw (`rotate-right -> rotate-left -> ...`, observed up to 18 pulses). The cause was `align_body_to_gaze()` calling `motion_pulse_with_gaze()` after every turn pulse: that helper immediately ran the camera-pan tracker, so pan compensation and chassis yaw fought over the same bearing error.
- `scripts/red_block/physical_state_machine_reference.py` now keeps camera pan fixed during chassis yaw absorption, chooses one chassis turn direction per re-face, uses a shorter 0.06 s turn pulse inside 10 degrees, accepts a small guarded centre crossing without an opposite pulse, and fails safely on a large crossing instead of starting left/right ping-pong. A face-orbit attempt also keeps its initially selected lateral side for the entire bounded attempt; the former one-time opposite-side reversal was removed.
- The failed follow-up `파란색 블럭 위에 올려봐` was a deterministic parser bug, not a missing place skill. With one color before `위에`, `harness/goals.py` previously treated blue as the source and left destination unknown. While `PROBABLE_HELD red`, that produced no deterministic place plan, fell through to the model, and the model attempted `search`, which the executive correctly rejected as `OBJECT_ALREADY_CARRIED`.
- A single color in Korean destination-only placement phrasing is now the support/destination: `PLACE(subject=UNKNOWN, target=BLUE_BLOCK)`. When grasp state says red is already carried, the executive resolves the source from state and emits only `place(target_color="red", destination_color="blue")`; it never restarts search/pick.
- Added exact regressions for the operator phrase, for no-search follow-up placement while carrying, for one-direction body re-face after a small zero crossing, and for never crossing back to the opposite face-orbit side.
- Guarded REAL reload now preserves only one-shot conservative carry evidence (`PROBABLE_HELD|HELD` + held color) across the Oracle service restart. This prevents a code reload from forgetting a possibly occupied gripper and authorizing a new acquisition. The snapshot is consumed once by the new REAL process and removed; ordinary planner/vision state is not persisted.
- Verification: targeted routing/control suite 61/61 passed; REAL-critical suite 218/218 passed; reload-safety targeted suite 135/135 passed; final full repository suite 547/547 passed.
- Immutable Pi package deployed and manifest-verified at `b9ee07cb49f0cc122e84e275f0baa2d7c4bd2e2feb0c9aec435df3f1c22b04e9`; remote `py_compile` passed for the face-align/approach/place/watchdog paths.
- REAL service was reloaded only after confirming no active physical skill and a free actuator lease. Pre-reload `PROBABLE_HELD red` survived the reload unchanged. A service-level `execute=false` probe of `파란색 블럭 위에 올려봐` set `task.destination_color=blue` while keeping `PROBABLE_HELD red`; no physical skill process started and `/api/cancel` reported `action_terminated=false`.
- No physical manipulation was initiated as part of this fix/deployment/verification.

## 2026-08-30 — Stale PROBABLE_HELD no longer deadlocks a new REAL pick

- Operator reproduced `OBJECT_ALREADY_CARRIED` twice with the exact request `빨간색 블럭 집어봐`. REAL state still contained `PROBABLE_HELD red` from an earlier weak camera-only pick inference, so the strict executive rejected `search` before any physical skill started.
- Read-only Pi inspection found the causal precision-pick handoff at `/tmp/ugrp-red-carry-handoff.json`, but it was about 1557–1996 seconds old. The Pi-side carry contract has `CARRY_MAX_AGE_SECONDS = 600`, so Oracle was preserving a weak belief long after its causal evidence had expired.
- Added `harness.real_carry`: a read-only SSH verifier for the Pi precision-pick handoff. `PROBABLE_HELD` is retained only while the handoff is known, fresh, same-colour, and contains a closed-gripper command. Known stale/missing/invalid evidence downgrades only the weak belief to `UNKNOWN`; positive `HELD` is never auto-cleared. SSH/transport uncertainty also never clears carry (fail closed).
- Every REAL executable turn now reconciles weak carry evidence before executive preconditions. `observe_scene` reports the same carry evidence, and the guarded activation script preserves `PROBABLE_HELD` across a service restart only when its Pi causal handoff is still fresh.
- Exact-request web regressions verify stale evidence is cleared before any motion and fresh evidence remains blocked. Relevant REAL suite: 230/230 passed. Full repository: 557/558 passed; the only failure is an unrelated SIM eye-in-hand camera-offset test (`[0.067,0,0.0136]` current model vs `[0.070,0,0.025]` old expected), independently reproducible and tied to a SIM XML modified before this REAL fix.
- No Pi manipulation code changed; immutable Pi package signature remains `b9ee07cb49f0cc122e84e275f0baa2d7c4bd2e2feb0c9aec435df3f1c22b04e9`.
- Guarded Oracle REAL reload was performed only with no active physical skill and a free actuator lease. Because the Pi carry handoff was stale, the new process correctly started with `grasp.state=UNKNOWN`, `held_object_color=null`. Camera returned to ~24 ms age and the actuator lease remained free.
- No physical search/track/approach/pick/place/put_down was executed during this repair.


## 2026-08-30 — MasterPi control-latency PR staged separately, not applied to live source

- Prepared an isolated PR staging checkout at `/home/ubuntu/worktrees/ugrp-control-latency-pr` because this live project directory is intentionally not a Git repository and must not be initialized as one.
- Local PR branch: `perf/masterpi-control-latency`; commit: `7a9b999` (`perf: reduce MasterPi terminal control latency`).
- Review artifacts are in `work/prs/masterpi-control-latency/`: `PR_BODY.md` and `0001-perf-reduce-MasterPi-terminal-control-latency.patch`.
- Scope: adaptive error-banded final-x servo correction plus threshold-preserving early exit for pre/post grasp visual verification. The live controller source in `/home/ubuntu/projects/ugrp` has **not** been changed by this PR staging work.
- Focused REAL/SIM controller regression passed 73/73. Full staging regression ran 563 tests with one failure that is independently reproducible on the untouched live checkout: the existing eye-in-hand SIM camera mount geometry test expects an older offset.
- No physical actuator action, deployment, or service restart was performed.

## 2026-08-30 — Stacked PR prepared for bounded hybrid continuous approach

- Prepared, but did not apply/deploy, stacked PR branch `feat/masterpi-hybrid-visual-servo` in isolated staging checkout `/home/ubuntu/worktrees/ugrp-control-latency-pr`.
- Base is the prior latency PR commit `7a9b999`; new commit is `3e541fb feat: add hybrid continuous visual servo`.
- The proposed controller uses observe-while-moving only at >=38 cm, stops at/before the 36 cm coarse boundary or on vision/range/pan/time uncertainty, and preserves the existing stopped controller below 38 cm including 25.5 cm face planning and 22 cm capture staging.
- Focused staged regression passed 78/78; full staged regression passed 567/568 with only the already-known unrelated SIM camera-mount expectation mismatch.
- No REAL/SIM production controller file, service, GPU worker, Pi package, motor, or servo was changed by preparing this PR.
- Review artifacts are under `work/prs/masterpi-hybrid-visual-servo/`.

## 2026-08-30 — REAL close-target yaw loss and coarse-staging overshoot fixed
- Latest physical traces showed red at the bottom of the eye-in-hand image (`ny≈0.87–0.89`) and the controller performed chassis yaw before creating clearance. A minimum-speed `rotate-right` then pushed the target out of view, causing repeated `track handoff lost` / local-recovery failures before face alignment.
- `approach.py` now enforces `distance/visibility clearance -> camera centring -> chassis yaw`. A close near-look handoff no longer bypasses face alignment: it retreats straight, restores search geometry, reacquires, and joins the common 25.5 cm face-normal pipeline.
- `physical_state_machine_reference.py` adds bounded straight-backward `retreat_for_body_alignment()`. It measures image-y progress with pan/tilt frozen and refuses yaw if retreat does not create clearance. Automatic pan-limit body realignment inside `approach_with_locked_gaze()` uses the same clearance gate.
- Coarse metric staging is now a radius band rather than the old one-sided `radius <= target => arrived` rule. If the robot is already inside the requested 25.5/22 cm staging radius it performs bounded backward pulses and verifies range increase; it no longer treats an overshoot such as 18.5 cm at a 22 cm target as success.
- The earlier same-session fixes remain: MJPEG EOF reconnect is bounded and transparent; face-orbit progress is verified from frozen-pan target-bearing change rather than trusting the lateral command or minAreaRect trend alone.
- Regression: modified REAL failure-gate tests 43/43, red-block approach 24/24, real tool-hardening 23/23, REAL-critical combined suite 250/250.
- Immutable Pi package `89b0083b42d847fa3d89670dc8b3036148a6a08ee89890d4fba5d3533761eb56` was deployed with `_ensure_deployed(...)=True`; that function validates the remote sha256 manifest before returning success. No live manipulation was initiated by this repair session.
- After deployment the Pi became unreachable from Oracle: Tailscale ping and SSH to `100.119.44.65` both timed out and the Oracle camera frame stopped advancing. This is currently a transport/device-availability blocker, separate from the manipulation-controller fixes above.

## 2026-08-30 — REAL per-request causal trace + automatic RCA system completed in source

- The previous REAL recorder was only partially wired: `recorder.py` could emit camera/detection/pose events, but production `deploy_and_run()` did not enable it, `outputs/real_traces/` received no physical runs, and the single legacy `/tmp/ugrp1-pick-red-debug.jpg` could be overwritten by a later failure. Cross-layer correlation depended on timestamps/PIDs and manual journal reconstruction.
- Added a thread-safe top-level REAL `run_id` using `contextvars` in `harness/execution_trace.py`. Every REAL executable agent turn now has one durable directory containing the original user command, initial world state, planner/executive timeline, exact planner input images, post-action observations, current streamed commanded pose metadata, final result, and child skill spans.
- Each physical skill gets a unique `span_id` under the same `run_id`. `scripts/red_block/deploy.py` now explicitly enables Pi-side tracing, captures live stdout/stderr while preserving current UI diagnostics, copies `/tmp/ugrp-real-traces/<run>/<span>` back to Oracle, writes immutable per-span metadata/logs, and deletes only a successfully copied remote span. Trace I/O/copy failures are non-fatal to robot control.
- `scripts/red_block/recorder.py` schema is now v2 with run/span/event IDs. REAL production enables `UGRP_REAL_TRACE_ALL_FRAMES=1`: every camera frame actually consumed by a controller decision is JPEG-preserved and tagged with the current commanded PWM pose/age. Detections retain `frame_seq`, so a detection can be traced to its exact input image. JPEG-save failures are recorded instead of being indistinguishable from intentional sampling.
- The precision approach controller's private `LiveVideo` path was separately instrumented. This closes the previous observability gap around face alignment, pre-capture, fixed capture, and grasp preparation; those consumed MJPEG frames now pass through the same recorder.
- `harness.loop` exposes the exact image sent to each model call through `on_planner_input`; the REAL web harness copies that exact file into the run and records the compact world state and streamed commanded pose for the same planner call. This distinguishes “what the UI camera showed” from “what the model actually received”.
- Added `harness/trace_analysis.py` and `scripts/analyze_real_trace.py`. Each finished run automatically gets `analysis.json` + `analysis.md` with the first failure, original failure reason/code, last detection, last commanded pose, last actuator command, previous/current input frames, controller decision tail, and a detected `visible -> lost` transition with exact before/after frames and poses. A run that recovers from an intermediate error is labeled `COMPLETED_WITH_FAILURES` rather than falsely collapsed to `FAILED`.
- Failure debug images are now copied to each immutable skill span (`debug-failure.jpg`) while retaining the legacy latest-debug file for compatibility, so later failures cannot destroy the only evidence for an earlier one.
- Truth boundary remains explicit: servo pose is commanded PWM state because this MasterPi stack has no independent joint encoder feedback; chassis motion is command/timing evidence because validated wheel odometry is not available. The RCA system must not convert those into measured joint angles or traveled centimetres.
- Documentation: `docs/real_trace_system.md`; manual analysis is `.venv-sim/bin/python scripts/analyze_real_trace.py latest` or a specific run ID.
- Verification so far: planner-image/run-context targeted tests passed; harness/intent/executive regression **101/101 passed** before the added planner callback regression; REAL recorder/RCA/tool-hardening/failure-gate/place regression **104/104 passed**; post-hardening recorder/RCA/tool-hardening regression **29/29 passed**. A final full-suite result is recorded below after completion.
- **No physical MasterPi motion and no REAL service restart was performed while building this observability system.** The long-running service therefore does not automatically contain the new source yet. Safe activation requires a deliberate `systemctl --user restart ugrp-real.service` only after confirming no physical skill/actuator lease is active. The next physical request after that reload will also deploy the updated immutable Pi package automatically via the existing content-signature path.
- Final full repository regression after all trace changes: **570/571 passed in 86.887 s**. The sole failure is the pre-existing unrelated SIM eye-in-hand camera mount expectation mismatch: current canonical local offset `[0.067, 0, 0.0136]` vs old test expectation `[0.070, 0, 0.025]`, the same failure already independently recorded before this observability work. No trace/RCA, REAL control, harness, planner-input, recorder, deploy-wrapper, or cancellation regression failed.
- Live activation check after implementation: `scripts/activate_real_spatial_stack.sh --check-only` returned **255 / could not verify MasterPi actuator lease** because the Pi SSH path was not reachable. The current Oracle harness camera cache was also stale by ~456 s and `talk_only=true`; no physical skill subprocess was active locally. Per the fail-closed activation contract, the REAL service was **not forcibly restarted** and no actuator command was sent. Source is complete but the long-running service may still have modules loaded from its 09:21 UTC start; perform the guarded activation only after Pi connectivity returns and lease verification succeeds.

### 2026-08-30 — PR staging note: single-process grasp task runner

- Prepared stacked PR commit `dabca61` (`feat: run grasp pipeline in one Pi task process`) in the isolated staging checkout; canonical source files were not replaced or deployed.
- The proposed `grasp(target_color)` path executes `search -> track -> approach -> pick` inside one fresh Pi process with one `Robot`/actuator lease, one hardware probe, and one shared fresh MJPEG stream while directly calling the existing REAL skill functions.
- Existing individual skills remain available as a legacy/debug fallback. Wrong-object `put_down` recovery remains intact. Recoverable target loss may use one bounded search recovery; strict REAL still refuses an automatic second grasp after a visually proven miss.
- REAL/SIM actor evidence remains conservative (`PROBABLE_HELD`, never invented `HELD`), and SIM invokes the same task runner through the hardware adapter.
- Validation in staging: focused 143/143 PASS; full suite 577/578 PASS with only the known pre-existing eye-in-hand camera mount expectation mismatch.
- No physical MasterPi deployment or actuation was performed for this PR draft.

## 2026-08-30 — REAL trace system extended with dashboard RCA, safe evidence API, and self-overhead telemetry

- Added read-only forensic endpoints to the REAL backend: `/api/real-traces`, `/api/real-traces/<run_id>`, and image-only `/api/real-traces/<run_id>/asset/<relative-path>`. Run IDs/asset paths are resolved underneath `outputs/real_traces` only; absolute paths, `..` traversal, non-image evidence files, and unknown runs are rejected. Regression explicitly verifies encoded `%2e%2e/run.json` returns 404.
- The shared dashboard now has a REAL-only `최근 REAL 실행 분석` panel. It shows recent run status, user command, failure reason, deterministic RCA class/confidence, last-success→first-failure boundary, latest evidence frames, pose/actuator/detection evidence, controller decision tail, and trace overhead. The static dashboard already reads the new UI from disk; the live REAL backend route will become available after the guarded REAL service reload.
- Automatic RCA now correlates the exact commands between the last `visible=true` and first paired `visible=false` observations. It distinguishes target loss after arm pose changes from loss after chassis motion and from loss with no recorded actuation, while explicitly keeping the physical mechanism (slip/stall/occlusion/object displacement) unclaimed unless directly sensed. Additional classes cover camera transport, carry-state conflict, causal handoff invalidity, implausible range change, face alignment non-convergence, camera alignment limits, and visually verified grasp misses.
- The analyzer now records `failure_boundary` (`last_successful_skill -> first_failed_skill`) and sorts spans by physical execution start time before selecting the first failed span. Recovered runs still retain intermediate failure evidence under `COMPLETED_WITH_FAILURES`.
- Added observability-of-observability. Pi `recorder.py` measures JPEG-save, pose-read, JSONL-write, and total recorder cost; successful/normal-exception process shutdown emits an aggregate `recorder_summary`. The Oracle deploy wrapper separately records `execution_duration_s` and post-skill `trace_copy_duration_s`, so trace-copy delay is not confused with physical controller runtime.
- Oracle-only microbenchmark (not a Pi validation): 120 random 640x480 BGR frames at JPEG quality 55 measured ~2.215 ms raw JPEG encode and ~2.707 ms total recorder per frame on this Oracle VM, leaving ~0.492 ms for pose-read/event/file overhead. A separate process-exit check confirmed `recorder_summary` is emitted by `atexit`. Actual Raspberry Pi overhead remains unmeasured until the Pi returns online and a real skill runs.
- Focused REAL trace/web/tool-hardening regression after these changes: **51/51 passed**. Final repository regression: **574/574 passed in 97.248 s (`OK`)**. Inline dashboard JavaScript also passes `node --check`.
- Final immutable Pi red-block package content signature from current source: `109897f01cb8181038c08470b0d8fc3d8184161c23199e90b82d1dee1fcdcd18`.
- Test-generated standalone trace directories were removed after verification; `outputs/real_traces/` is clean and waiting for the first real run under the new schema.
- Live activation remains intentionally fail-closed. Oracle Tailscale self is online, but peer `ugrp1` (`100.119.44.65`) reports `Online=false`, `LastSeen=2026-08-30T09:20:00.1Z`; `tailscale ping` times out and TCP/22 remains SYN-SENT. Therefore the MasterPi actuator lease cannot be verified and `scripts/activate_real_spatial_stack.sh --check-only` cannot authorize a restart. No forced REAL reload, package deployment, or physical actuator command was performed while the peer is offline.
- Attempted a second recovery path through the connected MacBook environment to see whether MasterPi was reachable on local LAN/mDNS even while Tailscale was offline. The MacBook connector itself returned a network connection failure before any host command could run, so no local-LAN claim can be made from that path. Oracle remains the only verified view: MasterPi Tailscale peer offline and lease unverifiable.

## 2026-08-30 — SIM far/close overlap boundary miss and destructive regrasp divergence fixed

- Reproduced the browser sequence exactly on Azure seed 11: `search -> track -> approach -> pick(fail) -> search -> track -> approach(fail)`.
- The second `approach` did not lose the target. After reaching the 22 cm coarse staging band, horizontal alignment stayed centered (`nx` ending around **0.494**) and the pre-capture vertical coordinate progressed monotonically **0.417 -> 0.438 -> 0.458 -> 0.481 -> 0.504 -> 0.527 -> 0.552 -> 0.577 -> 0.602 -> 0.629 -> 0.658 -> 0.688 -> 0.717**.
- The old handoff gate required exactly `ny >= 0.720` with at most 12 forward pulses, so a healthy trajectory failed by only **0.003** at the final bounded sample. The close camera is already visible in the measured-camera overlap at substantially larger range, so another forward pulse is not required for safety.
- Added `PRECAPTURE_TARGET_TOLERANCE_NY = 0.01`; the far-to-close visibility handoff now accepts `ny >= 0.710`. A regression reproduces the observed **0.717 / 0.720** boundary and verifies that the controller switches camera poses without another chassis pulse.
- The repeated acquisition after the first failed pick was a separate harness-policy divergence: non-strict SIM deliberately allowed one whole-task retry after `GRASP_NOT_ACQUIRED`, whereas strict REAL stopped after the first destructive miss. That retry changes the object/base initial condition because a missed grasp can push/reorient the block, so it is invalid for REAL-parity task execution. SIM now follows the same rule as REAL: a visually/physically proven grasp miss terminates the current user turn; robustness experiments must reset/replay an episode explicitly.
- Focused boundary/retry regressions: **4/4 passed**. Related REAL/SIM controller suite: **110/110 passed**. SIM servo parity: **2/2 passed**. Full repository regression: **575/575 passed in 88.990 s (`OK`)**.
- This does **not** claim the grasp itself is fixed. The separate provisional 17.5 cm final TCP/gripper geometry mismatch remains under investigation. No REAL robot actuator was commanded by this repair.

### Follow-up — measured close-camera capture needed one more bounded creep

- After the far/close overlap fix, a fresh seed-11 browser run advanced one stage further and exposed the next fixed-budget boundary: close-camera `ny` progressed monotonically **0.135 -> 0.160 -> 0.192 -> 0.244 -> 0.298 -> 0.354 -> 0.406** with `nx≈0.497`, but the safe capture-window lower edge is **0.417** (`0.442 - 0.025`). The six-creep budget therefore stopped **0.011** short while vision remained centered and improving.
- `CAPTURE_MAX_CREEP_PULSES` is now **7**. The extra pulse remains inside the existing safety checks: every sample rejects excessive x drift, backwards visual progress, and `ny > 0.505` before blind descent. A replay regression verifies `0.406 -> 0.458` enters the calibrated target window and immediately stops.
- REAL failure-gate suite: **45/45 passed**. Final repository regression after the extra bounded capture creep: **576/576 passed in 91.113 s (`OK`)**.
- This change only extends the bounded image-servo opportunity to reach the existing physical capture window; it does not alter the capture target, grasp depth, provisional 17.5 cm TCP plan, or success criteria.

## 2026-08-30 — SIM red-pick physical drop + false verifier failure fixed
- Reproduced the exact browser command `빨간 블럭 집어봐` through the Azure A10 bridge with the shared REAL `search -> track -> approach -> pick` source.
- Phase-level MuJoCo diagnostics showed that the gripper did obtain symmetric bilateral contact and began lifting, but the old nominal target cube was 50 g. For a 30 mm cube that implies ~1850 kg/m^3, incompatible with the lightweight foam/sponge-style Hiwonder task target. Mass sweeps isolated the effect: 50 g and 20 g dropped; 10 g and below held. The structural-only SIM now uses `PROVISIONAL_TARGET_BLOCK_MASS_KG = 0.005` and remains explicitly uncalibrated/`validated=false` until the user's physical cube is weighed.
- Friction-only sweeps (`mu=3.4..8`) and gripper stiffness sweeps did not fix the old 50 g drop, so the repair does not hide the problem by artificially increasing finger friction.
- Found a second independent false-negative: the post-grasp verifier treated any visible red component as a failed floor grasp. A physically held cube can remain visible to the eye-in-hand camera. Verification now counts only detections geometrically compatible with the floor plane from the saved verification pose. A known floor miss projects to a valid floor point (~12.6 cm); a known held cube is incompatible with the floor projection and is ignored by the miss detector.
- Exact browser seed 11 result after deployment: `search ok`, `track ok`, `approach ok`, `pick ok`; remote reason: `MuJoCo evaluator confirmed the physical grasp`.
- Robustness replay after the repair: seeds 11, 12, 13, 14, 15 all complete `search -> track -> approach -> pick`, set `grasp_color=red`, and end with the red cube centre at ~24.3-24.6 cm.
- Full local regression after updating the old 50 g collision-test assumption: **580/580 tests passed** in 86.826 s.
- The deterministic final chat wording remains conservative (`PROBABLE_HELD`) because the agent is intentionally restricted to REAL-equivalent sensor evidence; privileged MuJoCo ground truth is used only as a SIM evaluator, not as actor evidence.
- No REAL robot actuator was commanded during this diagnosis or repair.

### 2026-08-30 — MasterPi chassis silhouette rebuilt from manufacturer dimensions
- Re-audited the V2 body against Hiwonder's official MasterPi 185×162×343 mm dimension drawing and product/assembly photos.
- Corrected a geometry interpretation bug: the drawing's 101 mm chassis/electronics height had been treated as the structural deck height, then a 30 mm solid `rear_cage` was stacked above it. The resulting visible electronics body reached ~126 mm from the floor and looked substantially too bulky.
- Rebuilt the mobile base as thin sheet-metal panels around an invisible stable collision proxy: nominal central frame 140×100 mm, lower metal body 36–70 mm from the floor, structural deck top 74 mm.
- Replaced the solid 76×96×30 mm rear cage with an open Pi/electronics frame (thin side rails, top plate, diagonal braces, visible Pi board/heatsink) constrained below the 101 mm manufacturer body-height envelope. The collision proxy remains invisible so appearance and stable contact geometry are no longer conflated.
- Kept the official 65 mm wheels and 185×162 mm wheel envelope, and did not alter the REAL controller, arm FK/IK link lengths, servo mapping, measured camera profile, or gripper control.
- Added regression gates for official chassis height, inner-wheel body width, and the absence of the old solid `rear_cage` visual.
- Geometry/controller regression set: 86 tests passed before deployment. Physical dynamics calibration remains unvalidated; these chassis subdimensions are drawing/photo-derived nominal values pending direct caliper measurements.

## 2026-08-30 — MasterPi chassis morphology re-audited against manufacturer dimensions/assembly

- Re-audited the V2 chassis against Hiwonder's MasterPi dimension drawing and assembly sequence rather than preserving earlier photo-only box estimates.
- Manufacturer envelope remains **185 x 162 x 343 mm**, with **65 mm** mecanum wheels and a **101 mm** chassis/electronics silhouette before the arm.
- The matching Hiwonder orange mecanum wheel is **65 x 31 mm**. With the official 162 mm outside width this gives a **131 mm wheel-centre track** and a **100 mm clear inner width**.
- The lower sheet-metal body is now constrained to the **120 mm front/rear axle spacing x 100 mm inner-wheel width**. The previous 140 mm lower-body length overhung each wheel axis by about 10 mm and made the SIM body visibly too long/bulky.
- The Raspberry Pi/expansion-board stack now follows the assembly orientation: the roughly **85 mm** board axis runs laterally and the **56 mm** axis runs front-to-back. The cage footprint is kept within the axle-bounded body.
- Removed the visually misleading continuous upper cage side walls and full solid top slab. The visible electronics stack is now represented as board layers, four slim standoffs, and perimeter/cross rails approximating Hiwonder's perforated/open top bracket. A transparent collision proxy remains for stable contact.
- Added physical-geometry regressions for 65x31 mm wheels, 120 mm axle-bounded lower body, 56x85 mm Pi orientation/cage containment, and the absence of the old solid cage walls/lid.
- This is a **manufacturer-constrained structural/visual morphology correction**, not a claim that dynamics are physically calibrated. `validated=false` / `STRUCTURAL_ONLY_DYNAMICS_UNCALIBRATED` remains unchanged; chassis/servo/contact dynamics still require REAL metrology.

### 2026-08-30 — MasterPi visual envelope corrected from official servo/bracket dimensions
- Re-audited the V2 appearance model against Hiwonder's MasterPi dimension drawing (185×162×343 mm, 65 mm wheels, 101 mm chassis/body-top envelope) and official component dimensions rather than shrinking the whole robot by eye.
- The main visual-bulk error was in the arm hardware: standard servo visuals were 38–50 mm wide although the LD-1501MG body is ~40×20×40.5 mm, and arm bracket plates were modeled as 6 mm thick although Hiwonder metal servo brackets are 2 mm aluminium.
- Updated standard-servo visual envelopes to 40×20×40.5 mm, the gripper micro-servo visual to the LFD-01M 22.3×12×23.2 mm envelope, plate thickness to 2 mm, and arm side-plate spacing to a ~27 mm outside envelope. Collision capsules, joint axes, IK/FK, camera calibration, and dynamics were intentionally unchanged.
- Added `tests/test_masterpi_visual_geometry.py` so the arm cannot silently regress to the old bulky silhouette. Regenerated `sim/masterpi_scene_v2.xml` from the same source-of-truth.

## 2026-08-30 — SIM third-person presentation upgraded independently from robot camera
- Split human-facing observer rendering from the robot perception camera in the V2 twin. `robot_cam` remains the physical-camera contract at **640x480** with the measured fisheye/remap path and JPEG quality **76**; this preserves REAL-equivalent perception geometry.
- Added an independent observer renderer with a default **1280x720** framebuffer and raised the live front-follow third-person JPEG quality to **92**. Observer resolution/quality are presentation-only and do not feed planner perception or alter the calibrated robot camera.
- Increased the generated MuJoCo offscreen framebuffer capacity to 1280x720 and regenerated `sim/masterpi_scene_v2.xml` from the same V2 source so the checked-in runtime XML remains synchronized.
- Added regression gates that the observer pixel count exceeds the robot sensor contract and that third-person JPEG quality is higher than first-person quality. Targeted camera/twin/UI suite: **24/24 passed**.
- Deployed the canonical bundle to the active Azure A10 worker; it reconnected authoritative with one remote WebSocket, queue depth 0, no inflight command, and the local/remote bundle digests matched.
- Live coworker readback after deployment: first-person **640x480**; third-person **1280x720**. No REAL service or physical MasterPi actuator was touched.

## 2026-08-30 — SIM self-observer / execution forensics added and deployed

- Root problem addressed: SIM actions previously published only transient browser video plus final result/state. Human-facing third-person motion was not durably preserved, so later debugging often reasoned from exit codes/final state rather than from what the robot actually did over time.
- Added privileged SIM-only `sim/self_observer.py`. It compares pre/post MuJoCo truth and visual detections to classify simulator/runtime anomalies such as false pick success/failure, destructive grasp misses, one-sided contact, approach moving away/pushing the target, perception actions translating/rotating the chassis, implausible chassis pose, and false placement success. Unknown failed actions remain explicitly `ACTION_FAILED_UNCLASSIFIED` so the observer can be extended instead of guessing.
- The remote GPU worker now records each command from the pre-action boundary through execution to the post-action boundary. At a bounded ~0.4 s cadence it preserves the exact live robot-camera frame, the human third-person observer frame, and synchronous privileged SIM state. Default cap is 36 samples/action.
- Privileged trace payloads are stripped in `sim.bridge` before the result reaches the robot actor. The bridge persists each action under `outputs/sim_traces/<trace_id>/` as immutable `trace.json`, `analysis.json`, `analysis.md`, and JPEG frame sequences. The result only exposes non-privileged `sim_trace_id` and observer status.
- Added read-only bridge discovery endpoints `GET /sim/traces` and `GET /sim/traces/latest` and `scripts/analyze_sim_trace.py`, which creates a temporally distributed contact sheet combining 3rd-person and robot-camera keyframes for human/agent review.
- Added mandatory workflow to `agent.md`: future SIM behavior fixes must replay the user sequence, inspect self-observer analysis + frame timeline before changing code, separate actor evidence from privileged truth, compare traces after the fix, and extend the observer whenever a new visible failure remains unclassified.
- Deployed the canonical updated bundle to the active Azure A10 worker and restarted only the SIM bridge. REAL services/physical MasterPi were not touched.
- Live Azure seed-11 validation: `reset -> search -> track -> approach -> pick` all produced traces. The pick trace captured 19 third-person + 19 robot-camera frames across the ~7.6 s action and classified `CLEAN`; final MuJoCo truth independently showed `grasp_color=red`, bilateral contact, lifted=true, stable=true. A contact sheet was generated successfully.
- Verification: self-observer focused assertions 4/4 passed; SIM bridge/frame targeted unit tests 20/20 passed; broader SIM bridge/transport/UI/shared-source/servo regression 42/42 passed. `.venv-sim` does not currently include pytest, so these were run through direct assertions and unittest rather than pytest.

## 2026-08-30 — MasterPi morphology re-audit against current Hiwonder assembly/product imagery

- User reported that the simulator robot still looked substantially unlike the physical MasterPi. Re-opened the canonical geometry source rather than assuming the prior visual pass was correct.
- **Verified manufacturer evidence:** current Hiwonder product/docs list a metal-bracket MasterPi with 185×162 mm footprint, ~343 mm height on the global product page, 1.1 kg mass, 65 mm-class mecanum wheels used by the existing nominal model, 4DOF+gripper, 480P eye-in-hand camera, LD-1501MG/standard servo family and LFD-01M micro gripper servo. Current assembly imagery clearly shows (a) broad perforated sheet-metal electronics cover/side panels rather than the prior rail-only cage interpretation, (b) open orange arm brackets with large cut-outs, (c) thin metal gripper fingers with orange end pads, and (d) spoke-style grey wheel hubs. The manufacturer pages are internally inconsistent on total height (343 vs 347 mm across regional pages), so the existing 343 mm nominal is retained rather than silently choosing a variant.
- **Important correction:** `sim/masterpi_dynamics_v2.py` still used a reduced-order pair of symmetric slide joints for the gripper. The physical unit is linkage/servo driven. The dynamic slide coordinate is intentionally retained for now because no public pivot/link-length drawing or direct caliper/STP measurement is available; claiming an invented linkage as physical would be worse. Visible linkage arms/pivots and pads are now separated from the invisible contact proxy so visual fidelity can improve without destabilizing the already-tested grasp contact model.
- Replaced the prior rail-only electronics-cage visual interpretation with thin left/right sheet panels and a broad top cover plus non-colliding slot decals. The invisible `rear_cage_collision` proxy remains separate.
- Reworked arm visuals into separated upper/lower orange rails (open-frame silhouette) while retaining existing kinematic link lengths/collision capsules. Rail envelope was widened after direct render review because the first pass still looked unnaturally thin relative to official photos.
- Reworked the gripper render into thin aluminium linkage/finger bars, visible pivot discs and slim orange tip pads. `left_finger`/`right_finger` are now invisible physical contact proxies; `*_finger_pad_visual` are presentation-only. No simulator teleport/constraint grasp was introduced.
- Reworked each wheel hub from a solid grey disc to a small centre hub plus eight radial spokes; the existing eight orange visual mecanum rollers and invisible reduced-order support/contact cylinder remain unchanged.
- Regenerated `sim/masterpi_scene_v2.xml` from the canonical runtime XML. This also fixes a source/static-scene drift that existed before this pass.
- Camera parity was re-run against the *same measured physical parity points*. The morphology change altered visible gripper occlusion and improved the fixed replay from mean/max 4.88/9.73 px to 3.02/7.90 px. `CAMERA_PARITY_POINTS` were not changed; only the renderer-dependent evidence constants were updated.
- **Regression evidence:** `tests.test_masterpi_visual_geometry + tests.test_masterpi_physical_geometry + tests.test_masterpi_nominal_twin` = 33/33 OK. Manipulation/contact regressions `tests.test_real_stack_sim_servo + tests.test_grasp_physics + tests.test_continuous_physics + tests.test_pick_red_block + tests.test_physics_fidelity_gate` = 28/28 OK.
- Render evidence written to `outputs/masterpi_geometry_review/morphology_v2/` and manually inspected in straight calibration/reference views; reset/floor-search pose was also rendered separately under `outputs/masterpi_geometry_review/reset_pose/` to verify the gripper is horizontal in the actual working pose.
- Production browser SIM is Azure-authoritative (`NV6ADS_A10_V5`). Verified worker bundle digest `21ded0fef20c08b795eeff37d6686d61cdd46578d3c917e0adea77a8e27270a1` on both Oracle source bundle and Azure worker, restarted the Azure worker, and observed it return authoritative with a fresh instance/episode and live state updates. Thus the browser is not left on the pre-change worker bundle.
- **Still explicitly not claimed:** CAD-level dimensional identity. No MasterPi `.stp/.step/.stl/.obj` is present in the project/Oracle filesystem, and Hiwonder's public Q&A says CAD is not public while an STP can be provided through support. Until the purchaser STP or direct caliper measurements are available, detailed sheet cut-outs, gripper pivots, exact bracket outlines, wheel spoke cross-sections, and variant-specific 343/347 mm total height remain nominal/photo-derived. Do not relabel these as measured.

## 2026-08-30 — Azure long-action WebSocket reset fixed; seed-11 red pick E2E passes

### Symptom
- Azure SIM repeatedly changed worker instance/episode during long `approach`/`pick` actions.
- Bridge logs showed repeated `keepalive ping timeout` disconnects (1011), commonly around long synchronous controller/physics work.
- This caused otherwise-correct commands to surface as `SIM_EPISODE_RESET` and prevented reliable browser E2E validation.

### Root cause
- `scripts/run_mujoco_ws_worker.py` executes `world.act(...)` synchronously. Long controller actions can take tens of wall-clock seconds.
- Both worker and bridge additionally enabled the websockets library protocol keepalive at `ping_interval=10`, `ping_timeout=10`.
- The project already has explicit application-level `ping`/`pong`, worker/state publication, authority tracking, and bounded command timeout (up to 300 s). The extra 10 s protocol keepalive was redundant and could tear down a healthy in-flight action.

### Fix
- Disabled websockets library protocol keepalive on both ends:
  - `scripts/run_mujoco_ws_worker.py`: `ping_interval=None`, `ping_timeout=None`
  - `sim/bridge.py`: `ping_interval=None`, `ping_timeout=None`
- Kept the existing application-level ping/pong, authority/worker-state checks, and command timeout as the transport liveness authority.
- No REAL robot service or actuator was touched.

### Related vision / gripper corrections carried into this validation
- MasterPi orange material was moved outside BOTH REAL red-detector families, not HSV alone:
  - `sim/masterpi_scene.xml` orange is now `rgba=".95 .60 .03 1"`.
  - Nominal render color is around HSV H~19 and LAB A~153, outside the red HSV window and below the LAB-red A>=158 threshold.
- This eliminated the observed target-lock jump from the true red cube to orange robot geometry during body alignment.
- The slim physical-style gripper/contact geometry remains active; before gripper closure the red cube stayed stationary in the final trace.

### Validation
- Transport regression suite: 23/23 passed:
  - `tests.test_sim_transport_recovery`
  - `tests.test_sim_bridge`
  - `tests.test_gpu_worker_frame_reuse`
  - `tests.test_sim_self_observer`
- Same Azure worker instance/episode remained fixed for the entire seed-11 sequence:
  - instance `9b68c201339574a5`, episode `2`
  - `search`: PASS
  - `track`: PASS
  - `approach`: PASS, 36 s wall-clock, no episode change
  - `pick`: PASS, 27 s HTTP wall-clock / 18.336 s worker action, no episode change
- No bridge ping-timeout disconnect occurred during the long `approach` or `pick`. One earlier no-close-frame disconnect at 15:28:41 preceded the stabilized validation sequence.

### Final pick evidence
- Trace: `20260830T153011Z_ep0002_seed11_pick_d1efbb4a19e52d79`
- Self-observer status: `CLEAN`
- 21 captured timeline frames; contact sheet generated.
- Red block displacement while gripper remained open (PWM 2000 through pregrasp): 0.0 mm across captured samples.
- First bilateral closing contact (PWM 1500): red block moved about 2.44 mm from the preceding sample, rather than the prior large pre-contact push.
- Final state:
  - `grasp_color=red`
  - left/right contact = true/true
  - bilateral contact = true
  - normal forces ~= 0.237 N / 0.237 N
  - `lifted=true`
  - `stable=true`
  - red centre ~= `[0.4071, 0.0263, 0.2446]` m
- Large subsequent red-block XYZ changes in the trace occur after bilateral contact while the arm is lifting/carrying the cube; they are not pre-contact pushes.

### Current fidelity caveat
- Dynamics remain `V2_STRUCTURAL_UNCALIBRATED` / `training_ready=false`. This validation establishes controller/transport/structural behavior for the reproduced seed, not calibrated sim-to-real dynamics.


## 2026-08-30 — SIM browser display backlog removed with latest-frame-only presentation
- User reported visible SIM stutter and severe screen delay. Live bridge health at diagnosis was healthy (`remote_authoritative=true`, Azure `NV6ADS_A10_V5`, queue depth 0, no inflight command), and cached bridge snapshots were locally fast: first-person ~15.8 KB and third-person ~71.8 KB with millisecond-scale local reads. This did not prove the remote render path itself was jitter-free, but it ruled out a saturated Oracle command queue as the immediate explanation.
- The live browser still consumed both SIM views through long-lived MJPEG responses (`/api/camera/stream` and `/api/observer/cctv_front_left/stream`). The harness proxied MJPEG chunks directly and had no latest-frame drop policy. Therefore a slow browser/network could accumulate historical JPEG parts in TCP and display old motion even after the simulator had advanced.
- SIM presentation now uses latest-frame-only snapshot refreshes instead. First-person repeatedly reads the cached `/api/camera`; third-person uses a new read-only harness proxy `/api/observer/<name>/snapshot` backed by the bridge's existing latest observer snapshot. Each next request is scheduled only after the previous image loads (nominal 100 ms), so there is at most one presentation request in flight per view and no historical MJPEG queue to replay. Generation/visibility guards cancel refresh loops on mode/tab changes.
- Existing MJPEG backend routes remain for compatibility/controller paths; the change is browser presentation-only and does not alter robot perception, task policy, MuJoCo dynamics, camera geometry, or REAL services.
- The proposed third-person JPEG-quality reduction was intentionally reverted rather than deployed without separate visual-quality validation; production worker quality therefore remains 92 at 1280x720.
- Verification: `tests.test_sim_ui_consistency + tests.test_harness.WebTests` = 30/30 passed. Before the live UI reload the bridge was authoritative with queue depth 0 and no inflight command. Only `ugrp-sim-coworker.service` was restarted. Post-reload HTML exposes `data-snapshot-src=/api/observer/cctv_front_left/snapshot`; five live first-person latest snapshots completed in ~1.6–2.5 ms each and five third-person latest snapshots in ~4.2–13.7 ms each through the coworker on Oracle. No lingering ESTAB/CLOSE-WAIT localhost stream sockets remained after the snapshot probes.
- Remaining caveat: these measurements validate the Oracle-side presentation path and backlog elimination, not end-to-end latency from the user's Mac nor GPU render callback jitter during a long action. If visible motion still hitches after this change, the next investigation should timestamp Azure `world.frame_callback` render/send duration versus physics/controller progress rather than weakening simulation fidelity by guesswork.

## 2026-08-31 — SIM first-person accumulated lag removed; camera mount recentered conservatively

### User-visible symptoms
- User reported that the SIM first-person camera was still visibly late even after the browser had been changed from long-lived MJPEG playback to latest-snapshot display.
- User also reported that the eye-in-hand camera did not look mechanically centered.

### Evidence / root cause: first-person latency
- Browser snapshot serving itself was not the remaining bottleneck. The Azure worker still built one WebSocket publication by rendering the 640x480 robot camera and then the 1280x720 observer camera before sending the combined payload.
- During physics actions, `MasterPiProductionV2._physics_step()` only invoked the presentation callback every 50 physics steps at 1x (0.002 s timestep => ~10 Hz). This capped first-person freshness before the frame even left Azure.
- The first decoupling pass exposed a second, more important bug: the idle `ws.recv()` timeout called `emit_live(force=True)`. `force=True` bypassed the new observer cadence, so the 1280x720 observer still transmitted on effectively every first-person refresh. Live measurement showed both streams around 13 fps and the generated->bridge timestamp gap growing to ~1.7 s after the connection had run for a while. That was a real producer/transport backlog, not merely browser decoding.

### Latency fix
- `scripts/run_mujoco_ws_worker.py` now publishes first-person `robot_cam` immediately and sends the observer as a separate `observer_frame` message. The first-person send never waits for the high-resolution observer render.
- Normal idle timeout now calls `emit_live(force=False)`, so stream cadence gates are respected. Forced publication is reserved for sync/action boundaries.
- First-person target interval is 0.05 s; the physics presentation callback was tightened from every ~0.10 simulated s to ~0.05 simulated s at 1x. Observer presentation is independently limited to ~0.20 s and its JPEG default was reduced from quality 92 to 86 while keeping 1280x720 resolution.
- `sim.bridge` accepts observer-only publications without changing/fencing robot state and records first-person/observer render+arrival metadata for live latency diagnosis.
- The browser remains latest-frame-only, so even if the client is temporarily slow it cannot replay an old MJPEG queue.

### Live latency validation after Azure deployment
- Azure A10 worker was redeployed with the corrected canonical bundle; SIM bridge/UI were reloaded only after verifying queue depth 0 and no inflight command. REAL services/hardware were not touched.
- Six-second idle readback after the force-gate fix:
  - first-person: ~14.9 fps, median generated interval ~62.5 ms, generated->Oracle arrival ~50 ms and stable (first/median/last approximately 49.6/50.4/49.7 ms; max ~69.4 ms), i.e. no cumulative seconds-long queue;
  - observer: ~4.5 fps, median generated interval ~202 ms.
- Live `search(target_color=red)` validation while the controller was active:
  - first-person ~13.7 fps;
  - median generated->Oracle arrival ~51.8 ms, max ~80.7 ms, final ~50.8 ms;
  - search completed successfully using the shared REAL search implementation.
- SIM was reset back to seed 11 after the measurement.

### Evidence / correction: camera center
- The previous production `CAMERA_LOCAL_*` values were the result of a nine-point fit collected at only one servo pose. The calibration plan requires 12 fit + 12 held-out hand-eye observations, and no held-out physical hand-eye set exists yet.
- That one-pose fit encoded a **+8.12 mm lateral translation** and approximately **+5.53 deg optical yaw** relative to the gripper. Direct forward fixture points were consequently around x~370 px even though image width is 640 px. Promoting those lateral/yaw terms as a validated physical mount was unjustified.
- The measured physical 640x480 intrinsics and fisheye distortion remain unchanged.
- The production mount is now explicitly `CENTERED_STRUCTURAL_MOUNT_UNVALIDATED`:
  - local position `[67.0, 0.0, 13.6] mm` (mechanical lateral centreline);
  - optical yaw 0 deg and roll 0 deg;
  - only the ~7.46 deg pitch component from the historical image fit is retained provisionally because candidate replay showed that it preserves the existing range-change response, whereas blindly applying the older `-7.95 deg` compatibility pitch made range change grossly exaggerated.
- The old one-pose 6-DoF transform is preserved as `CAMERA_PARITY_FIT_LOCAL_*` archival evidence rather than deleted, so provenance remains available without forcing production geometry to follow an unvalidated off-centre fit.
- `camera_mount_status` and a new calibration ID are exposed in SIM state so future work cannot silently describe this centered structural correction as measured/held-out calibrated.

### Regression / deployment status
- Camera/transport/UI/geometry/shared-controller focused suite: **74/74 passed** after the final change.
- Live authoritative worker reports `camera_mount_status=CENTERED_STRUCTURAL_MOUNT_UNVALIDATED`, seed 11, queue depth 0, no inflight command.
- This fixes the demonstrated off-centre production mount and accumulated display backlog. Exact physical hand-eye calibration still requires the planned external 12-fit + 12-held-out dataset; do not relabel this centered mount as calibrated until that gate passes.

## 2026-08-31 — PLACE destination search exposed explicitly; transient SIM carry contact no longer means immediate loss

### User-visible symptoms
- A follow-up command such as "파란색 블럭 위에 올려줘" showed only `place(red, blue)` and did not show an explicit blue-target search step.
- SIM `place` could abort with `carried red block was lost during delivery; aborted immediately` during a small delivery turn even though the block visually still appeared to be held.

### Root cause: destination search was hidden inside `place`
- `TaskExecutive.plan_calls()` intentionally collapsed a PLACE goal to one `place(target_color, destination_color)` call when a block was already held.
- The shared `scripts/red_block/place.py` did in fact call `acquire_target(destination_color)` internally before placement, so the destination was being searched, but that acquisition was invisible in the top-level plan/progress UI.
- Reusing ordinary `search(destination)` while a block is held is unsafe: the normal acquisition pipeline uses the ordinary search/gripper pose and is intentionally blocked by the executive with `OBJECT_ALREADY_CARRIED`. The delivery search in `place.py` is different: it keeps the gripper closed/high and uses guarded camera/body turns.

### Fix: explicit carry-safe destination acquisition
- Added public action `search_destination(target_color, destination_color)` in the shared REAL/SIM action contract.
- `search_destination` routes through `place.py --locate-only`, which requires the carried-block handoff, keeps the gripper closed, runs the existing delivery-safe `acquire_target(destination_color)`, stops, and returns without releasing the object.
- The deterministic executive now compiles a held red->blue PLACE follow-up as:
  `search_destination(red, blue) -> place(red, blue)`.
- A full from-empty PLACE plan is now:
  `search(target) -> track(target) -> approach(target) -> pick(target) -> search_destination(target, destination) -> place(target, destination)`.
- Ordinary `search/track/approach/pick` remain blocked while the gripper may hold an object; only `search_destination` has the special carried-object precondition.
- `place` still re-acquires the destination internally before the destructive release sequence. This is deliberate fail-safe redundancy until there is a persisted, freshness-checked destination handoff across action processes.

### Root cause: SIM carry watchdog treated one contact sample as a drop
- Historical failing trace: `20260830T161737Z_ep0004_seed11_place_4f656c290974769b`.
- Frames before the abort showed a stable lifted red grasp. At the final sampled state:
  - `grasp_color=red` still held logically;
  - `red_z=0.2296 m`, so the block was still about 23 cm above the floor;
  - `lifted=true`;
  - `grip_error_m=0.0248`, only 24.8 mm from the grip site;
  - but both finger contacts became false for that sample.
- The old `SimRobot.carry_intact()` required `bilateral_contact` on every single check, so one transient contact gap after a small delivery rotation immediately returned false and raised the user-visible `lost during delivery` error.

### Fix: sustained-loss carry watchdog
- `SimRobot.carry_intact()` now keeps immediate hard-failure gates for actual physical evidence of loss:
  - logical `grasp_color` cleared/mismatched;
  - block falls below the lifted threshold;
  - block-to-grip separation exceeds 40 mm.
- Bilateral contact still immediately confirms carry and clears the miss timer.
- If bilateral contact is temporarily absent but the block remains lifted, logically carried, and within 40 mm of the grip site, the watchdog allows a short **0.22 s simulated-time grace window**. Persistent no-contact beyond that window still fails.
- This does not attach the object or overwrite MuJoCo physics; it only prevents one noisy contact sample from being promoted into an immediate semantic `CARRY_LOST` decision.

### Self-observer / diagnostics
- Added deterministic issue `CARRY_CONTACT_TRANSIENT_AT_PLACE_ABORT` for the case where a failed place started held and ends still logically held/lifted but bilateral contact is absent.
- Replaying the historical failing trace's exact before/after state through the current observer now classifies that failure as `CARRY_CONTACT_TRANSIENT_AT_PLACE_ABORT` instead of `ACTION_FAILED_UNCLASSIFIED`.
- The stored historical `analysis.json` itself remains historical/stale; `scripts/analyze_sim_trace.py` reads it rather than recomputing it.

### Validation / deployment
- Planner/action/shared-controller/carry/failure/transport/UI focused unittest regression: **145 tests passed after the final observer classification change**; focused planner/place/shared-source suite was **69/69 passed**.
- The observer module's test file uses pytest-style free functions, but pytest is not installed in `.venv-sim`; the exact historical state was therefore replayed directly through `diagnose_action()` and produced the expected new classification.
- Oracle coworker was restarted so the new action catalog/executive is live. Live registry contains `search_destination`, and a held red->blue goal compiles to `search_destination -> place`.
- Azure production bundle contains the new `carry_intact()` grace logic, `MigratedRealStack.search_destination()`, and `place.py --locate-only` path.

### Important separate regression
- A fresh current seed-11 end-to-end replay after the preceding camera recenter work reached `search -> track -> approach` but failed at `pick` before `place` with `GRASP_NOT_ACQUIRED` / no bilateral grasp.
- Therefore this entry verifies the destination-search architecture and fixes the demonstrated false carry-loss abort, but **does not claim that the current full red-on-blue episode is end-to-end successful**. The camera/hand-eye -> pick regression must be fixed separately before re-validating the entire placement chain.

## 2026-08-31 — SIM first-person UI must bypass startup cache for snapshot-backed camera sources
- Reproduced a display-only freeze in the SIM coworker UI: the authoritative bridge `/snapshot` changed while `search` moved the simulated eye-in-hand camera, but `/api/camera` returned one unchanged startup JPEG.
- Root cause: `ChatHandler._camera()` returned `ChatState.cached_frame()` before invoking the explicit `camera_url` grabber. Snapshot-backed SIM/cloud sources do not run the physical low-latency MJPEG pump, so that cache was never refreshed by the UI path.
- Added `ChatState.prefer_fresh_camera_snapshot`; `serve_chat()` enables it for explicit `camera_url` sources. `/api/camera` now fetches the current snapshot first for those sources while preserving the REAL physical cache-first path.
- Added regressions proving snapshot-backed cameras bypass stale cache and physical cameras retain cache-first behavior.
- Live verification after restarting `ugrp-sim-coworker.service`: during seed-11 `search`, UI `/api/camera` produced 3 distinct hashes and matched the bridge `/snapshot` on all 6 sampled requests (before fix: UI unique=1 while bridge unique=2).

## 2026-08-31 — REAL manipulation latency: remove trace-transfer stall and use coarse-to-fine visual servoing

### Measured latency, not guesswork
- Recent REAL traces expose four wall-clock timestamps per skill. Representative successful runs showed:
  - `search`: 7.24 s physical execution + **11.88 s trace copy**;
  - `track`: 4.29 s physical execution + **10.21 s trace copy**;
  - another `search`: 17.99 s physical execution + **9.76 s trace copy**.
- Thus the largest user-visible pause between skills was not wheel motion or vision inference. It was recursive `scp -r` of 5--24 small JPEG/event files over the often DERP-relayed Tailscale path.
- This delay was functionally harmful as well as slow: a 30 s close-near-look handoff could expire while the harness was copying trace files between `search`, `track`, and `approach`.

### Transport fix
- `scripts/red_block/deploy.py::_copy_remote_trace()` now fetches the completed remote trace as **one gzip tar stream over one SSH command**, safely extracts it locally with Python's `tarfile` data filter, and deletes the remote source only after successful extraction.
- The full trace evidence is preserved; this is not a reduction in diagnostics.
- SSH `ControlPersist` for REAL skill transport increased from 120 s to 600 s to avoid paying a cold DERP/SSH handshake again after short operator pauses.
- Unit regression verifies the trace path uses `ssh ... tar -czf -` and no recursive SCP.
- A live synthetic transfer benchmark was attempted but the Pi was offline/Tailscale-unreachable at that moment, so no new physical-network speedup number is claimed yet.

### Controller fix: coarse-to-fine where there is visual margin
- A real pre-capture trajectory advanced `ny=0.423 -> 0.742` through eight identical 60 ms chassis pulses. The motion time itself was only 0.48 s; repeated stop/settle/re-sample cycles dominated.
- `visual_precapture_approach()` now uses bounded image-space coarse-to-fine pulse durations:
  - `ny < 0.55`: 100 ms;
  - `0.55 <= ny < 0.65`: 80 ms;
  - near the 0.72 overlap boundary: the proven 60 ms pulse remains.
- A replay plant using the measured ~0.04 `ny` progress per 60 ms reaches the overlap in **6 pulses instead of 8**.
- The verified post-drive three-frame sample is carried into the next loop rather than immediately re-reading the same stopped state.
- A three-frame `fine_align_horizontal()` pass is no longer run unconditionally at every loop head. The existing three-frame sample is accepted when x error is <=0.025; any larger drift still invokes pan alignment and then a fresh sample. The hard x failure gate remains 0.07.
- Redundant `robot.stop()` calls immediately after `Robot.drive()` were removed from the visual creep loops because `Robot.drive()` already stops all motors in its `finally` path.
- The close final-capture chassis pulse remains 60 ms. That geometry is too sensitive to accelerate without new physical calibration.

### Close-camera pan latency
- A recent physical transition entered pre-capture around pan 1536, reset pan to 1500 for the close view, and then walked back to ~1564 in many 8-pulse servo nudges.
- The capture-pose transition now preserves the already-servoed pan when it lies inside the normal body-alignment envelope; otherwise it still falls back to base centre.
- `fine_align_horizontal()` keeps the old 8-pulse maximum near the target, but allows a bounded 24-pulse camera-pan step when normalized x error is >=0.05. This affects only the camera pan servo, not chassis motion, and every step is still followed by fresh visual feedback.

### Validation / deployment state
- Focused latency/REAL regression: **163/163 passed**.
- Expanded REAL-critical regression: **323/323 passed**.
- Full repository unittest discovery: **619/619 passed**.
- The local source is ready and `deploy_and_run()` will content-address/deploy it automatically before the next physical skill.
- At validation time `ugrp1` was `offline, last seen ~10m ago`; actuator lease could not be verified and camera age was stale, so no physical actuation, immutable Pi deployment, or live timing trial was attempted/claimed.

## 2026-08-31 — REAL arm/chassis concurrency without racing the I2C bus

### Motivation
- The physical sequence still looked overly serialized: wait for one servo/arm move to finish, then start the next subsystem.
- MasterPi PWM servo interpolation continues on the controller after the I2C target command is accepted. Therefore physical motions can overlap even though I2C command writes themselves remain serialized.

### Implementation
- Added `Robot.move_servos_and_drive(...)` as the guarded arm/chassis overlap primitive.
  - Servo target I2C frames are written first and the PWM controller begins interpolation.
  - Chassis wheel targets are then written immediately.
  - Wheels stop at their own bounded deadline; the caller waits only for any remaining servo interpolation/settle time.
  - No Python threads write the I2C bus concurrently.
  - Motor stop is guaranteed in `finally`, including partial motor-start failure.
  - Existing precision/near-look/carry-return handoffs are invalidated with the same motion semantics as ordinary arm/chassis commands.
- First real use is the **empty-sector search fallback**: when a full head sweep found no target, head recenter and chassis sector rotation now execute as one motion group. The peripheral-target turn remains sequential/frozen-gaze because that branch has measurement authority and must preserve the acquired bearing.

### Servo-only serialization removed
- Pick pre-descent transition previously did `base yaw -> wait -> gripper open -> wait -> shoulder/elbow/wrist hover -> wait`.
  - It now issues gripper + base + joints 3/4/5 as one 1.25 s PWM interpolation after the verification reference has already been captured.
  - Approximate commanded wait budget drops from ~2.70 s to ~1.40 s before descent, without changing the descent/grasp/verification sequence.
- Place hover transition previously did `base yaw -> wait -> carried-arm hover -> wait`.
  - Base + joints 3/4/5 now interpolate together while the gripper stays closed.
  - Approximate commanded wait budget drops by ~0.65 s before the release descent.

### Deliberately still sequential
- Face-angle measurement/orbit verification, range measurement, final visual capture, descent, gripper-close, and stack verification remain sequential.
- No automatic post-pick chassis retreat was added: chassis displacement would invalidate the saved arm-only return path used by `put_down()` to return the block to its original pickup site.

### Validation / deployment
- New dedicated parallel-motion tests: 4/4 passed, including stop-on-partial-motor-start-failure.
- REAL-critical regression: 329/329 passed.
- Full repository unittest discovery: **623/623 passed**.
- Immutable Pi package deployed and manifest/remote `py_compile` verified:
  `001cc8524e0932858743b6dd0c1d37dff3096b34582fc429a76c8e00c2ea00c6`.
- Oracle REAL service reactivated only after actuator lease was verified free. No physical skill/action was executed.
- Post-reload camera was fresh (~63 ms age) and actuator lease remained free.

## 2026-08-31 — REAL MJPEG owner conflict removed; camera restart is transparent to manipulation

### User-visible failure
- REAL `approach` first recovered from `target disappeared during final horizontal alignment`, then a retry failed with `MJPEG reader failed after 3 reconnects ... 127.0.0.1:8080/stream ... Connection refused`.
- Trace `20260831T054556Z-real-ba3be222` shows the second failure was `ROBOT_UNAVAILABLE` only after the destination recovery/search had already found red again.

### Root cause evidence
- On MasterPi, `ugrp-camera.service` received SIGTERM at 14:47:38 KST and exited successfully; a different orphan uStreamer started at 14:47:39 with the legacy `--host=0.0.0.0 --quality=80` command line.
- Active legacy launch scripts (`run_masterpi.sh`, `run_red_block.sh`, `start_masterpi_red_pick.sh`) explicitly did `pkill -f '[u]streamer'`, slept, and started their own `nohup /usr/bin/ustreamer` process. This competed with `ugrp-camera.service` and created a real port-8080 refusal window.
- `LiveVideo` allowed only 3 reconnects at 80 ms spacing and its constructor did not retry an initial connection refusal, so that ownership handoff could be promoted to a permanent skill failure.

### Fix
- MasterPi camera ownership is now single-owner systemd:
  - `ugrp-camera.service`: `Restart=always`, `RestartSec=0.25`, bound to Pi-local `127.0.0.1:8080`.
  - Active top-level legacy runner scripts no longer kill or launch uStreamer; they only `systemctl --user start ugrp-camera.service` and poll service + `/snapshot` readiness.
  - `vla_new/camera_watch.sh` is now a compatibility entrypoint that only starts/checks the same systemd service and never owns a camera process itself.
  - Historical backup directories were not rewritten. A pre-change backup was saved under `~/MasterPi/tools/backups/camera_owner_fix_20260831T145258/`.
- Existing orphan uStreamer PID 58006 was migrated while the actuator lease was verified free. The live listener is now the service MainPID and there is exactly one `/usr/bin/ustreamer` process.
- Shared REAL controller `LiveVideo` now uses 10 reconnects at 120 ms spacing and retries the initial stream open as well. This tolerates a short service restart without converting it to `ROBOT_UNAVAILABLE`.
- New immutable REAL package deployed: `0e8fef2f78335e857046a9c15645bf279b44c49aa745ce2617a7fcd7338d2c5b`.

### Validation
- New unit regressions cover both initial connection-refusal recovery and multi-attempt reader recovery.
- Focused REAL/camera/activation/tool-hardening suite: **85/85 passed**.
- Physical no-actuation restart test: while the new `LiveVideo` was reading the actual Pi MJPEG stream, SIGTERM was injected into the service uStreamer. systemd replaced PID `78606 -> 81037`; the reader completed its frame sequence without failure.
- Extended physical no-actuation test repeated the restart (`81037 -> 83022`) and read 36 samples across 3.666 s; decoded sequence advanced `1 -> 57`, proving post-restart live frames continued rather than relying only on a short buffered tail.
- Final Pi state: `ugrp-camera.service=active`, MainPID `83022`, exactly one uStreamer process, port 8080 owned by that MainPID, valid JPEG `/snapshot`.
- Oracle REAL `/api/camera` returned valid JPEGs after the restart (35.7 ms cold sample, then 1.3/0.8 ms local cached samples).
- No motor or servo command was sent during the ownership migration/restart tests; actuator lease was checked free before destructive camera-process transitions.

### Remaining separate behavior
- The earlier `target disappeared during final horizontal alignment` is a visual-control miss, not this camera transport bug. The executive already performed its bounded `search` recovery successfully in the observed run. This entry fixes the subsequent camera outage that prevented that recovery from continuing.

## 2026-08-31 — REAL MasterPi predictive low-latency acquisition activated

- **Observed bottleneck from physical trace**: the reported slow pickup was dominated by controller scheduling rather than low motor command speed. The 2026-08-31 REAL run spent only about 2 s in search and about 4 s in track, while approach consumed about 33 s. The approach trace repeatedly executed 0.06 s chassis pulses followed by STOP, camera re-stabilization, and another pulse; separate search/track/approach processes also added SSH/Python/camera startup gaps. The same trace later failed the existing face-orbit physical-progress gate rather than from a motor-speed command error.
- **Single-process acquisition fast path**: added `scripts/red_block/task_runner.py` and optional shared-camera/shared-video ownership to search/approach/pick. The harness now has a hidden `program_runner` path that fuses only the exact public `search -> track -> approach -> pick` sequence for one target colour into one Pi process and one MJPEG lifetime. The public tool contract is unchanged (17 tools); `run_program` is not exposed to the model, and the harness still records/updates the four public stage results separately so existing executive preconditions and recoveries remain authoritative.
- **Predictive/receding-horizon far approach**: the >=38 cm coarse approach can now keep the forward command active while consuming fresh locked-target frames instead of STOP-before-every-observation. Each observation estimates a short 0.18 s future range from visual closing rate. Target loss, image-x drift, invalid metric range, implausible closing rate, >1.8 cm prediction disagreement, predicted crossing of the coarse stop boundary, pan/body-realignment limits, hard radius, or the 0.45 s bounded streaming window pre-empt continuation. `robot.stop()` is in the `finally` path and therefore runs before recovery/replanning. This predictive path is disabled in the close precision region; the existing stopped face/capture/grasp safety gates remain in control there.
- **Near-field latency**: final horizontal pan correction is now error-banded (up to 48 PWM for very coarse error, 28 for coarse, 14 for fine, historical 8-PWM precision step near target). Multi-frame pickup verification can terminate once its threshold result is already determined, while retaining the existing floor-plane compatibility checks and confidence semantics.
- **Regression evidence**: focused predictive/task-runner/hidden-fusion/REAL failure-gate/tool-hardening/executive suite passed **109/109**. The full repository unit-test discovery subsequently passed **643/643 in 206.417 s**. An earlier full-suite invocation was killed only by its externally imposed 120 s command timeout; it did not report a test failure before termination. Syntax compilation passed and no patch reject files remain.
- **Activation/deployment**: guarded activation reported `MasterPi actuator lease: free`; Oracle `ugrp-real.service` was restarted through `activate_real_spatial_stack.sh`, which explicitly sends no MasterPi actuator command. The service returned active with `talk_only=false`, `execute_allowed=true`, low-latency camera enabled, and a fresh observed camera age of about **21 ms**. The immutable Pi package was pre-deployed as files only (no motor/servo/STOP action), signature `96c0158ead2fb85a684fd0acf3f48878b6687c4c48738cfbe92fec906c224ba5`.
- **Physical-validation boundary**: no new physical search/track/approach/pick or primitive motion was issued while implementing or validating this change. Therefore reduced physical task time and the predictive controller's real stopping behaviour are **not yet measured on hardware**. The next operator-requested pickup should use the activated hidden single-process fast path and pre-deployed package; compare its REAL trace stage timings and predictive stop reasons against the ~53 s pre-change run before claiming a measured speedup.

## 2026-08-31 — REAL face-orbit lateral breakaway diagnosis and fix

### Observed on REAL MasterPi
- Failure trace: `outputs/real_traces/20260831T055400Z-real-8bd00663`, approach span `20260831T055415Z-approach-9143d169`.
- The failing controller issued the verified right-strafe MotorTransport pattern `(1:+35, 2:+35, 3:+35, 4:+35)`, so this was not a simple left/right sign-map bug. Battery telemetry during that run was ~7.886 V.
- At speed 35 / 0.20 s, the failing run's measured target-bearing shifts were `+1.35, +2.09, +1.53, -0.68, +0.55 deg`; the face estimate did not converge (`37.5 -> 41.9 -> 35.2 -> 45.8 -> 37.3 deg`). The guard correctly stopped after consecutive <1 deg physical progress.
- Controlled REAL A/B tests at the existing approach arm posture showed generic speed 35 is marginal for lateral breakaway: one centered `right@35,0.10s` sample produced about `-0.31 deg` (essentially no useful motion / wrong sign).
- Raising only lateral speed to 40 changed the same-posture response materially: centered `right@40,0.10s` produced about `+2.03 deg`; from rest, `left@40,0.10s` was still only `-0.06 deg`, but the production-length `left@40,0.20s` produced about `-2.09 deg` in the correct direction.
- Folding/retracting the arm changed the response in some trials but did not reliably solve it by itself (e.g. a folded-support-pose `right@35,0.10s` trial was only ~`+0.03 deg`). Therefore load distribution / roller traction can contribute, but the evidence does **not** support claiming center of gravity alone as the root cause.
- The strongest current diagnosis is insufficient lateral breakaway margin at speed 35, combined with real mecanum traction/stiction asymmetry. Wheel/roller condition, floor friction, and posture-dependent normal load remain possible mechanical contributors; there is no wheel odometry or force sensing to separate them quantitatively.

### Code change
- `scripts/red_block/physical_state_machine_reference.py`
  - added `ORBIT_SPEED = 40` for short verified face-orbit strafes only;
  - retained `ORBIT_PULSE_SECONDS = 0.20`;
  - `orbit_strafe_measure_bearing()` now calls `start_motion(..., ORBIT_SPEED)`;
  - left `MOTOR_SPEED = 35` unchanged for ordinary forward/yaw motion;
  - deliberately retained the `<1.0 deg for 2 consecutive pulses` physical-progress safety guard. We did **not** mask the fault by lowering the verification threshold.
- `tests/test_real_failure_gates.py` updated narrowly to assert the orbit primitive uses `ORBIT_SPEED == 40`.

### Validation
- `python3 -m py_compile` passed for the modified source/test files.
- `.venv-sim/bin/python -m unittest tests.test_real_failure_gates.NearFieldCenteringTests`: **47/47 passed**.
- Immutable REAL package signature after the patch: `96c0158ead2fb85a684fd0acf3f48878b6687c4c48738cfbe92fec906c224ba5`.
- A subsequent REAL full-task run using that exact package, `outputs/real_traces/20260831T062135Z-real-3ce9f0b1`, completed `approach` with `execution_status=COMPLETED`, `outcome_status=ACHIEVED`, `range_verified=true` (approach elapsed ~26.64 s), then continued to pick in that separate task.
- Important limitation: in that successful run, the first face estimate was already near-normal (`floor-edge=-87.1 deg`, square error `2.9 deg`), so the controller correctly skipped face-orbit. Therefore that run verifies no approach regression from the patch, but it is **not** an end-to-end orbit-speed trial. The direct REAL A/B pulses above are the evidence that 40/0.20 clears the previous lateral breakaway failure in both directions.

### Remaining follow-up if the failure recurs
- Inspect all mecanum rollers for binding/free rotation and compare wheel contact loads on a flat surface.
- Repeat left/right 40/0.20 tests at several arm poses and floor locations; if one direction remains systematically weak, measure individual motor/wheel contribution rather than weakening the bearing-progress guard.

## 2026-08-31 — REAL grasp latency: fused task process + receding-horizon predictive approach activated

### Observed bottleneck
- The reported slow REAL run was not primarily a low wheel-speed problem. In the live log, `search` completed in about 2 s and `track` in about 4 s, while `approach` consumed about 33 s. The controller repeatedly issued 60--200 ms chassis pulses, STOPped, re-observed, re-faced, and re-stabilized the camera.
- Separate skill processes also imposed avoidable boundaries between `search -> track -> approach -> pick`: separate Python/SSH/Robot/camera setup even when the user's goal was one continuous acquisition task.

### Fused acquisition path
- The exact public acquisition sequence `search(target) -> track(target) -> approach(target) -> pick(target)` now has a hidden fast path in `harness.loop`: `scripts.robot_actions.run_program()` launches `scripts/red_block/task_runner.py` once.
- `task_runner.py` retains one `Robot`/actuator lease and one fresh MJPEG stream across all four stages and calls the existing stage implementations directly. Public tool names, preconditions, stage results, and standalone diagnostic actions remain unchanged.
- The fast path is enabled by default and can be disabled with `UGRP_FUSED_ACQUISITION=0` for debugging.

### Predict-first / stop-on-disagreement control
- Far approach now uses a bounded receding-horizon controller only while safely outside the precision zone (`>=38 cm`, with the existing stopped controller taking over before/at the 36 cm coarse boundary).
- While one forward command remains active, each fresh RGB-derived metric observation estimates the current radial closing rate and projects the block range **0.18 s ahead**. The next continuation is effectively prepared in advance rather than waiting for a full STOP/restart cycle.
- Continued motion is revoked immediately and `robot.stop()` runs in `finally` when any of these occur: target loss, lateral image drift, invalid range, implausible closing rate, prediction residual above the bounded envelope, projected entry into the guarded coarse boundary, pan/body-realignment boundary, hard-radius gate, wall-clock bound, or frame bound.
- This controller uses only REAL-equivalent actor observations and commanded state; it does not use MuJoCo truth or invent wheel odometry. Near-contact face alignment/final capture/descent remain conservative and stopped/verified.
- Dedicated regression demonstrates anticipatory braking: at a measured 38.5 cm sample, the 0.18 s projection is 35.26 cm and the controller stops with `predicted-boundary` before the measured state itself crosses the 36 cm coarse boundary. A separate test stops on an implausible observed closing rate rather than continuing a stale prediction.

### Activation / verification
- Current LIVE predictive/fusion focused regression: **141/141 passed**.
- A repository-wide unittest run reached 643 tests before the external 240 s command timeout injected SIGTERM into `test_spatial_memory.SpatialMemoryTests.test_head_first_search_avoids_chassis_yaw_when_target_is_reachable`; this was a runner-timeout artifact, not a reproduced assertion failure. The interrupted test passes independently: **1/1 in 3.512 s**.
- Oracle `ugrp-real.service` was guarded-reloaded only after repeated actuator-lease checks returned free. Post-reload status: service active, `execute_allowed=true`, `talk_only=false`, low-latency REAL camera active. No MasterPi motor or servo command was sent during reload.
- The current immutable Pi package was pre-deployed without actuation and passed remote manifest verification plus `py_compile`: `96c0158ead2fb85a684fd0acf3f48878b6687c4c48738cfbe92fec906c224ba5`. Actuator lease was free before and after deployment.
- **Not yet claimed:** a measured end-to-end physical speedup on the next real grasp. Software path, service activation, and Pi package are verified; the actual wall-clock improvement must be measured from the next physical run trace.

## 2026-08-31 — red pick → blue stack end-to-end SIM completion

### Reproduced failure chain
- The migrated REAL stack could pick red and find blue, but delivery initially failed through several distinct physical/control faults rather than one planner error: destructive carry micro-pulses, unsafe low-wrist camera dwell, stale raw-fisheye geometry, sequential arm lowering transients, pickup TCP offset leaking into placement geometry, and finally stale postcondition verification.
- The decisive carry traces showed that lowering servo 5 while holding a friction grasp consumed the remaining grip margin. The destination could be approached safely with the high transport wrist, but repeated near-view motion or a second close-view after the final chassis trim caused the cube to slip.
- The former SIM evaluator also still required the archived 50-mm-cube vertical gap (`0.038..0.070 m`) after production had switched to the measured 30-mm task cube, so a physically correct `xy≈4 mm, dz≈30 mm` stack was incorrectly rejected.

### Shared REAL/SIM fixes
- `scripts/red_block/geometry.py`
  - uses measured raw-fisheye intrinsics/distortion, physical camera mount, and wheel-radius floor offset for destination range/bearing;
  - separates pickup TCP correction from placement centre geometry with `PLACE_GRIP_CENTER_RADIAL_OFFSET_CM = 0.0`.
- `scripts/red_block/place.py`
  - keeps chassis motion in the high `p5=1400` transport wrist;
  - uses bounded low-start-count carry turns/forward segments and a single close measurement;
  - performs the final longitudinal trim without a second destructive close-view, with the final trim reduced to 0.18 s;
  - moves carried-arm hover/release joints concurrently;
  - adds immediate release-view support verification. A correctly stacked upper cube may occlude blue completely, so success no longer requires both colours to be simultaneously visible. The fallback requires a stable, centred, non-bottom-clipped squat upper-cube silhouette and rejects the reproduced floor-miss silhouette.
- `sim/real_stack_adapter.py`
  - mirrors REAL settle timing after the terminal trim;
  - derives SIM stack-evaluator tolerances from `TARGET_BLOCK_SIDE_M/TARGET_BLOCK_HALF_M` rather than stale hard-coded 50-mm-cube thresholds.
- Also fixed the deployed REAL approach logging regression where `fine_align_horizontal()` referenced stale `step_limit` naming in one worker revision; deployed worker file hashes were checked against Oracle source before the final run.

### Final verification
- Focused shared REAL/SIM regression suite: 41/41 passed.
- Full remote-authoritative Azure SIM E2E, seed 11:
  - reset → search(red) → track(red) → approach(red) → pick(red) → search_destination(blue) → place(red on blue): all succeeded.
  - final place trace: `20260831T063722Z_ep0021_seed11_place_b7ae636ec225dccb`.
  - result: `REAL place.py visually verified red-on-blue; MuJoCo evaluator confirmed the stack`.
  - final centres: red `[0.6995, -0.1980, 0.0447]`, blue `[0.6961, -0.1966, 0.0148]`, approximately 3.7 mm planar offset and 29.9 mm vertical centre separation.
  - semantic relation updated to `red: ON_BLUE`; `grasp_color` cleared after release.

### Calibration boundary
- This proves the shared action stack completes the task in the current production digital twin. `sim/masterpi_dynamics_calibration.json` is still `validated:false`; do not treat this SIM completion as physical-robot calibration proof.

## 2026-08-31 — lower, more stable capture grasp

- User-observed instability matched the SIM contact trace: the former capture target `CAPTURE_GRASP_HEIGHT_CM=0.0` was axle-relative, so the first bilateral finger contact occurred at about 2.93 cm above the floor while the 30-mm cube centre was about 1.58 cm. The fingers were pinching close to the cube's top edge.
- Lowered the fixed capture TCP by 6 mm: `CAPTURE_GRASP_HEIGHT_CM=-0.60`. This is intentionally conservative because the current provisional REAL reach is still 17.5 cm; lowering further pushes servo 5 onto the 2500 pulse hard limit. The new REAL IK fixture is approximately `{3:1193,4:1930,5:2485}` and preserves a small wrist-servo margin.
- Added a final 0.4-cm descent waypoint before the new lower target so the last approach is not one large vertical jump.
- Regression coverage now explicitly requires a negative capture height and servo-5 <=2485 at the current REAL reach. Relevant focused suites passed 84/84.
- Remote-authoritative Azure SIM seed11 full E2E succeeded after the change. New pick trace `20260831T065256Z_ep0022_seed11_pick_65115592f0a77192`: first bilateral finger height 2.34 cm, versus 2.93 cm before; finger-centre minus cube-centre offset improved from 1.35 cm to 0.85 cm.
- The same lower grasp remained intact through destination search and placement. Place trace `20260831T065305Z_ep0022_seed11_place_aaaa8545c9e0f994` succeeded with red-on-blue final planar centre error about 1.0 mm and vertical centre separation about 29.9 mm.
- Deployed the new immutable REAL package to ugrp1 without executing any motor/servo action; package manifest and remote `py_compile` were verified. Physical grasp improvement itself still needs the next real run trace for confirmation.

## 2026-08-31 — Default embodied planner changed to VLM-owned action selection

### Problem
The active MasterPi coworker path exposed public skills to the VLM, but a structured `TaskExecutive` fastpath could still compile common manipulation goals into fixed sequences such as `search -> track -> approach -> pick -> search_destination -> place`. Precondition failures could also trigger hidden deterministic recovery actions. That made successful behavior difficult to attribute to the VLM's own planning and conflicted with the research boundary that the agent proposes actions while the environment/executive only validates them.

### Design intent
- The default embodied path lets the VLM choose the next public skill from the current camera evidence and actor-visible `world_state`.
- `TaskExecutive.check()` remains deterministic as a safety/validity gate. It may reject an impossible or unsafe action, but normal VLM mode does not turn the rejection into an automatically executed recovery.
- Planner-facing rejection/state data reports the failure and required state, but does not expose `recommended_recovery`; the next action must be selected by the VLM.
- The prompt describes action semantics, arguments, contracts, sensor state, and self geometry, but does not prescribe situation-to-action rules such as `target not visible -> search`.
- Low-level skill controllers remain deterministic and bounded. Raw servo/motor control is not delegated to the VLM.
- The previous structured goal compiler/fused acquisition path is preserved only as an explicit rule baseline/debug mode with `UGRP_STRUCTURED_RED_FASTPATH=1`.

### Changes
- `harness/loop.py`: structured fastpath default changed from enabled to disabled; default precondition rejection no longer dispatches executive recovery; planner-facing results omit `recommended_recovery`.
- `harness/protocol.py`: removed situation-specific policy guidance and action-specific examples; added explicit VLM-owned action/replanning contract.
- `harness/executive.py`: documented `check()` as the runtime gate and `plan_calls()` as legacy baseline/debug planning only.
- `scripts/serve_sim_coworker.sh`: interactive SIM no longer forces the deterministic fastpath on; it defaults to `UGRP_STRUCTURED_RED_FASTPATH=0` while still allowing an explicit `1` override.
- Tests that exercise the old deterministic baseline now opt into the flag explicitly. A default-mode regression verifies that a rejected `pick` does not cause a hidden `search`; `search` executes only when selected on the next model turn.

### Validation
- Architecture-focused regression after final routing changes: `123 tests`, `OK`.
- Earlier broader targeted manipulation/harness regression: `235 tests`, `OK`.
- SIM runtime is to be restarted and checked separately so the running service inherits `UGRP_STRUCTURED_RED_FASTPATH=0` and the live `/api/turn` trace can confirm model-owned action selection.

### Boundary
No physical MasterPi command was issued for this architecture change. The REAL skill controllers and safety contracts remain in place. This updates the current browser/MasterPi prototype behavior; it does not by itself merge the official research plan and the Isaac Lab revision into a newly approved experimental specification.

### Final runtime verification and policy-leak cleanup
- The running `ugrp-sim-coworker.service` was restarted after the architecture change and its actual process environment was verified as `UGRP_STRUCTURED_RED_FASTPATH=0`.
- A live SIM request, `빨간 블록 찾아줘`, produced model-owned actions `approach -> approach -> search -> approach -> search` before the model finished. This is intentionally not a good fixed policy; it is evidence that the previous goal compiler was no longer silently choosing the SEARCH action for the model.
- That live run exposed a second policy leak: raw skill failure `reason` text could contain imperative advice such as `run search again`. Planner-facing tool/rejection feedback was therefore tightened again: it now carries objective fields such as `failure_code`, `required_state`, status, and observations, while omitting both `recommended_recovery` and free-form `reason`. Full debug/UI payloads still keep those fields for diagnosis.
- AI-facing chassis action descriptions were reduced to action effects and parameter bounds. Phrases such as `장애물 회피`, `시야 변경이나 방향 정렬`, `다음 행동을 결정`, and `새 장면을 확인` were removed from the public schema so that the action catalog does not smuggle in a weak situation-to-action policy.
- The hidden deterministic program runner remains hidden from the public tool surface. Its regression no longer hard-codes a public-action count, because the invariant is non-exposure of the hidden executor rather than a frozen number of public skills.
- Final architecture-relevant regression: `240 tests`, `OK`.
- A prior full `unittest discover` run reached `671 tests`; its structured-baseline expectation has since been fixed. Three errors in `tests/test_red_block_approach.py` were from a concurrent arm-face/approach refactor whose test fixtures lacked newly added precision APIs/constants, and are outside this planner-policy change.
- No physical MasterPi action was issued, and the REAL service was not restarted as part of this change.

### 2026-08-31 — REAL arm-only face alignment validated; further trials paused for low battery

- REAL run `20260831T075640Z-real-armonly2`, approach span `20260831T075655Z-approach-ca0fa521`, reached the new final arm-relative face check with the chassis stopped.
- At the final stopped observation, `floor-edge=-75.0deg`, `arm-bearing=+13.0deg`, and arm-relative face error was `2.0deg`; the controller correctly accepted the face without a chassis re-face or lateral pulse. This is direct REAL evidence that servo-6/arm-ray alignment can replace the old final chassis-facing step when geometry is already suitable.
- The run then failed closed before pick because `visual-radius=13.26cm`, the existing +5.0 cm longitudinal reach correction produced `block-radius=18.26cm`, and the resulting fingertip radius `18.76cm` exceeded the calibrated 18.0 cm maximum. Do not widen the reach envelope from this result.
- Offline replay of the same REAL close-pose frames showed the face estimator remained valid at `ny~=0.367` (edge length about 2.75 cm). The next bounded staging candidate is therefore `ARM_FACE_CAPTURE_TARGET_NY=0.42`, so the chassis approaches slightly closer *before* final side placement while preserving the no-re-face invariant afterward.
- The next REAL trial was NOT executed: repeated read-only battery probes were 6.746, 6.666, 6.746, and 6.756 V. An explicit STOP was issued and further actuation was paused because low voltage would confound mecanum/servo validation.

### 2026-08-31 — arm-face staging separated from final hand-eye grasp depth (supersedes the 0.42 single-stage candidate)

- No additional physical MasterPi motion was executed for this change. After the operator requested code-only application, all validation below used Oracle source/tests and archived REAL traces only; no Pi deployment or service reload was performed.
- Archived REAL close-pose frames from `20260831T055135Z-real-d4872c04` show that at `ny=0.408/0.412` the measured visual radius is already about `12.17-12.18 cm`, but `estimate_face_alignment()` returns no valid floor-face projection. Therefore moving the *face-measurement* gate itself to `ny=0.42` is not supported by REAL evidence and is superseded.
- The controller now uses two distinct depths:
  1. `ARM_FACE_CAPTURE_TARGET_NY=0.30` for reliable floor-face observation and any required lateral+servo6 face placement.
  2. After the face is accepted, preserve that measured floor-edge direction, forbid any chassis re-face/lateral motion, and use the existing bounded `visual_capture_approach()` to continue **straight forward only** to the calibrated hand-eye target `CAPTURE_TARGET_NY=0.442`.
- After that final straight refinement, the controller recomputes the arm bearing and compares it with the saved face-normal direction. If the inferred arm-face mismatch exceeds the existing `FACE_ALIGNMENT_ABORT_DEG`, it stops/fails closed rather than planning a grasp.
- Only after the final `ny~=0.442` hand-eye depth does it re-sample metric range, apply the existing +5.0 cm longitudinal reach correction, and run the existing calibrated fingertip envelope check. The envelope was not widened.
- This specifically addresses the latest REAL failure where `ny~=0.312` yielded a valid `-75 deg` face / `+13 deg` arm bearing but a `18.76 cm` fingertip request outside the `18.0 cm` calibrated maximum.
- Regression validation with no hardware actuation: `NearFieldCenteringTests + ApproachPlannerTests = 78/78`, plus `py_compile` for the modified controller/approach/tests. Local content signature is `bdaafa970fd7fb86ba23df2d690a793215319b34c97ec38b3d6110da0e88b580`; this signature has **not** been deployed to the robot.

## 2026-08-31 — SIM 화면 정지: Azure worker bundle의 runtime module 누락 수정

### 관찰된 현상
- 브라우저 SIM에서 오른쪽 action log는 `track`/`approach` 등으로 진행되지만 1인칭과 3인칭 MuJoCo 화면이 움직이지 않는 현상을 사용자 화면에서 확인했다.
- 브라우저 자체의 `/api/camera` polling은 약 100 ms cadence로 살아 있었고 cache-busting query와 `Cache-Control: no-store`도 정상이라 UI cache가 원인은 아니었다.
- 사용자와 동일한 seed `1099490316`을 유지한 채 `move_forward(speed=35,duration=0.8)`를 재현했다. 첫 이상 시점은 physics motion 전이며, 원격 action result가 즉시 `No module named 'scripts.robot_actions'`로 실패했다.
- 수정 전 trace `20260831T080230Z_ep0026_seed1099490316_move_forward_7aa0a1b86f98265a`: `base_shift_m=0.0`, `base_yaw_change_deg=0.0`, captured frames=2, 기존 observer 판정은 `ACTION_FAILED_UNCLASSIFIED`였다.

### 원인
- Azure의 실제 실행 프로세스는 `/home/azureuser/ugrp-worker`에서 `MasterPiProductionV2` single-robot worker를 실행 중이었다.
- `MasterPiProductionV2._real_action()`은 chassis primitive 실행 시 `scripts.robot_actions.PRIMITIVE_MOTIONS`를 runtime import한다.
- 그러나 `scripts/gpu_worker_bundle.py`의 production bundle 목록에 `scripts/robot_actions.py`가 빠져 있었고 실제 Azure worker 디렉터리에도 파일이 없었다. 따라서 move/strafe/turn 명령은 MuJoCo physics에 도달하기 전에 import failure로 종료되었고, 결과적으로 browser가 받을 pose/JPEG 변화도 없었다.

### 수정
- 현재 live Azure worker 디렉터리에 `scripts/robot_actions.py`만 최소 hotfix로 배포했다. 원격 `py_compile`과 SHA-256 일치를 확인했고 worker 재시작은 필요하지 않았다(runtime import이므로 다음 action부터 반영).
- `scripts/gpu_worker_bundle.py`에 `scripts/robot_actions.py`를 영구 포함하고, `tests/test_sim_ui_consistency.py`에 bundle 누락 회귀 검사를 추가했다.
- 동시 진행 중인 3x worker/bridge 변경을 이 화면 수정과 섞어 배포하지 않기 위해 전체 GPU bundle은 강제 재배포하지 않았다. live worker에는 원인 파일만 최소 적용했다.
- `sim/self_observer.py`에 `SIM_WORKER_RUNTIME_DEPENDENCY_MISSING` 분류를 추가했다. `No module named`/`ModuleNotFoundError`/`ImportError`로 action이 physics 전에 실패하면 더 이상 `ACTION_FAILED_UNCLASSIFIED`로 남지 않는다.

### 동일 seed 재검증
- 같은 seed `1099490316`, 같은 `move_forward(speed=35,duration=0.8)` replay가 수정 후 성공했다.
- 성공 trace `20260831T080323Z_ep0026_seed1099490316_move_forward_9efa457bb50bbbb6`: self-observer `CLEAN`, `base_shift_m=0.3406`, 실행 중 `robot_xy`와 1인칭 JPEG hash가 각각 8개의 서로 다른 단계로 관측됐다.
- browser가 실제로 쓰는 경로도 별도 확인했다. 같은 seed에서 `turn_left(speed=35,duration=0.5)` 실행 중 `/api/sim/state` yaw가 7단계, `/api/camera`가 7개 서로 다른 frame, `/api/observer/cctv_front_left/snapshot`이 4개 서로 다른 frame으로 갱신됐다.
- 해당 turn trace `20260831T080353Z_ep0026_seed1099490316_turn_left_3685c0be15fe1c30`: self-observer `CLEAN`, yaw change `26.25 deg`.
- 검증 뒤 seed `1099490316`으로 다시 reset해 사용자의 장면을 복원했다.

### 회귀/경계
- bundle + provider regression: 29 tests OK.
- observer 분류를 포함한 focused regression: 29 tests OK.
- REAL MasterPi에는 어떤 motor/servo 명령도 보내지 않았고 REAL service도 재시작하지 않았다.

### 2026-08-31 — two-stage arm-face controller deployed to REAL and Azure SIM without physical actuation

- Supersedes the earlier deployment-boundary note that signature `bdaafa970fd7fb86ba23df2d690a793215319b34c97ec38b3d6110da0e88b580` had not yet been deployed.
- REAL: actuator ownership was checked free first. The immutable Pi package `bdaafa970fd7fb86ba23df2d690a793215319b34c97ec38b3d6110da0e88b580` was uploaded as files only, its SHA-256 manifest and remote `py_compile` passed, and Oracle `ugrp-real.service` was reloaded. Remote source readback contains `ARM_FACE_CAPTURE_TARGET_NY = 0.30` and the final hand-eye-depth refinement path. No search/track/approach/pick, motor, or servo action was executed as part of deployment.
- SIM: because unrelated 3x work is concurrently changing the canonical GPU bundle, the full bundle was deliberately not rolled out. Only `scripts/red_block/physical_state_machine_reference.py` and `scripts/red_block/approach.py` were copied to the active Azure A10 worker, exact local/remote SHA-256 equality and remote `py_compile` were verified, and only the Azure SIM worker service was restarted so its resident imports load the new controller.
- After the SIM restart, the bridge observed a fresh Azure worker instance `a996c99a45e09f35`, episode `27`, `remote_authoritative=true`, queue depth `0`, and no inflight command. This is deployment verification only; no new SIM manipulation episode was run in this deployment step.
- Physical validation boundary remains unchanged: the earlier REAL run directly validated arm-relative face alignment (`-75deg` floor edge, `+13deg` arm bearing, `2deg` error), but the newly added `ny~=0.30 face staging -> straight ny~=0.442 hand-eye refinement -> grasp` sequence has not yet been physically executed end-to-end. The previous low-battery condition is therefore not being hidden by a deployment claim.


## 2026-08-31 — 3× R1/R2/R3/TEAM tabs: independent sessions completed

### UI/session contract
- `R1`, `R2`, and `R3` are independent agent/chat sessions in both SIM and REAL. Each mode/robot pair owns its own in-flight request, abort controller, started state, and live log DOM.
- Tab navigation is presentation-only. Switching away from a running robot no longer sends `/api/cancel` and no longer aborts its SSE stream. The live DOM is moved into a detached session root, so hidden output continues to update and is restored intact when the operator returns.
- The Stop button still targets only the currently visible robot session. A small running dot and `실행 중` note show when a robot in the current mode still has an active request.
- `TEAM` remains an aggregate/coordination view over the neutral peer bus; it is not a fourth planner and does not centrally choose actions for R1/R2/R3.

### Shared-world execution boundary
- SIM R1/R2/R3 use separate harness/chat processes (`8082/8084/8085`) but route actions with `robot_id` into one shared MuJoCo world/worker. This preserves independent reasoning sessions while keeping all robots and blocks in the same physical state.
- The bridge/worker intentionally serializes low-level MuJoCo actions at the shared-world boundary because migrated REAL controller compatibility still contains process-global control state. This does not cancel or merge agent sessions; concurrent requests can wait in the shared action queue.

### Truthful slot status
- TEAM SIM ONLINE status now requires namespaced state/frame publication for that specific robot. A connected legacy single-robot GPU worker is no longer enough to make R2/R3 appear online falsely.
- REAL slot status continues to respect `robot_configured`; disabled placeholder R2/R3 slots are not presented as real connected hardware.

### Validation / runtime
- Inline browser JavaScript: `node --check` passed.
- `tests/test_sim_ui_consistency.py`: 11/11 passed, including the independent in-flight tab regression.
- `tests/test_sim_bridge.py` under `.venv-sim`: 18/18 passed.
- Read-only live probes confirmed SIM children report `r1/r2/r3` independently and bridge health contains namespaced state + frame metadata for all three robots with `inflight=None` before reload.
- `ugrp-sim-coworker.service` was restarted only after confirming no bridge inflight action. After reload, ports 8082/8084/8085 are live and `/api/team?mode=sim` reports all three current namespaced slots online with `worker_online=true`.
- No SIM motion command and no physical MasterPi motor/servo command was issued during this 3× tab work. REAL service was not restarted.

### 2026-08-31 — 3× tabs follow-up: mode-isolated peer bus and shared-reset safety

- This is a platform/demo hardening addendum to the 3× R1/R2/R3/TEAM tab entry above, not a research result.
- The TEAM peer bus is now namespaced by execution world. SIM uses `/tmp/ugrp-team-bus-sim.json` and REAL uses `/tmp/ugrp-team-bus-real.json` by default (with environment overrides available), so a proposal, goal, ACK, or event produced in simulation cannot enter the physical planner context merely because both stacks run on the same Oracle host. Legacy un-namespaced paths remain only as a compatibility path for standalone tests/tools.
- `harness.web` selects the namespace from the active mode for TEAM reads/goals/resets and for each robot planner's peer context. `scripts.robot_actions` also propagates the current UI/team namespace for peer send/ACK tools.
- ACK authority is now addressed: a sender cannot ACK its own proposal, and a robot cannot ACK a direct message addressed to another robot. Broadcast (`recipient=all`) remains acknowledgeable by peers.
- SIM seed reset is treated as a new shared-world episode: the SIM peer bus is cleared while the REAL bus is untouched. The browser refuses a shared-world reset while any R1/R2/R3 SIM session is still running; after a successful reset, all three SIM tab logs/session-start state are cleared together instead of leaving stale per-robot episode history visible.
- The served live UI at `8082` was read back after the change and contains the independent-session logic, no-cancel tab-navigation contract, per-tab running marker, shared-reset guard, and mode-qualified TEAM goal request.
- Read-only live status after the change: SIM R1/R2/R3 are all online through their independent harness backends; REAL R1 is online while unconfigured REAL R2/R3 remain offline placeholders. SIM and REAL TEAM views both had zero current peer messages at readback; `/tmp/ugrp-team-bus-sim.json` exists as the active namespaced SIM bus.
- Final focused regression: `tests.test_multi_robot_platform + tests.test_sim_ui_consistency = 17/17`, `OK`. Harness regression: `tests.test_harness = 81/81`, `OK`. `py_compile` for the modified Python modules and `node --check` for the served inline browser JavaScript also passed.
- Remaining architecture boundary: the three SIM agents have independent chat/planner sessions and observations, but low-level MuJoCo actions are still serialized at the shared-world command lock. Peer messages are visible to another robot on its next planner turn; there is not yet an autonomous peer-message wake-up loop that starts another robot's turn by itself.

## 2026-08-31 — autonomous VLM `track` 반복 루프: alignment state semantics + policy-free stagnation feedback

### 관찰된 현상
- 사용자 UI에서 하나의 명령이 `track 실행 시작 -> 정렬 · 실행`을 반복한 뒤 마지막에 `중지했습니다`로 끝났다.
- 기존 default planner는 이미 deterministic goal compiler/recovery가 꺼져 있었으므로, 반복은 hidden rule sequence가 아니라 VLM이 매 turn 같은 public action을 다시 고른 결과였다.
- 첫 번째 상태 모순은 `track` 성공 직후 발생했다. `StateEstimator.update_tool_result()`는 TRACK `ACHIEVED`를 받아 `target.centered=true`로 만들었지만, 다음 passive RGB refresh가 raw `cx`의 `|cx-0.5| <= 0.065`만으로 centered를 다시 계산해 false로 덮어쓸 수 있었다.
- 이 판정은 실제 TRACK controller 계약과 달랐다. `scripts/red_block/track.py`는 optical-center lock뿐 아니라 안전한 pan/tilt servo-limit lock도 성공 postcondition으로 인정한다. 따라서 `centered`는 단순 pixel-centre boolean이 아니라 controller가 검증한 alignment handoff여야 한다.
- 두 번째 반복 경로는 precondition 거부였다. target이 안 보이는 상태에서 VLM이 `track`을 반복하면 Executive는 매번 `TARGET_NOT_VISIBLE`로 올바르게 거부했지만, 거부된 proposal은 tool step budget을 소모하지 않아 같은 proposal이 `max_iters`까지 반복될 수 있었다.

### 수정
- `harness/state.py`
  - `_alignment_verified` latch를 추가했다.
  - TRACK `ACHIEVED` 또는 APPROACH의 명시적 `center_verified=true`는 alignment postcondition을 latch한다.
  - passive camera refresh는 raw cx가 optical center에서 벗어나 있어도 이 검증된 postcondition을 지우지 않는다.
  - target loss, failed TRACK, SEARCH/APPROACH/PICK/PLACE/search_destination/put_down, 또는 chassis motion처럼 실제 alignment handoff를 무효화할 수 있는 사건에서만 latch를 해제한다.
  - 설계 의도는 코드 주석에 남겼다: public `target.centered`는 `cx ~= 0.5` 자체가 아니라 검증된 alignment contract다.
- `harness/loop.py`
  - autonomous mode에서 동일 action이 동일 failure/required_state/관련 world state 아래 다시 거부되면 `stagnation.same_rejected_action_count`, `relevant_state_changed=false`라는 객관적 반복 사실을 planner context에 추가한다.
  - 특정 recovery action은 제시하지 않으며 `recommended_recovery`도 계속 숨긴다. 즉 Executive가 `search`/`approach` 등을 대신 고르지 않고 VLM이 다음 행동을 계속 결정한다.
  - 성공한 tool도 `recent_action_evidence`를 통해 same-action streak와 before/after sensor 변화가 planner에 노출되어, 반복의 실제 진행 여부를 VLM 스스로 판단할 수 있다.
- `tests/test_state_executive.py`
  - off-centre passive refresh가 verified TRACK alignment를 지우지 않는 회귀 추가.
  - target loss/new search가 latch를 적절히 해제하는 회귀 추가.
  - 동일 precondition rejection의 stagnation fact가 노출되되 recovery policy는 주입되지 않는 회귀 추가.

### 검증
- focused harness/state regression: `tests.test_state_executive + tests.test_harness = 108 tests`, 모두 OK.
- 실제 Azure 3x shared-world R1, seed `1099490316`에서 TRACK을 실행:
  - worker result: `outcome_status=ACHIEVED`, `center_verified=true`, measured target `cx~=0.455`.
  - StateEstimator는 TRACK 직후 `centered=true`; 이후 별도의 fresh R1 snapshot (`cx~=0.456`)을 passive refresh로 다시 넣은 뒤에도 `centered=true`, centered streak가 1 -> 2로 유지됐다.
  - trace `20260831T082509Z_ep0001_seed1099490316_track_bd1fd60fed81e556`: self-observer `CLEAN`, action result true, 물리 anomaly 없음. `scripts/analyze_sim_trace.py` contact sheet 생성/검토 완료.
- 수정 후 fresh autonomous run은 이전의 `track x N` 패턴 대신 `search -> search -> move_forward`로 다른 행동을 스스로 선택하는 것까지 관찰됐다. 이 run은 3x 동시 rollout 과정에서 `ugrp-sim-coworker.service`가 외부에서 08:21:53 재시작되어 HTTP turn이 중간 종료되었으므로 full-task completion 근거로는 사용하지 않는다.
- 현재 runtime은 Azure `masterpi_multi_v2_shared_world` 3-robot worker이며 R1/R2/R3 namespaced snapshots가 모두 살아 있다.
- 진단 후 shared world를 동일 seed `1099490316`으로 reset했고 R1 pose `[0.0, -0.42]`, yaw `0.0`을 확인했다.

### 경계
- 이번 수정은 반복을 깨기 위해 `TARGET_NOT_VISIBLE -> search` 같은 상황별 행동 정책을 다시 넣지 않았다.
- VLM은 여전히 public action을 자유롭게 고른다. harness는 verified postcondition을 일관되게 보존하고, 같은 infeasible proposal이 진전 없이 반복되었다는 사실만 알려준다.
- REAL MasterPi에는 어떤 motor/servo 명령도 보내지 않았다.

## 2026-08-31 — REAL approach→pick loop: stale pose trust + too-close capture recovery fixed

### Observed failure chain
- User-reported REAL behavior was reproduced from the newest traces rather than inferred from planner behavior alone.
- `outputs/real_traces/20260831T082003Z-real-d77a5f81` repeatedly failed `approach` with `precision approach requires a trusted full arm pose ... pose telemetry is stale`; `pick` was then correctly rejected with `PREGRASP_PLAN_MISSING` because no successful approach had produced `task.phase=PREGRASP_READY` / a fresh FK-IK handoff.
- The same run and `20260831T081815Z-real-ccc7777a` exposed a second independent failure after search refreshed pose trust: close-view `approach` could enter around `ny=0.512/0.515` while the final hand-eye target is `0.442`, then fail immediately as “passed the hand-eye grasp window”. The planner therefore saw repeated unsuccessful approach/pick attempts even though the target remained close and visible.

### Root causes
1. `approach.py` treated the 15 s commanded-pose trust timestamp as an unrecoverable prerequisite and told the caller to run search. The robot already retained a complete known pose, and `search.py` already had the safer primitive needed to restore trust: concurrently re-command the exact known pose and re-record it.
2. `visual_capture_approach()` supported bounded forward creep but had no symmetric bounded retreat when the target entered the fixed close camera view slightly too near. A recoverable depth error was therefore promoted to a whole-skill failure.

### Fix
- `scripts/red_block/approach.py`
  - missing arm-pose fields and unknown camera-pose families still fail closed;
  - a complete known search/capture pose whose timestamp alone is stale is reasserted exactly with `Robot.reassert_pose_together()` and freshness is rechecked before metric projection;
  - for capture-family starts, the causal near-look handoff is validated before reassertion because the actuator command intentionally invalidates the token.
- `scripts/red_block/physical_state_machine_reference.py`
  - close capture now has at most 5 verified `backward@35, 0.06 s` retreat pulses;
  - every pulse requires fresh visual progress and retains the existing horizontal-drift and target-lock gates;
  - recovery retreats into the actual requested window, including the earlier arm-face staging target (`ny~=0.30`), not merely below the global too-close ceiling.
- `scripts/robot_actions.py`
  - residual stale-pose failure is exposed objectively as `APPROACH_POSE_STALE / arm.pose_trust=FRESH`;
  - inability to recover safe close depth is exposed as `TARGET_TOO_CLOSE / target.capture_depth=SAFE`;
  - neither adds a forced planner recovery, preserving VLM-owned next-action selection.
- Removed an obsolete test assertion that froze the public tool count at 17. The current invariant is that legacy/hidden composite executors (`fetch`, `carry`, hidden program runner) are not AI-facing; peer/team tools may legitimately change the count.

### Validation and activation
- Syntax compilation passed for all modified source/test files.
- Focused approach/failure/tool suite: **114/114 passed**.
- Broader REAL manipulation/executive/fusion/place regression: **200/200 passed**.
- MasterPi actuator lease was verified free before deployment and again after activation. No running search/track/approach/pick/task-runner process remained.
- Immutable REAL package was deployed as files only and remotely verified by full SHA-256 manifest plus `py_compile`: `d0e4081b19e6a7421091d5b1300019e413cc1428782f13cb44d647177ed2cfd2`.
- Oracle `ugrp-real.service` was restarted using the guarded activation script, which sends no MasterPi motor/servo/STOP command. Post-activation state: service active/running, `execute_allowed=true`, `talk_only=false`, low-latency camera active with a sampled frame age of 35.3 ms, actuator lease free.
- No physical manipulation action was issued during this fix or deployment. The software loop cause is fixed and activated; actual end-to-end `approach -> PREGRASP_READY -> pick` on hardware remains to be confirmed by the next operator-requested REAL pickup trace.


### 2026-08-31 — VLM 판단 근거 확장 + runtime 재현성 보강

- 사용자 결정: 반복 행동을 횟수로 강제 차단하는 anti-loop 정책보다, VLM이 직전 행동의 실제 진전 여부를 판단할 수 있는 관측 근거를 제공하는 것을 우선한다. `max_steps`와 Executive precondition은 안전/예산 경계로 유지하지만, 동일 action 자체를 금지하지 않는다.
- `harness/loop.py`의 planner-facing `world_state`에 `goal_evidence`와 최근 최대 3개의 `recent_action_evidence`를 추가했다. 각 action evidence에는 `same_action_streak`, `outcome_status`, `failure_code`, 관측 count 전후, target/grasp/task의 실제 변경 필드, `center_error_reduction`, `target_cy_delta`, `target_area_ratio_delta`가 포함된다. 특정 다음 action을 추천하거나 반복을 금지하는 필드는 없다.
- `harness/protocol.py`도 이 필드가 관측 사실이며 policy가 아님을 명시한다. 회귀에서는 동일 `search`를 두 번 연속 선택해도 둘 다 실행되며, 두 번째 판단 시 `same_action_streak=2`, 변경 필드 없음, 동시에 `goal_evidence.sensor_status=ACHIEVED`를 모델이 볼 수 있음을 검증했다.
- `harness/web.py`의 SIM `/api/status`가 실물 endpoint `ugrp1`을 R1/R2/R3 모두에 잘못 표시하던 metadata를 수정했다. `robot_endpoint`는 REAL mode에서만 노출한다. SIM coworker를 shared-world queue idle 상태에서 재시작한 뒤 R1/R2/R3 모두 `talk_only=false`, namespaced camera live, `robot_endpoint=null`을 확인했다. Azure worker는 `masterpi_multi_v2_shared_world`, `remote_authoritative=true`, queue 0/inflight none이었다.
- `tests/test_sim_self_observer.py`의 pytest-style free-function 12개가 canonical `unittest discover`에서 누락되던 문제를 `load_tests()` adapter로 수정했다. pytest 전용 기능은 쓰지 않으므로 별도 pytest 설치 없이 두 runner에서 모두 실행 가능하다. 관련 harness/state/observer 회귀 120 tests, broader harness/bridge/UI/observer 회귀 122 tests 모두 OK.
- `.webui_secret_key` 권한을 `0644 -> 0600`으로 제한했다.
- GPU worker recovery의 dependency drift를 막기 위해 `scripts/gpu_worker_bundle.py`의 production requirements를 현재 활성 Azure A10 worker에서 직접 확인한 조합으로 pin했다: MuJoCo 3.12.0, NumPy 2.5.2, Pillow 12.3.0, websockets 17.1, opencv-python-headless 5.0.0.93, gymnasium 1.3.0. 중복되어 있던 `scripts/red_block/put_down.py` bundle entry도 제거했다. bundle/recovery 관련 28 tests OK. 이 변경은 다음 worker recovery/deploy부터 재현 가능한 조합을 사용하며, 현재 정상 worker를 단지 pin 적용을 위해 강제 재배포하지 않았다.
- Oracle `.venv-sim`과 REAL Pi의 Python/OpenCV/NumPy 버전은 여전히 서로 다르다(Oracle Python 3.12.3/OpenCV 5.0.0/NumPy 2.5.2, Pi Python 3.13.5/OpenCV 4.10.0/NumPy 2.2.4). 이것이 현재 실패의 직접 원인이라는 증거는 없으므로 억지로 Pi 환경을 변경하지 않았다. 플랫폼별 compatibility boundary로 계속 관리한다.
- REAL Oracle harness 재적용은 첫 시도에서 다른 process의 actuator lease 때문에 안전하게 거부됐다. 이후 lease가 free임을 다시 확인한 뒤 `activate_real_spatial_stack.sh`로 Oracle-side service만 재시작했다. 최종 `REAL talk_only=false`, `execute_allowed=true`, camera live를 확인했으며 이 reload 과정에서 MasterPi motor/servo/STOP 명령은 보내지 않았다.

## 2026-08-31 — REAL approach→pick 반복 원인 수정: close-view median 지연 + pose trust 복구

### 실제 증거와 원인
- 사용자 증상은 `pick` 구현이 호출되지 않는 단일 문제가 아니었다. 최신 REAL run `20260831T082003Z-real-d77a5f81`에서 `pick`은 `PREGRASP_PLAN_MISSING`으로 거절됐고, 이는 직전 `approach`가 `ACHIEVED`하지 못해 `task.phase=PREGRASP_READY`가 생성되지 않았기 때문이다.
- 직전 run `20260831T081815Z-real-ccc7777a`와 최신 run의 close approach는 최종 hand-eye target `ny=0.442` 부근에서 `ny=0.512~0.515`까지 지나쳐 fail-closed했다.
- 결정적인 frame-level evidence는 skill trace `20260831T082105Z-approach-be2ce487`다. 한 60 ms 전진 뒤 잠금된 세 프레임의 `ny`가 `0.352 -> 0.390 -> 0.421`로 진행했다. 안전한 grasp-window 하한은 `0.442-0.025=0.417`이므로 freshest frame `0.421`에서는 이미 전진을 멈춰야 했다. 그러나 `_capture_sample()`이 세 프레임 median `0.390`을 control depth로 반환해 한 번 더 60 ms 전진했고, 다음 view가 `ny~=0.512`로 overshoot했다.
- 실패 뒤 capture-family arm pose에 남은 상태에서는 commanded-pose timestamp가 수백 초 stale해질 수 있었다. 실제 run 시작의 `initial_pose_age_s`는 약 `413.8 s`였다. 이 상태에서 approach가 계속 실패하면 successful FK/IK handoff가 만들어지지 않아 pick gate가 계속 닫히는 반복이 생겼다.

### 수정
- `scripts/red_block/physical_state_machine_reference.py`의 `_capture_sample()`은 검출 noise를 위한 median을 유지하되, chassis pulse 직후 control depth가 가장 최신 잠금 프레임보다 뒤처지지 않도록 `control_ny=max(median_ny, latest_ny)`를 사용한다.
  - 전진 중에는 최신 프레임이 이미 가까움을 보이면 더 일찍 멈추므로 overshoot에 보수적이다.
  - 후진 recovery 중에는 median보다 더 많은 retreat을 성급하게 주장하지 않으므로 역시 보수적이다.
- 현재 controller의 pose-trust 경로는 stale timestamp 자체를 물리 pose 불일치로 오인해 무한히 실패하지 않도록, 알려진 exact arm/camera pose를 `reassert_pose_together()`로 재명령한 뒤 freshness를 다시 확인한다. capture-family에서는 causal near-look handoff를 먼저 검증한 뒤 exact pose를 재assert한다.
- stale trust가 실제로 복구되지 않으면 planner-facing failure는 generic `SKILL_FAILED`가 아니라 `APPROACH_POSE_STALE / arm.pose_trust=FRESH`로 분류된다. 정상 VLM mode에서 deterministic recovery action을 숨겨 실행하지 않는 기존 경계는 유지한다.
- current close controller에는 too-close entry가 생겨도 blind grasp로 통과시키지 않고, 요청된 hand-eye window까지 bounded straight backward recovery 후 재측정하는 fail-closed path도 유지된다.

### 검증 / REAL 활성화
- 실제 `.352/.390/.421` 시퀀스를 고정 회귀 테스트로 추가했다. 이 테스트는 control depth가 `.421`이 되어 safe-window 하한 `.417`을 넘음을 확인한다.
- focused REAL approach/failure/tool suite: `115/115` passed.
- state/executive/place regression: `78/78` passed. 합계 `193/193` passed.
- 새 immutable MasterPi package signature: `211dcfcb54824fefc60f43b59d14150acbde946610629bb6512171e5bbf0b8b8`.
- actuator lease가 free인 것을 확인한 뒤 package를 **파일만** 배포했다. 원격 `.ugrp-files.sha256` 검증, `py_compile`, 그리고 remote source의 `control_ny = max(median_ny, latest_ny)` readback이 통과했다.
- `scripts/activate_real_spatial_stack.sh`로 Oracle `ugrp-real.service`를 guarded reload했다. reload 후 service `active`, `execute_allowed=true`, `talk_only=false`, low-latency camera active (`camera_frame_age_ms` 약 40 ms), actuator lease도 다시 free임을 확인했다.
- 이 수정/배포 과정에서는 MasterPi motor/servo/search/track/approach/pick 명령을 실행하지 않았다. 따라서 다음 실제 grasp에서 `approach=ACHIEVED -> PREGRASP_READY -> pick`이 end-to-end로 이어지는지는 아직 물리 검증 전이며, 현재는 코드/회귀/배포 상태까지만 확인됐다.

## 2026-08-31 — R1/R2/R3 live peer-context refresh inside active turns

### 관찰된 경계
- 기존 3× 협업 구조는 R1/R2/R3 독립 세션, shared MuJoCo world, TEAM peer bus까지 정상 동작했지만 `/api/turn` 진입 시점에 `team_bus.context_for(...)`를 한 번만 읽었다.
- 따라서 R2가 이미 여러 VLM/tool step으로 한 turn을 수행 중일 때 R1이 새 peer message를 보내면, R2는 같은 turn의 다음 판단에서 그 메시지를 볼 수 없고 다음 operator turn까지 기다려야 했다.
- TEAM을 중앙 planner로 만들지 않는다는 기존 설계는 맞지만, 이 stale snapshot은 동등 peer 간 자연어 협업의 반응성을 불필요하게 제한했다.

### 수정
- `harness/loop.py`
  - 기존 static `planner_context` 호환은 유지하고, live caller용 `planner_context_provider`를 추가했다.
  - provider는 **매 VLM 판단 직전** 호출되며, 반환된 peer evidence를 ephemeral system context로 한 번만 삽입한다. persistent history에는 누적하지 않는다.
  - embodied `auto_observe`가 long-context compaction을 수행한 **뒤에** live context를 삽입하도록 순서를 고정했다. 따라서 실제 로봇/SIM autonomous path에서도 peer context가 압축 과정에서 사라지지 않는다.
- `harness/chat.py`가 provider를 `run_loop()`까지 그대로 전달한다.
- `harness/web.py`는 R1/R2/R3 각자의 `robot_id`와 `ui_mode` namespace로 `team_bus.context_for()`를 다시 읽는 `planner_context_snapshot()`을 제공한다.
- 결과적으로 같은 active turn 도중 다른 로봇이 보낸 목표/메시지/최근 team event가 recipient의 **다음 VLM decision**부터 보인다. 특정 행동을 강제하거나 중앙 planner가 대신 결정하지 않는다.
- idle 로봇을 peer message만으로 자동 기동시키는 wake-up scheduler는 여전히 없다. 이는 별도 기능 경계다.

### 회귀 / 활성화
- 새 회귀 `test_live_planner_context_refreshes_before_each_model_decision`은 `auto_observe=True` context compaction 경로에서 연속 두 planner call이 서로 다른 peer message sequence를 읽고, 매 call에 stale context가 중복 누적되지 않음을 검증한다.
- `tests.test_harness + tests.test_state_executive = 109/109`, OK.
- `tests.test_multi_robot_platform + tests.test_sim_ui_consistency + tests.test_sim_bridge = 35/35`, OK.
- `py_compile`은 `harness/loop.py`, `harness/chat.py`, `harness/web.py`, 관련 test에서 통과했다.
- 활성화 전 bridge에 기존 R1 SIM action(`approach`, 이후 `track`)이 inflight인 것을 확인해 강제 restart/cancel하지 않았다. 기존 run이 자연 종료되어 `inflight=None`, `queue_depth=0`이 된 뒤에만 `ugrp-sim-coworker.service`를 재시작했다.
- restart 후 R1/R2/R3는 각각 `r1/r2/r3`, `ui_mode=sim`, `coordination=decentralized_peer_bus`로 다시 online이며 TEAM은 세 slot 모두를 읽는다. Azure bridge는 `remote_authoritative=true`, `remote_ws_connected=true`, `inflight=None`, `queue_depth=0`을 유지했다.
- 이 변경/활성화 과정에서 REAL MasterPi motor/servo 명령은 보내지 않았고 `ugrp-real.service`도 수정/재시작하지 않았다.

## 2026-08-31 — UGRP live source를 Git canonical repository로 전환

- 사용자 결정으로 `/home/ubuntu/projects/ugrp`를 더 이상 “Git 초기화 금지” 디렉터리로 유지하지 않고 Git을 canonical source로 사용한다.
- 새 이력을 0부터 만들지 않고, 기존 격리 checkout의 기준 스냅샷 `1099fa3` (`chore: snapshot latest UGRP runtime before latency stack`)의 Git metadata를 live project에 안전하게 연결했다. 기존 `.git` 금지 안내 파일은 프로젝트 밖 migration backup에 보존했다.
- 현재 14 GB 프로젝트에서 대부분의 용량은 OS image, `.venv-sim`, `work/`, `outputs/` 등 runtime/build artifact였다. Git metadata는 전환 직후 약 3.1 MB였고, 기존 tracked source는 약 5.2 MB 규모였다.
- `.gitignore`를 source/config/docs 중심 정책으로 갱신했다. `outputs/`, `stress_logs/`, `.venv-*`, secrets, model weights, Raspberry Pi image/kernel build artifacts, temporary conflict/orig files는 신규 history에서 제외한다.
- `.githooks/pre-commit`을 추가하고 `core.hooksPath=.githooks`로 설정해 staged blob 하나가 20 MiB를 초과하면 commit을 거부하도록 했다. 대형 asset이 정말 버전 관리 대상이면 명시적으로 Git LFS 등을 선택해야 한다.
- SIM/REAL은 Git branch를 분리하지 않고 동일 commit을 기준으로 배포한다. 환경별 설정과 배포 상태만 구분해 기록한다.
- 원격 repository는 아직 설정하지 않았고, 이번 전환에서 외부 push는 수행하지 않는다.

## 2026-08-31 — SIM face alignment pure-strafe removed after user-visible regression

- User observed that SIM `approach` still moved laterally after the preceding arm-face deployment. Inspection confirmed the implementation itself was still wrong, not merely stale deployment: both `align_block_face()` and `align_block_face_with_arm_yaw()` still called `orbit_strafe_measure_bearing()`, which directly issued mecanum `left/right` wheel commands. A regression test even encoded the obsolete behavior as `...strafes_then_moves_servo6...`.
- Face-alignment runtime no longer calls the strafe helper. Both face controllers now use `face_reposition_dogleg()`: bounded `rotate -> backward -> inverse rotate`, followed by stopped visual remeasurement. The final arm-relative path then uses servo 6 for horizontal pointing and `visual_capture_approach()` for straight forward/back depth restoration. Post-reposition face-error improvement is the progress authority; two stalled attempts fail closed.
- The historical `orbit_strafe_measure_bearing()` helper and explicit public `strafe_left/strafe_right` primitives remain available for diagnostics/manual commands only. They are not reachable from `approach` face alignment.
- Regression locks the new invariant: the dog-leg motor sequence contains no direct `left/right`, both face controllers never call `orbit_strafe_measure_bearing()`, and no-improvement cases stop. Focused REAL/SIM/shared-controller regression: **105/105 passed**. Current controller SHA-256 is `bcca6d78bb17d239f5c881508f9ce0645b7a825169f3b1df2c6263abad140428`.
- Deployed only the corrected shared controller file to the active Azure A10 shared-world worker after waiting for bridge `inflight=None` and queue depth 0. Remote SHA-256 matches Oracle exactly; remote `py_compile` passed; the fresh authoritative worker is multi-robot contract `masterpi-v2-three-robot-shared-world-v2`, episode 2. REAL was not redeployed or actuated in this correction.
- Replayed the user's problem seed `2101568344` on R1 as `reset -> search(red) -> track(red) -> approach(red)`. New approach trace: `20260831T083741Z_ep0002_seed2101568344_approach_a63240a348cb1e65`. Worker logs show `arm-face reposition=...` and contain no `orbit physical ... direction=left/right` or `lateral pulse` messages. Trace-sampled wheel states contain only stop, rotate-left and forward patterns; no mecanum left/right pattern appears in captured samples. A contact sheet was generated for temporal review.
- The replay still failed safely later because the fixed-pose face estimator eventually returned zero valid samples (`only 0 valid block-face samples`); this is a separate face-observation/convergence problem, not evidence of the removed pure-strafe path. The whole approach shifted ~0.56 m because it began ~0.71 m from the target and performed the normal approach trajectory; that total displacement must not be mislabeled as a single side strafe.

## 2026-08-31 — TEAM을 Grok-style shared bot chat으로 전환

### 목표 / 구조
- 사용자 요청대로 기존의 `공통 목표 + peer message 목록` 중심 TEAM 화면을, R1/R2/R3가 하나의 방에서 각각 독립 bot identity로 대화하는 shared chat 형태로 전환했다.
- TEAM은 여전히 중앙 planner가 아니다. shared transcript와 delivery queue를 보관하고, 어떤 peer에게 새 turn을 전달할지만 라우팅한다. 실제 답변/행동 선택은 각 R1/R2/R3의 독립 VLM + world state + Executive가 한다.
- `@R1`, `@R2`, `@R3`는 해당 peer만 호출한다. `@TEAM`/`@ALL` 또는 mention 없는 일반 room message는 R1/R2/R3 모두에게 독립 turn 기회를 준다.
- `send_peer_message`는 shared room에도 peer handoff로 표시되고 addressed peer가 idle이면 wake한다. 반면 일반 bot final reply는 room에만 표시되고 다른 bot을 다시 자동 wake하지 않는다. 따라서 자연어 handoff는 가능하지만 단순 답변 때문에 reply storm이 생기지는 않는다.

### 구현
- `harness/team_bus.py`
  - namespace별 `chat` transcript와 `wakeups` delivery state를 peer bus에 추가했다.
  - mention parser, operator room post, agent room reply, peer-message mirror/wakeup을 추가했다.
  - 이미 active turn인 robot이 planner context에서 같은 room message를 읽으면 queued/claimed wake ticket을 `absorbed` 처리해 중복 turn을 막는다.
- `harness/web.py`
  - `/api/team/chat` endpoint를 추가했다.
  - 각 ChatState에 nonblocking per-robot turn lock을 두어 operator turn과 auto-wake가 겹치면 두 번째 요청은 `409 robot_busy`로 fail-fast한다.
  - R1 neutral dispatcher가 SIM/REAL TEAM wake queue를 전달한다. SIM은 해당 robot의 독립 `/api/turn`을 사용하고, REAL auto-wake는 항상 `chat_only=true`라서 물리 actuator를 자동 실행하지 않는다.
  - planner context에 최신 shared `team_chat`도 매 VLM decision 직전 전달한다.
- `harness/static/index.html`
  - TEAM을 공용 chat timeline + composer + @mention 안내 + R1/R2/R3 camera/status 카드로 재구성했다.
  - queued/claimed delivery는 각각 `QUEUED`/`THINKING`으로 보이며 operator와 각 robot reply가 동일 timeline에 identity별로 표시된다.

### 회귀
- `tests.test_harness + tests.test_state_executive`: **111/111**, OK.
- `tests.test_multi_robot_platform + tests.test_sim_ui_consistency + tests.test_sim_bridge + tests.test_llm_responsiveness`: **46/46**, OK.
- Python `py_compile` 및 served inline JavaScript `node --check` 통과.
- 회귀에는 TEAM mention routing, no-mention 3-agent delivery, peer handoff wake, active-turn wake absorption, REAL chat-only auto-wake, overlapping-turn rejection이 포함된다.

### live SIM 검증
- activation 전 shared MuJoCo bridge `inflight=None`, `queue_depth=0` 및 R1/R2/R3 `turn_busy=false`를 확인한 뒤 `ugrp-sim-coworker.service`만 재시작했다. GPU bridge/worker와 REAL service는 재시작하지 않았다.
- served UI readback에서 `TEAM chat`, `team-chat-form`, `/api/team/chat`, `@R1` contract를 확인했다.
- 단독 mention: `@R2 TEAM 채팅 연결 확인이야...`를 보냈고 R2가 같은 TEAM timeline에 `저는 R2이며 TEAM 채팅 연결이 확인되었습니다.`라고 실제 응답했다. wake는 `done`, R2는 다시 idle이 됐다.
- no-mention group message: `TEAM 그룹 채팅 확인이야. 각자 자기 이름만 한 단어로 답해줘.`를 보냈고 세 독립 wake가 모두 1회에 `done`; 같은 room에 `R1`, `R2`, `R3`가 각각 별도 bot reply로 올라왔다.
- 두 live test 모두 `turn_intent=(False, False)`였고 MuJoCo bridge는 전후 `inflight=None`, `queue_depth=0`; R1/R2/R3 `last_action.name=None`이었다. 즉 이 chat 검증 때문에 SIM robot motion은 발생하지 않았다.
- 이번 작업에서 REAL MasterPi motor/servo 명령은 보내지 않았다.

## 2026-08-31 — TEAM을 Grok Bot식 shared group chat + idle peer wakeup으로 확장

### 요구 / 설계
- 사용자가 원한 TEAM은 상태 대시보드가 아니라 R1/R2/R3가 같은 방에서 각각 독립 Bot처럼 대화하고, `@R2`처럼 특정 Bot을 부르며, Bot끼리도 업무를 넘길 수 있는 group-chat UX다.
- TEAM은 중앙 planner가 아니다. 메시지 저장/라우팅/전달 상태만 담당하고, 호출된 R1/R2/R3는 기존의 독립 `ChatState`, planner history, camera/world evidence, Executive를 사용해 자기 다음 행동을 직접 결정한다.

### 구현
- `harness/team_bus.py`
  - SIM/REAL namespace별 shared room transcript(`chat`)와 wakeup delivery queue를 추가했다.
  - `@R1/@R2/@R3`는 해당 Bot만, `@everyone/@TEAM` 또는 멘션 없는 room message는 세 Bot 모두를 wake 대상으로 만든다.
  - `send_peer_message`는 기존 planner-facing peer message를 유지하면서 같은 내용을 TEAM room에도 mirror하고 수신 Bot을 깨운다.
  - Bot의 일반 final reply는 room에만 표시되고 추가 wakeup을 만들지 않아 자동 reply storm을 막는다.
  - wakeup은 `pending -> claimed -> done/error` 상태를 가지며 atomic file lock으로 claim한다. 이미 진행 중인 robot turn이 같은 메시지를 live planner context로 읽은 경우 그 wakeup을 `absorbed` 처리해 HTTP 409 뒤 중복 재실행되는 race를 막는다.
- `harness/web.py`
  - robot별 `_turn_lock`과 `turn_busy` 상태를 추가해 같은 Bot의 동시 turn을 차단한다.
  - `/api/team/chat` endpoint를 추가했다.
  - neutral dispatcher는 pending wakeup을 해당 robot의 기존 `/api/turn`으로 전달한다. SIM은 `execute=true`; REAL 자동 wake는 항상 `execute=false`, `chat_only=true`다.
  - 각 Bot의 final answer는 `team_autowake_reply`로 같은 TEAM room에 돌아온다.
  - active VLM planner context에는 `team_chat`도 포함된다.
- `harness/static/index.html`
  - TEAM을 실제 공용 대화 UI로 변경했다. operator는 오른쪽, R1/R2/R3는 같은 transcript의 Bot 메시지로 보인다.
  - Bot 상태는 ONLINE/OFFLINE뿐 아니라 QUEUED/THINKING을 표시한다.
  - 입력창은 `@R1/@R2/@R3`, `@everyone/@TEAM`, 멘션 없는 group message를 안내한다.
  - REAL TEAM은 자동응답이 대화 전용이며 물리 동작은 각 REAL 탭에서 명시적으로 해야 한다는 안전 문구를 노출한다.

### 회귀 / live 검증
- 전체 harness/state regression: **112/112 OK**.
- multi-robot/UI/bridge regression: **45/45 OK**.
- Python `py_compile` 및 served inline JavaScript `node --check` 통과.
- targeted live SIM room test: `@R2 TEAM 연결 확인. 네 이름만 R2라고 답해줘.` -> wake target은 R2 하나만 생성되고 R2가 idle에서 자동으로 깨어나 room에 `R2`라고 답했다. turn intent는 `agent=false`, `execute_effective=false`였다.
- no-mention live SIM group test: `TEAM 연결 확인. 각자 자신의 이름만 한 줄로 답해줘.` -> R1/R2/R3 wakeup 3개가 생성되고 세 독립 세션이 각각 `R1`, `R2`, `R3`로 답했다. 세 wakeup 모두 `done`.
- public peer tool live test: R1의 `send_peer_message(recipient=r3, ...)` -> peer message `m000001`이 TEAM room `c000013`에 mirror되고 R3 wakeup이 생성되어, R3가 room에 `R3`로 답했다.
- 위 세 live test는 모두 대화성 문장만 사용했고 SIM bridge는 테스트 전후 `inflight=None`, `queue_depth=0`이었다. MuJoCo actuator action은 실행되지 않았다.
- REAL 물리 로봇에는 이번 작업에서 motor/servo/search/track/approach/pick 명령을 전혀 보내지 않았다. REAL automatic TEAM wake의 `execute=false + chat_only=true`는 unit regression으로 고정했다.

### 2026-08-31 — TEAM Grok-style 병렬 구현 최종 보정
- 직전의 두 TEAM/Grok 섹션은 같은 기능을 병렬로 구현·병합하는 과정에서 기록된 중간 스냅샷이다. 최종 source/runtime 기준은 이 보정 항목을 따른다.
- 최종 회귀: `tests.test_harness + tests.test_state_executive = 112/112`, OK.
- 최종 multi/team/UI/bridge/LLM responsiveness 회귀: `tests.test_multi_robot_platform + tests.test_sim_ui_consistency + tests.test_sim_bridge + tests.test_llm_responsiveness = 48/48`, OK.
- live SIM 검증은 세 경로 모두 성공했다: (1) `@R2` 단독 mention -> R2만 auto-wake/reply, (2) no-mention group message -> R1/R2/R3 각각 auto-wake/reply, (3) public `send_peer_message` R1->R3 -> room mirror + R3 recipient wake/reply.
- live chat 검증 동안 모든 turn은 conversation intent였고 shared MuJoCo bridge는 계속 `inflight=None`, `queue_depth=0`; actuator action은 실행되지 않았다.
- served TEAM UI는 `TEAM chat`, `/api/team/chat`, `QUEUED`, `THINKING`, `@everyone/@TEAM` 안내를 실제 응답 HTML에서 확인했다.
- 최종 runtime readback: R1/R2/R3 `turn_busy=false`, `talk_only=false`; bridge `remote_authoritative=true`, `inflight=None`, `queue_depth=0`.
- REAL TEAM automatic wake는 회귀에서 `execute=false` + `chat_only=true`가 고정되어 있으며 이번 작업에서 REAL MasterPi 물리 명령은 실행하지 않았다.

## 2026-08-31 — TEAM chat를 TEAM 탭에만 격리

- 사용자 피드백: TEAM group chat은 TEAM 페이지에만 있어야 하며 R1/R2/R3 개별 페이지에 TEAM chat이 보여서는 안 된다.
- 원인 1: `team-panel`에 HTML `hidden` 속성은 있었지만 author CSS의 `.team-panel { display:flex; }`가 브라우저 기본 `[hidden] { display:none }`보다 우선할 수 있었다. 따라서 TEAM panel이 robot 탭에서도 렌더될 수 있는 구조였다.
- 수정: 공통 CSS에 `[hidden] { display: none !important; }`를 추가해 `showRobotPanel()`/`showTeamPanel()`의 hidden 상태가 실제 렌더링에서도 강제되도록 했다.
- 원인 2: TEAM auto-wake가 내부적으로 각 robot의 `/api/turn`을 사용하면서 TEAM prompt/final도 해당 R1/R2/R3 `ChatState.history`에 저장하고 있었다.
- 수정: `team_wake=true` turn은 planner가 그 메시지를 일시적으로 받아 판단하는 것은 유지하되 private robot history에는 user/assistant 메시지를 append하지 않는다. 답변은 기존처럼 TEAM transcript에만 `team_autowake_reply`로 기록한다.
- focused regression: TEAM private-history isolation + SIM UI suite **13/13 OK**. multi-robot/UI/bridge/LLM responsiveness **48/48 OK**.
- 실제 Chromium computed-style 검증: R1에서 `team-panel hidden=true/display=none`, TEAM에서 `team-panel display=flex` + `robot-app display=none`, R2로 돌아오면 다시 `team-panel hidden=true/display=none`을 확인했다.
- SIM coworker만 idle 상태에서 재시작해 반영했다. 이후 R1/R2/R3 모두 `turn_busy=false`, bridge `inflight=None`, `queue_depth=0`, `remote_authoritative=true` 확인. REAL 서비스/물리 로봇은 건드리지 않았다.
- 참고: 전체 `tests.test_harness + tests.test_state_executive` 실행에서는 이 변경과 무관하게 동시 작업 중 추가된 `test_action_synchronous_sensor_evidence_skips_redundant_post_action_image`가 ReplayCompleter queue 소진(`IndexError`)으로 1건 실패했다. 해당 실패는 standalone에서도 재현되며 TEAM isolation focused test는 통과한다.

## 2026-08-31 — TEAM/private chat live browser E2E 재검증

- 사용자 요청에 따라 정적 코드/단위 테스트만이 아니라 현재 실행 중인 SIM UI(`127.0.0.1:8082`)를 headless Chromium DevTools로 직접 조작해 탭 전환과 실제 메시지 송수신을 검증했다.
- 렌더링 검증: R1/R2/R3 각각에서 `team-panel hidden=true`, computed `display=none`, layout visibility=false, `robot-app display=grid`; TEAM에서만 `team-panel display=flex`, `robot-app display=none`을 확인했다.
- TEAM -> R2 E2E: TEAM 입력창에서 `@R2 E2E-090807 ...`을 실제 submit했다. 생성된 chat은 `c000009`, wakeup은 R2 하나(`w000007`)뿐이었고 status=`done`; R2는 TEAM transcript에 정확히 `R2-E2E-090807-OK`로 응답했다. 같은 token은 R1/R2/R3 private UI 어디에도 나타나지 않았고 세 private DOM은 전후 변경되지 않았다.
- R2 private -> R2 E2E: R2 개별 입력창에서 `PRIVATE-090920 ...`을 실제 submit했고 R2 private UI에만 `R2-PRIVATE-090920-OK`가 나타났다. R1/R3 private UI에는 token이 없었고 TEAM UI/API transcript에도 token이 없었으며 TEAM chat count도 증가하지 않았다.
- 두 live E2E 모두 실행 전후 R1/R2/R3 `base_xyz`, `base_yaw`가 동일했고 motor command는 0, `last_action.name=None`이었다. 즉 검증 중 MuJoCo actuator motion은 발생하지 않았다.
- 회귀 재실행: `tests.test_multi_robot_platform + tests.test_sim_ui_consistency + tests.test_sim_bridge + tests.test_llm_responsiveness = 48/48 OK`; `WebTests.test_team_wake_does_not_pollute_private_robot_history = 1/1 OK`.
- REAL service/실물 MasterPi는 건드리지 않았다.

## 2026-09-01 — REAL `빨간 블럭 잡아봐`: premature pick + face-route servo corridor contradiction

### 관찰된 현상 / 실제 trace
- 사용자 UI에는 `search` 성공 뒤 `PREGRASP_PLAN_MISSING`, 이어서 `approach 실행 시작`까지만 보였지만, canonical REAL trace `outputs/real_traces/20260901T065618Z-real-a822ae0f`에는 backend가 그 뒤까지 실행한 기록이 남아 있다.
- 실제 step 순서는 `search(성공) -> pick(Executive precondition 거절) -> approach(실행 후 실패) -> track(성공) -> ...`였다.
- `pick` 거절은 정상 safety gate였다. search 뒤 target은 visible/centered이고 visual `range_class=PREGRASP`였지만 `task.phase=TARGET_ACQUISITION`이어서, 직전 successful approach가 생성해야 하는 fresh FK/IK handoff `task.phase=PREGRASP_READY`가 없었다. 따라서 `PREGRASP_PLAN_MISSING`은 pick actuator 실패가 아니라 planner가 아직 executable=false인 pick을 너무 일찍 제안한 것이다.
- 실제 approach의 독립 실패는 face-route staging에서 발생했다: corrected radius `26.47 cm`, target bearing `+1.5 deg`, floor edge `+42.2 deg`, arm-face error `49.3 deg`. nearest face normal은 약 `-47.8 deg`이고 6-deg movement target은 `-41.8 deg`가 되어 기존 servo-6 hard corridor `+/-38 deg`를 넘어 planner가 즉시 중단했다.

### 수정
- `harness/protocol.py`: normal autonomous planner에게 `world_state.action_contract_status[tool].executable=false`인 tool은 현재 validity gate상 제출하면 안 된다는 규칙을 명시했다. 특정 recovery tool은 지정하지 않으며 VLM이 executable action을 스스로 고르는 기존 정책을 유지한다. 특히 visual `target.range_class=PREGRASP`와 precision grasp handoff를 구분해, `pick`은 `causal_handoffs.precision_pick_plan.available=true` / `task.phase=PREGRASP_READY`일 때만 valid하다고 명시했다.
- `scripts/red_block/physical_state_machine_reference.py`: ideal face-route arm bearing이 hard corridor를 조금 넘을 때 무조건 abort하지 않고, servo-6 hard edge에서 2 deg margin을 둔 usable corridor(`+/-36 deg`)로 투영한다. 투영된 bearing에서도 arm-face residual이 `FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG=14 deg` 이내일 때만 진행하고, 그보다 크면 기존처럼 fail-closed한다.
- exact REAL geometry 회귀에서 `-41.8 deg` ideal target은 `-36.0 deg`로 투영되며 residual face error `11.8 deg`, planned translation `16.14 cm`가 나온다. 반대로 corridor 투영 후 residual이 24 deg인 synthetic case는 계속 RuntimeError로 거절된다.

### 검증 / 경계
- exact focused regressions 5/5 OK.
- `tests.test_real_failure_gates`: 71/71 OK.
- `tests.test_state_executive`: 27/27 OK.
- `tests.test_harness.ProtocolTests`: 9/9 OK.
- broader `tests.test_harness`에는 이번 변경과 무관한 기존/병행 수정 회귀 `test_action_synchronous_sensor_evidence_skips_redundant_post_action_image`가 prepared completer replies를 소진해 `IndexError: pop from empty list` 한 건이 남아 있다. 새 prompt/route focused suites는 통과했다.
- 이번 진단/수정/검증에서는 REAL MasterPi motor/servo/search/track/approach/pick 명령을 새로 보내지 않았고 REAL service/package도 재배포·재시작하지 않았다. 따라서 수정은 현재 Oracle source에만 있으며, 실물 end-to-end 성공은 아직 확인되지 않았다.

## 2026-09-01 — per-robot authoritative LLM public-skill queue

### 요구 / 문제
- 기존 autonomous loop의 `Plan`은 `pending_plan`이라는 turn-local 문자열 목록에 불과했고, plan을 받은 뒤에도 VLM이 다음 tool을 매번 다시 제출해야 했다. 이 때문에 하나의 grasp 목표에서 `search` 뒤 아직 `PREGRASP_READY`가 아닌데 VLM이 `pick`을 새로 제출하는 등 plan 순서와 실제 제출 순서가 분리될 수 있었다.
- 사용자 요구는 LLM이 선택한 public skill도 명시적 queue에 들어가고, LLM이 현재 queue를 매 판단에서 볼 수 있으며, 각 queued skill은 실행 직전 최신 WorldState로 Executive precondition을 다시 검사하는 구조다. Low-level motor/servo pulse는 queue에 올리지 않는다.

### 구현
- 새 `harness/action_queue.py`에 robot-local `RobotActionQueue`를 추가했다. 각 항목은 `QUEUED/RUNNING/COMPLETED/BLOCKED/FAILED/CANCELLED`, tool args, source, failure_code/required_state/reason/outcome을 보존하며 revision, pause reason, recent history를 제공한다.
- 각 `ChatState`가 자기 queue를 하나 소유한다. 따라서 SIM R1/R2/R3와 REAL R1/R2/R3 slot은 서로 queue를 공유하지 않는다.
- VLM의 `Plan`은 더 이상 "다음 tool을 다시 호출하라"는 힌트가 아니다. plan의 public tool names를 validated calls로 변환해 not-yet-started queue를 교체하고, executor가 head부터 순서대로 claim한다.
- VLM의 단일 `ToolCall`도 실제 dispatch 전에 같은 queue에 먼저 들어간다. paused queue에서 explicit tool call을 하면 recovery로 head에 prepend되고, 새 Plan은 아직 시작하지 않은 pending work를 교체한다.
- 모든 queue head는 **claim 후 dispatch 직전** `TaskExecutive.check()`를 다시 통과해야 한다. BLOCKED이면 해당 item을 history에 남기고 queue를 pause하며 뒤 pending item은 실행하지 않는다. 실제 skill이 FAILED/NOT_ACHIEVED여도 동일하게 뒤 queue를 pause한다.
- 성공한 queued skill 뒤에는 기존 post-action observation/StateEstimator 갱신을 거친 다음 다음 head의 precondition을 검사한다. 따라서 `search -> track -> approach -> pick`에서 pick은 plan 작성 당시가 아니라 successful approach가 `PREGRASP_READY`를 만든 이후에만 통과할 수 있다.
- `_planner_decision_context()`의 `world_state.action_queue`에 `running`, `pending`, `paused`, `pause_reason`, `history`, `revision`과 각 running/pending item의 현재 `executable_now`/`blocking_fact`를 노출한다. system prompt도 queue를 authoritative public-skill order로 설명한다.
- `/api/status`와 turn result payload에도 queue snapshot을 노출한다.
- 이전 turn에서 pending work가 남아도 새 사용자 turn 진입만으로 자동 재개하지 않는다. `new_turn_review`로 pause해 VLM이 current queue + current WorldState를 본 뒤 명시적으로 Plan을 교체하거나 recovery tool을 제출해야 한다.
- `max_steps`에 도달한 상태에서는 queue head를 claim하지 않도록 해 RUNNING orphan을 방지한다.

### 검증 / 활성화
- 새 회귀는 (1) Plan 한 번으로 `search -> track -> approach -> pick`이 정확한 순서로 dispatch되고 strict precondition이 각 head에서 재평가됨, (2) `PREGRASP_PLAN_MISSING` pick head는 BLOCKED되고 뒤 approach는 pending에 남음, (3) 실패한 approach 뒤 queue가 pause되어 LLM이 pending/history를 보고 Plan을 교체 가능, (4) 이전 turn pending queue가 `new_turn_review` 상태로 다음 LLM 입력에 보임, (5) `/api/status`가 robot-local queue를 노출함을 검증한다.
- queue-focused Protocol+Sequence regression: 27/27 OK.
- broader harness/state/REAL-failure/multi-robot/SIM-UI/bridge regression: **243/243 OK**.
- `py_compile` 및 queue 변경 파일 `git diff --check` 통과.
- SIM bridge가 `inflight=None`, `queue_depth=0`, R1/R2/R3 turn이 idle인 경계에서 `ugrp-sim-coworker.service`를 reload했다. 이후 SIM R1(8082), R2(8084), R3(8085) 모두 독립 empty `action_queue`를 live `/api/status`에서 확인했다.
- REAL R1 turn이 idle인 상태에서 `ugrp-real.service`도 harness code만 reload했다. startup은 camera connectivity probe/snapshot만 수행하며 motor/servo skill을 실행하지 않는다. 이후 REAL R1(8083)은 `execute_allowed=true`, `turn_busy=false`, camera age 약 72 ms, 독립 empty `action_queue`를 노출했다. 이번 queue 구현/활성화 중 REAL search/track/approach/pick 및 motor/servo 명령은 보내지 않았다.

### 경계
- 이번 구현은 **LLM public-skill queue**다. 같은 로봇에 동시에 들어오는 별도 operator HTTP `/api/turn` 요청은 아직 기존 per-robot `_turn_lock`을 사용하며 busy 중인 두 번째 요청은 `409 robot_busy`다. operator request/task queue는 별도 계층으로 아직 구현하지 않았다.
- queue는 현재 `ChatState` 수명 동안 유지되는 in-memory authoritative state다. 서비스 재시작을 넘어 durable하게 복구하는 disk-backed queue/journal은 아직 구현하지 않았다. 재시작 시 empty queue로 시작한다.

## 2026-09-02 — 클라우드 GPU 퇴역, 코드 감사 수정, 테스트 불일치 정리, sim-to-real 병목

### 출발점
- 사용자 요청: Azure A10 등 클라우드 연결 해제와 문서 갱신, 감사에서 나온 코드 문제 전부 수정, sim-to-real 병목·SIM 문제 파악, 큰 문제 해결, 선택이 필요한 것은 보고.
- Oracle 작업 트리에 08-31~09-01 작업(queue, TEAM chat, face-route, precision fallback)이 미커밋 상태였고, 로그가 "243/243 OK"라고 적은 뒤 `harness/loop.py`가 다시 바뀌어 `tests.test_harness` 5건이 실패 중이었다. 먼저 현재 트리를 `53f63dc` WIP 스냅샷으로 커밋해 복구 지점을 만들었다.
- Mac↔Oracle 편집 경로 확인: Syncthing 폴더 `changmin-projects`는 `sendreceive`이고 `.git`·`.venv`·`__pycache__`는 `.stignore`. Mac 편집이 수 초 내 Oracle에 도달함을 sha256으로 확인했다(테스트 직전에는 해시 재확인 필요 — 한 번 sync race로 오탐이 있었다).

### 클라우드 GPU 퇴역 (D-20260902-1)
- `az account list`: "Azure for Students"만 존재. `UGRP GPU PAYG` 구독과 `ugrp-a10-sim` VM은 없음. 따라서 deallocate 대상 없음.
- `.env.gpu`: `UGRP_GPU_PROVIDERS='mac'`, `UGRP_ALLOW_AZURE_WORKER=0`, `UGRP_ALLOW_COLAB_WORKER=0`. `gpu_worker_recover.py --preflight` → `{"order":["mac"],"configured":["mac"]}`.
- `ugrp-azure-idle-controller.timer` disable+stop. 3일 22시간 떠 있던 `.venv-lightning/bin/python -`(stdin 스크립트, systemd 잔존) 종료.
- 비밀 파일 5개(`.env.gpu`, `.sim_bridge_token`, `.sim_worker_env`, `.groq_keys`, `.webui_secret_key`) Oracle·Mac 양쪽 0600.
- `docs/cloud_simulation.md` 전면 개정, `ROADMAP.md` §0 스냅샷·C단계 상태 갱신.

### 감사에서 나온 코드 문제 → 수정
**`sim/bridge.py`** (라이브 프로덕션 브리지)
- `ROBOT_IDS` NameError: `/robot/{id}/stream` 요청 시 500. 모듈 상수로 정의.
- `wait_result` 타임아웃 시 inflight가 남아 이후 모든 명령이 막힘 → 타임아웃 시 inflight/queue에서 제거하고 `abandoned`에 기록, 늦게 도착한 결과는 상태만 반영하고 응답은 폐기.
- 같은 프로세스 재접속: `register_remote`가 권한을 항상 내리고 failover는 `inflight is None`에서만 재부여 → 실행 중 명령이 있으면 데드락. 권한을 잃은 instance_id를 기억해 **같은 프로세스**가 다시 오면 즉시 권한 복원. 다른 프로세스는 기존처럼 park.
- inflight를 remote/local 실행자 양쪽에 재전달할 수 있던 경로 → `inflight_owner`로 소유자 고정.
- 권한 회수 직후 도착한 result를 버리던 경로 → 현재 generation이면 수락.
- `results` 무제한 누적 → 64개 상한. 명령 큐 무제한 → `UGRP_SIM_MAX_QUEUE_DEPTH`(16) 초과 시 HTTP 429 `SIM_QUEUE_FULL`.
- trace `seed`가 경로에 그대로 들어감 → 라벨 sanitize + trace root 이탈 검사.
- `Content-Length` 무제한 → worker 16 MiB / control 64 KiB, 음수·비정상 400/413. MJPEG 스트림 동시 연결 `UGRP_SIM_MAX_STREAMS`(24).
- `/command`, `/remote/authority`, `/sim/speed` POST 무인증 → `X-UGRP-Sim-Token` 필수. 호출자(`scripts/sim_actions.py`, `harness/web.py`, `scripts/sim_worker_failover.py`)는 새 `sim/bridge_client.py`(`UGRP_SIM_TOKEN` env → `.sim_bridge_token`)로 헤더 부착.
- `sim/remote_proxy.py` 기본 포트 8093이 브리지 WS 포트와 충돌 → 8094.
- `scripts/run_mujoco_ws_worker.py`: 소켓 끊김 뒤 재전달된 명령을 다시 실행하던 경로 → 최근 8개 완료 결과를 보관하고 같은 id는 재실행 없이 결과만 재전송.
- `scripts/sim_worker_failover.py`: 하드코딩 cwd → `ROOT`.

**REAL 안전**
- `scripts/red_block/remote_watchdog.py`: SIGKILL 경로에서 스킬의 `finally: stop_all`이 건너뛰어질 수 있었음 → 종료 후 같은 패키지의 `masterpi_control.py stop`을 독립 실행(SIGTERM/SIGKILL 모두).
- `scripts/masterpi_control.py`: `write_motor` |speed|>40 거부, `_run_motion` 구간 duration (0, 2.0] 강제, `motor` CLI ±40(기존 ±100).
- `scripts/red_block/robot.py`: `Robot.drive()` speed 31..40·duration 0.05..2.0 검사.
- `dashboard/server.py`: 비-loopback 바인딩은 `MASTERPI_DASHBOARD_TOKEN` 없으면 기동 거부, 토큰 설정 시 POST 액추에이터 API에 헤더 필수.
- `harness/cli.py` `--host` 기본 `0.0.0.0` → `127.0.0.1`(서비스 스크립트는 이미 127.0.0.1 명시). `harness/web.py` Basic Auth 비교를 `hmac.compare_digest`로, 요청 본문 12 MiB 상한(turn 이미지 data-URL 포함).

**harness**
- `harness/loop.py`: 모델이 JSON 봉투를 3회 연속 못 내면 `stopped="protocol_error"`로 turn 종료(기존은 planner 예산까지 재프롬프트). `harness/static/index.html` 중단 라벨에 `goal_achieved`(표시 안 함)·`tool_budget`·`planner_budget`·`protocol_error`·`robot_offline`·`carry_lost` 추가.
- `scripts/robot_actions.py`: `precision_handoff.py`가 내는 "precision pick plan is missing/stale/…" 실패가 `SKILL_FAILED`로 분류되던 회귀 → `PREGRASP_PLAN_MISSING` + recovery `approach`(이 로그 08-26 항목의 명세 복원).
- `scripts/sim_actions.py`: approach `verification_source` 라벨을 REAL(`camera_metric_arm_reach_controller`)과 일치시킴.
- `scripts/gpu_worker_bundle.py`: 09-01에 추가된 `approach_precision_fallback.py`, `pick_precision_fallback.py`, `pickup_strategy.py` 누락 → 번들에 포함.

**정리**
- `.orig/.rej` 15개 삭제. Syncthing 충돌 사본 2개(08-13 ROADMAP, 08-22 decision_log 구판)는 삭제 대신 `docs/archive/`로 이동(추적 파일이던 conflict 사본은 rename).
- `requirements-sim.txt` 신설: Oracle `.venv-sim`·Mac `.venv-sim-worker-mac`에 실제 설치된 버전(mujoco 3.12.0, numpy 2.5.2, opencv-headless 5.0.0.93, pillow 12.3.0, websockets 15.0.1) 고정.
- `harness/protocol.py`의 도달 불가 구프롬프트 블록(23줄) 제거. `harness/catalog.py` `PLAN_SKIP`에 비활성화된 횡이동 토큰(`strafe_*`, `횡이동` 등)이 추가돼 옛 plan이 미해결 pseudo-tool을 queue에 남기지 않는다(같은 날 병행 편집, 전체 회귀에 포함).

### 테스트 불일치 정리
- 09-01 "243/243" 이후 `harness/loop.py`에 문서화되지 않은 세 변경이 있었다: (1) 센서 기준 goal ACHIEVED 시 planner 재호출 없이 `goal_achieved` 종료, (2) 도구 예산 소진 뒤 tool 제안은 `tool_budget` 종료, (3) 텍스트 프롬프트에서 이미지 경로 문자열 제거(이미지는 multimodal 인자로 그대로 전달). 실패 5건(+`test_llm_responsiveness` 1건)은 모두 이 의도된 변경에 맞지 않는 **옛 테스트 기대치**였고, 프로덕션 회귀는 아니었다. 안전 게이트(Executive 재검사, max_steps, REAL chat_only)는 그대로. 테스트만 갱신.
- 추가로 3개 모듈 밖에서 발견된 기존 실패 4건: `test_real_place_actions` 2건(위 `PREGRASP_PLAN_MISSING` 회귀 + 라벨 불일치 → 코드 수정), `test_colab_recovery` 1건(번들 누락 → 코드 수정), `test_real_radar_ui` 1건(`apiPath` 리팩터 이후 옛 문자열 → 테스트 갱신).
- 새 회귀: bridge 회복 6건(`SimBridgeRecoveryTests`), watchdog 3건(`tests/test_remote_watchdog.py`), control 2건, dashboard 2건, loop protocol_error 2건.
- 전체 `tests/test_*.py` 65개 모듈 명시 실행: **798 tests, 0 failures** (수정 전 동일 명령은 failures=2 + 앞선 6건).
- 주의: `python -m unittest discover -s tests`는 `tests/__init__.py`가 없어 동작하지 않는다. 모듈을 명시하거나 `ls tests/test_*.py`로 목록을 만들어 실행한다.

### sim-to-real 병목 (읽기 전용 분석 결과 요약; 상세는 세션 기록)
순위 | 병목 | 심각도 | 성격
--- | --- | --- | ---
1 | 물리 캘리브레이션 데이터 0건 — `sim/masterpi_dynamics_calibration.json` 전부 null, `calibration/masterpi/*.jsonl`은 계획만 | blocker | 측정 캠페인(O6)
2 | 섀시 힘/감쇠/슬립·서보 deadband/rate·그리퍼 μ/kp·블록 질량이 명목/추정 → 같은 speed·duration이 같은 변위가 아님 | blocker | O6에 종속
3 | grasp reach 17.5 cm 임시값·hand-eye depth. SIM 접촉 근거는 카메라 프로필 변경 후 무효 판정. REAL miss를 SIM이 예측하지 못함 | blocker | 1회 TCP↔블록 계측 + 상수 freeze(사용자 결정)
4 | REAL pose = commanded PWM, 섀시 오도메트리 없음. SIM spatial memory는 MuJoCo base pose 사용(`masterpi_production_v2.py` ~261–286) → REAL에 대응물 없음 | major | 엔지니어링(SIM에 REAL 계약과 같은 "odometry unavailable" 모드) + O8
5 | face alignment 샘플 0·corridor 기하 | major | 엔지니어링(멀티 가설/타임아웃 fallback)
6 | 인지 도메인 갭: SIM에 조명/노이즈/압축 모델 없음, REAL은 왜곡 원본에서 검출, `harness/real_geometry.py` 카메라 상수가 SIM 실측 마운트와 불일치 | major | 엔지니어링 + 검출기 선택(고전 HSV/LAB vs 학습형)
7 | REAL 로봇 1/3 | blocker(다중로봇 주장) | O7
8 | L1–L3 값·조건 비교·실험 로거 미구현, O1 미승인 | blocker(연구 산출물) | O1/O2 결정 후 Phase D

- 이번 세션에서 해결한 "큰 문제"는 인프라 계층이다: 브리지 데드락·이중 실행·권한 유실·무인증 제어, REAL 정지 보장. 물리 병목 1–3은 코드로 해결되지 않고 **실측이 필요**하다.

### 서비스 반영 / 경계
- SIM idle 경계(`inflight=None`, `queue_depth=0`, R1/R2/R3 `turn_busy=false`)에서 `ugrp-sim-bridge`, `ugrp-sim-failover`, `ugrp-sim-coworker`를 재기동하고 Mac 워커를 새 코드로 재시작했다(04:25Z). 확인: Mac 워커 `bc0d1b177941b045` authoritative, `/robot/r1/stream` 200, `/robot/r9/stream` 404, 토큰 없는 `/command`·`/remote/authority` 401, 토큰 있는 `/sim/speed` 200, 64 KiB 초과 본문 413, `sim_actions._post_bridge_action("observe_scene")` → Mac 워커 경유 `ok=true`. R1/R2/R3 coworker는 빈 queue로 재기동.
- `ugrp-real.service`도 재기동했다(REAL turn idle, motor/servo/skill 명령 없음). **주의**: 재기동 시점에 ugrp1이 꺼져 있어(ping/SSH 불가) `serve_real.sh`의 시작 시 카메라 probe가 실패했고 REAL R1은 `execute: dry-run`(`execute_allowed=false`, `--no-camera`)으로 올라왔다. 09-01 세션에서 `execute_allowed=true`였던 것은 그때 로봇이 켜진 상태에서 시작했기 때문이다. 실행 권한은 **서비스 시작 시 1회** 결정되므로, ugrp1을 켠 뒤 `systemctl --user restart ugrp-real.service`가 필요하다. 이 설계(오프라인 로봇을 명령 대상으로 취급하지 않음)는 의도된 안전장치라 바꾸지 않았다.
- 미커밋 상태였던 Azure/Colab systemd unit 파일(`~/.config/systemd/user/ugrp-azure-idle-controller.*`, `ugrp-sim-colab-recover.service`)은 disabled로 남겨 두었다(삭제하지 않음).

### 2026-09-03 — Mac 로컬 SIM 스택 기동 + Coworker UI 라이트 테마 (platform/demo)

- **한 일**: Mac에서 로컬 SIM 스택을 새로 기동했다. `.venv-sim` 브리지(`:8091`, 토큰 인증), 기존 로컬 WS 워커(`run_mujoco_ws_worker.py --seed 11`, `MUJOCO_GL` 미설정 시 egl 실패 → `cgl` 필요)를 `/remote/authority`로 authoritative 전환(r1/r2/r3 프레임 정상), `serve_sim_coworker.sh` 기본값(gemini-3.7-flash, 프록시 `:8391` 살아 있음)으로 SIM R1/R2/R3(`:8082/8084/8085`) 기동. REAL(`:8083/8086/8087`, launchd `com.ugrp.real`)·대시보드(`:8765`, launchd `com.changmin.masterpi-dashboard`)는 기존 실행을 확인만 했다.
- **브라우저 정리**: 사용자 지시로 SIM/REAL 외 종료 — 구 Coworker(`:8080`)·Open WebUI(`:3000`) 프로세스 종료, 대시보드는 `launchctl unload com.changmin.masterpi-dashboard` (plist 유지, `load`로 복귀 가능).
- **UI**: `harness/static/index.html`의 `<style>`만 라이트 테마로 교체 (SIM/REAL 공용). HTML 구조·ID·클래스·JS 무변경. 상태 pill(ok/warn/bad/thinking)은 파스텔 배경+진한 글자로 가독화, 보내기 계열 버튼은 기존 검정 버튼 의도 유지 위해 dark slate(`#1f2937`)로 통일, 카메라·레이더 영상면은 dark 유지. CSS brace 222/222, inline JS `node --check` 통과, 6개 포트에서 신규 토큰 문자열 서빙 확인. 서버 재기동 불필요(요청마다 파일 읽기).
- **확인 안 함**: 실제 브라우저 렌더 육안 확인은 사용자 새로고침에 위임. 대시보드(`dashboard/frontend`)는 손대지 않음.
- **다음 행동**: 사용자 피드백 반영 후 필요시 대시보드도 동일 토큰으로 정돈.

### 2026-09-03 — systemctl 503 수정 + SIM 워커 행걸 + 기능 테스트 (platform/demo)

- **증상**: SIM 탭 "SIM 켜기" → `SIM 시작 실패 · [Errno 2] No such file or directory: 'systemctl'`. 원인: `harness/web.py`의 `/api/sim/wake`·`/api/sim/activity`가 오프라인 시 Oracle systemd(`systemctl --user start ugrp-sim-gpu-recover.service`)를 무조건 호출. Mac 로컬 스택에는 systemctl이 없어 `FileNotFoundError` → 503. 동일 명령 직접 재현 확인.
- **수정**: `request_gpu_recovery()` 헬퍼 추가 (`harness/web.py`). `shutil.which("systemctl")`이 없으면 `(False, 사유)` 반환, wake 응답에 `recovery_detail` 필드 추가( additive, JS 무수정). Oracle 동작 동일. 회귀 테스트 2건 추가 (`test_harness.WebTests`: wake 우아한 오프라인 200, activity 200 유지). `WebTests`+`test_sim_ui_consistency` 51개 통과.
- **SIM 워커 행걸 (미해결, 관찰만)**: 구 WS 워커(53192)가 `r2 stack_on` 48s 완료 직후 로그 없이 소멸. 신 워커(47813)는 WS 연결·authoritative 유지 상태에서 CPU 2.8s/7min·스레드 전부 idle·`worker_age_s` 300+ 증가, 즉 sync 이후 완전 교착. 브리지에는 parallel inflight + queue 2가 고착. 사용자 승인 후 워커 kill(`kill -9`, 스레드 전원 idle 확인) + 브리지 재기동(큐 정리) + 워커 재기동 + authority 재부여로 복구. 동일 패턴 2회目이므로 워치독/자동복구検討 필요 — 다음 작업 후보.
- **기능 테스트 결과 (Mac 로컬, 모두 통과)**: SIM r1 — `/api/status`, `/api/tools`(19→SIM 카탈로그), `/api/sim/state`(authoritative, mac), speed 1↔3 + invalid 400, seed reset=7 + invalid 400×2, dry-run turn(실시간 vision "빨간 큐브" 확인), execute turn(빨간 블록 탐색·접근 후 final), cancel, observer CCTV 200, wake 200 online:true. SIM r2/r3 status 정상(execute:true). TEAM — overview(3봇 online, seed 7), goal set/reset. REAL r1/r2/r3 — status/tools만 확인 (`execute_allowed=false`, `talk_only=true`라 하드웨어 무동작 보장). REAL turn·drive·arm은 실행하지 않음.

### 2026-09-04 — P1 3-agent 병렬 검증 Phase 1 (읽기전용, 별도 세션)

- **목적**: Orca 세션이 P0 조작 분리(`approach/pick/physical_state_machine_reference`, `robot_actions`, `harness/loop·protocol·state·executive`, `sim/real_stack_adapter`, 관련 tests)를 편집 중이라 공유 MuJoCo 월드 간섭을 피하기 위해 live turn 없이 읽기전용 확인만 수행. REAL은 건드리지 않음.
- **별도 LLM 인스턴스 (프로세스 분리 확인)**: SIM coworker 3개가 별도 OS 프로세스(PID 48665/48666/48667, ELAPSED 동일 55:41, supervisor `serve_sim_coworker.sh`의 `start_child` 3개)로 기동 중. 각자 `--robot-id r1/r2/r3`, 포트 `:8082/8084/8085`, 카메라 네임스페이스 `/robot/r1|r2|r3/snapshot`로 분리. 동일 backend/model(`gemini-3.7-flash`)이지만 프로세스별 독립 ChatState이므로 completer 세션 공유 없음. `UGRP_MULTI_LLM_AGENTS=1`, `GROQ_ROBOT_MODEL_SHARDING=1`.
- **ChatState/ActionQueue/history 격리 (확인)**: `/api/status` 3대 모두 `turn_busy=false`. R1은 queue `revision=4`, history에 `approach/COMPLETED/ACHIEVED` 1건, `observation_count=4`, `task.phase=PREGRASP_READY`(23:32 approach trace와 일치). R2/R3는 `revision=0`, history空, `observation_count=0`. 세션 간 queue/history 섞임 없음.
- **TEAM bus 프로세스 공유 구조 (확인)**: `harness/team_bus.py`는 파일 기반(`/tmp/ugrp-team-bus-sim.json`) + `fcntl`排他락 + atomic replace라 3개 프로세스에서 실제로 공유됨. 현재 bus는 비어 있음(`goal=null`, chat/messages/wakeups 0, events 1, 마지막 기록 23:32). 즉 공유 경로는 구조적으로成立, 행위적 E2E(TEAM 명령→3 agent→3 queue→3 행동)는 미검증.
- **브리지/워커 생존**: `/health` 정상, `worker_age_s≈0.26`, `remote_authoritative=true`(mac, `8bba76454bad8d91`). R1/R2/R3 스냅샷 모두 `image/jpeg` 반환. coworker status의 `camera_frame_age_ms` 수백만 ms는 관측 캐시(마지막 turn 시점)라 stall이 아님 — R2/R3는 turn을 한 번도 안 돌려 `observation_count=0`.
- **미해결 질문**: 브리지 `/robot/r1|r2|r3/state`의 seed가 `7/8/9`로 다르고 coworker `sim_seed=7`과 불일치. 공유 월드 reset이 세 로봇 seed를 통일하는지, 의도된 per-robot 분기인지 미확인 — Orca P0 작업과 무관해 보이니 Phase 2에서 reset 후 재확인.
- **Phase 2 보류 사유**: live 동시 turn timeline·R1 지연 격리·TEAM E2E는 (1) 실행 중 coworker가 Orca 편집 이전 코드이고 (2) 공유 월드에서 동시 turn이 서로의 관측을 오염시키므로, Orca 편집 종료+idle 가드+재시작 후에 수행. Orca 작업 파일이 00:00–00:07에 계속 바뀌고 있어 지금은 대기.

### 2026-09-04 — P1 3-agent 병렬 검증 Phase 2 (live, 단독 세션에서 사용자 지시로 강행)

- **전제**: 사용자 명시 지시로 live turn 강행. 공유월드 오염 방지를 위해 전부 `execute:false` dry-run(로봇 무동작, planner만 실행). REAL 미접촉. 한계: 실행 중 coworker는 Orca 편집 이전 코드이므로 행위 검증은 구코드 기준이며, Orca 재시작 후 재확인 필요.
- **동시 turn timeline (확인)**: 00:21 R2+R3에 서로 다른 작업(빨간/파란 블록 보고+접근계획) 거의 동시 POST → 둘 다 `turn_busy=True` 겹침 확인 → 각 5–6초에 HTTP 200 독립 완료. R2 응답이 "동료 로봇 r1이 이미 빨간 블록에 접근하여 정렬 완료"를 언급해 peer 상황 반영 확인. 양쪽 `tools:[]`, `pending_plan:[]`, 월드 무동작.
- **R1 지연 격리 (확인)**: R1 turn 기동 후 `turn_busy=True`인 상태에서 R2 turn POST → 409 없이 HTTP 200 수락, R1 17.1s·R2 15.3s 겹쳐 실행 후 독립 완료. `_turn_lock`이 프로세스별 객체라 구조적으로도 격리. R2 응답에 "다른 로봇이 빨간색 물체 근처에서 작업 중"이 보여 공유월드 peer 가시성도 확인.
- **TEAM 크로스프로세스 (확인)**: `/api/team/goal`을 R1 포트(8082)로 설정 → bus 파일에서 확인 → `/api/team/reset`을 R3 포트(8085, 다른 프로세스)로 수행 → goal cleared 확인. bus는 검증 전과 동일하게 비워 둠.
- **Queue/history 격리 유지**: 검증 후 R2/R3는 `observation_count` 0→2(관측만), queue `revision=0`·history空 그대로. R1은 obs 4→5, 기존 history(`approach/COMPLETED`) 보존.
- **남은 것**: TEAM 명령→3 agent 판단→3 queue→3 SIM 로봇 **실제 동작** E2E는 motion이 필요해 Orca P0 완료·재시작 후에 수행. 브리지 per-robot seed 7/8/9 vs `sim_seed=7` 불일치도 그때 reset 후 재확인.

### 2026-09-04 — SIM 명령 병목 측정 (별도 세션, 사용자 지시)

- **방법**: (1) 워커 로그 `action_s` 채굴, (2) 코드상 직렬화 지점 확인, (3) live 병렬 부하 실측. 모션 명령은 발사하지 않음(Orca P0 테스트와 공유월드 충돌 방지) — 모션 경로는 로그 채굴로만 분석. REAL 미접촉.
- **B1 — 모션 실행시간이 압도적 1위 (워커 `action_s`)**: `reset` 0.03–0.6s, `pick` 1.3s, `parallel composite` 0.04–0.08s, `track` 8.2s, `search` 3–30s, `approach` 4–50s, `stack_on` 48s, `team_tower` 127–211s. 특히 approach 4s↔50s, search 3s↔30s 편차가 큼 → 스킬 내부 phase 루프(서보 스텝+카메라 왕복)가 원인 추정. phase별 계측이 없어 분해 불가 — Orca P0 분리 작업에 phase 타이밍 로그 추가를 건의.
- **B2 — 브리지 single-inflight 직렬화 (코드 확인)**: `sim/bridge.py`는 inflight 1슬롯+FIFO 큐(상한 16, 초과 429). 단일로봇 모션은 전부 직렬 실행. `parallel{}` 복합 명령만 예외. 다로봇 동시 모션은 여기서 줄 섬.
- **B3 — 브리지 HTTP 간헐적 전체 멈춤 (재현됨)**: 단독 스냅샷 4–85ms·state 1ms인데, 21병렬 → 전부 약 3.9s, 6병렬 1회차 → 전부 약 1.7s, 이후 회차는 0.03–0.12s로 정상. all-finish-together 패턴이라 큐잉이 아니라 브리지 프로세스 전역 멈춤(GIL — 워커 WS sync 대량 파싱 의심). 스냅샷/state 경로(`do_GET`, 락 없음, 캐시 서빙)에는 문제가 없어서 sync 수신 경로와 분리/경량화가 필요. 정밀 재현·수정은 별도 작업으로 분리.
- **B4 — LLM planner (참고치)**: dry-run turn 5–17s(R2/R3 동시 5–6s, R1/R2 겹침 15–17s). 3병렬에서도 contention 징후 없음(제미니 프록시 정상). 모션 turn은 스텝당 planner 호출이라 더 김.
- **B5 — TEAM bus EXCLUSIVE 락 (현재는 무죄)**: 읽기(`snapshot`·`context_for`·`peer_summary`)까지 `LOCK_EX`라 구조적으론 convoy 가능. 실측 60병렬 스냅샷 med 16ms·max 55ms로 현 규모에선 무시 가능. 나중에 turn당 bus 접근이 늘면 `LOCK_SH`로 고칠 것.
- **비어 있음 확인**: 브리지 큐 압박 흔적 로그에 없음(`queue_depth=0` 유지), 스냅샷 단독 ms 수준, `/health` 정상.

### 2026-09-04 — 모션 자체 분석 방법 + 데모 (별도 세션, 읽기전용)

- **방법**: `outputs/sim_traces/<id>/trace.json` 프레임 시계열(`t_monotonic` + `base_xyz`/`base_yaw` + `arm_qpos` + `commanded_pwm`/`motor_commands` + privileged 물체좌표)로 4개 지표 추출 — 직선거리 대비 실이동(우회율), 정지 시간 비율, 저속 구간, 명령-실행乖离. `analysis.json`은 before/after만이라 부족하고 contact sheet(`scripts/analyze_sim_trace.py`)는 육안용.
- **데모** (`20260903T143245Z...approach`, worker_action_s=23.7s, 36프레임): 직선 0.528m·실이동 0.650m로 우회율 1.23배(이동 자체는 효율적). 반면 wall 21.1s 중 정지(<2mm/스텝) 12.4s로 **59%가 정지**. 즉 병목은 '움직임'이 아니라 '멈춰 있는 구간'(vision 대기·멀티프레임 확인·서보 정착·phase 전이·재시도).
- **부족한 것**: 프레임별 controller-phase 라벨이 없어 어느 정지가 face-route고 어느 게 capture 재시도인지 구분 불가. `commanded_pwm`/`motor_commands`는 매 프레임 이미 있으니 phase 태그만 추가되면 명령-실행 비교까지 가능. Orca P0 작업에 phase enter/exit 로그(+trace 프레임 phase 필드) 추가를 건의.

### 2026-09-04 — approach 정지 구간 영상 대조 분석 (별도 세션, 읽기전용)

- **대상**: `20260903T143245Z...approach`(23.7s, 성공). 프레임 시계열을 BASE 이동/ARM 이동/idle로 분류 + 로봇·observer 영상 대조.
- **타임라인**: F1–9 주행 → F10–12 정지 2.3s → F13–14 주행 → **F15–23 정지 5.5s** → F24 미세이동 → **F25–30 정지 3.5s** → F31–33 주행 → F35 최종 정렬 성공. 팔은 전 구간 무동작(`darm` 최대 0.003).
- **정체의 정체**: F9 로봇시야에 빨간 블록이 잡혀 있었으나 F15 시야에는 노랑·파랑만 남고 빨강 이탈. F23까지도 빨강 없음. 즉 두 장시간 정지(합 9s)는 **회전 중 타겟이 좁은 카메라 FOV를 벗어나 생긴 수동 vision 대기**다. 오탐지는 없었고(노랑·파랑에釣られ지 않음) 최종 재획득·정렬은 성공 — 실패가 아니라 비효율.
- **핵심 패턴**: 타겟 상실 → 능동 재획득(sweep/기억 방위로 선회) 없이 제자리 대기. `spatial_memory`에 빨강의 마지막 방위·거리(`bearing 13.55°` 등)가 있는데도 활용하지 않음.
- **수정 방향**: (1) FOV 이탈 시 last-known-bearing 즉시 선회, (2) vision-wait에 타임아웃+능동 sweep, (3) 회전 중 타겟 유지 기동(제자리 회전 대신 arc). (1)은 기존 memory 필드만으로 가능해 P0 분리 없이도 들어감.

### 2026-09-04 — 사람 같은注視 구조 설계: pursue 모드 (별도 세션, 사용자 지시)

- **의도**: approach 중 타겟 상실→수동 대기(9s급 정지) 패턴을 사람 시각운동처럼 바꿈. 토글은 "주시 모드": `fixate`(현행 빤히 고정, 기본값) vs `pursue`(쫓기). trace에 모드 기록해 on/off 비교.
- **눈→머리→몸 위계**: 서보 6+3=눈, 차체 yaw=몸. 눈이 먼저 쫓고(±25° 상한), 끝에 걸리면 기존 `align_body_to_gaze`로 몸에 이양. face 단계 여유(±34°) 침범 금지.
- **전정안반사 피드포워드**: 차체 펄스 명령을 미리 아니까 펄스 전 게이즈를 반대방향으로 선보상, 펄스 후 프레임 1장으로 `tracking_step` 1회 다듬기. 지금의 "펄스→멈춤→3연측" 대신 "선보상→펄스→1회 트림", 정밀 측정은 N펄스마다 1회.
- **단속성 재획득**: 1~2프레임 유예(깜빡임 무시) → `spatial_memory` 마지막 방위로 홱(saccade) → 스윕 → 실패 시 멈추고 search 인계(fail-closed 유지).
- **하지 않는 것**: 진짜 동시제어(차체+게이즈 병렬 루프) — 스레드·서보 경합과 검증 비용 때문에 제외. per-pulse 순차로 동등 효과.
- **구현 위치**: `approach_with_locked_gaze` 루프 내부 1곳(`physical_state_machine_reference.py`) + `run_coarse_approach` 파라미터 전달(`approach.py`). 둘 다 Orca P0 작업 중이라 **구현 보류, 설계만 확정**. Orca 완료 후 SIM on/off 동일seed replay로 검증 예정. PID는 정지 기준이라 pursue용 게인 하향 또는 속도 피드포워드가 필요하며 SIM에서 먼저 확인.

### 2026-09-04 — pursue 프로토타입 검증 (/tmp, repo 무접촉)

- **이유**: 구현 위치 2곳이 Orca 편집 중이라 저장소 대신 `/tmp/ugrp-gaze/`에 순수 로직만 프로토타입. `track.py` PID(`tracking_step`)와 `geometry.PULSE_PER_DEGREE`를 그대로 재사용.
- **내용** (`gaze_hold.py`): `pursue_step` 상태머신(TRACKING/SATURATED/LOST_BLINK/LOST_SACCADE/LOST_SEARCH), VOR 피드포워드(`feedforward_pan`, 왕복 항등), pursue 상한 ±25°(face 여유 ±34° 보존, 초과 시 body 이양 신호), 깜빡임 유예 2프레임, 기억 방위 홱이동(`saccade_pan`, 클리핑 보고).
- **검증**: `test_gaze_hold.py` 8/8 통과(수렴·포화·피드포워드 역함수·유예·홱이동량 13.55°→약 150펄스·클립·재획득 복구).
- **통합 지시서(Orca용)**: `approach_with_locked_gaze` 스텝 루프에서 펄스 후 fresh 프레임에 `pursue_step` 1회 호출로 `centre_gaze` 전체 호출을 대체, `SATURATED`면 기존 `align_body_to_gaze` 경로, `gaze_mode` 파라미터(`fixate` 기본)로 토글, trace에 모드 기록. SIM 동일seed on/off replay는 Orca 완료 후 별도 세션에서 수행.

### 2026-09-04 — pursue 통합 구현 (별도 세션, 사용자 지시로 강행)

- **경위**: Orca가 00:51 이후 잠잠해(CPU 0.2–0.4%, 파일 무변경) 사용자 지시로 통합 강행. 충돌 방지를 위해 (1) 순수 로직은 신규 파일 `scripts/red_block/gaze_hold.py`로 분리, (2) 기존 파일은 최소 훅만. 작업 전후 md5 고정, Orca 덮어쓰기 없음 확인.
- **훅** (`physical_state_machine_reference.py`): `motion_pulse_with_gaze`에 `gaze_mode` 추가. pursue면 펄스 후 `centre_gaze(confirmations=3)` 대신 fresh 1프레임+`tracking_update` 1회+`clamp_hold(±25°)` 후 반환(클램프 초과분은 서보 재넛지로 정정). blob 없으면 기존 recovery 그대로. `approach_with_locked_gaze`에 `gaze_mode` 추가해 내부 2개 호출점에 전달(기본 `fixate`라 기존 호출자 무영향).
- **훅** (`approach.py`): `run_coarse_approach`/`run_approach`/CLI에 `--gaze-mode fixate|pursue`(기본 fixate) 추가, staging 호출에 전달, deploy extra에도 전달.
- **검증**: 신규 `tests/test_gaze_hold.py` 9/9, 기존 `test_red_block_approach.py` 27/27 통과. 기본값 fixate라 기존 동작 불변.
- **남은 것**: SIM on/off 동일seed replay는 실행 중 coworker(구코드) 재시작이 필요해 Orca 조율 후 수행. saccade 본체(LOST_SACCADE 사용)는 phase 2로 남김 — 현재 손실 시 기존 recovery 유지.

### 2026-09-04 — SIM 워커 재시작 + gaze A/B (별도 세션, 사용자 지시)

- **조율**: 재시작 전 Orca 무활동 확인(마지막 기록 00:51, CPU 0.2–0.4%, turn 전부 idle, bridge inflight None/queue 0). 본 항목이 공유 공지 역할을 겸함.
- **범위**: 재시작 대상은 SIM 워커(PID 60824)만. 브리지·coworker·REAL은 손대지 않음. 근거: SIM `approach`는 `masterpi_production_v2.act → _real_action → MigratedRealStack.approach → approach_mod.run_approach`로 워커 프로세스 안에서 REAL 소스를 실행하므로, 워커만 새 코드(`gaze_hold` 훅)를 물고 있으면 됨.
- **A/B 통로**: `UGRP_GAZE_MODE` env (fixate 기본). 어댑터가 `gaze_mode` 없이 호출해도 env가 적용되도록 `run_coarse_approach`/`run_approach` 기본값을 None→env→fixate로 변경. Orca 소유 tool 스키마 무수정.

### 2026-09-04 — gaze A/B 중단, 선행 버그 발견 (별도 세션)

- **경위**: SIM `approach`는 워커 프로세스 안에서 REAL 소스를 실행함을 확인(`masterpi_production_v2.act → _real_action → MigratedRealStack.approach → approach_mod.run_approach`). 워커만 재시작하면 됨(브리지·coworker 무접촉). 그 사이 누군가(Orca 추정) 01:06에 워커를 이미 재시작했고 seed11 파이프라인을 live 테스트 중이었음. 종료 대기 후 A/B 강행.
- **A/B 불발**: pursue 2회 모두 펄스 전 초기 추정(`estimate_block`→None)에서 실패해 pursue 코드까지 도달하지 못함. 실패 지점은 gaze_mode 분기 이전이라 fixate도 동일하게 실패하는 코드경로상 사실.
- **진짜 발견 (선행 버그)**: 동일 seed11에서 search→track이 pan을 하드웨어 끝(1950)까지 몰고 타겟은 중심 밖(nx=0.422)에 둔 채終わり, approach 초기 추정이 죽음. Orca 성공 케이스는 track 종료 pan=1590(중앙 근처)이라 살았음. 즉 동일 seedでも track 수렴점이 달라져 파이프라인이 갈림 — PID의 wall-clock 의존(dt) 탓으로 보이며, seed-replay 비교가능성에 주의 필요. 추가로 `reset`이 MuJoCo 월드는 되돌려도 서보 자세는 남겨서 조건 매칭이 깨짐.
- **복구**: pursue 워커 내리고 fixate 기본 워커로 재기동 + authority 부여 확인(instance `1ad02adf`, idle). 공유 환경 원상복구(추가 코드만 dormant). pursue SIM 검증은 위 선행 버그(estimate 강건성/track railing) 해소 후에 재시도. saccade 본체는 여전히 phase 2.

### 2026-09-04 — 사용자 live run 해부: fine-x 세금 + close-pose pan 스냅 (별도 세션)

- **대상**: ep0008 seed12 approach (r2, 37.8s, SUSPICIOUS). 거리 0.191→0.192m로 차체는 거의 안 움직이고 끝남.
- **느림 원인**: pre-capture 0.10s 펄스마다 5-iteration fine-x 재센터가 따라붙음(서보넛지+프레임+정착×5, 3연속 안정 확인). 12펄스×약 2.5s ≈ 30s가 여기서 나감. 약 10:1 오버헤드. 현재 pursue 훅은 `motion_pulse_with_gaze`만 커버하고 `visual_precapture_approach`는 미커버 → phase 2에서 동일 single-trim 적용 필요.
- **실패 원인**: 12펄스 내내 nx=0.49 완벽 추적(pan 1757→1908) 후 close-pose 진입 시 pan을 1908→1500으로 스냅 (`enter_fixed_capture_pose`, `BODY_REALIGN_LIMIT=70` 초과라 리셋). 36° 점프에 블록 상실 → 6프레임 확인 실패 → SUSPICIOUS. 최종 프레임에 블록이 정중앙에 크게 찍혀 있는데도 판정이 실패인 게dib 포인트 — 추적 성공을 진입 스냅이 날림.
- **수정 방향**: (a) close 진입 시 inherited pan 유지하고 거기서 확인(어차피 바로 뒤 fine-align이 따라옴), (b) 또는 close 진입 전 far 구간에서 pan→차체 이양을 먼저 끝내기. (a)가 작음. Orca P0 영역이라 구현은 인계.

### 2026-09-04 — fine-x 세금 + pan 스냅 직접 수정 (별도 세션, 사용자 지시로 강행)

- **조율**: Orca 무활동 확인(approach.py 01:24 편집이 마지막, 6분+ 경과, inflight None) 후 작업. 전후 md5 고정, Orca 덮어쓰기 없음.
- **수정 1 — close-pose pan 스냅 제거** (`establish_capture_pose`): `BODY_REALIGN_LIMIT=70` 조건 삭제. 물리 서보 범위 안이면 inherited pan 그대로 진입하고 뒤 확인에 맡김. **기본 동작이 바뀌는 변경**이라 명시 — 근거는 ep0008 r2 실측(1908→1500 스냅이 추적 성공을 날림)과 기존 주석 자체의 1536 사례. Orca 재검증 필요.
- **수정 2 — pre-capture fine-x 세금 절감** (pursue 한정): `fine_align_horizontal`에 `confirmations` 파라미터(기본 3 유지) → `_sample_capture_with_optional_x_align` → `visual_precapture_approach` → `approach.py`로 `gaze_mode` 전달. pursue면 drift 시 1회 확인만. fixate 기본 동작 불변.
- **검증**: 컴파일 OK, `test_gaze_hold` 9/9, `test_red_block_approach` 28/28(Orca 추가분 포함) 통과.
- **미적용**: 실행 중 워커(01:3x 기동)는 수정 전 코드. SIM 검증은 워커 재시작 후. saccade 본체는 여전히 phase 2.

### 2026-09-04 — 2번 해결: retreat 예산 분리 + 오버슛 엡실론 (별도 세션, SIM 실측)

- **원인 확정**: creep 예산과 retreat가 하나의 루프 카운터를 공유. 막판 creep 오버슛 시 retreat 1회만 뛰고 루프 탈출해 검증 없이 사망했음(fixate 38.2s False 재현 확인).
- **수정**: (1) creep/retreat 예산 분리 — retreat는 자체 캡(5회)까지 검증·복귀, (2) `CAPTURE_WINDOW_OVERSHOOT_EPS_NY=0.015` — 최소 유효 펄스(~0.02ny)가 윈도우 폭(0.05) 수준이라 bang-bang으로는 안착 불가. 수 mm 오버슛은 pick이 재측정하므로 ready로 인정. creep粒度 자체(검증된 60ms)는 손대지 않음.
- **실측**: fixate 동일조건(0.82m 출발) CLEAN 성공, 75.8s. 참고로 pursue 동일조건은 40.6s CLEAN — staging 재센터 55→31프레임 효과가 그대로 드러남. seek 22.8s 등 search 편차는 별개.
- **상태**: 실행 중 워커가 최신 코드(fixate 기본)라 별도 원복 불요. saccade·fine-x 피드포워드·track railing은 잔여.

### 2026-09-04 — 1번 해결: feedforward + median-trim + deadbeat (별도 세션, SIM 실측)

- **1차 실패 분석**: key 없는 단일 memory가 0.28s 펄스 보정량을 0.10s creep에 재생 → 눈 진동 → ny 투영 흔들림 → 진행 게이트가 후진으로 오판. 교훈: memory은 모션 식별자별 분리.
- **2차 실패 분석**: single-frame verify가 3-median 대비 ±0.02 지터 → trim이 노이즈만 쫓고 정중앙 행진(nx 0.487→0.545) 무대응. 교훈: trim은 루프가 이미 낸 median으로, 추가 프레임 없이.
- **3차 실패 분석**: `tracking_step` 내부 0.05 deadband가 0.025–0.05 드리프트 보정을 통째로 0으로 만듦(pan 13펄스 고정). 교훈: trim은 보정 전용 직접 비례식으로. `step=(target-nx)*HFOV*PPD`, 부호 검증됨.
- **실측**: pursue+deadbeat CLEAN 성공 (0.82→0.14m). fine-x 38→5, pan 전 구간 추적(1469→1339). 단 벽시계 53.8s로 pursue v1(40.6s)보다 긺 — deadbeat 풀게인이 모델 오차에서 진동(nx 0.433 좌측 오버슛)하기 때문. 다음 한 줄: 게인 0.5–0.7 댐핑.
- **복원**: fixate 기본 워커 재기동+authority 확인(idle). 잔여: 게인 댐핑, 3번 track railing, saccade 본체.

### 2026-09-04 — 게인 0.6 실측: 환경 오염으로 무효 (별도 세션)

- **결과**: CLEAN 성공은 했으나 wall 122s. 같은 조건 deadbeat 53.8s와 비교 불가 — 판정 불가.
- **원인**: 실측 중 두 번째 MuJoCo 워커(PID 99691, `--url ws://127.0.0.1:8293`, 약 02:34 기동, 출처 불명)가 같은 머신에서 물리 렌더링 중이었고, 전체 로드 67까지 치솜. 손대지 않은 search까지 125s(평소 9–26s)가 걸린 게 환경 증거. 99691은 누구 건지 몰라 끄지 않음 — 사용자 확인 필요.
- **유효한 관측**: 제어 프로파일은 deadbeat와 동일(fine-x 5, trim 8, CLEAN). 게인이 망가뜨린 건 없음. 속도 효과는 머신 진정 후 재측정 필요.
- **복원**: fixate 기본 워커 재기동+authority 확인(idle). 잔여: 게인 재측정, 3번 track railing, saccade 본체.

### 2026-09-04 — 3번 해결 확인: Orca bearing 흡수가 이미 커버 (별도 세션, SIM 실측)

- **실측**: seed11 r1 파이프라인. track 종료 pan=1950(레일) 그대로 재현 → `metric range unavailable at inherited bearing +45.3deg; perform one bounded body re-face` 발동 → 그래도 추정 불가 → `guarded far-view staging`으로 진입 → CLEAN 성공 (1.10→0.15m, 106.9s).
- **결론**: 3번은 Orca의 bearing 흡수+far-view 폴백이 이미 해결. 내 작업 불요. 느리지만(106s) 죽던 게 산다.
- **잔여**: saccade 본체(phase 2)만 남음. 다만 손실 시나리오는 bearing 흡수·far-view·lock 용서로 이미 커버돼서 우선순위 낮음.

### 2026-09-04 — metric range 원거리 불가 원인 분석 + 해결안 (별도 세션)

- **메커니즘**: `estimate_block`은 바닥-광선 삼각측량. `forward = h/tan(pitch)`라 오차 증폭이 `1/sin²(pitch)`. 카메라 높이 약 12cm에서 1.1m.target은 pitch 약 −6° → 픽셀 1° 오차가 거리 18cm로 증폭. 30cm에선 같은 1°가 약 1cm. 즉 멀수록 물리적으로 잴 수 없는 구조 + 하드캡(70cm, ray −5°)이 이중으로 막음. seed11(1.1m)은 캡부터 걸림.
- **핵심**: 캡은 자의적이지만 노이즈 증폭은 물리다. 그래서 '캡 풀기'만으론 안 되고 불확실성을 다뤄야 함.
- **해결안 순위**: (D) 크기 기반 거리측정 — 블록 3cm 기지 규격 + 핀홀 모델은 grazing과 무관하게 동작. floor-ray 근거리와 융합. 가장 쌈. (E) 이득표 추측항법 — blind 구간만 기지 bearing + 15cm/s로 dead-reckoning. 단거리라 드리프트 무해. (A) 불확실성 전파 — 픽셀 노이즈→거리 σ, 이진 reject 대신 σ 가중 제어. 정석이나 공수 중. (C)能動 주시 — far용 카메라 자세 별도. 실험용.
- **주의**: (B) 폴백 펄스 키우기는 충돌 리스크라 비추. 지금 폴백이 느린 게 정상이지 버그가 아님.

### 2026-09-04 — D 구현: 크기 거리측정 live, 동작하나 과소 (별도 세션)
- **구현**: `size_range_estimate` (핀홀 각도, 프레임 독립) + `estimate_block` 폴백 + `source` provenance 필드. 단위 21+... `test_gaze_hold` SizeRangeTests 포함 전체 통과.
- **실측** (seed11 r1): 폴백 7회 발동, `unavailable` 0회, CLEAN 성공 (1.10→0.15m). 커버리지는 달성.
- **문제**: 71.6cm로 추정했는데 실제 109cm — 35% 과소. 원인 추정 1순위는 어안렌즈 (`camera_raw_fisheye=True`인데 핀홀로 계산). 중앙부 확대가 블록을 크게 보이게 함.
- **이번이 느린 이유** (129s vs 106s): 과소 추정 탓에 staging 플랜이 어긋나 fine-x 36회. 폴백 자체는 staging을 52→32펄스로 줄였음.
- **다음**: (i) 실측-진리 쌍 모아 스케일 핏 (임시), (ii) 어안 undistort 정식 (보정 영역). 둘 다 calibration 성격이라 여기서 멈추고 인계.

### 2026-09-04 — 어안 과소 수정: cube-model fit (별도 세션)

- **원인 확정**: 핀홀이 아니라 원근법. 정방향 투영 대조(n=332, 실측 trace+진리)로 검출기·블록크기·어안모델은 정상(med 오차 1%) 확인. 남은 −25%는 기울어진 시야에서 윗면·옆면이 같이 보여 bbox가 커지는 효과 — `min(d_w,d_h)`이 과소 쪽만 고름. 어안 undistort만으론 −27%→−23%라 부족.
- **수정**: `geometry.cube_range_estimate_cm` — blob 중심 bearing(보정 어안) 위에 30mm cube를 놓고 forward 투영 extent가 검출 bbox와 맞을 때까지 golden-section. `_size_block_estimate`가 우선 사용, 기존 undistort sizing은 degraded 폴백. 프레임 하단 잘림은 reject(근거리 +49% 과대 원인이었음, `SIZE_BOTTOM_TRUNC_NY=0.98`).
- **검증**: 기록 356프레임 bias +8%/MAE 14%/med +1%(기존 far −27%). 실제 코드경로 sweep far n=18: bias +8%/MAE 9%, unavailable 0. `test_size_fallback_far`를 실측 fixture로 교체(진리 95.4→94.2cm). 51 tests 통과.
- **한계**: 45° 회전 블록은 최대 −29% 과소 가능(모서리 부풀림). live A/B는 후속.

### 2026-09-04 — R1 오작동 신고 해소 (별도 세션)

- **증상**: 브라우저 R1이 죽어보임. 실제: 프로세스·UI·LLM·vision 정상. 원인은 (1) turn 없으면 카메가 fossil(2시간짜리 옛날 프레임 표시), (2) 월드 reset이 에피소드 경계로 planner 상태/history를 설계대로 파기 (`state.py:97`, `web.py:594`) — 오늘 밤 수십 번 reset으로 R1이 계속 기억상실. 둘 다 고장이 아니라 동작 방식.
- **검증**: dry-run turn 2.5s 정상 + 직접 파이프라인 search 2.2s / track 1.5s / approach 24.3s CLEAN (pregrasp). R1 정상.
- **운영 규칙**: 월드 reset 후에는 해당 로봇 turn 1번으로 재관측해야 화면·상태가 살아남. fossil 화면 = turn 돌리라는 신호.

### 2026-09-04 — 속도: predictive 재활성화 실패 후 원복 (별도 세션)

- **내역**: 64s 중 motion 5s, sample 9s, centre 8s, fine-x 36회(대부분 stable 확인, 4s), face dog-leg 4회 ~12s, 나머지는 settle/servo 대기. 스텝당 vision 오버헤드 ~1s 구조.
- **시도**: predictive cap 0.28→0.60 (신뢰 estimator 기반). live에서 0.60s 펄스가 87.28→66.38로 읽혀 gate refuse. motion sweep(정지/구름 분리) 결과 SIM 휠 게인이 야생 (0.28s→3.6~21cm, 0.60s→72~79cm, V2_STRUCTURAL_UNCALIBRATED) — duration sizing은 estimator와 무관하게 불가. 즉시 원복, 최종 검증 CLEAN 67.2s.
- **결론**: 60s대는 안전 규율(stop-measure-pulse)의 구조적 하한. 절반 이하는 연속 서보잉 프로젝트급. fine-x 36은 무죄(centering 확인용).

### 2026-09-04 — A/B: cube-fit live CLEAN + gate 3종 세트 (별도 세션)

- **결과** (seed11 R1, load 5-9): CLEAN 1.095→0.149m, approach 63.7s, fine-x 36회(동일), refusal 0. 첫 원거리 108.2cm(구 71.6).
- **도중 3연속 게이트 사건** (동일 시나리오 결정적 재현으로 각 원인 확정):
  1. viewpoint: 펄스마다 centre_gaze가 pan 16° 돌림 → gate 병진모델과 충돌. `dpan` 추적+0.5%/deg 허용.
  2. transient triplet: 68.96cm 3연속 일치 후 자가해소(진리 90.9, 최종프레임 87.7). refuse 전 1회 재측정(`_confirm_range_jump`), 지속될 때만 refuse.
  3. fusion cliff: far floor가 70캡 언저리에서 깜빡이며 69→68.96 (floor_ray, tilt 추적 855→806). `FLOOR_FAR_CUT_CM=50` 하드컷 — 50 초과는 size만, floor는 폴백. 321프레임 sweep에서 false-veto 0 확인.
- **tilt 해석항**: floor 호흡 `d=-h/sin²(p)` 유도식 그대로 게이트에 (`ch·dtilt/sin²p`, cap 6). 경험치가 아니라 물리.
- **부수**: step print에 src/blob 탑재(포렌식용), `/robot/rX/state`는 로컬 워커에서 `world_seed` 직접 송출 확인.
- **운영**: 원격 GPU 단절 → 로컬 Mac 워커로 failover (venv, authority 수동). `scripts.run_mujoco_ws_worker` 좀비 1기 정리. 로컬 워커가 repo 코드라 수정 즉시 반영됨(단, worker 재시작 필요 — import 캐시).

### 2026-09-04 — seed 불일치 해소: world_seed provenance (별도 세션)

- **원인**: 버그가 아니라 라벨링. `reset(12)` → 로봇별 `c.seed = 12+idx` (로컬 노이즈 분리용, 정당). 근데 trace가 로봇 seed를 THE seed로 기록 → R2/R3 trace가 13/14로 보임. trace seed로 replay하면 엉뚱한 월드(13/14)가 뜸 — provenance 결함.
- **수정**: (1) worker `state`+trace에 `world_seed` 추가 (다음 worker 재배포 때 live). (2) bridge가 reset 성공 결과의 world state seed를 에피소드별로 기억 → trace dir명·trace.json에 주입. worker값 우선, 에피소드 바뀌면 파기, 없으면 기존 동작.
- **검증**: live 재현(reset 12 → r1/r2/r3 = 12/13/14) 후 unit 4케이스 + live R2 probe: dir `seed12`, trace.json `seed:13`+`world_seed:12`. platform 38 passed.
- **주의**: bridge 재시작 후 remote authority 수동 복구 (`/remote/authority`, token은 `.sim_bridge_token`). 후속: bridge가 `/robot/rX/state` 응답에도 `world_seed` 주입(캐시 불변, 에피소드 가드) — live 확인 r1/r2/r3 = 12/13/14 + world 12. worker 원격 코드의 자체 필드는 다음 재배포 때.

### 2026-09-04 — R1 탭 암전 수정: `/sim/r1` 프록시 누락 (별도 세션)

- **원인**: UI의 SIM R1 탭은 하드코딩 로컬(`""`)이었고, 서버 프록시 라우트에 `/sim/r1`이 없었음(R2/R3만). `:8082` 본인 페이지에선 보여서, `:8083`(REAL) 페이지에서만 R1이 암전. R2/R3는 프록시라 멀쩡 — "R1만 안 떠"와 정확히 일치.
- **수정**: (1) `web.py` routes에 `("/sim/r1", ("sim","r1"))` 추가 — prefix strip 구조라 셀프 프록시 무한루프 없음(검증됨). (2) `index.html` `robotApiPrefix`가 `/api/status` 1회 조회로 본인 슬롯 판별, 본인만 로컬, 나머지는 프록시.
- **검증**: PY_OK + node --check OK. `:8083/sim/r1` → SIM R1 상태·카메라 200, `:8082` 셀프 정상, R2·직접 경로 회귀 정상. `:8083` + SIM coworker 3대 재시작(전부 idle 확인 후).
- **사용자 조치**: 브라우저 새로고침 1번.

### 2026-09-04 — 예측 펄싱 중단: 추정기가 거짓말 (별도 세션, trace truth 대조)

- **실측**: 0.80s 예측 펄스가 plausible gate에 2회 연속 걸림(25–27cm 주장).
- **Truth 대조**: trace 특권 상태로 재보니 펄스당 이동은 0.06m 양자 수준으로 일정. 0.8s도 0.06–0.12m. 추정기가 긴 펄스에서 2배+ 부풀림. 즉 게이트가 정상 동작을 죽인 게 아니라 추정기 오류를 잡은 것 — 게이트 무죄, 추정기 유죄.
- **파급**: `motion_gain_table.json` 자체가 추정기 출력이라 이득값 신뢰 불가. duration 비례 사이징 전제 붕괴.
- **조치**: 예측 캡을 기존값으로 되돌려 고정 0.28s 복원(구조는 유지). 추정기 강건성(긴 펄스 후 프레임 median)이 Orca/precision 영역의 다음 과제. 워커 최신 코드로 재기동+authority(idle).
- **미해결 질문**: reset에 seed=12를 줘도 trace seed가 13/14로 찍힘. 동시 행위자 없음 확인(turn idle, Orca quiet). 워커 내부 seed 유도 방식 미확인.

### 2026-09-04 — 모션 이득표 채굴 (별도 세션, 읽기전용)

- **방법**: 워커 로그 펄스 89개 (`range delta after Xs pulse` + 직전 모터 속도). sim_speed=1, 직진만, multi-seed 혼합.
- **결과** (`outputs/motion_gain_table.json`): 전진 0.28s → −4.21cm(σ0.63), 전진 0.2s → −2.86cm(σ0.04), 후진 0.06s → +0.57cm(σ0.02). 초당 이득 전진 약 −15cm/s로 선형. 후진은 +9.6cm/s로 비대칭이라 미러링 금지.
- **주의**: 0.28s 분산이 0.2s의 15배 — 긴 펄스일수록 미끄러짐/조건 의존. verify-after-pulse 유지 필수. 최소 유효 펄스 0.06s/0.57cm.
- **다음**: 예측 펄스 사이징(거리÷이득으로 첫 펄스 결정, 검증 후 보정)으로 staging 펄스 수 절반 이하 목표. yaw·곡선·속도별 테이블은 미수집.

### 2026-09-04 — 게인 0.6 재측정: 차이 없음 (별도 세션)

- 조용한 머신(로드 7.9)에서 r2·seed12 동일 파이프라인: CLEAN 성공, action 50.2s (fine-x 5, trim 8, pan 1355→1474 부드러움).
- gain 1.0 (40.6s)과 같은 펄스 횟수·같은 프로파일. 10s 차이는 측정 중 로드 상승(7.9→12.8) 수준의 노이즈로 판단 — 게인 효과 아님.
- 결론: 게인 0.6 유지(진동 관측 없음, 안전). 1번 종료. 복원: fixate 워커+authority(idle).

### 2026-09-04 — 운영 정리: Orca 제외 + 고아 제거 + 단일스택 규율 (별도 세션)

- **Orca 제외**: 사용자 결정으로 Orca 세션 미사용. P0 파일 조율 오버헤드 해소, 본 세션이 직접 소유.
- **고아 제거** (사용자 승인): 두 번째 SIM 스택(`:8291` 브리지 + `:8293` 워커, 02:34경 기동) 종료, 수요일부터 CPU 1300시간 먹던 좀비 python3.9(29012, `$HOME` cwd 정체불명 서버) 종료. 포트 회수 확인. Codex는 유지(사용자 선택).
- **효과**: 로드 67 → 6.5(8코어 기준 정상). 내 스택 무사(authoritative, idle, coworker 3대 정상).
- **앞으로 규율**: (1) SIM 스택은 1개만 — 워커 기동 전 기존 프로세스·포트 확인, (2) timed 실측은 로드 8 미만에서만 + 결과에 로드 병기, (3) 09-03/금일 확인된 워커 재접속 후 명령수집 중단 패턴은 워치독 과제로 유지.

### 2026-09-04 — gaze A/B 실측 (별도 세션, 사용자 지시로 직접 수행)

- **조건**: r2·동일 파이프라인(reset→search→track→approach, 직접 `/command`, LLM 없음). fixate→pursue→복원 순으로 워커 재시작하며 측정. fixate 38.2s 실패 vs pursue 41.9s 성공이라 벽시계 단순 비교는 무의미 — 경로 길이·종료 상태가 다름.
- **fixate (신코드, pan-snap 제거 포함)**: 38.2s False. pan-snap blindness는 해소됨(close 진입 유지, capture window 0→8펄스 진행). 새 벽: ny=0.408로 윈도우 상한(0.402) 0.006 초과 → bounded retreat 첫 펄스에서 사망. creep粒度·retreat 강건성이 다음 병목(Orca 영역).
- **pursue**: 41.9s **CLEAN 성공** (0.82→0.15m, pregrasp 확립). staging 8회 full 재센터가 pursue trim 8회로 대체(gaze_frames 55→31). nx 0.74→0.59 수렴 + pan 1608→1528로 눈→차체 이양이 로그에 그대로 찍힘. pre-capture fine-x는 46회로 여전 — 최종 1-confirmation 효과가 미미하거나 경로가 길어서. fine-x 루프 자체가 남은 대어.
- **복원**: fixate 기본 워커로 재기동+authority 확인(instance `f60a63b4`, idle). saccade 본체·capture 윈도우·retreat는 phase 2/Orca 영역.


### 2026-09-05 — 사용자 요청: 창고 연구 타당성 세 문제 수정

- 승인 범위: 사용자가 중앙 역할 배정, 비교군 부재, 정확한 SIM 상태의 actor 사용이라는 진단에 대해 “그거 다 해결해주라” 및 “이어서 해줘”라고 지시했다. 기존 3-MasterPi MuJoCo 범위의 구현/검증이며 공식 제출계획과 Isaac Lab 수정안은 병합하지 않았다.
- 변경: warehouse_runtime/protocol/research로 로봇별 LLM 선택과 referee/transport 분리. 기본 TEAM은 llm_peer_comm, 기존 team_zone_transfer는 central_baseline으로 표시. 각 에이전트가 같은 cargo/roster/destination에 독립 동의해야 실행하며, 역할 선택/수정의 중앙 fallback은 없다.
- 센서: 색상+배경 검출은 노란 로봇 부품을 crate로 오인해 폐기. ArUco 11/12/13 + 로봇별 RGB-D pan scan + 2mm noise + 로봇별 ID 기억으로 교체. 기억에는 옛 range/bearing을 재사용하지 않으며 현재 시야와 구분. Reset/external action, revision/sequence, replay를 검증한다.
- 평가: rule/no-comm/peer-NL 동일 episode 스펙·seed·예산, raw 응답·정확한 actor 입력·usage·실패·source hash 저장. no-comm에는 peer 내용 없음, LLM peer는 자연어만 수신. source 변경 중 실행은 중단하고 결과 혼합 금지.
- 사용자 흐름: snapshot polling이 rendering subscription을 만들지 않아 검은/오래된 카메라를 보이던 문제를 bounded refresh로 수정. 실제 TEAM UI 요청과 관찰카메라를 확인. 실제 장애물은 sensor에서 숨겨지는 overlay group에서 분리. 배포 번들에 새 모듈/마커 asset 포함.
- 검증: 전체968 tests 통과 후 추가37 집중 검증 통과. 격리 번들 BUNDLE_OK 3. 현재 live에서 3/3 화물 성공, 큐/실행중0. 별도 최종 영상에서도 3/3 성공, pose-reset setter를 예외로 막은 상태에서 호출0. 영상 프레임 검토 완료.
- 비교 pilot: 최신 기억 경로 seed11 normal/gripper_failure/goal_revision에서 rule3/3, peer-NL3/3, no-comm0/3. 통계적 우월성 결론은 아님; 이전18개 실행과 각 소스해시를 구분한다. 상세와 파일 링크: docs/warehouse_research_verification_20260905.md.
- 남은 연구 한계: 마커 없는 인식, 실물 센서/동역학 보정, 충분한 held-out 반복, 완전 분산 저수준 조작은 완료로 주장하지 않는다. 공통 저수준 controller의 SIM ideal feedback은 모든 조건에 동일한 실험 affordance다.
- 로컬 실행: bridge8091/worker8093 + coworkers8082/8084/8085. 이번 세션이 만든 PID와 로그는 outputs/warehouse_research/live/. 기존 TEAM 기록과 섞지 않도록 이 경로의 별도 bus를 사용. 비밀값은 기록하지 않았다.


### 2026-09-05 — 사용자 재지적: 실제 병렬 동작과 전방 주행 수정

- 사용자 요구: “병렬로 움직이게”, “진짜 사람 3명처럼”, “앞을 보고 앞으로”. 이전 연구 경로는 LLM만 병렬이고 staging/return/clearance는 순차, scout는 대체로 정지였다. 이전 완료 보고가 이 요구를 충족하지 못했음을 인정한다.
- 구현: sim/warehouse_crew.py의 로봇별 비차단 navigator를 단일 물리 tick에서 함께 갱신. 한 로봇의 이동이 peer를 STOP시키던 경로 제거. 두 carrier의 개별 접근과 scout의 pickup/route 관찰을 겹쳐 실행하고, 공동 파지/운반만 필요한 장벽 유지. release clearance도 두 carrier 함께 실행. 센서 pan scan도 한 servo mapping으로 병렬 실행.
- 주행: 각 waypoint의 이동 방향을 먼저 향하고 positive-forward로 이동. 복귀는 뒤를 향하도록 회전한 뒤 전진. BRAKE 전환으로 회전 전 관성 제거, final yaw는 위치 유지 보정. B→A loaded formation도 chassis heading pi와 arm-relative yaw로 구성. 짧은 접촉/위치 보정은 별도이며 무조건 후진 0이라고 주장하지 않는다.
- 증거: actual delayed-r1 test에서 r2/r3 계속 전진, actual chassis 3-way overlap, 뒤쪽 목표의 turn→forward 검증 통과. A→B→A 실제 pipe 왕복 성공. 별도 3 cargo 물리 실행에서 robot-to-robot penetration >2mm 0건. 1배속 영상은 outputs/warehouse_research/parallel-crew-reviewed/parallel-forward-1x.mp4, 궤적은 motion.jsonl, 결과 result.json.
- 실제 LLM 영상/최신 live 모두 3/3, 5 rounds, 3 cargo actions 성공. 번들만 임시 디렉터리에 복사한 실제 이송 BUNDLE_CREW_OK. queue/inflight 0, worker contract masterpi-v2-three-robot-parallel-crew-v5.
- 측정: 영상 실행에서 translation-only 2-way34.3s/3-way7.9s. 이동 또는 회전을 합친 3-way45.1s(simtime). 가장 긴 연속 nonrotating 후방 이동 약2.99cm, pose setter0. 세 로봇 총 이동 각각9.45/10.07/8.39m. scout의 실제 RGB-D 관찰24회. 이 수치는 세 로봇이 매 순간 전진했다는 뜻이 아니다. 스캔/파지 장벽/장애물 양보는 정지 가능.
- 테스트: 관련63개 중 기존 고정pair순서/scout명칭 가정2개 실패를 실제 역할/측정출처 계약으로 갱신했고 해당2개 재실행통과. 추가 집중47개 통과. 이전968 전체통과 수치를 이번 수정 후 전체회귀라고 재사용하지 않는다.
- 한계: 로봇별 locomotion task는 독립적으로 진행하지만 물리 통합은 하나의 공유clock, 고수준 LLM은 여전히 라운드 기반이며 공동 화물은 한 번에 하나다. 매 motor tick마다 독립 LLM이 재판단하거나 물체3개를 동시에 드는 구조로 주장하지 않는다. 손목 카메라는 파지 중 물체/그리퍼를 보며, 별도 전방 카메라가 생긴 것은 아니다.


### 2026-09-05 — 단독/공동 혼합 화물 및 다음 작업 선택

- 승인: 사용자가 R3의 단순 관찰/대기를 지적하고 “혼자서 옮길 수 있는 것”을 제안한 뒤 “ㄱㄱ”로 구현 지시. 새 AGENTS 지침의 Sol 기본 위임을 반영하려 했으나 기존 하위 실행이 pending_init에 멈추고 Sol 생성도 thread limit으로 실패하여 주 에이전트가 구현/검증했다.
- 구현: opt-in mixed CargoSpec(긴판자2명+30g급작은상자2개1명), ArUco14/15, 1명/2명 부분합의 및 reservations, independent agent loops, shared-clock SoloCargoTask와 공동이송 overlap. 혼합과제에서는 scout 강제할당 제거. 완료로봇의 다음claim은 다른화물 작업 중에도 수락된다. sourceA에 한정, 기존연구scene/physics한계는 별도로 유지한다.
- 관측/실패수정: 작은박스 lane을 공동grasp envelope밖으로 분리. 단독DOCK의 과도한8mmgate를25mm로 조정하고 실제 delivery120mm와 안정성은 유지. 완료화물의 오래된peer report가 새claim을 막던검증경계를 수정. 실패한joint는 참여자만stop하고 해당constraint만release한다.
- 실제LLM 검증: outputs/warehouse_research/mixed-llm-final/result.json 및 episode.jsonl. R1/R2판자, R3작은상자1→2. 첫solo완료23.85s, 다음soloaccepted25.60s, joint완료50.06s, 전체64.90s(sim). 모든3화물SUCCESS. 단독/공동taskoverlap38.09s, 두cargo의동시평면이동0s, peer penetration>2mm0. 과장해 화물들이항상함께이동했다고표현하지않는다.
- 사용자흐름: latest worker contract masterpi-v2-mixed-solo-joint-v6, mixed default +3independentLLM backends. 실제TEAM naturalcommand3/3완료,6wake done. 기록은 outputs/warehouse_research/live 및 team cr000005.
- 검증: 전체1002 tests /238.341s/OK; 이후flatmixedfixture집중7tests/OK; 격리배포bundle BUNDLE_MIXED_OK. 영상은 mixed-llm-final/mixed-solo-joint-1x.mp4, 실제render review-sheet 확인. 현재두작업중하나의실패시다른로봇강제정지하지않는cleanup추가 후 compile및집중회귀확인.
- 범위: predefined tagged cargo의 SIM task이다. arbitrary 물체의무게추론, 실물센서/동역학보정, 임의장애물일반화완료주장금지. physicalcontroller geometry는ideal SIMfeedback이며LLM입력은자기camera/memory/수신메시지뿐이다.


### 2026-09-05 — 작업 중 재판단·센서 불확실성·하중·재현 평가

- 사용자 “각각 검토해보고 구현해봐” 승인. 네 부분을 따로 검토했다. Sol 위임은 thread limit으로 시작하지 못해 직접 구현했다.
- task_recovery.py의 versioned participant-only RecoveryEvent와 SoloCargoTask pause/recover를 추가했다. 공동 작업은 기존 함수의 physics tick에 cooperative pause gate를 넣어 다른 solo task는 계속 갱신한다. joint navigation 및 진행 중 carry segment는 재계획 요청 후 경로를 갱신한다. joint를 전부 별도 public skill로 분해한 것은 아니다.
- busy 상태에서도 recovery event가 있으면 LLM을 호출한다. 동료 consent 직후 관측이 ROBOT_BUSY로 바뀌어 agent_loop가 종료되던 실제 race를 수정했다. stale event와 비참여자 재개는 거부한다. solo cancel/help는 물체를 낮추고 열고 후퇴한 뒤 종료한다.
- RGB-D 거리 표준편차(MAD) 및 current_view/confidence gate 추가. 마커에서 질량·물체 크기를 만들지 않는다. 실제 controller는 여전히 ideal SIM geometry를 사용하며, calibrated sensor-only motion이라고 주장하지 않는다.
- 가상 wrist wrench 센서를 추가했다. cargo body_mass/pose를 읽지 않고 정지한 wrist 외력의200표본에서 하중과 불확실성을 계산한다. 동일 시각 물체의 물리 질량을 바꾼 시험에서30g→29.8g,180g→179.5g. 무거운 물체는 AWAIT_HELP→agent request_help→safe lower/open→ASSISTANCE_REQUESTED. 실제 MasterPi force/odometry 센서 calibration은 없음.
- 기존 작은 상자에 두 gripper를 붙이는 공동운반 시험은 grip_site_y 정렬 실패. 이를 완료로 꾸미지 않았다. 공동 받침/역할 교대/긴 물체만 공동 대상 중 사용자의 선호 질문을 남겼다. 물리적 도움 동작은 그 선택 전까지 안전한 내려놓기·지원대기까지만 구현.
- 실제 LLM 장애물 시험 recovery-obstacle-llm04에서14.10s pause,16.79s r1/r2 replan 합의. 그동안 r3 APPROACH→LOWER 진행.3cargo 성공, peer collision0. 장애물은 controlled_scenario_event로 넣은 실제 geom이며 카메라 자동 감지라고 표현하지 않는다.
- evaluate_mixed_recovery.py가 source hash, seed, 호출 budget, timeout, scenario 발생·복구 여부를 보존한다. recovery-comparison-01의 seed11 obstacle rule/no-comm/peer 각1회 모두 성공. 통신LLM 우월성의 증거는 아님. 더 많은 시드/반복은 별도 본실험 필요.
- 전체1005tests/168.460s/OK, 이후load·safe help 집중26tests/OK, 격리bundle RECOVERY_BUNDLE_OK. UI/worker는 masterpi-v2-event-recovery-load-v7로 갱신. 세부검토: docs/event_recovery_review_20260905.md.


## 2026-09-05 — CoELA식 모듈 분리와 A/B/C 통신 실험 구현

- 사용자 승인: AI끼리 통신이 필요한가를 연구하기 위해 CoELA식 분리와 A/B/C 비교 설계 후 “ㄱㄱ”로 구현 승인. 기존 MuJoCo 3-MasterPi 범위이며 Isaac Lab/REAL 확대는 없음.
- coela_modules.py / coela_runtime.py: 독립 관측·기억·소통·계획·실행. 팀 전역 available/idle 정보, 전역 완료에 따른 기억 삭제, 전역 상태 기반 깨우기를 새 경로에서 제거. 직접 관측/수신 보고는 별도 보존. 공동 제어의 최소 동기화는 공통 유지.
- evaluate_coela.py: no-message / structured / natural, 선택적 송신·지정 수신자, 원문·수신·실행·토큰 기록, 초기 센서 준비와 실행 시간 분리, 고정 source hash와 실패 포함 분모.
- 개발 시행 coela-dev-01은 초기 준비가 실행 예산을 소모했으므로 비교 제외 표시. coela-pilot-02 정상 seed11 A/B/C 각1회 모두3화물 성공, 송신0/11/10. 우월성의 통계적 증거로 주장하지 않음.
- coela-private-video-03: R1만 장애물 원인을 알고 R3는 cause_unknown. R1의 replan 제안, R3의 원인 질문과 재계획 참여가 실제 한국어 메시지로 기록. 공동 pause35.832→recovered42.636 SIM초, 3화물 성공, 대화15개/67.3초 영상. 장애물 source는 controlled_scenario_event이며 카메라 자동감지가 아님.
- 39개 집중 회귀 통과. 정보 비간섭·수신자/정형 schema·느린 에이전트 독립성·멱등 pending·잘못된 원문/usage 보존 포함. 영상39/61초 프레임을 직접 확인하고, 원문15개와 overlay 데이터를 대조함.
- 실제 TEAM /api/team/chat→세 HTTP LLM 서버→공유 worker 흐름도 SUCCESS. team-cr000006-1788610980402339000.jsonl에서 strict_local, 판단12/7/12회, 메시지9개, 오류0. 최신 worker contract masterpi-v2-coela-local-view-v8, queue0/inflight없음. 기본 serve_sim_coworker는 coela, legacy 명시 설정으로 구 경로 유지.
- 남은 본실험: held-out seeds·모델 반복 수 확정, 비공개 사건 A/B/C 전체 반복, 통신 지연/손실, 메시지 차단 분기 검증. 현재 제어기는 ideal SIM geometry를 계속 사용하며 실제 센서 보정·작은 상자의 물리적 지원 인계는 미완료. 다음 세션은 docs/coela_communication_study.md를 기준으로 진행.


## 2026-09-05 — CoELA 다중 시드18회 진단 (핵심 코드 수정 없음)

- 사용자 요청: 여러 시드에서 돌리고 로그로 문제점을 찾는다. seed12/23/37 × 정상/비공개 장애물 × A/B/C,120초/로봇24회로18회 완료. 모든 source hash 고정·일치. outputs/warehouse_research/coela-multiseed-01/report.md가 최종 진단이다.
- 전체 운반 완료: 정상 A/B/C 각3/3; 비공개 장애물 A0/3,B1/3,C2/3. 해당 장애물 사건에 대한 replan 복구는 A0/3,B2/3,C3/3. 작은 표본이고 물리 실패가 섞였으므로 자연어 우월성 증거로 단정하지 않는다.
- seed37/B 비공개: R2 작은상자1+R1 작은상자2가 접근90초 시간초과. 센서 준비를 포함한 고정 동작 재현에서 동일하게 실패했고23.958초 이후 약76초 접촉·교착. 각 로봇 단독 진단은 각각36.028/38.192초에 성공. 지속 접촉·진행 정체에서 양보/재계획이 없는 동시 이동 문제가 최우선. 초기 준비를 생략하면 실패가 재현되지 않았으므로 재현의 준비 상태도 보존해야 한다.
- 첫 판단16/18회 같은 상자 쏠림. RESERVED59건. 판단866회 중wait491회,54개 로봇-실행 중13개가24회 한도 소진. seed37/B에서는 R3가75.928초 이후 판단 종료했지만 남은제안으로102.642초 새공동작업 시작,108.744초 장애물에 대응 불가.
- STALE_RECOVERY_EVENT5, REPLAYED_PROPOSAL5, INVALID_TASK_GOAL12. 늦은모델응답의현재상태검증부족. 37/A 비공개에서121.473초 마감후recover제출1건도관찰. 실제추가복구성공은없음.
- 초기의 “37/B 장애물 미발생” 해석은 잘못이었다. 최초 접근 실패 후 실행 막바지에 사건이 주입됐고 복구되지 않았음. 최종 report와review-notes에 정정. 실패 reason덮어쓰기가 최초원인을가리므로 이후평가기록개선필요.
- 이번에는 진단 요청 범위로 핵심 소스를 고치지 않았다. 다음 수정 순서: 동적접촉·교착제어→늦은응답/마감검사→재호출/예산/미확정제안관리→초기분담/평가정리→같은18회재실행.


## 2026-09-06 — CoELA 진단 결함 수정 완료

- 사용자 “모두 해결해줘”, 연속 “이어서 해줘” 승인으로 진단·물리 재현·최소 수정·검증을 수행.
- 동적 동료 양보/회피 간격 및 legacy beam 포함, 늦은 응답·마감·퇴역 lease 정리,
  일반18/복구6 예산, intent 벽시계TTL, 본인 잠정 claim의 메시지 starvation 방지,
  명시적 관측 보고와 시각·실패 원인 보존을 통합했다.
- 마지막 Solo 정체는 최종 자세의 마찰/집기 정밀도 및 접근 경로에서 target cargo를 제외하던 문제였다.
  waypoint3.5cm/terminal1cm 분리, 제한된 terminal pose 보정, 잡기 전 target cargo 포함으로 수정.
  일반 주행 방향은 face-then-forward, 최종 미세 정렬은 양방향 보정 가능. pose reset/teleport 없음.
- 최종 정확 seed23 장애물 재현 87.152 SIM초 전체 성공, target approach displacement 없음.
  실제 LLM 동일시드120초 한도79.448초 전체·복구 성공, 오류·접촉0. 전체1,052개 테스트 통과.
- 최종 TEAM journal team-cr000008-1788626658348024000.jsonl 전체 성공, 각18call/16message/오류0.
  v9 재시작 후 queue0/inflight없음. 영상 coela-final-video-08/23-private_obstacle-0-natural/dialogue-1x.mp4,
  86.1초/14개 실제한국어발화,32/73초 직접프레임 확인 및 HTTP200.
- fixed-05는18회 완료했으나 마지막정렬/경로보정 전 진단이다. 새최종18회 통계는 없음.
  작은표본/모델변동/일반예산압력/발화와상태불일치/idealSIM 제약은 남는 연구 한계다.
- 다음세션 기준 outputs/warehouse_research/coela-fixes-final-report.md, docs/current_architecture_todo.md.

## 2026-09-06 — 카메라 목적지 운반·독립 3로봇 실행, Gemini 판단 명시

- 사용자 “2까지 진행해줘”: 단독 목적지 운반과 세 로봇 독립 동시 실행 승인. 이어 “llm이 그래도 판단해야지”로 실제 LLM의 행동 판단을 명시했다. 이전 deterministic 시연은 Gemini 실험이 아니며 소급해 이름을 바꾸지 않는다.
- 새 경로: 각 로봇의 원본 손목·차체 RGB 두 장, 로컬 기억, 작업 ID/목적지 이름만 Gemini로 전달. 실제 모델 응답으로 접근/집기/이동/대기/놓기/종료를 선택하고 기존 시각 실행기는 선택된 조작만 수행한다. 규칙 주행 방향 추천이나 정답 위치는 전달하지 않는다.
- 시뮬레이션 차체 RGB 카메라 추가는 손목 시야의 운반 중 가림을 해결하기 위한 센서 구성 변경이다. 물체 크기, 구역0.82m, 사전 설치 장애물 유지. 실제 하드웨어 적용은 미검증.
- 초기 실제 모델 smoke에서 R1/R2/R3 각각 Gemini 접근 결정, 별도 집기 결정, 이후 서로 다른 전진·회전 명령을 확인했다. 16call 예산 시행은 전원 배달 성공이 아니며 원본 실패 기록 보존. 확대 검증 진행 중; 상세 계약 docs/camera_team_llm.md.

## 2026-09-06 — 카메라 팀 판단 모델 Gemini 3.8 Flash 선택

- 사용자 “3.8 flash로 하는 게 어때”에 따라 camera-team 실행기의 기본 모델을 `gemini-3.8-flash`로 변경했다. 기존 공용 proxy 및 과거 3.7 결과는 변경하지 않았다.
- 새 시행 `outputs/warehouse_research/gemini38-team-dev-01`: seed41, 세 로봇, 600 SIM초/각120call, noslip3/impratio10. 실제 응답의 response_model이 세 로봇 모두 gemini-3.8-flash임을 확인했다. 각자 접근·집기 선택, R2의 우회 주행·그립 재확인·장애물 veto 후 제자리 회전 선택을 관찰했다.
- 집중 검사22개 통과. 결과 보고서는 accepted 판단만 적용된 이유로 표시하고 rejected/discarded, 요청 모델/응답 모델을 분리한다.
- 이 기록 시점 실행은 진행 중이다. 운반 완료나 3.7 대비 성능 우월성, 다중 시드 완료를 주장하지 않는다. 다음 확인은 해당 실행의 result.json 및 원본 카메라/판단/물리 로그다.

## 2026-09-06 — Gemini 3.8 카메라 팀 3시드 최종 결과와 별도 재파지 검증

- 정상 코호트: seed41/58/73, 각 3로봇, 600 SIM초·로봇당 180call. 세 실행 모두 source hash `8cb3389821bdb89f7c5730a52a5875f9054692ed60ed817c24fcf8cd9ecad48e`로 동일하다.
- 실제 결과: 총 6/9 배송 성공, 팀 전체 성공 1/3. 시드별로 41은 1/3, 58은 3/3, 73은 2/3이다. 실패 3건은 visual-grip 2건과 세부 subtype을 확정하지 못한 proxy-connection 1건이다. 실패를 분모에서 제외하지 않는다.
- 각 로봇은 자기 손목·차체 RGB, 자기 기억, 자기 스킬 상태만 받은 독립 Gemini planner였다. 로봇 간 통신은 아직 없으므로 통신 효과나 협업 우월성의 증거로 해석하지 않는다.
- 검증: camera/LLM/recovery 집중 51 tests, 앞서 수행한 platform 48 tests 통과. 두 수치는 서로 다른 검사 묶음이며 전체 회귀 수치로 합산하지 않는다.
- 정상 코호트가 이미 코드를 import한 뒤 near-field recovery 수정이 들어갔다. 따라서 이 수정은 위 고정 source cohort에 포함되지 않았고 정상 9건 중 recovery 사용도 0건이다.
- 별도 newer-source `recovery-probe-03`: dev-01 R2 raw command를 현재 physics로 released 시점까지 재생하되 qpos/teleport를 쓰지 않았다. fresh own RGB placement=outside 뒤 실제 Gemini 3.8 두 호출이 approach→pick을 선택했고, 기존 strict left/right/home attachment 검사를 통과해 carrying에 도달했다. 사후 referee 상승량은 6.877cm였다.
- probe-03은 물리 재파지까지만 확인했다. 목적지로 다시 이동·놓기·검증한 full re-delivery가 아니며, 정상 코호트 6/9에 성공으로 더하지 않는다. 원 실행과 물리 source가 다른 독립 진단임을 result와 영상에 명시했다.


### 2026-09-06 Camera repair continuation — validation before matched cohort

- User authorized fixing the diagnosed Gemini camera-team failures. Preserve RGB-only decision boundaries, actual model-selected actions, asynchronous three-robot physics and unchanged contact settings.
- Added typed transient proxy failures with fresh-observation retries, model-authored TTL-limited communication, own-image progress history and actual-message video/report transcripts.
- Development seed-41 status run finished 2/3; R2 attachment rejection while physically lifted motivated complete left/right sweep plus strict home verification. Detached physical control still rejects. Do not combine development revision with the upcoming frozen revision.
- Full redelivery diagnostic 01 failed (no release); diagnostic 02 succeeded through actual approach/pick/drive/release/finish, 16 calls. This is a synchronous single-robot recovery diagnostic, not team/cohort success.
- Focused suite currently 102 passed; final corner geometry and saved probe-triplet audit pending. Next: freeze final source and run seeds 41/89/97 across none/status/natural, preserving every outcome and video.


### 2026-09-06 Camera repair — final handoff / external inference block

- Implemented camera grip full-sweep + strict home checks (39 saved operational probes pass; explicitly detached physical control rejects), exact orthogonal two-visible-edge known-square reconstruction with <=3deg residual and retained 5cm margin, own-image progress feedback, TTL-limited actual status/natural communication and visible transcripts.
- Full R2 redelivery diagnostic 02 confirmed approach/pick/12 drives/release/finish with 16 actual Gemini responses. Other robots stationary, synchronous inference; not counted as team success. Earlier redelivery01 failure remains preserved.
- Development repaired01 outcomes: seed41 none3/3, status2/3, natural2/3; natural eventually emitted16 messages. These processes used earlier source, not the final repair hash.
- Final repaired02 matched9 runs used source825e9ba1a9da4c77d04cded3aa522befd287c175b34f4fa7b9352bc8a35fbceb. Shared sustained429 stopped all27 robots. Seed41 had347 accepted decisions before outage; seeds89/97 had0. Preserve0/9 teams and0/27 deliveries, separately classify infrastructure failure; no communication efficacy conclusion. All9 videos validated.
- Last single actual provider availability recheck still429, noRetry-After/reset time. Local proxy still listening; period quota vs rolling rate limit cannot be distinguished. No further quota-consuming trial launched.
- Post-cohort added safe Retry-After parsing and slower429backoff, plus new cohort runner that halts remaining trials on inference failure even with process exit0. Added tests including no skipping ordinary physical failures. Current focused suite108passed before final report-only fix.
- Remaining: once Gemini is available, use NEW repaired03 directory and evaluate_gemini_cohort --seeds41 89 97 --jobs1 (each world retains3 independent async robots). Do not claim final multi-seed success yet. Inspect current source/manifests on resume;02 differs from current post-outage recovery/report code.

- Final report-only correction: manifest communication is retained for unstarted cells; infrastructure-blocked not-started trials render separately and do not masquerade as executed failures. Cohort runner also detects unresolved retry cooldown at simulation budget end and stops subsequent trials. Added classifier/report regressions passed7tests; runtime retry tests passed8. These changes postdate saved02 source. Final deliverables: repaired02/index.html, offline-verification.json, cohort-interpretation.json, provider-recheck.json, dialogue-qa.jpg; separate successful recovery-redelivery-02/redelivery-1x.mp4.


### 2026-09-06 OAuth/quota diagnosis correction

- User challenged whether OAuth was in use because displayed usage seemed unchanged. Inspected actual listener proxy: OAuth Bearer from Antigravity credentials, 3.8 alias routes to daily backend gemini-3.8-flash-medium. No API key route in this inference path.
- Direct authenticated read-only model catalog returned HTTP200. One minimal real completion returned429 with explicit upstream message Individual quota reached; reset in41m45s. Catalog reset2026-09-06T06:20:11Z =15:20:11KST today. OAuth is working; prior rate-cap-vs-quota uncertainty is now resolved to individual quota per upstream text.
- Proxy quota display explicitly appends daily3.7-flash-tiered only, not3.8. This can omit current3.8 quota; actual user usage dashboard was not inspected. Catalog remainingFraction absent, not inferred as numeric0. No proxy edit/restart or account switch. Safe evidence saved repaired02/oauth-quota-diagnosis.json.


### 2026-09-06 Bounded camera execution and compact inputs

- User approved correcting excessive LLM calls. Preserve own wrist/nav originals, LLM-authored velocity/duration/skills, private actor memory and message boundary. No automatic route selection.
- Guarded drive now allows LLM duration up to4seconds, with fresh own-nav veto every<=0.25seconds. New messages or two-second RGB stagnation interrupt; own wrist attachment monitor remains active. Cancel removes scheduled commands, including stop during execution; off-grid physical timestamps trim current slice instead of dropping every slice or replaying expired motion.
- Memory retains latest real placement_evidence status/reason/stage/identity and recent3actions/criticalfeedback, drops redundant geometry/hashes. Previous nav image only for relevant change comparison. Actual compact model_context and transmitted messages are audited; text context bounded6000characters. Defaultcalls40/robot and reportedinput120000/robot (one-call overshoot possible); candidate pilot stricter30/60000.
- Offline saved422requestmemories reduced characters4742487→225694 (95.24%); latest placement status/stage preserved across404eligible requests. This is memory text reduction, not measured total token or quota saving.
- Actual simulator no-API execution probe: same4seconds commanded rotation and16RGBguards, four1second decisions vs one4second decision; no interruption. Explicit OFFLINE_TEST_DOUBLE, no delivery-success claim. Evidence event-execution-check-01/verification.json.
- 92focusedtests passed after integration. Time now past stated provider reset; launched only solo41 actualGemini pilot, max30calls/60000inputtokens/300simseconds/max2transientfailures, video recorded. No team/cohort expansion before outcome.


### 2026-09-06 Event-driven cost repair — verified result

- Integrated guarded bounded drive and semantic inputs,92focusedtests passed. Every raw0.25s slice has freshownnav check; expiry neverreplaysmissedslices; cancel clears pendingmotion. Model-context audit includes actually transmitted compactinbox and full boundedtext, separatefromrawreceivedinbox.
- Real Gemini3.8 solo41 pilot finished at179.038simseconds with15calls,61,898reportedinputtokens (average4,126.5),33actualimageinputs,maxcontext1812chars. Inputcap60,000 exceeded onlybylastresponse1,898tokens, thennomorerequests. 30currentrequestimagehashes verified. Full video8fps1433frames/179.125s and visiblecamera inspectionverified.
- Outcome LLM_INPUT_TOKEN_BUDGET: lifted6.94cm and carried, did NOTreachdestination orrelease. Do NOTclaimend-to-end deliverysuccess. Model chose1.5s drives despite4s ceiling; actualcaller-count reduction tocompletion remains unproven. No team expansion or additionalpaidtrial launched.
- Outputs: coela-event-pilot-01/index.html,solo-41/motion-1x.mp4,verification.json,motion-qa.jpg; event-execution-check-01 holds explicitnoAPItestdouble executioncomparison andsavedmemorycomparison.
- Defaults now40calls/robot and120k reportedinputtokens/robot; cohorttrialconcurrency1. This is a request/token brake, not exactsubscriptionquota accounting. Need futurework on visual navigation efficiency withinbudget beforeteamcommunication trials. Preservepilotfailure, notincreasebudgetsilently.


### 2026-09-07 Mac runtime availability verification

- User requested live verification of existing Mac simulation availability. Apple M3/arm64 and existing .venv-sim-worker-mac validated; MuJoCo3.12.0,cv2,websockets,numpy imports succeeded through Mac connector. No source/runtime configuration changes.
- Independent existing MultiMasterPiProductionV2 mixed world seed11:1500physics steps,3simseconds,R1 wheel-command motion0.58565m,finite state;3robot cameras and30-frame video rendered and decoded. Before/after overhead and R1 camera visually inspected. This is a fixed motor runtime probe, zero LLM calls, not delivery or collaboration success. Evidence outputs/mac-runtime-check-20260907/.
- Existing Mac loopback8091 bridge reports connected authoritative local/MAC worker; queue0,inflightNone. Oracle SSH succeeds and Oracle loopback8091 bridge is healthy but no remote worker connected. No restart,authority switch or live-world commands performed.
- .sim_bridge_token and .sim_worker_env absent at documented repository paths on both hosts; .env.gpu exists. Other authentication locations/in-memory configuration and deletion/sync causality were not verified. Local runtime works; historical Oracle-to-Mac recovery path is not validated.


### 2026-09-07 Camera budget candidate integrated on Mac; one live pilot failed honestly

- User requested continuation from `ugrp_budget_repair_report.md`. Verified all11candidate code/test base hashes against Mac, applied only harness/scripts/tests; preserved existing .gitignore and decision-log changes, unchanged sim physics. Report retained at docs/camera_budget_repair_20260907.md.
- Located historical RGB/video artifacts intact in /Users/changmin/Project-Archives/20260907/ugrp/outputs/warehouse_research. Four read-only reference symlinks restored test access; no archived files moved/edited. Mac focused122passed,29subtests,zero failures/skips after restoration and ordered action-chain validator correction.
- Exactly one real Gemini3.8 solo41 pilot,30calls/60000input/300SIMseconds,impratio10,noslip3,communicationnone. Output camera-budget-repair-dev-01/solo-41,source da86e3f4dd0ca695d93a46cd5f3277a4a55d69eaa8df7b374c34cffe483e122b. 14calls,56321reportedinput,0unknownusage/unsettledreservations;170.828activeSIMseconds+1passivesettle. End LLM_INPUT_TOKEN_BUDGET_PREFLIGHT:lift6.936cm,not inside,no release.0peer/obstacle penetration metric. No429 in this run; no extra availability completion or automatic rerun.
- Video1375frames fully decoded;8chronological spectator frames and all14originalnav inputs reviewed.9in-place rotation decisions cost36981inputtokens/13.5elapsedcontrolseconds,then2forwarddrives,thenobstacle-veto. This is not deliverysuccess or proof of general efficiency improvement.
- Live audit found candidate integration defect: executor emits top-level execution, runner read details.execution.236logged macro results but0/14requests receivedlast_macro. Fixed runner record_command and regression through real executor callback to planner transmitted context with explicit offline completer. Added verifier trace-to-context gate; it now correctly rejects original pilot feedback. Related final85tests/23subtests pass. No further real Gemini calls.
- Current runner+verifier differ from frozen live source; do not relabel this run as validation of final fix. Next validation requires a NEW fixed-budget solo41 run, verify last_macro reaches actual provider,then inspect navigation and release. No budget increase,team expansion,or claim that feedback fix alone solves delivery. Details docs/camera_budget_repair_mac_20260907.md;analysis and QA outputs/camera-budget-integration-20260907/.

### 2026-09-07 Google Drive evidence retention requested and verified

- User requested evidence for future UGRP work be retained in Drive. Added the workflow to local agent.md and the existing Drive agent.md; full text read-back matched while preserving original instructions.
- Evidence root: https://drive.google.com/drive/folders/14vmzOC0TgdFMwBqiLMppM4i8HivRxmes ; current run: https://drive.google.com/drive/folders/16XG3SAiqH1sAayM1MHgeSLEqixLvDoe8 .
- Uploaded report, failed-pilot video, decision camera overview, ZIP containing 907 evidence files, and SHA-256 manifest. Read-back metadata verified all five file sizes and parents. Local receipt: outputs/drive-evidence-20260907/upload-receipt.json. No further experiment was run; failed delivery and post-pilot source differences remain explicitly documented.

### 2026-09-07 Fixed execution-feedback actual Gemini retest

- User authorized same-budget solo41 retest. Ran exactly one new actual Gemini3.8Flash trial: camera-feedback-fixed-20260907/solo-41. Source 6ae7efdebb983da2ed73d6b1befb8c78ccc2114e2bcbea961d45e9acd119fb21. Only runner and verifier differ from prior pilot manifest; no physics/prompt/budget changes.
- 13 calls,56462 input tokens,176.130 active SIMseconds plus1.000settle; no usage gaps/unsettled reservations/inference errors. Actual transmitted last_macro matches command trace for all12post-first requests. Accounting/RGB/config gates passed; full delivery/release gates failed honestly.
- Outcome LLM_INPUT_TOKEN_BUDGET_PREFLIGHT;lift6.95cm,no destination entry/release.7rotation decisions consume31188inputtokens,2forwardexecutions,2forwardguardrejections. Call12 explicitly reacts to call11 rejection by fwd0turn; call13 still blocked. Target error1.651m versus previous1.650m,not improvement evidence.
- All1418video frames decoded;12chronological scenes and13originalNAV inputs visually reviewed. Sol independently reviewed raw feedback and remaining navigation failure. motion_confirmed:false intentionally means no physical displacement attestation; do not flip it to true.
- Next repair focus: ownRGB destination bearing/size and clearance changes for turn/avoidance evidence; first inspect saved calls6-8 and11-13. No hidden-coordinate route,guard relaxation,budget increase,or additional paid trial. Report docs/camera_feedback_fixed_retest_20260907.md.
- Drive uploaded report,video,camera sheet,1124-file evidence/source ZIP,and manifest; all five sizes and parent folder verified. https://drive.google.com/drive/folders/1VZDIozd1CLoWUSugmApFO6dHzIdpPzFE . Receipt outputs/camera-feedback-fixed-review-20260907/upload-receipt.json.


### 2026-09-08 Navigation trial/error: two actual solo41 completions

- User authorized recording hypotheses and iterative actual Gemini experiments. Added ownRGB navigation_evidence and model-authored navigation_note, clarified safe longer existing macro choices and normalized control units. Fixed newly introduced screen-coordinate note rejection; malformed auxiliary notes preserve valid actions with recorded diagnostic.
- C1/C2/C3 at60k failed honestly(55967/52631/56703input). C2 had1note-parser rejection, C3 had1stale response. Frozen3-call probe used16519input and no physical commands.
- Unchanged C3 policy with120k cap completed twice: D1 18calls94065input204.252SIMseconds; D2 17calls86818input201.766SIMseconds. Both VISUAL_RELEASE_CONFIRMED; all14 explicitly120k automated checks passed. D1-D2 manifest difference only verifier script; policy/physics unchanged.
- Decoded all1643/1623video frames and manually reviewed chronological samples,all18/17originalNAV inputs,and finalwrist. Destination-inside,stable,noattachment,visualrelease passed; obstacle/peer>2mm penetration0. Current execution sources match D2 manifest.121tests+23subtests passed; six owned process groups stopped.
- Scope: seed41/one robot only;60k success and generalization remain unproven. No claim that memory or budget alone caused improvement. Persistent bypass-side state is a future unimplemented cost-reduction candidate. Total current-task70actualcalls/362703reportedinput.
- Local report docs/navigation_trials_20260908.md; full journal,failed/successful trials,source snapshots,and camera/video evidence outputs/navigation-trials-20260908/. No Drive work per project exception.
