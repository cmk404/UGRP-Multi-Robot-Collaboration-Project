# 사전등록: 빔 재파지 뒤 RGB 좌표 고정 (v60)

2026-09-25 작성, 실행 전 커밋.

## 가설

W2(v58)와 V2(v56)에서는 쌍 파지에 실패한 뒤 다시 접근할 때 학습된 lateral 단계가 `rgb_pose_outside_support`로 멈췄다. 오프라인 분석으로 찾은 원인은 다음과 같다.

- 빔은 움직이지 않았다.
- hue≤24 빔 마스크가 노란 북쪽 절반을 가장자리만 잡는다. 실패한 파지 뒤에는 그 가장자리가 가늘어져 검출된 축 길이가 123 px에서 95 px로 줄었다.
- 그래서 TOP 평행 이동량이 (212,96)에서 (212,82)로 바뀌었고, 표준화된 입력이 지원 밖으로 나갔다.

v60 조치: 재파지 재접근에서 hue≤35 빔 중심이 앵커에서 2 px 이하로 움직였고 hue≤24 축이 hue≤35 축 길이의 90 % 미만으로 잘렸으면, 실패한 파지 때의 평행 이동량을 다시 쓴다. 그 밖에는 기존처럼 새로 계산한다.

예측: 같은 주입 조건에서 재접근의 lateral 단계가 지원 안에 머물고, 빔이 dock_b로 운반된다.

## 조건

- 소스: 이 커밋, 번들 `rgb-standard-dispatch-v60`.
- 실행기: `scripts.run_dispatch_e2e`. W2와 같은 인자를 쓴다.
  - `--variant open --seed 11 --task "서로 역할과 순서를 합의해서 beam과 box를 dock_b로 옮겨"`
  - `--model gemini-3.8-flash --coordination dynamic --diagnostic-fail-grasp-once`
  - `--executor skills --contact-profile local_contact_fine --record-replay`
  - 동기 SIM, 뷰어 없음, weld OFF.
- 모델: W2와 같은 `team-recovery-fix/outputs/dispatch-models/e78a5a5777f5bc48/models/{grasp,varied}` 15개 파일(목록 해시 `a8d3e85c…`).
  - reference-top은 이 worktree의 fixture를 쓴다. W2와 SHA-256 `35e99013…`로 같다.
- 반복: X1, X2 두 번. LLM 결정이 regrasp가 아니면(retry·abort) 재파지 경로는 확인되지 않은 것으로 기록한다. 추가 실행은 새 사전등록으로만 한다.
- Claude 물리·학습 잠금 안에서 차례로 돈다. 시작과 끝의 부하 평균을 기록한다.

## 판정

- 기록할 지표:
  - 재접근 중 `transform.regrasp_binding.applied`와 그 근거(hue24/hue35 길이, 이동 px)
  - lateral 단계 결과와 지원 거리
  - 빔·상자 운반, 끝 상태, LLM 호출, 토큰, 제어 종료 SIM초
- 조치 확인: 재파지 재접근에서 `applied=true`이고 lateral 단계가 `rgb_pose_outside_support` 없이 넘어가야 한다.
- 복구 성공: 조치 확인에 더해 빔이 dock_b 성공 판정을 받아야 한다. 판정은 평가 전용 출력이며 로봇 입력이 아니다.
- 두 실행으로는 비율을 주장하지 않는다. W2의 한 번 실패를 고친 진단 확인으로만 보고한다.
