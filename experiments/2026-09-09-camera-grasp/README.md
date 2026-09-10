# 좌표 없이 공동 파지 시험 — 실패

사용자 요청: 자기 시점+상단 카메라만으로 물체를 집을 수 있는지 실제 시험. Issue #30 / PR #24.

## 조건

실행 SHA15db9f58f65eb3ba7289e7ec7c90402a851d7833, seed11, Gemini3.8 Flash,2개 독립 stateless 정책. 각 로봇30판단/총60호출,30SIM초. 초기60cm거리 fixture,원래 카메라/마찰/물체/모터물리 유지,손잡이와weld 없음. 고정 task=grasp와 서보 기능 설명을 추가했으며 실제 서보값/좌표/접촉/기억/동료 상태 입력은 없음. 낮추기·파지·상승 좌표매크로와 평가 기반 보정 없음. 모델 추론 중 물리는 정지하며 각 응답 뒤1SIM초의 raw명령만 실행.

## 결과

- 실제 요청60건 모두 자기 JPEG1장+공용 상단JPEG1장과 고정 설명만 포함함을 전송body/원본바이트 대조로 확인.
- 두 로봇 각각 gripper open(servo1=2000)24회. 나머지는 접근 주행과 팔방향 조정. close명령과 shoulder/elbow/wrist 조절 없음.
- 0.1SIM초 간격300개 평가표본에서 r1/r3 양쪽 손가락 파지 모두0회. 최대상승-0.000049m(바닥안정화 수준),성공유지0초. grasp_success=false.
- 성공 기준은 >=3cm 상승+양쪽로봇 bilateral+양쪽weld OFF가 표본시간 기준2초 유지. 20점=1.9초를2초로 세지 않으며,정답판정은 예산종료뒤에만 계산됨.
- 보고된입력토큰160,020. 실제요금미확인,금액추정없음.
- 201 tests/122 subtests 통과. referee threshold/contact/weld/drop 단위검증 포함. GitHub CI는 PR에서 별도 확인. 시험세션 종료.

## 시각 검토와 해석

시청영상0,2,4,6,10,15,20,29초프레임과8번째 실제 자기/상단입력을 검토. 접근 뒤 팔을 내려 집지 못하고 대부분 정지상태에서 열기명령을 반복함. 자기영상에 손가락이 거의 보이지 않고 상단영상에서는 작음. 그리퍼 상태를 시각적으로 구분하기 어렵다는 가설은 있지만,시야·prompt·stateless 영향은 분리실험하지 않아 원인으로 확정하지 않음.

이1회실패는 현재2개현재영상/raw서보정책의 실패이며,좌표 없는 시각파지 자체의 불가능성을 뜻하지 않음. 이전좌표기반성공으로 대체하지 않음. 다음검증방향은 영상에서 손가락/물체관계가 관측가능한지와 시각적인 팔조절을 분리해 확인하는 것.

원본 /Users/changmin/projects/ugrp/outputs/camera-grasp-20260909-first. raw영상/전체요청은 로컬보관,원격에는코드/평가/요약/대표실제입력/원본해시. main병합은 사용자명시승인대기.

재현: `.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run camera-grasp -- .venv-sim-worker-mac/bin/python scripts/run_camera_pair_transport.py --out-dir outputs/NEW-camera-grasp --rounds 30 --task grasp`
