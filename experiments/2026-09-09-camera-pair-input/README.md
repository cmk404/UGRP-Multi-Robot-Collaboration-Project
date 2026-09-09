# 자기 시점 + 상단 카메라 입력 경계 변경

요청: 좌표 기반 제어를 거부하고 로봇마다 자기 시점과 위에서 보는 영상만 입력으로 사용. Issue #29 / PR #24.

## 변경

- 독립 CameraPairPlanner r1/r3에 own robot_cam JPEG와 공용 고정 cctv_top JPEG 두 장만 전달. 고정 작업/제어 명령 설명 외 텍스트 관측 없음. 상태/좌표/접촉/PWM/메모리/동료 메시지 입력 없음.
- 각 모델이 raw drive/look/arm/wait를 선택. executor는 모터·서보 명령의 시간 제한/보간만 수행. 좌표 이동, IK 파지·상승 매크로, 접촉 기반 단계 전환 없음.
- fixture 최초 배치와 카메라 장착 설정은 환경 구성에만 존재. 실제 물체/로봇 좌표는 evaluation-only.jsonl 및 시청용 비디오 표시에만 사용하며 모델 입력에는 제공되지 않음.
- 이전 좌표 진단의 CLI 기본 실행 차단. 역사적 진단은 명시적 --ground-truth-diagnostic 없이 실행되지 않음. 모델 실패 시 좌표 기반 대체 경로 없음. 정상 물리 접촉·마찰 유지, weld 사용 없음.

## 실제 검증

1. 5eab546: 실제 요청2건, 응답 JSON 형식 문제로 명령 적용 전 중단. 입력2건 감사 통과. 최초 원문 응답을 실패 로그에 저장하지 못한 한계를 보존.
2. 2dc55ae: 표준 JSON 코드펜스 허용 및 실패 응답 보존 후 재실행. Gemini3.8 Flash 두 독립 인스턴스, seed11,8rounds/8SIM초,16응답/16명령(주행15,wait1) 완료. 모델 reported prompt_tokens 총39,280. provider 요금은 확인되지 않아 금액 추정 없음.
3. 실제 전송 HTTP body16건을 원본 JPEG와 바이트 단위로 비교: 정확히 자기 이미지1장+같은 시각 상단 이미지1장, 고정 텍스트 일치, 추가 관측 필드 없음. 두 로봇 첫 실제 입력/전송 body 예시를 이 폴더에 저장.
4. 비디오8개 프레임(약1초 간격) 검토: 실제 접근과 방향 변경 확인. 파지/운반은 달성하지 못했으며 입력 제한 성공과 구분. final result의 transport_success=false는 운반 성공 주장이 없다는 뜻이며 자동 완주 판정기를 구현한 것은 아님.
5. 필수 회귀196 tests/122 subtests 통과. 구 CLI 실행이 거부됨 확인. GitHub CI는 PR에서 별도 확인. 시험세션 종료.

## 범위/한계

이 변경은 공동 운반용 새 실행 경로의 입력/동작 경계를 구현·실제 검증한 결과다. 기존 저장소의 모든 독립 실행기를 전면 대체한 것은 아님. 현재 각 모델은 현재2개 영상만 받으며 자체 이력/동료 통신도 제공하지 않는다. 운반을 위해서는 이 영상 경계 안에서 시각적 자기 식별, 접근·파지 판단을 추가 검증해야 한다. 좌표 기반 성공 결과는 이 경로의 성능 근거로 사용하지 않는다.

raw 위치: /Users/changmin/projects/ugrp/outputs/camera-pair-20260909-{first,validated}. 전체 영상과16건 원본 요청은 로컬 보관, 원격은 코드/보고/첫 입력 예시/해시/평가 기록. main병합은 명시적 사용자 승인 대기.

실행: `.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run camera-pair -- .venv-sim-worker-mac/bin/python scripts/run_camera_pair_transport.py --out-dir outputs/NEW-camera-pair --rounds 8`
감사: `.venv-sim-worker-mac/bin/python scripts/audit_camera_pair_inputs.py outputs/NEW-camera-pair`
