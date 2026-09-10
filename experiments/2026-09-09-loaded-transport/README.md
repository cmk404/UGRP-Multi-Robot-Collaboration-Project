# 60cm 접근 → ㄱ자 파지 → 50cm 운반

요청: 들어 올린 뒤 다른 위치까지 이동. Issue #26, PR #24.

최종 코드 ac3626f (전체 SHA는 final-manifest.json). seed11, Python/MuJoCo/macOS 환경은 manifest 참조. 기존 접촉/마찰/질량/서보 설정 유지, 손잡이 없음, weld OFF. 초기 fixture 배치 외 pose setter 없음. 목표는 현재 위치 기준 +X 0.5m이며 바퀴 제어는 알려진 시뮬레이터 좌표를 사용한다. 카메라 탐색/LLM 협업은 아니다.

## 결과와 전체 시도

- 최초 2313827: 동시/2초 지연 모두 물체 유지했으나 실제 0.5521m 이동, 목표 초과 약5.2cm로 도착 실패. 2조건 모두 실패로 보존.
- 최종 ac3626f: 운반 구간의 기존 wheel controller motor scale만0.18로 제한. 목표 허용오차는 그대로3cm. 2조건 모두 성공, 실제0.51395/0.51396m 이동, 오차약1.4cm.
- 각 최종 조건은60cm 접근, 물리 파지와 상승,2초 유지,운반,정지 후2초 유지까지 연속 실행. 접근 구간 물체 접촉0step.
- 이동 및 최종 유지 약5.10SIM초/조건,0.1초 간격51표본에서 양쪽 로봇의 양쪽 손가락 접촉 유지. 해당 구간 모든 물리 step에서 weld OFF 검사.
- 최종 상승 높이는5.875/5.872cm. 운반 전 약7.16cm보다약1.29cm 낮아짐. 장시간 미끄러짐/유지 성능은 검증되지 않았으며 이2초 도착 유지 이상의 안정성을 주장하지 않음.
- 최종 전체SIM시간17.808/20.638초. 경로 동작: 접근1회(정지/미세정렬 포함), 공동 파지/상승1회,운반1회,도착 유지1회. 실제 세부 sample/time은 trace 참조. LLM호출0,API비용0.
- 로컬 필수회귀189 tests 및107 subtests 통과. GitHub CI는 PR에서 별도 확인.
- 최종 지연 영상 전체12개 균등 프레임(약1.9초 간격) 시각 검토: 출발,접근,ㄱ자 파지,상승,녹색 영역으로운반,정지 확인. 연속 원본 영상도 함께 제공. 모든 실행 세션 종료.

한계: 직선 +X 50cm,seed11의2조건만 검증. 회전/다른 방향/장애물 회피/내려놓기/장시간 유지/실물은 미검증. 성공률은 최종2/2이며 이전 실패2건을 별도 보존. raw영상과로그는 로컬만 보관; 원격은 manifest/summary/hashes/source. main병합은 명시적 사용자승인 대기.

원본: /Users/changmin/projects/ugrp/outputs/transport-grasp-20260909-{initial,slow}

재현: `.venv-sim-worker-mac/bin/python scripts/ugrp_session.py run transport-check -- .venv-sim-worker-mac/bin/python scripts/probe_dual_grasp_sync.py --out-dir outputs/NEW-transport --seed 11 --fps 12 --no-weld --side-grasp --approach-distance 0.6 --transport-distance 0.5`
