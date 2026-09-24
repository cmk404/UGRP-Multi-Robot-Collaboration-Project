# v40 카메라 기반 화물 회복 물리 비교

이 문서는 소스 동결과 물리 실행 전에 정한 비교 계약이다. 기계 판독 기준은 [physical-protocol.json](physical-protocol.json)이다. v29 원초기 위치 진단 두 번의 실패와 미실행 근방 6회는 원본 그대로 남긴다. v40의 범위는 동일 open 장면의 작은 초기 위치 변화이며, relink 기능 단독 인과 효과나 다른 지도로의 일반화는 주장하지 않는다.

기준은 `action-act-refinement`의 `931d910998a94361342c372ec2675c475daa625f`, 후보는 커밋·동결할 `rolling-cargo-recovery` v40이다. 두 팔은 `open`, seed 11, `dock_a`, `local_contact_fine`, grasp-start overlap, native realtime 1배속, 10 FPS, 동일 저장 계획·grasp/stage 모델·카메라 및 weld OFF를 쓴다. 후보에만 `--rolling-visual-servo --bounded-carrier-relink`를 추가한다. 학생 제어에는 own/TOP RGB와 자기 발행 명령 이력, 사전 지도만 허용한다.

먼저 원초기 위치 `[0,0,0]`에서 후보 진단 `original/candidate-diagnostic-1`, `original/candidate-diagnostic-2`를 각각 한 번 실행한다. 두 실행 모두 물리 성공, 프로토콜 완결, raw·manager·launch·freeze·source 해시 연결과 입력/RGB/lease/weld 경계 감사를 통과해야 근방 비교를 시작한다. 실패 또는 증거 부재 시 근방 6회는 미실행으로 남긴다.

근방 조건과 순서는 고정한다.

| 조건 | spawn offset | 첫 팔 | 둘째 팔 |
| --- | --- | --- | --- |
| roll-a | `[.004,-.005,.15]` | 기준 | 후보 |
| roll-b | `[-.004,.005,-.15]` | 후보 | 기준 |
| roll-c | `[.005,.004,.35]` | 기준 | 후보 |

각 실행의 내부 최대 시간은 350초, 소유 자식 제한은 360초, 프로세스 외부 한도는 370초다. 진단 프로세스 합계 740초, 근방 비교 합계 2220초이며 새 실행 전에 370초 할당이 남아야 한다. 자동 재시도나 대체 조건은 없다. 근방 팔의 보통 물리 실패는 결과로 남기고, 소스·입력·RGB·lease·weld 감사가 유효하면 다음 팔을 계속 기록한다. 인프라 간섭, 증거 손상, 불변식 위반은 이후 실행을 중단한다.

세 대응쌍 모두 양쪽 물리 성공·프로토콜 완결·완전 감사·경쟁 없는 유효 시간이 있어야 속도 방향성 판정이 가능하다. 후보 세 실행의 **전체 임무 완료 wall time 중앙값**이 기준 세 실행 중앙값보다 10% 이상 작아야 속도 문턱을 통과한다. 성공한 실행만 사후 선택해 시간을 계산하지 않으며, 실패·미실행·시간 제외는 분모에 남긴다. 개별 대응쌍 차이도 모두 표시한다.

접근 SIM 시간, 빔 미세 정렬 SIM 시간, 상자 통행 허가 대기 SIM 시간, 나머지 단계별 SIM 시간, 전체 wall time, motion SIM/wall 비율과 호스트 부하를 각각 보고한다. grasp-start overlap에서는 접근 단축이 빔 파지 대기에 흡수될 수 있으므로 접근 시간 또는 발행 명령 창의 단축만으로 전체 임무 가속을 주장하지 않는다. 발행 명령 창은 실제 측정된 이동이 아니다. motion SIM/wall 0.98 같은 임의의 사후 컷은 두지 않는다. 외부 프로세스 경쟁이나 불완전 감사는 시간 판정에서 제외하고 이유를 보존한다. Claude의 동시 실행 결과 및 다른 코호트와 합산하지 않는다.

소스 커밋 후 부모 실행자가 source/bundle/계획/모델/helper/host-guard 해시를 동결한다. 단일 실행 도우미는 고정 host guard의 `foreign_snapshot`과 `run_owned`를 써서 시작 전과 실행 중 다른 물리·훈련 작업을 감시한다. 간섭 때 자신의 프로세스 그룹만 정리하고 다음 실행을 막는다. 각 실행 후 부모가 별도 raw 감사를 수행해 해시 연결·물리 결과·입력 및 물리 경계가 담긴 영수증을 만들면 다음 실행이 열린다. 원본과 실패 증거를 덮어쓰지 않는다.

실행 형식은 `python3 /Users/changmin/projects/ugrp/outputs/act-action-training-20260924/audit-tools/run_cargo_recovery_trial.py original/candidate-diagnostic-1 --freeze <동결 JSON 절대경로>`이다. 한 호출은 한 팔만 실행한다. 다음 호출 전에 `/Users/changmin/projects/ugrp/outputs/act-action-training-20260924/audits/cargo-recovery-original-candidate-diagnostic-1.json` 같은 별도 감사 영수증이 필요하다. 영수증의 `schema`, `trial`, `raw`, `manager`, `launch`, `freeze`, `source_sha`, 다섯 SHA-256 연결 필드, `audit_passed`, `boundary_pass` 일곱 항목, `outcome.physical_success`, `outcome.protocol_complete`, `outcome.wall_s`, `outcome.timing_eligible`를 도우미가 실제 파일 및 결과와 대조한다. 도우미는 감사 영수증을 스스로 만들지 않는다.
