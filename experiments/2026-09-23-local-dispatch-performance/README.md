# 로컬 dispatch 속도·병렬 실행 검증 — 2026-09-23

표준 MuJoCo 창 실행의 지연을 촬영·자원 잠금·observer 비용으로 나누어 수정했다. PR #118의 접근 동기화 위에 쌓은 PR #124이며, 별도 물리 복구 PR #120과 결합하지 않았다. 사전 조건과 후속 유한 실행 예산은 [protocol.json](protocol.json), 모든 성공·실패와 원본 경로·해시는 [results.json](results.json)에 있다.

## 변경

- 표준 dispatch는 pair가 사용하지 않는 own camera와 중복 overview 촬영을 생략한다. 고정 상태의 4 paired capture에서 actor 이미지 bytes와 물리 상태가 같았다. 촬영 중앙값은 0.19417→0.11699초, **39.75% 감소**다. 이 수치는 전체 임무 시간 감소율이 아니다.
- 합의된 open-map의 독립된 경로·자원일 때 기존 동시 운반 gate를 자동 선택한다. 선행 조건·공유 경로·지형 등은 직렬 유지, 공용 하역 구역은 독점 유지한다. `overlap_selection`에 적용 이유·계획 해시를 남긴다. 이전 조건은 `--full-capture --serial-route`로 선택한다.
- v6에서는 fine physics step마다 호출하던 native GUI 확인·짧은 sleep을 wall 시간 기준으로 묶었다. v7에서는 **복제된 관찰 창만** 그림자·반사를 끄고 MuJoCo 3.12 state-only sync를 사용한다. actor RGB·원본 영상 렌더러·물리 timestep·제어 명령 주기는 유지했다.
- v5/v6 번들을 원본 그대로 보존하고 최종 v7/workflow1.5.0으로 등록했다. 공통 adapter의 legacy/MacroQueue 계약과 이 실험의 local_contact_fine/VisualMacro 실행은 별도 조건이다. 이 결과로 공통 adapter의 물리 자격을 승계하지 않는다.

## 실제 결과

open/seed11/dock_a, 같은 저장 계획·RGB grasp/stage 모델·local_contact_fine(0.00025초), weld OFF, video10fps다. 새 외부 LLM 호출은 전부0이며 기록의 fixture 투표6회와 구분한다. 미기록 비용은 null로 보존한다.

| 실행 | 실행 SHA | 현실 초 | SIM 초 | 두 화물 물리 성공 | 명령(SETUP 제외) | 동시 적재 이동 SIM 초 |
|---|---|---:|---:|---|---:|---:|
| baseline | `dbf7073` | 367.060 | 167.700 | True | 936 | 0.0 |
| candidate | `c44b571` | 285.490 | 140.300 | True | 949 | 2.9 |
| native-window | `c44b571` | 46.790 | 19.900 | False | 139 | 0.0 |
| native-full | `c44b571` | 601.993 | 128.400 | False | 938 | 2.9 |
| native-v6 | `dd02695` | 429.020 | 140.300 | True | 949 | 2.9 |
| native-v7 | `6916e8c` | 330.727 | 140.300 | True | 949 | 2.9 |

headless baseline/candidate는 각각1회이고 둘 다 완주했다. 현실 시간은 **367.060→285.490초(22.22% 감소)**다. 촬영 최적화와 운반 중첩을 함께 적용했으므로 효과를 각 변경으로 분리하지 않는다. strict loaded-motion은 issued TRANSIT와 실제 referee 변위가 함께 있는 구간만 합산해 **0→2.9 SIM초**이며, 항상 모든 로봇이 동시에 움직였다는 뜻은 아니다.

`native-window`는 의도한45초 시작 확인의 예산 종료이고, `native-full`은 v5에서600초 예산을 소진한 미완료다. 두 실행을 완주 성공률의 반복 표본으로 합치지 않는다. v6와 v7은 viewer=true/realtime_factor=2의 별도 전체 실행이다. v6 **429.020초**, v7 **330.727초**이며, 둘 다 완주했고 관찰 창 비용 감소로 현실 시간이22.91% 줄었다. issued/referee/pair/solo 원본4개가 byte-identical임을 독립 감사했다. 샘플 접촉·weld는 원본 evaluation에 기록하며, 샘플 사이의 모든 순간을 무접촉이라고 주장하지 않는다.

## 왜 화면에서 느린가

SIM 시간은 물리 적분이 진행한 시간이고 wall 시간은 사용자가 실제로 기다린 시간이다. v6는 같은140.300 SIM초를429.020 현실 초에 처리했다. 직선 적재 운반의 실제 물리 속도는 약0.11m/SIM초(10Hz referee 표본 중앙값)이므로 이 처리율에서는 눈에 약0.036m/현실초로 보인다. `forward=.08`은 정규화 모터 명령이며 0.08m/s가 아니다. 2배속 설정은 목표 상한이고 계산 비용 때문에 달성되지 않을 수 있다. 화면에 과거 영상을 느리게 재생하는 현상과 구분한다.

사용자가 봤던 원본 실행의 LLM3개는 이미 병렬 호출이었다. 역할 제안을 수정·동의하는4라운드에 약42.1초가 걸렸다. 이번 결과는 저장 계획 재생으로서 LLM 응답·합의 지연을 개선했다는 근거가 아니다. v6의3초 프로세스 표본은 native 렌더링·잠금 대기를 주요 후보로 가리켰지만, 그 표본 비율을 전체 임무 시간 비율로 확대하지 않는다.

## 검증·보존

초기 관련 회귀184 passed/1 skipped, v6 통합 회귀132 passed/1 skipped, v7 변경 영향 회귀24 passed/1 skipped를 확인했다. 건너뛴 항목은 opt-in viewer lifecycle 테스트이고, 별도 실제 창 전체 실행을 수행했다. registry 불변성과 최종 source hash 검사를 통과했다. 실행 SHA와 이후 결과 기록 커밋 SHA는 다르다.

각 run의 manager manifest는 source/input 불변을 기록한다. headless c44와 native v6의 issued/referee/pair/solo trace는 byte-identical임을 독립 감사했다. 원본과 추가 감사는 로컬 `/Users/changmin/projects/ugrp/outputs/simulation-performance-20260923`에 보존하며 raw의 원격 백업을 주장하지 않는다. 새 native TensorBoard 스냅샷은 기본 체크아웃 `outputs/tensorboard`에 추가하고 실제 이벤트 로드·원본 MP4 등록/Range206을 확인한다. 보기 설정은 기본 체크아웃 `outputs/tensorboard-view.json`의 `local_dispatch_performance_20260923` 및 native 비교 항목에 저장한다.

원문 조사와 추후 설계 제안은 [연속 시각 제어 조사](../../docs/continuous_visual_control_review_20260923.md)에 분리한다. 주행 중 관측·다음 추론 중첩은 이번 PR에서 도입한 기능이 아니다. 새 지도·다른 seed·ACT 학생 또는 새 LLM 전체 실행으로 일반화하지 않는다.
