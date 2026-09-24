# 2026-09-25 실행 잠금 누락과 독립 검토

## 실행 중단

원래 v50 진단 코호트는 1회 시작, **인프라 중단 1회, 유효 물리 진단 0회,
후속 진단 NOT_RUN 1회**로 종료했다. 실행 소스는
`0139faf849bc63230aae5ab0057aab695494f2e6`, 번들은
`rgb-standard-dispatch-v50` (SHA-256
`d00d9403dfb0048a2135ad5e208e8f27e743f8eb1e05dc7fd237c5d3449a33b2`)이다.

17:23:38 UTC에 공용 잠금 획득과 외부 작업 검사를 통과했다. 17:23:45 UTC에
`plan-objective-preview` worktree의 `scripts/run_ci_tests.py`가 새 pytest
PID 79539를 실행했다. 엄격한 부하 감시가 자신의 시뮬레이션 그룹만 중단했다.
SETUP 단계였으며 프로세스 8.274626초, 원본 result 6.918656초였다.
운반 판단이나 회복 기전을 평가하지 못했으므로 로봇 실패나 속도 표본으로 세지 않는다.
원본 result의 false 플래그를 바꾸지 않았고 외부 요약에서는 물리 결과를 null로 둔다.
소스·입력 고정과 자기 프로세스 정리를 확인했다. 다른 작업은 종료하지 않았다.

[v50-infrastructure-abort.json](v50-infrastructure-abort.json)에 원본 경로와 해시를
기록했다. raw 로그·입력·관리 manifest는 로컬 보관이며 원격 전체 백업이 아니다.
첫 코호트는 재시도하지 않는다. 후속 실행은 원래 분모를 보존한 별도 사전 계획이 필요하다.

## 원인과 수정 범위

실험 실행기는 원자적 공용 잠금을 사용했지만 전체 테스트 실행기는 사용하지 않았다.
시작 직전 status 조회만으로는 이후 다른 프로세스의 실행을 막을 수 없다.
`run_ci_tests.py`에 동일한 잠금 획득을 추가하고 점유 시 **pytest 생성 전 거부**한다.
자신이 만든 그룹의 종료를 확인한 뒤만 잠금을 반환하고, SIGINT/SIGTERM/SIGHUP도
자기 자식에게 전달한다. Git 공통 디렉터리를 비교하므로 연결된 worktree는 같은 잠금을
쓰고 CI나 별도 clone은 개발자 Mac 경로를 생성하지 않는다.
기존 controller/물리/관측/실행 번들은 변경하지 않는다.
이전 브랜치의 실행기와 직접 pytest 호출은 자동 보호되지 않으며 별도 잠금이 필요하다.

## Claude 결과 독립 검토

[plan-guidance-independent-review.json](plan-guidance-independent-review.json)은
다른 작업의 15회 결과·계획·해시를 대조한 기록이며 root 자체 실험 분모에 넣지 않는다.
C0 평균 제어 종료 157.34 SIM초, C1/C2 각각 141.26 SIM초 (평균 10.2% 감소),
세 조건 중앙값은 모두 141.26 SIM초다. 22% 감소는 직렬 계획과 병렬 계획 부분집합의
비교값이며 전체 조건 효과나 실시간 wall 속도 개선으로 확대하지 않는다.
전체 픽셀/물리 재감사와 새 실행은 이 검토에 포함하지 않았다.

## 검증

소스 구문 및 `git diff --check` 확인. 다른 작업의 물리 잠금이 있어 로컬 pytest는
실행하지 않았으며 GitHub CI에서 잠금 충돌·성공/실패 종료·spawn 오류·SIGTERM 정리를
검증한다. 새 v50 결과는 기본 체크아웃의 TensorBoard에 별도 인프라 기록으로 등록한다.
