# Native console: 명령 입력과 작동 방식

실행 소스 `7fbfd86`에서 Mac의 기존 MuJoCo 환경을 사용했다. 기존 연구 장면과 카메라/로봇 형상을 유지하며 weld는 사용하지 않았다. 실제 영상·평가·모델 입력/응답은 `outputs/console-check-v1/`에 로컬 보관한다. 전체 해시를 result.json과 대조했다.

- manual: `r1 앞으로 0.5초` → 명령 1개, r1 평면 이동 약 0.01478m.
- single: 한국어 요청 → Gemini 3.8 Flash 1호출 → drive 1개, r1 이동 약 0.00238m.
- independent: 같은 지시를 세 로봇 각각의 RGB에 전달, 3호출/3명령.
- peer: 3호출/3명령과 각 로봇의 동료 메시지 생성. 실제 1라운드만 실행했으므로 다음 라운드 수신은 fixture 테스트로 검증했다.
- terminal: 실제 PTY에서 1번 선택, 한국어 동작, script 전환/실행, 정지, 초기화, peer/manual 전환, 상태, 종료. 모델 호출 0회, 총 4개 명령. 마지막 reset으로 최종 위치는 초기 상태다.

모델 smoke 명령은 각각 `bash scripts/open_simulation.command console configs/simulation/local.json --mode <llm-single|llm-independent|llm-peer> --task '자기 로봇을 0.3초 동안 천천히 앞으로 움직여. 이번 응답에는 이동 행동을 선택하고 done은 false로 해.' --max-rounds 1 --max-calls <1|3|3> --exit-after-task --wall-seconds 90 --capture --video --output outputs/console-check-v1/<single|independent|peer>`였다. peer에만 `다른 로봇에게 자기 계획을 짧게 알려줘.`를 추가했다. 시간은 verification.json의 실측값을 따른다. model_latency_s는 개별 요청 wall 시간의 합이므로 병렬 round wall 시간과 다르다.

모든 5회 실행에 console 오류·물리 상태 오류가 없었고 소유 프로세스 종료를 확인했다. 이 결과는 명령 연결 검증이며 운반 성공, 학습 정책 통합, 장기 안정성 또는 통신 효과의 근거가 아니다. 모델의 done과 물리 성공도 구분한다. console은 protocol_complete=false를 유지한다.

선행 recovery의 사용자 창 `outputs/interactive-recovery-v1`도 이번에 종료됐다. 186.262 SIM초의 BADQACC 1건을 기록하고 창이 유지된 채 1800.106 실제 초 제한으로 끝났다. 정확한 GUI 조작은 기록되지 않아 원인을 특정하지 않는다. 실패 기록도 새 TensorBoard snapshot에 포함했다.

검증: 오프라인 1490 passed / 7 skipped / 198 subtests; 신규 console 경계 18개 포함. 이후 TensorBoard 세션 완료/모델 지연 분리 변경은 관련 16개 테스트 통과. Ubuntu native console은 PR110 CI에서 확인한다. 기본 체크아웃과 다른 작업의 실행/뷰어/실험 소스는 변경하지 않았다.

대시보드 및 검증 상태는 dashboard.json, 실행 SHA·환경·호출 원문 해시·검증 범위는 verification.json을 참조한다. 영상/원시 요청은 Git에 업로드하지 않았다.
