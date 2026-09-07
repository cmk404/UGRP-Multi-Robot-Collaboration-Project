# 작업 중 재판단·센서·도움 요청 검토 및 구현

2026-09-05. 사용자의 “각각 검토해보고 구현해봐” 요청에 대한 작업 범위와 증거.

| 항목 | 구현한 내용 | 검증과 남은 범위 |
| --- | --- | --- |
| 작업 중 재판단 | 작업별 RecoveryEvent, busy agent event wake, participant-only resume/replan, stale event 거부, solo pause/resume와 안전한 내려놓기 | 실제 LLM 장애물 재계획 및3/3도착. joint 내부는 tick단위 pause gate이며 모든phase가별도publicskill로분해된것은아님 |
| 독립 작업 | 공동pause 동안 single physics clock이 solo state를 계속 진행. ROBOT_BUSY 상태변화가 agentloop를종료시키던race 수정 | 실제pause2.69s 중R3 APPROACH→LOWER, 공동resume후전체완료 |
| 센서 불확실성·하중 | RGB-D MAD 기반거리표준편차, fresh/current_view/confidencegate. 가상wristforce에서stationary200sample하중추정 | 30g→29.8g,180g→179.5g 추정.180g은help대기. 실제하드웨어force/odometry calibration은없음 |
| 도움 요청 | 측정하중초과→사건→request_help→안전하게낮춤/해제→지원대기, 모르는width/mass는관측필요판정 | 작은상자에두그리퍼를붙이는기존joint방식은정렬실패. 받침/교대/긴물체한정중사용자선택대기 |
| 평가 | 새mixed-recovery runner가동일seed·호출예산·최대시간·장애물위치로rule/no-comm/peer비교.미발생scenario를성공으로처리하지않음 | seed11 장애물pilot 세조건각1/1성공. 통신LLM우월성증거는아님 |

## 실제 증거

- `outputs/warehouse_research/recovery-obstacle-llm04/result.json`: joint pause14.10s, replan완료16.79s, 3cargo성공, peer collision0.
- `outputs/warehouse_research/recovery-obstacle-llm04/episode.jsonl`: r1/r2 각LLM이직접replan응답. r3는자기작업계속.
- `outputs/warehouse_research/recovery-obstacle-llm04/mixed-solo-joint-1x.mp4`:1배속전체영상.
- `outputs/warehouse_research/recovery-comparison-01/design.json` 및 `summary.json`: sourcehash별고정pilot.
- 전체1005tests /168.460s /OK 이후가상loadsensor·safehandoff추가집중회귀별도기록.

## 경계를 숨기지 않는 조건

장애물은 테스트에서 추가한 실제충돌geom이며 이벤트출처는 `controlled_scenario_event`이다.
카메라가스스로장애물을발견했다고표현하지않는다. Controller의경로/정렬은여전히SIMgeometry를
사용한다. 완전한sensor-only motion control에는 calibrated odometry/FK와관측map제공자가
필요하며, 현재MasterPi에는그계측기반이없다. 마커에서화물질량을읽거나색상으로추측하지않는다.
가상손목센서는MuJoCo의wrist wrench를측정하고화물body_mass/pose를읽지않는다.

도움요청이곧물리적공동운반완료를뜻하지않는다. 작은상자는두그리퍼접근공간이부족하므로
지원방식선택전에는지지되지않는자동handoff를실행하지않는다. 현재는안전한낮춤/해제까지구현했다.
