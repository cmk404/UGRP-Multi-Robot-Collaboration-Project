# 장시간 파지 검증

기준: PR #48 / d3ab17d, 실행 f8deb28. narrow-door 운반 90.7초 동안 lift가 7.530 → 4.919 cm로 감소했다. 동작과 제어 입력을 보존한 채 팔 처짐/물체 상대 미끄러짐을 분리한다.

첫 비교는 같은 고정 시작 조건, 1600 PWM, impratio 10 / noslip 0 / weld OFF다. 카메라/FOV·형상·마찰·질량·학생 모델·팔 명령·들어올리기·10.2초 대기를 유지한다. stationary와 shuttle 각각 최대 300초, 같은 원본의 180초와 300초 prefix를 평가한다. 독립 반복 두 번으로 세지 않는다.

정지 조건은 원래 간격을 RGB로 유지한다. 왕복 조건은 최초 RGB 기준에서 world +x 방향으로 0–16 cm, 12초 주기의 부드러운 왕복을 발행한다. 카메라로 위치·자세를 계속 보정하며, 사전 지도에서 왕복 구간의 발자국을 확인한다. 실제 좌표·관절·접촉·평가로 행동이나 단계 전환을 수정하지 않는다. 영상 불확실/간격 이탈은 공동 정지한다. 예산 종료는 명령 개수로 결정한다.

출력 전용 평가: 전체 lift/hold 기존 기준 + 180/300초 완료, 접촉·lift 3cm·수평 10도·간격 2cm·표본 간격 0.15초·충돌 없음·weld OFF·불변 조건. 장시간에는 초기 운반 높이에서 최대 감소 1cm 이하를 추가한다. 실패/미완료는 모두 기록한다. 집게 중심 world z와 물체 상대 z, 관절각은 평가 파일에만 추가한다.

근본 원인을 좁힌 뒤 한 변수만 바꾼 후보를 커밋해 비교한다. 최종 후보의 stationary/shuttle 5분과 기존 6개 지형 전체를 같은 코드 버전에서 확인한다. 외부 LLM 호출/비용 0. 원본은 outputs, 보고서·선택 영상·원본 해시는 experiments. PR은 #48 위에 작성하며 main 병합은 승인 뒤 수행한다.

## 기준 실패와 명시적 솔버 비교

실행 2db7acb: 정지 93.94초, 왕복 94.24초에 own RGB 물체 소실로 공동 정지했다. 정지 90초에서 물체는 약 25.85mm 하강했으나 집게 중심은 약 0.35mm 하강했다. 팔 처짐보다 집게 내부 상대 미끄러짐이 주된 관측이다.

후보는 noslip_iterations 0→3만 변경한다. MuJoCo의 [Preventing slip](https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip)은 soft contact 정규화가 지속적 미끄러짐을 만들 수 있고 NoSlip 후처리로 억제할 수 있다고 설명한다. 수치 솔버 변경의 효과를 검증하는 것이며 실물 성능 개선이나 제어기의 일반화를 뜻하지 않는다. 마찰/질량/형상/카메라/1600 PWM/팔 명령/두 RGB 제어기는 그대로 유지한다. 기본값은 0, 후보 실행 명령에 --noslip-iterations 3을 명시한다.

후보에 실제 endurance actuator trace를 추가하고, 300초 명령 완료 후 고정 place 시연을 실행해 바닥 방출/양쪽 접촉 해제를 출력 전용으로 확인한다. 이 추가 동작은 처음 300초 비교 구간에 영향을 주지 않는다. 기준 기록에는 release_check_required가 없으므로 이전 평가를 그대로 재현한다. 180/300초 접촉 충돌 판정은 전체 endurance 구간 누적값을 보수적으로 공유한다.

NoSlip 후보 863bbae는 두 모드 모두 초기 24.5 SIM초에 영상 간격 안정화 조건을 만족하지 못해 출발 전 종료했다. lift/hold 중 bilateral 누락 1표본, 간격 최대 변화 1.469cm로 기존 기준 파지보다 퇴행했다. 이 설정은 채택하지 않는다.

다음 비교는 기준의 elliptic cone / NoSlip 0을 유지하고 impratio 10→100 한 변수만 변경한다. 위 공식 문서의 elliptic cone에서 friction impedance를 높여 soft contact 미끄러짐을 줄이는 방법을 검사한다. solver2/Newton, iterations80, tolerance1e-8, dt.002는 유지한다. 명시적 --impratio 100 --noslip-iterations 0 후보를 2개 5분 조건과 기존 6지형에서 검증한다.

impratio100 / 기존 RGB PD 후보 fd68a2c도 양쪽 모드 출발 전 실패했다. 초기 접촉은 유지했으나 간격은 2.776cm 줄었고 영상 안정화도 만족하지 못했다. 동일 PD에서 더 강한 접촉 조건은 잔류 위치 오차를 남겼다.

그 다음 후보는 impratio100 / NoSlip0을 유지하고 RGB 위치 오차의 누적 보정만 추가한다. integral gain2, world 축별 보정 기여 ±0.08m/s 제한, 기존 최종 포트 속도 제한/복구 범위/정지 조건은 유지한다. 영상 실패 이후 적분하지 않는다. 파지 hold와 endurance의 자기 RGB+공용 RGB 위치 오차에서만 계산한다. 기본 gain0과 명시적 --spacing-integral 2를 기록하고 저장 영상으로 재현한다. 물리 파라미터와 제어 보정을 함께 바꾼 최종 후보가 되므로 최종 효과를 어느 한 변경 단독 효과로 주장하지 않는다.

RGB 적분 보정 후보 ff2d423도 출발 전 실패했다. 최대 간격 변화 2.752cm로 개선이 미미하고 최종 횡방향 명령이 ±0.1m/s에 포화됐다. 이 후보도 채택하지 않는다.

다음은 baseline의 impratio10/NoSlip0/integral0으로 복귀하고 네 손가락–빔 접촉만 명시적 pair로 기술한다. 모델의 geom defaults를 정적으로 컴파일해 기존 동적 혼합 규칙의 normal solref/solimp/condim/friction/margin/gap을 그대로 복사한다. 유일한 접촉법칙 변경은 elliptic cone의 solreffriction을 0 -3000으로 지정해 마찰 방향 속도 감쇠를 높이는 것이다. 바퀴/바닥/관절/형상/질량/마찰계수/정상 방향 접촉법칙과 모든 제어기는 baseline과 같다. [MuJoCo pair solreffriction](https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-pair-solreffriction) 및 [friction solver](https://mujoco.readthedocs.io/en/stable/modeling.html#friction) 정의를 따른다. 부착 제약이나 adhesion은 추가하지 않는다. 접촉이 사라지면 마찰도 사라지는 일반 접촉이며, fixed place 후 완전 분리를 확인한다.

마찰 감쇠 3000 후보 17e59b9는 13.174초에 QACC 수치 불안정 경고가 발생했고 이후 영상 guard가 초기 파지에 실패했다. 5분 시험으로 진입하지 못한 실패이며 채택하지 않는다.

다음 후보는 감쇠를 원래 solref .007 1로 복구하고 네 손가락–빔 접촉의 solimp 첫 두 값만 혼합 기준 .91/.975→.995/.999로 높인다. 나머지 solimp 형상값, normal/friction reference, 마찰계수/마진/형상/질량/바퀴/전역 솔버/제어는 baseline과 동일하다. 이 변경은 정상 방향과 마찰 방향의 접촉 유연성을 함께 줄이는 명시적 모델 변경이다. 사용자에게 제어기 개선이나 실물 파지 증거로 제시하지 않는다. --stiff-finger-contact 옵션으로 비교하며 기본은 이전 동적 접촉이다.

## 최종 채택 범위

국소 impedance 후보 887e95d도 lift/hold 중 간격 변화 3.115cm와 접촉 누락 3표본으로 실패했다. 모든 변경 후보를 제외한다. 최종 코드에서는 기존 파지/주행/물리/카메라를 baseline과 동일하게 복구하고, 장시간 시험/출력 전용 finger 진단/입력 감사만 유지한다. 제외된 후보의 소스는 해당 실행 SHA에서, 실패 결과와 입력 감사는 기록 파일에서 확인한다. 최종 감사 도구는 제외된 후보 설정을 거부하며 해당 과거 소스로 재현하도록 요구한다.

최종 코드로 stationary/shuttle 각 300초 예산의 기준 시험을 다시 실행한다. 조기 중단은 180/300초 모두 실패로 평가한다. 채택할 행동/물리 변경 후보가 없으므로 이번 최종 단계의 6지형 재시험은 적용 대상이 아니다. 기존 6지형의 실행 코드는 PR #48과 바이트 단위 동일함을 확인한다. 3/5분 성공을 선언하지 않고 실패 분석·후속 파지 자세 연구를 남긴다.
