# 주행 중 주변을 계속 확인하는 제어: 원문 조사 (2026-09-23)

## 이 작업에 대한 답

영상 주행 연구의 공통 구조는 **관측 → 짧은 경로/속도 명령 생성 → 움직이는 동안 저수준 추종 → 새 관측으로 보정**이다. 물리 엔진의 매 세부 step마다 큰 모델을 다시 호출하지 않는다. 반대로 긴 경로를 한 번 계산하고 끝까지 무관측으로 실행하지도 않는다. 모델이 낸 짧은 지평의 상대 waypoint를 로봇별 제어기가 추종하는 ViNT, 영상 특징 오차를 속도에 바로 반영하는 visual servoing, 미래 동작 일부만 실행하고 다시 계획하는 Diffusion Policy가 서로 다른 구현이다. 느린 추론이 실제로 병목일 때는 RTC가 현재 동작과 다음 추론을 겹친다. 이 문단은 아래 원문의 방법을 UGRP 질문에 맞춰 종합한 **설계 추론**이다.

**이번 native `open`·독립 북/남 경로의 직접 실행 루프:** `scripts/run_dispatch_skills.py`는 ACT 모델 인자가 없으면 `pair.carry(ImageRoute(..., 'beam'))`를 호출한다. `open` 정적 지도에는 회전 경로를 켜는 `service_island`가 없어 `scripts/dispatch_pair_skill.py`의 `BoundPairSkill.carry()`가 실행된다. 실제 루프는 `capture('carry') → harness/dispatch_skill_binding.py:ImageRoute.observe(top RGB)`로 화물 픽셀과 사전 지도 waypoint 오차를 얻고, 양쪽 own RGB의 결합 추정과 `harness/pair_carry_policy.py:PairCarryPolicy.step()`의 동기·skew 허가를 거쳐 약 `.20` **시뮬레이션초** 이동한다. `ImageRoute`는 픽셀 오차 × `.002`를 정규화 mecanum 명령으로 바꾸고 전진을 최대 `.08`로 제한하며, 목표 허용범위에 들어오거나 skew 회복·허가 대기 중이면 0 명령을 낸다. 세 번째 box 로봇은 `SkillScene._physics()`의 `_solo_tick()`으로 물리 step 사이에 진행한다. 따라서 현 체감상 느림의 첫 분석 지점은 **이 open 경로**의 정지/회복 비율, 실제 영상 변위·명령 속도, 0.2 SIM초당 `capture`·인식·세부 물리·viewer의 벽시계 비용이다. 이 구현의 0.2 SIM초는 화면 5 Hz나 0.2초 벽시계 갱신을 뜻하지 않는다.

별도의 장애물 지도에는 `service_island`가 있어 `carry_with_rotation()`과 `harness/dispatch_pair_navigation.py:PairNavigator.decide()`를 사용한다. 여기서는 현 RGB formation/payload, swept-footprint 정적 지도 경로, 공통 속도, motor clipping/hold gate가 중요하다. 아래의 회전 제어기 제안은 이 경로에만 해당하며, 이번 `open` 실험에서 호출됐다고 해석하지 않는다. 논문 방법 자체는 MuJoCo fine physics step이나 RGB 렌더 병목의 자동 해법이 아니다.

## 원문 비교

| 원문/범위 | 관측 → 출력 → 계속 실행 | 언제 새 관측·계획인가 | 현재 입력 조건과의 관계 |
|---|---|---|---|
| [Visual Servo Control I](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf), 2006, 제어 원리 | 영상 특징의 현재값과 목표값의 오차를 만들고, 근사 interaction matrix로 **카메라 속도**를 계산한다. | 영상이 갱신되는 제어 루프마다 오차를 다시 계산한다. 논문은 고정 Hz를 제시하지 않는다. | RGB에서 보이는 shaft·wheel 특징 오차를 `PairNavigator.decide()`의 속도 보정에 쓰는 원리에 직접 맞는다. 로봇/물체 GT 자세가 필요하다는 뜻은 아니다. |
| [ViNT](https://proceedings.mlr.press/v229/shah23a/shah23a.pdf), CoRL 2023, 지상 이동 | 현재·과거 RGB와 목표 영상 → **5개 미래 상대 waypoint**. 로봇별 저수준 제어기가 waypoint를 실제 속도로 추종한다. | CARLA 부록 실험은 4 Hz RGB 문맥, 현재 시점에서 1.5초 뒤의 목표 영상을 고르고 짧은 궤적을 PID로 추종한다. 4 Hz는 해당 실험 조건이지 보편 요구 주기가 아니다. | 상대 waypoint + 저수준 추종기 분리는 적합. 그 CARLA 실험의 odometry 기반 목표 노드 선택을 현 학생 입력에 그대로 쓰지 않는다. 카메라·하중·목표 표현에 맞는 학습/검증도 따로 필요하다. |
| [NoMaD](https://arxiv.org/pdf/2310.07896), ICRA 2024, 미지 환경 이동 | 현재·과거 RGB와 선택적 목표 RGB → diffusion으로 **8개 미래 행동**. 먼 목표는 관측으로 쌓은 위상 그래프의 subgoal로 나눈다. | 원문 본문은 8개 중 몇 개를 실행하고 관측을 갱신하는지, 제어 Hz, 중간 취소 규칙을 수치로 명시하지 않는다. | 영상 입력과 단기 행동 시퀀스 개념은 적합. 목표가 정적 지도로 이미 주어진 이 작업에 탐험용 goal masking/새 모델 학습은 당장 필수는 아니다. |
| [NaVILA](https://navila-bot.github.io/static/navila_paper.pdf), RSS 2025, 다리 로봇 이동 | 느린 VLM: RGB·언어 → “전진 75 cm” 같은 중간 명령. 빠른 이동 정책: 중간 명령을 속도·관절 목표로 실현하며 장애물을 회피한다. | 논문은 두 시간 척도를 명시하지만 VLM·저수준 정책의 실행 Hz는 명시하지 않는다. Go2 LiDAR의 **15 Hz는 센서 주기**이고, 전진 **0.5 m/s는 명령 속도 예시**다. | 계층 분리는 적합하나 저수준 정책은 LiDAR 높이지도와 관절·자세의 proprioception을 사용한다. UGRP 학생 허용 입력으로 원 구현을 그대로 이식할 수 없다. |
| [ACT](https://arxiv.org/pdf/2304.13705), 2023, 양팔 조작 | RGB **+ 측정 관절 위치** → 여러 목표 관절 위치를 한 chunk로 출력. 단순 chunk는 k step 실행 후 관측; temporal ensembling은 매 step 새 chunk를 겹쳐 평균한다. | k-step 방식은 chunk 중간에 새 정책 결정을 반영하지 않는다. 매 step ensembling 방식은 새 관측을 반영한다. 논문의 5 vs 50 Hz는 사람의 **원격 조작** 비교이며 UGRP 권장 관측 주기가 아니다. | 중첩·점진 보정 개념은 유용하나 원 정책은 측정 관절을 사용하므로 현 제어 입력 규칙에 그대로 적용할 수 없다. |
| [Diffusion Policy](https://arxiv.org/pdf/2303.04137), 2023/2025, 조작 | 최근 `T_o` 관측 → `T_p` 미래 행동 예측 → 앞의 `T_a`만 실행 → 다시 관측·계획하는 receding horizon. | 실행한 `T_a` 중에는 정책을 다시 계산하지 않는다. 실제 로봇 표에서는 작업별 `T_a=6` 또는 `8` **step**이며 Hz나 초가 아니다. 예상 밖 변화에 대한 정책 반응은 다음 재계획 시점부터다. | 기존 RGB 제어에 짧은 명령 묶음을 시험할 근거는 되지만, 실제 논문의 position-action 모델·학습 결과를 UGRP mecanum 운반 성공으로 전이할 수 없다. |
| [Real-Time Chunking (RTC)](https://arxiv.org/pdf/2506.07339), NeurIPS 2025, 조작 | 현 chunk가 실행되는 동안 새 관측에서 **다음 chunk를 비동기 추론**한다. 이미 실행될 앞부분을 고정하고 남은 동작을 일관되게 생성한다. | 제어기는 매 `Δt`마다 관측을 받고 현재 chunk의 다음 동작을 사용한다. 추론 완료 후 새 chunk로 교체한다. 논문의 50 Hz는 예시 π₀ 제어 목표이며 RTC 일반 권장 주기가 아니다. 긴 지연에서도 새 관측 반영은 추론 완료 뒤다. | “계속 움직이며 다음 판단”이라는 실행 구조는 유용. 원문의 inpainting은 flow/diffusion 정책용이므로 현재 규칙 기반 `PairNavigator`에 그대로 복사할 알고리즘은 아니다. |

**중간 관측/취소의 정확한 의미:** ACT의 기본 k-step 방식과 Diffusion Policy의 `T_a`-step 실행은 중간에 정책을 다시 계산하지 않는다. 이는 하드웨어 비상 중지가 불가능하다는 주장이 아니다. RTC는 중간에 관측과 추론을 이어가지만 새 판단이 도착하기까지 기존 동작을 수행한다. ViNT의 PID나 영상 servo는 저수준 제어 오차를 실행 도중 수정할 수 있다. 각 논문의 별도 비상 정지 API·취소 보장 시간은 확인한 본문에 명시되지 않았으므로 현재 안전 gate를 보존한다.

## 현재 코드에 적용할 우선순위

1. **이번 open 경로를 먼저 계측한다.** `BoundPairSkill.carry()`의 `ImageRoute.observe()` 픽셀 waypoint, `PairCarryPolicy.step()`의 `CRUISE/SETTLE/ALIGN/REJOIN/CONFIRM`, 모터 명령 0 비율과 실제 RGB 변위를 각각 기록한다. 동시에 0.2 SIM초당 `capture`, RGB 인식, physics, viewer, wall-time을 분리한다. 이 측정으로 물리 계산 지연인지, 관측 지연인지, 시각 안전 gate/낮은 속도 명령인지 구별한다. 원칙은 ViNT/NaVILA의 목표·추종 분리와 IBVS의 영상 오차 피드백이다.
2. **짧은 지평을 open 경로의 실험 조건으로 검토한다.** `ImageRoute`가 정적 지도 waypoint와 현재 화물 RGB를 이미 가지므로, 넓은 직선에서는 소수의 가까운 상대 waypoint/속도 지평을 연속 추종하고 이후 새 RGB로 오차를 보정할 수 있다. 도착 경계·결합 신뢰도 저하·skew 위험에서는 지평을 줄이고 기존 동기/hold gate를 유지한다. 이는 ViNT/visual servo/DP 구조를 현 제어기에 맞춘 **설계 제안**이다. 논문의 4 Hz·5 waypoint·8 step을 복사하거나 실험 없이 명령 지속시간만 늘리면 된다는 뜻은 아니다. 장애물 회전 경로에 같은 제안을 확장하려면 `carry_with_rotation()`의 `.2`, `PairNavigator.decide()`의 `.2` 기반 참조·span-rate, `rigid_pair_commands()`의 duration/clipping과 swept-footprint 검사를 함께 다뤄야 한다.
3. **느린 추론이 실제로 동작 사이 빈 시간을 만든다면 겹친다.** LLM 역할 합의는 이미 고수준에서 병렬이고, RGB 운반은 그 이후다. open 경로의 `BoundPairSkill.carry()`/`ImageRoute.observe()`가 벽시계 stall의 원인인지 확인한 뒤에만 현재 명령 실행 중 다음 관측 처리·다음 속도 계산을 파이프라인화한다. MuJoCo/카메라 상태의 동시 접근·프레임 시간·명령 유효기간·취소 시점을 명시적으로 관리해야 한다. RTC의 시간 중첩을 참고하되 diffusion inpainting이나 새 학습은 지금 필요하지 않다. **물리 fine-step이나 렌더가 계산량 대부분이면 이 중첩만으로 실시간보다 빨라지지 않는다.**

이 문서는 논문 원문과 현 소스의 정적 읽기 조사다. 새로운 실행·학습·물리 성공 검증은 수행하지 않았다. 정확한 출처와 절/페이지별 검증 사실은 [출처별 검증 기록](continuous_visual_control_sources_20260923.json)에 있다.

## 원문 링크

- Chaumette & Hutchinson (2006), *Visual Servo Control I* — https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf
- Shah et al. (2023), *ViNT* — https://proceedings.mlr.press/v229/shah23a/shah23a.pdf
- Sridhar et al. (2024), *NoMaD* — https://arxiv.org/pdf/2310.07896
- Cheng et al. (2025), *NaVILA* — https://navila-bot.github.io/static/navila_paper.pdf
- Zhao et al. (2023), *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware* — https://arxiv.org/pdf/2304.13705
- Chi et al. (2023/2025), *Diffusion Policy* — https://arxiv.org/pdf/2303.04137
- Black et al. (2025), *Real-Time Execution of Action Chunking Flow Policies* — https://arxiv.org/pdf/2506.07339
