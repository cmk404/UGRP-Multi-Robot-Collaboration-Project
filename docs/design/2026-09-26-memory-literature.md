# 2026-09-26 로봇 기억 문헌 조사: 자기 위치, 물체 위치, 다중 로봇 공유 (Kiro)

- 작성: Kiro(`kiro/memory-literature`, `kiro/`는 Kiro 작업 표시). 2026-09-26. 첫 세션(계정 1)이 문헌 수집 중 멈췄고, 계정 2에서 이어서 썼다.
- 상태: 문헌 조사와 설계 권고다. 코드·시뮬레이션·모델 호출은 없다. 권고는 검증된 결과가 아니다.
- 읽는 쪽: 기억 작업 `kiro/zone-owncam-memory`(PR #211, `harness/owncam_memory.py` v2).
- 따른 결정: 2026-09-25/26 사용자 결정(4조건, 순환 지휘자, 입력 경계, 자기 카메라 실행, `cargo_noslip_v1`, 짝 상태 채널 전 조건, 재사용 우선). Codex 통합 설계의 `r1` 고정 지휘자·별도 commander·교사 실행기 권고는 이 결정으로 대체된 것으로 읽었다.
- 질문: 발표된 로봇 시스템은 (a) 자기 자세, (b) 물체(상자) 위치를 어떻게 기억하나? (c) 다중 로봇·LLM 에이전트 시스템은 본 것을 어떻게 저장하고 공유하나? 우리 설계에는 무엇을 쓰나?

**확인 방법.** 논문은 arXiv 초록 페이지 메타데이터, DOI(Crossref·OpenAlex), 본문 PDF 텍스트로 확인했다. 표의 확인 수준 표시는 다음과 같다.

- `[본문]`: 해당 서술을 본문에서 찾았다.
- `[초록]`: 초록에서만 확인했다.
- `[서지]`: 제목·저자·연도만 확인했다. 내용 서술은 제목 수준으로 한정했다.

저장소는 GitHub API로 라이선스와 기본 브랜치 HEAD 커밋을, 패키지는 PyPI로 버전·휠을 2026-09-26에 확인했다. 확인하지 못한 것은 "미확인"으로 적었다.

## 0. 결론

1. **자기 자세: 우리 문제는 SLAM이 아니라 "주어진 지도에서의 위치 추정"이다.**
   - 입자 필터 MCL과 sensor resetting이 딱 맞는다[P1][P3]. 현재 `harness/owncam_localizer.py`가 이 구조다.
   - 루프 폐쇄·자세 그래프·키프레임 지도는 필요 없다. 지도가 이미 있기 때문이다.
   - 조사한 의미 지도·LLM 기억 시스템은 모두 자세를 다른 데서 받는다: 시뮬레이터 좌표(CoELA), 오라클(RoCo), 사전 지도 위 AMCL(ReMEmbR), 안정된 SLAM(DynaMem), 자세가 붙은 RGB-D(ConceptGraphs 등). **우리 조건(단안 어안, 깊이 없음, 명령 기반 주행 기록)의 자세 문제를 풀어 주는 시스템은 없다.** 물체 기억의 정확도는 자세 오차를 물려받으므로, 트랙에 관측 당시 자세 σ를 남겨야 한다.
2. **물체 기억은 세 계열로 나뉜다.**
   - (i) SLAM 상태의 표식: EKF-SLAM, FastSLAM, QuadricSLAM, CubeSLAM.
   - (ii) 물체별 추적기: 칼만 필터 + 대응 + 존재 확률. SORT, persistence filter.
   - (iii) 밀집 의미·특징 지도와 3D 장면 그래프(VLMaps, ConceptGraphs, HOV-SG, Hydra, Khronos, DynaMem), 그리고 텍스트 일화 기억(ReMEmbR, Generative Agents).
   - 우리에게 맞는 것은 **(ii)에 정적 지도 개체(구역·픽업 칸·문)를 이름표로 붙인 구조**다. 사실상 지도 파일로 만든 "가벼운 장면 그래프"다. PR #211 memory_v2가 이미 (ii)다.
3. **옮겨진 물체: "부재 증거"와 "증거 부재"를 구분하는 것이 핵심이다.**
   - Khronos[P29]는 광선 기하 검증으로, DynaMem[P37]은 시야 절두체 광선 투사로 이를 구분한다. Wong 등[P18]은 계량 격자와 물체 모델을 따로 두고 "안 보이는 것도 정보"로 쓴다.
   - 공통 규칙: 보였어야 할 곳에서만 미검출을 부재 증거로 센다. 시야 밖·가림·먼 거리는 세지 않는다.
   - 시간이 지나면 존재 믿음이 줄어든다. persistence filter[P14]가 이를 재귀 베이즈로 푼다.
   - memory_v2에는 시야·거리 조건이 이미 있다. **연속 3회 미검출 카운터(`ABSENT_MISSES`)를 존재 확률로 바꾸기를 권한다**(4.2절).
4. **다중 로봇 SLAM은 비언어 자료를 공유한다.** 장소 인식 벡터, 특징점·기술자, 자세 그래프, 부분 지도다. Kimera-Multi는 데이터셋당 수십 MB를 주고받는다[P42]. 우리 주 조건에서는 금지된 채널이며 참고 조건 R에서만 쓸 수 있다.
5. **LLM 다중 에이전트 시스템의 기억과 메시지.**
   - 기억: 템플릿으로 만든 텍스트 상태(의미 지도·과업 진행·자기·타인 상태)와 행동·대화 이력이다. 메시지: 이 텍스트를 LLM이 받아 생성한 자유 문장이다(CoELA[P48], RoCo[P49]).
   - 일부는 공동 텍스트 기억(LLaMAR[P52])이나 합친 지도(Co-NavGPT[P47])를 쓴다. 우리에게는 둘 다 비언어 공유 채널이다.
   - 보고된 효과:
     - CoELA: AI끼리는 통신을 꺼도 성능이 크게 떨어지지 않았다. 통신이 시간을 쓰기 때문이라고 설명한다.
     - Guo 등: 지정 리더가 통신 비용을 거의 늘리지 않고(최대 3%) 효율을 최대 30% 올렸다[P51].
     - Li 등: 명시적 믿음 상태가 완료 라운드를 28.3 → 12.3으로 줄였다[P53].
     - RoCo: 거짓 "완료" 주장이 동료에게 전파됐다.
   - **메시지에 근거(관측 시각·출처·확신)를 체계적으로 붙이는 규약은 찾지 못했다.**
6. **권고의 뼈대**(4절).
   - 수치 기억이 원본이다. 결정적 템플릿이 관련 항목 상위 K개를 모든 조건에 똑같이 프롬프트로 렌더링한다.
   - 메시지는 LLM이 쓴다. 담는 요소는 다섯 가지다: 무엇, 어디(지도 개체 ID), 상태, 언제(SIM 초), 확신.
   - 확신은 LLM 자기평가가 아니라 기억 값에서 규칙으로 계산한다. LLM은 말로 확신을 밝힐 때 과신하는 경향이 있다[P64].
   - 들은 주장은 따로 저장하고, 자기 트랙의 칼만 측정으로 합치지 않는다.
7. **결정이 필요한 틈:** 정형 스키마(`STRUCT_FIELDS`)에는 출처(누가 봤나)와 관측 참조 필드가 없다. 자유 한국어가 출처를 말하면 조건 ②와 ④의 정보량이 달라진다(4.4절 5항).

## 1. 우리 조건과 판단 기준

| 항목 | 우리 조건 | 판단에 쓰는 질문 |
|---|---|---|
| 센서 | 팔 끝 어안 RGB 1대(640×480), 깊이 없음. 발행 명령으로 추정한 메카넘 주행 기록 | 깊이·스테레오·LiDAR·IMU가 필요한가 |
| 지도 | 정적 2D 평면도(벽·문·통로·구역 A/B/C·픽업 칸), 버전·해시. 태그 제거 중. 문·문기둥·모서리는 비전으로 인식(PR #227) | 지도를 새로 만드는가, 주어진 지도를 쓰는가 |
| 계산 | CPU만(Mac M3), MuJoCo, 로봇 3대, SIM 시간 비용 | GPU·ROS·대형 모델이 필요한가 |
| 물체 | 상자 종류가 주문서로 정해진 닫힌 집합. 로봇이 옮기면 위치가 바뀜 | 개방 어휘가 필요한가, 이동을 다루는가 |
| 공유 | 기억은 비공개. 조건 채널(없음·자유 한국어·지휘자·정형)로만 공유. 짝 상태 채널은 과업 내용이 없는 공통 설비 | 비언어 공유 채널을 전제하는가 |
| 공용 환경 | `.venv-sim-worker-mac`: Python 3.12.13, numpy 2.5.2, OpenCV 5.0.0. scipy·filterpy·gtsam·networkx·matplotlib 없음(2026-09-26 확인) | 새 의존성이 필요한가 |

## 2. 조사

표의 `[P#]`는 참고 자료의 논문 번호다.

### 2.1 고전 확률 로보틱스: 자세와 표식

| 작업 | 자세 기억 | 물체·지도 기억 | 대응 | 불확실성 | 변화·신선도 | 계산·코드 | 우리 적합 |
|---|---|---|---|---|---|---|---|
| MCL [P1] `[초록]` | 표본(입자) 집합으로 밀도 표현 | 주어진 지도 | 측정 모델에 내재 | 다봉 분포 가능 | 정적 가정 | PythonRobotics `Localization/particle_filter` (MIT) | **높음.** `owncam_localizer.py`의 원형 |
| 동적 환경 Markov 위치 추정 [P2] `[본문]` | 격자 Markov 믿음 | 고정 지도 | — | 격자 믿음, 엔트로피 | entropy filter·distance filter로 지도에 없는 물체(센서를 가리는 사람 무리)의 측정을 거름. 기억하지 않음 | — | 중간. 동료 로봇·짐이 가린 측정을 거르는 발상 |
| Sensor resetting [P3] `[초록]`, 모호한 표식 [P4] `[서지]` | MCL이 길을 잃으면 센서 기반으로 재표본 | — | — | — | 모델 오류·미모델 이동에 강건 | 제한된 계산에서 실시간(RoboCup 다리 로봇) | **높음.** `owncam_localizer._reset_from`(main l254)이 같은 방식 |
| EKF-SLAM [P5] `[서지]`, [P6] `[본문]` | 자세 + 모든 표식을 한 상태 벡터로 | 점 표식 | 최근접·마할라노비스 | 결합 가우시안. 공분산 원소 O(K²)[P6] | 없음 | PythonRobotics `SLAM/EKFSLAM` (MIT) | 부분. 대응 규칙만 PR #211이 이미 적응 |
| FastSLAM [P6] `[본문]` | 입자 = 경로 | 입자마다 표식 K개의 작은 EKF | 입자마다 최대우도 | 입자 + 표식 EKF | 없음 | O(M log K). PythonRobotics `FastSLAM1/2` | 낮음. 지도가 주어짐. 다만 "입자마다 물체 트랙" 구조는 자세·물체 오차 상관을 정확히 다룬다(우리는 `floor_var`로 근사) |
| GraphSLAM [P7] `[초록]`, 튜토리얼 [P8] `[서지]` | 자세 그래프(오프라인), 변수 소거 뒤 최적화 | 표식 노드. 10⁸개 이상 특징 | 탐욕적 대응 | 정보 행렬 | 없음 | PythonRobotics `GraphBasedSLAM` | 낮음 |
| iSAM2 [P9] `[초록]`, 인자 그래프 [P10] `[서지]` | Bayes tree 기반 증분 평활 | 표식 변수 | 외부 | 주변 공분산 | 없음 | gtsam 4.3.0 (BSD). macOS universal2 cp312 휠, numpy만 필요(PyPI) | 선택. 자세·상자 합동 평활이 필요해질 때만 |
| JCBB [P11] `[초록]` | — | — | 결합 호환성 검정. 게이트 최근접은 예측 오차 상관을 무시해 오대응을 받아들인다 | — | — | 조합 탐색 | 낮음. 한 프레임에 상자 검출이 1–3개다 |
| Covariance Intersection [P12] `[초록]` | — | — | — | 교차 상관을 모를 때도 일관된 결합 | — | — | **개념.** 들은 주장을 합치면 이중 계산 위험이 있다(4.5절) |
| SLAM 조사 [P13] `[서지]` | — | — | — | — | — | — | 배경 |

### 2.2 변하는 환경: 신선도·부재 증거·옮겨진 물체

| 작업 | 표현 | 부재·이동 처리 | 계산·코드 | 우리 적합 |
|---|---|---|---|---|
| Persistence filter [P14] `[본문]` | 특징마다 생존 시간 T의 사전분포. 검출기의 미검출 확률 P_M·오경보 확률 P_F | 관측이 없으면 존재 믿음이 사전분포를 따라 줄어든다. 검출·미검출로 재귀 베이즈 갱신 | [코드](https://github.com/david-m-rosen/Persistence-Filter) LGPL-3.0, `1c7c812`. Python판은 `scipy.special.expn` 사용 | **높음.** `ABSENT_MISSES` 카운터의 원리적 대체(4.2절) |
| Dynamic maps [P15] `[초록]` | 시간 척도 5개의 표본 지도 | 오래된 기억이 척도별 속도로 흐려짐. 정지·비정지 요소를 함께 추적 | — | 중간. 격자 감쇠 `GRID_TAU_S`와 같은 발상 |
| FreMEn [P16] `[초록]` | 상태 변화의 주파수 스펙트럼 | 주기적 변화로 미래 상태 예측 | — | 낮음. 에피소드가 짧고 주기가 없다 |
| Lifelong localization [P17] `[초록]` | Rao-Blackwellized PF + HMM | 변하는 환경에서 평생 위치 추정 | — | 참고 |
| Not seeing is also believing [P18] `[초록]` | 점유 격자(계량)와 물체 기반 세계 모델을 **따로 유지**하고 필요할 때 결합 | 빈 공간 관측을 물체 믿음에 반영 | — | **높음.** memory_v2의 격자 + 트랙 분리와 같은 구조 |
| 부분 시야 의미 세계 모델 [P19] `[초록]` | 물체 기반 모델, 속성 검출 | 군집 기반 데이터 연관 | — | 참고 |
| POCD [P20] `[초록]` | 물체마다 정지성 점수 + TSDF 변화량 | 기하·의미 정보로 베이지안 갱신 | 체적 TSDF(깊이) | 낮음. 발상만 |
| SORT [P21] `[초록]` | 칼만 필터 트랙 | 헝가리안 대응. 260 Hz | [코드](https://github.com/abewley/sort) GPL-3.0 | 원리만. 영상 좌표 추적기이고, GPL은 복사하지 않는다 |

### 2.3 물체 수준 SLAM·3D 장면 그래프

| 작업 | 자세 기억 | 물체 기억 | 대응 | 변화 처리 | 센서·계산 | 코드·라이선스 | 우리 적합 |
|---|---|---|---|---|---|---|---|
| SLAM++ [P22] `[본문 일부]` | 물체 그래프 위 자세 그래프 최적화, 루프 폐쇄·재위치 | 사전 물체 DB의 반복 물체를 6DoF 노드로 | 실시간 3D 물체 인식·ICP | 옮겨진 물체 검출 | 손에 든 깊이 카메라, GPGPU | 미공개(미확인) | 낮음. 깊이·GPU 필요 |
| CubeSLAM [P23] `[본문]` | 카메라·물체·점을 함께 다중 시점 BA | 3D 직육면체 | 특징점 매칭으로 물체 대응. 2D 상자 겹침은 가림·반복 물체에 약함 | 동적 물체 궤적 최적화 | **단안** | [cube_slam](https://github.com/shichaoy/cube_slam) BSD-3, `c371989`(ROS) | 낮음. 지도가 주어져 SLAM이 불필요. 단안 척도·무늬 없는 벽 |
| QuadricSLAM [P24] `[본문]` | 인자 그래프 SLAM(주행 기록 + 검출) | 이중 이차곡면 표식. 2D 경계 상자가 직접 제약 | **대응이 풀렸다고 가정** | 없음 | 일반 원근 카메라, GTSAM | [quadricslam](https://github.com/qcr/quadricslam) BSD-3, 1.0.2(PyPI, `gtsam-quadrics`·scipy·matplotlib 필요) | 낮음 |
| Fusion++ [P25] `[서지]` | — | 물체 수준 체적 | — | — | — | — | 제목 수준 |
| Kimera [P26] `[초록]` | VIO + 강건 자세 그래프 최적화 | 3D 계량·의미 메시 | — | — | 시각-관성. **CPU 실시간** | [Kimera](https://github.com/MIT-SPARK/Kimera) BSD-2 | 낮음. IMU·스테레오 전제 |
| 3D Dynamic Scene Graphs [P27] `[초록]` | VIO | 노드 = 물체·벽·방·사람·로봇, 엣지 = 포함·인접. 방·장소 계층 | — | 움직이는 에이전트 | 시각-관성 | — | **개념.** 지도 개체 계층(방 → 구역 → 칸 → 상자) |
| Hydra [P28] `[초록]` | 3D 장면 그래프 위 루프 폐쇄·최적화 | ESDF → 장소 위상 지도 → 방 분할, 층별 계층 | 층별 계층 기술자로 루프 검출 | — | Ubuntu 24.04·ROS2 Jazzy에서 시험(README) | [Hydra](https://github.com/MIT-SPARK/Hydra) BSD-2, `2e58a35` | 낮음 |
| Khronos [P29] `[본문]` | 짧은 활성 창 추적 + 긴 시간은 인자 그래프(GTSAM, GNC) | 물체 "조각". 재방문 조각을 전역 최적화로 연관 | 기하 검증 + 강건 최적화 | **광선 기하 검증으로 부재 증거와 증거 부재를 구분(d_ray = 30 cm).** 마지막 존재와 첫 부재 사이에 사라졌다고 추정 | RGB-D + 주행 기록 + 의미 분할. i7-12700H 노트북 CPU | [Khronos](https://github.com/MIT-SPARK/Khronos) BSD-3, `63faadd` | **발상 높음, 코드 낮음.** 깊이·ROS 전제 |
| Panoptic Multi-TSDFs [P30] `[서지]` | — | 다해상도 체적 | — | 장기 변화 | — | — | 제목 수준 |
| Clio [P31] `[초록]` | — | 과업 목록에 필요한 물체·구조만 남김(정보 병목) | — | — | — | — | **개념.** 기억 요약 상위 K를 주문서 관련 항목으로 고름 |

### 2.4 LLM 로봇의 의미 지도·장면 그래프·일화 기억

| 작업 | 자세 출처 | 물체 기억 | 대응·갱신 | 신선도·이동 | 센서·계산 | 코드·라이선스 | 우리 적합 |
|---|---|---|---|---|---|---|---|
| VLMaps [P32] `[본문 일부]` | 주행 기록 접근을 가정 | 위에서 본 격자, 셀마다 LSeg 시각-언어 임베딩(H×W×C) | 3D 재구성 투영 | 없음 | RGB-D | [vlmaps](https://github.com/vlmaps/vlmaps) MIT, `58060f9` | 낮음. 깊이·학습 인코더 필요, 닫힌 상자 종류에는 과함 |
| ConceptGraphs [P33] `[본문]` | 자세가 붙은 RGB-D | 물체 노드 = 점군 + CLIP/DINO 특징 + LVLM 캡션. 관계 엣지 | 의미(특징 코사인) + 기하(최근접 비율) 유사도 합으로 탐욕 대응, 문턱 미만이면 새 물체. 특징은 관측 수 가중 평균 | 없음 | README: CUDA PyTorch, OpenAI API | [concept-graphs](https://github.com/concept-graphs/concept-graphs) MIT, `93277a0` | **대응 규칙만 참고.** 우리 유사도 = 종류 일치 + 마할라노비스 |
| HOV-SG [P34] `[초록]` | RGB-D | 층·방·물체 계층, 각 층에 개방 어휘 특징 | — | — | RGB-D | [HOV-SG](https://github.com/hovsg/HOV-SG) MIT, `d6e65a5` | 계층 개념만 |
| SayPlan [P35] `[본문]` | — | **미리 만든** 3D 장면 그래프를 JSON으로 직렬화(층·방·자산·물체 노드: 이름·종류·위치·상태·속성) | LLM이 collapse/expand로 관련 부분 그래프만 펼침 | **지도 생성 뒤 물체가 정적이라고 가정**(한계로 명시) | LLM 토큰 한도 관리 | — | **직렬화·부분 펼침 발상 높음.** 정적 지도 개체 JSON + 관련 항목만 |
| ReMEmbR [P36] `[본문]` | 사전 지도 위 AMCL(배치 로봇) | t초 구간 영상 → VILA 캡션 → 캡션 임베딩 + 위치 + 시각을 벡터 DB에 저장. LLM이 질의 함수를 부름 | 검색 | 시각을 질의에 사용 | Jetson Orin 32GB, GPT-4o. 기억 구축 때 VILA1.5-13b에 3초마다 6프레임 | [remembr](https://github.com/NVIDIA-AI-IOT/remembr) **NVIDIA 비상업 라이선스** | 낮음. 캡션마다 모델 호출(SIM 시간 비용), 라이선스, 캡션 환각 채점 어려움. "항목에 위치·시각을 붙인다"만 따름 |
| DynaMem [P37] `[본문]` | 안정된 SLAM에 의존 | 복셀마다 점·특징 벡터·**최근 관측 시각**·영상 ID | 복셀 가중 평균 | **카메라 절두체 안의 복셀이 관측과 어긋나면 광선 투사로 제거.** 최근 관측 시각 기반 탐색 가치 지도 | RGB-D, Stretch | [stretch_ai](https://github.com/hello-robot/stretch_ai) Apache-2.0 (`docs/dynamem.md`) | **발상 높음**(시야 안에서만 지움, 최근 관측 시각). 코드는 깊이 전제 |
| NLMap [P38] `[초록]` | — | 자연어로 질의 가능한 장면 표현(VLM) | LLM이 필요한 물체를 제안해 질의 | — | — | — | 참고 |
| DovSG [P39] `[초록]`, MoMa-LLM [P40] `[초록]` | RGB-D | 동적 개방 어휘 장면 그래프, 탐색하며 갱신 | — | 변화에 따라 국소 갱신(초록 수준) | RGB-D | — | 참고 |

### 2.5 다중 로봇 지도 공유

| 작업 | 각 로봇의 기억 | 무엇을 보내나 | 검증·강건성 | 구조 | 코드·라이선스 | 우리 주 조건 |
|---|---|---|---|---|---|---|
| 협력 다중 로봇 위치 추정 [P41] `[서지]` | 확률적 위치 믿음 | (초록·본문 접근 못 함) | — | — | — | 제목 수준. 로봇끼리 서로를 관측해 위치를 결합하는 계열 |
| Kimera-Multi [P42] `[본문]` | 로봇별 Kimera 궤적·메시 | 장소 인식 BoW 벡터, 기하 검증용 특징점·기술자, 분산 자세 그래프 최적화 자료. 예: Vicon Room 2에서 합계 24.4 MB(영상 전송 중앙식 1,259 MB) | 분산 GNC로 로봇 간 오루프 제거 | 완전 분산, 피어 간 | [Kimera-Multi](https://github.com/MIT-SPARK/Kimera-Multi) LICENSE 없음 | **금지 채널.** R만 |
| Swarm-SLAM [P43] `[초록]` | 로봇별 SLAM 추정(관성·LiDAR·스테레오·RGB-D 지원) | 로봇 간 루프 폐쇄 후보. 우선순위로 통신 절약 | — | 분산, ad-hoc 망 로봇 3대 | [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM) MIT(ROS 2) | 금지 채널 |
| DOOR-SLAM [P44] `[초록]` | 자세 그래프 | 원시 센서 자료 없이 로봇 간 루프 후보 | 분산 PCM으로 가짜 로봇 간 루프 제거 | 피어 간, 완전 연결 불필요 | — | 금지 채널 |
| PCM [P45] `[초록]` | — | 지도 간 루프 폐쇄 후보 | 최대 쌍별 일관 집합 | — | — | **개념.** 들은 주장끼리·자기 관측과의 일관성 검사 |
| Hydra-Multi [P46] `[초록]` | 로봇별 장면 그래프 | 증분 입력을 중앙에서 하나의 장면 그래프로 합침 | 로봇 간 루프로 노드 조정 | **중앙** | — | 금지 채널 |
| Co-NavGPT [P47] `[초록·본문 일부]` | 로봇별 점군 부분 지도 | 부분 지도를 전역 3D 지도로 합치고 VLM이 경계(frontier)를 배정 | — | **중앙** | — | 금지 채널(R과 비슷한 구조) |

### 2.6 LLM 다중 에이전트: 무엇을 기억하고 무엇을 보내나

| 작업 | 기억 | 자기 위치 출처 | 메시지 | 통신 비용·제약 | 보고된 효과·실패 | 코드·라이선스 |
|---|---|---|---|---|---|---|
| CoELA [P48] `[본문]` | 의미 기억(0.125 m 격자 의미 지도: 대상·용기·목적지·에이전트, 과업 진행, 자기·타인 상태), 일화 기억(행동·대화 이력), 절차 기억 | **시뮬레이터가 주는 위치·회전**(관측 공간) + 깊이 | 기억을 **템플릿으로 텍스트화**한 뒤 LLM이 자유 문장 생성. "정확하고 유용하고 짧게, 반복하지 말라" 지시 + 씨앗 메시지 2개 | 한 프레임 500자 이하. 통신이 시간을 씀 | 기억 모듈을 빼면 단계 수가 거의 두 배. AI끼리는 통신을 꺼도 큰 하락 없음. 남이 물체를 바꾸면 기억이 틀림 | [CoELA](https://github.com/UMass-Embodied-AGI/CoELA) LICENSE 없음 |
| RoCo [P49] `[본문]` | 라운드 이력(과거 대화·실행 행동) | 오라클 상태(한계로 명시) | 관측을 설명 함수로 자연어 프롬프트화. 자유 토론 뒤 마지막 화자가 부분 과업 계획 요약 → 검증기(파싱·제약·IK·충돌) | 라운드마다 유한 교환 | 한 에이전트의 거짓 "완료"를 다른 로봇들이 반복하고 기다림 | [robot-collab](https://github.com/MandiZhao/robot-collab) MIT |
| 중앙 vs 분산 [P50] `[본문]` | 단계 이력: 대화·상태·행동 | 사전 정의 함수가 합성 | 4가지 틀(중앙·분산·혼합 2) | 전체 이력은 토큰 예산을 빠르게 소진 | 혼합이 성공률·확장성 최고. **상태-행동 이력만(대화 제외)**이 성능·토큰의 최적 절충 | 프로젝트 페이지 |
| 조직된 팀 [P51] `[본문]` | CoELA식 모듈(기억 포함) | — | 방송 또는 수신자 선택, 침묵 가능 | 통신 비용 측정 | 무조직 팀은 중복·혼란 메시지. 지정 리더가 효율 최대 30%↑, 추가 통신 최대 3% | — |
| LLaMAR [P52] `[본문]` | **모든 에이전트의 공동 텍스트 기억 M**(관측·행동·실패 이유·담당 부분 과업) | — | 모든 에이전트 행동을 한 번에 정하는 중앙 결정 | — | Verifier LM이 영상으로 완료 확인(오라클 없음) | — |
| ToM [P53] `[본문]` | **명시적 믿음 상태**(프롬프트로 갱신·유지) | — | 자유 문장 | 입력 한도로 전체 이력 보존 불가 | GPT-4 + 믿음: 완료 12.3 라운드(없으면 28.3), 유효 행동 86.1%(71.8%). 과업 상태 환각·긴 문맥 실패 | — |
| REVECA [P54] `[본문]` | 관측을 관련도 4단계(Strong/Medium/Low/None)로 저장. 동료 정보(쥔 물체·위치·메시지·완료 계획)는 직접 관측과 대화에서 추출해 통합 | — | 자연어 | — | 궤적 기반 검증: 동료가 이미 끝냈을 가능성 예측으로 헛계획 방지 | — |
| CaPo [P55] `[초록]` | 메타 계획 | — | 토론으로 메타 계획 작성, 진행에 따라 조정 | — | 중복 단계 감소 | — |
| COMBO [P56] `[초록]` | 부분 자기 시야에서 생성 모델로 전체 세계 상태 추정 | — | — | — | — | — |
| EMOS [P57] `[초록]` | "Robot Resume": 역할 연기 대신, 에이전트가 URDF를 읽고 기구학 도구를 불러 자기 신체 능력 설명을 만듦 | — | — | — | — | — |
| SMART-LLM [P58] `[초록]` | — | — | 단일 LLM이 분해·연합·배정 | — | — | [SMART-LLM](https://github.com/SMARTlab-Purdue/SMART-LLM) LICENSE 없음 |
| Statler [P59] `[초록·본문 일부]` | 세계 상태 추정을 LLM writer가 갱신, reader가 읽음 | — | — | — | Code-as-Policies보다 좋음 | [statler](https://github.com/ripl/statler) LICENSE 없음 |
| Generative Agents [P60] `[본문]` | 경험의 자연어 기록 + 반성 | — | — | — | 검색 점수 = 최근성(마지막 검색 뒤 게임 시간당 0.995 지수 감쇠) + 중요도 + 관련도(임베딩 코사인) | [generative_agents](https://github.com/joonspk-research/generative_agents) Apache-2.0 |
| MAST [P61] `[초록]` | — | — | — | — | 실패 14양상, 3범주: 시스템 설계, 에이전트 간 불일치, 과업 검증 | — |

### 2.7 불확실성의 언어화

- Kent[P62] `[본문]`: 정보 분석 보고의 추정 어휘를 다섯 단계와 확률 범위에 고정했다.
  - "certain" 100%, "almost certain" 93%(±약 6%), "probable" 75%(±약 12%), "chances about even" 50%(±약 10%), "probably not" 30%(±약 10%), "almost certainly not" 7%(±약 5%), "impossibility" 0%.
  - 교훈: 11단계를 먼저 시도했다가 5단계로 줄였다. 단어가 전할 수 있는 정밀도에는 한계가 있다.
- Tian 등[P63] `[초록]`: RLHF LLM이 말로 밝힌 확신은 조건부 확률보다 보정이 좋았다(ECE 상대 약 50% 감소).
- Xiong 등[P64] `[초록]`: LLM은 말로 확신을 밝힐 때 과신하는 경향이 있다.
- KnowNo[P65] `[초록]`: 등각 예측으로 LLM 계획기의 불확실성을 보정하고, 모를 때 도움을 요청한다.
- 우리 설계의 함의:
  - 관측 확신은 LLM이 아니라 **기억 값에서 규칙으로** 정한다.
  - 메시지에는 **적은 수의 고정 등급**(`low`/`medium`/`high` = 낮음/보통/높음)만 쓴다. 이 등급은 이미 `harness/zone_study_protocol.py` `CONFIDENCE`(l77)에 있다.

## 3. 종합: 우리 조건 적합도

| 계열 | 대표 | 깊이·GPU·ROS 필요 | 주어진 2D 지도 활용 | 이동 물체 | 비공개 기억 + 언어 공유와의 관계 | 판단 |
|---|---|---|---|---|---|---|
| 지도 기반 입자 위치 추정 | MCL, SRL | 없음 | 핵심 | 동적 측정 거름[P2] | 자기 것만 | **채택(이미 사용)** |
| 표식 SLAM | EKF-SLAM, FastSLAM, GraphSLAM, iSAM2 | 없음(코드 있음) | 지도를 만드는 문제 | 없음 | — | 대응·갱신 수식만 |
| 물체별 추적 + 존재 확률 | KF + 게이트 대응, persistence filter | 없음 | 개체 이름표로 사용 | **부재 증거** | 항목 단위로 말하기 쉬움 | **채택·보강** |
| 물체 SLAM | SLAM++, CubeSLAM, QuadricSLAM | 깊이 또는 단안 SLAM | 거의 안 씀 | 일부 | — | 기각 |
| 3D 장면 그래프 | 3D DSG, Hydra, Khronos, Clio | 깊이·ROS | 계층 개념 | Khronos 강함 | 계층을 지도 파일로 대신 | 개념만 |
| 밀집 특징·개방 어휘 지도 | VLMaps, ConceptGraphs, HOV-SG, DynaMem | 깊이·GPU | 안 씀 | DynaMem 강함 | 특징 벡터는 말로 못 옮김 | 기각(발상만) |
| 텍스트 일화 기억 | ReMEmbR, Generative Agents | VLM 호출 | 위치만 | 시각 필드 | 텍스트라 옮기기 쉬움, 환각 위험 | 형식만(시각·위치 필드) |
| 다중 로봇 SLAM 공유 | Kimera-Multi, Swarm-SLAM, DOOR-SLAM, Hydra-Multi | 다양 | — | — | **비언어 채널** | R만 |
| LLM 다중 에이전트 | CoELA, RoCo, LLaMAR, ToM, REVECA | 시뮬레이터 좌표·오라클 | — | 약함 | 템플릿 텍스트 → LLM 메시지 | **메시지 생성 구조 채택**, 근거 필드 추가 |

## 4. 우리 설계 권고

### 4.1 무엇을 기억하나 (로봇마다 비공개)

| 층 | 내용 | 근거 문헌 | 현재 코드 |
|---|---|---|---|
| L0 지도 개체 | 구역 A/B/C, 픽업 칸, 문·문기둥·모서리 ID와 좌표. 모든 기억 항목의 이름표 | 3D DSG의 방·장소 층[P27], SayPlan JSON[P35] | 정적 지도 파일(모든 로봇 동일, 버전·해시) |
| L1 자기 자세 | PF 평균·공분산·N_eff, 마지막 표식 고정 시각·종류, 마지막 정지 둘러보기 고정. 관측마다 그때의 자세·σ 기록 | MCL·SRL[P1][P3], 항목에 위치·시각 붙이기[P36][P37] | `owncam_localizer.py`, `owncam_pose_source.PoseReport`(PR #211 l30), memory_v2 관측 로그 |
| L2 상자 트랙 | 상자마다 2D KF 평균·공분산, 종류, 상태(`tentative`/`confirmed`/`absent`/`claimed`/`held`/`placed`), 처음·마지막 관측 시각, near/far 적중, **존재 확률**, 붙은 지도 개체(`location_ref`), 근거 관측 ID | SORT식 KF + 대응[P21], persistence[P14], 조각 연관[P29] | `owncam_memory.BoxTrack`(PR #211 l411), `owncam_memory_kf.py` |
| L3 바닥 점유 | 5 cm log-odds 격자, 시간에 따라 "모름"으로 감쇠 | 격자·물체 분리[P18], 다중 시간 척도 망각[P15] | memory_v2 격자(`GRID_TAU_S` = 120 s) |
| L4 들은 주장 | 발신자, `message_id`, 도착 SIM 시각, 주장이 말한 관측 시각, 품목·개체·상태·확신, 자기 확인 결과(`unverified`/`confirmed`/`contradicted`/`unobservable`) | CoELA의 기억·타인 설명 불일치[P48], REVECA 검증[P54], RoCo 전파 실패[P49] | 아직 없음(PR #210 설계 4(b)의 `source: heard` 제안) |
| L5 LLM 믿음 | `BELIEF_KEYS`(region, last_visual_anchor, …, confidence, sources) | 명시적 믿음[P53], Statler[P59] | `zone_study_prompts_ko.py` l56 |

- L1–L4는 수치 기억이고 호스트 코드가 관리한다. L5는 LLM이 쓴다. **L5가 L1–L4를 대신하지 않는다.** 텍스트만으로 상태를 유지하면 과업 상태 환각과 긴 문맥 실패가 보고됐다[P53].
- 자세는 입자 필터로 충분하다. 루프 폐쇄·키프레임 지도는 만들지 않는다. 표식 측정원만 태그에서 PR #227 비전(벽·문기둥·모서리)으로 바뀐다.
- 자세 오차와 트랙 오차의 상관은 지금처럼 "트랙 σ ≥ 최선 관측 당시 자세 σ"(`floor_var`)로 근사한다. 정확히 다루는 방법은 FastSLAM식 입자별 트랙이나 인자 그래프 합동 평활이다[P6][P9]. 오프라인 감사에서 근사가 문제로 드러날 때만 검토한다.

### 4.2 재사용할 코드와 구체적 변경 제안

| 용도 | 재사용 대상 | 버전·라이선스 | 제안 |
|---|---|---|---|
| 자세 PF | `harness/owncam_localizer.py`(PythonRobotics `particle_filter.py` 적응, main l114) | PythonRobotics `b2020cd`, MIT | 유지. 측정원만 PR #227로 교체 |
| 상자 KF | `harness/owncam_memory_kf.py` `kf_predict`/`kf_update`(filterpy 1.4.5 적응) | filterpy 1.4.5, MIT | 유지 |
| 대응 | 같은 파일 `associate`(l125): χ²(2) 99% 게이트 9.21, 1대1, 모호 여유 2.0이면 거부 | PythonRobotics EKF-SLAM 대응 적응 | 유지. 상자 수가 적어 JCBB[P11]의 이득은 작다 |
| 시야·가림 | `ViewModel.point_in_view`(l402), `occluded`(l261) | 저장소 | 부재 증거의 조건으로 계속 사용 |
| **존재 확률** | persistence filter[P14]의 모형. **코드는 복사하지 않는다**(LGPL-3.0, scipy 필요) | 논문 수식 | `ABSENT_MISSES = 3` 카운터 대신 트랙마다 존재 확률 p를 둔다. 생존 사전분포를 지수 분포(위험률 λ)로 두면, 이 모형은 "있음 → 사라짐" 2상태 은닉 마르코프 모형의 전향 필터와 같아진다(**우리 유도, 논문의 일반 사전분포는 쓰지 않음**). 예측: p ← p·e^(−λΔt). 갱신: 검출이면 p(1−P_M)/[p(1−P_M)+(1−p)P_F], 시야 안 미검출이면 p·P_M/[p·P_M+(1−p)(1−P_F)]. P_M은 거리·가림·자세(`near`/`far_coarse`)별로 dev 기록에서 맞추고 고정한다. `absent` 판정 문턱은 사전 등록한다 |
| 확인 규칙 | PR #193 `JudgmentTracker`(연속 2회 일치, `unknown`은 `no`가 되지 않음) | 저장소 | 설계 참고(memory_v2 `CONFIRM_NEAR_HITS = 2`가 이미 같은 규칙) |
| 모호하면 멈춤 | `harness/multi_object_tracking.py` `CargoTracker`(l60) | 저장소 | 원칙만: 한 검출이 두 트랙에 걸리면 어느 쪽도 갱신하지 않는다 |
| 기억 필드 | `harness/coela_modules.py` `Memory`(l30): 화물별 `last_seen_sim_time`(l48), 동료 의도 TTL 12 s(l64·l136) | 저장소 | L4 들은 주장의 만료 필드 형식으로 따름 |
| 메시지 스키마 | `zone_study_protocol.py` `STRUCT_FIELDS`(l73)·`STRUCT_STATES`(l76)·`CONFIDENCE`(l77)·`INBOX_FIELDS`(l88) | 저장소 | 그대로 사용(4.4절) |
| 합동 평활(선택) | gtsam `Pose2`·`Point2`·방위-거리 인자 | 4.3.0, BSD, cp312 macOS universal2 휠, 의존성 numpy(PyPI 메타데이터에 pytest도 있음) | **보류.** 공용 환경에 새 패키지를 넣어야 하고, 지도가 주어진 상황에서 이득이 불확실하다 |

검토 후 기각한 공개 코드:

- Stone Soup 1.9.1(MIT): scipy·matplotlib·plotly 등 의존성이 무겁다.
- SORT(GPL-3.0): 라이선스 문제가 있고, 영상 좌표 추적기다.
- Persistence-Filter 코드(LGPL-3.0, scipy): 수식만 쓴다.
- quadricslam 1.0.2: `gtsam-quadrics`·scipy·matplotlib 의존성이 있고, 대응이 풀렸다고 가정한다.
- Kimera·Hydra·Khronos·Swarm-SLAM: ROS와 깊이·IMU를 전제한다.
- VLMaps·ConceptGraphs·HOV-SG·DynaMem(stretch_ai): RGB-D·GPU를 전제한다.
- ReMEmbR: 비상업 라이선스이고, 호출마다 VLM이 필요하다.
- CoELA·Statler·SMART-LLM: LICENSE 파일이 없어 복사하지 않는다.

### 4.3 피할 것

1. **주 조건의 비언어 공유.** 다음은 주 조건에서 쓰지 않고, 참고 조건 R에서만 허용한다.
   - 공유 지도·장면 그래프 서버[P46][P47]
   - 공동 텍스트 기억[P52]
   - 기술자·자세 그래프 교환[P42][P43][P44]
   - 다른 로봇 영상
2. **밀집 특징 지도·개방 어휘.** 깊이·GPU가 필요하다. 상자 종류는 주문서로 닫혀 있다. 특징 벡터는 말로 옮길 수 없어 조건 간 정보량 비교를 흐린다.
3. **캡션 + 벡터 DB 일화 기억.** 관측마다 모델을 호출하므로 SIM 시간 비용이 든다. 캡션 오류는 채점하기 어렵다. 대신 구조화된 관측 로그와 결정적 템플릿을 쓴다.
4. **LLM을 유일한 기억 저장소로 쓰는 설계.** 환각·긴 문맥 실패[P53]와 거짓 완료 전파[P49]가 보고됐다.
5. **들은 주장을 자기 칼만 트랙의 측정으로 합치기.**
   - 중계된 같은 관측이 두 번 들어오면 이중 계산이 된다. 교차 상관을 모르는 결합 문제다[P12].
   - 발신자의 자세 오차도 모른다.
6. **LLM이 스스로 매긴 확신.** 과신 경향이 있다[P64].
7. **"안 보임 = 없음".** 시야 밖·가림·먼 거리의 미검출은 부재 증거가 아니다[P18][P29][P37].
8. **모호할 때 임의 선택.** 두 트랙 사이에서 고르지 말고 갱신을 거부한다.

### 4.4 기억을 한국어 메시지로 옮기는 규칙 (비언어 부채널 없이)

1. **호스트는 요약까지만 만든다. 번역하지 않는다.**
   - 호스트 코드는 기억(L1–L4)을 결정적 템플릿으로 프롬프트 요약에 렌더링한다. 메시지는 LLM이 쓴다.
   - 기억 객체·좌표 배열·트랙 ID 목록을 메시지에 붙이지 않는다.
   - inbox는 `INBOX_FIELDS` 허용 목록만 담는다.
   - 짝 상태 채널(aligning/ready/lift/carry/put_down/abort)은 기억 요약과 메시지에 섞지 않는다.
2. **요약은 모든 조건에서 같다.** 같은 템플릿·같은 K·같은 정렬 규칙을 쓴다.
   - 정렬: 주문서 품목·목적 구역·경로의 문 관련도 → 확신 → 최근성. 관련도 선별은 Clio[P31]·REVECA[P54], 최근성은 Generative Agents[P60]의 발상이다.
   - 토큰은 Codex 설계의 "수신 메시지·개인 기억 2,000 tokens" 안에서 쓴다.
   - `no_comm`도 같은 요약을 받는다. 들은 주장 칸만 비어 있다.
3. **개체 우선.** 좌표 대신 지도 개체 ID(구역 글자, 픽업 칸, 문)로 말한다. ID·구역 글자·JSON 키는 원문 그대로 둔다. 좌표 오차가 커도 "`bay_2`에 있음"은 개체 수준에서 검증할 수 있다.
4. **다섯 요소를 고정 어휘로 담는다.** 무엇, 어디, 상태, 언제(정수 SIM 초), 확신이다.
   - 상태·확신 단어는 아래 표로 고정한다. Kent의 교훈대로 등급을 적게 두고 대응을 고정한다[P62].
   - "언제"는 나이가 아니라 **관측 시각(절대 SIM 초)**으로 말한다. 정형 필드 `observed_at_sim_s`와 같고, 받는 쪽이 자기 시각으로 나이를 계산한다.
5. **"같은 정보" 원칙과 출처 문제.**
   - 정형 스키마에는 출처(누가 봤나)와 관측 ID 필드가 없다. 따라서 자유 한국어에만 "r2가 봤다"·"obs-0042"를 요구하면 ②와 ④의 정보량이 달라진다.
   - 선택지는 두 가지다.
     - (a) 둘 다 출처를 말하지 않는다. 발신자 자신이 본 것만 말하고, 중계는 자기 주장으로 말한다.
     - (b) 새 프로토콜 버전에서 `STRUCT_FIELDS`에 출처 필드를 추가하고 한국어에도 같은 요소를 요구한다.
   - **(a)를 기본으로 권한다.** 관측 참조는 (발신자, `observed_at_sim_s`) 쌍으로 대신한다. PR #210 설계 4(b)의 "본문에 관측 ID" 제안과 다르므로 기억 작업에서 결정한다.
6. **확신은 규칙으로 계산하고 LLM은 옮기기만 한다.** 규칙 예시(값은 dev에서 정하고 파일럿 전에 고정):
   - `high`: `confirmed`, near 적중 2회 이상, σ ≤ 0.05 m, 나이 ≤ `STALE_AGE_S`(60 s), 존재 확률 ≥ 0.9.
   - `medium`: `confirmed`이지만 오래됐거나 σ ≤ 0.15 m. 또는 far 적중만 2회 이상.
   - `low`: `tentative`, far 1회, 확인하지 않은 들은 주장.
7. **자기 위치도 개체 수준으로 말한다.** 예: "`door_narrow` 앞, A구역 쪽". `BELIEF_KEYS`의 `region`·`last_visual_anchor`와 정형 `zone`·`location_ref`에 대응하며, 좌표는 말하지 않는다.

| 기억 값 | 한국어 고정 표현(예) | 정형 필드 |
|---|---|---|
| 트랙 종류 | `green_box`(원문) | `item` |
| 붙은 지도 개체 | `bay_2`, `A` | `location_ref`, `zone` |
| `tentative` | 있는 듯함(미확인) | `state: suspected` |
| `confirmed` | 있음 | `state: present` |
| `absent`(존재 확률 < 문턱) | 없음 | `state: absent` |
| `held` / `placed` | 들고 있음 / 놓음 | `state: held` / `placed` |
| 문 통과 판단 | `door_narrow` 막힘 / 지나갈 수 있음 | `passage` + `blocked` / `clear` |
| 모름 | 모름 | `state: unknown` |
| 마지막 관측 시각 | 131초에 봄 | `observed_at_sim_s: 131` |
| 규칙 확신 | 확신 높음 / 보통 / 낮음 | `confidence: high` / `medium` / `low` |

예시(같은 내용):

```text
[자유 한국어] green_box가 bay_2에 있음. 131초에 봄, 확신 높음. door_narrow는 128초에 막혀 있었음, 확신 보통.
```

```json
[{"act": "inform", "item": "green_box", "location_ref": "bay_2", "state": "present", "observed_at_sim_s": 131, "confidence": "high"},
 {"act": "inform", "passage": "door_narrow", "state": "blocked", "observed_at_sim_s": 128, "confidence": "medium"}]
```

한국어 메시지 하나에 주장 두 개가 들어갈 수 있다. 조건 비교는 메시지 수가 아니라 **주장 수**로 센다. 한국어 주장은 평가 전용 파서로 추출한다.

### 4.5 들은 주장의 처리

- **저장.** L4에 따로 둔다. 나이는 도착 시각이 아니라 주장이 말한 관측 시각으로 잰다.
- **쓰임.** 들은 주장은 어디를 볼지와 무엇을 맡을지를 바꿀 수 있다. 다음은 할 수 없다.
  - 트랙을 `confirmed`로 만드는 것.
  - 파지·배치 직전의 자기 카메라 확인을 건너뛰게 하는 것.
- **자기 확인.** 해당 개체를 자기 카메라로 보면 `confirmed`/`contradicted`/`unobservable`을 새 관측으로 남기고 주장에 연결한다. RoCo의 검증기와 LLaMAR의 Verifier, REVECA의 검증 모듈이 같은 역할을 한다[P49][P52][P54].
- **반박.** 통신 조건에서는 반박을 `correct` 행동(정형) 또는 정정 문장(한국어)으로 보낼 수 있다. 보낼지는 LLM이 정한다.
- **중계.** 지휘자 조건에서 지휘자가 추종자 보고를 다른 추종자에게 전하면, 그것은 지휘자 자신의 주장이다. 원래 관측 시각은 유지하고, 중계 과정에서 확신을 올리지 않는다. 같은 관측의 반복 수신은 증거로 누적하지 않는다[P12].

### 4.6 기억 작업에 제안하는 검증 (사전 등록용 초안)

| 검사 | 방법 | 기준 예 |
|---|---|---|
| 부재 판정 | 기록 프레임을 오프라인 재생하고 평가 전용 GT로 채점. `ABSENT_MISSES` 카운터와 존재 확률을 비교 | 거짓 `absent` 0, 실제 제거 후 `absent`까지 걸린 SIM 시간 |
| 존재 확률 보정 | dev 기록의 p 구간별 실제 존재 비율 | 구간별 오차 사전 등록 |
| 결정성 | 같은 기억 → 같은 요약 바이트(조건 4개) | 해시 일치 |
| 채널 격리 | `no_comm`에서 L4 항목 0개, inbox에 기억 객체 없음 | 테스트 통과 |
| ②·④ 동등성 | 평가 전용 파서로 한국어 주장의 5요소를 추출해 정형과 비교 | 표현 못 한 요소는 따로 보고 |

### 4.7 열린 문제

- 출처 필드(4.4절 5항): (a)와 (b) 중 하나를 사용자·기억 작업이 결정해야 한다.
- 파라미터 λ·P_M·P_F와 `absent` 문턱: dev에서만 맞추고 test 전에 고정한다.
- 동료 로봇을 자기 카메라로 본 관측은 허용 입력이다. [P41] 계열처럼 이를 자기 PF 측정으로 쓸지는 따로 검토한다. 동료의 믿음은 대화로만 얻는다.
- 조사한 LLM 시스템은 모두 좋은 자세를 전제한다. 자세가 나쁠 때 대화의 가치가 달라지는지는 문헌 근거가 없다. 우리 실험이 직접 답해야 한다.

## 참고 자료

### 논문

확인 수준: `[본문]` 본문에서 해당 서술 확인, `[초록]` 초록만, `[서지]` 서지만.

- [P1] F. Dellaert, D. Fox, W. Burgard, S. Thrun, "Monte Carlo Localization for Mobile Robots," ICRA 1999. https://doi.org/10.1109/ROBOT.1999.772544 `[초록]`
- [P2] D. Fox, W. Burgard, S. Thrun, "Markov Localization for Mobile Robots in Dynamic Environments," JAIR 11, 391–427, 1999. https://arxiv.org/abs/1106.0222 `[본문]`
- [P3] S. Lenser, M. Veloso, "Sensor Resetting Localization for Poorly Modelled Mobile Robots," ICRA 2000. https://doi.org/10.1109/ROBOT.2000.844766 `[초록]`
- [P4] B. Coltin, M. Veloso, "Multi-observation sensor resetting localization with ambiguous landmarks," Autonomous Robots, 2013. https://doi.org/10.1007/s10514-013-9347-y `[서지]`
- [P5] H. Durrant-Whyte, T. Bailey, "Simultaneous Localization and Mapping: Part I," IEEE Robotics & Automation Magazine, 2006. https://doi.org/10.1109/MRA.2006.1638022 `[서지]`
- [P6] M. Montemerlo, S. Thrun, D. Koller, B. Wegbreit, "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem," AAAI 2002. https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf `[본문]`
- [P7] S. Thrun, M. Montemerlo, "The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures," IJRR, 2006. https://doi.org/10.1177/0278364906065387 `[초록]`
- [P8] G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard, "A Tutorial on Graph-Based SLAM," IEEE ITS Magazine, 2010. https://doi.org/10.1109/MITS.2010.939925 `[서지]`
- [P9] M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. J. Leonard, F. Dellaert, "iSAM2: Incremental smoothing and mapping using the Bayes tree," IJRR (Crossref 발행 연도 2011). https://doi.org/10.1177/0278364911430419 `[초록]`
- [P10] F. Dellaert, M. Kaess, "Factor Graphs for Robot Perception," Foundations and Trends in Robotics, 2017. https://doi.org/10.1561/2300000043 `[서지]`
- [P11] J. Neira, J. D. Tardós, "Data association in stochastic mapping using the joint compatibility test," IEEE Trans. Robotics and Automation, 2001. https://doi.org/10.1109/70.976019 `[초록]`
- [P12] S. J. Julier, J. K. Uhlmann, "A non-divergent estimation algorithm in the presence of unknown correlations," ACC 1997. https://doi.org/10.1109/ACC.1997.609105 `[초록]`
- [P13] C. Cadena, L. Carlone, H. Carrillo 외, "Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age," 2016. https://arxiv.org/abs/1606.05830 `[서지]`
- [P14] D. M. Rosen, J. Mason, J. J. Leonard, "Towards Lifelong Feature-Based Mapping in Semi-Static Environments," ICRA 2016. https://doi.org/10.1109/ICRA.2016.7487237 `[본문]`
- [P15] P. Biber, T. Duckett, "Dynamic Maps for Long-Term Operation of Mobile Service Robots," RSS 2005. https://doi.org/10.15607/RSS.2005.I.003 `[초록]`
- [P16] T. Krajník, J. P. Fentanes, J. M. Santos, T. Duckett, "FreMEn: Frequency Map Enhancement for Long-Term Mobile Robot Autonomy in Changing Environments," IEEE T-RO, 2017. https://doi.org/10.1109/TRO.2017.2665664 `[초록]`
- [P17] G. D. Tipaldi, D. Meyer-Delius, W. Burgard, "Lifelong localization in changing environments," IJRR, 2013. https://doi.org/10.1177/0278364913502830 `[초록]`
- [P18] L. L. S. Wong, T. Lozano-Pérez, L. P. Kaelbling, "Not seeing is also believing: Combining object and metric spatial information," IEEE ICRA 2014. http://hdl.handle.net/1721.1/100724 `[초록]`
- [P19] L. L. S. Wong, L. P. Kaelbling, T. Lozano-Pérez, "Data association for semantic world modeling from partial views," IJRR, 2015. https://doi.org/10.1177/0278364914559754 `[초록]`
- [P20] J. Qian, V. Chatrath, J. Yang, J. Servos, A. P. Schoellig, S. L. Waslander, "POCD: Probabilistic Object-Level Change Detection and Volumetric Mapping in Semi-Static Scenes," RSS 2022. https://arxiv.org/abs/2205.01202 `[초록]`
- [P21] A. Bewley, Z. Ge, L. Ott, F. Ramos, B. Upcroft, "Simple Online and Realtime Tracking," ICIP 2016. https://arxiv.org/abs/1602.00763 `[초록]`
- [P22] R. F. Salas-Moreno, R. A. Newcombe, H. Strasdat, P. H. J. Kelly, A. J. Davison, "SLAM++: Simultaneous Localisation and Mapping at the Level of Objects," CVPR 2013. https://doi.org/10.1109/CVPR.2013.178 (PDF: http://www.doc.ic.ac.uk/~ajd/Publications/salas-moreno_etal_cvpr2013.pdf) `[본문 일부]`
- [P23] S. Yang, S. Scherer, "CubeSLAM: Monocular 3D Object SLAM," IEEE T-RO (arXiv 2018). https://arxiv.org/abs/1806.00557 `[본문]`
- [P24] L. Nicholson, M. Milford, N. Sünderhauf, "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks in Object-oriented SLAM," RA-L (arXiv 2018). https://arxiv.org/abs/1804.04011 `[본문]`
- [P25] J. McCormac, R. Clark, M. Bloesch, A. J. Davison, S. Leutenegger, "Fusion++: Volumetric Object-Level SLAM," 2018. https://arxiv.org/abs/1808.08378 `[서지]`
- [P26] A. Rosinol, M. Abate, Y. Chang, L. Carlone, "Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping," 2019. https://arxiv.org/abs/1910.02490 `[초록]`
- [P27] A. Rosinol, A. Gupta, M. Abate, J. Shi, L. Carlone, "3D Dynamic Scene Graphs: Actionable Spatial Perception with Places, Objects, and Humans," 2020. https://arxiv.org/abs/2002.06289 `[초록]`
- [P28] N. Hughes, Y. Chang, L. Carlone, "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization," RSS 2022. https://arxiv.org/abs/2201.13360 `[초록]`
- [P29] L. Schmid, M. Abate, Y. Chang, L. Carlone, "Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments," RSS 2024. https://arxiv.org/abs/2402.13817 `[본문]`
- [P30] L. Schmid, J. Delmerico, J. Schönberger 외, "Panoptic Multi-TSDFs: a Flexible Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic Scene Consistency," ICRA 2022. https://arxiv.org/abs/2109.10165 `[서지]`
- [P31] D. Maggio, Y. Chang, N. Hughes 외, "Clio: Real-time Task-Driven Open-Set 3D Scene Graphs," 2024. https://arxiv.org/abs/2404.13696 `[초록]`
- [P32] C. Huang, O. Mees, A. Zeng, W. Burgard, "Visual Language Maps for Robot Navigation," ICRA 2023. https://arxiv.org/abs/2210.05714 `[본문 일부]`
- [P33] Q. Gu, A. Kuwajerwala, S. Morin 외, "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning," 2023. https://arxiv.org/abs/2309.16650 `[본문]`
- [P34] A. Werby, C. Huang, M. Büchner, A. Valada, W. Burgard, "Hierarchical Open-Vocabulary 3D Scene Graphs for Language-Grounded Robot Navigation," 2024. https://arxiv.org/abs/2403.17846 `[초록]`
- [P35] K. Rana, J. Haviland, S. Garg, J. Abou-Chakra, I. Reid, N. Suenderhauf, "SayPlan: Grounding Large Language Models using 3D Scene Graphs for Scalable Robot Task Planning," CoRL 2023. https://arxiv.org/abs/2307.06135 `[본문]`
- [P36] A. Anwar, J. Welsh, J. Biswas, S. Pouya, Y. Chang, "ReMEmbR: Building and Reasoning Over Long-Horizon Spatio-Temporal Memory for Robot Navigation," 2024. https://arxiv.org/abs/2409.13682 `[본문]`
- [P37] P. Liu, Z. Guo, M. Warke 외, "DynaMem: Online Dynamic Spatio-Semantic Memory for Open World Mobile Manipulation," 2024. https://arxiv.org/abs/2411.04999 `[본문]`
- [P38] B. Chen, F. Xia, B. Ichter 외, "Open-vocabulary Queryable Scene Representations for Real World Planning" (NLMap), 2022. https://arxiv.org/abs/2209.09874 `[초록]`
- [P39] Z. Yan, S. Li, Z. Wang 외, "Dynamic Open-Vocabulary 3D Scene Graphs for Long-term Language-Guided Mobile Manipulation" (DovSG), RA-L 2025. https://arxiv.org/abs/2410.11989 `[초록]`
- [P40] D. Honerkamp, M. Büchner, F. Despinoy, T. Welschehold, A. Valada, "Language-Grounded Dynamic Scene Graphs for Interactive Object Search with Mobile Manipulation" (MoMa-LLM), RA-L, 2024. https://arxiv.org/abs/2403.08605 `[초록]`
- [P41] D. Fox, W. Burgard, H. Kruppa, S. Thrun, "A Probabilistic Approach to Collaborative Multi-Robot Localization," Autonomous Robots, 2000. https://doi.org/10.1023/A:1008937911390 `[서지]`(초록·본문 접근 못 함)
- [P42] Y. Tian, Y. Chang, F. H. Arias, C. Nieto-Granda, J. P. How, L. Carlone, "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems," IEEE T-RO (arXiv 2021). https://arxiv.org/abs/2106.14386 `[본문]`
- [P43] P.-Y. Lajoie, G. Beltrame, "Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems," 2023. https://arxiv.org/abs/2301.06230 `[초록]`
- [P44] P.-Y. Lajoie, B. Ramtoula, Y. Chang, L. Carlone, G. Beltrame, "DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams," 2019. https://arxiv.org/abs/1909.12198 `[초록]`
- [P45] J. G. Mangelson, D. Dominic, R. M. Eustice, R. Vasudevan, "Pairwise Consistent Measurement Set Maximization for Robust Multi-robot Map Merging," 2018. http://robots.engin.umich.edu/publications/jmangelson-2018a.pdf `[초록]`
- [P46] Y. Chang, N. Hughes, A. Ray, L. Carlone, "Hydra-Multi: Collaborative Online Construction of 3D Scene Graphs with Multi-Robot Teams," 2023. https://arxiv.org/abs/2304.13487 `[초록]`
- [P47] B. Yu, Q. Yuan, K. Li, H. Kasaei, M. Cao, "Co-NavGPT: Multi-Robot Cooperative Visual Semantic Navigation Using Vision Language Models," 2023. https://arxiv.org/abs/2310.07937 `[초록·본문 일부]`
- [P48] H. Zhang, W. Du, J. Shan 외, "Building Cooperative Embodied Agents Modularly with Large Language Models" (CoELA), ICLR 2024. https://arxiv.org/abs/2307.02485 `[본문]`
- [P49] Z. Mandi, S. Jain, S. Song, "RoCo: Dialectic Multi-Robot Collaboration with Large Language Models," 2023. https://arxiv.org/abs/2307.04738 `[본문]`
- [P50] Y. Chen, J. Arkin, Y. Zhang, N. Roy, C. Fan, "Scalable Multi-Robot Collaboration with Large Language Models: Centralized or Decentralized Systems?," 2023. https://arxiv.org/abs/2309.15943 `[본문]`
- [P51] X. Guo, K. Huang, J. Liu 외, "Embodied LLM Agents Learn to Cooperate in Organized Teams," 2024. https://arxiv.org/abs/2403.12482 `[본문]`
- [P52] S. Nayak, A. M. Orozco, M. T. Have 외, "LLaMAR: Long-Horizon Planning for Multi-Agent Robots in Partially Observable Environments," 2024. https://arxiv.org/abs/2407.10031 `[본문]`
- [P53] H. Li, Y. Q. Chong, S. Stepputtis 외, "Theory of Mind for Multi-Agent Collaboration via Large Language Models," EMNLP 2023. https://arxiv.org/abs/2310.10701 `[본문]`
- [P54] S. Seo, S. Noh, J. Lee, S. Lim, W. H. Lee, H. Kang, "REVECA: Adaptive Planning and Trajectory-based Validation in Cooperative Language Agents using Information Relevance and Relative Proximity," AAAI 2025. https://arxiv.org/abs/2405.16751 `[본문]`
- [P55] J. Liu, P. Zhou, Y. Du 외, "CaPo: Cooperative Plan Optimization for Efficient Embodied Multi-Agent Cooperation," ICLR 2025. https://arxiv.org/abs/2411.04679 `[초록]`
- [P56] H. Zhang, Z. Wang, Q. Lyu 외, "COMBO: Compositional World Models for Embodied Multi-Agent Cooperation," ICLR 2025. https://arxiv.org/abs/2404.10775 `[초록]`
- [P57] J. Chen, C. Yu, X. Zhou 외, "EMOS: Embodiment-aware Heterogeneous Multi-robot Operating System with LLM Agents," 2024. https://arxiv.org/abs/2410.22662 `[초록]`
- [P58] S. S. Kannan, V. L. N. Venkatesh, B.-C. Min, "SMART-LLM: Smart Multi-Agent Robot Task Planning using Large Language Models," 2023. https://arxiv.org/abs/2309.10062 `[초록]`
- [P59] T. Yoneda, J. Fang, P. Li 외, "Statler: State-Maintaining Language Models for Embodied Reasoning," ICRA 2024. https://arxiv.org/abs/2306.17840 `[초록·본문 일부]`
- [P60] J. S. Park, J. C. O'Brien, C. J. Cai, M. R. Morris, P. Liang, M. S. Bernstein, "Generative Agents: Interactive Simulacra of Human Behavior," 2023. https://arxiv.org/abs/2304.03442 `[본문]`
- [P61] M. Cemri, M. Z. Pan, S. Yang 외, "Why Do Multi-Agent LLM Systems Fail?," 2025. https://arxiv.org/abs/2503.13657 `[초록]`
- [P62] S. Kent, "Words of Estimative Probability," CIA Center for the Study of Intelligence(공개 PDF). https://www.cia.gov/resources/csi/static/Words-of-Estimative-Probability.pdf `[본문]`
- [P63] K. Tian, E. Mitchell, A. Zhou 외, "Just Ask for Calibration: Strategies for Eliciting Calibrated Confidence Scores from Language Models Fine-Tuned with Human Feedback," EMNLP 2023. https://arxiv.org/abs/2305.14975 `[초록]`
- [P64] M. Xiong, Z. Hu, X. Lu 외, "Can LLMs Express Their Uncertainty? An Empirical Evaluation of Confidence Elicitation in LLMs," ICLR 2024. https://arxiv.org/abs/2306.13063 `[초록]`
- [P65] A. Z. Ren, A. Dixit, A. Bodrova 외, "Robots That Ask For Help: Uncertainty Alignment for Large Language Model Planners" (KnowNo), CoRL 2023. https://arxiv.org/abs/2307.01928 `[초록]`

### 공개 코드·라이브러리

이 PR은 코드를 복사하거나 적응하지 않았다. "재사용"은 기억 작업(PR #211)이 이미 적응한 것이고, "권고"는 이 문서의 제안이다. 라이선스는 GitHub API의 SPDX 값과 LICENSE 파일에서 확인했고, 커밋은 2026-09-26 기본 브랜치 HEAD다.

| 저장소·패키지 | 버전·커밋 | 라이선스 | 쓰임 |
|---|---|---|---|
| https://github.com/AtsushiSakai/PythonRobotics | `b2020cd613d0` | MIT(LICENSE 파일, API는 NOASSERTION) | 재사용: `particle_filter.py` → `owncam_localizer.py`, `EKFSLAM/ekf_slam.py` 대응 → `owncam_memory_kf.py`. 검토: `FastSLAM1/2`, `GraphBasedSLAM`, `histogram_filter` |
| https://github.com/rlabbe/filterpy | PyPI 1.4.5 (저장소 HEAD `3b51149`) | MIT | 재사용: `kalman_filter.predict/update` → `owncam_memory_kf.py`. scipy 의존이라 설치하지 않음 |
| https://github.com/borglab/gtsam | PyPI 4.3.0 (저장소 `develop` HEAD `5e9c80c`) | BSD(LICENSE.BSD) | 권고(보류): 합동 평활이 필요할 때만 |
| https://github.com/david-m-rosen/Persistence-Filter | `1c7c812ec9b2` | LGPL-3.0 | 권고: 수식만 사용, 코드 미사용 |
| https://github.com/abewley/sort | `2236dff` | GPL-3.0 | 원리만(기각) |
| https://github.com/dstl/Stone-Soup | PyPI 1.9.1 | MIT | 기각(의존성) |
| https://github.com/shichaoy/cube_slam | `c371989` | BSD-3(LICENSE 파일) | 검토만 |
| https://github.com/qcr/quadricslam | PyPI 1.0.2 (저장소 HEAD `11a42ac`) | BSD-3 | 검토만 |
| https://github.com/MIT-SPARK/Kimera | `fc33760` | BSD-2 | 검토만 |
| https://github.com/MIT-SPARK/Hydra | `2e58a35` | BSD-2 | 검토만 |
| https://github.com/MIT-SPARK/Khronos | `63faadd` | BSD-3 | 발상(부재 증거)만 |
| https://github.com/MIT-SPARK/Kimera-Multi | `814bfe2` | LICENSE 없음 | 검토만 |
| https://github.com/MISTLab/Swarm-SLAM | `af17c4b` | MIT | 검토만 |
| https://github.com/vlmaps/vlmaps | `58060f9` | MIT | 검토만 |
| https://github.com/concept-graphs/concept-graphs | `93277a0` | MIT | 대응 규칙 참고 |
| https://github.com/hovsg/HOV-SG | `d6e65a5` | MIT | 검토만 |
| https://github.com/NVIDIA-AI-IOT/remembr | `964faab` | NVIDIA License(비상업 사용 한정, 3.3절) | 기각 |
| https://github.com/hello-robot/stretch_ai (DynaMem) | `6272b28` | Apache-2.0 | 발상(시야 안 제거)만 |
| https://github.com/UMass-Embodied-AGI/CoELA | `3e12dea` | LICENSE 없음 | 설계 참고(`harness/coela_modules.py`는 저장소 자체 구현) |
| https://github.com/MandiZhao/robot-collab | `9943870` | MIT | 설계 참고 |
| https://github.com/ripl/statler | `0812ef3` | LICENSE 없음 | 검토만 |
| https://github.com/SMARTlab-Purdue/SMART-LLM | `be42930` | LICENSE 없음 | 검토만 |
| https://github.com/joonspk-research/generative_agents | `fe05a71` | Apache-2.0 | 검색 점수 발상만 |

### 저장소 내부 모듈·PR

- main `3261c3aa`:
  - `harness/owncam_localizer.py`(l114 `OwnCamLocalizer`, l254 `_reset_from`)
  - `harness/coela_modules.py`(l30 `Memory`, l48 `last_seen_sim_time`, l64·l136 의도 TTL 12 s)
  - `harness/multi_object_tracking.py`(l60 `CargoTracker`)
  - `harness/zone_study_protocol.py`(l73 `STRUCT_FIELDS`, l76 `STRUCT_STATES`, l77 `CONFIDENCE`, l84 `DECISION_SOURCES`, l88 `INBOX_FIELDS`)
  - `harness/zone_study_prompts_ko.py`(l56 `BELIEF_KEYS`)
  - `docs/design/2026-09-26-markerless-localization-and-memory.md`(PR #210, 4(b) 관측 항목·들은 주장 형식)
  - `docs/design/2026-09-25-zone-dialogue-study-design-codex.md`(토큰 예산 표, 자기 위치 belief)
- PR #211 `kiro/zone-owncam-memory` `de6b520c`:
  - `harness/owncam_memory.py`: l54 `SCHEMA` v2, l84 `ABSENT_MISSES`, l85 `ABSENT_RANGE_M`, l87 `STALE_AGE_S`, l97 `GRID_TAU_S`, l261 `occluded`, l402 `point_in_view`, l411 `BoxTrack`, l894 `snapshot`
  - `harness/owncam_memory_kf.py`: l54 χ² 게이트, l57 모호 여유, l125 `associate`
  - `harness/owncam_pose_source.py`: l30 `PoseReport`, l56 `PoseLimits`
  - `docs/design/2026-09-26-owncam-memory-reuse.md`
- PR #227 `kiro/zone-vision-loc`: 태그 없는 벽·문 비전 측정. 자세 측정원 교체.
- PR #193 `kiro/zone-own-perception`: `harness/zone_own_outcome.py` `JudgmentTracker` 확인 규칙.

### 문서·웹 페이지

- arXiv 초록 페이지(`https://arxiv.org/abs/<id>`)와 PDF: 위 arXiv 논문 전부.
- Crossref REST API(`https://api.crossref.org/works/<DOI>`)와 OpenAlex(`https://api.openalex.org/works/doi:<DOI>`): DOI 논문의 서지·초록.
- MIT DSpace https://dspace.mit.edu/handle/1721.1/100724 ([P18] 초록·서지).
- DynaMem 프로젝트 페이지 https://dynamem.github.io (코드 위치 `stretch_ai/docs/dynamem.md`).
- PyPI JSON(`https://pypi.org/pypi/<name>/json`): gtsam, filterpy, stonesoup, quadricslam, opencv-python, scipy의 버전·휠·의존성.
- 각 저장소 README: Hydra(Ubuntu 24.04·ROS2 Jazzy), ConceptGraphs(CUDA·OpenAI), VLMaps(RGB-D 수집).
