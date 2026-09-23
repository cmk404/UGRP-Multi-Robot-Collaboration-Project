# Mac RGB 계약 재생 독립 사후감사

고정 소스 `6089f80519cab7b9c3c886f92ca71668249a41ea`, D3 로컬 진단 2건의 실행 원본을 읽기 전용으로 확인했다. `artifact-finalization.json`의 완료 영수증과 inventory SHA가 일치하고, `artifact-hashes.json`에 기록된 **1,284개 파일 전부** 현재 원본 SHA와 일치한다. 두 child는 종료 코드 0, reaped, process group gone이며 평가 시각과 종료 supervisor clock이 일치한다. 외부 모델 호출·메시지는 각각 0건이다. 이는 유한 물리 진단의 결과이며 통신 코호트 또는 실물 로봇 성공 근거가 아니다.

| 조건 | 실제 진행 및 수정 효과 | 종료 경계 | 물리 평가 |
|---|---|---|---|
| solo `r2` | 실제 자기 RGB 960×720 원본 SHA와 `cv2.resize(640×480)→JPEG95` 변환 SHA를 **153개 SKILL_DECISION 모두 재계산·일치**시켰다. 접근 82, 하강 16, 닫기 1, 올리기 15, 운반 27결정까지 진행했다. 따라서 기존 해상도 불일치로 초기에 멈추던 경로는 해소됐다. | 65.5·70.75 SIM초에 `VISUAL_GRASP_DRIFT`로 두 번 재관측하고, 72.7초 `VISUAL_ATTACHMENT_UNCONFIRMED` 주장으로 종료했다. 변환/worker freshness 실패가 아니다(154 worker 소비 모두 `completed_fresh`). | 상자 최대 상승 0.06186m, 변위 0.10652m이나 최종 슬롯 밖이며 물리 성공 `false`. 73.05 SIM초, 701명령. |
| joint `r1/r3` | 실제 TOP 프레임 두 쌍으로 독립 own-motion claim을 재계산했다: r1 2,586px, r3 2,023px, 둘 다 `valid=true`, `fresh=true`. 각 actor가 lower/upper 식별 3+3결정을 마치고 coarse 94결정씩 실행했다. 초기 identity 불인식은 해소됐다. | 26.45초 r3 `r3-000156` TOP에서 `own_wheel_heading_unresolved`; 26.5초 두 actor의 같은 lease가 `rgb_skill_unavailable`로 종료됐다. r3 coarse **94개 이전 예측과 실패 예측을 동일한 원본 이미지로 재계산·일치**시켰다. 실패 프레임에는 422 local yellow 픽셀이 있으나 선택 형상의 최소 사각형은 약 74×63px, 각도 −90°로 기존 four-corner heading gate(짧은 변 최대 53px, 각도 최대 18°)를 통과하지 못했다. | beam 상승·변위 0, 물리 성공 `false`. 27.05 SIM초, 410명령. |

joint 마지막 프레임은 실제 960×720 TOP이며 카메라 identity는 각 실행 전체에서 하나로 유지됐다. `raw_yellow_pixels=47,463` 중 작은 색 성분 913px, 현지 crop 422px이다. 실패는 **초기 own-motion 식별, 카메라 FOV 변경, worker staleness가 아니라 coarse wheel-heading 형상 지원 경계**다. r3 실패를 기록하고 두 actor를 멈췄으며 임의 lane/좌표 fallback은 실행되지 않았다. 마스크 형상 실패를 유발한 개별 픽셀의 의미나 향후 일반화는 이 두 실행만으로 확정할 수 없다. joint worker 소비 200건 모두 `completed_fresh`였다.

`review.json`에 원본 경로·식별 claim·변환 검산·coarse 실패 mask와 형상 수치를 보존했다. `audit.py`는 저장된 JPEG·JSON만 재계산하며 시뮬레이터·모델을 호출하지 않는다.
