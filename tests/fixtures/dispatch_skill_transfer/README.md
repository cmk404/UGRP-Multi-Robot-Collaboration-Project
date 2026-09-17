# 실제 RGB 색상 회귀 입력

`outputs/dispatch-skills-s6/rgb/pair-210-carry-anchor-r1.jpg` 및 `pair-228-carry-r1.jpg`의 원본 바이트. 실행 SHA `47a71bc`, 모델 `dispatch-transfer-t4`, open/seed11 기록 계획 진단. 카메라/로봇/빔 변경 없음, weld OFF. 동일 빔의 황색 영역이 기존 HSV 상한24를 넘어 실제 마스크가 잘리는 오류를 재현한다. 카메라 입력 특징 검사용이며 물리적 운반 성공 증거는 아니다.

`beam-top-lit.jpg`: S7 source ca11f79, pair-233-carry-top; loaded beam under unchanged lighting, raw shared camera.

`box-top-held.jpg`: S8 source 6e349db, solo-121-top; raw shared camera with held box and cyan floor distractors.
